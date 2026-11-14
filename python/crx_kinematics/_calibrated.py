# SPDX-License-Identifier: Apache-2.0
# SPDX-FileCopyrightText: 2026 Centre Technologique en Aérospatiale
"""Owned calibrated-model inputs and local IK results, in command degrees/mm."""

from __future__ import annotations

import ctypes as ct
from dataclasses import dataclass
from enum import IntEnum
import math
import operator
import threading
import weakref

import numpy as np

from . import _calibrated_native as native
from ._certificates import CertificateFailure
from ._native import ClosedSolverError, NativeError
from ._solver import Coverage, CrxSolver, _array, _pose


def _owned(values, dtype=np.float64):
    result = np.array(values, dtype=dtype, copy=True, order="C")
    result.flags.writeable = False
    return result


def _integer(value, low, high, name):
    if isinstance(value, (bool, np.bool_)):
        raise TypeError(f"{name} must be an integer")
    value = operator.index(value)
    if not low <= value <= high:
        raise ValueError(f"{name} must be in [{low}, {high}]")
    return value


def _pointer(array):
    return array.ctypes.data_as(ct.POINTER(ct.c_double))


@dataclass(frozen=True, eq=False)
class CalibratedFrame:
    """Exact homogeneous quaternion/position declaration; lengths are mm.

    A matrix conversion records the quaternion chosen to define its SO(3)
    rotation. The certificate concerns this declared rotation. No floating-point
    matrix is assumed to be exactly orthogonal.
    """

    quaternion: np.ndarray = (1., 0., 0., 0.)
    position: np.ndarray = (0., 0., 0.)

    def __post_init__(self):
        q = _array(self.quaternion, (4,), "quaternion", copy=True)
        p = _array(self.position, (3,), "position", copy=True)
        if not np.any(q):
            raise ValueError("quaternion must be nonzero")
        object.__setattr__(self, "quaternion", _owned(q))
        object.__setattr__(self, "position", _owned(p))

    @classmethod
    def from_matrix(cls, value):
        pose = _pose(value, "frame", copy=True)
        r = pose[:3, :3]
        candidates = np.array([1+np.trace(r), 1+2*r[0, 0]-np.trace(r),
                               1+2*r[1, 1]-np.trace(r), 1+2*r[2, 2]-np.trace(r)])
        index = int(np.argmax(candidates))
        q = np.empty(4)
        q[index] = .5*math.sqrt(float(candidates[index]))
        divisor = 4*q[index]
        if index == 0:
            q[1:] = [(r[2, 1]-r[1, 2])/divisor, (r[0, 2]-r[2, 0])/divisor,
                     (r[1, 0]-r[0, 1])/divisor]
        else:
            i = index-1
            j, k = (i+1) % 3, (i+2) % 3
            q[0] = (r[k, j]-r[j, k])/divisor
            q[j+1] = (r[j, i]+r[i, j])/divisor
            q[k+1] = (r[k, i]+r[i, k])/divisor
        if q[0] < 0:
            q = -q
        return cls(q, pose[:3, 3])

    def as_matrix(self):
        w, x, y, z = self.quaternion/np.max(np.abs(self.quaternion))
        norm = w*w+x*x+y*y+z*z
        output = np.eye(4)
        output[:3, :3] = np.array([
            [w*w+x*x-y*y-z*z, 2*(x*y-w*z), 2*(x*z+w*y)],
            [2*(x*y+w*z), w*w-x*x+y*y-z*z, 2*(y*z-w*x)],
            [2*(x*z-w*y), 2*(y*z+w*x), w*w-x*x-y*y+z*z]])/norm
        output[:3, 3] = self.position
        return output


def _frame(value):
    return (CalibratedFrame(value.quaternion, value.position) if isinstance(value, CalibratedFrame)
            else CalibratedFrame.from_matrix(value))


@dataclass(frozen=True, eq=False)
class CalibratedGeometry:
    """Owned declared model. DH columns(alpha correction rad,a mm,theta correction rad,d mm)."""

    dh: np.ndarray
    alpha_quarters: np.ndarray
    theta_quarters: np.ndarray
    senses: np.ndarray
    base: CalibratedFrame
    tool: CalibratedFrame
    joint_limits: np.ndarray
    limit_units: str
    length_scale: float


@dataclass(frozen=True)
class CalibratedOptions:
    max_iterations: int = 24
    max_backtracks: int = 12
    maximum_step_rad: float = .25
    normalized_position_tolerance: float = 1e-13
    orientation_tolerance_rad: float = 1e-13
    uniqueness_radius_rad: float = 0.
    certify: bool = True

    def __post_init__(self):
        _integer(self.max_iterations, 0, 128, "max_iterations")
        _integer(self.max_backtracks, 0, 32, "max_backtracks")
        for name in ("maximum_step_rad", "normalized_position_tolerance",
                     "orientation_tolerance_rad", "uniqueness_radius_rad"):
            value = getattr(self, name)
            if isinstance(value, (bool, np.bool_, str, bytes)):
                raise TypeError(f"{name} must be a real number")
            value = float(value)
            if not math.isfinite(value) or value < 0 or (name == "maximum_step_rad" and value == 0):
                raise ValueError(f"{name} must be finite and {'positive' if name == 'maximum_step_rad' else 'nonnegative'}")
        if not isinstance(self.certify, (bool, np.bool_)):
            raise TypeError("certify must be a boolean")


class CalibratedStatus(IntEnum):
    CONVERGED = 0
    INVALID_INPUT = 1
    MODEL_MISMATCH = 2
    ARITHMETIC_FAILURE = 3
    ITERATION_BUDGET = 4
    SINGULAR_STEP = 5
    LINE_SEARCH_FAILURE = 6


@dataclass(frozen=True)
class CalibratedIdentity:
    model_schema: int
    model_words: tuple[int, ...]
    target_words: tuple[int, ...]


@dataclass(frozen=True, eq=False)
class CalibratedBallCertificate:
    """One exact calibrated root near the represented increment centre.

    Jacobian signs are LOCAL labels. Only commands_admitted proves the root
    respects the declared real command limits. No coverage/aspect/nearest claim.
    """

    identity: CalibratedIdentity
    increment_center_rad: np.ndarray
    command_center_rad: np.ndarray
    existence_radius_rad: float
    uniqueness_radius_rad: float
    beta: float
    lipschitz: float
    eta: float
    inverse_residual: float
    coarse_eta: float
    pose_defect: float
    screw_norm_sum: float
    angular_norm_sum: float
    anchor_position_norm: float
    command_reserve_lower_rad: float
    preconditioner: np.ndarray
    local_jacobian_sign: int
    command_jacobian_sign: int
    commands_admitted: bool
    used_log_series: bool


@dataclass(frozen=True, eq=False)
class CalibratedAttempt:
    source_index: int
    seed_degrees: np.ndarray
    commands_degrees: np.ndarray
    commands_rad: np.ndarray
    increments_rad: np.ndarray
    status: CalibratedStatus
    iterations: int
    backtracks: int
    evaluations: int
    normalized_position_error: float
    orientation_error_rad: float
    numerical_limits_satisfied: bool
    certificate_requested: bool
    certificate_attempted: bool
    certificate_failure: CertificateFailure
    certificate: CalibratedBallCertificate | None


@dataclass(frozen=True, eq=False)
class CalibratedResult:
    """Owned snapshots. Solutions are command-degree numerical centres.

    With proofs enabled, solutions contain only centres whose exact local root
    is certified AND admitted. All other attempts remain available. With proofs
    disabled, solutions are explicitly numerical converged in-limit candidates.
    Coverage is always INCONCLUSIVE; duplicate seeds/roots remain separate rows.
    """

    solutions: np.ndarray
    solution_indices: np.ndarray
    attempts: tuple[CalibratedAttempt, ...]
    coverage: Coverage
    geometry: CalibratedGeometry
    target: CalibratedFrame
    identity: CalibratedIdentity
    numerical_count: int
    numerical_inside_count: int
    local_count: int
    admitted_count: int
    iterations: int
    backtracks: int
    evaluations: int
    owned_native_payload_bytes: int
    proofs_requested: bool
    nominal_count: int = 0
    nominal_truncated: bool = False

    @property
    def success(self):
        return len(self.solutions) > 0


def _quarters(value, name):
    raw = np.asarray(value)
    if raw.shape != (6,):
        raise ValueError(f"{name} must have shape (6,)")
    return _owned([_integer(x, -(2**31), 2**31-1, name) for x in raw], np.int32)


def _word(value):
    return int(np.array([value], dtype=np.float64).view(np.uint64)[0])


def _model_words(geometry):
    words = []
    for i, (ar, a, tr, d) in enumerate(geometry.dh):
        words.extend([int(geometry.alpha_quarters[i]) & (2**64-1), _word(ar),
                      int(geometry.theta_quarters[i]) & (2**64-1), _word(tr), _word(a), _word(d)])
    for values in [geometry.senses, geometry.base.quaternion, geometry.base.position,
                   geometry.tool.quaternion, geometry.tool.position,
                   geometry.joint_limits[0], geometry.joint_limits[1], [geometry.length_scale]]:
        words.extend(_word(value) for value in values)
    words.append(1 if geometry.limit_units == "degrees" else 0)
    return tuple(words)


def _release(binding, handle):
    binding.destroy(handle)


class CalibratedSolver:
    """Separate calibrated DH path; nominal is used only to propose seeds.

    Declared F=base*DH_1*...*DH_6*tool is in the same output frame as target.
    Supply base/tool accordingly; no hidden reference/TCP composition occurs.
    The nominal solver keeps its own existing frame and numerical policy.
    Calls/closure are serialized. Returned snapshots survive model closure.
    """

    def __init__(self, nominal: CrxSolver, dh, *, alpha_quarters=(0,)*6,
                 theta_quarters=(0,)*6, senses=(1., 1., -1., -1., -1., -1.),
                 base=None, tool=None, joint_limits=None, limit_units="degrees", length_scale=None):
        if not isinstance(nominal, CrxSolver):
            raise TypeError("nominal must be a CrxSolver")
        if limit_units not in ("degrees", "radians"):
            raise ValueError("limit_units must be 'degrees' or 'radians'")
        if joint_limits is None:
            if limit_units != "degrees":
                raise ValueError("Supply explicit joint_limits when limit_units is radians")
            joint_limits = nominal.joint_limits
        rows = _owned(_array(dh, (6, 4), "dh", copy=True))
        limits = _owned(_array(joint_limits, (2, 6), "joint_limits", copy=True))
        if np.any(limits[0] > limits[1]):
            raise ValueError("Lower joint limits must not exceed upper limits")
        signs = _owned(_array(senses, (6,), "senses", copy=True))
        if not np.all((signs == 1) | (signs == -1)):
            raise ValueError("senses must be exactly +1 or -1")
        if length_scale is None:
            length_scale = float(np.max(np.abs(rows[:, [1, 3]]))) or 1.
        length_scale = float(length_scale)
        if not math.isfinite(length_scale) or length_scale <= 0:
            raise ValueError("length_scale must be finite and positive")
        geometry = CalibratedGeometry(rows, _quarters(alpha_quarters, "alpha_quarters"),
            _quarters(theta_quarters, "theta_quarters"), signs,
            CalibratedFrame() if base is None else _frame(base),
            CalibratedFrame() if tool is None else _frame(tool), limits, limit_units, length_scale)
        binding = native.Bindings(nominal._library)
        value = native.ModelInput()
        for i in range(6):
            value.dh[i][:] = geometry.dh[i]
        value.alpha_quarters[:], value.theta_quarters[:] = geometry.alpha_quarters, geometry.theta_quarters
        value.senses[:] = geometry.senses
        value.base_quaternion[:], value.base_position[:] = geometry.base.quaternion, geometry.base.position
        value.tool_quaternion[:], value.tool_position[:] = geometry.tool.quaternion, geometry.tool.position
        value.lower_commands[:], value.upper_commands[:] = geometry.joint_limits
        value.length_scale, value.limit_units = length_scale, int(limit_units == "degrees")
        handle = ct.c_void_p()
        native.check(binding.create(ct.byref(value), ct.byref(handle)))
        if not handle.value:
            raise NativeError("Calibrated API returned a null model")
        self._nominal, self._binding, self._geometry, self._handle = nominal, binding, geometry, handle
        self._words = _model_words(geometry)
        self._lock = threading.RLock()
        self._finalizer = weakref.finalize(self, _release, binding, handle)

    @property
    def geometry(self):
        return self._geometry

    def _open(self):
        if not self._finalizer.alive:
            raise ClosedSolverError("Calibrated solver is closed")

    def close(self):
        with self._lock:
            self._finalizer()

    def __enter__(self):
        with self._lock:
            self._open()
        return self

    def __exit__(self, *_):
        self.close()

    def __copy__(self):
        raise TypeError("CalibratedSolver owns a native handle; construct another instance")

    def __deepcopy__(self, _memo):
        raise TypeError("CalibratedSolver owns a native handle; construct another instance")

    def fk(self, joints):
        commands = np.ascontiguousarray(np.radians(_array(joints, (6,), "joints", copy=True)))
        if not np.isfinite(commands).all():
            raise ValueError("Command conversion exceeds the numerical range")
        output = np.empty((4, 4))
        with self._lock:
            self._open()
            native.check(self._binding.fk(self._handle, _pointer(commands), _pointer(output)))
        return output

    def jacobian(self, joints):
        """Numerical body Jacobian(v;omega), mm/radian, command-radian columns."""
        commands = np.ascontiguousarray(np.radians(_array(joints, (6,), "joints", copy=True)))
        if not np.isfinite(commands).all():
            raise ValueError("Command conversion exceeds the numerical range")
        output = np.empty((6, 6))
        with self._lock:
            self._open()
            native.check(self._binding.jacobian(self._handle, _pointer(commands), _pointer(output)))
        return output

    def ik(self, pose, *, seed=None, max_candidates=64, options=None):
        """Discover nominal seeds, then correct each independently on this model.

        max_candidates includes an optional explicit seed. An empty/truncated
        nominal discovery cannot establish calibrated infeasibility/coverage.
        """
        with self._lock:
            self._open()
        frame = _frame(pose)
        maximum = _integer(max_candidates, 1, 512, "max_candidates")
        explicit = [] if seed is None else [_array(seed, (6,), "seed", copy=True)]
        capacity = maximum-len(explicit)
        nominal_count, truncated = 0, False
        seeds = explicit
        if capacity:
            proposed = self._nominal.identify(frame.as_matrix(), max_solutions=capacity)
            nominal_count, truncated = len(proposed.solutions), proposed.truncated
            seeds = [*explicit, *proposed.solutions]
        return self._correct(frame, np.array(seeds, dtype=np.float64).reshape(-1, 6), options,
                             nominal_count, truncated)

    def correct(self, pose, candidates, *, options=None):
        """Correct explicit proposed coupled command-degree rows; no discovery claim."""
        return self._correct(_frame(pose), candidates, options, 0, False)

    def _correct(self, frame, candidates, options, nominal_count, nominal_truncated):
        if options is None:
            options = CalibratedOptions()
        if not isinstance(options, CalibratedOptions):
            raise TypeError("options must be CalibratedOptions or None")
        raw = np.asarray(candidates)
        if raw.shape == (0,):
            raw = raw.reshape(0, 6)
        if raw.ndim != 2 or raw.shape[1:] != (6,) or len(raw) > 512:
            raise ValueError("candidates must have shape (N,6), with 0<=N<=512")
        degrees = _array(raw, raw.shape, "candidates", copy=True)
        radians = np.ascontiguousarray(np.radians(degrees))
        if not np.isfinite(radians).all():
            raise ValueError("Command conversion exceeds the numerical range")
        request = self._binding.defaults()
        request.max_iterations, request.max_backtracks = options.max_iterations, options.max_backtracks
        request.maximum_step = options.maximum_step_rad
        request.normalized_position_tolerance = options.normalized_position_tolerance
        request.orientation_tolerance = options.orientation_tolerance_rad
        request.uniqueness_radius = options.uniqueness_radius_rad
        request.flags = 0 if options.certify else 1
        target = native.Target()
        target.quaternion[:], target.position[:] = frame.quaternion, frame.position
        query = ct.c_void_p()
        with self._lock:
            self._open()
            native.check(self._binding.query_create(self._handle, ct.byref(target),
                _pointer(radians) if len(radians) else None, len(radians), ct.byref(request), ct.byref(query)))
            if not query.value:
                raise NativeError("Calibrated API returned a null query")
            try:
                return self._decode(query, frame, radians, degrees, options, nominal_count, nominal_truncated)
            finally:
                self._binding.query_destroy(query)

    def _decode(self, query, frame, seeds_rad, seeds_degrees, options, nominal_count, nominal_truncated):
        count = len(seeds_rad)
        summary, binding = native.Summary(), native.Identity()
        native.check(self._binding.summary_read(query, ct.byref(summary)))
        native.check(self._binding.identity_read(query, ct.byref(binding)))
        target_words = tuple(_word(value) for value in [*frame.quaternion, *frame.position])
        if (summary.coverage != Coverage.INCONCLUSIVE or summary.seed_count != count or
            binding.model_schema != 2 or binding.model_word_count != 70 or
            tuple(binding.model_words) != self._words or tuple(binding.target_words) != target_words or
            binding.length_scale != self.geometry.length_scale or
            summary.length_scale != self.geometry.length_scale or
            binding.limit_units != int(self.geometry.limit_units == "degrees")):
            raise NativeError("Calibrated query returned an inconsistent model/target identity")
        identity = CalibratedIdentity(2, self._words, target_words)
        attempts = []
        for index in range(count):
            row = native.Result()
            native.check(self._binding.result_read(query, index, ct.byref(row)))
            try:
                status, failure = CalibratedStatus(row.status), CertificateFailure(row.certificate_failure)
            except ValueError as exc:
                raise NativeError("Calibrated query returned an unknown outcome") from exc
            if (row.seed_index != index or row.flags & ~63 or
                bool(row.flags & 1) != options.certify or
                np.array(row.seed_commands_rad).tobytes() != seeds_rad[index].tobytes() or
                (row.flags & 2 and not row.flags & 1) or
                (row.flags & (8 | 32) and (not row.flags & 2 or status != CalibratedStatus.CONVERGED)) or
                (row.flags & (16 | 32) and not row.flags & 8) or
                (row.flags & 4 and status != CalibratedStatus.CONVERGED) or
                row.iterations > options.max_iterations or
                row.evaluations > 1+options.max_iterations*(options.max_backtracks+1)):
                raise NativeError("Calibrated query returned inconsistent attempt flags")
            commands, increments = _owned(row.commands_rad), _owned(row.increments_rad)
            if not np.isfinite(commands).all() or not np.isfinite(increments).all():
                raise NativeError("Calibrated query returned nonfinite commands")
            certificate = None
            if row.flags & 8:
                if (failure != CertificateFailure.CERTIFIED or row.local_jacobian_sign not in (-1, 1) or
                    row.command_jacobian_sign not in (-1, 1) or not
                    (0 <= row.existence_radius <= row.uniqueness_radius and row.uniqueness_radius > 0)):
                    raise NativeError("Calibrated query returned an invalid local proof")
                certificate = CalibratedBallCertificate(identity, increments, commands,
                    row.existence_radius, row.uniqueness_radius, row.beta, row.lipschitz, row.eta,
                    row.inverse_residual, row.coarse_eta, row.pose_defect, row.screw_norm_sum,
                    row.angular_norm_sum, row.anchor_position_norm, row.command_reserve_lower,
                    _owned(np.array(row.preconditioner).reshape(6, 6)), row.local_jacobian_sign,
                    row.command_jacobian_sign, bool(row.flags & 16), bool(row.flags & 32))
            attempts.append(CalibratedAttempt(index, _owned(seeds_degrees[index]),
                _owned(np.degrees(commands)), commands, increments, status, row.iterations,
                row.backtracks, row.evaluations, row.normalized_position_error, row.orientation_error,
                bool(row.flags & 4), bool(row.flags & 1), bool(row.flags & 2), failure, certificate))
        numerical = sum(attempt.status == CalibratedStatus.CONVERGED for attempt in attempts)
        inside = sum(attempt.numerical_limits_satisfied for attempt in attempts)
        local = sum(attempt.certificate is not None for attempt in attempts)
        admitted = sum(attempt.certificate is not None and attempt.certificate.commands_admitted for attempt in attempts)
        if ((numerical, inside, local, admitted) != (summary.numerical_count, summary.numerical_inside_count,
                                                    summary.local_count, summary.admitted_count) or
            sum(attempt.iterations for attempt in attempts) != summary.iterations or
            sum(attempt.backtracks for attempt in attempts) != summary.backtracks or
            sum(attempt.evaluations for attempt in attempts) != summary.evaluations):
            raise NativeError("Calibrated query summary disagrees with its owned attempts")
        selected = [attempt.source_index for attempt in attempts
                    if ((attempt.certificate is not None and attempt.certificate.commands_admitted)
                        if options.certify else (attempt.status == CalibratedStatus.CONVERGED and
                                                attempt.numerical_limits_satisfied))]
        solutions = np.array([attempts[i].commands_degrees for i in selected]).reshape(-1, 6)
        return CalibratedResult(_owned(solutions), _owned(selected, np.int64), tuple(attempts),
            Coverage.INCONCLUSIVE, self.geometry, frame, identity, numerical, inside, local, admitted,
            summary.iterations, summary.backtracks, summary.evaluations, summary.owned_payload_bytes,
            bool(options.certify), nominal_count, bool(nominal_truncated))
