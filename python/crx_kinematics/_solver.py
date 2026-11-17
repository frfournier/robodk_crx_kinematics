# SPDX-License-Identifier: Apache-2.0
# SPDX-FileCopyrightText: 2026 Centre Technologique en Aérospatiale
"""NumPy interface using RoboDK command degrees and millimetres."""

from __future__ import annotations

import ctypes as ct
import operator
import threading
import weakref
from dataclasses import dataclass
from enum import IntEnum
from typing import TYPE_CHECKING

import numpy as np
from numpy.typing import ArrayLike, NDArray

from ._native import (
    ClosedSolverError,
    FibreMetadata,
    IKMetadata,
    NativeError,
    NativeLibrary,
    check,
    release,
)

if TYPE_CHECKING:
    import os
    from pathlib import Path
    from types import TracebackType

FloatArray = NDArray[np.float64]
IntArray = NDArray[np.int32]
_UNIT_TOLERANCE = 128 * np.finfo(np.float64).eps


class IKStatus(IntEnum):
    """Native numerical outcome; an empty search is not proof of infeasibility."""

    STRICT_CANDIDATES = 0
    STRICT_SEED = 1
    STRICT_LOCAL = 2
    APPROXIMATE_SEED = 3
    NO_CANDIDATE = 4
    NO_OUTPUT_REQUESTED = 5
    INVALID_INPUT = 6
    UNSUPPORTED_MODEL = 7
    UNRESOLVED = 8
    NUMERICAL_RANGE_FAILURE = 9
    WORK_LIMIT = 10


@dataclass(frozen=True)
class IKResult:
    """Owned arrays, ordered by native selection policy.

    Undefined configuration rows contain -1 with configuration_valid=False.
    Residuals describe the requested TCP pose, while strict/approximate status
    reflects the solver's flange-pose tolerances. A long tool can amplify a
    flange orientation residual into a larger TCP position residual.
    """

    solutions: FloatArray
    configurations: IntArray
    configuration_valid: NDArray[np.bool_]
    status: IKStatus
    truncated: bool
    used_singular_representatives: bool
    position_residuals_mm: FloatArray
    orientation_residuals_deg: FloatArray

    @property
    def success(self) -> bool:
        return self.solutions.shape[0] > 0

    @property
    def is_approximate(self) -> bool:
        return self.status == IKStatus.APPROXIMATE_SEED


@dataclass(frozen=True)
class FibreIdentificationResult:
    """Discovered pose-fibre solutions with numerical identification labels.

    ``aspect_signs`` is the Jacobian determinant sign for regular solutions
    and zero for singular solutions. ``regular`` records that distinction
    directly. ``wrist_addresses`` has columns ``(arc, zero_based_root_rank,
    base_flip_bit)``. Arcs follow increasing wrist-circle angle in ``[-pi, pi)``;
    ranks are zero-based within each arc. The flip bit is zero on the positive
    shoulder side and one on the negative side. Invalid rows contain
    ``[-1, -1, -1]`` and have ``wrist_address_valid=False``. Address validity
    means the numerical root assignment was unambiguous; it is not a formal
    completeness certificate.
    """

    solutions: FloatArray
    aspect_signs: IntArray
    regular: NDArray[np.bool_]
    wrist_addresses: IntArray
    wrist_address_valid: NDArray[np.bool_]
    status: IKStatus
    truncated: bool
    used_singular_representatives: bool

    @property
    def success(self) -> bool:
        return self.solutions.shape[0] > 0


def _array(
    value: ArrayLike, shape: tuple[int, ...], name: str, *, copy: bool = False,
) -> FloatArray:
    raw = np.asarray(value)
    if raw.shape != shape:
        raise ValueError(f"{name} must have shape {shape}; received {raw.shape}")
    if raw.dtype.kind not in "iuf":
        raise TypeError(f"{name} must contain real numeric values, not {raw.dtype}")
    with np.errstate(over="ignore", invalid="ignore"):
        result = np.require(raw, dtype=np.float64, requirements=["C", "A"])
        if copy:
            result = result.copy(order="C")
    if not np.isfinite(result).all():
        raise ValueError(f"{name} must contain only finite float64 values")
    return result


def _pose(value: ArrayLike, name: str, *, copy: bool = False) -> FloatArray:
    pose = _array(value, (4, 4), name, copy=copy)
    rotation = pose[:3, :3]
    with np.errstate(over="ignore", invalid="ignore"):
        valid = (
            np.linalg.norm(pose[3] - [0, 0, 0, 1]) <= _UNIT_TOLERANCE
            and np.linalg.norm(rotation.T @ rotation - np.eye(3)) <= _UNIT_TOLERANCE
            and abs(np.linalg.det(rotation) - 1) <= _UNIT_TOLERANCE
        )
    if not valid:
        raise ValueError(f"{name} must be a rigid homogeneous transform")
    return pose


def _inverse(pose: FloatArray) -> FloatArray:
    inverse = np.eye(4)
    inverse[:3, :3] = pose[:3, :3].T
    with np.errstate(over="ignore", invalid="ignore"):
        inverse[:3, 3] = -pose[:3, :3].T @ pose[:3, 3]
    if not np.isfinite(inverse).all():
        raise ValueError("Transform inverse exceeds the float64 numerical range")
    return inverse


def _skew(vector: FloatArray) -> FloatArray:
    x, y, z = vector
    return np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]], dtype=np.float64)


class CrxSolver:
    """Local DLL solver for a nominal CRX model, with immutable settings.

    Parameters
    ----------
    model : str
        CRX-5iA, CRX-10iA, CRX-10iA-L, CRX-20iA-L, CRX-25iA, CRX-30iA, or
        CRX-3iA.
    dll_path : path-like, optional
        Explicit DLL file. Otherwise use RoboDK API installation discovery.
    joint_limits : array-like, shape (2, 6), optional
        Lower and upper RoboDK command limits in degrees.
    tool, reference : array-like, shape (4, 4), optional
        Flange-to-TCP and base-to-reference transforms in millimetres.
        Both default to identity. Outputs describe TCP relative to reference.

    Calls are serialized per instance. Compatible NumPy inputs are passed to
    native code without copying; other layouts and dtypes are normalized.
    Do not mutate an input while a solver call is running. Inputs are never
    modified, and all returned arrays own their memory and survive closure.
    """

    def __init__(
        self, model: str, *, dll_path: str | os.PathLike[str] | None = None,
        joint_limits: ArrayLike | None = None, tool: ArrayLike | None = None,
        reference: ArrayLike | None = None,
    ) -> None:
        self._lock = threading.RLock()
        self._tool = np.eye(4) if tool is None else _pose(tool, "tool", copy=True)
        self._reference = np.eye(4) if reference is None else _pose(
            reference, "reference", copy=True
        )
        self._tool_inverse = _inverse(self._tool)
        self._reference_inverse = _inverse(self._reference)
        limits = None if joint_limits is None else _array(
            joint_limits, (2, 6), "joint_limits", copy=True
        )
        if limits is not None and np.any(limits[0] > limits[1]):
            raise ValueError("Lower joint limits must not exceed upper joint limits")
        library = NativeLibrary(dll_path)
        handle = ct.c_void_p()
        try:
            names = tuple(library.model_name(i).decode("ascii") for i in range(7))
            if model not in names:
                raise ValueError(f"Unknown model {model!r}; choose one of {names}")
            check(library.create(names.index(model), limits, ct.byref(handle)))
            if not handle.value:
                raise NativeError("Native API returned a null solver handle")
            self._limits = np.empty((2, 6), dtype=np.float64)
            check(library.limits(handle, self._limits))
            self._version = library.library_version().decode("ascii")
        except BaseException:
            release(library, handle)
            raise
        self._library = library
        self._handle = handle
        self._model = model
        self._finalizer = weakref.finalize(self, release, library, handle)

    @property
    def model(self) -> str:
        return self._model

    @property
    def joint_limits(self) -> FloatArray:
        return self._limits.copy()

    @property
    def tool(self) -> FloatArray:
        return self._tool.copy()

    @property
    def reference(self) -> FloatArray:
        return self._reference.copy()

    @property
    def dll_path(self) -> Path:
        return self._library.path

    @property
    def native_version(self) -> str:
        return self._version

    @property
    def abi_version(self) -> int:
        return 1

    def _require_open(self) -> None:
        if not self._finalizer.alive:
            raise ClosedSolverError("Solver is closed")

    def close(self) -> None:
        """Release the native handle; repeated close calls are harmless."""
        with self._lock:
            self._finalizer()

    def __enter__(self) -> CrxSolver:
        with self._lock:
            self._require_open()
        return self

    def __exit__(
        self, exc_type: type[BaseException] | None,
        exc_value: BaseException | None, traceback: TracebackType | None,
    ) -> None:
        self.close()

    def __copy__(self) -> CrxSolver:
        raise TypeError("Solver handles cannot be copied; construct another CrxSolver")

    def __deepcopy__(self, memo: dict[int, object]) -> CrxSolver:
        raise TypeError("Solver handles cannot be copied; construct another CrxSolver")

    def _flange(self, joints: FloatArray) -> FloatArray:
        flange = np.empty((4, 4), dtype=np.float64)
        check(self._library.fk(self._handle, joints, flange))
        return flange

    def _tcp(self, flange: FloatArray) -> FloatArray:
        with np.errstate(over="ignore", invalid="ignore"):
            result = self._reference_inverse @ flange @ self._tool
        if not np.isfinite(result).all():
            raise NativeError("TCP transform exceeds the float64 numerical range")
        return result

    def fk(self, joints: ArrayLike) -> FloatArray:
        """Return TCP pose relative to reference, shape (4, 4), mm."""
        command = _array(joints, (6,), "joints")
        with self._lock:
            self._require_open()
            return self._tcp(self._flange(command))

    def configuration(self, joints: ArrayLike) -> IntArray:
        """Return [REAR, LOWERARM, FLIP], or raise ConfigurationError.

        Classification does not enforce joint limits. An arm singularity can
        make classification undefined even when FK/IK remain valid.
        """
        command = _array(joints, (6,), "joints")
        with self._lock:
            self._require_open()
            result = np.empty(3, dtype=np.int32)
            check(self._library.configuration(self._handle, command, result))
            return result

    def jacobian(self, joints: ArrayLike) -> FloatArray:
        """Return geometric TCP Jacobian, shape (6, 6).

        J @ qdot_deg_s gives [vx,vy,vz,wx,wy,wz] in mm/s and rad/s.
        Both vectors are expressed in the fixed reference frame. Linear
        velocity is measured at the TCP origin, not the reference origin.
        Angular rows are angular velocity components, not Euler derivatives.
        """
        command = _array(joints, (6,), "joints")
        with self._lock:
            self._require_open()
            native = np.empty((6, 6), dtype=np.float64)
            check(self._library.jacobian(self._handle, command, native))
            flange = self._flange(command)
            with np.errstate(over="ignore", invalid="ignore"):
                offset = flange[:3, :3] @ self._tool[:3, 3]
                native[:3] -= _skew(offset) @ native[3:]
                rotation = self._reference[:3, :3].T
                result = np.empty((6, 6), dtype=np.float64)
                result[:3] = rotation @ native[:3]
                result[3:] = rotation @ native[3:]
                result *= np.pi / 180
            if not np.isfinite(result).all():
                raise NativeError("TCP Jacobian exceeds the float64 numerical range")
            return result

    def ik(
        self, pose: ArrayLike, *, seed: ArrayLike | None = None,
        max_solutions: int = 1,
    ) -> IKResult:
        """Solve a TCP target, preserving native outcomes and full turns.

        One requested solution with a seed enables nearby search, without a
        globally-closest guarantee. Multiple solutions are ranked by squared
        command travel from the seed; without a seed, lexicographically.
        Approximate seed fallback is retained and explicitly marked. Empty
        results do not certify infeasibility or exhaustive singular coverage.
        """
        target = _pose(pose, "pose")
        command = None if seed is None else _array(seed, (6,), "seed")
        if isinstance(max_solutions, (bool, np.bool_)):
            raise TypeError("max_solutions must be a positive integer")
        capacity = operator.index(max_solutions)
        if not 1 <= capacity <= np.iinfo(np.int32).max:
            raise ValueError("max_solutions must be in [1, 2147483647]")
        with np.errstate(over="ignore", invalid="ignore"):
            flange_target = np.ascontiguousarray(self._reference @ target @ self._tool_inverse)
        if not np.isfinite(flange_target).all():
            raise ValueError("Flange target exceeds the float64 numerical range")
        with self._lock:
            self._require_open()
            output = np.empty((capacity, 6), dtype=np.float64)
            metadata = IKMetadata()
            check(self._library.ik(self._handle, flange_target, command,
                                   capacity, output, ct.byref(metadata)))
            if not 0 <= metadata.count <= capacity:
                raise NativeError("Native API returned an invalid solution count")
            try:
                status = IKStatus(metadata.status)
            except ValueError as exc:
                raise NativeError("Native API returned an unknown IK status") from exc
            if status in (IKStatus.INVALID_INPUT, IKStatus.UNSUPPORTED_MODEL):
                raise ValueError(f"Native solver rejected the request: {status.name}")
            successful_status = status in (
                IKStatus.STRICT_CANDIDATES, IKStatus.STRICT_SEED,
                IKStatus.STRICT_LOCAL, IKStatus.APPROXIMATE_SEED,
            )
            if successful_status != (metadata.count > 0):
                raise NativeError("Native IK status and solution count disagree")
            solutions = output[:metadata.count].copy()
            if not np.isfinite(solutions).all():
                raise NativeError("Native API returned nonfinite solutions")
            configs = np.full((len(solutions), 3), -1, dtype=np.int32)
            valid = np.zeros(len(solutions), dtype=np.bool_)
            position = np.empty(len(solutions), dtype=np.float64)
            angle = np.empty(len(solutions), dtype=np.float64)
            for i, solution in enumerate(solutions):
                code = self._library.configuration(self._handle, solution, configs[i])
                if code != -3:
                    check(code)
                    valid[i] = True
                actual = self._tcp(self._flange(solution))
                position[i] = np.hypot.reduce(actual[:3, 3] - target[:3, 3])
                chord = np.linalg.norm(actual[:3, :3] - target[:3, :3])
                angle[i] = np.degrees(2 * np.arcsin(np.clip(chord / (2 * np.sqrt(2)), 0, 1)))
            return IKResult(solutions, configs, valid, status, bool(metadata.truncated),
                            bool(metadata.used_singular_representatives), position, angle)

    def identify(
        self, pose: ArrayLike, *, max_solutions: int = 64,
    ) -> FibreIdentificationResult:
        """Identify discovered inverse-kinematics branches for a TCP target.

        This always performs full discovery and does not take a seed. Wrist-
        circle addresses are numerical labels; ambiguous assignments are
        reported through ``wrist_address_valid``.
        """
        target = _pose(pose, "pose")
        if isinstance(max_solutions, (bool, np.bool_)):
            raise TypeError("max_solutions must be a positive integer")
        capacity = operator.index(max_solutions)
        if not 1 <= capacity <= np.iinfo(np.int32).max:
            raise ValueError("max_solutions must be in [1, 2147483647]")
        with np.errstate(over="ignore", invalid="ignore"):
            flange_target = np.ascontiguousarray(
                self._reference @ target @ self._tool_inverse
            )
        if not np.isfinite(flange_target).all():
            raise ValueError("Flange target exceeds the float64 numerical range")
        with self._lock:
            self._require_open()
            if self._library.identify is None:
                raise NotImplementedError(
                    "This native DLL does not export fibre identification; "
                    "load a 0.5.0 or newer DLL."
                )
            solutions = np.empty((capacity, 6), dtype=np.float64)
            signs = np.empty(capacity, dtype=np.int32)
            regular_i = np.empty(capacity, dtype=np.int32)
            addresses = np.empty((capacity, 3), dtype=np.int32)
            address_valid_i = np.empty(capacity, dtype=np.int32)
            metadata = FibreMetadata()
            check(self._library.identify(
                self._handle, flange_target, capacity, solutions, signs,
                regular_i, addresses, address_valid_i, ct.byref(metadata),
            ))
            if not 0 <= metadata.count <= capacity:
                raise NativeError("Native API returned an invalid identification count")
            try:
                status = IKStatus(metadata.status)
            except ValueError as exc:
                raise NativeError("Native API returned an unknown identification status") from exc
            if status in (IKStatus.INVALID_INPUT, IKStatus.UNSUPPORTED_MODEL):
                raise ValueError(f"Native solver rejected the request: {status.name}")
            successful_status = status in (
                IKStatus.STRICT_CANDIDATES, IKStatus.STRICT_SEED,
                IKStatus.STRICT_LOCAL, IKStatus.APPROXIMATE_SEED,
            )
            if successful_status != (metadata.count > 0):
                raise NativeError("Identification status and solution count disagree")
            count = metadata.count
            solutions = solutions[:count].copy()
            signs = signs[:count].copy()
            regular = regular_i[:count].astype(np.bool_)
            addresses = addresses[:count].copy()
            address_valid = address_valid_i[:count].astype(np.bool_)
            if (not np.isfinite(solutions).all()
                    or not np.isin(signs, (-1, 0, 1)).all()
                    or not np.isin(regular_i[:count], (0, 1)).all()
                    or not np.isin(address_valid_i[:count], (0, 1)).all()):
                raise NativeError("Native identification returned invalid metadata")
            if np.any((signs == 0) != ~regular):
                raise NativeError("Aspect signs disagree with regularity metadata")
            if np.any(addresses[~address_valid] != -1):
                raise NativeError("Invalid wrist addresses must use the -1 sentinel")
            valid_addresses = addresses[address_valid]
            if (np.any(valid_addresses[:, :2] < 0)
                    or np.any((valid_addresses[:, 2] < 0)
                              | (valid_addresses[:, 2] > 1))):
                raise NativeError("Native identification returned an invalid wrist address")
            return FibreIdentificationResult(
                solutions, signs, regular, addresses, address_valid, status,
                bool(metadata.truncated),
                bool(metadata.used_singular_representatives),
            )
