# SPDX-License-Identifier: Apache-2.0
# SPDX-FileCopyrightText: 2026 Centre Technologique en Aérospatiale
"""Owned verified-query snapshots and an optional immutable native audit cache."""

from __future__ import annotations

import ctypes as ct
from dataclasses import dataclass, field, replace
from enum import IntEnum, IntFlag
import operator
import threading
import weakref

import numpy as np

from . import _catalogue_native as abi
from ._certificates import CertificateFailure, decode_ball_record, _readonly
from ._native import BallRecord, LibraryError, NativeError, check
from ._solver import Coverage, IKStatus


class CommandProfile(IntEnum):
    CALLBACK = 0
    DECOUPLED_USER = 1


class Admission(IntEnum):
    NO = 0
    YES = 1
    UNDECIDED = 2


class Ranking(IntEnum):
    NOT_ATTEMPTED = 0
    INCONCLUSIVE = 1
    UNIQUE = 2
    POSSIBLE_MINIMIZERS = 3
    NO_ADMISSIBLE = 4


class QueryPreparation(IntEnum):
    NOT_PREPARED = 0
    READY = 1
    INVALID_INPUT = 2
    UNSUPPORTED_MODEL = 3
    UNSUPPORTED_ENVIRONMENT = 4
    ARITHMETIC_FAILURE = 5


class CommandFailure(IntEnum):
    SUCCESS = 0
    INVALID_INPUT = 1
    UNSUPPORTED_ENVIRONMENT = 2
    IDENTITY_MISMATCH = 3
    INCOMPLETE_PHASE_COVER = 4
    RANGE_UNAVAILABLE = 5
    WORK_LIMIT = 6
    ARITHMETIC_UNAVAILABLE = 7


class PhaseFailure(IntEnum):
    SUCCESS = 0
    INVALID_INPUT = 1
    UNSUPPORTED_ENVIRONMENT = 2
    IDENTITY_MISMATCH = 3
    INCOMPLETE_ROOT_COVER = 4
    INVALID_BALL = 5
    COUNT_MISMATCH = 6
    BALLS_NOT_SEPARATED = 7
    RANGE_UNAVAILABLE = 8
    PAIR_WORK_LIMIT = 9


class RootStatus(IntEnum):
    COMPLETE = 0
    INCONCLUSIVE = 1
    INVALID_INPUT = 2
    UNSUPPORTED_ENVIRONMENT = 3
    ARITHMETIC_UNAVAILABLE = 4


class RootKind(IntEnum):
    EXCLUSION = 0
    SIMPLE = 1
    UNRESOLVED = 2


class RootReason(IntEnum):
    NONE = 0
    CHART_BOUNDARY = 1
    DEPTH_LIMIT = 2
    WORK_LIMIT = 3
    LEAF_LIMIT = 4
    SPLIT_NOT_SEPARATED = 5
    ZERO_POLYNOMIAL = 6
    ZERO_DERIVATIVE = 7
    ZERO_DETERMINANT = 8
    ARITHMETIC_FAILURE = 9


class NearStratum(IntFlag):
    """Numerical proximity/enclosure ambiguity; no exact family classification."""
    DETERMINANT_ZERO = 1
    BASE_AXIS = 2
    COINCIDENT_SPHERES = 4
    VERTICAL_ARM = 8
    POLYNOMIAL_ENCLOSES_ZERO = 16
    ARM_SHELL = 32


class RecoverySource(IntFlag):
    DETERMINANT = 1
    BOUNDARY = 2
    UNRESOLVED_DOMAIN = 4
    CARDINAL = 8


class RecoveryStatus(IntEnum):
    CANDIDATES = 0
    NO_CANDIDATE = 1
    NEEDS_REFINEMENT = 2
    INVALID_INPUT = 3
    NUMERICAL_RANGE_FAILURE = 4


@dataclass(frozen=True, eq=False)
class FibreRecovery:
    """Owned numerical canonical witnesses, separately from exact-problem balls.

    Defects are normalized numerical magnitudes, not proof intervals; NaN means
    unavailable. Position errors use normalized lengths, orientation radians.
    """
    index: int
    source: RecoverySource
    near_stratum: NearStratum
    used_singular_representatives: bool
    refinement_attempted: bool
    candidate_count: int
    attempted_balls: int
    certified_balls: int
    numerical_status: RecoveryStatus
    phase: float
    delta_defect: float
    base_axis_defect: float
    coincident_defect: float
    vertical_defect: float
    shell_defect: float
    compatibility_defect: float
    canonical_rad: np.ndarray
    position_errors: np.ndarray
    orientation_errors: np.ndarray


@dataclass(frozen=True)
class FibreStrata:
    """Bounded numerical recovery diagnostics; not a continuous-fibre verdict."""
    recovery_budget_exhausted: bool
    source_domains: int
    candidate_budget_exhausted: bool
    total_ball_attempts: int
    total_certified_balls: int
    nomination_count: int
    recoveries: tuple[FibreRecovery, ...]


@dataclass(frozen=True)
class CatalogueOptions:
    enabled: bool = True
    max_nodes: int = 8192
    max_depth: int = 40
    max_leaves: int = 4096
    max_chart_attempts: int = 8
    max_candidate_certificates: int = 512
    max_pair_tests: int = 120
    max_lifts: int = 1 << 20

    def native(self, profile, capacity):
        if not isinstance(self.enabled, (bool, np.bool_)): raise TypeError("enabled must be boolean")
        record = abi.Options()
        record.struct_size, record.version = ct.sizeof(record), 1
        record.flags, record.profile, record.capacity = int(not self.enabled), int(profile), capacity
        for name, lower, upper in [("max_nodes", 0, 1 << 20), ("max_depth", 0, 50),
                ("max_leaves", 2, 1 << 20), ("max_chart_attempts", 1, 8),
                ("max_candidate_certificates", 0, 1 << 20), ("max_pair_tests", 0, 1 << 20),
                ("max_lifts", 0, 1 << 20)]:
            value = getattr(self, name)
            if isinstance(value, (bool, np.bool_)): raise TypeError(f"{name} must be an integer")
            value = operator.index(value)
            if not lower <= value <= upper: raise ValueError(f"{name} must be in [{lower}, {upper}]")
            setattr(record, name, value)
        return record


@dataclass(frozen=True, eq=False)
class PhaseCertificate:
    """Owned local ball snapshot. This is not a native reuse-cache handle."""
    canonical_center_rad: np.ndarray
    existence_radius_rad: float
    uniqueness_radius_rad: float
    beta: float
    lipschitz: float
    eta: float
    coarse_eta: float
    inverse_residual: float
    pose_defect: float
    screw_norm_sum: float
    angular_norm_sum: float
    anchor_position_norm: float
    length_scale: float
    aspect_sign: int
    identity_schema: int
    input_words: np.ndarray
    preconditioner: np.ndarray
    rotation_deviation_upper: np.ndarray
    target_relative_trace: np.ndarray
    closed_form_determinant: np.ndarray
    reused: bool
    used_log_series: bool
    reuse_failure: CertificateFailure


@dataclass(frozen=True, eq=False)
class CatalogueSummary:
    coverage: Coverage
    preparation: QueryPreparation
    context_failure: CommandFailure
    phase_failure: PhaseFailure
    command_failure: CommandFailure
    ranking: Ranking
    identity_schema: int
    phase_count: int
    root_status: RootStatus
    root_count: int
    chart_shift: int
    chart_attempts: int
    root_nodes: int
    entire_circle_unresolved: bool
    root_leaf_count: np.ndarray
    nominal_infeasible: bool
    enumeration_complete: bool
    admission_complete: bool
    command_infeasible: bool
    returned_count: int
    truncated: bool
    has_seed: bool
    profile: CommandProfile
    numerical_status: IKStatus
    numerical_count: int
    numerical_truncated: bool
    used_singular_representatives: bool
    box_lifts: int
    yes: int
    no: int
    undecided: int
    command_count: int
    input_words: np.ndarray
    raw_words: np.ndarray
    length_scale: float
    rotation_deviation_upper: np.ndarray
    raw_provenance_available: bool = True


@dataclass(frozen=True, eq=False)
class CertifiedCommand:
    phase: int
    admission: Admission
    possible_minimizer: bool
    turns: np.ndarray
    radians: np.ndarray
    travel_squared: np.ndarray | None
    proposed_travel_squared: float


@dataclass(frozen=True, eq=False)
class CommandRange:
    phase: int
    available: bool
    empty: bool
    count: int
    first: np.ndarray
    last: np.ndarray
    base: np.ndarray


@dataclass(frozen=True, eq=False)
class RootChart:
    chart: int
    status: RootStatus
    leaf_count: int
    nodes: int
    roots: int
    exclusions: int
    unresolved: int
    max_depth: int
    descriptor: np.ndarray
    residual: np.ndarray
    derivative: np.ndarray
    determinant: np.ndarray
    omitted_residual: np.ndarray
    omitted_determinant: np.ndarray


@dataclass(frozen=True, eq=False)
class RootLeaf:
    chart: int
    index: int
    kind: RootKind
    reason: RootReason
    depth: int
    derivative_sign: int
    determinant_sign: int
    tangent: np.ndarray
    phase: np.ndarray
    endpoints: np.ndarray
    residual_bernstein: np.ndarray
    derivative_bernstein: np.ndarray
    determinant_bernstein: np.ndarray


def _release(library, destroy, handle):
    destroy(handle)


class _Owner:
    def __init__(self, library, handle):
        self.library = library
        self.handle = ct.c_void_p(handle.value)
        self.lock = threading.RLock()
        self.finalizer = weakref.finalize(self, _release, library, library.catalogue_destroy, self.handle)

    def close(self):
        with self.lock: self.finalizer()

    def read(self, name, record_type, *indices):
        with self.lock:
            if not self.finalizer.alive: raise ValueError("Catalogue audit cache is closed")
            record = record_type()
            record.struct_size, record.version = ct.sizeof(record), 1
            check(getattr(self.library, name)(self.handle, *indices, ct.byref(record)))
            if (record.struct_size, record.version, record.reserved) != (ct.sizeof(record), 1, 0):
                raise NativeError("Native catalogue returned invalid record negotiation")
            return record


def _enum(kind, value):
    try: return kind(value)
    except ValueError as exc: raise NativeError(f"Native catalogue returned unknown {kind.__name__}") from exc


def _decode(record, output_type, enums=None, booleans=()):
    values = {}
    for name in output_type.__dataclass_fields__:
        if not hasattr(record, name): continue
        value = getattr(record, name)
        values[name] = _readonly(value) if isinstance(value, ct.Array) else value
    for name, kind in (enums or {}).items(): values[name] = _enum(kind, values[name])
    for name in booleans:
        if values[name] not in (0, 1): raise NativeError("Native catalogue returned invalid boolean")
        values[name] = bool(values[name])
    return output_type(**values)


def _summary(record):
    result = _decode(record, CatalogueSummary, dict(coverage=Coverage, preparation=QueryPreparation,
        context_failure=CommandFailure, phase_failure=PhaseFailure, command_failure=CommandFailure,
        ranking=Ranking, root_status=RootStatus, profile=CommandProfile, numerical_status=IKStatus),
        ("entire_circle_unresolved", "nominal_infeasible", "enumeration_complete", "admission_complete",
         "command_infeasible", "truncated", "has_seed", "numerical_truncated", "used_singular_representatives"))
    # Conservative storage bound includes ordinary discovery plus 32*8 partial
    # exceptional nominations. It is not an exact solution-count theorem.
    if (result.phase_count > 1024 or result.command_count > 1 << 20 or result.numerical_count > 512
            or result.returned_count > min(512, result.command_count) or result.identity_schema not in (0, 1, 2)
            or result.chart_shift >= 8 or result.chart_attempts > 8
            or np.any(result.root_leaf_count > 1 << 20)):
        raise NativeError("Native catalogue returned invalid counts or identity")
    if result.enumeration_complete and result.box_lifts != result.yes+result.no+result.undecided:
        raise NativeError("Native catalogue returned inconsistent command counts")
    if result.command_infeasible and (not result.admission_complete or result.yes):
        raise NativeError("Native catalogue returned inconsistent infeasibility")
    if result.admission_complete and (not result.enumeration_complete or result.undecided):
        raise NativeError("Native catalogue returned inconsistent admission")
    if result.enumeration_complete and result.coverage != Coverage.COMPLETE:
        raise NativeError("Native catalogue enumerated an incomplete phase cover")
    if result.coverage == Coverage.COMPLETE and (result.phase_failure != PhaseFailure.SUCCESS
            or result.root_status != RootStatus.COMPLETE or result.phase_count != 2*result.root_count):
        raise NativeError("Native catalogue returned inconsistent phase coverage")
    if result.ranking == Ranking.UNIQUE and (not result.enumeration_complete or result.yes == 0):
        raise NativeError("Native catalogue returned inconsistent nearest claim")
    return result


def _index(value, bound):
    if isinstance(value, (bool, np.bool_)): raise TypeError("Index must be an integer")
    index = operator.index(value)
    if not 0 <= index < bound: raise IndexError("Catalogue index out of range")
    return index


@dataclass(frozen=True, eq=False)
class CatalogueResult:
    """Verified nominal query plus explicitly numerical fallback commands.

    commands is the requested prefix; command(i) audits all potential lifts.
    Uncertain limits and possible-minimizer bands do not assert feasibility or
    equal exact travel. Arrays own read-only buffers. close() releases only the
    native audit cache; materialized snapshots survive query/solver closure.
    """
    summary: CatalogueSummary
    phases: tuple[PhaseCertificate, ...]
    commands: tuple[CertifiedCommand, ...]
    numerical_solutions: np.ndarray
    _owner: _Owner | None = field(default=None, repr=False, compare=False)

    @property
    def coverage(self) -> Coverage: return self.summary.coverage

    @property
    def nearest_certified(self) -> bool: return self.summary.ranking == Ranking.UNIQUE

    @property
    def strata(self) -> FibreStrata | None:
        """Owned diagnostics, or None for DLLs without the optional endpoints."""
        owner = self._owner
        if owner is None or getattr(owner.library, "catalogue_strata_read", None) is None:
            return None
        with owner.lock:
            info = self._read("catalogue_strata_read", abi.Strata)
            if (info.count > 32 or info.recovery_budget_exhausted not in (0, 1)
                    or info.candidate_budget_exhausted not in (0, 1)
                    or not info.total_certified_balls <= info.total_ball_attempts <= info.nomination_count <= 1024
                    or info.total_certified_balls != self.summary.phase_count):
                raise NativeError("Native catalogue returned invalid recovery counts")
            recoveries = []
            for index in range(info.count):
                record = self._read("catalogue_recovery_read", abi.Recovery, index)
                if (record.index != index or record.source == 0 or record.source & ~15
                        or record.near_stratum & ~63 or record.candidate_count > 8
                        or not record.certified_balls <= record.attempted_balls <= record.candidate_count):
                    raise NativeError("Native catalogue returned invalid recovery metadata")
                recovery = _decode(record, FibreRecovery, dict(source=RecoverySource,
                    near_stratum=NearStratum, numerical_status=RecoveryStatus),
                    ("used_singular_representatives", "refinement_attempted"))
                count = record.candidate_count
                recovery = replace(recovery,
                    canonical_rad=_readonly(recovery.canonical_rad[:count]),
                    position_errors=_readonly(recovery.position_errors[:count]),
                    orientation_errors=_readonly(recovery.orientation_errors[:count]))
                if (not np.isfinite(recovery.canonical_rad).all()
                        or not np.isfinite(recovery.position_errors).all()
                        or not np.isfinite(recovery.orientation_errors).all()
                        or np.any(recovery.position_errors < 0) or np.any(recovery.orientation_errors < 0)
                        or (recovery.numerical_status == RecoveryStatus.CANDIDATES) != bool(count)):
                    raise NativeError("Native catalogue returned invalid numerical witnesses")
                recoveries.append(recovery)
            return FibreStrata(bool(info.recovery_budget_exhausted), info.source_domains,
                bool(info.candidate_budget_exhausted), info.total_ball_attempts,
                info.total_certified_balls, info.nomination_count, tuple(recoveries))

    def close(self) -> None:
        if self._owner is not None: self._owner.close()

    def _read(self, name, record, *indices):
        if self._owner is None: raise LibraryError("This result has no native catalogue audit cache")
        return self._owner.read(name, record, *indices)

    def phase(self, index: int) -> PhaseCertificate:
        record = self._read("catalogue_phase", BallRecord, _index(index, self.summary.phase_count))
        return PhaseCertificate(**decode_ball_record(record))

    def command(self, index: int) -> CertifiedCommand:
        record = self._read("catalogue_command_read", abi.Command, _index(index, self.summary.command_count))
        if record.cost_available not in (0, 1) or record.reserved2:
            raise NativeError("Native command returned invalid flags")
        values = _decode(record, CertifiedCommand, dict(admission=Admission), ("possible_minimizer",))
        if record.phase >= self.summary.phase_count or not np.isfinite(values.radians).all() or np.any(values.radians[:, 0] > values.radians[:, 1]):
            raise NativeError("Native command returned invalid phase or box")
        if record.cost_available:
            if not np.isfinite(values.travel_squared).all() or not 0 <= values.travel_squared[0] <= values.travel_squared[1]:
                raise NativeError("Native command returned invalid travel interval")
            return values
        return replace(values, travel_squared=None)

    def command_range(self, phase: int) -> CommandRange:
        record = self._read("catalogue_range_read", abi.Range, _index(phase, self.summary.phase_count))
        return _decode(record, CommandRange, booleans=("available", "empty"))

    def chart(self, index: int) -> RootChart:
        record = self._read("catalogue_chart_read", abi.Chart, _index(index, 2))
        return _decode(record, RootChart, dict(status=RootStatus))

    def leaf(self, chart: int, index: int) -> RootLeaf:
        chart = _index(chart, 2)
        record = self._read("catalogue_leaf_read", abi.Leaf, chart, _index(index, int(self.summary.root_leaf_count[chart])))
        return _decode(record, RootLeaf, dict(kind=RootKind, reason=RootReason))


def read_query(library, pointer):
    owner = None
    try:
        owner = _Owner(library, pointer)
        summary = _summary(owner.read("catalogue_read", abi.Summary))
        result = CatalogueResult(summary, (), (), _readonly(np.empty((0, 6))), owner)
        phases = tuple(result.phase(i) for i in range(summary.phase_count))
        if any(phase.identity_schema != summary.identity_schema or
               not np.array_equal(phase.input_words, summary.input_words) for phase in phases):
            raise NativeError("Native catalogue phase identity disagrees with summary")
        commands = tuple(result.command(i) for i in range(summary.returned_count))
        numerical = np.empty((summary.numerical_count, 6))
        for i, row in enumerate(numerical):
            check(library.catalogue_numerical(owner.handle, i, row.ctypes.data_as(ct.POINTER(ct.c_double))))
        if not np.isfinite(numerical).all(): raise NativeError("Native query returned nonfinite numerical commands")
        return replace(result, phases=phases, commands=commands, numerical_solutions=_readonly(numerical))
    except BaseException:
        if owner is None: library.catalogue_destroy(pointer)
        else: owner.close()
        raise


def old_dll_result(solver, target, limits, seed, weights, profile, capacity):
    """Numerical compatibility only; rounded limit getters supply no provenance."""
    from ._solver import CrxSolver
    box = solver._limits.copy() if limits is None else limits.copy()
    mapped = box.copy()
    mapped_seed = None if seed is None else seed.copy()
    if profile == CommandProfile.DECOUPLED_USER:
        mapped[:, 2] += mapped[:, 1]
        if mapped_seed is not None: mapped_seed[2] += mapped_seed[1]
    with CrxSolver(solver.model, dll_path=solver.dll_path, joint_limits=mapped,
                   tool=solver._tool, reference=solver._reference) as fallback:
        numerical = fallback.ik(target, seed=mapped_seed, max_solutions=512)
    rows = numerical.solutions.copy()
    if profile == CommandProfile.DECOUPLED_USER:
        rows[:, 2] -= rows[:, 1]
        rows = rows[np.all((rows >= box[0]) & (rows <= box[1]), axis=1)]
    if seed is not None:
        with np.errstate(over="ignore", invalid="ignore"):
            costs = np.sum(weights*(rows-seed)**2, axis=1)
        rows = rows[np.argsort(costs, kind="stable")]
    record = abi.Summary()
    record.version, record.struct_size = 1, ct.sizeof(record)
    record.root_status, record.entire_circle_unresolved = 2, 1
    record.context_failure = record.phase_failure = record.command_failure = 1
    record.profile, record.has_seed = int(profile), seed is not None
    record.numerical_status = int(numerical.status) if len(rows) else int(IKStatus.NO_CANDIDATE)
    if capacity == 0: record.numerical_status = int(IKStatus.NO_OUTPUT_REQUESTED)
    record.numerical_count = min(capacity, len(rows))
    record.numerical_truncated = numerical.truncated or len(rows) > capacity
    record.used_singular_representatives = numerical.used_singular_representatives
    summary = replace(_summary(record), raw_provenance_available=False,
                       raw_words=_readonly(np.empty(0, dtype=np.uint64)))
    return CatalogueResult(summary, (), (), _readonly(rows[:capacity]))
