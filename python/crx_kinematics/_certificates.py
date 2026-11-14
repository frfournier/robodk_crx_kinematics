# SPDX-License-Identifier: Apache-2.0
# SPDX-FileCopyrightText: 2026 Centre Technologique en Aérospatiale
"""Owned local-pose proof snapshots and immutable native reuse handles."""

from contextlib import contextmanager
import ctypes as ct
from dataclasses import dataclass, field
from enum import IntEnum
import threading
import weakref

import numpy as np

from ._native import BallRecord, CertificateOptions, LibraryError, NativeError, check


class CertificateFailure(IntEnum):
    """Failed hypotheses are inconclusive; none establish infeasibility."""

    CERTIFIED = 0
    INVALID_INPUT = 1
    UNSUPPORTED_ENVIRONMENT = 2
    KINEMATICS_UNAVAILABLE = 3
    INVALID_PRECONDITIONER = 4
    INVERSE_RESIDUAL = 5
    UNCERTAIN_SIGN = 6
    TARGET_LOG_DOMAIN = 7
    RADIUS_INEQUALITY = 8
    ARITHMETIC_UNAVAILABLE = 9
    ASPECT_DISAGREEMENT = 10
    IDENTITY_MISMATCH = 11
    NOT_STRICT = 12
    NOT_REQUESTED = 13
    UNSUPPORTED_EXACT_MODEL = 14


class _CertificateOwner:
    def __init__(self, library, handle):
        self.library = library
        self.handle = ct.c_void_p(handle)
        self.lock = threading.RLock()
        self.finalizer = weakref.finalize(self, _release_cache, library,
                                          library.certificate_destroy, self.handle)

    def close(self):
        with self.lock:
            self.finalizer()

    @contextmanager
    def borrow(self, library):
        with self.lock:
            if not self.finalizer.alive:
                raise ValueError("Certificate reuse handle is closed")
            if library.path != self.library.path:
                raise ValueError("Certificate reuse requires the same DLL")
            yield self.handle


def _release_cache(library, destroy, handle):
    # Keep both the DLL owner and the original function alive through teardown.
    destroy(handle)


@dataclass(frozen=True, eq=False)
class BallCertificate:
    """One exact nominal pose root in rho, unique in R, in canonical radians.

    The read-only arrays own their buffers. The centre is explicit: rounding
    command degrees and applying raw encoder offsets need not yield that point.
    This certifies no command limits, nearest selection or global coverage.
    close() releases only the immutable native reuse cache; the proof snapshot
    remains readable and survives solver closure.
    """

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
    _owner: _CertificateOwner = field(repr=False, compare=False)

    def close(self):
        self._owner.close()

    @property
    def reusable(self):
        return self._owner.finalizer.alive


def _readonly(values, shape=None):
    result = np.array(values, copy=True)
    if shape is not None:
        result = result.reshape(shape).copy()
    result.setflags(write=False)
    return result


def read_certificate(library, handle):
    # Caller retains raw-handle ownership until this function succeeds.
    record = BallRecord()
    record.struct_size, record.version = ct.sizeof(record), 1
    check(library.certificate_read(handle, ct.byref(record)))
    if (record.struct_size != ct.sizeof(record) or record.version != 1 or record.reserved != 0
            or record.identity_schema not in (1, 2) or record.aspect_sign not in (-1, 1)
            or record.reused not in (0, 1) or record.used_log_series not in (0, 1)):
        raise NativeError("Native certificate returned invalid layout or flags")
    scalar_names = ["existence_radius_rad", "uniqueness_radius_rad", "beta", "lipschitz",
                    "eta", "coarse_eta", "inverse_residual", "pose_defect", "screw_norm_sum",
                    "angular_norm_sum", "anchor_position_norm", "length_scale"]
    scalars = {name: float(getattr(record, name)) for name in scalar_names}
    arrays = {
        "canonical_center_rad": _readonly(record.canonical_center_rad),
        "input_words": _readonly(record.input_words),
        "preconditioner": _readonly(record.preconditioner, (6, 6)),
        "rotation_deviation_upper": _readonly(record.rotation_deviation_upper),
        "target_relative_trace": _readonly(record.target_relative_trace),
        "closed_form_determinant": _readonly(record.closed_form_determinant),
    }
    if (not np.isfinite(list(scalars.values())).all()
            or any(not np.isfinite(value).all() for key, value in arrays.items() if key != "input_words")
            or not 0 <= scalars["existence_radius_rad"] <= scalars["uniqueness_radius_rad"]
            or scalars["uniqueness_radius_rad"] <= 0 or scalars["beta"] <= 0
            or scalars["lipschitz"] <= 0 or scalars["eta"] < 0 or scalars["length_scale"] <= 0
            or not 0 <= scalars["inverse_residual"] < 1
            or np.any(arrays["rotation_deviation_upper"] < 0)
            or arrays["target_relative_trace"][0] < 1
            or arrays["target_relative_trace"][0] > arrays["target_relative_trace"][1]
            or arrays["closed_form_determinant"][0] > arrays["closed_form_determinant"][1]):
        raise NativeError("Native certificate returned invalid enclosures")
    try:
        reuse_failure = CertificateFailure(record.reuse_failure)
    except ValueError as exc:
        raise NativeError("Native certificate returned an unknown reuse outcome") from exc
    return BallCertificate(**arrays, **scalars, aspect_sign=record.aspect_sign,
                           identity_schema=record.identity_schema, reused=bool(record.reused),
                           used_log_series=bool(record.used_log_series), reuse_failure=reuse_failure,
                           _owner=_CertificateOwner(library, handle))


def destroy_pending(library, handles):
    for i, pointer in enumerate(handles):
        if pointer:
            library.certificate_destroy(pointer)
            handles[i] = None


@contextmanager
def certificate_buffers(library, capacity):
    if any(getattr(library, name, None) is None for name in (
            "ik_certified", "identify_certified", "certificate_read", "certificate_destroy")):
        raise LibraryError("This DLL does not provide local pose certificates")
    options = CertificateOptions(ct.sizeof(CertificateOptions), 1, 0, 0)
    handles = (ct.c_void_p * capacity)()
    failures = np.empty(capacity, dtype=np.int32)
    try:
        yield options, handles, failures
    finally:
        destroy_pending(library, handles)


def decode_certificates(library, handles, failures, count):
    result = []
    try:
        for i in range(count):
            try:
                reason = CertificateFailure(int(failures[i]))
            except ValueError as exc:
                raise NativeError("Native certificate returned an unknown failure") from exc
            if bool(handles[i]) != (reason == CertificateFailure.CERTIFIED):
                raise NativeError("Native certificate handle and outcome disagree")
            certificate = read_certificate(library, handles[i]) if handles[i] else None
            if certificate is not None:
                handles[i] = None
            result.append(certificate)
        return tuple(result)
    except BaseException:
        for certificate in result:
            if certificate is not None:
                certificate.close()
        raise
    finally:
        destroy_pending(library, handles)
