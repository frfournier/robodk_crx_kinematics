# SPDX-License-Identifier: Apache-2.0
# SPDX-FileCopyrightText: 2026 Centre Technologique en Aérospatiale
"""Linked DLL proof soundness, additive ABI, ownership and wrapper contracts."""

from concurrent.futures import ThreadPoolExecutor
from contextlib import ExitStack
import ctypes as ct
import gc
import math
import shutil
import weakref

import mpmath as mp
import numpy as np
import pytest

from crx_kinematics import (BallCertificate, CertificateFailure, Coverage, CrxSolver,
                            IKStatus, LibraryError, NativeError, _native)
from crx_reference import rotation, translation
from test_crx_ball_certificate import check_certificate, exact_kinematics
from test_crx_exact_target import unflatten

COMMAND = np.array([20., 30., 40., 50., 60., 70.])


@pytest.mark.parametrize("capacity", [1, 64, 512])
def test_certified_identification_preserves_capacity_limited_turn_prefix(factory, capacity):
    solver = factory(joint_limits=np.array([[-720.]*6, [720.]*6]))
    target = solver.fk(COMMAND)
    numerical = solver.identify(target, max_solutions=capacity)
    result = solver.identify(target, max_solutions=capacity, certify=True)
    try:
        assert len(result.solutions) == capacity and result.truncated
        np.testing.assert_array_equal(result.solutions, numerical.solutions)
        assert result.status == numerical.status
        assert result.truncated == numerical.truncated
        assert result.used_singular_representatives == numerical.used_singular_representatives
        np.testing.assert_array_equal(result.wrist_addresses, numerical.wrist_addresses)
        np.testing.assert_array_equal(result.wrist_address_valid, numerical.wrist_address_valid)
        assert result.coverage == numerical.coverage == Coverage.NOT_ATTEMPTED
        mask = result.regular_certified
        assert mask.any()
        np.testing.assert_array_equal(mask, result.certificate_failures == CertificateFailure.CERTIFIED)
        assert result.regular[mask].all()
        for i in np.flatnonzero(mask):
            assert result.aspect_signs[i] == result.certificates[i].aspect_sign
        np.testing.assert_array_equal(result.regular[~mask], numerical.regular[~mask])
        np.testing.assert_array_equal(result.aspect_signs[~mask], numerical.aspect_signs[~mask])
        with mp.workdps(200):
            successful = np.flatnonzero(mask)
            for i in dict.fromkeys([successful[0], successful[-1]]):
                validate_proof(result.certificates[i], target, np.eye(4), np.eye(4))
    finally:
        close_proofs(result)


@pytest.fixture
def factory(kinematics_lib):
    with ExitStack() as stack:
        def create(model="CRX-10iA", **kwargs):
            return stack.enter_context(CrxSolver(model, dll_path=kinematics_lib._name, **kwargs))
        yield create


def close_proofs(result):
    for value in result.certificates or ():
        if value is not None:
            value.close()


def validate_proof(certificate, target, reference, tool):
    assert isinstance(certificate, BallCertificate) and certificate.identity_schema == 2
    raw = certificate.input_words.view(np.float64).tolist()
    input = unflatten(raw)
    for name, pose in [("target", target), ("base", reference), ("tool", tool)]:
        np.testing.assert_array_equal(input[name]["position"], pose[:3, 3])
        np.testing.assert_array_equal(input[name]["input_rotation"], pose[:3, :3].ravel())
    answer = {"failure": 0, "centre": certificate.canonical_center_rad.tolist(),
              "constants": [certificate.existence_radius_rad, certificate.uniqueness_radius_rad,
                            certificate.inverse_residual, certificate.beta, certificate.lipschitz,
                            certificate.eta, certificate.pose_defect, certificate.screw_norm_sum,
                            certificate.angular_norm_sum, certificate.anchor_position_norm,
                            certificate.aspect_sign],
              "trace": certificate.target_relative_trace.tolist(),
              "preconditioner": certificate.preconditioner.ravel().tolist(),
              "coarse_eta": certificate.coarse_eta, "used_log_series": certificate.used_log_series}
    expected = exact_kinematics(input, answer["centre"], reference_mode=True)
    check_certificate(answer, kinematics=expected)
    for bound, squared in zip(certificate.rotation_deviation_upper,
                               expected[0]["deviations_squared"], strict=True):
        assert mp.mpf(float(bound))**2 >= mp.mpf(squared.numerator)/squared.denominator
        assert bound < 1e-12
    for array in [certificate.canonical_center_rad, certificate.input_words,
                  certificate.preconditioner, certificate.rotation_deviation_upper]:
        assert array.flags.owndata and not array.flags.writeable


def test_linked_identification_certificates_enclose_independent_roots(factory):
    solver = factory()
    target = solver.fk(COMMAND)
    numerical = solver.identify(target)
    result = solver.identify(target, certify=True)
    try:
        np.testing.assert_array_equal(result.solutions, numerical.solutions)
        assert len(result.solutions) > 1 and result.regular_certified.all()
        assert result.coverage == Coverage.NOT_ATTEMPTED
        assert numerical.certificates is None and not numerical.regular_certified.any()
        assert np.all(result.certificate_failures == CertificateFailure.CERTIFIED)
        with mp.workdps(200):
            for certificate, sign in zip(result.certificates, result.aspect_signs, strict=True):
                validate_proof(certificate, target, np.eye(4), np.eye(4))
                assert certificate.aspect_sign == sign
    finally:
        close_proofs(result)


@pytest.mark.parametrize("offset,status", [(0., IKStatus.STRICT_SEED), (.01, IKStatus.STRICT_LOCAL)])
def test_strict_seed_and_local_attachments(factory, offset, status):
    solver = factory()
    target = solver.fk(COMMAND)
    result = solver.ik(target, seed=COMMAND+offset, certify=True)
    try:
        assert result.status == status and result.certificates[0] is not None
        with mp.workdps(200):
            validate_proof(result.certificates[0], target, np.eye(4), np.eye(4))
    finally:
        close_proofs(result)


@pytest.mark.parametrize("turns", [False, True])
def test_raw_reference_tool_and_full_turns(factory, turns):
    tool = translation(35, -24, 92) @ rotation(0, .3)
    reference = translation(-120, 80, 45) @ rotation(1, -.4)
    limits = np.array([[-720.]*6, [720.]*6]) if turns else None
    solver = factory(reference=reference, tool=tool, joint_limits=limits)
    command = COMMAND.copy()
    if turns:
        command[[0, 5]] += [360., -360.]
    target = solver.fk(command)
    result = solver.ik(target, seed=command, certify=True)
    try:
        assert result.status == IKStatus.STRICT_SEED and result.certificates[0] is not None
        # Compare the legacy degree/radian roundtrip, including its represented
        # command values, rather than assuming integer degrees echo bitwise.
        np.testing.assert_array_equal(result.solutions, solver.ik(target, seed=command).solutions)
        with mp.workdps(200):
            validate_proof(result.certificates[0], target, reference, tool)
        if turns:
            assert result.certificates[0].canonical_center_rad[0] > 2*math.pi
    finally:
        close_proofs(result)


def test_actual_approximate_fallback_never_receives_certificate(factory):
    solver = factory()
    # Triangle-reach maximum: arm aligns with the offset forearm/flange.
    q = np.array([0., 0., 90+math.degrees(math.atan2(150, 700)), -90., 0., 0.])
    target = solver.fk(q)
    target[:3, 3] += .005*target[:3, 3]/np.linalg.norm(target[:3, 3])
    result = solver.ik(target, seed=q, max_solutions=64, certify=True)
    assert result.status == IKStatus.APPROXIMATE_SEED
    assert result.certificates == (None,)
    np.testing.assert_array_equal(result.certificate_failures, [CertificateFailure.NOT_STRICT])


def test_cache_survives_solver_close_reuse_revalidates_and_cache_close_is_independent(factory):
    first = factory()
    target = first.fk(COMMAND)
    initial = first.ik(target, seed=COMMAND, certify=True)
    old = initial.certificates[0]
    assert old is not None
    first.close()
    next_solver = factory()
    changed = target.copy()
    changed[0, 3] += 1e-7
    result = next_solver.ik(changed, seed=COMMAND, certify=True, reuse=old)
    try:
        current = result.certificates[0]
        assert current is not None and current.reused
        assert current.reuse_failure == CertificateFailure.CERTIFIED
        with mp.workdps(200):
            validate_proof(current, changed, np.eye(4), np.eye(4))
        other = factory("CRX-5iA")
        other_result = other.ik(other.fk(COMMAND), seed=COMMAND, certify=True, reuse=old)
        try:
            fresh = other_result.certificates[0]
            assert fresh is not None and not fresh.reused
            assert fresh.reuse_failure == CertificateFailure.IDENTITY_MISMATCH
        finally:
            close_proofs(other_result)
        np.testing.assert_equal(np.shares_memory(old.input_words, current.input_words), False)
        old.close()
        old.close()
        assert not old.reusable and current.reusable and old.beta > 0
        with pytest.raises(ValueError, match="closed"):
            next_solver.ik(changed, seed=COMMAND, certify=True, reuse=old)
    finally:
        old.close()
        close_proofs(result)


def test_native_read_concurrency_and_python_finalizer(factory, monkeypatch):
    solver = factory()
    library = solver._library
    destroyed = []
    destroy = library.certificate_destroy
    def tracking(pointer):
        destroyed.append(pointer.value if isinstance(pointer, ct.c_void_p) else pointer)
        destroy(pointer)
    monkeypatch.setattr(library, "certificate_destroy", tracking)
    result = solver.ik(solver.fk(COMMAND), seed=COMMAND, certify=True)
    certificate = result.certificates[0]
    def read(_):
        record = _native.BallRecord()
        record.struct_size, record.version = ct.sizeof(record), 1
        assert library.certificate_read(certificate._owner.handle, ct.byref(record)) == 0
        return record.beta
    with ThreadPoolExecutor(max_workers=4) as executor:
        assert list(executor.map(read, range(24))) == [certificate.beta]*24
    weak = weakref.ref(certificate)
    del certificate, result
    gc.collect()
    assert weak() is None and len(destroyed) == 1


def test_old_dll_optional_exports_and_reuse_validation(factory, monkeypatch, tmp_path):
    solver = factory()
    target = solver.fk(COMMAND)
    for name in ["ik_certified", "identify_certified", "certificate_read", "certificate_destroy"]:
        monkeypatch.setattr(solver._library, name, None)
    assert solver.ik(target).certificates is None
    assert solver.identify(target).certificates is None
    with pytest.raises(LibraryError, match="certificates"):
        solver.ik(target, certify=True)
    with pytest.raises(LibraryError, match="certificates"):
        solver.identify(target, certify=True)
    with pytest.raises(TypeError, match="boolean"):
        solver.ik(target, certify=1)
    with pytest.raises(TypeError, match="boolean"):
        solver.identify(target, certify=1)
    with pytest.raises(TypeError, match="BallCertificate"):
        solver.ik(target, certify=True, reuse=object())
    source = factory()
    result = source.ik(target, seed=COMMAND, certify=True)
    try:
        with pytest.raises(ValueError, match="requires certify"):
            source.ik(target, reuse=result.certificates[0])
        duplicate = tmp_path/"copied.dll"
        shutil.copy2(source.dll_path, duplicate)
        with CrxSolver("CRX-10iA", dll_path=duplicate) as other:
            with pytest.raises(ValueError, match="same DLL"):
                other.ik(target, seed=COMMAND, certify=True, reuse=result.certificates[0])
    finally:
        close_proofs(result)


@pytest.mark.parametrize("field,bad", [("struct_size", 0), ("struct_size", ct.sizeof(_native.CertificateOptions)+4),
                                      ("version", 2), ("flags", 2), ("reserved", 1)])
def test_c_negotiation_is_atomic(factory, field, bad):
    solver = factory()
    library = solver._library
    target = solver.fk(COMMAND)
    capacity = 3
    solutions = np.full((capacity+2, 6), 12345.)
    failures = np.full(capacity+2, 12345, dtype=np.int32)
    handles = (ct.c_void_p*(capacity+2))(*([0x12345]*(capacity+2)))
    options = _native.CertificateOptions(ct.sizeof(_native.CertificateOptions), 1, 0, 0)
    setattr(options, field, bad)
    before = bytes(options)
    metadata = _native.IKMetadata(99, 99, 99, 99)
    assert library.ik_certified(solver._handle, target, np.eye(4), np.eye(4), None,
        capacity, solutions, ct.byref(metadata), handles, failures, ct.byref(options), None) == -1
    assert (metadata.status, metadata.count) == (int(IKStatus.INVALID_INPUT), 0)
    assert np.all(solutions == 12345) and np.all(failures == 12345)
    assert list(handles) == [0x12345]*(capacity+2) and bytes(options) == before


def test_c_enabled_disabled_candidates_canaries_and_certificate_read_layout(factory):
    solver = factory()
    library = solver._library
    target = solver.fk(COMMAND)
    capacity = 3
    outputs = []
    for flag in [1, 0]:
        solutions = np.full((capacity+2, 6), 12345.)
        failures = np.full(capacity+2, 12345, dtype=np.int32)
        handles = (ct.c_void_p*(capacity+2))()
        handles[capacity] = handles[capacity+1] = 0x12345
        options = _native.CertificateOptions(ct.sizeof(_native.CertificateOptions), 1, flag, 0)
        metadata = _native.IKMetadata()
        try:
            assert library.ik_certified(solver._handle, target, np.eye(4), np.eye(4), None,
                capacity, solutions, ct.byref(metadata), handles, failures, ct.byref(options), None) == 0
            assert metadata.count == capacity and metadata.truncated
            assert np.all(solutions[capacity:] == 12345) and np.all(failures[capacity:] == 12345)
            assert list(handles)[capacity:] == [0x12345, 0x12345]
            if flag:
                assert list(handles)[:capacity] == [None]*capacity
                assert np.all(failures[:capacity] == CertificateFailure.NOT_REQUESTED)
            else:
                assert all(list(handles)[:capacity]) and np.all(failures[:capacity] == 0)
                for field, bad in [("struct_size", 0), ("version", 2), ("reserved", 1)]:
                    record = _native.BallRecord()
                    record.struct_size, record.version = ct.sizeof(record), 1
                    setattr(record, field, bad)
                    before = bytes(record)
                    assert library.certificate_read(handles[0], ct.byref(record)) == -1
                    assert bytes(record) == before
                class Padded(ct.Structure):
                    _fields_ = [("before", ct.c_uint64*2), ("record", _native.BallRecord),
                                ("after", ct.c_uint64*2)]
                padded = Padded()
                padded.before[:] = padded.after[:] = [0xABCDEF]*2
                padded.record.struct_size, padded.record.version = ct.sizeof(_native.BallRecord), 1
                assert library.certificate_read(handles[0], ct.byref(padded.record)) == 0
                assert list(padded.before) == list(padded.after) == [0xABCDEF]*2
            outputs.append(solutions[:capacity].copy())
        finally:
            for pointer in list(handles)[:metadata.count]:
                if pointer:
                    library.certificate_destroy(pointer)
    np.testing.assert_array_equal(*outputs)
    library.certificate_destroy(None)


@pytest.mark.parametrize("field,bad", [("struct_size", 0), ("version", 2), ("reserved", 1)])
def test_c_identification_metadata_negotiation_preserves_all_buffers(factory, field, bad):
    solver = factory()
    capacity = 3
    target = solver.fk(COMMAND)
    solutions = np.full((capacity+2, 6), 12345.)
    signs = np.full(capacity+2, 12345, dtype=np.int32)
    regular = signs.copy()
    addresses = np.full((capacity+2, 3), 12345, dtype=np.int32)
    valid = signs.copy()
    failures = signs.copy()
    handles = (ct.c_void_p*(capacity+2))(*([0x12345]*(capacity+2)))
    metadata = _native.FibreMetadata()
    extension = _native.FibreExtension(ct.sizeof(_native.FibreExtension), 1, 12345, 0)
    setattr(extension, field, bad)
    before = bytes(extension)
    options = _native.CertificateOptions(ct.sizeof(_native.CertificateOptions), 1, 0, 0)
    assert solver._library.identify_certified(solver._handle, target, np.eye(4), np.eye(4),
        capacity, solutions, signs, regular, addresses, valid, ct.byref(metadata),
        ct.byref(extension), handles, failures, ct.byref(options)) == -1
    assert metadata.count == 0 and metadata.status == IKStatus.INVALID_INPUT
    assert all(np.all(array == 12345) for array in [solutions, signs, regular, addresses, valid, failures])
    assert bytes(extension) == before and list(handles) == [0x12345]*(capacity+2)


@pytest.mark.parametrize("fault", ["status", "count", "record", "reason"])
def test_wrapper_frees_every_native_proof_on_failed_response(factory, monkeypatch, fault):
    solver = factory()
    library = solver._library
    destroyed = []
    original_destroy = library.certificate_destroy
    def destroy(pointer):
        destroyed.append(pointer.value if isinstance(pointer, ct.c_void_p) else pointer)
        original_destroy(pointer)
    monkeypatch.setattr(library, "certificate_destroy", destroy)
    target = solver.fk(COMMAND)
    if fault == "record":
        original_read = library.certificate_read
        reads = []
        def read(pointer, output):
            result = original_read(pointer, output)
            reads.append(1)
            if len(reads) == 2:
                ct.cast(output, ct.POINTER(_native.BallRecord)).contents.eta = math.nan
            return result
        monkeypatch.setattr(library, "certificate_read", read)
        with pytest.raises(NativeError, match="invalid enclosures"):
            solver.identify(target, certify=True)
        assert len(destroyed) > 1 and len(destroyed) == len(set(destroyed))
    else:
        original = library.ik_certified
        def bad(*args):
            result = original(*args)
            metadata = ct.cast(args[7], ct.POINTER(_native.IKMetadata)).contents
            if fault == "status":
                metadata.status = 999
            elif fault == "count":
                metadata.count = 2
            else:
                args[9][0] = 999
            return result
        monkeypatch.setattr(library, "ik_certified", bad)
        with pytest.raises(NativeError, match="unknown|invalid"):
            solver.ik(target, seed=COMMAND, certify=True)
        assert len(destroyed) == 1
