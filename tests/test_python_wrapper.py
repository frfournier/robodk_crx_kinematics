# SPDX-License-Identifier: Apache-2.0
# SPDX-FileCopyrightText: 2026 Centre Technologique en Aérospatiale
"""Production API contracts, independent geometry, and ABI memory boundaries."""

from concurrent.futures import ThreadPoolExecutor
from contextlib import ExitStack
import ctypes as ct
import copy
import gc
import json
from pathlib import Path
import sys
import threading
import types
import weakref

import numpy as np
import pytest

from crx_kinematics import (
    ClosedSolverError, ConfigurationError, CrxSolver, FibreIdentificationResult, IKStatus,
    JointLimitError, LibraryError, NativeError,
)
from crx_kinematics import _native
from crx_asset_reference import derive_model, forward_model, rigid_inverse
from crx_reference import rotation, translation
from crx_test_support import RobotT, _set_row, _call_command_ik


CAPTURE = json.loads((Path(__file__).parent / "fixtures/crx_asset_frames.json").read_text())
ASSETS = CAPTURE["assets"]
COMMAND = np.array([23., 60., -70., -37., 51., 83.])


@pytest.fixture
def factory(kinematics_lib):
    with ExitStack() as stack:
        def create(model="CRX-10iA", **kwargs):
            return stack.enter_context(CrxSolver(model, dll_path=kinematics_lib._name, **kwargs))
        yield create


def asset_robot(asset):
    model = derive_model(asset)
    robot = RobotT()
    robot.data[1][1] = 6
    _set_row(robot, 3, np.diag(model["motion_map"]), col=4)
    for key, row in (("base", 9), ("tool", 28)):
        _set_row(robot, row, [*model[key][:3, 3], 0, 0, 0])
    for i, row in enumerate(model["dh"]):
        _set_row(robot, 10+i, [*row, 0])
    _set_row(robot, 30, asset["lower_limits_deg"])
    _set_row(robot, 31, asset["upper_limits_deg"])
    return robot


@pytest.mark.parametrize("asset", ASSETS, ids=lambda a: a["asset"])
def test_model_catalogue_and_legacy_parity(factory, kinematics_lib, asset):
    name = asset["asset"].removeprefix("Fanuc-").removesuffix("-Custom.robot")
    solver = factory(name)
    reference = derive_model(asset)
    robot = asset_robot(asset)
    np.testing.assert_allclose(solver.joint_limits,
                               [asset["lower_limits_deg"], asset["upper_limits_deg"]], atol=1e-12)
    assert solver.model == name and solver.abi_version == 1
    assert solver.dll_path.is_absolute() and solver.native_version
    for observation in [asset["zero"], *asset["probes"], *asset["samples"]]:
        q = np.array(observation["commands_deg"])
        np.testing.assert_allclose(solver.fk(q), observation["flange_pose"], atol=2e-9, rtol=0)
    # J2/J3 commands, including independent limit corners and full turns.
    commands = [COMMAND, np.zeros(6)]
    for j2 in solver.joint_limits[:, 1]:
        for j3 in solver.joint_limits[:, 2]:
            q = COMMAND.copy()
            q[1:3] = [j2, j3]
            commands.append(q)
    for q in commands:
        expected = forward_model(reference, q)[1]
        np.testing.assert_allclose(solver.fk(q), expected, atol=2e-9, rtol=0)
        result = solver.ik(expected, seed=q)
        assert result.status == IKStatus.STRICT_SEED
        np.testing.assert_allclose(result.solutions[0], q, atol=1e-9, rtol=0)
    target = solver.fk(COMMAND)
    result = solver.ik(target, seed=COMMAND, max_solutions=8)
    count, _, old = _call_command_ik(kinematics_lib, robot, target.ravel(order="F"), COMMAND, 8)
    assert count == len(result.solutions)
    np.testing.assert_allclose(result.solutions, old, atol=1e-10, rtol=0)
    for q, config, valid in zip(result.solutions, result.configurations, result.configuration_valid):
        native_config = (ct.c_double * 3)()
        code = kinematics_lib.Joints2Config((ct.c_double * 6)(*q), native_config, ct.byref(robot))
        assert valid == (code == 1)
        if valid:
            np.testing.assert_array_equal(config, native_config)


@pytest.mark.parametrize("asset", ASSETS, ids=lambda a: a["asset"])
def test_jacobian_independent_central_difference(factory, asset):
    name = asset["asset"].removeprefix("Fanuc-").removesuffix("-Custom.robot")
    tool = translation(35, -24, 92) @ rotation(0, .3)
    reference = translation(-120, 80, 45) @ rotation(1, -.4)
    solver = factory(name, tool=tool, reference=reference)
    model = derive_model(asset)

    def expected(q):
        return rigid_inverse(reference) @ forward_model(model, q)[1] @ tool

    rng = np.random.default_rng(4471)
    for q in [COMMAND, np.zeros(6), np.array([0, 0, 90, 0, 0, 0]),
              *rng.uniform(-80, 80, size=(5, 6))]:
        pose = expected(q)
        analytical = solver.jacobian(q)
        numerical = np.empty((6, 6))
        step = 1e-4  # command degrees
        for joint in range(6):
            delta = np.eye(6)[joint] * step
            plus, minus = expected(q + delta), expected(q - delta)
            dp = (plus[:3, 3] - minus[:3, 3]) / (2*step)
            dr = (plus[:3, :3] - minus[:3, :3]) / (2*step)
            omega = dr @ pose[:3, :3].T
            velocity = np.array([omega[2, 1], omega[0, 2], omega[1, 0]])
            numerical[:, joint] = np.r_[dp, velocity]
        np.testing.assert_allclose(analytical, numerical, atol=3e-8, rtol=2e-7)
        assert analytical.dtype == np.float64 and analytical.flags.owndata


def test_tool_reference_roundtrip_and_residuals(factory):
    tool = translation(35, -24, 92) @ rotation(0, .3)
    reference = translation(-120, 80, 45) @ rotation(1, -.4)
    solver = factory(tool=tool, reference=reference)
    base = factory()
    target = rigid_inverse(reference) @ base.fk(COMMAND) @ tool
    np.testing.assert_allclose(solver.fk(COMMAND), target, atol=2e-12)
    result = solver.ik(np.asfortranarray(target), seed=COMMAND)
    assert result.success and not result.is_approximate
    np.testing.assert_allclose(result.solutions[0], COMMAND, atol=1e-10)
    assert result.position_residuals_mm[0] < 1e-9
    assert result.orientation_residuals_deg[0] < 1e-9


def test_fibre_identification_metadata_and_transforms(factory):
    tool = translation(35, -24, 92) @ rotation(0, .3)
    reference = translation(-120, 80, 45) @ rotation(1, -.4)
    base = factory()
    solver = factory(tool=tool, reference=reference)
    target = rigid_inverse(reference) @ base.fk(COMMAND) @ tool
    identified = solver.identify(np.asfortranarray(target), max_solutions=64)
    assert isinstance(identified, FibreIdentificationResult)
    assert identified.success and identified.status == IKStatus.STRICT_CANDIDATES
    assert identified.solutions.ndim == 2 and identified.solutions.shape[1] == 6
    count = len(identified.solutions)
    assert identified.aspect_signs.shape == (count,)
    assert identified.regular.shape == (count,)
    assert identified.wrist_addresses.shape == (count, 3)
    assert identified.wrist_address_valid.shape == (count,)
    assert np.isfinite(identified.solutions).all()
    assert np.all(np.isin(identified.aspect_signs, (-1, 0, 1)))
    np.testing.assert_array_equal(identified.aspect_signs == 0, ~identified.regular)
    assert np.all(identified.wrist_addresses[~identified.wrist_address_valid] == -1)
    for q, sign, regular in zip(identified.solutions,
                                identified.aspect_signs,
                                identified.regular):
        jacobian_det = np.linalg.det(solver.jacobian(q))
        if regular:
            assert sign == (1 if jacobian_det > 0 else -1)
        else:
            assert sign == 0
    assert any(np.linalg.norm(q - COMMAND) < 1e-8 for q in identified.solutions)
    single = solver.identify(target, max_solutions=1)
    assert len(single.solutions) == 1 and single.truncated
    assert single.aspect_signs.shape == (1,)
    assert single.wrist_addresses.shape == (1, 3)
    narrow = factory(joint_limits=np.array([COMMAND-.25, COMMAND+.25]),
                     tool=tool, reference=reference)
    limited = narrow.identify(target, max_solutions=8)
    assert len(limited.solutions) == 1
    np.testing.assert_allclose(limited.solutions[0], COMMAND, atol=1e-9, rtol=0)


def test_fibre_identification_singular_and_unaddressable_rows(factory):
    solver = factory()
    q = [0, 0, 90, 0, 0, 0]
    result = solver.identify(solver.fk(q), max_solutions=64)
    assert result.success and np.any(~result.regular)
    np.testing.assert_array_equal(result.aspect_signs[~result.regular], 0)
    assert np.all(result.wrist_addresses[~result.wrist_address_valid] == -1)


def test_identification_requires_optional_native_symbol(factory):
    solver = factory()
    solver._library.identify = None
    with pytest.raises(NotImplementedError, match="0.5.0 or newer"):
        solver.identify(solver.fk(COMMAND))


def test_singular_configuration_does_not_discard_ik(factory):
    solver = factory()
    q = [0, 0, 90, 0, 0, 0]
    with pytest.raises(ConfigurationError):
        solver.configuration(q)
    result = solver.ik(solver.fk(q), seed=q)
    assert result.success
    assert not result.configuration_valid[0]
    np.testing.assert_array_equal(result.configurations[0], [-1, -1, -1])
    assert np.isfinite(solver.jacobian(q)).all()


def test_empty_numerical_and_truncated_results(factory):
    solver = factory()
    target = solver.fk(COMMAND)
    first = solver.ik(target, max_solutions=1)
    many = solver.ik(target, max_solutions=64)
    assert first.truncated and len(many.solutions) > 1
    np.testing.assert_allclose(first.solutions, many.solutions[:1], atol=1e-10)
    assert list(map(tuple, many.solutions)) == sorted(map(tuple, many.solutions))
    target[0, 3] = 10000
    empty = solver.ik(target)
    assert empty.status == IKStatus.NO_CANDIDATE and not empty.success
    assert empty.solutions.shape == (0, 6) and empty.configurations.shape == (0, 3)
    for bound, status in ((18000, IKStatus.WORK_LIMIT), (1e20, IKStatus.NUMERICAL_RANGE_FAILURE)):
        broad = factory(joint_limits=np.array([[-bound]*6, [bound]*6]))
        result = broad.ik(solver.fk(COMMAND), max_solutions=8)
        assert result.status == status and not result.success


def test_approximate_result_metadata_and_residuals(factory, monkeypatch):
    # Deterministic boundary test: drive the rare approximate outcome through
    # the ABI seam without depending on discovery reaching that fallback.
    solver = factory()
    target = solver.fk(COMMAND)
    target[0, 3] += .005

    def approximate(handle, pose, seed, capacity, output, metadata):
        output[0] = COMMAND
        result = ct.cast(metadata, ct.POINTER(_native.IKMetadata)).contents
        result.status, result.count = int(IKStatus.APPROXIMATE_SEED), 1
        return 0

    monkeypatch.setattr(solver._library, "ik", approximate)
    result = solver.ik(target, seed=COMMAND)
    assert result.success and result.is_approximate
    np.testing.assert_allclose(result.position_residuals_mm, [.005], atol=1e-10)


@pytest.mark.parametrize("value", [np.zeros((6, 1)), [0]*5, [0]*7,
                                  [0, 0, np.nan, 0, 0, 0], [0, 0, np.inf, 0, 0, 0]])
def test_reject_bad_vectors(factory, value):
    solver = factory()
    for method in (solver.fk, solver.configuration, solver.jacobian):
        with pytest.raises(ValueError):
            method(value)
    with pytest.raises(ValueError):
        solver.ik(np.eye(4), seed=value)


@pytest.mark.parametrize("value", [np.zeros(6, dtype=complex), ["0"]*6,
                                  np.zeros(6, dtype=object), [False]*6])
def test_reject_nonreal_dtypes(factory, value):
    with pytest.raises(TypeError):
        factory().fk(value)


@pytest.mark.parametrize("change", ["bottom", "scale", "reflection", "nan", "flat"])
def test_invalid_rigid_pose(factory, change):
    solver = factory()
    pose = np.eye(4)
    if change == "bottom": pose[3, 0] = 1e-6
    if change == "scale": pose[0, 0] = 1.001
    if change == "reflection": pose[0, 0] = -1
    if change == "nan": pose[0, 3] = np.nan
    if change == "flat": pose = pose.ravel()
    with pytest.raises(ValueError): solver.ik(pose)
    with pytest.raises(ValueError): factory(tool=pose)
    with pytest.raises(ValueError): factory(reference=pose)


def test_capacity_limits_and_model_validation(factory):
    solver = factory()
    for bad in (0, -1, 2**31):
        with pytest.raises(ValueError): solver.ik(np.eye(4), max_solutions=bad)
    for bad in (True, 1.5, "2"):
        with pytest.raises(TypeError): solver.ik(np.eye(4), max_solutions=bad)
        with pytest.raises(TypeError): solver.identify(np.eye(4), max_solutions=bad)
    for bad in (0, -1, 2**31):
        with pytest.raises(ValueError): solver.identify(np.eye(4), max_solutions=bad)
    with pytest.raises(ValueError): factory("unknown")
    with pytest.raises(ValueError): factory(joint_limits=[[1]*6, [0]*6])
    for method in (solver.fk, solver.jacobian):
        with pytest.raises(JointLimitError): method([1000]*6)
    narrow = factory(joint_limits=np.array([COMMAND-.25, COMMAND+.25]))
    result = narrow.ik(solver.fk(COMMAND), seed=COMMAND)
    np.testing.assert_allclose(result.solutions[0], COMMAND, atol=1e-9)


def test_array_layout_ownership_and_immutability(factory):
    tool = np.eye(4)
    solver = factory(tool=tool)
    tool[0, 3] = 999
    assert solver.tool[0, 3] == 0
    q = np.arange(12, dtype=np.float32)[::2]
    q.flags.writeable = False
    original = q.copy()
    expected = solver.fk(q.astype(float))
    for value in (q, q.astype(">f8"), q[::-1][::-1], q.astype(np.int16)):
        np.testing.assert_allclose(solver.fk(value), expected, atol=1e-12)
    unaligned = np.ndarray((6,), dtype=np.float64, buffer=bytearray(49), offset=1)
    unaligned[:] = q
    np.testing.assert_allclose(solver.fk(unaligned), expected, atol=1e-12)
    np.testing.assert_array_equal(q, original)
    limits = solver.joint_limits
    limits[:] = 0
    assert np.any(solver.joint_limits != 0)
    result = solver.ik(expected, seed=q)
    arrays = [expected, solver.configuration(q), solver.jacobian(q),
              result.solutions, result.configurations, result.configuration_valid,
              result.position_residuals_mm, result.orientation_residuals_deg]
    for array in arrays:
        assert array.flags.owndata and array.flags.c_contiguous and array.flags.aligned
    solver.close()
    solver.close()
    for array in arrays:
        assert np.isfinite(array).all()
    with pytest.raises(ClosedSolverError): solver.fk(q)


def test_independent_handles_threads_and_finalization(factory, kinematics_lib, monkeypatch):
    first, second = factory(), factory("CRX-25iA")
    expected = [first.fk(COMMAND), second.fk(COMMAND)]
    with ThreadPoolExecutor(4) as pool:
        outputs = list(pool.map(lambda i: (first, second)[i % 2].fk(COMMAND), range(16)))
    for i, output in enumerate(outputs):
        np.testing.assert_array_equal(output, expected[i % 2])
    transient = CrxSolver("CRX-10iA", dll_path=kinematics_lib._name)
    library = transient._library
    destroy = library.destroy
    destroyed = []
    monkeypatch.setattr(library, "destroy", lambda handle: (destroyed.append(True), destroy(handle)))
    ref = weakref.ref(transient)
    del transient
    gc.collect()
    assert ref() is None and destroyed == [True]


def test_close_waits_for_active_call(factory, monkeypatch):
    solver = factory()
    entered, finish = threading.Event(), threading.Event()
    fk = solver._library.fk

    def blocked(*args):
        entered.set()
        assert finish.wait(10)
        return fk(*args)

    monkeypatch.setattr(solver._library, "fk", blocked)
    with ThreadPoolExecutor(2) as pool:
        operation = pool.submit(solver.fk, COMMAND)
        assert entered.wait(10)
        closure = pool.submit(solver.close)
        try:
            assert not closure.done()
        finally:
            finish.set()
        assert operation.result().shape == (4, 4)
        closure.result()


def test_discovery_and_explicit_path_do_not_connect(tmp_path, monkeypatch):
    extension = tmp_path / "bin/robotextensions/crx_kinematics.dll"
    extension.parent.mkdir(parents=True)
    extension.write_bytes(b"test")
    calls = []
    module = types.ModuleType("robodk.robolink")
    module.getPathRoboDK = lambda: (calls.append(True), str(tmp_path / "bin/RoboDK.exe"))[1]
    monkeypatch.setitem(sys.modules, "robodk.robolink", module)
    assert _native.resolve_library(extension) == extension
    assert calls == []
    assert _native.resolve_library(None) == extension
    assert calls == [True]
    with pytest.raises(LibraryError): _native.resolve_library(tmp_path / "missing.dll")
    assert calls == [True]
    monkeypatch.setitem(sys.modules, "robodk.robolink", None)
    with pytest.raises(LibraryError, match="extra"): _native.resolve_library(None)


@pytest.mark.parametrize("failure", ["missing_exports", "abi", "architecture", "dependency"])
def test_loader_failures_cleanup(tmp_path, monkeypatch, failure):
    path = tmp_path / "test.dll"
    path.write_bytes(b"test")
    closed = []
    monkeypatch.setattr(_native.os, "add_dll_directory",
                        lambda _: types.SimpleNamespace(close=lambda: closed.append(True)))

    class Function:
        def __call__(self): return 999

    def load(_):
        if failure in ("architecture", "dependency"):
            raise OSError("test loader failure")
        return (types.SimpleNamespace(crx_v1_abi_version=Function())
                if failure == "abi" else types.SimpleNamespace())

    monkeypatch.setattr(_native.ct, "CDLL", load)
    with pytest.raises(LibraryError): _native.NativeLibrary(path)
    assert closed == [True]


def test_native_buffers_and_ndpointer_guards(factory):
    solver = factory()
    native = solver._library
    pose = np.full((4, 4), 12345.)
    assert native.fk(solver._handle, np.full(6, np.nan), pose) == -1
    assert np.all(pose == 12345.)
    assert native.fk(solver._handle, np.full(6, 1000.), pose) == -2
    assert np.all(pose == 12345.)
    with pytest.raises(ct.ArgumentError): native.fk(solver._handle, np.zeros(6, dtype=np.float32), pose)
    with pytest.raises(ct.ArgumentError): native.fk(solver._handle, np.zeros(6), pose.T)
    pose.flags.writeable = False
    with pytest.raises(ct.ArgumentError): native.fk(solver._handle, np.zeros(6), pose)
    output = np.full((9, 6), 12345.)
    metadata = _native.IKMetadata()
    target = solver.fk(COMMAND)
    assert native.ik(solver._handle, target, None, 8, output, ct.byref(metadata)) == 0
    assert 0 < metadata.count <= 8
    assert np.all(output[metadata.count:] == 12345.)
    output[:] = 12345.
    target[0, 3] = np.nan
    assert native.ik(solver._handle, target, None, 8, output, ct.byref(metadata)) == 0
    assert metadata.status == IKStatus.INVALID_INPUT and metadata.count == 0
    assert np.all(output == 12345.)


def test_bad_native_count_and_error_mapping(factory, monkeypatch):
    solver = factory()

    def corrupt(handle, pose, seed, capacity, output, metadata):
        ct.cast(metadata, ct.POINTER(_native.IKMetadata)).contents.count = capacity + 1
        return 0

    monkeypatch.setattr(solver._library, "ik", corrupt)
    with pytest.raises(NativeError, match="count"): solver.ik(np.eye(4))
    monkeypatch.setattr(solver._library, "ik", lambda *args: -4)
    with pytest.raises(NativeError): solver.ik(np.eye(4))


def test_solver_cannot_share_handle_through_copy(factory):
    solver = factory()
    for copier in (copy.copy, copy.deepcopy):
        with pytest.raises(TypeError, match="cannot be copied"):
            copier(solver)
    assert np.isfinite(solver.fk(COMMAND)).all()


def test_transform_numerical_overflow_is_explicit(factory):
    reference = np.eye(4)
    reference[0, 3] = 1.7e308
    solver = factory(reference=reference)
    target = np.eye(4)
    target[0, 3] = 1.7e308
    with pytest.raises(ValueError, match="numerical range"):
        solver.ik(target)


def test_constructor_failure_destroys_handle(factory, monkeypatch):
    solver = factory()
    library = solver._library
    create, destroy = library.create, library.destroy
    calls = []

    def record_create(*args):
        calls.append("create")
        return create(*args)

    def record_destroy(handle):
        calls.append("destroy")
        destroy(handle)

    monkeypatch.setattr(library, "create", record_create)
    monkeypatch.setattr(library, "destroy", record_destroy)
    monkeypatch.setattr(library, "limits", lambda *args: -4)
    monkeypatch.setattr("crx_kinematics._solver.NativeLibrary", lambda _: library)
    with pytest.raises(NativeError):
        CrxSolver("CRX-10iA", dll_path=solver.dll_path)
    assert calls == ["create", "destroy"]
