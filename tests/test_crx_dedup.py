"""Singular home postures collapse without losing command turns or valid FK."""
import ctypes

import numpy as np
import pytest

from crx_asset_reference import forward_models
from crx_test_support import RobotT, _call_command_ik, _to_c_array
from fixtures.generate_solver_cases import corpus


MODELS, REFERENCES, _ = corpus(0, 0, 20261016)


@pytest.mark.parametrize("model_id", range(len(MODELS)),
                         ids=[model["name"] for model in MODELS])
@pytest.mark.parametrize("seeded", [False, True])
def test_home_command_deduplication(kinematics_lib, model_id, seeded):
    table = MODELS[model_id]["robot"]
    robot = RobotT.from_buffer_copy(np.ascontiguousarray(table))
    pose = (ctypes.c_double * 16)()
    assert kinematics_lib.SolveFK(_to_c_array([0]*6), pose, ctypes.byref(robot)) == 1
    seed = np.zeros(6) if seeded else None
    count, best, commands = _call_command_ik(kinematics_lib, robot, pose, seed, 4096)
    assert 0 < count < 4096
    commands = np.asarray(commands)
    near_home = np.max(np.abs(commands), axis=1) < 1e-3
    assert np.count_nonzero(near_home) == 1
    if seeded:
        np.testing.assert_array_equal(best, np.zeros(6))

    # Every admitted whole turn of home remains represented; no modulo-360
    # comparison may erase physically distinct commands.
    for axis in range(6):
        for turn in (-360., 360.):
            witness = np.zeros(6)
            witness[axis] = turn
            if np.all(witness >= table[30, :6]) and np.all(witness <= table[31, :6]):
                assert np.count_nonzero(np.max(np.abs(commands-witness), axis=1) < 1e-3) == 1
    target = np.asarray(pose).reshape(4, 4, order="F")
    poses = forward_models(REFERENCES[model_id], commands)
    assert np.all(np.linalg.norm(poses[:, :3, 3] - target[:3, 3], axis=1) <= 1e-4)
    angle = 2*np.arcsin(np.minimum(1, np.linalg.norm(
        poses[:, :3, :3] - target[:3, :3], axis=(1, 2))/(2*np.sqrt(2))))
    assert np.all(angle <= np.radians(.001))
    assert np.all(commands >= table[30, :6] - 1e-10)
    assert np.all(commands <= table[31, :6] + 1e-10)
    for capacity in (1, 4, 8):
        actual = _call_command_ik(kinematics_lib, robot, pose, seed, capacity)
        assert actual[0] == min(capacity, count)
        np.testing.assert_array_equal(actual[2], commands[:capacity])
