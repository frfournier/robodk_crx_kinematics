"""Single-output seed tracking, checked against independent asset FK."""
import numpy as np
import pytest

from crx_reference import reference_fk
from crx_test_support import (
    RobotT, _call_command_ik, _set_row, _to_fk_api_joints_deg, check_solver_pose,
)
from fixtures.generate_solver_cases import corpus


MODELS, REFERENCES, CASES = corpus(32, 14, 20261014)


@pytest.fixture(scope="module")
def nearby_models():
    robots = [RobotT.from_buffer_copy(np.ascontiguousarray(model["robot"]))
              for model in MODELS]
    return MODELS, REFERENCES, robots


@pytest.mark.parametrize("case", CASES, ids=lambda case: case["name"])
def test_nearby_family(kinematics_lib, nearby_models, case):
    check_solver_pose(kinematics_lib, nearby_models, case, True, capacity=1)


@pytest.mark.parametrize("alternatives", [False, True])
@pytest.mark.parametrize("turns", [-1, 0, 1])
@pytest.mark.parametrize("jitter", [0., .3])
def test_nearby_turns_transforms_and_packing(
    kinematics_lib, crx_10ia, alternatives, turns, jitter,
):
    _set_row(crx_10ia, 9, [120, -35, 80, .2, -.3, .4])
    _set_row(crx_10ia, 28, [20, -10, 40, -.1, .2, -.3])
    user = np.array([20., -40., 45., 30., -50., -60.])
    command = np.array(_to_fk_api_joints_deg(user)) + 360 * turns
    _set_row(crx_10ia, 30, command - 2)
    _set_row(crx_10ia, 31, command + 2)
    target, _ = reference_fk(crx_10ia, user)
    count, best, _ = _call_command_ik(
        kinematics_lib, crx_10ia, target.ravel(order="F"), command + jitter,
        capacity=1, alternatives=alternatives,
    )
    assert count == 1
    np.testing.assert_allclose(best, command, atol=1e-5, rtol=0)


@pytest.mark.parametrize("seed", [None, [160., 100., 200., -140., 80., 170.]])
def test_full_search_fallback_keeps_ranked_prefix(kinematics_lib, crx_10ia, seed):
    target, _ = reference_fk(crx_10ia, [20., -40., 45., 30., -50., -60.])
    pose = target.ravel(order="F")
    full = _call_command_ik(kinematics_lib, crx_10ia, pose, seed, capacity=64)
    single = _call_command_ik(kinematics_lib, crx_10ia, pose, seed, capacity=1)
    assert full[0] > 0 and single[0] == 1
    np.testing.assert_array_equal(single[1], full[1])


@pytest.mark.parametrize("failure", ["invalid", "unsupported", "limits", "unreachable"])
def test_nearby_cannot_bypass_validation(kinematics_lib, crx_10ia, failure):
    user = [20., -40., 45., 30., -50., -60.]
    command = _to_fk_api_joints_deg(user)
    target, _ = reference_fk(crx_10ia, user)
    if failure == "invalid":
        target[0, 0] += .001
    elif failure == "unsupported":
        crx_10ia.data[10][1] = 1.
    elif failure == "limits":
        # Only an approximate, strictly invalid seed is admitted by the box.
        command[0] += .0001
        _set_row(crx_10ia, 30, command)
        _set_row(crx_10ia, 31, command)
    else:
        target[0, 3] = 10000.
    count, _, _ = _call_command_ik(
        kinematics_lib, crx_10ia, target.ravel(order="F"), command, capacity=1,
    )
    assert count <= 0
