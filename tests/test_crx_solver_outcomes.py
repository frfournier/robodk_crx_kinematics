"""Outcome policy through both real DLLs, including unchanged output buffers."""

import ctypes

import numpy as np
import pytest

from crx_reference import reference_fk
from test_crx_kinematics import _to_c_array, _to_fk_api_joints_deg


@pytest.fixture(params=["kinematics_lib", "polynomial_kinematics_lib"])
def outcome_lib(request):
    return request.getfixturevalue(request.param)


def call(lib, robot, target, seed=None, capacity=8, alternatives=True):
    best = (ctypes.c_double * 6)(*([12345.] * 6))
    output = (ctypes.c_double * (12 * max(0, capacity) + 12))(
        *([12345.] * (12 * max(0, capacity) + 12)))
    count = lib.SolveIK(
        _to_c_array(target.ravel(order="F")), best,
        output if alternatives else None, capacity,
        _to_c_array(_to_fk_api_joints_deg(seed)) if seed is not None else None,
        ctypes.byref(robot),
    )
    if count <= 0:
        assert list(best) == [12345.] * 6
        assert list(output) == [12345.] * len(output)
    elif alternatives:
        assert 0 < count <= capacity
        assert list(best) == list(output[:6])
        for index in range(count):
            assert list(output[12*index+6:12*index+12]) == [0.] * 6
        assert list(output[12*count:]) == [12345.] * (len(output) - 12*count)
    else:
        assert list(output) == [12345.] * len(output)
    return count, np.array(best)


@pytest.mark.parametrize("capacity", [0, -1])
def test_no_output_requested(outcome_lib, crx_10ia, capacity):
    target, _ = reference_fk(crx_10ia, [0.] * 6)
    assert call(outcome_lib, crx_10ia, target, capacity=capacity)[0] == 0


@pytest.mark.parametrize("seeded", [False, True])
@pytest.mark.parametrize("failure", ["invalid", "unsupported", "work", "range", "empty"])
def test_failure_mapping_and_untouched_buffers(outcome_lib, crx_10ia, seeded, failure):
    seed = [20., -40., 45., 30., -50., -60.]
    target, _ = reference_fk(crx_10ia, seed)
    expected = -1
    if failure == "invalid":
        target[0, 3] = np.nan
    elif failure == "unsupported":
        crx_10ia.data[10][1] = 1.
    elif failure in ("work", "range"):
        bound = 18000. if failure == "work" else 1e20
        for axis in range(6):
            crx_10ia.data[30][axis] = -bound
            crx_10ia.data[31][axis] = bound
    else:
        target[0, 3] = 10000.
        expected = 0
    assert call(outcome_lib, crx_10ia, target, seed if seeded else None)[0] == expected


@pytest.mark.parametrize("alternatives", [False, True])
def test_approximate_empty_search_compatibility(kinematics_lib, crx_10ia, alternatives):
    # Arm fully extended along X, with the flange orientation fixing the wrist
    # circle perpendicular to X. Moving farther along X has no exact solution.
    seed = [0., 90., 0., 0., 0., 0.]
    target, _ = reference_fk(crx_10ia, seed)
    target[0, 3] += .005
    assert call(kinematics_lib, crx_10ia, target)[0] == 0
    count, command = call(kinematics_lib, crx_10ia, target, seed,
                          alternatives=alternatives)
    assert count == 1
    np.testing.assert_allclose(command, _to_fk_api_joints_deg(seed), atol=1e-10, rtol=0)
    # Check the deliberately approximate residual with an independent FK.
    user = command.copy()
    user[2] -= user[1]
    actual, _ = reference_fk(crx_10ia, user)
    assert .0049 < np.linalg.norm(actual[:3, 3] - target[:3, 3]) < .0051
    np.testing.assert_allclose(actual[:3, :3], target[:3, :3], atol=1e-12, rtol=0)
    target[0, 3] += .015
    assert call(kinematics_lib, crx_10ia, target, seed)[0] == 0


def test_approximate_seed_still_obeys_command_limits(kinematics_lib, crx_10ia):
    seed = [0., 90., 0., 0., 0., 0.]
    target, _ = reference_fk(crx_10ia, seed)
    target[0, 3] += .005
    crx_10ia.data[31][1] = 89.999
    assert call(kinematics_lib, crx_10ia, target, seed)[0] == 0


@pytest.mark.parametrize("offset", [.00005, .005, .02])
@pytest.mark.parametrize("seeded", [False, True])
def test_unresolved_discovery_cannot_be_masked_by_seed(
    polynomial_kinematics_lib, crx_10ia, offset, seeded,
):
    seed = [0., 90., 0., 0., 0., 0.]
    target, _ = reference_fk(crx_10ia, seed)
    target[0, 3] += offset
    # The polynomial path reports unresolved geometry here, unlike the legacy
    # scanner's empty search. Even a seed within strict tolerance cannot turn
    # that handoff into success. The native test checks the private status.
    assert call(polynomial_kinematics_lib, crx_10ia, target,
                seed if seeded else None)[0] == -1


@pytest.mark.parametrize("offset,expected", [(.00005, 1), (.0099, 1), (.0101, 0)])
def test_empty_search_position_thresholds(kinematics_lib, crx_10ia, offset, expected):
    seed = [0., 90., 0., 0., 0., 0.]
    target, _ = reference_fk(crx_10ia, seed)
    target[0, 3] += offset
    assert call(kinematics_lib, crx_10ia, target, seed)[0] == expected


@pytest.mark.parametrize("angle,expected", [(.0099, 1), (.0101, 0)])
def test_empty_search_orientation_thresholds(kinematics_lib, crx_10ia, angle, expected):
    seed = [0., 90., 0., 0., 0., 0.]
    target, _ = reference_fk(crx_10ia, seed)
    target[0, 3] += .005
    radians = np.radians(angle)
    cosine, sine = np.cos(radians), np.sin(radians)
    rotation = np.array([[1, 0, 0], [0, cosine, -sine], [0, sine, cosine]])
    target[:3, :3] = rotation @ target[:3, :3]
    assert call(kinematics_lib, crx_10ia, target, seed)[0] == expected


def test_filtered_discovery_does_not_enable_approximate_seed(outcome_lib, crx_10ia):
    witness = [20., -40., 45., 30., -50., -60.]
    target, _ = reference_fk(crx_10ia, witness)
    assert call(outcome_lib, crx_10ia, target)[0] > 0
    seed = witness.copy()
    seed[0] += .0001
    approximate, _ = reference_fk(crx_10ia, seed)
    assert .0001 < np.linalg.norm(approximate[:3, 3] - target[:3, 3]) < .01
    # The only admissible command is within approximate tolerance, but strict
    # discovery was nonempty. Limit filtering must not enable seed fallback.
    for axis, command in enumerate(_to_fk_api_joints_deg(seed)):
        crx_10ia.data[30][axis] = command
        crx_10ia.data[31][axis] = command
    assert call(outcome_lib, crx_10ia, target, seed)[0] == 0
