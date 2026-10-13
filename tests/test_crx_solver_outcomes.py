"""Outcome policy through both real DLLs, including unchanged output buffers."""

import numpy as np
import pytest

from crx_reference import reference_fk
from crx_test_support import _call_command_ik, _to_fk_api_joints_deg


def call(lib, robot, target, seed=None, capacity=8, alternatives=True):
    command_seed = _to_fk_api_joints_deg(seed) if seed is not None else None
    count, best, _ = _call_command_ik(
        lib, robot, target.ravel(order="F"), command_seed, capacity, alternatives,
    )
    return count, np.array(best)


@pytest.mark.parametrize("capacity", [0, -1])
def test_no_output_requested(solver_lib, crx_10ia, capacity):
    target, _ = reference_fk(crx_10ia, [0.] * 6)
    assert call(solver_lib, crx_10ia, target, capacity=capacity)[0] == 0


@pytest.mark.parametrize("seeded", [False, True])
@pytest.mark.parametrize("failure", ["invalid", "unsupported", "work", "range", "empty"])
def test_failure_mapping_and_untouched_buffers(solver_lib, crx_10ia, seeded, failure):
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
    assert call(solver_lib, crx_10ia, target, seed if seeded else None)[0] == expected


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


@pytest.mark.parametrize("offset", [.00005, .005, .0099, .0101, .02])
@pytest.mark.parametrize("seeded", [False, True])
def test_boundary_refinement_and_unresolved_seed_policy(
    polynomial_kinematics_lib, crx_10ia, offset, seeded,
):
    seed = [0., 90., 0., 0., 0., 0.]
    target, _ = reference_fk(crx_10ia, seed)
    target[0, 3] += offset
    count, command = call(polynomial_kinematics_lib, crx_10ia, target,
                          seed if seeded else None)
    if offset < 1e-4:  # Public strict position contract.
        assert count > 0
        user = command.copy()
        user[2] -= user[1]
        actual, _ = reference_fk(crx_10ia, user)
        assert np.linalg.norm(actual[:3, 3] - target[:3, 3]) <= 1e-4
        np.testing.assert_allclose(actual[:3, :3], target[:3, :3], atol=2e-5, rtol=0)
    else:
        # A nearby seed cannot mask unresolved geometry outside the strict gate.
        assert count == -1


@pytest.mark.parametrize("offset,expected", [(.00005, 1), (.0099, 1), (.0101, 0)])
@pytest.mark.parametrize("seeded", [False, True])
def test_empty_search_position_thresholds(kinematics_lib, crx_10ia, offset, expected, seeded):
    seed = [0., 90., 0., 0., 0., 0.]
    target, _ = reference_fk(crx_10ia, seed)
    target[0, 3] += offset
    assert call(kinematics_lib, crx_10ia, target, seed if seeded else None)[0] == (
        expected if seeded else 0
    )


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


def test_filtered_discovery_does_not_enable_approximate_seed(solver_lib, crx_10ia):
    witness = [20., -40., 45., 30., -50., -60.]
    target, _ = reference_fk(crx_10ia, witness)
    assert call(solver_lib, crx_10ia, target)[0] > 0
    seed = witness.copy()
    seed[0] += .0001
    approximate, _ = reference_fk(crx_10ia, seed)
    assert .0001 < np.linalg.norm(approximate[:3, 3] - target[:3, 3]) < .01
    # The only admissible command is within approximate tolerance, but strict
    # discovery was nonempty. Limit filtering must not enable seed fallback.
    for axis, command in enumerate(_to_fk_api_joints_deg(seed)):
        crx_10ia.data[30][axis] = command
        crx_10ia.data[31][axis] = command
    assert call(solver_lib, crx_10ia, target, seed)[0] == 0
