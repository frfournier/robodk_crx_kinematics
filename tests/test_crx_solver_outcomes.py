"""Outcome policy through the C ABI, including unchanged output buffers."""

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
def test_no_output_requested(kinematics_lib, crx_10ia, capacity):
    target, _ = reference_fk(crx_10ia, [0.] * 6)
    assert call(kinematics_lib, crx_10ia, target, capacity=capacity)[0] == 0


@pytest.mark.parametrize("seeded", [False, True])
@pytest.mark.parametrize("failure", ["invalid", "unsupported", "work", "range", "empty"])
def test_failure_mapping_and_untouched_buffers(kinematics_lib, crx_10ia, seeded, failure):
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
    assert call(kinematics_lib, crx_10ia, target, seed if seeded else None)[0] == expected


@pytest.mark.parametrize("offset", [.00005, .005, .0099, .0101, .02])
@pytest.mark.parametrize("seeded", [False, True])
@pytest.mark.parametrize("alternatives", [False, True])
def test_boundary_refinement_and_unresolved_seed_policy(
    kinematics_lib, crx_10ia, offset, seeded, alternatives,
):
    seed = [0., 90., 0., 0., 0., 0.]
    target, _ = reference_fk(crx_10ia, seed)
    target[0, 3] += offset
    count, command = call(kinematics_lib, crx_10ia, target,
                          seed if seeded else None, alternatives=alternatives)
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


def test_filtered_discovery_does_not_enable_approximate_seed(kinematics_lib, crx_10ia):
    witness = [20., -40., 45., 30., -50., -60.]
    target, _ = reference_fk(crx_10ia, witness)
    assert call(kinematics_lib, crx_10ia, target)[0] > 0
    seed = witness.copy()
    seed[0] += .0001
    approximate, _ = reference_fk(crx_10ia, seed)
    assert .0001 < np.linalg.norm(approximate[:3, 3] - target[:3, 3]) < .01
    # The only admissible command is within approximate tolerance, but strict
    # discovery was nonempty. Limit filtering must not enable seed fallback.
    for axis, command in enumerate(_to_fk_api_joints_deg(seed)):
        crx_10ia.data[30][axis] = command
        crx_10ia.data[31][axis] = command
    assert call(kinematics_lib, crx_10ia, target, seed)[0] == 0
