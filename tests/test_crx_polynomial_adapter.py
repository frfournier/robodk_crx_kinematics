"""Comparison DLL: canonical discovery through unchanged RoboDK finalization."""

import ctypes
import json
from pathlib import Path

import numpy as np
import pytest

from conftest import RobotT, _set_row
from crx_asset_reference import derive_model, forward_model
from crx_reference import missing_postures, reference_fk, xyzwpr
from test_crx_discovery import (
    discovery_joints, test_seed_free_discovery as _check_seed_free_discovery,
)
from test_crx_kinematics import _call_ik, _to_c_array, _to_fk_api_joints_deg

FIXTURE_DIR = Path(__file__).parent / "fixtures"
ASSETS = json.loads((FIXTURE_DIR / "crx_asset_frames.json").read_text())["assets"]
CASES = json.loads((FIXTURE_DIR / "CRX10iA-solutions.json").read_text())["test_cases"]


def check_results(robot, target, result):
    count, best, solutions = result
    assert count == len(solutions)
    if count:
        assert best == solutions[0]
    for solution in solutions:
        actual, _ = reference_fk(robot, solution)
        np.testing.assert_allclose(actual[:3, 3], target[:3, 3], atol=1.0001e-4, rtol=0)
        np.testing.assert_allclose(actual[:3, :3], target[:3, :3], atol=2e-5, rtol=0)
        command = np.array(_to_fk_api_joints_deg(solution))
        assert np.all(command >= np.array(robot.data[30][:6]) - 1e-10)
        assert np.all(command <= np.array(robot.data[31][:6]) + 1e-10)


@pytest.mark.parametrize("case_id", range(64))
def test_seed_free_witnesses_through_api(polynomial_kinematics_lib, crx_10ia, case_id):
    # The four production defects are ordinary passing requirements here.
    _check_seed_free_discovery(polynomial_kinematics_lib, crx_10ia, case_id)


@pytest.mark.parametrize("case", CASES, ids=lambda case: case["name"])
def test_fixture_candidates_against_baseline(
    polynomial_kinematics_lib, kinematics_lib, crx_10ia, case,
):
    target = xyzwpr([*case["target"]["xyz_mm"], *np.radians(case["target"]["wpr_deg"])])
    pose = target.ravel(order="F").tolist()
    # Compare complete finite catalogues, not prefixes after turn expansion.
    baseline = _call_ik(kinematics_lib, crx_10ia, pose, approx=None, max_solutions=4096)
    actual = _call_ik(polynomial_kinematics_lib, crx_10ia, pose, approx=None, max_solutions=4096)
    assert baseline[0] < 4096 and actual[0] < 4096
    check_results(crx_10ia, target, actual)
    assert not missing_postures(baseline[2], actual[2])


@pytest.mark.parametrize("asset", ASSETS, ids=lambda asset: asset["asset"])
def test_asset_random_api_witnesses(polynomial_kinematics_lib, asset):
    model = derive_model(asset)
    robot = RobotT()
    robot.data[1][1] = 6
    for key, row in (("base", 9), ("tool", 28)):
        np.testing.assert_allclose(model[key][:3, :3], np.eye(3), atol=1e-12, rtol=0)
        _set_row(robot, row, [*model[key][:3, 3], 0, 0, 0])
    _set_row(robot, 3, np.rint(np.diag(model["motion_map"])), col=4)
    for index, row in enumerate(model["dh"]):
        _set_row(robot, 10 + index, [*row, 0])
    lower, upper = np.array(asset["lower_limits_deg"]), np.array(asset["upper_limits_deg"])
    _set_row(robot, 30, lower)
    _set_row(robot, 31, upper)
    rng = np.random.default_rng(20261009)
    for _ in range(64):
        commands = rng.uniform(lower + 2, upper - 2)
        user = commands.copy()
        user[2] -= user[1]
        target = forward_model(model, commands)[1]
        actual = _call_ik(polynomial_kinematics_lib, robot, target.ravel(order="F").tolist(),
                          approx=None, max_solutions=64)
        assert actual[0] >= 0, (asset["asset"], commands.tolist())
        check_results(robot, target, actual)
        assert not missing_postures([user.tolist()], actual[2]), (asset["asset"], commands)


@pytest.mark.parametrize("case_id", [8, 36, 42, 57])
@pytest.mark.parametrize("capacity", [1, 3, 64])
def test_selection_capacity_and_packing(polynomial_kinematics_lib, crx_10ia, case_id, capacity):
    witness = discovery_joints(case_id)
    target, _ = reference_fk(crx_10ia, witness)
    seed = discovery_joints(case_id + 100)
    full = _call_ik(polynomial_kinematics_lib, crx_10ia, target.ravel(order="F").tolist(),
                    approx=seed, max_solutions=64)
    check_results(crx_10ia, target, full)
    assert not missing_postures([witness], full[2])
    command_seed = np.array(_to_fk_api_joints_deg(seed))
    distances = [np.sum((np.array(_to_fk_api_joints_deg(q)) - command_seed)**2) for q in full[2]]
    # Ranking uses radians in C++; decoding to degrees in NumPy
    # can reverse the last bit of mathematically tied scores.
    np.testing.assert_allclose(distances, sorted(distances), atol=1e-9, rtol=0)
    # Check the actual C ABI buffer boundaries and reserved fields as well as
    # the ranked prefix; helper decoding alone would hide a packing overrun.
    output = (ctypes.c_double * (12 * capacity + 12))(*([12345.] * (12 * capacity + 12)))
    best = (ctypes.c_double * 6)()
    count = polynomial_kinematics_lib.SolveIK(
        _to_c_array(target.ravel(order="F")), best, output, capacity,
        _to_c_array(_to_fk_api_joints_deg(seed)), ctypes.byref(crx_10ia),
    )
    assert count == min(capacity, full[0])
    for index in range(count):
        np.testing.assert_allclose(output[12*index:12*index+6],
                                   _to_fk_api_joints_deg(full[2][index]), atol=1e-10, rtol=0)
        assert list(output[12*index+6:12*index+12]) == [0.] * 6
    assert list(output[12*count:]) == [12345.] * (len(output) - 12*count)


def test_transformed_base_tool_and_restricted_limits(polynomial_kinematics_lib, crx_10ia):
    _set_row(crx_10ia, 9, [120, -35, 80, .2, -.3, .4])
    _set_row(crx_10ia, 28, [20, -10, 40, -.1, .2, -.3])
    witness = discovery_joints(8)
    command = np.array(_to_fk_api_joints_deg(witness))
    _set_row(crx_10ia, 30, command - .1)
    _set_row(crx_10ia, 31, command + .1)
    target, _ = reference_fk(crx_10ia, witness)
    actual = _call_ik(polynomial_kinematics_lib, crx_10ia, target.ravel(order="F").tolist(),
                      approx=None, max_solutions=64)
    check_results(crx_10ia, target, actual)
    assert not missing_postures([witness], actual[2])


def test_non_asset_senses_are_explicitly_unsupported(polynomial_kinematics_lib, crx_10ia):
    _set_row(crx_10ia, 3, [-1, 1, -1, -1, -1, -1], col=4)
    witness = discovery_joints(1)
    target, _ = reference_fk(crx_10ia, witness)
    pose = target.ravel(order="F").tolist()
    for seed in (None, witness):
        assert _call_ik(polynomial_kinematics_lib, crx_10ia, pose, approx=seed)[0] == -1
