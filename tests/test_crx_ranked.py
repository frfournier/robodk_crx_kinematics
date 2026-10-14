"""Ranked prefixes and restricted-domain discovery against the full-search DLL."""
import numpy as np
import pytest

from crx_asset_reference import forward_models
from crx_test_support import RobotT, _call_command_ik
from fixtures.generate_solver_cases import corpus


MODELS, REFERENCES, CASES = corpus(16, 14, 20261016)
REACHABLE = [dict(case, domain_index=index) for index, case in enumerate(CASES)
             if case["witnesses"] and case["reachable"]]


@pytest.mark.parametrize("case", REACHABLE, ids=lambda case: case["name"])
@pytest.mark.parametrize("capacity,seeded", [(1, False), (4, True), (64, False)])
def test_ranked_restricted_against_full_search(
    kinematics_lib, baseline_kinematics_lib, case, capacity, seeded,
):
    model_id = case["model"]
    table = MODELS[model_id]["robot"].copy()
    witness = np.degrees(case["witnesses"][0])
    # Cycle through J3-only, J6-only and their intersection, with exact
    # boundaries, narrow boxes and intervals crossing the phase cut.
    index = case["domain_index"]
    axes = ((2,), (5,), (2, 5))[index % 3]
    width = (.001, 2., 45.)[(index // 3) % 3]
    turns = (-1, 0, 1)[(index // 9) % 3]
    for axis in axes:
        table[30, axis] = witness[axis] + 360 * turns - width
        table[31, axis] = witness[axis] + 360 * turns + (0 if index % 2 else width)
    seed = np.array(case["seed_deg"]) if seeded else None
    if seed is not None:
        seed[list(axes)] += 360 * turns
    robot = RobotT.from_buffer_copy(np.ascontiguousarray(table))
    target = np.asarray(case["target"])
    args = (robot, target.ravel(order="F"), seed, capacity)
    expected = _call_command_ik(baseline_kinematics_lib, *args)
    actual = _call_command_ik(kinematics_lib, *args)
    assert actual[0] == expected[0], case["name"]
    if actual[0] <= 0:
        return
    np.testing.assert_allclose(actual[2], expected[2], atol=1e-8, rtol=0)
    commands = np.asarray(actual[2])
    poses = forward_models(REFERENCES[model_id], commands)
    position = np.linalg.norm(poses[:, :3, 3] - target[:3, 3], axis=1)
    angle = 2*np.arcsin(np.minimum(1, np.linalg.norm(
        poses[:, :3, :3] - target[:3, :3], axis=(1, 2))/(2*np.sqrt(2))))
    assert np.all(position <= 1e-4)
    assert np.all(angle <= np.radians(.001))
    allowance = 128*np.finfo(float).eps*np.maximum(1, np.maximum(abs(table[30, :6]), abs(table[31, :6])))
    assert np.all(commands >= table[30, :6] - allowance)
    assert np.all(commands <= table[31, :6] + allowance)
