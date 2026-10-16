"""Complete-DLL regression coverage for the versioned performance corpus."""
import ctypes
import hashlib
import json
from pathlib import Path

import numpy as np
import pytest

from crx_test_support import RobotT, check_solver_pose
from crx_fixture_support import assert_regenerated_cases_match
from fixtures.generate_solver_cases import (
    corpus, DEFAULT_RANDOM_PER_MODEL, DEFAULT_STRESS_PER_GROUP_MODEL, DEFAULT_SEED,
)

FIXTURES = Path(__file__).parent / "fixtures"
CASES = json.loads((FIXTURES / "crx_solver_cases.json").read_text())


@pytest.fixture(scope="module")
def generated_models():
    models, references, generated = corpus(
        DEFAULT_RANDOM_PER_MODEL, DEFAULT_STRESS_PER_GROUP_MODEL, DEFAULT_SEED)
    frozen = (FIXTURES / "crx_solver_cases.json").read_text()
    assert hashlib.sha256(frozen.encode()).hexdigest() == "e582fae793d0c91717a557409c240787ac9a36e6d80c382d5f56b3483e9269e0"
    # Keep the recorded corpus exact. Regeneration can differ by roundoff
    # between Windows and Linux math libraries, without changing the cases.
    assert_regenerated_cases_match(generated, CASES)
    names = {case["name"] for case in generated}
    assert len(names) == len(generated)
    robots = []
    for model in models:
        robot = RobotT()
        storage = np.ascontiguousarray(model["robot"])
        assert ctypes.sizeof(robot) == storage.nbytes
        ctypes.memmove(ctypes.byref(robot), storage.ctypes.data, storage.nbytes)
        robots.append(robot)
    return models, references, robots


@pytest.mark.parametrize("seeded", [False, True], ids=["no-seed", "nearby-seed"])
@pytest.mark.parametrize("fixture_case", CASES, ids=lambda case: case["name"])
def test_generated_solver_pose(kinematics_lib, generated_models, fixture_case, seeded):
    check_solver_pose(kinematics_lib, generated_models, fixture_case, seeded)


@pytest.mark.parametrize("seeded", [False, True], ids=["no-seed", "nearby-seed"])
@pytest.mark.parametrize("turns", [-1, 0, 1])
@pytest.mark.parametrize("inset_deg", [0., .001])
def test_boundary_witness_survives_other_feasible_postures(
    kinematics_lib, generated_models, seeded, turns, inset_deg,
):
    case = next(case for case in CASES if case["name"] ==
                "Fanuc-CRX-10iA-Custom.robot:joint_limits:22")
    models, references, robots = generated_models
    model_id = case["model"]
    models, robots = list(models), list(robots)
    model = dict(models[model_id])
    model["robot"] = model["robot"].copy()
    model["robot"][30, 4] += 360 * turns + inset_deg
    model["robot"][31, 4] += 360 * turns
    models[model_id] = model
    robots[model_id] = RobotT.from_buffer_copy(np.ascontiguousarray(model["robot"]))
    case = dict(case, seed_deg=list(case["seed_deg"]))
    case["seed_deg"][4] += 360 * turns
    commands = check_solver_pose(
        kinematics_lib, (models, references, robots), case, seeded)
    witness = np.degrees(case["witnesses"][0])
    separation = np.max(np.abs((commands - witness + 180) % 360 - 180), axis=1)
    if inset_deg == 0:
        assert np.min(separation) < 1e-6
    else:
        assert np.min(separation) > 1e-4
    allowance = 128 * np.finfo(float).eps * max(1, abs(model["robot"][30, 4]))
    assert np.all(commands[:, 4] >= model["robot"][30, 4] - allowance)


@pytest.mark.parametrize("seeded", [False, True], ids=["no-seed", "nearby-seed"])
def test_singular_rotation_rounding_preserves_candidates(
    kinematics_lib, generated_models, seeded,
):
    # Replacing Eigen's (1-cosine)+cosine axis diagonal with literal 1 changes
    # polishing at this singular target, dropping ten established commands.
    case = next(case for case in CASES if case["name"] == "MIN_Z")
    commands = check_solver_pose(
        kinematics_lib, generated_models, case, seeded)
    assert commands is not None
    assert len(commands) == 26  # Best output followed by all 25 commands.
