"""Complete-DLL regression coverage for the versioned performance corpus.

Known misses are exact-ID, strict xfails in routine runs. The release-readiness
command --require-solver-reliability removes those exceptions and must pass
before treating this corpus as solved. Invalid outputs are never xfailed.
"""
import ctypes
import hashlib
import json
from pathlib import Path

import numpy as np
import pytest

from crx_test_support import RobotT
from crx_asset_reference import forward_models
from fixtures.generate_solver_cases import (
    corpus, DEFAULT_RANDOM_PER_MODEL, DEFAULT_STRESS_PER_GROUP_MODEL, DEFAULT_SEED,
)

FIXTURES = Path(__file__).parent / "fixtures"
CASES = json.loads((FIXTURES / "crx_solver_cases.json").read_text())
KNOWN = json.loads((FIXTURES / "crx_solver_known_failures.json").read_text())
POSITION_MM, ANGLE_DEG = 1e-4, 1e-3  # Unchanged public strict-pose contract.
CAPACITY, STRIDE = 64, 12


class MissingFixtureSolve(AssertionError):
    """Recorded no-solution/handoff; never a malformed or inaccurate output."""


def pytest_generate_tests(metafunc):
    if "fixture_case" not in metafunc.fixturenames:
        return
    params = []
    require_all = metafunc.config.getoption("--require-solver-reliability")
    for backend in ("legacy", "polynomial"):
        for case in CASES:
            marks = []
            if case["name"] in KNOWN[backend] and not require_all:
                marks.append(pytest.mark.xfail(
                    strict=True, raises=MissingFixtureSolve,
                    reason=f"#19 B: recorded {backend} {case['group']} discovery miss; strict reliability gate still fails",
                ))
            params.append(pytest.param(backend, case, marks=marks,
                                      id=f"{backend}:{case['name']}"))
    metafunc.parametrize("backend,fixture_case", params)


@pytest.fixture(scope="module")
def generated_models():
    models, references, generated = corpus(
        DEFAULT_RANDOM_PER_MODEL, DEFAULT_STRESS_PER_GROUP_MODEL, DEFAULT_SEED)
    rendered = json.dumps(generated, indent=2) + "\n"
    assert rendered == (FIXTURES / "crx_solver_cases.json").read_text()
    assert hashlib.sha256(rendered.encode()).hexdigest() == KNOWN["metadata"]["corpus_sha256_lf"]
    names = {case["name"] for case in generated}
    assert len(names) == len(generated)
    for backend in ("legacy", "polynomial"):
        assert set(KNOWN[backend]) <= names, "failure ledger contains a stale case ID"
    robots = []
    for model in models:
        robot = RobotT()
        storage = np.ascontiguousarray(model["robot"])
        assert ctypes.sizeof(robot) == storage.nbytes
        ctypes.memmove(ctypes.byref(robot), storage.ctypes.data, storage.nbytes)
        robots.append(robot)
    return models, references, robots


def check_generated_solver_pose(request, generated_models, backend, fixture_case, seeded):
    models, references, robots = generated_models
    case = fixture_case
    model_id = case["model"]
    library = request.getfixturevalue("kinematics_lib" if backend == "legacy"
                                      else "polynomial_kinematics_lib")
    target = np.asarray(case["target"])
    pose = (ctypes.c_double * 16)(*target.ravel(order="F"))
    best = (ctypes.c_double * 6)()
    output = (ctypes.c_double * (CAPACITY * STRIDE))()
    seed = (ctypes.c_double * 6)(*case["seed_deg"]) if seeded else None
    count = library.SolveIK(pose, best, output, CAPACITY, seed,
                            ctypes.byref(robots[model_id]))
    assert count <= CAPACITY
    if not case["reachable"]:
        assert count <= 0, "accepted a target outside the arm's total reach"
        return
    if count <= 0:
        expected = KNOWN[backend].get(case["name"])
        if expected is not None:
            assert count == expected, "recorded failure changed ABI outcome"
        raise MissingFixtureSolve(f"{case['name']}: reachable pose returned {count}")
    # Validate best and every candidate, regardless of whether a miss on this
    # case was recorded. Invalid returns raise ordinary assertions, never xfail.
    commands = np.vstack([np.asarray(best), np.asarray(output).reshape(CAPACITY, STRIDE)[:count, :6]])
    assert np.isfinite(commands).all()
    np.testing.assert_allclose(commands[0], commands[1], atol=np.finfo(float).eps, rtol=0)
    actual = forward_models(references[model_id], commands)
    position = np.linalg.norm(actual[:, :3, 3] - target[:3, 3], axis=1)
    angle = np.degrees(2*np.arcsin(np.minimum(
        1, np.linalg.norm(actual[:, :3, :3] - target[:3, :3], axis=(1, 2))/(2*np.sqrt(2)))))
    assert np.all(position <= POSITION_MM), position
    assert np.all(angle <= ANGLE_DEG), angle
    low, high = models[model_id]["robot"][[30, 31], :6]
    allowance = 128*np.finfo(float).eps*np.maximum(1, np.maximum(abs(low), abs(high)))
    assert np.all(commands >= low - allowance) and np.all(commands <= high + allowance)
    return commands


@pytest.mark.parametrize("seeded", [False, True], ids=["no-seed", "nearby-seed"])
def test_generated_solver_pose(request, generated_models, backend, fixture_case, seeded):
    check_generated_solver_pose(request, generated_models, backend, fixture_case, seeded)


@pytest.mark.parametrize("seeded", [False, True], ids=["no-seed", "nearby-seed"])
def test_singular_rotation_rounding_preserves_candidates(request, generated_models, seeded):
    # Replacing Eigen's (1-cosine)+cosine axis diagonal with literal 1 changes
    # polishing at this singular target, dropping ten established commands.
    case = next(case for case in CASES if case["name"] == "MIN_Z")
    commands = check_generated_solver_pose(
        request, generated_models, "polynomial", case, seeded)
    assert commands is not None
    assert len(commands) == 26  # Best output followed by all 25 commands.
