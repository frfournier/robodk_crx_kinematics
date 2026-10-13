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

from crx_test_support import RobotT, MissingFixtureSolve, check_solver_pose
from fixtures.generate_solver_cases import (
    corpus, DEFAULT_RANDOM_PER_MODEL, DEFAULT_STRESS_PER_GROUP_MODEL, DEFAULT_SEED,
)

FIXTURES = Path(__file__).parent / "fixtures"
CASES = json.loads((FIXTURES / "crx_solver_cases.json").read_text())
KNOWN = json.loads((FIXTURES / "crx_solver_known_failures.json").read_text())


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


@pytest.mark.parametrize("seeded", [False, True], ids=["no-seed", "nearby-seed"])
def test_generated_solver_pose(request, generated_models, backend, fixture_case, seeded):
    library = request.getfixturevalue("kinematics_lib" if backend == "legacy"
                                      else "polynomial_kinematics_lib")
    check_solver_pose(library, generated_models, fixture_case, seeded,
                      expected_failure=KNOWN[backend].get(fixture_case["name"]))


@pytest.mark.parametrize("seeded", [False, True], ids=["no-seed", "nearby-seed"])
def test_singular_rotation_rounding_preserves_candidates(
    polynomial_kinematics_lib, generated_models, seeded,
):
    # Replacing Eigen's (1-cosine)+cosine axis diagonal with literal 1 changes
    # polishing at this singular target, dropping ten established commands.
    case = next(case for case in CASES if case["name"] == "MIN_Z")
    commands = check_solver_pose(
        polynomial_kinematics_lib, generated_models, case, seeded)
    assert commands is not None
    assert len(commands) == 26  # Best output followed by all 25 commands.
