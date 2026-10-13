"""Frozen independent reachable stress poses; failures stay explicit in totals."""
import ctypes
import hashlib
import json
from pathlib import Path

import numpy as np
import pytest

from crx_asset_reference import forward_models
from crx_test_support import RobotT
from fixtures.generate_accuracy_stress import corpus
from test_crx_polynomial_discovery import MERGED_ELBOW_PAIRS

FIXTURE = Path(__file__).parent / "fixtures/crx_accuracy_stress.json"
# This nomination remains unresolved in every attempted chart. It is counted
# as a strict failure in benchmarks, never accepted with a weaker FK gate.
UNRESOLVED = {"Fanuc-CRX-10iA-L-Custom.robot@0.001:base_axis:12"}


@pytest.fixture(scope="module")
def stress_inputs():
    models, references, generated = corpus()
    cases = json.loads(FIXTURE.read_text())
    assert hashlib.sha256(FIXTURE.read_text().encode()).hexdigest() == "931e0f2a8e0d9547ec47aed6abd571988dbd5b540b3f6f4fbf7de9ddf47b8a5f"
    assert cases == generated
    assert len(cases) == 3024
    return models, references, cases


@pytest.mark.parametrize("model_id", range(18))
@pytest.mark.parametrize("seeded", [False, True])
def test_independent_stress_commands(polynomial_kinematics_lib, stress_inputs, model_id, seeded):
    models, references, cases = stress_inputs
    model = models[model_id]
    robot = np.ascontiguousarray(model["robot"])
    robot_pointer = ctypes.cast(robot.ctypes.data, ctypes.POINTER(RobotT))
    capacity = 512  # Check all command lifts, including the 64-output tail.
    commands, targets, failures = [], [], set()
    for case in cases:
        if case["model"] != model_id:
            continue
        target = np.asarray(case["target"])
        pose = (ctypes.c_double * 16)(*target.ravel(order="F"))
        best = (ctypes.c_double * 6)()
        output = (ctypes.c_double * (12 * capacity))()
        seed = (ctypes.c_double * 6)(*case["seed_deg"]) if seeded else None
        count = polynomial_kinematics_lib.SolveIK(
            pose, best, output, capacity, seed, robot_pointer)
        if count <= 0:
            assert count == -1
            failures.add(case["name"])
            continue
        assert count < capacity, "increase test capacity to validate all commands"
        q = np.vstack([np.asarray(best), np.asarray(output).reshape(capacity, 12)[:count, :6]])
        commands.extend(q)
        targets.extend([target] * len(q))
    expected = {case["name"] for case in cases
                if case["model"] == model_id and case["name"] in UNRESOLVED}
    assert failures == expected
    q, targets = np.asarray(commands), np.asarray(targets)
    assert np.isfinite(q).all()
    low, high = robot[[30, 31], :6]
    allowance = 128 * np.finfo(float).eps * np.maximum(1, np.maximum(abs(low), abs(high)))
    assert ((q >= low - allowance) & (q <= high + allowance)).all()
    actual = forward_models(references[model_id], q)
    position = np.linalg.norm(actual[:, :3, 3] - targets[:, :3, 3], axis=1)
    chord = np.linalg.norm(actual[:, :3, :3] - targets[:, :3, :3], axis=(1, 2))
    angle = np.degrees(2*np.arcsin(np.minimum(1, chord / (2*np.sqrt(2)))))
    assert np.max(position) <= 1e-4
    assert np.max(angle) <= 1e-3


@pytest.mark.parametrize("pair", MERGED_ELBOW_PAIRS)
@pytest.mark.parametrize("scale", [.001, 1, 1000])
def test_complete_dll_retains_merged_elbows(polynomial_kinematics_lib, stress_inputs, pair, scale):
    models, references, _ = stress_inputs
    model_id = next(i for i, model in enumerate(models)
                    if model["name"] == f"Fanuc-CRX-10iA-Custom.robot@{scale:g}")
    target = forward_models(references[model_id], np.degrees([pair[0]]))[0]
    robot = np.ascontiguousarray(models[model_id]["robot"])
    pose = (ctypes.c_double * 16)(*target.ravel(order="F"))
    best = (ctypes.c_double * 6)()
    output = (ctypes.c_double * (12 * 64))()
    count = polynomial_kinematics_lib.SolveIK(
        pose, best, output, 64, None, ctypes.cast(robot.ctypes.data, ctypes.POINTER(RobotT)))
    assert 0 < count < 64
    joints = np.radians(np.asarray(output).reshape(64, 12)[:count, :6])
    witnesses = [np.asarray(q) for q in pair]
    for q in witnesses[:]:
        flipped = q.copy()
        flipped[:4] = [q[0] - np.pi, -q[1], np.pi - q[2], q[3] - np.pi]
        witnesses.append(flipped)
    for witness in witnesses:
        difference = (joints - witness + np.pi) % (2*np.pi) - np.pi
        assert np.min(np.max(abs(difference), axis=1)) < 1e-6
