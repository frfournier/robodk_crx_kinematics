"""Frozen independent reachable stress poses; failures stay explicit in totals."""
import ctypes
import hashlib
import json
from pathlib import Path

import numpy as np
import pytest

from crx_asset_reference import canonical_forward, forward_models, rigid_inverse
from crx_test_support import RobotT
from fixtures.generate_accuracy_stress import corpus
from crx_fixture_support import MERGED_ELBOW_PAIRS, assert_regenerated_cases_match

FIXTURE = Path(__file__).parent / "fixtures/crx_accuracy_stress.json"


@pytest.fixture(scope="module")
def stress_inputs():
    models, references, generated = corpus()
    cases = json.loads(FIXTURE.read_text())
    assert hashlib.sha256(FIXTURE.read_text().encode()).hexdigest() == "931e0f2a8e0d9547ec47aed6abd571988dbd5b540b3f6f4fbf7de9ddf47b8a5f"
    assert_regenerated_cases_match(generated, cases)
    assert len(cases) == 3024
    return models, references, cases


@pytest.mark.parametrize("model_id", range(18))
@pytest.mark.parametrize("seeded", [False, True])
def test_independent_stress_commands(kinematics_lib, stress_inputs, model_id, seeded):
    models, references, cases = stress_inputs
    model = models[model_id]
    robot = np.ascontiguousarray(model["robot"])
    robot_pointer = ctypes.cast(robot.ctypes.data, ctypes.POINTER(RobotT))
    capacity = 512  # Check all command lifts, including the 64-output tail.
    commands, targets = [], []
    for case in cases:
        if case["model"] != model_id:
            continue
        target = np.asarray(case["target"])
        pose = (ctypes.c_double * 16)(*target.ravel(order="F"))
        best = (ctypes.c_double * 6)()
        output = (ctypes.c_double * (12 * capacity))()
        seed = (ctypes.c_double * 6)(*case["seed_deg"]) if seeded else None
        count = kinematics_lib.SolveIK(
            pose, best, output, capacity, seed, robot_pointer)
        assert count > 0, f"{case['name']}: reachable pose returned {count}"
        assert count < capacity, "increase test capacity to validate all commands"
        q = np.vstack([np.asarray(best), np.asarray(output).reshape(capacity, 12)[:count, :6]])
        commands.extend(q)
        targets.extend([target] * len(q))
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


@pytest.mark.parametrize("scale", [.001, 1, 1000])
def test_near_axis_split_root_windows(canonical_probe, stress_inputs, scale):
    models, _, cases = stress_inputs
    case = next(case for case in cases if case["name"] ==
                "Fanuc-CRX-10iA-L-Custom.robot@0.001:base_axis:12")
    model = models[case["model"]]
    target = (rigid_inverse(model["base"]) @ np.asarray(case["target"])
              @ rigid_inverse(model["tool"]))
    lengths = np.asarray(model["lengths"]) * (scale / .001)
    target[:3, 3] *= scale / .001
    # Coefficient rounding splits the near-axis root into nearby real roots.
    # Discovery receives no witness/seed and must not confine each nomination
    # to a disjoint window smaller than its coefficient uncertainty.
    result = canonical_probe("--discover", [
        *lengths, *target[:3, :].flat, 1e-4, np.radians(1e-3),
    ])
    assert result["status"] == "Candidates", result
    assert result["solutions"]
    for row in result["solutions"]:
        actual = canonical_forward(lengths, row[:6])
        assert np.linalg.norm(actual[:3, 3] - target[:3, 3]) <= 1e-4
        chord = np.linalg.norm(actual[:3, :3] - target[:3, :3])
        assert np.degrees(2*np.arcsin(min(1, chord / (2*np.sqrt(2))))) <= 1e-3


@pytest.mark.parametrize("pair", MERGED_ELBOW_PAIRS)
@pytest.mark.parametrize("scale", [.001, 1, 1000])
def test_complete_dll_retains_merged_elbows(kinematics_lib, stress_inputs, pair, scale):
    models, references, _ = stress_inputs
    model_id = next(i for i, model in enumerate(models)
                    if model["name"] == f"Fanuc-CRX-10iA-Custom.robot@{scale:g}")
    target = forward_models(references[model_id], np.degrees([pair[0]]))[0]
    robot = np.ascontiguousarray(models[model_id]["robot"])
    pose = (ctypes.c_double * 16)(*target.ravel(order="F"))
    best = (ctypes.c_double * 6)()
    output = (ctypes.c_double * (12 * 64))()
    count = kinematics_lib.SolveIK(
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
