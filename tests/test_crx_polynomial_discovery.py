"""Target-only discovery: Python owns model mapping and independent FK oracles."""

import json
import os
from pathlib import Path
import subprocess

import numpy as np
import pytest

from crx_asset_reference import canonical_forward, derive_model, rigid_inverse
from crx_reference import reference_fk, translation, xyzwpr
from test_crx_discovery import discovery_joints

ROOT = Path(__file__).resolve().parents[1]
ASSETS = json.loads((ROOT / "tests/fixtures/crx_asset_frames.json").read_text())["assets"]
FIXTURES = json.loads((ROOT / "tests/fixtures/CRX10iA-solutions.json").read_text())["test_cases"]


@pytest.fixture(scope="module")
def discover():
    configured = os.environ.get("CRXKIN_ROOT_PROBE_PATH")
    path = Path(configured) if configured else ROOT / "build/Release/crx_canonical_tests.exe"
    if configured:
        assert path.is_file(), f"Configured native probe is missing: {path}"
    elif not path.is_file():
        pytest.skip("Build crx_canonical_tests or set CRXKIN_ROOT_PROBE_PATH")

    def run(lengths, target, position_tolerance=1e-6, orientation_tolerance=1e-9):
        # Only lengths, target and physical acceptance tolerances cross into
        # C++. No expected angle, generating posture or solver seed is sent.
        values = [*lengths, *np.asarray(target)[:3, :].flat,
                  position_tolerance, orientation_tolerance]
        completed = subprocess.run(
            [str(path), "--discover", *(repr(float(value)) for value in values)],
            check=True, capture_output=True, text=True, timeout=20,
        )
        return json.loads(completed.stdout)
    return run


def check_discovery(result, lengths, target, expected, joint_tolerance=1e-7):
    assert result["status"] == "Candidates", result
    solutions = np.array(result["solutions"])
    assert 2 <= len(solutions) <= 36
    joints = solutions[:, :6]
    assert np.isfinite(solutions).all()
    assert np.max(np.abs(joints)) <= np.pi
    for witness in expected:
        error = (joints - witness + np.pi) % (2 * np.pi) - np.pi
        assert np.min(np.max(np.abs(error), axis=1)) < joint_tolerance, witness
    for index, row in enumerate(solutions):
        assert row[6] <= 1e-6
        assert row[7] <= 1e-9
        pose = canonical_forward(lengths, row[:6])
        np.testing.assert_allclose(pose[:3, 3], target[:3, 3], atol=1e-6, rtol=0)
        np.testing.assert_allclose(pose[:3, :3], target[:3, :3], atol=1e-9, rtol=0)
        for previous in joints[:index]:
            delta = (row[:6] - previous + np.pi) % (2 * np.pi) - np.pi
            assert np.max(np.abs(delta)) > 1e-10


@pytest.mark.parametrize("asset", ASSETS, ids=lambda asset: asset["asset"])
def test_asset_target_only_discovery(discover, asset):
    model = derive_model(asset)
    dh = model["dh"]
    lengths = [dh[2, 1], dh[3, 3], dh[5, 3], -dh[4, 3]]
    base = model["base"] @ translation(0, 0, dh[0, 3])
    for sample in asset["samples"]:
        target = rigid_inverse(base) @ sample["flange_pose"] @ rigid_inverse(model["tool"])
        expected = np.radians(sample["commands_deg"])
        result = discover(lengths, target)
        check_discovery(result, lengths, target, [expected])


@pytest.mark.parametrize("case_id", range(64))
def test_legacy_discovery_witnesses_without_seed(discover, crx_10ia, case_id):
    # Includes 8, 36, 42, 57 as ordinary passing requirements here. Existing
    # production xfails stay until the new path is integrated into SolveIK.
    user = discovery_joints(case_id)
    target, _ = reference_fk(crx_10ia, user)
    q = np.radians(user)
    q[2] += q[1]
    lengths = [crx_10ia.data[12][1], crx_10ia.data[13][3],
               crx_10ia.data[15][3], -crx_10ia.data[14][3]]
    check_discovery(discover(lengths, target), lengths, target, [q])


def test_discovery_checker_detects_missing_and_duplicate_postures():
    lengths = [540, 540, 160, 150]
    q = np.array([0.4, -0.7, 0.8, 0.6, -0.9, -1.2])
    flipped = q.copy()
    flipped[:4] = [q[0] - np.pi, -q[1], np.pi - q[2], q[3] - np.pi]
    # Use a wrist-equivalent full turn in the oracle's expected vector.
    target = canonical_forward(lengths, q)
    valid = {"status": "Candidates", "solutions": [
        [*q, 0, 0], [*flipped, 0, 0],
    ]}
    check_discovery(valid, lengths, target, [q + 2 * np.pi, flipped])
    with pytest.raises(AssertionError):
        check_discovery({"status": "Candidates", "solutions": [[*q, 0, 0]]},
                        lengths, target, [q, flipped])
    with pytest.raises(AssertionError):
        check_discovery({"status": "Candidates", "solutions": [[*q, 0, 0]] * 2},
                        lengths, target, [q])


@pytest.mark.parametrize("case", FIXTURES, ids=lambda case: case["name"])
def test_recorded_postures_without_scanner(discover, case):
    # All fixture targets must now be handled by the polynomial component.
    lengths = [540, 540, 160, 150]
    target = xyzwpr([*case["target"]["xyz_mm"], *np.radians(case["target"]["wpr_deg"])])
    result = discover(lengths, target)
    expected = []
    for solution in case["solutions"]:
        q = np.radians(solution["joints_deg"])
        q[2] += q[1]
        expected.append(q)
    if case["name"] == "MIN_Z":
        # Rounded fixture XYZ and recorded joints disagree by 0.00425 mm.
        # At this workspace boundary, exact target IK has different base/wrist
        # postures. Validate the requested pose, not an inconsistent witness.
        error = np.linalg.norm(canonical_forward(lengths, expected[0])[:3, 3] - target[:3, 3])
        assert 0.004 < error < 0.005
        expected = []
    check_discovery(result, lengths, target, expected, joint_tolerance=np.radians(0.009))
    if case["name"] == "ABBES-TABLE6":
        assert len(result["solutions"]) == 16
    if case["name"] in {"ALL8", "ONLY7", "ABBES-TABLE4"}:
        # ONLY7 excludes a command-limited posture; this stage has no limits.
        assert len(result["solutions"]) == 8


def test_min_z_generating_posture_is_recovered(discover):
    lengths = [540, 540, 160, 150]
    q = np.radians([0, 179.99, 89.99, 0, 0, 0])
    target = canonical_forward(lengths, q)
    check_discovery(discover(lengths, target), lengths, target, [q], joint_tolerance=1e-5)


@pytest.mark.parametrize("asset_index,q", [
    (0, [-1.4727355738033883, 1.1126962093825525, 3.709078502262402,
         0.9883300375877239, 0.004221004556941241, -1.5688012073304018]),
    (5, [-2.6442824151374285, -1.8272487262313033, 1.5707963093416042,
         2.588068342746596, 0.41008305466623035, -2.3444911435310343]),
    (3, np.radians([-40.487164643723816, -120.00397722297943, -87.94631530285062,
                    183.4975662256848, 86.9534717145915, -47.19813442729409])),
    (1, np.radians([-138.23383120822047, -37.30338268062005, -90.0081134368952,
                    88.97839812467765, -116.35252116367036, 112.43211695606419])),
], ids=["near-chart-endpoint", "close-root-near-tangent", "small-elbow-height", "nearly-folded"])
def test_snapshot_conditioning_regressions(discover, asset_index, q):
    dh = derive_model(ASSETS[asset_index])["dh"]
    lengths = [dh[2, 1], dh[3, 3], dh[5, 3], -dh[4, 3]]
    q = np.array(q)
    target = canonical_forward(lengths, q)
    check_discovery(discover(lengths, target), lengths, target, [q], joint_tolerance=1e-6)


@pytest.mark.parametrize("asset", ASSETS, ids=lambda asset: asset["asset"])
def test_straight_arm_and_nearby_branches_without_scanner(discover, asset):
    dh = derive_model(asset)["dh"]
    lengths = [dh[2, 1], dh[3, 3], dh[5, 3], -dh[4, 3]]
    for offset in [-1e-4, -1e-8, 0, 1e-8, 1e-4]:
        for sample in asset["samples"][:2]:
            q = np.radians(sample["commands_deg"])
            q[2] = np.pi / 2 + offset
            target = canonical_forward(lengths, q)
            check_discovery(discover(lengths, target), lengths, target, [q], joint_tolerance=1e-6)


@pytest.mark.parametrize("q", [[0, 0, 0, 0, -90, 0], [0, 90, 180, 0, -90, 0]])
def test_multiple_root_perturbations_preserve_witness(discover, q):
    lengths = [540, 540, 160, 150]
    for offset in [-1e-4, -1e-8, 0, 1e-8, 1e-4]:
        witness = np.radians(q)
        witness[0] += .37
        witness[3] += offset
        witness[5] += .41
        target = canonical_forward(lengths, witness)
        check_discovery(discover(lengths, target), lengths, target, [witness], joint_tolerance=1e-6)


@pytest.mark.parametrize("q5", [0.0, 1e-14, -1e-14, np.pi, -np.pi])
def test_zero_wrist_sine_does_not_overwrite_j6(discover, q5):
    lengths = [2, 1.5, 0.25, 0.375]
    q = np.array([0.4, -0.7, 0.8, 0.6, q5, -1.2])
    target = canonical_forward(lengths, q)
    check_discovery(discover(lengths, target), lengths, target, [q])


def test_unresolved_and_empty_results_are_distinct(discover):
    continuous = discover([1, 1, 0, 0], np.eye(4))
    assert continuous == {"status": "NeedsRefinement", "solutions": []}
    empty = discover([1, 1, 0.2, 0.25], np.eye(4))
    assert empty == {"status": "NoCandidate", "solutions": []}
    lengths = [2, 1.5, 0.25, 0.375]
    q = np.array([0.4, -0.7, 0.8, 0.6, -0.9, -1.2])
    target = canonical_forward(lengths, q)
    for pos, ang in [(1e-30, 1e-9), (1e-6, 1e-30)]:
        rejected = discover(lengths, target, pos, ang)
        assert rejected == {"status": "NeedsRefinement", "solutions": []}


def test_exact_omitted_point_is_not_lost(discover):
    lengths = [1.25, 1, 2, 0.25]
    target = np.array([[0, 0, -1, -1.25], [-1, 0, 0, 0.75],
                       [0, 1, 0, 1], [0, 0, 0, 1]], dtype=float)
    angle = np.arctan2(1, 0.75)
    q = np.array([angle, np.pi/2, np.pi, angle, np.pi/2, -np.pi/2])
    result = discover(lengths, target)
    check_discovery(result, lengths, target, [q])
    assert len(result["solutions"]) == 8
