"""Offline asset-derived evidence for the nominal coordinate bridge."""

import hashlib
import ctypes
import json
from pathlib import Path

import numpy as np
import pytest

from crx_asset_reference import (
    canonical_forward, derive_model, dh_transform, forward_model,
)
from crx_assets import APPROVED_ROBOT_ASSETS
from crx_reference import rotation, translation
from conftest import RobotT, _set_row

ROOT = Path(__file__).resolve().parents[1]
CAPTURE = json.loads((ROOT / "tests/fixtures/crx_asset_frames.json").read_text())
ASSETS = CAPTURE["assets"]


def test_asset_capture_provenance():
    assert CAPTURE["schema"] == 1
    assert CAPTURE["mode"] == "nominal"
    assert CAPTURE["length_unit"] == "mm"
    assert CAPTURE["command_unit"] == "degree"
    assert CAPTURE["custom_extension_bypassed"] is False
    assert CAPTURE["robodk_version"]
    assert {asset["asset"] for asset in ASSETS} == set(APPROVED_ROBOT_ASSETS)
    assert len(ASSETS) == len(APPROVED_ROBOT_ASSETS)
    for asset in ASSETS:
        digest = hashlib.sha256((ROOT / "assets" / asset["asset"]).read_bytes()).hexdigest()
        assert digest == asset["sha256"] == APPROVED_ROBOT_ASSETS[asset["asset"]]
        assert len(asset["lower_limits_deg"]) == len(asset["upper_limits_deg"]) == 6
        assert np.all(np.array(asset["lower_limits_deg"]) < asset["upper_limits_deg"])
        assert np.isfinite(asset["joint_type"])


def all_observations(asset):
    return [asset["zero"], *asset["probes"], *asset["samples"]]


@pytest.mark.parametrize("asset", ASSETS, ids=lambda asset: asset["asset"])
def test_asset_dh_identity(asset):
    model = derive_model(asset)
    # Measured command motion is diagonal for these six assets. The C++ core
    # uses decoupled J3, while the external RoboDK commands used here do not.
    np.testing.assert_allclose(model["motion_map"], np.diag([1, 1, -1, -1, -1, -1]),
                               atol=1e-12, rtol=0)
    for observation in all_observations(asset):
        frames, flange = forward_model(model, observation["commands_deg"])
        np.testing.assert_allclose(frames, observation["joint_frames"], atol=2e-9, rtol=0)
        np.testing.assert_allclose(flange, observation["flange_pose"], atol=2e-9, rtol=0)

    # Compare the home transform and the signed serial screw axes/line moments.
    # This checks the nominal chain identity beyond a collection of FK samples.
    rows = model["dh"]
    a, b, c, r = rows[2, 1], rows[3, 3], rows[5, 3], -rows[4, 3]
    base = model["base"] @ translation(0, 0, rows[0, 3])
    origins = [np.zeros(3), np.zeros(3), np.array([0, 0, a]),
               np.array([0, 0, a]), np.array([b, -r, a]), np.array([b+c, -r, a])]
    axes = [[0, 0, 1], [0, 1, 0], [0, -1, 0],
            [-1, 0, 0], [0, -1, 0], [-1, 0, 0]]
    pose = model["base"]
    senses = np.diag(model["motion_map"])
    for index, row in enumerate(rows):
        before_motion = pose @ rotation(0, row[0]) @ translation(row[1], 0, 0)
        omega = senses[index] * before_motion[:3, 2]
        point = before_motion[:3, 3]
        expected_omega = base[:3, :3] @ axes[index]
        expected_point = (base @ np.r_[origins[index], 1])[:3]
        np.testing.assert_allclose(omega, expected_omega, atol=1e-12, rtol=0)
        np.testing.assert_allclose(np.cross(point, omega),
                                   np.cross(expected_point, expected_omega), atol=2e-9, rtol=0)
        pose = pose @ dh_transform(row)
    np.testing.assert_allclose(base @ canonical_forward([a, b, c, r], np.zeros(6)) @ model["tool"],
                               asset["zero"]["flange_pose"], atol=2e-9, rtol=0)
    for observation in all_observations(asset):
        q = np.radians(observation["commands_deg"])
        expected = base @ canonical_forward([a, b, c, r], q) @ model["tool"]
        np.testing.assert_allclose(expected, observation["flange_pose"], atol=2e-9, rtol=0)


@pytest.mark.parametrize("asset", ASSETS, ids=lambda asset: asset["asset"])
def test_existing_callback_matches_asset_frames(kinematics_lib, asset):
    model = derive_model(asset)
    senses = np.rint(np.diag(model["motion_map"]))
    robot = RobotT()
    robot.data[1][1] = 6
    # These six captured assets have translation-only base/tool transforms.
    # Assert that evidence rather than introduce a general pose adapter here.
    for key, row in (("base", 9), ("tool", 28)):
        np.testing.assert_allclose(model[key][:3, :3], np.eye(3), atol=1e-12, rtol=0)
        _set_row(robot, row, [*model[key][:3, 3], 0, 0, 0])
    _set_row(robot, 3, senses, col=4)
    for index, row in enumerate(model["dh"]):
        _set_row(robot, 10 + index, [*row, 0])
    # CAD FK intentionally bypasses limits: this test establishes coordinate
    # conventions, not the still-pending command/decoupled limit policy.
    for observation in all_observations(asset):
        commands = (ctypes.c_double * 6)(*observation["commands_deg"])
        pose = (ctypes.c_double * 16)()
        frames = (ctypes.c_double * (16 * 7))()
        assert kinematics_lib.SolveFK_CAD(commands, pose, frames, 7, ctypes.byref(robot)) == 1
        actual_pose = np.array(pose).reshape(4, 4, order="F")
        actual_frames = np.array(frames).reshape(7, 4, 4).transpose(0, 2, 1)
        np.testing.assert_allclose(actual_pose, observation["flange_pose"], atol=2e-9, rtol=0)
        np.testing.assert_allclose(actual_frames, observation["joint_frames"], atol=2e-9, rtol=0)


def test_asset_reference_detects_wrong_coupling_and_flange():
    asset = ASSETS[1]
    model = derive_model(asset)
    sample = asset["samples"][0]
    model["motion_map"][2, 1] = -1
    assert np.max(np.abs(forward_model(model, sample["commands_deg"])[1]
                         - sample["flange_pose"])) > 1
    model = derive_model(asset)
    model["tool"] = model["tool"] @ rotation(0, np.pi)
    assert np.max(np.abs(forward_model(model, sample["commands_deg"])[1]
                         - sample["flange_pose"])) > 0.1
