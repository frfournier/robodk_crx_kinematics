import hashlib
import json
import os
from pathlib import Path
import sys

import numpy as np
import pytest
from crx_assets import APPROVED_ROBOT_ASSETS
from crx_asset_reference import derive_model, forward_model, forward_models
from robodk.robolink import (
    ITEM_TYPE_ROBOT,
    Robolink,
    TargetReachError,
    WINDOWSTATE_HIDDEN,
)
from robodk.robomath import Mat, invH, transl, rotx, roty

from crx_test_support import _to_fk_api_joints_deg
from crx_fixture_support import (
    CONFIG_FIXTURE_CASES,
    POSE_ANG_TOL_DEG,
    POSE_POS_TOL_MM,
    SOLUTION_PARAMS,
    _fmt_f,
    _fmt_joints,
    _fmt_pose16,
    _expected_robodk_config,
    _mat_to_pose16_col_major,
    _pose_err_pos_mm_ang_deg,
)

_ROBOT_ASSET = (
    Path(__file__).resolve().parents[1] / "assets" / "Fanuc-CRX-10iA-Custom.robot"
)
_ROBODK_ARGS = ["/NOSPLASH", "/NOSHOW", "/NEWINSTANCE"]
_MIN_ROBODK_VERSION = (6, 0, 6)


def _sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        while True:
            chunk = stream.read(1024 * 1024)
            if not chunk:
                break
            digest.update(chunk)
    return digest.hexdigest()


def _resolve_robodk_root() -> Path:
    env_root = os.environ.get("ROBODK_ROOT", "").strip()
    if env_root:
        return Path(env_root)

    if sys.platform == "win32":
        try:
            import winreg

            with winreg.OpenKey(winreg.HKEY_LOCAL_MACHINE, r"SOFTWARE\RoboDK") as key:
                instdir, _ = winreg.QueryValueEx(key, "INSTDIR")
            instdir_str = str(instdir).strip()
            if instdir_str:
                return Path(instdir_str)
        except OSError:
            pass
        return Path("C:/RoboDK")

    if sys.platform == "darwin":
        return Path.home() / "RoboDK" / "RoboDK.app" / "Contents" / "MacOS"

    return Path.home() / "RoboDK" / "bin"


def _robotextensions_dir(robodk_root: Path) -> Path:
    if sys.platform == "win32":
        return robodk_root / "bin" / "robotextensions"
    return robodk_root / "robotextensions"


def _require_robolink_extension() -> None:
    if sys.platform != "win32":
        return

    repo_root = Path(__file__).resolve().parents[1]
    build_dll = Path(os.environ.get(
        "CRXKIN_LIBRARY_PATH", repo_root / "build" / "Release" / "crx_kinematics.dll"
    ))
    if not build_dll.exists():
        pytest.fail(
            "\n".join(
                [
                    "RoboDK tests require a built DLL at:",
                    f"{build_dll}",
                    "Build first to avoid stale RoboDK extension tests.",
                ]
            )
        )

    robodk_root = _resolve_robodk_root()
    robotextensions_dir = _robotextensions_dir(robodk_root)
    robodk_dll = robotextensions_dir / "crx_kinematics.dll"

    is_mismatch = not robodk_dll.exists()
    if not is_mismatch:
        is_mismatch = _sha256_file(build_dll) != _sha256_file(robodk_dll)

    if not is_mismatch:
        return

    pytest.fail(
        "\n".join(
            [
                "RoboDK extension DLL is missing or differs from the build.",
                f"build_dll={build_dll}",
                f"robodk_dll={robodk_dll}",
                "Deploy the matching DLL explicitly before running live tests.",
            ]
        )
    )


@pytest.fixture(scope="session")
def robolink_crx_10ia(request):
    if not request.config.getoption("--run-robodk"):
        pytest.skip("Live RoboDK tests require --run-robodk and explicit deployment")
    _require_robolink_extension()

    if not _ROBOT_ASSET.exists():
        pytest.fail(f"RoboDK robot asset not found: {_ROBOT_ASSET}")

    try:
        executable = (_resolve_robodk_root() / "bin" / "RoboDK.exe"
                      if sys.platform == "win32" else None)
        rdk = Robolink(robodk_path=str(executable) if executable else None,
                       args=_ROBODK_ARGS, quit_on_close=True)
    except Exception as exc:
        pytest.fail(f"Unable to initialize RoboDK API session: {exc}")

    try:
        assert rdk.isNewInstance(), "Live tests require their own RoboDK instance"
        version_text = str(rdk.Version())
        version = tuple(int(part) for part in version_text.split(".")[:3])
        assert version >= _MIN_ROBODK_VERSION, (
            f"RoboDK {version_text} is unsupported; RoboDK 6.0.6 or newer is required"
        )
        rdk.setWindowState(WINDOWSTATE_HIDDEN)
        rdk.CloseStation()
        robot = rdk.AddFile(str(_ROBOT_ASSET))
        assert robot.Valid() and robot.Type() == ITEM_TYPE_ROBOT, (
            f"Failed to load RoboDK robot asset: {_ROBOT_ASSET}"
        )
        yield robot
    finally:
        rdk.Disconnect()


@pytest.mark.parametrize(
    "asset_name",
    list(APPROVED_ROBOT_ASSETS),
)
def test_robolink_crx_family_configuration(robolink_crx_10ia, asset_name: str):
    rdk = robolink_crx_10ia.RDK()
    asset_path = _ROBOT_ASSET.parent / asset_name
    robot = (
        robolink_crx_10ia
        if asset_path == _ROBOT_ASSET
        else rdk.AddFile(str(asset_path))
    )
    assert robot.Valid(), f"Failed to load RoboDK robot asset: {asset_path}"

    try:
        config = robot.JointsConfig([0.0] * 6)
        config_values = config.list() if hasattr(config, "list") else list(config)
        assert [int(value) for value in config_values[:3]] == [0, 0, 0]
    finally:
        if robot is not robolink_crx_10ia:
            robot.Delete()


@pytest.mark.parametrize("p", SOLUTION_PARAMS)
def test_robolink_forward_kinematics(robolink_crx_10ia, p):
    hdr = f"TC{p['case_id']} '{p['case_name']}' SOL{p['sol_id']}"
    fk_joints_deg = _to_fk_api_joints_deg(p["joints_deg"])

    joints_config = robolink_crx_10ia.JointsConfig(fk_joints_deg)

    joints_config_vals = (
        joints_config.list() if hasattr(joints_config, "list") else list(joints_config)
    )
    if p["case_name"] in CONFIG_FIXTURE_CASES:
        expected_config = _expected_robodk_config(p["config"])
        assert [int(value) for value in joints_config_vals[:3]] == expected_config, (
            f"{hdr}: RoboDK JointsConfig mismatch: "
            f"expected={expected_config}, actual={joints_config_vals}"
        )

    lower, upper, _ = robolink_crx_10ia.JointLimits()
    if any(q < lo or q > hi for q, lo, hi in
           zip(fk_joints_deg, lower.list(), upper.list())):
        # Four Table 5 fixture commands exceed this asset's J3 upper limit.
        # Rejection is expected behavior, not a reason to skip a live failure.
        with pytest.raises(TargetReachError):
            robolink_crx_10ia.SolveFK(fk_joints_deg)
        return
    pose = robolink_crx_10ia.SolveFK(fk_joints_deg)

    fk_pose16 = _mat_to_pose16_col_major(pose)
    pos_err, ang_err = _pose_err_pos_mm_ang_deg(p["target_pose16"], fk_pose16)

    assert pos_err <= POSE_POS_TOL_MM and ang_err <= POSE_ANG_TOL_DEG, "\n".join(
        [
            f"{hdr}: RoboDK FK pose mismatch",
            f"  pos_err_mm: {_fmt_f(pos_err)} (tol={POSE_POS_TOL_MM})",
            f"  ang_err_deg: {_fmt_f(ang_err)} (tol={_fmt_f(POSE_ANG_TOL_DEG)})",
            f"  fixture_joints_deg: {_fmt_joints(p['joints_deg'])}",
            f"  robolink_fk_joints_deg: {_fmt_joints(fk_joints_deg)}",
            "  target_pose16:",
            _fmt_pose16(p["target_pose16"]),
            "  robolink_fk_pose16:",
            _fmt_pose16(fk_pose16),
        ]
    )


@pytest.mark.skipif(sys.platform != "win32", reason="Windows DLL deployment guard")
@pytest.mark.parametrize("installed", [None, b"stale", b"current"])
def test_live_test_setup_never_deploys(tmp_path, monkeypatch, installed):
    repo = tmp_path / "repo"
    build_dll = repo / "build" / "Release" / "crx_kinematics.dll"
    build_dll.parent.mkdir(parents=True)
    build_dll.write_bytes(b"current")
    robodk_root = tmp_path / "RoboDK"
    deployed_dll = robodk_root / "bin" / "robotextensions" / build_dll.name
    if installed is not None:
        deployed_dll.parent.mkdir(parents=True)
        deployed_dll.write_bytes(installed)

    monkeypatch.setattr(__name__ + ".__file__", str(repo / "tests" / "live.py"))
    monkeypatch.setattr(__name__ + "._resolve_robodk_root", lambda: robodk_root)
    monkeypatch.delenv("CRXKIN_LIBRARY_PATH", raising=False)

    if installed == b"current":
        _require_robolink_extension()
    else:
        with pytest.raises(pytest.fail.Exception, match="Deploy the matching DLL"):
            _require_robolink_extension()

    if installed is None:
        assert not robodk_root.exists()
    else:
        assert deployed_dll.read_bytes() == installed


_ASSET_CAPTURE = json.loads((
    Path(__file__).parent / "fixtures" / "crx_asset_frames.json"
).read_text())["assets"]


def _coupling_commands(asset):
    # These are raw RoboDK commands. Deliberately do not use the fixture's
    # decoupled-to-command helper: that would hide a second J2/J3 conversion.
    commands = [[0.0] * 6]
    for j2, j3 in ((60, 0), (-60, 0), (0, 60), (0, -60),
                   (60, 60), (-60, -60), (60, -60), (-60, 60)):
        commands.append([23, j2, j3, -37, 51, 83])
    commands.extend(sample["commands_deg"] for sample in asset["samples"])
    lower, upper = asset["lower_limits_deg"], asset["upper_limits_deg"]
    # J2 and J3 independently reach their limits; neither limit is a limit
    # on J2 + J3. Include all four corners to expose that ambiguity.
    for j2 in (lower[1], upper[1]):
        for j3 in (lower[2], upper[2]):
            commands.append([23, j2, j3, -37, 51, 83])
    for j3 in (-360, -180, 180, 360):
        if lower[2] <= j3 <= upper[2]:
            commands.append([23, 60, j3, -37, 51, 83])
    return commands


def _assert_live_pose(actual, expected):
    actual, expected = np.asarray(actual), np.asarray(expected)
    np.testing.assert_allclose(actual[..., :3, 3], expected[..., :3, 3],
                               atol=2e-5, rtol=0)
    np.testing.assert_allclose(actual[..., :3, :3], expected[..., :3, :3],
                               atol=2e-7, rtol=0)


@pytest.mark.parametrize("asset", _ASSET_CAPTURE, ids=lambda asset: asset["asset"])
def test_robolink_command_fk_and_cad(robolink_crx_10ia, asset):
    path = _ROBOT_ASSET.parent / asset["asset"]
    assert _sha256_file(path) == APPROVED_ROBOT_ASSETS[asset["asset"]]
    robot = robolink_crx_10ia.RDK().AddFile(str(path))
    assert robot.Valid()
    try:
        robot.setAccuracyActive(0)
        lower, upper, _ = robot.JointLimits()
        lower, upper = np.array(lower.list()), np.array(upper.list())
        np.testing.assert_array_equal(lower, asset["lower_limits_deg"])
        np.testing.assert_array_equal(upper, asset["upper_limits_deg"])
        model = derive_model(asset)
        for commands in _coupling_commands(asset):
            frames, flange = forward_model(model, commands)
            _assert_live_pose(robot.SolveFK(commands).rows, flange)
            _assert_live_pose([p.rows for p in robot.JointPoses(commands)], frames)
            robot.setJoints(commands)
            np.testing.assert_allclose(robot.Joints().list(), commands, atol=1e-9, rtol=0)
    finally:
        robot.Delete()


@pytest.mark.parametrize("asset", _ASSET_CAPTURE, ids=lambda asset: asset["asset"])
def test_robolink_command_coupling(robolink_crx_10ia, asset):
    robot = robolink_crx_10ia.RDK().AddFile(str(_ROBOT_ASSET.parent / asset["asset"]))
    assert robot.Valid()
    try:
        robot.setAccuracyActive(0)
        lower, upper, _ = robot.JointLimits()
        lower, upper = np.array(lower.list()), np.array(upper.list())
        model = derive_model(asset)
        for commands in _coupling_commands(asset):
            _, flange = forward_model(model, commands)

            # Target is from recorded geometry and a separate Python DH product,
            # not this live DLL's FK. Validate every returned IK branch likewise.
            target = Mat(flange.tolist())
            exact = robot.SolveIK(target, commands).list()[:6]
            np.testing.assert_allclose(exact, commands, atol=1e-5, rtol=0)
            nearby = np.clip(np.array(commands) + [1, -2, 3, -1, 2, -3], lower, upper)
            robot.setJoints(nearby.tolist())
            solutions = [robot.SolveIK(target, nearby.tolist()).list()[:6],
                         robot.SolveIK(target).list()[:6]]
            all_solutions = robot.SolveIK_All(target)
            assert all_solutions.size(1) > 0, f"No IK branches for {commands}"
            solutions.extend(np.asarray(all_solutions.rows)[:6, :].T.tolist())
            joints = np.asarray(solutions)
            assert joints.shape[1] == 6 and np.isfinite(joints).all()
            assert np.all(joints >= lower - 1e-6)
            assert np.all(joints <= upper + 1e-6)
            _assert_live_pose(forward_models(model, joints),
                              np.broadcast_to(flange, (len(joints), 4, 4)))
    finally:
        robot.Delete()


@pytest.mark.parametrize("asset", _ASSET_CAPTURE, ids=lambda asset: asset["asset"])
def test_robolink_home_deduplication(robolink_crx_10ia, asset):
    robot = robolink_crx_10ia.RDK().AddFile(str(_ROBOT_ASSET.parent / asset["asset"]))
    assert robot.Valid()
    try:
        robot.setAccuracyActive(0)
        robot.setJoints([0]*6)
        target = robot.SolveFK([0]*6)
        commands = np.asarray(robot.SolveIK_All(target).rows)[:6, :].T
        assert np.count_nonzero(np.max(np.abs(commands), axis=1) < 1e-3) == 1
        difference = np.max(np.abs(commands[:, None, :] - commands[None, :, :]), axis=2)
        assert np.all(difference[np.triu_indices(len(commands), 1)] > 1e-4)
        np.testing.assert_array_equal(robot.SolveIK(target, [0]*6).list()[:6], np.zeros(6))
        model = derive_model(asset)
        _assert_live_pose(forward_models(model, commands),
                          np.broadcast_to(np.asarray(target.rows), (len(commands), 4, 4)))
    finally:
        robot.Delete()


@pytest.mark.parametrize("asset", _ASSET_CAPTURE, ids=lambda asset: asset["asset"])
def test_robolink_coupling_with_tool_reference_and_limits(robolink_crx_10ia, asset):
    robot = robolink_crx_10ia.RDK().AddFile(str(_ROBOT_ASSET.parent / asset["asset"]))
    assert robot.Valid()
    try:
        robot.setAccuracyActive(0)
        commands = [23, 60, -70, -37, 51, 83]
        model = derive_model(asset)
        _, flange = forward_model(model, commands)
        tool = transl(35, -24, 92) * rotx(0.3)
        reference = transl(-120, 80, 45) * roty(-0.4)
        target = invH(reference) * Mat(flange.tolist()) * tool
        _assert_live_pose(robot.SolveFK(commands, tool, reference).rows, target.rows)
        actual = robot.SolveIK(target, commands, tool, reference).list()[:6]
        np.testing.assert_allclose(actual, commands, atol=1e-5, rtol=0)
        # Updating a loaded asset must reach the callback in command space.
        # J3=-70 is valid here even though J2+J3=-10 is outside these bounds.
        lower, upper = np.array(commands) - 0.25, np.array(commands) + 0.25
        robot.setJointLimits(lower.tolist(), upper.tolist())
        actual = robot.SolveIK(target, commands, tool, reference).list()[:6]
        np.testing.assert_allclose(actual, commands, atol=1e-5, rtol=0)
        solutions = robot.SolveIK_All(target, tool, reference)
        assert solutions.size(1) > 0
        joints = np.asarray(solutions.rows)[:6, :].T
        assert np.all(joints >= lower - 1e-6)
        assert np.all(joints <= upper + 1e-6)
        _assert_live_pose(forward_models(model, joints),
                          np.broadcast_to(flange, (len(joints), 4, 4)))
    finally:
        robot.Delete()
