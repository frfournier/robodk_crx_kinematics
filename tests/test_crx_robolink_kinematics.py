import hashlib
import os
from pathlib import Path
import sys

import pytest
from crx_assets import APPROVED_ROBOT_ASSETS
from robodk.robolink import (
    ITEM_TYPE_ROBOT,
    Robolink,
    TargetReachError,
    WINDOWSTATE_HIDDEN,
)

from crx_test_support import _to_fk_api_joints_deg
from test_crx_kinematics import (
    CONFIG_FIXTURE_CASES,
    POSE_ANG_TOL_DEG,
    POSE_POS_TOL_MM,
    _PARAMS,
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


def _require_robolink_extension_or_skip() -> None:
    if sys.platform != "win32":
        return

    repo_root = Path(__file__).resolve().parents[1]
    build_dll = repo_root / "build" / "Release" / "crx_kinematics.dll"
    if not build_dll.exists():
        pytest.skip(
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
    robodk_dll = robotextensions_dir / build_dll.name

    is_mismatch = not robodk_dll.exists()
    if not is_mismatch:
        is_mismatch = _sha256_file(build_dll) != _sha256_file(robodk_dll)

    if not is_mismatch:
        return

    pytest.skip(
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
    _require_robolink_extension_or_skip()

    if not _ROBOT_ASSET.exists():
        pytest.skip(f"RoboDK robot asset not found: {_ROBOT_ASSET}")

    try:
        rdk = Robolink(args=_ROBODK_ARGS, quit_on_close=True)
    except Exception as exc:
        pytest.skip(f"Unable to initialize RoboDK API session: {exc}")

    version_text = str(rdk.Version())
    version = tuple(int(part) for part in version_text.split(".")[:3])
    assert version >= _MIN_ROBODK_VERSION, (
        f"RoboDK {version_text} is unsupported; RoboDK 6.0.6 or newer is required"
    )

    try:
        rdk.setWindowState(WINDOWSTATE_HIDDEN)
    except Exception:
        pass

    try:
        rdk.CloseStation()
    except Exception:
        pass

    robot = rdk.AddFile(str(_ROBOT_ASSET))
    if not robot.Valid() or robot.Type() != ITEM_TYPE_ROBOT:
        candidate = rdk.Item(_ROBOT_ASSET.stem, ITEM_TYPE_ROBOT)
        if candidate.Valid():
            robot = candidate

    if not robot.Valid():
        try:
            rdk.Disconnect()
        except Exception:
            pass
        pytest.skip(f"Failed to load RoboDK robot asset: {_ROBOT_ASSET}")

    try:
        yield robot
    finally:
        try:
            rdk.Disconnect()
        except Exception:
            pass


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


@pytest.mark.parametrize("p", _PARAMS)
def test_robolink_forward_kinematics(robolink_crx_10ia, p):
    hdr = f"TC{p['case_id']} '{p['case_name']}' SOL{p['sol_id']}"
    fk_joints_deg = _to_fk_api_joints_deg(p["joints_deg"])

    try:
        joints_config = robolink_crx_10ia.JointsConfig(fk_joints_deg)
        pose = robolink_crx_10ia.SolveFK(fk_joints_deg)
    except TargetReachError as exc:
        pytest.skip(
            "\n".join(
                [
                    f"{hdr}: RoboDK FK/config rejected coupled joints",
                    f"fixture_joints_deg={_fmt_joints(p['joints_deg'])}",
                    f"robolink_fk_joints_deg={_fmt_joints(fk_joints_deg)}",
                    f"error={exc}",
                ]
            )
        )

    joints_config_vals = (
        joints_config.list() if hasattr(joints_config, "list") else list(joints_config)
    )
    if p["case_name"] in CONFIG_FIXTURE_CASES:
        expected_config = _expected_robodk_config(p["config"])
        assert [int(value) for value in joints_config_vals[:3]] == expected_config, (
            f"{hdr}: RoboDK JointsConfig mismatch: "
            f"expected={expected_config}, actual={joints_config_vals}"
        )

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
