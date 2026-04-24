import hashlib
import os
from pathlib import Path
import shutil
import sys

import numpy as np
import pytest
from robodk.robolink import (
    ITEM_TYPE_ROBOT,
    Robolink,
    TargetReachError,
    WINDOWSTATE_HIDDEN,
)

from test_crx_kinematics import (
    JOINT_TOL_DEG,
    POSE_ANG_TOL_DEG,
    POSE_POS_TOL_MM,
    _PARAMS,
    _fmt_f,
    _fmt_joints,
    _fmt_pose16,
    _joints_close,
    _mat_to_pose16_col_major,
    _from_fk_api_joints_deg,
    _pose_err_pos_mm_ang_deg,
    _to_fk_api_joints_deg,
    _wrapped_diff_deg,
)

_ROBOT_ASSET = (
    Path(__file__).resolve().parents[1] / "assets" / "Fanuc-CRX-10iA-Custom.robot"
)
_ROBODK_ARGS = ["/NOSPLASH", "/NOSHOW", "/NEWINSTANCE"]
_ROBOLINK_FUZZ_SEEDS = 128
_ROBOLINK_FUZZ_LIMIT_MARGIN_DEG = 2.0
_ROBOLINK_FUZZ_POSE_POS_TOL_MM = POSE_POS_TOL_MM
_ROBOLINK_FUZZ_POSE_ANG_TOL_DEG = POSE_ANG_TOL_DEG


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


def _sync_robolink_extension_or_skip() -> None:
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

    try:
        robotextensions_dir.mkdir(parents=True, exist_ok=True)
        shutil.copy2(build_dll, robodk_dll)
    except Exception as exc:
        pytest.skip(
            "\n".join(
                [
                    "RoboDK extension DLL is stale and copy failed.",
                    f"build_dll={build_dll}",
                    f"robodk_dll={robodk_dll}",
                    f"error={exc}",
                    "Skipping RoboDK tests to avoid stale results.",
                ]
            )
        )

    if _sha256_file(build_dll) != _sha256_file(robodk_dll):
        pytest.skip(
            "\n".join(
                [
                    "RoboDK extension DLL mismatch remains after copy.",
                    f"build_dll={build_dll}",
                    f"robodk_dll={robodk_dll}",
                    "Skipping RoboDK tests to avoid stale results.",
                ]
            )
        )


def _sample_joints_in_limits(
    rng: np.random.Generator, lower_limits: list[float], upper_limits: list[float]
) -> list[float]:
    sampled: list[float] = []
    for lower, upper in zip(lower_limits, upper_limits):
        lo = float(lower)
        hi = float(upper)
        if hi <= lo:
            sampled.append(lo)
            continue
        margin = min(_ROBOLINK_FUZZ_LIMIT_MARGIN_DEG, 0.45 * (hi - lo))
        sampled.append(float(rng.uniform(lo + margin, hi - margin)))
    return sampled


def _fit_angle_to_limits(angle_deg: float, lower_deg: float, upper_deg: float) -> float | None:
    candidates = [
        angle_deg + 360.0 * turns
        for turns in range(-2, 3)
        if (lower_deg - 1e-9) <= (angle_deg + 360.0 * turns) <= (upper_deg + 1e-9)
    ]
    if not candidates:
        return None
    return min(candidates, key=lambda c: abs(c - angle_deg))


def _wrap_joints_to_limits(
    joints_deg: list[float], lower_limits: list[float], upper_limits: list[float]
) -> list[float] | None:
    wrapped: list[float] = []
    for joint, lower, upper in zip(joints_deg, lower_limits, upper_limits):
        fitted = _fit_angle_to_limits(float(joint), float(lower), float(upper))
        if fitted is None:
            return None
        wrapped.append(float(fitted))
    return wrapped

@pytest.fixture(scope="session")
def robolink_crx_10ia():
    _sync_robolink_extension_or_skip()

    if not _ROBOT_ASSET.exists():
        pytest.skip(f"RoboDK robot asset not found: {_ROBOT_ASSET}")

    try:
        rdk = Robolink(args=_ROBODK_ARGS, quit_on_close=True)
    except Exception as exc:
        pytest.skip(f"Unable to initialize RoboDK API session: {exc}")

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
    print(f"{hdr}: robolink_conf_rlf={joints_config_vals}")

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


@pytest.mark.parametrize("p", _PARAMS)
def test_robolink_inverse_kinematics(robolink_crx_10ia, p):
    hdr = f"TC{p['case_id']} '{p['case_name']}' SOL{p['sol_id']}"
    fk_joints_deg = _to_fk_api_joints_deg(p["joints_deg"])

    try:
        fk_pose = robolink_crx_10ia.SolveFK(fk_joints_deg)
    except TargetReachError as exc:
        pytest.skip(
            "\n".join(
                [
                    f"{hdr}: FK seed rejected before IK",
                    f"fixture_joints_deg={_fmt_joints(p['joints_deg'])}",
                    f"robolink_fk_joints_deg={_fmt_joints(fk_joints_deg)}",
                    f"error={exc}",
                ]
            )
        )

    try:
        ik_best = robolink_crx_10ia.SolveIK(fk_pose, joints_approx=fk_joints_deg)
    except TargetReachError as exc:
        raise AssertionError(
            "\n".join(
                [
                    f"{hdr}: RoboDK IK failed on FK-derived pose",
                    f"  fixture_joints_deg: {_fmt_joints(p['joints_deg'])}",
                    f"  robolink_fk_joints_deg: {_fmt_joints(fk_joints_deg)}",
                    f"  error: {exc}",
                ]
            )
        ) from exc

    ik_best_vals = ik_best.list() if hasattr(ik_best, "list") else list(ik_best)
    assert len(ik_best_vals) >= 6, "\n".join(
        [
            f"{hdr}: RoboDK IK returned no 6-axis solution",
            f"  fixture_joints_deg: {_fmt_joints(p['joints_deg'])}",
            f"  robolink_fk_joints_deg: {_fmt_joints(fk_joints_deg)}",
            f"  ik_raw: {ik_best_vals}",
            "  fk_pose16:",
            _fmt_pose16(_mat_to_pose16_col_major(fk_pose)),
        ]
    )

    ik_best_coupled_6 = [float(v) for v in ik_best_vals[:6]]
    ik_best_6 = _from_fk_api_joints_deg(ik_best_coupled_6)
    max_wrapped_diff = max(
        _wrapped_diff_deg(a, b) for a, b in zip(ik_best_6, p["joints_deg"])
    )
    assert _joints_close(ik_best_6, p["joints_deg"], JOINT_TOL_DEG), "\n".join(
        [
            f"{hdr}: IK best solution diverges from fixture branch",
            f"  tol_deg: {JOINT_TOL_DEG}",
            f"  max_wrapped_diff_deg: {_fmt_f(max_wrapped_diff)}",
            f"  fixture_joints_deg: {_fmt_joints(p['joints_deg'])}",
            f"  ik_best_coupled_deg: {_fmt_joints(ik_best_coupled_6)}",
            f"  ik_best_joints_deg: {_fmt_joints(ik_best_6)}",
        ]
    )


@pytest.mark.parametrize("seed", range(_ROBOLINK_FUZZ_SEEDS), ids=lambda s: f"seed_{s:03d}")
def test_robolink_fuzz_fk_ik_roundtrip(robolink_crx_10ia, seed: int):
    lower_limits, upper_limits, _ = robolink_crx_10ia.JointLimits()
    lower_src = lower_limits.list() if hasattr(lower_limits, "list") else lower_limits
    upper_src = upper_limits.list() if hasattr(upper_limits, "list") else upper_limits
    lower = [float(v) for v in lower_src]
    upper = [float(v) for v in upper_src]
    rng = np.random.default_rng(seed)

    coupled_seed = _sample_joints_in_limits(rng, lower, upper)
    decoupled_seed = _from_fk_api_joints_deg(coupled_seed)
    hdr = f"FUZZ seed={seed:03d}"

    try:
        pose = robolink_crx_10ia.SolveFK(coupled_seed)
    except TargetReachError as exc:
        pytest.skip(
            "\n".join(
                [
                    f"{hdr}: RoboDK FK rejected sampled seed",
                    f"coupled_seed_deg={_fmt_joints(coupled_seed)}",
                    f"error={exc}",
                ]
            )
        )

    try:
        ik_best = robolink_crx_10ia.SolveIK(pose, joints_approx=coupled_seed)
    except TargetReachError as exc:
        raise AssertionError(
            "\n".join(
                [
                    f"{hdr}: IK failed for FK-generated pose",
                    f"coupled_seed_deg={_fmt_joints(coupled_seed)}",
                    f"decoupled_seed_deg={_fmt_joints(decoupled_seed)}",
                    f"error={exc}",
                ]
            )
        ) from exc

    ik_vals = ik_best.list() if hasattr(ik_best, "list") else list(ik_best)
    assert len(ik_vals) >= 6, "\n".join(
        [
            f"{hdr}: IK returned no 6-axis solution",
            f"coupled_seed_deg={_fmt_joints(coupled_seed)}",
            f"decoupled_seed_deg={_fmt_joints(decoupled_seed)}",
            f"ik_raw={ik_vals}",
        ]
    )

    ik_coupled = [float(v) for v in ik_vals[:6]]
    ik_decoupled = _from_fk_api_joints_deg(ik_coupled)
    ik_coupled_wrapped = _wrap_joints_to_limits(ik_coupled, lower, upper)
    if ik_coupled_wrapped is None:
        pytest.skip(
            "\n".join(
                [
                    f"{hdr}: IK branch has no equivalent coupled representation within limits",
                    f"ik_decoupled_deg={_fmt_joints(ik_decoupled)}",
                    f"ik_coupled_deg={_fmt_joints(ik_coupled)}",
                ]
            )
        )

    pose_seed16 = _mat_to_pose16_col_major(pose)
    try:
        pose_roundtrip = robolink_crx_10ia.SolveFK(ik_coupled_wrapped)
    except TargetReachError as exc:
        pytest.skip(
            "\n".join(
                [
                    f"{hdr}: FK rejected IK result after limit wrapping",
                    f"ik_decoupled_deg={_fmt_joints(ik_decoupled)}",
                    f"ik_coupled_deg={_fmt_joints(ik_coupled)}",
                    f"ik_coupled_wrapped_deg={_fmt_joints(ik_coupled_wrapped)}",
                    f"error={exc}",
                ]
            )
        )
    pose_roundtrip16 = _mat_to_pose16_col_major(pose_roundtrip)

    pos_err, ang_err = _pose_err_pos_mm_ang_deg(pose_seed16, pose_roundtrip16)
    assert (
        pos_err <= _ROBOLINK_FUZZ_POSE_POS_TOL_MM
        and ang_err <= _ROBOLINK_FUZZ_POSE_ANG_TOL_DEG
    ), "\n".join(
        [
            f"{hdr}: FK->IK->FK pose roundtrip mismatch",
            f"  pos_err_mm: {_fmt_f(pos_err)} (tol={_ROBOLINK_FUZZ_POSE_POS_TOL_MM})",
            f"  ang_err_deg: {_fmt_f(ang_err)} (tol={_fmt_f(_ROBOLINK_FUZZ_POSE_ANG_TOL_DEG)})",
            f"  coupled_seed_deg: {_fmt_joints(coupled_seed)}",
            f"  decoupled_seed_deg: {_fmt_joints(decoupled_seed)}",
            f"  ik_decoupled_deg: {_fmt_joints(ik_decoupled)}",
            f"  ik_coupled_deg: {_fmt_joints(ik_coupled)}",
            f"  ik_coupled_wrapped_deg: {_fmt_joints(ik_coupled_wrapped)}",
            "  pose_seed16:",
            _fmt_pose16(pose_seed16),
            "  pose_roundtrip16:",
            _fmt_pose16(pose_roundtrip16),
        ]
    )
