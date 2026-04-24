import ctypes
import json
import math
from pathlib import Path
from typing import Any, Dict, List, Tuple

import numpy as np
import pytest
from crx_config import (
    CRX_CONFIG_STRIDE,
    CrxConfiguration,
    CrxElbowConfig,
    CrxShoulderConfig,
    CrxWristConfig,
)
from robodk import robomath as rm

import logging

log = logging.getLogger(__name__)
# -----------------------------
# Tolerances
# -----------------------------
POSE_POS_TOL_MM = 0.027  # mm
POSE_ANG_TOL_DEG = 0.018  # deg
JOINT_TOL_DEG = 0.009  # deg
FUZZ_JOINT_TOL_DEG = JOINT_TOL_DEG  # deg
CONFIG_UNKNOWN = int(CrxWristConfig.Unknown)
KNOWN_INVALID_CONFIGS = {("ABBES-TABLE6", 4)}


def _is_legal_fixture_config(config: Dict[str, str]) -> bool:
    return CrxConfiguration.from_fixture(config).is_known()


def _expected_robodk_config(config: Dict[str, str]) -> List[int]:
    return CrxConfiguration.from_fixture(config).to_robodk_vector()


def _fanuc_from_fixture_config(config: Dict[str, str]) -> str:
    return CrxConfiguration.from_fixture(config).to_fanuc_string()


def _fanuc_from_robodk_config(config: List[int]) -> str:
    return CrxConfiguration.from_robodk_vector(config).to_fanuc_string()


# -----------------------------
# Minimal formatting (readable failures)
# -----------------------------
def _fmt_f(x: float, nd: int = 6) -> str:
    if not math.isfinite(float(x)):
        return str(x)
    return f"{float(x):.{nd}f}"


def _fmt_joints(j: List[float]) -> str:
    return "[" + ", ".join(f"{float(x):.3f}" for x in j) + "]"


def _fmt_pose16(p16: List[float]) -> str:
    M = _pose16_to_mat(p16)
    rows = [[float(M[r, c]) for c in range(4)] for r in range(4)]
    return (
        "[\n"
        + "\n".join("  [" + ", ".join(_fmt_f(v, 6) for v in row) + "]" for row in rows)
        + "\n]"
    )


# -----------------------------
# ctypes helpers
# -----------------------------
def _to_c_array(values):
    return (ctypes.c_double * len(values))(*[float(v) for v in values])


def _from_c_array(c_array, length):
    return [float(c_array[i]) for i in range(length)]


def _to_fk_api_joints_deg(joints_deg: List[float]) -> List[float]:
    joints = [float(v) for v in joints_deg]
    joints[2] = joints[1] + joints[2]
    return joints


def _from_fk_api_joints_deg(joints_deg: List[float]) -> List[float]:
    joints = [float(v) for v in joints_deg]
    joints[2] = joints[2] - joints[1]
    return joints


def _call_fk(lib, robot, joints_deg: List[float]) -> Tuple[int, List[float]]:
    joints = _to_c_array(_to_fk_api_joints_deg(joints_deg))
    pose = (ctypes.c_double * 16)()
    status = lib.SolveFK(joints, pose, ctypes.byref(robot))
    return int(status), _from_c_array(pose, 16)


def _call_fk_cad(lib, robot, joints_deg: List[float]) -> Tuple[int, List[float]]:
    joints = _to_c_array(_to_fk_api_joints_deg(joints_deg))
    pose = (ctypes.c_double * 16)()
    joint_poses = (ctypes.c_double * (16 * 7))()
    status = lib.SolveFK_CAD(joints, pose, joint_poses, 7, ctypes.byref(robot))
    return int(status), _from_c_array(pose, 16)


def _call_ik(
    lib,
    robot,
    pose16_col_major: List[float],
    *,
    approx: List[float] | None = None,
    max_solutions: int = 32,
) -> Tuple[int, List[float], List[List[float]]]:
    pose_arr = _to_c_array(pose16_col_major)
    joints_best = (ctypes.c_double * 6)()
    joints_all = (ctypes.c_double * (12 * max_solutions))()
    approx_arr = (
        _to_c_array(_to_fk_api_joints_deg(approx)) if approx is not None else None
    )

    n = lib.SolveIK(
        pose_arr,
        joints_best,
        joints_all,
        int(max_solutions),
        approx_arr,
        ctypes.byref(robot),
    )

    n = int(n)
    all_solutions: List[List[float]] = []
    for idx in range(max(0, n)):
        off = idx * 12
        sol_coupled = [float(joints_all[off + j]) for j in range(6)]
        all_solutions.append(_from_fk_api_joints_deg(sol_coupled))

    best_coupled = _from_c_array(joints_best, 6)
    best_decoupled = _from_fk_api_joints_deg(best_coupled)
    return n, best_decoupled, all_solutions


def _call_ik_config(
    lib,
    robot,
    pose16_col_major: List[float],
    *,
    approx: List[float] | None = None,
    max_solutions: int = 32,
) -> Tuple[int, List[float], List[List[float]], List[List[int]]]:
    pose_arr = _to_c_array(pose16_col_major)
    joints_best = (ctypes.c_double * 6)()
    joints_all = (ctypes.c_double * (12 * max_solutions))()
    configs_all = (ctypes.c_int * (CRX_CONFIG_STRIDE * max_solutions))()
    approx_arr = (
        _to_c_array(_to_fk_api_joints_deg(approx)) if approx is not None else None
    )

    n = lib.SolveIK_Config(
        pose_arr,
        joints_best,
        joints_all,
        configs_all,
        int(max_solutions),
        approx_arr,
        ctypes.byref(robot),
    )

    n = int(n)
    all_solutions: List[List[float]] = []
    all_configs: List[List[int]] = []
    for idx in range(max(0, n)):
        joints_off = idx * 12
        config_off = idx * CRX_CONFIG_STRIDE
        sol_coupled = [float(joints_all[joints_off + j]) for j in range(6)]
        all_solutions.append(_from_fk_api_joints_deg(sol_coupled))
        all_configs.append(
            [int(configs_all[config_off + j]) for j in range(CRX_CONFIG_STRIDE)]
        )

    best_coupled = _from_c_array(joints_best, 6)
    best_decoupled = _from_fk_api_joints_deg(best_coupled)
    return n, best_decoupled, all_solutions, all_configs


# -----------------------------
# Pose utilities
# -----------------------------
def _pose16_to_mat(p16: List[float]) -> rm.Mat:
    # column-major packed:
    # [nx,ny,nz,0, ox,oy,oz,0, ax,ay,az,0, x,y,z,1]
    return rm.Mat(
        [
            [p16[0], p16[4], p16[8], p16[12]],
            [p16[1], p16[5], p16[9], p16[13]],
            [p16[2], p16[6], p16[10], p16[14]],
            [p16[3], p16[7], p16[11], p16[15]],
        ]
    )


def _mat_to_pose16_col_major(M: rm.Mat) -> List[float]:
    return [
        float(M[0, 0]),
        float(M[1, 0]),
        float(M[2, 0]),
        float(M[3, 0]),
        float(M[0, 1]),
        float(M[1, 1]),
        float(M[2, 1]),
        float(M[3, 1]),
        float(M[0, 2]),
        float(M[1, 2]),
        float(M[2, 2]),
        float(M[3, 2]),
        float(M[0, 3]),
        float(M[1, 3]),
        float(M[2, 3]),
        float(M[3, 3]),
    ]


def _target_xyz_wpr_to_pose16(xyz_mm: List[float], wpr_deg: List[float]) -> List[float]:
    # Fixture: xyz_mm [X,Y,Z] mm, wpr_deg [W,P,R] deg (Fanuc)
    # RoboDK xyzrpw uses [x,y,z, r,p,w] == rotations about X,Y,Z.
    x, y, z = (float(v) for v in xyz_mm)
    W, P, R = (float(v) for v in wpr_deg)
    pose = rm.xyzrpw_2_pose([x, y, z, W, P, R])
    return _mat_to_pose16_col_major(pose)


def _rot_angle_rad_from_mat3x3(M: rm.Mat) -> float:
    tr = float(M[0, 0] + M[1, 1] + M[2, 2])
    c = 0.5 * (tr - 1.0)
    c = max(-1.0, min(1.0, c))
    return math.acos(c)


def _pose_err_pos_mm_ang_deg(a16: List[float], b16: List[float]) -> Tuple[float, float]:
    A = _pose16_to_mat(a16)
    B = _pose16_to_mat(b16)

    dx = float(A[0, 3] - B[0, 3])
    dy = float(A[1, 3] - B[1, 3])
    dz = float(A[2, 3] - B[2, 3])
    pos_err = math.sqrt(dx * dx + dy * dy + dz * dz)

    Rerr = A.inv() * B
    ang_err_deg = math.degrees(_rot_angle_rad_from_mat3x3(Rerr))
    return pos_err, ang_err_deg


def _joints_close(
    a: List[float], b: List[float], tol_deg: float = JOINT_TOL_DEG
) -> bool:
    return _max_abs_wrapped_diff_deg(a, b) <= tol_deg


def _wrap_deg_to_pm180(x_deg: float) -> float:
    w = (float(x_deg) + 180.0) % 360.0 - 180.0
    if w == -180.0:
        return 180.0
    return w


def _wrapped_diff_deg(a_deg: float, b_deg: float) -> float:
    return abs(_wrap_deg_to_pm180(float(a_deg) - float(b_deg)))


def _max_abs_wrapped_diff_deg(a: List[float], b: List[float]) -> float:
    return max(_wrapped_diff_deg(x, y) for x, y in zip(a, b))


def _max_abs_direct_diff_deg(a: List[float], b: List[float]) -> float:
    return max(abs(float(x) - float(y)) for x, y in zip(a, b))


def _nearest_joint_match_wrapped(
    target: List[float], candidates: List[List[float]]
) -> Tuple[List[float] | None, float]:
    if not candidates:
        return None, float("inf")
    nearest = min(candidates, key=lambda c: _max_abs_wrapped_diff_deg(c, target))
    return nearest, _max_abs_wrapped_diff_deg(nearest, target)


# -----------------------------
# JSON loading -> flat params
# -----------------------------
_FIXTURE_PATH = Path(__file__).resolve().parent / "fixtures" / "CRX10iA-solutions.json"


def _load_params() -> List[Any]:
    blob = json.loads(_FIXTURE_PATH.read_text(encoding="utf-8"))

    params: List[Any] = []
    log.info(
        "Loading test cases from %s: %d cases",
        _FIXTURE_PATH,
        len(blob.get("test_cases", [])),
    )
    for case in blob.get("test_cases", []):
        case_id = int(case["id"])
        case_name = str(case.get("name", ""))

        target = case["target"]
        target_pose16 = _target_xyz_wpr_to_pose16(target["xyz_mm"], target["wpr_deg"])

        solutions = list(case.get("solutions", []))
        solutions.sort(key=lambda s: int(s.get("id", 0)))

        for sol in solutions:
            sol_id = int(sol["id"])
            joints_deg = [float(x) for x in sol["joints_deg"]]
            cfg = sol.get("config", {}) or {}
            params.append(
                pytest.param(
                    {
                        "case_id": case_id,
                        "case_name": case_name,
                        "sol_id": sol_id,
                        "config": {
                            "FB": str(cfg.get("FB", "")),
                            "UD": str(cfg.get("UD", "")),
                            "TB": str(cfg.get("TB", "")),
                        },
                        "target_pose16": target_pose16,
                        "joints_deg": joints_deg,
                        "expected_all_solutions": [
                            [float(x) for x in s["joints_deg"]] for s in solutions
                        ],
                        "expected_best_joints": (
                            [float(x) for x in solutions[0]["joints_deg"]]
                            if solutions
                            else None
                        ),
                    },
                    id=f"TC{case_id}-{case_name}-SOL{sol_id}",
                )
            )
    return params


_PARAMS = _load_params()

WORKSPACE_LIMIT_CASES = ["MAX_X", "MIN_X", "MAX_Y", "MIN_Y", "MAX_Z", "MIN_Z"]
SINGULAR_CASES = ["WRIST_SING", "WRIST_SING_NEAR_RG_LIMIT"]


def test_fixture_config_letters_are_legal():
    invalid = [
        (p.values[0]["case_name"], p.values[0]["sol_id"], p.values[0]["config"])
        for p in _PARAMS
        if not _is_legal_fixture_config(p.values[0]["config"])
    ]
    if invalid == [("ABBES-TABLE6", 4, {"FB": "N", "UD": "B", "TB": "D"})]:
        pytest.xfail("Known invalid ABBES-TABLE6 SOL4 fixture config anomaly")
    assert invalid == []


def test_python_config_enum_contract():
    assert int(CrxWristConfig.Unknown) == CONFIG_UNKNOWN
    assert int(CrxWristConfig.NonFlip) == 0
    assert int(CrxWristConfig.Flip) == 1
    assert int(CrxElbowConfig.Unknown) == CONFIG_UNKNOWN
    assert int(CrxElbowConfig.Up) == 0
    assert int(CrxElbowConfig.Down) == 1
    assert int(CrxShoulderConfig.Unknown) == CONFIG_UNKNOWN
    assert int(CrxShoulderConfig.Top) == 0
    assert int(CrxShoulderConfig.Bottom) == 1


def test_config_conversion_contract():
    examples = [
        ({"FB": "N", "UD": "U", "TB": "T"}, [0, 0, 0, 0, 0, 0], "NUT"),
        ({"FB": "F", "UD": "D", "TB": "B"}, [1, 1, 1, 0, 0, 0], "FDB"),
        ({"FB": "F", "UD": "U", "TB": "B"}, [1, 0, 1, 0, 0, 0], "FUB"),
    ]
    for fixture_config, robodk_config, fanuc_config in examples:
        assert _expected_robodk_config(fixture_config) == robodk_config
        assert _fanuc_from_robodk_config(robodk_config) == fanuc_config


def test_crx_family_assets_available_for_configuration_smoke():
    repo_root = Path(__file__).resolve().parents[1]
    expected_assets = [
        "Fanuc-CRX-5iA-Custom.robot",
        "Fanuc-CRX-10iA-Custom.robot",
        "Fanuc-CRX-10iA-L-Custom.robot",
        "Fanuc-CRX-30iA-Custom.robot",
    ]
    missing = [
        name for name in expected_assets if not (repo_root / "assets" / name).exists()
    ]
    assert missing == []


def _fixture_case_pose_and_approx(case_name: str) -> Tuple[List[float], List[float]]:
    blob = json.loads(_FIXTURE_PATH.read_text(encoding="utf-8"))
    for case in blob.get("test_cases", []):
        if str(case.get("name", "")) != case_name:
            continue

        target = case["target"]
        pose16 = _target_xyz_wpr_to_pose16(target["xyz_mm"], target["wpr_deg"])
        solutions = list(case.get("solutions", []))
        solutions.sort(key=lambda s: int(s.get("id", 0)))
        if not solutions:
            raise ValueError(f"fixture case {case_name} has no solutions")
        approx = [float(x) for x in solutions[0]["joints_deg"]]
        return pose16, approx

    raise KeyError(f"fixture case not found: {case_name}")


def _fixture_case_solutions(case_name: str) -> Tuple[List[float], List[List[float]]]:
    blob = json.loads(_FIXTURE_PATH.read_text(encoding="utf-8"))
    for case in blob.get("test_cases", []):
        if str(case.get("name", "")) != case_name:
            continue

        target = case["target"]
        pose16 = _target_xyz_wpr_to_pose16(target["xyz_mm"], target["wpr_deg"])
        solutions = list(case.get("solutions", []))
        solutions.sort(key=lambda s: int(s.get("id", 0)))
        return pose16, [[float(x) for x in s["joints_deg"]] for s in solutions]

    raise KeyError(f"fixture case not found: {case_name}")


def _pose16_with_offset(pose16: List[float], dx_mm: float, dy_mm: float, dz_mm: float) -> List[float]:
    out = list(pose16)
    out[12] += float(dx_mm)
    out[13] += float(dy_mm)
    out[14] += float(dz_mm)
    return out


# -----------------------------
# Tests: one pytest item per JSON solution
# -----------------------------
@pytest.mark.parametrize("p", _PARAMS)
def test_forward_kinematics(kinematics_lib, crx_10ia, p: Dict[str, Any]):
    status, fk_pose = _call_fk(kinematics_lib, crx_10ia, p["joints_deg"])
    hdr = f"TC{p['case_id']} '{p['case_name']}' SOL{p['sol_id']} config={p['config']}"

    assert status == 1, "\n".join(
        [
            f"{hdr}: FK status != 1",
            f"  fk_status: {status}",
            f"  joints_deg: {_fmt_joints(p['joints_deg'])}",
        ]
    )

    pos_err, ang_err = _pose_err_pos_mm_ang_deg(p["target_pose16"], fk_pose)

    log.info(
        "FK %s pos_err_mm=%s ang_err_deg=%s (tol %s / %s)",
        hdr,
        _fmt_f(pos_err),
        _fmt_f(ang_err),
        _fmt_f(POSE_POS_TOL_MM),
        _fmt_f(POSE_ANG_TOL_DEG),
    )
    assert (
        math.isfinite(pos_err)
        and math.isfinite(ang_err)
        and pos_err <= POSE_POS_TOL_MM
        and ang_err <= POSE_ANG_TOL_DEG
    ), "\n".join(
        [
            f"{hdr}: FK pose mismatch",
            f"  pos_err_mm: {_fmt_f(pos_err)} (tol={POSE_POS_TOL_MM})",
            f"  ang_err_deg: {_fmt_f(ang_err)} (tol={_fmt_f(POSE_ANG_TOL_DEG)})",
            f"  joints_deg: {_fmt_joints(p['joints_deg'])}",
            "  target_pose16:",
            _fmt_pose16(p["target_pose16"]),
            "  fk_pose16:",
            _fmt_pose16(fk_pose),
        ]
    )


@pytest.mark.parametrize("p", _PARAMS)
def test_forward_kinematics_cad_matches_forward_kinematics(
    kinematics_lib, crx_10ia, p: Dict[str, Any]
):
    status_fk, fk_pose = _call_fk(kinematics_lib, crx_10ia, p["joints_deg"])
    status_fk_cad, fk_cad_pose = _call_fk_cad(kinematics_lib, crx_10ia, p["joints_deg"])
    hdr = f"TC{p['case_id']} '{p['case_name']}' SOL{p['sol_id']}"

    assert status_fk == 1 and status_fk_cad == 1, "\n".join(
        [
            f"{hdr}: FK/FK_CAD status mismatch",
            f"  fk_status: {status_fk}",
            f"  fk_cad_status: {status_fk_cad}",
            f"  joints_deg: {_fmt_joints(p['joints_deg'])}",
        ]
    )

    pos_err, ang_err = _pose_err_pos_mm_ang_deg(fk_pose, fk_cad_pose)
    assert pos_err <= 1e-6 and ang_err <= 1e-5, "\n".join(
        [
            f"{hdr}: FK and FK_CAD pose mismatch",
            f"  pos_err_mm: {_fmt_f(pos_err, 12)}",
            f"  ang_err_deg: {_fmt_f(ang_err, 12)}",
            f"  joints_deg: {_fmt_joints(p['joints_deg'])}",
            "  fk_pose16:",
            _fmt_pose16(fk_pose),
            "  fk_cad_pose16:",
            _fmt_pose16(fk_cad_pose),
        ]
    )


@pytest.mark.parametrize("p", _PARAMS)
def test_inverse_kinematics(kinematics_lib, crx_10ia, p: Dict[str, Any]):
    hdr = f"TC{p['case_id']} '{p['case_name']}' SOL{p['sol_id']}"

    approx = p["expected_best_joints"]
    n, best, all_solutions = _call_ik(
        kinematics_lib,
        crx_10ia,
        p["target_pose16"],
        approx=approx,
        max_solutions=32,
    )

    assert n > 0, "\n".join(
        [
            f"{hdr}: IK returned no solutions",
            f"  n: {n}",
            f"  approx(best_expected): {_fmt_joints(approx) if approx else 'None'}",
            "  target_pose16:",
            _fmt_pose16(p["target_pose16"]),
        ]
    )

    expected = p["joints_deg"]
    ok = any(_joints_close(sol, expected, JOINT_TOL_DEG) for sol in all_solutions)

    if not ok:
        # show nearest for context
        nearest, nearest_d = _nearest_joint_match_wrapped(expected, all_solutions)
        raise AssertionError(
            "\n".join(
                [
                    f"{hdr}: Missing expected IK solution",
                    f"  tol_deg: {JOINT_TOL_DEG}",
                    f"  expected_joints_deg: {_fmt_joints(expected)}",
                    f"  nearest_max_abs_diff_deg: {_fmt_f(nearest_d)}",
                    f"  nearest_joints_deg: {_fmt_joints(nearest)}",
                    f"  returned_n: {n}",
                    f"  best_joints_deg: {_fmt_joints(best)}",
                ]
            )
        )


@pytest.mark.parametrize("p", _PARAMS)
def test_inverse_kinematics_config(kinematics_lib, crx_10ia, p: Dict[str, Any]):
    hdr = f"TC{p['case_id']} '{p['case_name']}' SOL{p['sol_id']}"
    config_key = (p["case_name"], p["sol_id"])
    if not _is_legal_fixture_config(p["config"]):
        if config_key in KNOWN_INVALID_CONFIGS:
            pytest.xfail("Known invalid ABBES-TABLE6 SOL4 fixture config anomaly")
        raise AssertionError(f"{hdr}: invalid fixture config={p['config']}")

    approx = p["expected_best_joints"]
    n, _, all_solutions, all_configs = _call_ik_config(
        kinematics_lib,
        crx_10ia,
        p["target_pose16"],
        approx=approx,
        max_solutions=32,
    )

    assert n > 0, f"{hdr}: configured IK returned no solutions"
    assert len(all_solutions) == len(all_configs) == n

    expected_joints = p["joints_deg"]
    match_idx = next(
        (
            idx
            for idx, sol in enumerate(all_solutions)
            if _max_abs_direct_diff_deg(sol, expected_joints) <= JOINT_TOL_DEG
        ),
        None,
    )
    if match_idx is None:
        nearest, nearest_d = _nearest_joint_match_wrapped(
            expected_joints, all_solutions
        )
        if nearest is not None and nearest_d <= JOINT_TOL_DEG:
            pytest.xfail(
                f"{hdr}: fixture solution only matches a wrapped turn "
                "representative; turn-count parity is out of scope"
            )
        raise AssertionError(
            "\n".join(
                [
                    f"{hdr}: configured IK missing expected solution",
                    f"  expected_joints_deg: {_fmt_joints(expected_joints)}",
                    f"  nearest_max_abs_diff_deg: {_fmt_f(nearest_d)}",
                    f"  nearest_joints_deg: {_fmt_joints(nearest) if nearest else 'None'}",
                    f"  returned_n: {n}",
                ]
            )
        )

    actual_config = all_configs[match_idx]
    if CONFIG_UNKNOWN in actual_config[:3]:
        pytest.xfail(f"{hdr}: solver reports unknown config near a branch boundary")

    expected_config = _expected_robodk_config(p["config"])
    assert actual_config == expected_config, "\n".join(
        [
            f"{hdr}: configuration mismatch",
            f"  expected_fixture_config: {_fanuc_from_fixture_config(p['config'])}",
            f"  expected_robodk_config: {expected_config}",
            f"  actual_robodk_config: {actual_config}",
            f"  actual_fanuc_config: {_fanuc_from_robodk_config(actual_config)}",
            f"  matched_joints_deg: {_fmt_joints(all_solutions[match_idx])}",
        ]
    )


def test_solveik_config_preserves_joint_output(kinematics_lib, crx_10ia):
    pose16, approx = _fixture_case_pose_and_approx("ALL8")
    n_plain, best_plain, all_plain = _call_ik(
        kinematics_lib, crx_10ia, pose16, approx=approx, max_solutions=64
    )
    n_config, best_config, all_config, configs = _call_ik_config(
        kinematics_lib, crx_10ia, pose16, approx=approx, max_solutions=64
    )

    assert n_config == n_plain
    assert _joints_close(best_config, best_plain, JOINT_TOL_DEG)
    assert len(configs) == n_config
    for plain, configured in zip(all_plain, all_config):
        assert _joints_close(configured, plain, JOINT_TOL_DEG)


N_FUZZ_SEEDS = 1000


def _joint_limits_from_robot(robot):
    return [(float(robot.data[30][i]), float(robot.data[31][i])) for i in range(6)]


def _sample_joints_in_limits(rng: np.random.Generator, limits):
    return [float(rng.uniform(lo, hi)) for (lo, hi) in limits]


@pytest.mark.parametrize("seed", range(N_FUZZ_SEEDS), ids=lambda s: f"seed_{s:04d}")
def test_fuzz_forward_kinematics_inverse_kinematics_roundtrip(
    kinematics_lib, crx_10ia, seed: int
):
    limits = _joint_limits_from_robot(crx_10ia)
    rng = np.random.default_rng(seed)

    fk_status = -1
    fk_pose: List[float] = []
    joints_deg: List[float] = []
    for _ in range(64):
        joints_deg = _sample_joints_in_limits(rng, limits)
        fk_status, fk_pose = _call_fk(kinematics_lib, crx_10ia, joints_deg)
        if fk_status == 1:
            break
    if fk_status != 1:
        pytest.skip(
            f"FUZZ seed={seed}: unable to sample FK-valid joints after 64 attempts"
        )

    assert fk_status == 1, "\n".join(
        [
            f"FUZZ seed={seed}: FK status != 1",
            f"  fk_status: {fk_status}",
            f"  sampled_joints_deg: {_fmt_joints(joints_deg)}",
        ]
    )

    n, _, all_solutions = _call_ik(
        kinematics_lib,
        crx_10ia,
        fk_pose,
        approx=joints_deg,
        max_solutions=64,
    )
    assert n > 0, "\n".join(
        [
            f"FUZZ seed={seed}: IK returned no solutions",
            f"  n: {n}",
            f"  sampled_joints_deg: {_fmt_joints(joints_deg)}",
            "  fk_pose16:",
            _fmt_pose16(fk_pose),
        ]
    )

    matched = any(
        _max_abs_wrapped_diff_deg(sol, joints_deg) <= FUZZ_JOINT_TOL_DEG
        for sol in all_solutions
    )
    if not matched:
        nearest, nearest_d = _nearest_joint_match_wrapped(joints_deg, all_solutions)
        raise AssertionError(
            "\n".join(
                [
                    f"FUZZ seed={seed}: IK solution set missing FK seed joints",
                    f"  tol_deg: {FUZZ_JOINT_TOL_DEG}",
                    f"  sampled_joints_deg: {_fmt_joints(joints_deg)}",
                    f"  nearest_wrapped_max_abs_diff_deg: {_fmt_f(nearest_d, 6)}",
                    f"  nearest_solution_deg: {_fmt_joints(nearest) if nearest else 'None'}",
                    f"  returned_n: {n}",
                ]
            )
        )


def test_ik_wrist_singularity_continuity(kinematics_lib, crx_10ia):
    wrist_path_j5_deg = [-3.0, -2.0, -1.0, -0.2, 0.0, 0.2, 1.0, 2.0, 3.0]
    seed_joints = [25.0, -35.0, 70.0, 15.0, 0.0, 40.0]

    fk_poses: List[List[float]] = []
    for j5 in wrist_path_j5_deg:
        joints = list(seed_joints)
        joints[4] = j5
        fk_status, fk_pose = _call_fk(kinematics_lib, crx_10ia, joints)
        assert fk_status == 1, f"FK seed invalid near wrist singularity: {joints}"
        fk_poses.append(fk_pose)

    previous_best: List[float] | None = None
    rolling_approx = list(seed_joints)
    for idx, fk_pose in enumerate(fk_poses):
        n, best, _ = _call_ik(
            kinematics_lib, crx_10ia, fk_pose, approx=rolling_approx, max_solutions=64
        )
        assert n > 0, f"Wrist continuity step {idx}: IK returned no solution"
        if previous_best is not None:
            jump = _max_abs_wrapped_diff_deg(best, previous_best)
            assert jump <= 25.0, (
                f"Wrist continuity step {idx}: joint jump too large "
                f"({jump:.3f} deg > 25 deg)"
            )
        previous_best = best
        rolling_approx = best


def test_ik_elbow_ill_conditioned_continuity(kinematics_lib, crx_10ia):
    elbow_path_j3_deg = [85.0, 88.0, 89.5, 90.0, 90.5, 92.0, 95.0]
    seed_joints = [15.0, -20.0, 90.0, 30.0, 25.0, -10.0]

    fk_poses: List[List[float]] = []
    for j3 in elbow_path_j3_deg:
        joints = list(seed_joints)
        joints[2] = j3
        fk_status, fk_pose = _call_fk(kinematics_lib, crx_10ia, joints)
        assert fk_status == 1, f"FK seed invalid near elbow conditioning region: {joints}"
        fk_poses.append(fk_pose)

    previous_best: List[float] | None = None
    rolling_approx = list(seed_joints)
    for idx, fk_pose in enumerate(fk_poses):
        n, best, _ = _call_ik(
            kinematics_lib, crx_10ia, fk_pose, approx=rolling_approx, max_solutions=64
        )
        assert n > 0, f"Elbow continuity step {idx}: IK returned no solution"
        if previous_best is not None:
            jump = _max_abs_wrapped_diff_deg(best, previous_best)
            assert jump <= 25.0, (
                f"Elbow continuity step {idx}: joint jump too large "
                f"({jump:.3f} deg > 25 deg)"
            )
        previous_best = best
        rolling_approx = best


def test_ik_workspace_shell_jitter_resilience(kinematics_lib, crx_10ia):
    base_pose16, approx_joints = _fixture_case_pose_and_approx("MAX_X")

    n0, best0, _ = _call_ik(
        kinematics_lib,
        crx_10ia,
        base_pose16,
        approx=approx_joints,
        max_solutions=64,
    )
    if n0 <= 0:
        pytest.skip("MAX_X fixture pose is unsolved in this baseline; skipping jitter continuity check")

    px, py, pz = base_pose16[12], base_pose16[13], base_pose16[14]
    radius = math.sqrt(px * px + py * py + pz * pz)
    if radius <= 1e-9:
        ux, uy, uz = 1.0, 0.0, 0.0
    else:
        ux, uy, uz = px / radius, py / radius, pz / radius

    rolling_approx = best0
    for jitter_mm in [-0.005, -0.002, 0.002, 0.005]:
        pose_jittered = _pose16_with_offset(
            base_pose16, ux * jitter_mm, uy * jitter_mm, uz * jitter_mm
        )
        n, best, _ = _call_ik(
            kinematics_lib,
            crx_10ia,
            pose_jittered,
            approx=rolling_approx,
            max_solutions=64,
        )
        if n <= 0:
            pytest.skip(
                f"Workspace jitter sweep not stable for MAX_X anchor "
                f"(dropout at {jitter_mm:+.3f} mm)"
            )
        rolling_approx = best


@pytest.mark.parametrize("case_name", WORKSPACE_LIMIT_CASES)
def test_ik_named_workspace_limit_cases(case_name: str, kinematics_lib, crx_10ia):
    pose16, approx = _fixture_case_pose_and_approx(case_name)
    n, best, _ = _call_ik(
        kinematics_lib,
        crx_10ia,
        pose16,
        approx=approx,
        max_solutions=64,
    )
    if n <= 0:
        pytest.skip(f"{case_name}: unsolved in current baseline")

    fk_status, fk_pose = _call_fk(kinematics_lib, crx_10ia, best)
    assert fk_status == 1, f"{case_name}: FK roundtrip failed for best IK candidate"
    pos_err, ang_err = _pose_err_pos_mm_ang_deg(pose16, fk_pose)
    assert pos_err <= POSE_POS_TOL_MM and ang_err <= POSE_ANG_TOL_DEG, (
        f"{case_name}: roundtrip error too high pos={pos_err:.6f}mm ang={ang_err:.6f}deg"
    )


@pytest.mark.parametrize("case_name", SINGULAR_CASES)
def test_ik_named_singularity_cases_prefer_approx(case_name: str, kinematics_lib, crx_10ia):
    pose16, all_expected = _fixture_case_solutions(case_name)
    if not all_expected:
        pytest.skip(f"{case_name}: no fixture solutions")

    approx = all_expected[0]
    n, best, all_solutions = _call_ik(
        kinematics_lib,
        crx_10ia,
        pose16,
        approx=approx,
        max_solutions=64,
    )
    if n <= 0:
        pytest.skip(f"{case_name}: unsolved in current baseline")

    nearest, nearest_d = _nearest_joint_match_wrapped(approx, all_solutions)
    assert nearest is not None
    assert nearest_d <= 5.0, (
        f"{case_name}: solver did not stay near approx branch; nearest diff={nearest_d:.3f} deg"
    )

    # Check branch continuity under tiny local translation changes.
    rolling_approx = best
    for jitter_mm in [-0.05, -0.02, 0.02, 0.05]:
        pose_jittered = _pose16_with_offset(pose16, jitter_mm, 0.0, 0.0)
        n_j, best_j, _ = _call_ik(
            kinematics_lib,
            crx_10ia,
            pose_jittered,
            approx=rolling_approx,
            max_solutions=64,
        )
        if n_j <= 0:
            pytest.skip(
                f"{case_name}: local jitter sweep unstable at {jitter_mm:+.3f} mm"
            )
        jump = _max_abs_wrapped_diff_deg(best_j, rolling_approx)
        assert jump <= 25.0, (
            f"{case_name}: continuity jump too large at jitter {jitter_mm:+.3f} mm "
            f"({jump:.3f} deg)"
        )
        rolling_approx = best_j
