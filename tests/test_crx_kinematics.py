import ctypes
import json
import math
from pathlib import Path
from typing import Any, Dict, List, Tuple

import numpy as np
import pytest
from robodk import robomath as rm

import logging
log = logging.getLogger(__name__)
# -----------------------------
# Tolerances
# -----------------------------
POSE_POS_TOL_MM = 3e-2  # mm
POSE_ANG_TOL_DEG = 2e-2  # deg
JOINT_TOL_DEG = 0.01  # deg
FUZZ_JOINT_TOL_DEG = JOINT_TOL_DEG  # deg


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


def _call_fk(lib, robot, joints_deg: List[float]) -> Tuple[int, List[float]]:
    joints = _to_c_array(joints_deg)
    pose = (ctypes.c_double * 16)()
    status = lib.SolveFK(joints, pose, ctypes.byref(robot))
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
    approx_arr = _to_c_array(approx) if approx is not None else None

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
        all_solutions.append([float(joints_all[off + j]) for j in range(6)])

    return n, _from_c_array(joints_best, 6), all_solutions


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
_FIXTURE_PATH = (
    Path(__file__).resolve().parent / "fixtures" / "CRX10iA-solutions.json"
)


def _load_params() -> List[Any]:
    blob = json.loads(_FIXTURE_PATH.read_text(encoding="utf-8"))

    params: List[Any] = []
    log.info("Loading test cases from %s: %d cases", _FIXTURE_PATH, len(blob.get("test_cases", [])))
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


# -----------------------------
# Tests: one pytest item per JSON solution
# -----------------------------
@pytest.mark.parametrize("p", _PARAMS)
def test_fk_against_target(kinematics_lib, crx_robot, p: Dict[str, Any]):
    status, fk_pose = _call_fk(kinematics_lib, crx_robot, p["joints_deg"])
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
def test_ik_contains_expected_solution(kinematics_lib, crx_robot, p: Dict[str, Any]):
    hdr = f"TC{p['case_id']} '{p['case_name']}' SOL{p['sol_id']}"

    approx = p["expected_best_joints"]
    n, best, all_solutions = _call_ik(
        kinematics_lib,
        crx_robot,
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


N_FUZZ_SEEDS = 200  # => 200 independent pytest tests


def _joint_limits_from_robot(robot):
    return [(float(robot.data[30][i]), float(robot.data[31][i])) for i in range(6)]


def _sample_joints_in_limits(rng: np.random.Generator, limits):
    return [float(rng.uniform(lo, hi)) for (lo, hi) in limits]


@pytest.mark.parametrize("seed", range(N_FUZZ_SEEDS), ids=lambda s: f"seed_{s:04d}")
def test_fuzz_fk_ik_roundtrip_seeded(kinematics_lib, crx_robot, seed: int):
    limits = _joint_limits_from_robot(crx_robot)
    rng = np.random.default_rng(seed)

    fk_status = -1
    fk_pose: List[float] = []
    joints_deg: List[float] = []
    for _ in range(64):
        joints_deg = _sample_joints_in_limits(rng, limits)
        fk_status, fk_pose = _call_fk(kinematics_lib, crx_robot, joints_deg)
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
        crx_robot,
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
