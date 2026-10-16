"""Recorded regression fixture data and pose comparisons shared by native/live tests."""

import json
import math
from pathlib import Path
from typing import Dict, List, Tuple

import numpy as np
import pytest
from robodk import robomath as rm

POSE_POS_TOL_MM = 0.027  # mm
POSE_ANG_TOL_DEG = 0.018  # deg
JOINT_TOL_DEG = 0.009  # deg
CONFIG_FIXTURE_CASES = {
    "HOME",
    "PERCH",
    "ALLZERO",
    "SQUARE90",
    "ALL8",
    "ONLY7",
    "ABBES-TABLE4",
    "COUPLING1",
    "COUPLING2",
    "COUPLING3",
    "COUPLING4",
    "COUPLING5",
    "COUPLING6",
    "COUPLING7",
    "COUPLING8",
    "WRIST_SING",
    "WRIST_SING_NEAR_RG_LIMIT",
}


def assert_regenerated_cases_match(generated, recorded):
    """Allow math-library roundoff while preserving corpus structure and metadata."""
    for actual, expected in zip(generated, recorded, strict=True):
        assert actual.keys() == expected.keys()
        for field, value in expected.items():
            if field in ("target", "witnesses", "seed_deg"):
                observed, reference = np.asarray(actual[field]), np.asarray(value)
                assert observed.shape == reference.shape
                scale = max(1., np.abs(reference).max(initial=0))
                np.testing.assert_allclose(
                    observed, reference, rtol=0, atol=32 * np.finfo(float).eps * scale,
                    err_msg=f"{expected['name']}: {field}",
                )
            elif field == "offset_rad" and value is not None:
                np.testing.assert_allclose(
                    actual[field], value, rtol=32 * np.finfo(float).eps, atol=0,
                    err_msg=f"{expected['name']}: {field}",
                )
            else:
                assert actual[field] == value, f"{expected['name']}: {field}"


def _is_legal_fixture_config(config: Dict[str, str]) -> bool:
    return (
        config.get("TB") in {"T", "B"}
        and config.get("UD") in {"U", "D"}
        and config.get("FB") in {"N", "F"}
    )


def _expected_robodk_config(config: Dict[str, str]) -> List[int]:
    return [
        {"T": 0, "B": 1}[config["TB"]],
        {"U": 0, "D": 1}[config["UD"]],
        {"N": 0, "F": 1}[config["FB"]],
    ]


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


_FIXTURE_PATH = Path(__file__).resolve().parent / "fixtures" / "CRX10iA-solutions.json"


def _load_cases():
    cases = []
    for source in json.loads(_FIXTURE_PATH.read_text(encoding="utf-8"))["test_cases"]:
        solutions = sorted(source["solutions"], key=lambda solution: int(solution["id"]))
        cases.append({
            "case_id": int(source["id"]),
            "case_name": source["name"],
            "target_pose16": _target_xyz_wpr_to_pose16(
                source["target"]["xyz_mm"], source["target"]["wpr_deg"]),
            "solutions": solutions,
        })
    return cases


CASES = _load_cases()
CASE_PARAMS = [pytest.param(case, id=f"TC{case['case_id']}-{case['case_name']}")
               for case in CASES]
SOLUTION_PARAMS = [
    pytest.param(
        {**case, "sol_id": int(solution["id"]),
         "joints_deg": solution["joints_deg"], "config": solution["config"]},
        id=f"TC{case['case_id']}-{case['case_name']}-SOL{solution['id']}",
    )
    for case in CASES for solution in case["solutions"]
]


def _fixture_case_solutions(case_name: str) -> Tuple[List[float], List[List[float]]]:
    for case in CASES:
        if case["case_name"] == case_name:
            return (list(case["target_pose16"]),
                    [list(solution["joints_deg"]) for solution in case["solutions"]])
    raise KeyError(f"fixture case not found: {case_name}")


def _fixture_case_pose_and_approx(case_name: str) -> Tuple[List[float], List[float]]:
    pose, solutions = _fixture_case_solutions(case_name)
    if not solutions:
        raise ValueError(f"fixture case {case_name} has no solutions")
    return pose, solutions[0]


# Reproductions from a deterministic near-double-root sweep (seed 20261013).
# Independent sphere/base-plane branches were polished with the upstream
# NumPy reference at 26e27c3dae5b2a3c84e1e35fb9faf4c113439e46. Canonical
# radians are stored here; no reference solver is needed to run these tests.
# The target is derived from one witness only, never passed with an IK seed.
MERGED_ELBOW_PAIRS = [
    (
        [1.2846196877740317, 0.014575964159554644, 0.7431200420044268,
         -3.3341878330183563e-08, 0.05221572955259335, 2.293367093214428],
        [1.284619687774032, 0.8422522582116572, 2.398472620156496,
         2.4858111807875285e-09, -0.7754605545473734, 2.293367058142878],
    ),
    (
        [-0.718138546639957, -0.9028352204259087, 1.3992252847326752,
         -7.098671053282173e-08, -1.1604247273379276, 1.3903144867892374],
        [-0.7181385466399571, -0.7312641586451563, 1.742367407185094,
         -6.699400072564219e-08, -1.3319957880095936, 1.3903144743156668],
    ),
    (
        [1.1595086470403193, -1.4974839103438753, 1.1947661103463019,
         1.1290627321119101e-08, -1.4859141154799533, -1.4828980072447888],
        [1.1595086470403193, -1.1214536970315663, 1.9468265368449182,
         1.1744233722926812e-08, -1.8619443286662605, -1.482898002916359],
    ),
]
