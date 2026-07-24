#!/usr/bin/env python
"""A/B benchmark for crx_kinematics DLLs with differential correctness checks."""

from __future__ import annotations

import argparse
import ctypes
import json
import math
import os
import statistics
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np
from robodk import robomath as rm

ROBOT_ROWS = 32
ROBOT_STRIDE = 20
DOFS = 6

MAX_SOLUTIONS = 64
FK_FUZZ_COUNT = 256
IK_FUZZ_COUNT = 128

POSE_DIFF_TOL = 1e-9
JOINT_DIFF_TOL_DEG = 1e-3

EDGE_DEFAULT_PATTERNS = ("MAX", "MIN", "SING")
EDGE_DEFAULT_JITTER_MM = (0.0, -0.05, 0.05, -0.2, 0.2)


class RobotT(ctypes.Structure):
    _fields_ = [("data", (ctypes.c_double * ROBOT_STRIDE) * ROBOT_ROWS)]


def _to_c_array(values: list[float]):
    return (ctypes.c_double * len(values))(*[float(v) for v in values])


def _from_c_array(c_array, length: int) -> list[float]:
    return [float(c_array[i]) for i in range(length)]


def _set_row(robot: RobotT, row: int, values: list[float], col: int = 0) -> None:
    for idx, value in enumerate(values):
        robot.data[row][col + idx] = float(value)


def _build_robot() -> RobotT:
    robot = RobotT()
    robot.data[1][1] = float(DOFS)

    kRobotBaseXyzwprRow = 9
    kRobotToolXyzwprRow = 28
    kRobotJointLowerLimitRow = 30
    kRobotJointUpperLimitRow = 31
    kRobotJointSensesRow = 3
    kRobotJointSensesCol = 4
    kRobotDhBaseRow = 10

    _set_row(robot, kRobotBaseXyzwprRow, [0.0, 0.0, -245.0, 0.0, 0.0, 0.0])
    _set_row(robot, kRobotToolXyzwprRow, [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    _set_row(
        robot,
        kRobotJointSensesRow,
        [1.0, 1.0, -1.0, -1.0, -1.0, -1.0],
        col=kRobotJointSensesCol,
    )
    _set_row(
        robot,
        kRobotJointLowerLimitRow,
        [-190.0, -180.0, -195.0, -190.0, -180.0, -225.0],
    )
    _set_row(
        robot, kRobotJointUpperLimitRow, [190.0, 180.0, 375.0, 190.0, 180.0, 225.0]
    )

    half_pi = math.pi / 2.0
    dh_rows = [
        [0.0, 0.0, 0.0, 245.0, 0.0],
        [-half_pi, 0.0, -half_pi, 0.0, 0.0],
        [0.0, 540.0, 0.0, 0.0, 0.0],
        [-half_pi, 0.0, 0.0, 540.0, 0.0],
        [half_pi, 0.0, 0.0, -150.0, 0.0],
        [-half_pi, 0.0, 0.0, 160.0, 0.0],
    ]
    for idx, row in enumerate(dh_rows):
        _set_row(robot, kRobotDhBaseRow + idx, row)

    return robot


@dataclass(frozen=True)
class FkCase:
    name: str
    joints_deg: list[float]


@dataclass(frozen=True)
class IkCase:
    name: str
    pose16_col_major: list[float]
    approx_deg: list[float] | None


class KinematicsLib:
    def __init__(self, library_path: Path):
        self.path = library_path.resolve()
        if not self.path.exists():
            raise FileNotFoundError(f"DLL not found: {self.path}")

        if os.name == "nt":
            try:
                os.add_dll_directory(str(self.path.parent))
            except Exception:
                pass

        self.lib = ctypes.CDLL(str(self.path))
        real_ptr = ctypes.POINTER(ctypes.c_double)
        robot_ptr = ctypes.POINTER(RobotT)

        self.lib.SolveFK.argtypes = [real_ptr, real_ptr, robot_ptr]
        self.lib.SolveFK.restype = ctypes.c_int
        self.lib.SolveFK_CAD.argtypes = [
            real_ptr,
            real_ptr,
            real_ptr,
            ctypes.c_int,
            robot_ptr,
        ]
        self.lib.SolveFK_CAD.restype = ctypes.c_int
        self.lib.SolveIK.argtypes = [
            real_ptr,
            real_ptr,
            real_ptr,
            ctypes.c_int,
            real_ptr,
            robot_ptr,
        ]
        self.lib.SolveIK.restype = ctypes.c_int

    def solve_fk(
        self, robot: RobotT, joints_deg: list[float]
    ) -> tuple[int, list[float]]:
        joints = _to_c_array(joints_deg)
        pose = (ctypes.c_double * 16)()
        status = int(self.lib.SolveFK(joints, pose, ctypes.byref(robot)))
        return status, _from_c_array(pose, 16)

    def solve_ik(
        self,
        robot: RobotT,
        pose16_col_major: list[float],
        approx_deg: list[float] | None,
        max_solutions: int = MAX_SOLUTIONS,
    ) -> tuple[int, list[float], list[list[float]]]:
        pose_arr = _to_c_array(pose16_col_major)
        joints_best = (ctypes.c_double * DOFS)()
        joints_all = (ctypes.c_double * (2 * DOFS * max_solutions))()
        approx_arr = _to_c_array(approx_deg) if approx_deg is not None else None

        n = int(
            self.lib.SolveIK(
                pose_arr,
                joints_best,
                joints_all,
                int(max_solutions),
                approx_arr,
                ctypes.byref(robot),
            )
        )
        all_solutions: list[list[float]] = []
        for idx in range(max(0, n)):
            off = idx * 12
            all_solutions.append([float(joints_all[off + j]) for j in range(DOFS)])
        return n, _from_c_array(joints_best, DOFS), all_solutions


def _mat_to_pose16_col_major(mat: rm.Mat) -> list[float]:
    return [
        float(mat[0, 0]),
        float(mat[1, 0]),
        float(mat[2, 0]),
        float(mat[3, 0]),
        float(mat[0, 1]),
        float(mat[1, 1]),
        float(mat[2, 1]),
        float(mat[3, 1]),
        float(mat[0, 2]),
        float(mat[1, 2]),
        float(mat[2, 2]),
        float(mat[3, 2]),
        float(mat[0, 3]),
        float(mat[1, 3]),
        float(mat[2, 3]),
        float(mat[3, 3]),
    ]


def _target_xyz_wpr_to_pose16(xyz_mm: list[float], wpr_deg: list[float]) -> list[float]:
    x, y, z = (float(v) for v in xyz_mm)
    w, p, r = (float(v) for v in wpr_deg)
    pose = rm.xyzrpw_2_pose([x, y, z, w, p, r])
    return _mat_to_pose16_col_major(pose)


def _wrap_deg_to_pm180(x_deg: float) -> float:
    w = (float(x_deg) + 180.0) % 360.0 - 180.0
    return 180.0 if w == -180.0 else w


def _max_abs_wrapped_diff_deg(a: list[float], b: list[float]) -> float:
    return max(abs(_wrap_deg_to_pm180(x - y)) for x, y in zip(a, b))


def _load_fixture_cases(fixture_path: Path) -> tuple[list[FkCase], list[IkCase]]:
    blob = json.loads(fixture_path.read_text(encoding="utf-8"))
    fk_cases: list[FkCase] = []
    ik_cases: list[IkCase] = []

    for case in blob.get("test_cases", []):
        case_id = int(case.get("id", -1))
        case_name = str(case.get("name", ""))
        target = case["target"]
        pose16 = _target_xyz_wpr_to_pose16(target["xyz_mm"], target["wpr_deg"])

        solutions = list(case.get("solutions", []))
        solutions.sort(key=lambda s: int(s.get("id", 0)))
        approx = [float(x) for x in solutions[0]["joints_deg"]] if solutions else None
        ik_cases.append(
            IkCase(
                name=f"TC{case_id}-{case_name}",
                pose16_col_major=pose16,
                approx_deg=approx,
            )
        )

        for solution in solutions:
            sol_id = int(solution.get("id", 0))
            joints = [float(x) for x in solution["joints_deg"]]
            fk_cases.append(
                FkCase(name=f"TC{case_id}-{case_name}-SOL{sol_id}", joints_deg=joints)
            )

    return fk_cases, ik_cases


def _parse_csv_tokens(raw: str) -> list[str]:
    return [token.strip() for token in raw.split(",") if token.strip()]


def _parse_csv_floats(raw: str) -> list[float]:
    out: list[float] = []
    for token in _parse_csv_tokens(raw):
        out.append(float(token))
    return out


def _pose16_with_offset(
    pose16: list[float], dx_mm: float, dy_mm: float, dz_mm: float
) -> list[float]:
    out = list(pose16)
    out[12] += float(dx_mm)
    out[13] += float(dy_mm)
    out[14] += float(dz_mm)
    return out


def _build_edge_cases(
    fixture_path: Path,
    *,
    name_patterns: list[str],
    jitter_mm: list[float],
) -> tuple[list[FkCase], list[IkCase]]:
    blob = json.loads(fixture_path.read_text(encoding="utf-8"))
    patterns = [pattern.upper() for pattern in name_patterns]

    fk_cases: list[FkCase] = []
    ik_cases: list[IkCase] = []

    for case in blob.get("test_cases", []):
        case_id = int(case.get("id", -1))
        case_name = str(case.get("name", ""))
        case_name_upper = case_name.upper()
        if not any(pattern in case_name_upper for pattern in patterns):
            continue

        target = case["target"]
        base_pose16 = _target_xyz_wpr_to_pose16(target["xyz_mm"], target["wpr_deg"])

        solutions = list(case.get("solutions", []))
        solutions.sort(key=lambda s: int(s.get("id", 0)))
        approx = [float(x) for x in solutions[0]["joints_deg"]] if solutions else None

        base_name = f"TC{case_id}-{case_name}"
        ik_cases.append(
            IkCase(
                name=f"{base_name}-EDGE_BASE",
                pose16_col_major=base_pose16,
                approx_deg=approx,
            )
        )

        px, py, pz = base_pose16[12], base_pose16[13], base_pose16[14]
        radius = math.sqrt(px * px + py * py + pz * pz)
        if radius <= 1e-12:
            ux, uy, uz = 1.0, 0.0, 0.0
        else:
            ux, uy, uz = px / radius, py / radius, pz / radius

        for delta_mm in jitter_mm:
            if abs(delta_mm) <= 1e-15:
                continue
            jittered_pose16 = _pose16_with_offset(
                base_pose16, ux * delta_mm, uy * delta_mm, uz * delta_mm
            )
            ik_cases.append(
                IkCase(
                    name=f"{base_name}-EDGE_JITTER_{delta_mm:+.3f}MM",
                    pose16_col_major=jittered_pose16,
                    approx_deg=approx,
                )
            )

        for solution in solutions:
            sol_id = int(solution.get("id", 0))
            joints = [float(x) for x in solution["joints_deg"]]
            fk_cases.append(
                FkCase(name=f"{base_name}-EDGE_SOL{sol_id}", joints_deg=joints)
            )

    return fk_cases, ik_cases


def _joint_limits_from_robot(robot: RobotT) -> list[tuple[float, float]]:
    return [(float(robot.data[30][i]), float(robot.data[31][i])) for i in range(DOFS)]


def _build_fuzz_cases(
    baseline: KinematicsLib,
    robot: RobotT,
    seed: int,
    fk_count: int = FK_FUZZ_COUNT,
    ik_count: int = IK_FUZZ_COUNT,
) -> tuple[list[FkCase], list[IkCase]]:
    rng = np.random.default_rng(seed)
    limits = _joint_limits_from_robot(robot)

    fk_cases: list[FkCase] = []
    for idx in range(fk_count):
        joints = [float(rng.uniform(lo, hi)) for (lo, hi) in limits]
        fk_cases.append(FkCase(name=f"FUZZ_FK_{idx:04d}", joints_deg=joints))

    ik_cases: list[IkCase] = []
    attempts = 0
    max_attempts = ik_count * 40
    while len(ik_cases) < ik_count and attempts < max_attempts:
        attempts += 1
        joints = [float(rng.uniform(lo, hi)) for (lo, hi) in limits]
        status, pose = baseline.solve_fk(robot, joints)
        if status == 1:
            ik_cases.append(
                IkCase(
                    name=f"FUZZ_IK_{len(ik_cases):04d}",
                    pose16_col_major=pose,
                    approx_deg=joints,
                )
            )

    if len(ik_cases) < ik_count:
        raise RuntimeError(
            f"Unable to generate enough FK-valid fuzz IK targets: {len(ik_cases)}/{ik_count}"
        )

    return fk_cases, ik_cases


def _assert_fk_parity(
    baseline: KinematicsLib,
    candidate: KinematicsLib,
    robot: RobotT,
    cases: list[FkCase],
) -> None:
    for case in cases:
        b_status, b_pose = baseline.solve_fk(robot, case.joints_deg)
        c_status, c_pose = candidate.solve_fk(robot, case.joints_deg)
        if b_status != c_status:
            raise RuntimeError(
                f"FK status mismatch on {case.name}: baseline={b_status}, candidate={c_status}"
            )
        if b_status == 1:
            max_pose_diff = max(abs(x - y) for x, y in zip(b_pose, c_pose))
            if max_pose_diff > POSE_DIFF_TOL:
                raise RuntimeError(
                    f"FK pose mismatch on {case.name}: max_abs_diff={max_pose_diff:.3e} > {POSE_DIFF_TOL:.3e}"
                )


def _assert_solution_set_parity(
    base_solutions: list[list[float]], cand_solutions: list[list[float]]
) -> None:
    remaining = list(range(len(cand_solutions)))
    for base_idx, base_solution in enumerate(base_solutions):
        best_idx = -1
        best_diff = float("inf")
        for cand_idx in remaining:
            diff = _max_abs_wrapped_diff_deg(base_solution, cand_solutions[cand_idx])
            if diff < best_diff:
                best_diff = diff
                best_idx = cand_idx
        if best_idx < 0 or best_diff > JOINT_DIFF_TOL_DEG:
            raise RuntimeError(
                "IK solution set mismatch: "
                f"base_index={base_idx} nearest_diff_deg={best_diff:.6f} tol={JOINT_DIFF_TOL_DEG}"
            )
        remaining.remove(best_idx)
    if remaining:
        raise RuntimeError(
            f"IK solution set mismatch: {len(remaining)} unmatched candidate solutions"
        )


def _assert_ik_parity(
    baseline: KinematicsLib,
    candidate: KinematicsLib,
    robot: RobotT,
    cases: list[IkCase],
) -> None:
    for case in cases:
        b_status, b_best, b_all = baseline.solve_ik(
            robot, case.pose16_col_major, case.approx_deg
        )
        c_status, c_best, c_all = candidate.solve_ik(
            robot, case.pose16_col_major, case.approx_deg
        )

        if b_status != c_status:
            raise RuntimeError(
                f"IK status mismatch on {case.name}: baseline={b_status}, candidate={c_status}"
            )
        if b_status > 0 and c_status <= 0:
            raise RuntimeError(
                f"IK no-solution regression on {case.name}: baseline={b_status}, candidate={c_status}"
            )

        if b_status > 0:
            best_diff = _max_abs_wrapped_diff_deg(b_best, c_best)
            if best_diff > JOINT_DIFF_TOL_DEG:
                raise RuntimeError(
                    f"IK best-solution mismatch on {case.name}: diff_deg={best_diff:.6f} > {JOINT_DIFF_TOL_DEG}"
                )
            _assert_solution_set_parity(b_all, c_all)


def _percentile(values: list[float], percentile: float) -> float:
    if not values:
        return float("nan")
    sorted_values = sorted(values)
    if len(sorted_values) == 1:
        return float(sorted_values[0])
    pos = (len(sorted_values) - 1) * (percentile / 100.0)
    lo = int(math.floor(pos))
    hi = int(math.ceil(pos))
    if lo == hi:
        return float(sorted_values[lo])
    weight = pos - lo
    return float(sorted_values[lo] * (1.0 - weight) + sorted_values[hi] * weight)


def _pct_speedup(baseline_ns: int, candidate_ns: int) -> float:
    if baseline_ns <= 0:
        return 0.0
    return ((baseline_ns - candidate_ns) / baseline_ns) * 100.0


def _summarize(records: list[dict[str, float]]) -> dict[str, Any]:
    baseline_ns = [float(r["baseline_ns"]) for r in records]
    candidate_ns = [float(r["candidate_ns"]) for r in records]
    speedup_pct = [float(r["speedup_pct"]) for r in records]

    return {
        "samples": len(records),
        "baseline_ns": {
            "median": statistics.median(baseline_ns),
            "mean": statistics.mean(baseline_ns),
            "p90": _percentile(baseline_ns, 90.0),
        },
        "candidate_ns": {
            "median": statistics.median(candidate_ns),
            "mean": statistics.mean(candidate_ns),
            "p90": _percentile(candidate_ns, 90.0),
        },
        "speedup_pct": {
            "median": statistics.median(speedup_pct),
            "mean": statistics.mean(speedup_pct),
            "p10": _percentile(speedup_pct, 10.0),
            "p90": _percentile(speedup_pct, 90.0),
            "min": min(speedup_pct),
            "max": max(speedup_pct),
        },
    }


def _run_ab_workload(
    baseline_runner,
    candidate_runner,
    warmup: int,
    reps: int,
) -> list[dict[str, float]]:
    records: list[dict[str, float]] = []
    total = warmup + reps
    for idx in range(total):
        if idx % 2 == 0:
            t0 = time.perf_counter_ns()
            baseline_runner()
            b_ns = time.perf_counter_ns() - t0
            t0 = time.perf_counter_ns()
            candidate_runner()
            c_ns = time.perf_counter_ns() - t0
        else:
            t0 = time.perf_counter_ns()
            candidate_runner()
            c_ns = time.perf_counter_ns() - t0
            t0 = time.perf_counter_ns()
            baseline_runner()
            b_ns = time.perf_counter_ns() - t0

        if idx >= warmup:
            records.append(
                {
                    "rep": idx - warmup,
                    "baseline_ns": float(b_ns),
                    "candidate_ns": float(c_ns),
                    "speedup_pct": _pct_speedup(b_ns, c_ns),
                }
            )
    return records


def _run_fk_batch(lib: KinematicsLib, robot: RobotT, cases: list[FkCase]) -> None:
    for case in cases:
        lib.solve_fk(robot, case.joints_deg)


def _run_ik_batch(lib: KinematicsLib, robot: RobotT, cases: list[IkCase]) -> None:
    for case in cases:
        lib.solve_ik(robot, case.pose16_col_major, case.approx_deg)


def _ns_to_ms(value_ns: float) -> float:
    return value_ns / 1_000_000.0


def _print_summary(name: str, summary: dict[str, Any]) -> None:
    b = summary["baseline_ns"]
    c = summary["candidate_ns"]
    s = summary["speedup_pct"]
    print(f"\n{name} summary")
    print(
        f"  baseline  median={_ns_to_ms(b['median']):.3f} ms  mean={_ns_to_ms(b['mean']):.3f} ms  p90={_ns_to_ms(b['p90']):.3f} ms"
    )
    print(
        f"  candidate median={_ns_to_ms(c['median']):.3f} ms  mean={_ns_to_ms(c['mean']):.3f} ms  p90={_ns_to_ms(c['p90']):.3f} ms"
    )
    print(
        f"  speedup%  median={s['median']:.2f}%  mean={s['mean']:.2f}%  p10={s['p10']:.2f}%  p90={s['p90']:.2f}%"
    )


def _parse_args() -> argparse.Namespace:
    repo_root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--baseline-dll", type=Path, required=True, help="Path to baseline DLL."
    )
    parser.add_argument(
        "--candidate-dll", type=Path, required=True, help="Path to candidate DLL."
    )
    parser.add_argument(
        "--reps",
        type=int,
        default=40,
        help="Measured repetitions per workload (default: 40).",
    )
    parser.add_argument(
        "--warmup",
        type=int,
        default=8,
        help="Warmup repetitions per workload (default: 8).",
    )
    parser.add_argument(
        "--seed", type=int, default=42, help="Deterministic fuzz seed (default: 42)."
    )
    parser.add_argument(
        "--fuzz-fk-count",
        type=int,
        default=FK_FUZZ_COUNT,
        help=f"Number of random FK fuzz cases in full mode (default: {FK_FUZZ_COUNT}).",
    )
    parser.add_argument(
        "--fuzz-ik-count",
        type=int,
        default=IK_FUZZ_COUNT,
        help=f"Number of random IK fuzz cases in full mode (default: {IK_FUZZ_COUNT}).",
    )
    parser.add_argument(
        "--edge-only",
        action="store_true",
        help="Run only edge-case workloads built from fixture names matching --edge-patterns.",
    )
    parser.add_argument(
        "--edge-patterns",
        type=str,
        default=",".join(EDGE_DEFAULT_PATTERNS),
        help="Comma-separated fixture name tokens for edge workload selection (default: MAX,MIN,SING).",
    )
    parser.add_argument(
        "--edge-jitter-mm",
        type=str,
        default=",".join(str(v) for v in EDGE_DEFAULT_JITTER_MM),
        help="Comma-separated radial pose jitters (mm) for edge IK workloads.",
    )
    parser.add_argument(
        "--json-out", type=Path, default=None, help="Optional JSON report path."
    )
    parser.add_argument(
        "--fixture-json",
        type=Path,
        default=repo_root / "tests" / "fixtures" / "CRX10iA-solutions.json",
        help="Fixture JSON path.",
    )
    return parser.parse_args()


def main() -> int:
    args = _parse_args()
    if args.fuzz_fk_count < 0 or args.fuzz_ik_count < 0:
        raise RuntimeError("--fuzz-fk-count and --fuzz-ik-count must be >= 0")

    baseline = KinematicsLib(args.baseline_dll)
    candidate = KinematicsLib(args.candidate_dll)
    robot = _build_robot()

    edge_patterns = _parse_csv_tokens(args.edge_patterns)
    edge_jitter_mm = _parse_csv_floats(args.edge_jitter_mm)
    edge_fk_cases, edge_ik_cases = _build_edge_cases(
        args.fixture_json,
        name_patterns=edge_patterns,
        jitter_mm=edge_jitter_mm,
    )

    fixture_fk_cases, fixture_ik_cases = _load_fixture_cases(args.fixture_json)
    if args.edge_only:
        fuzz_fk_cases, fuzz_ik_cases = [], []
        fk_cases = edge_fk_cases
        ik_cases = edge_ik_cases
    else:
        fuzz_fk_cases, fuzz_ik_cases = _build_fuzz_cases(
            baseline,
            robot,
            args.seed,
            fk_count=args.fuzz_fk_count,
            ik_count=args.fuzz_ik_count,
        )
        fk_cases = fixture_fk_cases + fuzz_fk_cases
        ik_cases = fixture_ik_cases + fuzz_ik_cases

    if not fk_cases or not ik_cases:
        raise RuntimeError(
            f"No benchmark cases built (fk={len(fk_cases)}, ik={len(ik_cases)}). "
            "Check --edge-patterns / fixture content."
        )

    print("Benchmark setup")
    print(f"  baseline:  {baseline.path}")
    print(f"  candidate: {candidate.path}")
    print(f"  mode:      {'edge-only' if args.edge_only else 'full'}")
    print(f"  edge patterns: {edge_patterns}")
    print(f"  edge jitter mm: {edge_jitter_mm}")
    print(f"  edge FK cases: {len(edge_fk_cases)}")
    print(f"  edge IK cases: {len(edge_ik_cases)}")
    print(f"  fixture FK cases: {len(fixture_fk_cases)}")
    print(f"  fixture IK cases: {len(fixture_ik_cases)}")
    print(f"  fuzz FK cases:    {len(fuzz_fk_cases)}")
    print(f"  fuzz IK cases:    {len(fuzz_ik_cases)}")
    print(f"  total FK cases:   {len(fk_cases)}")
    print(f"  total IK cases:   {len(ik_cases)}")
    print(f"  warmup/reps:      {args.warmup}/{args.reps}")
    print(f"  seed:             {args.seed}")
    print(f"  fuzz FK/IK count: {args.fuzz_fk_count}/{args.fuzz_ik_count}")

    print("\nRunning differential correctness checks...")
    _assert_fk_parity(baseline, candidate, robot, fk_cases)
    _assert_ik_parity(baseline, candidate, robot, ik_cases)
    print("  parity: PASS")

    fk_records = _run_ab_workload(
        baseline_runner=lambda: _run_fk_batch(baseline, robot, fk_cases),
        candidate_runner=lambda: _run_fk_batch(candidate, robot, fk_cases),
        warmup=args.warmup,
        reps=args.reps,
    )
    ik_records = _run_ab_workload(
        baseline_runner=lambda: _run_ik_batch(baseline, robot, ik_cases),
        candidate_runner=lambda: _run_ik_batch(candidate, robot, ik_cases),
        warmup=args.warmup,
        reps=args.reps,
    )

    fk_summary = _summarize(fk_records)
    ik_summary = _summarize(ik_records)
    _print_summary("FK", fk_summary)
    _print_summary("IK", ik_summary)

    ik_gate = ik_summary["speedup_pct"]["median"] >= 3.0
    fk_gate = fk_summary["speedup_pct"]["median"] >= -1.0
    overall = ik_gate and fk_gate
    print("\nDecision gates")
    print(f"  IK median speedup >= 3.0%:  {'PASS' if ik_gate else 'FAIL'}")
    print(f"  FK median slowdown <= 1.0%: {'PASS' if fk_gate else 'FAIL'}")
    print(f"  overall: {'PASS' if overall else 'FAIL'}")

    report = {
        "timestamp_epoch_s": time.time(),
        "baseline_dll": str(baseline.path),
        "candidate_dll": str(candidate.path),
        "mode": "edge-only" if args.edge_only else "full",
        "seed": args.seed,
        "fuzz_fk_count": args.fuzz_fk_count,
        "fuzz_ik_count": args.fuzz_ik_count,
        "warmup": args.warmup,
        "reps": args.reps,
        "edge_selection": {
            "patterns": edge_patterns,
            "jitter_mm": edge_jitter_mm,
            "edge_fk": len(edge_fk_cases),
            "edge_ik": len(edge_ik_cases),
        },
        "workload_sizes": {
            "fixture_fk": len(fixture_fk_cases),
            "fixture_ik": len(fixture_ik_cases),
            "fuzz_fk": len(fuzz_fk_cases),
            "fuzz_ik": len(fuzz_ik_cases),
            "total_fk": len(fk_cases),
            "total_ik": len(ik_cases),
        },
        "parity": {
            "pass": True,
            "pose_diff_tol": POSE_DIFF_TOL,
            "joint_diff_tol_deg": JOINT_DIFF_TOL_DEG,
        },
        "fk": {"summary": fk_summary, "records": fk_records},
        "ik": {"summary": ik_summary, "records": ik_records},
        "gates": {
            "ik_median_speedup_ge_3pct": ik_gate,
            "fk_median_slowdown_not_worse_than_1pct": fk_gate,
            "overall_pass": overall,
        },
    }

    if args.json_out is not None:
        args.json_out.parent.mkdir(parents=True, exist_ok=True)
        args.json_out.write_text(json.dumps(report, indent=2), encoding="utf-8")
        print(f"\nWrote JSON report: {args.json_out.resolve()}")

    return 0 if overall else 3


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except RuntimeError as exc:
        print(f"\nERROR: {exc}", file=sys.stderr)
        raise SystemExit(2)
