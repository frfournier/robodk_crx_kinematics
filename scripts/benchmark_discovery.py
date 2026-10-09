"""Reproducible production-DLL versus polynomial-component timing snapshot.

Build crx_discovery_benchmark with a Release CMake preset first. This script
generates inputs and checks results; all timing takes place inside native loops.
"""

import argparse
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import platform
import subprocess
import sys

import numpy as np
import pandas as pd

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tests"))
from crx_asset_reference import canonical_forward, derive_model, rigid_inverse
from crx_reference import translation, xyzwpr


def sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def make_corpus(random_count, stress_count, seed):
    capture = json.loads((ROOT / "tests/fixtures/crx_asset_frames.json").read_text())
    models, cases = [], []
    rng = np.random.default_rng(seed)
    for asset in capture["assets"]:
        model = derive_model(asset)
        dh = model["dh"]
        robot = np.zeros((32, 20))
        robot[1, 1] = 6
        for key, row in (("base", 9), ("tool", 28)):
            np.testing.assert_allclose(model[key][:3, :3], np.eye(3), atol=1e-12, rtol=0)
            robot[row, :3] = model[key][:3, 3]
        robot[3, 4:10] = np.rint(np.diag(model["motion_map"]))
        robot[10:16, :4] = dh
        robot[30, :6], robot[31, :6] = asset["lower_limits_deg"], asset["upper_limits_deg"]
        models.append({
            "name": asset["asset"], "robot": robot,
            "lengths": [dh[2, 1], dh[3, 3], dh[5, 3], -dh[4, 3]],
            "base": model["base"] @ translation(0, 0, dh[0, 3]),
            "tool": model["tool"], "asset_sha256": asset["sha256"],
        })

    def add(model_id, name, group, target, witnesses):
        model = models[model_id]
        canonical = rigid_inverse(model["base"]) @ target @ rigid_inverse(model["tool"])
        cases.append({"model": model_id, "name": name, "group": group,
                      "target": target.tolist(), "canonical": canonical.tolist(),
                      "witnesses": [list(q) for q in witnesses]})

    ten = next(i for i, m in enumerate(models) if m["name"] == "Fanuc-CRX-10iA-Custom.robot")
    fixtures = json.loads((ROOT / "tests/fixtures/CRX10iA-solutions.json").read_text())
    for case in fixtures["test_cases"]:
        target = xyzwpr([*case["target"]["xyz_mm"], *np.radians(case["target"]["wpr_deg"])])
        witnesses = []
        for solution in case["solutions"]:
            q = np.radians(solution["joints_deg"])
            q[2] += q[1]
            witnesses.append(q)
        add(ten, case["name"], "fixtures", target, witnesses)

    for model_id, model in enumerate(models):
        lower, upper = model["robot"][30, :6], model["robot"][31, :6]
        for group, count in (("random", random_count), ("stress", stress_count)):
            for index in range(count):
                # Sample the intersection of command J3 and the existing core's
                # decoupled J3 limits; do not benchmark knowingly illegal seeds.
                while True:
                    command = rng.uniform(lower + 2, upper - 2)
                    lo = max(lower[2], lower[2] + command[1]) + 2
                    hi = min(upper[2], upper[2] + command[1]) - 2
                    if lo >= hi:
                        continue
                    command[2] = rng.uniform(lo, hi)
                    if group == "stress":
                        if index % 2 == 0:
                            command[4] = (0, -1e-6, 1e-6)[(index // 2) % 3]
                        else:
                            command[2] = 90 + (0, -1e-6, 1e-6)[(index // 2) % 3]
                    decoupled = command.copy()
                    decoupled[2] -= command[1]
                    if np.all(command >= lower) and np.all(command <= upper) and np.all(decoupled >= lower) and np.all(decoupled <= upper):
                        break
                q = np.radians(command)
                target = model["base"] @ canonical_forward(model["lengths"], q) @ model["tool"]
                add(model_id, f"{model['name']}:{group}:{index}", group, target, [q])

    for index in range(64):
        q = np.radians(np.random.default_rng(index).uniform(-150, 150, size=6))
        q[2] += q[1]
        model = models[ten]
        target = model["base"] @ canonical_forward(model["lengths"], q) @ model["tool"]
        add(ten, f"legacy:{index}", "legacy-witnesses", target, [q])
    return models, cases


def summarize(output, models, cases, metadata):
    timings = pd.read_csv(output / "timings.csv")
    solutions = pd.read_csv(output / "solutions.csv")
    medians = timings.groupby(["case", "path"])["us"].median().unstack()
    # Every repeat must have the same functional outcome.
    for field in ("status", "count"):
        assert timings.groupby(["case", "path"])[field].nunique().max() == 1
    outcomes = timings[timings["round"] == 0].set_index(["case", "path"])
    details = []
    for index, case in enumerate(cases):
        item = {"case": index, "name": case["name"], "group": case["group"],
                "asset": models[case["model"]]["name"], "witnesses": len(case["witnesses"])}
        target = np.asarray(case["canonical"])
        for path in ("production", "polynomial"):
            rows = solutions[(solutions["case"] == index) & (solutions["path"] == path)]
            joints = rows[[f"q{i}" for i in range(1, 7)]].to_numpy()
            invalid, max_pos, max_ang = 0, 0.0, 0.0
            for q in joints:
                pose = canonical_forward(models[case["model"]]["lengths"], q)
                pos = float(np.linalg.norm(pose[:3, 3] - target[:3, 3]))
                angle = float(2 * np.arcsin(min(1, np.linalg.norm(pose[:3, :3] - target[:3, :3]) / (2*np.sqrt(2)))))
                invalid += int(pos > 1e-4 + 1e-8 or angle > np.radians(1e-3) + 1e-10)
                max_pos, max_ang = max(max_pos, pos), max(max_ang, angle)
            hits = sum(any(np.max(np.abs((q - w + np.pi) % (2*np.pi) - np.pi)) <= np.radians(0.009)
                           for q in joints) for w in case["witnesses"])
            item.update({f"{path}_us": medians.loc[index, path],
                         f"{path}_status": int(outcomes.loc[(index, path), "status"]),
                         f"{path}_count": len(joints), f"{path}_witness_hits": hits,
                         f"{path}_invalid": invalid, f"{path}_max_position_mm": max_pos,
                         f"{path}_max_angle_rad": max_ang})
        details.append(item)
    frame = pd.DataFrame(details)
    frame.to_csv(output / "per-pose.csv", index=False)
    groups = {}
    for group in ("fixtures", "random", "stress", "legacy-witnesses"):
        subset = frame[frame["group"] == group]
        common = subset[(subset.production_status == 0) & (subset.polynomial_status == 0)]
        stats = {"poses": len(subset), "common_success": len(common)}
        for path in ("production", "polynomial"):
            stats[path] = {
                "median_us": float(subset[f"{path}_us"].median()),
                "p95_us": float(subset[f"{path}_us"].quantile(.95)),
                "p99_us": float(subset[f"{path}_us"].quantile(.99)),
                "mean_us": float(subset[f"{path}_us"].mean()),
                "status_counts": {str(k): int(v) for k, v in subset[f"{path}_status"].value_counts().items()},
                "witness_hits": int(subset[f"{path}_witness_hits"].sum()),
                "witness_total": int(subset.witnesses.sum()),
                "invalid_solutions": int(subset[f"{path}_invalid"].sum()),
            }
        stats["common_success_speedup"] = float(common.production_us.mean() / common.polynomial_us.mean()) if len(common) else None
        if len(common):
            paired = timings[timings["case"].isin(common["case"])]
            paired = paired.groupby(["round", "path"])["us"].mean().unstack()
            ratios = paired.production / paired.polynomial
            stats["common_success_speedup_round_range"] = [float(ratios.min()), float(ratios.max())]
        fallback = subset.polynomial_us + np.where(subset.polynomial_status != 0, subset.production_us, 0)
        stats["estimated_fallback_mean_us"] = float(fallback.mean())
        stats["estimated_fallback_speedup"] = float(subset.production_us.mean() / fallback.mean())
        groups[group] = stats
    report = {"metadata": metadata, "groups": groups}
    (output / "summary.json").write_text(json.dumps(report, indent=2) + "\n")
    lines = [
        "# Production versus polynomial discovery: performance snapshot", "",
        f"Measured at {metadata['measured_at_utc']}. CPU: {metadata['cpu']['Name']}.",
        "Windows x64, Visual Studio clang-cl Release `/O2 /Ob2 /DNDEBUG`, Eigen allocation instrumentation disabled.",
        f"{len(cases)} poses; {metadata['rounds']} rounds of {metadata['repeats']} calls per pose/path; three warmup calls.",
        "Native timing loops, one thread pinned to the first allowed logical CPU. Shuffled pose order each round; alternating implementation order.",
        "No seed. Both use 0.0001 mm / 0.001 degree acceptance tolerances. Production uses its actual DLL with capacity 64.", "",
        "**Different scope:** production includes the RoboDK adapter, command limits, turns and result packing. Polynomial measures canonical discovery only, with no fallback or command handling. Refinement timings are unfinished work, not solved poses.", "",
        "Numbers below are median per-pose batch times; p95 describes variation across poses, not individual-call tail latency.", "",
        "| Corpus | Poses | Production p50 / p95 (us) | Polynomial p50 / p95 (us) | Polynomial candidates / handoffs | Speedup on common successes |",
        "|---|---:|---:|---:|---:|---:|",
    ]
    for name, stats in groups.items():
        p, n = stats["production"], stats["polynomial"]
        statuses = n["status_counts"]
        speedup = stats["common_success_speedup"]
        speedup_text = f"{speedup:.2f}x" if speedup is not None else "n/a"
        lines.append(f"| {name} | {stats['poses']} | {p['median_us']:.2f} / {p['p95_us']:.2f} | {n['median_us']:.2f} / {n['p95_us']:.2f} | {statuses.get('0', 0)} / {statuses.get('2', 0)} | {speedup_text} ({stats['common_success']} poses) |")
    lines += ["", "Speedup is the ratio of arithmetic mean per-pose times on the same subset returning candidates in both paths; solution lists may differ due to limits/turns.", "",
              "## Result checks", "", "Every returned posture was checked with the independent Python canonical FK. Witness matching uses 0.009 degrees modulo full turns; singular families can return a different valid representative.", "",
              "| Corpus | Production witness hits | Polynomial witness hits | Invalid FK results (production / polynomial) | Estimated polynomial + fallback speedup |",
              "|---|---:|---:|---:|---:|"]
    for name, stats in groups.items():
        p, n = stats["production"], stats["polynomial"]
        lines.append(f"| {name} | {p['witness_hits']}/{p['witness_total']} | {n['witness_hits']}/{n['witness_total']} | {p['invalid_solutions']} / {n['invalid_solutions']} | {stats['estimated_fallback_speedup']:.2f}x |")
    lines += ["", "Fallback column is a cost estimate: polynomial time plus production time whenever polynomial did not return candidates. It is **not** a measured integrated solver or correctness guarantee; command handling costs are missing.", "",
              "Random poses are uniform in the six assets' joint ranges, with a 2-degree margin and both coupled/decoupled J3 limits satisfied. This is joint-space sampling, not a uniform Cartesian workspace distribution. Stress cases alternate zero/near-zero wrist sine and straight/near-straight elbows. Legacy witnesses are the separate existing 64-case regression corpus.", "",
              "## Fixture details", "", "| Fixture | Production us / solutions | Polynomial us / solutions | Polynomial outcome |", "|---|---:|---:|---|"]
    labels = {0: "Candidates", 1: "NoCandidate", 2: "NeedsRefinement", 3: "InvalidInput", 4: "NumericalRangeFailure"}
    for row in frame[frame["group"] == "fixtures"].itertuples():
        lines.append(f"| {row.name} | {row.production_us:.2f} / {row.production_count} | {row.polynomial_us:.2f} / {row.polynomial_count} | {labels[row.polynomial_status]} |")
    lines += ["", "## Reproduction and provenance", "", "```powershell", "cmake --build --preset windows-clang-release-tidy --target crx_discovery_benchmark", "uv run python scripts/benchmark_discovery.py", "```", "",
              f"Base commit: `{metadata['git_head']}` with uncommitted implementation changes. `summary.json` records source, DLL, executable, corpus and asset hashes plus compiler commands.",
              "Raw data: `timings.csv` (all rounds), `solutions.csv` (returned postures), `per-pose.csv` (latencies, outcomes and residuals), `corpus.json`, `native.log`. Snapshot does not measure RoboDK IPC, cold-start latency or deployment behavior."]
    (output / "report.md").write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(json.dumps(groups, indent=2))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=ROOT / "build/performance-snapshot")
    parser.add_argument("--random-per-model", type=int, default=128)
    parser.add_argument("--stress-per-model", type=int, default=16)
    parser.add_argument("--seed", type=int, default=20261009)
    parser.add_argument("--repeats", type=int, default=8)
    parser.add_argument("--rounds", type=int, default=9)
    args = parser.parse_args()
    if min(args.random_per_model, args.stress_per_model, args.repeats, args.rounds) < 1:
        parser.error("counts must be positive")
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    executable = ROOT / "build/Release/crx_discovery_benchmark.exe"
    dll = ROOT / "build/Release/crx_kinematics.dll"
    models, cases = make_corpus(args.random_per_model, args.stress_per_model, args.seed)
    (output / "corpus.json").write_text(json.dumps(cases, indent=2) + "\n")
    values = [len(models), len(cases)]
    for model in models:
        values.extend(model["robot"].flat)
    for case in cases:
        values.extend([case["model"], *models[case["model"]]["lengths"],
                       *np.asarray(case["canonical"]).flat, *np.asarray(case["target"]).ravel(order="F")])
    (output / "input.txt").write_text(" ".join(format(v, ".17g") for v in values))
    source_paths = [*sorted((ROOT / "src").glob("*.cpp")), *sorted((ROOT / "src").glob("*.h")),
                    *sorted((ROOT / "include").glob("*.h")), Path(__file__),
                    ROOT / "scripts/benchmark_discovery.cpp", ROOT / "CMakeLists.txt",
                    ROOT / "CMakePresets.json", ROOT / "uv.lock",
                    ROOT / "tests/crx_asset_reference.py", ROOT / "tests/crx_reference.py",
                    ROOT / "tests/fixtures/crx_asset_frames.json", ROOT / "tests/fixtures/CRX10iA-solutions.json"]
    compile_commands = json.loads((ROOT / "build/vscode/windows-clang-release-tidy/compile_commands.json").read_text())
    cpu = {"Name": platform.processor(), "NumberOfLogicalProcessors": os.cpu_count()}
    try:
        import winreg
        with winreg.OpenKey(winreg.HKEY_LOCAL_MACHINE,
                            r"HARDWARE\DESCRIPTION\System\CentralProcessor\0") as key:
            cpu["Name"] = winreg.QueryValueEx(key, "ProcessorNameString")[0].strip()
    except OSError:
        pass
    metadata = {
        "measured_at_utc": datetime.now(timezone.utc).isoformat(), "cpu": cpu,
        "compile_commands": [entry for entry in compile_commands
                             if "crx_discovery.cpp" in entry["file"] or "benchmark_discovery.cpp" in entry["file"]],
        "platform": platform.platform(), "processor": platform.processor(),
        "git_head": subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=ROOT, text=True).strip(),
        "git_status": subprocess.check_output(["git", "status", "--short"], cwd=ROOT, text=True),
        "seed": args.seed, "repeats": args.repeats, "rounds": args.rounds,
        "warmup_calls_per_pose_per_path": 3, "pose_count": len(cases),
        "position_tolerance_mm": 1e-4, "orientation_tolerance_deg": 1e-3,
        "production_seed": None, "production_capacity": 64,
        "dll_sha256": sha256(dll), "benchmark_sha256": sha256(executable),
        "corpus_sha256": sha256(output / "corpus.json"),
        "source_sha256": {str(p.relative_to(ROOT)): sha256(p) for p in source_paths},
        "assets": {m["name"]: m["asset_sha256"] for m in models},
        "timing": "Per-pose median of round batch means; p95/p99 across poses, not individual-call tails",
        "scope": "Production DLL includes adapter/limits/turns/packing; polynomial is canonical discovery with no fallback or command semantics",
    }
    with (output / "native.log").open("w") as log:
        subprocess.run([str(executable), str(dll), str(output / "input.txt"),
                        str(output / "timings.csv"), str(output / "solutions.csv"),
                        str(args.repeats), str(args.rounds)], check=True, stderr=log)
    summarize(output, models, cases, metadata)


if __name__ == "__main__":
    main()
