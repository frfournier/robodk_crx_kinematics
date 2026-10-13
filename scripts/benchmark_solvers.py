"""Snapshot complete scanner/polynomial DLL speed, accuracy and reliability.

Native timing excludes Python and validation. The independent checker uses
captured model transforms, not either DLL's FK or internal acceptance result.
"""
import argparse
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import platform
import shutil
import subprocess
import sys
import zipfile

import numpy as np
import pandas as pd

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tests"))
from fixtures.generate_solver_cases import (
    corpus, CHALLENGES, OFFSETS_RAD, DEFAULT_RANDOM_PER_MODEL,
    DEFAULT_STRESS_PER_GROUP_MODEL, DEFAULT_SEED,
)
from crx_asset_reference import forward_models as reference_poses
POSITION_MM, ANGLE_DEG = 1e-4, 1e-3  # Public strict-pose contract.
EPS = np.finfo(float).eps
WITNESS_DEG = .009  # Existing regression posture-matching convention.
PATHS, MODES = ("legacy", "polynomial"), ("no_seed", "nearby_seed")


def sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def validate(output, models, references, cases, rounds):
    timings = pd.read_csv(output / "timings.csv")
    solutions = pd.read_csv(output / "solutions.csv")
    assert len(timings) == len(cases) * len(MODES) * len(PATHS) * rounds
    assert timings.groupby(["case", "mode", "path"]).size().eq(rounds).all()
    assert timings.us.gt(0).all() and np.isfinite(timings.us).all()
    assert timings.groupby(["case", "mode", "path"])["count"].nunique().max() == 1
    targets = np.array([item["target"] for item in cases])
    model_ids = np.array([item["model"] for item in cases])
    solutions["position_mm"] = np.nan
    solutions["angle_deg"] = np.nan
    solutions["limits_ok"] = False
    columns = [f"q{i}" for i in range(1, 7)]
    for model_id, (model, reference) in enumerate(zip(models, references)):
        selected = model_ids[solutions["case"].to_numpy()] == model_id
        commands = solutions.loc[selected, columns].to_numpy()
        actual = reference_poses(reference, commands)
        target = targets[solutions.loc[selected, "case"].to_numpy()]
        position = np.linalg.norm(actual[:, :3, 3] - target[:, :3, 3], axis=1)
        frobenius = np.linalg.norm(actual[:, :3, :3] - target[:, :3, :3], axis=(1, 2))
        angle = np.degrees(2*np.arcsin(np.minimum(1, frobenius/(2*np.sqrt(2)))))
        low, high = model["robot"][[30, 31], :6]
        # Unit-aware binary64 allowance for degree conversion at a command bound.
        allowance = 128 * EPS * np.maximum(1, np.maximum(abs(low), abs(high)))
        limits_ok = ((commands >= low - allowance) & (commands <= high + allowance)).all(axis=1)
        solutions.loc[selected, "position_mm"] = position
        solutions.loc[selected, "angle_deg"] = angle
        solutions.loc[selected, "limits_ok"] = limits_ok
    solutions["valid"] = ((solutions.position_mm <= POSITION_MM)
                          & (solutions.angle_deg <= ANGLE_DEG)
                          & solutions.limits_ok)
    solutions.to_csv(output / "errors.csv", index=False)
    grouped = {key: rows for key, rows in solutions.groupby(["case", "mode", "path"])}
    rows = []
    for (index, mode, path), times in timings.groupby(["case", "mode", "path"]):
        item = cases[index]
        returned = grouped.get((index, mode, path), solutions.iloc[:0])
        all_rows = returned[returned.solution >= 0]
        best = returned[returned.solution < 0]
        count = int(times["count"].iloc[0])
        assert len(all_rows) == max(count, 0)
        valid = all_rows[all_rows.valid]
        q = valid[columns].to_numpy()
        hits = 0
        for witness in item["witnesses"]:
            distance = abs((q - np.degrees(witness) + 180) % 360 - 180)
            hits += int(len(q) > 0 and np.any(np.max(distance, axis=1) <= WITNESS_DEG))
        solved = count > 0 and bool(returned.valid.all()) and len(best) == 1
        rows.append(dict(case=index, mode=mode, path=path, name=item["name"],
                         asset=models[item["model"]]["name"], group=item["group"],
                         reachable=item["reachable"], offset_rad=item["offset_rad"],
                         count=count, solved=solved, median_us=float(times.us.median()),
                         invalid_solutions=int((~all_rows.valid).sum()),
                         best_valid=bool(len(best) and best.valid.iloc[0]),
                         witness_hits=hits, witnesses=len(item["witnesses"])))
    frame = pd.DataFrame(rows)
    frame.to_csv(output / "per-pose.csv", index=False)
    stress = frame[frame.group.isin(CHALLENGES)]
    stress.groupby(["mode", "path", "group", "offset_rad"]).agg(
        poses=("solved", "size"), solves=("solved", "sum"),
        solve_fraction=("solved", "mean")).to_csv(output / "stress-offsets.csv")
    frame[(frame.reachable & ~frame.solved) | (frame.invalid_solutions > 0)
          | (~frame.reachable & (frame["count"] > 0))].to_csv(output / "failures.csv", index=False)
    return timings, solutions, frame


def statistics(timings, errors, frame, capacity):
    def percentiles(values):
        return {label: float(values.quantile(fraction)) if len(values) else None
                for label, fraction in (("p50", .5), ("p95", .95), ("p99", .99), ("max", 1))}

    results = {}
    groups = ["all_reachable", "challenging", *frame.group.unique()]
    for mode in MODES:
        mode_results = {}
        for group in groups:
            subset = frame[(frame["mode"] == mode) &
                           (frame.reachable if group == "all_reachable" else
                            frame.group.isin(CHALLENGES) if group == "challenging" else
                            frame.group.eq(group))]
            common = subset.pivot(index="case", columns="path", values="solved").all(axis=1)
            common_ids = common[common].index
            stats = {"poses": int(subset["case"].nunique()), "common_valid_poses": len(common_ids)}
            for path in PATHS:
                data = subset[subset.path == path]
                err = errors[(errors["mode"] == mode) & (errors.path == path)
                             & errors["case"].isin(data["case"]) & (errors.solution >= 0)]
                best = errors[(errors["mode"] == mode) & (errors.path == path)
                              & errors["case"].isin(data["case"]) & (errors.solution < 0)]
                stats[path] = dict(
                    timing_us=percentiles(data.median_us), mean_us=float(data.median_us.mean()),
                    strict_solves=int(data.solved.sum()), strict_solve_pct=float(100*data.solved.mean()),
                    positive_returns=int((data["count"] > 0).sum()),
                    empty_returns=int((data["count"] == 0).sum()),
                    fallback_returns=int((data["count"] < 0).sum()),
                    capacity_returns=int((data["count"] >= capacity).sum()),
                    returned_solutions=int(len(err)), invalid_solutions=int((~err.valid).sum()),
                    position_mm=percentiles(err.position_mm), angle_deg=percentiles(err.angle_deg),
                    best_position_mm=percentiles(best.position_mm), best_angle_deg=percentiles(best.angle_deg),
                    common_position_mm=percentiles(err[err["case"].isin(common_ids)].position_mm),
                    common_angle_deg=percentiles(err[err["case"].isin(common_ids)].angle_deg),
                    witness_hits=int(data.witness_hits.sum()), witness_total=int(data.witnesses.sum()))
            if len(common_ids):
                common_times = subset[subset["case"].isin(common_ids)].groupby("path").median_us.mean()
                rounds = timings[(timings["mode"] == mode) & timings["case"].isin(common_ids)]
                rounds = rounds.groupby(["round", "path"]).us.mean().unstack()
                ratio = rounds.legacy / rounds.polynomial
                stats["common_speedup"] = float(common_times.legacy / common_times.polynomial)
                stats["speedup_round_range"] = [float(ratio.min()), float(ratio.max())]
            mode_results[group] = stats
        results[mode] = mode_results
    return results


def report(output, metadata, stats, frame):
    lines = ["# Complete IK solver benchmark snapshot", "",
             f"Measured {metadata['measured_at_utc']} on {metadata['cpu']}; Windows x64 clang-cl Release.",
             "Legacy = current 960-interval scanner DLL. New = integrated polynomial comparison DLL.",
             "Both include model adaptation, discovery, command limits/turns, FK acceptance, ranking and result packing.",
             "Neither calls the other as fallback. Released v0.3.x binaries are not the baseline.", "",
             f"{metadata['pose_count']} poses, six captured assets; capacity {metadata['capacity']}; "
             f"{metadata['rounds']} rounds × {metadata['repeats']} native calls per pose/path/mode, plus three warmups.",
             "One thread pinned to the first allowed logical CPU; shuffled pose/mode order and alternating DLL order.",
             "Times are per-pose medians of batch means. p95 measures variation across poses, not individual-call tail latency.", "",
             "## Unseeded speed and reliability", "",
             "A strict solve requires a positive return, a valid best command, and every returned command within joint limits and "
             "0.0001 mm / 0.001 degree of the target under independent transform-product FK. Empty/fallback returns count as failures on reachable poses.", "",
             "| Corpus | Poses | Legacy p50 / p95 µs | New p50 / p95 µs | Legacy strict solve | New strict solve | Common-success speedup |",
             "|---|---:|---:|---:|---:|---:|---:|"]
    for group, row in stats["no_seed"].items():
        if group == "unreachable":
            continue
        old, new = row["legacy"], row["polynomial"]
        speed = f"{row['common_speedup']:.2f}×" if "common_speedup" in row else "n/a"
        lines.append(f"| {group} | {row['poses']} | {old['timing_us']['p50']:.2f} / {old['timing_us']['p95']:.2f} | "
                     f"{new['timing_us']['p50']:.2f} / {new['timing_us']['p95']:.2f} | "
                     f"{old['strict_solves']}/{row['poses']} ({old['strict_solve_pct']:.2f}%) | "
                     f"{new['strict_solves']}/{row['poses']} ({new['strict_solve_pct']:.2f}%) | {speed} |")
    lines += ["", "Speedup is the ratio of mean per-pose times on targets strictly solved by both. "
              "Fast fallback/empty responses do not improve that ratio. Overall rows include the separately listed subsets.", "",
              "## Accuracy of returned commands", "",
              "All reachable unseeded outputs; this is conditional on returning a solution. Different solvers can return different numbers/postures. "
              "The JSON also records matched-common-target statistics and best-output errors. Rotation error uses the stable Frobenius/sine formula, not acos near one.", "",
              "| Solver | Commands | Invalid | Position p50 / p95 / max (mm) | Rotation p50 / p95 / max (deg) |",
              "|---|---:|---:|---:|---:|"]
    for path in PATHS:
        row = stats["no_seed"]["all_reachable"][path]
        pos, ang = row["position_mm"], row["angle_deg"]
        lines.append(f"| {path} | {row['returned_solutions']} | {row['invalid_solutions']} | "
                     f"{pos['p50']:.3g} / {pos['p95']:.3g} / {pos['max']:.3g} | "
                     f"{ang['p50']:.3g} / {ang['p95']:.3g} / {ang['max']:.3g} |")
    lines += ["", "## Challenging poses with a nearby seed", "",
              "Each seed is the generating/legal witness perturbed independently by up to ±1 degree per axis, clipped to command limits. "
              "It is not an exact-solution seed. Strict validation still applies, so approximate-seed acceptance cannot inflate this score.", "",
              "| Corpus | Legacy strict solve | New strict solve | Common-success speedup |",
              "|---|---:|---:|---:|"]
    for group in ("random", "challenging", *CHALLENGES):
        row = stats["nearby_seed"][group]
        speed = f"{row['common_speedup']:.2f}×" if "common_speedup" in row else "n/a"
        lines.append(f"| {group} | {row['legacy']['strict_solve_pct']:.2f}% | "
                     f"{row['polynomial']['strict_solve_pct']:.2f}% | {speed} |")
    lines += ["", "## Reliability by asset (unseeded challenges)", "",
              "| Asset | Poses | Legacy strict solve | New strict solve |", "|---|---:|---:|---:|"]
    stress = frame[(frame["mode"] == "no_seed") & frame.group.isin(CHALLENGES)]
    for asset, subset in stress.groupby("asset"):
        rates = subset.groupby("path").solved.mean()*100
        lines.append(f"| {asset} | {subset['case'].nunique()} | {rates.legacy:.2f}% | {rates.polynomial:.2f}% |")
    lines += ["", "## Controls, branch coverage and limitations", ""]
    for mode in MODES:
        control = stats[mode]["unreachable"]
        total = stats[mode]["all_reachable"]
        lines.append(f"- {mode}: unreachable controls {control['poses']}; positive returns "
                     f"legacy/new {control['legacy']['positive_returns']}/{control['polynomial']['positive_returns']}. "
                     f"Capacity-limit returns on reachable targets: {total['legacy']['capacity_returns']}/{total['polynomial']['capacity_returns']}.")
        lines.append(f"- {mode}: supplied legal witness recovery, modulo turns within {WITNESS_DEG} degree: "
                     f"legacy {total['legacy']['witness_hits']}/{total['legacy']['witness_total']}; "
                     f"new {total['polynomial']['witness_hits']}/{total['polynomial']['witness_total']}. "
                     "A singular family can have a different valid representative; witness recovery is not solve rate or exhaustive branch completeness.")
    common = stats["no_seed"]["challenging"]
    if "common_speedup" in common:
        lo, hi = common["speedup_round_range"]
        lines.append(f"- Unseeded challenging common-success speedup across rounds: {lo:.2f}–{hi:.2f}× "
                     f"on {common['common_valid_poses']} common targets (range, not a confidence interval).")
    lines += ["", "Generated targets use independently derived modified-DH transforms from the six captured assets. "
              "Random joints have a 2-degree margin; generated commands satisfy both command limits and the legacy decoupled-J3 domain. "
              "Stress cases use wrist zero, extended/folded elbow, base-axis radial cancellation, combined folded/base/wrist degeneracy, "
              "and each command limit, with exact and signed offsets of 1e-12 through .01 radians. "
              "See corpus.json for every offset. Fixtures retain their recorded Cartesian targets; their reachability is observational. "
              "This deliberately balanced stress mix is not a measured production workload or uniform Cartesian sampling.", "",
              "No generic RoboDK fallback, IPC, cold-start or deployment timing. Shared-machine frequency/load variation is not controlled; "
              "paired rounds mitigate order effects. A sample solve rate is not a completeness proof or a guarantee at every singularity.", "",
              "## Reproduction", "", "```powershell",
              "cmake --preset windows-clang-release-tidy",
              "cmake --build --preset windows-clang-release-tidy --target crx_solver_benchmark",
              f"uv run python scripts/benchmark_solvers.py --output {output.relative_to(ROOT).as_posix()} "
              f"--random-per-model {metadata['random_per_model']} --stress-per-group-model {metadata['stress_per_group_model']} "
              f"--repeats {metadata['repeats']} --rounds {metadata['rounds']} --seed {metadata['seed']} --capacity {metadata['capacity']}",
              "```", "", "Choose a new output directory for a repeat; existing snapshots are not overwritten. "
              "summary.json records source/DLL/executable/asset/corpus hashes, git state, CPU and compile commands. "
              "Raw artifacts: timings.csv, solutions.csv, errors.csv, per-pose.csv, failures.csv, stress-offsets.csv, input.txt, corpus.json and native.log. "
              "sources.zip preserves the measured project sources and reference inputs; binaries/ preserves the measured DLLs and runner. "
              "The Eigen header tree is identified by its content hash."]
    (output / "report.md").write_text("\n".join(lines) + "\n", encoding="utf-8")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=ROOT / "build/solver-performance-snapshot-20261009")
    parser.add_argument("--random-per-model", type=int, default=DEFAULT_RANDOM_PER_MODEL)
    parser.add_argument("--stress-per-group-model", type=int, default=DEFAULT_STRESS_PER_GROUP_MODEL)
    parser.add_argument("--repeats", type=int, default=8)
    parser.add_argument("--rounds", type=int, default=9)
    parser.add_argument("--seed", type=int, default=DEFAULT_SEED)
    parser.add_argument("--capacity", type=int, default=64)
    args = parser.parse_args()
    if min(args.random_per_model, args.stress_per_group_model, args.repeats, args.rounds, args.capacity) < 1:
        parser.error("counts must be positive")
    output = args.output.resolve()
    if output.exists() and any(output.iterdir()):
        parser.error("snapshot directory must be empty/new")
    output.mkdir(parents=True, exist_ok=True)
    models, references, cases = corpus(args.random_per_model, args.stress_per_group_model, args.seed)
    (output / "corpus.json").write_text(json.dumps(cases, indent=2) + "\n")
    values = [len(models), len(cases)]
    for model in models:
        values.extend(model["robot"].flat)
    for item in cases:
        values.extend([item["model"], *np.asarray(item["target"]).ravel(order="F"), *item["seed_deg"]])
    (output / "input.txt").write_text(" ".join(format(value, ".17g") for value in values))
    executable = ROOT / "build/Release/crx_solver_benchmark.exe"
    dlls = [ROOT / "build/Release" / name for name in
            ("crx_kinematics.dll", "crx_kinematics_polynomial.dll")]
    binary_dir = output / "binaries"
    binary_dir.mkdir()
    for binary in [executable, *dlls]:
        shutil.copy2(binary, binary_dir / binary.name)
    executable = binary_dir / executable.name
    dlls = [binary_dir / dll.name for dll in dlls]
    compile_commands = json.loads((ROOT / "build/vscode/windows-clang-release-tidy/compile_commands.json").read_text())
    cpu = platform.processor()
    if os.name == "nt":
        import winreg
        with winreg.OpenKey(winreg.HKEY_LOCAL_MACHINE, r"HARDWARE\DESCRIPTION\System\CentralProcessor\0") as key:
            cpu = winreg.QueryValueEx(key, "ProcessorNameString")[0].strip()
    sources = [*sorted((ROOT / "src").glob("*.[ch]*")), *sorted((ROOT / "include").glob("*.h")), Path(__file__),
               ROOT / "scripts/benchmark_solvers.cpp", ROOT / "scripts/benchmark_discovery.py",
               ROOT / "tests/fixtures/generate_solver_cases.py", ROOT / "tests/fixtures/generate_discovery_cases.py",
               ROOT / "tests/crx_asset_reference.py", ROOT / "tests/crx_reference.py",
               ROOT / "CMakeLists.txt", ROOT / "CMakePresets.json", ROOT / "uv.lock",
               ROOT / "pyproject.toml", ROOT / ".python-version", ROOT / ".clang-tidy", ROOT / ".clang-format",
               ROOT / "tests/fixtures/crx_asset_frames.json", ROOT / "tests/fixtures/CRX10iA-solutions.json"]
    eigen_headers = {path.relative_to(ROOT / "third_party/eigen").as_posix(): sha256(path)
                     for path in sorted((ROOT / "third_party/eigen/Eigen").rglob("*")) if path.is_file()}
    metadata = dict(measured_at_utc=datetime.now(timezone.utc).isoformat(), cpu=cpu,
                    logical_cpus=os.cpu_count(), platform=platform.platform(),
                    git_head=subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=ROOT, text=True).strip(),
                    git_status=subprocess.check_output(["git", "status", "--short"], cwd=ROOT, text=True),
                    eigen_headers_sha256=hashlib.sha256(json.dumps(eigen_headers, sort_keys=True).encode()).hexdigest(),
                    pose_count=len(cases), seed=args.seed, repeats=args.repeats, rounds=args.rounds,
                    capacity=args.capacity, random_per_model=args.random_per_model,
                    stress_per_group_model=args.stress_per_group_model,
                    position_tolerance_mm=POSITION_MM, angle_tolerance_deg=ANGLE_DEG,
                    warmup_calls=3, nearby_seed_jitter_deg=1, stress_offsets_rad=OFFSETS_RAD,
                    corpus_sha256=sha256(output / "corpus.json"),
                    input_sha256=sha256(output / "input.txt"),
                    dll_sha256={path: sha256(dll) for path, dll in zip(PATHS, dlls)},
                    executable_sha256=sha256(executable),
                    source_sha256={str(path.relative_to(ROOT)): sha256(path) for path in sources},
                    assets={model["name"]: model["asset_sha256"] for model in models},
                    compile_commands=[entry for entry in compile_commands
                                      if "benchmark_solvers.cpp" in entry["file"] or "crx_solver.cpp" in entry["file"]])
    with zipfile.ZipFile(output / "sources.zip", "w", zipfile.ZIP_DEFLATED) as archive:
        for path in sources:
            archive.write(path, path.relative_to(ROOT))
    print(f"Timing {len(cases)} poses, two modes, two complete DLLs", flush=True)
    with (output / "native.log").open("w") as log:
        subprocess.run([str(executable), *map(str, dlls), str(output / "input.txt"),
                        str(output / "timings.csv"), str(output / "solutions.csv"),
                        str(args.repeats), str(args.rounds), str(args.capacity)], check=True, stderr=log)
    print("Validating every returned command with independent FK", flush=True)
    timings, errors, frame = validate(output, models, references, cases, args.rounds)
    stats = statistics(timings, errors, frame, args.capacity)
    (output / "summary.json").write_text(json.dumps(dict(metadata=metadata, groups=stats), indent=2) + "\n")
    report(output, metadata, stats, frame)
    print(json.dumps({mode: stats[mode]["challenging"] for mode in MODES}, indent=2))
    print(output / "report.md")


if __name__ == "__main__":
    main()
