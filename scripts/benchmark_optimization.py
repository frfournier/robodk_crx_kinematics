"""Paired before/after DLL measurements using the native FK/IK runner.

Build crx_solver_benchmark with a Release preset. Save the baseline DLL before
editing. This script snapshots both builds and compares counts and ordered
outputs, then checks IK accuracy with independent modified-DH FK.
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

import numpy as np
import pandas as pd

from benchmark_solvers import ROOT, corpus, validate


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def summarize(timings, solutions, cases):
    keys = ["case", "mode", "round"]
    counts = timings.pivot(index=keys, columns="path", values="count")
    columns = [name for name in solutions.columns if name.startswith("q")]
    ordered = []
    for side in ("baseline", "candidate"):
        ordered.append(solutions[solutions.path == side].set_index(
            ["case", "mode", "solution"])[columns].sort_index())
    before, after = ordered
    common = before.index.intersection(after.index)
    old_values = before.loc[common].to_numpy()
    new_values = after.loc[common].to_numpy()
    differences = np.abs(old_values - new_values)
    result = dict(
        changed_counts=int((counts.baseline != counts.candidate).sum()),
        unmatched_output_rows=len(before.index.symmetric_difference(after.index)),
        max_ordered_output_difference=float(differences.max(initial=0)),
        signed_zero_differences=int(((old_values == 0) & (new_values == 0) &
                                    (np.signbit(old_values) != np.signbit(new_values))).sum()),
        nonfinite_outputs=int((~np.isfinite(solutions[columns].to_numpy())).sum()),
        groups={},
    )
    timings = timings.copy()
    timings["group"] = [cases[index]["group"] for index in timings["case"]]
    for mode, data in timings.groupby("mode"):
        groups = {"all": data, **dict(tuple(data.groupby("group")))}
        for group, subset in groups.items():
            medians = subset.groupby(["case", "path"]).us.median().unstack()
            rounds = subset.groupby(["round", "path"]).us.mean().unstack()
            ratios = rounds.baseline / rounds.candidate
            stats = {}
            for side, label in (("baseline", "before"), ("candidate", "after")):
                values = medians[side]
                stats[label] = dict(
                    mean_us=float(values.mean()),
                    p50_us=float(values.median()),
                    p95_us=float(values.quantile(.95)),
                    p99_us=float(values.quantile(.99)),
                    throughput_per_second=float(1e6 / values.mean()),
                    raw_p95_us=float(subset[subset.path == side].us.quantile(.95)),
                    raw_p99_us=float(subset[subset.path == side].us.quantile(.99)),
                )
            stats["speedup"] = float(medians.baseline.mean() / medians.candidate.mean())
            stats["round_speedup_range"] = [float(ratios.min()), float(ratios.max())]
            result["groups"][f"{mode}/{group}"] = stats
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--baseline-dll", required=True, type=Path)
    parser.add_argument("--candidate-dll", type=Path, default=ROOT / "build/Release/crx_kinematics.dll")
    parser.add_argument("--runner", type=Path, default=ROOT / "build/Release/crx_solver_benchmark.exe")
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--random-per-model", type=int, default=64)
    parser.add_argument("--stress-per-group-model", type=int, default=28)
    parser.add_argument("--seed", type=int, default=20261009)
    parser.add_argument("--rounds", type=int, default=9)
    parser.add_argument("--repeats", type=int, default=3)
    parser.add_argument("--fk-repeats", type=int, default=100)
    parser.add_argument("--capacity", type=int, default=64)
    parser.add_argument("--baseline-revision", help="Commit used to build the baseline artifact")
    parser.add_argument("--require-identical", action="store_true",
                        help="Fail on count, ordering or floating-point output differences")
    args = parser.parse_args()
    if min(args.random_per_model, args.stress_per_group_model, args.rounds,
           args.repeats, args.fk_repeats, args.capacity) < 1:
        parser.error("counts must be positive")
    output = args.output.resolve()
    if output.exists() and any(output.iterdir()):
        parser.error("output directory must be empty/new")
    output.mkdir(parents=True, exist_ok=True)
    models, references, cases = corpus(args.random_per_model, args.stress_per_group_model, args.seed)
    (output / "corpus.json").write_text(json.dumps(cases, indent=2) + "\n")
    values = [len(models), len(cases)]
    for model in models:
        values.extend(model["robot"].flat)
    for case in cases:
        values.extend([case["model"], *np.asarray(case["target"]).ravel(order="F"), *case["seed_deg"]])
    (output / "input.txt").write_text(" ".join(format(value, ".17g") for value in values))
    runner = output / args.runner.name
    shutil.copy2(args.runner, runner)
    metadata = dict(measured_at_utc=datetime.now(timezone.utc).isoformat(),
                    platform=platform.platform(), processor=platform.processor(),
                    logical_cpus=os.cpu_count(), cases=len(cases),
                    arguments={key: str(value) if isinstance(value, Path) else value
                               for key, value in vars(args).items()},
                    runner_sha256=digest(runner), corpus_sha256=digest(output / "corpus.json"),
                    git_status=subprocess.check_output(["git", "status", "--short"], cwd=ROOT, text=True),
                    binaries={})
    results = {}
    libraries = []
    for label, source in (("baseline", args.baseline_dll), ("candidate", args.candidate_dll)):
        destination = output / f"{label}.dll"
        shutil.copy2(source, destination)
        metadata["binaries"][label] = digest(destination)
        libraries.append(destination)
    for mode, repeats in (("ik", args.repeats), ("fk", args.fk_repeats)):
        directory = output / mode
        directory.mkdir()
        command = [str(runner), *map(str, libraries), str(output / "input.txt"),
                   str(directory / "timings.csv"), str(directory / "solutions.csv"),
                   str(repeats), str(args.rounds), str(args.capacity)]
        if mode == "fk":
            command.append("fk")
        with (directory / "native.log").open("w") as log:
            subprocess.run(command, check=True, stderr=log)
        timings = pd.read_csv(directory / "timings.csv", float_precision="round_trip")
        solutions = pd.read_csv(directory / "solutions.csv", float_precision="round_trip")
        result = summarize(timings, solutions, cases)
        if mode == "ik":
            _, errors, _ = validate(directory, models, references, cases, args.rounds)
            result["accuracy"] = {}
            for side, label in (("baseline", "before"), ("candidate", "after")):
                rows = errors[errors.path == side]
                result["accuracy"][label] = dict(
                    invalid_outputs=int((~rows.valid).sum()),
                    max_position_mm=float(rows.position_mm.max()),
                    max_angle_deg=float(rows.angle_deg.max()),
                )
        results[mode] = result
        print(mode, json.dumps({key: value for key, value in result.items()
                                        if key != "groups"}), flush=True)
        for key, value in result["groups"].items():
            if key.endswith("/all"):
                print(key, json.dumps(value), flush=True)
    (output / "summary.json").write_text(json.dumps(dict(metadata=metadata, results=results), indent=2) + "\n")
    if any(result["nonfinite_outputs"] or
           result.get("accuracy", {}).get("after", {}).get("invalid_outputs", 0)
           for result in results.values()):
        raise SystemExit("Invalid outputs; inspect the accuracy artifacts")
    # Exact comparison intentionally catches ordering and singular branch drift.
    if args.require_identical and any(result["changed_counts"] or result["unmatched_output_rows"] or
           result["max_ordered_output_difference"] or result["signed_zero_differences"] or
           result["nonfinite_outputs"] or
           any(side["invalid_outputs"] for side in result.get("accuracy", {}).values())
           for result in results.values()):
        raise SystemExit("Differential outputs changed; inspect artifacts before retaining the experiment")


if __name__ == "__main__":
    main()
