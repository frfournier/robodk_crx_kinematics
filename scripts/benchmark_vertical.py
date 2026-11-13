# SPDX-License-Identifier: Apache-2.0
# SPDX-FileCopyrightText: 2026 Centre Technologique en Aérospatiale
"""Paired vertical/general discovery and full-DLL subfamily measurements.

Targets and references are generated independently of either DLL. Explicit
closed full-turn command boxes distinguish geometric coverage from catalogue
defaults. Exact vertical, near-vertical, coplanar and generic groups are separate.
"""

import argparse
from datetime import UTC, datetime
import json
from pathlib import Path
import platform
import shutil
import subprocess
import sys
import zipfile

import benchmark_solvers as bench
import numpy as np
import pandas as pd
from crx_asset_reference import canonical_forward, forward_models
from crx_reference import rotation
from crx_vertical_reference import vertical_reference

ROOT = Path(__file__).resolve().parents[1]


def make_cases(seed):
    models, references, _ = bench.corpus(0, 0, seed)
    rng = np.random.default_rng(seed)
    cases = []
    for model in models:
        assert np.array_equal(model["base"], np.eye(4))
        assert np.array_equal(model["tool"], np.eye(4))
        model["robot"][30, :6] = -180
        model["robot"][31, :6] = 180

    def add(model_id, name, group, target, witnesses=(), oracle=None, seed_q=None):
        witnesses = [np.asarray(q).tolist() for q in witnesses]
        source_seed = witnesses[0] if witnesses else np.zeros(6) if seed_q is None else seed_q
        cases.append({
            "model": model_id, "name": name, "group": group,
            "target": np.asarray(target).tolist(), "canonical": np.asarray(target).tolist(),
            "reachable": bool(witnesses) or bool(oracle and oracle["family"]),
            "offset_rad": 0., "witnesses": witnesses,
            "seed_deg": np.degrees(source_seed).tolist(),
            "finite_vertical_oracle": oracle is not None and not oracle["family"],
            "family_oracle": bool(oracle and oracle["family"]),
            "oracle_axes": [] if oracle is None else [axis.tolist() for axis in oracle["axes"]],
        })

    for model_id, model in enumerate(models):
        a, b, c, r = lengths = np.array(model["lengths"])
        for normal in [-1., 1.]:
            nearby_samples = 0
            for sample in range(16):
                yaw = float(rng.uniform(-np.pi, np.pi))
                angle = sample*.37
                radius = a*(.15 + .09*sample)
                height = a*((sample % 7)-3)*.2
                if sample == 0:
                    height = 0.
                target = np.eye(4)
                target[:3, :3] = [[np.cos(yaw), -normal*np.sin(yaw), 0],
                                     [np.sin(yaw), normal*np.cos(yaw), 0], [0, 0, normal]]
                target[:3, 3] = [radius*np.cos(angle), radius*np.sin(angle), height+normal*c]
                oracle = vertical_reference(lengths, target)
                add(model_id, f"{model['name']}:vertical:{normal:g}:{sample}", "vertical",
                    target, oracle["solutions"], oracle)
                if nearby_samples < 2 and oracle["solutions"] and not oracle["family"]:
                    nearby_samples += 1
                    for tilt in [-1e-7, 1e-7]:
                        seed_q = np.asarray(oracle["solutions"][0]).copy()
                        witness = seed_q.copy()
                        witness[4] += tilt
                        nearby = canonical_forward(lengths, witness)
                        assert np.hypot(*nearby[:2, 2]) > 0
                        add(model_id, f"{model['name']}:near-vertical:{normal:g}:{sample}:{tilt:g}",
                            "near_vertical", nearby, [witness], seed_q=seed_q)
            assert nearby_samples == 2
            for height in [0., .3*a]:
                target = np.diag([1., normal, normal, 1.])
                target[:3, 3] = [abs(r), 0., height+normal*c]
                oracle = vertical_reference(lengths, target)
                add(model_id, f"{model['name']}:tangent:{normal:g}:{height:g}", "vertical_boundary",
                    target, oracle["solutions"], oracle)
        for sample in range(16):
            q = rng.uniform(-np.pi, np.pi, 6)
            target = canonical_forward(lengths, q)
            add(model_id, f"{model['name']}:generic:{sample}", "generic", target, [q])
            # q4=q6=pi/2 and q5=0 give an exact xz-plane subfamily. Construct
            # its represented matrix/translation directly, avoiding sin(pi)
            # artefacts. Independent FK still validates the supplied witness.
            q = np.array([0., *rng.uniform(-1.4, 1.4, 2), np.pi/2, 0., np.pi/2])
            target = canonical_forward(lengths, q)
            target[1, 3] = 0.
            target[:3, :3] = rotation(1, q[1]-q[2]+np.pi/2)[:3, :3]
            add(model_id, f"{model['name']}:coplanar:{sample}", "coplanar", target, [q])
    # Check the input witnesses through the captured modified-DH reference,
    # independently of the canonical construction and both measured DLLs.
    for model_id, reference in enumerate(references):
        selected = [(case, q) for case in cases if case["model"] == model_id
                    for q in case["witnesses"]]
        if selected:
            actual = forward_models(reference, np.degrees([q for _, q in selected]))
            targets = np.array([case["target"] for case, _ in selected])
            assert np.linalg.norm(actual[:, :3, 3]-targets[:, :3, 3], axis=1).max() <= bench.POSITION_MM
            assert np.linalg.norm(actual[:, :3, :3]-targets[:, :3, :3], axis=(1, 2)).max() <= 1e-10
    return models, references, cases


def component_results(folder, models, cases, rounds):
    timings = pd.read_csv(folder / "timings.csv")
    solutions = pd.read_csv(folder / "solutions.csv")
    assert set(timings.path) == {"general", "closed_form"}
    assert set(timings["case"]) == set(range(len(cases)))
    assert set(timings["round"]) == set(range(rounds))
    assert not timings.duplicated(["case", "path", "round"]).any()
    assert set(solutions.path) <= {"general", "closed_form"}
    assert set(solutions["case"]) <= set(range(len(cases)))
    assert np.isfinite(solutions[[f"q{i}" for i in range(1, 7)]]).all().all()
    assert len(timings) == len(cases)*2*rounds
    assert timings.groupby(["case", "path"]).size().eq(rounds).all()
    assert timings.groupby(["case", "path"])[["status", "count", "vertical_closed_form"]].nunique().le(1).all().all()
    assert timings.us.gt(0).all() and np.isfinite(timings.us).all()
    assert not timings.loc[timings.path == "general", "vertical_closed_form"].any()
    grouped = {key: rows[[f"q{i}" for i in range(1, 7)]].to_numpy()
               for key, rows in solutions.groupby(["case", "path"])}
    times = timings.groupby(["case", "path"]).us.median().unstack()
    outcomes = timings[timings["round"] == 0].set_index(["case", "path"])
    rows = []
    for index, case in enumerate(cases):
        record = {"case": index, "name": case["name"], "group": case["group"],
                  "oracle_count": len(case["witnesses"]) if case["finite_vertical_oracle"] else None,
                  "family_oracle": case["family_oracle"]}
        before = grouped.get((index, "general"), np.empty((0, 6)))
        after = grouped.get((index, "closed_form"), np.empty((0, 6)))
        for path, joints in [("general", before), ("closed_form", after)]:
            assert len(joints) == int(outcomes.loc[(index, path), "count"]), (case["name"], path)
            for i, q in enumerate(joints):
                if i:
                    distance = np.max(np.abs((joints[:i]-q+np.pi) % (2*np.pi)-np.pi), axis=1)
                    # Match the existing private discovery deduplication
                    # contract (1e-10 rad), including resolved near-fold pairs.
                    assert distance.min() > 1e-10, (case["name"], path, "duplicate posture")
            invalid = 0
            for q in joints:
                actual = canonical_forward(models[case["model"]]["lengths"], q)
                target = np.asarray(case["canonical"])
                position = np.linalg.norm(actual[:3, 3]-target[:3, 3])
                angle = np.degrees(2*np.arcsin(min(1., np.linalg.norm(actual[:3, :3]-target[:3, :3])/(2*np.sqrt(2)))))
                invalid += int(position > bench.POSITION_MM or angle > bench.ANGLE_DEG)
            witnesses = np.asarray(case["witnesses"]).reshape(-1, 6)
            hits = sum(bool(len(joints)) and np.min(np.max(np.abs((joints-q+np.pi) % (2*np.pi)-np.pi), axis=1))
                       <= np.radians(bench.WITNESS_DEG) for q in witnesses)
            record.update({f"{path}_count": len(joints), f"{path}_invalid": invalid,
                           f"{path}_witness_hits": int(hits), f"{path}_us": float(times.loc[index, path]),
                           f"{path}_status": int(outcomes.loc[(index, path), "status"])})
            assert invalid == 0, (case["name"], path)
            if path == "closed_form" and case["finite_vertical_oracle"]:
                assert hits == len(witnesses), (case["name"], len(witnesses), hits)
                assert len(joints) == len(witnesses), (case["name"], len(witnesses), len(joints))
        lost = sum(not len(after) or np.min(np.max(np.abs((after-q+np.pi) % (2*np.pi)-np.pi), axis=1))
                   > np.radians(bench.WITNESS_DEG) for q in before)
        record["lost_general_postures"] = int(lost)
        record["used_vertical_closed_form"] = bool(outcomes.loc[(index, "closed_form"), "vertical_closed_form"])
        if case["group"] in {"near_vertical", "coplanar", "generic"} or case["family_oracle"]:
            assert not record["used_vertical_closed_form"], case["name"]
        rows.append(record)
    frame = pd.DataFrame(rows)
    frame.to_csv(folder / "per-case.csv", index=False)
    assert frame.lost_general_postures.sum() == 0, frame[frame.lost_general_postures > 0].to_dict("records")
    result = {}
    for group in ["all", *frame.group.unique()]:
        subset = frame if group == "all" else frame[frame.group == group]
        result[group] = {"poses": len(subset), "used_vertical_closed_form": int(subset.used_vertical_closed_form.sum()),
                         "count_changes": int(subset.general_count.ne(subset.closed_form_count).sum()),
                         "lost_general_postures": int(subset.lost_general_postures.sum()),
                         "timing_us": {path: {label: float(subset[f"{path}_us"].quantile(fraction))
                                              for label, fraction in [("p50", .5), ("p95", .95), ("p99", .99), ("max", 1)]}
                                       for path in ["general", "closed_form"]}}
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--before", type=Path, required=True)
    parser.add_argument("--after", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--skip-component", action="store_true")
    parser.add_argument("--rounds", type=int, default=9)
    parser.add_argument("--repeats", type=int, default=8)
    parser.add_argument("--capacity", type=int, default=512)
    parser.add_argument("--seed", type=int, default=20261113)
    args = parser.parse_args()
    if min(args.rounds, args.repeats, args.capacity) < 1:
        parser.error("counts must be positive")
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    models, references, cases = make_cases(args.seed)
    assert len(cases) == 456
    (output / "corpus.json").write_text(json.dumps(cases, indent=2)+"\n")
    (output / "models.json").write_text(json.dumps([
        {key: value.tolist() if isinstance(value, np.ndarray) else value for key, value in model.items()}
        for model in models], indent=2)+"\n")
    values = [len(models), len(cases)]
    for model in models:
        values.extend(model["robot"].flat)
    for case in cases:
        values.extend([case["model"], *np.asarray(case["target"]).ravel(order="F"), *case["seed_deg"]])
    (output / "input.txt").write_text(" ".join(format(v, ".17g") for v in values))
    components = None
    if not args.skip_component:
        folder = output / "component"
        folder.mkdir()
        values = [len(models), len(cases)]
        for model in models:
            values.extend(model["robot"][:32].flat)
        for case in cases:
            values.extend([case["model"], *models[case["model"]]["lengths"],
                           *np.asarray(case["canonical"]).flat, *np.asarray(case["target"]).ravel(order="F")])
        (folder / "input.txt").write_text(" ".join(format(v, ".17g") for v in values))
        runner = folder / "runner.exe"
        shutil.copy2(ROOT / "build/Release/crx_discovery_benchmark.exe", runner)
        with (folder / "native.log").open("w") as log:
            subprocess.run([str(runner), "--paired-discovery", str(folder/"input.txt"), str(folder/"timings.csv"),
                            str(folder/"solutions.csv"), str(args.repeats), str(args.rounds)], check=True, stderr=log)
        assert "Thread affinity mask" in (folder/"native.log").read_text()
        components = component_results(folder, models, cases, args.rounds)
        (folder / "summary.json").write_text(json.dumps(components, indent=2)+"\n")
    for name, source in [("before.dll", args.before), ("after.dll", args.after),
                         ("runner.exe", ROOT/"build/Release/crx_solver_benchmark.exe")]:
        shutil.copy2(source, output/name)
    with (output/"native.log").open("w") as log:
        subprocess.run([str(output/"runner.exe"), str(output/"before.dll"), str(output/"after.dll"),
                        str(output/"input.txt"), str(output/"timings.csv"), str(output/"solutions.csv"),
                        str(args.repeats), str(args.rounds), str(args.capacity)], check=True, stderr=log)
    assert "Thread affinity mask" in (output/"native.log").read_text()
    times, errors, frame = bench.validate(output, models, references, cases, args.rounds)
    assert frame.invalid_solutions.sum() == 0
    paired = pd.read_csv(output/"comparison.csv")
    assert not paired.lost_solve.any() and not paired.lost_witnesses.any()
    stats = bench.statistics(times, errors, frame, args.capacity)
    sources = [*sorted((ROOT/"src").glob("*.[ch]*")), *sorted((ROOT/"include").glob("*.h")),
               Path(__file__), ROOT/"scripts/benchmark_discovery.cpp", ROOT/"scripts/benchmark_solvers.cpp",
               ROOT/"scripts/benchmark_solvers.py", ROOT/"tests/crx_vertical_reference.py",
               ROOT/"tests/crx_reference.py", ROOT/"tests/crx_asset_reference.py", ROOT/"tests/fixtures/crx_asset_frames.json",
               ROOT/"CMakeLists.txt", ROOT/"CMakePresets.json", ROOT/"pyproject.toml", ROOT/"uv.lock"]
    with zipfile.ZipFile(output/"sources.zip", "w", zipfile.ZIP_DEFLATED) as archive:
        for source in sources:
            archive.write(source, source.relative_to(ROOT))
    commands = ROOT/"build/vscode/windows-clang-release-tidy/compile_commands.json"
    shutil.copy2(commands, output/"compile_commands.json")
    metadata = {"measured_at_utc": datetime.now(UTC).isoformat(), "platform": platform.platform(),
                "cpu": platform.processor(), "rounds": args.rounds, "repeats": args.repeats,
                "capacity": args.capacity, "seed": args.seed, "warmups": 3,
                "method": "Native paired batch means, per-pose medians; quantiles span poses, not individual-call tails",
                "limits": "Explicit benchmark-only closed [-180,180] command degrees on all six axes",
                "component_scope": "Same candidate executable with vertical path disabled/enabled, no command handling",
                "full_scope": "Two immutable complete DLLs including mapping, limits, turns and packing",
                "command": [sys.executable, *sys.argv],
                "source_hashes": {p.relative_to(ROOT).as_posix(): bench.sha256(p) for p in sources},
                "hashes": {p.relative_to(output).as_posix(): bench.sha256(p) for p in output.rglob("*") if p.is_file()}}
    (output/"summary.json").write_text(json.dumps({"metadata": metadata, "components": components, "groups": stats}, indent=2)+"\n")
    print(json.dumps({"components": components, "no_seed": {name: stats["no_seed"][name]
                      for name in ["vertical", "vertical_boundary", "near_vertical", "coplanar", "generic"]}}, indent=2))


if __name__ == "__main__":
    main()
