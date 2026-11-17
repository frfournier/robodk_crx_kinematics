# SPDX-License-Identifier: Apache-2.0
# SPDX-FileCopyrightText: 2026 Centre Technologique en Aérospatiale
"""Paired complete-DLL experiment on immutable existing or independent inputs."""
import argparse
import json
import os
import platform
import shutil
import subprocess
import sys
import zipfile
from datetime import UTC, datetime
from pathlib import Path

import benchmark_solvers as bench
import numpy as np
import pandas as pd
from crx_fixture_support import assert_regenerated_cases_match
from fixtures.generate_accuracy_stress import corpus as stress_corpus


def validate_counters(path, frame, label):
    counters = pd.read_csv(path)
    keys = ["case", "mode"]
    expected = frame[frame.path == label].set_index(keys)
    assert not counters.duplicated(keys).any(), "Duplicate counter records"
    counters = counters.set_index(keys).sort_index()
    expected = expected.sort_index()
    assert counters.index.equals(expected.index), "Missing or unknown counter inputs"
    events = [column for column in counters if column.startswith("cramer_")]
    assert len(events) == 6, "Incomplete refinement counters"
    assert np.isfinite(counters[events].to_numpy()).all()
    assert (counters[events] >= 0).all().all()
    assert (counters[events] % 1 == 0).all().all()
    assert counters.cramer_evaluations.ge(counters.cramer_runs).all()
    assert counters.cramer_guard_rejections.le(counters.cramer_evaluations).all()
    assert counters.cramer_runs.eq(
        counters.cramer_initial_rejections + counters.cramer_successes
        + counters.cramer_recovery_failures
    ).all(), "Refinement outcomes do not partition runs"
    assert counters["count"].eq(expected["count"].clip(lower=0)).all(), (
        "Instrumented and uninstrumented solves disagree"
    )
    return {name: int(counters[name].sum()) for name in events}


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--before", type=Path, required=True)
    parser.add_argument("--after", type=Path, default=bench.ROOT / "build/Release/crx_kinematics.dll")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--corpus", choices=("existing", "stress"), default="stress")
    parser.add_argument("--rounds", type=int, default=5)
    parser.add_argument("--repeats", type=int, default=3)
    parser.add_argument("--capacity", type=int, default=64)
    parser.add_argument("--after-sources", type=Path, help="Saved source ZIP for an archived experiment DLL")
    parser.add_argument("--before-revision")
    parser.add_argument("--after-revision")
    parser.add_argument("--before-counters", type=Path, help="Matched, separate untimed baseline executable")
    parser.add_argument("--after-counters", type=Path, help="Matched, separate untimed candidate executable")
    parser.add_argument("--compile-commands", type=Path,
                        default=bench.ROOT / "build/vscode/windows-clang-release-tidy/compile_commands.json")
    args = parser.parse_args()
    if min(args.rounds, args.repeats, args.capacity) < 1:
        parser.error("rounds, repeats and capacity must be positive")
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    if args.corpus == "stress":
        models, refs, generated = stress_corpus()
        frozen = bench.ROOT / "tests/fixtures/crx_accuracy_stress.json"
    else:
        models, refs, generated = bench.corpus(bench.DEFAULT_RANDOM_PER_MODEL,
                                              bench.DEFAULT_STRESS_PER_GROUP_MODEL, bench.DEFAULT_SEED)
        frozen = bench.ROOT / "tests/fixtures/crx_solver_cases.json"
    cases = json.loads(frozen.read_text())
    assert_regenerated_cases_match(generated, cases)
    shutil.copy2(frozen, output / "corpus.json")
    values = [len(models), len(cases)]
    for model in models:
        values.extend(model["robot"].flat)
    for case in cases:
        values.extend([case["model"], *np.asarray(case["target"]).ravel(order="F"), *case["seed_deg"]])
    (output / "input.txt").write_text(" ".join(format(v, ".17g") for v in values))
    for name, source in (("before.dll", args.before), ("after.dll", args.after),
                         ("runner.exe", bench.ROOT / "build/Release/crx_solver_benchmark.exe")):
        shutil.copy2(source, output / name)
    shutil.copy2(args.compile_commands, output / "compile_commands.json")
    if args.after_sources:
        shutil.copy2(args.after_sources, output / "sources.zip")
    else:
        with zipfile.ZipFile(output / "sources.zip", "w", zipfile.ZIP_DEFLATED) as archive:
            for directory in ("src", "include"):
                for source in sorted((bench.ROOT / directory).glob("*.[ch]*")):
                    archive.write(source, source.relative_to(bench.ROOT))
            for name in ("CMakeLists.txt", "CMakePresets.json", ".clang-format", ".clang-tidy",
                         "pyproject.toml", "uv.lock", ".python-version"):
                archive.write(bench.ROOT / name, name)
            references = [
                *sorted((bench.ROOT / "tests/fixtures").glob("*.py")),
                *sorted((bench.ROOT / "tests/fixtures").glob("*.json")),
                *sorted((bench.ROOT / "python/crx_kinematics").glob("*.py")),
                *sorted((bench.ROOT / "scripts").glob("benchmark*.py")),
                *sorted((bench.ROOT / "scripts").glob("benchmark*.cpp")),
                *(bench.ROOT / "tests" / name for name in (
                    "crx_asset_reference.py", "crx_fixture_support.py", "crx_reference.py",
                )),
            ]
            for source in references:
                archive.write(source, source.relative_to(bench.ROOT))
    cpu = platform.processor()
    if os.name == "nt":
        import winreg
        with winreg.OpenKey(winreg.HKEY_LOCAL_MACHINE,
                            r"HARDWARE\DESCRIPTION\System\CentralProcessor\0") as key:
            cpu = winreg.QueryValueEx(key, "ProcessorNameString")[0].strip()
    eigen_headers = {
        path.relative_to(bench.ROOT / "third_party/eigen").as_posix(): bench.sha256(path)
        for path in sorted((bench.ROOT / "third_party/eigen/Eigen").rglob("*"))
        if path.is_file()
    }
    (output / "eigen-headers.json").write_text(json.dumps(eigen_headers, indent=2))
    metadata = {"labels": {"baseline": "baseline", "candidate": "candidate"},
                    "measured_at_utc": datetime.now(UTC).isoformat(),
                    "platform": platform.platform(), "cpu": cpu,
                    "python": platform.python_version(), "numpy": np.__version__,
                    "pandas": pd.__version__,
                    "corpus": args.corpus, "rounds": args.rounds, "repeats": args.repeats, "capacity": args.capacity,
                    "position_tolerance_mm": bench.POSITION_MM, "angle_tolerance_deg": bench.ANGLE_DEG,
                    "warmup_calls": 3, "pinned_thread": True,
                    "revisions": {"baseline": args.before_revision, "candidate": args.after_revision},
                    "command": [sys.executable, *sys.argv],
                    "harness_sha256": bench.sha256(Path(__file__)),
                    "validator_sha256": bench.sha256(Path(bench.__file__)),
                    "hashes": {p.name: bench.sha256(p) for p in output.iterdir() if p.is_file()}}
    print(f"Timing {len(cases)} frozen {args.corpus} poses", flush=True)
    with (output / "native.log").open("w") as log:
        subprocess.run([str(output / "runner.exe"), str(output / "before.dll"),
                        str(output / "after.dll"), str(output / "input.txt"),
                        str(output / "timings.csv"), str(output / "solutions.csv"),
                        str(args.repeats), str(args.rounds), str(args.capacity)], check=True, stderr=log)
    times, errors, frame = bench.validate(output, models, refs, cases, args.rounds)
    metadata["counters"] = {}
    for label, runner in (("baseline", args.before_counters), ("candidate", args.after_counters)):
        if runner is None:
            continue
        saved_runner = output / f"{label}-counters.exe"
        shutil.copy2(runner, saved_runner)
        counter_path = output / f"{label}-counters.csv"
        subprocess.run([str(saved_runner), str(output / "input.txt"),
                        str(counter_path), str(args.capacity)], check=True)
        metadata["counters"][label] = validate_counters(counter_path, frame, label)
        metadata["hashes"][saved_runner.name] = bench.sha256(saved_runner)
        metadata["hashes"][counter_path.name] = bench.sha256(counter_path)
    stats = bench.statistics(times, errors, frame, args.capacity)
    (output / "summary.json").write_text(json.dumps({"metadata": metadata, "groups": stats}, indent=2))
    for group in ("all_reachable", "challenging", "base_axis", "mixed_singular"):
        print(group, json.dumps({path: {key: stats["no_seed"][group][path][key]
                                      for key in ("strict_solves", "timing_us", "position_mm", "angle_deg", "witness_hits")}
                                 for path in bench.PATHS}), flush=True)


if __name__ == "__main__":
    main()
