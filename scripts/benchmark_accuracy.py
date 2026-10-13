"""Paired complete-DLL experiment on immutable existing or independent inputs."""
import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import platform
import shutil
import subprocess
import zipfile

import numpy as np

import benchmark_solvers as bench
from fixtures.generate_accuracy_stress import corpus as stress_corpus


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--before", type=Path, required=True)
    parser.add_argument("--after", type=Path, default=bench.ROOT / "build/Release/crx_kinematics_polynomial.dll")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--corpus", choices=("existing", "stress"), default="stress")
    parser.add_argument("--rounds", type=int, default=5)
    parser.add_argument("--repeats", type=int, default=3)
    parser.add_argument("--capacity", type=int, default=64)
    parser.add_argument("--after-sources", type=Path, help="Saved source ZIP for an archived experiment DLL")
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
    assert cases == generated, "Frozen inputs differ from model/generator version"
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
    if args.after_sources:
        shutil.copy2(args.after_sources, output / "sources.zip")
    else:
        with zipfile.ZipFile(output / "sources.zip", "w", zipfile.ZIP_DEFLATED) as archive:
            for source in sorted((bench.ROOT / "src").glob("*.[ch]*")):
                archive.write(source, source.relative_to(bench.ROOT))
    metadata = dict(labels={"legacy": "before polynomial", "polynomial": "after polynomial"},
                    measured_at_utc=datetime.now(timezone.utc).isoformat(),
                    platform=platform.platform(), cpu=platform.processor(),
                    corpus=args.corpus, rounds=args.rounds, repeats=args.repeats, capacity=args.capacity,
                    position_tolerance_mm=bench.POSITION_MM, angle_tolerance_deg=bench.ANGLE_DEG,
                    warmup_calls=3, pinned_thread=True,
                    hashes={p.name: bench.sha256(p) for p in output.iterdir() if p.is_file()})
    print(f"Timing {len(cases)} frozen {args.corpus} poses", flush=True)
    with (output / "native.log").open("w") as log:
        subprocess.run([str(output / "runner.exe"), str(output / "before.dll"),
                        str(output / "after.dll"), str(output / "input.txt"),
                        str(output / "timings.csv"), str(output / "solutions.csv"),
                        str(args.repeats), str(args.rounds), str(args.capacity)], check=True, stderr=log)
    times, errors, frame = bench.validate(output, models, refs, cases, args.rounds)
    stats = bench.statistics(times, errors, frame, args.capacity)
    (output / "summary.json").write_text(json.dumps(dict(metadata=metadata, groups=stats), indent=2))
    for group in ("all_reachable", "challenging", "base_axis", "mixed_singular"):
        print(group, json.dumps({path: {key: stats["no_seed"][group][path][key]
                                      for key in ("strict_solves", "timing_us", "position_mm", "angle_deg", "witness_hits")}
                                 for path in bench.PATHS}), flush=True)


if __name__ == "__main__":
    main()
