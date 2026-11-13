# SPDX-License-Identifier: Apache-2.0
# SPDX-FileCopyrightText: 2026 Centre Technologique en Aérospatiale
"""Native certification-foundation costs, widths, domains and independent checks.

This is separate from full-DLL solver timing. Numerical circle preparation
does less work than full exact model/frame preparation; they are separate rows.
"""

import argparse
from datetime import UTC, datetime
from fractions import Fraction as F
import itertools
import json
import os
from pathlib import Path
import platform
import shutil
import subprocess
import sys
import zipfile

import benchmark_solvers as bench  # Registers the repository's test references.
import crx_exact_reference as ref
from fixtures.benchmark_models import benchmark_models
import mpmath as mp
import numpy as np
import pandas as pd

ROOT = Path(__file__).resolve().parents[1]


def pose(q, p):
    return {"quaternion": list(map(float, q)), "position": list(map(float, p)),
            "input_rotation": [float(x) for x in sum(ref.quaternion_rotation(q), [])]}


def flatten(input):
    result = [*input["lengths"], input["base_height"], *input["encoder_zero"],
              *input["senses"], *input["lower_commands"], *input["upper_commands"]]
    for key in ["target", "base", "tool"]:
        for field in ["quaternion", "position", "input_rotation"]:
            result += input[key][field]
    assert len(result) == 77
    return result


def corpus(seed):
    models, _ = benchmark_models(seed)
    rng = np.random.default_rng(seed)
    cases = []
    groups = ["point", "box", "frames", "signed", "encoder_seam", "large_encoder", "quaternion_scale_and_large_trig", "division_seam"]
    for model, scale, sample in itertools.product(models, [.001, 1., 1000.], range(8)):
        lengths = np.asarray(model["lengths"])*scale
        size = abs(lengths).max()
        q = rng.uniform(-2, 2, 6)
        width = 1e-9 if sample == 1 else 0.
        identity = pose([1., 0., 0., 0.], [0., 0., 0.])
        input = {"lengths": lengths.tolist(), "base_height": float(model["robot"][10, 3]*scale),
                 "encoder_zero": model["robot"][10:16, 2].tolist(), "senses": model["robot"][3, 4:10].tolist(),
                 "lower_commands": np.radians(model["robot"][30, :6]).tolist(),
                 "upper_commands": np.radians(model["robot"][31, :6]).tolist(),
                 "target": pose(rng.uniform(-1, 1, 4), rng.uniform(-1, 1, 3)*size),
                 "base": identity, "tool": identity}
        if sample == 2:
            input["base"] = pose(rng.uniform(-1, 1, 4), rng.uniform(-.5, .5, 3)*size)
            input["tool"] = pose(rng.uniform(-1, 1, 4), rng.uniform(-.2, .2, 3)*size)
        if sample == 3:
            input["lengths"][1] *= -1
            input["lengths"][3] *= -1
        if sample == 4:
            input["encoder_zero"] = [float(np.nextafter(np.pi, np.inf))]*6
        if sample == 5:
            input["encoder_zero"] = [1e20]*6
        if sample == 6:
            exponent = -900 if scale < 1 else 900
            input["target"]["quaternion"] = np.ldexp(input["target"]["quaternion"], exponent).tolist()
        a = [float(q[0]-width), float(q[0]+width)]
        b = [float(q[1]-width), float(q[1]+width)]
        if sample == 6:
            a = [2**20-1., 2**20+1.]
        if sample == 7:
            b = [-1e-12, 1e-12]
        expected = ref.prepare(input)
        canonical = np.eye(4)
        canonical[:3, :3] = [[float(x) for x in row] for row in expected["rotation"]]
        canonical[:3, 3] = [float(x*expected["scale"]) for x in expected["translation"]]
        cases.append({"name": f"{model['name']}:{scale:g}:{groups[sample]}", "group": groups[sample],
                      "input": input, "canonical": canonical.tolist(),
                      "joints": [[float(x-width), float(x+width)] for x in q],
                      "chart": [0., .3, np.pi/2, -np.pi/4][sample % 4], "a": a, "b": b})
    return cases


def contains(bounds, value):
    lo, hi = bounds
    assert np.isfinite([lo, hi]).all() and lo <= hi
    if isinstance(value, F):
        assert F(lo) <= value <= F(hi)
    else:
        assert ref.number(F(lo)) <= value <= ref.number(F(hi))


def validate(output, cases, rounds):
    timings = pd.read_csv(output/"timings.csv", float_precision="round_trip")
    bounds = pd.read_csv(output/"bounds.csv", float_precision="round_trip")
    assert np.isfinite(timings.us).all() and timings.us.gt(0).all()
    assert set(timings.valid) <= {0, 1}
    assert not timings.duplicated(["case", "operation", "path", "round"]).any()
    assert set(timings["case"]) == set(range(len(cases)))
    assert set(timings["round"]) == set(range(rounds))
    assert timings.groupby(["case", "operation", "path"]).size().eq(rounds).all()
    assert timings.groupby(["case", "operation", "path"]).valid.nunique().eq(1).all()
    expected_pairs = {(op, path) for op in ["add", "multiply", "divide", "sqrt", "trig_pair", "matrix3_product", "norm3", "coefficient_convolution", "canonical_fk"] for path in ["double", "interval"]}
    expected_pairs |= {("circle_prepare", "double"), ("exact_prepare", "interval"), ("polynomial_bundle", "interval"), ("command_fk", "interval")}
    for _, group in timings.groupby("case"):
        assert set(zip(group.operation, group.path)) == expected_pairs
    assert np.isfinite(bounds[["lo", "hi"]]).all().all() and bounds.lo.le(bounds.hi).all()
    assert not bounds.duplicated(["case", "field", "component"]).any()
    assert set(bounds["case"]) == set(range(len(cases)))
    for _, rows in bounds.groupby(["case", "field"]):
        assert sorted(rows.component) == list(range(len(rows)))
    grouped = {key: value.sort_values("component")[["lo", "hi"]].to_numpy()
               for key, value in bounds.groupby(["case", "field"])}
    width_rows = []
    with mp.workdps(200):
        for index, item in enumerate(cases):
            input = item["input"]
            expected = ref.prepare(input)
            for field in ["prepare_status", "bundle_status", "fk_status", "command_status"]:
                assert grouped[index, field].tolist() == [[0., 0.]], (item["name"], field)
            for field in ["lengths", "translation", "center"]:
                for interval, value in zip(grouped[index, field], expected[field], strict=True):
                    contains(interval, value)
            for interval, value in zip(grouped[index, "rotation"], sum(expected["rotation"], []), strict=True):
                contains(interval, value)
            for interval, value in zip(grouped[index, "deviation"], expected["deviations_squared"], strict=True):
                lo, hi = map(F, interval)
                assert 0 <= lo and lo*lo <= value <= hi*hi
            for field, values in zip(["residual", "derivative", "determinant"], ref.coefficients(expected, F(float(item["chart"]))), strict=True):
                for interval, value in zip(grouped[index, field], values, strict=True):
                    contains(interval, value)
            for corner in [0, .5, 1]:
                joints = [F(lo)+(F(hi)-F(lo))*F(corner) for lo, hi in item["joints"]]
                for command in [False, True]:
                    q = ref.command_angles(input, joints) if command else list(map(ref.number, joints))
                    rotation, translation = ref.forward(expected["lengths"], q)
                    prefix = "command" if command else "fk"
                    for field, values in [(f"{prefix}_rotation", rotation), (f"{prefix}_translation", translation)]:
                        for interval, value in zip(grouped[index, field], values, strict=True):
                            contains(interval, value)
            for field in ["rotation", "translation", "center", "residual", "derivative", "determinant", "fk_rotation", "fk_translation", "command_rotation", "command_translation"]:
                box = grouped[index, field]
                width_rows.append({"case": index, "group": item["group"], "field": field,
                                   "max_width": float((box[:, 1]-box[:, 0]).max())})
    pd.DataFrame(width_rows).to_csv(output/"widths.csv", index=False)
    medians = timings.groupby(["case", "operation", "path"]).agg(us=("us", "median"), valid=("valid", "first")).reset_index()
    medians["group"] = [cases[i]["group"] for i in medians["case"]]
    medians.to_csv(output/"per-case.csv", index=False)
    summaries = {}
    for group in ["all", *dict.fromkeys(x["group"] for x in cases)]:
        selected = medians if group == "all" else medians[medians.group == group]
        summaries[group] = {f"{operation}/{path}": {"poses": len(rows), "finite_results": int(rows.valid.sum()),
                            "failed_results": int((~rows.valid.astype(bool)).sum()),
                            "timing_us": {key: float(rows.us.quantile(q)) for key, q in [("p50", .5), ("p95", .95), ("p99", .99), ("max", 1)]}}
                            for (operation, path), rows in selected.groupby(["operation", "path"])}
    width_frame = pd.DataFrame(width_rows)
    widths = width_frame.groupby(["group", "field"]).max_width.agg(["median", "max"]).reset_index().to_dict("records")
    diagnostics = {group: {"poses": len(indices),
                   "uncertain_encoder_principal_cases": int(sum(grouped[i, "uncertain_principals"][0, 0] > 0 for i in indices)),
                   "full_range_scalar_trig_cases": int(sum(grouped[i, "scalar_trig_full_range"][0, 0] > 0 for i in indices))}
                   for group in ["all", *dict.fromkeys(x["group"] for x in cases)]
                   for indices in [[i for i, item in enumerate(cases) if group == "all" or item["group"] == group]]}
    return {"groups": summaries, "widths": widths, "diagnostics": diagnostics,
            "independent_checks": "all frame/coefficient fields and three FK box samples on every case"}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--rounds", type=int, default=9)
    parser.add_argument("--repeats", type=int, default=64)
    parser.add_argument("--seed", type=int, default=20261113)
    parser.add_argument("--compile-commands", type=Path,
                        default=ROOT/"build/vscode/windows-clang-release-tidy/compile_commands.json")
    parser.add_argument("--runner", type=Path,
                        default=ROOT/"build/Release/crx_certification_benchmark.exe")
    args = parser.parse_args()
    if min(args.rounds, args.repeats) < 1:
        parser.error("counts must be positive")
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    cases = corpus(args.seed)
    (output/"corpus.json").write_text(json.dumps(cases, indent=2)+"\n", encoding="utf-8")
    values = [len(cases)]
    for case in cases:
        values += [*flatten(case["input"]), *np.asarray(case["canonical"]).flat,
                   *itertools.chain.from_iterable(case["joints"]), case["chart"], *case["a"], *case["b"]]
    (output/"input.txt").write_text(" ".join(format(v, ".17g") for v in values))
    runner = output/"runner.exe"
    shutil.copy2(args.runner, runner)
    with (output/"native.log").open("w") as log:
        subprocess.run([str(runner), str(output/"input.txt"), str(output/"timings.csv"), str(output/"bounds.csv"),
                        str(args.repeats), str(args.rounds)], check=True, stderr=log)
    assert "Thread affinity mask" in (output/"native.log").read_text()
    result = validate(output, cases, args.rounds)
    cpu = platform.processor()
    if os.name == "nt":
        import winreg
        with winreg.OpenKey(winreg.HKEY_LOCAL_MACHINE, r"HARDWARE\DESCRIPTION\System\CentralProcessor\0") as key:
            cpu = winreg.QueryValueEx(key, "ProcessorNameString")[0].strip()
    sources = [*sorted((ROOT/"src").glob("*.[ch]*")), Path(__file__), ROOT/"scripts/benchmark_certification.cpp",
               ROOT/"scripts/benchmark_solvers.py", ROOT/"tests/crx_reference.py", ROOT/"tests/crx_asset_reference.py",
               ROOT/"tests/crx_exact_reference.py", ROOT/"tests/fixtures/crx_asset_frames.json",
               *sorted((ROOT/"tests/fixtures").glob("*.py")), ROOT/"CMakeLists.txt", ROOT/"CMakePresets.json", ROOT/"pyproject.toml", ROOT/"uv.lock"]
    with zipfile.ZipFile(output/"sources.zip", "w", zipfile.ZIP_DEFLATED) as archive:
        for source in sources:
            archive.write(source, source.relative_to(ROOT))
    shutil.copy2(args.compile_commands, output/"compile_commands.json")
    result["metadata"] = {"measured_at_utc": datetime.now(UTC).isoformat(), "cpu": cpu, "platform": platform.platform(),
                          "rounds": args.rounds, "repeats": args.repeats, "warmups": 3, "poses": len(cases), "seed": args.seed,
                          "mpmath": mp.__version__, "numpy": np.__version__, "pandas": pd.__version__, "python": platform.python_version(),
                          "method": "Native paired batch means; per-pose round medians; quantiles span poses. Includes volatile indirect-call and result-consumption overhead.",
                          "scope": "Exact preparation includes model identity, frame quaternion composition, raw zeros and deviations; numerical circle preparation is a separate, smaller task. No certificate or coverage claim.",
                          "corpus": "Bounded finite nominal inputs constructed from six asset geometries; no IK selection. Unsupported domain behavior is checked separately by tests.",
                          "command": [sys.executable, *sys.argv], "source_hashes": {p.relative_to(ROOT).as_posix(): bench.sha256(p) for p in sources},
                          "hashes": {p.name: bench.sha256(p) for p in output.iterdir() if p.is_file()}}
    (output/"summary.json").write_text(json.dumps(result, indent=2)+"\n", encoding="utf-8")
    print(json.dumps(result["groups"]["all"], indent=2))


if __name__ == "__main__":
    main()
