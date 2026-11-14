# SPDX-License-Identifier: Apache-2.0
# SPDX-FileCopyrightText: 2026 Centre Technologique en Aérospatiale
"""Native local-certificate costs, conditioning, reuse, widths and exact-root checks."""

import argparse
from datetime import UTC, datetime
from fractions import Fraction as F
import itertools
import json
import math
import os
from pathlib import Path
import platform
import shutil
import struct
import subprocess
import sys
import zipfile

import benchmark_solvers as bench
import crx_exact_reference as ref
from fixtures.benchmark_models import benchmark_models
from test_crx_ball_certificate import check_certificate, exact_kinematics, prepare_reference
from test_crx_exact_target import contains, unflatten
import mpmath as mp
import numpy as np
import pandas as pd

ROOT = Path(__file__).resolve().parents[1]


def frame(q=(1., 0., 0., 0.), p=(0., 0., 0.)):
    result = np.eye(4)
    result[:3, :3] = [[float(x) for x in row] for row in ref.quaternion_rotation(q)]
    result[:3, 3] = p
    return result.tolist()


def source_target(lengths, zeros, senses, command, reference, tool):
    # Commands first become represented radians, as in the C ABI. The exact
    # canonical map and elementary transforms then construct a witness before
    # the resulting raw target matrix is rounded once for the query.
    radians = [F(float(x*(math.pi/180))) for x in command]
    q = ref.command_angles({"encoder_zero": zeros, "senses": senses}, radians)
    r, p = ref.forward(lengths, q)
    rotation = mp.matrix([r[i:i+3] for i in range(0, 9, 3)])
    rr = mp.matrix([row[:3] for row in reference[:3]])
    ru = mp.matrix([row[:3] for row in tool[:3]])
    pr = mp.matrix([row[3] for row in reference[:3]])
    pu = mp.matrix([row[3] for row in tool[:3]])
    rt = rr.T*rotation*ru
    pt = rr.T*(mp.matrix(p)+rotation*pu-pr)
    target = np.eye(4)
    target[:3, :3] = [[float(rt[i, j]) for j in range(3)] for i in range(3)]
    target[:3, 3] = list(map(float, pt))
    return target.tolist(), list(map(float, q))


def corpus(seed):
    models, _ = benchmark_models(seed)
    ids = {name: i+1 for i, name in enumerate([
        "Fanuc-CRX-5iA-Custom.robot", "Fanuc-CRX-10iA-Custom.robot",
        "Fanuc-CRX-10iA-L-Custom.robot", "Fanuc-CRX-20iA-L-Custom.robot",
        "Fanuc-CRX-25iA-Custom.robot", "Fanuc-CRX-30iA-Custom.robot"])}
    rng = np.random.default_rng(seed)
    groups = ["point", "track_seed", "track_local", "frames", "near_singular",
              "singular", "signed", "large_zero"]
    cases = []
    with mp.workdps(200):
        for model, scale, group in itertools.product(models, [.001, 1., 1000.], groups):
            lengths = (np.asarray(model["lengths"])*scale).tolist()
            height = float(model["robot"][10, 3]*scale)
            zeros = [0., -math.pi/2, 0., 0., 0., 0.]
            senses = [1., 1., -1., -1., -1., -1.]
            previous = rng.uniform(-70, 70, 6)
            command = previous.copy()
            reference, tool = frame(), frame()
            if group == "track_seed": command += .000001
            if group == "track_local": command += .01
            if group == "frames":
                reference = frame([.7, -.2, .3, .1], [scale*120, -scale*80, scale*45])
                tool = frame([.9, .1, -.3, .2], [-scale*35, scale*24, scale*92])
            if group in ["near_singular", "singular"]:
                previous[1] = command[1] = 0.
                previous[4] = command[4] = math.degrees(1e-8) if group == "near_singular" else 0.
            if group == "signed": lengths[1] *= -1
            if group == "large_zero": zeros[0] = 1e20
            before, before_q = source_target(lengths, zeros, senses, previous, reference, tool)
            target, centre = source_target(lengths, zeros, senses, command, reference, tool)
            cases.append({"name": f"{model['name']}:{scale:g}:{group}", "group": group,
                          "model_id": ids[model["name"]] if scale == 1 and group not in ["signed", "large_zero"] else -1,
                          "lengths": lengths, "height": height, "zeros": zeros, "senses": senses,
                          "limits": [-720.]*6+[720.]*6, "previous_target": before,
                          "target": target, "reference": reference, "tool": tool,
                          "previous_centre": before_q, "centre": centre, "seed": previous.tolist()})
        for model in models:
            a, b, c, r = model["lengths"]
            command = [0., 0., 90+math.degrees(math.atan2(r, b+c)), -90., 0., 0.]
            zeros = [0., -math.pi/2, 0., 0., 0., 0.]
            senses = [1., 1., -1., -1., -1., -1.]
            before, centre = source_target(model["lengths"], zeros, senses, command, frame(), frame())
            target = np.array(before)
            target[:3, 3] += .005*target[:3, 3]/np.linalg.norm(target[:3, 3])
            cases.append({"name": f"{model['name']}:1:approximate", "group": "approximate",
                          "model_id": ids[model["name"]], "lengths": list(model["lengths"]),
                          "height": float(model["robot"][10, 3]), "zeros": zeros, "senses": senses,
                          "limits": [-720.]*6+[720.]*6, "previous_target": before,
                          "target": target.tolist(), "reference": frame(), "tool": frame(),
                          "previous_centre": centre, "centre": centre, "seed": command})
    assert len(cases) == 150
    return cases


def packet(cases):
    result = [len(cases)]
    for item in cases:
        result += [item["model_id"], *item["lengths"], item["height"], *item["zeros"],
                   *item["senses"], *item["limits"]]
        for key in ["previous_target", "target", "reference", "tool"]:
            result += list(np.asarray(item[key]).flat)
        for key in ["previous_centre", "centre", "seed"]:
            result += item[key]
    return " ".join(format(float(value), ".17g") for value in result)


def words(row):
    bits = [int(getattr(row, f"w{i}"), 16) for i in range(77)]
    return [struct.unpack('<d', struct.pack('<Q', word))[0] for word in bits]


def raw_words(row):
    return [int(getattr(row, f"w{i}"), 16) for i in range(77)]


def validate(output, cases, rounds):
    word_types = {f"w{i}": str for i in range(77)}
    timing = pd.read_csv(output/"timings.csv", float_precision="round_trip")
    proofs = pd.read_csv(output/"certificates.csv", dtype=word_types, float_precision="round_trip")
    source = pd.read_csv(output/"inputs.csv", dtype=word_types)
    bounds = pd.read_csv(output/"enclosures.csv", float_precision="round_trip")
    domains = pd.read_csv(output/"domains.csv", float_precision="round_trip")
    commands = pd.read_csv(output/"commands.csv", float_precision="round_trip")
    assert len(source) == len(domains) == len(cases)
    assert set(source["case"]) == set(range(len(cases)))
    assert set(timing["case"]) == set(range(len(cases)))
    assert np.isfinite(timing.us).all() and timing.us.gt(0).all()
    assert not timing.duplicated(["case", "operation", "round"]).any()
    assert timing.groupby(["case", "operation"]).size().eq(rounds).all()
    outcome_columns = ["failure", "status", "count", "certified", "reused", "reuse_failure"]
    assert timing.groupby(["case", "operation"])[outcome_columns].nunique().eq(1).all().all()
    assert set(timing["round"]) == set(range(rounds))
    assert not proofs.duplicated(["case", "operation"]).any()
    assert np.isfinite(bounds[["lo", "hi"]]).all().all() and bounds.lo.le(bounds.hi).all()
    assert not bounds.duplicated(["case", "field", "component"]).any()
    bound_groups = {key: value.sort_values("component")[["lo", "hi"]].to_numpy()
                    for key, value in bounds.groupby(["case", "field"])}
    expected = {}
    with mp.workdps(200):
        for index, item in enumerate(cases):
            row = source[source["case"] == index].iloc[0]
            assert row.schema == 2
            raw = words(row)
            input = unflatten(raw)
            np.testing.assert_array_equal(input["lengths"], item["lengths"])
            assert input["base_height"] == item["height"]
            np.testing.assert_array_equal(input["encoder_zero"], item["zeros"])
            np.testing.assert_array_equal(input["senses"], item["senses"])
            np.testing.assert_array_equal(input["lower_commands"]+input["upper_commands"], np.asarray(item["limits"])*(math.pi/180))
            for key, field in [("target", "target"), ("base", "reference"), ("tool", "tool")]:
                matrix = np.asarray(item[field])
                np.testing.assert_array_equal(input[key]["position"], matrix[:3, 3])
                np.testing.assert_array_equal(input[key]["input_rotation"], matrix[:3, :3].ravel())
            value = exact_kinematics(input, item["centre"], reference_mode=True)
            expected[index] = (input, value)
            problem, rotation, translation, jacobian, axes, origins = value
            values = {"target_rotation": sum(problem["rotation"], []), "target_translation": problem["translation"],
                      "rotation": list(rotation), "translation": list(translation), "jacobian": list(jacobian)}
            values.update({f"axis{i}": list(vector) for i, vector in enumerate(axes)})
            values.update({f"origin{i}": list(vector) for i, vector in enumerate(origins)})
            for field, entries in values.items():
                box = bound_groups[index, field]
                assert len(box) == len(entries)
                for interval, exact in zip(box, entries, strict=True): contains(interval, exact)
            assert {key[1] for key in bound_groups if key[0] == index} == set(values)
            operations = {"prepare", "kinematics", "certify", "reuse"}
            if item["model_id"] >= 0: operations |= {"api_disabled", "api_enabled", "api_reuse", "api_parent"}
            assert set(timing[timing["case"] == index].operation) == operations
            assert set(proofs[proofs["case"] == index].operation) == operations-{"prepare", "kinematics"}
            assert set(commands[commands["case"] == index].operation) == {op for op in operations if op.startswith("api_")}
        for row in proofs.itertuples(index=False):
            assert row.certified in (0, 1) and row.reused in (0, 1)
            assert bool(row.certified) == (row.failure == 0)
            observed = timing[(timing["case"] == row.case) & timing.operation.eq(row.operation)].iloc[0]
            for name in outcome_columns: assert getattr(row, name) == observed[name], (row.case, row.operation, name)
            if not row.certified: continue
            input, derived = expected[row.case]
            assert row.schema == 2 and raw_words(row) == raw_words(source[source["case"] == row.case].iloc[0])
            centre = [getattr(row, f"q{i}") for i in range(6)]
            if centre != cases[row.case]["centre"]:
                derived = exact_kinematics(input, centre, reference_mode=True)
            assert row.scale == float(derived[0]["scale"])
            contains([row.det_lo, row.det_hi], mp.det(derived[3]))
            for name, squared in zip(["dev_target", "dev_reference", "dev_tool"],
                                      derived[0]["deviations_squared"], strict=True):
                bound = mp.mpf(getattr(row, name))
                assert bound >= 0 and bound**2 >= ref.number(squared)
            answer = {"failure": 0, "centre": centre,
                      "constants": [row.rho, row.radius, row.inverse_residual, row.beta, row.lipschitz,
                                    row.eta, row.defect, row.screw_sum, row.angular_sum, row.position_norm, row.sign],
                      "preconditioner": [getattr(row, f"x{i}") for i in range(36)],
                      "trace": [row.trace_lo, row.trace_hi], "coarse_eta": row.coarse_eta,
                      "used_log_series": bool(row.series)}
            check_certificate(answer, kinematics=derived)
            assert cases[row.case]["group"] != "approximate"
    for (index, operation), group in commands.groupby(["case", "operation"]):
        baseline = commands[(commands["case"] == index) & commands.operation.eq("api_disabled")]
        columns = ["status", "count", "truncated", "singular_representatives", *[f"q{i}" for i in range(6)]]
        np.testing.assert_array_equal(group[columns].to_numpy(), baseline[columns].to_numpy())
    medians = timing.groupby(["case", "operation"]).agg(us=("us", "median"), failure=("failure", "first"),
                 certified=("certified", "first"), reused=("reused", "first"), status=("status", "first"),
                 reuse_failure=("reuse_failure", "first")).reset_index()
    medians["group"] = [cases[i]["group"] for i in medians["case"]]
    medians.to_csv(output/"per-case.csv", index=False)
    summaries = {}
    for group in ["all", *dict.fromkeys(item["group"] for item in cases)]:
        selected = medians if group == "all" else medians[medians.group == group]
        summaries[group] = {operation: {"poses": len(rows), "certified": int(rows.certified.sum()),
            "reused": int(rows.reused.sum()), "failures": {str(k): int(v) for k, v in rows.failure.value_counts().items()},
            "reuse_failures_on_successful_proof": {str(k): int(v) for k, v in rows[rows.certified.eq(1)].reuse_failure.value_counts().items()},
            "statuses": {str(k): int(v) for k, v in rows.status.value_counts().items()},
            "timing_us": {name: float(rows.us.quantile(q)) for name, q in [("p50", .5), ("p95", .95), ("p99", .99), ("max", 1)]}}
            for operation, rows in selected.groupby("operation")}
    proofs["group"] = [cases[i]["group"] for i in proofs["case"]]
    conditioning = {}
    for (group, operation), rows in proofs[proofs.certified.eq(1)].groupby(["group", "operation"]):
        conditioning[f"{group}/{operation}"] = {"certificates": len(rows), **{
            name: {label: float(rows[name].quantile(q)) for label, q in [("p50", .5), ("p95", .95), ("max", 1)]}
            for name in ["rho", "radius", "beta", "lipschitz", "inverse_residual"]}}
    return {"groups": summaries, "conditioning": conditioning,
            "cache_eligibility": {"component_previous": int(domains.previous_valid.sum()),
                                  "api_previous": int(domains.api_previous_valid.sum())},
            "widths": {name: {label: float(domains[name].quantile(q)) for label, q in [("p50", .5), ("p95", .95), ("max", 1)]}
                       for name in ["rotation_width", "translation_width", "jacobian_width"]},
            "independent_validation": "Every prepared target/FK/axis/Jacobian box; every successful private/API ball and refined root; every ordered API command under all three modes."}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--dll", type=Path, default=ROOT/"build/Release/crx_kinematics.dll")
    parser.add_argument("--parent-dll", type=Path, required=True)
    parser.add_argument("--runner", type=Path, default=ROOT/"build/Release/crx_local_certificate_benchmark.exe")
    parser.add_argument("--compile-commands", type=Path, default=ROOT/"build/vscode/windows-clang-release-tidy/compile_commands.json")
    parser.add_argument("--sources", type=Path)
    parser.add_argument("--revision", default="working-tree")
    parser.add_argument("--rounds", type=int, default=9)
    parser.add_argument("--repeats", type=int, default=64)
    parser.add_argument("--seed", type=int, default=20261113)
    args = parser.parse_args()
    if min(args.rounds, args.repeats) < 1: parser.error("counts must be positive")
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    cases = corpus(args.seed)
    (output/"corpus.json").write_text(json.dumps(cases, indent=2)+"\n", encoding="utf-8")
    (output/"input.txt").write_text(packet(cases))
    shutil.copy2(args.runner, output/"runner.exe")
    shutil.copy2(args.dll, output/"candidate.dll")
    shutil.copy2(args.parent_dll, output/"parent.dll")
    shutil.copy2(args.compile_commands, output/"compile_commands.json")
    command = [str(output/"runner.exe"), str(output/"candidate.dll"), str(output/"parent.dll"), str(output/"input.txt"),
               str(output/"timings.csv"), str(output/"certificates.csv"), str(output/"domains.csv"),
               str(args.repeats), str(args.rounds)]
    with (output/"native.log").open("w") as log:
        subprocess.run(command, check=True, stderr=log)
    result = validate(output, cases, args.rounds)
    cpu = platform.processor()
    if os.name == "nt":
        import winreg
        with winreg.OpenKey(winreg.HKEY_LOCAL_MACHINE, r"HARDWARE\DESCRIPTION\System\CentralProcessor\0") as key:
            cpu = winreg.QueryValueEx(key, "ProcessorNameString")[0].strip()
    if args.sources:
        shutil.copy2(args.sources, output/"sources.zip")
    else:
        with zipfile.ZipFile(output/"sources.zip", "w", zipfile.ZIP_DEFLATED) as archive:
            for directory in ["src", "include", "python", "scripts", "tests", "docs"]:
                for path in sorted((ROOT/directory).rglob("*")):
                    if path.is_file() and "__pycache__" not in path.parts and path.suffix != ".pyc":
                        archive.write(path, path.relative_to(ROOT))
            for name in ["CMakeLists.txt", "CMakePresets.json", "crxkinematics.pro", "pyproject.toml", "uv.lock", ".clang-tidy", ".clang-format"]:
                archive.write(ROOT/name, name)
    result["metadata"] = {"measured_at_utc": datetime.now(UTC).isoformat(), "cpu": cpu,
        "platform": platform.platform(), "python": platform.python_version(), "numpy": np.__version__,
        "pandas": pd.__version__, "mpmath": mp.__version__, "source_identity": args.revision,
        "poses": len(cases), "api_poses": sum(item["model_id"] >= 0 for item in cases),
        "rounds": args.rounds, "repeats": args.repeats, "seed": args.seed,
        "method": "Serial pinned native batch means; per-case round medians; quantiles across cases. Three warmup calls plus per-batch untimed equality checks; shuffled case/operation order. Complete result consumption and volatile indirect calls.",
        "api_scope": "Complete DLL calls include immutable-proof allocation, snapshot reading and destruction. Disabled/enabled/reuse receive identical inputs; ordered commands checked against the parent's legacy C ABI with ordinary raw-frame composition included in its timed call. Private math preparation/certification/reuse are separate stages.",
        "domain": "Six captured geometries, three scales and eight groups, plus six approximate-boundary controls; no IK/certification-success selection. Signed/scaled/large-zero models unsupported by the builtin public model catalogue are explicit private-only rows.",
        "command": [sys.executable, *sys.argv], "native_command": command,
        "hashes": {p.name: bench.sha256(p) for p in output.iterdir() if p.is_file()}}
    (output/"summary.json").write_text(json.dumps(result, indent=2)+"\n", encoding="utf-8")
    print(json.dumps(result["groups"]["all"], indent=2))


if __name__ == "__main__":
    main()
