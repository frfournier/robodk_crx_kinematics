# SPDX-License-Identifier: Apache-2.0
# SPDX-FileCopyrightText: 2026 Centre Technologique en Aérospatiale
"""Frozen synthetic calibrated workloads, independent audits and wrapper costs.

Run with PYTHONPATH=python;scripts;tests in a synced repository environment.
The high precision oracle is the tracked, independently differentiated DH test
implementation. No private swarm files or installed RoboDK instance are used.
Preparation and verification are untimed. Run timing serially after all builds
and audits finish. Native component calls use production libraries; public API
costs use the explicitly loaded DLL. These are distinct measurements.
"""

import argparse
from concurrent.futures import ProcessPoolExecutor
import copy
import csv
import ctypes as ct
from datetime import UTC, datetime
import hashlib
import json
import math
import os
from pathlib import Path
import platform
import random
import struct
import subprocess
import time

import mpmath as mp
import numpy as np

from crx_kinematics import CalibratedFrame, CalibratedOptions, CalibratedSolver, CrxSolver
from crx_kinematics import _calibrated_native as native
from crx_asset_reference import derive_model
from test_crx_calibrated import ASSETS, _check_calibrated_certificate, _target_at

ROOT = Path(__file__).resolve().parents[1]
OPERATIONS = ("prepare_model", "prepare_solver", "prepare_target", "kinematics",
              "correct_disabled", "correct_enabled", "api_prepare", "api_disabled", "api_enabled")
SENSES = [1., 1., -1., -1., -1., -1.]


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def save(path, value):
    Path(path).write_text(json.dumps(value, indent=2, allow_nan=False)+"\n", encoding="utf-8")


def load(path):
    return json.loads(Path(path).read_text(encoding="utf-8"))


def target(case):
    return CalibratedFrame(case["target"][:4], case["target"][4:])


def solver(nominal, case):
    rows = case["rows"]
    return CalibratedSolver(nominal, [[r[1], r[2], r[4], r[5]] for r in rows],
        alpha_quarters=[r[0] for r in rows], theta_quarters=[r[3] for r in rows],
        senses=case["senses"], base=CalibratedFrame(case["base"][:4], case["base"][4:]),
        tool=CalibratedFrame(case["tool"][:4], case["tool"][4:]),
        joint_limits=[case["lower"], case["upper"]],
        limit_units="degrees" if case.get("limit_units", 0) else "radians", length_scale=case["scale"])


def nominal_solver(case, dll):
    return CrxSolver(case["nominal_model"], dll_path=dll,
        joint_limits=case["nominal_limits"], reference=case["reference"], tool=case["nominal_tool"])


def options(case, proofs=True):
    return CalibratedOptions(**case["options"], certify=proofs)


def records(case):
    value = native.ModelInput()
    for i, row in enumerate(case["rows"]):
        value.dh[i][:] = row[1], row[2], row[4], row[5]
    value.alpha_quarters[:] = [r[0] for r in case["rows"]]
    value.theta_quarters[:] = [r[3] for r in case["rows"]]
    value.senses[:] = case["senses"]
    value.base_quaternion[:], value.base_position[:] = case["base"][:4], case["base"][4:]
    value.tool_quaternion[:], value.tool_position[:] = case["tool"][:4], case["tool"][4:]
    value.lower_commands[:], value.upper_commands[:] = case["lower"], case["upper"]
    value.length_scale, value.limit_units = case["scale"], case.get("limit_units", 0)
    pose = native.Target()
    pose.quaternion[:], pose.position[:] = case["target"][:4], case["target"][4:]
    request = native.Options()
    opts = options(case)
    request.max_iterations, request.max_backtracks = opts.max_iterations, opts.max_backtracks
    request.maximum_step = opts.maximum_step_rad
    request.normalized_position_tolerance = opts.normalized_position_tolerance
    request.orientation_tolerance = opts.orientation_tolerance_rad
    request.uniqueness_radius = opts.uniqueness_radius_rad
    return value, pose, request


def quarter_rows(raw):
    result = []
    for alpha, a, theta, d in raw:
        aq, tq = int(round(alpha/(math.pi/2))), int(round(theta/(math.pi/2)))
        # Captured nominal twists/phases are exact quarter turns by the nominal
        # signature. Perturbations below remain separately represented radians.
        assert abs(alpha-aq*math.pi/2) < 1e-12 and abs(theta-tq*math.pi/2) < 1e-12
        result.append([aq, 0., float(a), tq, 0., float(d)])
    return result


def corpus(parent):
    cases = []
    witness = [30., -40., 45., 50., -35., 20.]
    groups = ["zero", "small", "frames", "near_singular", "tight_limits",
              "iteration_budget", "far_seed", "scaled", "senses", "turns"]
    for asset in ASSETS:
        raw = derive_model(asset)
        name = asset["asset"].removeprefix("Fanuc-").removesuffix("-Custom.robot")
        for group in groups:
            case = dict(name=f"{name}:{group}", group=group, synthetic=True,
                nominal_model=name, nominal_limits=[[-180.]*6, [180.]*6],
                reference=np.eye(4).tolist(), nominal_tool=np.eye(4).tolist(),
                rows=quarter_rows(raw["dh"]), senses=SENSES.copy(),
                base=[1., 0., 0., 0., *map(float, raw["base"][:3, 3])],
                tool=[1., 0., 0., 0., *map(float, raw["tool"][:3, 3])],
                lower=[-180.]*6, upper=[180.]*6, limit_units=1,
                scale=float(np.max(np.abs(raw["dh"][:, [1, 3]]))),
                options={}, operations=(1 << len(OPERATIONS))-1,
                auto=group in ["zero", "small", "frames"], source_asset=asset["asset"],
                source_asset_sha256=asset["sha256"])
            command = witness.copy()
            if group != "zero":
                for i, row in enumerate(case["rows"]):
                    row[1], row[4] = (i-2.5)*1e-5, (2.5-i)*1e-5
                    row[2] += (i+1)*.005
                    row[5] += (i-3)*.004
            if group == "frames":
                reference = CalibratedFrame([.9, .1, -.2, .15], [120., -80., 45.]).as_matrix()
                tcp = CalibratedFrame([.8, -.1, .2, .05], [-35., 24., 92.]).as_matrix()
                base = np.linalg.inv(reference)@raw["base"]
                tool = raw["tool"]@tcp
                b, t = CalibratedFrame.from_matrix(base), CalibratedFrame.from_matrix(tool)
                case.update(reference=reference.tolist(), nominal_tool=tcp.tolist(),
                    base=[*b.quaternion.tolist(), *b.position.tolist()],
                    tool=[*t.quaternion.tolist(), *t.position.tolist()])
            if group == "near_singular": command[4] = math.degrees(1e-8)
            if group == "senses": case["senses"] = [1., -1., 1., -1., 1., -1.]
            if group == "turns":
                command[0] += 720.
                case.update(lower=[-1080.]*6, upper=[1080.]*6)
            if group == "scaled":
                for row in case["rows"]: row[2], row[5] = row[2]*1e-3, row[5]*1e-3
                for frame in [case["base"], case["tool"]]: frame[4:] = [v*1e-3 for v in frame[4:]]
                case["scale"] *= 1e-3
            if group == "tight_limits":
                case.update(lower=[v-1e-12 for v in command], upper=[v+1e-12 for v in command])
            if group == "iteration_budget": case["options"] = dict(max_iterations=0)
            case["witness_degrees"] = command
            case["x"] = (np.radians(command)*case["senses"]).tolist()
            with mp.workdps(160): case["target"] = _target_at(case)
            if case["auto"]:
                with nominal_solver(case, parent) as nominal:
                    discovery = nominal.identify(target(case).as_matrix(), max_solutions=64)
                    case["seeds_degrees"] = discovery.solutions.tolist()
                case["seed_source"] = "actual F04 nominal identify, ordered and frozen"
            else:
                displacement = 80. if group == "far_seed" else .03
                case["seeds_degrees"] = [[v+displacement for v in command]]
                case["seed_source"] = "explicit synthetic nearby witness" if displacement == .03 else "explicit distant proposal"
            cases.append(case)
    empty = copy.deepcopy(cases[1])
    empty.update(name="empty", group="empty", auto=False, seeds_degrees=[],
        seed_source="explicit empty proposal set")
    cases.append(empty)
    cap = copy.deepcopy(cases[1])
    cap.update(name="seed_cap", group="seed_cap", auto=False,
        seeds_degrees=[witness]*512, seed_source="512 repeated explicit seeds; budget zero", options=dict(max_iterations=0))
    cases.append(cap)
    singular = copy.deepcopy(cases[1])
    singular.update(name="coaxial", group="coaxial", auto=False,
        rows=[[0, 0., 0., 0, 0., 0.] for _ in range(6)],
        base=[1., 0., 0., 0., 0., 0., 0.], tool=[1., 0., 0., 0., 0., 0., 0.],
        x=[0.]*6, seeds_degrees=[[0.]*6], seed_source="explicit singular coaxial chain")
    with mp.workdps(160): singular["target"] = _target_at(singular)
    singular["target"][4] += .01
    cases.append(singular)
    return cases


def source_files():
    selected = []
    for directory in ["include", "src", "python/crx_kinematics", "scripts", "tests"]:
        selected.extend(p for p in (ROOT/directory).rglob("*")
            if p.is_file() and p.suffix in [".h", ".cpp", ".py", ".json", ".csv"]
            and "__pycache__" not in p.parts)
    selected.extend(ROOT/name for name in ["CMakeLists.txt", "CMakePresets.json", ".clang-tidy", ".clang-format", "uv.lock"])
    return {p.relative_to(ROOT).as_posix(): sha(p) for p in sorted(selected)}


def prepare(args):
    directory = args.dataset.resolve()
    directory.mkdir(parents=True, exist_ok=False)
    cases = corpus(str(args.parent.resolve()))
    save(directory/"cases.json", cases)
    with (directory/"cases.bin").open("wb") as stream:
        stream.write(b"CRXCAL1\0"+struct.pack("<I", len(cases)))
        for case in cases:
            name = case["name"].encode("ascii")
            stream.write(struct.pack("<I", len(name))+name+struct.pack("<I", case["operations"]))
            for record in records(case): stream.write(bytes(record))
            seeds = np.radians(np.asarray(case["seeds_degrees"], dtype="<f8")).reshape(-1, 6)
            stream.write(struct.pack("<I", len(seeds))+seeds.tobytes())
    files = {name: sha(directory/name) for name in ["cases.json", "cases.bin"]}
    binaries = {key: dict(path=str(getattr(args, key).resolve()), sha256=sha(getattr(args, key)))
                for key in ["parent", "candidate", "driver"]}
    save(directory/"manifest.json", dict(schema=1, created=datetime.now(UTC).isoformat(),
        synthetic=True, cases=len(cases), seeds=sum(len(c["seeds_degrees"]) for c in cases),
        files=files, binaries=binaries, sources=source_files(),
        python=platform.python_version(), numpy=np.__version__, mpmath=mp.__version__,
        platform=platform.platform()))
    print(f"Prepared {len(cases)} frozen synthetic cases")


def guard(directory):
    manifest = load(directory/"manifest.json")
    assert manifest["sources"] == source_files(), "Source identity changed after dataset freeze"
    for name, digest in manifest["files"].items(): assert sha(directory/name) == digest, name
    for item in manifest["binaries"].values(): assert sha(item["path"]) == item["sha256"], item["path"]
    return manifest, load(directory/"cases.json")


def numeric_snapshot(result):
    records = []
    for row in result.attempts:
        records.append((int(row.status), row.commands_rad.tobytes(), row.increments_rad.tobytes(),
            struct.pack("<dd", row.normalized_position_error, row.orientation_error_rad),
            row.iterations, row.backtracks, row.evaluations))
    return records


def audit_certificate(job):
    case, row = job
    local = copy.deepcopy(case)
    local["x"] = [float(row[f"increment{i}"]) for i in range(6)]
    names = ["rho", "radius", "beta", "lipschitz", "eta", "inverse_residual", "screw_sum", "angular_sum", "position_norm", "command_reserve"]
    answer = {name: float(row[name]) for name in names}
    answer.update(certified=True, failure=0, admitted=bool(int(row["admitted"])),
        local_sign=int(row["local_sign"]), command_sign=int(row["command_sign"]),
        preconditioner=[[float(row[f"inverse{i*6+j}"]) for j in range(6)] for i in range(6)])
    with mp.workdps(160): _check_calibrated_certificate(local, answer)
    return case["name"], int(row["seed"])


def verify(args):
    directory = args.dataset.resolve()
    manifest, cases = guard(directory)
    subprocess.run([manifest["binaries"]["driver"]["path"], "verify", str(directory/"cases.bin"),
        manifest["binaries"]["candidate"]["path"], str(directory/"native")], check=True)
    with (directory/"native-audit.csv").open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    by_name = {case["name"]: case for case in cases}
    jobs, unique = [], set()
    for row in rows:
        if int(row["local"]):
            case = by_name[row["case"]]
            # Audit every distinct positive proof, including component/DLL
            # differences. Identical serialized evidence is checked once.
            key = (row["case"], *(row[name] for name in row if name not in ["case", "operation", "seed"]))
            if key not in unique:
                unique.add(key)
                jobs.append((case, row))
    with ProcessPoolExecutor(max_workers=args.workers) as pool:
        audited = list(pool.map(audit_certificate, jobs))
    checks = []
    for case in cases:
        with nominal_solver(case, manifest["binaries"]["candidate"]["path"]) as nominal, solver(nominal, case) as calibrated:
            frame = target(case)
            seeds = np.array(case["seeds_degrees"]).reshape(-1, 6)
            disabled = calibrated.correct(frame, seeds, options=options(case, False))
            enabled = calibrated.correct(frame, seeds, options=options(case, True))
            assert numeric_snapshot(disabled) == numeric_snapshot(enabled), case["name"]
            assert int(enabled.coverage) == int(disabled.coverage) == 2
            api = [row for row in rows if row["case"] == case["name"] and row["operation"] == "api_enabled"]
            assert len(api) == len(enabled.attempts)
            for row, attempt in zip(api, enabled.attempts, strict=True):
                assert int(row["status"]) == int(attempt.status)
                assert np.array([float(row[f"command{i}"]) for i in range(6)]).tobytes() == attempt.commands_rad.tobytes()
                assert tuple(int(row[n]) for n in ["iterations", "backtracks", "evaluations"]) == (attempt.iterations, attempt.backtracks, attempt.evaluations)
                assert bool(int(row["local"])) == (attempt.certificate is not None)
                assert bool(int(row["admitted"])) == bool(attempt.certificate and attempt.certificate.commands_admitted)
            if case["auto"]:
                automatic = calibrated.ik(frame, max_candidates=64, options=options(case))
                assert numeric_snapshot(automatic) == numeric_snapshot(enabled), case["name"]
            checks.append(dict(case=case["name"], seeds=len(seeds), numerical=enabled.numerical_count,
                local=enabled.local_count, admitted=enabled.admitted_count,
                iterations=enabled.iterations, backtracks=enabled.backtracks, evaluations=enabled.evaluations,
                coverage=int(enabled.coverage), payload=enabled.owned_native_payload_bytes))
    guard(directory)
    save(directory/"verification.json", dict(cases=len(cases), audit_rows=len(rows),
        distinct_positive_proofs=len(audited), precision_digits=160,
        proof_toggle_numeric_bits_and_work_equal=True, python_dll_commands_equal=True,
        automatic_frozen_parent_seeds_equal=True, snapshots=checks))
    print(f"Verified {len(cases)} cases and {len(audited)} distinct proofs at 160 digits")


def pin():
    if os.name != "nt": raise RuntimeError("This benchmark requires the supported Windows platform")
    kernel = ct.WinDLL("kernel32", use_last_error=True)
    kernel.GetCurrentProcess.restype = ct.c_void_p
    kernel.GetCurrentThread.restype = ct.c_void_p
    kernel.GetProcessAffinityMask.argtypes = [ct.c_void_p, ct.POINTER(ct.c_size_t), ct.POINTER(ct.c_size_t)]
    kernel.SetThreadAffinityMask.argtypes = [ct.c_void_p, ct.c_size_t]
    kernel.SetThreadAffinityMask.restype = ct.c_size_t
    process, system = ct.c_size_t(), ct.c_size_t()
    assert kernel.GetProcessAffinityMask(kernel.GetCurrentProcess(), ct.byref(process), ct.byref(system))
    bit = process.value & -process.value
    assert kernel.SetThreadAffinityMask(kernel.GetCurrentThread(), bit)
    return dict(process_affinity=process.value, thread_affinity=bit)


def measure_python(args, manifest, cases):
    rng = random.Random(args.seed)
    objects, configs = [], []
    def prepare_one(nominal, case):
        value = solver(nominal, case)
        value.close()
    for case in cases:
        nominal = nominal_solver(case, manifest["binaries"]["candidate"]["path"])
        calibrated = solver(nominal, case)
        objects.append((nominal, calibrated))
        frame, seeds = target(case), np.array(case["seeds_degrees"]).reshape(-1, 6)
        configs.append((case["name"], "python_prepare", lambda n=nominal, c=case: prepare_one(n, c)))
        for proofs in [False, True]:
            request = options(case, proofs)
            configs.append((case["name"], f"python_correct_{'enabled' if proofs else 'disabled'}",
                lambda c=calibrated, f=frame, s=seeds, o=request: c.correct(f, s, options=o)))
            if case["auto"]:
                configs.append((case["name"], f"python_ik_{'enabled' if proofs else 'disabled'}",
                    lambda c=calibrated, f=frame, o=request: c.ik(f, max_candidates=64, options=o)))
    def timed(function, repeats):
        before = time.perf_counter_ns()
        result = None
        for _ in range(repeats): result = function()
        return time.perf_counter_ns()-before, result
    repeats = []
    for _, _, function in configs:
        count = 1
        while timed(function, count)[0] < args.minimum_ms*1e6 and count < 2**20: count *= 2
        repeats.append(count)
    with (args.dataset/"python-timing.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(["round", "case", "operation", "repeats", "elapsed_ns", "ns_per_call"])
        order = list(range(len(configs)))
        for round_index in range(args.rounds):
            rng.shuffle(order)
            for i in order:
                name, operation, function = configs[i]
                elapsed, _ = timed(function, repeats[i])
                writer.writerow([round_index, name, operation, repeats[i], elapsed, elapsed/repeats[i]])
            stream.flush()
    for nominal, calibrated in objects: calibrated.close(); nominal.close()


def measure(args):
    directory = args.dataset.resolve()
    manifest, cases = guard(directory)
    assert (directory/"verification.json").is_file(), "Complete untimed verification first"
    assert not (directory/"measurement.json").exists(), "Do not overwrite a completed timing run"
    affinity = pin()
    started = datetime.now(UTC).isoformat()
    # Explicit subprocess boundaries serialize native and wrapper timing.
    subprocess.run([manifest["binaries"]["driver"]["path"], "measure", str(directory/"cases.bin"),
        manifest["binaries"]["candidate"]["path"], str(directory/"native-measured"),
        str(args.rounds), str(args.minimum_ms), str(args.seed)], check=True)
    measure_python(args, manifest, cases)
    guard(directory)
    save(directory/"measurement.json", dict(started=started, finished=datetime.now(UTC).isoformat(),
        rounds=args.rounds, minimum_ms=args.minimum_ms, random_seed=args.seed, **affinity,
        environment={name: os.environ.get(name) for name in ["OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS"]},
        files={name: sha(directory/name) for name in ["native-measured-timing.csv", "python-timing.csv"]}))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("mode", choices=["prepare", "verify", "measure"])
    parser.add_argument("--dataset", type=Path, required=True)
    parser.add_argument("--parent", type=Path)
    parser.add_argument("--candidate", type=Path)
    parser.add_argument("--driver", type=Path)
    parser.add_argument("--workers", type=int, default=3)
    parser.add_argument("--rounds", type=int, default=9)
    parser.add_argument("--minimum-ms", type=float, default=5.)
    parser.add_argument("--seed", type=int, default=20261113)
    args = parser.parse_args()
    if args.mode == "prepare" and not all([args.parent, args.candidate, args.driver]):
        parser.error("prepare requires --parent, --candidate and --driver")
    if not 1 <= args.workers <= 4 or not 1 <= args.rounds <= 100 or not 1 <= args.minimum_ms <= 1000:
        parser.error("workers=1..4, rounds=1..100, minimum-ms=1..1000")
    {"prepare": prepare, "verify": verify, "measure": measure}[args.mode](args)


if __name__ == "__main__":
    main()
