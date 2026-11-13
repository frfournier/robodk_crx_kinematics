# SPDX-License-Identifier: Apache-2.0
# SPDX-FileCopyrightText: 2026 Centre Technologique en Aérospatiale
"""Paired identification timing, with separate legacy/extended candidate paths.

Measures wall time through ctypes with preallocated outputs, not pure C++ time.
No wrapper postprocessing or instrumentation enters the timed call. Inputs are
a deterministic prefix of each asset/group in the frozen v1 solver corpus.
"""

import argparse
import ctypes as ct
import json
import platform
import time
from collections import Counter
from datetime import UTC, datetime
from pathlib import Path

import benchmark_solvers as bench
import numpy as np
import pandas as pd
from crx_kinematics._native import FibreExtension, FibreMetadata, NativeLibrary, check


def pin_thread():
    kernel = ct.WinDLL("kernel32", use_last_error=True)
    kernel.GetCurrentProcess.restype = ct.c_void_p
    kernel.GetCurrentThread.restype = ct.c_void_p
    kernel.GetProcessAffinityMask.argtypes = [
        ct.c_void_p, ct.POINTER(ct.c_size_t), ct.POINTER(ct.c_size_t),
    ]
    kernel.SetThreadAffinityMask.argtypes = [ct.c_void_p, ct.c_size_t]
    kernel.SetThreadAffinityMask.restype = ct.c_size_t
    process_mask, system_mask = ct.c_size_t(), ct.c_size_t()
    if not kernel.GetProcessAffinityMask(
        kernel.GetCurrentProcess(), ct.byref(process_mask), ct.byref(system_mask)
    ):
        raise ct.WinError(ct.get_last_error())
    affinity = process_mask.value & -process_mask.value
    if not kernel.SetThreadAffinityMask(kernel.GetCurrentThread(), affinity):
        raise ct.WinError(ct.get_last_error())
    return affinity


def run(args):
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    models, references, _ = bench.corpus(
        bench.DEFAULT_RANDOM_PER_MODEL, bench.DEFAULT_STRESS_PER_GROUP_MODEL,
        bench.DEFAULT_SEED,
    )
    corpus_path = bench.ROOT / "tests/fixtures/crx_solver_cases.json"
    cases = json.loads(corpus_path.read_text())
    model_path = output / "models.json"
    model_path.write_text(json.dumps([
        dict(name=model["name"], robot=model["robot"].tolist()) for model in models
    ], indent=2))
    selected, buckets = [], Counter()
    for index, case in enumerate(cases):
        key = (case["model"], case["group"])
        if buckets[key] < args.cases_per_group:
            selected.append(index)
            buckets[key] += 1
    libraries = [NativeLibrary(args.before), NativeLibrary(args.after)]
    handles, rows = [], []
    try:
        for library in libraries:
            names = {library.model_name(i).decode(): i for i in range(7)}
            model_handles = []
            for model in models:
                name = model["name"].removeprefix("Fanuc-").removesuffix("-Custom.robot")
                handle = ct.c_void_p()
                check(library.create(
                    names[name], np.ascontiguousarray(model["robot"][[30, 31], :6]),
                    ct.byref(handle),
                ))
                handles.append((library, handle))
                model_handles.append(handle)
            library.model_handles = model_handles
            if library.identify is None:
                raise RuntimeError("Both DLLs must export legacy identification")
        if libraries[1].identify2 is None:
            raise RuntimeError("Candidate must export extended identification")
        affinity = pin_thread()
        timings, outcomes = [], []
        paths = (("baseline", libraries[0], False),
                 ("candidate_legacy", libraries[1], False),
                 ("candidate_extended", libraries[1], True))
        for index in selected:
            case = cases[index]
            target = np.ascontiguousarray(case["target"])
            buffers = (
                np.empty((args.capacity, 6)), np.empty(args.capacity, np.int32),
                np.empty(args.capacity, np.int32),
                np.empty((args.capacity, 3), np.int32), np.empty(args.capacity, np.int32),
            )
            for label, library, extended in paths:
                metadata = FibreMetadata()
                extension = FibreExtension(ct.sizeof(FibreExtension), 1, 0, 0)
                arguments = (library.model_handles[case["model"]], target,
                             args.capacity, *buffers, ct.byref(metadata))
                function = library.identify2 if extended else library.identify
                if extended:
                    arguments += (ct.byref(extension),)
                rows.append((index, label, function, arguments, metadata, buffers,
                             extension))
        # Shuffle and alternate paths each round. Warmups and validation stay
        # outside recorded timings; all calls use the same immutable targets.
        rng = np.random.default_rng(20261113)
        for round_index in range(-1, args.rounds):
            for row_index in rng.permutation(len(rows)):
                index, label, function, arguments, metadata, buffers, extension = rows[row_index]
                repeats = 3 if round_index < 0 else args.repeats
                started = time.perf_counter_ns()
                for _ in range(repeats):
                    check(function(*arguments))
                elapsed_us = (time.perf_counter_ns() - started) / (1000 * repeats)
                if round_index >= 0:
                    timings.append(dict(case=index, path=label, round=round_index,
                                        us=elapsed_us, status=metadata.status,
                                        count=metadata.count))
        for index, label, function, arguments, metadata, buffers, extension in rows:
            check(function(*arguments))
            assert 0 <= metadata.count <= args.capacity
            case = cases[index]
            commands = buffers[0][:metadata.count].copy()
            poses = bench.reference_poses(references[case["model"]], commands)
            target = np.asarray(case["target"])
            position = np.linalg.norm(poses[:, :3, 3] - target[:3, 3], axis=1)
            chord = np.linalg.norm(poses[:, :3, :3] - target[:3, :3], axis=(1, 2))
            angle = np.degrees(2 * np.arcsin(np.minimum(1, chord / (2 * np.sqrt(2)))))
            lower, upper = models[case["model"]]["robot"][[30, 31], :6]
            allowance = 128 * np.finfo(float).eps * np.maximum(1, np.maximum(abs(lower), abs(upper)))
            invalid = int(np.count_nonzero(
                (position > bench.POSITION_MM) | (angle > bench.ANGLE_DEG)
                | ~((commands >= lower - allowance)
                    & (commands <= upper + allowance)).all(axis=1)
            ))
            outcomes.append(dict(case=index, name=case["name"], path=label,
                                 status=metadata.status, count=metadata.count,
                                 truncated=metadata.truncated, invalid=invalid))
        frame = pd.DataFrame(timings)
        frame.to_csv(output / "timings.csv", index=False)
        pd.DataFrame(outcomes).to_csv(output / "outcomes.csv", index=False)
        medians = frame.groupby(["case", "path"]).us.median().unstack()
        metadata = dict(
            measured_at_utc=datetime.now(UTC).isoformat(), platform=platform.platform(),
            cpu=platform.processor(), affinity_mask=affinity, rounds=args.rounds,
            repeats=args.repeats, capacity=args.capacity, cases=selected,
            method=__doc__, corpus_sha256=bench.sha256(corpus_path),
            model_sha256=bench.sha256(model_path),
            before_sha256=bench.sha256(args.before), after_sha256=bench.sha256(args.after),
            runner_sha256=bench.sha256(Path(__file__)),
            summaries={label: {name: float(medians[label].quantile(fraction))
                               for name, fraction in (("p50_us", .5), ("p95_us", .95),
                                                      ("p99_us", .99), ("max_us", 1))}
                       for label, _, _ in paths},
        )
        (output / "summary.json").write_text(json.dumps(metadata, indent=2))
        print(json.dumps(metadata["summaries"], indent=2))
        if any(row["invalid"] for row in outcomes):
            raise RuntimeError("Identification returned invalid commands")
    finally:
        for library, handle in handles:
            library.destroy(handle)
        for library in libraries:
            library.close_directory()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--before", type=Path, required=True)
    parser.add_argument("--after", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--rounds", type=int, default=9)
    parser.add_argument("--repeats", type=int, default=8)
    parser.add_argument("--capacity", type=int, default=512)
    parser.add_argument("--cases-per-group", type=int, default=4)
    arguments = parser.parse_args()
    if min(arguments.rounds, arguments.repeats, arguments.capacity,
           arguments.cases_per_group) <= 0:
        parser.error("rounds, repeats, capacity and cases-per-group must be positive")
    run(arguments)
