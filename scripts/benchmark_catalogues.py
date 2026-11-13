# SPDX-License-Identifier: Apache-2.0
# SPDX-FileCopyrightText: 2026 Centre Technologique en Aérospatiale
"""Reproducible isolated stage and actual DLL catalogue-query benchmarks.

Default is untimed verification. --time must run with other builds, tests and
benchmarks stopped. Native stages exclude scratch copying/cleanup and checksum
validation; actual C API measurements include query/summary/destruction. Clock
overhead is retained. This does not substitute for the ordinary-IK regression
and matched refinement-counter comparisons in benchmark_solvers.py.
"""

import argparse
from contextlib import ExitStack
from datetime import UTC, datetime
import hashlib
import json
import os
from pathlib import Path
import platform
import random
import shutil
import subprocess
import sys
import time
import zipfile

import mpmath as mp
import numpy as np
import pandas as pd

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'tests'))
from crx_kinematics import CatalogueOptions, CommandProfile, CrxSolver
import crx_catalogue_reference as oracle
from test_crx_command_certificate import commands as command_map
from test_crx_exact_target import flatten, pose, unflatten
from test_crx_root_certificate import witness
from validate_catalogues import corpus


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def raw_context(lower=None, upper=None, seed=None):
    return dict(lower=[-180.]*6 if lower is None else list(lower),
        upper=[180.]*6 if upper is None else list(upper),
        seed=[0.]*6 if seed is None else list(seed), weights=[1., 2., .25, 4., .5, 3.])


def generate(candidate, seed, small):
    inputs, _ = corpus(64, seed)
    cases = []
    def append(name, group, source, raw, **kwargs):
        item = dict(name=name, group=group, source=source, raw=raw, mode=1,
            model_id=-1, capacity=64, profile=0, seeded=1,
            original_limits=[-180.]*6+[180.]*6, root_options=[8192, 40, 4096, 8],
            candidate_budget=512, pair_budget=120, lift_budget=1 << 20)
        item.update(kwargs)
        item['id'] = len(cases)
        cases.append(item)
    for model_id, asset in enumerate(inputs, start=1):
        with CrxSolver(asset['model'], dll_path=candidate, joint_limits=asset['limits']) as solver:
            indices = [0, 63] if small else [0, 7, 24, 40, 63]
            for index in indices:
                target, command_seed = asset['poses'][index], asset['seeds'][index]
                query = solver.catalogue(target, seed=command_seed, max_solutions=0)
                try: source = unflatten(query.summary.input_words.view(np.float64).tolist())
                finally: query.close()
                capacities = [1] if small else [1, 64, 512]
                for capacity in capacities:
                    append(f"{asset['model']}:{index}:cap{capacity}", 'captured_singular' if index < 8 else 'random',
                        source, raw_context(*asset['limits'], command_seed), mode=2, model_id=model_id,
                        original_limits=asset['limits'].ravel().tolist(), capacity=capacity)
            target, command_seed = asset['poses'][-1], asset['seeds'][-1].copy()
            append(f"{asset['model']}:unseeded", 'unseeded', source,
                raw_context(*asset['limits']), mode=2, model_id=model_id,
                original_limits=asset['limits'].ravel().tolist(), seeded=0)
            command_seed[2] -= command_seed[1]
            query = solver.catalogue(target, seed=command_seed, max_solutions=0,
                profile=CommandProfile.DECOUPLED_USER, joint_limits=np.array([[-180.]*6, [180.]*6]))
            try: source = unflatten(query.summary.input_words.view(np.float64).tolist())
            finally: query.close()
            append(f"{asset['model']}:decoupled", 'decoupled', source, raw_context(seed=command_seed),
                mode=2, model_id=model_id, original_limits=asset['limits'].ravel().tolist(), profile=1)
    append('sixteen', 'sixteen', witness(), raw_context())
    append('root_work', 'root_work', witness(), raw_context(), root_options=[0, 40, 4096, 8])
    append('candidate_work', 'candidate_work', witness(), raw_context(), candidate_budget=0)
    append('pair_work', 'pair_work', witness(), raw_context(), pair_budget=0)
    append('lift_work', 'lift_work', witness(), raw_context(), lift_budget=0)
    append('multiple_turns', 'multiple_turns', witness(), raw_context([-360.]*6, [360.]*6))
    append('travel_overflow', 'travel_overflow', witness(), raw_context(seed=[1e308]*6))
    for name, quaternion in [('seam', [1., -1., -1., -1.]), ('infinity', [1., -1., 0., 0.])]:
        source = witness()
        source['lengths'] = [1., .5, .25, .125]
        source['target'] = pose(quaternion, [1.125, .25, -.5])
        append(name, name, source, raw_context())
    zero = witness()
    zero['lengths'] = [1., 1., 0., 0.]
    zero['target'] = pose()
    append('zero_polynomial', 'zero_polynomial', zero, raw_context())
    far = witness()
    far['target']['position'] = [20., 30., -40.]
    append('no_roots', 'no_roots', far, raw_context())
    overflow = witness()
    overflow['target']['position'] = [1e100, -1e100, 1e100]
    append('polynomial_overflow', 'polynomial_overflow', overflow, raw_context())
    reference = oracle.exhaustive(witness())
    if reference.outcome != 'finite': raise RuntimeError('Independent sixteen-phase witness unresolved')
    with mp.workdps(200):
        centre = [float(x*180/mp.pi) for x in command_map(witness(), reference.configurations[0], 0)]
    lower, upper = [x-.01 for x in centre], [x+.01 for x in centre]
    lower[0] = upper[0] = centre[0]
    append('boundary', 'boundary', witness(), raw_context(lower, upper, centre))
    lower, upper, middle = [x-.01 for x in centre], [x+.01 for x in centre], centre.copy()
    lower[0], upper[0], middle[0] = centre[0]-1, centre[0]+361, centre[0]+180
    append('possible_minimizers', 'possible_minimizers', witness(), raw_context(lower, upper, middle))
    return cases


def packets(cases):
    rows = []
    for item in cases:
        raw = item['raw']
        values = [item['id'], item['mode'], item['model_id'], item['profile'], item['seeded'], item['capacity'],
            *item['root_options'], item['candidate_budget'], item['pair_budget'], item['lift_budget']]
        floats = [*item['original_limits'], *flatten(item['source']), *raw['lower'], *raw['upper'], *raw['seed'], *raw['weights']]
        rows.append(' '.join([*map(str, values), *(repr(float(x)) for x in floats)]))
    return '\n'.join(rows)+'\n'


def summarize(output, cases, timed, rounds):
    outcomes = pd.read_csv(output/'native/outcomes.csv')
    samples = pd.read_csv(output/'native/timings.csv')
    if outcomes.duplicated(['case', 'operation']).any(): raise RuntimeError('Duplicate outcome rows')
    if set(outcomes['case']) != set(range(len(cases))): raise RuntimeError('Missing benchmark cases')
    if timed:
        if samples.empty or not np.isfinite(samples[['us', 'total_ns']].to_numpy()).all() or (samples['us'] <= 0).any():
            raise RuntimeError('Invalid timing samples')
        counts = samples.groupby(['case', 'operation']).size()
        if not (counts == rounds).all() or len(counts) != len(outcomes): raise RuntimeError('Incomplete timed population')
    elif not samples.empty: raise RuntimeError('Verification unexpectedly collected timings')
    summary = dict(case_count=len(cases), eligible_operations=len(outcomes), timed=timed,
        timing_method='Per-call steady_clock boundaries; reset/copy/cleanup/consumption outside; clock overhead retained',
        native_api_scope='Actual DLL query + summary + destroy; model construction excluded',
        stage_implementation='Shipped strict command source in separate stage-only object, no runtime timing hooks',
        ordinary_ik_regression='Requires separate paired complete-DLL and counter comparisons',
        outcomes=outcomes.groupby('operation').size().to_dict(), groups={})
    if timed:
        medians = samples.groupby(['case', 'operation'], as_index=False)['us'].median()
        descriptions = pd.DataFrame([dict(case=item['id'], group=item['group'], capacity=item['capacity']) for item in cases])
        medians = medians.merge(descriptions, on='case', validate='many_to_one')
        for keys, frame in medians.groupby(['operation', 'group', 'capacity']):
            operation, group, capacity = keys
            summary['groups'][f'{operation}:{group}:cap{capacity}'] = dict(count=len(frame),
                p50_us=float(frame.us.quantile(.5)), p95_us=float(frame.us.quantile(.95)), max_us=float(frame.us.max()))
        medians.to_csv(output/'case-medians.csv', index=False)
    return summary


def matrix(source):
    result = np.eye(4)
    result[:3, :3] = np.array(source['input_rotation']).reshape(3, 3)
    result[:3, 3] = source['position']
    return result


def python_queries(candidate, cases, output, timed, rounds, minimum_ms):
    """Time the shipped typed wrapper separately from native stage costs."""
    rows, prepared = [], []
    def signature(result):
        summary = result.summary
        return (int(summary.coverage), int(summary.ranking), summary.root_count,
            summary.phase_count, summary.command_count, summary.yes, summary.undecided,
            int(summary.numerical_status), summary.numerical_count, summary.numerical_truncated)
    with ExitStack() as stack:
        solvers = {}
        for item in cases:
            if item['model_id'] < 0 or item['mode'] != 2: continue
            source, raw = item['source'], item['raw']
            reference, tool = matrix(source['base']), matrix(source['tool'])
            key = item['model_id'], tuple(item['original_limits']), reference.tobytes(), tool.tobytes()
            if key not in solvers:
                names = ['CRX-3iA', 'CRX-5iA', 'CRX-10iA', 'CRX-10iA-L', 'CRX-20iA-L', 'CRX-25iA', 'CRX-30iA']
                solvers[key] = stack.enter_context(CrxSolver(names[item['model_id']], dll_path=candidate,
                    joint_limits=np.array(item['original_limits']).reshape(2, 6), reference=reference, tool=tool))
            solver = solvers[key]
            target = matrix(source['target'])
            arguments = dict(seed=np.array(raw['seed']) if item['seeded'] else None, weights=np.array(raw['weights']),
                joint_limits=np.array([raw['lower'], raw['upper']]), profile=CommandProfile(item['profile']), max_solutions=item['capacity'])
            options = dict(max_nodes=item['root_options'][0], max_depth=item['root_options'][1],
                max_leaves=item['root_options'][2], max_chart_attempts=item['root_options'][3],
                max_candidate_certificates=item['candidate_budget'], max_pair_tests=item['pair_budget'], max_lifts=item['lift_budget'])
            numerical = None
            for enabled in [False, True]:
                requested = CatalogueOptions(enabled=enabled, **options)
                result = solver.catalogue(target, options=requested, **arguments)
                expected, array = signature(result), result.numerical_solutions.copy()
                result.close()
                if enabled and not np.array_equal(array, numerical): raise RuntimeError('Enabled/disabled numerical fallback differs')
                numerical = array
                prepared.append((item['id'], 'python_enabled' if enabled else 'python_disabled', solver,
                                 target, arguments, requested, expected, array))
        # Verification does not enter a clock branch. All result/cache checks
        # happen outside timing; only query construction and native close count.
        if timed:
            generator = random.Random(20261113)
            order = list(range(len(prepared)))
            for round in range(-1, rounds):
                generator.shuffle(order)
                for index in order:
                    ident, operation, solver, target, arguments, options, expected, numerical = prepared[index]
                    elapsed, iterations = 0, 0
                    budget_ns = 0 if round < 0 else minimum_ms*1e6
                    while True:
                        result = None # Snapshot destruction excluded from the next call.
                        start = time.perf_counter_ns()
                        result = solver.catalogue(target, options=options, **arguments)
                        result.close()
                        stop = time.perf_counter_ns()
                        elapsed += stop-start
                        if signature(result) != expected or not np.array_equal(result.numerical_solutions, numerical):
                            raise RuntimeError('Typed wrapper outcome changed during timing')
                        iterations += 1
                        if elapsed >= budget_ns or iterations >= 16384: break
                    if round >= 0:
                        rows.append(dict(case=ident, operation=operation, round=round, iterations=iterations,
                                         total_ns=elapsed, us=elapsed/iterations/1000))
    frame = pd.DataFrame(rows, columns=['case', 'operation', 'round', 'iterations', 'total_ns', 'us'])
    frame.to_csv(output/'python-timings.csv', index=False)
    return dict(eligible_operations=len(prepared), numerical_parity=True,
        scope='Typed wrapper query, snapshots and native cache close; array destruction and validation outside timing')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--candidate', type=Path, required=True)
    parser.add_argument('--parent', type=Path, required=True)
    parser.add_argument('--parent-revision', help='Verified commit corresponding to the parent binary')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--driver', type=Path, default=ROOT/'build/Release/crx_catalogue_benchmark.exe')
    parser.add_argument('--compile-commands', type=Path, default=ROOT/'build/vscode/windows-clang-release-tidy/compile_commands.json')
    parser.add_argument('--seed', type=int, default=20261113)
    parser.add_argument('--rounds', type=int, default=7)
    parser.add_argument('--minimum-ms', type=float, default=5.)
    parser.add_argument('--time', action='store_true')
    parser.add_argument('--small', action='store_true', help='Smaller verification corpus; do not use for the final timing report')
    args = parser.parse_args()
    if args.time and args.small: parser.error('--small is verification only')
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    if any(output.iterdir()): parser.error('Output must be new/empty')
    cases = generate(args.candidate.resolve(), args.seed, args.small)
    (output/'cases.json').write_text(json.dumps(cases, indent=2)+'\n')
    (output/'packets.txt').write_text(packets(cases))
    binaries = output/'binaries'
    binaries.mkdir()
    for name, source in [('candidate.dll', args.candidate), ('parent.dll', args.parent), ('driver.exe', args.driver)]:
        shutil.copy2(source, binaries/name)
    sources = [*sorted((ROOT/'src').glob('*.[ch]*')), *sorted((ROOT/'include').glob('*.h')),
        *sorted((ROOT/'python/crx_kinematics').glob('*.py')), Path(__file__), ROOT/'scripts/benchmark_catalogues.cpp',
        ROOT/'scripts/catalogue_command_stages.cpp', ROOT/'scripts/catalogue_command_stages.h',
        ROOT/'scripts/validate_catalogues.py', ROOT/'CMakeLists.txt', ROOT/'CMakePresets.json',
        ROOT/'pyproject.toml', ROOT/'uv.lock', ROOT/'tests/fixtures/crx_asset_frames.json', *sorted((ROOT/'tests').glob('*.py'))]
    source_hashes = {path.relative_to(ROOT).as_posix(): sha(path) for path in sources}
    compile_commands = json.loads(args.compile_commands.read_text())
    selected = [entry for entry in compile_commands if any(label in entry['command'] for label in [
        'crx_catalogue_benchmark.dir', 'crx_catalogue_benchmark_commands.dir', 'crx_kinematics.dir'])]
    if any('CRXKIN_DISCOVERY_COUNTERS' in entry['command'] for entry in selected): raise RuntimeError('Counter instrumentation in a timed target')
    with zipfile.ZipFile(output/'sources.zip', 'w', zipfile.ZIP_DEFLATED) as archive:
        for path in sources: archive.write(path, path.relative_to(ROOT).as_posix())
    (output/'compile_commands.json').write_text(json.dumps(selected, indent=2)+'\n')
    binary_hashes = {path.name: sha(path) for path in binaries.iterdir()}
    metadata = dict(created_utc=datetime.now(UTC).isoformat(), timed=args.time,
        parent_revision=args.parent_revision,
        candidate_original_path=str(args.candidate.resolve()), parent_original_path=str(args.parent.resolve()),
        driver_original_path=str(args.driver.resolve()),
        seed=args.seed, rounds=args.rounds, minimum_ms=args.minimum_ms,
        cpu=platform.processor(), logical_cpus=os.cpu_count(), platform=platform.platform(),
        binary_hashes=binary_hashes, source_hashes=source_hashes,
        corpus_sha=sha(output/'cases.json'), packets_sha=sha(output/'packets.txt'),
        warmup='one untimed pass plus a randomized warmup round', case_count=len(cases))
    (output/'metadata.json').write_text(json.dumps(metadata, indent=2)+'\n')
    command = [str(binaries/'driver.exe'), '--time' if args.time else '--verify', str(output/'packets.txt'),
        str(binaries/'candidate.dll'), str(binaries/'parent.dll'), str(output/'native'), str(args.rounds), str(args.minimum_ms)]
    with (output/'native.log').open('w') as log:
        subprocess.run(command, check=True, stdout=log, stderr=subprocess.STDOUT)
    wrapper = python_queries(binaries/'candidate.dll', cases, output, args.time, args.rounds, args.minimum_ms)
    if binary_hashes != {path.name: sha(path) for path in binaries.iterdir()} or any(sha(ROOT/name) != value for name, value in source_hashes.items()):
        raise RuntimeError('Benchmark source/binary inputs changed')
    result = summarize(output, cases, args.time, args.rounds)
    result['python'] = wrapper
    if args.time:
        frame = pd.read_csv(output/'python-timings.csv')
        grouped = frame.groupby(['case', 'operation'], as_index=False)['us'].median()
        grouped.to_csv(output/'python-case-medians.csv', index=False)
        result['python']['populations'] = {operation: dict(count=len(values), p50_us=float(values.us.quantile(.5)),
            p95_us=float(values.us.quantile(.95)), max_us=float(values.us.max())) for operation, values in grouped.groupby('operation')}
    (output/'summary.json').write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps(dict(output=str(output), cases=len(cases), eligible_operations=result['eligible_operations'], timed=args.time)), flush=True)


if __name__ == '__main__':
    main()
