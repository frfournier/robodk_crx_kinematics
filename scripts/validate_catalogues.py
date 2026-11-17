# SPDX-License-Identifier: Apache-2.0
# SPDX-FileCopyrightText: 2026 Centre Technologique en Aérospatiale
"""Offline exhaustive nominal-query acceptance on every captured CRX asset.

Run with uv from the repository root. Default: 1000 targets for each of six
captured assets. Cases use captured/independent modified-DH targets, never native
IK outputs. Exact rational Sturm isolation and escalating high-precision full
reconstruction independently audit each claim. Unresolved oracle cases remain
unverified, not agreement. Output is resumable only for an identical corpus,
DLL, runner, oracle and wrapper snapshot. This is not a performance benchmark.
"""

from __future__ import annotations

import argparse
from collections import Counter
from concurrent.futures import ProcessPoolExecutor, as_completed
import hashlib
import itertools
import json
from pathlib import Path
import platform
import sys

import mpmath as mp
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'tests'))
from crx_kinematics import Admission, Coverage, CrxSolver, Ranking
from crx_asset_reference import derive_model, forward_models
from crx_assets import APPROVED_ROBOT_ASSETS
import crx_catalogue_reference as oracle
from test_crx_exact_target import unflatten
from test_crx_command_certificate import commands as command_map


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def digest(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(',', ':')).encode()).hexdigest()


def corpus(per_asset, seed):
    capture_path = ROOT/'tests/fixtures/crx_asset_frames.json'
    capture = json.loads(capture_path.read_text())
    if capture['mode'] != 'nominal' or capture['command_unit'] != 'degree':
        raise ValueError('Expected the approved nominal degree capture')
    assets = capture['assets']
    if {a['asset'] for a in assets} != set(APPROVED_ROBOT_ASSETS):
        raise ValueError('Captured asset inventory changed')
    result = []
    for number, asset in enumerate(assets):
        name = asset['asset']
        if sha(ROOT/'assets'/name) != asset['sha256'] or asset['sha256'] != APPROVED_ROBOT_ASSETS[name]:
            raise ValueError(f'Asset bytes do not match approved capture: {name}')
        rng = np.random.default_rng(seed+number)
        limits = np.array([asset['lower_limits_deg'], asset['upper_limits_deg']])
        commands = rng.uniform(limits[0], limits[1], size=(per_asset, 6))
        observations = [asset['zero'], *asset['probes'], *asset['samples']]
        for index, observation in enumerate(observations[:per_asset]):
            commands[index] = observation['commands_deg']
        poses = forward_models(derive_model(asset), commands)
        for index, observation in enumerate(observations[:per_asset]):
            poses[index] = observation['flange_pose']
        seeds = np.clip(commands+rng.uniform(-60, 60, commands.shape), limits[0], limits[1])
        model = name.removeprefix('Fanuc-').removesuffix('-Custom.robot')
        result.append(dict(asset=name, asset_sha=asset['sha256'], model=model,
                           limits=limits, commands=commands, poses=poses, seeds=seeds))
    return result, sha(capture_path)


def contains(box, value):
    if not mp.mpf(float(box[0])) <= value <= mp.mpf(float(box[1])):
        raise AssertionError('An independently reconstructed root/cost escaped its enclosure')


def check_case(query, target):
    summary = query.summary
    if summary.identity_schema != 2:
        return dict(oracle='unresolved', reason='Exact preparation unavailable', verified=False)
    exact_input = unflatten(summary.input_words.view(np.float64).tolist())
    if not np.array_equal(exact_input['target']['input_rotation'], target[:3, :3].ravel()) or not np.array_equal(exact_input['target']['position'], target[:3, 3]):
        raise AssertionError('Query source identity does not retain the requested raw target')
    reference = oracle.exhaustive(exact_input, reference=True)
    result = dict(oracle=reference.outcome, oracle_wrist_roots=reference.wrist_roots,
                  precision=reference.precision, attempts=reference.attempts,
                  reason=reference.reason, verified=False)
    if reference.outcome in ('exceptional', 'degenerate'):
        if summary.coverage == Coverage.COMPLETE or query.nearest_certified or summary.command_infeasible:
            raise AssertionError('Finite/global claim on an exact exceptional oracle stratum')
        return result
    if reference.outcome != 'finite':
        return result
    if summary.coverage == Coverage.COMPLETE:
        if summary.root_count != reference.wrist_roots or summary.phase_count != len(reference.configurations):
            raise AssertionError('Complete phase count disagrees with exhaustive oracle')
    raw = summary.raw_words.view(np.float64).reshape(4, 6)
    with mp.workdps(max(200, reference.precision)):
        lower, upper, seed = [[mp.mpf(float(x))*mp.pi/180 for x in values] for values in raw[:3]]
        weights = [mp.mpf(float(x)) for x in raw[3]]
        base_roots, matched = [], []
        for ball in query.phases:
            indices = [i for i, q in enumerate(reference.configurations) if max(
                oracle.circle_distance(x, mp.mpf(float(c))) for x, c in zip(q, ball.canonical_center_rad, strict=True)) <= ball.existence_radius_rad]
            if len(indices) != 1:
                raise AssertionError('Local existence ball does not match exactly one independently reconstructed root')
            matched += indices
            q = [x+2*mp.pi*mp.nint((mp.mpf(float(c))-x)/(2*mp.pi))
                 for x, c in zip(reference.configurations[indices[0]], ball.canonical_center_rad, strict=True)]
            base_roots.append(command_map(exact_input, q, summary.profile))
        if summary.coverage == Coverage.COMPLETE and sorted(matched) != list(range(len(reference.configurations))):
            raise AssertionError('Complete catalogue duplicated/omitted exact phases')
        if summary.enumeration_complete:
            rows = [query.command(i) for i in range(summary.command_count)]
            potential = {(row.phase, tuple(map(int, row.turns))): (i, row) for i, row in enumerate(rows)}
            if len(potential) != len(rows): raise AssertionError('Duplicate command-lift identity')
            feasible = {}
            for phase, base in enumerate(base_roots):
                sets = [range(int(mp.ceil((lo-x)/(2*mp.pi))), int(mp.floor((hi-x)/(2*mp.pi)))+1)
                        for lo, hi, x in zip(lower, upper, base, strict=True)]
                for turns in itertools.product(*sets):
                    key = phase, turns
                    if key not in potential: raise AssertionError('Feasible exact command omitted from complete enumeration')
                    feasible[key] = sum(w*(x+2*mp.pi*m-s)**2 for w, x, m, s in zip(weights, base, turns, seed, strict=True))
            if summary.admission_complete and len(feasible) != summary.yes:
                raise AssertionError('Resolved command-admission count is wrong')
            for key, (_, row) in potential.items():
                values = [x+2*mp.pi*m for x, m in zip(base_roots[row.phase], row.turns, strict=True)]
                for box, value in zip(row.radians, values, strict=True): contains(box, value)
                if row.admission == Admission.YES and key not in feasible:
                    raise AssertionError('Certified Yes root violates original exact degree limits')
                if row.travel_squared is not None:
                    contains(row.travel_squared, sum(w*(x-s)**2 for w, x, s in zip(weights, values, seed, strict=True)))
            if summary.ranking in (Ranking.UNIQUE, Ranking.POSSIBLE_MINIMIZERS):
                minimum = min(feasible.values())
                if any(not rows[potential[key][0]].possible_minimizer for key, value in feasible.items() if value == minimum):
                    raise AssertionError('Global ranking omitted an actual nearest exact command')
            if summary.ranking == Ranking.UNIQUE:
                if rows[0].admission != Admission.YES or any(rows[0].travel_squared[1] >= row.travel_squared[0] for row in rows[1:]):
                    raise AssertionError('Strict nearest separation is not established')
        # Retain full high-precision reconstruction rather than merely counts.
        result['configurations'] = [[mp.nstr(x, reference.precision) for x in q] for q in reference.configurations]
    result['verified'] = True
    return result


def validate_batch(library, asset, model, limits, batch):
    """Each process owns its DLL/model and mpmath context; no shared FP state."""
    results = []
    with CrxSolver(model, dll_path=library, joint_limits=limits) as solver:
        for index, target, seed in batch:
            query = solver.catalogue(target, seed=seed, max_solutions=0)
            summary = query.summary
            row = dict(asset=asset, index=index, coverage=int(summary.coverage),
                phase_count=summary.phase_count, root_count=summary.root_count,
                root_nodes=summary.root_nodes, root_leaf_count=summary.root_leaf_count.tolist(),
                max_depth=max(query.chart(i).max_depth for i in range(2)),
                ranking=int(summary.ranking), admission_complete=summary.admission_complete,
                yes=summary.yes, no=summary.no, undecided=summary.undecided,
                command_count=summary.command_count, nominal_infeasible=summary.nominal_infeasible,
                command_infeasible=summary.command_infeasible,
                input_words=[format(int(x), 'x') for x in summary.input_words],
                raw_words=[format(int(x), 'x') for x in summary.raw_words],
                balls=[dict(centre=ball.canonical_center_rad.tolist(), rho=ball.existence_radius_rad,
                            R=ball.uniqueness_radius_rad) for ball in query.phases])
            try:
                row.update(check_case(query, target))
            except Exception as error:
                row.update(error=f'{type(error).__name__}: {error}', verified=False)
            finally:
                query.close()
            results.append(row)
    return results


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--library', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--per-asset', type=int, default=1000)
    parser.add_argument('--seed', type=int, default=20261113)
    parser.add_argument('--resume', action='store_true')
    parser.add_argument('--jobs', type=int, default=1, help='Independent audit processes, 1..8; no timing measurements')
    args = parser.parse_args()
    if not 1 <= args.per_asset <= 1000: parser.error('--per-asset must be in [1,1000]')
    if not 1 <= args.jobs <= 8: parser.error('--jobs must be in [1,8]')
    cases, capture_sha = corpus(args.per_asset, args.seed)
    arrays = {f'{i}_{name}': item[name] for i, item in enumerate(cases) for name in ['limits', 'commands', 'poses', 'seeds']}
    fingerprints = {name: hashlib.sha256(value.astype('<f8').tobytes()).hexdigest() for name, value in arrays.items()}
    source_paths = [Path(__file__), *sorted((ROOT/'python/crx_kinematics').glob('*.py')),
        *sorted((ROOT/'tests').glob('*.py')), ROOT/'pyproject.toml', ROOT/'uv.lock']
    manifest = dict(schema=1, per_asset=args.per_asset, seed=args.seed, capture_sha=capture_sha,
        library_sha=sha(args.library), corpus_arrays=fingerprints,
        environment=dict(python=sys.version, numpy=np.__version__, mpmath=mp.__version__, platform=platform.platform()),
        assets={item['asset']: item['asset_sha'] for item in cases},
        source_hashes={path.relative_to(ROOT).as_posix(): sha(path) for path in source_paths})
    manifest['identity'] = digest(manifest)
    args.output.mkdir(parents=True, exist_ok=True)
    manifest_path = args.output/'manifest.json'
    rows_path = args.output/'cases.jsonl'
    completed = {}
    asset_inputs = {item['asset']: item for item in cases}
    if args.resume:
        if json.loads(manifest_path.read_text()) != manifest: raise ValueError('Resume identity differs; use a new output directory')
        for line in rows_path.read_text().splitlines():
            row = json.loads(line)
            key = row['asset'], row['index']
            if key in completed: raise ValueError('Duplicate completed case')
            if key[0] not in asset_inputs or not isinstance(key[1], int) or not 0 <= key[1] < args.per_asset:
                raise ValueError('Completed case is outside this corpus')
            item = asset_inputs[key[0]]
            exact_input = unflatten(np.array([int(x, 16) for x in row['input_words']], dtype=np.uint64).view(np.float64).tolist())
            target = item['poses'][key[1]]
            if not np.array_equal(exact_input['target']['input_rotation'], target[:3, :3].ravel()) or not np.array_equal(exact_input['target']['position'], target[:3, 3]):
                raise ValueError('Completed case target does not match its corpus index')
            expected_raw = np.concatenate([item['limits'].ravel(), item['seeds'][key[1]], np.ones(6)]).view(np.uint64)
            if not np.array_equal(np.array([int(x, 16) for x in row['raw_words']], dtype=np.uint64), expected_raw):
                raise ValueError('Completed case command context does not match corpus')
            completed[key] = row
    else:
        if manifest_path.exists() or rows_path.exists(): raise ValueError('Output already exists; use --resume or a new directory')
        manifest_path.write_text(json.dumps(manifest, indent=2)+'\n')
        np.savez_compressed(args.output/'inputs.npz', **arrays)
    statistics = Counter()
    def count(row):
        statistics['cases'] += 1
        statistics['oracle_'+row.get('oracle', 'error')] += 1
        statistics['native_coverage_'+str(row['coverage'])] += 1
        statistics['native_ranking_'+str(row['ranking'])] += 1
        statistics['verified'] += bool(row.get('verified'))
        statistics['errors'] += bool(row.get('error'))
    for row in completed.values(): count(row)
    batches = []
    for item in cases:
        pending = [(index, target, seed) for index, (target, seed) in enumerate(zip(item['poses'], item['seeds'], strict=True))
                   if (item['asset'], index) not in completed]
        for start in range(0, len(pending), 16):
            batches.append((str(args.library.resolve()), item['asset'], item['model'], item['limits'], pending[start:start+16]))
    def consume(rows, output):
        for row in rows:
            output.write(json.dumps(row, separators=(',', ':'))+'\n')
            output.flush()
            count(row)
            if statistics['cases'] % 10 == 0: print(json.dumps(dict(statistics)), flush=True)
    with rows_path.open('a', encoding='utf-8') as output:
        if args.jobs == 1:
            for batch in batches: consume(validate_batch(*batch), output)
        else:
            with ProcessPoolExecutor(max_workers=args.jobs) as executor:
                futures = [executor.submit(validate_batch, *batch) for batch in batches]
                for future in as_completed(futures): consume(future.result(), output)
    # Source or binary mutation during the audit invalidates this run.
    if sha(args.library) != manifest['library_sha'] or any(sha(ROOT/path) != value for path, value in manifest['source_hashes'].items()):
        raise RuntimeError('Audit inputs changed during execution')
    report = dict(manifest_identity=manifest['identity'], statistics=dict(statistics),
        expected_cases=6*args.per_asset, full_required_corpus=args.per_asset == 1000,
        execution_jobs=args.jobs,
        oracle_unresolved_is_agreement=False, performance_measurement=False,
        result_file_sha=sha(rows_path))
    if statistics['cases'] != 6*args.per_asset:
        raise RuntimeError('Audit did not cover every required corpus index')
    (args.output/'report.json').write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(report, indent=2), flush=True)
    return 1 if statistics['errors'] else (2 if statistics['oracle_unresolved'] else 0)


if __name__ == '__main__':
    raise SystemExit(main())
