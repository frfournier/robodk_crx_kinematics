# SPDX-License-Identifier: Apache-2.0
# SPDX-FileCopyrightText: 2026 Centre Technologique en Aérospatiale
"""Independent exceptional/nearby query audit on the six approved CRX assets.

Captured and independent modified-DH targets are supplemented by deterministic
translation perturbations. Exact rational root isolation and high-precision
transform products audit global claims, local balls and numerical witnesses
separately. An unresolved oracle is retained as unverified, never agreement.
This correctness experiment collects no timings.
"""

from __future__ import annotations

import argparse
from collections import Counter
from concurrent.futures import ProcessPoolExecutor, as_completed
import json
import math
from pathlib import Path
import sys

import mpmath as mp
import numpy as np

from validate_catalogues import ROOT, check_case, corpus, digest, sha
sys.path.insert(0, str(ROOT/'tests'))
from crx_kinematics import Coverage, CrxSolver, RecoveryStatus
import crx_exact_reference as ref
from test_crx_ball_certificate import check_certificate, exact_kinematics, prepare_reference
from test_crx_exact_target import unflatten


def recovery_packet(recovery):
    def value(item):
        if isinstance(item,np.ndarray): return [value(x) for x in item]
        if isinstance(item,(float,np.floating)): return float(item) if math.isfinite(item) else None
        if isinstance(item,(int,np.integer)): return int(item)
        return item
    return {name:value(item) for name,item in vars(recovery).items()}


def ball_packet(ball):
    return dict(failure=0, centre=ball.canonical_center_rad.tolist(),
        constants=[ball.existence_radius_rad, ball.uniqueness_radius_rad, ball.inverse_residual,
            ball.beta, ball.lipschitz, ball.eta, ball.pose_defect, ball.screw_norm_sum,
            ball.angular_norm_sum, ball.anchor_position_norm, ball.aspect_sign],
        trace=ball.target_relative_trace.tolist(), preconditioner=ball.preconditioner.ravel().tolist(),
        coarse_eta=ball.coarse_eta, used_log_series=ball.used_log_series)


def audit_recoveries(query, exact_input):
    strata = query.strata
    if strata is None:
        raise ValueError('This audit requires the optional exceptional diagnostic endpoints')
    with mp.workdps(200):
        problem = prepare_reference(exact_input)
        lengths = list(map(ref.number, problem['lengths']))
        target_r = mp.matrix([[ref.number(x) for x in row] for row in problem['rotation']])
        target_p = mp.matrix(list(map(ref.number, problem['translation'])))
        for recovery in strata.recoveries:
            theta = mp.mpf(recovery.phase)
            u = [ref.number(v)*mp.cos(theta)+ref.number(w)*mp.sin(theta)
                 for v,w in zip(problem['v'],problem['w'],strict=True)]
            x = [ref.number(p)+lengths[3]*v for p,v in zip(problem['center'],u,strict=True)]
            scale = max(abs(lengths[0]),abs(lengths[1]),mp.norm(mp.matrix(x)))
            delta = abs(sum(v*w for v,w in zip(x,ref.cross(u,[-x[1],x[0],mp.mpf(0)]),strict=True)))/scale**2
            defects = [delta,mp.sqrt(x[0]**2+x[1]**2)/scale,
                max(mp.norm(mp.matrix(x))/scale,abs(lengths[0]**2-lengths[1]**2)/scale**2)]
            for actual,expected in zip([recovery.delta_defect,recovery.base_axis_defect,recovery.coincident_defect],defects,strict=True):
                if abs(mp.mpf(actual)-expected) > mp.mpf('1e-12'):
                    raise AssertionError('Numerical stratum defect disagrees with independent geometry')
            for candidate in recovery.canonical_rad:
                rotation, position = ref.forward(lengths, list(map(mp.mpf, candidate)))
                actual_r = mp.matrix([rotation[3*i:3*i+3] for i in range(3)])
                if mp.norm(actual_r-target_r) > mp.mpf('5e-12') or mp.norm(mp.matrix(position)-target_p) > mp.mpf('5e-12'):
                    raise AssertionError('Numerical recovered configuration fails independent full FK')
        for ball in query.phases:
            check_certificate(ball_packet(ball),
                kinematics=exact_kinematics(exact_input, ball.canonical_center_rad.tolist(), reference_mode=True))
    if query.coverage != Coverage.COMPLETE and (query.nearest_certified or query.summary.nominal_infeasible
                                               or query.summary.command_infeasible):
        raise AssertionError('A partial exceptional catalogue established a global claim')
    return dict(recovery_budget_exhausted=strata.recovery_budget_exhausted,
        candidate_budget_exhausted=strata.candidate_budget_exhausted,
        total_ball_attempts=strata.total_ball_attempts, total_certified_balls=strata.total_certified_balls,
        nomination_count=strata.nomination_count, source_domains=strata.source_domains,
        recovery_witnesses_checked=True, local_balls_checked=True,
        recoveries=[recovery_packet(r) for r in strata.recoveries])


def validate_batch(library, model, asset, limits, batch):
    results = []
    with CrxSolver(model, dll_path=library, joint_limits=limits) as robot:
        for index, kind, target, seed in batch:
            query = robot.catalogue(target, seed=seed, max_solutions=0)
            summary = query.summary
            row = dict(asset=asset,index=index,kind=kind,target=target.tolist(),seed=seed.tolist(),
                coverage=int(summary.coverage),ranking=int(summary.ranking),
                root_count=summary.root_count,phase_count=summary.phase_count,root_nodes=summary.root_nodes,
                input_words=[format(int(x),'x') for x in summary.input_words],
                raw_words=[format(int(x),'x') for x in summary.raw_words])
            try:
                exact_input = unflatten(summary.input_words.view(np.float64).tolist())
                row.update(audit_recoveries(query,exact_input))
                row['balls'] = [ball_packet(ball) for ball in query.phases]
                row.update(check_case(query,target))
            except Exception as error:
                row.update(error=f'{type(error).__name__}: {error}',verified=False)
            finally:
                query.close()
            results.append(row)
    return results


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--library',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--per-asset',type=int,default=64)
    parser.add_argument('--nearby',type=int,default=4)
    parser.add_argument('--jobs',type=int,default=1)
    args = parser.parse_args()
    if not 1 <= args.per_asset <= 1000 or not 0 <= args.nearby <= 16 or not 1 <= args.jobs <= 8:
        parser.error('Require per-asset1..1000, nearby0..16 and jobs1..8')
    cases,capture_sha = corpus(args.per_asset,20261113)
    inputs = {}
    batches = []
    for number,item in enumerate(cases):
        targets = list(item['poses'])
        seeds = list(item['seeds'])
        kinds = ['captured_or_independent_mdh']*len(targets)
        for index in range(min(args.nearby,len(item['poses']))):
            for sign in [-1,1]:
                target = item['poses'][index].copy()
                target[0,3] += sign*2.**-30*max(1.,np.linalg.norm(target[:3,3]))
                targets.append(target)
                seeds.append(item['seeds'][index])
                kinds.append('nearby_translation')
        inputs[f'{number}_poses'] = np.asarray(targets)
        inputs[f'{number}_seeds'] = np.asarray(seeds)
        inputs[f'{number}_limits'] = item['limits']
        entries = [(index,kind,target,seed) for index,(kind,target,seed) in enumerate(zip(kinds,targets,seeds,strict=True))]
        for first in range(0,len(entries),8):
            batches.append((str(args.library.resolve()),item['model'],item['asset'],item['limits'],entries[first:first+8]))
    source_paths = {Path(__file__),Path(__file__).with_name('validate_catalogues.py'),
        *ROOT.joinpath('tests').glob('*.py'),*ROOT.joinpath('python/crx_kinematics').glob('*.py'),
        *ROOT.joinpath('src').glob('*.cpp'),*ROOT.joinpath('src').glob('*.h'),
        *ROOT.joinpath('include').glob('*.h'),ROOT/'pyproject.toml',ROOT/'uv.lock'}
    source_hashes = {p.relative_to(ROOT).as_posix():sha(p) for p in sorted(source_paths)}
    manifest = dict(schema=1,library_sha=sha(args.library),capture_sha=capture_sha,
        per_asset=args.per_asset,nearby=args.nearby,source_hashes=source_hashes,
        assets={item['asset']:item['asset_sha'] for item in cases},
        inputs={key:digest(value.tolist()) for key,value in inputs.items()},
        environment=dict(python=sys.version,numpy=np.__version__,mpmath=mp.__version__),
        expected_cases=sum(len(v) for k,v in inputs.items() if k.endswith('_poses')),
        oracle_unresolved_is_agreement=False,performance_measurement=False)
    manifest['identity'] = digest(manifest)
    args.output.mkdir(parents=True,exist_ok=False)
    (args.output/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    np.savez_compressed(args.output/'inputs.npz',**inputs)
    statistics = Counter()
    with ProcessPoolExecutor(max_workers=args.jobs) as pool, (args.output/'cases.jsonl').open('w') as rows:
        pending = [pool.submit(validate_batch,*batch) for batch in batches]
        for future in as_completed(pending):
            for row in future.result():
                rows.write(json.dumps(row)+'\n')
                statistics['cases'] += 1
                statistics['errors'] += int('error' in row)
                statistics['verified_finite'] += int(row.get('verified',False))
                statistics['oracle_'+row.get('oracle','unavailable')] += 1
                statistics['complete' if row['coverage'] == int(Coverage.COMPLETE) else 'inconclusive'] += 1
                statistics['recovery_queries'] += bool(row.get('recoveries'))
                for recovery in row.get('recoveries',[]):
                    statistics['recovery_attempts'] += 1
                    statistics['recovery_candidates'] += recovery['candidate_count']
                    statistics['recovery_local_proofs'] += recovery['certified_balls']
                    refined = recovery['refinement_attempted']
                    success = recovery['numerical_status'] == int(RecoveryStatus.CANDIDATES)
                    statistics['refinement_attempts'] += bool(refined)
                    statistics['refinement_successes' if success else 'refinement_failures'] += bool(refined)
            rows.flush()
            print(json.dumps(statistics),flush=True)
    for name,want in source_hashes.items():
        if sha(ROOT/name) != want: raise ValueError(f'Audit source changed during run: {name}')
    if sha(args.library) != manifest['library_sha']: raise ValueError('Audit library changed during run')
    if sha(ROOT/'tests/fixtures/crx_asset_frames.json') != capture_sha:
        raise ValueError('Captured asset inputs changed during run')
    for name,want in manifest['assets'].items():
        if sha(ROOT/'assets'/name) != want: raise ValueError(f'Approved asset changed during run: {name}')
    assert statistics['cases'] == manifest['expected_cases']
    report = dict(manifest_identity=manifest['identity'],statistics=statistics,
        expected_cases=manifest['expected_cases'],case_file_sha=sha(args.output/'cases.jsonl'),
        oracle_unresolved_is_agreement=False,performance_measurement=False)
    (args.output/'report.json').write_text(json.dumps(report,indent=2)+'\n')
    return 1 if statistics['errors'] else 0


if __name__ == '__main__':
    raise SystemExit(main())
