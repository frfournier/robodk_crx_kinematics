# SPDX-License-Identifier: Apache-2.0
# SPDX-FileCopyrightText: 2026 Centre Technologique en Aérospatiale
"""Independent untimed proof audit for the frozen nominal path workload.

Run with uv run python tests/audit_crx_paths.py --dataset FILE.json
--harness build/Release/crx_path_benchmark.exe --backend build/Release/crx_path_tool.exe
--output build/path-audits. Uses exact rational intervals and 180/200-digit
independent FK, Jacobian, root, chart and command-enumeration checks.
"""

import argparse
from fractions import Fraction as F
import hashlib
import itertools
import json
import math
from pathlib import Path
import struct
import subprocess
import sys

import mpmath as mp

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT/'python'))
from crx_kinematics.paths import encode_request, _strict_json
from path_interval_reference import I, pi_interval, sincos, audit_uniform
from test_crx_paths import audit_enclosure, trace_request, inclusion
from test_crx_exact_target import unflatten
from test_crx_ball_certificate import check_certificate, exact_kinematics, prepare_reference
from test_crx_root_certificate import check_geometry, check_partition, chart_coefficients, contains
import crx_exact_reference as ref

sha = lambda path: hashlib.sha256(Path(path).read_bytes()).hexdigest()
load = lambda path: _strict_json(Path(path).read_text(encoding='utf-8'))
dump = lambda path, value: Path(path).write_text(json.dumps(value, indent=2, allow_nan=False)+'\n', encoding='utf-8')

point_cache=set()
geometry_cache=set()
counts=dict(cases=0,distinct_points=0,cells=0,links=0,complete_paths=0,components=0,command_lifts=0,complete_endpoint_covers=0)

def decoded(words):return unflatten(struct.unpack('<77d',struct.pack('<77Q',*words)))

def point(record):
    if record['failure']!=0:return
    key=json.dumps(record,sort_keys=True,separators=(',',':'),allow_nan=False)
    if key in point_cache:return
    source=decoded(record['input_words'])
    assert list(struct.unpack('<77Q',struct.pack('<77d',*record['input'])))==record['input_words']
    with mp.workdps(180):
        kinematics=exact_kinematics(source,record['centre'],record['schema']==2)
        check_certificate(record,kinematics=kinematics)
    point_cache.add(key);counts['distinct_points']+=1

def packet(task):
    cover=task['root_cover'];charts=cover['charts']
    def endpoints(leaf):
        values=[]
        for bound in leaf['endpoints']:
            if bound is None or any(x is None for x in bound):
                # The owned graph report retains unavailable interval pairs as
                # [null,null]; the older root oracle uses a scalar null. Only
                # unresolved leaves may omit the required endpoint signs.
                assert leaf['kind']==2
                values.append(None)
            else:values.append(bound)
        return values
    return dict(schema=task['input_schema'],words=[format(x,'x') for x in task['input_words']],
      complete=cover['complete'],status=cover['status'],roots=cover['simple_roots'],shift=cover['shift'],
      polynomials=[dict(p=c['residual'],d=c['derivative'],q=c['determinant']) for c in charts],
      partitions=[dict(status=c['status'],roots=c['roots'],exclusions=c['exclusions'],unresolved=c['unresolved'],
        depth=c['max_depth'],leaves=[dict(kind=l['kind'],reason=l['reason'],depth=l['depth'],t=l['tangent'],
        phase=l['phase'],endpoints=endpoints(l),p=l['residual_bernstein'],d=l['derivative_bernstein'],
        q=l['determinant_bernstein'],derivative_sign=l['derivative_sign'],determinant_sign=l['determinant_sign'])
        for l in c['leaves']]) for c in charts])

def separated(a,b):
    # No integer period can bring this coordinate's existence intervals into
    # contact. All divisions/pi bounds use independent 192-bit rational I.
    radius=F(a['constants'][0])+F(b['constants'][0])
    for first,last in zip(a['centre'],b['centre'],strict=True):
        delta=F(first)-F(last)
        possible=I(delta-radius,delta+radius)/(2*pi_interval())
        if math.ceil(possible.lo)>math.floor(possible.hi):return True
    return False

def endpoint(task,graph):
    if not task['prepared']:
        assert not task['certified_infeasible'];return
    source=decoded(task['input_words'])
    assert task['raw_target_words']==task['input_words'][29:45]
    shape=packet(task)
    key=json.dumps(shape,sort_keys=True,separators=(',',':'),allow_nan=False)
    if key not in geometry_cache:
        with mp.workdps(200):
            if all(part['leaves'] for part in shape['partitions']):
                check_geometry(shape,source,graph['convention']==2)
            else:
                assert not shape['complete']
                problem=prepare_reference(source) if graph['convention']==2 else ref.prepare(source)
                for index,part in enumerate(shape['partitions']):
                    if part['leaves']:
                        exact=chart_coefficients(problem,shape['shift']+8*index)
                        for name,coefficients in zip(('p','d','q'),exact,strict=True):
                            for bound,value in zip(shape['polynomials'][index][name],coefficients,strict=True):contains(bound,value)
                        check_partition(part,exact[0],exact[2])
        geometry_cache.add(key)
    for record in task['balls']:point(record)
    if task['phase_complete']:
        assert shape['complete'] and len(task['balls'])==2*shape['roots']
        assert all(separated(a,b) for a,b in itertools.combinations(task['balls'],2))
        counts['complete_endpoint_covers']+=1
    if task['enumeration_complete']:commands(task,graph,source)
    if task['certified_infeasible']:
        assert (task['phase_complete'] and not task['balls']) or (task['admission_complete'] and task['yes']==0)

def commands(task,graph,source):
    pi=pi_interval();period=2*pi
    limits=[I(x)*pi/180 for x in graph['lower_degrees']],[I(x)*pi/180 for x in graph['upper_degrees']]
    total=0
    rows={}
    for index in task['vertices']:
        vertex=graph['vertices'][index]
        if vertex['has_command_lift']:
            key=(vertex['phase'],tuple(vertex['turns']))
            assert key not in rows;rows[key]=vertex
    yes=no=undecided=0
    for phase,ran in enumerate(task['ranges']):
        ball=task['balls'][phase]
        centre=[]
        for i,c in enumerate(ball['centre']):
            sign=[1,1,-1,-1,-1,-1][i]
            offset=(I(source['encoder_zero'][i])+(pi/2 if i==1 else I(0)))*sign
            centre.append(sign*source['senses'][i]*(I(c)-offset))
        if graph['profile']==1:centre[2]-=centre[1]
        for i,value in enumerate(centre):
            radius=F(ball['constants'][0])*(2 if graph['profile']==1 and i==2 else 1)
            value+=I(-radius,radius)
            assert F(ran['base'][i][0])<=value.lo<=value.hi<=F(ran['base'][i][1])
        bounds=[]
        for i,base in enumerate(ran['base']):
            low=(limits[0][i]-I(base[1]))/period
            high=(limits[1][i]-I(base[0]))/period
            bounds.append((math.ceil(low.lo),math.floor(high.hi)))
        if ran['empty']:
            assert any(a>b for a,b in bounds);continue
        assert ran['available']
        assert all(a<=lo and b>=hi for a,b,(lo,hi) in zip(ran['first'],ran['last'],bounds,strict=True))
        combinations=math.prod(b-a+1 for a,b in zip(ran['first'],ran['last'],strict=True))
        assert combinations==ran['count'];total+=combinations
        if not task['vertices_complete']:continue
        for turns in itertools.product(*(range(a,b+1) for a,b in zip(ran['first'],ran['last'],strict=True))):
            reference=[I(*box)+period*k for box,k in zip(ran['base'],turns,strict=True)]
            vertex=rows.get((phase,turns))
            if vertex is None:
                assert any(value.hi<lower.lo or value.lo>upper.hi
                    for value,lower,upper in zip(reference,*limits,strict=True));no+=1
            else:
                for value,box in zip(reference,vertex['command_box'],strict=True):
                    assert F(box[0])<=value.lo<=value.hi<=F(box[1])
                if vertex['admission']==1:
                    assert all(F(box[0])>=lower.hi and F(box[1])<=upper.lo
                        for box,lower,upper in zip(vertex['command_box'],*limits,strict=True));yes+=1
                else:assert vertex['admission']==2;undecided+=1
    assert total==task['box_lifts']
    if task['vertices_complete']:
        assert (yes,no,undecided)==(task['yes'],task['no'],task['undecided'])
        assert len(rows)==yes+undecided
        counts['command_lifts']+=total

def vertex(row,graph):
    point(row['root'])
    if row['membership']==0:
        assert not row['usable'];return
    assert row['root']['failure']==0
    source=decoded(row['root']['input_words'])
    assert row['root']['input_words']==graph['tasks'][row['task']]['input_words']
    if row['has_command_lift']:
        callback=[I(*box) for box in row['command_box']]
        if graph['profile']==1:callback[2]+=callback[1]
        for i,(value,box) in enumerate(zip(callback,row['canonical_box'],strict=True)):
            sign=[1,1,-1,-1,-1,-1][i]
            offset=(I(source['encoder_zero'][i])+(pi_interval()/2 if i==1 else I(0)))*sign
            canonical=offset+sign*source['senses'][i]*value
            assert F(box[0])<=canonical.lo<=canonical.hi<=F(box[1])
        distance=max(max(abs(F(lo)-F(c)),abs(F(hi)-F(c))) for (lo,hi),c in zip(row['canonical_box'],row['root']['centre'],strict=True))
        assert distance<=F(row['inclusion'][1])<=F(row['root']['constants'][1])
    if row['partial_admission']['local']:
        request=dict(input=source,last=source['target'],convention=graph['convention'],
            profile=graph['profile'],lower=graph['lower_degrees'],upper=graph['upper_degrees'],parameter=[0.,1.])
        cell={**row['partial_admission'], 'preparation_failure':0,'enclosure_failure':0}
        audit_enclosure(request,cell);audit_uniform(request,cell);counts['cells']+=1
    if row['usable']:
        assert row['admission']==1 and (row['has_command_lift'] or row['partial_admission']['admitted'])

def trace(record):
    # Failed endpoint/domain checks can retain no partition; never an edge.
    if not record['leaves']:
        assert not record['complete'];return
    request=trace_request(record)
    before=F(0);uncovered=F(0)
    for leaf in record['leaves']:
        lo,hi=map(F,leaf['parameter']);assert lo==before<hi;before=hi
        if not leaf['accepted']:uncovered+=hi-lo
        cell=leaf['cell']
        if cell['local']:
            point(cell['point'])
            local={**request,'parameter':leaf['parameter']}
            answer={**cell,'preparation_failure':0,'enclosure_failure':0}
            audit_enclosure(local,answer);audit_uniform(local,answer);counts['cells']+=1
        if leaf['accepted']:assert cell['admitted'] and leaf['failure']==0
    assert before==1
    assert F(record['uncovered_measure'][0])<=uncovered<=F(record['uncovered_measure'][1])
    for link in record['links']:
        assert link['certified'] and link['second']==link['first']+1
        a,b=[record['leaves'][link[key]] for key in ('first','second')]
        assert a['accepted'] and b['accepted'] and a['parameter'][1]==b['parameter'][0]
        a,b=a['cell'],b['cell']
        assert inclusion(a['point']['centre'],a['rho'],b['point']['centre'])<=F(link['inclusion'][1])<=F(b['point']['constants'][1])
        counts['links']+=1
    if record['complete']:
        assert uncovered==0 and len(record['links'])+1==len(record['leaves'])
        a,b=record['leaves'][0]['cell'],record['leaves'][-1]['cell']
        assert inclusion(record['first']['centre'],record['first']['constants'][0],a['point']['centre'])<=F(record['initial_inclusion'][1])<=F(a['point']['constants'][1])
        assert inclusion(b['point']['centre'],b['rho'],record['last']['centre'])<=F(record['final_inclusion'][1])<=F(record['last']['constants'][1])
        counts['complete_paths']+=1

def relation(pair,graph):
    record=pair['relation']
    if not record['comparable']:
        assert not any(record[key] for key in ('singularity_required','base_flip_verified',
                        'shoulder_wall_required','opposite_shoulder_signs'))
        return
    first,last=[graph['vertices'][pair[key]]['root'] for key in ('first','last')]
    assert first['failure']==last['failure']==0 and first['schema']==last['schema']
    assert first['input_words'][:29]+first['input_words'][45:]==last['input_words'][:29]+last['input_words'][45:]
    assert record['singularity_required']==(first['constants'][-1]!=last['constants'][-1])
    for point_record,key in ((first,'first_shoulder_sine'),(last,'last_shoulder_sine')):
        angle=I(point_record['centre'][1])+I(-point_record['constants'][0],point_record['constants'][0])
        assert -10<=angle.lo<=angle.hi<=10
        ends=[sincos(x)[0] for x in (angle.lo,angle.hi)]
        lo,hi=min(x.lo for x in ends),max(x.hi for x in ends)
        for odd in range(-9,10,2):
            critical=pi_interval()*odd/2
            if critical.lo<=angle.hi and critical.hi>=angle.lo:
                if odd%4==1:hi=F(1)
                else:lo=F(-1)
        assert F(record[key][0])<=lo<=hi<=F(record[key][1])
    sign=lambda bound:1 if bound[0]>0 else -1 if bound[1]<0 else 0
    opposite=sign(record['first_shoulder_sine'])*sign(record['last_shoulder_sine'])==-1
    assert opposite==record['opposite_shoulder_signs']
    if record['base_flip_verified']:
        assert first['input_words']==last['input_words'] and not record['singularity_required']
        image=[I(c)+I(-first['constants'][0],first['constants'][0]) for c in first['centre']]
        image[0]+=pi_interval();image[1]=-image[1]
        image[2]=pi_interval()-image[2];image[3]+=pi_interval()
        for value,turn,box in zip(image,record['flip_turns'],record['flip_image'],strict=True):
            value+=2*pi_interval()*turn
            assert F(box[0])<=value.lo<=value.hi<=F(box[1])
        distance=max(max(abs(F(lo)-F(c)),abs(F(hi)-F(c))) for (lo,hi),c in zip(record['flip_image'],last['centre'],strict=True))
        assert distance<=F(record['flip_inclusion'][1])<=F(last['constants'][1])
    assert record['shoulder_wall_required']==(opposite or record['base_flip_verified'])

def graph_audit(graph):
    assert graph['global_coverage']=='Inconclusive'
    for task in graph['tasks']:endpoint(task,graph)
    for row in graph['vertices']:vertex(row,graph)
    for pair in graph['pairs']:relation(pair,graph)
    for attempt in graph['attempts']:
        pair=graph['pairs'][attempt['pair']]
        assert attempt['path']['first']==graph['vertices'][pair['first']]['root']
        assert attempt['path']['last']==graph['vertices'][pair['last']]['root']
        trace(attempt['path'])
    adjacency={i:set() for i in range(len(graph['vertices']))}
    for edge in graph['edges']:
        attempt=graph['attempts'][edge];assert attempt['path']['complete']
        pair=graph['pairs'][attempt['pair']]
        adjacency[pair['first']].add(pair['last']);adjacency[pair['last']].add(pair['first'])
    pending=set(adjacency);components=[]
    while pending:
        seen=set();stack=[min(pending)]
        while stack:
            v=stack.pop()
            if v in seen:continue
            seen.add(v);stack.extend(adjacency[v]-seen)
        pending-=seen;components.append(sorted(seen))
    assert sorted(components)==sorted(c['vertices'] for c in graph['components'])
    covered=False
    for component in graph['components']:
        tasks=sorted({graph['vertices'][v]['task'] for v in component['vertices'] if graph['vertices'][v]['usable']})
        assert tasks==component['admitted_tasks']
        assert component['covers_all_tasks']==(len(tasks)==len(graph['tasks']))
        covered|=component['covers_all_tasks']
    infeasible=any(t['certified_infeasible'] for t in graph['tasks'])
    assert graph['status']==(3 if infeasible else 2 if covered else 1)
    counts['components']+=len(components)
    options,work=graph['options'],graph['work']
    for key,limit in [('root_nodes','max_root_nodes'),('endpoint_certificates','max_endpoint_certificates'),
      ('enumerated_lifts','max_lifts'),('pair_tests','max_pair_tests'),('path_attempts','max_paths'),
      ('path_nodes','max_path_nodes'),('path_certificates','max_path_certificates')]:assert work[key]<=options[limit]
    assert len(graph['vertices'])<=options['max_vertices']

def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--dataset', type=Path, required=True)
    parser.add_argument('--harness', type=Path, required=True)
    parser.add_argument('--backend', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args(argv)
    tree = REPO_ROOT
    harness, backend = args.harness.resolve(), args.backend.resolve()
    workload = load(args.dataset)
    assert workload['schema'] == 1 and len(workload['cases']) == 42
    assert len(workload['source_assets']) == 6
    names = [case['id'] for case in workload['cases']]
    assert len(set(names)) == len(names)
    assert all(isinstance(name, str) and 1 <= len(name) <= 80 and
               all(c in 'abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_-' for c in name)
               for name in names)
    # Guard the intended workload before invoking any native query. An alias
    # in a generator must not silently replace a moving case by a constant one.
    starts={}
    for case in workload['cases']:
        if case['model']=='synthetic':continue
        first,last=[task['target'] for task in case['request']['tasks']]
        delta=last['position_mm'][0]-first['position_mm'][0]
        expected=case['declared_translation_delta_mm']
        assert (delta==0)==(expected==0),case['id']
        if expected:assert abs(delta/expected-1)<.001,case['id']
        assert first==starts.setdefault(case['model'],first),case['id']
    for asset in workload['source_assets']:
        filename = (tree/asset['path']).resolve()
        assert filename.is_relative_to(tree) and sha(filename) == asset['sha256']
    paths = set()
    for directory in ('src', 'include', 'python', 'scripts', 'tests'):
        paths.update(p for p in (tree/directory).rglob('*') if p.is_file()
            and p.suffix in ('.cpp', '.h', '.py', '.json') and '__pycache__' not in p.parts)
    for name in ('CMakeLists.txt', 'CMakePresets.json', 'pyproject.toml', 'uv.lock', '.clang-format', '.clang-tidy'):
        paths.add(tree/name)
    sources = {p.relative_to(tree).as_posix(): sha(p) for p in sorted(paths)}
    artifacts = dict(harness=sha(harness), backend=sha(backend))
    identity = hashlib.sha256(json.dumps(dict(sources=sources, artifacts=artifacts,
        dataset_sha256=sha(args.dataset)), sort_keys=True).encode()).hexdigest()
    out = args.output/('feature-audit-'+identity[:16])
    out.mkdir(parents=True, exist_ok=True)
    manifest = dict(identity=identity, sources=sources, artifacts=artifacts, dataset_sha256=sha(args.dataset))
    if (out/'manifest.json').exists():
        assert load(out/'manifest.json') == manifest
    else:
        dump(out/'manifest.json', manifest)
    summaries=[]
    for case in workload['cases']:
        name=case['id'];request_file=out/(name+'.bin')
        request_file.write_bytes(encode_request(case['request']))
        target=out/(name+'.json')
        if not target.exists():
            result=subprocess.run([str(harness),str(request_file),'--audit'],capture_output=True,check=True,timeout=180)
            target.write_bytes(result.stdout)
        result=load(target);assert result['audit_only']
        actual=subprocess.run([str(backend),str(request_file)],capture_output=True,check=True,timeout=180)
        assert json.loads(actual.stdout)==result['graph'],name+' harness/tool drift'
        actual_graph=result['graph']
        graph_audit(actual_graph)
        expected_samples=sum(len(attempt['path']['leaves']) for attempt in actual_graph['attempts'])
        assert len(result['samples'])==expected_samples
        for sample in result['samples']:
            assert any(attempt['pair']==sample['pair'] and any((leaf['parameter'][0]/2+leaf['parameter'][1]/2)==sample['parameter'] for leaf in attempt['path']['leaves']) for attempt in actual_graph['attempts'])
        counts['cases']+=1
        summaries.append(dict(id=name,model=case['model'],kind=case['kind'],status=actual_graph['status'],
          vertices=len(actual_graph['vertices']),edges=len(actual_graph['edges']),attempts=len(actual_graph['attempts']),
          phase_complete=sum(t['phase_complete'] for t in actual_graph['tasks']),cells=sum(len(a['path']['leaves']) for a in actual_graph['attempts']),
          numerical_samples=len(result['samples']),work=actual_graph['work'],retained_payload_bytes=actual_graph['retained_payload_bytes'],
          packet_sha256=sha(request_file),audit_sha256=sha(target)))
        dump(out/'progress.json',dict(counts=counts,cases=summaries))
        print(name,'status',actual_graph['status'],'edges',len(actual_graph['edges']),'cells',summaries[-1]['cells'],flush=True)

    for name, want in sources.items():
        assert sha(tree/name) == want, name
    assert sha(harness) == artifacts['harness'] and sha(backend) == artifacts['backend']
    dump(out/'verification.json', dict(identity=identity, counts=counts, cases=summaries,
        all_positive_points_180_digits=True, all_uniform_cells_rational_192_bits=True,
        all_links_and_endpoint_membership_exact=True, all_complete_root_covers_independent=True,
        command_enumeration_and_components_independent=True, harness_matches_shipped_tool=True))
    print('All42independent workload audits passed.', json.dumps(counts), flush=True)
    return out


if __name__ == '__main__':
    main()
