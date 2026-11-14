# SPDX-License-Identifier: Apache-2.0
# SPDX-FileCopyrightText: 2026 Centre Technologique en Aérospatiale
"""Independent exact geometry and high-precision FK of exceptional witnesses."""

from concurrent.futures import ThreadPoolExecutor
import ctypes as ct
import json
import math
import os
from pathlib import Path
import subprocess
import sys

import mpmath as mp
import numpy as np
import pytest

from crx_kinematics import (CatalogueOptions, Coverage, NativeError, NearStratum, RecoverySource,
    RecoveryStatus)
from crx_kinematics import _catalogue_native as abi
import crx_exact_reference as ref
from test_catalogue_api import COMMAND, factory, queries
from test_crx_ball_certificate import check_certificate, exact_kinematics
from test_crx_exact_target import pose


def problem(lengths=(1., 1., 0., .25), quaternion=(.5, .5, .5, .5), position=(1., -.25, 0.)):
    return dict(lengths=list(lengths), base_height=0., encoder_zero=[0.]*6, senses=[1.]*6,
        lower_commands=[0.]*6, upper_commands=[0.]*6, target=pose(quaternion, position),
        base=pose(), tool=pose())


@pytest.fixture(scope="module")
def exceptional_probe():
    configured = os.environ.get("CRXKIN_EXCEPTIONAL_PROBE_PATH")
    suffix = ".exe" if sys.platform == "win32" else ""
    path = Path(configured) if configured else Path(__file__).resolve().parents[1]/f"build/Release/crx_exceptional_tests{suffix}"
    if configured:
        assert path.is_file(), f"Configured exceptional probe missing: {path}"
    elif not path.is_file():
        pytest.skip("Build crx_exceptional_tests or set CRXKIN_EXCEPTIONAL_PROBE_PATH")

    def run(inputs, nodes=256, balls=512):
        lines = [" ".join(map(repr, [*item["lengths"], *item["target"]["quaternion"],
            *item["target"]["position"], nodes, balls])) for item in inputs]
        result = subprocess.run([str(path), "--batch"], input="\n".join(lines)+"\n",
            text=True, capture_output=True, check=True, timeout=120)
        answers = [json.loads(line) for line in result.stdout.splitlines()]
        assert len(answers) == len(inputs)
        return answers
    return run


def check_witnesses(item, answer):
    with mp.workdps(160):
        prepared = ref.prepare(item)
        lengths = list(map(ref.number, prepared["lengths"]))
        expected_r = mp.matrix([[ref.number(x) for x in row] for row in prepared["rotation"]])
        expected_p = mp.matrix(list(map(ref.number, prepared["translation"])))
        for recovery in answer["recoveries"]:
            assert recovery["source"] & ~15 == 0 and 0 < recovery["source"]
            assert recovery["near_stratum"] & ~63 == 0
            assert len(recovery["candidates"]) <= 8
            assert recovery["certified_balls"] <= recovery["attempted_balls"] <= len(recovery["candidates"])
            for candidate in recovery["candidates"]:
                assert recovery["status"] == 0
                q = list(map(mp.mpf, candidate["q"]))
                rotation, position = ref.forward(lengths, q)
                actual_r = mp.matrix([rotation[3*i:3*i+3] for i in range(3)])
                assert mp.norm(actual_r-expected_r) < mp.mpf('5e-12')
                assert mp.norm(mp.matrix(position)-expected_p) < mp.mpf('5e-12')
                assert candidate["position_error"] <= 1e-12
                assert candidate["orientation_error"] <= 1e-12
        for ball in answer["balls"]:
            check_certificate(ball, kinematics=exact_kinematics(item, ball["centre"]))
    assert answer["attempted_balls"] >= answer["certified_balls"] == len(answer["balls"])
    assert len(answer["recoveries"]) <= 32
    if not answer["root_complete"]:
        assert not answer["complete"] and not answer["infeasible"]
        assert answer["coverage"] == int(Coverage.INCONCLUSIVE)


def test_exact_common_zero_retains_both_witness_signs_with_local_proofs(exceptional_probe):
    item = problem()
    answer = exceptional_probe([item])[0]
    assert not answer["root_complete"]
    common = [r for r in answer["recoveries"] if abs(math.remainder(r["phase"], 2*math.pi)) < 1e-12
              and r["near_stratum"] & NearStratum.DETERMINANT_ZERO]
    assert common and common[0]["certified_balls"] >= 4
    with mp.workdps(160):
        # At x=(1,0,0), u=Y, the independent sphere/plane equations are
        # y_x=1/2, y_y=0, y_z=+/-sqrt(3)/2. Neither sign may be discarded.
        heights = []
        for candidate in common[0]["candidates"]:
            q = list(map(mp.mpf, candidate["q"]))
            elbow = mp.matrix([mp.cos(q[0])*mp.sin(q[1]), mp.sin(q[0])*mp.sin(q[1]), mp.cos(q[1])])
            assert abs(elbow[0]-mp.mpf('.5')) < mp.mpf('1e-12') and abs(elbow[1]) < mp.mpf('1e-12')
            assert abs(abs(elbow[2])-mp.sqrt(3)/2) < mp.mpf('1e-12')
            heights.append(mp.sign(elbow[2]))
        assert set(heights) == {-1, 1}
    check_witnesses(item, answer)


@pytest.mark.parametrize("position, flag", [((0., 0., 0.), NearStratum.COINCIDENT_SPHERES),
    ((0., 0., 2.), NearStratum.VERTICAL_ARM)])
def test_continuous_fibres_keep_representatives_without_isolated_root_proofs(exceptional_probe, position, flag):
    item = problem((1., 1., 0., 0.), (1., 0., 0., 0.), position)
    answer = exceptional_probe([item])[0]
    assert not answer["complete"] and answer["certified_balls"] == 0
    selected = [r for r in answer["recoveries"] if r["near_stratum"] & flag]
    assert selected and any(r["used_representatives"] and r["candidates"] for r in selected)
    assert all(r["near_stratum"] & NearStratum.POLYNOMIAL_ENCLOSES_ZERO for r in selected)
    check_witnesses(item, answer)


@pytest.mark.parametrize("sign", [-1., 1.])
def test_on_axis_noncoincident_elbow_circle_and_nearby_cases(exceptional_probe, sign):
    item = problem((5., 3., 0., 0.), (.5, -.5, -.5, -.5), (0., 0., sign*4))
    # The zero wrist offset keeps x on-axis at every nomination. At u=Z the
    # arm equations give the exact circle z=+/-4, horizontal radius3.
    answer = exceptional_probe([item])[0]
    selected = [r for r in answer["recoveries"] if r["near_stratum"] & NearStratum.BASE_AXIS]
    assert selected and any(r["used_representatives"] and r["candidates"] for r in selected)
    assert not any(r["near_stratum"] & NearStratum.COINCIDENT_SPHERES for r in selected)
    assert answer["certified_balls"] == 0
    check_witnesses(item, answer)


@pytest.mark.parametrize("scale", [2.**-20, 1., 2.**20])
def test_signed_link_scaling_and_perturbed_common_roots(exceptional_probe, scale):
    inputs = []
    for perturb in [-1e-8, -1e-12, 0., 1e-12, 1e-8]:
        item = problem((scale, -scale, 0., .25*scale), position=(scale, (-.25+perturb)*scale, 0.))
        inputs.append(item)
    for item, answer in zip(inputs, exceptional_probe(inputs), strict=True):
        check_witnesses(item, answer)
        assert not answer["complete"]


@pytest.mark.parametrize("offset", [-1e-8, 0., 1e-8])
def test_shell_tangency_and_nearby_targets_remain_separate_from_exact_families(exceptional_probe, offset):
    item = problem((1., 1., 0., 0.), position=(2.+offset, 0., 0.))
    answer = exceptional_probe([item])[0]
    check_witnesses(item, answer)
    if offset == 0:
        assert answer["recoveries"] and any(r["near_stratum"] & NearStratum.ARM_SHELL
            and r["candidates"] for r in answer["recoveries"])
        assert answer["certified_balls"] == 0


def test_normalized_numerical_defects_match_independent_geometry(exceptional_probe):
    item = problem()
    answer = exceptional_probe([item])[0]
    with mp.workdps(160):
        prepared = ref.prepare(item)
        for recovery in answer["recoveries"]:
            theta = mp.mpf(recovery["phase"])
            u = [ref.number(v)*mp.cos(theta)+ref.number(w)*mp.sin(theta)
                 for v,w in zip(prepared['v'],prepared['w'],strict=True)]
            x = [ref.number(p)+ref.number(prepared['lengths'][3])*v
                 for p,v in zip(prepared['center'],u,strict=True)]
            a,b = map(ref.number,prepared['lengths'][:2])
            scale = max(abs(a),abs(b),mp.norm(mp.matrix(x)))
            n = [-x[1],x[0],mp.mpf(0)]
            delta = abs(sum(v*w for v,w in zip(x,ref.cross(u,n),strict=True)))/scale**2
            defects = [delta, mp.sqrt(x[0]**2+x[1]**2)/scale,
                       max(mp.norm(mp.matrix(x))/scale,abs(a*a-b*b)/scale**2)]
            for actual, expected in zip(recovery['defects'][:3],defects,strict=True):
                assert abs(mp.mpf(actual)-expected) < mp.mpf('1e-13')


@pytest.mark.parametrize("nodes, balls", [(0, 0), (0, 1), (1, 4), (16, 0), (256, 3)])
def test_recovery_and_local_ball_budgets_never_complete_unresolved_cover(exceptional_probe, nodes, balls):
    item = problem()
    answer = exceptional_probe([item], nodes=nodes, balls=balls)[0]
    assert answer["root_nodes"] <= nodes and answer["attempted_balls"] <= balls
    assert answer["recoveries"] and not answer["complete"] and not answer["infeasible"]
    assert any(r["candidates"] for r in answer["recoveries"])
    check_witnesses(item, answer)


def test_optional_strata_snapshots_lifetime_and_concurrent_reads(factory, queries):
    solver = factory()
    regular = solver.catalogue(solver.fk(COMMAND), seed=COMMAND)
    queries.append(regular)
    assert regular.strata is not None and not regular.strata.recoveries
    query = solver.catalogue(solver.fk(COMMAND), max_solutions=0, options=CatalogueOptions(max_nodes=0))
    queries.append(query)
    solver.close()
    assert query.coverage == Coverage.INCONCLUSIVE and not query.nearest_certified
    assert not query.summary.nominal_infeasible and query.numerical_solutions.shape == (0,6)
    with ThreadPoolExecutor(max_workers=4) as pool:
        snapshots = list(pool.map(lambda _: query.strata, range(8)))
    assert all(value.recoveries for value in snapshots)
    snapshot = snapshots[0]
    assert all(isinstance(r.source, RecoverySource) and isinstance(r.near_stratum, NearStratum)
               and isinstance(r.numerical_status, RecoveryStatus) for r in snapshot.recoveries)
    for recovery in snapshot.recoveries:
        assert recovery.canonical_rad.shape == (recovery.candidate_count,6)
        assert not recovery.canonical_rad.flags.writeable
    query.close()
    assert snapshot.recoveries and snapshot.source_domains >= len(snapshot.recoveries)
    with pytest.raises(ValueError, match="closed"): query.strata


def test_older_catalogue_dll_without_optional_strata_keeps_query(factory, queries, monkeypatch):
    solver = factory()
    monkeypatch.setattr(solver._library, "catalogue_strata_read", None)
    monkeypatch.setattr(solver._library, "catalogue_recovery_read", None)
    query = solver.catalogue(solver.fk(COMMAND), seed=COMMAND)
    queries.append(query)
    assert query.coverage == Coverage.COMPLETE and query.nearest_certified and query.strata is None


def test_public_ball_budget_is_explicit_and_does_not_hide_recovery(factory, queries):
    solver = factory()
    query = solver.catalogue(solver.fk(COMMAND), max_solutions=0,
        options=CatalogueOptions(max_nodes=0, max_candidate_certificates=0))
    queries.append(query)
    diagnostics = query.strata
    assert diagnostics.candidate_budget_exhausted and diagnostics.nomination_count > 0
    assert diagnostics.total_ball_attempts == diagnostics.total_certified_balls == 0
    assert diagnostics.recoveries and query.coverage == Coverage.INCONCLUSIVE
    assert not query.nearest_certified and not query.summary.nominal_infeasible


@pytest.mark.parametrize("field", ["count", "candidate_budget_exhausted", "nomination_count"])
def test_wrapper_rejects_invalid_strata_metadata(factory, queries, monkeypatch, field):
    solver = factory()
    query = solver.catalogue(solver.fk(COMMAND), max_solutions=0, options=CatalogueOptions(max_nodes=0))
    queries.append(query)
    original = solver._library.catalogue_strata_read
    def corrupt(*args):
        code = original(*args)
        record = ct.cast(args[-1],ct.POINTER(abi.Strata)).contents
        setattr(record,field,99999)
        return code
    monkeypatch.setattr(solver._library, "catalogue_strata_read", corrupt)
    with pytest.raises(NativeError, match="invalid recovery counts"): query.strata


@pytest.mark.parametrize("name,record_type,indices", [("catalogue_strata_read", abi.Strata, ()),
    ("catalogue_recovery_read", abi.Recovery, (0,))])
def test_optional_native_record_negotiation_and_atomic_failure(factory, queries, name, record_type, indices):
    assert ct.sizeof(abi.Strata) == 40 and ct.sizeof(abi.Recovery) == 616
    solver = factory()
    query = solver.catalogue(solver.fk(COMMAND), options=CatalogueOptions(max_nodes=0), max_solutions=0)
    queries.append(query)
    class Guarded(ct.Structure):
        _fields_ = [("before", ct.c_uint64), ("record", record_type), ("after", ct.c_uint64)]
    guarded = Guarded()
    guarded.before = guarded.after = 0x0123456789ABCDEF
    record = guarded.record
    function = getattr(solver._library, name)
    record.struct_size, record.version = ct.sizeof(record), 1
    assert function(query._owner.handle, *indices, ct.byref(record)) == 0
    assert guarded.before == guarded.after == 0x0123456789ABCDEF
    for field, value in [("struct_size", 1), ("version", 2), ("reserved", 1)]:
        record.struct_size, record.version, record.reserved = ct.sizeof(record), 1, 0
        setattr(record, field, value)
        before = bytes(guarded)
        assert function(query._owner.handle, *indices, ct.byref(record)) == -1
        assert bytes(guarded) == before
    record.struct_size, record.version, record.reserved = ct.sizeof(record), 1, 0
    before = bytes(guarded)
    assert function(None, *indices, ct.byref(record)) == -1 and bytes(guarded) == before
    if indices:
        assert function(query._owner.handle, 2**32-1, ct.byref(record)) == -1
        assert bytes(guarded) == before
