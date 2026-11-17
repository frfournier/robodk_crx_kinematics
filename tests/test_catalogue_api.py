# SPDX-License-Identifier: Apache-2.0
# SPDX-FileCopyrightText: 2026 Centre Technologique en Aérospatiale
"""Linked-query proof soundness, ABI negotiation, lifetime and optional fallback."""

from concurrent.futures import ThreadPoolExecutor
from contextlib import ExitStack
import ctypes as ct
import gc
import itertools
import math
import weakref

import mpmath as mp
import numpy as np
import pytest

from crx_kinematics import (Admission, CatalogueOptions, CommandProfile, Coverage,
    CrxSolver, IKStatus, NativeError, Ranking, RootStatus)
from crx_kinematics import _catalogue_native as abi
from crx_kinematics._native import BallRecord
import crx_catalogue_reference as oracle
from test_crx_ball_certificate import check_certificate, exact_kinematics
from test_crx_command_certificate import commands as exact_commands
from test_crx_exact_target import contains, unflatten
from test_crx_root_certificate import check_geometry
from crx_reference import rotation, translation

COMMAND = np.array([20., 30., 40., 50., 60., 70.])


@pytest.fixture
def factory(kinematics_lib):
    with ExitStack() as stack:
        def create(model="CRX-10iA", **kwargs):
            return stack.enter_context(CrxSolver(model, dll_path=kinematics_lib._name, **kwargs))
        yield create


@pytest.fixture
def queries():
    values = []
    yield values
    for value in values: value.close()


def geometry_packet(query):
    summary = query.summary
    charts = [query.chart(i) for i in range(2)]
    partitions = []
    for chart in charts:
        leaves = [query.leaf(chart.chart, i) for i in range(chart.leaf_count)]
        partitions.append(dict(status=int(chart.status), roots=chart.roots, exclusions=chart.exclusions,
            unresolved=chart.unresolved, depth=chart.max_depth, leaves=[dict(kind=int(leaf.kind),
            reason=int(leaf.reason), depth=leaf.depth, t=leaf.tangent.tolist(), phase=leaf.phase.tolist(),
            endpoints=leaf.endpoints.tolist(), p=leaf.residual_bernstein.tolist(),
            d=leaf.derivative_bernstein.tolist(), q=leaf.determinant_bernstein.tolist(),
            derivative_sign=leaf.derivative_sign, determinant_sign=leaf.determinant_sign) for leaf in leaves]))
    return dict(schema=summary.identity_schema, words=[format(int(x), 'x') for x in summary.input_words],
        complete=summary.root_status == RootStatus.COMPLETE, status=int(summary.root_status),
        roots=summary.root_count, shift=summary.chart_shift, partitions=partitions,
        polynomials=[dict(p=chart.residual.tolist(), d=chart.derivative.tolist(), q=chart.determinant.tolist()) for chart in charts])


def audit(query, constants=True):
    summary = query.summary
    assert summary.coverage == Coverage.COMPLETE
    assert summary.preparation == 1 and summary.root_status == RootStatus.COMPLETE
    assert summary.phase_count == 2*summary.root_count == len(query.phases)
    assert summary.enumeration_complete and summary.command_count == summary.yes+summary.undecided
    assert summary.box_lifts == summary.yes+summary.no+summary.undecided
    input = unflatten(summary.input_words.view(np.float64).tolist())
    raw = summary.raw_words.view(np.float64).reshape(4, 6)
    rows = [query.command(i) for i in range(summary.command_count)]
    with mp.workdps(200):
        check_geometry(geometry_packet(query), input, reference=True)
        reference = oracle.exhaustive(input, reference=True)
        assert reference.outcome == "finite", reference
        lower, upper, seed = [[mp.mpf(float(x))*mp.pi/180 for x in values] for values in raw[:3]]
        weights = [mp.mpf(float(x)) for x in raw[3]]
        base_roots, matches = [], []
        for phase, ball in enumerate(query.phases):
            np.testing.assert_array_equal(ball.input_words, summary.input_words)
            if constants:
                packet = dict(centre=ball.canonical_center_rad.tolist(), failure=0,
                    constants=[ball.existence_radius_rad, ball.uniqueness_radius_rad,
                        ball.inverse_residual, ball.beta, ball.lipschitz, ball.eta, ball.pose_defect,
                        ball.screw_norm_sum, ball.angular_norm_sum, ball.anchor_position_norm, ball.aspect_sign],
                    trace=ball.target_relative_trace.tolist(), preconditioner=ball.preconditioner.ravel().tolist(),
                    coarse_eta=ball.coarse_eta, used_log_series=ball.used_log_series)
                check_certificate(packet, kinematics=exact_kinematics(input, packet["centre"], reference_mode=True))
            indices = [i for i, q in enumerate(reference.configurations) if max(
                oracle.circle_distance(x, mp.mpf(float(y))) for x, y in zip(q, ball.canonical_center_rad, strict=True)) <= ball.existence_radius_rad]
            assert len(indices) == 1
            matches += indices
            canonical = [x+2*mp.pi*mp.nint((mp.mpf(float(c))-x)/(2*mp.pi))
                         for x, c in zip(reference.configurations[indices[0]], ball.canonical_center_rad, strict=True)]
            base = exact_commands(input, canonical, summary.profile)
            base_roots.append(base)
            centre = exact_commands(input, ball.canonical_center_rad.tolist(), summary.profile)
            phase_range = query.command_range(phase)
            assert phase_range.available
            for axis in range(6):
                radius = mp.mpf(ball.existence_radius_rad)*(2 if summary.profile == 1 and axis == 2 else 1)
                contains(phase_range.base[axis], centre[axis]-radius)
                contains(phase_range.base[axis], centre[axis]+radius)
        assert sorted(matches) == list(range(len(query.phases)))
        potential = {(row.phase, tuple(map(int, row.turns))): (i, row) for i, row in enumerate(rows)}
        assert len(potential) == len(rows)
        feasible = {}
        for phase, base in enumerate(base_roots):
            turn_sets = [range(int(mp.ceil((lo-x)/(2*mp.pi))), int(mp.floor((hi-x)/(2*mp.pi)))+1)
                         for lo, hi, x in zip(lower, upper, base, strict=True)]
            for turns in itertools.product(*turn_sets):
                key = phase, turns
                assert key in potential, "Exact feasible command was omitted"
                feasible[key] = sum(w*(x+2*mp.pi*m-s)**2 for w, x, m, s in zip(weights, base, turns, seed, strict=True))
        if summary.admission_complete: assert len(feasible) == summary.yes
        for key, (_, row) in potential.items():
            exact = [x+2*mp.pi*m for x, m in zip(base_roots[row.phase], row.turns, strict=True)]
            for box, value in zip(row.radians, exact, strict=True): contains(box, value)
            if row.admission == Admission.YES: assert key in feasible
            if summary.has_seed and row.travel_squared is not None:
                contains(row.travel_squared, sum(w*(x-s)**2 for w, x, s in zip(weights, exact, seed, strict=True)))
        if summary.ranking in (Ranking.UNIQUE, Ranking.POSSIBLE_MINIMIZERS):
            minimum = min(feasible.values())
            assert all(rows[potential[key][0]].possible_minimizer for key, cost in feasible.items() if cost == minimum)
        if summary.ranking == Ranking.UNIQUE:
            assert rows[0].admission == Admission.YES and rows[0].possible_minimizer
            assert all(rows[0].travel_squared[1] < row.travel_squared[0] for row in rows[1:])
    return input


@pytest.mark.parametrize("capacity", [0, 1, 64, 512])
def test_linked_complete_catalogue_independent_proofs_and_prefix(factory, queries, capacity):
    solver = factory()
    target = solver.fk(COMMAND)
    result = solver.catalogue(target, seed=COMMAND, max_solutions=capacity)
    queries.append(result)
    audit(result, constants=capacity == 1)
    assert result.nearest_certified
    assert len(result.commands) == min(capacity, result.summary.command_count)
    assert len(result.numerical_solutions) <= capacity
    assert result.summary.truncated == (result.summary.command_count > capacity)


def test_original_degree_limits_raw_seed_and_weights_survive_conversion(factory, queries):
    limits = np.array([[-170.123456789, -30.2, -179.99, -185.1234567, -178.65, -200.007],
                       [170.654321, 150.999999, 300.12345, 185.123456, 178.9, 230.3]])
    solver = factory(joint_limits=limits)
    target = solver.fk(COMMAND)
    seed = np.array([-0., 0., -0., 0., 0., 0.])
    weights = np.array([1., 2., .25, 4., .5, 3.])
    result = solver.catalogue(target, seed=seed, weights=weights)
    queries.append(result)
    expected = np.concatenate([limits.ravel(), seed, weights]).view(np.uint64)
    np.testing.assert_array_equal(result.summary.raw_words, expected)
    assert np.any(solver.joint_limits.view(np.uint64) != limits.view(np.uint64)), "Fixture must exercise lossy round trip"
    audit(result, constants=False)


@pytest.mark.parametrize("profile", [CommandProfile.CALLBACK, CommandProfile.DECOUPLED_USER])
def test_explicit_profile_and_multiturn_box(factory, queries, profile):
    solver = factory(joint_limits=np.array([[-720.]*6, [720.]*6]))
    target = solver.fk(COMMAND)
    seed = COMMAND.copy()
    if profile == CommandProfile.DECOUPLED_USER: seed[2] -= seed[1]
    limits = np.array([[-360.]*6, [360.]*6])
    result = solver.catalogue(target, seed=seed, joint_limits=limits, profile=profile, max_solutions=1)
    queries.append(result)
    audit(result, constants=False)
    assert result.summary.yes == result.summary.phase_count*64
    assert result.summary.profile == profile and result.nearest_certified
    np.testing.assert_array_equal(result.summary.raw_words[:12], limits.ravel().view(np.uint64))
    for row in result.numerical_solutions:
        callback = row.copy()
        if profile == CommandProfile.DECOUPLED_USER: callback[2] += callback[1]
        np.testing.assert_allclose(solver.fk(callback), target, atol=1e-8, rtol=0)


@pytest.mark.parametrize("model", ["CRX-5iA", "CRX-10iA-L", "CRX-25iA"])
def test_public_query_retains_raw_reference_tool_and_tcp_composition(factory, queries, model):
    reference = translation(120., -80., 45.) @ rotation(2, .3) @ rotation(0, -.2)
    tool = translation(-35., 24., 92.) @ rotation(1, -.4) @ rotation(2, .15)
    solver = factory(model, reference=reference, tool=tool)
    target = solver.fk(COMMAND)
    result = solver.catalogue(target, seed=COMMAND, max_solutions=1)
    queries.append(result)
    input = audit(result, constants=False)
    assert result.nearest_certified
    for name, matrix in [('target', target), ('base', reference), ('tool', tool)]:
        np.testing.assert_array_equal(input[name]['input_rotation'], matrix[:3, :3].ravel())
        np.testing.assert_array_equal(input[name]['position'], matrix[:3, 3])


@pytest.mark.parametrize("options, coverage", [(CatalogueOptions(max_nodes=0), Coverage.INCONCLUSIVE),
    (CatalogueOptions(max_candidate_certificates=0), Coverage.INCONCLUSIVE),
    (CatalogueOptions(max_pair_tests=0), Coverage.INCONCLUSIVE),
    (CatalogueOptions(max_lifts=0), Coverage.COMPLETE),
    (CatalogueOptions(enabled=False), Coverage.NOT_ATTEMPTED)])
def test_failed_or_disabled_proof_preserves_unproved_numerical_fallback(factory, queries, options, coverage):
    solver = factory()
    target = solver.fk(COMMAND)
    result = solver.catalogue(target, seed=COMMAND, options=options)
    queries.append(result)
    assert result.coverage == coverage and len(result.numerical_solutions) > 0
    assert not result.nearest_certified and not result.summary.command_infeasible
    assert not result.summary.admission_complete
    for row in result.numerical_solutions: np.testing.assert_allclose(solver.fk(row), target, atol=1e-8, rtol=0)


def test_pose_and_command_infeasibility_are_distinct(factory, queries):
    solver = factory()
    target = solver.fk(COMMAND)
    constrained = solver.catalogue(target, joint_limits=np.zeros((2, 6)))
    far = target.copy()
    far[:3, 3] = [20000., 30000., -40000.]
    excluded = solver.catalogue(far, options=CatalogueOptions(max_lifts=0))
    queries.extend([constrained, excluded])
    assert constrained.coverage == excluded.coverage == Coverage.COMPLETE
    assert constrained.summary.command_infeasible and not constrained.summary.nominal_infeasible
    assert excluded.summary.command_infeasible and excluded.summary.nominal_infeasible
    assert not excluded.phases and not excluded.commands


def test_boundary_uncertainty_remains_unresolved_in_public_query(factory, queries):
    solver = factory()
    target = solver.fk(COMMAND)
    original = solver.catalogue(target, seed=COMMAND)
    queries.append(original)
    centre = np.degrees(original.commands[0].radians.mean(axis=1))
    limits = np.array([centre-.01, centre+.01])
    limits[:, 0] = centre[0]
    boundary = solver.catalogue(target, seed=centre, joint_limits=limits)
    queries.append(boundary)
    assert boundary.coverage == Coverage.COMPLETE and boundary.summary.undecided > 0
    assert not boundary.summary.admission_complete and not boundary.summary.command_infeasible
    assert not boundary.nearest_certified
    audit(boundary, constants=False)


def test_real_approximate_numerical_fallback_coexists_with_pose_exclusion(factory, queries):
    solver = factory()
    seed = np.array([0., 0., 90+math.degrees(math.atan2(150, 700)), -90., 0., 0.])
    target = solver.fk(seed)
    target[:3, 3] += .005*target[:3, 3]/np.linalg.norm(target[:3, 3])
    result = solver.catalogue(target, seed=seed)
    queries.append(result)
    assert result.summary.numerical_status == IKStatus.APPROXIMATE_SEED
    assert len(result.numerical_solutions) == 1
    assert not result.nearest_certified and not result.commands
    # Numerical approximate acceptance is a different predicate from exact pose
    # feasibility: the exact target can be excluded while retaining a near seed.
    if result.coverage == Coverage.COMPLETE: assert result.summary.nominal_infeasible


@pytest.mark.parametrize("update", [{"seed": np.full(6, 1e308)},
    {"joint_limits": np.full((2, 6), 1e20)}])
def test_travel_or_range_overflow_never_claims_nearest(factory, queries, update):
    solver = factory()
    result = solver.catalogue(solver.fk(COMMAND), **update)
    queries.append(result)
    assert result.coverage == Coverage.COMPLETE and not result.nearest_certified
    assert not result.summary.command_infeasible
    if "seed" in update:
        assert result.summary.admission_complete and result.commands
        assert all(row.travel_squared is None for row in result.commands)
    else:
        assert not result.summary.enumeration_complete


def test_snapshots_and_native_audit_survive_solver_closure(factory, queries):
    solver = factory()
    result = solver.catalogue(solver.fk(COMMAND), seed=COMMAND)
    queries.append(result)
    solver.close()
    for function in [lambda: result.command(0), lambda: result.phase(0), lambda: result.chart(0), lambda: result.leaf(0, 0)]: function()
    arrays = [result.summary.raw_words, result.summary.input_words, result.summary.root_leaf_count,
              result.numerical_solutions, result.commands[0].radians, result.phases[0].preconditioner]
    for array in arrays:
        assert array.flags.owndata and not array.flags.writeable
        with pytest.raises(ValueError): array.flat[0] = 0
    result.close()
    result.close()
    for array in arrays: assert array.size
    with pytest.raises(ValueError, match="closed"): result.command(0)


def test_concurrent_native_reads_and_exactly_once_gc_cleanup(factory, monkeypatch):
    solver = factory()
    library = solver._library
    destroy, destroyed = library.catalogue_destroy, []
    def tracked(pointer):
        destroyed.append(pointer.value)
        destroy(pointer)
    monkeypatch.setattr(library, "catalogue_destroy", tracked)
    result = solver.catalogue(solver.fk(COMMAND), seed=COMMAND)
    handle = result._owner.handle
    def read(_):
        record = abi.Summary()
        record.struct_size, record.version = ct.sizeof(record), 1
        assert library.catalogue_read(handle, ct.byref(record)) == 0
        return record.command_count
    with ThreadPoolExecutor(max_workers=4) as executor:
        assert list(executor.map(read, range(24))) == [result.summary.command_count]*24
    reference = weakref.ref(result)
    del result
    gc.collect()
    assert reference() is None and len(destroyed) == 1


@pytest.mark.parametrize("field,bad", [("struct_size", 1), ("version", 99), ("reserved", 1),
    ("flags", 2), ("profile", 2), ("capacity", 513), ("max_nodes", (1 << 20)+1),
    ("max_depth", 51), ("max_leaves", 1), ("max_chart_attempts", 0), ("max_lifts", (1 << 20)+1)])
def test_query_negotiation_is_atomic(factory, field, bad):
    solver = factory()
    target = solver.fk(COMMAND)
    options = CatalogueOptions().native(CommandProfile.CALLBACK, 64)
    setattr(options, field, bad)
    before = bytes(options)
    pointer = ct.c_void_p(0x12345)
    snapshots = [target.copy(), solver._reference.copy(), solver._tool.copy()]
    assert solver._library.catalogue_query(solver._handle, target, solver._reference, solver._tool,
        None, None, None, ct.byref(options), ct.byref(pointer)) == -1
    assert pointer.value is None and bytes(options) == before
    for original, array in zip(snapshots, [target, solver._reference, solver._tool], strict=True): np.testing.assert_array_equal(original, array)


@pytest.mark.parametrize("name,record_type,indices", [("catalogue_read", abi.Summary, ()),
    ("catalogue_phase", BallRecord, (0,)), ("catalogue_command_read", abi.Command, (0,)),
    ("catalogue_range_read", abi.Range, (0,)), ("catalogue_chart_read", abi.Chart, (0,)),
    ("catalogue_leaf_read", abi.Leaf, (0, 0))])
def test_negotiated_reads_preserve_canaries_and_reject_layouts(factory, queries, name, record_type, indices):
    solver = factory()
    result = solver.catalogue(solver.fk(COMMAND), seed=COMMAND)
    queries.append(result)
    class Guarded(ct.Structure):
        _fields_ = [("before", ct.c_uint64), ("record", record_type), ("after", ct.c_uint64)]
    guarded = Guarded()
    guarded.before = guarded.after = 0x0123456789ABCDEF
    record = guarded.record
    record.struct_size, record.version = ct.sizeof(record), 1
    function = getattr(solver._library, name)
    assert function(result._owner.handle, *indices, ct.byref(record)) == 0
    assert guarded.before == guarded.after == 0x0123456789ABCDEF
    for field, bad in [("struct_size", 1), ("version", 2), ("reserved", 1)]:
        record.struct_size, record.version, record.reserved = ct.sizeof(record), 1, 0
        setattr(record, field, bad)
        before = bytes(guarded)
        assert function(result._owner.handle, *indices, ct.byref(record)) == -1
        assert bytes(guarded) == before


@pytest.mark.parametrize("fault", ["summary", "phase", "command"])
def test_wrapper_cleans_query_handle_on_malformed_native_response(factory, monkeypatch, fault):
    solver = factory()
    library = solver._library
    destroy, destroyed = library.catalogue_destroy, []
    def tracked(pointer):
        destroyed.append(pointer.value)
        destroy(pointer)
    monkeypatch.setattr(library, "catalogue_destroy", tracked)
    name, record_type = {"summary": ("catalogue_read", abi.Summary),
                         "phase": ("catalogue_phase", BallRecord), "command": ("catalogue_command_read", abi.Command)}[fault]
    original = getattr(library, name)
    def corrupt(*args):
        code = original(*args)
        record = ct.cast(args[-1], ct.POINTER(record_type)).contents
        if fault == "summary": record.coverage = 99
        elif fault == "phase": record.eta = math.nan
        else: record.phase = 99999
        return code
    monkeypatch.setattr(library, name, corrupt)
    with pytest.raises(NativeError): solver.catalogue(solver.fk(COMMAND), seed=COMMAND)
    gc.collect()
    assert len(destroyed) == 1


def test_old_dll_fallback_never_claims_coverage_or_original_limit_provenance(factory, monkeypatch):
    solver = factory()
    target = solver.fk(COMMAND)
    monkeypatch.setattr(solver._library, "catalogue_query", None)
    result = solver.catalogue(target, seed=COMMAND, max_solutions=1)
    assert result.coverage == Coverage.NOT_ATTEMPTED and not result.phases and not result.commands
    assert not result.summary.raw_provenance_available and not result.summary.raw_words.size
    assert len(result.numerical_solutions) == 1
    np.testing.assert_allclose(solver.fk(result.numerical_solutions[0]), target, atol=1e-8, rtol=0)
    result.close()


@pytest.mark.parametrize("name,indices,record_type", [("catalogue_phase", (2**32-1,), BallRecord),
    ("catalogue_command_read", (2**64-1,), abi.Command), ("catalogue_range_read", (2**32-1,), abi.Range),
    ("catalogue_chart_read", (2,), abi.Chart), ("catalogue_leaf_read", (2, 0), abi.Leaf),
    ("catalogue_leaf_read", (0, 2**32-1), abi.Leaf)])
def test_out_of_range_native_reads_leave_records_unchanged(factory, queries, name, indices, record_type):
    solver = factory()
    result = solver.catalogue(solver.fk(COMMAND), seed=COMMAND)
    queries.append(result)
    record = record_type()
    record.struct_size, record.version = ct.sizeof(record), 1
    before = bytes(record)
    assert getattr(solver._library, name)(result._owner.handle, *indices, ct.byref(record)) == -1
    assert bytes(record) == before


@pytest.mark.parametrize("update", ["limits", "seed", "weights", "pose", "reference", "tool", "model"])
def test_native_query_rejects_invalid_inputs_without_allocating_or_mutating(factory, update):
    solver = factory()
    target, reference, tool = solver.fk(COMMAND), np.eye(4), np.eye(4)
    limits, seed, weights = np.array([[-180.]*6, [180.]*6]), COMMAND.copy(), np.ones(6)
    handle = solver._handle
    if update == "limits": limits[0, 0] = limits[1, 0]+1
    elif update == "seed": seed[0] = math.inf
    elif update == "weights": weights[0] = 0
    elif update == "pose": target[0, 0] = math.nan
    elif update == "reference": reference[3, 0] = 1
    elif update == "tool": tool[0, 0] = -1
    else: handle = None
    arrays = [target, reference, tool, limits, seed, weights]
    before = [array.tobytes() for array in arrays]
    options = CatalogueOptions().native(CommandProfile.CALLBACK, 64)
    pointer = ct.c_void_p(0x12345)
    assert solver._library.catalogue_query(handle, target, reference, tool, limits, seed, weights,
        ct.byref(options), ct.byref(pointer)) == -1
    assert pointer.value is None and before == [array.tobytes() for array in arrays]


@pytest.mark.parametrize("update,error", [({"max_solutions": -1}, ValueError),
    ({"max_solutions": True}, TypeError), ({"max_solutions": 513}, ValueError),
    ({"weights": [0]*6}, ValueError), ({"weights": [math.nan]*6}, ValueError),
    ({"profile": 99}, ValueError), ({"profile": True}, TypeError),
    ({"profile": CommandProfile.DECOUPLED_USER}, ValueError),
    ({"options": CatalogueOptions(max_nodes=-1)}, ValueError),
    ({"options": CatalogueOptions(enabled=1)}, TypeError), ({"options": object()}, TypeError)])
def test_invalid_wrapper_arguments(factory, update, error):
    solver = factory()
    with pytest.raises(error): solver.catalogue(solver.fk(COMMAND), **update)
