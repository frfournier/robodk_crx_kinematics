# SPDX-License-Identifier: Apache-2.0
# SPDX-FileCopyrightText: 2026 Centre Technologique en Aérospatiale
"""Calibrated C ABI ownership, proof, negotiation and failure-atomicity checks."""

import ctypes as ct
import math

import mpmath as mp
import numpy as np
import pytest

from crx_kinematics import _calibrated_native as native
from crx_kinematics._native import NativeLibrary
from test_crx_calibrated import (
    _check_calibrated_certificate, _oracle, _synthetic, _target_at,
)


@pytest.fixture(scope="module")
def api(kinematics_lib):
    library = NativeLibrary(kinematics_lib._name)
    binding = native.Bindings(library)
    yield binding
    library.close_directory()


def pointer(array):
    return array.ctypes.data_as(ct.POINTER(ct.c_double))


def model_input(case):
    value = native.ModelInput()
    for i, (aq, ar, a, tq, tr, d) in enumerate(case["rows"]):
        value.dh[i][:] = [ar, a, tr, d]
        value.alpha_quarters[i], value.theta_quarters[i] = aq, tq
    value.senses[:] = case["senses"]
    value.base_quaternion[:], value.base_position[:] = case["base"][:4], case["base"][4:]
    value.tool_quaternion[:], value.tool_position[:] = case["tool"][:4], case["tool"][4:]
    value.lower_commands[:], value.upper_commands[:] = case["lower"], case["upper"]
    value.length_scale = case["scale"]
    value.limit_units = case.get("limit_units", 0)
    return value


def target_input(case):
    value = native.Target()
    value.quaternion[:], value.position[:] = case["target"][:4], case["target"][4:]
    return value


def case_input():
    with mp.workdps(160):
        case = _synthetic(1000.)
        case["x"] = [.3, -.7, .4, .8, -.6, .2]
        case["target"] = _target_at(case)
    return case


def create(api, case):
    value = model_input(case)
    before = bytes(value)
    handle = ct.c_void_p()
    native.check(api.create(ct.byref(value), ct.byref(handle)))
    assert handle.value and bytes(value) == before
    return handle, value


def query(api, model, case, seeds, options=None):
    value = target_input(case)
    before = bytes(value)
    handle = ct.c_void_p()
    seed_pointer = pointer(seeds) if len(seeds) else None
    native.check(api.query_create(model, ct.byref(value), seed_pointer, len(seeds),
                                  None if options is None else ct.byref(options), ct.byref(handle)))
    assert handle.value and bytes(value) == before
    return handle


def read(api, handle, kind=native.Summary, index=None):
    result = kind()
    if kind is native.Result:
        native.check(api.result_read(handle, index, ct.byref(result)))
    else:
        method = api.summary_read if kind is native.Summary else api.identity_read
        native.check(method(handle, ct.byref(result)))
    return result


def proof_record(record):
    return {"certified": bool(record.flags & 8), "failure": record.certificate_failure,
            "admitted": bool(record.flags & 16), "local_sign": record.local_jacobian_sign,
            "command_sign": record.command_jacobian_sign,
            "rho": record.existence_radius, "radius": record.uniqueness_radius,
            **{key: getattr(record, key) for key in ["beta", "lipschitz", "eta", "inverse_residual"]},
            "screw_sum": record.screw_norm_sum, "angular_sum": record.angular_norm_sum,
            "position_norm": record.anchor_position_norm, "command_reserve": record.command_reserve_lower,
            "preconditioner": np.array(record.preconditioner).reshape(6, 6).tolist()}


def test_calibrated_c_layouts_and_defaults(api):
    assert [ct.sizeof(kind) for kind in [native.ModelInput, native.Target, native.Options,
                                       native.Summary, native.Identity, native.Result]] == [528, 80, 64, 88, 656, 608]
    assert api.abi_version() == 1
    options = api.defaults()
    assert (options.max_iterations, options.max_backtracks, options.flags) == (24, 12, 0)
    assert options.maximum_step == .25
    assert options.normalized_position_tolerance == options.orientation_tolerance == 1e-13


def test_calibrated_c_fk_jacobian_and_owned_input(api):
    case = case_input()
    handle, supplied = create(api, case)
    try:
        # Creation owns its geometry; subsequent caller-buffer edits are irrelevant.
        supplied.dh[2][1] += 1000.
        commands = np.array(case["x"])*np.array(case["senses"])
        before = commands.tobytes()
        output, jacobian = np.empty((4, 4)), np.empty((6, 6))
        native.check(api.fk(handle, pointer(commands), pointer(output)))
        native.check(api.jacobian(handle, pointer(commands), pointer(jacobian)))
        assert commands.tobytes() == before
        with mp.workdps(110):
            pose, derivative = _oracle(case)
            expected = np.array(pose.tolist(), dtype=float)
            expected[:3, 3] *= case["scale"]
            expected_j = np.array(derivative.tolist(), dtype=float)*np.array(case["senses"])
            expected_j[:3] *= case["scale"]
        np.testing.assert_allclose(output, expected, atol=5e-10, rtol=0)
        np.testing.assert_allclose(jacobian, expected_j, atol=5e-10, rtol=0)
        commands[2] = math.nan
        sentinel = np.full((4, 4), 1234.5)
        before = sentinel.tobytes()
        assert api.fk(handle, pointer(commands), pointer(sentinel)) == -1
        assert sentinel.tobytes() == before
    finally:
        api.destroy(handle)
    api.destroy(None)


def test_calibrated_c_query_owns_proofs_after_model_close(api):
    case = case_input()
    handle, _ = create(api, case)
    seeds = np.array([np.array(case["x"])*np.array(case["senses"])+.001], dtype=np.float64)
    before = seeds.tobytes()
    query_handle = query(api, handle, case, seeds)
    api.destroy(handle)
    try:
        summary = read(api, query_handle)
        assert summary.coverage == 2 and summary.seed_count == 1
        assert summary.numerical_count == summary.numerical_inside_count == summary.local_count == summary.admitted_count == 1
        assert summary.evaluations <= 1+24*13
        assert seeds.tobytes() == before
        identity = read(api, query_handle, native.Identity)
        assert identity.model_schema == 2 and identity.model_word_count == 70
        assert identity.model_words[69] == case.get("limit_units", 0)
        assert tuple(identity.target_words) == tuple(np.array(case["target"]).view(np.uint64))
        record = read(api, query_handle, native.Result, 0)
        assert record.status == 0 and record.flags & 31 == 31
        corrected_case = {**case, "x": list(record.increments_rad)}
        with mp.workdps(160):
            _check_calibrated_certificate(corrected_case, proof_record(record))
    finally:
        api.query_destroy(query_handle)
    api.query_destroy(None)


@pytest.mark.parametrize("kind,method", [(native.Summary, "summary_read"),
                                        (native.Identity, "identity_read"),
                                        (native.Result, "result_read")])
@pytest.mark.parametrize("bad", ["size", "version", "reserved"])
def test_calibrated_c_readers_fail_atomically(api, kind, method, bad):
    case = case_input()
    handle, _ = create(api, case)
    seeds = np.array([np.array(case["x"])*np.array(case["senses"])])
    q = query(api, handle, case, seeds)
    try:
        buffer = (ct.c_ubyte*(ct.sizeof(kind)+32))()
        ct.memset(ct.addressof(buffer), 0xA7, ct.sizeof(buffer))
        value = kind.from_buffer(buffer, 16)
        ct.memset(ct.addressof(value), 0, ct.sizeof(kind))
        value.struct_size, value.version = ct.sizeof(kind), 1
        if bad == "size": value.struct_size -= 8
        elif bad == "version": value.version = 2
        else: value.reserved[-1] = 1
        before = bytes(buffer)
        args = [q, 0, ct.byref(value)] if kind is native.Result else [q, ct.byref(value)]
        assert getattr(api, method)(*args) == -1
        assert bytes(buffer) == before
        value.struct_size, value.version = ct.sizeof(kind), 1
        value.reserved[-1] = 0
        native.check(getattr(api, method)(*args))
        assert bytes(buffer)[:16] == bytes(buffer)[-16:] == bytes([0xA7])*16
    finally:
        api.query_destroy(q)
        api.destroy(handle)


def test_calibrated_c_creation_rejects_layouts_and_seed_overflow_atomically(api):
    case = case_input()
    value = model_input(case)
    output = ct.c_void_p(0xA5A5)
    for bad in ["size", "version", "reserved", "units", "scale"]:
        value = model_input(case)
        if bad == "size": value.struct_size -= 8
        elif bad == "version": value.version = 2
        elif bad == "reserved": value.reserved[0] = 1
        elif bad == "units": value.limit_units = 2
        else: value.length_scale = 0
        before = bytes(value)
        assert api.create(ct.byref(value), ct.byref(output)) == -1
        assert output.value == 0xA5A5 and bytes(value) == before
    handle, _ = create(api, case)
    try:
        target = target_input(case)
        seeds = np.zeros((1, 6))
        assert api.query_create(handle, ct.byref(target), pointer(seeds), 513, None, ct.byref(output)) == -1
        assert output.value == 0xA5A5
        options = api.defaults(); options.flags = 2
        assert api.query_create(handle, ct.byref(target), pointer(seeds), 0, ct.byref(options), ct.byref(output)) == -1
        assert output.value == 0xA5A5
    finally:
        api.destroy(handle)


def test_calibrated_c_zero_and_max_seed_snapshots_are_inconclusive(api):
    case = case_input()
    handle, _ = create(api, case)
    options = api.defaults()
    options.max_iterations = 0
    options.flags = 1
    queries, summaries = [], []
    try:
        for count in [0, 1, 512]:
            seeds = np.tile(np.array(case["x"])*np.array(case["senses"])+.001, (count, 1))
            q = query(api, handle, case, seeds, options)
            queries.append(q)
            summary = read(api, q)
            summaries.append(summary)
            assert summary.seed_count == summary.evaluations == count
            assert summary.coverage == 2 and summary.numerical_count == summary.local_count == summary.admitted_count == 0
            if count:
                record = read(api, q, native.Result, count-1)
                assert record.status == 4 and not record.flags & 8
                assert math.isnan(record.existence_radius)
        base, one, maximum = [summary.owned_payload_bytes for summary in summaries]
        assert maximum-base == 512*(one-base)
    finally:
        for q in queries: api.query_destroy(q)
        api.destroy(handle)


def test_calibrated_c_proof_option_preserves_numerical_commands(api):
    case = case_input()
    handle, _ = create(api, case)
    seeds = np.array([np.array(case["x"])*np.array(case["senses"])+.001])
    queries = []
    try:
        options = api.defaults()
        for flags in [0, 1]:
            options.flags = flags
            queries.append(query(api, handle, case, seeds, options))
        records = [read(api, q, native.Result, 0) for q in queries]
        assert bytes(records[0].commands_rad) == bytes(records[1].commands_rad)
        assert records[0].flags & 8 and not records[1].flags & 8
        assert records[0].evaluations == records[1].evaluations
    finally:
        for q in queries: api.query_destroy(q)
        api.destroy(handle)


@pytest.mark.parametrize("bad", ["target_size", "target_version", "target_reserved", "zero_quaternion",
                                "seed_nan", "options_size", "options_version", "options_reserved", "iterations"])
def test_calibrated_c_query_invalid_inputs_leave_output_unchanged(api, bad):
    case = case_input()
    handle, _ = create(api, case)
    value, options = target_input(case), api.defaults()
    proposed = np.array([np.array(case["x"])*np.array(case["senses"])])
    if bad == "target_size": value.struct_size -= 8
    elif bad == "target_version": value.version = 2
    elif bad == "target_reserved": value.reserved[0] = 1
    elif bad == "zero_quaternion": value.quaternion[:] = [0.]*4
    elif bad == "seed_nan": proposed[0, 0] = math.nan
    elif bad == "options_size": options.struct_size -= 8
    elif bad == "options_version": options.version = 2
    elif bad == "options_reserved": options.reserved[0] = 1
    else: options.max_iterations = 129
    output = ct.c_void_p(0xB5B5)
    before = (bytes(value), bytes(options), proposed.tobytes())
    try:
        assert api.query_create(handle, ct.byref(value), pointer(proposed), 1,
                                ct.byref(options), ct.byref(output)) == -1
        assert output.value == 0xB5B5
        assert (bytes(value), bytes(options), proposed.tobytes()) == before
    finally:
        api.destroy(handle)
