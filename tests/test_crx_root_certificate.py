# SPDX-License-Identifier: Apache-2.0
# SPDX-FileCopyrightText: 2026 Centre Technologique en Aérospatiale
"""Independent exact Bernstein, Sturm and chart/phase enclosure validation."""

import copy
from fractions import Fraction as F
import itertools
import json
import math
import os
from pathlib import Path
import random
import struct
import subprocess
import sys

import mpmath as mp
import pytest

import crx_exact_reference as ref
import polynomial_reference as sturm
from test_crx_ball_certificate import generated, prepare_reference
from test_crx_exact_target import case, contains, flatten, pose


@pytest.fixture(scope="module")
def cover_probe():
    configured = os.environ.get("CRXKIN_COVER_PROBE_PATH")
    suffix = ".exe" if sys.platform == "win32" else ""
    path = Path(configured) if configured else Path(__file__).resolve().parents[1]/f"build/Release/crx_root_certificate_tests{suffix}"
    if configured:
        assert path.is_file(), f"Configured root certificate probe missing: {path}"
    elif not path.is_file():
        pytest.skip("Build crx_root_certificate_tests or set CRXKIN_COVER_PROBE_PATH")

    def run(requests):
        lines = []
        for request in requests:
            options = [request.get(key, default) for key, default in [
                ("max_nodes", 8192), ("max_depth", 40), ("max_leaves", 4096), ("max_chart_attempts", 8)]]
            if "input" in request:
                values = [2 if request.get("reference") else 1, *options, *flatten(request["input"])]
            else:
                coefficients = lambda values, count: [pair for value in [*values, *([0.]*(count-len(values)))]
                    for pair in (value if isinstance(value, (tuple, list)) else (value, value))]
                values = [0, *options, *coefficients(request["p"], 9), *coefficients(request.get("q", [1.]), 5)]
            lines.append(" ".join(repr(float(x)) if i > 4 else str(x) for i, x in enumerate(values)))
        result = subprocess.run([str(path), "--batch"], input="\n".join(lines)+"\n",
                                text=True, capture_output=True, check=True, timeout=120)
        answers = [json.loads(line) for line in result.stdout.splitlines()]
        assert len(answers) == len(requests)
        return answers
    return run


def evaluate(coefficients, point):
    value = 0
    for coefficient in reversed(coefficients): value = value*point+coefficient
    return value


def bernstein(coefficients, left, right):
    # Direct binomial expansion in the reference's exact rational/mp domain;
    # no production de Casteljau intermediates or stored enclosures are reused.
    degree = len(coefficients)-1
    translated = [(right-left)**j*sum(coefficients[i]*math.comb(i, j)*left**(i-j)
                    for i in range(j, degree+1)) for j in range(degree+1)]
    return [sum(F(math.comb(k, j), math.comb(degree, j))*translated[j]
                for j in range(k+1)) for k in range(degree+1)]


def padded(values, count):
    return [*values, *([F(0)]*(count-len(values)))]


def check_partition(answer, power, determinant, expected=None):
    leaves = answer["leaves"]
    assert leaves and leaves[0]["t"][0] == -1 and leaves[-1]["t"][1] == 1
    assert all(a["t"][1] == b["t"][0] for a, b in itertools.pairwise(leaves))
    assert answer["roots"] == sum(leaf["kind"] == 1 for leaf in leaves)
    assert answer["exclusions"] == sum(leaf["kind"] == 0 for leaf in leaves)
    assert answer["unresolved"] == sum(leaf["kind"] == 2 for leaf in leaves)
    assert (answer["status"] == 0) == (answer["unresolved"] == 0)
    assert answer["depth"] == max(leaf["depth"] for leaf in leaves)
    derivative = [(i+1)*power[i+1] for i in range(8)]
    root_leaves = 0
    for leaf in leaves:
        a, b = map(F, leaf["t"])
        assert a < b and leaf["depth"] <= 50
        for field, coefficients in [("p", power), ("d", derivative), ("q", determinant)]:
            for interval, value in zip(leaf[field], bernstein(coefficients, a, b), strict=True):
                contains(interval, value)
        for interval, point in zip(leaf["endpoints"], [a, b], strict=True):
            if interval is not None: contains(interval, evaluate(power, point))
            else: assert leaf["kind"] == 2
        signs = lambda values: 1 if all(lo > 0 for lo, _ in values) else (-1 if all(hi < 0 for _, hi in values) else 0)
        if leaf["kind"] == 0:
            assert signs(leaf["p"]) != 0
            if expected is not None: assert not any(a <= value <= b for value in expected)
        elif leaf["kind"] == 1:
            root_leaves += 1
            assert leaf["reason"] == 0
            assert signs(leaf["d"]) == leaf["derivative_sign"] != 0
            assert signs(leaf["q"]) == leaf["determinant_sign"] != 0
            endpoint_signs = [1 if lo > 0 else -1 if hi < 0 else 0 for lo, hi in leaf["endpoints"]]
            assert endpoint_signs[0] == -endpoint_signs[1] != 0
            if expected is not None: assert sum(a < value < b for value in expected) == 1
        else:
            assert leaf["reason"] != 0
    if expected is not None and answer["status"] == 0:
        assert root_leaves == sum(-1 <= value <= 1 for value in expected)


def polynomial(roots):
    result = [F(1)]
    for root in roots:
        result = ref.poly_mul(result, [-F(root), F(1)])
    assert all(F(float(value)) == value for value in result)
    return result


@pytest.mark.parametrize("roots", [[F(0)], [F(-1, 2), F(0), F(1, 2)],
    [F(i, 16) for i in [-13, -10, -6, -2, 1, 5, 9, 14]], [F(3, 8)], []])
def test_exact_rational_bernstein_and_dyadic_root_partition(cover_probe, roots):
    coefficients = polynomial(roots)
    answer = cover_probe([{"p": coefficients}])[0]
    assert answer["status"] == 0
    check_partition(answer, padded(coefficients, 9), padded([F(1)], 5), roots)


def test_repeated_common_and_boundary_roots_cannot_complete(cover_probe):
    examples = [([F(-1, 2), F(-1, 2), F(1, 2), F(1, 2)], [1]),
                ([F(0)], [0, 1]), ([F(-1)], [1]), ([F(1)], [1])]
    requests = [{"p": polynomial(roots), "q": q, "max_depth": 12, "max_nodes": 128}
                for roots, q in examples]
    for answer, request, (roots, q) in zip(cover_probe(requests), requests, examples, strict=True):
        assert answer["status"] == 1 and answer["unresolved"] > 0
        assert answer["nodes"] <= 128 and len(answer["leaves"]) <= 4096
        check_partition(answer, padded(request["p"], 9), padded(list(map(F, q)), 5))


def test_zero_polynomial_degree_drops_and_uncertain_leading_slot(cover_probe):
    zero, constant, uncertain = cover_probe([{"p": [0.]}, {"p": [-2.]},
        {"p": [1., 0., 0., 0., 0., 0., 0., 0., [-1e-20, 1e-20]]}])
    assert zero["status"] == 1 and zero["nodes"] == 1 and zero["roots"] == 0
    assert constant["status"] == uncertain["status"] == 0
    assert constant["roots"] == uncertain["roots"] == 0
    check_partition(zero, padded([F(0)], 9), padded([F(1)], 5))
    for leading in [-1e-20, 0., 1e-20]:
        check_partition(uncertain, [F(1), *([F(0)]*7), F(leading)], padded([F(1)], 5), [])


def test_all_budget_frontiers_are_retained(cover_probe):
    coefficients = polynomial([F(-1, 2), F(0), F(1, 2)])
    requests = [{"p": coefficients, **options} for options in [
        {"max_nodes": 0}, {"max_nodes": 1}, {"max_depth": 0}, {"max_leaves": 1}]]
    for request, answer in zip(requests, cover_probe(requests), strict=True):
        assert answer["status"] == 1 and answer["unresolved"] > 0
        assert answer["nodes"] <= request.get("max_nodes", 8192)
        assert len(answer["leaves"]) <= request.get("max_leaves", 4096)
        check_partition(answer, padded(coefficients, 9), padded([F(1)], 5))


def witness():
    return {"lengths": [1., .6, .2, .5], "base_height": 0., "encoder_zero": [0.]*6,
            "senses": [1.]*6, "lower_commands": [-4.]*6, "upper_commands": [4.]*6,
            "target": pose([60., -7., 7., -19.], [27/64, 15/32, 55/128]),
            "base": pose(), "tool": pose()}


def chart_coefficients(problem, shift):
    if shift % 4 == 0:
        # Exact quarter-turns avoid fabricating mp.sin(pi) cancellation residue.
        cosine, sine = [(1, 0), (0, 1), (-1, 0), (0, -1)][(shift//4) % 4]
        rotated = dict(problem)
        rotated["v"] = [cosine*x+sine*y for x, y in zip(problem["v"], problem["w"], strict=True)]
        rotated["w"] = [-sine*x+cosine*y for x, y in zip(problem["v"], problem["w"], strict=True)]
        return ref.coefficients(rotated, 0)
    return ref.coefficients(problem, mp.pi*shift/8)


def check_geometry(answer, input, reference=False):
    problem = prepare_reference(input) if reference else ref.prepare(input)
    assert answer["schema"] == (2 if reference else 1)
    words = [format(struct.unpack('<Q', struct.pack('<d', value))[0], 'x') for value in flatten(input)]
    assert answer["words"] == words
    assert answer["complete"] == (answer["status"] == 0)
    assert answer["roots"] == sum(p["roots"] for p in answer["partitions"])
    for chart in range(2):
        exact = chart_coefficients(problem, answer["shift"]+8*chart)
        for name, values in zip(["p", "d", "q"], exact, strict=True):
            for interval, value in zip(answer["polynomials"][chart][name], values, strict=True): contains(interval, value)
        check_partition(answer["partitions"][chart], exact[0], exact[2])
        for leaf in answer["partitions"][chart]["leaves"]:
            alpha = mp.pi*(answer["shift"]+8*chart)/8
            a, b = map(lambda x: ref.number(F(x)), leaf["t"])
            contains(leaf["phase"], alpha+2*mp.atan(a))
            contains(leaf["phase"], alpha+2*mp.atan(b))
    if answer["complete"]:
        # Original chart alpha=0 has rational coefficients for the exact raw
        # quaternion problem. Exact Sturm isolation independently checks all
        # finite roots; its projective infinity root is counted separately.
        power, _, determinant = ref.coefficients(problem, 0)
        roots = sturm.isolate_real_roots(power)
        infinity = power[-1] == 0
        assert answer["roots"] == len(roots)+infinity
        assert all(root.multiplicity == 1 for root in roots)
        if infinity: assert power[-2] != 0 and determinant[-1] != 0
    return problem


def test_exact_sixteen_solution_witness_cover(cover_probe):
    input = witness()
    answer = cover_probe([{"input": input}])[0]
    assert answer["complete"] and answer["roots"] == 8
    with mp.workdps(200): check_geometry(answer, input)


def test_exact_chart_seam_root_forces_shift_without_losing_it(cover_probe):
    input = witness()
    input["lengths"] = [1., .5, .25, .125]
    input["target"] = pose([1., -1., -1., -1.], [1.125, .25, -.5])
    p, d, q = ref.coefficients(ref.prepare(input), 0)
    assert evaluate(p, F(-1)) == 0 and evaluate(d, F(-1)) != 0 and evaluate(q, F(-1)) != 0
    failed, shifted = cover_probe([{"input": input, "max_chart_attempts": 1}, {"input": input}])
    assert not failed["complete"] and failed["partitions"][0]["leaves"][0]["reason"] == 1
    assert shifted["complete"] and shifted["shift"] > 0
    with mp.workdps(200):
        check_geometry(failed, input)
        check_geometry(shifted, input)


def test_exact_degree_drop_retains_simple_projective_infinity_root(cover_probe):
    input = witness()
    input["lengths"] = [1., .5, .25, .125]
    input["target"] = pose([1., -1., 0., 0.], [1.125, .25, -.5])
    power, _, determinant = ref.coefficients(ref.prepare(input), 0)
    assert power[-1] == 0 and power[-2] != 0 and determinant[-1] != 0
    answer = cover_probe([{"input": input}])[0]
    assert answer["complete"]
    with mp.workdps(200): check_geometry(answer, input)


@pytest.mark.parametrize("reference", [False, True])
def test_independent_raw_geometry_frames_and_unreachable_cover(cover_probe, reference):
    inputs = [case(i) for i in range(4)]
    far = witness()
    far["target"]["position"] = [20., 30., -40.]
    inputs.append(far)
    requests = [{"input": input, "reference": reference, "max_nodes": 2048, "max_depth": 28} for input in inputs]
    with mp.workdps(200):
        for input, answer in zip(inputs, cover_probe(requests), strict=True):
            assert answer["nodes"] <= 2048
            check_geometry(answer, input, reference)
        assert answer["complete"] and answer["roots"] == 0


def test_multiple_root_limits_and_circle_budgets_remain_inconclusive(cover_probe):
    inputs = [generated(20, wrist=x, shoulder=0.)["input"] for x in [1e-8, 1e-12, 0.]]
    inputs += [witness()]
    requests = [{"input": input, "max_nodes": 64, "max_depth": 12} for input in inputs]
    requests[-1]["max_nodes"] = 0
    with mp.workdps(200):
        for request, answer in zip(requests, cover_probe(requests), strict=True):
            assert not answer["complete"] and answer["nodes"] <= request["max_nodes"]
            assert sum(p["unresolved"] for p in answer["partitions"]) > 0
            check_geometry(answer, request["input"])


def test_unavailable_coefficients_retain_the_entire_unresolved_circle(cover_probe):
    input = witness()
    input["target"]["position"] = [1e100, -1e100, 1e100]
    answer = cover_probe([{"input": input}])[0]
    assert answer["status"] == 4 and not answer["complete"]
    assert answer["entire_circle_unresolved"] and answer["roots"] == 0
    assert all(not partition["leaves"] for partition in answer["partitions"])
