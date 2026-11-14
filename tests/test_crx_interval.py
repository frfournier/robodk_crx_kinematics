# SPDX-License-Identifier: Apache-2.0
# SPDX-FileCopyrightText: 2026 Centre Technologique en Aérospatiale
"""Independent rational and high-precision containment of native enclosures."""

from fractions import Fraction as F
import itertools
import json
import math
import os
from pathlib import Path
import random
import subprocess
import sys

import mpmath as mp
import pytest


@pytest.fixture(scope="module")
def interval_probe():
    configured = os.environ.get("CRXKIN_INTERVAL_PROBE_PATH")
    suffix = ".exe" if sys.platform == "win32" else ""
    path = Path(configured) if configured else Path(__file__).resolve().parents[1] / f"build/Release/crx_interval_tests{suffix}"
    if configured:
        assert path.is_file(), f"Configured interval probe is missing: {path}"
    elif not path.is_file():
        pytest.skip("Build crx_interval_tests or set CRXKIN_INTERVAL_PROBE_PATH")

    def run(requests):
        text = "\n".join(" ".join([operation, *(repr(float(x)) for x in values)])
                         for operation, values in requests)+"\n"
        result = subprocess.run([str(path), "--batch"], input=text, text=True,
                                capture_output=True, check=True, timeout=40)
        answers = [json.loads(line) for line in result.stdout.splitlines()]
        assert len(answers) == len(requests)
        return answers
    return run


def _interval(rng, distribution):
    if distribution == "ordinary":
        centre = rng.uniform(-100, 100)
        radius = 10**rng.uniform(-16, 1)
        return centre-radius, centre+radius
    if distribution == "subnormal":
        tiny = math.ulp(0.)
        return tuple(sorted(rng.randint(-1000, 1000)*tiny for _ in range(2)))
    return tuple(sorted(math.ldexp(rng.uniform(-1, 1), rng.randint(-1070, 1020)) for _ in range(2)))


@pytest.mark.parametrize("operation", ["add", "sub", "mul", "div"])
@pytest.mark.parametrize("distribution", ["ordinary", "subnormal", "exponent"])
def test_all_endpoint_extrema_contained_exactly(interval_probe, operation, distribution):
    rng = random.Random(f"interval-{operation}-{distribution}-20261113")
    boxes = [(_interval(rng, distribution), _interval(rng, distribution)) for _ in range(350)]
    answers = interval_probe([(operation, [*a, *b]) for a, b in boxes])
    checked = 0
    for (a, b), answer in zip(boxes, answers, strict=True):
        result = answer["values"][0]
        if operation == "div" and b[0] <= 0 <= b[1]:
            assert result is None
            continue
        arithmetic = {"add": lambda a, b: a+b, "sub": lambda a, b: a-b,
                      "mul": lambda a, b: a*b, "div": lambda a, b: a/b}[operation]
        exact = [arithmetic(F(x), F(y)) for x, y in itertools.product(a, b)]
        if result is None:
            # A finite enclosure may be unavailable near binary64 overflow.
            # Invalid ordinary-range operations must not silently pass this test.
            assert max(abs(x) for x in exact) >= F(math.nextafter(sys.float_info.max, 0.))
            continue
        lo, hi = map(F, result)
        assert lo <= min(exact) <= max(exact) <= hi, (a, b, result)
        checked += 1
    assert checked > (0 if operation == "div" and distribution == "subnormal" else 100)


def test_fail_closed_domains_and_signed_zero(interval_probe):
    tiny = math.ulp(0.)
    bad = [("div", [1, 1, -tiny, tiny]), ("div", [0, 0, -0., 0.]),
           ("sqrt", [-tiny, 1]), ("sqrt", [-1, -1]),
           ("add", [sys.float_info.max]*4), ("mul", [sys.float_info.max]*4),
           ("add", [math.nan, 1, 0, 0]), ("sub", [2, 1, 0, 0]),
           ("sqrt", [0, math.inf]), ("trig", [-math.inf, math.inf])]
    for answer in interval_probe(bad):
        assert answer["values"][0] is None
    requests = [(op, [a, a, b, b]) for op in ["add", "sub", "mul"]
                for a, b in itertools.product([-0., 0.], repeat=2)]
    for answer in interval_probe(requests):
        assert answer["values"][0][0] <= 0 <= answer["values"][0][1]


def test_sqrt_and_square_rational_containment(interval_probe):
    rng = random.Random(2026111301)
    boxes = [tuple(sorted(abs(x) for x in _interval(rng, "exponent"))) for _ in range(400)]
    boxes += [(0., math.ulp(0.)), (0., 0.), (1., 1.), (sys.float_info.max, sys.float_info.max)]
    answers = interval_probe([("sqrt", a) for a in boxes])
    for (a, b), answer in zip(boxes, answers, strict=True):
        lo, hi = map(F, answer["values"][0])
        assert 0 <= lo and lo*lo <= F(a) and hi*hi >= F(b)
    boxes = [_interval(rng, "ordinary") for _ in range(300)] + [(-math.ulp(0.), math.ulp(0.))]
    for a, answer in zip(boxes, interval_probe([("square", a) for a in boxes]), strict=True):
        expected = [F(x)**2 for x in a]
        if a[0] <= 0 <= a[1]:
            expected.append(F(0))
        lo, hi = map(F, answer["values"][0])
        assert lo <= min(expected) <= max(expected) <= hi


def _mp(value):
    exact = F(value)
    return mp.mpf(exact.numerator)/exact.denominator


def _check_trig(angles, answers):
    for (lo, hi), answer in zip(angles, answers, strict=True):
        candidates = [_mp(lo), _mp(hi), (_mp(lo)+_mp(hi))/2]
        # Include every interior sine/cosine extremum for narrow intervals.
        if hi-lo < 8:
            first = int(mp.ceil(_mp(lo)/(mp.pi/2)))
            last = int(mp.floor(_mp(hi)/(mp.pi/2)))
            candidates += [k*mp.pi/2 for k in range(first, last+1)]
        for function, interval in zip([mp.sin, mp.cos], answer["values"][:2], strict=True):
            assert interval is not None
            lower, upper = map(_mp, interval)
            for x in candidates:
                exact = function(x)
                assert lower <= exact <= upper, (lo, hi, interval, mp.nstr(exact, 50))


def test_trig_reduction_extrema_and_neighbours(interval_probe):
    rng = random.Random(2026111302)
    points = [0., -0., math.ulp(0.), -math.ulp(0.), 1e20, -1e20,
              sys.float_info.max, -sys.float_info.max, 2**20, math.nextafter(2**20, math.inf)]
    for k in range(-40, 41):
        boundary = k*math.pi/4
        points += [math.nextafter(boundary, -math.inf), boundary, math.nextafter(boundary, math.inf)]
    points += [rng.uniform(-2**20, 2**20) for _ in range(500)]
    angles = [(x, x) for x in points]
    with mp.workdps(200):
        _check_trig(angles, interval_probe([("trig", angle) for angle in angles]))


def test_trig_intervals_cover_interior_extrema(interval_probe):
    rng = random.Random(2026111303)
    angles = [_interval(rng, "ordinary") for _ in range(250)]
    angles += [(0., math.pi), (-math.pi, math.pi), (1e20, 1e20+1e6),
               (-2., 2.), (math.pi/2-1e-8, math.pi/2+1e-8)]
    with mp.workdps(200):
        _check_trig(angles, interval_probe([("trig", angle) for angle in angles]))


def test_principal_reduction_retains_exact_raw_angle(interval_probe):
    rng = random.Random(2026111304)
    points = [rng.uniform(-1e6, 1e6) for _ in range(400)]
    points += [math.pi, math.nextafter(math.pi, math.inf), -math.pi, 1e20, -1e20]
    angles = [(x, x) for x in points]
    angles += [(math.nextafter(math.pi, -math.inf), math.nextafter(math.pi, math.inf))]
    answers = interval_probe([("phase", angle) for angle in angles])
    with mp.workdps(200):
        for angle, answer in zip(angles, answers, strict=True):
            lower, upper = map(_mp, answer["values"][2])
            for raw in angle:
                multiple = mp.floor((_mp(raw)+mp.pi)/(2*mp.pi))
                # Keep an already principal dyadic input exact. Adding and
                # subtracting an approximate pi can invent oracle roundoff at
                # an exactly represented singleton enclosure.
                exact = _mp(raw) if multiple == 0 else _mp(raw)-multiple*2*mp.pi
                assert lower <= exact <= upper
            if answer["seam"]:
                assert lower <= -mp.pi and upper >= mp.pi
    assert answers[-1]["seam"]


def test_matrix_products_norms_and_vector_norm(interval_probe):
    rng = random.Random(2026111305)
    boxes = [[_interval(rng, "ordinary") for _ in range(8)] for _ in range(100)]
    answers = interval_probe([("matrix", list(itertools.chain.from_iterable(box))) for box in boxes])
    for box, answer in zip(boxes, answers, strict=True):
        left, right = box[:4], box[4:]
        for row, col in itertools.product(range(2), repeat=2):
            products = [[F(a)*F(b) for a, b in itertools.product(left[2*row+k], right[2*k+col])] for k in range(2)]
            expected_lo = sum(min(p) for p in products)
            expected_hi = sum(max(p) for p in products)
            lo, hi = map(F, answer["values"][2*row+col])
            assert lo <= expected_lo <= expected_hi <= hi
        maxima = [max(abs(F(x)) for x in item) for item in left]
        assert F(answer["values"][4][1]) >= max(maxima)
        assert F(answer["values"][5][1]) >= max(sum(maxima[:2]), sum(maxima[2:]))
    vectors = [[_interval(rng, "ordinary") for _ in range(3)] for _ in range(100)]
    for vector, answer in zip(vectors, interval_probe([("norm", list(itertools.chain.from_iterable(v))) for v in vectors]), strict=True):
        lo, hi = map(F, answer["values"][0])
        for point in itertools.product(*vector):
            exact_squared = sum(F(x)**2 for x in point)
            assert lo**2 <= exact_squared <= hi**2
