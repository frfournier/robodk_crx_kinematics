"""Independent exact root counts and differential numerical-candidate tests."""

from fractions import Fraction
import json
import os
from pathlib import Path
import subprocess

import pytest

from polynomial_reference import (
    IsolationWorkLimit,
    ZeroPolynomialError,
    isolate_real_roots,
)


def multiply(a, b):
    result = [Fraction(0)] * (len(a) + len(b) - 1)
    for i, x in enumerate(a):
        for j, y in enumerate(b):
            result[i + j] += x * y
    return result


def from_roots(roots):
    result = [Fraction(1)]
    for root in roots:
        result = multiply(result, [-Fraction(root), Fraction(1)])
    return result


@pytest.mark.parametrize("roots", [
    [0], [-3, 0, 2], [Fraction(1, 3), Fraction(-2, 7)],
    [1] * 8, [-2] * 3 + [0] * 2 + [3] * 3,
    [Fraction(1), Fraction(1) + Fraction(1, 2**40)],
    [Fraction(i, 4) for i in range(1, 9)],
])
def test_exact_factor_catalogue(roots):
    roots = list(map(Fraction, roots))
    intervals = isolate_real_roots(from_roots(roots))
    distinct = sorted(set(roots))
    assert len(intervals) == len(distinct)
    for interval, root in zip(intervals, distinct):
        assert interval.left <= root <= interval.right
        assert interval.multiplicity == roots.count(root)
        assert interval.right - interval.left <= Fraction(1, 2**48)
    assert all(a.right < b.left for a, b in zip(intervals, intervals[1:]))


def test_irrational_roots_and_complex_factors():
    # (x^2-2)^3 (x^2+1): two real roots, both with multiplicity three.
    p = [1]
    for factor in ([-2, 0, 1], [-2, 0, 1], [-2, 0, 1], [1, 0, 1]):
        p = multiply(p, factor)
    intervals = isolate_real_roots(p)
    assert len(intervals) == 2
    negative, positive = intervals
    assert negative.left < negative.right < 0
    assert negative.left**2 > 2 > negative.right**2
    assert 0 < positive.left < positive.right
    assert positive.left**2 < 2 < positive.right**2
    assert [root.multiplicity for root in intervals] == [3, 3]


def test_exact_statuses_and_budgets():
    assert isolate_real_roots([7, 0, 0]) == []
    assert isolate_real_roots([1, 0, 1], max_subdivisions=0) == []
    with pytest.raises(ZeroPolynomialError):
        isolate_real_roots([0, 0, 0])
    with pytest.raises(IsolationWorkLimit):
        isolate_real_roots([-2, 0, 1], max_subdivisions=0)
    with pytest.raises(IsolationWorkLimit):
        isolate_real_roots(from_roots([0, Fraction(1, 2**50)]), max_subdivisions=10)
    with pytest.raises(ValueError):
        isolate_real_roots([1, 1], width=0)


def test_exact_reference_does_not_trim_small_leading_term():
    roots = isolate_real_roots([1, Fraction(1, 2**60), 0])
    assert len(roots) == 1
    assert roots[0].left <= -(2**60) <= roots[0].right


@pytest.fixture(scope="module")
def native_probe():
    configured = os.environ.get("CRXKIN_ROOT_PROBE_PATH")
    path = (Path(configured) if configured else
            Path(__file__).resolve().parents[1] / "build/Release/crx_canonical_tests.exe")
    if configured:
        assert path.is_file(), f"Configured root probe is missing: {path}"
    elif not path.is_file():
        pytest.skip("Build crx_canonical_tests or set CRXKIN_ROOT_PROBE_PATH")

    def run(coefficients):
        completed = subprocess.run(
            [str(path), "--roots", *(repr(float(value)) for value in coefficients)],
            check=True, capture_output=True, text=True, timeout=20,
        )
        return json.loads(completed.stdout)
    return run


def match_simple_roots(reference, candidates, tolerance=1e-7):
    """Small corpus matcher; count and one-to-one matching detect dropped roots."""
    assert len(reference) == len(candidates)
    remaining = [complex(real, imaginary) for real, imaginary, _ in candidates]
    for root in reference:
        assert root.multiplicity == 1
        midpoint = float((root.left + root.right) / 2)
        nearest = min(range(len(remaining)), key=lambda i: abs(remaining[i] - midpoint))
        assert abs(remaining.pop(nearest) - midpoint) < tolerance


@pytest.mark.parametrize("degree", range(1, 9))
@pytest.mark.parametrize("scale", [Fraction(1, 2**400), -1, 2**400],
                         ids=["small", "negative", "large"])
def test_native_simple_roots_against_exact_reference(native_probe, degree, scale):
    p = [value * scale for value in from_roots(Fraction(i, 4) for i in range(1, degree + 1))]
    # Reference exactly the binary coefficients received by the native process.
    represented = [float(value) for value in p]
    reference = isolate_real_roots(represented)
    result = native_probe(represented)
    assert result["status"] == "Candidates"
    assert result["degree"] == degree
    match_simple_roots(reference, result["roots"])
    assert all(0 <= residual < 1e-13 for _, _, residual in result["roots"])


def test_differential_check_detects_missing_and_duplicated_candidates(native_probe):
    p = from_roots([-1, 0, 1])
    reference = isolate_real_roots(p)
    candidates = native_probe(p)["roots"]
    match_simple_roots(reference, candidates)
    with pytest.raises(AssertionError):
        match_simple_roots(reference, candidates[:-1])
    with pytest.raises(AssertionError):
        match_simple_roots(reference, [candidates[0], candidates[0], candidates[2]])


@pytest.mark.parametrize("degree", [2, 4, 8])
def test_native_repeated_root_candidates_are_not_multiplicity_certificates(native_probe, degree):
    p = from_roots([1] * degree)
    reference = isolate_real_roots(p)
    assert len(reference) == 1 and reference[0].multiplicity == degree
    result = native_probe(p)
    assert result["status"] == "Candidates"
    assert len(result["roots"]) == degree
    # Repeated roots are ill-conditioned: test the cluster, not native reality
    # or exact multiplicity, neither of which this backend claims to determine.
    assert all(abs(complex(real, imaginary) - 1) < 0.05
               for real, imaginary, _ in result["roots"])


def test_near_real_complex_pair_is_preserved(native_probe):
    p = [1e-20, 0, 1]
    assert isolate_real_roots(p) == []
    result = native_probe(p)
    assert result["status"] == "Candidates"
    assert len(result["roots"]) == 2
    assert sorted(root[1] for root in result["roots"]) == pytest.approx([-1e-10, 1e-10], abs=1e-24)


@pytest.mark.parametrize("p", [
    [-2, 0, 1],
    from_roots([1, Fraction(1025, 1024)]),
    from_roots([Fraction(1, 100), 10000]),
])
def test_native_nonuniform_and_irrational_roots(native_probe, p):
    represented = [float(value) for value in p]
    result = native_probe(represented)
    assert result["status"] == "Candidates"
    match_simple_roots(isolate_real_roots(represented), result["roots"])


def test_native_mixed_real_and_complex_roots(native_probe):
    p = multiply(from_roots([-1, 0, 2]), [1, 0, 1])
    result = native_probe(p)
    assert result["status"] == "Candidates" and len(result["roots"]) == 5
    real_candidates = [root for root in result["roots"] if abs(root[1]) < 1e-10]
    match_simple_roots(isolate_real_roots(p), real_candidates)
    nonreal = [complex(real, imaginary) for real, imaginary, _ in result["roots"]
               if abs(imaginary) >= 1e-10]
    assert sorted(root.imag for root in nonreal) == pytest.approx([-1, 1])


@pytest.mark.parametrize("p,status,degree", [
    ([0, 0, 0], "ZeroPolynomial", -1),
    ([1, 0, 0], "ConstantPolynomial", 0),
    ([1, 0, 1e-20], "UnresolvedLeadingCoefficient", 2),
    ([float.fromhex('0x0.0000000000001p-1022'), 1e300], "NumericalRangeFailure", 1),
])
def test_native_non_candidate_statuses(native_probe, p, status, degree):
    result = native_probe(p)
    assert result == {"status": status, "degree": degree, "roots": []}
