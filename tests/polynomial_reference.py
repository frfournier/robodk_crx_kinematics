"""Offline exact real-root reference for degree-at-most-eight polynomials.

Ascending rational coefficients, square-free factorization, and Sturm counts.
Fraction(float) describes the stored binary value exactly, not the uncertain
polynomial from which floating-point geometric coefficients were computed.
This reference certifies only that supplied rational polynomial. It is not a
runtime IK backend and makes no bound on rational intermediate bit lengths.
"""

from dataclasses import dataclass
from fractions import Fraction
from math import ceil


class ZeroPolynomialError(ValueError):
    """Every real number is a root; a finite list cannot represent the result."""


class IsolationWorkLimit(RuntimeError):
    """Subdivision budget exhausted; no partial list is published."""


@dataclass(frozen=True)
class RootInterval:
    left: Fraction
    right: Fraction
    multiplicity: int


def _trim(p):
    p = tuple(p)
    while p and p[-1] == 0:
        p = p[:-1]
    return p


def _evaluate(p, x):
    value = Fraction(0)
    for coefficient in reversed(p):
        value = value * x + coefficient
    return value


def _derivative(p):
    return tuple(i * p[i] for i in range(1, len(p)))


def _divide(p, q):
    if not q:
        raise ZeroDivisionError("zero polynomial divisor")
    remainder = list(p)
    quotient = [Fraction(0)] * max(0, len(p) - len(q) + 1)
    while remainder and len(remainder) >= len(q):
        offset = len(remainder) - len(q)
        factor = remainder[-1] / q[-1]
        quotient[offset] = factor
        for i, coefficient in enumerate(q):
            remainder[offset + i] -= factor * coefficient
        remainder = list(_trim(remainder))
    return _trim(quotient), tuple(remainder)


def _exact_quotient(p, q):
    quotient, remainder = _divide(p, q)
    assert not remainder
    return quotient


def _monic(p):
    return tuple(value / p[-1] for value in p) if p else ()


def _gcd(p, q):
    while q:
        p, q = q, _divide(p, q)[1]
    return _monic(p)


def _square_free_factors(p):
    common = _gcd(p, _derivative(p))
    remaining = _exact_quotient(p, common)
    radical = remaining
    factors = []
    multiplicity = 1
    while len(remaining) > 1:
        shared = _gcd(remaining, common)
        factor = _exact_quotient(remaining, shared)
        if len(factor) > 1:
            factors.append((factor, multiplicity))
        remaining = shared
        common = _exact_quotient(common, shared)
        multiplicity += 1
    return radical, factors


def _sturm(p):
    sequence = [p, _derivative(p)]
    while sequence[-1]:
        remainder = _divide(sequence[-2], sequence[-1])[1]
        if not remainder:
            break
        # Positive rescaling controls fraction sizes without changing signs.
        scale = abs(remainder[-1])
        sequence.append(tuple(-value / scale for value in remainder))
    return sequence


def _variations(sequence, x):
    signs = []
    for p in sequence:
        value = _evaluate(p, x)
        if value:
            signs.append(value > 0)
    return sum(a != b for a, b in zip(signs, signs[1:]))


def isolate_real_roots(coefficients, *, width=Fraction(1, 2**48),
                       max_subdivisions=8192):
    """Return disjoint rational intervals with exact multiplicities.

    Non-point intervals have nonroot endpoints and exactly one distinct real
    root in their interior. Point intervals are exact rational roots. Zero
    polynomials and exhausted work budgets raise separate exceptions.
    """
    p = _trim(Fraction(value) for value in coefficients)
    width = Fraction(width)
    if width <= 0 or not isinstance(max_subdivisions, int) or max_subdivisions < 0:
        raise ValueError("positive width and nonnegative integer budget required")
    if len(p) > 9:
        raise ValueError("reference supports degree at most eight")
    if not p:
        raise ZeroPolynomialError("identically zero polynomial")
    if len(p) == 1:
        return []
    p = _monic(p)
    radical, factors = _square_free_factors(p)
    sequence = _sturm(radical)

    def count_open(left, right):
        # Sturm variation at an exact root equals the right-hand limit.
        return (_variations(sequence, left) - _variations(sequence, right)
                - int(_evaluate(radical, right) == 0))

    bound = Fraction(1 + ceil(max(abs(value) for value in radical[:-1])))
    pending = [(-bound, bound, count_open(-bound, bound))]
    intervals = []
    subdivisions = 0
    while pending:
        left, right, count = pending.pop()
        if count == 0:
            continue
        if (count == 1 and right - left <= width
                and _evaluate(radical, left) and _evaluate(radical, right)):
            intervals.append((left, right))
            continue
        if subdivisions >= max_subdivisions:
            raise IsolationWorkLimit("exact root isolation subdivision budget exhausted")
        subdivisions += 1
        midpoint = (left + right) / 2
        if _evaluate(radical, midpoint) == 0:
            intervals.append((midpoint, midpoint))
        pending.append((midpoint, right, count_open(midpoint, right)))
        pending.append((left, midpoint, count_open(left, midpoint)))

    result = []
    for left, right in sorted(intervals):
        matches = [multiplicity for factor, multiplicity in factors
                   if (_evaluate(factor, left) == 0 if left == right else
                       _evaluate(factor, left) * _evaluate(factor, right) < 0)]
        assert len(matches) == 1
        result.append(RootInterval(left, right, matches[0]))
    return result
