# SPDX-License-Identifier: Apache-2.0
# SPDX-FileCopyrightText: 2026 Centre Technologique en Aérospatiale
"""Offline exact wrist-root isolation and independent high-precision recovery.

This oracle uses raw quaternion/length fractions, rational Sturm isolation and
elementary transform products. No native nomination, inverse, FK or recovery
output is used. Unresolved arithmetic/strata are explicit, never agreement.
"""

from dataclasses import dataclass, field
from fractions import Fraction as F

import mpmath as mp

import crx_exact_reference as ref
import polynomial_reference as sturm
from test_crx_ball_certificate import prepare_reference


@dataclass
class ReferenceResult:
    outcome: str
    configurations: list = field(default_factory=list)
    wrist_roots: int | None = None
    precision: int = 0
    attempts: list = field(default_factory=list)
    reason: str = ""


class RefinementUnresolved(ArithmeticError):
    pass


def circle_distance(first, second):
    difference = first-second
    return abs(mp.atan2(mp.sin(difference), mp.cos(difference)))


def rotation(axis, angle):
    result = mp.eye(3)
    i, j = [(1, 2), (2, 0), (0, 1)][axis]
    result[i, i] = result[j, j] = mp.cos(angle)
    result[i, j], result[j, i] = -mp.sin(angle), mp.sin(angle)
    return result


def recover(problem, tangent=None):
    """Recover two full configurations from the exact scalar incidence relation."""
    a, b, _, r = map(ref.number, problem["lengths"])
    p = mp.matrix(list(map(ref.number, problem["center"])))
    v = mp.matrix(list(map(ref.number, problem["v"])))
    w = mp.matrix(list(map(ref.number, problem["w"])))
    if tangent is None:
        u = -v
    else:
        # Rational half-angle circle point avoids a separate atan/sin roundtrip.
        u = ((1-tangent*tangent)*v+2*tangent*w)/(1+tangent*tangent)
    x = p+r*u
    cross = lambda first, second: mp.matrix(ref.cross(list(first), list(second)))
    n = cross(mp.matrix([0, 0, 1]), x)
    determinant = mp.fdot(x, cross(u, n))
    if determinant == 0:
        raise RefinementUnresolved("zero determinant in recovery")
    k = (a*a+mp.fdot(x, x)-b*b)/2
    elbow = (k*cross(u, n)+mp.fdot(x, u)*cross(n, x))/determinant
    q0 = mp.atan2(x[1], x[0])
    rz = rotation(2, q0)
    local_elbow = rz.T*elbow/a
    q1 = mp.atan2(local_elbow[0], local_elbow[2])
    forearm = rz.T*(x-elbow)/b
    q2 = q1-mp.atan2(-forearm[2], forearm[0])
    arm = rz*rotation(1, q1-q2)
    local_u = arm.T*u
    q3 = mp.atan2(-local_u[2], local_u[1])
    wrist = arm*rotation(0, -q3)
    target = mp.matrix([[ref.number(value) for value in row] for row in problem["rotation"]])
    fixed = mp.matrix([[0, 0, 1], [0, -1, 0], [1, 0, 0]])
    relative = wrist.T*target*fixed
    q4 = mp.atan2(relative[2, 0], relative[0, 0])
    q5 = mp.atan2(relative[1, 2], relative[1, 1])
    first = [q0, q1, q2, q3, q4, q5]
    second = [q0-mp.pi, -q1, mp.pi-q2, q3-mp.pi, q4, q5]
    return first, second


def exhaustive(input, reference=False, precisions=(160, 240, 400, 640), max_subdivisions=8192):
    problem = prepare_reference(input) if reference else ref.prepare(input)
    power, _, determinant = ref.coefficients(problem, 0)
    nonzero = [i for i, value in enumerate(power) if value]
    if not nonzero:
        return ReferenceResult("degenerate", reason="exact zero residual polynomial; full stratum unresolved")
    degree = nonzero[-1]
    infinity = degree < 8
    try:
        roots = sturm.isolate_real_roots(power, max_subdivisions=max_subdivisions)
        common = sturm._gcd(sturm._trim(power), sturm._trim(determinant))
        common_real = sturm.isolate_real_roots(common, max_subdivisions=max_subdivisions)
    except sturm.IsolationWorkLimit as error:
        return ReferenceResult("unresolved", reason=f"exact isolation budget: {error}")
    if any(root.multiplicity != 1 for root in roots) or (infinity and degree != 7):
        return ReferenceResult("exceptional", wrist_roots=len(roots)+infinity, reason="exact multiple wrist root")
    if common_real or (infinity and determinant[-1] == 0):
        return ReferenceResult("exceptional", wrist_roots=len(roots)+infinity, reason="exact common residual/determinant root")
    attempts = []
    for precision in precisions:
        with mp.workdps(precision):
            tolerance = mp.power(10, -3*precision//4)
            fk_tolerance = mp.power(10, -precision//2)
            coefficients = list(map(ref.number, power))
            evaluate = lambda t: mp.polyval(list(reversed(coefficients)), t)
            points = []
            try:
                for root in roots:
                    left, right = ref.number(root.left), ref.number(root.right)
                    if left == right:
                        point = left
                    else:
                        point = mp.findroot(evaluate, (left+right)/2, solver="newton",
                                            tol=tolerance, maxsteps=100)
                        if not left <= point <= right:
                            raise RefinementUnresolved("refined root left its exact isolation interval")
                    if abs(evaluate(point)) > fk_tolerance:
                        raise RefinementUnresolved("scalar refinement residual")
                    points.append(point)
                if infinity:
                    points.append(None)
                configurations = [q for point in points for q in recover(problem, point)]
                target_rotation = sum(problem["rotation"], [])
                target_translation = problem["translation"]
                for q in configurations:
                    actual_r, actual_p = ref.forward(problem["lengths"], q)
                    defect = max(abs(x-ref.number(y)) for x, y in zip(
                        [*actual_r, *actual_p], [*target_rotation, *target_translation], strict=True))
                    if defect > fk_tolerance:
                        raise RefinementUnresolved(f"reconstructed FK defect {mp.nstr(defect, 8)}")
                for i, q in enumerate(configurations):
                    for other in configurations[:i]:
                        if max(circle_distance(x, y) for x, y in zip(q, other, strict=True)) <= fk_tolerance:
                            raise RefinementUnresolved("unseparated reconstructed phases")
                attempts.append({"precision": precision, "outcome": "finite"})
                return ReferenceResult("finite", configurations, len(points), precision, attempts)
            except (ValueError, ZeroDivisionError, RefinementUnresolved) as error:
                attempts.append({"precision": precision, "outcome": "unresolved", "reason": str(error)})
    return ReferenceResult("unresolved", wrist_roots=len(roots)+infinity, attempts=attempts,
                           reason="high-precision reconstruction did not resolve")
