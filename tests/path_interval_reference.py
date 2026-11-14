# SPDX-License-Identifier: Apache-2.0
# SPDX-FileCopyrightText: 2026 Centre Technologique en Aérospatiale
"""Independent rational-interval audit of uniform path proof records.

Arithmetic rounds OUTWARD to a dyadic grid using integer division. Trigonometry
uses rational Taylor polynomials and Lagrange remainders. FK/Jacobians use
differentiated homogeneous products, not the native moved-axis cross products.
This oracle is intentionally separate from the timed implementation.
"""

from fractions import Fraction as F
from functools import lru_cache
import math


GRID = 1 << 192


class I:
    __slots__ = ("lo", "hi")

    def __init__(self, lo=0, hi=None):
        if isinstance(lo, I):
            self.lo, self.hi = lo.lo, lo.hi
            return
        lo, hi = F(lo), F(lo if hi is None else hi)
        assert lo <= hi
        self.lo = F(lo.numerator*GRID//lo.denominator, GRID)
        self.hi = F(-((-hi.numerator*GRID)//hi.denominator), GRID)

    def __add__(self, other):
        other = I(other)
        return I(self.lo+other.lo, self.hi+other.hi)
    __radd__ = __add__

    def __neg__(self): return I(-self.hi, -self.lo)
    def __sub__(self, other): return self+-I(other)
    def __rsub__(self, other): return I(other)+-self

    def __mul__(self, other):
        other = I(other)
        values = [self.lo*other.lo, self.lo*other.hi, self.hi*other.lo, self.hi*other.hi]
        return I(min(values), max(values))
    __rmul__ = __mul__

    def __truediv__(self, other):
        other = I(other)
        assert not other.lo <= 0 <= other.hi
        return self*I(1/other.hi, 1/other.lo)

    def magnitude(self): return max(abs(self.lo), abs(self.hi))

    def square(self):
        values = [self.lo*self.lo, self.hi*self.hi]
        return I(0 if self.lo <= 0 <= self.hi else min(values), max(values))


def zeros(n, m): return [[I(0) for _ in range(m)] for _ in range(n)]


def identity(n):
    result = zeros(n, n)
    for i in range(n): result[i][i] = I(1)
    return result


def matmul(a, b):
    return [[sum((a[i][k]*b[k][j] for k in range(len(b))), I(0))
             for j in range(len(b[0]))] for i in range(len(a))]


def transpose(a): return list(map(list, zip(*a, strict=True)))


@lru_cache(maxsize=8192)
def sincos(value):
    x = F(value)
    if not x: return I(0), I(1)
    sine, cosine = F(0), F(0)
    count = 32
    for k in range(count):
        sine += (-1)**k*x**(2*k+1)/math.factorial(2*k+1)
        cosine += (-1)**k*x**(2*k)/math.factorial(2*k)
    sr = abs(x)**(2*count)/math.factorial(2*count)
    cr = abs(x)**(2*count-1)/math.factorial(2*count-1)
    return I(sine-sr, sine+sr), I(cosine-cr, cosine+cr)


@lru_cache(maxsize=1)
def pi_interval():
    def atan(inverse):
        x = F(1, inverse)
        n = 80
        centre = sum((-1)**k*x**(2*k+1)/(2*k+1) for k in range(n))
        remainder = x**(2*n+1)/(2*n+1)
        return I(centre-remainder, centre+remainder)
    return 16*atan(5)-4*atan(239)


def sqrt_upper(value):
    assert value >= 0
    precision = 10**60
    integer = math.isqrt(value.numerator*precision**2//value.denominator)
    return F(integer if integer**2*value.denominator == value.numerator*precision**2 else integer+1, precision)


def norm_upper(vector): return sqrt_upper(sum(value.square().hi for value in vector))


def cross(a, b):
    return [a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0]]


def rotation(axis, angle, sign):
    sine, cosine = sincos(sign*F(angle))
    r, d = identity(4), zeros(4, 4)
    i, j = [(1, 2), (2, 0), (0, 1)][axis]
    r[i][i] = r[j][j] = cosine
    r[i][j], r[j][i] = -sine, sine
    d[i][i] = d[j][j] = -sign*sine
    d[i][j], d[j][i] = -sign*cosine, sign*cosine
    return r, d


@lru_cache(maxsize=2048)
def kinematics(lengths, centre):
    a, b, c, radius = map(F, lengths)
    shifts = [[0, 0, 0], [0, 0, a], [0, 0, 0], [b, -radius, 0], [c, 0, 0], [0, 0, 0]]
    axes_spec = [(2, 1), (1, 1), (1, -1), (0, -1), (1, -1), (0, -1)]
    factors, derivatives, prefixes, axes, origins = [], [], [identity(4)], [], []
    for value, (axis, sign), shift in zip(centre, axes_spec, shifts, strict=True):
        before = prefixes[-1]
        axes.append([sign*before[i][axis] for i in range(3)])
        origins.append([before[i][3] for i in range(3)])
        r, d = rotation(axis, value, sign)
        t = identity(4)
        for i in range(3): t[i][3] = I(shift[i])
        factors.append(matmul(r, t))
        derivatives.append(matmul(d, t))
        prefixes.append(matmul(before, factors[-1]))
    final = identity(4)
    for i, row in enumerate([[0, 0, 1], [0, -1, 0], [1, 0, 0]]):
        for j, v in enumerate(row): final[i][j] = I(v)
    pose = matmul(prefixes[-1], final)
    rt = transpose([row[:3] for row in pose[:3]])
    jacobian = zeros(6, 6)
    for column in range(6):
        derivative = matmul(prefixes[column], derivatives[column])
        for factor in factors[column+1:]: derivative = matmul(derivative, factor)
        derivative = matmul(derivative, final)
        angular = matmul(rt, [row[:3] for row in derivative[:3]])
        linear = matmul(rt, [[row[3]] for row in derivative[:3]])
        for i in range(3): jacobian[i][column] = linear[i][0]
        jacobian[3][column], jacobian[4][column], jacobian[5][column] = angular[2][1], angular[0][2], angular[1][0]
    return pose, jacobian, axes, origins


def determinant_sign(matrix):
    a = [list(map(F, row)) for row in matrix]
    result = F(1)
    for i in range(len(a)):
        pivot = next((j for j in range(i, len(a)) if a[j][i]), None)
        assert pivot is not None
        if pivot != i: a[i], a[pivot], result = a[pivot], a[i], -result
        result *= a[i][i]
        for j in range(i+1, len(a)):
            factor = a[j][i]/a[i][i]
            for k in range(i+1, len(a)): a[j][k] -= factor*a[i][k]
    return 1 if result > 0 else -1


def audit_uniform(request, record):
    """Check all new path hypotheses on whole intervals with rational bounds."""
    assert record["local"]
    point, source = record["point"], request["input"]
    scale_value = max(abs(F(x)) for x in source["lengths"])
    lengths = tuple(F(x)/scale_value for x in source["lengths"])
    pose, jacobian, axes, origins = kinematics(lengths, tuple(point["centre"]))
    constants = list(map(F, point["constants"]))
    _, radius, error, beta, lipschitz, _, _, screw_sum, angular_sum, p0, sign = constants
    raw_inverse = [list(map(F, point["preconditioner"][i:i+6])) for i in range(0, 36, 6)]
    inverse = [[I(x) for x in row] for row in raw_inverse]
    residual = matmul(inverse, jacobian)
    actual_error = max(sum((I(int(i == j))-residual[i][j]).magnitude() for j in range(6)) for i in range(6))
    inverse_norm = max(sum(map(abs, row)) for row in raw_inverse)
    assert actual_error <= error < 1
    assert inverse_norm/(1-error) <= beta
    assert determinant_sign(raw_inverse) == sign
    assert sum(norm_upper(axis) for axis in axes) <= angular_sum
    assert sum(norm_upper(axis)+norm_upper(cross(origin, axis)) for axis, origin in zip(axes, origins, strict=True)) <= screw_sum
    assert norm_upper([pose[i][3] for i in range(3)]) <= p0
    pi = pi_interval()
    reach = radius*screw_sum+p0
    lam = pi.hi*(p0+reach)/2
    b = pi.hi/2+pi.hi**2*lam/8
    c = sqrt_upper(F(2))*pi.hi**2/8+(sqrt_upper(F(2))*pi.hi**3/16+3*sqrt_upper(F(2))*pi.hi**2/16)*lam+pi.hi**2/8
    assert b*(1+reach)**2*(c+1)*screw_sum**2 <= lipschitz
    rt = transpose([row[:3] for row in pose[:3]])
    target = [[I(*record["rotation"][3*i+j]) for j in range(3)] for i in range(3)]
    relative = matmul(rt, target)
    translation = matmul(rt, [[I(*record["translation"][i])-pose[i][3]] for i in range(3)])
    trace = sum((relative[i][i] for i in range(3)), I(0))
    assert trace.lo >= 1
    h = zeros(4, 4)
    for i in range(3):
        for j in range(3): h[i][j] = relative[i][j]-int(i == j)
        h[i][3] = translation[i][0]
    defect = max(sum(x.magnitude() for x in row) for row in h)
    assert defect <= F(record["defect"])
    if record["used_log_series"]:
        assert defect < F(1, 2)
        power, logarithm = h, zeros(4, 4)
        for n in range(1, 7):
            for i in range(4):
                for j in range(4): logarithm[i][j] += ((1 if n % 2 else -1)*power[i][j])/n
            if n != 6: power = matmul(power, h)
        twist = [[logarithm[0][3]], [logarithm[1][3]], [logarithm[2][3]],
                 [logarithm[2][1]], [logarithm[0][2]], [logarithm[1][0]]]
        correction = matmul(inverse, twist)
        tail = defect**7/(7*(1-defect))
        bound = (max(row[0].magnitude() for row in correction)+inverse_norm*tail)/(1-error)
    else:
        bound = 3*pi.hi*beta*defect
    eta, rho = F(record["eta"]), F(record["rho"])
    assert bound <= eta
    assert 0 <= rho <= radius and radius*angular_sum < pi.lo
    assert beta*lipschitz*radius < 1 and beta*lipschitz*rho*rho/2+eta <= rho
    reference_senses = [1, 1, -1, -1, -1, -1]
    commands = []
    for i, (value, sense, reference) in enumerate(zip(point["centre"], source["senses"], reference_senses, strict=True)):
        offset = (I(source["encoder_zero"][i])+(pi/2 if i == 1 else I(0)))*reference
        commands.append((I(value)-offset)*(sense*reference))
    if request.get("profile", 0): commands[2] -= commands[1]
    for i, command in enumerate(commands):
        lower = command-I(request.get("lower", [-720.]*6)[i])*pi/180
        upper = I(request.get("upper", [720.]*6)[i])*pi/180-command
        row_norm = 2 if request.get("profile", 0) and i == 2 else 1
        assert F(record["slacks"][i]) <= min(lower.lo, upper.lo)
        assert record["slack_lipschitz"][i] == row_norm
        if record["admitted"]: assert row_norm*rho <= min(lower.lo, upper.lo)
