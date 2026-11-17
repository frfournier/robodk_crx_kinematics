# SPDX-License-Identifier: Apache-2.0
# SPDX-FileCopyrightText: 2026 Centre Technologique en Aérospatiale
"""Independent rational quaternion/frame and geometric polynomial oracles."""

from fractions import Fraction as F

import mpmath as mp


def number(value):
    if isinstance(value, F):
        return mp.mpf(value.numerator)/value.denominator
    return mp.mpf(value)


def transpose(a):
    return list(map(list, zip(*a, strict=True)))


def dot(a, b):
    return sum(x*y for x, y in zip(a, b, strict=True))


def cross(a, b):
    return [a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0]]


def matmul(a, b):
    return [[dot(row, col) for col in transpose(b)] for row in a]


def matvec(a, x):
    return [dot(row, x) for row in a]


def quaternion_rotation(values):
    w, x, y, z = map(F, values)
    n = w*w+x*x+y*y+z*z
    return [[(w*w+x*x-y*y-z*z)/n, 2*(x*y-w*z)/n, 2*(x*z+w*y)/n],
            [2*(x*y+w*z)/n, (w*w-x*x+y*y-z*z)/n, 2*(y*z-w*x)/n],
            [2*(x*z-w*y)/n, 2*(y*z+w*x)/n, (w*w-x*x-y*y+z*z)/n]]


def prepare(input):
    scale = F(max(abs(x) for x in input["lengths"]))
    lengths = [F(x)/scale for x in input["lengths"]]
    rt, rb, ru = [quaternion_rotation(input[key]["quaternion"]) for key in ["target", "base", "tool"]]
    rotation = matmul(matmul(transpose(rb), rt), transpose(ru))
    difference = [(F(t)-F(b))/scale for t, b in zip(input["target"]["position"], input["base"]["position"], strict=True)]
    tool_position = [F(x)/scale for x in input["tool"]["position"]]
    translation = [x-y for x, y in zip(matvec(transpose(rb), difference), matvec(rotation, tool_position), strict=True)]
    translation[2] -= F(input["base_height"])/scale
    center = [x-lengths[2]*n for x, n in zip(translation, transpose(rotation)[2], strict=True)]
    deviations_squared = []
    for key, exact in zip(["target", "base", "tool"], [rt, rb, ru], strict=True):
        deviations_squared.append(sum((F(x)-r)**2 for x, r in zip(input[key]["input_rotation"], sum(exact, []), strict=True)))
    return {"scale": scale, "lengths": lengths, "rotation": rotation, "translation": translation,
            "center": center, "v": transpose(rotation)[0], "w": transpose(rotation)[1],
            "deviations_squared": deviations_squared}


def poly_add(a, b):
    return [(a[i] if i < len(a) else 0)+(b[i] if i < len(b) else 0) for i in range(max(len(a), len(b)))]


def poly_scale(a, factor):
    return [factor*x for x in a]


def poly_mul(a, b):
    result = [0]*(len(a)+len(b)-1)
    for i, x in enumerate(a):
        for j, y in enumerate(b):
            result[i+j] += x*y
    return result


def coefficients(problem, chart):
    # At alpha=0 everything is rational, including rotations and compositions.
    # For other charts only sin(alpha),cos(alpha) introduce transcendental data.
    if chart == 0:
        convert, sine, cosine = F, F(0), F(1)
    else:
        convert = number
        sine, cosine = mp.sin(number(chart)), mp.cos(number(chart))
    a, b, _, r = map(convert, problem["lengths"])
    p = list(map(convert, problem["center"]))
    v = [cosine*convert(x)+sine*convert(y) for x, y in zip(problem["v"], problem["w"], strict=True)]
    w = [-sine*convert(x)+cosine*convert(y) for x, y in zip(problem["v"], problem["w"], strict=True)]
    q = [convert(1), convert(0), convert(1)]
    u = [[v[i], 2*w[i], -v[i]] for i in range(3)]
    x = [poly_add(poly_scale(q, p[i]), poly_scale(u[i], r)) for i in range(3)]
    m = [dot(p, v), 2*dot(p, w), -dot(p, v)]
    g = poly_add(m, poly_scale(q, r))
    p_squared = dot(p, p)
    s = poly_add(poly_scale(q, p_squared+r*r), poly_scale(m, 2*r))
    difference = a*a-b*b
    j = poly_scale(poly_add(s, poly_scale(q, -difference)), convert(1)/2)
    # Derive the Cramer numerator from the original triple products:
    # N = x C - zhat*g*j, C=h*g+(r^2-(P0+d)/2)U.
    # This is a vector-polynomial construction, independent of production's
    # two scalar quartic factors and its symmetric square/convolution helpers.
    c = poly_add(poly_scale(g, p[2]), poly_scale(u[2], r*r-(p_squared+r*r+difference)/2))
    numerator = [poly_mul(component, c) for component in x]
    numerator[2] = poly_add(numerator[2], poly_scale(poly_mul(g, j), -1))
    determinant = poly_add(poly_scale(poly_mul(g, q), p[2]),
                           poly_scale(poly_mul(poly_add(poly_scale(q, p_squared), poly_scale(m, r)), u[2]), -1))
    residual = [0]*9
    for component in numerator:
        residual = poly_add(residual, poly_mul(component, component))
    residual = poly_add(residual, poly_scale(poly_mul(determinant, determinant), -a*a))
    return residual, [(i+1)*residual[i+1] for i in range(8)], determinant


def geometric(problem, phase=None, axis=None):
    a, b, _, r = map(number, problem["lengths"])
    p = list(map(number, problem["center"]))
    if axis is None:
        axis = [number(v)*mp.cos(phase)+number(w)*mp.sin(phase)
                for v, w in zip(problem["v"], problem["w"], strict=True)]
    x = [p[i]+r*axis[i] for i in range(3)]
    n = cross([0, 0, 1], x)
    k = (a*a+dot(x, x)-b*b)/2
    axis_cross_n = cross(axis, n)
    determinant = dot(x, axis_cross_n)
    numerator = [k*v+dot(x, axis)*w for v, w in zip(axis_cross_n, cross(n, x), strict=True)]
    return dot(numerator, numerator)-a*a*determinant*determinant, determinant


def forward(lengths, q):
    a, b, c, r = map(number, lengths)

    def rotation(axis, angle):
        matrix = mp.eye(3)
        i, j = [(1, 2), (2, 0), (0, 1)][axis]
        matrix[i, i] = matrix[j, j] = mp.cos(angle)
        matrix[i, j], matrix[j, i] = -mp.sin(angle), mp.sin(angle)
        return matrix

    pose, position = mp.eye(3), mp.matrix([0, 0, 0])
    for axis, angle, shift in [(2, q[0], [0, 0, 0]), (1, q[1], [0, 0, a]),
                                (1, -q[2], [0, 0, 0]), (0, -q[3], [b, -r, 0]),
                                (1, -q[4], [c, 0, 0]), (0, -q[5], [0, 0, 0])]:
        pose = pose*rotation(axis, angle)
        position += pose*mp.matrix(shift)
    pose = pose*mp.matrix([[0, 0, 1], [0, -1, 0], [1, 0, 0]])
    return [pose[i, j] for i in range(3) for j in range(3)], list(position)


def command_angles(input, command):
    senses = [1, 1, -1, -1, -1, -1]
    reference = [0, -mp.pi/2, 0, 0, 0, 0]
    return [senses[i]*(number(F(input["encoder_zero"][i]))-reference[i]+
                       input["senses"][i]*number(command[i])) for i in range(6)]
