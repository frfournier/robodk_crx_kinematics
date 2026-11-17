# SPDX-License-Identifier: Apache-2.0
# SPDX-FileCopyrightText: 2026 Centre Technologique en Aérospatiale
"""Independent vertical IK oracle from intersections of horizontal circles.

Circle/sphere discriminants use exact rational arithmetic on the supplied
binary64 geometry. Joint reconstruction and FK remain numerical; this is a
test oracle, not a configuration or coverage certificate.
"""

from fractions import Fraction
import math

import numpy as np
from crx_reference import rotation


def _circle_points(center, wrist_radius, squared_radius):
    # Intersect |x_h-center|²=r² with |x_h|²=R². Unlike production, no
    # cos/sin phase equation or target wrist-plane projections are used.
    squared_center = sum(value * value for value in center)
    if squared_radius < 0:
        return []
    alpha = (squared_center + squared_radius - wrist_radius**2) / (2 * squared_center)
    beta_squared = (squared_radius - alpha**2 * squared_center) / squared_center
    if beta_squared < 0:
        return []
    p = np.array([float(value) for value in center])
    along = float(alpha) * p
    offset = math.sqrt(float(beta_squared)) * np.array([-p[1], p[0]])
    return [along] if beta_squared == 0 else [along + offset, along - offset]


def _reconstruct(elbow, wrist, axis, target_rotation, a, b):
    horizontal = wrist[:2] if np.hypot(*wrist[:2]) > 0 else elbow[:2]
    if np.hypot(*horizontal) == 0:
        return None  # An undetermined base phase is not a finite catalogue.
    base = math.atan2(horizontal[1], horizontal[0])
    fixed = np.array([[0., 0., 1.], [0., -1., 0.], [1., 0., 0.]])
    result = []
    for angle in [base, base + np.pi]:
        undo = rotation(2, -angle)[:3, :3]
        upper, point = undo @ elbow, undo @ wrist
        shoulder = math.atan2(upper[0] / a, upper[2] / a)
        lower = (point - upper) / b
        phi = math.atan2(-lower[2], lower[0])
        local_axis = rotation(1, -phi)[:3, :3] @ undo @ axis
        fourth = math.atan2(-local_axis[2], local_axis[1])
        frame = (rotation(2, angle) @ rotation(1, phi)
                 @ rotation(0, -fourth))[:3, :3]
        normal = frame.T @ target_rotation[:, 2]
        fifth = math.atan2(normal[2], normal[0])
        last = (frame @ rotation(1, -fifth)[:3, :3]).T @ target_rotation @ fixed.T
        sixth = math.atan2(-last[2, 1], last[1, 1])
        result.append((np.array([angle, shoulder, shoulder - phi, fourth, fifth, sixth])
                       + np.pi) % (2 * np.pi) - np.pi)
    return result


def vertical_reference(lengths, target):
    lengths = np.asarray(lengths, dtype=float)
    target = np.asarray(target, dtype=float)
    assert target[0, 2] == target[1, 2] == target[2, 0] == target[2, 1] == 0
    assert abs(target[2, 2]) == 1
    scale = Fraction(float(np.max(np.abs(lengths))))
    a, b, c, r = [Fraction(float(value)) / scale for value in lengths]
    center = [Fraction(float(value)) / scale for value in target[:2, 3]]
    h = Fraction(float(target[2, 3])) / scale - c * int(target[2, 2])
    squared_center = sum(value * value for value in center)
    if r == 0:
        raise ValueError("The finite vertical oracle needs a nonzero wrist offset")
    if squared_center == 0:
        family = any(a*a == r*r + (h + sign*b)**2 for sign in [-1, 1])
        return {"family": family, "solutions": [], "axes": []}
    solutions, axes = [], []
    family = False
    for branch, radius_squared in [
        (0, squared_center - r*r),
        (1, a*a - (h+b)**2),
        (-1, a*a - (h-b)**2),
    ]:
        for horizontal in _circle_points(center, r, radius_squared):
            axis = np.r_[(horizontal - np.array([float(v) for v in center])) / float(r), 0.]
            wrist = np.r_[horizontal, float(h)]
            if branch:
                elbows = [wrist + np.array([0., 0., branch*float(b)])]
            else:
                squared_distance = radius_squared + h*h
                if squared_distance == 0:
                    family |= a*a == b*b
                    continue
                k = (a*a + squared_distance - b*b) / 2
                height_squared = a*a - k*k/squared_distance
                if height_squared < 0:
                    continue
                if radius_squared == 0:
                    z = k/h
                    radius_squared_y = a*a - z*z
                    if radius_squared_y < 0:
                        continue
                    center_y = np.array([0., 0., float(z)])
                    offset = math.sqrt(float(radius_squared_y)) * np.array([-axis[1], axis[0], 0.])
                else:
                    center_y = float(k/squared_distance) * wrist
                    horizontal_norm = math.sqrt(float(radius_squared))
                    direction = np.r_[-float(h)*horizontal/horizontal_norm, horizontal_norm]
                    direction /= math.sqrt(float(squared_distance))
                    offset = math.sqrt(float(height_squared)) * direction
                elbows = [center_y] if np.all(offset == 0) else [center_y + offset, center_y - offset]
            for elbow in elbows:
                recovered = _reconstruct(elbow, wrist, axis, target[:3, :3], float(a), float(b))
                if recovered is None:
                    family = True
                    continue
                for q in recovered:
                    if all(np.max(np.abs((q-other+np.pi) % (2*np.pi)-np.pi)) > 1e-8
                           for other in solutions):
                        solutions.append(q)
                if all(np.linalg.norm(axis-other) > 1e-8 for other in axes):
                    axes.append(axis)
    return {"family": family, "solutions": solutions, "axes": axes}
