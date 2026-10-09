"""Independent transform-product reference for the legacy callback model.

This is derived test geometry, not an asset extraction or a family-support claim.
Commands at the C ABI are degrees with coupled J3; model limits and the existing
test helpers use decoupled J3. DH angles are radians, lengths are millimetres.
No native FK/IK call is used to construct reference targets.
"""

import math

import numpy as np


def rotation(axis, angle):
    c, s = math.cos(angle), math.sin(angle)
    result = np.eye(4)
    i, j = ((1, 2), (2, 0), (0, 1))[axis]
    result[i, i] = result[j, j] = c
    result[i, j], result[j, i] = -s, s
    return result


def translation(x, y, z):
    result = np.eye(4)
    result[:3, 3] = [x, y, z]
    return result


def xyzwpr(values):
    x, y, z, w, p, r = values
    return translation(x, y, z) @ rotation(2, r) @ rotation(1, p) @ rotation(0, w)


def reference_fk(robot, joints_decoupled_deg):
    """Return the flange/tool pose and all seven base/joint frames."""
    sensed = np.radians(joints_decoupled_deg) * np.array(robot.data[3][4:10])
    motion = sensed.copy()
    motion[2] -= sensed[1]
    frames = [xyzwpr(robot.data[9][:6])]
    for index in range(6):
        alpha, a, theta, d, prismatic = robot.data[10 + index][:5]
        if prismatic:
            d += motion[index]
        else:
            theta += motion[index]
        # Modified DH via elementary transforms, independent of the native
        # implementation's expanded matrix and right-angle snapping.
        local = (
            rotation(0, alpha)
            @ translation(a, 0, 0)
            @ rotation(2, theta)
            @ translation(0, 0, d)
        )
        frames.append(frames[-1] @ local)
    return frames[-1] @ xyzwpr(robot.data[28][:6]), frames


def missing_postures(expected, actual, tolerance_deg=0.009):
    """Compare geometric postures modulo turns, never configuration flags."""
    missing = []
    for posture in expected:
        if not any(
            np.max(np.abs((np.asarray(candidate) - posture + 180) % 360 - 180))
            <= tolerance_deg
            for candidate in actual
        ):
            missing.append(posture)
    return missing
