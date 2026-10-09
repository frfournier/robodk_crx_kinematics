"""Derive a nominal modified-DH representation from captured link transforms.

This is an offline reference, not a runtime model recognizer. The derivation
must reproduce every captured link and flange, not merely the zero pose.
"""

import math

import numpy as np

from crx_reference import rotation, translation


def rigid_inverse(pose):
    result = np.eye(4)
    result[:3, :3] = pose[:3, :3].T
    result[:3, 3] = -result[:3, :3] @ pose[:3, 3]
    return result


def dh_transform(row, motion=0):
    alpha, a, theta, d = row
    return (rotation(0, alpha) @ translation(a, 0, 0)
            @ rotation(2, theta + motion) @ translation(0, 0, d))


def derive_model(asset):
    frames = np.asarray(asset["zero"]["joint_frames"])
    if frames.shape != (7, 4, 4):
        raise ValueError("Expected base and six revolute joint frames")
    rows = []
    for before, after in zip(frames[:-1], frames[1:]):
        local = rigid_inverse(before) @ after
        alpha = math.atan2(-local[1, 2], local[2, 2])
        theta = math.atan2(-local[0, 1], local[0, 0])
        rows.append([alpha, local[0, 3], theta,
                     -math.sin(alpha) * local[1, 3]
                     + math.cos(alpha) * local[2, 3]])
        if not np.allclose(dh_transform(rows[-1]), local, atol=1e-9, rtol=0):
            raise ValueError("Link transform is not modified DH")

    motion_map = np.zeros((6, 6))
    for index, probe in enumerate(asset["probes"]):
        step = math.radians(probe["commands_deg"][index])
        moved = np.asarray(probe["joint_frames"])
        for joint, row in enumerate(rows):
            local = rigid_inverse(moved[joint]) @ moved[joint + 1]
            theta = math.atan2(-local[0, 1], local[0, 0])
            motion_map[joint, index] = math.remainder(theta-row[2], 2*math.pi) / step
    return {
        "base": frames[0],
        "tool": rigid_inverse(frames[-1]) @ np.asarray(asset["zero"]["flange_pose"]),
        "dh": np.asarray(rows), "motion_map": motion_map,
    }


def forward_model(model, commands_deg):
    motion = model["motion_map"] @ np.radians(commands_deg)
    frames = [model["base"]]
    for row, angle in zip(model["dh"], motion):
        frames.append(frames[-1] @ dh_transform(row, angle))
    return np.asarray(frames), frames[-1] @ model["tool"]


def canonical_forward(lengths, q):
    a, b, c, r = lengths
    fixed = np.eye(4)
    fixed[:3, :3] = [[0, 0, 1], [0, -1, 0], [1, 0, 0]]
    return (rotation(2, q[0]) @ rotation(1, q[1]) @ translation(0, 0, a)
            @ rotation(1, -q[2]) @ rotation(0, -q[3]) @ translation(b, -r, 0)
            @ rotation(1, -q[4]) @ translation(c, 0, 0)
            @ rotation(0, -q[5]) @ fixed)
