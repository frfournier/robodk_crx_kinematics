"""Capture nominal CRX joint frames in a separate hidden RoboDK instance.

Run with uv run python tests/capture_crx_assets.py --robodk-path PATH.
No deployment, station save, robot motion or asset modification is performed.
Only public JointPoses/SolveFK observations are used; no binary asset parser.
"""

import argparse
import hashlib
import json
from pathlib import Path
import re

import numpy as np
from robodk.robolink import Robolink

from crx_assets import APPROVED_ROBOT_ASSETS


def capture(robot, commands):
    return {
        "commands_deg": list(commands),
        "joint_frames": [pose.rows for pose in robot.JointPoses(commands)],
        "flange_pose": robot.SolveFK(commands).rows,
    }


def encode_capture(result):
    # Keep numeric vectors/matrix rows on one line for reviewable fixture diffs.
    text = json.dumps(result, indent=2, allow_nan=False)
    return re.sub(r"\[\s*([-+0-9.eE,\s]+)\]",
                  lambda match: "[" + " ".join(match[1].split()) + "]", text) + "\n"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--robodk-path", required=True)
    parser.add_argument(
        "--output", type=Path,
        default=Path(__file__).parent / "fixtures" / "crx_asset_frames.json",
    )
    args = parser.parse_args()
    assets = Path(__file__).resolve().parents[1] / "assets"
    for name, digest in APPROVED_ROBOT_ASSETS.items():
        if hashlib.sha256((assets / name).read_bytes()).hexdigest() != digest:
            raise ValueError(f"Asset hash mismatch: {name}")

    rdk = Robolink(
        robodk_path=args.robodk_path,
        args=["/NOSPLASH", "/NOSHOW", "/NEWINSTANCE"], quit_on_close=True,
    )
    result = {
        "schema": 1, "robodk_version": rdk.Version(),
        "mode": "nominal", "length_unit": "mm", "command_unit": "degree",
        "source": "JointPoses and SolveFK; separate unsaved station",
        "custom_extension_bypassed": False,
        "assets": [],
    }
    extension = Path(args.robodk_path).parent / "robotextensions" / "crx_kinematics.dll"
    result["installed_crx_extension_sha256"] = (
        hashlib.sha256(extension.read_bytes()).hexdigest() if extension.exists() else None
    )
    try:
        for name, digest in APPROVED_ROBOT_ASSETS.items():
            robot = rdk.AddFile(str(assets / name))
            if not robot.Valid():
                raise ValueError(f"Cannot load {name}")
            try:
                robot.setAccuracyActive(0)
                lower, upper, joint_type = robot.JointLimits()
                probes = []
                for index in range(6):
                    command = [0.0] * 6
                    command[index] = 10.0
                    probes.append(capture(robot, command))
                # Independent multi-joint evidence, away from command limits.
                rng = np.random.default_rng(20261009)
                samples = [capture(robot, row.tolist()) for row in
                           rng.uniform(-80, 80, size=(16, 6))]
                result["assets"].append({
                    "asset": name, "sha256": digest,
                    "lower_limits_deg": lower.list(),
                    "upper_limits_deg": upper.list(), "joint_type": joint_type,
                    "zero": capture(robot, [0.0] * 6),
                    "probes": probes, "samples": samples,
                })
                print(f"Captured {name}")
            finally:
                robot.Delete()
    finally:
        rdk.Disconnect()
    args.output.write_text(encode_capture(result), encoding="utf-8")


if __name__ == "__main__":
    main()
