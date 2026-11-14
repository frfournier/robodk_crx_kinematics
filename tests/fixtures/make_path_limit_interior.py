# SPDX-License-Identifier: Apache-2.0
# SPDX-FileCopyrightText: 2026 Centre Technologique en Aérospatiale
"""Regenerate the synthetic interior-limit regression with 120-digit IK.

Run from the repository root with uv run python tests/fixtures/make_path_limit_interior.py.
The nominal target translation follows a tangent to one joint's level surface.
Its second-order curvature puts the middle outside a limit that admits both
ends. This is an intentional synthetic fixture; no RoboDK asset is modified.
"""

import copy
import json
from pathlib import Path
import sys

import mpmath as mp

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import crx_exact_reference as ref
from test_crx_ball_certificate import exact_kinematics, generated


def generate():
    with mp.workdps(120):
        fixture = generated(20)
        source = fixture["input"]
        rotation = mp.matrix([[ref.number(x) for x in row]
                              for row in ref.quaternion_rotation(source["target"]["quaternion"])])
        lengths = list(map(mp.mpf, source["lengths"]))

        def solve(target, guess):
            position = mp.matrix(list(map(mp.mpf, target)))
            def residual(*angles):
                r, p = ref.forward(lengths, angles)
                delta = rotation.T*mp.matrix([r[i:i+3] for i in range(0, 9, 3)])
                return [*(mp.matrix(p)-position), (delta[2, 1]-delta[1, 2])/2,
                        (delta[0, 2]-delta[2, 0])/2, (delta[1, 0]-delta[0, 1])/2]
            root = mp.findroot(residual, list(guess), tol=mp.mpf("1e-110"), maxsteps=30)
            assert max(map(abs, residual(*root))) < mp.mpf("1e-100")
            return list(root)

        root = solve(source["target"]["position"], fixture["centre"])
        _, r, _, jacobian, *_ = exact_kinematics(source, list(map(float, root)))
        gradient = (jacobian**-1)[:, :3]*r.T
        axis = 1
        direction = [gradient[axis, 1], -gradient[axis, 0], mp.mpf(0)]
        norm = mp.norm(mp.matrix(direction))
        direction = [x/norm for x in direction]
        step = mp.mpf("0.0001")
        first, last = copy.deepcopy(source), copy.deepcopy(source["target"])
        first["target"]["position"] = [float(mp.mpf(p)-step*d)
            for p, d in zip(source["target"]["position"], direction, strict=True)]
        last["position"] = [float(mp.mpf(p)+step*d)
            for p, d in zip(source["target"]["position"], direction, strict=True)]
        points, roots = [], []
        for s in (mp.mpf(0), mp.mpf(".5"), mp.mpf(1)):
            target = copy.deepcopy(first["target"])
            target["position"] = [float((1-s)*mp.mpf(a)+s*mp.mpf(b))
                for a, b in zip(first["target"]["position"], last["position"], strict=True)]
            points.append(target)
            roots.append(solve(target["position"], root))
        commands = [(q[axis]-mp.pi/2)*180/mp.pi for q in roots]
        lower, upper = [-720.]*6, [720.]*6
        if commands[1] > max(commands[0], commands[2]):
            upper[axis] = float((commands[1]+max(commands[0], commands[2]))/2)
        else:
            assert commands[1] < min(commands[0], commands[2])
            lower[axis] = float((commands[1]+min(commands[0], commands[2]))/2)
        return dict(provenance="Synthetic seed-20 tangent-translation regression; no robot measurements.",
                    axis=axis, input=first, last=last, point_targets=points,
                    point_roots=[[float(x) for x in q] for q in roots], lower=lower, upper=upper)


if __name__ == "__main__":
    destination = Path(__file__).with_name("path_limit_interior.json")
    destination.write_text(json.dumps(generate(), indent=2)+"\n", encoding="utf-8", newline="\n")
    print(destination)
