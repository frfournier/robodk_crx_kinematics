"""Independent, frozen command-space stress corpus; no IK-based filtering.

Version 1 uses seed 2026101301, six captured models, and three length scales.
Every case has a command-limit-valid FK witness. Failures remain in the corpus.
"""
import copy
import json
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tests"))
from crx_asset_reference import derive_model, forward_model
from fixtures.generate_discovery_cases import make_corpus
from fixtures.generate_solver_cases import CHALLENGES


def corpus():
    originals, _ = make_corpus(0, 0, 0)
    assets = json.loads((ROOT / "tests/fixtures/crx_asset_frames.json").read_text())["assets"]
    rng = np.random.default_rng(2026101301)
    models, references, cases = [], [], []
    for scale in (.001, 1., 1000.):
        for original, asset in zip(originals, assets):
            model = copy.deepcopy(original)
            reference = derive_model(asset)
            reference["dh"][:, [1, 3]] *= scale
            for key in ("base", "tool"):
                reference[key][:3, 3] *= scale
                model[key][:3, 3] *= scale
            model["lengths"] = np.asarray(model["lengths"]) * scale
            model["robot"][10:16, 1] *= scale
            model["robot"][10:16, 3] *= scale
            model["robot"][[9, 28], :3] *= scale
            model["name"] += f"@{scale:g}"
            robot = np.zeros((93, 20))
            robot[:32] = model["robot"]
            model["robot"] = robot
            model_id = len(models)
            models.append(model)
            references.append(reference)
            low, high = np.radians(robot[[30, 31], :6])
            for group in (*CHALLENGES, "random"):
                for index in range(24):
                    # Continuous log offsets, unlike the existing discrete grid.
                    offset = 0. if index % 8 == 0 else 10 ** rng.uniform(-13, -2)
                    delta = offset * rng.choice([-1, 1])
                    q = rng.uniform(low, high)
                    if group == "wrist":
                        q[4] = delta
                    elif group == "extended_elbow":
                        q[2] = np.pi / 2 + delta
                    elif group == "folded_elbow":
                        q[2] = -np.pi / 2 + delta
                    elif group == "base_axis":
                        a, b = model["lengths"][:2]
                        q[1] = np.arctan2(-b*np.cos(q[2]), a+b*np.sin(q[2])) + delta
                    elif group == "mixed_singular":
                        q[1], q[2], q[3], q[4] = delta, -np.pi/2 + delta, delta, delta
                    elif group == "joint_limits":
                        axis = index % 6
                        q[axis] = low[axis] + offset if index % 2 else high[axis] - offset
                    q = np.clip(q, low, high)
                    _, target = forward_model(reference, np.degrees(q))
                    seed = np.clip(np.degrees(q) + rng.uniform(-1, 1, 6),
                                   np.degrees(low), np.degrees(high))
                    cases.append(dict(model=model_id, name=f"{model['name']}:{group}:{index}",
                                      group=group, target=target.tolist(), witnesses=[q.tolist()],
                                      seed_deg=seed.tolist(), offset_rad=offset, reachable=True))
    return models, references, cases


if __name__ == "__main__":
    output = Path(__file__).with_name("crx_accuracy_stress.json")
    rendered = json.dumps(corpus()[2], indent=2) + "\n"
    if "--check" in sys.argv:
        assert output.read_text() == rendered, "Frozen stress corpus changed"
    else:
        output.write_text(rendered)
    print(f"Frozen v1 corpus: {len(json.loads(rendered))} cases")
