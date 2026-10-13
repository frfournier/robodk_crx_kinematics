"""Generate versioned stress fixtures used by regressions and performance runs.

The v1 default corpus reproduces the complete 2026-10-09 snapshot, modulo newlines.
Do not reorder RNG draws or change the default parameters without a version bump.
"""
import argparse
import itertools
import json
from pathlib import Path
import sys
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tests"))
from fixtures.generate_discovery_cases import make_corpus
from crx_asset_reference import derive_model, forward_model

ROBOT_ROWS, ROBOT_STRIDE = 93, 20
CHALLENGES = ("wrist", "extended_elbow", "folded_elbow", "base_axis",
              "mixed_singular", "joint_limits")
# Experimental inputs, not solver tolerances: logarithmic offsets from exact
# singularity through ordinary geometry. Frozen from the 2026-10-09 snapshot.
OFFSETS_RAD = (0., 1e-12, 1e-10, 1e-8, 1e-6, 1e-4, .01)
GENERATOR_VERSION = 1
DEFAULT_SEED = 20261009
DEFAULT_RANDOM_PER_MODEL = 256
DEFAULT_STRESS_PER_GROUP_MODEL = 64


def legal_lift(q, model):
    """Choose a witness admitted by both command and old decoupled J3 limits."""
    low, high = np.radians(model["robot"][[30, 31], :6])
    choices = []
    for angle, lo, hi in zip(q, low, high):
        turns = range(int(np.ceil((lo - angle) / (2*np.pi))),
                      int(np.floor((hi - angle) / (2*np.pi))) + 1)
        values = [angle + 2*np.pi*k for k in turns]
        if not values:
            return None
        choices.append(sorted(values, key=lambda value: abs(value - angle)))
    result = np.array([values[0] for values in choices])
    for second, third in itertools.product(choices[1], choices[2]):
        if low[2] <= third - second <= high[2]:
            result[1:3] = second, third
            return result
    return None


def corpus(random_count, stress_count, seed):
    models, original = make_corpus(0, 0, seed)
    capture = json.loads((ROOT / "tests/fixtures/crx_asset_frames.json").read_text())
    references = [derive_model(asset) for asset in capture["assets"]]
    rng = np.random.default_rng(seed)
    cases = []

    def add(model_id, name, group, target, witnesses, offset=None, reachable=True):
        model = models[model_id]
        legal = [lift for q in witnesses if (lift := legal_lift(q, model)) is not None]
        # Fixtures remain observations even if no supplied witness has legal turns.
        center = legal[0] if legal else np.radians(model["robot"][[30, 31], :6].mean(axis=0))
        nearby = np.clip(np.degrees(center) + rng.uniform(-1, 1, 6),
                         model["robot"][30, :6], model["robot"][31, :6])
        cases.append(dict(model=model_id, name=name, group=group,
                          target=np.asarray(target).tolist(),
                          witnesses=[q.tolist() for q in legal],
                          seed_deg=nearby.tolist(), offset_rad=offset,
                          reachable=reachable))

    for item in original:
        add(item["model"], item["name"], item["group"], item["target"],
            item["witnesses"])
    for model_id, model in enumerate(models):
        low, high = np.radians(model["robot"][[30, 31], :6])
        for group in ("random", *CHALLENGES, "unreachable"):
            count = random_count if group == "random" else stress_count
            for index in range(count):
                offset = OFFSETS_RAD[index % len(OFFSETS_RAD)]
                delta = offset * (-1 if index % 2 else 1)
                for attempt in range(10000):
                    q = rng.uniform(low + np.radians(2), high - np.radians(2))
                    if group == "wrist":
                        q[4] = delta
                    elif group == "extended_elbow":
                        q[2] = np.pi/2 + delta
                    elif group == "folded_elbow":
                        q[2] = -np.pi/2 + delta
                    elif group == "base_axis":
                        a, b = model["lengths"][:2]
                        q[1] = np.arctan2(-b*np.cos(q[2]), a + b*np.sin(q[2])) + delta
                    elif group == "mixed_singular":
                        q[1], q[2], q[4] = delta, -np.pi/2 + delta, delta
                    elif group == "joint_limits":
                        axis = index % 6
                        q[axis] = low[axis] + offset if (index // 6) % 2 else high[axis] - offset
                    if (np.all(q >= low) and np.all(q <= high)
                            and low[2] <= q[2] - q[1] <= high[2]):
                        break
                else:
                    raise RuntimeError(f"Cannot generate {model['name']} {group}:{index}")
                _, target = forward_model(references[model_id], np.degrees(q))
                if group == "unreachable":
                    reach = sum(abs(value) for value in model["lengths"])
                    target[:3, 3] = model["base"][:3, 3] + [4*reach, 0, 0]
                add(model_id, f"{model['name']}:{group}:{index}", group, target,
                    [q], offset if group in CHALLENGES else None,
                    reachable=group != "unreachable")
    for model in models:
        padded = np.zeros((ROBOT_ROWS, ROBOT_STRIDE))
        padded[:len(model["robot"])] = model["robot"]
        model["robot"] = padded
    return models, references, cases


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--check", action="store_true")
    parser.add_argument("--output", type=Path, default=Path(__file__).with_name("crx_solver_cases.json"))
    args = parser.parse_args()
    _, _, cases = corpus(DEFAULT_RANDOM_PER_MODEL, DEFAULT_STRESS_PER_GROUP_MODEL, DEFAULT_SEED)
    rendered = json.dumps(cases, indent=2) + "\n"
    if args.check:
        if args.output.read_text(encoding="utf-8") != rendered:
            parser.error("fixture corpus differs; review generator/version before regenerating")
        print(f"Verified {len(cases)} v{GENERATOR_VERSION} cases")
    else:
        args.output.write_text(rendered, encoding="utf-8")
        print(f"Wrote {len(cases)} v{GENERATOR_VERSION} cases to {args.output}")


if __name__ == "__main__":
    main()
