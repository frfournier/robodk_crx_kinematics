"""Deterministic fixture/asset discovery corpus shared by tests and benchmarks."""
import json
from pathlib import Path
import numpy as np
from crx_asset_reference import canonical_forward, derive_model, rigid_inverse
from crx_reference import translation, xyzwpr

ROOT = Path(__file__).resolve().parents[2]


def make_corpus(random_count, stress_count, seed):
    capture = json.loads((ROOT / "tests/fixtures/crx_asset_frames.json").read_text())
    models, cases = [], []
    rng = np.random.default_rng(seed)
    for asset in capture["assets"]:
        model = derive_model(asset)
        dh = model["dh"]
        robot = np.zeros((32, 20))
        robot[1, 1] = 6
        for key, row in (("base", 9), ("tool", 28)):
            np.testing.assert_allclose(model[key][:3, :3], np.eye(3), atol=1e-12, rtol=0)
            robot[row, :3] = model[key][:3, 3]
        robot[3, 4:10] = np.rint(np.diag(model["motion_map"]))
        robot[10:16, :4] = dh
        robot[30, :6], robot[31, :6] = asset["lower_limits_deg"], asset["upper_limits_deg"]
        models.append({
            "name": asset["asset"], "robot": robot,
            "lengths": [dh[2, 1], dh[3, 3], dh[5, 3], -dh[4, 3]],
            "base": model["base"] @ translation(0, 0, dh[0, 3]),
            "tool": model["tool"], "asset_sha256": asset["sha256"],
        })

    def add(model_id, name, group, target, witnesses):
        model = models[model_id]
        canonical = rigid_inverse(model["base"]) @ target @ rigid_inverse(model["tool"])
        cases.append({"model": model_id, "name": name, "group": group,
                      "target": target.tolist(), "canonical": canonical.tolist(),
                      "witnesses": [list(q) for q in witnesses]})

    ten = next(i for i, m in enumerate(models) if m["name"] == "Fanuc-CRX-10iA-Custom.robot")
    fixtures = json.loads((ROOT / "tests/fixtures/CRX10iA-solutions.json").read_text())
    for case in fixtures["test_cases"]:
        target = xyzwpr([*case["target"]["xyz_mm"], *np.radians(case["target"]["wpr_deg"])])
        witnesses = []
        for solution in case["solutions"]:
            q = np.radians(solution["joints_deg"])
            q[2] += q[1]
            witnesses.append(q)
        add(ten, case["name"], "fixtures", target, witnesses)

    for model_id, model in enumerate(models):
        lower, upper = model["robot"][30, :6], model["robot"][31, :6]
        for group, count in (("random", random_count), ("stress", stress_count)):
            for index in range(count):
                # Sample the intersection of command J3 and the existing core's
                # decoupled J3 limits; do not benchmark knowingly illegal seeds.
                while True:
                    command = rng.uniform(lower + 2, upper - 2)
                    lo = max(lower[2], lower[2] + command[1]) + 2
                    hi = min(upper[2], upper[2] + command[1]) - 2
                    if lo >= hi:
                        continue
                    command[2] = rng.uniform(lo, hi)
                    if group == "stress":
                        if index % 2 == 0:
                            command[4] = (0, -1e-6, 1e-6)[(index // 2) % 3]
                        else:
                            command[2] = 90 + (0, -1e-6, 1e-6)[(index // 2) % 3]
                    decoupled = command.copy()
                    decoupled[2] -= command[1]
                    if np.all(command >= lower) and np.all(command <= upper) and np.all(decoupled >= lower) and np.all(decoupled <= upper):
                        break
                q = np.radians(command)
                target = model["base"] @ canonical_forward(model["lengths"], q) @ model["tool"]
                add(model_id, f"{model['name']}:{group}:{index}", group, target, [q])

    for index in range(64):
        q = np.radians(np.random.default_rng(index).uniform(-150, 150, size=6))
        q[2] += q[1]
        model = models[ten]
        target = model["base"] @ canonical_forward(model["lengths"], q) @ model["tool"]
        add(ten, f"legacy:{index}", "legacy-witnesses", target, [q])
    return models, cases
