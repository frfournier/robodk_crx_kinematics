# SPDX-License-Identifier: Apache-2.0
# SPDX-FileCopyrightText: 2026 Centre Technologique en Aérospatiale
"""Measure exact represented rank loss, without adding a runtime predicate."""

import argparse
from fractions import Fraction as F
import json
from pathlib import Path

import numpy as np
import pandas as pd

import benchmark_solvers as bench
from fixtures.generate_accuracy_stress import corpus as stress_corpus

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--experiments", type=Path, required=True, help="Parent of existing-cramer, stress-cramer, specialized-reachable-cramer experiments")
artifact = parser.parse_args().experiments.resolve()
results = {}
records = []
for corpus in ["existing", "stress", "specialized"]:
    if corpus == "existing":
        models, _, _ = bench.corpus(bench.DEFAULT_RANDOM_PER_MODEL, bench.DEFAULT_STRESS_PER_GROUP_MODEL, bench.DEFAULT_SEED)
    elif corpus == "stress":
        models, _, _ = stress_corpus()
    else:
        models = json.loads((artifact/"specialized-reachable-cramer/models.json").read_text())
    folder = artifact/("specialized-reachable-cramer" if corpus == "specialized" else f"{corpus}-cramer")
    cases = json.loads((folder/"corpus.json").read_text())
    rank_lost, nonvertical, disagreements = [], [], []
    for index, case in enumerate(cases):
        model = models[case["model"]]
        assert np.array_equal(model["base"], np.eye(4)) and np.array_equal(model["tool"], np.eye(4))
        target = np.array(case["target"])
        lengths = np.asarray(model["lengths"])
        scale = np.abs(lengths).max()
        p = target[:3, 3]/scale - (lengths[2]/scale)*target[:3, 2]
        v, w, n = [list(map(F, target[:3, column])) for column in range(3)]
        pf = list(map(F, p))
        dot = lambda x, y: sum(a*b for a, b in zip(x, y))
        determinant = dot(pf, v)*w[2]-dot(pf, w)*v[2]
        normal_determinant = pf[1]*n[0]-pf[0]*n[1]
        vertical = n[0] == n[1] == v[2] == w[2] == 0
        if (determinant == 0) != (normal_determinant == 0):
            disagreements.append(index)
        if determinant == 0:
            rank_lost.append(index)
            if not vertical:
                nonvertical.append(index)
            records.append({"corpus": corpus, "case": index, "name": case["name"],
                            "group": case["group"], "vertical": vertical,
                            "normal_identity_also_zero": normal_determinant == 0})
    frame = pd.read_csv(folder/"per-pose.csv")
    bounds = {}
    for mode in bench.MODES:
        subset = frame[(frame.path == "candidate") & (frame["mode"] == mode)]
        eligible = subset[subset["case"].isin(nonvertical)]
        bounds[mode] = {"eligible_total_us": float(eligible.median_us.sum()),
                        "all_total_us": float(subset.median_us.sum()),
                        "maximum_fraction_even_if_eligible_calls_free": float(eligible.median_us.sum()/subset.median_us.sum())}
    results[corpus] = {"poses": len(cases), "exact_rank_loss": len(rank_lost),
                       "exact_nonvertical_rank_loss": len(nonvertical),
                       "normal_identity_zero_disagreements": disagreements,
                       "potential_total_cost_upper_bounds": bounds}
pd.DataFrame(records).to_csv(artifact/"coplanar-frequency-cases.csv", index=False)
result = {"method": "Exact Fraction dot/product determinant on represented prepared p,v,w; no tolerance and no runtime implementation.",
          "scope": "All recorded model base/tool frames verified identity; vertical cases excluded from optional-quartic value bound.",
          "identity_caution": "Rounded near-orthogonal columns need not satisfy v cross w == normal exactly; both tests recorded separately.",
          "corpora": results}
(artifact/"coplanar-frequency.json").write_text(json.dumps(result, indent=2)+"\n")
print(json.dumps(result, indent=2))
