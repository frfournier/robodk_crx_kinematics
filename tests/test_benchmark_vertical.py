# SPDX-License-Identifier: Apache-2.0
# SPDX-FileCopyrightText: 2026 Centre Technologique en Aérospatiale
"""Reject corrupted benchmark evidence against an independent four-pose fibre."""

import importlib
from pathlib import Path

import numpy as np
import pandas as pd
import pytest
from crx_vertical_reference import vertical_reference


@pytest.mark.parametrize("corruption", [None, "missing", "duplicate", "nonfinite", "wrong_count", "wrong_pose"])
def test_component_evidence_requires_complete_valid_postures(tmp_path, monkeypatch, corruption):
    monkeypatch.syspath_prepend(str(Path(__file__).resolve().parents[1] / "scripts"))
    benchmark = importlib.import_module("benchmark_vertical")
    lengths = [1., 1., 8/27, 5/18]
    target = np.eye(4)
    target[:3, 3] = [.1, 0., .025+lengths[2]]
    oracle = vertical_reference(lengths, target)
    assert not oracle["family"] and len(oracle["solutions"]) == 4
    case = {"name": "four-pose fibre", "group": "vertical", "model": 0,
            "canonical": target.tolist(), "witnesses": oracle["solutions"],
            "finite_vertical_oracle": True, "family_oracle": False}
    timings = pd.DataFrame([{"case": 0, "path": path, "round": 0, "us": 1.,
                             "status": 0, "count": 4, "vertical_closed_form": path == "closed_form"}
                            for path in ["general", "closed_form"]])
    solutions = pd.DataFrame([{"case": 0, "path": path, **dict(zip([f"q{i}" for i in range(1, 7)], q))}
                              for path in ["general", "closed_form"] for q in oracle["solutions"]])
    if corruption == "missing":
        solutions = solutions.iloc[:-1]
        timings.loc[timings.path == "closed_form", "count"] = 3
    elif corruption == "duplicate":
        solutions.iloc[-1, 2:] = solutions.iloc[-2, 2:].to_numpy()
    elif corruption == "nonfinite":
        solutions.loc[7, "q1"] = np.inf
    elif corruption == "wrong_count":
        timings.loc[timings.path == "closed_form", "count"] = 3
    elif corruption == "wrong_pose":
        solutions.loc[7, "q2"] += .1
    timings.to_csv(tmp_path / "timings.csv", index=False)
    solutions.to_csv(tmp_path / "solutions.csv", index=False)
    if corruption:
        with pytest.raises(AssertionError):
            benchmark.component_results(tmp_path, [{"lengths": lengths}], [case], 1)
    else:
        result = benchmark.component_results(tmp_path, [{"lengths": lengths}], [case], 1)
        assert result["vertical"]["count_changes"] == 0
        assert result["vertical"]["lost_general_postures"] == 0
