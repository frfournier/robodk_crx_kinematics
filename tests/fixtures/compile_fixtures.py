#!/usr/bin/env python3
"""
Convert CRX10iA-KIN.xlsx -> grouped JSON

- Group by TEST CASE
- Keep shared target pose once per test case
- List solutions by SOLUTION ID
- Remove redundant per-row pose fields
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any

import pandas as pd


def xlsx_to_grouped_json(xlsx_path: str | Path, json_path: str | Path) -> None:
    xlsx_path = Path(__file__).parent / xlsx_path
    json_path = Path(__file__).parent / Path(json_path)

    df = pd.read_excel(xlsx_path, sheet_name="Sheet1").dropna(how="all")

    # Basic normalization
    df["TEST CASE"] = df["TEST CASE"].astype(int)
    df["SOLUTION ID"] = df["SOLUTION ID"].astype(int)

    # Ensure config is stored as strings (some sheets may contain blanks/NaN)
    for c in ("FB", "UD", "TB"):
        if c in df.columns:
            df[c] = df[c].astype(str)

    test_cases: list[dict[str, Any]] = []

    for tc, g in df.groupby("TEST CASE", sort=True):
        g = g.sort_values("SOLUTION ID")
        first = g.iloc[0]

        target = {
            "xyz_mm": [float(first["X"]), float(first["Y"]), float(first["Z"])],
            "wpr_deg": [float(first["W"]), float(first["P"]), float(first["R"])],
        }

        solutions: list[dict[str, Any]] = []
        for _, row in g.iterrows():
            solutions.append(
                {
                    "id": int(row["SOLUTION ID"]),
                    "joints_deg": [float(row[f"J{i}"]) for i in range(1, 7)],
                    "config": {"FB": row["FB"], "UD": row["UD"], "TB": row["TB"]},
                }
            )

        test_cases.append(
            {
                "id": int(tc),
                "name": str(first["TEST NAME"]),
                "target": target,
                "solutions": solutions,
            }
        )

    out = {
        "meta": {
            "robot_model": "CRX10iA",
            "units": {"position": "mm", "orientation": "deg", "joints": "deg"},
        },
        "test_cases": test_cases,
    }

    json_path.parent.mkdir(parents=True, exist_ok=True)
    with json_path.open("w", encoding="utf-8") as f:
        json.dump(out, f, indent=2, ensure_ascii=False)


if __name__ == "__main__":
    # Example usage:
    xlsx_to_grouped_json(
        "CRX10iA-solutions.xlsx",
        "CRX10iA-solutions.json",
    )