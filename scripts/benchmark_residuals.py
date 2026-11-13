# SPDX-License-Identifier: Apache-2.0
# SPDX-FileCopyrightText: 2026 Centre Technologique en Aérospatiale
"""Record isolated native residual/preparation/Cramer component timings.

This does not time DLL calls, certificates or Python wrapper operations.
"""

import argparse
from datetime import UTC, datetime
import hashlib
import json
from pathlib import Path
import platform
import shutil
import subprocess
import sys
import zipfile

import numpy as np
import pandas as pd

ROOT = Path(__file__).resolve().parents[1]


def sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--rounds", type=int, default=9)
    parser.add_argument("--repeats", type=int, default=256)
    args = parser.parse_args()
    if min(args.rounds, args.repeats) < 1:
        parser.error("rounds and repeats must be positive")
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    runner = output / "runner.exe"
    shutil.copy2(ROOT / "build/Release/crx_residual_benchmark.exe", runner)
    with (output / "native.log").open("w") as log:
        subprocess.run([str(runner), str(output), str(args.repeats), str(args.rounds)],
                       check=True, stderr=log)
    inputs = pd.read_csv(output / "input.csv")
    checks = pd.read_csv(output / "checks.csv")
    timings = pd.read_csv(output / "timings.csv")
    assert len(inputs) == 1186 and inputs["case"].is_unique
    assert checks["case"].equals(inputs["case"])
    assert checks.legacy_cramer_ready.equals(checks.candidate_cramer_ready)
    assert np.isfinite(timings.ns).all() and timings.ns.ge(0).all()
    assert not timings.duplicated(["case", "path", "round"]).any()
    assert timings.groupby(["case", "path"]).size().eq(args.rounds).all()
    assert set(timings["case"]) == set(inputs["case"])
    assert timings.path.nunique() == 10
    medians = timings.groupby(["case", "path"]).ns.median().unstack()
    frame = inputs.set_index("case")[["model", "group"]].join(medians)
    frame.to_csv(output / "per-case.csv")
    groups = {}
    for name in ["all", *inputs.group.unique()]:
        subset = frame if name == "all" else frame[frame.group == name]
        rows = checks[checks["case"].isin(subset.index)]
        groups[name] = {
            "cases": len(subset),
            "geometric_fallbacks": int(rows.scalar_fallback.sum()),
            "cramer_ready": int(rows.candidate_cramer_ready.sum()),
            "timing_ns": {
                method: {"p50": float(subset[method].median()),
                         "p95": float(subset[method].quantile(.95)),
                         "p99": float(subset[method].quantile(.99)),
                         "max": float(subset[method].max())}
                for method in medians.columns
            },
        }
    sources = [*(ROOT / "src").glob("*.[ch]*"), *(ROOT / "include").glob("*.h"),
               ROOT / "scripts/benchmark_residuals.cpp",
               ROOT / "scripts/benchmark_residuals_baseline.h", Path(__file__),
               ROOT / "CMakeLists.txt", ROOT / "CMakePresets.json"]
    with zipfile.ZipFile(output / "sources.zip", "w", zipfile.ZIP_DEFLATED) as archive:
        for source in sorted(sources):
            archive.write(source, source.relative_to(ROOT))
    commands = ROOT / "build/vscode/windows-clang-release-tidy/compile_commands.json"
    shutil.copy2(commands, output / "compile_commands.json")
    cpu = platform.processor()
    if sys.platform == "win32":
        import winreg
        with winreg.OpenKey(winreg.HKEY_LOCAL_MACHINE,
                            r"HARDWARE\DESCRIPTION\System\CentralProcessor\0") as key:
            cpu = winreg.QueryValueEx(key, "ProcessorNameString")[0].strip()
    metadata = {
        "scope": "Native compiled component batch means; no DLL or Python timing",
        "measured_at_utc": datetime.now(UTC).isoformat(),
        "cpu": cpu, "platform": platform.platform(),
        "python": platform.python_version(), "numpy": np.__version__, "pandas": pd.__version__,
        "rounds": args.rounds, "repeats": args.repeats, "warmup_calls": 3,
        "affinity": "first allowed CPU, enforced by native runner; see native.log",
        "order": "shuffled cases and methods each round, seed 20261113",
        "overhead": "loop and result consumption included, loop path recorded without subtraction",
        "preparation": "complete output fields consumed; candidate includes invariant-result consumption",
        "geometry": "six nominal asset ratios; signed/scaled synthetic targets and exact vertical/near-axis/fold probes",
        "baseline": "benchmark-only copies of Identification geometric kernels, not an older DLL",
        "private_storage_bytes": json.loads((output / "layout.json").read_text()),
        "command": [sys.executable, *sys.argv],
        "compile_commands": [entry for entry in json.loads(commands.read_text())
                             if "crx_residual_benchmark.dir" in entry["output"]],
        "source_hashes": {p.relative_to(ROOT).as_posix(): sha256(p) for p in sorted(sources)},
        "eigen_header_hashes": {p.relative_to(ROOT / "third_party/eigen").as_posix(): sha256(p)
                                for p in sorted((ROOT / "third_party/eigen/Eigen").rglob("*"))
                                if p.is_file()},
        "hashes": {p.name: sha256(p) for p in output.iterdir() if p.is_file()},
    }
    (output / "summary.json").write_text(json.dumps({"metadata": metadata, "groups": groups}, indent=2) + "\n")
    print(json.dumps(groups["all"], indent=2))


if __name__ == "__main__":
    main()
