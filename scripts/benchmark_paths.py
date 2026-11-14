# SPDX-License-Identifier: Apache-2.0
# SPDX-FileCopyrightText: 2026 Centre Technologique en Aérospatiale
"""Reproduce native path/graph stages from the same public JSON request."""

import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import tempfile

from crx_kinematics.paths import encode_request, _strict_json


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("request", type=Path)
    parser.add_argument("--case", help="Select one id from a frozen workload manifest")
    parser.add_argument("--harness", type=Path, required=True)
    parser.add_argument("--dll", type=Path, help="Named-model metadata DLL")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--audit", action="store_true", help="Run checks without collecting timings")
    parser.add_argument("--iterations", type=int, default=3)
    parser.add_argument("--rounds", type=int, default=5)
    args = parser.parse_args(argv)
    try:
        if not 1 <= args.iterations <= 10000 or not 1 <= args.rounds <= 15:
            raise ValueError("iterations must be 1..10000 and rounds 1..15")
        if args.request.stat().st_size > 1048576:
            raise ValueError("JSON request exceeds 1 MiB")
        raw = args.request.read_bytes()
        request = _strict_json(raw.decode("utf-8-sig"))
        if args.case is not None:
            if not isinstance(request, dict) or not isinstance(request.get("cases"), list):
                raise ValueError("--case requires a frozen workload manifest")
            if not all(isinstance(case, dict) and "id" in case and "request" in case
                       for case in request["cases"]):
                raise ValueError("Each workload case requires id and request")
            selected = [case for case in request["cases"] if case.get("id") == args.case]
            if len(selected) != 1:
                raise ValueError("--case must select exactly one declared workload id")
            request = selected[0]["request"]
        elif isinstance(request, dict) and "cases" in request:
            raise ValueError("A workload manifest requires --case")
        packet = encode_request(request, dll_path=args.dll)
        with tempfile.TemporaryDirectory(prefix="crx-path-benchmark-") as directory:
            filename = Path(directory)/"request.bin"
            filename.write_bytes(packet)
            command = [str(args.harness.resolve()), str(filename)]
            command += ["--audit"] if args.audit else [str(args.iterations), str(args.rounds)]
            result = subprocess.run(command, capture_output=True, check=False)
        if result.returncode:
            raise ValueError(result.stderr.decode("utf-8", errors="replace").strip()
                             or f"Harness exited {result.returncode}")
        native = _strict_json(result.stdout.decode("utf-8"))
        if bool(native["audit_only"]) != args.audit:
            raise ValueError("Harness returned an unexpected measurement mode")
        report = dict(schema=1, request_sha256=hashlib.sha256(raw).hexdigest(),
                      case=args.case,
                      packet_sha256=hashlib.sha256(packet).hexdigest(),
                      harness_sha256=hashlib.sha256(args.harness.read_bytes()).hexdigest(),
                      python=sys.version, native=native)
        args.output.write_text(json.dumps(report, allow_nan=False, separators=(",", ":"))+"\n", encoding="utf-8")
        return 0
    except (OSError, ValueError, TypeError, OverflowError, ImportError, RuntimeError) as error:
        print(f"benchmark_paths: {error}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
