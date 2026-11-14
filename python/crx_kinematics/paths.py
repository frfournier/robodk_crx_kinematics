# SPDX-License-Identifier: Apache-2.0
# SPDX-FileCopyrightText: 2026 Centre Technologique en Aérospatiale
"""JSON frontend for the native bounded branch-coverage tool.

Run ``python -m crx_kinematics.paths request.json --backend crx_path_tool.exe``.
The Python package contains no solver executable. Raw geometry requests need
no solver DLL; named models use the existing solver DLL to obtain exact model
provenance. All proof decisions run in C++.
"""

from __future__ import annotations

import argparse
import json
import math
import os
from pathlib import Path
import struct
import subprocess
import sys
import tempfile


GRAPH_DEFAULTS = {
    "max_vertices": 512, "max_pair_tests": 4096, "max_paths": 32,
    "max_root_nodes": 32768, "max_endpoint_certificates": 4096,
    "max_lifts": 4096, "max_path_nodes": 4096,
    "max_path_certificates": 32768,
}
ENDPOINT_DEFAULTS = {
    "max_nodes": 8192, "max_depth": 40, "max_leaves": 4096,
    "max_chart_attempts": 8, "max_candidate_certificates": 512,
    "max_pair_tests": 120,
}
TRACKING_DEFAULTS = {
    "max_nodes": 1023, "max_depth": 32,
    "max_point_certificates": 8192, "max_trials_per_node": 17,
}


def _object(value, keys, label):
    if not isinstance(value, dict):
        raise ValueError(f"{label} must be an object")
    unknown = value.keys() - set(keys)
    if unknown:
        raise ValueError(f"Unknown {label} fields: {sorted(unknown)}")
    return value


def _reals(value, count, label):
    if not isinstance(value, list) or len(value) != count:
        raise ValueError(f"{label} must contain {count} finite numbers")
    if any(isinstance(x, bool) or not isinstance(x, (int, float)) for x in value):
        raise ValueError(f"{label} must contain numbers")
    result = [float(x) for x in value]
    if not all(math.isfinite(x) for x in result):
        raise ValueError(f"{label} must contain finite numbers")
    return result


def _pose(value, label):
    _object(value, ("quaternion", "position_mm", "input_rotation"), label)
    q = _reals(value.get("quaternion", [1, 0, 0, 0]), 4, f"{label}.quaternion")
    p = _reals(value.get("position_mm", [0, 0, 0]), 3, f"{label}.position_mm")
    scale = max(map(abs, q))
    if not scale:
        raise ValueError(f"{label}.quaternion must be nonzero")
    # Diagnostic matrix only. The exact native target always uses the original
    # raw homogeneous quaternion; this rounded matrix defines no geometry.
    w, x, y, z = [v/scale for v in q]
    n = w*w+x*x+y*y+z*z
    rotation = [(w*w+x*x-y*y-z*z)/n, 2*(x*y-w*z)/n, 2*(x*z+w*y)/n,
                2*(x*y+w*z)/n, (w*w-x*x+y*y-z*z)/n, 2*(y*z-w*x)/n,
                2*(x*z-w*y)/n, 2*(y*z+w*x)/n, (w*w-x*x-y*y+z*z)/n]
    return q+p+_reals(value.get("input_rotation", rotation), 9, f"{label}.input_rotation")


def _options(value, defaults, label):
    _object(value, defaults, label)
    result = dict(defaults)
    for key, number in value.items():
        if type(number) is not int or not 0 <= number <= 2**32-1:
            raise ValueError(f"{label}.{key} must be an unsigned 32-bit integer")
        result[key] = number
    return result


def _model(value, request, first_pose, dll_path):
    convention = request.get("convention", "reference_map" if isinstance(value, str) else "base_pose")
    if convention not in ("base_pose", "reference_map"):
        raise ValueError("convention must be base_pose or reference_map")
    if isinstance(value, str):
        if convention != "reference_map":
            raise ValueError("Named models require reference_map convention")
        import numpy as np
        from . import CatalogueOptions, CrxSolver, QueryPreparation

        with CrxSolver(value, dll_path=dll_path) as solver:
            snapshot = solver.catalogue(np.eye(4), options=CatalogueOptions(
                max_nodes=0, max_candidate_certificates=0, max_lifts=0))
            summary = snapshot.summary
            snapshot.close()
        if summary.preparation != QueryPreparation.READY:
            raise ValueError("DLL did not provide supported exact nominal model provenance")
        words = [int(x) for x in summary.input_words]
        if len(words) != 77:
            raise ValueError("DLL returned an unsupported exact model identity")
        raw = list(struct.unpack("<77d", struct.pack("<77Q", *words)))
        # DLL metadata supplies the model only. Neither its rounded matrix
        # target nor its endpoint root catalogue is reused for this request.
        raw[29:45] = first_pose
        raw[45:61] = _pose(request.get("reference", {}), "reference")
        raw[61:77] = _pose(request.get("tool", {}), "tool")
    else:
        keys = ("lengths_mm", "base_height_mm", "encoder_zero_radians", "senses",
                "lower_commands_radians", "upper_commands_radians", "base", "tool")
        _object(value, keys, "model")
        for key in ("lengths_mm", "lower_commands_radians", "upper_commands_radians"):
            if key not in value:
                raise ValueError(f"Raw model requires {key}")
        if "reference" in request or "tool" in request:
            raise ValueError("Raw model frames belong in model.base and model.tool")
        lengths = _reals(value["lengths_mm"], 4, "model.lengths_mm")
        height = _reals([value.get("base_height_mm", 0)], 1, "model.base_height_mm")
        zeros = _reals(value.get("encoder_zero_radians", [0]*6), 6, "model.encoder_zero_radians")
        senses = _reals(value.get("senses", [1]*6), 6, "model.senses")
        lower = _reals(value["lower_commands_radians"], 6, "model.lower_commands_radians")
        upper = _reals(value["upper_commands_radians"], 6, "model.upper_commands_radians")
        if lengths[0] == 0 or lengths[1] == 0 or any(x not in (-1, 1) for x in senses):
            raise ValueError("Raw model needs nonzero a/b lengths and unit senses")
        if any(a > b for a, b in zip(lower, upper, strict=True)):
            raise ValueError("Raw model lower bounds exceed upper bounds")
        raw = (lengths+height+zeros+senses+lower+upper+first_pose
               +_pose(value.get("base", {}), "model.base")
               +_pose(value.get("tool", {}), "model.tool"))
    return raw, {"base_pose": 1, "reference_map": 2}[convention]


def encode_request(request, *, dll_path=None):
    """Validate a schema-1 JSON object and encode the bounded native request."""
    _object(request, ("schema", "model", "convention", "reference", "tool", "profile",
                      "limits_degrees", "seed_degrees", "weights", "tasks",
                      "connections", "options", "endpoint", "tracking"), "request")
    if type(request.get("schema")) is not int or request["schema"] != 1:
        raise ValueError("Request schema must be 1")
    tasks = request.get("tasks")
    if not isinstance(tasks, list) or not 1 <= len(tasks) <= 64:
        raise ValueError("tasks must contain 1 to 64 entries")
    ids, poses = [], []
    for task in tasks:
        _object(task, ("id", "target"), "task")
        name = task.get("id")
        if (not isinstance(name, str) or not 1 <= len(name) <= 256
                or any(not 32 <= ord(c) <= 126 for c in name) or name in ids):
            raise ValueError("Task IDs must be unique printable ASCII strings of 1 to 256 characters")
        ids.append(name)
        if "target" not in task:
            raise ValueError("Each task requires a target")
        poses.append(_pose(task["target"], f"task {name}.target"))
    connections = request.get("connections", list(zip(ids, ids[1:])))
    if not isinstance(connections, list) or len(connections) > 256:
        raise ValueError("connections must contain at most 256 ID pairs")
    pairs = []
    for pair in connections:
        if not isinstance(pair, (list, tuple)) or len(pair) != 2 or any(x not in ids for x in pair):
            raise ValueError("Each connection must name two declared task IDs")
        pairs.append([ids.index(x) for x in pair])
    limits = request.get("limits_degrees")
    _object(limits, ("lower", "upper"), "limits_degrees")
    lower = _reals(limits.get("lower"), 6, "limits_degrees.lower")
    upper = _reals(limits.get("upper"), 6, "limits_degrees.upper")
    if any(a > b for a, b in zip(lower, upper, strict=True)):
        raise ValueError("Lower degree limits exceed upper limits")
    seed = _reals(request.get("seed_degrees", [0]*6), 6, "seed_degrees")
    weights = _reals(request.get("weights", [1]*6), 6, "weights")
    if any(x <= 0 for x in weights):
        raise ValueError("weights must be positive")
    profile = request.get("profile", "callback")
    if profile not in ("callback", "decoupled_user"):
        raise ValueError("profile must be callback or decoupled_user")
    raw, convention = _model(request.get("model"), request, poses[0], dll_path)
    graph = _options(request.get("options", {}), GRAPH_DEFAULTS, "options")
    endpoint = _options(request.get("endpoint", {}), ENDPOINT_DEFAULTS, "endpoint")
    tracking = _options(request.get("tracking", {}), TRACKING_DEFAULTS, "tracking")
    packet = bytearray(b"CRXPATH1")
    packet += struct.pack("<4I", len(tasks), len(pairs), convention, profile == "decoupled_user")
    packet += struct.pack("<101dI", *(raw+lower+upper+seed+weights), "seed_degrees" in request)
    packet += struct.pack("<18I", *graph.values(), *endpoint.values(), *tracking.values())
    for name, pose in zip(ids, poses, strict=True):
        packet += struct.pack("<I", len(name))+name.encode("ascii")+struct.pack("<16d", *pose)
    for pair in pairs:
        packet += struct.pack("<2I", *pair)
    return bytes(packet)


def _strict_json(text):
    def invalid_constant(value):
        raise ValueError(f"Nonfinite JSON number {value}")
    def unique_pairs(pairs):
        result = {}
        for key, value in pairs:
            if key in result:
                raise ValueError(f"Duplicate JSON field {key}")
            result[key] = value
        return result
    return json.loads(text, parse_constant=invalid_constant, object_pairs_hook=unique_pairs)


def main(argv=None):
    parser = argparse.ArgumentParser(description="Build a certified subgraph of nominal CRX task branches.")
    parser.add_argument("request", type=Path, help="Schema-1 JSON request")
    parser.add_argument("--backend", type=Path, default=os.environ.get("CRXKIN_PATH_BACKEND"),
                        help="Path to the CMake-built crx_path_tool.exe")
    parser.add_argument("--dll", type=Path, help="Solver DLL providing named-model provenance")
    parser.add_argument("--output", type=Path, help="Atomically replace this JSON report after success")
    args = parser.parse_args(argv)
    try:
        if args.backend is None or not args.backend.is_file():
            raise ValueError("Supply --backend or CRXKIN_PATH_BACKEND pointing to crx_path_tool.exe")
        if args.request.stat().st_size > 1048576:
            raise ValueError("JSON request exceeds 1 MiB")
        packet = encode_request(_strict_json(args.request.read_text(encoding="utf-8-sig")), dll_path=args.dll)
        with tempfile.TemporaryDirectory(prefix="crx-path-request-") as directory:
            filename = Path(directory)/"request.bin"
            filename.write_bytes(packet)
            result = subprocess.run([str(args.backend.resolve()), str(filename)],
                                    capture_output=True, check=False)
        if result.returncode:
            raise ValueError(result.stderr.decode("utf-8", errors="replace").strip()
                             or f"Native backend exited {result.returncode}")
        # Native output is owned evidence. Parsing detects a failed/truncated
        # output stream, and does not turn this JSON into an accepted proof.
        _strict_json(result.stdout.decode("utf-8"))
        if args.output is None:
            sys.stdout.buffer.write(result.stdout)
        else:
            temporary = None
            try:
                with tempfile.NamedTemporaryFile(dir=args.output.resolve().parent,
                                                 prefix=".crx-path-", delete=False) as stream:
                    temporary = Path(stream.name)
                    stream.write(result.stdout)
                os.replace(temporary, args.output)
            finally:
                if temporary is not None:
                    temporary.unlink(missing_ok=True)
        return 0
    except (OSError, ValueError, TypeError, OverflowError, ImportError, RuntimeError) as error:
        print(f"crx-paths: {error}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
