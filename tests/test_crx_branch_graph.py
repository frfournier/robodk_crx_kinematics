# SPDX-License-Identifier: Apache-2.0
# SPDX-FileCopyrightText: 2026 Centre Technologique en Aérospatiale
"""Production CLI and independent branch-graph/evidence audits."""

import copy
from fractions import Fraction as F
import json
import os
from pathlib import Path
import struct
import subprocess
import sys

import mpmath as mp
import pytest

from crx_kinematics.paths import encode_request, _strict_json
import crx_exact_reference as ref
from path_interval_reference import I, audit_uniform, pi_interval, sincos
from test_crx_exact_target import unflatten
from test_crx_paths import audit_cell, audit_enclosure, inclusion, trace_request
from test_crx_root_certificate import check_geometry


ROOT = Path(__file__).resolve().parents[1]


@pytest.fixture(scope="module")
def backend():
    configured = os.environ.get("CRXKIN_PATH_BACKEND")
    path = Path(configured) if configured else ROOT/"build/Release/crx_path_tool.exe"
    if configured:
        assert path.is_file(), path
    elif not path.is_file():
        pytest.skip("Build crx_path_tool or set CRXKIN_PATH_BACKEND")
    return path


@pytest.fixture(scope="module")
def example():
    return json.loads((ROOT/"docs/examples/path-coverage.json").read_text(encoding="utf-8"))


@pytest.fixture(scope="module")
def run_graph(backend, tmp_path_factory):
    directory = tmp_path_factory.mktemp("branch-graph")
    def run(request):
        path = directory/"request.bin"
        path.write_bytes(encode_request(request))
        result = subprocess.run([str(backend), str(path)], capture_output=True,
                                check=True, timeout=90)
        return json.loads(result.stdout)
    return run


@pytest.fixture(scope="module")
def graph(run_graph, example):
    return run_graph(example)


def decoded(words):
    return unflatten(struct.unpack("<77d", struct.pack("<77Q", *words)))


def test_graph_components_use_only_complete_edges(graph):
    assert graph["status"] == 2 and graph["global_coverage"] == "Inconclusive"
    assert len(graph["vertices"]) == 32 and len(graph["components"]) == 16
    adjacency = {i: set() for i in range(len(graph["vertices"]))}
    assert len(graph["edges"]) == 16
    for edge in graph["edges"]:
        attempt = graph["attempts"][edge]
        trace = attempt["path"]
        pair = graph["pairs"][attempt["pair"]]
        a, b = pair["first"], pair["last"]
        assert trace["complete"] and pair["attempted"] and pair["attempt"] == edge
        assert trace["first"] == graph["vertices"][a]["root"]
        assert trace["last"] == graph["vertices"][b]["root"]
        adjacency[a].add(b)
        adjacency[b].add(a)
    remaining, expected = set(adjacency), []
    while remaining:
        reached, stack = set(), [min(remaining)]
        while stack:
            v = stack.pop()
            if v in reached:
                continue
            reached.add(v)
            stack.extend(adjacency[v]-reached)
        remaining -= reached
        expected.append(sorted(reached))
    assert sorted(expected) == sorted(part["vertices"] for part in graph["components"])
    for part in graph["components"]:
        tasks = sorted({graph["vertices"][v]["task"] for v in part["vertices"]
                        if graph["vertices"][v]["usable"]})
        assert tasks == part["admitted_tasks"] == [0, 1]
        assert part["covers_all_tasks"]
    assert graph["retained_payload_bytes"] > 0


def test_reported_endpoint_cover_has_independent_exact_root_count(graph):
    for task in graph["tasks"]:
        root = task["root_cover"]
        charts = root["charts"]
        packet = dict(schema=task["input_schema"], words=[format(x, "x") for x in task["input_words"]],
                      complete=root["complete"], status=root["status"], roots=root["simple_roots"],
                      shift=root["shift"], polynomials=[dict(p=c["residual"], d=c["derivative"],
                      q=c["determinant"]) for c in charts], partitions=[dict(status=c["status"],
                      roots=c["roots"], exclusions=c["exclusions"], unresolved=c["unresolved"],
                      depth=c["max_depth"], leaves=[dict(kind=l["kind"], reason=l["reason"],
                      depth=l["depth"], t=l["tangent"], phase=l["phase"], endpoints=l["endpoints"],
                      p=l["residual_bernstein"], d=l["derivative_bernstein"], q=l["determinant_bernstein"],
                      derivative_sign=l["derivative_sign"], determinant_sign=l["determinant_sign"])
                      for l in c["leaves"]]) for c in charts])
        with mp.workdps(180):
            check_geometry(packet, decoded(task["input_words"]), graph["convention"] == 2)


def test_endpoint_lift_membership_and_raw_limits(graph):
    pi = pi_interval()
    for vertex in graph["vertices"]:
        assert vertex["usable"] and vertex["admission"] == 1
        assert vertex["has_command_lift"] and vertex["membership"] == 2
        source = decoded(vertex["root"]["input_words"])
        assert vertex["root"]["input_words"] == graph["tasks"][vertex["task"]]["input_words"]
        signs = [1, 1, -1, -1, -1, -1]
        for i, (box, cb, centre) in enumerate(zip(vertex["command_box"], vertex["canonical_box"],
                                                 vertex["root"]["centre"], strict=True)):
            # Independent exact-degree admission and command-to-canonical map.
            lower = I(graph["lower_degrees"][i])*pi/180
            upper = I(graph["upper_degrees"][i])*pi/180
            assert lower.hi <= F(box[0]) <= F(box[1]) <= upper.lo
            offset = (I(source["encoder_zero"][i])+(pi/2 if i == 1 else I(0)))*signs[i]
            value = offset+I(*box)*signs[i]*source["senses"][i]
            assert F(cb[0]) <= value.lo <= value.hi <= F(cb[1])
            bound = max(abs(F(cb[0])-F(centre)), abs(F(cb[1])-F(centre)))
            assert bound <= F(vertex["inclusion"][1]) <= F(vertex["root"]["constants"][1])


def test_failed_preparation_retains_raw_model_and_target_provenance(run_graph, example):
    request = copy.deepcopy(example)
    request["model"]["base_height_mm"] = -0.0
    # All inputs remain finite and shape-correct; target normalization exceeds
    # the representable range. No exact prepared problem or root is claimed.
    request["model"]["lengths_mm"] = [1e-300, 1e-300, 0., 0.]
    for task in request["tasks"]:
        task["target"]["position_mm"] = [1e300, 0., 0.]
    graph = run_graph(request)
    assert graph["status"] == 1 and not graph["edges"]
    assert all(not task["prepared"] and task["preparation"] != 0 for task in graph["tasks"])
    assert graph["raw_model_words"][4] == 1 << 63
    for task in graph["tasks"]:
        position = struct.unpack("<3d", struct.pack("<3Q", *task["raw_target_words"][4:7]))
        assert position == (1e300, 0., 0.)
    limits = struct.unpack("<24d", struct.pack("<24Q", *graph["raw_limit_words"]))
    assert limits[:6] == tuple(request["limits_degrees"]["lower"])


def test_exact_singular_root_prevents_false_task_infeasibility(run_graph, example):
    request = copy.deepcopy(example)
    request["model"]["lengths_mm"] = [1., .5, .25, .125]
    for task in request["tasks"]:
        task["target"] = dict(quaternion=[0., 1., 0., 1.], position_mm=[.75, -.125, 1.])
    request["options"]["max_paths"] = 1
    graph = run_graph(request)
    # q=(0,0,0,0,0,0) is an exact singular FK witness for these dyadic
    # lengths/pose. Callback commands=(0,-90,0,0,0,0) lie within the limits.
    assert graph["status"] != 3
    assert all(not task["certified_infeasible"] and not task["phase_complete"]
               for task in graph["tasks"])
    rotation, position = ref.forward([1., .5, .25, .125], [0.]*6)
    assert rotation == [0, 0, 1, 0, -1, 0, 1, 0, 0]
    assert position == [mp.mpf(".75"), mp.mpf("-.125"), mp.mpf(1)]


def test_graph_edges_pass_independent_uniform_cell_and_glue_audits(graph):
    for edge in graph["edges"]:
        trace = graph["attempts"][edge]["path"]
        request = trace_request(trace)
        before = F(0)
        for leaf in trace["leaves"]:
            assert leaf["accepted"]
            lo, hi = map(F, leaf["parameter"])
            assert lo == before < hi
            before = hi
            local = {**request, "parameter": leaf["parameter"]}
            cell = {**leaf["cell"], "preparation_failure": 0, "enclosure_failure": 0}
            audit_enclosure(local, cell)
            audit_uniform(local, cell)
            audit_cell(local, cell, parameters=(lo,), refine=False)
        assert before == 1 and len(trace["links"])+1 == len(trace["leaves"])
        for link in trace["links"]:
            first = trace["leaves"][link["first"]]["cell"]
            last = trace["leaves"][link["second"]]["cell"]
            assert inclusion(first["point"]["centre"], first["rho"], last["point"]["centre"]) <= F(link["inclusion"][1]) <= F(last["point"]["constants"][1])
        first, last = trace["leaves"][0]["cell"], trace["leaves"][-1]["cell"]
        assert inclusion(trace["first"]["centre"], trace["first"]["constants"][0], first["point"]["centre"]) <= F(trace["initial_inclusion"][1]) <= F(first["point"]["constants"][1])
        assert inclusion(last["point"]["centre"], last["rho"], trace["last"]["centre"]) <= F(trace["final_inclusion"][1]) <= F(trace["last"]["constants"][1])


def test_base_flip_relation_requires_full_interval_image(graph):
    pi = pi_interval()
    def sine_range(root):
        angle = I(root["centre"][1])+I(-root["constants"][0], root["constants"][0])
        endpoints = [sincos(x)[0] for x in (angle.lo, angle.hi)]
        lo, hi = min(x.lo for x in endpoints), max(x.hi for x in endpoints)
        assert -10 <= angle.lo <= angle.hi <= 10
        for odd in range(-9, 10, 2):
            critical = pi*odd/2
            if critical.lo <= angle.hi and critical.hi >= angle.lo:
                if odd % 4 == 1:
                    hi = F(1)
                else:
                    lo = F(-1)
        return I(lo, hi)
    flips, nonflip_same_sign = 0, 0
    for pair in graph["pairs"]:
        relation = pair["relation"]
        a, b = [graph["vertices"][pair[key]]["root"] for key in ("first", "last")]
        assert relation["singularity_required"] == (a["constants"][-1] != b["constants"][-1])
        for root, key in ((a, "first_shoulder_sine"), (b, "last_shoulder_sine")):
            sine = sine_range(root)
            assert F(relation[key][0]) <= sine.lo <= sine.hi <= F(relation[key][1])
        if relation["base_flip_verified"]:
            flips += 1
            assert relation["shoulder_wall_required"] and not relation["singularity_required"]
            image = [I(c)+I(-a["constants"][0], a["constants"][0]) for c in a["centre"]]
            image[0] += pi
            image[1] = -image[1]
            image[2] = pi-image[2]
            image[3] += pi
            for value, turn, bound in zip(image, relation["flip_turns"], relation["flip_image"], strict=True):
                value += 2*pi*turn
                assert F(bound[0]) <= value.lo <= value.hi <= F(bound[1])
            distance = max(max(abs(F(lo)-F(c)), abs(F(hi)-F(c))) for (lo, hi), c in zip(relation["flip_image"], b["centre"], strict=True))
            assert distance <= F(relation["flip_inclusion"][1]) <= F(b["constants"][1])
        elif not relation["singularity_required"] and not relation["opposite_shoulder_signs"]:
            nonflip_same_sign += a["centre"] != b["centre"]
            assert not relation["shoulder_wall_required"]
    assert flips and nonflip_same_sign
    with mp.workdps(100):
        for j in range(30):
            q = [mp.mpf(((j+3)*(k+5)*17)%211-105)/31 for k in range(6)]
            flipped = [q[0]+mp.pi, -q[1], mp.pi-q[2], q[3]+mp.pi, q[4], q[5]]
            a = ref.forward([1, .6, .2, .5], q)
            b = ref.forward([1, .6, .2, .5], flipped)
            assert max(abs(x-y) for x, y in zip(sum(a, []), sum(b, []), strict=True)) < mp.mpf("1e-95")


@pytest.mark.parametrize("overrides", [
    {"options": {"max_paths": 0}},
    {"options": {"max_pair_tests": 3, "max_paths": 0}},
    {"options": {"max_vertices": 0}},
    {"options": {"max_path_nodes": 0}},
    {"options": {"max_endpoint_certificates": 0}},
])
def test_work_exhaustion_retains_partial_tasks_without_infeasibility(run_graph, example, overrides):
    request = copy.deepcopy(example)
    request.update(overrides)
    graph = run_graph(request)
    assert graph["status"] == 1 and len(graph["tasks"]) == 2 and not graph["edges"]
    assert not any(task["certified_infeasible"] for task in graph["tasks"])
    for connection in graph["connections"]:
        assert connection["considered_pairs"] <= connection["total_pairs"]
        assert connection["scan_complete"] == (connection["considered_pairs"] == connection["total_pairs"])
    if overrides["options"].get("max_paths") == 0:
        assert len(graph["components"]) == len(graph["vertices"])


def test_incomplete_catalogue_retains_verified_partial_roots(run_graph, example):
    request = copy.deepcopy(example)
    request["options"]["max_root_nodes"] = 0
    graph = run_graph(request)
    assert graph["vertices"]
    assert all(not task["phase_complete"] and not task["vertices_complete"] for task in graph["tasks"])
    for vertex in graph["vertices"]:
        assert not vertex["has_command_lift"] and vertex["membership"] == 1
        if vertex["usable"]:
            assert vertex["partial_admission"]["admitted"]


@pytest.mark.parametrize("reverse", [False, True])
def test_graph_small_motion_and_reversal_keep_exact_endpoint_roots(run_graph, example, reverse):
    request = copy.deepcopy(example)
    request["tasks"][1]["target"]["position_mm"][0] += 1e-12
    if reverse:
        request["connections"] = [["again", "first"]]
    graph = run_graph(request)
    assert graph["status"] == 2 and len(graph["edges"]) == 16
    for edge in graph["edges"]:
        attempt = graph["attempts"][edge]
        pair = graph["pairs"][attempt["pair"]]
        trace = attempt["path"]
        assert trace["complete"]
        assert trace["first"]["input_words"] == graph["vertices"][pair["first"]]["root"]["input_words"]
        assert trace["last"]["input_words"] == graph["vertices"][pair["last"]]["root"]["input_words"]
        for leaf in trace["leaves"]:
            assert leaf["accepted"]
            audit_uniform({**trace_request(trace), "parameter": leaf["parameter"]}, leaf["cell"])


def test_graph_retains_distinct_real_turn_components(run_graph, example):
    request = copy.deepcopy(example)
    request["limits_degrees"]["lower"][0] = -360
    request["limits_degrees"]["upper"][0] = 360
    request["options"]["max_paths"] = 32
    graph = run_graph(request)
    assert graph["status"] == 2 and len(graph["vertices"]) == 64
    assert len(graph["edges"]) == len(graph["components"]) == 32
    for phase in range(16):
        vertices = [v for v in graph["vertices"] if v["task"] == 0 and v["phase"] == phase]
        assert len(vertices) == 2
        assert abs(vertices[0]["turns"][0]-vertices[1]["turns"][0]) == 1
        assert abs(vertices[0]["root"]["centre"][0]-vertices[1]["root"]["centre"][0]) > 6
    assert all(len(part["vertices"]) == 2 for part in graph["components"])


def test_decoupled_user_lifts_bind_to_the_correct_canonical_roots(run_graph, example):
    request = copy.deepcopy(example)
    request["profile"] = "decoupled_user"
    graph = run_graph(request)
    assert graph["status"] == 2 and graph["profile"] == 1 and len(graph["edges"]) == 16
    pi = pi_interval()
    for vertex in graph["vertices"]:
        raw = decoded(vertex["root"]["input_words"])
        callback = [I(*box) for box in vertex["command_box"]]
        callback[2] += callback[1]
        for i, (box, command) in enumerate(zip(vertex["canonical_box"], callback, strict=True)):
            sign = [1, 1, -1, -1, -1, -1][i]
            offset = (I(raw["encoder_zero"][i])+(pi/2 if i == 1 else I(0)))*sign
            q = offset+sign*raw["senses"][i]*command
            assert F(box[0]) <= q.lo <= q.hi <= F(box[1])
    for edge in graph["edges"]:
        trace = graph["attempts"][edge]["path"]
        for leaf in trace["leaves"]:
            assert leaf["cell"]["slack_lipschitz"][2] == 2
            audit_uniform({**trace_request(trace), "parameter": leaf["parameter"]}, leaf["cell"])


def test_nonzero_endpoint_quaternions_cannot_certify_a_vanishing_interior(run_graph, example):
    request = copy.deepcopy(example)
    request["tasks"][1]["target"]["quaternion"] = [-v for v in request["tasks"][0]["target"]["quaternion"]]
    request["options"].update(max_path_nodes=31, max_paths=16)
    graph = run_graph(request)
    assert all(task["phase_complete"] for task in graph["tasks"])
    assert graph["status"] == 1 and not graph["edges"]
    assert all(not attempt["path"]["complete"] for attempt in graph["attempts"])
    assert any(not leaf["accepted"] and leaf["parameter"][0] <= .5 <= leaf["parameter"][1]
               for attempt in graph["attempts"] for leaf in attempt["path"]["leaves"])


def test_cli_round_trip_and_atomic_error(backend, example, tmp_path):
    request, output = tmp_path/"request.json", tmp_path/"report.json"
    request.write_text(json.dumps(example), encoding="utf-8")
    command = [sys.executable, "-m", "crx_kinematics.paths", str(request),
               "--backend", str(backend), "--output", str(output)]
    result = subprocess.run(command, capture_output=True, check=False, timeout=90)
    assert result.returncode == 0, result.stderr
    assert json.loads(output.read_text(encoding="utf-8"))["status"] == 2
    previous = output.read_bytes()
    request.write_text('{"schema":1,"schema":1}', encoding="utf-8")
    result = subprocess.run(command, capture_output=True, check=False, timeout=90)
    assert result.returncode == 2 and b"Duplicate JSON field" in result.stderr
    assert output.read_bytes() == previous


@pytest.mark.parametrize("change", [
    {"schema": True}, {"unexpected": 1}, {"limits_degrees": None},
    {"tasks": []}, {"options": {"max_paths": True}},
    {"tracking": {"unknown": 3}}, {"profile": "guess"},
])
def test_json_frontend_rejects_invalid_requests(example, change):
    request = copy.deepcopy(example)
    request.update(change)
    with pytest.raises((ValueError, TypeError)):
        encode_request(request)


@pytest.mark.parametrize("payload", [b"", b"CRXPATH2"+bytes(1000), b"CRXPATH1"+bytes(16)],
                         ids=["empty", "version", "counts"])
def test_backend_rejects_invalid_wire_before_work(backend, tmp_path, payload):
    filename = tmp_path/"bad.bin"
    filename.write_bytes(payload)
    result = subprocess.run([str(backend), str(filename)], capture_output=True, timeout=10)
    assert result.returncode == 2 and not result.stdout


def test_native_request_reader_accepts_unicode_paths(backend, example, tmp_path):
    filename = tmp_path/"requête-路径.bin"
    filename.write_bytes(encode_request(example))
    result = subprocess.run([str(backend), str(filename)], capture_output=True, check=True, timeout=90)
    assert json.loads(result.stdout)["status"] == 2


def test_strict_json_rejects_nonfinite_and_duplicate_fields():
    for text in ('{"x":NaN}', '{"x":Infinity}', '{"x":1,"x":2}'):
        with pytest.raises(ValueError):
            _strict_json(text)


def test_frozen_asset_workload_retains_actual_motion_and_common_start_poses():
    workload = json.loads((ROOT/"tests/fixtures/crx_path_queries.json").read_text(encoding="utf-8"))
    assert len(workload["cases"]) == 42 and len(workload["source_assets"]) == 6
    starts = {}
    observed = 0
    for case in workload["cases"]:
        if case["model"] == "synthetic":
            continue
        first, last = [task["target"] for task in case["request"]["tasks"]]
        actual = last["position_mm"][0]-first["position_mm"][0]
        expected = case["declared_translation_delta_mm"]
        assert (actual == 0) == (expected == 0), case["id"]
        if expected:
            assert abs(actual/expected-1) < .001, case["id"]
        assert first["position_mm"][1:] == last["position_mm"][1:]
        assert first["quaternion"] == last["quaternion"]
        previous = starts.setdefault(case["model"], first)
        assert first == previous, case["id"]
        observed += 1
    assert observed == 24 and len(starts) == 6


@pytest.mark.parametrize("model", ["CRX-5iA", "CRX-10iA", "CRX-10iA-L", "CRX-20iA-L",
                                  "CRX-25iA", "CRX-30iA"])
def test_named_model_metadata_preserves_declared_raw_target(model, example, backend, kinematics_lib, tmp_path):
    request = copy.deepcopy(example)
    request["model"] = model
    request["options"].update(max_root_nodes=0, max_endpoint_certificates=0, max_paths=0)
    request["reference"] = {"quaternion": [2, 0, 0, 1], "position_mm": [3, -4, 5]}
    request["tool"] = {"quaternion": [3, 1, 0, 0], "position_mm": [2, -1, 4]}
    filename = tmp_path/"named.bin"
    filename.write_bytes(encode_request(request, dll_path=kinematics_lib._name))
    result = subprocess.run([str(backend), str(filename)], capture_output=True, check=True, timeout=90)
    graph = json.loads(result.stdout)
    assert graph["status"] == 1 and graph["convention"] == 2
    for task, expected in zip(graph["tasks"], request["tasks"], strict=True):
        assert task["prepared"] and not task["phase_complete"]
        raw = decoded(task["input_words"])
        assert raw["target"]["quaternion"] == expected["target"]["quaternion"]
        assert raw["target"]["position"] == expected["target"]["position_mm"]
        assert raw["base"]["quaternion"] == request["reference"]["quaternion"]
        assert raw["tool"]["position"] == request["tool"]["position_mm"]
