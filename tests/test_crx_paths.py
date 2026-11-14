# SPDX-License-Identifier: Apache-2.0
# SPDX-FileCopyrightText: 2026 Centre Technologique en Aérospatiale
"""Independent all-parameter rational enclosures and high-precision path roots."""

import copy
from fractions import Fraction as F
import json
import math
import os
from pathlib import Path
import subprocess
import struct

import mpmath as mp
import pytest

import crx_exact_reference as ref
from test_crx_ball_certificate import check_certificate, exact_kinematics, generated
from test_crx_exact_target import case as random_input, flatten, unflatten
from path_interval_reference import audit_uniform


@pytest.fixture(scope="module")
def path_probe():
    configured = os.environ.get("CRXKIN_PATH_PROBE_PATH")
    path = Path(configured) if configured else Path(__file__).resolve().parents[1]/"build/Release/crx_path_tests.exe"
    if configured:
        assert path.is_file(), path
    elif not path.is_file():
        pytest.skip("Build crx_path_tests or set CRXKIN_PATH_PROBE_PATH")

    def run(requests=None):
        if requests is None:
            command, payload = [str(path), "--witness"], None
        else:
            lines = []
            for request in requests:
                source = request["input"]
                pose = lambda raw: [*raw["quaternion"], *raw["position"], *raw["input_rotation"]]
                inverse = request.get("inverse")
                values = [*flatten(source), *pose(request.get("last", source["target"])),
                    *pose(request.get("point", source["target"])), *request.get("centre", [0.]*6),
                    *request.get("parameter", [0., 1.]), request.get("radius", 0.),
                    *request.get("lower", [-720.]*6), *request.get("upper", [720.]*6),
                    request.get("profile", 0), int(inverse is not None),
                    *(inverse if inverse is not None else [0.]*36)]
                lines.append(" ".join([str(request.get("convention", 1)), *(repr(float(x)) for x in values[:-38]),
                    str(int(values[-38])), str(int(values[-37])), *(repr(float(x)) for x in values[-36:])]))
            command, payload = [str(path), "--batch"], "\n".join(lines)+"\n"
        result = subprocess.run(command, input=payload, text=True, capture_output=True, check=True, timeout=90)
        rows = [json.loads(line) for line in result.stdout.splitlines()]
        assert len(rows) == (16 if requests is None else len(requests))
        return rows
    return run


def add(a, b):
    return [(a[i] if i < len(a) else F(0))+(b[i] if i < len(b) else F(0))
            for i in range(max(len(a), len(b)))]


def scale(a, value):
    return [x*F(value) for x in a]


def sub(a, b):
    return add(a, scale(b, -1))


def mul(a, b):
    result = [F(0)]*(len(a)+len(b)-1)
    for i, x in enumerate(a):
        for j, y in enumerate(b): result[i+j] += x*y
    return result


def total(values):
    result = [F(0)]
    for value in values: result = add(result, value)
    return result


def nonnegative(polynomial, lo, hi, depth=0):
    """Exact Bernstein proof over the WHOLE interval; no sampled acceptance."""
    n, width = len(polynomial)-1, hi-lo
    coefficients = [sum(polynomial[i]*math.comb(i, k)*lo**(i-k)*width**k
                        for i in range(k, n+1)) for k in range(n+1)]
    bernstein = [sum(coefficients[k]*F(math.comb(j, k), math.comb(n, k))
                     for k in range(j+1)) for j in range(n+1)]
    if min(bernstein) >= 0: return True
    if depth == 12: return False
    mid = (lo+hi)/2
    return nonnegative(polynomial, lo, mid, depth+1) and nonnegative(polynomial, mid, hi, depth+1)


def rational_path(request):
    """Independent homogeneous quaternion polynomials before any native bounds."""
    source, last = request["input"], request.get("last", request["input"]["target"])
    first = source["target"]
    q = [[F(a), F(b)-F(a)] for a, b in zip(first["quaternion"], last["quaternion"], strict=True)]
    w, x, y, z = q
    ww, xx, yy, zz = [mul(v, v) for v in q]
    denominator = total([ww, xx, yy, zz])
    rotation = [
        [sub(sub(add(ww, xx), yy), zz), scale(sub(mul(x, y), mul(w, z)), 2), scale(add(mul(x, z), mul(w, y)), 2)],
        [scale(add(mul(x, y), mul(w, z)), 2), sub(add(sub(ww, xx), yy), zz), scale(sub(mul(y, z), mul(w, x)), 2)],
        [scale(sub(mul(x, z), mul(w, y)), 2), scale(add(mul(y, z), mul(w, x)), 2), add(sub(sub(ww, xx), yy), zz)],
    ]
    rb = ref.quaternion_rotation(source["base"]["quaternion"])
    ru = ref.quaternion_rotation(source["tool"]["quaternion"])
    reference = request.get("convention", 1) == 2
    left = rb if reference else ref.transpose(rb)
    middle = [[total([scale(rotation[k][j], left[i][k]) for k in range(3)]) for j in range(3)] for i in range(3)]
    r = [[total([scale(middle[i][k], ru[j][k]) for k in range(3)]) for j in range(3)] for i in range(3)]
    length_scale = max(abs(F(v)) for v in source["lengths"])
    p = [[F(a)/length_scale, (F(b)-F(a))/length_scale]
         for a, b in zip(first["position"], last["position"], strict=True)]
    base = [F(v)/length_scale for v in source["base"]["position"]]
    tool = [F(v)/length_scale for v in source["tool"]["position"]]
    if reference:
        shift = [add(total([scale(p[j], left[i][j]) for j in range(3)]), [base[i]]) for i in range(3)]
    else:
        shift = [total([scale(sub(p[j], [base[j]]), left[i][j]) for j in range(3)]) for i in range(3)]
        shift[2] = sub(shift[2], [F(source["base_height"])/length_scale])
    position = [sub(mul(shift[i], denominator), total([scale(r[i][j], tool[j]) for j in range(3)])) for i in range(3)]
    return denominator, r, position


def audit_enclosure(request, answer):
    assert answer["preparation_failure"] == answer["enclosure_failure"] == 0
    denominator, rotation, position = rational_path(request)
    lo, hi = map(F, request.get("parameter", [0., 1.]))
    assert nonnegative(denominator, lo, hi)
    for polynomial, bound in zip([v for row in rotation for v in row]+position,
                                  answer["rotation"]+answer["translation"], strict=True):
        assert nonnegative(sub(polynomial, scale(denominator, F(bound[0]))), lo, hi)
        assert nonnegative(sub(scale(denominator, F(bound[1])), polynomial), lo, hi)
    first = request["input"]["target"]["quaternion"]
    last = request.get("last", request["input"]["target"])["quaternion"]
    common = max(abs(F(v)) for v in [*first, *last])
    norm = answer["quaternion_norm"]
    assert F(norm[0]) > 0
    assert nonnegative(sub(denominator, [F(norm[0])*common**2]), lo, hi)
    assert nonnegative(sub([F(norm[1])*common**2], denominator), lo, hi)


def audit_cell(request, answer, parameters=(F(0), F(1, 2), F(1)), refine=True):
    assert answer["local"] and answer["point"]["failure"] == 0
    audit_uniform(request, answer)
    point = answer["point"]
    centre = point["centre"]
    with mp.workdps(180):
        point_input = unflatten(point["input"])
        check_certificate(point, refine=refine, kinematics=exact_kinematics(point_input, centre, point["schema"] == 2))
        constants = point["constants"].copy()
        constants[0], constants[5], constants[6] = answer["rho"], answer["eta"], answer["defect"]
        family = {**point, "constants": constants, "coarse_eta": answer["coarse_eta"],
                  "used_log_series": answer["used_log_series"], "trace": answer["trace"]}
        first, last = request["input"]["target"], request.get("last", request["input"]["target"])
        for s in parameters:
            actual = copy.deepcopy(request["input"])
            actual["target"]["quaternion"] = [(1-s)*F(a)+s*F(b) for a, b in zip(first["quaternion"], last["quaternion"], strict=True)]
            actual["target"]["position"] = [(1-s)*F(a)+s*F(b) for a, b in zip(first["position"], last["position"], strict=True)]
            check_certificate(family, refine=refine, kinematics=exact_kinematics(actual, centre, point["schema"] == 2))
        reference_senses = [1, 1, -1, -1, -1, -1]
        offsets = [(mp.mpf(z)+(mp.pi/2 if i == 1 else 0))*reference_senses[i]
                   for i, z in enumerate(point_input["encoder_zero"])]
        commands = [mp.mpf(s)*r*(mp.mpf(c)-o) for c, s, r, o in
                    zip(centre, point_input["senses"], reference_senses, offsets, strict=True)]
        if request.get("profile", 0) == 1: commands[2] -= commands[1]
        row_norms = [1, 1, 2 if request.get("profile", 0) else 1, 1, 1, 1]
        for i, (command, row_norm) in enumerate(zip(commands, row_norms, strict=True)):
            lower = mp.mpf(request.get("lower", [-720.]*6)[i])*mp.pi/180
            upper = mp.mpf(request.get("upper", [720.]*6)[i])*mp.pi/180
            assert answer["slack_lipschitz"][i] == row_norm
            reserve = min(command-lower, upper-command)
            assert mp.mpf(answer["slacks"][i]) <= reserve
            if answer["admitted"]: assert row_norm*mp.mpf(answer["rho"]) <= reserve


@pytest.mark.parametrize("convention", [1, 2])
@pytest.mark.parametrize("scale_value", [1e-6, 1., 1e6])
def test_path_enclosures_cover_every_parameter_rationally(path_probe, convention, scale_value):
    requests = []
    for seed in range(4):
        source = random_input(seed+31, scale_value)
        last = copy.deepcopy(source["target"])
        last["quaternion"] = [v+(i-1.5)*.003 for i, v in enumerate(last["quaternion"])]
        last["position"] = [v+(i-1)*.01*scale_value for i, v in enumerate(last["position"])]
        requests.append(dict(input=source, last=last, convention=convention, parameter=[.125, .875]))
    for request, answer in zip(requests, path_probe(requests), strict=True): audit_enclosure(request, answer)


def test_path_sixteen_constant_lifts_and_independent_roots(path_probe):
    for answer in path_probe():
        request = dict(input=unflatten(answer["point"]["input"]), centre=answer["point"]["centre"])
        assert answer["failure"] == 0 and answer["admitted"]
        audit_enclosure(request, answer)
        audit_cell(request, answer, parameters=(F(1, 2),))


@pytest.mark.parametrize("profile", [0, 1])
def test_path_small_motion_uniform_bounds_and_roots(path_probe, profile):
    with mp.workdps(180): request = generated(20)
    request["last"] = copy.deepcopy(request["input"]["target"])
    request["last"]["position"][0] += 1e-12
    request["profile"] = profile
    answer = path_probe([request])[0]
    assert answer["failure"] == 0 and answer["admitted"]
    audit_enclosure(request, answer)
    audit_cell(request, answer)


def test_path_denominator_and_log_domains_remain_inconclusive(path_probe):
    with mp.workdps(180): request = generated(20)
    zero = copy.deepcopy(request)
    zero["last"] = copy.deepcopy(zero["input"]["target"])
    zero["last"]["quaternion"] = [-v for v in zero["last"]["quaternion"]]
    broad = copy.deepcopy(request)
    broad["last"] = copy.deepcopy(broad["input"]["target"])
    broad["last"]["position"][0] += .1
    rows = path_probe([zero, broad])
    assert rows[0]["enclosure_failure"] == 4 and not rows[0]["local"]
    assert rows[1]["failure"] != 0 and not rows[1]["admitted"]


def test_half_turn_target_requires_a_new_local_log_anchor(path_probe):
    with mp.workdps(180):
        request = generated(20)
    request["last"] = copy.deepcopy(request["input"]["target"])
    w, x, y, z = request["last"]["quaternion"]
    request["last"]["quaternion"] = [-x, w, -z, y]
    assert sum(F(a)*F(b) for a, b in zip(request["input"]["target"]["quaternion"],
                                       request["last"]["quaternion"], strict=True)) == 0
    # Exact orthogonal homogeneous quaternions differ by a half turn. The
    # endpoint denominator is positive, but this original anchored-log region
    # cannot certify that target. This says nothing about other path anchors.
    request["parameter"] = [1., 1.]
    answer = path_probe([request])[0]
    assert answer["enclosure_failure"] == 0 and not answer["local"] and not answer["admitted"]
    assert answer["failure"] == 7 and answer["point_failure"] == 7


def test_path_scaled_inverse_retains_neumann_factor(path_probe):
    with mp.workdps(180): request = generated(20)
    initial = path_probe([request])[0]
    assert initial["failure"] == 0
    request["centre"] = [x+1e-10 for x in request["centre"]]
    request["inverse"] = [.1*x for x in initial["point"]["preconditioner"]]
    request["last"] = copy.deepcopy(request["input"]["target"])
    request["last"]["position"][0] += 1e-12
    answer = path_probe([request])[0]
    assert answer["failure"] == 0 and answer["point"]["constants"][2] > .8
    audit_enclosure(request, answer)
    audit_cell(request, answer)


def test_path_real_turns_and_raw_limit_slacks(path_probe):
    with mp.workdps(180): request = generated(20)
    request["centre"][0] += 2*math.pi
    request["centre"][3] -= 2*math.pi
    broad = path_probe([request])[0]
    assert broad["failure"] == 0 and broad["admitted"]
    audit_cell(request, broad, parameters=(F(1, 2),))
    narrow = copy.deepcopy(request)
    narrow.update(lower=[-180.]*6, upper=[180.]*6)
    rejected = path_probe([narrow])[0]
    assert rejected["failure"] == 9 and rejected["local"] and not rejected["admitted"]
    audit_cell(narrow, rejected, parameters=(F(1, 2),))


def test_legal_endpoints_do_not_admit_an_interior_joint_limit_violation(path_probe):
    fixture = json.loads((Path(__file__).parent/"fixtures/path_limit_interior.json").read_text(encoding="utf-8"))
    requests = [dict(input=fixture["input"], last=fixture["last"], point=point,
                     centre=centre, parameter=[s, s], lower=fixture["lower"], upper=fixture["upper"])
                for s, point, centre in zip((0., .5, 1.), fixture["point_targets"],
                                            fixture["point_roots"], strict=True)]
    rows = path_probe(requests)
    assert rows[0]["admitted"] and rows[2]["admitted"]
    assert rows[1]["local"] and not rows[1]["admitted"] and rows[1]["failure"] == 9
    for request, row in zip(requests, rows, strict=True):
        audit_enclosure(request, row)
        audit_cell(request, row, parameters=(F(request["parameter"][0]),))
    # The ENTIRE middle root enclosure violates this actual degree limit;
    # this is stronger than a numerical midpoint evaluation or failed slack.
    with mp.workdps(180):
        axis = fixture["axis"]
        centre = mp.mpf(rows[1]["point"]["centre"][axis])-mp.pi/2
        rho = mp.mpf(rows[1]["rho"])
        lower, upper = [mp.mpf(fixture[key][axis])*mp.pi/180 for key in ("lower", "upper")]
        assert centre+rho < lower or centre-rho > upper


@pytest.mark.parametrize("wrist", [1e-2, 1e-4, 1e-6, 0.])
def test_path_singularity_approach_never_promotes_failed_hypotheses(path_probe, wrist):
    # With shoulder q1=0 the exact nominal determinant contains sin(q4).
    # Setting only q4=0 is not a singularity for every offset-wrist geometry.
    with mp.workdps(180): request = generated(20, wrist=wrist, shoulder=0.)
    answer = path_probe([request])[0]
    if answer["local"]:
        audit_cell(request, answer, parameters=(F(1, 2),))
    else:
        assert answer["failure"] != 0 and not answer["admitted"]
    if wrist == 0: assert not answer["local"]


@pytest.fixture(scope="module")
def tracking_reports():
    configured = os.environ.get("CRXKIN_PATH_TRACKING_PROBE_PATH")
    path = Path(configured) if configured else Path(__file__).resolve().parents[1]/"build/Release/crx_path_tracking_tests.exe"
    if configured: assert path.is_file(), path
    elif not path.is_file(): pytest.skip("Build crx_path_tracking_tests or set CRXKIN_PATH_TRACKING_PROBE_PATH")
    result = subprocess.run([str(path), "--witness"], text=True, capture_output=True, check=True, timeout=90)
    reports = [json.loads(line) for line in result.stdout.splitlines()]
    assert len(reports) == 3
    return reports


def trace_request(trace):
    values = [struct.unpack("<d", struct.pack("<Q", word))[0] for word in trace["path_words"]]
    assert len(values) == 93
    source = unflatten(values[:77])
    last = dict(quaternion=values[77:81], position=values[81:84], input_rotation=values[84:93])
    bits = lambda values: b"".join(struct.pack("<d", value) for value in values)
    assert bits(flatten(source)) == bits(trace["first"]["input"])
    assert trace["first"]["input_words"] == trace["path_words"][:77]
    expected_end = copy.deepcopy(source)
    expected_end["target"] = last
    assert bits(flatten(expected_end)) == bits(trace["last"]["input"])
    return dict(input=source, last=last, convention=trace["convention"], profile=trace["profile"],
                lower=trace["lower"], upper=trace["upper"])


def inclusion(first, rho, second):
    return max(abs(F(a)-F(b)) for a, b in zip(first, second, strict=True))+F(rho)


def test_adaptive_path_full_interval_proofs_and_root_membership(tracking_reports):
    assert tracking_reports[0]["complete"] and tracking_reports[1]["complete"]
    assert len(tracking_reports[1]["leaves"]) > 1 and tracking_reports[1]["work"]["subdivisions"] > 0
    for trace in tracking_reports[:2]:
        request = trace_request(trace)
        leaves = trace["leaves"]
        before = F(0)
        for index, leaf in enumerate(leaves):
            lo, hi = map(F, leaf["parameter"])
            assert lo == before and hi > lo and leaf["accepted"]
            before = hi
            local = {**request, "parameter": leaf["parameter"]}
            cell = {**leaf["cell"], "preparation_failure": 0, "enclosure_failure": 0}
            point_bits = [struct.unpack("<Q", struct.pack("<d", value))[0] for value in cell["point"]["input"]]
            assert point_bits == cell["point"]["input_words"]
            audit_enclosure(local, cell)
            # Independent rational Taylor/interval arithmetic checks the entire
            # target family, Jacobian inverse, log-series tail and all slacks.
            audit_uniform(local, cell)
            if index in {0, len(leaves)//2, len(leaves)-1}:
                audit_cell(local, cell, parameters=(lo, (lo+hi)/2, hi))
        assert before == 1 and trace["uncovered_measure"] == [0, 0]
        assert len(trace["links"])+1 == len(leaves)
        for link in trace["links"]:
            assert link["certified"] and link["second"] == link["first"]+1
            first, second = (leaves[link[key]]["cell"] for key in ["first", "second"])
            exact = inclusion(first["point"]["centre"], first["rho"], second["point"]["centre"])
            assert exact <= F(link["inclusion"][1]) <= F(second["point"]["constants"][1])
        first, last = leaves[0]["cell"], leaves[-1]["cell"]
        initial = inclusion(trace["first"]["centre"], trace["first"]["constants"][0], first["point"]["centre"])
        final = inclusion(last["point"]["centre"], last["rho"], trace["last"]["centre"])
        assert initial <= F(trace["initial_inclusion"][1]) <= F(first["point"]["constants"][1])
        assert final <= F(trace["final_inclusion"][1]) <= F(trace["last"]["constants"][1])


def test_adaptive_budget_retains_all_uncovered_parameters(tracking_reports):
    trace = tracking_reports[2]
    assert not trace["complete"] and trace["failure"] != 0
    assert trace["work"]["nodes"] <= trace["options"]["max_nodes"]
    assert trace["work"]["point_certificates"] <= trace["options"]["max_point_certificates"]
    before, uncovered = F(0), F(0)
    for leaf in trace["leaves"]:
        lo, hi = map(F, leaf["parameter"])
        assert lo == before and hi > lo
        before = hi
        if not leaf["accepted"]: uncovered += hi-lo
    assert before == 1 and uncovered > 0
    assert F(trace["uncovered_measure"][0]) <= uncovered <= F(trace["uncovered_measure"][1])
