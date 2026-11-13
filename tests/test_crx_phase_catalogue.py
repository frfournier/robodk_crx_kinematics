# SPDX-License-Identifier: Apache-2.0
# SPDX-FileCopyrightText: 2026 Centre Technologique en Aérospatiale
"""Independent full reconstruction, phase separation and completeness checks."""

import copy
import json
import math
import os
from pathlib import Path
import random
import struct
import subprocess
import sys

import mpmath as mp
import pytest

import crx_catalogue_reference as oracle
import crx_exact_reference as ref
from test_crx_ball_certificate import check_certificate, exact_kinematics, generated
from test_crx_exact_target import contains, flatten, pose
from test_crx_root_certificate import witness


@pytest.fixture(scope="module")
def catalogue_probe():
    configured = os.environ.get("CRXKIN_CATALOGUE_PROBE_PATH")
    suffix = ".exe" if sys.platform == "win32" else ""
    path = Path(configured) if configured else Path(__file__).resolve().parents[1]/f"build/Release/crx_phase_catalogue_tests{suffix}"
    if configured: assert path.is_file(), f"Configured phase catalogue probe missing: {path}"
    elif not path.is_file(): pytest.skip("Build crx_phase_catalogue_tests or set CRXKIN_CATALOGUE_PROBE_PATH")

    def run(requests, distance=False):
        lines = []
        for request in requests:
            if distance: values = request
            else:
                options = [request.get(key, default) for key, default in [
                    ("max_nodes", 8192), ("max_depth", 40), ("max_leaves", 4096),
                    ("max_chart_attempts", 8), ("max_candidate_certificates", 512), ("max_pair_tests", 120)]]
                values = [2 if request.get("reference") else 1, *options, *flatten(request["input"])]
            lines.append(" ".join(str(value) if not distance and i < 7 else repr(float(value)) for i, value in enumerate(values)))
        result = subprocess.run([str(path), "--distance" if distance else "--batch"],
            input="\n".join(lines)+"\n", text=True, capture_output=True, check=True, timeout=180)
        answers = [json.loads(line) for line in result.stdout.splitlines()]
        assert len(answers) == len(requests)
        return answers
    return run


def check_catalogue(answer, input, reference=False, proof_constants=True):
    words = [format(struct.unpack('<Q', struct.pack('<d', value))[0], 'x') for value in flatten(input)]
    assert answer["words"] == words and answer["schema"] == (2 if reference else 1)
    assert answer["complete"] == (answer["coverage"] == 1) == (answer["failure"] == 0)
    assert answer["infeasible"] == (answer["complete"] and len(answer["balls"]) == 0)
    assert answer["attempted_balls"] == len(answer["ball_failures"])
    assert answer["certified_balls"] == len(answer["balls"]) == answer["ball_failures"].count(0)
    assert answer["attempted_balls"] <= answer["nominations"]
    if answer["complete"]:
        assert answer["root_complete"] and answer["certified_balls"] == 2*answer["roots"]
        assert answer["expected"] == len(answer["balls"])
        assert not answer["unresolved"]
    problem = oracle.prepare_reference(input) if reference else ref.prepare(input)
    for ball in answer["balls"]:
        if proof_constants:
            kinematics = exact_kinematics(input, ball["centre"], reference_mode=reference)
            check_certificate(ball, kinematics=kinematics)
            contains(ball["determinant"], mp.det(kinematics[3]))
    if answer["complete"]:
        reference_result = oracle.exhaustive(input, reference)
        assert reference_result.outcome == "finite", reference_result
        assert len(reference_result.configurations) == len(answer["balls"])
        matched = []
        for q in reference_result.configurations:
            candidates = [i for i, ball in enumerate(answer["balls"]) if max(
                oracle.circle_distance(x, mp.mpf(y)) for x, y in zip(q, ball["centre"], strict=True)) <= ball["constants"][0]]
            assert len(candidates) == 1
            matched += candidates
        assert sorted(matched) == list(range(len(answer["balls"])))
        for i, ball in enumerate(answer["balls"]):
            for other in answer["balls"][:i]:
                distance = max(oracle.circle_distance(mp.mpf(a), mp.mpf(b))
                               for a, b in zip(ball["centre"], other["centre"], strict=True))
                assert distance > mp.mpf(ball["constants"][0])+other["constants"][0]
    return problem


def test_independent_complete_sixteen_phase_catalogue(catalogue_probe):
    input = witness()
    answer = catalogue_probe([{"input": input}])[0]
    assert answer["complete"] and len(answer["balls"]) == 16 and answer["pair_tests"] == 120
    with mp.workdps(200): check_catalogue(answer, input)


def test_raw_command_limits_do_not_change_phase_coverage(catalogue_probe):
    input = witness()
    constrained = copy.deepcopy(input)
    constrained["lower_commands"] = constrained["upper_commands"] = [0.]*6
    baseline, changed = catalogue_probe([{"input": input}, {"input": constrained}])
    assert baseline["complete"] and changed["complete"]
    assert baseline["words"] != changed["words"]
    assert [ball["centre"] for ball in baseline["balls"]] == [ball["centre"] for ball in changed["balls"]]
    with mp.workdps(200): check_catalogue(changed, constrained, proof_constants=False)


def test_incomplete_root_or_candidate_or_pair_budget_never_certifies(catalogue_probe):
    input = witness()
    requests = [{"input": input, **budget} for budget in [
        {"max_nodes": 0}, {"max_candidate_certificates": 0}, {"max_pair_tests": 0}]]
    for request, answer in zip(requests, catalogue_probe(requests), strict=True):
        assert not answer["complete"] and not answer["infeasible"]
        assert answer["failure"] == (4 if request.get("max_nodes") == 0 else 6 if request.get("max_candidate_certificates") == 0 else 9)
        if request.get("max_nodes") == 0: assert answer["unresolved"]


def test_zero_root_exclusion_is_the_only_empty_complete_case(catalogue_probe):
    far = witness()
    far["target"]["position"] = [20., 30., -40.]
    answer = catalogue_probe([{"input": far, "max_candidate_certificates": 0, "max_pair_tests": 0}])[0]
    assert answer["complete"] and answer["infeasible"] and not answer["balls"] and answer["nominations"] == 0
    with mp.workdps(200): check_catalogue(answer, far)


def test_circle_distance_uses_mathematical_period_and_preserves_pi_seams(catalogue_probe):
    rng = random.Random(44)
    pairs = [(0., 0.), (math.pi, 0.), (-math.pi, 0.), (2*math.pi, 0.),
             (math.nextafter(math.pi, math.inf), 0.), (math.nextafter(math.pi, -math.inf), 0.)]
    pairs += [(rng.uniform(-1e5, 1e5), rng.uniform(-1e5, 1e5)) for _ in range(80)]
    pairs += [(math.ldexp(1., power), -math.ldexp(1., power)) for power in [10, 20, 30, 40, 100, 900]]
    with mp.workdps(200):
        for pair, answer in zip(pairs, catalogue_probe(pairs, distance=True), strict=True):
            exact = oracle.circle_distance(mp.mpf(pair[0]), mp.mpf(pair[1]))
            if answer is not None: contains(answer, exact)
            else: assert max(abs(x) for x in pair) > 1e12
    assert catalogue_probe([(math.pi, 0.)], distance=True)[0][0] > 3


@pytest.mark.parametrize("reference", [False, True])
def test_independent_signed_scaled_and_framed_catalogues(catalogue_probe, reference):
    inputs = [generated(20+i, scale=scale, signed=i % 2 == 1, frames=True)["input"]
              for i, scale in enumerate([.001, 1., 1000.])]
    with mp.workdps(200):
        for input, answer in zip(inputs, catalogue_probe([{"input": input, "reference": reference} for input in inputs]), strict=True):
            check_catalogue(answer, input, reference)


def test_reference_oracle_retains_unresolved_and_exceptional_outcomes():
    input = witness()
    exhausted = oracle.exhaustive(input, max_subdivisions=0)
    assert exhausted.outcome == "unresolved" and not exhausted.configurations
    # Exact zero residual/determinant polynomial; no finite catalogue claim.
    zero = witness()
    zero["lengths"] = [1., 1., 0., 0.]
    zero["target"]["quaternion"] = [1., 0., 0., 0.]
    zero["target"]["position"] = [0., 0., 0.]
    assert oracle.exhaustive(zero).outcome == "degenerate"


def test_full_reconstruction_includes_the_omitted_chart_phase(catalogue_probe):
    input = witness()
    input["lengths"] = [1., .5, .25, .125]
    input["target"] = pose([1., -1., 0., 0.], [1.125, .25, -.5])
    power, _, _ = ref.coefficients(ref.prepare(input), 0)
    assert power[-1] == 0 and power[-2] != 0
    answer = catalogue_probe([{"input": input}])[0]
    assert answer["complete"]
    with mp.workdps(200): check_catalogue(answer, input)


def test_reference_precision_escalation_retains_failed_attempts(monkeypatch):
    original = oracle.mp.findroot
    def fail_low_precision(*args, **kwargs):
        if mp.mp.dps < 160: raise ValueError("injected low-precision refinement failure")
        return original(*args, **kwargs)
    monkeypatch.setattr(oracle.mp, "findroot", fail_low_precision)
    answer = oracle.exhaustive(witness(), precisions=(80, 160))
    assert answer.outcome == "finite" and answer.precision == 160
    assert [item["outcome"] for item in answer.attempts] == ["unresolved", "finite"]
    assert len(answer.configurations) == 16
    def fail_all(*args, **kwargs): raise ValueError("injected unresolved refinement")
    monkeypatch.setattr(oracle.mp, "findroot", fail_all)
    unresolved = oracle.exhaustive(witness(), precisions=(80, 160))
    assert unresolved.outcome == "unresolved" and not unresolved.configurations
    assert [item["outcome"] for item in unresolved.attempts] == ["unresolved", "unresolved"]


def test_arithmetic_failure_cannot_be_mistaken_for_an_empty_exclusion_cover(catalogue_probe):
    input = witness()
    input["target"]["position"] = [1e100, -1e100, 1e100]
    answer = catalogue_probe([{"input": input}])[0]
    assert not answer["complete"] and not answer["infeasible"]
    assert answer["failure"] == 4 and answer["root_status"] == 4
    assert answer["entire_circle_unresolved"] and not answer["balls"]
