# SPDX-License-Identifier: Apache-2.0
# SPDX-FileCopyrightText: 2026 Centre Technologique en Aérospatiale
"""Audit command boxes, full turn counts and ranking against independent roots."""

import itertools
import json
import math
import os
from pathlib import Path
import struct
import subprocess
import sys

import mpmath as mp
import pytest

import crx_catalogue_reference as oracle
from test_crx_exact_target import contains, flatten
from test_crx_root_certificate import witness


@pytest.fixture(scope="module")
def command_certificate_probe():
    configured = os.environ.get("CRXKIN_COMMAND_CERTIFICATE_PROBE_PATH")
    suffix = ".exe" if sys.platform == "win32" else ""
    path = Path(configured) if configured else Path(__file__).resolve().parents[1]/f"build/Release/crx_command_certificate_tests{suffix}"
    if configured: assert path.is_file(), f"Configured command certificate probe missing: {path}"
    elif not path.is_file(): pytest.skip("Build crx_command_certificate_tests or set CRXKIN_COMMAND_CERTIFICATE_PROBE_PATH")

    def run(requests):
        lines = []
        for request in requests:
            values = [request.get("profile", 0), int(request.get("seeded", True)),
                      2 if request.get("reference") else 1, request.get("budget", 1 << 20),
                      request.get("capacity", 512), request.get("candidate_budget", 512)]
            for key, default in [("lower", [-180]*6), ("upper", [180]*6),
                                 ("seed", [0]*6), ("weights", [1]*6)]:
                values += request.get(key, default)
            values += flatten(request["input"])
            lines.append(" ".join(str(value) if i < 6 else repr(float(value)) for i, value in enumerate(values)))
        result = subprocess.run([str(path), "--batch"], input="\n".join(lines)+"\n",
                                text=True, capture_output=True, check=True, timeout=180)
        answers = [json.loads(line) for line in result.stdout.splitlines()]
        assert len(answers) == len(requests)
        return answers
    return run


def commands(input, q, profile):
    reference_senses = [1, 1, -1, -1, -1, -1]
    offset = [(mp.mpf(z)+(mp.pi/2 if i == 1 else 0))*reference_senses[i]
              for i, z in enumerate(input["encoder_zero"])]
    values = [input["senses"][i]*reference_senses[i]*(mp.mpf(q[i])-offset[i]) for i in range(6)]
    if profile == 1: values[2] -= values[1]
    return values


def audit(answer, request, reference_result=None):
    assert answer["failure"] == 0 and answer["phase_complete"] and answer["enumeration_complete"]
    assert answer["yes"]+answer["no"]+answer["undecided"] == answer["box_lifts"]
    assert answer["admission_complete"] == (answer["undecided"] == 0)
    assert answer["infeasible"] == (answer["admission_complete"] and answer["yes"] == 0)
    assert answer["unique"] == (answer["ranking"] == 2)
    assert len(answer["lifts"]) == answer["yes"]+answer["undecided"]
    assert answer["returned_count"] == min(request.get("capacity", 512), len(answer["lifts"]))
    assert answer["truncated"] == (answer["returned_count"] < len(answer["lifts"]))
    lower = request.get("lower", [-180]*6)
    upper = request.get("upper", [180]*6)
    seed = request.get("seed", [0]*6)
    weights = request.get("weights", [1]*6)
    words = [format(struct.unpack("<Q", struct.pack("<d", float(value)))[0], "x")
             for values in [lower, upper, seed, weights] for value in values]
    assert answer["words"] == words
    profile = request.get("profile", 0)
    with mp.workdps(200):
        lower, upper, seed = [[mp.mpf(x)*mp.pi/180 for x in values] for values in [lower, upper, seed]]
        for key, exact in [("lower", lower), ("upper", upper), ("seed", seed)]:
            for enclosure, value in zip(answer[key], exact, strict=True): contains(enclosure, value)
        base_roots = []
        if reference_result is None: reference_result = oracle.exhaustive(request["input"], request.get("reference", False))
        assert reference_result.outcome == "finite", reference_result
        for phase, ball in enumerate(answer["balls"]):
            rho = mp.mpf(ball["rho"])
            matches = [q for q in reference_result.configurations if max(
                oracle.circle_distance(mp.mpf(x), mp.mpf(y)) for x, y in zip(q, ball["centre"], strict=True)) <= rho]
            assert len(matches) == 1
            q = [x+2*mp.pi*mp.nint((mp.mpf(c)-x)/(2*mp.pi))
                 for x, c in zip(matches[0], ball["centre"], strict=True)]
            base_roots.append(commands(request["input"], q, profile))
            centre = commands(request["input"], ball["centre"], profile)
            phase_range = answer["ranges"][phase]
            assert phase_range["available"]
            for i in range(6):
                radius = rho*(2 if profile == 1 and i == 2 else 1)
                contains(phase_range["base"][i], centre[i]-radius)
                contains(phase_range["base"][i], centre[i]+radius)
            assert phase_range["count"] == (0 if phase_range["empty"] else math.prod(
                last-first+1 for first, last in zip(phase_range["first"], phase_range["last"], strict=True)))
        potential = {(row["phase"], tuple(row["turns"])): (i, row)
                     for i, row in enumerate(answer["lifts"])}
        assert len(potential) == len(answer["lifts"])
        feasible = {}
        for phase, root in enumerate(base_roots):
            turn_sets = [range(int(mp.ceil((lo-x)/(2*mp.pi))), int(mp.floor((hi-x)/(2*mp.pi)))+1)
                         for lo, hi, x in zip(lower, upper, root, strict=True)]
            for turns in itertools.product(*turn_sets):
                key = (phase, turns)
                assert key in potential, "A feasible exact lift was lost"
                feasible[key] = sum(mp.mpf(w)*(x+2*mp.pi*m-s)**2
                    for w, x, m, s in zip(weights, root, turns, seed, strict=True))
        if answer["admission_complete"]: assert len(feasible) == answer["yes"]
        for key, (_, row) in potential.items():
            phase, turns = key
            exact = [x+2*mp.pi*m for x, m in zip(base_roots[phase], turns, strict=True)]
            for enclosure, value in zip(row["radians"], exact, strict=True): contains(enclosure, value)
            if row["admission"] == 1: assert key in feasible
            if request.get("seeded", True):
                cost = sum(mp.mpf(w)*(x-s)**2 for w, x, s in zip(weights, exact, seed, strict=True))
                if row["cost"] is not None: contains(row["cost"], cost)
            else: assert row["cost"] is None
        if answer["ranking"] in [2, 3]:
            assert feasible
            minimum = min(feasible.values())
            winners = [potential[key][0] for key, cost in feasible.items() if cost == minimum]
            assert set(winners) <= set(answer["minimizers"])
        if answer["unique"]:
            assert answer["minimizers"] == [0]
            winner = answer["lifts"][0]
            assert winner["admission"] == 1
            assert all(winner["cost"][1] < row["cost"][0] for row in answer["lifts"][1:])
        if answer["ranking"] == 4: assert not feasible
    return reference_result


def test_complete_limits_weights_and_prefix(command_certificate_probe):
    requests = [{"input": witness(), "capacity": cap, "weights": [1, 2, .25, 4, .5, 3]}
                for cap in [0, 1, 64, 512]]
    answers = command_certificate_probe(requests)
    reference = None
    for answer, request in zip(answers, requests, strict=True): reference = audit(answer, request, reference)
    assert all(a["unique"] and a["yes"] == 16 for a in answers)
    assert all(a["lifts"] == answers[0]["lifts"] for a in answers)


@pytest.mark.parametrize("profile", [0, 1])
def test_raw_offsets_senses_and_multiturn(command_certificate_probe, profile):
    input = witness()
    input["encoder_zero"] = [.13, -7.25, 13.5, -.007, 2**20+.25, -123.125]
    input["senses"] = [-1, 1, -1, 1, -1, 1]
    # Canonical target is independent of command-coordinate zeros and senses.
    with mp.workdps(200):
        centre = [float(x*180/mp.pi) for x in commands(input, [0]*6, profile)]
    request = {"input": input, "profile": profile,
               "lower": [x-360 for x in centre], "upper": [x+360 for x in centre],
               "seed": [x+.125 for x in centre], "weights": [.125, 8, 2, 1, 4, .5]}
    answer, = command_certificate_probe([request])
    audit(answer, request)
    assert answer["yes"] == 1024 and answer["unique"]


def test_boundary_uncertainty_and_possible_minimizers(command_certificate_probe):
    request = {"input": witness()}
    initial, = command_certificate_probe([request])
    with mp.workdps(200):
        centre = [float(x*180/mp.pi) for x in commands(request["input"], initial["balls"][0]["centre"], 0)]
    boundary = dict(request, lower=[x-.01 for x in centre], upper=[x+.01 for x in centre], seed=centre)
    boundary["lower"][0] = centre[0]
    boundary["upper"][0] = centre[0]
    tie = dict(request, lower=[x-.01 for x in centre], upper=[x+.01 for x in centre], seed=centre.copy())
    tie["lower"][0] = centre[0]-1
    tie["upper"][0] = centre[0]+361
    tie["seed"][0] += 180
    uncertain, tied = command_certificate_probe([boundary, tie])
    audit(uncertain, boundary)
    audit(tied, tie)
    assert uncertain["undecided"] > 0 and not uncertain["admission_complete"]
    assert not uncertain["infeasible"] and not uncertain["unique"]
    assert tied["ranking"] == 3 and len(tied["minimizers"]) == 2 and tied["yes"] == 2


def test_command_infeasibility_and_no_seed(command_certificate_probe):
    requests = [{"input": witness(), "lower": [0]*6, "upper": [0]*6},
                {"input": witness(), "seeded": False}]
    excluded, unranked = command_certificate_probe(requests)
    for answer, request in zip([excluded, unranked], requests, strict=True): audit(answer, request)
    assert excluded["infeasible"] and excluded["ranking"] == 4 and excluded["balls"]
    assert unranked["ranking"] == 0 and not unranked["minimizers"]


@pytest.mark.parametrize("update, failure", [({"budget": 0}, 6),
    ({"candidate_budget": 0}, 4), ({"lower": [1e20]*6, "upper": [1e20]*6}, 5),
    ({"lower": [-1e8]*6, "upper": [1e8]*6}, 5)])
def test_unavailable_competitors_never_become_global_claims(command_certificate_probe, update, failure):
    answer, = command_certificate_probe([dict({"input": witness()}, **update)])
    assert answer["failure"] == failure
    assert not answer["enumeration_complete"] and not answer["admission_complete"]
    assert not answer["unique"] and not answer["infeasible"] and not answer["minimizers"]


@pytest.mark.parametrize("update", [{"weights": [0]*6}, {"weights": [-1]*6},
    {"profile": 7}, {"lower": [181]*6, "upper": [180]*6}])
def test_invalid_context(command_certificate_probe, update):
    answer, = command_certificate_probe([dict({"input": witness()}, **update)])
    assert answer == {"prepare_failure": 1}


def test_cost_overflow_preserves_admission_without_nearest_claim(command_certificate_probe):
    request = {"input": witness(), "seed": [1e308]*6}
    answer, = command_certificate_probe([request])
    audit(answer, request)
    assert answer["admission_complete"] and answer["yes"] == 16
    assert answer["ranking"] == 1 and not answer["unique"] and not answer["minimizers"]


def test_raw_signed_zero_provenance(command_certificate_probe):
    request = {"input": witness(), "seed": [-0.0, 0, -0.0, 0, 0, 0]}
    answer, = command_certificate_probe([request])
    audit(answer, request)
    assert answer["words"][12] == answer["words"][14] == "8000000000000000"


def test_far_boundary_uncertainty_does_not_defeat_strict_nearest(command_certificate_probe):
    request = {"input": witness()}
    initial, = command_certificate_probe([request])
    winner = initial["lifts"][0]
    with mp.workdps(200):
        centre = [float((mp.mpf(lo)+hi)*90/mp.pi) for lo, hi in winner["radians"]]
    request.update(lower=[x-.01 for x in centre], upper=[x+.01 for x in centre], seed=centre)
    # The same phase's next turn touches the upper limit. Its status is unknown,
    # but even its lowest possible travel is much larger than the inside root's.
    request["upper"][0] = centre[0]+360
    answer, = command_certificate_probe([request])
    audit(answer, request)
    assert answer["yes"] == 1 and answer["undecided"] == 1
    assert not answer["admission_complete"] and answer["unique"]


def test_zero_root_pose_exclusion_survives_zero_command_work_budget(command_certificate_probe):
    input = witness()
    input["target"]["position"] = [20, 30, -40]
    request = {"input": input, "budget": 0}
    answer, = command_certificate_probe([request])
    audit(answer, request)
    assert answer["infeasible"] and not answer["balls"] and answer["box_lifts"] == 0


def test_huge_encoder_zero_is_unresolved_without_rounded_reduction(command_certificate_probe):
    input = witness()
    input["encoder_zero"][0] = 1e100
    answer, = command_certificate_probe([{"input": input}])
    assert answer["failure"] == 5 and answer["phase_complete"]
    assert not answer["enumeration_complete"] and not answer["infeasible"] and not answer["unique"]
