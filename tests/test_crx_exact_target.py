# SPDX-License-Identifier: Apache-2.0
# SPDX-FileCopyrightText: 2026 Centre Technologique en Aérospatiale
"""Exact rational frame/coefficient and high-precision FK enclosure checks."""

import copy
from fractions import Fraction as F
import itertools
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

import crx_exact_reference as ref


def pose(q=(1., 0., 0., 0.), p=(0., 0., 0.)):
    rotation = ref.quaternion_rotation(q)
    return {"quaternion": list(q), "position": list(p),
            "input_rotation": [float(x) for x in sum(rotation, [])]}


def case(seed=0, scale=1.):
    rng = random.Random(seed)
    quaternion = lambda: [rng.uniform(-1, 1) for _ in range(4)]
    position = lambda: [scale*rng.uniform(-1, 1) for _ in range(3)]
    return {"lengths": [1.25*scale, -.875*scale, .2*scale, .3*scale],
            "base_height": .35*scale, "encoder_zero": [rng.uniform(-8, 8) for _ in range(6)],
            "senses": [rng.choice([-1., 1.]) for _ in range(6)],
            "lower_commands": [-4.]*6, "upper_commands": [4.]*6,
            "target": pose(quaternion(), position()), "base": pose(quaternion(), position()),
            "tool": pose(quaternion(), position())}


def flatten(input):
    values = [*input["lengths"], input["base_height"], *input["encoder_zero"],
              *input["senses"], *input["lower_commands"], *input["upper_commands"]]
    for key in ["target", "base", "tool"]:
        for field in ["quaternion", "position", "input_rotation"]:
            values += input[key][field]
    assert len(values) == 77
    return values


def unflatten(values):
    iterator = iter(values)
    take = lambda n: [next(iterator) for _ in range(n)]
    result = {"lengths": take(4), "base_height": next(iterator), "encoder_zero": take(6),
              "senses": take(6), "lower_commands": take(6), "upper_commands": take(6)}
    for key in ["target", "base", "tool"]:
        result[key] = {"quaternion": take(4), "position": take(3), "input_rotation": take(9)}
    return result


@pytest.fixture(scope="module")
def exact_probe():
    configured = os.environ.get("CRXKIN_EXACT_PROBE_PATH")
    suffix = ".exe" if sys.platform == "win32" else ""
    path = Path(configured) if configured else Path(__file__).resolve().parents[1] / f"build/Release/crx_exact_tests{suffix}"
    if configured:
        assert path.is_file(), f"Configured exact probe missing: {path}"
    elif not path.is_file():
        pytest.skip("Build crx_exact_tests or set CRXKIN_EXACT_PROBE_PATH")

    def run(inputs, chart=(0., 0.), joints=None, command=False, adapter=False):
        if joints is None:
            joints = [(0., 0.)]*6
        requests = []
        for input in inputs:
            values = [*flatten(input), *chart, *itertools.chain.from_iterable(joints)]
            operation = "adapter" if adapter else "command" if command else "canonical"
            requests.append(" ".join([operation, *(repr(float(x)) for x in values)]))
        completed = subprocess.run([str(path), "--batch"], input="\n".join(requests)+"\n",
                                   text=True, capture_output=True, check=True, timeout=60)
        answers = [json.loads(line) for line in completed.stdout.splitlines()]
        assert len(answers) == len(inputs)
        return answers
    return run


def contains(interval, exact):
    assert interval is not None
    if isinstance(exact, F):
        assert F(interval[0]) <= exact <= F(interval[1]), (interval, exact)
    else:
        assert ref.number(F(interval[0])) <= exact <= ref.number(F(interval[1])), (interval, mp.nstr(exact, 40))


def check_preparation(input, answer):
    assert answer["status"] == 0
    expected = ref.prepare(input)
    assert F(answer["scale"]) == expected["scale"]
    for key in ["lengths", "translation", "center"]:
        for interval, value in zip(answer[key], expected[key], strict=True):
            contains(interval, value)
    for interval, value in zip(answer["rotation"], sum(expected["rotation"], []), strict=True):
        contains(interval, value)
    for interval, squared in zip(answer["deviation"], expected["deviations_squared"], strict=True):
        lower, upper = map(F, interval)
        assert 0 <= lower and lower*lower <= squared <= upper*upper
    words = [format(struct.unpack("<Q", struct.pack("<d", float(x)))[0], "x") for x in flatten(input)]
    assert answer["identity"] == words
    return expected


@pytest.mark.parametrize("scale", [1e-100, .001, 1., 1000., 1e100])
def test_exact_rational_quaternions_frames_and_normalization(exact_probe, scale):
    inputs = [case(seed, scale) for seed in range(24)]
    answers = exact_probe(inputs)
    for input, answer in zip(inputs, answers, strict=True):
        check_preparation(input, answer)


def test_quaternion_sign_scale_and_extreme_components(exact_probe):
    inputs = []
    for exponent, sign in itertools.product([-1000, -500, 0, 500, 1000], [-1, 1]):
        input = case(42)
        input["target"] = pose([math.ldexp(sign*x, exponent) for x in [1., -2., 3., 4.]])
        inputs.append(input)
    for quaternion in [[math.ulp(0.), 0., 0., 0.], [sys.float_info.max, math.ulp(0.), 0., -1.],
                       [math.ulp(0.)]*4, [sys.float_info.max]*4]:
        input = case(42)
        input["target"] = pose(quaternion)
        inputs.append(input)
    answers = exact_probe(inputs)
    rotations = []
    for input, answer in zip(inputs, answers, strict=True):
        rotations.append(check_preparation(input, answer)["rotation"])
    assert all(rotation == rotations[0] for rotation in rotations[:10])
    assert len({tuple(x["identity"]) for x in answers}) == len(inputs)


def test_overflow_avoiding_and_underflowing_normalization(exact_probe):
    large, tiny = sys.float_info.max, math.ulp(0.)
    inputs = []
    for lengths, first, second in [([large, large, 0., 0.], large, -large),
                                   ([1e-300, .8e-300, 0., 0.], large, large),
                                   ([tiny, large, 0., 0.], 0., 0.)]:
        input = case(0)
        input.update(lengths=lengths, base_height=0., target=pose(p=[first]*3),
                     base=pose(p=[second]*3), tool=pose())
        inputs.append(input)
    for input, answer in zip(inputs, exact_probe(inputs), strict=True):
        check_preparation(input, answer)


def test_signed_asset_ratios_across_scales(exact_probe):
    from crx_asset_reference import derive_model
    assets = json.loads((Path(__file__).parent / "fixtures/crx_asset_frames.json").read_text())["assets"]
    inputs = []
    for asset, signs, scale in itertools.product(assets, itertools.product([-1, 1], repeat=4), [.001, 1., 1000.]):
        dh = derive_model(asset)["dh"]
        lengths = [dh[2, 1], dh[3, 3], dh[5, 3], -dh[4, 3]]
        input = case(37, scale)
        input["lengths"] = [float(x*s*scale) for x, s in zip(lengths, signs, strict=True)]
        inputs.append(input)
    for input, answer in zip(inputs, exact_probe(inputs), strict=True):
        problem = check_preparation(input, answer)
        assert answer["polynomial_status"] == 0
        for key, values in zip(["residual", "derivative", "determinant"], ref.coefficients(problem, F(0)), strict=True):
            for interval, value in zip(answer[key], values, strict=True):
                contains(interval, value)


def test_existing_model_adapter_retains_supplied_target_and_raw_fields(exact_probe):
    inputs = [case(seed+140) for seed in range(20)]
    answers = exact_probe(inputs, adapter=True)
    for input, answer in zip(inputs, answers, strict=True):
        effective_values = [struct.unpack("<d", struct.pack("<Q", int(word, 16)))[0] for word in answer["identity"]]
        effective = unflatten(effective_values)
        # Base/tool quaternion extraction is an explicit floating input choice;
        # every other raw field, including the supplied target q, must survive.
        for i, (before, after) in enumerate(zip(flatten(input), effective_values, strict=True)):
            if i not in [*range(45, 49), *range(61, 65)]:
                assert struct.pack("<d", before) == struct.pack("<d", after)
        check_preparation(effective, answer)
        assert max(x[1] for x in answer["deviation"]) < 1e-12


def test_every_raw_identity_word_is_retained(exact_probe):
    values = flatten(case(7))
    inputs = []
    for index in range(len(values)):
        changed = values.copy()
        changed[index] = -changed[index] if 11 <= index < 17 else math.nextafter(changed[index], math.inf)
        inputs.append(unflatten(changed))
    answers = exact_probe(inputs)
    for input, answer in zip(inputs, answers, strict=True):
        check_preparation(input, answer)
    assert len({tuple(x["identity"]) for x in answers}) == len(inputs)


@pytest.mark.parametrize("chart", [0., .3, math.pi/2, -math.pi/4])
def test_all_direct_polynomial_coefficients_and_omitted_phase(exact_probe, chart):
    inputs = [case(seed) for seed in range(16)]
    answers = exact_probe(inputs, chart=(chart, chart))
    with mp.workdps(200):
        for input, answer in zip(inputs, answers, strict=True):
            problem = check_preparation(input, answer)
            assert answer["polynomial_status"] == 0
            expected = ref.coefficients(problem, F(chart))
            for name, coefficients in zip(["residual", "derivative", "determinant"], expected, strict=True):
                assert len(answer[name]) == len(coefficients)
                for interval, value in zip(answer[name], coefficients, strict=True):
                    contains(interval, value)
            axis = [-ref.number(v)*mp.cos(chart)-ref.number(w)*mp.sin(chart)
                    for v, w in zip(problem["v"], problem["w"], strict=True)]
            for interval, value in zip(answer["omitted"], ref.geometric(problem, axis=axis), strict=True):
                contains(interval, value)


def evaluate_box(coefficients, value):
    lo, hi = F(0), F(0)
    for a, b in reversed(coefficients):
        products = [lo*value, hi*value]
        lo, hi = min(products)+F(a), max(products)+F(b)
    return lo, hi


def test_polynomial_enclosures_at_independent_phases(exact_probe):
    chart = .7
    inputs = [case(seed+100) for seed in range(24)]
    answers = exact_probe(inputs, chart=(chart, chart))
    with mp.workdps(200):
        for input, answer in zip(inputs, answers, strict=True):
            problem = check_preparation(input, answer)
            for t in map(F, [-3., -.5, 0., .25, 2.]):
                residual, determinant = ref.geometric(problem, phase=ref.number(F(chart))+2*mp.atan(ref.number(t)))
                q = 1+t*t
                f_box = evaluate_box(answer["residual"], t)
                d_box = evaluate_box(answer["determinant"], t)
                contains([f_box[0]/q**4, f_box[1]/q**4], residual)
                contains([d_box[0]/q**2, d_box[1]/q**2], determinant)


def test_chart_intervals_and_degree_drops(exact_probe):
    input = case(23)
    answer = exact_probe([input], chart=(.2, .201))[0]
    problem = check_preparation(input, answer)
    with mp.workdps(200):
        for chart in map(F, [.2, .2005, .201]):
            for key, values in zip(["residual", "derivative", "determinant"], ref.coefficients(problem, chart), strict=True):
                for interval, value in zip(answer[key], values, strict=True):
                    contains(interval, value)
    special = case(0)
    special.update(lengths=[1., 1., 0., 0.], base_height=0., target=pose(), base=pose(), tool=pose())
    answer = exact_probe([special])[0]
    assert answer["polynomial_status"] == 0
    assert len(answer["residual"]) == 9 and len(answer["derivative"]) == 8 and len(answer["determinant"]) == 5
    assert all(x == [0., 0.] for key in ["residual", "derivative", "determinant"] for x in answer[key])


def test_uncertain_leading_coefficients_are_never_trimmed(exact_probe):
    # f(theta)=r^6*(1+cos(theta))^4, so P(t)=16*r^6 is constant.
    # The omitted point remains a root of f, and Delta vanishes identically;
    # no infeasibility or continuous-family conclusion follows from P alone.
    input = case(0)
    input.update(lengths=[1., 1., 0., .25], base_height=0., target=pose(p=[.25, 0., 0.]),
                 base=pose(), tool=pose())
    answer = exact_probe([input])[0]
    assert answer["polynomial_status"] == 0 and len(answer["residual"]) == 9
    contains(answer["residual"][0], F(1, 256))
    for coefficient in answer["residual"][1:]:
        contains(coefficient, F(0))
    assert answer["residual"][-1][0] < answer["residual"][-1][1]


@pytest.mark.parametrize("command", [False, True])
def test_shared_fk_encloses_point_and_box_inputs(exact_probe, command):
    inputs = [case(seed+80) for seed in range(20)]
    joints = [(.15*i-1e-9, .15*i+1e-9) for i in range(6)]
    answers = exact_probe(inputs, joints=joints, command=command)
    rng = random.Random(20261113)
    with mp.workdps(200):
        for input, answer in zip(inputs, answers, strict=True):
            problem = check_preparation(input, answer)
            assert answer["fk_status"] == 0
            for _ in range(5):
                q = [F(rng.uniform(lo, hi)) for lo, hi in joints]
                q = ref.command_angles(input, q) if command else list(map(ref.number, q))
                rotation, translation = ref.forward(problem["lengths"], q)
                for interval, exact in zip(answer["fk_rotation"], rotation, strict=True):
                    contains(interval, exact)
                for interval, exact in zip(answer["fk_translation"], translation, strict=True):
                    contains(interval, exact)


def test_raw_encoder_offsets_seams_and_extreme_arguments(exact_probe):
    inputs = []
    for raw in [math.pi, math.nextafter(math.pi, math.inf), -math.pi, 7., 1e20, -1e20, math.ulp(0.)]:
        input = case(32)
        input["encoder_zero"] = [raw]*6
        inputs.append(input)
    answers = exact_probe(inputs, command=True)
    with mp.workdps(200):
        for input, answer in zip(inputs, answers, strict=True):
            check_preparation(input, answer)
            offsets = ref.command_angles(input, [F(0)]*6)
            for offset, principal, seam, value in zip(answer["offset"], answer["principal"], answer["seams"], offsets, strict=True):
                contains(offset, value)
                multiple = mp.floor((value+mp.pi)/(2*mp.pi))
                reduced = value if multiple == 0 else value-multiple*2*mp.pi
                contains(principal, reduced)
                if seam:
                    assert ref.number(F(principal[0])) <= -mp.pi and ref.number(F(principal[1])) >= mp.pi
    assert all(answers[4]["seams"]) and all(answers[5]["seams"])


def test_invalid_inputs_and_arithmetic_failure_are_not_success(exact_probe):
    inputs = []
    for quaternion in [[0.]*4, [math.nan, 0., 0., 0.], [math.inf, 0., 0., 0.]]:
        input = case()
        input["target"]["quaternion"] = quaternion
        inputs.append(input)
    for field in ["lengths", "encoder_zero", "senses", "lower_commands"]:
        input = case()
        input[field][0] = math.nan
        inputs.append(input)
    input = case()
    input["lengths"][0] = 0.
    inputs.append(input)
    for answer in exact_probe(inputs):
        assert answer["status"] != 0 and "identity" not in answer
    input = case()
    input["lengths"] = [math.ulp(0.), math.ulp(0.), 0., 0.]
    input["target"]["position"] = [sys.float_info.max]*3
    answer = exact_probe([input])[0]
    assert answer["status"] == 4
    answer = exact_probe([case()], chart=(math.nan, math.nan))[0]
    assert answer["status"] == 0 and answer["polynomial_status"] == 1 and "residual" not in answer
