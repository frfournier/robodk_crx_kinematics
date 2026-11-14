# SPDX-License-Identifier: Apache-2.0
# SPDX-FileCopyrightText: 2026 Centre Technologique en Aérospatiale
"""Independent high-precision anchored FK/Jacobian and ball containment checks."""

import copy
from fractions import Fraction as F
import json
import math
import os
from pathlib import Path
import random
import subprocess
import sys

import mpmath as mp
import pytest

import crx_exact_reference as ref
from test_crx_exact_target import contains, flatten, pose, unflatten


@pytest.fixture(scope="module")
def probe():
    configured = os.environ.get("CRXKIN_BALL_PROBE_PATH")
    suffix = ".exe" if sys.platform == "win32" else ""
    path = Path(configured) if configured else Path(__file__).resolve().parents[1]/f"build/Release/crx_ball_tests{suffix}"
    if configured:
        assert path.is_file(), f"Configured ball probe missing: {path}"
    elif not path.is_file():
        pytest.skip("Build crx_ball_tests or set CRXKIN_BALL_PROBE_PATH")

    def run(requests=None):
        if requests is None:
            completed = subprocess.run([str(path), "--witness"], text=True,
                                       capture_output=True, check=True, timeout=60)
        else:
            lines = []
            for item in requests:
                inverse = item.get("inverse")
                values = [*flatten(item["input"]), *item["centre"], item.get("radius", 0.),
                          *(inverse if inverse is not None else [0.]*36)]
                reuse = item.get("reuse_input")
                if reuse is not None:
                    values += flatten(reuse)
                mode = (5 if item.get("reference_mode") else 2) if reuse is not None else (3 if item.get("reference_mode") else int(inverse is not None))
                lines.append(" ".join([str(mode), *(repr(float(x)) for x in values)]))
            completed = subprocess.run([str(path), "--batch"], input="\n".join(lines)+"\n",
                                       text=True, capture_output=True, check=True, timeout=60)
        answers = [json.loads(line) for line in completed.stdout.splitlines()]
        assert len(answers) == (16 if requests is None else len(requests))
        return answers
    return run


def matrix_rotation(axis, angle):
    matrix = mp.eye(3)
    i, j = [(1, 2), (2, 0), (0, 1)][axis]
    matrix[i, i] = matrix[j, j] = mp.cos(angle)
    matrix[i, j], matrix[j, i] = -mp.sin(angle), mp.sin(angle)
    return matrix


def prepare_reference(input):
    result = ref.prepare(input)
    rt, rr, ru = [ref.quaternion_rotation(input[key]["quaternion"]) for key in ["target", "base", "tool"]]
    rotation = ref.matmul(ref.matmul(rr, rt), ref.transpose(ru))
    scale = result["scale"]
    pt, pr, pu = [[F(x)/scale for x in input[key]["position"]] for key in ["target", "base", "tool"]]
    translation = [a+b-c for a, b, c in zip(ref.matvec(rr, pt), pr, ref.matvec(rotation, pu), strict=True)]
    result.update(rotation=rotation, translation=translation,
                  center=[translation[i]-result["lengths"][2]*rotation[i][2] for i in range(3)],
                  v=ref.transpose(rotation)[0], w=ref.transpose(rotation)[1])
    return result


def exact_kinematics(input, centre, reference_mode=False):
    problem = prepare_reference(input) if reference_mode else ref.prepare(input)
    lengths = list(map(ref.number, problem["lengths"]))
    q = [ref.number(F(x)) for x in centre]
    a, b, c, r = lengths
    rotation, position = mp.eye(3), mp.matrix([0, 0, 0])
    axes, origins = [], []
    for axis, sign, angle, shift in [(2, 1, q[0], [0, 0, 0]), (1, 1, q[1], [0, 0, a]),
                                    (1, -1, q[2], [0, 0, 0]), (0, -1, q[3], [b, -r, 0]),
                                    (1, -1, q[4], [c, 0, 0]), (0, -1, q[5], [0, 0, 0])]:
        axes.append(sign*rotation[:, axis])
        origins.append(position.copy())
        rotation = rotation*matrix_rotation(axis, sign*angle)
        position += rotation*mp.matrix(shift)
    rotation = rotation*mp.matrix([[0, 0, 1], [0, -1, 0], [1, 0, 0]])
    jacobian = mp.zeros(6)
    for j in range(6):
        # Differentiate elementary transform products rather than the
        # production axis-cross-offset/body-transform formula.
        def sample(value):
            angles = q.copy()
            angles[j] = value
            values, shift = ref.forward(lengths, angles)
            return mp.matrix([*values, *shift])
        derivative = mp.diff(sample, q[j])
        dr = mp.matrix([[derivative[3*i+k] for k in range(3)] for i in range(3)])
        dp = rotation.T*mp.matrix(list(derivative)[9:])
        omega = rotation.T*dr
        jacobian[:, j] = mp.matrix([*dp, omega[2, 1], omega[0, 2], omega[1, 0]])
    # Algebraically cancel the two adjacent Y rotations before evaluating these
    # two entries. In F.T Rx(q5) Ry(q4) Rx(q3) ey, the body z component is the
    # pre-F x component sin(q4) sin(q3); the third joint has the opposite axis.
    # Numerically differentiating and multiplying unsimplified rotations leaves
    # an arbitrary-precision cancellation residue when q4 is exactly zero.
    # This identity preserves exact zeros without a tolerance or mp.chop.
    jacobian[5, 1] = mp.sin(q[4])*mp.sin(q[3])
    jacobian[5, 2] = -jacobian[5, 1]
    return problem, rotation, position, jacobian, axes, origins


def rowsum(matrix):
    return max(sum(abs(matrix[i, j]) for j in range(matrix.cols)) for i in range(matrix.rows))


def check_kinematics(answer):
    assert answer["kinematics_status"] == 0
    input = unflatten(answer["input"])
    expected = exact_kinematics(input, answer["centre"], answer["schema"] == 2)
    for field, values in [("target_rotation", sum(expected[0]["rotation"], [])),
                          ("target_translation", expected[0]["translation"])]:
        for interval, value in zip(answer[field], values, strict=True):
            contains(interval, value)
    _, rotation, position, jacobian, axes, origins = expected
    for field, values in [("rotation", list(rotation)), ("translation", list(position)),
                          ("jacobian", list(jacobian))]:
        for interval, value in zip(answer[field], values, strict=True):
            contains(interval, value)
    for field, vectors in [("axes", axes), ("origins", origins)]:
        for bounds, vector in zip(answer[field], vectors, strict=True):
            for interval, value in zip(bounds, vector, strict=True):
                contains(interval, value)
    contains(answer["closed_determinant"], mp.det(jacobian))
    return expected


def check_certificate(answer, refine=True, kinematics=None):
    assert answer["failure"] == 0
    problem, rotation, position, jacobian, axes, origins = (
        check_kinematics(answer) if kinematics is None else kinematics)
    rho, radius, error, beta, lipschitz, eta, defect, screw_sum, angular_sum, p0, sign = map(mp.mpf, answer["constants"])
    assert 0 <= rho <= radius and radius > 0
    inverse = mp.matrix([answer["preconditioner"][i:i+6] for i in range(0, 36, 6)])
    inverse_error = mp.eye(6)-inverse*jacobian
    assert rowsum(inverse_error) <= error < 1
    assert rowsum(inverse)/(1-error) <= beta
    assert rowsum(jacobian**-1) <= beta
    assert mp.sign(mp.det(jacobian)) == mp.sign(mp.det(inverse)) == sign
    actual_angular = sum(mp.norm(axis) for axis in axes)
    actual_screw = actual_angular+sum(mp.norm(mp.matrix(ref.cross(list(origin), list(axis))))
                                      for origin, axis in zip(origins, axes, strict=True))
    assert actual_angular <= angular_sum and actual_screw <= screw_sum and mp.norm(position) <= p0
    target_r = mp.matrix([[ref.number(x) for x in row] for row in problem["rotation"]])
    target_p = mp.matrix(list(map(ref.number, problem["translation"])))
    relative_r = rotation.T*target_r
    relative_p = rotation.T*(target_p-position)
    trace = sum(relative_r[i, i] for i in range(3))
    contains(answer["trace"], trace)
    assert trace >= 1
    g = mp.eye(4)
    g[:3, :3], g[:3, 3] = relative_r, relative_p
    assert rowsum(g-mp.eye(4)) <= defect
    coarse_eta = mp.mpf(answer["coarse_eta"])
    assert 3*mp.pi*beta*defect <= coarse_eta
    logarithm = mp.logm(g)
    twist = mp.matrix([logarithm[0, 3], logarithm[1, 3], logarithm[2, 3],
                       logarithm[2, 1], logarithm[0, 2], logarithm[1, 0]])
    assert max(abs(x) for x in (jacobian**-1)*twist) <= eta
    if answer["used_log_series"]:
        assert defect < .5 and eta < coarse_eta
    else:
        assert eta == coarse_eta
    reach = radius*screw_sum+p0
    lam = (mp.pi/2)*(p0+reach)
    b = mp.pi/2+mp.pi**2*lam/8
    c = mp.sqrt(2)*mp.pi**2/8+(mp.sqrt(2)*mp.pi**3/16+3*mp.sqrt(2)*mp.pi**2/16)*lam+mp.pi**2/8
    assert b*(1+reach)**2*(c+1)*screw_sum**2 <= lipschitz
    assert radius*angular_sum < mp.pi
    assert beta*lipschitz*radius < 1 and 2*beta*lipschitz*eta <= 1
    assert beta*lipschitz*rho*rho/2+eta <= rho
    if refine:
        def residual(*angles):
            r, p = ref.forward(problem["lengths"], angles)
            rot = mp.matrix([r[i:i+3] for i in range(0, 9, 3)])
            angular = target_r.T*rot
            return mp.matrix([*(mp.matrix(p)-target_p),
                              (angular[2, 1]-angular[1, 2])/2,
                              (angular[0, 2]-angular[2, 0])/2,
                              (angular[1, 0]-angular[0, 1])/2])
        q = list(map(mp.mpf, answer["centre"]))
        root = mp.findroot(residual, q, tol=mp.mpf('1e-150'), maxsteps=30)
        assert max(abs(x) for x in residual(*root)) < mp.mpf('1e-130')
        r, p = ref.forward(problem["lengths"], root)
        assert max(abs(a-b) for a, b in zip(r, target_r, strict=True)) < mp.mpf('1e-130')
        assert max(abs(a-b) for a, b in zip(p, target_p, strict=True)) < mp.mpf('1e-130')
        assert max(abs(root[i]-q[i]) for i in range(6)) <= rho


def quaternion_product(first, second):
    w, x, y, z = first
    a, b, c, d = second
    return [w*a-x*b-y*c-z*d, w*b+x*a+y*d-z*c,
            w*c-x*d+y*a+z*b, w*d+x*c-y*b+z*a]


def generated(seed=0, scale=1., signed=False, frames=False, wrist=None, shoulder=None):
    rng = random.Random(seed)
    q = [rng.uniform(-2, 2) for _ in range(6)]
    if wrist is not None:
        q[4] = wrist
    if shoulder is not None:
        q[1] = shoulder
    lengths = [scale, (.6 if not signed else -.6)*scale, .2*scale, .5*scale]
    quaternion = [mp.mpf(1), mp.mpf(0), mp.mpf(0), mp.mpf(0)]
    for axis, angle in [(2, q[0]), (1, q[1]), (1, -q[2]), (0, -q[3]), (1, -q[4]), (0, -q[5])]:
        factor = [mp.cos(mp.mpf(angle)/2), mp.mpf(0), mp.mpf(0), mp.mpf(0)]
        factor[axis+1] = mp.sin(mp.mpf(angle)/2)
        quaternion = quaternion_product(quaternion, factor)
    quaternion = quaternion_product(quaternion, [0, 1/mp.sqrt(2), 0, 1/mp.sqrt(2)])
    _, shift = ref.forward(lengths, list(map(mp.mpf, q)))
    input = {"lengths": lengths, "base_height": 0., "encoder_zero": [0.]*6,
             "senses": [1.]*6, "lower_commands": [-4.]*6, "upper_commands": [4.]*6,
             "target": pose(list(map(float, quaternion)), list(map(float, shift))),
             "base": pose(), "tool": pose()}
    if frames:
        input["base"] = pose([.7, -.2, .3, .1], [scale*.1, scale*.2, -scale*.3])
        input["tool"] = pose([.9, .1, -.3, .2], [scale*.15, -scale*.2, scale*.05])
        b, u = input["base"], input["tool"]
        world_q = quaternion_product(quaternion_product(list(map(mp.mpf, b["quaternion"])), quaternion),
                                     list(map(mp.mpf, u["quaternion"])))
        rb = mp.matrix([[ref.number(x) for x in row] for row in ref.quaternion_rotation(b["quaternion"])])
        canonical_r = mp.matrix([[ref.number(x) for x in row] for row in ref.quaternion_rotation(input["target"]["quaternion"])])
        world_p = mp.matrix(b["position"])+rb*(mp.matrix(shift)+canonical_r*mp.matrix(u["position"]))
        input["target"] = pose(list(map(float, world_q)), list(map(float, world_p)))
    return {"input": input, "centre": q}


def test_sixteen_solution_witness_and_perturbed_roots(probe):
    with mp.workdps(200):
        answers = probe()
        for answer in answers:
            check_certificate(answer)


@pytest.mark.parametrize("scale,signed,frames", [(1., False, False), (1e-3, True, False), (1e3, False, True)])
def test_generated_canonical_balls_and_jacobians(probe, scale, signed, frames):
    with mp.workdps(200):
        requests = [generated(i+20, scale, signed, frames) for i in range(8)]
        answers = probe(requests)
        for answer in answers:
            check_certificate(answer)


def test_scaled_preconditioner_requires_neumann_residual_factor(probe):
    with mp.workdps(200):
        request = generated(20)
        initial = probe([request])[0]
        request["centre"] = [x+1e-9 for x in request["centre"]]
        request["inverse"] = [.1*x for x in initial["preconditioner"]]
        answer = probe([request])[0]
        check_certificate(answer)
        assert answer["constants"][2] > .8


@pytest.mark.parametrize("scale", [.001, 1., 1000.])
def test_raw_reference_composition_without_a_rounded_flange_target(probe, scale):
    with mp.workdps(200):
        requests = []
        for seed in range(4):
            item = generated(seed+20, scale)
            input = item["input"]
            canonical_q = list(map(mp.mpf, input["target"]["quaternion"]))
            canonical_r = mp.matrix([[ref.number(x) for x in row]
                                     for row in ref.quaternion_rotation(input["target"]["quaternion"])])
            _, canonical_p = ref.forward(input["lengths"], list(map(mp.mpf, item["centre"])))
            reference = pose([.7, -.2, .3, .1], [scale*.2, -scale*.1, scale*.3])
            tool = pose([.9, .1, -.3, .2], [-scale*.1, scale*.05, scale*.12])
            rr = mp.matrix([[ref.number(x) for x in row] for row in ref.quaternion_rotation(reference["quaternion"])])
            conjugate = [reference["quaternion"][0], *[-x for x in reference["quaternion"][1:]]]
            target_q = quaternion_product(quaternion_product(list(map(mp.mpf, conjugate)), canonical_q),
                                           list(map(mp.mpf, tool["quaternion"])))
            target_p = rr.T*(mp.matrix(canonical_p)+canonical_r*mp.matrix(tool["position"])-mp.matrix(reference["position"]))
            input.update(base_height=.35*scale, base=reference, tool=tool,
                         target=pose(list(map(float, target_q)), list(map(float, target_p))))
            item["reference_mode"] = True
            requests.append(item)
        for answer in probe(requests):
            assert answer["schema"] == 2
            check_certificate(answer)
        changed = copy.deepcopy(requests[-1]["input"])
        changed["target"]["position"][1] += 1e-12*scale
        check_certificate(probe([{**requests[-1], "reuse_input": changed}])[0])


def test_invalid_preconditioners_radii_and_centres(probe):
    with mp.workdps(200):
        base = generated(20)
    requests = []
    for inverse in [[0.]*36, [math.nan]*36, [math.inf]*36]:
        requests.append({**base, "inverse": inverse})
    for radius in [-1., math.nan, math.inf, 1e6, 1e-30]:
        requests.append({**base, "radius": radius})
    for value in [math.nan, math.inf]:
        requests.append({**base, "centre": [value]*6})
    answers = probe(requests)
    assert all(answer["failure"] != 0 and "constants" not in answer for answer in answers)
    assert answers[0]["failure"] == 5
    assert answers[1]["failure"] == answers[2]["failure"] == 4


def test_reuse_rechecks_target_reserve_and_every_model_word(probe):
    with mp.workdps(200):
        base = generated(20)
        changed = copy.deepcopy(base["input"])
        changed["target"]["position"][0] += 1e-12
        answer = probe([{**base, "reuse_input": changed}])[0]
        check_certificate(answer)
        assert answer["input"] == flatten(changed)
        far = copy.deepcopy(changed)
        far["target"]["position"][0] += .1
        failed = probe([{**base, "reuse_input": far}])[0]
        assert failed["failure"] != 0 and "constants" not in failed
        requests = []
        # Only target identity words 29..44 may change during reuse.
        words = flatten(changed)
        for index in [*range(29), *range(45, 77)]:
            mutated = words.copy()
            if 11 <= index < 17:
                mutated[index] *= -1 # Keep unit senses valid.
            else:
                mutated[index] = math.nextafter(mutated[index], math.inf)
            requests.append({**base, "reuse_input": unflatten(mutated)})
        for failed in probe(requests):
            assert failed["failure"] == 11 and "constants" not in failed


def test_log_boundary_and_singular_limits_are_inconclusive(probe):
    with mp.workdps(200):
        base = generated(20)
        half_turn = copy.deepcopy(base)
        half_turn["input"]["target"]["quaternion"] = quaternion_product(base["input"]["target"]["quaternion"], [0., 1., 0., 0.])
        answer = probe([half_turn])[0]
        assert answer["failure"] == 7 and "constants" not in answer
        # Offset wrist: q5=0 alone need not be singular. Setting q2=0 as
        # well zeros every term in the independent determinant formula.
        requests = [generated(20, wrist=x, shoulder=0.) for x in [1e-2, 1e-4, 1e-8, 1e-12, 0.]]
        answers = probe(requests)
        assert answers[0]["failure"] == 0
        determinants = []
        for answer in answers:
            determinants.append(abs(mp.det(check_kinematics(answer)[3])))
            if answer["failure"] == 0:
                check_certificate(answer)
        assert all(a > b for a, b in zip(determinants, determinants[1:]))
        assert all(answer["failure"] != 0 for answer in answers[-2:])
