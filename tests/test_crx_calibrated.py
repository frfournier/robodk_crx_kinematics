# SPDX-License-Identifier: Apache-2.0
# SPDX-FileCopyrightText: 2026 Centre Technologique en Aérospatiale
"""Independent high-precision modified-DH oracle for the calibrated PoE bridge.

Geometry perturbations below are synthetic, never measured calibration data.
The oracle differentiates transform products, rather than reusing space screws
or the production cross-product Jacobian construction.
"""

import copy
import json
import math
import os
from pathlib import Path
import random
import subprocess

import mpmath as mp
import numpy as np
import pytest

from crx_asset_reference import derive_model

ROOT = Path(__file__).resolve().parents[1]
ASSETS = json.loads((ROOT/"tests/fixtures/crx_asset_frames.json").read_text())["assets"]


def _probe(cases, certificates=False, correct=False):
    configured = os.environ.get("CRXKIN_CALIBRATED_PROBE_PATH")
    executable = Path(configured) if configured else ROOT/"build/Release/crx_calibrated_tests.exe"
    if configured:
        assert executable.is_file(), f"Configured calibrated probe missing: {executable}"
    elif not executable.is_file():
        pytest.skip("Matching calibrated native probe unavailable")
    lines = [str(len(cases))]
    for case in cases:
        values = [case["scale"], case.get("limit_units", 0)]
        for row in case["rows"]:
            values.extend(row)
        values.extend(case["senses"])
        for frame in (case["base"], case["tool"]):
            values.extend(frame)
        values.extend(case["lower"])
        values.extend(case["upper"])
        values.extend(case["x"])
        if certificates or correct:
            values.extend(case["target"])
        if certificates:
            inverse = case.get("inverse")
            values.extend([case.get("radius", 0.), int(inverse is not None)])
            values.extend(inverse if inverse is not None else [0.]*36)
        if correct:
            values.extend([case.get("max_iterations", 24), case.get("max_backtracks", 12),
                           case.get("maximum_step", .25), case.get("position_tolerance", 1e-13),
                           case.get("orientation_tolerance", 1e-13), case.get("radius", 0.),
                           int(case.get("certify", True))])
        lines.append(" ".join(format(value, ".17g") for value in values))
    mode = "--correct" if correct else ("--certificates" if certificates else "--batch")
    result = subprocess.run([str(executable), mode], input="\n".join(lines)+"\n",
                            text=True, capture_output=True, check=True, timeout=90)
    records = [json.loads(line) for line in result.stdout.splitlines()]
    assert len(records) == len(cases)
    return records


def _rotation(axis, angle, derivative=False):
    c, s = mp.cos(angle), mp.sin(angle)
    result = mp.zeros(4) if derivative else mp.eye(4)
    i, j = ((1, 2), (2, 0), (0, 1))[axis]
    result[i, i] = result[j, j] = -s if derivative else c
    result[i, j] = -c if derivative else -s
    result[j, i] = c if derivative else s
    return result


def _translation(axis, distance):
    result = mp.eye(4)
    result[axis, 3] = distance
    return result


def _frame(raw, scale):
    w, x, y, z = map(mp.mpf, raw[:4])
    norm = w*w+x*x+y*y+z*z
    result = mp.eye(4)
    result[:3, :3] = mp.matrix([
        [w*w+x*x-y*y-z*z, 2*(x*y-w*z), 2*(x*z+w*y)],
        [2*(x*y+w*z), w*w-x*x+y*y-z*z, 2*(y*z-w*x)],
        [2*(x*z-w*y), 2*(y*z+w*x), w*w-x*x-y*y+z*z],
    ])/norm
    for i in range(3):
        result[i, 3] = mp.mpf(raw[i+4])/scale
    return result


def _oracle(case):
    scale = mp.mpf(case["scale"])
    base, tool = _frame(case["base"], scale), _frame(case["tool"], scale)
    fixed, joints, derivatives = [], [], []
    for row, increment in zip(case["rows"], case["x"], strict=True):
        aq, ar, a, tq, tr, d = row
        alpha, theta = aq*mp.pi/2+mp.mpf(ar), tq*mp.pi/2+mp.mpf(tr)
        fixed.append(_rotation(0, alpha)*_translation(0, mp.mpf(a)/scale))
        joints.append(_rotation(2, theta+mp.mpf(increment))*_translation(2, mp.mpf(d)/scale))
        derivatives.append(_rotation(2, theta+mp.mpf(increment), True)
                           *_translation(2, mp.mpf(d)/scale))
    pose = base.copy()
    for b, joint in zip(fixed, joints, strict=True):
        pose = pose*b*joint
    pose = pose*tool
    jacobian = mp.zeros(6)
    for column in range(6):
        derivative = base.copy()
        for i in range(6):
            derivative = derivative*fixed[i]*(derivatives[i] if i == column else joints[i])
        derivative = derivative*tool
        body = pose[:3, :3].T*derivative[:3, :3]
        linear = pose[:3, :3].T*derivative[:3, 3]
        for i in range(3):
            jacobian[i, column] = linear[i]
        jacobian[3, column], jacobian[4, column], jacobian[5, column] = (
            body[2, 1], body[0, 2], body[1, 0])
    return pose, jacobian


def _contains(matrix, enclosure):
    assert len(enclosure) == matrix.rows
    for i in range(matrix.rows):
        assert len(enclosure[i]) == matrix.cols
        for j in range(matrix.cols):
            lo, hi = enclosure[i][j]
            assert lo is not None and hi is not None and lo <= hi
            assert mp.mpf(lo) <= matrix[i, j] <= mp.mpf(hi), (i, j, matrix[i, j], lo, hi)
            assert mp.mpf(hi)-mp.mpf(lo) < mp.mpf("1e-7")


def _contains_pose(pose, record):
    _contains(pose[:3, :3], record["rotation"])
    _contains(pose[:3, 3], record["translation"])


def _synthetic(scale=1.0, perturb=True, quarters=True):
    # Generic six-axis chain. Nonorthogonal twists and off-pattern offsets are
    # intentional; this cannot be admitted by the nominal CRX signature.
    rows = [
        [0, 0.0, 0.0, 0, 0.0, .24],
        [-1, 0.0, 0.0, -1, 0.0, 0.0],
        [2, 0.0, .54, 0, 0.0, 0.0],
        [-1, 0.0, 0.0, 0, 0.0, .54],
        [1, 0.0, 0.0, 0, 0.0, -.15],
        [-1, 0.0, 0.0, 0, 0.0, .16],
    ]
    for i, row in enumerate(rows):
        if perturb:
            row[1] = (i-2.5)*.00012
            row[2] += (i+1)*.00003
            row[4] = (2.5-i)*.00018
            row[5] += (i-3)*.00002
        if not quarters:
            row[1] = row[0]*math.pi/2+row[1]
            row[4] = row[3]*math.pi/2+row[4]
            row[0] = row[3] = 0
        row[2] *= scale
        row[5] *= scale
    return dict(scale=.54*scale, rows=rows, senses=[1, 1, -1, -1, -1, -1],
                base=[2., .25, -.5, .125, .2*scale, -.1*scale, .3*scale],
                tool=[1., -.1, .2, -.25, .03*scale, -.02*scale, .01*scale],
                lower=[-12.]*6, upper=[12.]*6, x=[0.]*6)


def _quaternion_product(a, b):
    w, x, y, z = a
    s, u, v, t = b
    return [w*s-x*u-y*v-z*t, w*u+x*s+y*t-z*v,
            w*v-x*t+y*s+z*u, w*t+x*v-y*u+z*s]


def _target_at(case):
    pose, _ = _oracle(case)
    q = list(map(mp.mpf, case["base"][:4]))
    q = [value/mp.sqrt(sum(t*t for t in q)) for value in q]
    for row, increment in zip(case["rows"], case["x"], strict=True):
        aq, ar, _, tq, tr, _ = row
        a = (aq*mp.pi/2+mp.mpf(ar))/2
        t = (tq*mp.pi/2+mp.mpf(tr)+mp.mpf(increment))/2
        q = _quaternion_product(q, [mp.cos(a), mp.sin(a), 0, 0])
        q = _quaternion_product(q, [mp.cos(t), 0, 0, mp.sin(t)])
    tool = list(map(mp.mpf, case["tool"][:4]))
    tool = [value/mp.sqrt(sum(t*t for t in tool)) for value in tool]
    q = _quaternion_product(q, tool)
    return [*map(float, q), *(float(pose[i, 3]*mp.mpf(case["scale"])) for i in range(3))]


@pytest.mark.parametrize("scale", [1e-6, 1., 1e6])
@pytest.mark.parametrize("perturb,quarters", [(False, True), (True, True), (True, False)])
def test_calibrated_dh_poe_and_differentiated_jacobian(scale, perturb, quarters):
    rng = random.Random(198038)
    cases = [_synthetic(scale, perturb, quarters) for _ in range(9)]
    for case in cases[1:]:
        case["x"] = [rng.uniform(-3*math.pi, 3*math.pi) for _ in range(6)]
    with mp.workdps(110):
        for case, record in zip(cases, _probe(cases), strict=True):
            assert record["status"] == record["dh_status"] == record["poe_status"] == record["kinematics_status"] == 0
            pose, jacobian = _oracle(case)
            _contains_pose(pose, record["dh"])
            _contains_pose(pose, record["poe"])
            _contains(jacobian, record["jacobian"])
            home_case = copy.deepcopy(case)
            home_case["x"] = [0.]*6
            home, _ = _oracle(home_case)
            _contains_pose(home, record["home"])


@pytest.mark.parametrize("asset", ASSETS, ids=lambda asset: asset["asset"])
def test_calibrated_bridge_zero_perturbation_asset_parity(asset):
    model = derive_model(asset)
    # Preserve the captured shoulder-frame translation. Rotations in this
    # inventory are identity; synthetic cases above cover general quaternions.
    np.testing.assert_allclose(model["base"][:3, :3], np.eye(3), atol=1e-12, rtol=0)
    np.testing.assert_allclose(model["tool"][:3, :3], np.eye(3), atol=1e-12, rtol=0)
    cases = []
    observations = [asset["zero"], *asset["probes"], *asset["samples"][:3]]
    for observation in observations:
        command = np.radians(observation["commands_deg"])
        case = dict(scale=float(np.max(np.abs(model["dh"][:, [1, 3]]))),
                    rows=[[0, float(alpha), float(a), 0, float(theta), float(d)]
                          for alpha, a, theta, d in model["dh"]],
                    senses=[1, 1, -1, -1, -1, -1],
                    base=[1., 0., 0., 0., *map(float, model["base"][:3, 3])],
                    tool=[1., 0., 0., 0., *map(float, model["tool"][:3, 3])],
                    lower=[-12.]*6, upper=[12.]*6,
                    x=list(model["motion_map"]@command))
        cases.append(case)
    with mp.workdps(110):
        for case, observation, record in zip(cases, observations, _probe(cases), strict=True):
            assert record["status"] == record["dh_status"] == record["poe_status"] == 0
            pose, jacobian = _oracle(case)
            _contains_pose(pose, record["dh"])
            _contains_pose(pose, record["poe"])
            _contains(jacobian, record["jacobian"])
            approximate = np.array(pose.tolist(), dtype=float)
            approximate[:3, 3] *= case["scale"]
            np.testing.assert_allclose(approximate, observation["flange_pose"], atol=2e-9, rtol=0)


def test_calibrated_model_rejects_invalid_inputs():
    cases = []
    for scale in [0., -1., math.inf, math.nan]:
        case = _synthetic(); case["scale"] = scale; cases.append(case)
    for sense in [0., .999999, 2., math.nan]:
        case = _synthetic(); case["senses"][3] = sense; cases.append(case)
    case = _synthetic(); case["base"][:4] = [0.]*4; cases.append(case)
    case = _synthetic(); case["tool"][5] = math.nan; cases.append(case)
    case = _synthetic(); case["rows"][0][1] = math.inf; cases.append(case)
    case = _synthetic(); case["lower"][2] = 13.; cases.append(case)
    assert all(record == {"status": 1} for record in _probe(cases))


def test_calibrated_declared_quarters_and_quaternion_scaling():
    cases = []
    for exponent in [-500, 0, 500]:
        case = _synthetic()
        case["x"] = [.1, -.2, .3, -.4, .5, -.6]
        for frame in (case["base"], case["tool"]):
            frame[:4] = [math.ldexp(value, exponent) for value in frame[:4]]
        # Extreme signed quarter declarations reduce exactly modulo four;
        # this does not reduce or round the independently stored correction.
        case["rows"][0][0] = -(2**31)
        case["rows"][1][3] = 2**31-1
        cases.append(case)
    with mp.workdps(110):
        for case, record in zip(cases, _probe(cases), strict=True):
            assert record["status"] == record["dh_status"] == record["poe_status"] == 0
            pose, jacobian = _oracle(case)
            _contains_pose(pose, record["dh"])
            _contains_pose(pose, record["poe"])
            _contains(jacobian, record["jacobian"])


def _rowsum(matrix):
    return max(sum(abs(matrix[i, j]) for j in range(matrix.cols))
               for i in range(matrix.rows))


def _moved_screws(case):
    scale = mp.mpf(case["scale"])
    prefix = _frame(case["base"], scale)
    axes, origins = [], []
    for row, increment in zip(case["rows"], case["x"], strict=True):
        aq, ar, a, tq, tr, d = row
        prefix = prefix*_rotation(0, aq*mp.pi/2+mp.mpf(ar))*_translation(0, mp.mpf(a)/scale)
        axes.append(prefix[:3, 2])
        origins.append(prefix[:3, 3])
        prefix = prefix*_rotation(2, tq*mp.pi/2+mp.mpf(tr)+mp.mpf(increment))*_translation(2, mp.mpf(d)/scale)
    return axes, origins


def _check_calibrated_certificate(case, answer):
    assert answer["certified"] and answer["failure"] == 0
    pose, jacobian = _oracle(case)
    inverse = mp.matrix(answer["preconditioner"])
    rho, radius, beta, lipschitz, eta, error, screw_sum, angular_sum, p0 = (
        mp.mpf(answer[name]) for name in ["rho", "radius", "beta", "lipschitz", "eta",
                                        "inverse_residual", "screw_sum", "angular_sum", "position_norm"])
    assert 0 <= rho <= radius and radius > 0
    assert _rowsum(mp.eye(6)-inverse*jacobian) <= error < 1
    assert _rowsum(inverse)/(1-error) <= beta
    assert _rowsum(jacobian**-1) <= beta
    assert mp.sign(mp.det(jacobian)) == mp.sign(mp.det(inverse)) == answer["local_sign"]
    assert answer["command_sign"] == answer["local_sign"]*math.prod(case["senses"])
    axes, origins = _moved_screws(case)
    actual_angular = sum(mp.norm(axis) for axis in axes)
    actual_screw = actual_angular
    for origin, axis in zip(origins, axes, strict=True):
        cross = mp.matrix([origin[1]*axis[2]-origin[2]*axis[1],
                           origin[2]*axis[0]-origin[0]*axis[2],
                           origin[0]*axis[1]-origin[1]*axis[0]])
        actual_screw += mp.norm(cross)
    assert actual_angular <= angular_sum and actual_screw <= screw_sum
    assert mp.norm(pose[:3, 3]) <= p0
    target = _frame(case["target"], mp.mpf(case["scale"]))
    relative = mp.eye(4)
    relative[:3, :3] = pose[:3, :3].T*target[:3, :3]
    relative[:3, 3] = pose[:3, :3].T*(target[:3, 3]-pose[:3, 3])
    logarithm = mp.logm(relative)
    twist = mp.matrix([logarithm[0, 3], logarithm[1, 3], logarithm[2, 3],
                       logarithm[2, 1], logarithm[0, 2], logarithm[1, 0]])
    assert max(abs(value) for value in (jacobian**-1)*twist) <= eta
    reach = radius*screw_sum+p0
    lam = mp.pi*(p0+reach)/2
    b = mp.pi/2+mp.pi**2*lam/8
    c = (mp.sqrt(2)*mp.pi**2/8
         +(mp.sqrt(2)*mp.pi**3/16+3*mp.sqrt(2)*mp.pi**2/16)*lam+mp.pi**2/8)
    assert b*(1+reach)**2*(c+1)*screw_sum**2 <= lipschitz
    assert radius*angular_sum < mp.pi
    assert beta*lipschitz*radius < 1 and 2*beta*lipschitz*eta <= 1
    assert beta*lipschitz*rho*rho/2+eta <= rho

    def residual(*increments):
        sample = {**case, "x": list(increments)}
        actual, _ = _oracle(sample)
        angular = target[:3, :3].T*actual[:3, :3]
        return mp.matrix([*(actual[:3, 3]-target[:3, 3]),
                          (angular[2, 1]-angular[1, 2])/2,
                          (angular[0, 2]-angular[2, 0])/2,
                          (angular[1, 0]-angular[0, 1])/2])

    centre = list(map(mp.mpf, case["x"]))
    root = mp.findroot(residual, centre, tol=mp.mpf("1e-145"), maxsteps=20)
    actual, _ = _oracle({**case, "x": list(root)})
    assert max(abs(value) for value in actual-target) < mp.mpf("1e-125")
    assert max(abs(root[i]-centre[i]) for i in range(6)) <= rho
    conversion = mp.pi/180 if case.get("limit_units", 0) == 1 else mp.mpf(1)
    reserve = min(min(mp.mpf(sense)*value-mp.mpf(lo)*conversion, mp.mpf(hi)*conversion-mp.mpf(sense)*value)
                  for value, sense, lo, hi in zip(centre, case["senses"], case["lower"], case["upper"], strict=True))
    assert mp.mpf(answer["command_reserve"]) <= reserve
    if answer["admitted"]:
        assert reserve >= rho
        for value, sense, lo, hi in zip(root, case["senses"], case["lower"], case["upper"], strict=True):
            assert mp.mpf(lo)*conversion <= mp.mpf(sense)*value <= mp.mpf(hi)*conversion


@pytest.mark.parametrize("scale", [1e-6, 1., 1e6])
@pytest.mark.parametrize("senses", [[1.]*6, [1., -1., -1., 1., -1., 1.]])
def test_calibrated_ball_constants_and_independent_root(scale, senses):
    with mp.workdps(160):
        cases = []
        for centre in [[.3, -.7, .4, .8, -.6, .2], [-.4, -.6, .5, -.7, .8, -.2]]:
            case = _synthetic(scale)
            case["senses"] = senses
            case["x"] = centre
            case["target"] = _target_at(case)
            cases.append(case)
        for case, answer in zip(cases, _probe(cases, certificates=True), strict=True):
            _check_calibrated_certificate(case, answer)
            assert answer["admitted"]


def test_calibrated_ball_limit_admission_is_separate():
    with mp.workdps(160):
        case = _synthetic()
        case["x"] = [.3, -.7, .4, .8, -.6, .2]
        case["target"] = _target_at(case)
        cases = [case, copy.deepcopy(case), copy.deepcopy(case)]
        command = case["senses"][1]*case["x"][1]
        cases[1]["upper"][1] = command
        cases[2]["upper"][1] = command-.1
        for i, (candidate, answer) in enumerate(zip(cases, _probe(cases, certificates=True), strict=True)):
            _check_calibrated_certificate(candidate, answer)
            assert answer["admitted"] == (i == 0)


def test_calibrated_ball_negative_hypotheses_and_scaled_inverse():
    with mp.workdps(160):
        case = _synthetic()
        case["x"] = [.3, -.7, .4, .8, -.6, .2]
        case["target"] = _target_at(case)
        baseline = _probe([case], certificates=True)[0]
        _check_calibrated_certificate(case, baseline)
        scaled = copy.deepcopy(case)
        scaled["inverse"] = [float(mp.mpf(value)*mp.mpf(".9"))
                             for row in baseline["preconditioner"] for value in row]
        answer = _probe([scaled], certificates=True)[0]
        _check_calibrated_certificate(scaled, answer)
        bad = []
        for radius in [-1., 1., 1e-20]:
            item = copy.deepcopy(case); item["radius"] = radius; bad.append(item)
        item = copy.deepcopy(case); item["inverse"] = [0.]*36; bad.append(item)
        item = copy.deepcopy(case); item["inverse"] = [math.nan]*36; bad.append(item)
        item = copy.deepcopy(case)
        item["target"][:4] = _quaternion_product(item["target"][:4], [0., 1., 0., 0.])
        bad.append(item)
        assert all(not answer["certified"] for answer in _probe(bad, certificates=True))


@pytest.mark.parametrize("scale", [1e-6, 1., 1e6])
def test_calibrated_correction_independent_target_and_proof_parity(scale):
    with mp.workdps(160):
        true_case = _synthetic(scale)
        true_case["x"] = [2*math.pi+.3, -.7, .4, .8, -.6, .2]
        target = _target_at(true_case)
        candidate = copy.deepcopy(true_case)
        candidate["target"] = target
        candidate["x"] = [x+.001*(i+1) for i, x in enumerate(true_case["x"])]
        disabled = {**candidate, "certify": False}
        proven, numerical = _probe([candidate, disabled], correct=True)
        oracle_pose, oracle_jacobian = _oracle(candidate)
        np.testing.assert_allclose(proven["point_pose"]["rotation"],
                                   np.array(oracle_pose[:3, :3].tolist(), dtype=float), atol=2e-13, rtol=0)
        np.testing.assert_allclose(proven["point_pose"]["translation"],
                                   np.array(oracle_pose[:3, 3].tolist(), dtype=float), atol=2e-13, rtol=0)
        np.testing.assert_allclose(proven["point_jacobian"],
                                   np.array(oracle_jacobian.tolist(), dtype=float), atol=2e-13, rtol=0)
        assert proven["correction_status"] == numerical["correction_status"] == 0
        assert proven["iterations"] > 0 and proven["iterations"] <= 24
        assert proven["evaluations"] <= 1+24*13
        assert proven["commands"][0] > 2*math.pi
        for key in ["commands", "increments"]:
            assert np.asarray(proven[key]).tobytes() == np.asarray(numerical[key]).tobytes()
        for key in ["iterations", "backtracks", "evaluations", "position_error", "orientation_error"]:
            assert proven[key] == numerical[key]
        assert proven["certificate_attempted"] and not numerical["certificate_attempted"]
        assert not numerical["certified"]
        _check_calibrated_certificate({**candidate, "x": proven["increments"]}, proven)


def test_calibrated_correction_budgets_and_dispatch_do_not_certify():
    with mp.workdps(160):
        case = _synthetic()
        case["x"] = [.3, -.7, .4, .8, -.6, .2]
        case["target"] = _target_at(case)
        case["x"] = [value+.001 for value in case["x"]]
        exhausted = {**case, "max_iterations": 0}
        answer = _probe([exhausted], correct=True)[0]
        assert answer["correction_status"] == 4 and answer["evaluations"] == 1
        assert not answer["certificate_attempted"] and not answer["certified"]
        loose = {**case, "position_tolerance": .1, "orientation_tolerance": .1}
        answer = _probe([loose], correct=True)[0]
        assert answer["correction_status"] == 0 and answer["iterations"] == 0
        assert answer["certificate_attempted"] and not answer["certified"]
        invalid = [{**case, key: value} for key, value in [
            ("max_iterations", 129), ("max_backtracks", 33), ("maximum_step", 0.),
            ("position_tolerance", -1.), ("orientation_tolerance", math.nan), ("radius", -1.)]]
        assert all(answer["correction_status"] == 1 and not answer["certificate_attempted"]
                   for answer in _probe(invalid, correct=True))


@pytest.mark.parametrize("asset", ASSETS, ids=lambda asset: asset["asset"])
def test_calibrated_correction_processes_both_nominal_partners(asset):
    from crx_kinematics import CrxSolver

    configured = os.environ.get("CRXKIN_LIBRARY_PATH")
    dll = Path(configured) if configured else ROOT/"build/Release/crx_kinematics.dll"
    if configured:
        assert dll.is_file(), f"Configured nominal library missing: {dll}"
    elif not dll.is_file():
        pytest.skip("Matching nominal DLL unavailable")
    model = derive_model(asset)
    assert np.allclose(model["base"][:3, :3], np.eye(3))
    assert np.allclose(model["tool"][:3, :3], np.eye(3))
    rows = [[0, float(alpha)+(i-2.5)*1e-5, float(a)+(i+1)*.005,
             0, float(theta)+(2.5-i)*1e-5, float(d)+(i-3)*.004]
            for i, (alpha, a, theta, d) in enumerate(model["dh"])]
    case = dict(scale=float(np.max(np.abs(model["dh"][:, [1, 3]]))), rows=rows,
                senses=[1., 1., -1., -1., -1., -1.],
                base=[1., 0., 0., 0., *map(float, model["base"][:3, 3])],
                tool=[1., 0., 0., 0., *map(float, model["tool"][:3, 3])],
                lower=[-math.pi]*6, upper=[math.pi]*6,
                x=list(np.radians([30., -40., 45., 50., -35., 20.])
                       *np.array([1., 1., -1., -1., -1., -1.])))
    with mp.workdps(160):
        case["target"] = _target_at(case)
        target_pose = np.array(_frame(case["target"], mp.mpf(case["scale"])).tolist(), dtype=float)
        target_pose[:3, 3] *= case["scale"]
        name = asset["asset"].removeprefix("Fanuc-").removesuffix("-Custom.robot")
        with CrxSolver(name, dll_path=dll, joint_limits=np.array([[-180.]*6, [180.]*6])) as nominal:
            discovered = nominal.identify(target_pose, max_solutions=64)
        groups = {}
        for seed, address, valid, regular in zip(discovered.solutions, discovered.wrist_addresses,
                                                discovered.wrist_address_valid, discovered.regular, strict=True):
            if valid and regular:
                groups.setdefault(tuple(address[:2]), {})[int(address[2])] = np.radians(seed)
        pairs = [group for group in groups.values() if set(group) == {0, 1}]
        assert pairs, "Expected at least one regular numerical nominal partner pair"

        def conditioning(pair):
            return max(_rowsum(_oracle({**case, "x": list(pair[flip]*case["senses"])})[1]**-1)
                       for flip in [0, 1])

        pair = min(pairs, key=conditioning)
        candidates = [{**case, "x": list(pair[flip]*case["senses"])} for flip in [0, 1]]
        answers = _probe(candidates, correct=True)
        for candidate, answer in zip(candidates, answers, strict=True):
            assert answer["correction_status"] == 0 and answer["iterations"] > 0
            assert answer["numerical_limits"] and answer["admitted"]
            _check_calibrated_certificate({**candidate, "x": answer["increments"]}, answer)
        first, second = [np.array(answer["commands"]) for answer in answers]
        assert np.max(np.abs(np.remainder(first-second+math.pi, 2*math.pi)-math.pi)) > .1
        # Calibration breaks the nominal base-flip identity. Computing a second
        # candidate by flipping the corrected first command would be unsound.
        flipped = first.copy()
        flipped[0] -= math.pi
        flipped[1] = -flipped[1]
        flipped[2] = math.pi-flipped[2]
        flipped[3] -= math.pi
        wrong, _ = _oracle({**case, "x": list(flipped*case["senses"])})
        exact_target = _frame(case["target"], mp.mpf(case["scale"]))
        assert max(abs(value) for value in wrong-exact_target) > mp.mpf("1e-7")


@pytest.mark.parametrize("magnitude", [180., 1e308, float.fromhex("0x0.0000000000001p-1022")])
def test_calibrated_degree_bounds_enclose_exact_conversion(magnitude):
    case = _synthetic()
    case["limit_units"] = 1
    case["lower"] = [-magnitude]*6
    case["upper"] = [magnitude]*6
    answer = _probe([case])[0]
    assert answer["status"] == 0
    with mp.workdps(160):
        bound = mp.mpf(magnitude)*mp.pi/180
        for lower, upper in zip(answer["lower_bounds"], answer["upper_bounds"], strict=True):
            assert mp.mpf(lower[0]) <= -bound <= mp.mpf(lower[1])
            assert mp.mpf(upper[0]) <= bound <= mp.mpf(upper[1])


def test_calibrated_degree_limits_admit_whole_real_command_balls():
    with mp.workdps(160):
        case = _synthetic()
        case["x"] = [2*math.pi+.3, -.7, .4, .8, -.6, .2]
        case["target"] = _target_at(case)
        case["limit_units"] = 1
        case["lower"] = [-720.]*6
        case["upper"] = [720.]*6
        boundary = copy.deepcopy(case)
        boundary["upper"][0] = math.degrees(case["senses"][0]*case["x"][0])
        for candidate, answer in zip([case, boundary], _probe([case, boundary], certificates=True), strict=True):
            _check_calibrated_certificate(candidate, answer)
        assert _probe([case], certificates=True)[0]["admitted"]
        assert not _probe([boundary], certificates=True)[0]["admitted"]
    invalid = _synthetic(); invalid["limit_units"] = 2
    assert _probe([invalid]) == [{"status": 1}]


def test_calibrated_singularity_approach_keeps_local_failure_inconclusive():
    with mp.workdps(160):
        cases = []
        condition = []
        for epsilon in [1e-2, 1e-4, 1e-6, 0.]:
            case = _synthetic(1000., perturb=False)
            case["x"] = [.3, epsilon, .4, .8, epsilon, .2]
            case["target"] = _target_at(case)
            cases.append(case)
            if epsilon:
                condition.append(_rowsum(_oracle(case)[1]**-1))
        assert condition[-1] > 100*condition[0]
        answers = _probe(cases, certificates=True)
        for case, answer in zip(cases, answers, strict=True):
            if answer["certified"]:
                _check_calibrated_certificate(case, answer)
        assert not answers[-1]["certified"]


def test_calibrated_singular_correction_reports_failed_step_without_proof():
    with mp.workdps(160):
        case = _synthetic(1000., perturb=False)
        case["rows"] = [[0, 0., 0., 0, 0., 0.] for _ in range(6)]
        case["rows"][0][2] = 1000.
        case["x"] = [0.]*6
        case["target"] = _target_at(case)
        case["target"][4] += 10.
        answer = _probe([case], correct=True)[0]
        assert answer["correction_status"] == 5 and answer["evaluations"] == 1
        assert not answer["certificate_attempted"] and not answer["certified"]
