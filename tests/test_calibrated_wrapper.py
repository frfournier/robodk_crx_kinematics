# SPDX-License-Identifier: Apache-2.0
# SPDX-FileCopyrightText: 2026 Centre Technologique en Aérospatiale
"""Public calibrated NumPy API, exact-result semantics and ownership checks."""

from concurrent.futures import ThreadPoolExecutor
import copy
import math

import mpmath as mp
import numpy as np
import pytest

from crx_kinematics import (
    CalibratedFrame, CalibratedOptions, CalibratedSolver, CalibratedStatus,
    ClosedSolverError, Coverage, CrxSolver,
)
from crx_asset_reference import derive_model
from test_crx_calibrated import ASSETS, _check_calibrated_certificate, _oracle, _target_at
from test_crx_calibrated_api import case_input


def solver(nominal, case, **kwargs):
    rows = case["rows"]
    options = dict(alpha_quarters=[row[0] for row in rows], theta_quarters=[row[3] for row in rows],
        senses=case["senses"], base=CalibratedFrame(case["base"][:4], case["base"][4:]),
        tool=CalibratedFrame(case["tool"][:4], case["tool"][4:]),
        joint_limits=[case["lower"], case["upper"]],
        limit_units="degrees" if case.get("limit_units", 0) else "radians", length_scale=case["scale"])
    options.update(kwargs)
    return CalibratedSolver(nominal, [[row[1], row[2], row[4], row[5]] for row in rows], **options)


def target(case):
    return CalibratedFrame(case["target"][:4], case["target"][4:])


def seeds(case):
    return np.degrees(np.array(case["x"])*np.array(case["senses"]))[None, :]+.05


def proof(certificate):
    return dict(certified=True, failure=0, admitted=certificate.commands_admitted,
        local_sign=certificate.local_jacobian_sign, command_sign=certificate.command_jacobian_sign,
        rho=certificate.existence_radius_rad, radius=certificate.uniqueness_radius_rad,
        beta=certificate.beta, lipschitz=certificate.lipschitz, eta=certificate.eta,
        inverse_residual=certificate.inverse_residual, screw_sum=certificate.screw_norm_sum,
        angular_sum=certificate.angular_norm_sum, position_norm=certificate.anchor_position_norm,
        command_reserve=certificate.command_reserve_lower_rad,
        preconditioner=certificate.preconditioner.tolist())


def test_calibrated_python_owned_exact_results_survive_closure(kinematics_lib):
    case = case_input()
    supplied = seeds(case)
    before = supplied.tobytes()
    with CrxSolver("CRX-10iA", dll_path=kinematics_lib._name) as nominal:
        with solver(nominal, case) as calibrated:
            submitted_target = target(case)
            result = calibrated.correct(submitted_target, supplied)
            assert not np.shares_memory(result.target.quaternion, submitted_target.quaternion)
            assert not np.shares_memory(result.target.position, submitted_target.position)
            assert result.success and result.coverage == Coverage.INCONCLUSIVE
            assert result.proofs_requested and result.local_count == result.admitted_count == 1
            assert result.attempts[0].seed_degrees.tobytes() == before
            geometry = calibrated.geometry
        # Closing the calibrated model does not close an externally owned nominal solver.
        assert np.isfinite(nominal.fk([0.]*6)).all()
    assert supplied.tobytes() == before
    certificate = result.attempts[0].certificate
    assert certificate is not None and certificate.identity == result.identity
    assert result.geometry is geometry
    arrays = [result.solutions, result.solution_indices, geometry.dh, geometry.senses,
              geometry.joint_limits, result.target.quaternion, result.target.position,
              certificate.command_center_rad, certificate.increment_center_rad, certificate.preconditioner]
    for value in arrays:
        assert value.flags.owndata and not value.flags.writeable
        with pytest.raises(ValueError):
            value.flat[0] = 0
    with mp.workdps(160):
        _check_calibrated_certificate({**case, "x": list(certificate.increment_center_rad)}, proof(certificate))


def test_calibrated_python_fk_and_body_jacobian(kinematics_lib):
    case = case_input()
    commands = np.degrees(np.array(case["x"])*np.array(case["senses"]))
    evaluated = {**case, "x": list(np.radians(commands)*np.array(case["senses"]))}
    with CrxSolver("CRX-10iA", dll_path=kinematics_lib._name) as nominal:
        with solver(nominal, case) as calibrated:
            actual, jacobian = calibrated.fk(commands), calibrated.jacobian(commands)
            with mp.workdps(110):
                expected, derivative = _oracle(evaluated)
            expected = np.array(expected.tolist(), dtype=float)
            expected[:3, 3] *= case["scale"]
            derivative = np.array(derivative.tolist(), dtype=float)*np.array(case["senses"])
            derivative[:3] *= case["scale"]
            np.testing.assert_allclose(actual, expected, atol=5e-10, rtol=0)
            np.testing.assert_allclose(jacobian, derivative, atol=5e-10, rtol=0)


def test_calibrated_python_proofs_limits_and_numerics_are_separate(kinematics_lib):
    case = case_input()
    with CrxSolver("CRX-10iA", dll_path=kinematics_lib._name) as nominal:
        with solver(nominal, case) as calibrated:
            frame, proposed = target(case), seeds(case)
            exact = calibrated.correct(frame, proposed)
            numerical = calibrated.correct(frame, proposed, options=CalibratedOptions(certify=False))
            assert exact.solutions.tobytes() == numerical.solutions.tobytes()
            assert exact.attempts[0].commands_rad.tobytes() == numerical.attempts[0].commands_rad.tobytes()
            assert not numerical.proofs_requested and numerical.attempts[0].certificate is None
            loose = calibrated.correct(frame, proposed, options=CalibratedOptions(
                normalized_position_tolerance=.1, orientation_tolerance_rad=.1))
            assert loose.numerical_count == 1 and not loose.success and loose.local_count == 0
            assert loose.coverage == Coverage.INCONCLUSIVE
            exhausted = calibrated.correct(frame, proposed, options=CalibratedOptions(max_iterations=0))
            assert not exhausted.success and exhausted.attempts[0].status == CalibratedStatus.ITERATION_BUDGET
            empty = calibrated.correct(frame, [])
            assert empty.solutions.shape == (0, 6) and not empty.attempts
            assert empty.coverage == Coverage.INCONCLUSIVE and not empty.success
        limits = np.array([case["lower"], case["upper"]])
        limits[1, 0] = case["senses"][0]*case["x"][0]-.01
        with solver(nominal, case, joint_limits=limits) as calibrated:
            outside = calibrated.correct(target(case), seeds(case))
            assert outside.local_count == 1 and outside.admitted_count == 0 and not outside.success
            assert outside.attempts[0].certificate is not None
            assert outside.coverage == Coverage.INCONCLUSIVE


@pytest.mark.parametrize("quaternion", [[0., 1., 0., 0.], [0., 0., 1., 0.],
                                        [0., 0., 0., 1.], [2., .3, -.5, .1]])
def test_calibrated_frame_matrix_conversion_is_explicit_and_owned(quaternion):
    frame = CalibratedFrame(quaternion, [100., -200., 300.])
    matrix = frame.as_matrix()
    before = matrix.tobytes()
    recovered = CalibratedFrame.from_matrix(matrix)
    matrix[0, 3] = 9999.
    assert recovered.position[0] == 100.
    np.testing.assert_allclose(recovered.as_matrix(), frame.as_matrix(), atol=2e-14, rtol=0)
    assert len(before) == 16*8
    with pytest.raises(ValueError):
        CalibratedFrame([0.]*4)


@pytest.mark.parametrize("bad", [dict(max_iterations=True), dict(max_iterations=129),
    dict(max_backtracks=33), dict(maximum_step_rad=0), dict(orientation_tolerance_rad=-1),
    dict(normalized_position_tolerance=math.nan), dict(certify=1)])
def test_calibrated_options_reject_invalid_inputs(bad):
    with pytest.raises((TypeError, ValueError)):
        CalibratedOptions(**bad)


def test_calibrated_python_validates_geometry_and_optional_symbols(kinematics_lib, monkeypatch):
    case = case_input()
    with CrxSolver("CRX-10iA", dll_path=kinematics_lib._name) as nominal:
        for bad in [dict(senses=[1., 1., 1., 1., 1., .99999]), dict(length_scale=0),
                    dict(alpha_quarters=[0., 0., .5, 0., 0., 0.]), dict(limit_units="turns")]:
            with pytest.raises((TypeError, ValueError)):
                solver(nominal, case, **bad)
        original = nominal._library.lib

        class LegacySymbols:
            def __getattr__(self, name):
                if name.startswith("crx_cal_v1_"):
                    raise AttributeError(name)
                return getattr(original, name)

        monkeypatch.setattr(nominal._library, "lib", LegacySymbols())
        with pytest.raises(NotImplementedError, match="calibrated ABI"):
            solver(nominal, case)
        assert np.isfinite(nominal.fk([0.]*6)).all()


@pytest.mark.parametrize("asset", ASSETS, ids=lambda asset: asset["asset"])
def test_calibrated_python_automatic_nominal_seed_workflow(kinematics_lib, asset):
    raw = derive_model(asset)
    rows = [[0, float(a)+(i-2.5)*1e-5, float(b)+(i+1)*.005,
             0, float(t)+(2.5-i)*1e-5, float(d)+(i-3)*.004]
            for i, (a, b, t, d) in enumerate(raw["dh"])]
    case = dict(scale=float(np.max(np.abs(raw["dh"][:, [1, 3]]))), rows=rows,
        senses=[1., 1., -1., -1., -1., -1.],
        base=[1., 0., 0., 0., *map(float, raw["base"][:3, 3])],
        tool=[1., 0., 0., 0., *map(float, raw["tool"][:3, 3])],
        lower=[-180.]*6, upper=[180.]*6, limit_units=1,
        x=list(np.radians([30., -40., 45., 50., -35., 20.])*np.array([1., 1., -1., -1., -1., -1.])))
    with mp.workdps(160):
        case["target"] = _target_at(case)
    name = asset["asset"].removeprefix("Fanuc-").removesuffix("-Custom.robot")
    with CrxSolver(name, dll_path=kinematics_lib._name, joint_limits=[[-180.]*6, [180.]*6]) as nominal:
        with solver(nominal, case) as calibrated:
            result = calibrated.ik(target(case), max_candidates=64)
            assert result.success and result.nominal_count > 1
            assert len(result.attempts) == result.nominal_count
            assert result.coverage == Coverage.INCONCLUSIVE
            for i in result.solution_indices:
                attempt = result.attempts[i]
                assert attempt.certificate is not None and attempt.certificate.commands_admitted
                residual_case = {**case, "x": list(attempt.increments_rad)}
                with mp.workdps(110):
                    pose, _ = _oracle(residual_case)
                    expected = np.array(pose.tolist(), dtype=float)
                    expected[:3, 3] *= case["scale"]
                np.testing.assert_allclose(expected, target(case).as_matrix(), atol=2e-9, rtol=0)


def test_calibrated_python_concurrent_snapshot_and_close(kinematics_lib):
    case = case_input()
    with CrxSolver("CRX-10iA", dll_path=kinematics_lib._name) as nominal:
        calibrated = solver(nominal, case)
        frame, proposed = target(case), seeds(case)
        retained = [calibrated.correct(frame, proposed)]
        with ThreadPoolExecutor(max_workers=4) as pool:
            jobs = [pool.submit(calibrated.correct, frame, proposed) for _ in range(6)]
            closed = pool.submit(calibrated.close)
            for job in jobs:
                try:
                    retained.append(job.result())
                except ClosedSolverError:
                    pass
            closed.result()
        for result in retained:
            assert result.success and np.isfinite(result.solutions).all()
            assert not result.solutions.flags.writeable
        calibrated.close()
        with pytest.raises(ClosedSolverError):
            calibrated.correct(frame, proposed)


def test_calibrated_python_identity_is_a_raw_value_not_object_identity(kinematics_lib):
    case = case_input()
    with CrxSolver("CRX-10iA", dll_path=kinematics_lib._name) as nominal:
        with solver(nominal, case) as calibrated:
            first = calibrated.correct(target(case), seeds(case))
            second = calibrated.correct(target(case), seeds(case))
            assert first.identity == second.identity and first.identity is not second.identity
            assert hash(first.identity) == hash(second.identity)
            equivalent = CalibratedFrame(-target(case).quaternion, target(case).position)
            changed = calibrated.correct(equivalent, seeds(case))
            assert changed.success and changed.identity != first.identity
            assert changed.identity.model_words == first.identity.model_words


def test_calibrated_python_handle_cannot_be_copied(kinematics_lib):
    case = case_input()
    with CrxSolver("CRX-10iA", dll_path=kinematics_lib._name) as nominal:
        with solver(nominal, case) as calibrated:
            for operation in [copy.copy, copy.deepcopy]:
                with pytest.raises(TypeError, match="native handle"):
                    operation(calibrated)
            assert calibrated.correct(target(case), seeds(case)).success


def test_calibrated_explicit_correction_owns_model_after_nominal_close(kinematics_lib):
    case = case_input()
    nominal = CrxSolver("CRX-10iA", dll_path=kinematics_lib._name)
    try:
        with solver(nominal, case) as calibrated:
            nominal.close()
            assert calibrated.correct(target(case), seeds(case)).success
            assert np.isfinite(calibrated.fk(seeds(case)[0])).all()
            with pytest.raises(ClosedSolverError):
                calibrated.ik(target(case))
    finally:
        nominal.close()


def test_documented_zero_perturbation_geometry_matches_nominal_api(kinematics_lib):
    dh = np.zeros((6, 4))
    dh[0, 3], dh[2, 1], dh[3, 3], dh[4, 3], dh[5, 3] = 245, 540, 540, -150, 160
    with CrxSolver("CRX-10iA", dll_path=kinematics_lib._name) as nominal:
        with CalibratedSolver(nominal, dh, alpha_quarters=[0, -1, 0, -1, 1, -1],
                              theta_quarters=[0, -1, 0, 0, 0, 0],
                              base=CalibratedFrame(position=[0, 0, -245])) as calibrated:
            for commands in [[0.]*6, [30., -40., 45., 50., -35., 20.],
                             [-25., 30., -45., -40., 35., -20.]]:
                np.testing.assert_allclose(calibrated.fk(commands), nominal.fk(commands),
                                           atol=2e-10, rtol=0)
