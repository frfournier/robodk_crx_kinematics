"""Numerical model guards must not silently absorb axis calibration."""

import ctypes

import numpy as np
import pytest

from crx_reference import reference_fk
from crx_test_support import _call_joints_config, _to_c_array, _to_fk_api_joints_deg


@pytest.mark.parametrize("column", [0, 2], ids=["alpha", "theta_offset"])
@pytest.mark.parametrize("tilt", [-9.9e-6, -1e-8, -1e-14, 1e-14, 1e-8, 9.9e-6])
def test_dh_snapping_is_roundoff_only(solver_lib, crx_10ia, column, tilt):
    witness = [20., -40., 45., 30., -50., -60.]
    crx_10ia.data[11][column] += tilt
    target, _ = reference_fk(crx_10ia, witness)
    command = _to_c_array(_to_fk_api_joints_deg(witness))
    actual = (ctypes.c_double * 16)()
    assert solver_lib.SolveFK(command, actual, ctypes.byref(crx_10ia)) == 1
    # FK must retain actual nonsnapped geometry. This reference uses raw DH
    # transform products and never calls the library's snapping helper.
    np.testing.assert_allclose(np.array(actual).reshape(4, 4, order="F"),
                               target, atol=2e-10, rtol=0)
    best = (ctypes.c_double * 6)(*([12345.] * 6))
    alternatives = (ctypes.c_double * 12)(*([12345.] * 12))
    count = solver_lib.SolveIK(_to_c_array(target.ravel(order="F")), best,
                                alternatives, 1, command, ctypes.byref(crx_10ia))
    if abs(tilt) > 1e-10:
        assert count == -1
        assert list(best) == [12345.] * 6
        assert list(alternatives) == [12345.] * 12
    else:
        assert count == 1


@pytest.mark.parametrize("axis", range(6))
@pytest.mark.parametrize("ulps", [-256, -64, -1, 1, 64, 256])
def test_joint_sense_roundoff(solver_lib, crx_10ia, axis, ulps):
    witness = [20., -40., 45., 30., -50., -60.]
    target, _ = reference_fk(crx_10ia, witness)
    command = _to_c_array(_to_fk_api_joints_deg(witness))
    crx_10ia.data[3][4 + axis] += ulps * np.finfo(float).eps
    actual = (ctypes.c_double * 16)()
    status = solver_lib.SolveFK(command, actual, ctypes.byref(crx_10ia))
    if abs(ulps) > 128:
        assert status == -1
        return
    assert status == 1
    # A sense encodes a sign. Admitted roundoff must not become a joint gain.
    np.testing.assert_allclose(np.array(actual).reshape(4, 4, order="F"),
                               target, atol=2e-10, rtol=0)
    best = (ctypes.c_double * 6)()
    assert solver_lib.SolveIK(_to_c_array(target.ravel(order="F")), best,
                               None, 1, command, ctypes.byref(crx_10ia)) == 1


@pytest.mark.parametrize("ulps", [-256, -64, -1, 1, 64, 256])
def test_revolute_flag_roundoff(solver_lib, crx_10ia, ulps):
    witness = [20., -40., 45., 30., -50., -60.]
    target, _ = reference_fk(crx_10ia, witness)
    command = _to_c_array(_to_fk_api_joints_deg(witness))
    crx_10ia.data[11][4] = ulps * np.finfo(float).eps
    best = (ctypes.c_double * 6)()
    count = solver_lib.SolveIK(_to_c_array(target.ravel(order="F")), best,
                                None, 1, command, ctypes.byref(crx_10ia))
    assert count == (1 if abs(ulps) < 128 else -1)


@pytest.mark.parametrize("ulps", [-256, -64, -1, 1, 64, 256])
def test_diagnostic_character_roundoff(solver_lib, crx_10ia, capfd, ulps):
    from crx_test_support import ROBOT_NAME_ROW
    eps = np.finfo(float).eps
    crx_10ia.data[ROBOT_NAME_ROW][0] = ord("A") * (1 + ulps * eps)
    crx_10ia.data[ROBOT_NAME_ROW][1] = eps
    crx_10ia.data[ROBOT_NAME_ROW][2] = ord("X")
    crx_10ia.data[1][1] = 5
    status, _ = _call_joints_config(solver_lib, crx_10ia, [0.] * 6,
                                   (-9., -8., -7.))
    assert status == -1
    message = capfd.readouterr().err
    assert ("[A]" in message) == (abs(ulps) < 128)
    assert "X" not in message
