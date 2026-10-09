"""Seed-free discovery gates using derived legacy-model targets (#2/#19 A).

Each generating posture is a known witness, not a complete IK catalogue. These
tests neither count polynomial roots nor certify coverage of continuous fibres.
"""

import ctypes

import numpy as np
import pytest

from crx_reference import missing_postures, reference_fk
from test_crx_kinematics import _call_fk, _call_ik, _to_c_array, _to_fk_api_joints_deg


class MissingPostureError(AssertionError):
    """Only a missing witness is covered by the recorded legacy exceptions."""


# Reproduced against the pre-change DLL. Strict XPASS requires removing the
# exception when discovery is fixed; other failures are never swallowed.
KNOWN_MISSING_POSTURES = {8, 36, 42, 57}
DISCOVERY_CASES = [
    (
        pytest.param(
            case_id,
            marks=pytest.mark.xfail(
                strict=True,
                raises=MissingPostureError,
                reason="#2/#8/#17: legacy IK misses this seed-free witness",
            ),
        )
        if case_id in KNOWN_MISSING_POSTURES else case_id
    )
    for case_id in range(64)
]


def discovery_joints(case_id):
    # Deterministic derived probes, away from command turn boundaries.
    # The legacy callback fixture remains the model input until asset extraction.
    return np.random.default_rng(case_id).uniform(-150.0, 150.0, size=6).tolist()


@pytest.mark.parametrize("case_id", DISCOVERY_CASES)
def test_seed_free_discovery(kinematics_lib, crx_10ia, case_id):
    witness = discovery_joints(case_id)
    target, _ = reference_fk(crx_10ia, witness)
    count, _, solutions = _call_ik(
        kinematics_lib,
        crx_10ia,
        target.ravel(order="F").tolist(),
        approx=None,
        max_solutions=64,
    )
    assert count > 0, f"case {case_id}: no seed-free result"
    for solution in solutions:
        actual, _ = reference_fk(crx_10ia, solution)
        np.testing.assert_allclose(actual[:3, 3], target[:3, 3], atol=1e-4, rtol=0)
        np.testing.assert_allclose(actual[:3, :3], target[:3, :3], atol=2e-5, rtol=0)
    if missing_postures([witness], solutions):
        raise MissingPostureError(
            f"case {case_id}: missing generating posture {witness}; returned {count}"
        )


@pytest.mark.parametrize("case_id", sorted(KNOWN_MISSING_POSTURES))
def test_generating_seed_masks_missing_discovery(kinematics_lib, crx_10ia, case_id):
    witness = discovery_joints(case_id)
    target, _ = reference_fk(crx_10ia, witness)
    count, _, solutions = _call_ik(
        kinematics_lib,
        crx_10ia,
        target.ravel(order="F").tolist(),
        approx=witness,
        max_solutions=64,
    )
    assert count > 0
    assert not missing_postures([witness], solutions)


def test_missing_branch_detector():
    # A deliberately omitted branch must fail even if another posture remains.
    expected = [[0.0] * 6, [0.0, 30.0, 60.0, 90.0, 0.0, 0.0]]
    assert missing_postures(expected, expected) == []
    assert missing_postures(expected, [expected[0]]) == [expected[1]]
    assert missing_postures(expected, []) == expected
    lifted = [[angle + 360 for angle in posture] for posture in expected]
    assert missing_postures(expected, lifted) == []


@pytest.mark.parametrize("case_id", range(8))
def test_selection_with_independent_seed(kinematics_lib, crx_10ia, case_id):
    witness = discovery_joints(case_id)
    target, _ = reference_fk(crx_10ia, witness)
    independent_seed = discovery_joints(case_id + 64)
    seed_pose, _ = reference_fk(crx_10ia, independent_seed)
    assert np.linalg.norm(seed_pose[:3, 3] - target[:3, 3]) > 1.0
    count, best, solutions = _call_ik(
        kinematics_lib,
        crx_10ia,
        target.ravel(order="F").tolist(),
        approx=independent_seed,
        max_solutions=64,
    )
    assert count > 0
    assert best == solutions[0]
    assert not missing_postures([witness], solutions)
    for solution in solutions:
        actual, _ = reference_fk(crx_10ia, solution)
        np.testing.assert_allclose(actual[:3, 3], target[:3, 3], atol=1e-4, rtol=0)
        np.testing.assert_allclose(actual[:3, :3], target[:3, :3], atol=2e-5, rtol=0)


@pytest.mark.parametrize("case_id", range(8))
@pytest.mark.parametrize("transformed", [False, True])
def test_reference_fk_and_all_cad_frames(kinematics_lib, crx_10ia, case_id, transformed):
    if transformed:
        # Derived test transforms/signs; no claim that an asset has these values.
        crx_10ia.data[9][:6] = [125, -37, 92, 0.3, -0.5, 0.7]
        crx_10ia.data[28][:6] = [-15, 24, 33, -0.2, 0.4, -0.6]
        crx_10ia.data[3][4:10] = [-1, 1, 1, -1, 1, -1]
    joints = discovery_joints(case_id)
    expected, frames = reference_fk(crx_10ia, joints)
    status, actual = _call_fk(kinematics_lib, crx_10ia, joints)
    assert status == 1
    np.testing.assert_allclose(
        np.array(actual).reshape(4, 4, order="F"), expected, atol=1e-10, rtol=0,
    )
    pose = (ctypes.c_double * 16)()
    # Sentinels after the seventh frame detect writes beyond the CAD contract.
    output = (ctypes.c_double * (16 * 8))(*([12345.0] * (16 * 8)))
    status = kinematics_lib.SolveFK_CAD(
        _to_c_array(_to_fk_api_joints_deg(joints)),
        pose,
        output,
        8,
        ctypes.byref(crx_10ia),
    )
    assert status == 1
    np.testing.assert_allclose(
        np.array(pose).reshape(4, 4, order="F"), expected, atol=1e-10, rtol=0,
    )
    for index, frame in enumerate(frames):
        actual_frame = np.array(output[index * 16:(index + 1) * 16]).reshape(
            4, 4, order="F",
        )
        np.testing.assert_allclose(actual_frame, frame, atol=1e-10, rtol=0)
    assert list(output[16 * 7:]) == [12345.0] * 16


@pytest.mark.parametrize("capacity", [-1, 0, 6])
def test_cad_rejects_short_buffer_without_writing(kinematics_lib, crx_10ia, capacity):
    pose = (ctypes.c_double * 16)(*([12345.0] * 16))
    output = (ctypes.c_double * (16 * 7))(*([12345.0] * (16 * 7)))
    status = kinematics_lib.SolveFK_CAD(
        _to_c_array([0.0] * 6),
        pose,
        output,
        capacity,
        ctypes.byref(crx_10ia),
    )
    assert status == -1
    assert list(pose) == [12345.0] * 16
    assert list(output) == [12345.0] * (16 * 7)
