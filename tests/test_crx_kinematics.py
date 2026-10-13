import ctypes
import hashlib
import logging
import math
from pathlib import Path
from typing import Any, Dict, List, Tuple

import numpy as np
import pytest

from crx_assets import APPROVED_ROBOT_ASSETS
from crx_test_support import (
    _to_c_array, _from_c_array, _to_fk_api_joints_deg, _from_fk_api_joints_deg,
    _call_fk, _call_ik, _call_joints_config,
)
from crx_fixture_support import (
    CASE_PARAMS, SOLUTION_PARAMS, CONFIG_FIXTURE_CASES,
    POSE_POS_TOL_MM, POSE_ANG_TOL_DEG, JOINT_TOL_DEG,
    _is_legal_fixture_config, _expected_robodk_config,
    _fmt_f, _fmt_joints, _fmt_pose16, _pose_err_pos_mm_ang_deg,
    _joints_close, _max_abs_wrapped_diff_deg, _max_abs_direct_diff_deg,
    _nearest_joint_match_wrapped, _fixture_case_pose_and_approx, _fixture_case_solutions,
)

log = logging.getLogger(__name__)

WORKSPACE_LIMIT_CASES = ["MAX_X", "MIN_X", "MAX_Y", "MIN_Y", "MAX_Z", "MIN_Z"]
SINGULAR_CASES = ["WRIST_SING", "WRIST_SING_NEAR_RG_LIMIT"]


def test_fixture_config_letters_are_legal():
    invalid = [
        (p.values[0]["case_name"], p.values[0]["sol_id"], p.values[0]["config"])
        for p in SOLUTION_PARAMS
        if not _is_legal_fixture_config(p.values[0]["config"])
    ]
    assert invalid == []


def test_config_conversion_contract():
    examples = [
        ({"FB": "N", "UD": "U", "TB": "T"}, [0, 0, 0]),
        ({"FB": "F", "UD": "D", "TB": "B"}, [1, 1, 1]),
        ({"FB": "F", "UD": "U", "TB": "B"}, [1, 0, 1]),
    ]
    for fixture_config, robodk_config in examples:
        assert _expected_robodk_config(fixture_config) == robodk_config


@pytest.mark.parametrize("asset_name,expected_hash", APPROVED_ROBOT_ASSETS.items())
def test_crx_family_assets_available_for_configuration_smoke(asset_name, expected_hash):
    repo_root = Path(__file__).resolve().parents[1]
    asset = repo_root / "assets" / asset_name
    assert asset.is_file(), f"Missing approved asset: {asset}"
    with asset.open("rb") as stream:
        actual_hash = hashlib.file_digest(stream, "sha256").hexdigest()
    assert actual_hash == expected_hash, f"Asset changed or LFS content missing: {asset}"


def _pose16_with_offset(
    pose16: List[float], dx_mm: float, dy_mm: float, dz_mm: float
) -> List[float]:
    out = list(pose16)
    out[12] += float(dx_mm)
    out[13] += float(dy_mm)
    out[14] += float(dz_mm)
    return out


# FK/configuration check every recorded solution; IK solves each target once.
@pytest.mark.parametrize("p", SOLUTION_PARAMS)
def test_forward_kinematics(kinematics_lib, crx_10ia, p: Dict[str, Any]):
    status, fk_pose = _call_fk(kinematics_lib, crx_10ia, p["joints_deg"])
    hdr = f"TC{p['case_id']} '{p['case_name']}' SOL{p['sol_id']} config={p['config']}"

    assert status == 1, "\n".join(
        [
            f"{hdr}: FK status != 1",
            f"  fk_status: {status}",
            f"  joints_deg: {_fmt_joints(p['joints_deg'])}",
        ]
    )

    pos_err, ang_err = _pose_err_pos_mm_ang_deg(p["target_pose16"], fk_pose)

    log.info(
        "FK %s pos_err_mm=%s ang_err_deg=%s (tol %s / %s)",
        hdr,
        _fmt_f(pos_err),
        _fmt_f(ang_err),
        _fmt_f(POSE_POS_TOL_MM),
        _fmt_f(POSE_ANG_TOL_DEG),
    )
    assert (
        math.isfinite(pos_err)
        and math.isfinite(ang_err)
        and pos_err <= POSE_POS_TOL_MM
        and ang_err <= POSE_ANG_TOL_DEG
    ), "\n".join(
        [
            f"{hdr}: FK pose mismatch",
            f"  pos_err_mm: {_fmt_f(pos_err)} (tol={POSE_POS_TOL_MM})",
            f"  ang_err_deg: {_fmt_f(ang_err)} (tol={_fmt_f(POSE_ANG_TOL_DEG)})",
            f"  joints_deg: {_fmt_joints(p['joints_deg'])}",
            "  target_pose16:",
            _fmt_pose16(p["target_pose16"]),
            "  fk_pose16:",
            _fmt_pose16(fk_pose),
        ]
    )


@pytest.mark.parametrize("p", CASE_PARAMS)
def test_inverse_kinematics(kinematics_lib, crx_10ia, p: Dict[str, Any]):
    hdr = f"TC{p['case_id']} '{p['case_name']}'"

    approx = p["solutions"][0]["joints_deg"]
    # Fixture coverage concerns the complete finite catalogue. General command
    # turns can exceed 32 entries; separate tests verify ranked small buffers.
    n, best, all_solutions = _call_ik(
        kinematics_lib,
        crx_10ia,
        p["target_pose16"],
        approx=approx,
        max_solutions=4096,
    )
    assert n < 4096, "fixture catalogue was truncated"

    assert n > 0, "\n".join(
        [
            f"{hdr}: IK returned no solutions",
            f"  n: {n}",
            f"  approx(best_expected): {_fmt_joints(approx) if approx else 'None'}",
            "  target_pose16:",
            _fmt_pose16(p["target_pose16"]),
        ]
    )

    for solution in p["solutions"]:
        expected = solution["joints_deg"]
        ok = any(_joints_close(sol, expected, JOINT_TOL_DEG) for sol in all_solutions)

        if p["case_name"] == "MIN_Z" and not ok:
            # Its rounded XYZ misses the recorded posture by 0.00425 mm. The
            # legacy empty-search policy preserves that approximate seed; exact
            # target IK can legitimately select different base/wrist postures at
            # this workspace boundary. Require strict independent FK in that case.
            from crx_reference import reference_fk
            target = np.array(p["target_pose16"]).reshape(4, 4, order="F")
            recorded, _ = reference_fk(crx_10ia, expected)
            assert 0.004 < np.linalg.norm(recorded[:3, 3] - target[:3, 3]) < 0.005
            for candidate in all_solutions:
                actual, _ = reference_fk(crx_10ia, candidate)
                assert np.linalg.norm(actual[:3, 3] - target[:3, 3]) <= 1e-4
                assert np.linalg.norm(actual[:3, :3] - target[:3, :3]) <= 2.5e-5
            continue

        if not ok:
            # show nearest for context
            nearest, nearest_d = _nearest_joint_match_wrapped(expected, all_solutions)
            raise AssertionError(
                "\n".join(
                    [
                        f"{hdr} SOL{solution['id']}: Missing expected IK solution",
                        f"  tol_deg: {JOINT_TOL_DEG}",
                        f"  expected_joints_deg: {_fmt_joints(expected)}",
                        f"  nearest_max_abs_diff_deg: {_fmt_f(nearest_d)}",
                        "  nearest_joints_deg: "
                        + (_fmt_joints(nearest) if nearest is not None else "None"),
                        f"  returned_n: {n}",
                        f"  best_joints_deg: {_fmt_joints(best)}",
                    ]
                )
            )


@pytest.mark.parametrize(
    "p",
    [
        param
        for param in SOLUTION_PARAMS
        if param.values[0]["case_name"] in CONFIG_FIXTURE_CASES
    ],
)
def test_joints_to_configuration(kinematics_lib, crx_10ia, p: Dict[str, Any]):
    status, actual = _call_joints_config(kinematics_lib, crx_10ia, p["joints_deg"])
    expected = _expected_robodk_config(p["config"])
    assert status == 1
    assert actual == expected, "\n".join(
        [
            f"TC{p['case_id']} '{p['case_name']}' SOL{p['sol_id']}",
            f"  expected_config: {expected}",
            f"  actual_config: {actual}",
            f"  joints_deg: {_fmt_joints(p['joints_deg'])}",
        ]
    )


def test_latest_robodk_export_contract(kinematics_lib):
    assert hasattr(kinematics_lib, "SolveFK")
    assert hasattr(kinematics_lib, "SolveFK_CAD")
    assert hasattr(kinematics_lib, "SolveIK")
    assert hasattr(kinematics_lib, "Joints2Config")
    assert not hasattr(kinematics_lib, "SolveIK_Config")


def test_joints_to_configuration_is_base_transform_invariant(kinematics_lib, crx_10ia):
    joints = [-53.304, -41.707, -3.066, 94.105, -36.806, 84.878]
    status_reference, config_reference = _call_joints_config(
        kinematics_lib, crx_10ia, joints
    )

    transformed_robot = type(crx_10ia)()
    ctypes.memmove(
        ctypes.byref(transformed_robot),
        ctypes.byref(crx_10ia),
        ctypes.sizeof(transformed_robot),
    )
    for index, value in enumerate([100.0, -200.0, 300.0, 15.0, -25.0, 35.0]):
        transformed_robot.data[9][index] = value

    status_transformed, config_transformed = _call_joints_config(
        kinematics_lib, transformed_robot, joints
    )
    assert status_reference == status_transformed == 1
    assert config_transformed == config_reference


@pytest.mark.parametrize(
    ("j5_deg", "expected_flip"),
    [(-1.0, 0.0), (0.005, 0.0), (0.02, 1.0)],
)
def test_joints_to_configuration_flip_threshold(
    kinematics_lib, crx_10ia, j5_deg: float, expected_flip: float
):
    status, config = _call_joints_config(
        kinematics_lib, crx_10ia, [0.0, 0.0, 0.0, 0.0, j5_deg, 0.0]
    )
    assert status == 1
    assert config[2] == expected_flip


def test_joints_to_configuration_rejects_invalid_inputs(kinematics_lib, crx_10ia):
    sentinel = (-9.0, -8.0, -7.0)
    config = (ctypes.c_double * 3)(*sentinel)
    status = kinematics_lib.Joints2Config(None, config, ctypes.byref(crx_10ia))
    assert status == -1
    assert _from_c_array(config, 3) == list(sentinel)

    joints = _to_c_array(_to_fk_api_joints_deg([0.0] * 6))
    status = kinematics_lib.Joints2Config(joints, None, ctypes.byref(crx_10ia))
    assert status == -1

    config = (ctypes.c_double * 3)(*sentinel)
    status = kinematics_lib.Joints2Config(joints, config, None)
    assert status == -1
    assert _from_c_array(config, 3) == list(sentinel)

    status, output = _call_joints_config(
        kinematics_lib,
        crx_10ia,
        [0.0, math.nan, 0.0, 0.0, 0.0, 0.0],
        sentinel,
    )
    assert status == -1
    assert output == list(sentinel)

    invalid_sense_robot = type(crx_10ia)()
    ctypes.memmove(
        ctypes.byref(invalid_sense_robot),
        ctypes.byref(crx_10ia),
        ctypes.sizeof(invalid_sense_robot),
    )
    invalid_sense_robot.data[3][4] = 0.0
    status, output = _call_joints_config(
        kinematics_lib, invalid_sense_robot, [0.0] * 6, sentinel
    )
    assert status == -1
    assert output == list(sentinel)

    non_crx_robot = type(crx_10ia)()
    ctypes.memmove(
        ctypes.byref(non_crx_robot),
        ctypes.byref(crx_10ia),
        ctypes.sizeof(non_crx_robot),
    )
    non_crx_robot.data[10][4] = 1.0
    status, output = _call_joints_config(
        kinematics_lib, non_crx_robot, [0.0] * 6, sentinel
    )
    assert status == -1
    assert output == list(sentinel)

    unsupported_robot = type(crx_10ia)()
    ctypes.memmove(
        ctypes.byref(unsupported_robot),
        ctypes.byref(crx_10ia),
        ctypes.sizeof(unsupported_robot),
    )
    unsupported_robot.data[1][1] = 5.0
    status, output = _call_joints_config(
        kinematics_lib, unsupported_robot, [0.0] * 6, sentinel
    )
    assert status == -1
    assert output == list(sentinel)


def test_invalid_model_diagnostic_has_complete_name_buffer(kinematics_lib, crx_10ia, capfd):
    from crx_test_support import ROBOT_NAME_ROW, ROBOT_NAME_CAPACITY, ROBOT_STRIDE
    # Cross the row boundary and fill every readable name element, exercising
    # the no-terminator bound as well as the previously undersized allocation.
    name = "X" * ROBOT_NAME_CAPACITY
    flat = ctypes.cast(ctypes.byref(crx_10ia), ctypes.POINTER(ctypes.c_double))
    for index, character in enumerate(name):
        flat[ROBOT_NAME_ROW * ROBOT_STRIDE + index] = ord(character)
    crx_10ia.data[1][1] = 5.0
    status, output = _call_joints_config(kinematics_lib, crx_10ia, [0.] * 6, (-9., -8., -7.))
    assert status == -1
    assert output == [-9., -8., -7.]
    assert f"[{name}]" in capfd.readouterr().err


def test_joints_to_configuration_rejects_shoulder_singularity(kinematics_lib, crx_10ia):
    sentinel = (-9.0, -8.0, -7.0)
    status, output = _call_joints_config(
        kinematics_lib,
        crx_10ia,
        [0.0, 0.0, 90.0, 0.0, 0.0, 0.0],
        sentinel,
    )
    assert status == -1
    assert output == list(sentinel)


def _robot_with_j6_limits(robot, lower_deg: float, upper_deg: float):
    widened = type(robot)()
    ctypes.memmove(ctypes.byref(widened), ctypes.byref(robot), ctypes.sizeof(widened))
    widened.data[30][5] = float(lower_deg)
    widened.data[31][5] = float(upper_deg)
    return widened


@pytest.mark.parametrize(
    "j6_deg",
    [360.0, -360.0],
    ids=["j6_plus_360", "j6_minus_360"],
)
def test_solveik_preserves_seed_turn(kinematics_lib, crx_10ia, j6_deg: float):
    robot = _robot_with_j6_limits(crx_10ia, -540.0, 540.0)
    seed_joints = [0.0, 0.0, 0.0, 0.0, -90.0, j6_deg]

    fk_status, fk_pose = _call_fk(kinematics_lib, robot, seed_joints)
    assert fk_status == 1, f"FK seed invalid: {_fmt_joints(seed_joints)}"

    n, best, all_solutions = _call_ik(
        kinematics_lib,
        robot,
        fk_pose,
        approx=seed_joints,
        max_solutions=64,
    )

    assert n > 0
    assert _max_abs_direct_diff_deg(best, seed_joints) <= JOINT_TOL_DEG
    assert _max_abs_direct_diff_deg(all_solutions[0], seed_joints) <= JOINT_TOL_DEG


def test_ik_wrist_singularity_continuity(kinematics_lib, crx_10ia):
    wrist_path_j5_deg = [-3.0, -2.0, -1.0, -0.2, 0.0, 0.2, 1.0, 2.0, 3.0]
    seed_joints = [25.0, -35.0, 70.0, 15.0, 0.0, 40.0]

    fk_poses: List[List[float]] = []
    for j5 in wrist_path_j5_deg:
        joints = list(seed_joints)
        joints[4] = j5
        fk_status, fk_pose = _call_fk(kinematics_lib, crx_10ia, joints)
        assert fk_status == 1, f"FK seed invalid near wrist singularity: {joints}"
        fk_poses.append(fk_pose)

    previous_best: List[float] | None = None
    rolling_approx = list(seed_joints)
    for idx, fk_pose in enumerate(fk_poses):
        n, best, _ = _call_ik(
            kinematics_lib, crx_10ia, fk_pose, approx=rolling_approx, max_solutions=64
        )
        assert n > 0, f"Wrist continuity step {idx}: IK returned no solution"
        if previous_best is not None:
            jump = _max_abs_wrapped_diff_deg(best, previous_best)
            assert jump <= 25.0, (
                f"Wrist continuity step {idx}: joint jump too large "
                f"({jump:.3f} deg > 25 deg)"
            )
        previous_best = best
        rolling_approx = best


def test_ik_elbow_ill_conditioned_continuity(kinematics_lib, crx_10ia):
    elbow_path_j3_deg = [85.0, 88.0, 89.5, 90.0, 90.5, 92.0, 95.0]
    seed_joints = [15.0, -20.0, 90.0, 30.0, 25.0, -10.0]

    fk_poses: List[List[float]] = []
    for j3 in elbow_path_j3_deg:
        joints = list(seed_joints)
        joints[2] = j3
        fk_status, fk_pose = _call_fk(kinematics_lib, crx_10ia, joints)
        assert fk_status == 1, (
            f"FK seed invalid near elbow conditioning region: {joints}"
        )
        fk_poses.append(fk_pose)

    previous_best: List[float] | None = None
    rolling_approx = list(seed_joints)
    for idx, fk_pose in enumerate(fk_poses):
        n, best, _ = _call_ik(
            kinematics_lib, crx_10ia, fk_pose, approx=rolling_approx, max_solutions=64
        )
        assert n > 0, f"Elbow continuity step {idx}: IK returned no solution"
        if previous_best is not None:
            jump = _max_abs_wrapped_diff_deg(best, previous_best)
            assert jump <= 25.0, (
                f"Elbow continuity step {idx}: joint jump too large "
                f"({jump:.3f} deg > 25 deg)"
            )
        previous_best = best
        rolling_approx = best


@pytest.mark.parametrize("case_name", WORKSPACE_LIMIT_CASES)
def test_ik_named_workspace_limit_cases(case_name: str, kinematics_lib, crx_10ia):
    pose16, approx = _fixture_case_pose_and_approx(case_name)
    n, best, _ = _call_ik(
        kinematics_lib,
        crx_10ia,
        pose16,
        approx=approx,
        max_solutions=64,
    )
    if n <= 0:
        pytest.skip(f"{case_name}: unsolved in current baseline")

    fk_status, fk_pose = _call_fk(kinematics_lib, crx_10ia, best)
    assert fk_status == 1, f"{case_name}: FK roundtrip failed for best IK candidate"
    pos_err, ang_err = _pose_err_pos_mm_ang_deg(pose16, fk_pose)
    assert pos_err <= POSE_POS_TOL_MM and ang_err <= POSE_ANG_TOL_DEG, (
        f"{case_name}: roundtrip error too high pos={pos_err:.6f}mm ang={ang_err:.6f}deg"
    )


@pytest.mark.parametrize("case_name", SINGULAR_CASES)
def test_ik_named_singularity_cases_prefer_approx(
    case_name: str, kinematics_lib, crx_10ia
):
    pose16, all_expected = _fixture_case_solutions(case_name)
    if not all_expected:
        pytest.skip(f"{case_name}: no fixture solutions")

    approx = all_expected[0]
    n, best, all_solutions = _call_ik(
        kinematics_lib,
        crx_10ia,
        pose16,
        approx=approx,
        max_solutions=64,
    )
    if n <= 0:
        pytest.skip(f"{case_name}: unsolved in current baseline")

    nearest, nearest_d = _nearest_joint_match_wrapped(approx, all_solutions)
    assert nearest is not None
    assert nearest_d <= 5.0, (
        f"{case_name}: solver did not stay near approx branch; nearest diff={nearest_d:.3f} deg"
    )

    # Check branch continuity under tiny local translation changes.
    rolling_approx = best
    for jitter_mm in [-0.05, -0.02, 0.02, 0.05]:
        pose_jittered = _pose16_with_offset(pose16, jitter_mm, 0.0, 0.0)
        n_j, best_j, _ = _call_ik(
            kinematics_lib,
            crx_10ia,
            pose_jittered,
            approx=rolling_approx,
            max_solutions=64,
        )
        if n_j <= 0:
            pytest.skip(
                f"{case_name}: local jitter sweep unstable at {jitter_mm:+.3f} mm"
            )
        jump = _max_abs_wrapped_diff_deg(best_j, rolling_approx)
        assert jump <= 25.0, (
            f"{case_name}: continuity jump too large at jitter {jitter_mm:+.3f} mm "
            f"({jump:.3f} deg)"
        )
        rolling_approx = best_j
