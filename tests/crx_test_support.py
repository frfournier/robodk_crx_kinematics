"""Shared robot-table and C ABI plumbing; geometry references stay independent."""

import ctypes
from typing import List, Tuple


ROBOT_STRIDE = 20
# Adapter warnings read the ASCII name at row 90, across up to 59 doubles.
# A 32-row kinematics-only mock made invalid-input diagnostics read past its
# allocation, causing intermittent access violations as test layouts changed.
ROBOT_NAME_ROW = 90
ROBOT_NAME_CAPACITY = 59
ROBOT_ROWS = ROBOT_NAME_ROW + (ROBOT_NAME_CAPACITY + ROBOT_STRIDE - 1) // ROBOT_STRIDE
DOFS = 6


class RobotT(ctypes.Structure):
    _fields_ = [("data", (ctypes.c_double * ROBOT_STRIDE) * ROBOT_ROWS)]


def _set_row(robot: RobotT, row: int, values, col: int = 0) -> None:
    for idx, value in enumerate(values):
        robot.data[row][col + idx] = float(value)


def _call_command_ik(lib, robot, pose16, seed=None, capacity=64,
                     alternatives=True, initial=12345.0):
    """Call the raw C ABI in coupled command degrees and check output packing.

    The extra solution slot guards writes beyond the caller's capacity.
    Geometry, limits and solver policies are asserted by the calling tests.
    """
    best = (ctypes.c_double * 6)(*([initial] * 6))
    size = 12 * max(0, capacity) + 12
    output = (ctypes.c_double * size)(*([initial] * size))
    count = int(lib.SolveIK(
        _to_c_array(pose16), best, output if alternatives else None, capacity,
        _to_c_array(seed) if seed is not None else None, ctypes.byref(robot),
    ))
    if count <= 0:
        assert list(best) == [initial] * 6
        assert list(output) == [initial] * size
        return count, list(best), []
    assert count <= capacity
    commands = [list(output[12*i:12*i+6]) for i in range(count)] if alternatives else []
    if alternatives:
        assert list(best) == commands[0]
        for i in range(count):
            assert list(output[12*i+6:12*i+12]) == [0.] * 6
        assert list(output[12*count:]) == [initial] * (size - 12*count)
    else:
        assert list(output) == [initial] * size
    return count, list(best), commands


def _to_c_array(values):
    return (ctypes.c_double * len(values))(*[float(v) for v in values])


def _from_c_array(c_array, length):
    return [float(c_array[i]) for i in range(length)]


def _to_fk_api_joints_deg(joints_deg: List[float]) -> List[float]:
    joints = [float(v) for v in joints_deg]
    joints[2] = joints[1] + joints[2]
    return joints


def _from_fk_api_joints_deg(joints_deg: List[float]) -> List[float]:
    joints = [float(v) for v in joints_deg]
    joints[2] = joints[2] - joints[1]
    return joints


def _call_fk(lib, robot, joints_deg: List[float]) -> Tuple[int, List[float]]:
    joints = _to_c_array(_to_fk_api_joints_deg(joints_deg))
    pose = (ctypes.c_double * 16)()
    status = lib.SolveFK(joints, pose, ctypes.byref(robot))
    return int(status), _from_c_array(pose, 16)


def _call_ik(
    lib,
    robot,
    pose16_col_major: List[float],
    *,
    approx: List[float] | None = None,
    max_solutions: int = 32,
) -> Tuple[int, List[float], List[List[float]]]:
    seed = _to_fk_api_joints_deg(approx) if approx is not None else None
    count, best, commands = _call_command_ik(
        lib, robot, pose16_col_major, seed, max_solutions, initial=0.0,
    )
    return (count, _from_fk_api_joints_deg(best),
            [_from_fk_api_joints_deg(command) for command in commands])


def _call_joints_config(
    lib,
    robot,
    joints_deg: List[float],
    initial: Tuple[float, float, float] = (-9.0, -9.0, -9.0),
) -> Tuple[int, List[float]]:
    joints = _to_c_array(_to_fk_api_joints_deg(joints_deg))
    config = (ctypes.c_double * 3)(*initial)
    status = lib.Joints2Config(joints, config, ctypes.byref(robot))
    return int(status), _from_c_array(config, 3)
