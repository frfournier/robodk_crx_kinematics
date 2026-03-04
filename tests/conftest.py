import ctypes
import os
import sys
from pathlib import Path
import math
import pytest


ROBOT_ROWS = 32
ROBOT_STRIDE = 20
DOFS = 6


class RobotT(ctypes.Structure):
    _fields_ = [("data", (ctypes.c_double * ROBOT_STRIDE) * ROBOT_ROWS)]


def _set_row(robot: RobotT, row: int, values, col: int = 0) -> None:
    for idx, value in enumerate(values):
        robot.data[row][col + idx] = float(value)


def _library_path(repo_root: Path = None) -> Path:
    # Standard CMake structure: <repo>/build/Release/crx_kinematics.dll
    if repo_root is None:
        repo_root = Path(__file__).resolve().parents[1]

    if sys.platform == "win32":
        library_path = repo_root / "build" / "Release" / "crx_kinematics.dll"
        return library_path.resolve()


@pytest.fixture(scope="session")
def kinematics_lib():
    lib = _library_path()
    print(f"Loading crx_kinematics library from:\n  {lib}")

    if not lib.exists():
        pytest.skip(
            "crx_kinematics library not found at:\n"
            f"  {lib}\n\n"
            "Build it with:\n"
            "  scripts\\build_crx_kinematics_msvc.bat\n"
            "(Run from a VS x64 Native Tools Developer Prompt.)"
        )

    # Ensure Windows can locate any dependent DLLs placed next to the target
    try:
        os.add_dll_directory(str(lib.parent))
    except Exception:
        pass

    lib = ctypes.CDLL(str(lib))

    real_ptr = ctypes.POINTER(ctypes.c_double)
    robot_ptr = ctypes.POINTER(RobotT)

    lib.SolveFK.argtypes = [real_ptr, real_ptr, robot_ptr]
    lib.SolveFK.restype = ctypes.c_int

    lib.SolveFK_CAD.argtypes = [real_ptr, real_ptr, real_ptr, ctypes.c_int, robot_ptr]
    lib.SolveFK_CAD.restype = ctypes.c_int

    lib.SolveIK.argtypes = [
        real_ptr,
        real_ptr,
        real_ptr,
        ctypes.c_int,
        real_ptr,
        robot_ptr,
    ]
    lib.SolveIK.restype = ctypes.c_int

    return lib


@pytest.fixture
def crx_robot() -> RobotT:
    robot = RobotT()

    # nDOFs at row=1, col=1
    robot.data[1][1] = float(DOFS)

    _set_row(robot, 9, [0.0, 0.0, -245.0, 0.0, 0.0, 0.0])
    _set_row(robot, 28, [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    _set_row(robot, 3, [1.0, 1.0, -1.0, -1.0, -1.0, -1.0], col=4)
    _set_row(robot, 30, [-180.0, -180.0, -360.0, -190.0, -180.0, -190.0])
    _set_row(robot, 31, [180.0, 180.0, 430.0, 190.0, 180.0, 190.0])

    # alpha a theta d prismatic
    HALF_PI = math.pi / 2
    dh_rows = [
        [0.0, 0.0, 0.0, 245.0, 0.0],
        [-HALF_PI, 0.0, -HALF_PI, 0.0, 0.0],
        [0.0, 540.0, 0.0, 0.0, 0.0],
        [-HALF_PI, 0.0, 0.0, 540.0, 0.0],
        [HALF_PI, 0.0, 0.0, -150.0, 0.0],
        [-HALF_PI, 0.0, 0.0, 160.0, 0.0],
    ]
    for idx, row in enumerate(dh_rows):
        _set_row(robot, 10 + idx, row)

    return robot
