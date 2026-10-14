import ctypes
import json
import math
import os
import subprocess
import sys
from pathlib import Path

import pytest


from crx_test_support import RobotT, _set_row, DOFS


def pytest_addoption(parser):
    parser.addoption(
        "--require-solver-reliability", action="store_true", default=False,
        help="Require the solver DLL for the generated reachable-pose gate",
    )
    parser.addoption(
        "--run-robodk",
        action="store_true",
        default=False,
        help="Run live RoboDK tests using an already deployed matching DLL",
    )


def _library_path(repo_root: Path | None = None) -> Path:
    configured_path = os.environ.get("CRXKIN_LIBRARY_PATH")
    if configured_path:
        return Path(configured_path).expanduser().resolve()

    # Standard CMake structure: <repo>/build/Release/crx_kinematics.dll
    if repo_root is None:
        repo_root = Path(__file__).resolve().parents[1]

    if sys.platform == "win32":
        library_path = repo_root / "build" / "Release" / "crx_kinematics.dll"
        return library_path.resolve()


def _load_kinematics_library(lib):
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

    lib.Joints2Config.argtypes = [real_ptr, real_ptr, robot_ptr]
    lib.Joints2Config.restype = ctypes.c_int

    return lib


@pytest.fixture(scope="session")
def kinematics_lib(request):
    path = _library_path()
    if os.environ.get("CRXKIN_LIBRARY_PATH") or request.config.getoption("--require-solver-reliability"):
        assert path is not None and path.is_file(), f"Reliability gate requires the DLL: {path}"
    return _load_kinematics_library(path)


@pytest.fixture(scope="session")
def baseline_kinematics_lib():
    """Optional external snapshot, selected independently of the current build."""
    configured = os.environ.get("CRXKIN_BASELINE_LIBRARY_PATH")
    if not configured:
        pytest.skip("Set CRXKIN_BASELINE_LIBRARY_PATH to compare an external snapshot")
    path = Path(configured).expanduser().resolve()
    assert path.is_file(), f"Configured baseline DLL is missing: {path}"
    assert path != _library_path(), "Baseline and candidate must be separate DLL files"
    return _load_kinematics_library(path)


@pytest.fixture(scope="session")
def canonical_probe():
    configured = os.environ.get("CRXKIN_ROOT_PROBE_PATH")
    path = (Path(configured) if configured else
            Path(__file__).resolve().parents[1] / "build/Release/crx_canonical_tests.exe")
    if configured:
        assert path.is_file(), f"Configured native probe is missing: {path}"
    elif not path.is_file():
        pytest.skip("Build crx_canonical_tests or set CRXKIN_ROOT_PROBE_PATH")

    def run(mode, values):
        completed = subprocess.run(
            [str(path), mode, *(repr(float(value)) for value in values)],
            check=True, capture_output=True, text=True, timeout=20,
        )
        return json.loads(completed.stdout)
    return run


kRobotBaseXyzwprRow = 9
kRobotToolXyzwprRow = 28
kRobotJointLowerLimitRow = 30
kRobotJointUpperLimitRow = 31
kRobotJointSensesRow = 3
kRobotJointSensesCol = 4
kRobotDhBaseRow = 10


@pytest.fixture
def crx_10ia() -> RobotT:
    robot = RobotT()

    # nDOFs at row=1, col=1
    robot.data[1][1] = float(DOFS)

    _set_row(robot, kRobotBaseXyzwprRow, [0.0, 0.0, -245.0, 0.0, 0.0, 0.0])
    _set_row(robot, kRobotToolXyzwprRow, [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    _set_row(
        robot,
        kRobotJointSensesRow,
        [1.0, 1.0, -1.0, -1.0, -1.0, -1.0],
        col=kRobotJointSensesCol,
    )

    # Roboguide joint limits
    # _set_row(robot, kRobotJointLowerLimitRow, [-190.0, -179.99, -195.0, -190.0, -179.99, -225.0])
    # _set_row(robot, kRobotJointUpperLimitRow, [190.0, 179.99, 375.0, 190.0, 179.99, 225.0])

    # Wide joint limits for CRX-10iA
    _set_row(
        robot,
        kRobotJointLowerLimitRow,
        [-190.0, -180.0, -195.0, -190.0, -180.0, -225.0],
    )
    _set_row(
        robot, kRobotJointUpperLimitRow, [190.0, 180.0, 375.0, 190.0, 180.0, 225.0]
    )

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
        _set_row(robot, kRobotDhBaseRow + idx, row)

    return robot
