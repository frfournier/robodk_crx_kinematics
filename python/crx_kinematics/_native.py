# SPDX-License-Identifier: Apache-2.0
# SPDX-FileCopyrightText: 2026 Centre Technologique en Aérospatiale
"""Private ABI declarations. No RoboDK import, discovery, or DLL load at import."""

import ctypes as ct
import os
import struct
import sys
from pathlib import Path

import numpy as np
from numpy.ctypeslib import ndpointer


class LibraryError(RuntimeError):
    """The native library could not be found, loaded, or negotiated."""


class NativeError(RuntimeError):
    """The native API reported an infrastructure or contract failure."""


class JointLimitError(ValueError):
    """A joint command lies outside the configured limits."""


class ConfigurationError(ValueError):
    """Configuration flags are undefined at this arm posture."""


class ClosedSolverError(RuntimeError):
    """The solver handle has already been closed."""


class IKMetadata(ct.Structure):
    _fields_ = [(name, ct.c_int32) for name in (
        "status", "count", "truncated", "used_singular_representatives",
    )]


class FibreMetadata(ct.Structure):
    _fields_ = [(name, ct.c_int32) for name in (
        "status", "count", "truncated", "used_singular_representatives",
    )]

class FibreExtension(ct.Structure):
    _fields_ = [
        ("struct_size", ct.c_uint32),
        ("version", ct.c_uint32),
        ("coverage", ct.c_int32),
        ("reserved", ct.c_uint32),
    ]


def _array_type(dtype, shape, *, output=False):
    flags = ("C_CONTIGUOUS", "ALIGNED", "WRITEABLE") if output else (
        "C_CONTIGUOUS", "ALIGNED",
    )
    return ndpointer(dtype=dtype, shape=shape, ndim=len(shape), flags=flags)


def _optional(array_type):
    # ndpointer deliberately rejects None; only seed/limit pointers are nullable.
    class OptionalArray(array_type):
        @classmethod
        def from_param(cls, value):
            return None if value is None else array_type.from_param(value)

    return OptionalArray


def resolve_library(dll_path: str | os.PathLike[str] | None) -> Path:
    if dll_path is None:
        try:
            from robodk.robolink import getPathRoboDK
        except ImportError as exc:
            raise LibraryError(
                "DLL discovery requires crx-kinematics[robodk]; "
                "install that extra or supply dll_path."
            ) from exc
        try:
            executable = getPathRoboDK()
            if not executable:
                raise ValueError("RoboDK returned an empty executable path")
            dll_path = Path(executable).parent / "robotextensions" / "crx_kinematics.dll"
        except Exception as exc:
            raise LibraryError("RoboDK path discovery failed; supply dll_path.") from exc
    path = Path(dll_path).expanduser().resolve()
    if path.suffix.lower() != ".dll" or not path.is_file():
        raise LibraryError(f"Expected an existing DLL file: {path}")
    return path


def check(code: int) -> None:
    if code == 0:
        return
    if code == -1:
        raise ValueError("Native API rejected an invalid argument")
    if code == -2:
        raise JointLimitError("Joint command is outside the configured limits")
    if code == -3:
        raise ConfigurationError("Configuration is undefined at this arm singularity")
    raise NativeError(f"Native API failure (code {code})")


class NativeLibrary:
    def __init__(self, dll_path: str | os.PathLike[str] | None) -> None:
        if sys.platform != "win32" or struct.calcsize("P") != 8:
            raise LibraryError("The native wrapper requires Windows and 64-bit Python")
        self.path = resolve_library(dll_path)
        self.directory = None
        try:
            self.directory = os.add_dll_directory(str(self.path.parent))
            self.lib = ct.CDLL(str(self.path))
            self._bind()
        except Exception as exc:
            self.close_directory()
            if isinstance(exc, LibraryError):
                raise
            raise LibraryError(
                f"Cannot load compatible CRX ABI 1 from {self.path}. "
                "Use an updated Windows x64 DLL and ensure its dependencies are installed."
            ) from exc

    def _function(self, name, args, result=ct.c_int32) -> None:
        function = getattr(self.lib, "crx_v1_" + name)
        function.argtypes = args
        function.restype = result
        setattr(self, name, function)

    def _bind(self) -> None:
        self._function("abi_version", [], ct.c_uint32)
        if self.abi_version() != 1:
            raise LibraryError(f"Unsupported CRX ABI in {self.path}; expected ABI 1")
        self._function("library_version", [], ct.c_char_p)
        self._function("model_name", [ct.c_int32], ct.c_char_p)
        vector = _array_type(np.float64, (6,))
        pose = _array_type(np.float64, (4, 4))
        limits = _array_type(np.float64, (2, 6))
        self._function("create", [ct.c_int32, _optional(limits), ct.POINTER(ct.c_void_p)])
        self._function("destroy", [ct.c_void_p], None)
        self._function("limits", [ct.c_void_p, _array_type(np.float64, (2, 6), output=True)])
        self._function("fk", [ct.c_void_p, vector, _array_type(np.float64, (4, 4), output=True)])
        self._function("configuration", [ct.c_void_p, vector, _array_type(np.int32, (3,), output=True)])
        self._function("jacobian", [ct.c_void_p, vector, _array_type(np.float64, (6, 6), output=True)])
        solutions = ndpointer(dtype=np.float64, ndim=2,
                              flags=("C_CONTIGUOUS", "ALIGNED", "WRITEABLE"))
        self._function("ik", [ct.c_void_p, pose, _optional(vector), ct.c_int32,
                              solutions, ct.POINTER(IKMetadata)])
        try:
            identify = self.lib.crx_v1_identify
        except AttributeError:
            self.identify = None
        else:
            # The metadata arrays are capacity-sized; use unconstrained ndpointer
            # declarations here and validate their shapes before calling.
            identify.argtypes = [
                ct.c_void_p, pose, ct.c_int32, solutions,
                ndpointer(dtype=np.int32, ndim=1,
                          flags=("C_CONTIGUOUS", "ALIGNED", "WRITEABLE")),
                ndpointer(dtype=np.int32, ndim=1,
                          flags=("C_CONTIGUOUS", "ALIGNED", "WRITEABLE")),
                ndpointer(dtype=np.int32, ndim=2,
                          flags=("C_CONTIGUOUS", "ALIGNED", "WRITEABLE")),
                ndpointer(dtype=np.int32, ndim=1,
                          flags=("C_CONTIGUOUS", "ALIGNED", "WRITEABLE")),
                ct.POINTER(FibreMetadata),
            ]
            identify.restype = ct.c_int32
            self.identify = identify
        try:
            identify2 = self.lib.crx_v1_identify2
        except AttributeError:
            self.identify2 = None
        else:
            identify2.argtypes = [
                ct.c_void_p, pose, ct.c_int32, solutions,
                ndpointer(dtype=np.int32, ndim=1,
                          flags=("C_CONTIGUOUS", "ALIGNED", "WRITEABLE")),
                ndpointer(dtype=np.int32, ndim=1,
                          flags=("C_CONTIGUOUS", "ALIGNED", "WRITEABLE")),
                ndpointer(dtype=np.int32, ndim=2,
                          flags=("C_CONTIGUOUS", "ALIGNED", "WRITEABLE")),
                ndpointer(dtype=np.int32, ndim=1,
                          flags=("C_CONTIGUOUS", "ALIGNED", "WRITEABLE")),
                ct.POINTER(FibreMetadata), ct.POINTER(FibreExtension),
            ]
            identify2.restype = ct.c_int32
            self.identify2 = identify2

    def close_directory(self) -> None:
        if self.directory is not None:
            self.directory.close()
            self.directory = None


def release(library: NativeLibrary, handle: ct.c_void_p) -> None:
    # The finalizer owns the library and its search-directory cookie until the
    # native handle is destroyed. Do not manually unload a ctypes library.
    try:
        library.destroy(handle)
    finally:
        library.close_directory()
