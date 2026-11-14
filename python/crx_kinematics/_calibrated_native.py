# SPDX-License-Identifier: Apache-2.0
# SPDX-FileCopyrightText: 2026 Centre Technologique en Aérospatiale
"""Optional ctypes bindings for the separately negotiated calibrated ABI."""

import ctypes as ct

from ._native import LibraryError, NativeError

U, I, Q, D = ct.c_uint32, ct.c_int32, ct.c_uint64, ct.c_double
Vector = D*6


class _Header(ct.Structure):
    _fields_ = [("struct_size", U), ("version", U)]

    def __init__(self):
        super().__init__()
        self.struct_size = ct.sizeof(type(self))
        self.version = 1


class ModelInput(_Header):
    _fields_ = [("dh", (D*4)*6), ("alpha_quarters", I*6), ("theta_quarters", I*6),
                ("senses", Vector), ("base_quaternion", D*4), ("base_position", D*3),
                ("tool_quaternion", D*4), ("tool_position", D*3),
                ("lower_commands", Vector), ("upper_commands", Vector),
                ("length_scale", D), ("limit_units", U), ("reserved", U*3)]


class Target(_Header):
    _fields_ = [("quaternion", D*4), ("position", D*3), ("reserved", U*4)]


class Options(_Header):
    _fields_ = [("max_iterations", U), ("max_backtracks", U), ("maximum_step", D),
                ("normalized_position_tolerance", D), ("orientation_tolerance", D),
                ("uniqueness_radius", D), ("flags", U), ("reserved", U*3)]


class Summary(_Header):
    _fields_ = ([("coverage", I)] + [(name, U) for name in (
        "seed_count", "numerical_count", "numerical_inside_count", "local_count", "admitted_count")]
        + [(name, Q) for name in ("iterations", "backtracks", "evaluations", "owned_payload_bytes")]
        + [("length_scale", D), ("reserved", U*4)])


class Identity(_Header):
    _fields_ = [("model_schema", U), ("model_word_count", U), ("model_words", Q*70),
                ("target_words", Q*7), ("length_scale", D), ("limit_units", U), ("reserved", U*3)]


class Result(_Header):
    _fields_ = ([("seed_index", U), ("status", I), ("flags", U), ("certificate_failure", I),
                 ("local_jacobian_sign", I), ("command_jacobian_sign", I)]
        + [(name, U) for name in ("iterations", "backtracks", "evaluations", "reserved_counter")]
        + [(name, Vector) for name in ("seed_commands_rad", "commands_rad", "increments_rad")]
        + [(name, D) for name in (
        "normalized_position_error", "orientation_error", "existence_radius", "uniqueness_radius",
        "beta", "lipschitz", "eta", "inverse_residual", "coarse_eta", "pose_defect", "screw_norm_sum",
        "angular_norm_sum", "anchor_position_norm", "command_reserve_lower")]
        + [("preconditioner", D*36), ("reserved", U*4)])


def check(code):
    if code == 0:
        return
    if code == -1:
        raise ValueError("Invalid calibrated native request or unsupported record layout")
    if code == -5:
        raise NativeError("Calibrated enclosures require a supported IEEE floating-point environment")
    if code == -6:
        raise NativeError("Calibrated arithmetic could not enclose the declared model or target")
    raise NativeError(f"Calibrated native API failure (code {code})")


class Bindings:
    """Retain the loaded nominal library without changing its required symbols."""

    def __init__(self, library):
        self.library = library
        names = ("abi_version", "options_default", "create", "destroy", "fk", "jacobian",
                 "query_create", "query_destroy", "summary_read", "identity_read", "result_read")
        if not all(hasattr(library.lib, "crx_cal_v1_"+name) for name in names):
            raise NotImplementedError(
                "This DLL does not export calibrated IK; load a DLL with the calibrated ABI 1 extension")
        pointer = ct.POINTER(D)
        handle = ct.c_void_p
        signatures = {
            "abi_version": ([], U),
            "options_default": ([ct.POINTER(Options)], I),
            "create": ([ct.POINTER(ModelInput), ct.POINTER(handle)], I),
            "destroy": ([handle], None),
            "fk": ([handle, pointer, pointer], I),
            "jacobian": ([handle, pointer, pointer], I),
            "query_create": ([handle, ct.POINTER(Target), pointer, U,
                              ct.POINTER(Options), ct.POINTER(handle)], I),
            "query_destroy": ([handle], None),
            "summary_read": ([handle, ct.POINTER(Summary)], I),
            "identity_read": ([handle, ct.POINTER(Identity)], I),
            "result_read": ([handle, U, ct.POINTER(Result)], I),
        }
        for name, (arguments, result) in signatures.items():
            function = getattr(library.lib, "crx_cal_v1_"+name)
            function.argtypes, function.restype = arguments, result
            setattr(self, name, function)
        if self.abi_version() != 1:
            raise LibraryError("Unsupported calibrated native ABI; expected extension version 1")

    def defaults(self):
        output = Options()
        check(self.options_default(ct.byref(output)))
        return output
