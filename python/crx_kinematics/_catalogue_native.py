# SPDX-License-Identifier: Apache-2.0
# SPDX-FileCopyrightText: 2026 Centre Technologique en Aérospatiale
"""Negotiated additive catalogue ABI records; no loading at import."""

import ctypes as ct

U = ct.c_uint32
I = ct.c_int32
Q = ct.c_uint64
D = ct.c_double
Interval = D * 2


class Options(ct.Structure):
    _fields_ = [(name, U) for name in (
        "struct_size", "version", "reserved", "flags", "profile", "capacity",
        "max_nodes", "max_depth", "max_leaves", "max_chart_attempts",
        "max_candidate_certificates", "max_pair_tests",
    )] + [("max_lifts", Q)]


class Summary(ct.Structure):
    _fields_ = ([(name, U) for name in ("struct_size", "version", "reserved")]
        + [(name, I) for name in ("coverage", "preparation", "context_failure", "phase_failure", "command_failure", "ranking")]
        + [(name, U) for name in ("identity_schema", "phase_count", "root_status", "root_count", "chart_shift",
            "chart_attempts", "root_nodes", "entire_circle_unresolved")]
        + [("root_leaf_count", U*2)]
        + [(name, U) for name in ("nominal_infeasible", "enumeration_complete", "admission_complete", "command_infeasible",
            "returned_count", "truncated", "has_seed", "profile")]
        + [("numerical_status", I)]
        + [(name, U) for name in ("numerical_count", "numerical_truncated", "used_singular_representatives")]
        + [(name, Q) for name in ("box_lifts", "yes", "no", "undecided", "command_count")]
        + [("input_words", Q*77), ("raw_words", Q*24), ("length_scale", D), ("rotation_deviation_upper", D*3)])


class Command(ct.Structure):
    _fields_ = ([(name, U) for name in ("struct_size", "version", "reserved")]
        + [("admission", I)]
        + [(name, U) for name in ("phase", "possible_minimizer", "cost_available", "reserved2")]
        + [("turns", ct.c_int64*6), ("radians", Interval*6), ("travel_squared", Interval), ("proposed_travel_squared", D)])


class Range(ct.Structure):
    _fields_ = ([(name, U) for name in ("struct_size", "version", "reserved", "phase", "available", "empty")]
        + [("count", Q), ("first", ct.c_int64*6), ("last", ct.c_int64*6), ("base", Interval*6)])


class Chart(ct.Structure):
    _fields_ = ([(name, U) for name in ("struct_size", "version", "reserved", "chart")]
        + [("status", I)]
        + [(name, U) for name in ("leaf_count", "nodes", "roots", "exclusions", "unresolved", "max_depth")]
        + [("descriptor", Interval), ("residual", Interval*9), ("derivative", Interval*8),
           ("determinant", Interval*5), ("omitted_residual", Interval), ("omitted_determinant", Interval)])


class Leaf(ct.Structure):
    _fields_ = ([(name, U) for name in ("struct_size", "version", "reserved", "chart", "index")]
        + [("kind", I), ("reason", I), ("depth", U), ("derivative_sign", I), ("determinant_sign", I)]
        + [("tangent", Interval), ("phase", Interval), ("endpoints", Interval*2),
           ("residual_bernstein", Interval*9), ("derivative_bernstein", Interval*8), ("determinant_bernstein", Interval*5)])


class Strata(ct.Structure):
    _fields_ = [(name, U) for name in ("struct_size", "version", "reserved", "count",
        "recovery_budget_exhausted", "source_domains", "candidate_budget_exhausted",
        "total_ball_attempts", "total_certified_balls", "nomination_count")]


class Recovery(ct.Structure):
    _fields_ = ([(name, U) for name in ("struct_size", "version", "reserved", "index", "source", "near_stratum",
        "used_singular_representatives", "refinement_attempted", "candidate_count", "attempted_balls", "certified_balls")]
        + [("numerical_status", I)]
        + [(name, D) for name in ("phase", "delta_defect", "base_axis_defect", "coincident_defect",
            "vertical_defect", "shell_defect", "compatibility_defect")]
        + [("canonical_rad", (D*6)*8), ("position_errors", D*8), ("orientation_errors", D*8)])


def bind(library, vector, pose, limits, optional, ball_record):
    library.catalogue_strata_read = library.catalogue_recovery_read = None
    names = ("catalogue_query", "catalogue_destroy", "catalogue_read", "catalogue_phase",
             "catalogue_command_read", "catalogue_range_read", "catalogue_chart_read", "catalogue_leaf_read", "catalogue_numerical")
    if not all(hasattr(library.lib, "crx_v1_"+name) for name in names):
        for name in names: setattr(library, name, None)
        return
    library._function("catalogue_query", [ct.c_void_p, pose, pose, pose, optional(limits), optional(vector), optional(vector), ct.POINTER(Options), ct.POINTER(ct.c_void_p)])
    library._function("catalogue_destroy", [ct.c_void_p], None)
    library._function("catalogue_read", [ct.c_void_p, ct.POINTER(Summary)])
    for name, index, record in [("catalogue_phase", U, ball_record), ("catalogue_command_read", Q, Command),
                                 ("catalogue_range_read", U, Range), ("catalogue_chart_read", U, Chart)]:
        library._function(name, [ct.c_void_p, index, ct.POINTER(record)])
    library._function("catalogue_leaf_read", [ct.c_void_p, U, U, ct.POINTER(Leaf)])
    library._function("catalogue_numerical", [ct.c_void_p, U, ct.POINTER(D)])
    if all(hasattr(library.lib, "crx_v1_"+name) for name in ("catalogue_strata_read", "catalogue_recovery_read")):
        library._function("catalogue_strata_read", [ct.c_void_p, ct.POINTER(Strata)])
        library._function("catalogue_recovery_read", [ct.c_void_p, U, ct.POINTER(Recovery)])
