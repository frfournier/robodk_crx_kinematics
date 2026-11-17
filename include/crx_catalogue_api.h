// SPDX-License-Identifier: Apache-2.0
// SPDX-FileCopyrightText: 2026 Centre Technologique en Aérospatiale
#ifndef CRX_CATALOGUE_API_H
#define CRX_CATALOGUE_API_H

#include "crx_python_api.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Additive ABI 1, opt-in nominal proof query. All layouts negotiate exactly
 * (sizeof,1,reserved=0). Existing ABI 1 entry points retain their behavior.
 * No C++ implementation or R-18/R-19 proof is claimed formally verified.
 */
typedef struct crx_v1_catalogue_handle crx_v1_catalogue_handle;

enum crx_v1_command_profile {
  CRX_COMMAND_CALLBACK = 0,
  CRX_COMMAND_DECOUPLED_USER = 1
};

enum crx_v1_admission {
  CRX_ADMISSION_NO = 0,
  CRX_ADMISSION_YES = 1,
  CRX_ADMISSION_UNDECIDED = 2
};

enum crx_v1_ranking {
  CRX_RANK_NOT_ATTEMPTED = 0,
  CRX_RANK_INCONCLUSIVE = 1,
  CRX_RANK_UNIQUE = 2,
  CRX_RANK_POSSIBLE_MINIMIZERS = 3,
  CRX_RANK_NO_ADMISSIBLE = 4
};

enum crx_v1_catalogue_flags { CRX_CATALOGUE_DISABLED = 1 };

enum crx_v1_query_preparation {
  CRX_QUERY_NOT_PREPARED = 0,
  CRX_QUERY_READY = 1,
  CRX_QUERY_INVALID_INPUT = 2,
  CRX_QUERY_UNSUPPORTED_MODEL = 3,
  CRX_QUERY_UNSUPPORTED_ENVIRONMENT = 4,
  CRX_QUERY_ARITHMETIC_FAILURE = 5
};

enum crx_v1_command_failure {
  CRX_COMMAND_SUCCESS = 0,
  CRX_COMMAND_INVALID_INPUT = 1,
  CRX_COMMAND_UNSUPPORTED_ENVIRONMENT = 2,
  CRX_COMMAND_IDENTITY_MISMATCH = 3,
  CRX_COMMAND_INCOMPLETE_PHASE_COVER = 4,
  CRX_COMMAND_RANGE_UNAVAILABLE = 5,
  CRX_COMMAND_WORK_LIMIT = 6,
  CRX_COMMAND_ARITHMETIC_UNAVAILABLE = 7
};

enum crx_v1_phase_failure {
  CRX_PHASE_SUCCESS = 0,
  CRX_PHASE_INVALID_INPUT = 1,
  CRX_PHASE_UNSUPPORTED_ENVIRONMENT = 2,
  CRX_PHASE_IDENTITY_MISMATCH = 3,
  CRX_PHASE_INCOMPLETE_ROOT_COVER = 4,
  CRX_PHASE_INVALID_BALL = 5,
  CRX_PHASE_COUNT_MISMATCH = 6,
  CRX_PHASE_BALLS_NOT_SEPARATED = 7,
  CRX_PHASE_RANGE_UNAVAILABLE = 8,
  CRX_PHASE_PAIR_WORK_LIMIT = 9
};

enum crx_v1_root_status {
  CRX_ROOT_COMPLETE = 0,
  CRX_ROOT_INCONCLUSIVE = 1,
  CRX_ROOT_INVALID_INPUT = 2,
  CRX_ROOT_UNSUPPORTED_ENVIRONMENT = 3,
  CRX_ROOT_ARITHMETIC_UNAVAILABLE = 4
};

enum crx_v1_root_kind {
  CRX_ROOT_EXCLUSION = 0,
  CRX_ROOT_SIMPLE = 1,
  CRX_ROOT_UNRESOLVED = 2
};

enum crx_v1_root_reason {
  CRX_ROOT_REASON_NONE = 0,
  CRX_ROOT_CHART_BOUNDARY = 1,
  CRX_ROOT_DEPTH_LIMIT = 2,
  CRX_ROOT_WORK_LIMIT = 3,
  CRX_ROOT_LEAF_LIMIT = 4,
  CRX_ROOT_SPLIT_NOT_SEPARATED = 5,
  CRX_ROOT_ZERO_POLYNOMIAL = 6,
  CRX_ROOT_ZERO_DERIVATIVE = 7,
  CRX_ROOT_ZERO_DETERMINANT = 8,
  CRX_ROOT_ARITHMETIC_FAILURE = 9
};

/* Defaults: flags/profile=0, capacity=64, nodes=8192, depth=40, leaves=4096,
 * attempts=8, candidates=512, pairs=120, lifts=2^20. Capacity 0..512 limits the
 * display prefix after proof; it never limits phase coverage or proof counts.
 * nodes may be zero; leaves 2..2^20, depth<=50, attempts1..8, other
 * budgets<=2^20.
 */
typedef struct crx_v1_catalogue_options {
  uint32_t struct_size, version, reserved, flags;
  uint32_t profile, capacity, max_nodes, max_depth, max_leaves;
  uint32_t max_chart_attempts, max_candidate_certificates, max_pair_tests;
  uint64_t max_lifts;
} crx_v1_catalogue_options;

/* Coverage concerns nominal phases before limits/turns/capacity. Complete is
 * produced only by the sealed root/ball assembler. Nominal infeasibility is a
 * complete zero-root cover; command infeasibility also requires fully resolved
 * command admission. UNIQUE means one Yes lift beats every potential
 * competitor. POSSIBLE_MINIMIZERS is a containing set, not proof of equal exact
 * costs. Preparation=0 and coverage=NotAttempted when disabled. Identity
 * schema=0 means exact target preparation failed/not attempted. Other failure
 * fields then describe unavailable stages, not exclusions. raw_words:
 * lower6,upper6,seed6, weights6 exact degree/weight encodings;
 * input_words/schema follow ball records.
 */
typedef struct crx_v1_catalogue_summary {
  uint32_t struct_size, version, reserved;
  int32_t coverage, preparation, context_failure, phase_failure,
      command_failure, ranking;
  uint32_t identity_schema, phase_count, root_status, root_count, chart_shift;
  uint32_t chart_attempts, root_nodes, entire_circle_unresolved,
      root_leaf_count[2];
  uint32_t nominal_infeasible, enumeration_complete, admission_complete,
      command_infeasible;
  uint32_t returned_count, truncated, has_seed, profile;
  int32_t numerical_status;
  uint32_t numerical_count, numerical_truncated, used_singular_representatives;
  uint64_t box_lifts, yes, no, undecided, command_count;
  uint64_t input_words[77], raw_words[24];
  double length_scale, rotation_deviation_upper[3];
} crx_v1_catalogue_summary;

/* Intervals are [lo,hi], in canonical/command radians or squared radian travel.
 * Unavailable intervals contain NaNs and establish no claim. A command box
 * encloses the exact root, not an exact FK guarantee for its rounded midpoint.
 * All Yes/Undecided proof rows are indexed, including beyond returned_count.
 */
typedef struct crx_v1_catalogue_command {
  uint32_t struct_size, version, reserved;
  int32_t admission;
  uint32_t phase, possible_minimizer, cost_available, reserved2;
  int64_t turns[6];
  double radians[6][2], travel_squared[2], proposed_travel_squared;
} crx_v1_catalogue_command;

typedef struct crx_v1_catalogue_range {
  uint32_t struct_size, version, reserved, phase, available, empty;
  uint64_t count;
  int64_t first[6], last[6];
  double base[6][2];
} crx_v1_catalogue_range;

/* Exact chart semantics: alpha=chart_shift*pi/8; chart1 is opposite chart0.
 * theta=alpha+chart*pi+2*atan(t), -1<=t<=1. Descriptors are outward intervals.
 * Polynomial slots retain fixed degrees8,7,4, including leading zeros.
 */
typedef struct crx_v1_catalogue_chart {
  uint32_t struct_size, version, reserved, chart;
  int32_t status;
  uint32_t leaf_count, nodes, roots, exclusions, unresolved, max_depth;
  double descriptor[2], residual[9][2], derivative[8][2], determinant[5][2];
  double omitted_residual[2], omitted_determinant[2];
} crx_v1_catalogue_chart;

typedef struct crx_v1_catalogue_leaf {
  uint32_t struct_size, version, reserved, chart, index;
  int32_t kind, reason;
  uint32_t depth;
  int32_t derivative_sign, determinant_sign;
  double tangent[2], phase[2], endpoints[2][2];
  double residual_bernstein[9][2], derivative_bernstein[8][2],
      determinant_bernstein[5][2];
} crx_v1_catalogue_leaf;

/* pose/reference/tool use the raw forward-reference contract of ik_certified.
 * limits_deg NULL uses model's ORIGINAL degree box for callback profile. The
 * decoupled profile REQUIRES an explicit 2x6 box in user coordinates, where
 * user[2]=callback[2]-callback[1]. seed is optional6degrees in that same
 * profile; weights NULL means ones, otherwise six positive finite represented
 * weights. Raw degree inputs define mathematical pi/180, never a rounded
 * inverse getter. On error *output is NULL. Inputs/options remain unchanged. A
 * successful call owns immutable data even when proofs are inconclusive or
 * disabled; inspect summary. Numerical fallback rows remain explicitly
 * unproved. Handles survive model destruction, support concurrent reads, and
 * must be destroyed once after readers finish. NULL destroy is safe;
 * stale/foreign handles are UB.
 */
CRX_API int32_t crx_v1_catalogue_query(
    const crx_v1_handle *model, const double *pose, const double *reference,
    const double *tool, const double *limits_deg, const double *seed_deg,
    const double *weights, const crx_v1_catalogue_options *options,
    crx_v1_catalogue_handle **output);
CRX_API void crx_v1_catalogue_destroy(crx_v1_catalogue_handle *query);
/* Read negotiation/index errors leave the entire output unchanged. */
CRX_API int32_t crx_v1_catalogue_read(const crx_v1_catalogue_handle *,
                                      crx_v1_catalogue_summary *);
CRX_API int32_t crx_v1_catalogue_phase(const crx_v1_catalogue_handle *,
                                       uint32_t index,
                                       crx_v1_ball_certificate *);
CRX_API int32_t crx_v1_catalogue_command_read(const crx_v1_catalogue_handle *,
                                              uint64_t index,
                                              crx_v1_catalogue_command *);
CRX_API int32_t crx_v1_catalogue_range_read(const crx_v1_catalogue_handle *,
                                            uint32_t phase,
                                            crx_v1_catalogue_range *);
CRX_API int32_t crx_v1_catalogue_chart_read(const crx_v1_catalogue_handle *,
                                            uint32_t chart,
                                            crx_v1_catalogue_chart *);
CRX_API int32_t crx_v1_catalogue_leaf_read(const crx_v1_catalogue_handle *,
                                           uint32_t chart, uint32_t index,
                                           crx_v1_catalogue_leaf *);
/* Numerical fallback commands, selected profile in degrees; no proof attached.
 */
CRX_API int32_t crx_v1_catalogue_numerical(const crx_v1_catalogue_handle *,
                                           uint32_t index, double *command_deg);

#ifdef __cplusplus
}
#endif
#endif
