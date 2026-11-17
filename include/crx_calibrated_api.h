// SPDX-License-Identifier: Apache-2.0
// SPDX-FileCopyrightText: 2026 Centre Technologique en Aérospatiale
#ifndef CRX_CALIBRATED_API_H
#define CRX_CALIBRATED_API_H

#include "crx_python_api.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Additive calibrated ABI1: Windows x64/cdecl. Existing nominal layouts and
 * policy are unchanged. Every struct negotiates exactly(sizeof,1,reserved=0).
 * Failed creation/read/evaluation leaves caller outputs unchanged. Inputs and
 * outputs are nonoverlapping caller buffers. No array pointer is retained.
 * Immutable handles permit concurrent reads; destroy only after calls finish.
 * No C++ implementation is claimed formally verified.
 */
typedef struct crx_cal_v1_handle crx_cal_v1_handle;
typedef struct crx_cal_v1_query crx_cal_v1_query;

enum crx_cal_v1_error {
  CRX_CAL_UNSUPPORTED_ENVIRONMENT = -5,
  CRX_CAL_ARITHMETIC_UNAVAILABLE = -6
};

enum crx_cal_v1_limit_units { CRX_CAL_RADIANS = 0, CRX_CAL_DEGREES = 1 };

enum crx_cal_v1_flags { CRX_CAL_DISABLE_CERTIFICATION = 1 };

enum crx_cal_v1_correction_status {
  CRX_CAL_CONVERGED = 0,
  CRX_CAL_INVALID_INPUT = 1,
  CRX_CAL_MODEL_MISMATCH = 2,
  CRX_CAL_ARITHMETIC_FAILURE = 3,
  CRX_CAL_ITERATION_BUDGET = 4,
  CRX_CAL_SINGULAR_STEP = 5,
  CRX_CAL_LINE_SEARCH_FAILURE = 6
};

enum crx_cal_v1_result_flags {
  CRX_CAL_PROOF_REQUESTED = 1,
  CRX_CAL_PROOF_ATTEMPTED = 2,
  CRX_CAL_NUMERICAL_LIMITS = 4,
  CRX_CAL_LOCAL_PROOF = 8,
  CRX_CAL_COMMANDS_ADMITTED = 16,
  CRX_CAL_LOG_SERIES = 32
};

#define CRX_CAL_V1_MODEL_WORDS 70
#define CRX_CAL_V1_MAX_SEEDS 512

typedef struct crx_cal_v1_model_input {
  uint32_t struct_size;
  uint32_t version;
  /* Rows: alpha correction radians,a mm,theta correction radians,d mm.
   * Full angle = declared quarter*pi/2 + correction, without snapping.
   * Modified DH = Rx(alpha) Tx(a) Rz(theta+x) Tz(d).
   * x_i=senses_i*u_i; u is ALREADY COUPLED command radians. */
  double dh[6][4];
  int32_t alpha_quarters[6];
  int32_t theta_quarters[6];
  double senses[6];          /* Exact +1 or -1. */
  double base_quaternion[4]; /* Homogeneous w,x,y,z; nonzero, not normalized. */
  double base_position[3];
  double tool_quaternion[4];
  double tool_position[3];
  double
      lower_commands[6]; /* Raw bounds in limit_units, provenance retained. */
  double upper_commands[6];
  double
      length_scale; /* Positive mm scale; every proof length is normalized. */
  uint32_t limit_units;
  uint32_t reserved[3];
} crx_cal_v1_model_input;

typedef struct crx_cal_v1_target {
  uint32_t struct_size;
  uint32_t version;
  double quaternion[4]; /* Exact homogeneous quaternion defines target SO(3). */
  double
      position[3]; /* World position, mm, including model base/tool frames. */
  uint32_t reserved[4];
} crx_cal_v1_target;

typedef struct crx_cal_v1_options {
  uint32_t struct_size;
  uint32_t version;
  uint32_t max_iterations; /* 0..128 accepted Newton updates per seed. */
  uint32_t max_backtracks; /* 0..32 halvings per update. */
  double maximum_step; /* Positive physical increment infinity norm, radians. */
  double normalized_position_tolerance; /* Numerical dispatch only. */
  double orientation_tolerance;         /* Radians, numerical dispatch only. */
  double
      uniqueness_radius; /* 0=automatic checked proposal; positive=recheck. */
  uint32_t flags;
  uint32_t reserved[3];
} crx_cal_v1_options;

typedef struct crx_cal_v1_summary {
  uint32_t struct_size;
  uint32_t version;
  int32_t
      coverage; /* ALWAYS CRX_COVERAGE_INCONCLUSIVE, including zero seeds. */
  uint32_t seed_count;
  uint32_t numerical_count;
  uint32_t numerical_inside_count;
  uint32_t local_count;
  uint32_t admitted_count;
  uint64_t iterations;
  uint64_t backtracks;
  uint64_t evaluations;
  /* sizeof(query object)+seed_count*sizeof(owned record). Excludes allocator
   * overhead, caller buffers and transient preparation/correction stack work.
   */
  uint64_t owned_payload_bytes;
  double length_scale;
  uint32_t reserved[4];
} crx_cal_v1_summary;

typedef struct crx_cal_v1_identity {
  uint32_t struct_size;
  uint32_t version;
  uint32_t model_schema;
  uint32_t model_word_count;
  uint64_t model_words[CRX_CAL_V1_MODEL_WORDS];
  uint64_t target_words[7]; /* Raw quaternion, then position binary64 bits. */
  double length_scale;
  uint32_t limit_units;
  uint32_t reserved[3];
} crx_cal_v1_identity;

typedef struct crx_cal_v1_result {
  uint32_t struct_size;
  uint32_t version;
  uint32_t seed_index;
  int32_t status;
  uint32_t flags;
  int32_t certificate_failure; /* Existing crx_v1_certificate_failure values. */
  int32_t
      local_jacobian_sign; /* Physical increments; LOCAL, never an aspect. */
  int32_t command_jacobian_sign;
  uint32_t iterations;
  uint32_t backtracks;
  uint32_t evaluations;
  uint32_t reserved_counter;
  double seed_commands_rad[6];
  double commands_rad[6];
  double increments_rad[6];
  double normalized_position_error;
  double orientation_error;
  /* Proof fields are NaN unless LOCAL_PROOF is set. A local root is an
   * admissible IK result only if COMMANDS_ADMITTED is also set. Counts/rows
   * retain repeated seeds; no distinct-root/global nearest claim is made. */
  double existence_radius;
  double uniqueness_radius;
  double beta;
  double lipschitz;
  double eta;
  double inverse_residual;
  double coarse_eta;
  double pose_defect;
  double screw_norm_sum;
  double angular_norm_sum;
  double anchor_position_norm;
  double command_reserve_lower;
  double preconditioner[36]; /* Row major, physical increment coordinates. */
  uint32_t reserved[4];
} crx_cal_v1_result;

CRX_API uint32_t crx_cal_v1_abi_version(void);
/* Initialize output's size/version/reserved before requesting defaults.
 * Defaults:24 updates,12 halvings,.25 rad step,1e-13 dispatch tolerances,
 * automatic radius, proofs enabled. Null options in query_create use these. */
CRX_API int32_t crx_cal_v1_options_default(crx_cal_v1_options *output);
CRX_API int32_t crx_cal_v1_create(const crx_cal_v1_model_input *input,
                                  crx_cal_v1_handle **output);
CRX_API void crx_cal_v1_destroy(crx_cal_v1_handle *handle);
/* Numerical FK/body Jacobian, no proof or limit enforcement. Commands use
 * coupled radians; matrices are row major, translation/Jacobian lengths mm.
 * Jacobian twist order(v;omega), columns differentiate command radians. */
CRX_API int32_t crx_cal_v1_fk(const crx_cal_v1_handle *handle,
                              const double *commands_rad, double *pose16);
CRX_API int32_t crx_cal_v1_jacobian(const crx_cal_v1_handle *handle,
                                    const double *commands_rad,
                                    double *jacobian36);
/* Each proposed seed is corrected independently. Nominal partners must both
 * be submitted; no calibrated partner is synthesized from a nominal flip.
 * seed_count0..512; flat seeds array may be null only at count0. A failed
 * correction/admission is inconclusive, never calibrated infeasibility.
 * Query snapshots remain valid after destroying the model. */
CRX_API int32_t crx_cal_v1_query_create(const crx_cal_v1_handle *handle,
                                        const crx_cal_v1_target *target,
                                        const double *seed_commands_rad,
                                        uint32_t seed_count,
                                        const crx_cal_v1_options *options,
                                        crx_cal_v1_query **output);
CRX_API void crx_cal_v1_query_destroy(crx_cal_v1_query *query);
CRX_API int32_t crx_cal_v1_summary_read(const crx_cal_v1_query *query,
                                        crx_cal_v1_summary *output);
CRX_API int32_t crx_cal_v1_identity_read(const crx_cal_v1_query *query,
                                         crx_cal_v1_identity *output);
CRX_API int32_t crx_cal_v1_result_read(const crx_cal_v1_query *query,
                                       uint32_t index,
                                       crx_cal_v1_result *output);

#ifdef __cplusplus
}
#endif
#endif
