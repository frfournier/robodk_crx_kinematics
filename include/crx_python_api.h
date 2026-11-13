// SPDX-License-Identifier: Apache-2.0
// SPDX-FileCopyrightText: 2026 Centre Technologique en Aérospatiale

#ifndef CRX_PYTHON_API_H
#define CRX_PYTHON_API_H

#include <stdint.h>

#if defined(_WIN32)
#define CRX_API __declspec(dllexport)
#else
#define CRX_API __attribute__((visibility("default")))
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* ABI 1: Windows x64, IEEE double, cdecl. All matrices are ROW MAJOR.
 * Inputs/outputs are caller-owned, nonoverlapping buffers of the stated size.
 * Handles are immutable; concurrent reads are permitted. Destroy only after
 * all calls finish. Passing a stale/foreign handle is undefined behavior.
 * No function retains array pointers or permits exceptions across the ABI.
 */
typedef struct crx_v1_handle crx_v1_handle;

enum crx_v1_error {
  CRX_OK = 0,
  CRX_INVALID_ARGUMENT = -1,
  CRX_JOINT_LIMIT = -2,
  CRX_CONFIG_UNDEFINED = -3,
  CRX_INTERNAL_ERROR = -4
};

enum crx_v1_ik_status {
  CRX_STRICT_CANDIDATES = 0,
  CRX_STRICT_SEED = 1,
  CRX_STRICT_LOCAL = 2,
  CRX_APPROXIMATE_SEED = 3,
  CRX_NO_CANDIDATE = 4,
  CRX_NO_OUTPUT_REQUESTED = 5,
  CRX_INVALID_INPUT = 6,
  CRX_UNSUPPORTED_MODEL = 7,
  CRX_UNRESOLVED = 8,
  CRX_NUMERICAL_RANGE_FAILURE = 9,
  CRX_WORK_LIMIT = 10
};

typedef struct crx_v1_ik_result {
  int32_t status;
  int32_t count;
  int32_t truncated;
  int32_t used_singular_representatives;
} crx_v1_ik_result;

typedef struct crx_v1_fibre_result {
  int32_t status;
  int32_t count;
  int32_t truncated;
  int32_t used_singular_representatives;
} crx_v1_fibre_result;

enum crx_v1_coverage {
  CRX_COVERAGE_NOT_ATTEMPTED = 0,
  CRX_COVERAGE_COMPLETE = 1,
  CRX_COVERAGE_INCONCLUSIVE = 2,
  CRX_COVERAGE_CONTINUOUS_FIBRE = 3
};

/* Extension negotiation: caller initializes struct_size, version and reserved.
 * Only (sizeof(crx_v1_fibre_metadata), 1, 0) is currently supported.
 * Unsupported sizes/versions/reserved values return -1 without modifying
 * metadata or array outputs. Future layouts require an explicitly supported
 * size/version pair; existing layouts and their meanings remain unchanged.
 */
typedef struct crx_v1_fibre_metadata {
  uint32_t struct_size;
  uint32_t version;
  int32_t coverage;
  uint32_t reserved;
} crx_v1_fibre_metadata;

CRX_API uint32_t crx_v1_abi_version(void);
/* Returned strings have static lifetime; never free them. */
CRX_API const char *crx_v1_library_version(void);
/* IDs 0..6 follow increasing payload: 3iA, 5iA, 10iA, 10iA-L, 20iA-L,
 * 25iA, 30iA. Returns NULL for an unknown ID. */
CRX_API const char *crx_v1_model_name(int32_t model_id);
/* limits_deg: NULL for defaults, otherwise 2x6 [lower; upper].
 * On failure *output is NULL. Override limits must be finite and ordered. */
CRX_API int32_t crx_v1_create(int32_t model_id, const double *limits_deg,
                              crx_v1_handle **output);
CRX_API void crx_v1_destroy(crx_v1_handle *handle);
CRX_API int32_t crx_v1_limits(const crx_v1_handle *handle, double *limits_deg);
/* FK/config/Jacobian take six RoboDK command degrees without recoupling.
 * Output buffers remain unchanged on failure. FK/Jacobian enforce limits.
 * FK output: base-to-flange 4x4, mm. Config: three int32 [REAR,LOWERARM,FLIP].
 * Configuration may be undefined at an arm singularity (error -3).
 * Jacobian: 6x6 geometric [linear;angular], base frame at flange origin,
 * mm/radian and radian/radian. The Python layer converts columns to degrees.
 */
CRX_API int32_t crx_v1_fk(const crx_v1_handle *handle, const double *joints_deg,
                          double *pose);
CRX_API int32_t crx_v1_configuration(const crx_v1_handle *handle,
                                     const double *joints_deg, int32_t *config);
CRX_API int32_t crx_v1_jacobian(const crx_v1_handle *handle,
                                const double *joints_deg, double *jacobian);
/* pose: base-to-flange 4x4 mm; seed: optional six command degrees.
 * capacity must be positive. solutions holds capacity*6 doubles.
 * result is initialized even on failure; solutions is unchanged on failure.
 * Return 0 means result contains a solver outcome, not necessarily solutions.
 * status/count describe strict, approximate, empty, or numerical outcomes.
 * No completeness claim; capacity=1 with a seed enables nearby fast search.
 */
CRX_API int32_t crx_v1_ik(const crx_v1_handle *handle, const double *pose,
                          const double *seed_deg, int32_t capacity,
                          double *solutions, crx_v1_ik_result *result);
/* Additive ABI 1 endpoint. pose is base-to-flange, row-major, in mm.
 * Arrays have capacity rows; wrist_addresses has 3 int32 values per row:
 * [arc, zero-based root rank, base-flip bit]. Arcs follow increasing wrist
 * angle in [-pi,pi); bit 0 is positive shoulder side, bit 1 negative. Invalid
 * addresses use
 * [-1,-1,-1] with address_valid=0. aspect_sign is -1/+1 for regular postures,
 * 0 for numerically singular postures. Signs use canonical radians, not sensed
 * command coordinates: det(J_command) = det(J_canonical) * product(senses).
 * Regularity currently uses |det| > 1e-10*max(1,|a|,|b|,|r|)^3 and is
 * numerical. Addresses are target-local; seams/folds can change arcs/ranks. The
 * base-flip bit is undefined at sin(q[1])=0. Base-flip partners have equal
 * aspect signs; controller [REAR,LOWERARM,FLIP] flags need not identify a
 * unique solution. All outputs are caller-owned. No seed shortcut is used.
 * Discovery remains numerical and makes no completeness claim.
 */
CRX_API int32_t crx_v1_identify(const crx_v1_handle *handle, const double *pose,
                                int32_t capacity, double *solutions,
                                int32_t *aspect_signs, int32_t *regular,
                                int32_t *wrist_addresses,
                                int32_t *address_valid,
                                crx_v1_fibre_result *result);

/* Same arrays, units and numerical status as identify; adds negotiated
 * metadata. Coverage concerns the internal nominal phase catalogue, before
 * command limits, turn expansion and output capacity. Even Complete would not
 * imply an exhaustive returned prefix, resolved command admissibility or
 * nearest-command certification. This implementation always reports
 * NotAttempted. result is initialized on failure when non-NULL.
 */
CRX_API int32_t crx_v1_identify2(const crx_v1_handle *handle,
                                 const double *pose, int32_t capacity,
                                 double *solutions, int32_t *aspect_signs,
                                 int32_t *regular, int32_t *wrist_addresses,
                                 int32_t *address_valid,
                                 crx_v1_fibre_result *result,
                                 crx_v1_fibre_metadata *metadata);

/* Optional local pose certificates. Existing ABI 1 symbols/layouts retain
 * numerical behavior and do not perform proof work. A failed proof is
 * inconclusive, never infeasibility. No local certificate establishes coverage
 * or command limits.
 */
typedef struct crx_v1_certificate_handle crx_v1_certificate_handle;

enum crx_v1_certificate_failure {
  CRX_CERTIFIED = 0,
  CRX_CERT_INVALID_INPUT = 1,
  CRX_CERT_UNSUPPORTED_ENVIRONMENT = 2,
  CRX_CERT_KINEMATICS_UNAVAILABLE = 3,
  CRX_CERT_INVALID_PRECONDITIONER = 4,
  CRX_CERT_INVERSE_RESIDUAL = 5,
  CRX_CERT_UNCERTAIN_SIGN = 6,
  CRX_CERT_TARGET_LOG_DOMAIN = 7,
  CRX_CERT_RADIUS_INEQUALITY = 8,
  CRX_CERT_ARITHMETIC_UNAVAILABLE = 9,
  CRX_CERT_ASPECT_DISAGREEMENT = 10,
  CRX_CERT_IDENTITY_MISMATCH = 11,
  CRX_CERT_NOT_STRICT = 12,
  CRX_CERT_NOT_REQUESTED = 13,
  CRX_CERT_UNSUPPORTED_EXACT_MODEL = 14
};

enum crx_v1_certificate_flags { CRX_CERTIFICATION_DISABLED = 1 };

/* Initialize (sizeof,1,flags,0). Only flags=0 or DISABLED is supported.
 * DISABLED measures the same numerical nomination path without proof work.
 */
typedef struct crx_v1_certificate_options {
  uint32_t struct_size;
  uint32_t version;
  uint32_t flags;
  uint32_t reserved;
} crx_v1_certificate_options;

/* Initialize struct_size/version/reserved to (sizeof,1,0). Read negotiates this
 * exact layout before modifying it. All values are owned snapshots. rho/R are
 * CANONICAL RADIAN infinity radii around canonical_center_rad, not around a
 * rounded degree command. Existence holds in rho, uniqueness in R. Signs use
 * the canonical body Jacobian. length_scale converts normalized lengths to mm.
 * rotation_deviation_upper bounds input/projected Frobenius defects for
 * target/reference/tool. input_words are 77 exact binary64 encodings in order:
 * a,b,c,r,height; raw theta0[6],senses[6],lower/upper radian bounds[6 each];
 * target,reference,tool each quaternion[wxyz],position[3],input R[9 row-major].
 * Schema 2 means the raw forward-reference composition; native height cancels
 * its exact opposite base shift. Bounds do not certify original degree limits.
 */
typedef struct crx_v1_ball_certificate {
  uint32_t struct_size;
  uint32_t version;
  uint32_t reserved;
  uint32_t identity_schema;
  int32_t aspect_sign;
  int32_t reused;
  int32_t used_log_series;
  int32_t reuse_failure;
  double canonical_center_rad[6];
  double existence_radius_rad;
  double uniqueness_radius_rad;
  double beta;
  double lipschitz;
  double eta;
  double coarse_eta;
  double inverse_residual;
  double pose_defect;
  double screw_norm_sum;
  double angular_norm_sum;
  double anchor_position_norm;
  double length_scale;
  double rotation_deviation_upper[3];
  double target_relative_trace[2];
  double closed_form_determinant[2];
  double preconditioner[36];
  uint64_t input_words[77];
} crx_v1_ball_certificate;

/* Immutable certificate handles own their proof, independent of solver
 * lifetime. Destroy each returned non-NULL handle exactly once; NULL
 * destruction is safe. Handles must originate from this loaded library.
 * Concurrent reads are safe; destroy only after all readers/reuse calls finish.
 * Stale/foreign pointers are UB.
 */
CRX_API void crx_v1_certificate_destroy(crx_v1_certificate_handle *certificate);
CRX_API int32_t
crx_v1_certificate_read(const crx_v1_certificate_handle *certificate,
                        crx_v1_ball_certificate *output);

/* pose is requested TCP in its reference; reference maps reference -> native
 * base, tool maps flange -> TCP. Each is a required row-major rigid 4x4 in mm.
 * Ordinary composition nominates flange poses; original quaternion/frame inputs
 * define the one exact target used by proof. Certificates/failures have
 * capacity rows; only result.count rows are written. Failure=0 iff handle is
 * non-NULL. Approximate results have NOT_STRICT and NULL handles. Caller owns
 * successes. reuse may borrow a prior immutable handle, without consuming it.
 * Every model identity word, target domain and uniqueness-radius reserve is
 * revalidated; failure falls back to fresh proof. Numerical refinement/search
 * remains intact. Unsupported negotiation/invalid arguments leave
 * arrays/options unchanged; result is initialized when non-NULL. No pointer is
 * retained by the call.
 */
CRX_API int32_t crx_v1_ik_certified(const crx_v1_handle *handle,
                                    const double *pose, const double *reference,
                                    const double *tool, const double *seed_deg,
                                    int32_t capacity, double *solutions,
                                    crx_v1_ik_result *result,
                                    crx_v1_certificate_handle **certificates,
                                    int32_t *failures,
                                    const crx_v1_certificate_options *options,
                                    const crx_v1_certificate_handle *reuse);

/* Same raw-frame contract and ownership. Full numerical discovery remains
 * NotAttempted coverage. Successful proofs upgrade only their regularity/aspect
 * labels; NULL rows retain the numerical policy. Metadata negotiation follows
 * identify2. Handle presence identifies a certified regularity label.
 */
CRX_API int32_t crx_v1_identify_certified(
    const crx_v1_handle *handle, const double *pose, const double *reference,
    const double *tool, int32_t capacity, double *solutions,
    int32_t *aspect_signs, int32_t *regular, int32_t *wrist_addresses,
    int32_t *address_valid, crx_v1_fibre_result *result,
    crx_v1_fibre_metadata *metadata, crx_v1_certificate_handle **certificates,
    int32_t *failures, const crx_v1_certificate_options *options);

#ifdef __cplusplus
}
#endif
#endif
