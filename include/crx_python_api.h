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
 * 0 for singular postures. All outputs are caller-owned. No seed shortcut is
 * used. Discovery remains numerical and makes no completeness claim.
 */
CRX_API int32_t crx_v1_identify(const crx_v1_handle *handle, const double *pose,
                                int32_t capacity, double *solutions,
                                int32_t *aspect_signs, int32_t *regular,
                                int32_t *wrist_addresses,
                                int32_t *address_valid,
                                crx_v1_fibre_result *result);

#ifdef __cplusplus
}
#endif
#endif
