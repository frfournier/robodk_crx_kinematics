#define _USE_MATH_DEFINES

#include "crx_solver.h"

#include <algorithm>
#include <array>
#include <cassert>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <vector>

#include <Eigen/Geometry>

#include "crx_math_helpers.h"
#include "crx_pose_helpers.h"
#include "crx_vector_helpers.h"

namespace crx {
namespace {

static constexpr int kDhArgCount = 4;
static constexpr int kDhThetaIndex = 2;
static constexpr int kDhDIndex = 3;

static constexpr int kIkQSamples = 1440;
static constexpr int kRefineMaxIterations = 100;
static constexpr double kRefineXTolerance = 1e-12;
static constexpr double kRootZeroTolerance = 1e-14;

static constexpr double kSolutionAngleToleranceDeg = 1e-3;
static constexpr double kSolutionAngleToleranceRad =
    angle_conv::DegToRad(kSolutionAngleToleranceDeg);
static constexpr double kSolutionPositionToleranceMm = 1e-4;
static constexpr double kSolutionPositionToleranceMmSquared =
    kSolutionPositionToleranceMm * kSolutionPositionToleranceMm;

static constexpr double kJointLimitToleranceDeg = 1e-2;
static constexpr double kJointLimitToleranceRad =
    angle_conv::DegToRad(kJointLimitToleranceDeg);

static constexpr double kDhConventionToleranceRad = 1e-5;
static constexpr double kTriangleNegativeTolerance = 1e-9;
static constexpr double kTriangleShellRelativeTolerance =
    static_cast<double>(1.0e-6);

static constexpr std::size_t kMaxIkSolutions = 32;

static inline void BuildJointPoseInputRad(const DhRow &dh_row,
                                          double joint_motion_rad,
                                          real_T dh_pose_args[kDhArgCount]) {
  dh_pose_args[0] = static_cast<real_T>(SnapToRightAngleFamily(dh_row.alpha_rad));
  dh_pose_args[1] = static_cast<real_T>(dh_row.a);
  dh_pose_args[kDhThetaIndex] =
      static_cast<real_T>(SnapToRightAngleFamily(dh_row.theta0_rad));
  dh_pose_args[kDhDIndex] = static_cast<real_T>(dh_row.d);

  if (!dh_row.is_prismatic)
    dh_pose_args[kDhThetaIndex] += static_cast<real_T>(joint_motion_rad);
  else
    dh_pose_args[kDhDIndex] += static_cast<real_T>(joint_motion_rad);
}

static auto SolveFKCore(const Vec6 &user_joints_rad, PoseIsoRT &pose_out,
                        std::vector<PoseIsoRT> *joint_poses_out,
                        bool check_limits, const CrxModelData &model) -> int {
  if (joint_poses_out != nullptr &&
      joint_poses_out->size() < static_cast<std::size_t>(kDofCount + 1))
    return -1;

  PoseIsoRT accumulated_pose = model.base_transform;
  if (joint_poses_out != nullptr)
    (*joint_poses_out)[0] = accumulated_pose;

  const double sensed_joint2_rad =
      user_joints_rad[kJoint2Index] * model.joint_senses[kJoint2Index];

  for (int joint_id = 0; joint_id < kDofCount; ++joint_id) {
    const double sensed_joint_rad =
        user_joints_rad[joint_id] * model.joint_senses[joint_id];

    if (check_limits && (sensed_joint_rad < model.lower_limits_rad[joint_id] ||
                         sensed_joint_rad > model.upper_limits_rad[joint_id]))
      return -2;

    double joint_model_rad = sensed_joint_rad;
    if (joint_id == kJoint3Index)
      joint_model_rad = -sensed_joint2_rad + sensed_joint_rad;

    real_T dh_pose_args[kDhArgCount];
    BuildJointPoseInputRad(model.dh_rows[joint_id], joint_model_rad, dh_pose_args);

    accumulated_pose = accumulated_pose *
                       DHM_FromRad(dh_pose_args[0], dh_pose_args[1],
                                   dh_pose_args[2], dh_pose_args[3]);

    if (joint_poses_out != nullptr)
      (*joint_poses_out)[joint_id + 1] = accumulated_pose;
  }

  pose_out = accumulated_pose * FixedJ6ToToolIsometryFk() * model.tool_transform;
  return 1;
}

static auto IsFkRoundtripValid(const Vec6 &candidate_user_joints_rad,
                               const PoseIsoRT &target_pose,
                               const Eigen::Quaterniond &target_quat,
                               const CrxModelData &model) -> bool {
  PoseIsoRT fk_pose_isometry = PoseIsoRT::Identity();
  if (SolveFKCore(candidate_user_joints_rad, fk_pose_isometry, nullptr, false,
                  model) != 1)
    return false;

  const Vec3 position_error = fk_pose_isometry.translation().cast<double>() -
                              target_pose.translation().cast<double>();
  if (!std::isfinite(position_error.squaredNorm()) ||
      position_error.squaredNorm() > kSolutionPositionToleranceMmSquared)
    return false;

  const Eigen::Quaterniond fk_quaternion(
      fk_pose_isometry.linear().cast<double>());
  const double orientation_error_rad =
      fk_quaternion.angularDistance(target_quat);
  return std::isfinite(orientation_error_rad) &&
         orientation_error_rad <= kSolutionAngleToleranceRad;
}
struct CrxParams {
  std::array<double, kDofCount> alpha_rad{};
  std::array<double, kDofCount> a{};
  std::array<double, kDofCount> d{};
  std::array<double, kDofCount> theta0_rad{};
  double a2 = 0.0, r4 = 0.0, r5 = 0.0, r6 = 0.0;
};

struct IkTolerancePack {
  double length_scale = 1.0;
  double eps_len = 1e-10;
  double eps_len2 = 1e-20;
  double eps_xy = 1e-8;
  double eps_sin = 1e-8;
  double eps_reach_sq = 1e-9;
};

static inline auto BuildIkTolerancePack(const CrxParams &params)
    -> IkTolerancePack {
  IkTolerancePack tolerances;
  tolerances.length_scale =
      std::max({std::abs(params.a2), std::abs(params.r4), std::abs(params.r5),
                std::abs(params.r6), 1.0});
  const double machine_epsilon = std::numeric_limits<double>::epsilon();
  tolerances.eps_len =
      std::max(1e-12, 8.0 * machine_epsilon * tolerances.length_scale);
  tolerances.eps_len2 = tolerances.eps_len * tolerances.eps_len;
  tolerances.eps_xy =
      std::max(1e-12, 4096.0 * machine_epsilon * tolerances.length_scale);
  tolerances.eps_sin = 1e-10;
  tolerances.eps_reach_sq = std::max(
      1e-12, machine_epsilon * tolerances.length_scale *
                 tolerances.length_scale);
  return tolerances;
}

static auto ReadCrxParams(const CrxModelData &model, CrxParams &params) -> bool {
  for (int joint_id = 0; joint_id < kDofCount; ++joint_id) {
    const DhRow &dh_row = model.dh_rows[joint_id];
    if (dh_row.is_prismatic)
      return false; // prismatic not supported
    params.alpha_rad[joint_id] = SnapToRightAngleFamily(dh_row.alpha_rad);
    params.a[joint_id] = dh_row.a;
    params.theta0_rad[joint_id] = SnapToRightAngleFamily(dh_row.theta0_rad);
    params.d[joint_id] = dh_row.d;
  }
  params.a2 = params.a[2];
  params.r4 = params.d[3];
  params.r5 = params.d[4];
  params.r6 = params.d[5];
  return true;
}

static auto ConvertRoboDkDhToAnalyticIkConvention(CrxParams &params,
                                                  double &base_z_shift_mm)
    -> bool {
  // RoboDK stores the production FK model convention. The geometric IK in
  // Sec. 2.6 assumes a normalized internal convention; this adapter validates
  // expected CRX DH signatures and remaps only if they match exactly.
  base_z_shift_mm = 0.0;
  const double tol = kDhConventionToleranceRad;

  const bool alpha_ok = AngleDiffAbs(params.alpha_rad[0], 0.0) < tol &&
                        AngleDiffAbs(params.alpha_rad[1], -kHalfPi) < tol &&
                        AngleDiffAbs(params.alpha_rad[2], 0.0) < tol &&
                        AngleDiffAbs(params.alpha_rad[3], -kHalfPi) < tol &&
                        AngleDiffAbs(params.alpha_rad[4], kHalfPi) < tol &&
                        AngleDiffAbs(params.alpha_rad[5], -kHalfPi) < tol;

  const bool theta_ok = AngleDiffAbs(params.theta0_rad[0], 0.0) < tol &&
                        AngleDiffAbs(params.theta0_rad[1], -kHalfPi) < tol &&
                        AngleDiffAbs(params.theta0_rad[2], 0.0) < tol &&
                        AngleDiffAbs(params.theta0_rad[3], 0.0) < tol &&
                        AngleDiffAbs(params.theta0_rad[4], 0.0) < tol &&
                        AngleDiffAbs(params.theta0_rad[5], 0.0) < tol;

  const bool a_ok =
      std::abs(params.a[0]) <= kEpsilon && std::abs(params.a[1]) <= kEpsilon &&
      params.a[2] > 0.0 && std::abs(params.a[3]) <= kEpsilon &&
      std::abs(params.a[4]) <= kEpsilon && std::abs(params.a[5]) <= kEpsilon;

  const bool d_ok = params.d[0] > 0.0 && std::abs(params.d[1]) <= kEpsilon &&
                    std::abs(params.d[2]) <= kEpsilon && params.d[3] > 0.0 &&
                    params.d[4] <= -kEpsilon && // Pass-1 Fix #2
                    params.d[5] > 0.0;

  // Fail closed when a model differs from the CRX assumptions:
  // returning -1 asks RoboDK to use its generic numerical IK.
  if (!(alpha_ok && theta_ok && a_ok && d_ok))
    return false;

  // The next remaps align RoboDK FK convention with the paper's internal frame
  // setup. base_z_shift_mm is undone on T06 in SolveIK to keep external
  // behavior unchanged.
  base_z_shift_mm = params.d[0];
  params.d[0] = 0.0;
  params.alpha_rad[2] = M_PI;
  params.d[3] = -params.d[3];
  params.d[4] = -params.d[4];
  params.d[5] = -params.d[5];
  params.a2 = params.a[2];
  params.r4 = params.d[3];
  params.r5 = params.d[4];
  params.r6 = params.d[5];
  return true;
}

struct PlaneBasisXY {
  Vec3 x = Vec3::Zero();
  Vec3 y = Vec3::Zero();
};

// Specialized hot-path basis build for ConstructPlane(O4, UnitZ()).
static inline auto ConstructPlane_O4_UnitZ_XY(const Vec3 &O4,
                                              const IkTolerancePack &tolerances,
                                              PlaneBasisXY &basis) -> bool {
  const double o4_norm_sq = O4.squaredNorm();
  if (!std::isfinite(o4_norm_sq) || o4_norm_sq <= tolerances.eps_len2)
    return false;

  const Vec3 plane_x = O4 * (1.0 / std::sqrt(o4_norm_sq));

  Vec3 plane_y = Vec3::UnitZ() - plane_x.z() * plane_x;
  double plane_y_norm_sq = plane_y.squaredNorm();
  if (!std::isfinite(plane_y_norm_sq) || plane_y_norm_sq <= tolerances.eps_len2) {
    const Vec3 fallback_axis =
        (std::abs(plane_x.z()) > 0.99) ? Vec3::UnitY() : Vec3::UnitZ();
    plane_y = fallback_axis - plane_x.dot(fallback_axis) * plane_x;
    plane_y_norm_sq = plane_y.squaredNorm();
    if (!std::isfinite(plane_y_norm_sq) ||
        plane_y_norm_sq <= tolerances.eps_len2) {
      plane_y = plane_x.unitOrthogonal();
      plane_y_norm_sq = plane_y.squaredNorm();
      if (!std::isfinite(plane_y_norm_sq) ||
          plane_y_norm_sq <= tolerances.eps_len2)
        return false;
    }
  }
  plane_y *= (1.0 / std::sqrt(plane_y_norm_sq));

  basis.x = plane_x;
  basis.y = plane_y;

#ifndef NDEBUG
  const Vec3 plane_z = basis.x.cross(basis.y);
  const double det_like = basis.x.dot(basis.y.cross(plane_z));
  assert(std::abs(det_like - 1.0) < 1e-9 &&
         "ConstructPlane_O4_UnitZ_XY: basis is not orthonormal");
#endif
  return true;
}

static inline auto FindThirdTriangleCorner(double side_ab, double side_ac,
                                           double side_bc, double &x_coord,
                                           double &y_abs) -> bool {
  if (!(std::isfinite(side_ab) && std::isfinite(side_ac) &&
        std::isfinite(side_bc)))
    return false;
  if (side_ab <= kEpsilon)
    return false;

  x_coord = (side_ac * side_ac - side_bc * side_bc + side_ab * side_ab) /
            (2.0 * side_ab);
  const double y_squared = side_ac * side_ac - x_coord * x_coord;
  if (!std::isfinite(y_squared))
    return false;

  const double side_scale = std::max(
      std::max(std::abs(side_ab), std::abs(side_ac)), std::abs(side_bc));
  const double shell_len_tol =
      kTriangleShellRelativeTolerance * std::max(1.0, side_scale);
  const double eps_shell =
      std::max(kTriangleNegativeTolerance, shell_len_tol * shell_len_tol);

  if (y_squared < -eps_shell)
    return false;
  y_abs = std::sqrt((y_squared < 0.0) ? 0.0 : y_squared);
  return true;
}

static inline auto JointTransformRad(const CrxParams &params, int joint_id,
                                     double joint_angle_rad) -> PoseIsoRT {
  return DHM_FromRad(
      static_cast<real_T>(params.alpha_rad[joint_id]),
      static_cast<real_T>(params.a[joint_id]),
      static_cast<real_T>(params.theta0_rad[joint_id] + joint_angle_rad),
      static_cast<real_T>(params.d[joint_id]));
}

static auto DetermineJointValues(const Vec3 &o3_point, const Vec3 &o4_point,
                                 const Vec3 &o5_point,
                                 const PoseIsoRT &target_pose_06,
                                 const CrxParams &params,
                                 const IkTolerancePack &tolerances,
                                 const Vec6 *approx_joints_rad,
                                 Vec6 &out_joints_rad)
    -> bool {
  // This back-substitution corresponds to the paper's Step 6 (Eq. 16..22):
  // once a valid O3/O4 pair is known, solve J1..J6 from chained frame
  // reductions.
  const Vec3 o6_point = target_pose_06.translation().cast<double>();

  const double raw_J1 = std::atan2(o4_point.y(), o4_point.x());
  double J1 = raw_J1;
  const double shoulder_radius = std::hypot(o4_point.x(), o4_point.y());
  if (shoulder_radius < tolerances.eps_xy && approx_joints_rad != nullptr)
    J1 = (*approx_joints_rad)[0];

  const Mat3 rotation_l1_from_l0 =
      JointTransformRad(params, 0, J1).inverse().linear().cast<double>();

  const Vec3 o3_in_l1 = rotation_l1_from_l0 * o3_point;
  const double J2 = std::atan2(o3_in_l1.x(), o3_in_l1.z());

  const Vec3 o4_in_l1 = rotation_l1_from_l0 * o4_point;
  const Vec3 o4_minus_o3_in_l1 = o4_in_l1 - o3_in_l1;
  const double raw_J3 =
      std::atan2(o4_minus_o3_in_l1.z(), o4_minus_o3_in_l1.x());
  double J3 = raw_J3;
  const double elbow_radius =
      std::hypot(o4_minus_o3_in_l1.x(), o4_minus_o3_in_l1.z());
  if (elbow_radius < tolerances.eps_xy && approx_joints_rad != nullptr)
    J3 = (*approx_joints_rad)[2];

  const PoseIsoRT transform_l2_from_l1 =
      JointTransformRad(params, 1, J2).inverse();
  // Joint 3 uses the coupled variable (J2 + J3) in the DH chain.
  const PoseIsoRT transform_l3_from_l2 =
      JointTransformRad(params, 2, J2 + J3).inverse();
  const PoseIsoRT transform_l3_from_l0 = transform_l3_from_l2 *
                                         transform_l2_from_l1 *
                                         PoseIsoRT(rotation_l1_from_l0.cast<real_T>());

  const Vec3 o5_in_l3 = (transform_l3_from_l0 * o5_point.cast<real_T>())
                            .template head<3>()
                            .cast<double>();
  const double J4 = std::atan2(o5_in_l3.x(), o5_in_l3.z());

  const PoseIsoRT transform_l4_from_l0 =
      JointTransformRad(params, 3, J4).inverse() * transform_l3_from_l0;

  const Vec3 o6_in_l4 = (transform_l4_from_l0 * o6_point.cast<real_T>())
                            .template head<3>()
                            .cast<double>();
  const double J5 = std::atan2(o6_in_l4.x(), -o6_in_l4.z());

  const PoseIsoRT transform_l5_from_l0 =
      JointTransformRad(params, 4, J5).inverse() * transform_l4_from_l0;
  const Vec3 tool_x_axis_in_l5 =
      (transform_l5_from_l0.linear() * target_pose_06.linear())
          .cast<double>()
          .col(0);
  const double raw_J6 = std::atan2(-tool_x_axis_in_l5.z(), tool_x_axis_in_l5.x());
  double J6 = raw_J6;
  const double sin_j5 = std::sin(J5);
  if (std::abs(sin_j5) < tolerances.eps_sin && approx_joints_rad != nullptr)
    J6 = (*approx_joints_rad)[5];

  if (!(std::isfinite(J1) && std::isfinite(J2) && std::isfinite(J3) &&
        std::isfinite(J4) && std::isfinite(J5) && std::isfinite(J6)))
    return false;

  out_joints_rad << NormalizeRadKeepSignedPi(J1), NormalizeRadKeepSignedPi(J2),
      NormalizeRadKeepSignedPi(J3), NormalizeRadKeepSignedPi(J4),
      NormalizeRadKeepSignedPi(J5), NormalizeRadKeepSignedPi(J6);

  return true;
}

// ─────────────────────────── Circle sweep ───────────────────────────────────

struct CircleEvalContext {
  Vec3 o4_base = Vec3::Zero();
  Vec3 o4_cos_axis = Vec3::Zero();
  Vec3 o4_sin_axis = Vec3::Zero();
  Vec3 o5_point = Vec3::Zero();
  double reach_sq_clamp_tolerance = 0.0;
  double min_o0_o4_dist_sq = 0.0;
  double max_o0_o4_dist_sq = 0.0;
  bool has_min_o0_o4_bound = false;
};

struct CircleEvaluation {
  Vec3 o4_point = Vec3::Zero();
  Vec3 o3_up_point = Vec3::Zero();
  Vec3 o3_down_point = Vec3::Zero();
  double up_dot_product = std::numeric_limits<double>::quiet_NaN();
  double down_dot_product = std::numeric_limits<double>::quiet_NaN();
};

struct CircleDotEvaluation {
  Vec3 o4_point = Vec3::Zero();
  double triangle_x_coord = 0.0;
  double triangle_y_abs = 0.0;
  double up_dot_product = std::numeric_limits<double>::quiet_NaN();
  double down_dot_product = std::numeric_limits<double>::quiet_NaN();
};

static auto EvaluateCircleDots_cs(double cos_q, double sin_q,
                                  const CircleEvalContext &context,
                                  const CrxParams &params,
                                  const IkTolerancePack &tolerances,
                                  CircleDotEvaluation &evaluation,
                                  PlaneBasisXY *basis_out = nullptr) -> bool {
  // Step 2/3/4 core: for one q sample, compute O4(q), both O3 branches, then
  // the two normalized dot products UP(q)/DW(q) whose zeros define valid
  // solutions.
  evaluation.o4_point = context.o4_base + context.o4_cos_axis * cos_q +
                        context.o4_sin_axis * sin_q;

  double o0_o4_dist_sq = evaluation.o4_point.squaredNorm();
  if (!std::isfinite(o0_o4_dist_sq) || o0_o4_dist_sq <= tolerances.eps_len2)
    return false;
  if (o0_o4_dist_sq >
      context.max_o0_o4_dist_sq + context.reach_sq_clamp_tolerance)
    return false;
  if (o0_o4_dist_sq > context.max_o0_o4_dist_sq)
    o0_o4_dist_sq = context.max_o0_o4_dist_sq;
  if (context.has_min_o0_o4_bound) {
    if (o0_o4_dist_sq <
        context.min_o0_o4_dist_sq - context.reach_sq_clamp_tolerance)
      return false;
    if (o0_o4_dist_sq < context.min_o0_o4_dist_sq)
      o0_o4_dist_sq = context.min_o0_o4_dist_sq;
  }

  const double o0_o4_dist = std::sqrt(o0_o4_dist_sq);
  // The paper's triangle uses signed CRX lengths; -r4 preserves that
  // convention.
  if (!FindThirdTriangleCorner(o0_o4_dist, params.a2, -params.r4,
                               evaluation.triangle_x_coord,
                               evaluation.triangle_y_abs))
    return false;

  PlaneBasisXY basis;
  if (!ConstructPlane_O4_UnitZ_XY(evaluation.o4_point, tolerances, basis))
    return false;
  if (basis_out != nullptr)
    *basis_out = basis;

  const Vec3 o4_to_o5_vector = context.o5_point - evaluation.o4_point;
  const double o4_to_o5_norm_sq = o4_to_o5_vector.squaredNorm();
  if (!std::isfinite(o4_to_o5_norm_sq) || o4_to_o5_norm_sq <= tolerances.eps_len2)
    return false;
  const double inv_o4_to_o5_norm = 1.0 / std::sqrt(o4_to_o5_norm_sq);

  const double dx = o0_o4_dist - evaluation.triangle_x_coord;
  const double y_abs = evaluation.triangle_y_abs;
  const double o3_to_o4_norm_sq = dx * dx + y_abs * y_abs;
  if (!std::isfinite(o3_to_o4_norm_sq) || o3_to_o4_norm_sq <= tolerances.eps_len2)
    return false;
  const double inv_o3_to_o4_norm = 1.0 / std::sqrt(o3_to_o4_norm_sq);

  const double o4_to_o5_x = o4_to_o5_vector.dot(basis.x);
  const double o4_to_o5_y = o4_to_o5_vector.dot(basis.y);
  if (!std::isfinite(o4_to_o5_x) || !std::isfinite(o4_to_o5_y))
    return false;

  const double up_dot_raw = dx * o4_to_o5_x - y_abs * o4_to_o5_y;
  const double down_dot_raw = dx * o4_to_o5_x + y_abs * o4_to_o5_y;
  evaluation.up_dot_product = ClampCosineNearUnit(
      up_dot_raw * inv_o3_to_o4_norm * inv_o4_to_o5_norm);
  evaluation.down_dot_product = ClampCosineNearUnit(
      down_dot_raw * inv_o3_to_o4_norm * inv_o4_to_o5_norm);
  return std::isfinite(evaluation.up_dot_product) &&
         std::isfinite(evaluation.down_dot_product) &&
         std::abs(evaluation.up_dot_product) <= 1.0 + kCosineClampTolerance &&
         std::abs(evaluation.down_dot_product) <= 1.0 + kCosineClampTolerance;
}

static auto EvaluateCircleFull_cs(double cos_q, double sin_q,
                                  const CircleEvalContext &context,
                                  const CrxParams &params,
                                  const IkTolerancePack &tolerances,
                                  CircleEvaluation &evaluation) -> bool {
  CircleDotEvaluation dot_eval;
  PlaneBasisXY basis;
  if (!EvaluateCircleDots_cs(cos_q, sin_q, context, params, tolerances,
                             dot_eval, &basis))
    return false;

  evaluation.o4_point = dot_eval.o4_point;
  evaluation.o3_up_point = basis.x * dot_eval.triangle_x_coord +
                           basis.y * dot_eval.triangle_y_abs;
  evaluation.o3_down_point = basis.x * dot_eval.triangle_x_coord -
                             basis.y * dot_eval.triangle_y_abs;
  evaluation.up_dot_product = dot_eval.up_dot_product;
  evaluation.down_dot_product = dot_eval.down_dot_product;
  return true;
}

static auto EvaluateCircleDots(double sample_q, const CircleEvalContext &context,
                               const CrxParams &params,
                               const IkTolerancePack &tolerances,
                               CircleDotEvaluation &evaluation) -> bool {
  double sin_q = 0.0;
  double cos_q = 1.0;
  SinCos(sample_q, sin_q, cos_q);
  return EvaluateCircleDots_cs(cos_q, sin_q, context, params, tolerances,
                               evaluation);
}

static auto EvaluateCircleFull(double sample_q, const CircleEvalContext &context,
                               const CrxParams &params,
                               const IkTolerancePack &tolerances,
                               CircleEvaluation &evaluation) -> bool {
  double sin_q = 0.0;
  double cos_q = 1.0;
  SinCos(sample_q, sin_q, cos_q);
  return EvaluateCircleFull_cs(cos_q, sin_q, context, params, tolerances,
                               evaluation);
}

static inline auto HasBracket(double f_left, double f_right) -> bool {
  if (!(std::isfinite(f_left) && std::isfinite(f_right)))
    return false;
  if (std::abs(f_left) <= kRootZeroTolerance ||
      std::abs(f_right) <= kRootZeroTolerance)
    return true;
  return f_left * f_right < 0.0;
}

template <typename EvalFn>
static auto RefineZeroIllinois(double q_left, double q_right, double f_left,
                               double f_right, EvalFn eval_fn,
                               double &root_out_q) -> bool {
  // Keep the bracket invariant but reduce evaluations vs pure bisection.
  if (!(std::isfinite(f_left) && std::isfinite(f_right)))
    return false;
  if (q_right < q_left) {
    std::swap(q_left, q_right);
    std::swap(f_left, f_right);
  }
  if (std::abs(f_left) <= kRootZeroTolerance) {
    root_out_q = WrapRad2Pi(q_left);
    return true;
  }
  if (std::abs(f_right) <= kRootZeroTolerance) {
    root_out_q = WrapRad2Pi(q_right);
    return true;
  }
  if (f_left * f_right > 0.0)
    return false;

  double left_q = q_left;
  double right_q = q_right;
  // Weighted endpoint values used by Illinois' secant formula.
  double left_weight_f = f_left;
  double right_weight_f = f_right;
  // True endpoint function values (never damped), used for sign checks.
  double left_true_f = f_left;
  double right_true_f = f_right;
  double best_q =
      (std::abs(f_left) <= std::abs(f_right)) ? q_left : q_right;
  double best_abs_f = std::min(std::abs(f_left), std::abs(f_right));
  // -1: right endpoint moved, +1: left endpoint moved.
  int last_update_side = 0;
  int consecutive_same_side_updates = 0;
  for (int iter = 0; iter < kRefineMaxIterations; ++iter) {
    double next_q = 0.5 * (left_q + right_q);
    // Hybrid safeguard: if the same side keeps updating, force a midpoint step
    // to guarantee interval shrink in flat/singular regions.
    const bool force_midpoint = consecutive_same_side_updates >= 2;
    if (!force_midpoint) {
      const double denom = right_weight_f - left_weight_f;
      if (std::isfinite(denom) && std::abs(denom) > kEpsilon) {
        const double secant_q =
            (left_q * right_weight_f - right_q * left_weight_f) / denom;
        if (std::isfinite(secant_q) && secant_q > left_q && secant_q < right_q)
          next_q = secant_q;
      }
    }

    const double next_f = eval_fn(next_q);
    if (!std::isfinite(next_f))
      return false;
    if (std::abs(next_f) < best_abs_f) {
      best_abs_f = std::abs(next_f);
      best_q = next_q;
    }
    if (std::abs(next_f) <= kRootZeroTolerance ||
        (right_q - left_q) < kRefineXTolerance) {
      root_out_q = WrapRad2Pi(next_q);
      return true;
    }

    if (left_true_f * next_f < 0.0) {
      right_q = next_q;
      right_true_f = next_f;
      right_weight_f = next_f;
      if (last_update_side == -1)
        left_weight_f *= 0.5; // Illinois damping on repeated side updates.
      consecutive_same_side_updates =
          (last_update_side == -1) ? (consecutive_same_side_updates + 1) : 1;
      last_update_side = -1;
    } else if (right_true_f * next_f < 0.0) {
      left_q = next_q;
      left_true_f = next_f;
      left_weight_f = next_f;
      if (last_update_side == 1)
        right_weight_f *= 0.5; // Illinois damping on repeated side updates.
      consecutive_same_side_updates =
          (last_update_side == 1) ? (consecutive_same_side_updates + 1) : 1;
      last_update_side = 1;
    } else {
      // Includes exact zero and rare same-sign tie from floating-point noise.
      root_out_q = WrapRad2Pi(next_q);
      return true;
    }
  }
  root_out_q = WrapRad2Pi(best_q);
  return true;
}

static void DualSolutionRad(const Vec6 &source_solution,
                            Vec6 &dual_solution_out) {
  // Eq. (23) dual map (paper Sec. 2.6, Step 7), expressed in radians:
  // [J1-PI, -J2, PI-J3, J4-PI, J5, J6].
  dual_solution_out = source_solution;
  dual_solution_out[0] = source_solution[0] - M_PI;
  dual_solution_out[1] = -source_solution[1];
  dual_solution_out[2] = M_PI - source_solution[2];
  dual_solution_out[3] = source_solution[3] - M_PI;
  dual_solution_out[4] = source_solution[4];
  dual_solution_out[5] = source_solution[5];
  NormalizeUserSolutionDomains(dual_solution_out);
}

template <typename EmitFn>
static auto ForEachSignedPiVariant(const Vec6 &base_solution, EmitFn emit)
    -> bool {
  constexpr double kPiFlipTol = 1e-8;

  std::vector<int> pi_flip_joint_ids;
  pi_flip_joint_ids.reserve(kDofCount);
  for (int joint_id = 0; joint_id < kDofCount; ++joint_id)
    if (std::abs(std::abs(base_solution[joint_id]) - M_PI) <= kPiFlipTol)
      pi_flip_joint_ids.push_back(joint_id);

  // +PI and -PI represent the same physical angle but can map to different
  // controller branches/joint-limit edges. Emit both to keep branch choice
  // stable.
  const auto variant_count =
      static_cast<std::uint32_t>(1u << pi_flip_joint_ids.size());
  for (std::uint32_t mask = 0; mask < variant_count; ++mask) {
    Vec6 variant_solution = base_solution;
    for (std::size_t bit = 0; bit < pi_flip_joint_ids.size(); ++bit) {
      if ((mask & (1u << bit)) == 0u)
        continue;
      const int joint_id = pi_flip_joint_ids[bit];
      variant_solution[joint_id] =
          (base_solution[joint_id] >= 0.0) ? -M_PI : M_PI;
    }
    if (!emit(variant_solution))
      return false;
  }
  return true;
}

template <typename EmitFn>
static auto ForEachCandidateVariant(const Vec6 &base_solution, EmitFn emit)
    -> bool {
  if (!ForEachSignedPiVariant(base_solution,
                              [&](const Vec6 &signed_pi_variant) {
                                return emit(signed_pi_variant);
                              })) {
    return false;
  }

  Vec6 dual_solution = Vec6::Zero();
  DualSolutionRad(base_solution, dual_solution);
  return ForEachSignedPiVariant(dual_solution, [&](const Vec6 &dual_variant) {
    return emit(dual_variant);
  });
}

struct Candidate {
  Vec6 user_joints_rad = Vec6::Zero();
  double distance_sq = 0.0;
};

// ─────────────────────────── Closed-form CRX IK ─────────────────────────────

static void SolveCrxIk(const PoseIsoRT &target_pose_06, const CrxParams &params,
                       const Vec6 *approx_joints_rad,
                       std::vector<Vec6> &solutions_out) {
  solutions_out.clear();
  solutions_out.reserve(kMaxIkSolutions);

  const IkTolerancePack tolerances = BuildIkTolerancePack(params);

  CircleEvalContext circle_context;
  const Mat3 target_rotation = target_pose_06.linear().cast<double>();
  const Vec3 target_translation = target_pose_06.translation().cast<double>();
  const Vec3 target_r6 = target_rotation.col(2) * params.r6;
  circle_context.o4_base = target_translation + target_r6;
  circle_context.o4_cos_axis = target_rotation.col(0) * params.r5;
  circle_context.o4_sin_axis = target_rotation.col(1) * params.r5;
  circle_context.o5_point = target_translation + target_r6;

  // Step 3 inequality (Eq. 15): triangle feasibility window for d04.
  const double triangle_side_a2 = std::abs(params.a2);
  const double triangle_side_r4 = std::abs(params.r4);
  const double reach_margin =
      1e-6 * std::max(1.0, triangle_side_a2 + triangle_side_r4);
  const double max_o0_o4_dist =
      triangle_side_a2 + triangle_side_r4 + reach_margin;
  const double min_o0_o4_dist =
      std::abs(triangle_side_a2 - triangle_side_r4) - reach_margin;
  circle_context.reach_sq_clamp_tolerance = tolerances.eps_reach_sq;
  circle_context.max_o0_o4_dist_sq = max_o0_o4_dist * max_o0_o4_dist;
  circle_context.has_min_o0_o4_bound = min_o0_o4_dist > 0.0;
  circle_context.min_o0_o4_dist_sq = circle_context.has_min_o0_o4_bound
                                         ? min_o0_o4_dist * min_o0_o4_dist
                                         : 0.0;

  std::vector<Vec6> primal_solutions;
  primal_solutions.reserve(kMaxIkSolutions);

  const auto evaluate_up_dot = [&](double q_eval) -> double {
    CircleDotEvaluation evaluation;
    return EvaluateCircleDots(q_eval, circle_context, params, tolerances,
                              evaluation)
               ? evaluation.up_dot_product
               : std::numeric_limits<double>::quiet_NaN();
  };
  const auto evaluate_down_dot = [&](double q_eval) -> double {
    CircleDotEvaluation evaluation;
    return EvaluateCircleDots(q_eval, circle_context, params, tolerances,
                              evaluation)
               ? evaluation.down_dot_product
               : std::numeric_limits<double>::quiet_NaN();
  };

  Vec6 candidate_joints_rad = Vec6::Zero();
  const auto try_store_solution = [&](const Vec3 &o3_candidate,
                                      const CircleEvaluation &evaluation) {
    if (primal_solutions.size() >= kMaxIkSolutions)
      return;
    if (DetermineJointValues(o3_candidate, evaluation.o4_point,
                             circle_context.o5_point, target_pose_06, params,
                             tolerances, approx_joints_rad,
                             candidate_joints_rad)) {
      NormalizeVecKeepSignedPi(candidate_joints_rad);
      primal_solutions.push_back(candidate_joints_rad);
    }
  };

  const double sample_step_q = kTwoPi / static_cast<double>(kIkQSamples);
  double sin_step = 0.0;
  double cos_step = 1.0;
  SinCos(sample_step_q, sin_step, cos_step);

  CircleDotEvaluation previous_eval;
  CircleDotEvaluation current_eval;
  CircleEvaluation root_eval;

  double previous_q = 0.0;
  double cos_q = 1.0;
  double sin_q = 0.0;

  bool previous_ok =
      EvaluateCircleDots_cs(cos_q, sin_q, circle_context, params, tolerances,
                            previous_eval);

  for (int sample_idx = 1; sample_idx <= kIkQSamples; ++sample_idx) {
    const double current_q = sample_step_q * static_cast<double>(sample_idx);

    // Trig recurrence avoids repeated sin/cos calls in the hot loop.
    const double next_cos_q = cos_q * cos_step - sin_q * sin_step;
    const double next_sin_q = sin_q * cos_step + cos_q * sin_step;
    cos_q = next_cos_q;
    sin_q = next_sin_q;

    const bool current_ok =
        EvaluateCircleDots_cs(cos_q, sin_q, circle_context, params, tolerances,
                              current_eval);

    if (previous_ok && current_ok) {
      if (HasBracket(previous_eval.up_dot_product,
                     current_eval.up_dot_product)) {
        double root_q = 0.0;
        if (RefineZeroIllinois(
                previous_q, current_q, previous_eval.up_dot_product,
                current_eval.up_dot_product, evaluate_up_dot, root_q) &&
            EvaluateCircleFull(root_q, circle_context, params, tolerances,
                               root_eval)) {
          try_store_solution(root_eval.o3_up_point, root_eval);
        }
      }
      if (HasBracket(previous_eval.down_dot_product,
                     current_eval.down_dot_product)) {
        double root_q = 0.0;
        if (RefineZeroIllinois(
                previous_q, current_q, previous_eval.down_dot_product,
                current_eval.down_dot_product, evaluate_down_dot, root_q) &&
            EvaluateCircleFull(root_q, circle_context, params, tolerances,
                               root_eval)) {
          try_store_solution(root_eval.o3_down_point, root_eval);
        }
      }
    }
    previous_q = current_q;
    previous_eval = current_eval;
    previous_ok = current_ok;

    if (primal_solutions.size() >= kMaxIkSolutions)
      break;
  }

  // Keep SolveCrxIk focused on geometric roots. Dual/signed-pi expansion and
  // duplicate suppression happen after full FK/limit validation in SolveIK.
  solutions_out = primal_solutions;
}

static auto FinalizeIkCandidates(const std::vector<Vec6> &geometric_solutions,
                                 const CrxModelData &model,
                                 const PoseIsoRT &target_pose,
                                 const Eigen::Quaterniond &target_quaternion,
                                 const Vec6 *approx_joints_rad, int max_solutions,
                                 std::vector<Vec6> &solutions_out) -> int {
  solutions_out.clear();
  if (max_solutions <= 0 || geometric_solutions.empty())
    return 0;

  const Vec6 &lower_limits_rad = model.lower_limits_rad;
  const Vec6 &upper_limits_rad = model.upper_limits_rad;

  const bool has_approximate_joints = (approx_joints_rad != nullptr);
  Vec6 approx_joints_rad_local = Vec6::Zero();
  if (has_approximate_joints)
    approx_joints_rad_local = *approx_joints_rad;

  std::vector<Candidate> ranked_candidates;
  ranked_candidates.reserve(kMaxIkSolutions);

  auto is_direct_duplicate = [&](const Vec6 &candidate_joints_rad) {
    return std::any_of(ranked_candidates.begin(), ranked_candidates.end(),
                       [&](const Candidate &candidate) {
                         return MaxAbsDiffRadDirect(
                                    candidate_joints_rad,
                                    candidate.user_joints_rad) <=
                                kSolutionAngleToleranceRad;
                       });
  };

  auto replace_worst_candidate_if_better = [&](const Candidate &incoming) {
    auto worst_candidate_it =
        std::max_element(ranked_candidates.begin(), ranked_candidates.end(),
                         [](const Candidate &a, const Candidate &b) {
                           return a.distance_sq < b.distance_sq;
                         });
    if (worst_candidate_it != ranked_candidates.end() &&
        incoming.distance_sq < worst_candidate_it->distance_sq) {
      *worst_candidate_it = incoming;
    }
  };

  for (Vec6 solution_rad : geometric_solutions) {
    NormalizeUserSolutionDomains(solution_rad);

    const bool processed_all_variants =
        ForEachCandidateVariant(solution_rad, [&](Vec6 user_variant_rad) {
          if (!ClampToLimits(user_variant_rad, lower_limits_rad,
                             upper_limits_rad, kJointLimitToleranceRad))
            return true;

          if (!IsFkRoundtripValid(user_variant_rad, target_pose,
                                  target_quaternion, model))
            return true;

          if (is_direct_duplicate(user_variant_rad))
            return true;

          const double distance_sq =
              has_approximate_joints
                  ? WrappedDist2Rad(user_variant_rad, approx_joints_rad_local)
                  : static_cast<double>(ranked_candidates.size());

          if (ranked_candidates.size() < kMaxIkSolutions) {
            ranked_candidates.push_back(
                Candidate{user_variant_rad, distance_sq});
          } else if (has_approximate_joints) {
            replace_worst_candidate_if_better(
                Candidate{user_variant_rad, distance_sq});
          }
          return true;
        });

    if (!processed_all_variants)
      break;
  }

  if (has_approximate_joints) {
    Vec6 approx_candidate_rad = approx_joints_rad_local;
    if (ClampToLimits(approx_candidate_rad, lower_limits_rad, upper_limits_rad,
                      kJointLimitToleranceRad) &&
        IsFkRoundtripValid(approx_candidate_rad, target_pose, target_quaternion,
                           model) &&
        !is_direct_duplicate(approx_candidate_rad)) {

      if (ranked_candidates.size() < kMaxIkSolutions) {
        ranked_candidates.push_back(Candidate{approx_candidate_rad, 0.0});
      } else {
        replace_worst_candidate_if_better(Candidate{approx_candidate_rad, 0.0});
      }
    }
  }

  if (ranked_candidates.empty())
    return 0;

  std::sort(ranked_candidates.begin(), ranked_candidates.end(),
            [](const Candidate &a, const Candidate &b) {
              return a.distance_sq < b.distance_sq;
            });

  const int solution_count =
      std::min<int>(max_solutions, static_cast<int>(ranked_candidates.size()));

  solutions_out.reserve(solution_count);
  for (int solution_idx = 0; solution_idx < solution_count; ++solution_idx)
    solutions_out.push_back(ranked_candidates[solution_idx].user_joints_rad);

  return solution_count;
}
} // namespace

auto SolveFkIsometry(const CrxModelData &model, const Vec6 &user_joints_rad,
                     PoseIsoRT &pose_out, std::vector<PoseIsoRT> *joint_poses,
                     bool check_limits) -> int {
  return SolveFKCore(user_joints_rad, pose_out, joint_poses, check_limits, model);
}

auto SolveIkIsometry(const CrxModelData &model, const PoseIsoRT &target_pose,
                     const Vec6 *approx_joints_rad, int max_solutions,
                     std::vector<Vec6> &solutions_out) -> int {
  solutions_out.clear();
  if (max_solutions <= 0)
    return 0;

  CrxParams crx_params;
  if (!ReadCrxParams(model, crx_params))
    return -1;

  const Eigen::Quaterniond target_quaternion(
      target_pose.linear().cast<double>());

  const PoseIsoRT target_pose_06_raw =
      model.base_transform.inverse() * target_pose * model.tool_transform.inverse();

  double convention_base_z_shift_mm = 0.0;
  if (!ConvertRoboDkDhToAnalyticIkConvention(crx_params,
                                             convention_base_z_shift_mm))
    return -1;

  PoseIsoRT target_pose_06 = target_pose_06_raw;
  if (std::abs(convention_base_z_shift_mm) > kEpsilon) {
    PoseIsoRT convention_shift_transform = PoseIsoRT::Identity();
    convention_shift_transform.translation() << 0.0, 0.0,
        -convention_base_z_shift_mm;
    target_pose_06 = convention_shift_transform * target_pose_06_raw;
  }

  Vec6 approx_joints_rad_local = Vec6::Zero();
  const bool has_approximate_joints = (approx_joints_rad != nullptr);
  if (has_approximate_joints) {
    approx_joints_rad_local = *approx_joints_rad;
    NormalizeVecKeepSignedPi(approx_joints_rad_local);
  }

  std::vector<Vec6> geometric_solutions;

  const Vec6 *approx_for_finalize =
      has_approximate_joints ? &approx_joints_rad_local : nullptr;

  SolveCrxIk(target_pose_06, crx_params, nullptr, geometric_solutions);

  if (geometric_solutions.empty()) {
    if (has_approximate_joints) {
      Vec6 approx_fallback_joints_rad = approx_joints_rad_local;
      if (ClampToLimits(approx_fallback_joints_rad, model.lower_limits_rad,
                        model.upper_limits_rad, kJointLimitToleranceRad) &&
          IsFkRoundtripValid(approx_fallback_joints_rad, target_pose,
                             target_quaternion, model)) {
        solutions_out.push_back(approx_fallback_joints_rad);
        return 1;
      }
    }
    return 0;
  }

  return FinalizeIkCandidates(geometric_solutions, model, target_pose,
                              target_quaternion, approx_for_finalize,
                              max_solutions, solutions_out);
}

} // namespace crx






