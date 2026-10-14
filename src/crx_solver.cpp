#include "crx_solver.h"

#include <algorithm>
#include <array>
#include <cassert>
#include <cmath>
#include <cstddef>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "crx_command_lifts.h"
#include "crx_discovery.h"
#include "crx_joint_recovery.h"
#include "crx_math_helpers.h"
#include "crx_numerics.h"
#include "crx_types.h"

namespace crx {
namespace {

auto IsModelDataValid(const CrxModelData &model) -> bool {
  if (!model.base_transform.matrix().allFinite() ||
      !model.tool_transform.matrix().allFinite() ||
      !model.lower_limits_rad.allFinite() ||
      !model.upper_limits_rad.allFinite() ||
      (model.lower_limits_rad.array() > model.upper_limits_rad.array()).any()) {
    return false;
  }

  for (std::size_t joint_index = 0; joint_index < model.dh_rows.size();
       ++joint_index) {
    const DhRow &row = model.dh_rows[joint_index];
    if (!std::isfinite(row.alpha_rad) || !std::isfinite(row.a) ||
        !std::isfinite(row.theta0_rad) || !std::isfinite(row.d) ||
        !std::isfinite(model.joint_senses[joint_index]) ||
        !numerics::Scalar(std::abs(model.joint_senses[joint_index]))
             .isApproxToConstant(1.0, numerics::kUnitTolerance)) {
      return false;
    }
  }
  return true;
}

struct PreparedFk {
  struct Link {
    Scalar sine = 0, cosine = 1, a = 0, d = 0, theta = 0;
    bool is_prismatic = false;

    auto Pose(double motion) const -> PoseIsoRT {
      Scalar current_theta = theta, current_d = d;
      if (is_prismatic) {
        current_d += static_cast<Scalar>(motion);
      } else {
        current_theta += static_cast<Scalar>(motion);
      }
      const Scalar st = std::sin(current_theta);
      const Scalar ct = std::cos(current_theta);
      PoseIsoRT transform;
      transform.matrix() << ct, -st, 0.0, a, // Modified DH.
          cosine * st, cosine * ct, -sine, -current_d * sine, sine * st,
          sine * ct, cosine, current_d * cosine, 0.0, 0.0, 0.0, 1.0;
      return transform;
    }
  };

  const CrxModelData &model;
  std::array<Link, kDofCount> links;

  explicit PreparedFk(const CrxModelData &input) : model(input) {
    for (std::size_t i = 0; i < links.size(); ++i) {
      const auto &row = model.dh_rows[i];
      const Scalar alpha = SnapToRightAngleFamily(row.alpha_rad);
      links[i] = {std::sin(alpha),
                  std::cos(alpha),
                  row.a,
                  row.d,
                  SnapToRightAngleFamily(row.theta0_rad),
                  row.is_prismatic};
    }
  }
};

auto SolveFKCore(const Vec6 &user_joints_rad, PoseIsoRT &pose_out,
                 JointPoseBuffer *joint_poses_out, bool check_limits,
                 const PreparedFk &fk) -> int {
  const auto &model = fk.model;
  if (check_limits) {
    Vec6 command = ApplyFanucJointCoupling(user_joints_rad);
    if (!ClampCommandToLimits(command, model.lower_limits_rad,
                              model.upper_limits_rad)) {
      return -2;
    }
  }
  PoseIsoRT accumulated_pose = model.base_transform;
  if (joint_poses_out != nullptr) {
    (*joint_poses_out)[0] = accumulated_pose;
  }

  const double sensed_joint2_rad =
      user_joints_rad[kJoint2Index] * model.joint_senses[kJoint2Index];

  for (int joint_id = 0; joint_id < kDofCount; ++joint_id) {
    const auto eigen_index = static_cast<Eigen::Index>(joint_id);
    const auto storage_index = static_cast<std::size_t>(joint_id);
    const double sensed_joint_rad =
        user_joints_rad[eigen_index] * model.joint_senses[storage_index];

    double joint_model_rad = sensed_joint_rad;
    if (joint_id == kJoint3Index) {
      joint_model_rad = -sensed_joint2_rad + sensed_joint_rad;
    }

    accumulated_pose =
        accumulated_pose * fk.links[storage_index].Pose(joint_model_rad);

    if (joint_poses_out != nullptr) {
      (*joint_poses_out)[storage_index + 1U] = accumulated_pose;
    }
  }

  pose_out = accumulated_pose * model.tool_transform;
  return 1;
}

auto IsFkRoundtripValidWithTolerances(const Vec6 &candidate_user_joints_rad,
                                      const PoseIsoRT &target_pose,
                                      const Eigen::Quaterniond &target_quat,
                                      const PreparedFk &fk,
                                      double max_position_error_squared_mm,
                                      double max_orientation_error_rad)
    -> bool {
  PoseIsoRT fk_pose_isometry = PoseIsoRT::Identity();
  if (SolveFKCore(candidate_user_joints_rad, fk_pose_isometry, nullptr, false,
                  fk) != 1) {
    return false;
  }

  const Vec3 position_error = fk_pose_isometry.translation().cast<double>() -
                              target_pose.translation().cast<double>();
  if (!std::isfinite(position_error.squaredNorm()) ||
      position_error.squaredNorm() > max_position_error_squared_mm) {
    return false;
  }

  const Eigen::Quaterniond fk_quaternion(
      fk_pose_isometry.linear().cast<double>());
  const double orientation_error_rad =
      fk_quaternion.angularDistance(target_quat);
  return std::isfinite(orientation_error_rad) &&
         orientation_error_rad <= max_orientation_error_rad;
}

auto IsFkRoundtripValid(const Vec6 &candidate_user_joints_rad,
                        const PoseIsoRT &target_pose,
                        const Eigen::Quaterniond &target_quat,
                        const PreparedFk &fk) -> bool {
  return IsFkRoundtripValidWithTolerances(
      candidate_user_joints_rad, target_pose, target_quat, fk,
      (numerics::kSolutionPositionToleranceMm *
       numerics::kSolutionPositionToleranceMm),
      numerics::kSolutionAngleToleranceRad);
}

struct CrxParams {
  std::array<double, kDofCount> alpha_rad{};
  std::array<double, kDofCount> a{};
  std::array<double, kDofCount> d{};
  std::array<double, kDofCount> theta0_rad{};
  double a2 = 0.0, r4 = 0.0, r5 = 0.0, r6 = 0.0;
};

struct ClassificationTolerances {
  double length_scale = 1.0;
  double eps_len = 0.0;
  double eps_len2 = 0.0;
  double eps_xy = 0.0;
};

inline auto BuildClassificationTolerances(const CrxParams &params)
    -> ClassificationTolerances {
  ClassificationTolerances tolerances;
  tolerances.length_scale =
      std::max({std::abs(params.a2), std::abs(params.r4), std::abs(params.r5),
                std::abs(params.r6), 1.0});
  const double machine_epsilon = numerics::kEpsilon;
  tolerances.eps_len =
      std::max(numerics::kGeometryLengthFloorMm,
               8.0 * machine_epsilon * tolerances.length_scale);
  tolerances.eps_len2 = tolerances.eps_len * tolerances.eps_len;
  tolerances.eps_xy =
      std::max(numerics::kGeometryLengthFloorMm,
               numerics::kGeometryTolerance * tolerances.length_scale);
  return tolerances;
}

auto ReadCrxParams(const CrxModelData &model, CrxParams &params) -> bool {
  for (int joint_id = 0; joint_id < kDofCount; ++joint_id) {
    const auto storage_index = static_cast<std::size_t>(joint_id);
    const DhRow &dh_row = model.dh_rows[storage_index];
    if (dh_row.is_prismatic) {
      return false; // prismatic not supported
    }
    params.alpha_rad[storage_index] = SnapToRightAngleFamily(dh_row.alpha_rad);
    params.a[storage_index] = dh_row.a;
    params.theta0_rad[storage_index] =
        SnapToRightAngleFamily(dh_row.theta0_rad);
    params.d[storage_index] = dh_row.d;
  }
  params.a2 = params.a[2];
  params.r4 = params.d[3];
  params.r5 = params.d[4];
  params.r6 = params.d[5];
  return true;
}

auto RecognizeCrxGeometry(const CrxParams &params, double &base_z_shift_mm)
    -> bool {
  // Validate the CRX DH signature before extracting canonical geometry.
  base_z_shift_mm = 0.0;
  const double tol = numerics::kRightAngleSnapToleranceRad;
  // Captured nominal zero offsets carry subtraction roundoff proportional to
  // the robot's length scale. Keep the physical FK acceptance gate unchanged.
  double length_scale = 1.0;
  for (std::size_t i = 0; i < params.a.size(); ++i) {
    length_scale =
        std::max({length_scale, std::abs(params.a[i]), std::abs(params.d[i])});
  }
  const double length_tol =
      std::max(numerics::kDhLengthToleranceMm,
               numerics::kElementaryRoundoff * length_scale);

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

  const bool a_ok = std::abs(params.a[0]) <= length_tol &&
                    std::abs(params.a[1]) <= length_tol && params.a[2] > 0.0 &&
                    std::abs(params.a[3]) <= length_tol &&
                    std::abs(params.a[4]) <= length_tol &&
                    std::abs(params.a[5]) <= length_tol;

  const bool d_ok =
      params.d[0] > 0.0 && std::abs(params.d[1]) <= length_tol &&
      std::abs(params.d[2]) <= length_tol && params.d[3] > 0.0 &&
      params.d[4] <= -numerics::kDhLengthToleranceMm && // Pass-1 Fix #2
      params.d[5] > 0.0;

  // Fail closed when a model differs from the CRX assumptions.
  if (!(alpha_ok && theta_ok && a_ok && d_ok)) {
    return false;
  }

  base_z_shift_mm = params.d[0];
  return true;
}

struct PlaneBasisXY {
  Vec3 x = Vec3::Zero();
  Vec3 y = Vec3::Zero();
};

// Specialized hot-path basis build for ConstructPlane(O4, UnitZ()).
inline auto
ConstructPlane_O4_UnitZ_XY(const Vec3 &O4,
                           const ClassificationTolerances &tolerances,
                           PlaneBasisXY &basis) -> bool {
  const double o4_norm_sq = O4.squaredNorm();
  if (!std::isfinite(o4_norm_sq) || o4_norm_sq <= tolerances.eps_len2) {
    return false;
  }

  const Vec3 plane_x = O4 * (1.0 / std::sqrt(o4_norm_sq));

  Vec3 plane_y = Vec3::UnitZ() - plane_x.z() * plane_x;
  double plane_y_norm_sq = plane_y.squaredNorm();
  if (!std::isfinite(plane_y_norm_sq) ||
      plane_y_norm_sq <= tolerances.eps_len2) {
    // Choose the less aligned coordinate axis. Its projected squared norm is
    // at least 1/2 for a unit plane_x; no empirical .99 alignment cutoff.
    const Vec3 fallback_axis = (std::abs(plane_x.z()) > std::abs(plane_x.y()))
                                   ? Vec3::UnitY()
                                   : Vec3::UnitZ();
    plane_y = fallback_axis - plane_x.dot(fallback_axis) * plane_x;
    plane_y_norm_sq = plane_y.squaredNorm();
    if (!std::isfinite(plane_y_norm_sq) ||
        plane_y_norm_sq <= tolerances.eps_len2) {
      plane_y = plane_x.unitOrthogonal();
      plane_y_norm_sq = plane_y.squaredNorm();
      if (!std::isfinite(plane_y_norm_sq) ||
          plane_y_norm_sq <= tolerances.eps_len2) {
        return false;
      }
    }
  }
  plane_y *= (1.0 / std::sqrt(plane_y_norm_sq));

  basis.x = plane_x;
  basis.y = plane_y;

#ifndef NDEBUG
  const Vec3 plane_z = basis.x.cross(basis.y);
  const double det_like = basis.x.dot(basis.y.cross(plane_z));
  assert(std::abs(det_like - 1.0) < numerics::kPlaneOrthonormalityTolerance &&
         "ConstructPlane_O4_UnitZ_XY: basis is not orthonormal");
#endif
  return true;
}

void DualSolutionRad(const Vec6 &source_solution, Vec6 &dual_solution_out) {
  // Eq. (23) dual map (paper Sec. 2.6, Step 7), expressed in radians:
  // [J1-PI, -J2, PI-J3, J4-PI, J5, J6].
  dual_solution_out = source_solution;
  dual_solution_out[0] = source_solution[0] - angle_conv::kPi;
  dual_solution_out[1] = -source_solution[1];
  dual_solution_out[2] = angle_conv::kPi - source_solution[2];
  dual_solution_out[3] = source_solution[3] - angle_conv::kPi;
  dual_solution_out[4] = source_solution[4];
  dual_solution_out[5] = source_solution[5];
  NormalizeUserSolutionDomains(dual_solution_out);
}

auto NominateBoundaryPosture(const Vec6 &command, const CrxModelData &model,
                             Vec6 &bounded) -> bool {
  bounded = command;
  bool changed = false;
  for (int joint = 0; joint < kDofCount; ++joint) {
    const double lower = model.lower_limits_rad[joint];
    const double upper = model.upper_limits_rad[joint];
    if (upper - lower >= kTwoPi) {
      continue; // Every phase has a lift in an interval spanning a full turn.
    }
    const double middle = lower / 2 + upper / 2;
    const double nearest =
        command[joint] +
        kTwoPi * std::round((middle - command[joint]) / kTwoPi);
    const double clamped = std::clamp(nearest, lower, upper);
    if (!std::isfinite(nearest) ||
        std::abs(clamped - nearest) > numerics::kJointPhaseToleranceRad) {
      return false;
    }
    if (clamped != nearest) {
      bounded[joint] = clamped;
      changed = true;
    }
  }
  return changed;
}

auto FinalizeIkCandidates(const std::vector<Vec6> &geometric_solutions,
                          const PreparedFk &fk, const PoseIsoRT &target_pose,
                          const Eigen::Quaterniond &target_quaternion,
                          const Vec6 *approx_joints_rad, int max_solutions)
    -> IkResult {
  const auto &model = fk.model;
  IkResult result;
  std::vector<Vec6> command_postures;
  command_postures.reserve(2 * geometric_solutions.size());
  for (const Vec6 &solution : geometric_solutions) {
    command_postures.push_back(ApplyFanucJointCoupling(solution));
    Vec6 dual = Vec6::Zero();
    DualSolutionRad(solution, dual);
    command_postures.push_back(ApplyFanucJointCoupling(dual));
  }
  // Root reconstruction can place a boundary posture just beyond a limit,
  // even when other branches remain feasible. Nominate the actual boundary
  // only within posture identity tolerance, then recheck the complete pose.
  // Command enumeration and its limit roundoff allowance remain unchanged.
  for (auto &command : command_postures) {
    Vec6 bounded;
    if (NominateBoundaryPosture(command, model, bounded) &&
        IsFkRoundtripValid(RemoveFanucJointCoupling(bounded), target_pose,
                           target_quaternion, fk)) {
      command = bounded;
    }
  }
  const Vec6 command_seed = approx_joints_rad != nullptr
                                ? ApplyFanucJointCoupling(*approx_joints_rad)
                                : Vec6::Zero();
  const auto selected = SelectCommandLifts(
      command_postures, model.lower_limits_rad, model.upper_limits_rad,
      approx_joints_rad != nullptr ? &command_seed : nullptr, max_solutions,
      [&](const Vec6 &command) {
        return IsFkRoundtripValid(RemoveFanucJointCoupling(command),
                                  target_pose, target_quaternion, fk);
      });
  if (selected.status != CommandSelectionStatus::Complete) {
    switch (selected.status) {
    case CommandSelectionStatus::InvalidInput:
      result.status = IkStatus::InvalidInput;
      break;
    case CommandSelectionStatus::RangeFailure:
      result.status = IkStatus::NumericalRangeFailure;
      break;
    case CommandSelectionStatus::WorkLimit:
      result.status = IkStatus::WorkLimit;
      break;
    case CommandSelectionStatus::Complete:
      break;
    }
    return result;
  }
  result.solutions.reserve(selected.commands.size());
  for (const auto &command : selected.commands) {
    result.solutions.push_back(RemoveFanucJointCoupling(command.command_rad));
  }
  result.truncated = selected.truncated;
  result.status = result.solutions.empty() ? IkStatus::NoCandidate
                                           : IkStatus::StrictCandidates;
  return result;
}

// Seed fallback is permitted only for an empty geometric search, before limits
// and ranking. A filtered nonempty search must not enable approximate success.
void ApplyEmptySearchSeed(const PreparedFk &fk, const PoseIsoRT &target,
                          const Eigen::Quaterniond &rotation, const Vec6 *seed,
                          IkResult &result) {
  const auto &model = fk.model;
  result.status = IkStatus::NoCandidate;
  if (seed == nullptr) {
    return;
  }
  Vec6 command = ApplyFanucJointCoupling(*seed);
  if (!ClampCommandToLimits(command, model.lower_limits_rad,
                            model.upper_limits_rad)) {
    return;
  }
  const Vec6 user = RemoveFanucJointCoupling(command);
  const bool strict = IsFkRoundtripValid(user, target, rotation, fk);
  if (strict || IsFkRoundtripValidWithTolerances(
                    user, target, rotation, fk,
                    numerics::kApproxFallbackPositionToleranceMm *
                        numerics::kApproxFallbackPositionToleranceMm,
                    numerics::kApproxFallbackAngleToleranceRad)) {
    result.solutions.push_back(user);
    result.status = strict ? IkStatus::StrictSeed : IkStatus::ApproximateSeed;
  }
}

auto CorrectConstrainedCandidates(const std::vector<Vec6> &geometric_solutions,
                                  const CrxModelData &model,
                                  const canonical::Lengths &lengths,
                                  const PoseIsoRT &target_pose_06,
                                  const canonical::PoseTolerance &tolerance)
    -> std::vector<Vec6> {
  // Rounded Cartesian targets at a limit can have their exact geometric
  // posture just outside the command box (MIN_Z). Nominate the nearest box
  // point only within one bounded joint update, then solve the original pose
  // with active limits. This is strict FK correction, never limit slack or
  // the approximate empty-search seed policy.
  std::vector<Vec6> constrained;
  for (const Vec6 &user : geometric_solutions) {
    canonical::JointCandidate candidate;
    candidate.joints = ApplyFanucJointCoupling(user);
    const Vec6 middle = (model.lower_limits_rad + model.upper_limits_rad) / 2;
    candidate.joints +=
        kTwoPi *
        ((middle - candidate.joints) / kTwoPi).array().round().matrix();
    const Vec6 clamped = candidate.joints.cwiseMax(model.lower_limits_rad)
                             .cwiseMin(model.upper_limits_rad);
    if ((clamped - candidate.joints).cwiseAbs().maxCoeff() >
        numerics::kPolishMaxStepRad) {
      continue;
    }
    candidate.joints = clamped;
    if (canonical::PolishJointCandidate(
            lengths, target_pose_06, tolerance, numerics::kPolishIterations,
            candidate, &model.lower_limits_rad, &model.upper_limits_rad)) {
      constrained.push_back(RemoveFanucJointCoupling(candidate.joints));
    }
  }
  return constrained;
}

struct ArmClassificationGeometry {
  Vec3 o3_point = Vec3::Zero();
  Vec3 o4_point = Vec3::Zero();
  ClassificationTolerances tolerances{};
};

auto BuildArmClassificationGeometry(const CrxModelData &model,
                                    const Vec6 &user_joints_rad,
                                    ArmClassificationGeometry &geometry_out)
    -> bool {
  constexpr std::size_t kJoint1FrameIndex = 1;
  constexpr std::size_t kJoint3FrameIndex = 3;
  constexpr std::size_t kJoint4FrameIndex = 4;

  CrxParams crx_params;
  if (!ReadCrxParams(model, crx_params)) {
    return false;
  }

  double convention_base_z_shift_mm = 0.0;
  if (!RecognizeCrxGeometry(crx_params, convention_base_z_shift_mm)) {
    return false;
  }

  PoseIsoRT unused_pose = PoseIsoRT::Identity();
  JointPoseBuffer joint_poses;
  if (SolveFKCore(user_joints_rad, unused_pose, &joint_poses, false,
                  PreparedFk(model)) != 1) {
    return false;
  }

  const PoseIsoRT frame_1_inverse = joint_poses[kJoint1FrameIndex].inverse();
  geometry_out.o3_point = (frame_1_inverse * joint_poses[kJoint3FrameIndex])
                              .translation()
                              .cast<double>();
  geometry_out.o4_point = (frame_1_inverse * joint_poses[kJoint4FrameIndex])
                              .translation()
                              .cast<double>();
  geometry_out.tolerances = BuildClassificationTolerances(crx_params);
  return geometry_out.o3_point.allFinite() && geometry_out.o4_point.allFinite();
}
} // namespace

auto SolveFkIsometry(const CrxModelData &model, const Vec6 &user_joints_rad,
                     PoseIsoRT &pose_out, JointPoseBuffer *joint_poses,
                     bool check_limits) -> int {
  if (!IsModelDataValid(model) || !user_joints_rad.allFinite()) {
    return -1;
  }
  return SolveFKCore(user_joints_rad, pose_out, joint_poses, check_limits,
                     PreparedFk(model));
}

auto ClassifyArmPosture(const CrxModelData &model, const Vec6 &user_joints_rad,
                        ArmPosture &posture_out) -> bool {
  if (!IsModelDataValid(model) || !user_joints_rad.allFinite()) {
    return false;
  }

  ArmClassificationGeometry geometry;
  if (!BuildArmClassificationGeometry(model, user_joints_rad, geometry)) {
    return false;
  }

  const double shoulder_radius =
      std::hypot(geometry.o4_point.x(), geometry.o4_point.y());
  const double singular_radius = std::max(
      geometry.tolerances.eps_xy, numerics::kConfigShoulderRadiusFraction *
                                      geometry.tolerances.length_scale);
  if (!std::isfinite(shoulder_radius) || shoulder_radius <= singular_radius) {
    return false;
  }

  PlaneBasisXY plane_basis;
  if (!ConstructPlane_O4_UnitZ_XY(geometry.o4_point, geometry.tolerances,
                                  plane_basis)) {
    return false;
  }

  const double elbow_side = geometry.o3_point.dot(plane_basis.y);
  if (!std::isfinite(elbow_side) ||
      std::abs(elbow_side) <= geometry.tolerances.eps_xy) {
    return false;
  }

  const bool rear = geometry.o4_point.x() < 0.0;
  const bool geometric_lower_arm = elbow_side < -geometry.tolerances.eps_xy;
  const bool lower_arm = geometric_lower_arm != rear;
  const bool flip = user_joints_rad[4] > numerics::kConfigWristZeroRad;

  posture_out = ArmPosture{rear, lower_arm, flip};
  return true;
}

auto SolveIkIsometry(const CrxModelData &model, const PoseIsoRT &target_pose,
                     const Vec6 *approx_joints_rad, int max_solutions)
    -> IkResult {
  IkResult result;
  if (max_solutions <= 0) {
    result.status = IkStatus::NoOutputRequested;
    return result;
  }
  if (!IsModelDataValid(model) || !target_pose.matrix().allFinite() ||
      (approx_joints_rad != nullptr && !approx_joints_rad->allFinite())) {
    return result;
  }

  CrxParams crx_params;
  if (!ReadCrxParams(model, crx_params)) {
    result.status = IkStatus::UnsupportedModel;
    return result;
  }

  const Eigen::Quaterniond target_quaternion(
      target_pose.linear().cast<double>());

  const PoseIsoRT target_pose_06_raw = model.base_transform.inverse() *
                                       target_pose *
                                       model.tool_transform.inverse();

  double convention_base_z_shift_mm = 0.0;
  if (!RecognizeCrxGeometry(crx_params, convention_base_z_shift_mm)) {
    result.status = IkStatus::UnsupportedModel;
    return result;
  }

  PoseIsoRT target_pose_06 = target_pose_06_raw;
  if (std::abs(convention_base_z_shift_mm) > numerics::kDhLengthToleranceMm) {
    PoseIsoRT convention_shift_transform = PoseIsoRT::Identity();
    convention_shift_transform.translation() << 0.0, 0.0,
        -convention_base_z_shift_mm;
    target_pose_06 = convention_shift_transform * target_pose_06_raw;
  }

  std::vector<Vec6> geometric_solutions;

  // The six supported CRX assets use these joint senses.
  // Keep model recognition/equivalence evidence in the Python asset oracle.
  // The existing DH guard and base/tool conversion above remain authoritative.
  constexpr std::array<double, kDofCount> kCrxSenses{1, 1, -1, -1, -1, -1};
  if (!Eigen::Map<const Vec6>(model.joint_senses.data())
           .isApprox(Eigen::Map<const Vec6>(kCrxSenses.data()),
                     numerics::kUnitTolerance)) {
    result.status = IkStatus::UnsupportedModel;
    return result;
  }
  // Canonical lengths use positive arm offsets and the signed wrist offset.
  const canonical::Lengths lengths{crx_params.a2, crx_params.r4, crx_params.r6,
                                   -crx_params.r5};
  const canonical::PoseTolerance tolerance{
      numerics::kSolutionPositionToleranceMm,
      numerics::kSolutionAngleToleranceRad};
  const auto discovered =
      canonical::DiscoverJointCandidates(lengths, target_pose_06, tolerance);
  if (discovered.status != canonical::RecoveryStatus::Candidates &&
      discovered.status != canonical::RecoveryStatus::NoCandidate) {
    // Unresolved discovery must not be accepted through the seed policy.
    result.status =
        discovered.status == canonical::RecoveryStatus::InvalidInput
            ? IkStatus::InvalidInput
        : discovered.status == canonical::RecoveryStatus::NumericalRangeFailure
            ? IkStatus::NumericalRangeFailure
            : IkStatus::Unresolved;
    return result;
  }
  result.used_singular_representatives =
      discovered.used_singular_representatives;
  // Every command still receives a complete FK check; only model constants
  // are shared across those checks during this solve.
  const PreparedFk fk(model);
  if (discovered.status == canonical::RecoveryStatus::Candidates) {
    geometric_solutions.reserve(discovered.count);
    for (std::size_t i = 0; i < discovered.count; ++i) {
      // Discovery returns coupled canonical J3; the existing finalizer
      // consumes decoupled user J3, then selects lifts in command coordinates.
      Vec6 user = RemoveFanucJointCoupling(discovered.candidates[i].joints);
      // Subtracting two canonical representatives can add a full J3 turn.
      // Use principal geometric representatives before selecting command lifts.
      NormalizeVecKeepSignedPi(user);
      if (!IsFkRoundtripValid(user, target_pose, target_quaternion, fk)) {
        result.status = IkStatus::Unresolved;
        return result;
      }
      geometric_solutions.push_back(user);
    }
  }

  if (geometric_solutions.empty()) {
    ApplyEmptySearchSeed(fk, target_pose, target_quaternion, approx_joints_rad,
                         result);
    return result;
  }

  auto finalized =
      FinalizeIkCandidates(geometric_solutions, fk, target_pose,
                           target_quaternion, approx_joints_rad, max_solutions);
  if (finalized.status == IkStatus::NoCandidate) {
    const auto constrained = CorrectConstrainedCandidates(
        geometric_solutions, model, lengths, target_pose_06, tolerance);
    if (!constrained.empty()) {
      finalized =
          FinalizeIkCandidates(constrained, fk, target_pose, target_quaternion,
                               approx_joints_rad, max_solutions);
    }
  }
  finalized.used_singular_representatives =
      result.used_singular_representatives;
  return finalized;
}

} // namespace crx
