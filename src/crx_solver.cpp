// SPDX-License-Identifier: Apache-2.0
// SPDX-FileCopyrightText: 2026 Centre Technologique en Aérospatiale
/**
 * @file crx_solver.cpp
 * @brief Implementation of CRX inverse kinematics solver.
 */

#include "crx_solver.h"

#include <algorithm>
#include <array>
#include <cassert>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "crx_canonical.h"
#include "crx_command_lifts.h"
#include "crx_coordinates.h"
#include "crx_discovery.h"
#include "crx_joint_recovery.h"
#include "crx_joint_trig.h"
#include "crx_math_helpers.h"
#include "crx_numerics.h"
#include "crx_polynomial_roots.h"
#include "crx_types.h"

namespace crx {
namespace {

enum class ModelStatus : std::uint8_t { Ready, InvalidInput, UnsupportedModel };

// Raw RoboDK DH data ends here. All solver paths use this CRX-only geometry
// and coupled command radians, including FK and CAD link poses.
struct PreparedCrxModel {
  PoseIsoRT base_transform = PoseIsoRT::Identity();
  PoseIsoRT tool_transform = PoseIsoRT::Identity();
  canonical::Lengths lengths{};
  CanonicalCommandMap coordinates{};
  double base_height = 0.0;
  Vec6 lower_limits_rad = Vec6::Zero();
  Vec6 upper_limits_rad = Vec6::Zero();
};

auto PrepareCrxModel(const CrxModelData &model, PreparedCrxModel &output)
    -> ModelStatus {
  if (!model.base_transform.matrix().allFinite() ||
      !model.tool_transform.matrix().allFinite() ||
      !model.lower_limits_rad.allFinite() ||
      !model.upper_limits_rad.allFinite() ||
      (model.lower_limits_rad.array() > model.upper_limits_rad.array()).any()) {
    return ModelStatus::InvalidInput;
  }

  double length_scale = 1.0;
  for (std::size_t joint_index = 0; joint_index < model.dh_rows.size();
       ++joint_index) {
    const DhRow &row = model.dh_rows.at(joint_index);
    length_scale = std::max({length_scale, std::abs(row.a), std::abs(row.d)});
    if (!std::isfinite(row.alpha_rad) || !std::isfinite(row.a) ||
        !std::isfinite(row.theta0_rad) || !std::isfinite(row.d) ||
        !std::isfinite(model.joint_senses.at(joint_index)) ||
        !numerics::Scalar(std::abs(model.joint_senses.at(joint_index)))
             .isApproxToConstant(1.0, numerics::kUnitTolerance)) {
      return ModelStatus::InvalidInput;
    }
  }
  constexpr std::array<double, kDofCount> kAlpha{0,        -kHalfPi, 0,
                                                 -kHalfPi, kHalfPi,  -kHalfPi};
  constexpr std::array<double, kDofCount> kTheta{0, -kHalfPi, 0, 0, 0, 0};
  constexpr std::array<double, kDofCount> kSenses{1, 1, -1, -1, -1, -1};
  const double length_tol =
      std::max(numerics::kDhLengthToleranceMm,
               numerics::kElementaryRoundoff * length_scale);
  for (std::size_t i = 0; i < model.dh_rows.size(); ++i) {
    const auto &row = model.dh_rows.at(i);
    if (row.is_prismatic ||
        AngleDiffAbs(row.alpha_rad, kAlpha.at(i)) >
            numerics::kRightAngleSnapToleranceRad ||
        (i != 2 && std::abs(row.a) > length_tol) ||
        ((i == 1 || i == 2) && std::abs(row.d) > length_tol)) {
      return ModelStatus::UnsupportedModel;
    }
    // theta_i = theta0_i + sigma_i*u_i = theta_ref_i + sense_ref_i*q_i.
    // A sense encodes a sign; remove the admitted input roundoff.
    const auto axis = static_cast<Eigen::Index>(i);
    output.coordinates.diagonal[axis] =
        std::copysign(1.0, model.joint_senses.at(i)) * kSenses.at(i);
    const double direct_difference = row.theta0_rad - kTheta.at(i);
    if (std::abs(direct_difference) <= numerics::kRightAngleSnapToleranceRad) {
      output.coordinates.offset[axis] = 0.0;
    } else {
      // Reduce the stored rotation before subtracting the reference offset.
      // A remainder by binary64 2*pi accumulates its period error for large
      // encoder zeros; subtracting pi/2 first can also lose that entire shift.
      const double principal =
          std::atan2(std::sin(row.theta0_rad), std::cos(row.theta0_rad));
      const double difference =
          std::remainder(principal - kTheta.at(i), kTwoPi);
      output.coordinates.offset[axis] =
          std::abs(difference) <= numerics::kRightAngleSnapToleranceRad
              ? 0.0
              : difference * kSenses.at(i);
    }
  }
  const auto &rows = model.dh_rows;
  if (rows[2].a <= 0 || rows[0].d <= 0 || rows[3].d == 0 ||
      rows[4].d > -numerics::kDhLengthToleranceMm || rows[5].d <= 0) {
    return ModelStatus::UnsupportedModel;
  }
  output.base_transform = model.base_transform;
  output.tool_transform = model.tool_transform;
  output.lengths = {rows[2].a, rows[3].d, rows[5].d, -rows[4].d};
  output.base_height = rows[0].d;
  output.lower_limits_rad = model.lower_limits_rad;
  output.upper_limits_rad = model.upper_limits_rad;
  return ModelStatus::Ready;
}

struct PreparedIkProblem {
  PreparedCrxModel model;
  PoseIsoRT target_pose_06 = PoseIsoRT::Identity();
  Eigen::Quaterniond target_quaternion = Eigen::Quaterniond::Identity();
  canonical::PoseTolerance tolerance{numerics::kSolutionPositionToleranceMm,
                                     numerics::kSolutionAngleToleranceRad};
};

auto ModelStatusToIkStatus(ModelStatus status) -> IkStatus {
  return status == ModelStatus::UnsupportedModel ? IkStatus::UnsupportedModel
                                                 : IkStatus::InvalidInput;
}

auto PrepareIkProblem(const CrxModelData &model, const PoseIsoRT &target_pose,
                      PreparedIkProblem &output) -> ModelStatus {
  if (!target_pose.matrix().allFinite()) {
    return ModelStatus::InvalidInput;
  }
  const auto preparation = PrepareCrxModel(model, output.model);
  if (preparation != ModelStatus::Ready) {
    return preparation;
  }
  output.target_quaternion = Eigen::Quaterniond(target_pose.linear());
  output.target_pose_06 = output.model.base_transform.inverse() * target_pose *
                          output.model.tool_transform.inverse();
  output.target_pose_06.translation().z() -= output.model.base_height;
  return ModelStatus::Ready;
}

auto RecoveryStatusToIkStatus(canonical::RecoveryStatus status) -> IkStatus {
  switch (status) {
  case canonical::RecoveryStatus::Candidates:
    return IkStatus::StrictCandidates;
  case canonical::RecoveryStatus::NoCandidate:
    return IkStatus::NoCandidate;
  case canonical::RecoveryStatus::InvalidInput:
    return IkStatus::InvalidInput;
  case canonical::RecoveryStatus::NumericalRangeFailure:
    return IkStatus::NumericalRangeFailure;
  case canonical::RecoveryStatus::NeedsRefinement:
    return IkStatus::Unresolved;
  }
  return IkStatus::Unresolved;
}

auto CommandSelectionStatusToIkStatus(CommandSelectionStatus status)
    -> IkStatus {
  switch (status) {
  case CommandSelectionStatus::Complete:
    return IkStatus::StrictCandidates;
  case CommandSelectionStatus::InvalidInput:
    return IkStatus::InvalidInput;
  case CommandSelectionStatus::RangeFailure:
    return IkStatus::NumericalRangeFailure;
  case CommandSelectionStatus::WorkLimit:
    return IkStatus::WorkLimit;
  }
  return IkStatus::InvalidInput;
}

auto SolveFKCore(const Vec6 &command_rad, PoseIsoRT &pose_out,
                 JointPoseBuffer *joint_poses_out, bool check_limits,
                 const PreparedCrxModel &model) -> int {
  if (check_limits) {
    Vec6 command = command_rad;
    if (!ClampCommandToLimits(command, model.lower_limits_rad,
                              model.upper_limits_rad)) {
      return -2;
    }
  }
  PoseIsoRT accumulated_pose = model.base_transform;
  if (joint_poses_out != nullptr) {
    (*joint_poses_out)[0] = accumulated_pose;
  }

  static_assert(JointTrig::kCount == kDofCount);
  const Vec6 q = model.coordinates.FromCommand(command_rad);
  const JointTrig::Values angles{q[0],  q[1] - kHalfPi, -q[2],
                                 -q[3], -q[4],          -q[5]};
  constexpr std::array<double, kDofCount> kSine{0, -1, 0, -1, 1, -1};
  constexpr std::array<double, kDofCount> kCosine{1, 0, 1, 0, 0, 0};
  const std::array<double, kDofCount> a{0, 0, model.lengths.a, 0, 0, 0};
  const std::array<double, kDofCount> d{
      model.base_height, 0, 0, model.lengths.b, -model.lengths.r,
      model.lengths.c};
  const JointTrig trig(angles);
  for (int joint_id = 0; joint_id < kDofCount; ++joint_id) {
    const auto storage_index = static_cast<std::size_t>(joint_id);
    const double st = trig.sine.at(storage_index);
    const double ct = trig.cosine.at(storage_index);
    const double sa = kSine.at(storage_index);
    const double ca = kCosine.at(storage_index);
    PoseIsoRT link;
    link.matrix() << ct, -st, 0, a.at(storage_index), ca * st, ca * ct, -sa,
        -d.at(storage_index) * sa, sa * st, sa * ct, ca,
        d.at(storage_index) * ca, 0, 0, 0, 1;
    accumulated_pose = accumulated_pose * link;
    if (joint_poses_out != nullptr) {
      (*joint_poses_out)[storage_index + 1U] = accumulated_pose;
    }
  }

  pose_out = accumulated_pose * model.tool_transform;
  return 1;
}

auto IsFkRoundtripValidWithTolerances(const Vec6 &candidate_command_rad,
                                      const PoseIsoRT &target_pose,
                                      const Eigen::Quaterniond &target_quat,
                                      const PreparedCrxModel &fk,
                                      double max_position_error_squared_mm,
                                      double max_orientation_error_rad)
    -> bool {
  PoseIsoRT fk_pose_isometry = PoseIsoRT::Identity();
  if (SolveFKCore(candidate_command_rad, fk_pose_isometry, nullptr, false,
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

auto IsFkRoundtripValid(const Vec6 &candidate_command_rad,
                        const PoseIsoRT &target_pose,
                        const Eigen::Quaterniond &target_quat,
                        const PreparedCrxModel &fk) -> bool {
  return IsFkRoundtripValidWithTolerances(
      candidate_command_rad, target_pose, target_quat, fk,
      (numerics::kSolutionPositionToleranceMm *
       numerics::kSolutionPositionToleranceMm),
      numerics::kSolutionAngleToleranceRad);
}

struct ClassificationTolerances {
  double length_scale = 1.0;
  double eps_len = 0.0;
  double eps_len2 = 0.0;
  double eps_xy = 0.0;
};

inline auto BuildClassificationTolerances(const canonical::Lengths &lengths)
    -> ClassificationTolerances {
  ClassificationTolerances tolerances;
  tolerances.length_scale =
      std::max({std::abs(lengths.a), std::abs(lengths.b), std::abs(lengths.r),
                std::abs(lengths.c), 1.0});
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

auto NominateBoundaryPosture(const Vec6 &command, const PreparedCrxModel &model,
                             Vec6 &bounded) -> bool {
  bounded = command;
  bool changed = false;
  for (int joint = 0; joint < kDofCount; ++joint) {
    const double lower = model.lower_limits_rad[joint];
    const double upper = model.upper_limits_rad[joint];
    if (upper - lower >= kTwoPi) {
      continue; // Every phase has a lift in an interval spanning a full turn.
    }
    const double middle = (lower / 2) + (upper / 2);
    const double nearest =
        command[joint] +
        (kTwoPi * std::round((middle - command[joint]) / kTwoPi));
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

auto PrepareCommandPostures(const std::vector<Vec6> &geometric_solutions,
                            const PreparedCrxModel &model,
                            const PoseIsoRT &target_pose,
                            const Eigen::Quaterniond &target_quaternion)
    -> std::vector<Vec6> {
  auto command_postures = geometric_solutions;
  for (auto &command : command_postures) {
    Vec6 bounded;
    if (NominateBoundaryPosture(command, model, bounded) &&
        IsFkRoundtripValid(bounded, target_pose, target_quaternion, model)) {
      command = bounded;
    }
  }
  return command_postures;
}

auto FinalizeIkCandidates(const std::vector<Vec6> &geometric_solutions,
                          const PreparedCrxModel &fk,
                          const PoseIsoRT &target_pose,
                          const Eigen::Quaterniond &target_quaternion,
                          const Vec6 *approx_joints_rad, int max_solutions)
    -> IkResult {
  const auto &model = fk;
  IkResult result;
  // Root reconstruction can place a boundary posture just beyond a limit,
  // even when other branches remain feasible. Nominate the actual boundary
  // only within posture identity tolerance, then recheck the complete pose.
  // Command enumeration and its limit roundoff allowance remain unchanged.
  const auto command_postures = PrepareCommandPostures(
      geometric_solutions, model, target_pose, target_quaternion);
  const auto selected = SelectCommandLifts(
      command_postures, model.lower_limits_rad, model.upper_limits_rad,
      approx_joints_rad, max_solutions,
      [&](const Vec6 &command) {
        return IsFkRoundtripValid(command, target_pose, target_quaternion, fk);
      },
      numerics::kCommandLiftBudget, CommandSelectionMode::RankedPrefix);
  if (selected.status != CommandSelectionStatus::Complete) {
    result.status = CommandSelectionStatusToIkStatus(selected.status);
    return result;
  }
  result.solutions.reserve(selected.commands.size());
  for (const auto &command : selected.commands) {
    result.solutions.push_back(command.command_rad);
  }
  result.truncated = selected.truncated;
  result.status = result.solutions.empty() ? IkStatus::NoCandidate
                                           : IkStatus::StrictCandidates;
  return result;
}

// Seed fallback is permitted only for an empty geometric search, before limits
// and ranking. A filtered nonempty search must not enable approximate success.
void ApplyEmptySearchSeed(const PreparedCrxModel &fk, const PoseIsoRT &target,
                          const Eigen::Quaterniond &rotation, const Vec6 *seed,
                          IkResult &result) {
  const auto &model = fk;
  result.status = IkStatus::NoCandidate;
  if (seed == nullptr) {
    return;
  }
  Vec6 command = *seed;
  if (!ClampCommandToLimits(command, model.lower_limits_rad,
                            model.upper_limits_rad)) {
    return;
  }
  const bool strict = IsFkRoundtripValid(command, target, rotation, fk);
  if (strict || IsFkRoundtripValidWithTolerances(
                    command, target, rotation, fk,
                    numerics::kApproxFallbackPositionToleranceMm *
                        numerics::kApproxFallbackPositionToleranceMm,
                    numerics::kApproxFallbackAngleToleranceRad)) {
    result.solutions.push_back(command);
    result.status = strict ? IkStatus::StrictSeed : IkStatus::ApproximateSeed;
  }
}

auto CorrectConstrainedCandidates(const std::vector<Vec6> &geometric_solutions,
                                  const PreparedCrxModel &model,
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
  canonical::WristCircle circle;
  if (canonical::PrepareWristCircle(lengths, target_pose_06, circle) !=
      canonical::PreparationStatus::Ready) {
    return constrained;
  }
  Vec6 canonical_lower, canonical_upper;
  model.coordinates.CanonicalBounds(model.lower_limits_rad,
                                    model.upper_limits_rad, canonical_lower,
                                    canonical_upper);
  for (const Vec6 &command : geometric_solutions) {
    canonical::JointCandidate candidate;
    candidate.joints = command;
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
    candidate.joints = model.coordinates.FromCommand(clamped);
    if (canonical::PolishPreparedJointCandidate(
            circle, target_pose_06, tolerance, numerics::kPolishIterations,
            candidate, &canonical_lower, &canonical_upper)) {
      constrained.push_back(model.coordinates.ToCommand(candidate.joints));
      canonical::JointCandidate partner;
      if (canonical::RecoverBaseFlip(circle, target_pose_06, tolerance,
                                     candidate, partner)) {
        constrained.push_back(model.coordinates.ToCommand(partner.joints));
      }
    }
  }
  return constrained;
}

struct ArmClassificationGeometry {
  Vec3 o3_point = Vec3::Zero();
  Vec3 o4_point = Vec3::Zero();
  ClassificationTolerances tolerances{};
};

auto TryNearbySolution(const PreparedCrxModel &fk,
                       const canonical::Lengths &lengths,
                       const PoseIsoRT &target, const PoseIsoRT &canonical_pose,
                       const canonical::PoseTolerance &tolerance,
                       const Vec6 &seed) -> IkResult {
  IkResult result;
  const auto &model = fk;
  Vec6 command_seed = seed;
  canonical::WristCircle circle;
  // Preserve the canonical target validity gate even when the seed already
  // passes FK, and avoid turn arithmetic outside its representable range.
  if (canonical::PrepareWristCircle(lengths, canonical_pose, circle) !=
          canonical::PreparationStatus::Ready ||
      command_seed.cwiseAbs().maxCoeff() / kTwoPi > numerics::kMaxExactTurn ||
      !ClampCommandToLimits(command_seed, model.lower_limits_rad,
                            model.upper_limits_rad)) {
    return result;
  }
  const Eigen::Quaterniond rotation(target.linear().cast<double>());
  if (IsFkRoundtripValid(command_seed, target, rotation, fk)) {
    result.status = IkStatus::StrictSeed;
    result.solutions.push_back(command_seed);
    return result;
  }
  const auto nearby = canonical::DiscoverNearbyJointCandidates(
      lengths, canonical_pose, tolerance,
      model.coordinates.FromCommand(command_seed));
  if (nearby.status != canonical::RecoveryStatus::Candidates) {
    return result;
  }
  for (std::size_t i = 0; i < nearby.count; ++i) {
    Vec6 command = model.coordinates.ToCommand(nearby.candidates[i].joints);
    // The bounded neighborhood spans less than half a turn: only the lift
    // nearest the seed can belong to it. Other turns need no enumeration.
    command +=
        kTwoPi * ((command_seed - command) / kTwoPi).array().round().matrix();
    if ((command - command_seed).cwiseAbs().maxCoeff() >
            numerics::kNearbyRadiusRad ||
        !ClampCommandToLimits(command, model.lower_limits_rad,
                              model.upper_limits_rad)) {
      continue;
    }
    if (IsFkRoundtripValid(command, target, rotation, fk)) {
      result.status = IkStatus::StrictLocal;
      result.used_singular_representatives =
          nearby.used_singular_representatives;
      result.solutions.push_back(command);
      return result;
    }
  }
  return result;
}

auto BuildArmClassificationGeometry(const PreparedCrxModel &model,
                                    const Vec6 &command_rad,
                                    ArmClassificationGeometry &geometry_out)
    -> bool {
  constexpr std::size_t kJoint1FrameIndex = 1;
  constexpr std::size_t kJoint3FrameIndex = 3;
  constexpr std::size_t kJoint4FrameIndex = 4;

  PoseIsoRT unused_pose = PoseIsoRT::Identity();
  JointPoseBuffer joint_poses;
  if (SolveFKCore(command_rad, unused_pose, &joint_poses, false, model) != 1) {
    return false;
  }

  const PoseIsoRT frame_1_inverse = joint_poses[kJoint1FrameIndex].inverse();
  geometry_out.o3_point = (frame_1_inverse * joint_poses[kJoint3FrameIndex])
                              .translation()
                              .cast<double>();
  geometry_out.o4_point = (frame_1_inverse * joint_poses[kJoint4FrameIndex])
                              .translation()
                              .cast<double>();
  geometry_out.tolerances = BuildClassificationTolerances(model.lengths);
  return geometry_out.o3_point.allFinite() && geometry_out.o4_point.allFinite();
}

auto OrthoDeterminant(const canonical::Lengths &lengths, const Vec6 &q)
    -> double {
  // Expanded determinant of the arm-and-wrist geometric Jacobian. Its sign
  // labels the two regular posture aspects; its magnitude is compared with a
  // length-scaled threshold before a candidate is called regular.
  const double c2 = std::cos(q[2]);
  const double s2 = std::sin(q[2]);
  const double c3 = std::cos(q[3]);
  const double s3 = std::sin(q[3]);
  const double alpha =
      c2 * lengths.b * ((lengths.b * c2) - (lengths.r * s2 * s3));
  const double beta = c2 * c3 * s3 * lengths.r * lengths.a;
  const double gamma =
      (lengths.b * lengths.r * c2 * c2 * s3) +
      (lengths.b * lengths.b * c2 * s2) + (lengths.b * lengths.a * c2) -
      (lengths.r * lengths.a * s2 * s3) - (lengths.b * lengths.r * s3);
  return lengths.a * ((alpha * std::cos(q[1]) * std::sin(q[4])) +
                      (beta * std::sin(q[1]) * std::cos(q[4])) +
                      (gamma * std::sin(q[1]) * std::sin(q[4])));
}

auto CanonicalWristAngle(const PreparedCrxModel &model,
                         const PoseIsoRT &target_pose_06, const Vec6 &q,
                         double &angle) -> bool {
  PoseIsoRT unused_pose;
  JointPoseBuffer frames;
  if (SolveFKCore(q, unused_pose, &frames, false, model) != 1) {
    return false;
  }
  Vec3 o4 = model.base_transform.inverse() * frames[4].translation();
  o4.z() -= model.base_height;
  const Vec3 o5 = target_pose_06.translation() -
                  model.lengths.c * target_pose_06.linear().col(2);
  Vec3 axis = (o4 - o5) / model.lengths.r;
  const double norm = axis.norm();
  if (!axis.allFinite() || !std::isfinite(norm) ||
      norm <= numerics::kGeometryTolerance) {
    return false;
  }
  axis /= norm;
  angle = std::atan2(axis.dot(target_pose_06.linear().col(1)),
                     axis.dot(target_pose_06.linear().col(0)));
  return std::isfinite(angle);
}

auto DeltaAt(const canonical::WristCircle &circle, double angle) -> double {
  const Vec3 axis = circle.v * std::cos(angle) + circle.w * std::sin(angle);
  const Vec3 o4 = circle.p + circle.lengths.r * axis;
  // This scalar is the determinant of the wrist axis, vertical axis, and
  // wrist-center vectors. Its zeros mark the phases where the base-plane
  // intersection changes branch, so command limits are split at these points.
  return o4.dot(axis.cross(Vec3::UnitZ().cross(o4)));
}

struct WristRootLabel {
  double angle = 0.0;
  std::int32_t arc = -1;
  std::int32_t rank = -1;
};

auto CircularAngleDistance(double a, double b) -> double {
  return std::abs(std::remainder(a - b, 2 * angle_conv::kPi));
}

// Keep the first angle in each sorted tolerance cluster. Tolerance proximity
// is not transitive, so it cannot be used as std::unique's equivalence
// predicate. Inputs are canonical angles sorted in [-pi, pi].
auto CompactSortedAngles(std::vector<double> &angles, double tolerance)
    -> void {
  auto output = angles.begin();
  for (auto current = angles.begin(); current != angles.end(); ++current) {
    if (output == angles.begin() || (*current - *(output - 1)) > tolerance) {
      *output++ = *current;
    }
  }
  angles.erase(output, angles.end());
  if (angles.size() > 1 &&
      CircularAngleDistance(angles.front(), angles.back()) <= tolerance) {
    // The sorted endpoints are adjacent on the circle. Keep the first (lower)
    // canonical angle as the deterministic representative.
    angles.pop_back();
  }
}

auto BuildWristBoundaries(const canonical::WristCircle &circle,
                          std::vector<double> &boundaries) -> bool {
  constexpr int kFourierSamples = 5;
  constexpr int kBoundaryScaleSamples = 32;
  constexpr double kResidualTolerance = 1e-8;
  constexpr double kBoundaryMergeTolerance = 1e-7;
  double c0 = 0;
  double c1 = 0;
  double s1 = 0;
  double c2 = 0;
  double s2 = 0;
  // Delta is a trigonometric polynomial of order at most two. Five equally
  // spaced samples recover its constant, first-, and second-harmonic terms
  // without aliasing; the tan(theta/2) substitution then gives a quartic.
  for (int i = 0; i < kFourierSamples; ++i) {
    const double theta = 2 * angle_conv::kPi * i / kFourierSamples;
    const double value = DeltaAt(circle, theta);
    if (!std::isfinite(value)) {
      return false;
    }
    c0 += value / kFourierSamples;
    c1 += 2 * value * std::cos(theta) / kFourierSamples;
    s1 += 2 * value * std::sin(theta) / kFourierSamples;
    c2 += 2 * value * std::cos(2 * theta) / kFourierSamples;
    s2 += 2 * value * std::sin(2 * theta) / kFourierSamples;
  }
  canonical::Coefficients coefficients{};
  coefficients[0] = c0 + c1 + c2;
  coefficients[1] = (2 * s1) + (4 * s2);
  coefficients[2] = (2 * c0) - (6 * c2);
  coefficients[3] = (2 * s1) - (4 * s2);
  coefficients[4] = c0 - c1 + c2;
  if (canonical::RepresentedDegree(coefficients) < 0) {
    return false;
  }
  const auto zeros = canonical::FindRootCandidates(coefficients);
  if (zeros.status != canonical::RootStatus::Candidates &&
      zeros.status != canonical::RootStatus::ConstantPolynomial) {
    return false;
  }
  double delta_scale = 1.0;
  for (int i = 0; i < kBoundaryScaleSamples; ++i) {
    const double theta = 2 * angle_conv::kPi * i / kBoundaryScaleSamples;
    delta_scale = std::max(delta_scale, std::abs(DeltaAt(circle, theta)));
  }
  const double delta_tolerance = kResidualTolerance * delta_scale;
  for (std::size_t i = 0; i < zeros.count; ++i) {
    const auto &root = zeros.candidates[i].value;
    if (std::abs(root.imag()) >
        kResidualTolerance * (1 + std::abs(root.real()))) {
      continue;
    }
    const double theta =
        std::remainder(2 * std::atan(root.real()), 2 * angle_conv::kPi);
    if (std::abs(DeltaAt(circle, theta)) > delta_tolerance) {
      return false;
    }
    boundaries.push_back(theta);
  }
  if (std::abs(DeltaAt(circle, angle_conv::kPi)) <= delta_tolerance) {
    boundaries.push_back(-angle_conv::kPi);
  }
  std::sort(boundaries.begin(), boundaries.end());
  CompactSortedAngles(boundaries, kBoundaryMergeTolerance);
  return true;
}

auto AppendWristRootLabel(double wrapped, const std::vector<double> &boundaries,
                          std::vector<WristRootLabel> &labels) -> void {
  constexpr double kBoundaryMatchTolerance = 1e-7;
  if (boundaries.empty()) {
    labels.push_back({wrapped, 0, static_cast<std::int32_t>(labels.size())});
    return;
  }
  for (const double boundary : boundaries) {
    if (std::abs(std::remainder(wrapped - boundary, 2 * angle_conv::kPi)) <=
        kBoundaryMatchTolerance) {
      labels.push_back({wrapped, -1, -1});
      return;
    }
  }
  const auto upper =
      std::upper_bound(boundaries.begin(), boundaries.end(), wrapped);
  const std::size_t arc =
      upper == boundaries.begin()
          ? boundaries.size() - 1
          : static_cast<std::size_t>(upper - boundaries.begin() - 1);
  labels.push_back({wrapped, static_cast<std::int32_t>(arc), 0});
}

auto AssignWristRootRanks(std::vector<WristRootLabel> &labels,
                          std::size_t arc_count) -> void {
  // Labels arrive in ascending angle order; counting per arc preserves the
  // old rank definition in one pass.
  std::vector<std::int32_t> next_rank(arc_count, 0);
  for (auto &label : labels) {
    if (label.arc < 0) {
      continue;
    }
    label.rank = next_rank[static_cast<std::size_t>(label.arc)]++;
  }
}

auto BuildWristRootLabels(const canonical::WristCircle &circle,
                          std::vector<double> root_angles,
                          std::vector<WristRootLabel> &labels) -> bool {
  constexpr double kRootMergeTolerance = 1e-7;
  std::vector<double> boundaries;
  if (!BuildWristBoundaries(circle, boundaries)) {
    return false;
  }
  for (double &angle : root_angles) {
    angle = std::remainder(angle, 2 * angle_conv::kPi);
  }
  std::sort(root_angles.begin(), root_angles.end());
  CompactSortedAngles(root_angles, kRootMergeTolerance);
  labels.clear();
  labels.reserve(root_angles.size());
  for (const double angle : root_angles) {
    const double wrapped = std::remainder(angle, 2 * angle_conv::kPi);
    AppendWristRootLabel(wrapped, boundaries, labels);
  }
  AssignWristRootRanks(labels,
                       boundaries.empty() ? std::size_t{1} : boundaries.size());
  return true;
}
} // namespace

auto SolveFkIsometry(const CrxModelData &model, const Vec6 &command_rad,
                     PoseIsoRT &pose_out, JointPoseBuffer *joint_poses,
                     bool check_limits) -> int {
  PreparedCrxModel prepared;
  if (PrepareCrxModel(model, prepared) != ModelStatus::Ready ||
      !command_rad.allFinite()) {
    return -1;
  }
  return SolveFKCore(command_rad, pose_out, joint_poses, check_limits,
                     prepared);
}

auto ClassifyArmPosture(const CrxModelData &model, const Vec6 &command_rad,
                        ArmPosture &posture_out) -> bool {
  PreparedCrxModel prepared;
  if (PrepareCrxModel(model, prepared) != ModelStatus::Ready ||
      !command_rad.allFinite()) {
    return false;
  }

  ArmClassificationGeometry geometry;
  if (!BuildArmClassificationGeometry(prepared, command_rad, geometry)) {
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
  const bool flip = command_rad[4] > numerics::kConfigWristZeroRad;

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
  if (approx_joints_rad != nullptr && !approx_joints_rad->allFinite()) {
    return result;
  }
  PreparedIkProblem problem;
  const auto preparation = PrepareIkProblem(model, target_pose, problem);
  if (preparation != ModelStatus::Ready) {
    result.status = ModelStatusToIkStatus(preparation);
    return result;
  }
  const auto &fk = problem.model;
  const auto &target_quaternion = problem.target_quaternion;
  const auto &target_pose_06 = problem.target_pose_06;
  const auto &lengths = fk.lengths;
  const auto &tolerance = problem.tolerance;
  if (max_solutions == 1 && approx_joints_rad != nullptr) {
    auto nearby = TryNearbySolution(fk, lengths, target_pose, target_pose_06,
                                    tolerance, *approx_joints_rad);
    if (!nearby.solutions.empty()) {
      return nearby;
    }
  }
  const auto finish = [&](const canonical::JointDiscovery &discovered,
                          bool full_search) -> IkResult {
    IkResult outcome;
    std::vector<Vec6> geometric_solutions;
    if (discovered.status != canonical::RecoveryStatus::Candidates &&
        discovered.status != canonical::RecoveryStatus::NoCandidate) {
      // Unresolved discovery must not be accepted through the seed policy.
      outcome.status = RecoveryStatusToIkStatus(discovered.status);
      return outcome;
    }
    outcome.used_singular_representatives =
        discovered.used_singular_representatives;
    // Every command still receives a complete FK check; only model constants
    // are shared across those checks during this solve.
    if (discovered.status == canonical::RecoveryStatus::Candidates) {
      geometric_solutions.reserve(discovered.count);
      for (std::size_t i = 0; i < discovered.count; ++i) {
        const Vec6 command =
            fk.coordinates.ToCommand(discovered.candidates[i].joints);
        if (!IsFkRoundtripValid(command, target_pose, target_quaternion, fk)) {
          outcome.status = IkStatus::Unresolved;
          return outcome;
        }
        geometric_solutions.push_back(command);
      }
    }

    if (geometric_solutions.empty()) {
      outcome.status = IkStatus::NoCandidate;
      if (full_search) {
        ApplyEmptySearchSeed(fk, target_pose, target_quaternion,
                             approx_joints_rad, outcome);
      }
      return outcome;
    }

    auto finalized = FinalizeIkCandidates(geometric_solutions, fk, target_pose,
                                          target_quaternion, approx_joints_rad,
                                          max_solutions);
    if (full_search && finalized.status == IkStatus::NoCandidate) {
      const auto constrained = CorrectConstrainedCandidates(
          geometric_solutions, fk, lengths, target_pose_06, tolerance);
      if (!constrained.empty()) {
        finalized = FinalizeIkCandidates(constrained, fk, target_pose,
                                         target_quaternion, approx_joints_rad,
                                         max_solutions);
      }
    }
    finalized.used_singular_representatives =
        outcome.used_singular_representatives;
    return finalized;
  };
  // A filtered miss must not authorize approximate seed fallback or omit a
  // constrained nomination. Retry the original discovery before either.
  if (model.upper_limits_rad[2] - model.lower_limits_rad[2] < kTwoPi ||
      model.upper_limits_rad[5] - model.lower_limits_rad[5] < kTwoPi) {
    canonical::DiscoveryOptions options;
    Vec6 canonical_lower, canonical_upper;
    fk.coordinates.CanonicalBounds(model.lower_limits_rad,
                                   model.upper_limits_rad, canonical_lower,
                                   canonical_upper);
    options.lower_limits = &canonical_lower;
    options.upper_limits = &canonical_upper;
    const auto discovered = canonical::DiscoverJointCandidates(
        lengths, target_pose_06, tolerance, options);
    if (discovered.pruned_nominations == 0 &&
        discovered.status == canonical::RecoveryStatus::Candidates) {
      return finish(discovered, true);
    }
    if (discovered.status == canonical::RecoveryStatus::Candidates) {
      auto filtered = finish(discovered, false);
      if (!filtered.solutions.empty()) {
        return filtered;
      }
    }
  }
  return finish(
      canonical::DiscoverJointCandidates(lengths, target_pose_06, tolerance),
      true);
}

auto IdentifyIkIsometry(const CrxModelData &model, const PoseIsoRT &target_pose,
                        int max_solutions) -> FibreResult {
  FibreResult result;
  if (max_solutions <= 0) {
    result.status = IkStatus::NoOutputRequested;
    return result;
  }
  PreparedIkProblem problem;
  const auto preparation = PrepareIkProblem(model, target_pose, problem);
  if (preparation != ModelStatus::Ready) {
    result.status = ModelStatusToIkStatus(preparation);
    return result;
  }
  const auto &fk = problem.model;
  const auto &target_quaternion = problem.target_quaternion;
  const auto &target_pose_06 = problem.target_pose_06;
  const auto &tolerance = problem.tolerance;
  const auto discovered =
      canonical::DiscoverJointCandidates(fk.lengths, target_pose_06, tolerance);
  result.used_singular_representatives =
      discovered.used_singular_representatives;
  if (discovered.status != canonical::RecoveryStatus::Candidates) {
    result.status = RecoveryStatusToIkStatus(discovered.status);
    return result;
  }

  canonical::WristCircle circle;
  if (canonical::PrepareWristCircle(fk.lengths, target_pose_06, circle) !=
      canonical::PreparationStatus::Ready) {
    result.status = IkStatus::Unresolved;
    return result;
  }
  canonical::HalfAnglePolynomial residual_polynomial;
  const bool residual_is_discrete =
      canonical::BuildHalfAnglePolynomial(circle, 0, residual_polynomial) ==
          canonical::PreparationStatus::Ready &&
      canonical::RepresentedDegree(residual_polynomial.coefficients) > 0;
  std::vector<double> wrist_angles;
  std::vector<Vec6> geometric_solutions;
  geometric_solutions.reserve(discovered.count);
  wrist_angles.reserve(discovered.count);
  for (std::size_t i = 0; i < discovered.count; ++i) {
    const Vec6 candidate =
        fk.coordinates.ToCommand(discovered.candidates[i].joints);
    if (!IsFkRoundtripValid(candidate, target_pose, target_quaternion, fk)) {
      result.status = IkStatus::Unresolved;
      return result;
    }
    double angle = 0;
    if (!CanonicalWristAngle(fk, target_pose_06, candidate, angle)) {
      result.status = IkStatus::Unresolved;
      return result;
    }
    geometric_solutions.push_back(candidate);
    wrist_angles.push_back(angle);
  }
  std::vector<WristRootLabel> root_labels;
  const bool root_labels_valid =
      residual_is_discrete &&
      BuildWristRootLabels(circle, wrist_angles, root_labels);

  const auto command_postures = PrepareCommandPostures(
      geometric_solutions, fk, target_pose, target_quaternion);
  const auto selected = SelectCommandLifts(
      command_postures, fk.lower_limits_rad, fk.upper_limits_rad, nullptr,
      max_solutions,
      [&](const Vec6 &command) {
        return IsFkRoundtripValid(command, target_pose, target_quaternion, fk);
      },
      numerics::kCommandLiftBudget, CommandSelectionMode::RankedPrefix);
  if (selected.status != CommandSelectionStatus::Complete) {
    result.status = CommandSelectionStatusToIkStatus(selected.status);
    return result;
  }
  result.truncated = selected.truncated;
  for (const auto &lift : selected.commands) {
    std::size_t source = geometric_solutions.size();
    for (std::size_t i = 0; i < geometric_solutions.size(); ++i) {
      Vec6 difference = lift.command_rad - geometric_solutions[i];
      difference = difference.unaryExpr(
          [](double value) { return std::remainder(value, kTwoPi); });
      if (difference.cwiseAbs().maxCoeff() <=
          numerics::kCommandDedupToleranceRad) {
        source = i;
        break;
      }
    }
    if (source == geometric_solutions.size()) {
      result.solutions.clear();
      result.status = IkStatus::Unresolved;
      return result;
    }
    const Vec6 q = fk.coordinates.FromCommand(geometric_solutions[source]);
    const double determinant = OrthoDeterminant(fk.lengths, q);
    if (!std::isfinite(determinant)) {
      result.solutions.clear();
      result.status = IkStatus::NumericalRangeFailure;
      return result;
    }
    const double determinant_scale =
        std::max({1.0, std::abs(fk.lengths.a), std::abs(fk.lengths.b),
                  std::abs(fk.lengths.r)});
    const double singular_tolerance =
        1e-10 * determinant_scale * determinant_scale * determinant_scale;
    const bool regular = std::isfinite(determinant) &&
                         std::abs(determinant) > singular_tolerance;
    const double root_angle = wrist_angles[source];
    const double denominator = DeltaAt(circle, root_angle);
    const double residual_derivative =
        canonical::EvaluateResidualDerivative(circle, root_angle);
    if (!std::isfinite(denominator) || !std::isfinite(residual_derivative)) {
      result.solutions.clear();
      result.status = IkStatus::Unresolved;
      return result;
    }
    // f' = 2*b*Delta*det(Jcanonical). Compare signs separately so the
    // consistency check cannot overflow or underflow by multiplying factors.
    const bool derivative_delta_negative =
        std::signbit(residual_derivative) != std::signbit(denominator);
    const bool expected_determinant_negative =
        derivative_delta_negative != std::signbit(fk.lengths.b);
    if (regular && std::abs(denominator) > 1e-8 &&
        std::abs(residual_derivative) > 1e-8 &&
        expected_determinant_negative != std::signbit(determinant)) {
      result.solutions.clear();
      result.status = IkStatus::Unresolved;
      return result;
    }
    FibreSolution identified;
    identified.joints = lift.command_rad;
    identified.regular = regular;
    if (regular) {
      identified.aspect_sign = determinant > 0 ? 1 : -1;
    }
    if (root_labels_valid) {
      const double angle = std::remainder(root_angle, 2 * angle_conv::kPi);
      const auto label = std::find_if(
          root_labels.begin(), root_labels.end(),
          [&](const WristRootLabel &item) {
            return std::abs(std::remainder(item.angle - angle,
                                           2 * angle_conv::kPi)) <= 1e-7;
          });
      const double shoulder_side = std::sin(q[1]);
      if (label != root_labels.end() && label->arc >= 0 &&
          std::abs(shoulder_side) > 1e-8) {
        // Rank is zero based; bit 0 is the positive shoulder side.
        identified.wrist_address = {label->arc, label->rank,
                                    shoulder_side < 0 ? 1 : 0};
        identified.wrist_address_valid = true;
      }
    }
    result.solutions.push_back(identified);
  }
  result.status = result.solutions.empty() ? IkStatus::NoCandidate
                                           : IkStatus::StrictCandidates;
  return result;
}

} // namespace crx
