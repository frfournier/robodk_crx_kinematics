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
#include "crx_discovery.h"
#include "crx_joint_recovery.h"
#include "crx_joint_trig.h"
#include "crx_math_helpers.h"
#include "crx_numerics.h"
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
        AngleDiffAbs(row.theta0_rad, kTheta.at(i)) >
            numerics::kRightAngleSnapToleranceRad ||
        std::abs(model.joint_senses.at(i) - kSenses.at(i)) >
            numerics::kUnitTolerance ||
        (i != 2 && std::abs(row.a) > length_tol) ||
        ((i == 1 || i == 2) && std::abs(row.d) > length_tol)) {
      return ModelStatus::UnsupportedModel;
    }
  }
  const auto &rows = model.dh_rows;
  if (rows[2].a <= 0 || rows[0].d <= 0 || rows[3].d <= 0 ||
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
  const JointTrig::Values angles{command_rad[0],  command_rad[1] - kHalfPi,
                                 -command_rad[2], -command_rad[3],
                                 -command_rad[4], -command_rad[5]};
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
    const double sa = kSine.at(storage_index), ca = kCosine.at(storage_index);
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
                          const PreparedCrxModel &fk,
                          const PoseIsoRT &target_pose,
                          const Eigen::Quaterniond &target_quaternion,
                          const Vec6 *approx_joints_rad, int max_solutions)
    -> IkResult {
  const auto &model = fk;
  IkResult result;
  auto command_postures = geometric_solutions;
  // Root reconstruction can place a boundary posture just beyond a limit,
  // even when other branches remain feasible. Nominate the actual boundary
  // only within posture identity tolerance, then recheck the complete pose.
  // Command enumeration and its limit roundoff allowance remain unchanged.
  for (auto &command : command_postures) {
    Vec6 bounded;
    if (NominateBoundaryPosture(command, model, bounded) &&
        IsFkRoundtripValid(bounded, target_pose, target_quaternion, fk)) {
      command = bounded;
    }
  }
  const auto selected = SelectCommandLifts(
      command_postures, model.lower_limits_rad, model.upper_limits_rad,
      approx_joints_rad, max_solutions,
      [&](const Vec6 &command) {
        return IsFkRoundtripValid(command, target_pose, target_quaternion, fk);
      },
      numerics::kCommandLiftBudget, CommandSelectionMode::RankedPrefix);
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
    candidate.joints = clamped;
    if (canonical::PolishPreparedJointCandidate(
            circle, target_pose_06, tolerance, numerics::kPolishIterations,
            candidate, &model.lower_limits_rad, &model.upper_limits_rad)) {
      constrained.push_back(candidate.joints);
      canonical::JointCandidate partner;
      if (canonical::RecoverBaseFlip(circle, target_pose_06, tolerance,
                                     candidate, partner)) {
        constrained.push_back(partner.joints);
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
      lengths, canonical_pose, tolerance, command_seed);
  if (nearby.status != canonical::RecoveryStatus::Candidates) {
    return result;
  }
  for (std::size_t i = 0; i < nearby.count; ++i) {
    Vec6 command = nearby.candidates[i].joints;
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
  if (!target_pose.matrix().allFinite() ||
      (approx_joints_rad != nullptr && !approx_joints_rad->allFinite())) {
    return result;
  }
  PreparedCrxModel fk;
  const auto preparation = PrepareCrxModel(model, fk);
  if (preparation != ModelStatus::Ready) {
    result.status = preparation == ModelStatus::UnsupportedModel
                        ? IkStatus::UnsupportedModel
                        : IkStatus::InvalidInput;
    return result;
  }
  const Eigen::Quaterniond target_quaternion(target_pose.linear());
  PoseIsoRT target_pose_06 =
      fk.base_transform.inverse() * target_pose * fk.tool_transform.inverse();
  target_pose_06.translation().z() -= fk.base_height;
  const auto &lengths = fk.lengths;
  const canonical::PoseTolerance tolerance{
      numerics::kSolutionPositionToleranceMm,
      numerics::kSolutionAngleToleranceRad};
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
      outcome.status =
          discovered.status == canonical::RecoveryStatus::InvalidInput
              ? IkStatus::InvalidInput
          : discovered.status ==
                  canonical::RecoveryStatus::NumericalRangeFailure
              ? IkStatus::NumericalRangeFailure
              : IkStatus::Unresolved;
      return outcome;
    }
    outcome.used_singular_representatives =
        discovered.used_singular_representatives;
    // Every command still receives a complete FK check; only model constants
    // are shared across those checks during this solve.
    if (discovered.status == canonical::RecoveryStatus::Candidates) {
      geometric_solutions.reserve(discovered.count);
      for (std::size_t i = 0; i < discovered.count; ++i) {
        const Vec6 &command = discovered.candidates[i].joints;
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
    options.lower_limits = &model.lower_limits_rad;
    options.upper_limits = &model.upper_limits_rad;
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

} // namespace crx
