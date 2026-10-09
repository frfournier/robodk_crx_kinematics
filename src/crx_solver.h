#pragma once

#include <array>
#include <vector>

#include "crx_types.h"

namespace crx {

using JointPoseBuffer = std::array<PoseIsoRT, kDofCount + 1>;

enum class IkStatus {
  StrictCandidates,
  StrictSeed,
  ApproximateSeed,
  NoCandidate,
  NoOutputRequested,
  InvalidInput,
  UnsupportedModel,
  Unresolved,
  NumericalRangeFailure,
  WorkLimit
};

// Numerical outcomes only: neither candidates nor an empty search certify
// completeness/infeasibility. Failures contain no solutions. Truncation is a
// successful ranked prefix, distinct from exhausted discovery/selection work.
struct IkResult {
  IkStatus status = IkStatus::InvalidInput;
  bool truncated = false;
  // Canonical discovery used a deterministic free-base representative. This
  // describes discovery, even if command limits remove that representative.
  bool used_singular_representatives = false;
  std::vector<Vec6> solutions;
};

auto SolveFkIsometry(const CrxModelData &model, const Vec6 &user_joints_rad,
                     PoseIsoRT &pose_out, JointPoseBuffer *joint_poses,
                     bool check_limits) -> int;

auto SolveIkIsometry(const CrxModelData &model, const PoseIsoRT &target_pose,
                     const Vec6 *approx_joints_rad, int max_solutions)
    -> IkResult;

auto ClassifyArmPosture(const CrxModelData &model, const Vec6 &user_joints_rad,
                        ArmPosture &posture_out) -> bool;

} // namespace crx
