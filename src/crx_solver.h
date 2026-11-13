// SPDX-License-Identifier: Apache-2.0
// SPDX-FileCopyrightText: 2026 Centre Technologique en Aérospatiale

#pragma once

#include <array>
#include <cstdint>
#include <vector>

#include "crx_command_lifts.h"
#include "crx_types.h"

namespace crx {

// All joint vectors are coupled RoboDK commands in radians.
using JointPoseBuffer = std::array<PoseIsoRT, kDofCount + 1>;

enum class IkStatus {
  StrictCandidates,
  StrictSeed,
  StrictLocal,
  ApproximateSeed,
  NoCandidate,
  NoOutputRequested,
  InvalidInput,
  UnsupportedModel,
  Unresolved,
  NumericalRangeFailure,
  WorkLimit
};

// Coverage describes the internal phase catalogue before command limits,
// turn expansion or returned-prefix truncation. Numerical discovery alone
// never upgrades this field.
enum class Coverage { NotAttempted, Complete, Inconclusive, ContinuousFibre };

// Numerical outcomes only: neither candidates nor an empty search certify
// completeness/infeasibility. Failures contain no solutions. Truncation is a
// successful ranked prefix, distinct from exhausted discovery/selection work.
// With one output and a seed, StrictSeed/StrictLocal can return before full
// discovery: no global ranking or truncation claim is made for that fast path.
struct IkResult {
  IkStatus status = IkStatus::InvalidInput;
  bool truncated = false;
  // Canonical discovery used deterministic base/folded-family representatives.
  // This describes discovery, even if command limits remove that
  // representative.
  bool used_singular_representatives = false;
  std::vector<Vec6> solutions;
  Coverage coverage = Coverage::NotAttempted;
};

struct FibreSolution {
  Vec6 joints = Vec6::Zero();
  std::int32_t aspect_sign = 0;
  bool regular = false;
  std::array<std::int32_t, 3> wrist_address{-1, -1, -1};
  bool wrist_address_valid = false;
};

struct FibreResult {
  IkStatus status = IkStatus::InvalidInput;
  bool truncated = false;
  bool used_singular_representatives = false;
  std::vector<FibreSolution> solutions;
  Coverage coverage = Coverage::NotAttempted;
};

auto SolveFkIsometry(const CrxModelData &model, const Vec6 &command_rad,
                     PoseIsoRT &pose_out, JointPoseBuffer *joint_poses,
                     bool check_limits) -> int;

auto SolveIkIsometry(const CrxModelData &model, const PoseIsoRT &target_pose,
                     const Vec6 *approx_joints_rad, int max_solutions)
    -> IkResult;

auto IdentifyIkIsometry(const CrxModelData &model, const PoseIsoRT &target_pose,
                        int max_solutions) -> FibreResult;

auto ClassifyArmPosture(const CrxModelData &model, const Vec6 &command_rad,
                        ArmPosture &posture_out) -> bool;

} // namespace crx
