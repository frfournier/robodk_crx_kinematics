#pragma once

#include <array>
#include <vector>

#include "crx_types.h"

namespace crx {

using JointPoseBuffer = std::array<PoseIsoRT, kDofCount + 1>;

auto SolveFkIsometry(const CrxModelData &model, const Vec6 &user_joints_rad,
                     PoseIsoRT &pose_out, JointPoseBuffer *joint_poses,
                     bool check_limits) -> int;

auto SolveIkIsometry(const CrxModelData &model, const PoseIsoRT &target_pose,
                     const Vec6 *approx_joints_rad, int max_solutions,
                     std::vector<Vec6> &solutions_out) -> int;

auto ClassifyArmPosture(const CrxModelData &model, const Vec6 &user_joints_rad,
                        ArmPosture &posture_out) -> bool;

} // namespace crx
