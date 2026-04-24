#pragma once

#include <array>
#include <string>
#include <vector>

#include "crx_types.h"

namespace crx {

auto SolveFkIsometry(const CrxModelData &model, const Vec6 &user_joints_rad,
                     PoseIsoRT &pose_out, std::vector<PoseIsoRT> *joint_poses,
                     bool check_limits) -> int;

auto SolveIkIsometry(const CrxModelData &model, const PoseIsoRT &target_pose,
                     const Vec6 *approx_joints_rad, int max_solutions,
                     std::vector<Vec6> &solutions_out) -> int;

auto SolveIkIsometryConfigured(const CrxModelData &model,
                               const PoseIsoRT &target_pose,
                               const Vec6 *approx_joints_rad,
                               int max_solutions,
                               std::vector<CrxIkSolution> &solutions_out)
    -> int;

auto FanucConfigString(const CrxConfiguration &config) -> std::string;

auto RoboDkConfigVector(const CrxConfiguration &config)
    -> std::array<int, kRoboDkConfigStride>;

} // namespace crx
