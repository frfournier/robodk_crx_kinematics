#pragma once

#include <string>

#include "crx_kinematics.h"
#include "crx_types.h"

auto iRobot_Name(const robot_T *robot) -> std::string;

namespace crx {

struct IkResult;

// Preserve RoboDK's count/zero/generic-fallback ABI. Approximate seed success
// is deliberately retained for compatibility; no reserved fields are used.
auto IkResultToRoboDkReturn(const IkResult &result) -> int;

auto RoboDkDofCount(const robot_T *robot) -> int;
auto BuildModelFromRoboDkRobot(const robot_T *robot, CrxModelData &model)
    -> bool;
auto RoboDkPoseToCore(const real_T *pose, PoseIsoRT &pose_out) -> bool;
auto CorePoseToRoboDk(const PoseIsoRT &pose, real_T *pose_out) -> bool;
auto RoboDkJointsDegToCommandRad(const real_T *joints_deg, Vec6 &command_rad)
    -> bool;
auto CommandRadToRoboDkJointsDeg(const Vec6 &command_rad, real_T *joints_deg)
    -> bool;

} // namespace crx
