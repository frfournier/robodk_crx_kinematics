#pragma once

#include <string>

#include "crx_types.h"

auto iRobot_Name(const robot_T *robot) -> std::string;

namespace crx {

auto RoboDkDofCount(const robot_T *robot) -> int;
auto BuildModelFromRoboDkRobot(const robot_T *robot, CrxModelData &model)
    -> bool;
auto RoboDkJointsDegCoupledToUserRad(const real_T *joints_deg,
                                     Vec6 &joints_user_rad) -> bool;
auto UserJointsRadToRoboDkCoupledDeg(const Vec6 &joints_user_rad,
                                     real_T *joints_deg) -> bool;

} // namespace crx
