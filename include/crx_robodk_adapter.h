#pragma once

#include "crx_types.h"

namespace crx {

auto RoboDkDofCount(const robot_T *robot) -> int;
auto BuildModelFromRoboDkRobot(const robot_T *robot, CrxModelData &model)
    -> bool;

} // namespace crx
