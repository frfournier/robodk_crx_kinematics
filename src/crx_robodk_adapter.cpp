#include "crx_robodk_adapter.h"

#include "crx_math_helpers.h"
#include "crx_pose_helpers.h"

namespace {

static constexpr int kRobotTableStride = 20;
static constexpr int kRobotBaseXyzwprRow = 9;
static constexpr int kRobotToolXyzwprRow = 28;
static constexpr int kRobotJointLowerLimitRow = 30;
static constexpr int kRobotJointUpperLimitRow = 31;
static constexpr int kRobotJointSensesRow = 3;
static constexpr int kRobotJointSensesCol = 4;
static constexpr int kRobotDhBaseRow = 10;
static constexpr int kRobotDofRow = 1;
static constexpr int kRobotDofCol = 1;

static inline auto iRobot_At(const robot_T *r, int row, int col = 0)
    -> const real_T * {
  return reinterpret_cast<const real_T *>(r) + row * kRobotTableStride + col;
}

static inline auto iRobot_BaseXYZWPR(const robot_T *r) -> const real_T * {
  return iRobot_At(r, kRobotBaseXyzwprRow);
}

static inline auto iRobot_ToolXYZWPR(const robot_T *r) -> const real_T * {
  return iRobot_At(r, kRobotToolXyzwprRow);
}

static inline auto iRobot_JointLimLower(const robot_T *r) -> const real_T * {
  return iRobot_At(r, kRobotJointLowerLimitRow);
}

static inline auto iRobot_JointLimUpper(const robot_T *r) -> const real_T * {
  return iRobot_At(r, kRobotJointUpperLimitRow);
}

static inline auto iRobot_JointSenses(const robot_T *r) -> const real_T * {
  return iRobot_At(r, kRobotJointSensesRow, kRobotJointSensesCol);
}

static inline auto iRobot_DhmJoint(const robot_T *r, int joint)
    -> const real_T * {
  return iRobot_At(r, kRobotDhBaseRow + joint);
}

} // namespace

namespace crx {

auto RoboDkDofCount(const robot_T *robot) -> int {
  if (robot == nullptr)
    return 0;
  return static_cast<int>(*iRobot_At(robot, kRobotDofRow, kRobotDofCol));
}

auto BuildModelFromRoboDkRobot(const robot_T *robot, CrxModelData &model)
    -> bool {
  if (robot == nullptr)
    return false;
  if (RoboDkDofCount(robot) != kDofCount)
    return false;

  model.base_transform = XYZWPR_ToIsometry(iRobot_BaseXYZWPR(robot));
  model.tool_transform = XYZWPR_ToIsometry(iRobot_ToolXYZWPR(robot));

  const real_T *joint_senses = iRobot_JointSenses(robot);
  const real_T *lower_limits = iRobot_JointLimLower(robot);
  const real_T *upper_limits = iRobot_JointLimUpper(robot);
  const double deg_to_rad = angle_conv::DegToRad(1.0);

  for (int joint_id = 0; joint_id < kDofCount; ++joint_id) {
    model.joint_senses[joint_id] = NormalizeJointSense(joint_senses[joint_id]);
    model.lower_limits_rad[joint_id] =
        static_cast<double>(lower_limits[joint_id]) * deg_to_rad;
    model.upper_limits_rad[joint_id] =
        static_cast<double>(upper_limits[joint_id]) * deg_to_rad;

    const real_T *dh_row = iRobot_DhmJoint(robot, joint_id);
    DhRow row;
    row.alpha_rad = static_cast<double>(dh_row[0]);
    row.a = static_cast<double>(dh_row[1]);
    row.theta0_rad = static_cast<double>(dh_row[2]);
    row.d = static_cast<double>(dh_row[3]);
    row.is_prismatic = (dh_row[4] != 0.0);
    model.dh_rows[joint_id] = row;
  }
  return true;
}

} // namespace crx
