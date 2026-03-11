#include "crx_robodk_adapter.h"

#include <algorithm>
#include <cmath>
#include <string>

#include "crx_math_helpers.h"
#include "crx_pose_helpers.h"
#include "crx_vector_helpers.h"

namespace {

static constexpr int kRobotTableStride = 20;
static constexpr int kRobotDofRow = 1;
static constexpr int kRobotDofCol = 1;
static constexpr int kRobotJointSensesRow = 3;
static constexpr int kRobotJointSensesCol = 4;
static constexpr int kRobotBaseXyzwprRow = 9;
static constexpr int kRobotDhBaseRow = 10;
static constexpr int kRobotToolXyzwprRow = 28;
static constexpr int kRobotJointLowerLimitRow = 30;
static constexpr int kRobotJointUpperLimitRow = 31;
static constexpr int kRobotNameRow = 90;
static constexpr int kRobotNameSize = 59;

static inline auto iRobot_At(const robot_T *r, int row, int col = 0)
    -> const real_T * {
  return reinterpret_cast<const real_T *>(r) + row * kRobotTableStride + col;
}

static inline auto iRobot_Dof(const robot_T *r) -> const real_T * {
  return iRobot_At(r, kRobotDofRow, kRobotDofCol);
}

static inline auto iRobot_JointSenses(const robot_T *r) -> const real_T * {
  return iRobot_At(r, kRobotJointSensesRow, kRobotJointSensesCol);
}

static inline auto iRobot_BaseXYZWPR(const robot_T *r) -> const real_T * {
  return iRobot_At(r, kRobotBaseXyzwprRow);
}

static inline auto iRobot_DhmJoint(const robot_T *r, int joint)
    -> const real_T * {
  return iRobot_At(r, kRobotDhBaseRow + joint);
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

/*!
 * \brief iRobot_Name returns the name of the robot
 * \param r
 * \return
 */
static auto iRobot_Name(const robot_T *r) -> std::string {
  const real_T *name_buffer = iRobot_At(r, kRobotNameRow);
  const real_T *name_end =
      std::find(name_buffer, name_buffer + kRobotNameSize, 0.0);

  std::string name;
  name.reserve(static_cast<size_t>(name_end - name_buffer));

  for (const real_T *it = name_buffer; it != name_end; ++it) {
    if (!std::isfinite(*it) || std::trunc(*it) != *it || *it < 0.0 ||
        *it > 127.0) {
      break; // or return {}, or append '?', depending on policy
    }

    name.push_back(static_cast<char>(*it));
  }

  return name;
}

} // namespace

namespace crx {

auto RoboDkDofCount(const robot_T *robot) -> int {
  if (robot == nullptr)
    return 0;
  return static_cast<int>(*iRobot_Dof(robot));
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

auto RoboDkJointsDegCoupledToUserRad(const real_T *joints_deg,
                                     Vec6 &joints_user_rad) -> bool {
  if (joints_deg == nullptr)
    return false;

  // RoboDK API boundary convention for CRX uses coupled J3 (J2 + J3_decoupled).
  // Internal solver convention uses decoupled J3.
  DegArrayToRadVec(joints_deg, joints_user_rad);
  ConvertJ23CoupledToDecoupled(joints_user_rad);
  return true;
}

auto UserJointsRadToRoboDkCoupledDeg(const Vec6 &joints_user_rad,
                                     real_T *joints_deg) -> bool {
  if (joints_deg == nullptr)
    return false;

  // Convert back to RoboDK CRX API convention before returning solutions.
  Vec6 joints_api_rad = joints_user_rad;
  ConvertJ23DecoupledToCoupled(joints_api_rad);
  RadVecToDegArray(joints_api_rad, joints_deg);
  return true;
}

} // namespace crx
