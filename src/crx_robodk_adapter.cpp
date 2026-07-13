#include "crx_robodk_adapter.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <iostream>
#include <string>

#include <Eigen/Geometry>

#include "crx_kinematics.h"
#include "crx_math_helpers.h"
#include "crx_types.h"

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

static auto XYZWPRToCoreIsometry(const real_T xyzwpr[6]) -> crx::PoseIsoRT {
  using AngleAxis = Eigen::AngleAxis<crx::Scalar>;
  crx::PoseIsoRT transform = crx::PoseIsoRT::Identity();
  transform.linear() = (AngleAxis(xyzwpr[5], crx::Vec3::UnitZ()) *
                        AngleAxis(xyzwpr[4], crx::Vec3::UnitY()) *
                        AngleAxis(xyzwpr[3], crx::Vec3::UnitX()))
                           .toRotationMatrix();
  transform.translation() << xyzwpr[0], xyzwpr[1], xyzwpr[2];
  return transform;
}

static inline auto iRobot_At(const robot_T *r, int row, int col = 0)
    -> const real_T * {
  const auto offset = static_cast<std::ptrdiff_t>(row) * kRobotTableStride +
                      static_cast<std::ptrdiff_t>(col);
  return reinterpret_cast<const real_T *>(r) + offset;
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
} // namespace

auto iRobot_Name(const robot_T *r) -> std::string {
  if (r == nullptr)
    return {};

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

namespace {

static auto LogRobotAdapterWarning(const robot_T *robot,
                                   const std::string &message) -> void {
  std::clog << "crx_robodk_adapter";
  if (robot != nullptr) {
    const std::string robot_name = iRobot_Name(robot);
    if (!robot_name.empty())
      std::clog << " [" << robot_name << "]";
  }
  std::clog << ": " << message << '\n';
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
  if (robot == nullptr) {
    LogRobotAdapterWarning(robot, "null robot pointer");
    return false;
  }

  const int robot_dof = RoboDkDofCount(robot);
  if (robot_dof != kDofCount) {
    LogRobotAdapterWarning(robot, "expected " + std::to_string(kDofCount) +
                                      " DOF but received " +
                                      std::to_string(robot_dof));
    return false;
  }

  model.base_transform = XYZWPRToCoreIsometry(iRobot_BaseXYZWPR(robot));
  model.tool_transform = XYZWPRToCoreIsometry(iRobot_ToolXYZWPR(robot));

  const real_T *joint_senses = iRobot_JointSenses(robot);
  const real_T *lower_limits = iRobot_JointLimLower(robot);
  const real_T *upper_limits = iRobot_JointLimUpper(robot);
  const double deg_to_rad = angle_conv::DegToRad(1.0);

  for (int joint_id = 0; joint_id < kDofCount; ++joint_id) {
    const auto storage_index = static_cast<std::size_t>(joint_id);
    const auto eigen_index = static_cast<Eigen::Index>(joint_id);
    const double joint_sense = static_cast<double>(joint_senses[joint_id]);
    if (!std::isfinite(joint_sense) || std::abs(joint_sense) != 1.0) {
      LogRobotAdapterWarning(robot, "invalid joint sense at index " +
                                        std::to_string(joint_id));
      return false;
    }
    model.joint_senses[storage_index] = joint_sense;
    model.lower_limits_rad[eigen_index] =
        static_cast<double>(lower_limits[joint_id]) * deg_to_rad;
    model.upper_limits_rad[eigen_index] =
        static_cast<double>(upper_limits[joint_id]) * deg_to_rad;

    const real_T *dh_row = iRobot_DhmJoint(robot, joint_id);
    DhRow row;
    row.alpha_rad = static_cast<double>(dh_row[0]);
    row.a = static_cast<double>(dh_row[1]);
    row.theta0_rad = static_cast<double>(dh_row[2]);
    row.d = static_cast<double>(dh_row[3]);
    row.is_prismatic = (dh_row[4] != 0.0);
    model.dh_rows[storage_index] = row;
  }
  return true;
}

auto RoboDkPoseToCore(const real_T *pose, PoseIsoRT &pose_out) -> bool {
  if (pose == nullptr) {
    return false;
  }
  pose_out.matrix() = Eigen::Map<const PoseMatRT>(pose);
  return pose_out.matrix().allFinite();
}

auto CorePoseToRoboDk(const PoseIsoRT &pose, real_T *pose_out) -> bool {
  if (pose_out == nullptr || !pose.matrix().allFinite()) {
    return false;
  }
  Eigen::Map<PoseMatRT> pose_map(pose_out);
  pose_map = pose.matrix();
  return true;
}

auto RoboDkJointsDegCoupledToUserRad(const real_T *joints_deg,
                                     Vec6 &joints_user_rad) -> bool {
  if (joints_deg == nullptr)
    return false;

  // RoboDK API boundary convention for CRX uses coupled J3 (J2 + J3_decoupled).
  // The canonical internal CRX joint vector uses decoupled J3 in radians.
  joints_user_rad =
      Eigen::Map<const Vec6>(joints_deg) * angle_conv::DegToRad(1.0);
  joints_user_rad[kJoint3Index] -= joints_user_rad[kJoint2Index];
  return joints_user_rad.allFinite();
}

auto UserJointsRadToRoboDkCoupledDeg(const Vec6 &joints_user_rad,
                                     real_T *joints_deg) -> bool {
  if (joints_deg == nullptr)
    return false;

  // Convert back to RoboDK CRX API convention before returning solutions.
  Vec6 joints_api_rad = joints_user_rad;
  if (!joints_api_rad.allFinite()) {
    return false;
  }
  joints_api_rad[kJoint3Index] += joints_api_rad[kJoint2Index];
  Eigen::Map<Vec6> joints_map(joints_deg);
  joints_map = joints_api_rad * angle_conv::RadToDeg(1.0);
  return true;
}

} // namespace crx
