#pragma once

#include <array>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "crx_kinematics.h"

namespace crx {

inline constexpr int kPoseRowCount = 4;
inline constexpr int kPoseColCount = 4;
inline constexpr int kPoseElementCount = kPoseRowCount * kPoseColCount;

inline constexpr int kDofCount = 6;
inline constexpr int kSolutionStride = 2 * kDofCount;
inline constexpr int kRoboDkConfigStride = 6;

inline constexpr int kJoint2Index = 1;
inline constexpr int kJoint3Index = 2;

using PoseMatRT =
    Eigen::Matrix<real_T, kPoseRowCount, kPoseColCount, Eigen::ColMajor>;
using PoseIsoRT = Eigen::Transform<real_T, 3, Eigen::Isometry>;
using PoseMapConstRT = Eigen::Map<const PoseMatRT>;
using PoseMapRT = Eigen::Map<PoseMatRT>;

using Vec6 = Eigen::Matrix<double, kDofCount, 1>;
using Vec6r = Eigen::Matrix<real_T, kDofCount, 1>;
using Mat3 = Eigen::Matrix3d;
using Vec3 = Eigen::Vector3d;

enum class CrxWristConfig : int {
  Unknown = -1,
  NonFlip = 0,
  Flip = 1,
};

enum class CrxElbowConfig : int {
  Unknown = -1,
  Up = 0,
  Down = 1,
};

enum class CrxShoulderConfig : int {
  Unknown = -1,
  Top = 0,
  Bottom = 1,
};

struct CrxConfiguration {
  CrxWristConfig wrist = CrxWristConfig::Unknown;
  CrxElbowConfig elbow = CrxElbowConfig::Unknown;
  CrxShoulderConfig shoulder = CrxShoulderConfig::Unknown;
};

struct CrxIkSolution {
  Vec6 user_joints_rad = Vec6::Zero();
  CrxConfiguration config{};
};

struct DhRow {
  double alpha_rad = 0.0;
  double a = 0.0;
  double theta0_rad = 0.0;
  double d = 0.0;
  bool is_prismatic = false;
};

struct CrxModelData {
  PoseIsoRT base_transform = PoseIsoRT::Identity();
  PoseIsoRT tool_transform = PoseIsoRT::Identity();
  std::array<DhRow, kDofCount> dh_rows{};
  std::array<double, kDofCount> joint_senses{};
  Vec6 lower_limits_rad = Vec6::Zero();
  Vec6 upper_limits_rad = Vec6::Zero();
};

} // namespace crx
