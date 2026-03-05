#include <algorithm>
#include <vector>

#include "crx_kinematics.h"
#include "crx_pose_helpers.h"
#include "crx_robodk_adapter.h"
#include "crx_solver.h"
#include "crx_vector_helpers.h"

namespace {

static auto SolveFkApi(const real_T *joints,
                       real_T pose[crx::kPoseElementCount], real_T *joint_poses,
                       int max_poses, bool check_limits,
                       const robot_T *ptr_robot) -> int {
  if (joints == nullptr || pose == nullptr || ptr_robot == nullptr)
    return -1;
  if (crx::RoboDkDofCount(ptr_robot) != crx::kDofCount)
    return -1;

  crx::CrxModelData model;
  if (!crx::BuildModelFromRoboDkRobot(ptr_robot, model))
    return -1;

  crx::Vec6 joints_rad = crx::Vec6::Zero();
  if (!crx::RoboDkJointsDegCoupledToUserRad(joints, joints_rad))
    return -1;

  crx::PoseIsoRT fk_pose = crx::PoseIsoRT::Identity();
  std::vector<crx::PoseIsoRT> joint_pose_isometries;
  std::vector<crx::PoseIsoRT> *joint_pose_ptr = nullptr;
  if (joint_poses != nullptr) {
    if (max_poses < crx::kDofCount + 1)
      return -1;
    joint_pose_isometries.resize(crx::kDofCount + 1,
                                 crx::PoseIsoRT::Identity());
    joint_pose_ptr = &joint_pose_isometries;
  }

  const int status = crx::SolveFkIsometry(model, joints_rad, fk_pose,
                                          joint_pose_ptr, check_limits);
  if (status != 1)
    return status;

  crx::IsometryToPoseArray(fk_pose, pose);
  if (joint_poses != nullptr) {
    for (int pose_idx = 0; pose_idx < crx::kDofCount + 1; ++pose_idx) {
      crx::IsometryToPoseArray((*joint_pose_ptr)[pose_idx],
                               joint_poses + pose_idx * crx::kPoseElementCount);
    }
  }
  return 1;
}

} // namespace

extern "C" {

auto SolveFK(const real_T *joints, real_T pose[16], const robot_T *ptr_robot)
    -> int {
  return SolveFkApi(joints, pose, nullptr, 0, true, ptr_robot);
}

auto SolveFK_CAD(const real_T *joints, real_T pose[16], real_T *joint_poses,
                 int max_poses, const robot_T *ptr_robot) -> int {
  return SolveFkApi(joints, pose, joint_poses, max_poses, false, ptr_robot);
}

auto SolveIK(const real_T pose[16], real_T *joints, real_T *joints_all,
             int max_solutions, const real_T *joints_approx,
             const robot_T *ptr_robot) -> int {
  if (pose == nullptr || joints == nullptr || ptr_robot == nullptr)
    return -1;
  if (crx::RoboDkDofCount(ptr_robot) != crx::kDofCount)
    return -1;
  if (max_solutions <= 0)
    return 0;

  crx::CrxModelData model;
  if (!crx::BuildModelFromRoboDkRobot(ptr_robot, model))
    return -1;

  const crx::PoseIsoRT target_pose = crx::PoseArrayToIsometry(pose);
  crx::Vec6 approx_joints_rad = crx::Vec6::Zero();
  const crx::Vec6 *approx_joints_ptr = nullptr;
  if (joints_approx != nullptr) {
    if (!crx::RoboDkJointsDegCoupledToUserRad(joints_approx, approx_joints_rad))
      return -1;
    crx::NormalizeVecKeepSignedPi(approx_joints_rad);
    approx_joints_ptr = &approx_joints_rad;
  }

  std::vector<crx::Vec6> ranked_solutions_rad;
  const int solution_count =
      crx::SolveIkIsometry(model, target_pose, approx_joints_ptr, max_solutions,
                           ranked_solutions_rad);
  if (solution_count <= 0)
    return solution_count;

  if (!crx::UserJointsRadToRoboDkCoupledDeg(ranked_solutions_rad[0], joints))
    return -1;
  if (joints_all != nullptr) {
    for (int solution_idx = 0; solution_idx < solution_count; ++solution_idx) {
      real_T *solution_slot = joints_all + crx::kSolutionStride * solution_idx;
      std::fill(solution_slot, solution_slot + crx::kSolutionStride,
                static_cast<real_T>(0.0));
      if (!crx::UserJointsRadToRoboDkCoupledDeg(
              ranked_solutions_rad[solution_idx], solution_slot)) {
        return -1;
      }
    }
  }

  return solution_count;
}

} // extern "C"
