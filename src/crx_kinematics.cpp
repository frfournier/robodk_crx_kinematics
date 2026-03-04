/****************************************************************************
**
** Fanuc CRX (CRX-10iA family) — Custom Inverse Kinematics for RoboDK
**
** Overview
** --------
** This module provides a deterministic inverse-kinematics (IK) solver for the
** FANUC CRX collaborative robot family (6R serial arm with a *non-spherical
** wrist*). The solver is intended to be compiled as a RoboDK custom kinematics
** library, following the Plug-In-Interface "robotextensions/samplekinematics"
** contract (SolveFK/SolveIK entry points and RoboDK’s robot_T parameter
*layout).
**
** Algorithmic basis
** -----------------
** The IK method follows the fully geometric approach described by:
**
**   M. Abbes, G. Poisson, “Geometric Approach for Inverse Kinematics of the
**   FANUC CRX Collaborative Robot”, Robotics 13(6):91, 2024.
**   DOI: https://doi.org/10.3390/robotics13060091
**
** Key idea: reduce the 6-DoF IK to a 1-D root-finding problem over a scalar
** function derived from geometric constraints of the CRX architecture. The
** solver enumerates all valid postures (typically 8 / 12 / 16 solutions, when
** they exist) and remains robust near singularities by validating candidate
** solutions against FK consistency checks.
**
** Reference implementation & provenance
** -------------------------------------
** This RoboDK-oriented implementation is inspired by the open-source CRX FK/IK
** package by Daniel Cranston (MIT):
**
**   https://github.com/danielcranston/crx_kinematics
**
** Integration target (RoboDK custom kinematics sample):
**
**
*https://github.com/RoboDK/Plug-In-Interface/tree/master/robotextensions/samplekinematics
**
** Conventions & units (RoboDK integration notes)
** ----------------------------------------------
** - Poses are expressed as 4x4 homogeneous transforms (row-major in the RoboDK
**   C API), position in millimeters.
** - Joints are revolute; angles are interpreted/returned per RoboDK’s expected
**   units for custom kinematics (typically degrees at the interface boundary;
**   internal computation may use radians — keep conversions explicit).
** - Base/Tool adaptation:
**     * RoboDK provides a *robot base adaptor* and a *tool flange adaptor*
**       through robot_T. These must be applied consistently:
**         world_T_tcp = world_T_base * base_T_robot * FK(DH, q) *
*robot_T_flange * flange_T_tcp
**       and inverted accordingly for IK.
** - DH parameters are read from robot_T (as configured in the .robot model)
**   and must match the convention assumed by the solver.
** - FANUC-specific coupling used in this file:
**     * geometric J3 axis variable is (J2 + J3), not J3 alone
**     * dual posture relation follows Eq. (23) from the paper
**   (see comments near CoupledThetaRad / DualSolutionRad).
**
** Numerical behavior & validation
** -------------------------------
** - The solver enumerates candidate solutions by scanning/solving the 1-D
**   constraint and then back-substituting remaining joints.
** - Each candidate is validated by forward-kinematics reconstruction:
**     pose_error(position, orientation) <= tolerance
**   to reject spurious roots and to handle near-singular cases safely.
** - Solutions can be deduplicated by an angular threshold to remove numerical
**   duplicates created by root-finding tolerances.
**
** Safety / legal
** --------------
** - This implementation is provided “AS IS”, without warranty of any kind.
** - FANUC® and RoboDK® are trademarks of their respective owners; this project
**   is not affiliated with or endorsed by FANUC or RoboDK.
**
****************************************************************************/

#define _USE_MATH_DEFINES
#include <algorithm>
#include <array>
#include <cassert>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <numeric>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "angle_conversions_inline.h"
#include "crx_kinematics.h"

namespace {

// ──────────────────────────────────────────────────────────────────────────────
// Constants (prefer constexpr over macros)
// ──────────────────────────────────────────────────────────────────────────────

static constexpr double kTwoPi = 2.0 * M_PI;
static constexpr double kHalfPi = 0.5 * M_PI;
static constexpr double kEpsilon = 1e-12;

static constexpr int kPoseRowCount = 4;
static constexpr int kPoseColCount = 4;
static constexpr int kPoseElementCount = 16;

static constexpr int kDofCount = 6;

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

static constexpr int kDhArgCount = 4;
// Each solution slot: kDofCount joint values + kDofCount reserved words = 12.
static constexpr int kSolutionStride = 12;
static_assert(kSolutionStride == 2 * kDofCount,
              "kSolutionStride must be 2 * kDofCount");

static constexpr int kJoint2Index = 1;
static constexpr int kJoint3Index = 2;
static constexpr int kDhPrismaticFlagIndex = 4;
static constexpr int kDhThetaIndex = 2;
static constexpr int kDhDIndex = 3;

static constexpr int kIkQSamples = 1440;
static constexpr int kRefineMaxIterations = 100;
static constexpr double kRefineXTolerance = 1e-12;
static constexpr double kRootZeroTolerance = 1e-14;
static constexpr double kCosineClampTolerance = static_cast<double>(1.0e-9);

static constexpr double kSolutionAngleToleranceDeg = 1e-3;
static constexpr double kSolutionAngleToleranceRad =
    angle_conv::DegToRad(kSolutionAngleToleranceDeg);
static constexpr double kSolutionPositionToleranceMm = 1e-4;
static constexpr double kSolutionPositionToleranceMmSquared =
    kSolutionPositionToleranceMm * kSolutionPositionToleranceMm;

static constexpr double kJointLimitToleranceDeg = 1e-2;
static constexpr double kJointLimitToleranceRad =
    angle_conv::DegToRad(kJointLimitToleranceDeg);

static constexpr double kRightAngleSnapToleranceRad = 1e-5;
static constexpr double kDhConventionToleranceRad = 1e-5;
static constexpr double kTriangleNegativeTolerance = 1e-9;

static constexpr std::size_t kMaxIkSolutions = 32;

// ──────────────────────────────────────────────────────────────────────────────
// robot_T flat-array accessors
// ──────────────────────────────────────────────────────────────────────────────

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
static inline auto iRobot_DHM_JointId(const robot_T *r, int joint)
    -> const real_T * {
  return iRobot_At(r, kRobotDhBaseRow + joint);
}

static inline auto iRobot_nDOFs(const robot_T *r) -> int {
  return static_cast<int>(*iRobot_At(r, kRobotDofRow, kRobotDofCol));
}

// ──────────────────────────────────────────────────────────────────────────────
// Eigen type aliases
// ──────────────────────────────────────────────────────────────────────────────

using PoseMatRT =
    Eigen::Matrix<real_T, kPoseRowCount, kPoseColCount, Eigen::ColMajor>;
using PoseIsoRT = Eigen::Transform<real_T, 3, Eigen::Isometry>;
using PoseMapConstRT = Eigen::Map<const PoseMatRT>;
using PoseMapRT = Eigen::Map<PoseMatRT>;

using Vec6 = Eigen::Matrix<double, kDofCount, 1>;
using Vec6r = Eigen::Matrix<real_T, kDofCount, 1>;
using Mat3 = Eigen::Matrix3d;
using Vec3 = Eigen::Vector3d;

// ──────────────────────────────────────────────────────────────────────────────
// Pose helpers
// ──────────────────────────────────────────────────────────────────────────────

static inline auto PoseArrayToIsometry(const real_T pose[kPoseElementCount])
    -> PoseIsoRT {
  PoseIsoRT out;
  out.matrix() = PoseMapConstRT(pose);
  return out;
}

// FIX: avoid `PoseMapRT(pose) = ...` (most-vexing parse / parameter redeclare)
static inline void IsometryToPoseArray(const PoseIsoRT &iso,
                                       real_T pose[kPoseElementCount]) {
  PoseMapRT pose_map(pose);
  pose_map = iso.matrix();
}

// FANUC/RoboDK WPR: Rz(R)*Ry(P)*Rx(W), translation mm, angles rad.
static inline auto XYZWPR_ToIsometry(const real_T xyzwpr[6]) -> PoseIsoRT {
  using AA = Eigen::AngleAxis<real_T>;
  PoseIsoRT T = PoseIsoRT::Identity();
  T.linear() = (AA(xyzwpr[5], Eigen::Matrix<real_T, 3, 1>::UnitZ()) *
                AA(xyzwpr[4], Eigen::Matrix<real_T, 3, 1>::UnitY()) *
                AA(xyzwpr[3], Eigen::Matrix<real_T, 3, 1>::UnitX()))
                   .toRotationMatrix();
  T.translation() << xyzwpr[0], xyzwpr[1], xyzwpr[2];
  return T;
}

static inline auto DHM_FromRad(real_T alpha, real_T a, real_T theta, real_T d)
    -> PoseIsoRT {
  const real_T ca = std::cos(alpha), sa = std::sin(alpha);
  const real_T ct = std::cos(theta), st = std::sin(theta);
  PoseIsoRT T = PoseIsoRT::Identity();
  T.linear() << ct, -st, 0.0, st * ca, ct * ca, -sa, st * sa, ct * sa, ca;
  T.translation() << a, -d * sa, d * ca;
  return T;
}

// FK path consumes RoboDK's user-configurable tool transform from robot_T, so
// the built-in J6->tool adapter is identity here.
static inline auto FixedJ6ToToolIsometryFk() -> PoseIsoRT {
  return PoseIsoRT::Identity();
}

static inline auto FixedJ6ToToolIsometryAnalytic() -> PoseIsoRT {
  PoseIsoRT T = PoseIsoRT::Identity();
  T.linear() << 1.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, -1.0;
  return T;
}

static inline void XYZWPR_2_Pose(const real_T xyzwpr[kDofCount],
                                 real_T pose[kPoseElementCount]) {
  IsometryToPoseArray(XYZWPR_ToIsometry(xyzwpr), pose);
}

static inline void Pose_Mult(const real_T A[kPoseElementCount],
                             const real_T B[kPoseElementCount],
                             real_T out[kPoseElementCount]) noexcept {
  // tmp is load-bearing: out may alias A or B (e.g. FK loop)
  const PoseMatRT tmp = PoseMapConstRT(A) * PoseMapConstRT(B);
  PoseMapRT out_map(out);
  out_map = tmp;
}
// ──────────────────────────────────────────────────────────────────────────────
// Angle utilities — all radians
// ──────────────────────────────────────────────────────────────────────────────

static inline auto WrapRad2Pi(double a) -> double {
  double w = std::fmod(a, kTwoPi);
  return (w < 0.0) ? w + kTwoPi : w;
}
static inline auto NormalizeRadKeepSignedPi(double a) -> double {
  // O(1): map to [-pi, pi] and preserve signed pi for odd multiples.
  double w = std::fmod(a, kTwoPi);
  if (w > M_PI)
    w -= kTwoPi;
  else if (w < -M_PI)
    w += kTwoPi;
  return w;
}
static inline auto WrapRadPi(double a) -> double {
  return NormalizeRadKeepSignedPi(a);
}

static inline auto ClampCosineNearUnit(double value) -> double {
  if (!std::isfinite(value))
    return value;
  if (value > 1.0)
    return (value <= 1.0 + kCosineClampTolerance) ? 1.0 : value;
  if (value < -1.0)
    return (value >= -1.0 - kCosineClampTolerance) ? -1.0 : value;
  return value;
}
static inline void NormalizeVecKeepSignedPi(Vec6 &q) {
  q = q.unaryExpr([](double x) { return NormalizeRadKeepSignedPi(x); });
}
static inline void NormalizeUserSolutionDomains(Vec6 &q) {
  for (int i = 0; i < kDofCount; ++i) {
    if (i == kJoint3Index)
      continue;
    q[i] = NormalizeRadKeepSignedPi(q[i]);
  }
}

static inline auto
SnapToRightAngleFamily(double a, double tol = kRightAngleSnapToleranceRad)
    -> double {
  static const std::array<double, 5> refs = {-M_PI, -kHalfPi, 0.0, kHalfPi,
                                             M_PI};
  for (double r : refs)
    if (std::abs(WrapRadPi(a - r)) <= tol)
      return r;
  return a;
}
static inline auto AngleDiffAbs(double a, double b) -> double {
  return std::abs(WrapRadPi(a - b));
}

// ──────────────────────────────────────────────────────────────────────────────
// Public-boundary unit conversions (ONLY deg<->rad callsites)
// ──────────────────────────────────────────────────────────────────────────────

static inline void DegArrayToRadVec(const real_T *j_deg, Vec6 &out_rad) {
  out_rad = Eigen::Map<const Vec6>(j_deg) * angle_conv::DegToRad(1.0);
}

// FIX: avoid `Eigen::Map<Vec6r>(out) = expr;` (most-vexing parse)
static inline void RadVecToDegArray(const Vec6 &v, real_T *out) {
  Eigen::Map<Vec6r> out_map(out);
  out_map = (v * angle_conv::RadToDeg(1.0)).template cast<real_T>();
}

// ──────────────────────────────────────────────────────────────────────────────
// Joint limits and senses
// ──────────────────────────────────────────────────────────────────────────────

static inline void ReadJointLimitsRad(const robot_T *r, Vec6 &lo_rad,
                                      Vec6 &hi_rad) {
  const double k = angle_conv::DegToRad(1.0);
  const real_T *lo = iRobot_JointLimLower(r);
  const real_T *hi = iRobot_JointLimUpper(r);
  for (int i = 0; i < kDofCount; ++i) {
    lo_rad[i] = static_cast<double>(lo[i]) * k;
    hi_rad[i] = static_cast<double>(hi[i]) * k;
  }
}

static inline auto NormalizeJointSense(real_T s) -> double {
  if (!std::isfinite(s) || std::abs(s) <= kEpsilon)
    return 1.0;
  return (s >= 0.0) ? 1.0 : -1.0;
}
static inline auto ReadNormalizedJointSenses(const robot_T *r)
    -> std::array<double, kDofCount> {
  const real_T *raw = iRobot_JointSenses(r);
  std::array<double, kDofCount> out{};
  for (int i = 0; i < kDofCount; ++i)
    out[i] = NormalizeJointSense(raw[i]);
  return out;
}

static inline auto ClampToLimits(Vec6 &q, const Vec6 &lo, const Vec6 &hi,
                                 double tol_rad) -> bool {
  for (int i = 0; i < kDofCount; ++i) {
    if (q[i] < lo[i] - tol_rad || q[i] > hi[i] + tol_rad)
      return false;
    q[i] = std::min(hi[i], std::max(lo[i], q[i]));
  }
  return true;
}

// ──────────────────────────────────────────────────────────────────────────────
// Vec6 distance helpers (rad domain)
// ──────────────────────────────────────────────────────────────────────────────

static inline auto WrappedDist2Rad(const Vec6 &a, const Vec6 &b) -> double {
  double acc = 0.0;
  for (int i = 0; i < kDofCount; ++i) {
    const double d = WrapRadPi(a[i] - b[i]);
    acc += d * d;
  }
  return acc;
}
static inline auto WrappedMaxAbsDiffRad(const Vec6 &a, const Vec6 &b)
    -> double {
  double mx = 0.0;
  for (int i = 0; i < kDofCount; ++i)
    mx = std::max(mx, std::abs(WrapRadPi(a[i] - b[i])));
  return mx;
}
static inline auto MaxAbsDiffRadDirect(const Vec6 &a, const Vec6 &b) -> double {
  return (a - b).cwiseAbs().maxCoeff();
}

// ──────────────────────────────────────────────────────────────────────────────
// Solution storage — rad→deg conversion at write time only
// ──────────────────────────────────────────────────────────────────────────────

static inline void StoreSolution(real_T *all_solutions_deg, int solution_id,
                                 const Vec6 &solution_user_rad) {
  real_T *solution_slot = all_solutions_deg + kSolutionStride * solution_id;
  std::fill(solution_slot, solution_slot + kSolutionStride,
            static_cast<real_T>(0.0));
  RadVecToDegArray(solution_user_rad, solution_slot);
}

// ──────────────────────────────────────────────────────────────────────────────
// FK core
// ──────────────────────────────────────────────────────────────────────────────

static inline void BuildJointPoseInputRad(const real_T *dh_row,
                                          double joint_motion_rad,
                                          real_T dh_pose_args[kDhArgCount]) {
  // Snap alpha/theta offsets to exact right-angle family before composing FK.
  // This mirrors the CRX DHm assumptions in Robotics 2024, Sec. 2.5/2.6 and
  // avoids tiny import noise from robot files drifting branch selection.
  dh_pose_args[0] = static_cast<real_T>(SnapToRightAngleFamily(dh_row[0]));
  dh_pose_args[1] = dh_row[1];
  dh_pose_args[kDhThetaIndex] =
      static_cast<real_T>(SnapToRightAngleFamily(dh_row[kDhThetaIndex]));
  dh_pose_args[kDhDIndex] = dh_row[kDhDIndex];

  if (dh_row[kDhPrismaticFlagIndex] == 0.0)
    dh_pose_args[kDhThetaIndex] += static_cast<real_T>(joint_motion_rad);
  else
    dh_pose_args[kDhDIndex] += static_cast<real_T>(joint_motion_rad);
}

static auto SolveFKCore(const real_T *joints_user_deg,
                        real_T pose_out[kPoseElementCount],
                        real_T *joint_poses_out, int max_poses,
                        bool check_limits, const robot_T *ptr_robot) -> int {
  const int dof_count = iRobot_nDOFs(ptr_robot);
  if (dof_count != kDofCount)
    return -1;
  if (joint_poses_out != nullptr && max_poses < dof_count + 1)
    return -1;

  Vec6 user_joints_rad;
  DegArrayToRadVec(joints_user_deg, user_joints_rad);

  const auto joint_senses = ReadNormalizedJointSenses(ptr_robot);

  Vec6 lower_limits_rad, upper_limits_rad;
  if (check_limits)
    ReadJointLimitsRad(ptr_robot, lower_limits_rad, upper_limits_rad);

  real_T base_pose_local[kPoseElementCount];
  real_T *base_pose = (joint_poses_out != nullptr)
                          ? (joint_poses_out + kPoseElementCount * 0)
                          : base_pose_local;

  real_T tool_pose[kPoseElementCount], j6_to_tool_pose[kPoseElementCount];
  XYZWPR_2_Pose(iRobot_BaseXYZWPR(ptr_robot), base_pose);
  XYZWPR_2_Pose(iRobot_ToolXYZWPR(ptr_robot), tool_pose);
  IsometryToPoseArray(FixedJ6ToToolIsometryFk(), j6_to_tool_pose);

  real_T *accumulated_pose = base_pose;
  real_T next_pose_buffer[kPoseElementCount];

  // FANUC CRX convention: DH joint-3 variable is coupled as (J2 + J3), not J3.
  // This is the same convention used by the paper's IK formulas (Sec. 2.6, Eq.
  // 19).
  const double sensed_joint2_rad =
      user_joints_rad[kJoint2Index] * joint_senses[kJoint2Index];

  for (int joint_id = 0; joint_id < dof_count; ++joint_id) {
    const double sensed_joint_rad =
        user_joints_rad[joint_id] * joint_senses[joint_id];

    if (check_limits && (sensed_joint_rad < lower_limits_rad[joint_id] ||
                         sensed_joint_rad > upper_limits_rad[joint_id]))
      return -2;

    double joint_model_rad = sensed_joint_rad;
    if (joint_id == kJoint3Index)
      joint_model_rad = -sensed_joint2_rad + sensed_joint_rad;

    real_T dh_pose_args[kDhArgCount];
    BuildJointPoseInputRad(iRobot_DHM_JointId(ptr_robot, joint_id),
                           joint_model_rad, dh_pose_args);

    real_T joint_pose[kPoseElementCount];
    IsometryToPoseArray(DHM_FromRad(dh_pose_args[0], dh_pose_args[1],
                                    dh_pose_args[2], dh_pose_args[3]),
                        joint_pose);

    real_T *next_pose_ptr =
        (joint_poses_out != nullptr)
            ? (joint_poses_out + kPoseElementCount * (joint_id + 1))
            : next_pose_buffer;
    Pose_Mult(accumulated_pose, joint_pose, next_pose_ptr);
    accumulated_pose = next_pose_ptr;
  }

  real_T flange_pose[kPoseElementCount];
  Pose_Mult(accumulated_pose, j6_to_tool_pose, flange_pose);
  Pose_Mult(flange_pose, tool_pose, pose_out);
  return 1;
}

static auto IsFkRoundtripValid(const Vec6 &candidate_user_joints_rad,
                               const PoseIsoRT &target_pose,
                               const Eigen::Quaterniond &target_quat,
                               const robot_T *ptr_robot) -> bool {
  real_T candidate_user_joints_deg[kDofCount];
  real_T fk_pose_array[kPoseElementCount];
  RadVecToDegArray(candidate_user_joints_rad, candidate_user_joints_deg);
  if (SolveFKCore(candidate_user_joints_deg, fk_pose_array, nullptr, 0, false,
                  ptr_robot) != 1)
    return false;

  const PoseIsoRT fk_pose_isometry = PoseArrayToIsometry(fk_pose_array);
  const Vec3 position_error = fk_pose_isometry.translation().cast<double>() -
                              target_pose.translation().cast<double>();
  if (!std::isfinite(position_error.squaredNorm()) ||
      position_error.squaredNorm() > kSolutionPositionToleranceMmSquared)
    return false;

  const Eigen::Quaterniond fk_quaternion(
      fk_pose_isometry.linear().cast<double>());
  const double orientation_error_rad =
      fk_quaternion.angularDistance(target_quat);
  return std::isfinite(orientation_error_rad) &&
         orientation_error_rad <= kSolutionAngleToleranceRad;
}

// ──────────────────────────────────────────────────────────────────────────────
// IK — analytic CRX
//
// Paper map (robotics13060091.xhtml, Sec. 2.6):
// - Step 1 / Eq. (13): target pose to O6/O5                  -> SolveIK +
// circle_context.o5_point
// - Step 2 / Eq. (14): O4(q) circle parametrization           ->
// EvaluateCircle(_cs)
// - Step 3 / Eq. (15): O3_UP/O3_DW from triangle constraints  ->
// FindThirdTriangleCorner
// - Step 4/5: UP(q), DW(q) and zero search                    ->
// up_dot_product/down_dot_product + bisection
// - Step 6 / Eq. (16..22): recover J1..J6                     ->
// DetermineJointValues
// - Step 7 / Eq. (23): dual posture transform                 ->
// DualSolutionRad
// ──────────────────────────────────────────────────────────────────────────────

struct CrxParams {
  std::array<double, kDofCount> alpha_rad{};
  std::array<double, kDofCount> a{};
  std::array<double, kDofCount> d{};
  std::array<double, kDofCount> theta0_rad{};
  double a2 = 0.0, r4 = 0.0, r5 = 0.0, r6 = 0.0;
};

static auto ReadCrxParams(const robot_T *robot, CrxParams &params) -> bool {
  for (int joint_id = 0; joint_id < kDofCount; ++joint_id) {
    const real_T *dh_row = iRobot_DHM_JointId(robot, joint_id);
    if (dh_row[4] != 0.0)
      return false; // prismatic not supported
    params.alpha_rad[joint_id] = SnapToRightAngleFamily(dh_row[0]);
    params.a[joint_id] = dh_row[1];
    params.theta0_rad[joint_id] = SnapToRightAngleFamily(dh_row[2]);
    params.d[joint_id] = dh_row[3];
  }
  params.a2 = params.a[2];
  params.r4 = params.d[3];
  params.r5 = params.d[4];
  params.r6 = params.d[5];
  return true;
}

static auto ConvertRoboDkDhToAnalyticIkConvention(CrxParams &params,
                                                  double &base_z_shift_mm)
    -> bool {
  // RoboDK stores the production FK model convention. The geometric IK in
  // Sec. 2.6 assumes a normalized internal convention; this adapter validates
  // expected CRX DH signatures and remaps only if they match exactly.
  base_z_shift_mm = 0.0;
  const double tol = kDhConventionToleranceRad;

  const bool alpha_ok = AngleDiffAbs(params.alpha_rad[0], 0.0) < tol &&
                        AngleDiffAbs(params.alpha_rad[1], -kHalfPi) < tol &&
                        AngleDiffAbs(params.alpha_rad[2], 0.0) < tol &&
                        AngleDiffAbs(params.alpha_rad[3], -kHalfPi) < tol &&
                        AngleDiffAbs(params.alpha_rad[4], kHalfPi) < tol &&
                        AngleDiffAbs(params.alpha_rad[5], -kHalfPi) < tol;

  const bool theta_ok = AngleDiffAbs(params.theta0_rad[0], 0.0) < tol &&
                        AngleDiffAbs(params.theta0_rad[1], -kHalfPi) < tol &&
                        AngleDiffAbs(params.theta0_rad[2], 0.0) < tol &&
                        AngleDiffAbs(params.theta0_rad[3], 0.0) < tol &&
                        AngleDiffAbs(params.theta0_rad[4], 0.0) < tol &&
                        AngleDiffAbs(params.theta0_rad[5], 0.0) < tol;

  const bool a_ok =
      std::abs(params.a[0]) <= kEpsilon && std::abs(params.a[1]) <= kEpsilon &&
      params.a[2] > 0.0 && std::abs(params.a[3]) <= kEpsilon &&
      std::abs(params.a[4]) <= kEpsilon && std::abs(params.a[5]) <= kEpsilon;

  const bool d_ok = params.d[0] > 0.0 && std::abs(params.d[1]) <= kEpsilon &&
                    std::abs(params.d[2]) <= kEpsilon && params.d[3] > 0.0 &&
                    params.d[4] <= -kEpsilon && // Pass-1 Fix #2
                    params.d[5] > 0.0;

  // Fail closed when a model differs from the CRX assumptions:
  // returning -1 asks RoboDK to use its generic numerical IK.
  if (!(alpha_ok && theta_ok && a_ok && d_ok))
    return false;

  // The next remaps align RoboDK FK convention with the paper's internal frame
  // setup. base_z_shift_mm is undone on T06 in SolveIK to keep external
  // behavior unchanged.
  base_z_shift_mm = params.d[0];
  params.d[0] = 0.0;
  params.alpha_rad[2] = M_PI;
  params.d[3] = -params.d[3];
  params.d[4] = -params.d[4];
  params.d[5] = -params.d[5];
  params.a2 = params.a[2];
  params.r4 = params.d[3];
  params.r5 = params.d[4];
  params.r6 = params.d[5];
  return true;
}

static inline auto CoupledThetaRad(const Vec6 &solver_joints_rad,
                                   const CrxParams &params)
    -> std::array<double, kDofCount> {
  // Sec. 2.6 uses the CRX-specific coupled third axis variable (J2 + J3).
  // Keeping this in one helper prevents accidental uncoupled edits elsewhere.
  return {solver_joints_rad[0] + params.theta0_rad[0],
          solver_joints_rad[1] + params.theta0_rad[1],
          solver_joints_rad[1] + solver_joints_rad[2] + params.theta0_rad[2],
          solver_joints_rad[3] + params.theta0_rad[3],
          solver_joints_rad[4] + params.theta0_rad[4],
          solver_joints_rad[5] + params.theta0_rad[5]};
}

static inline auto ForwardFromSolverJoints(const Vec6 &solver_joints_rad,
                                           const CrxParams &params)
    -> PoseIsoRT {
  const auto coupled_theta_rad = CoupledThetaRad(solver_joints_rad, params);
  PoseIsoRT transform = PoseIsoRT::Identity();
  for (int joint_id = 0; joint_id < kDofCount; ++joint_id)
    transform = transform *
                DHM_FromRad(static_cast<real_T>(params.alpha_rad[joint_id]),
                            static_cast<real_T>(params.a[joint_id]),
                            static_cast<real_T>(coupled_theta_rad[joint_id]),
                            static_cast<real_T>(params.d[joint_id]));
  return transform * FixedJ6ToToolIsometryAnalytic();
}

static inline auto IsPoseConsistent(const Vec6 &solver_joints_rad,
                                    const PoseIsoRT &target_pose,
                                    const CrxParams &params) -> bool {
  const PoseIsoRT fk_pose = ForwardFromSolverJoints(solver_joints_rad, params);
  const Vec3 position_error = fk_pose.translation().cast<double>() -
                              target_pose.translation().cast<double>();
  if (!std::isfinite(position_error.squaredNorm()) ||
      position_error.squaredNorm() > kSolutionPositionToleranceMmSquared)
    return false;
  const double orientation_error_rad =
      Eigen::Quaterniond(fk_pose.linear().cast<double>())
          .angularDistance(
              Eigen::Quaterniond(target_pose.linear().cast<double>()));
  return std::isfinite(orientation_error_rad) &&
         orientation_error_rad <= kSolutionAngleToleranceRad;
}

// Specialized hot-path basis build for ConstructPlane(O4, UnitZ()).
static inline auto ConstructPlane_O4_UnitZ(const Vec3 &O4, Mat3 &R_plane)
    -> bool {
  constexpr double kPlaneEps2 = 1e-18;

  const double o4_norm_sq = O4.squaredNorm();
  if (!std::isfinite(o4_norm_sq) || o4_norm_sq <= kEpsilon * kEpsilon)
    return false;

  const Vec3 plane_x = O4 * (1.0 / std::sqrt(o4_norm_sq));

  // Prefer y close to projected +Z so q progression around the O4 circle stays
  // smooth most of the time; fall back near collinearity to avoid NaNs.
  Vec3 plane_y = Vec3::UnitZ() - plane_x.z() * plane_x;
  double plane_y_norm_sq = plane_y.squaredNorm();
  if (!std::isfinite(plane_y_norm_sq) || plane_y_norm_sq <= kPlaneEps2) {
    plane_y = plane_x.unitOrthogonal();
    plane_y_norm_sq = plane_y.squaredNorm();
    if (!std::isfinite(plane_y_norm_sq) || plane_y_norm_sq <= kPlaneEps2)
      return false;
  } else {
    plane_y *= (1.0 / std::sqrt(plane_y_norm_sq));
  }

  Vec3 plane_z = plane_x.cross(plane_y);
  const double plane_z_norm_sq = plane_z.squaredNorm();
  if (!std::isfinite(plane_z_norm_sq) || plane_z_norm_sq <= kPlaneEps2)
    return false;
  plane_z *= (1.0 / std::sqrt(plane_z_norm_sq));

  // Defensive re-orthogonalization against numeric drift.
  plane_y = plane_z.cross(plane_x);

  R_plane.col(0) = plane_x;
  R_plane.col(1) = plane_y;
  R_plane.col(2) = plane_z;

#ifndef NDEBUG
  assert(std::abs(R_plane.determinant() - 1.0) < 1e-9 &&
         "ConstructPlane_O4_UnitZ: R_plane is not a rotation matrix");
#endif
  return true;
}

static inline auto FindThirdTriangleCorner(double side_ab, double side_ac,
                                           double side_bc, double &x_coord,
                                           double &y_abs) -> bool {
  if (side_ab <= kEpsilon)
    return false;
  x_coord = (side_ac * side_ac - side_bc * side_bc + side_ab * side_ab) /
            (2.0 * side_ab);
  const double y_squared = side_ac * side_ac - x_coord * x_coord;
  if (y_squared < -kTriangleNegativeTolerance)
    return false;
  y_abs = std::sqrt(std::max(0.0, y_squared));
  return true;
}

static inline auto JointTransformRad(const CrxParams &params, int joint_id,
                                     double joint_angle_rad) -> PoseIsoRT {
  return DHM_FromRad(
      static_cast<real_T>(params.alpha_rad[joint_id]),
      static_cast<real_T>(params.a[joint_id]),
      static_cast<real_T>(params.theta0_rad[joint_id] + joint_angle_rad),
      static_cast<real_T>(params.d[joint_id]));
}

static auto DetermineJointValues(const Vec3 &o3_point, const Vec3 &o4_point,
                                 const Vec3 &o5_point,
                                 const PoseIsoRT &target_pose_06,
                                 const CrxParams &params, Vec6 &out_joints_rad)
    -> bool {
  // This back-substitution corresponds to the paper's Step 6 (Eq. 16..22):
  // once a valid O3/O4 pair is known, solve J1..J6 from chained frame
  // reductions.
  const Vec3 o6_point = target_pose_06.translation().cast<double>();

  const double J1 = std::atan2(o4_point.y(), o4_point.x());
  const Mat3 rotation_l1_from_l0 =
      JointTransformRad(params, 0, J1).inverse().linear().cast<double>();

  const Vec3 o3_in_l1 = rotation_l1_from_l0 * o3_point;
  const double J2 = std::atan2(o3_in_l1.x(), o3_in_l1.z());

  const Vec3 o4_in_l1 = rotation_l1_from_l0 * o4_point;
  const double J3 =
      std::atan2(o4_in_l1.z() - o3_in_l1.z(), o4_in_l1.x() - o3_in_l1.x());

  const PoseIsoRT transform_l2_from_l1 =
      JointTransformRad(params, 1, J2).inverse();
  // Joint 3 uses the coupled variable (J2 + J3) in the DH chain.
  const PoseIsoRT transform_l3_from_l2 =
      JointTransformRad(params, 2, J2 + J3).inverse();
  const PoseIsoRT transform_l3_from_l0 = transform_l3_from_l2 *
                                         transform_l2_from_l1 *
                                         PoseIsoRT(rotation_l1_from_l0);

  const Vec3 o5_in_l3 = (transform_l3_from_l0 * o5_point.cast<real_T>())
                            .template head<3>()
                            .cast<double>();
  const double J4 = std::atan2(o5_in_l3.x(), o5_in_l3.z());

  const PoseIsoRT transform_l4_from_l0 =
      JointTransformRad(params, 3, J4).inverse() * transform_l3_from_l0;

  const Vec3 o6_in_l4 = (transform_l4_from_l0 * o6_point.cast<real_T>())
                            .template head<3>()
                            .cast<double>();
  const double J5 = std::atan2(o6_in_l4.x(), -o6_in_l4.z());

  const PoseIsoRT transform_l5_from_l0 =
      JointTransformRad(params, 4, J5).inverse() * transform_l4_from_l0;
  const Vec3 tool_x_axis_in_l5 =
      (transform_l5_from_l0.linear() * target_pose_06.linear())
          .cast<double>()
          .col(0);
  const double J6 = std::atan2(-tool_x_axis_in_l5.z(), tool_x_axis_in_l5.x());

  if (!(std::isfinite(J1) && std::isfinite(J2) && std::isfinite(J3) &&
        std::isfinite(J4) && std::isfinite(J5) && std::isfinite(J6)))
    return false;

  out_joints_rad << NormalizeRadKeepSignedPi(J1), NormalizeRadKeepSignedPi(J2),
      NormalizeRadKeepSignedPi(J3), NormalizeRadKeepSignedPi(J4),
      NormalizeRadKeepSignedPi(J5), NormalizeRadKeepSignedPi(J6);

  return true;
}

// ─────────────────────────── Circle sweep ───────────────────────────────────

struct CircleEvalContext {
  Mat3 target_rotation = Mat3::Identity();
  Vec3 target_translation = Vec3::Zero();
  Vec3 o5_point = Vec3::Zero();
  double min_o0_o4_dist_sq = 0.0;
  double max_o0_o4_dist_sq = 0.0;
  bool has_min_o0_o4_bound = false;
};

struct CircleEvaluation {
  Vec3 o4_point = Vec3::Zero();
  Vec3 o3_up_point = Vec3::Zero();
  Vec3 o3_down_point = Vec3::Zero();
  double up_dot_product = std::numeric_limits<double>::quiet_NaN();
  double down_dot_product = std::numeric_limits<double>::quiet_NaN();
  bool is_valid = false;
};

static auto EvaluateCircle_cs(double cos_q, double sin_q,
                              const CircleEvalContext &context,
                              const CrxParams &params,
                              CircleEvaluation &evaluation,
                              Mat3 &plane_rotation_out) -> bool {
  // Step 2/3/4 core: for one q sample, compute O4(q), both O3 branches, then
  // the two normalized dot products UP(q)/DW(q) whose zeros define valid
  // solutions.
  evaluation.is_valid = false;

  const Vec3 o4_local(params.r5 * cos_q, params.r5 * sin_q, params.r6);
  evaluation.o4_point.noalias() = context.target_rotation * o4_local;
  evaluation.o4_point += context.target_translation;

  const double o0_o4_dist_sq = evaluation.o4_point.squaredNorm();
  if (!std::isfinite(o0_o4_dist_sq) || o0_o4_dist_sq <= kEpsilon * kEpsilon ||
      o0_o4_dist_sq > context.max_o0_o4_dist_sq ||
      (context.has_min_o0_o4_bound &&
       o0_o4_dist_sq < context.min_o0_o4_dist_sq))
    return false;

  if (!ConstructPlane_O4_UnitZ(evaluation.o4_point, plane_rotation_out))
    return false;

  const double o0_o4_dist = std::sqrt(o0_o4_dist_sq);
  double triangle_x_coord = 0.0;
  double triangle_y_abs = 0.0;
  // The paper's triangle uses signed CRX lengths; -r4 preserves that
  // convention.
  if (!FindThirdTriangleCorner(o0_o4_dist, params.a2, -params.r4,
                               triangle_x_coord, triangle_y_abs))
    return false;

  evaluation.o3_up_point =
      plane_rotation_out * Vec3(triangle_x_coord, triangle_y_abs, 0.0);
  evaluation.o3_down_point =
      plane_rotation_out * Vec3(triangle_x_coord, -triangle_y_abs, 0.0);

  const Vec3 o4_to_o5_vector = context.o5_point - evaluation.o4_point;
  const double o4_to_o5_norm_sq = o4_to_o5_vector.squaredNorm();
  if (!std::isfinite(o4_to_o5_norm_sq) ||
      o4_to_o5_norm_sq <= kEpsilon * kEpsilon)
    return false;
  const double inv_o4_to_o5_norm = 1.0 / std::sqrt(o4_to_o5_norm_sq);

  const Vec3 o3_to_o4_up_vector = evaluation.o4_point - evaluation.o3_up_point;
  const Vec3 o3_to_o4_down_vector =
      evaluation.o4_point - evaluation.o3_down_point;

  const double up_vector_norm_sq = o3_to_o4_up_vector.squaredNorm();
  const double down_vector_norm_sq = o3_to_o4_down_vector.squaredNorm();
  if (!std::isfinite(up_vector_norm_sq) ||
      up_vector_norm_sq <= kEpsilon * kEpsilon)
    return false;
  if (!std::isfinite(down_vector_norm_sq) ||
      down_vector_norm_sq <= kEpsilon * kEpsilon)
    return false;

  evaluation.up_dot_product = ClampCosineNearUnit(
      o3_to_o4_up_vector.dot(o4_to_o5_vector) *
      (1.0 / std::sqrt(up_vector_norm_sq)) * inv_o4_to_o5_norm);
  evaluation.down_dot_product = ClampCosineNearUnit(
      o3_to_o4_down_vector.dot(o4_to_o5_vector) *
      (1.0 / std::sqrt(down_vector_norm_sq)) * inv_o4_to_o5_norm);
  evaluation.is_valid = std::isfinite(evaluation.up_dot_product) &&
                        std::isfinite(evaluation.down_dot_product) &&
                        std::abs(evaluation.up_dot_product) <=
                            1.0 + kCosineClampTolerance &&
                        std::abs(evaluation.down_dot_product) <=
                            1.0 + kCosineClampTolerance;
  return evaluation.is_valid;
}

static auto EvaluateCircle(double sample_q, const CircleEvalContext &context,
                           const CrxParams &params,
                           CircleEvaluation &evaluation,
                           Mat3 &plane_rotation_out) -> bool {
  const double cos_q = std::cos(sample_q);
  const double sin_q = std::sin(sample_q);
  return EvaluateCircle_cs(cos_q, sin_q, context, params, evaluation,
                           plane_rotation_out);
}

static inline auto HasBracket(double f_left, double f_right) -> bool {
  if (!(std::isfinite(f_left) && std::isfinite(f_right)))
    return false;
  if (std::abs(f_left) <= kRootZeroTolerance ||
      std::abs(f_right) <= kRootZeroTolerance)
    return true;
  return f_left * f_right < 0.0;
}

template <typename EvalFn>
static auto RefineZeroBisection(double q_left, double q_right, double f_left,
                                double f_right, EvalFn eval_fn,
                                double &root_out_q) -> bool {
  // Deliberately simple and deterministic (no Newton step): Step 5 signals can
  // flatten near singular boundaries, where bracketing is more reliable.
  if (!(std::isfinite(f_left) && std::isfinite(f_right)))
    return false;
  if (std::abs(f_left) <= kRootZeroTolerance) {
    root_out_q = WrapRad2Pi(q_left);
    return true;
  }
  if (std::abs(f_right) <= kRootZeroTolerance) {
    root_out_q = WrapRad2Pi(q_right);
    return true;
  }
  if (f_left * f_right > 0.0)
    return false;

  double left_q = q_left;
  double right_q = q_right;
  for (int iter = 0; iter < kRefineMaxIterations; ++iter) {
    const double mid_q = 0.5 * (left_q + right_q);
    const double f_mid = eval_fn(mid_q);
    if (!std::isfinite(f_mid))
      return false;
    if (std::abs(f_mid) <= kRootZeroTolerance ||
        (right_q - left_q) < kRefineXTolerance) {
      root_out_q = WrapRad2Pi(mid_q);
      return true;
    }
    if (f_left * f_mid <= 0.0) {
      right_q = mid_q;
      f_right = f_mid;
    } else {
      left_q = mid_q;
      f_left = f_mid;
    }
  }
  root_out_q = WrapRad2Pi(0.5 * (left_q + right_q));
  return true;
}

static void DedupSolutions(const std::vector<Vec6> &input_solutions,
                           std::vector<Vec6> &unique_solutions) {
  unique_solutions.clear();
  unique_solutions.reserve(
      std::min<std::size_t>(kMaxIkSolutions, input_solutions.size()));
  for (Vec6 candidate_solution : input_solutions) {
    NormalizeVecKeepSignedPi(candidate_solution);
    const bool is_duplicate = std::any_of(
        unique_solutions.begin(), unique_solutions.end(),
        [&](const Vec6 &accepted_solution) {
          return WrappedMaxAbsDiffRad(candidate_solution, accepted_solution) <=
                 kSolutionAngleToleranceRad;
        });
    if (!is_duplicate) {
      if (unique_solutions.size() >= kMaxIkSolutions)
        break;
      unique_solutions.push_back(candidate_solution);
    }
  }
}

static void DualSolutionRad(const Vec6 &source_solution,
                            Vec6 &dual_solution_out) {
  // Eq. (23) dual map (paper Sec. 2.6, Step 7), expressed in radians:
  // [J1-PI, -J2, PI-J3, J4-PI, J5, J6].
  dual_solution_out = source_solution;
  dual_solution_out[0] = source_solution[0] - M_PI;
  dual_solution_out[1] = -source_solution[1];
  dual_solution_out[2] = M_PI - source_solution[2];
  dual_solution_out[3] = source_solution[3] - M_PI;
  dual_solution_out[4] = source_solution[4];
  dual_solution_out[5] = source_solution[5];
  NormalizeUserSolutionDomains(dual_solution_out);
}

static void AddDualSolutions(const std::vector<Vec6> &primal_solutions,
                             std::vector<Vec6> &all_solutions) {
  all_solutions.clear();
  all_solutions.reserve(
      std::min<std::size_t>(kMaxIkSolutions, primal_solutions.size() * 2u));
  for (const Vec6 &primal_solution : primal_solutions) {
    if (all_solutions.size() >= kMaxIkSolutions)
      return;
    all_solutions.push_back(primal_solution);
  }
  Vec6 dual_solution = Vec6::Zero();
  for (const Vec6 &primal_solution : primal_solutions) {
    if (all_solutions.size() >= kMaxIkSolutions)
      return;
    DualSolutionRad(primal_solution, dual_solution);
    NormalizeVecKeepSignedPi(dual_solution);
    all_solutions.push_back(dual_solution);
  }
}

template <typename EmitFn>
static auto ForEachSignedPiVariant(const Vec6 &base_solution, EmitFn emit)
    -> bool {
  constexpr double kPiFlipTol = 1e-8;

  std::vector<int> pi_flip_joint_ids;
  pi_flip_joint_ids.reserve(kDofCount);
  for (int joint_id = 0; joint_id < kDofCount; ++joint_id)
    if (std::abs(std::abs(base_solution[joint_id]) - M_PI) <= kPiFlipTol)
      pi_flip_joint_ids.push_back(joint_id);

  // +PI and -PI represent the same physical angle but can map to different
  // controller branches/joint-limit edges. Emit both to keep branch choice
  // stable.
  const auto variant_count =
      static_cast<std::uint32_t>(1u << pi_flip_joint_ids.size());
  for (std::uint32_t mask = 0; mask < variant_count; ++mask) {
    Vec6 variant_solution = base_solution;
    for (std::size_t bit = 0; bit < pi_flip_joint_ids.size(); ++bit) {
      if ((mask & (1u << bit)) == 0u)
        continue;
      const int joint_id = pi_flip_joint_ids[bit];
      variant_solution[joint_id] =
          (base_solution[joint_id] >= 0.0) ? -M_PI : M_PI;
    }
    if (!emit(variant_solution))
      return false;
  }
  return true;
}

template <typename EmitFn>
static auto ForEachCandidateVariant(const Vec6 &base_solution, EmitFn emit)
    -> bool {
  if (!ForEachSignedPiVariant(base_solution,
                              [&](const Vec6 &signed_pi_variant) {
                                return emit(signed_pi_variant);
                              })) {
    return false;
  }

  Vec6 dual_solution = Vec6::Zero();
  DualSolutionRad(base_solution, dual_solution);
  return ForEachSignedPiVariant(dual_solution, [&](const Vec6 &dual_variant) {
    return emit(dual_variant);
  });
}

struct Candidate {
  Vec6 user_joints_rad = Vec6::Zero();
  double distance_sq = 0.0;
};

// ─────────────────────────── Closed-form CRX IK ─────────────────────────────

static void SolveCrxIk(const PoseIsoRT &target_pose_06, const CrxParams &params,
                       std::vector<Vec6> &solutions_out) {
  solutions_out.clear();
  solutions_out.reserve(kMaxIkSolutions);

  CircleEvalContext circle_context;
  circle_context.target_rotation = target_pose_06.linear().cast<double>();
  circle_context.target_translation =
      target_pose_06.translation().cast<double>();
  circle_context.o5_point =
      circle_context.target_rotation * Vec3(0.0, 0.0, params.r6) +
      circle_context.target_translation;

  // Step 3 inequality (Eq. 15): triangle feasibility window for d04.
  const double triangle_side_a2 = std::abs(params.a2);
  const double triangle_side_r4 = std::abs(params.r4);
  const double reach_margin =
      1e-6 * std::max(1.0, triangle_side_a2 + triangle_side_r4);
  const double max_o0_o4_dist =
      triangle_side_a2 + triangle_side_r4 + reach_margin;
  const double min_o0_o4_dist =
      std::abs(triangle_side_a2 - triangle_side_r4) - reach_margin;
  circle_context.max_o0_o4_dist_sq = max_o0_o4_dist * max_o0_o4_dist;
  circle_context.has_min_o0_o4_bound = min_o0_o4_dist > 0.0;
  circle_context.min_o0_o4_dist_sq = circle_context.has_min_o0_o4_bound
                                         ? min_o0_o4_dist * min_o0_o4_dist
                                         : 0.0;

  std::vector<Vec6> primal_solutions;
  primal_solutions.reserve(kMaxIkSolutions);

  const auto evaluate_up_dot = [&](double q_eval) -> double {
    CircleEvaluation evaluation;
    Mat3 plane_rotation;
    return EvaluateCircle(WrapRad2Pi(q_eval), circle_context, params,
                          evaluation, plane_rotation)
               ? evaluation.up_dot_product
               : std::numeric_limits<double>::quiet_NaN();
  };
  const auto evaluate_down_dot = [&](double q_eval) -> double {
    CircleEvaluation evaluation;
    Mat3 plane_rotation;
    return EvaluateCircle(WrapRad2Pi(q_eval), circle_context, params,
                          evaluation, plane_rotation)
               ? evaluation.down_dot_product
               : std::numeric_limits<double>::quiet_NaN();
  };

  Vec6 candidate_joints_rad = Vec6::Zero();
  const auto try_store_solution = [&](const Vec3 &o3_candidate,
                                      const CircleEvaluation &evaluation) {
    if (primal_solutions.size() >= kMaxIkSolutions)
      return;
    if (DetermineJointValues(o3_candidate, evaluation.o4_point,
                             circle_context.o5_point, target_pose_06, params,
                             candidate_joints_rad)) {
      NormalizeVecKeepSignedPi(candidate_joints_rad);
      primal_solutions.push_back(candidate_joints_rad);
    }
  };

  // q sampling for Step 2/4. 1440 samples = 0.25 deg granularity.
  const double sample_step_q = kTwoPi / static_cast<double>(kIkQSamples);
  const double cos_step = std::cos(sample_step_q);
  const double sin_step = std::sin(sample_step_q);

  CircleEvaluation previous_eval;
  CircleEvaluation current_eval;
  CircleEvaluation root_eval;
  Mat3 previous_plane_rotation;
  Mat3 current_plane_rotation;
  Mat3 root_plane_rotation;

  double previous_q = 0.0;
  double cos_q = 1.0;
  double sin_q = 0.0;

  bool previous_ok = EvaluateCircle_cs(cos_q, sin_q, circle_context, params,
                                       previous_eval, previous_plane_rotation);

  for (int sample_idx = 1; sample_idx <= kIkQSamples; ++sample_idx) {
    const double current_q = sample_step_q * static_cast<double>(sample_idx);

    // Trig recurrence avoids repeated sin/cos calls in the hot loop.
    const double next_cos_q = cos_q * cos_step - sin_q * sin_step;
    const double next_sin_q = sin_q * cos_step + cos_q * sin_step;
    cos_q = next_cos_q;
    sin_q = next_sin_q;

    const bool current_ok =
        EvaluateCircle_cs(cos_q, sin_q, circle_context, params, current_eval,
                          current_plane_rotation);

    if (previous_ok && current_ok) {
      if (HasBracket(previous_eval.up_dot_product,
                     current_eval.up_dot_product)) {
        double root_q = 0.0;
        if (RefineZeroBisection(
                previous_q, current_q, previous_eval.up_dot_product,
                current_eval.up_dot_product, evaluate_up_dot, root_q) &&
            EvaluateCircle(root_q, circle_context, params, root_eval,
                           root_plane_rotation)) {
          try_store_solution(root_eval.o3_up_point, root_eval);
        }
      }
      if (HasBracket(previous_eval.down_dot_product,
                     current_eval.down_dot_product)) {
        double root_q = 0.0;
        if (RefineZeroBisection(
                previous_q, current_q, previous_eval.down_dot_product,
                current_eval.down_dot_product, evaluate_down_dot, root_q) &&
            EvaluateCircle(root_q, circle_context, params, root_eval,
                           root_plane_rotation)) {
          try_store_solution(root_eval.o3_down_point, root_eval);
        }
      }
    }
    previous_q = current_q;
    previous_eval = current_eval;
    previous_ok = current_ok;

    if (primal_solutions.size() >= kMaxIkSolutions)
      break;
  }

  std::vector<Vec6> all_solutions_with_duals;
  AddDualSolutions(primal_solutions, all_solutions_with_duals);
  DedupSolutions(all_solutions_with_duals, solutions_out);
}

} // namespace

// ──────────────────────────────────────────────────────────────────────────────
// Public entry points
// ──────────────────────────────────────────────────────────────────────────────

auto SolveFK(const real_T *joints, real_T pose[16], const robot_T *ptr_robot)
    -> int {
  return SolveFKCore(joints, pose, nullptr, 0, true, ptr_robot);
}

auto SolveFK_CAD(const real_T *joints, real_T pose[16], real_T *joint_poses,
                 int max_poses, const robot_T *ptr_robot) -> int {
  return SolveFKCore(joints, pose, joint_poses, max_poses, false, ptr_robot);
}

auto SolveIK(const real_T pose[16], real_T *joints, real_T *joints_all,
             int max_solutions, const real_T *joints_approx,
             const robot_T *ptr_robot) -> int {
  // External API expects RoboDK world/base/tool conventions at boundaries.
  if (iRobot_nDOFs(ptr_robot) != kDofCount)
    return -1;
  if (max_solutions <= 0)
    return 0;

  CrxParams crx_params;
  if (!ReadCrxParams(ptr_robot, crx_params))
    return -1;

  const PoseIsoRT base_transform =
      XYZWPR_ToIsometry(iRobot_BaseXYZWPR(ptr_robot));
  const PoseIsoRT tool_transform =
      XYZWPR_ToIsometry(iRobot_ToolXYZWPR(ptr_robot));
  const PoseIsoRT target_pose = PoseArrayToIsometry(pose);
  const Eigen::Quaterniond target_quaternion(
      target_pose.linear().cast<double>());

  // Remove RoboDK base/tool adapters to work in the CRX DH base frame.
  const PoseIsoRT target_pose_06_raw =
      base_transform.inverse() * target_pose * tool_transform.inverse();

  double convention_base_z_shift_mm = 0.0;
  if (!ConvertRoboDkDhToAnalyticIkConvention(crx_params,
                                             convention_base_z_shift_mm))
    return -1;

  PoseIsoRT target_pose_06 = target_pose_06_raw;
  if (std::abs(convention_base_z_shift_mm) > kEpsilon) {
    PoseIsoRT convention_shift_transform = PoseIsoRT::Identity();
    convention_shift_transform.translation() << 0.0, 0.0,
        -convention_base_z_shift_mm;
    // Undo the convention adapter translation so SolveCrxIk sees paper-style
    // frames.
    target_pose_06 = convention_shift_transform * target_pose_06_raw;
  }

  std::vector<Vec6> geometric_solutions;
  SolveCrxIk(target_pose_06, crx_params, geometric_solutions);

  // Limits converted once in rad
  Vec6 lower_limits_rad, upper_limits_rad;
  ReadJointLimitsRad(ptr_robot, lower_limits_rad, upper_limits_rad);

  // If geometric sweep found nothing, we still allow a validated "hold current
  // branch" answer from joints_approx. This keeps motion planners stable near
  // singular borders.
  if (geometric_solutions.empty()) {
    if (joints_approx != nullptr) {
      Vec6 approx_joints_rad = Vec6::Zero();
      DegArrayToRadVec(joints_approx, approx_joints_rad);
      if (ClampToLimits(approx_joints_rad, lower_limits_rad, upper_limits_rad,
                        kJointLimitToleranceRad) &&
          IsFkRoundtripValid(approx_joints_rad, target_pose, target_quaternion,
                             ptr_robot)) {
        RadVecToDegArray(approx_joints_rad, joints);
        if (joints_all != nullptr)
          StoreSolution(joints_all, 0, approx_joints_rad);
        return 1;
      }
    }
    return 0;
  }

  // Approx converted once at entry
  Vec6 approx_joints_rad = Vec6::Zero();
  const bool has_approximate_joints = (joints_approx != nullptr);
  if (has_approximate_joints) {
    DegArrayToRadVec(joints_approx, approx_joints_rad);
    NormalizeVecKeepSignedPi(approx_joints_rad);
  }

  std::vector<Candidate> ranked_candidates;
  ranked_candidates.reserve(kMaxIkSolutions);

  auto is_direct_duplicate = [&](const Vec6 &candidate_joints_rad) {
    return std::any_of(ranked_candidates.begin(), ranked_candidates.end(),
                       [&](const Candidate &candidate) {
                         return MaxAbsDiffRadDirect(
                                    candidate_joints_rad,
                                    candidate.user_joints_rad) <=
                                kSolutionAngleToleranceRad;
                       });
  };

  for (Vec6 solution_rad : geometric_solutions) {
    NormalizeUserSolutionDomains(solution_rad);

    const bool processed_all_variants =
        ForEachCandidateVariant(solution_rad, [&](Vec6 user_variant_rad) {
          if (!ClampToLimits(user_variant_rad, lower_limits_rad,
                             upper_limits_rad, kJointLimitToleranceRad))
            return true;

          if (!IsFkRoundtripValid(user_variant_rad, target_pose,
                                  target_quaternion, ptr_robot))
            return true;

          // Keep signed-pi variants distinct for deterministic branch matching.
          if (is_direct_duplicate(user_variant_rad))
            return true;

          const double distance_sq =
              has_approximate_joints
                  ? WrappedDist2Rad(user_variant_rad, approx_joints_rad)
                  : static_cast<double>(ranked_candidates.size());

          if (ranked_candidates.size() < kMaxIkSolutions) {
            ranked_candidates.push_back(
                Candidate{user_variant_rad, distance_sq});
          } else if (has_approximate_joints) {
            // keep best-k when approx provided
            auto worst_candidate_it = std::max_element(
                ranked_candidates.begin(), ranked_candidates.end(),
                [](const Candidate &a, const Candidate &b) {
                  return a.distance_sq < b.distance_sq;
                });
            if (worst_candidate_it != ranked_candidates.end() &&
                distance_sq < worst_candidate_it->distance_sq) {
              *worst_candidate_it = Candidate{user_variant_rad, distance_sq};
            }
          } // else: preserve first-k behavior
          return true;
        });

    if (!processed_all_variants)
      break;
  }

  // Optionally insert approx candidate if valid and not duplicate
  if (has_approximate_joints) {
    Vec6 approx_candidate_rad = approx_joints_rad;
    if (ClampToLimits(approx_candidate_rad, lower_limits_rad, upper_limits_rad,
                      kJointLimitToleranceRad) &&
        IsFkRoundtripValid(approx_candidate_rad, target_pose, target_quaternion,
                           ptr_robot) &&
        !is_direct_duplicate(approx_candidate_rad)) {

      if (ranked_candidates.size() < kMaxIkSolutions) {
        ranked_candidates.push_back(Candidate{approx_candidate_rad, 0.0});
      } else {
        auto worst_candidate_it =
            std::max_element(ranked_candidates.begin(), ranked_candidates.end(),
                             [](const Candidate &a, const Candidate &b) {
                               return a.distance_sq < b.distance_sq;
                             });
        if (worst_candidate_it != ranked_candidates.end() &&
            0.0 < worst_candidate_it->distance_sq) {
          *worst_candidate_it = Candidate{approx_candidate_rad, 0.0};
        }
      }
    }
  }

  if (ranked_candidates.empty())
    return 0;

  std::sort(ranked_candidates.begin(), ranked_candidates.end(),
            [](const Candidate &a, const Candidate &b) {
              return a.distance_sq < b.distance_sq;
            });

  const int solution_count =
      std::min<int>(max_solutions, static_cast<int>(ranked_candidates.size()));

  RadVecToDegArray(ranked_candidates[0].user_joints_rad, joints);
  if (joints_all != nullptr)
    for (int solution_idx = 0; solution_idx < solution_count; ++solution_idx)
      StoreSolution(joints_all, solution_idx,
                    ranked_candidates[solution_idx].user_joints_rad);

  return solution_count;
}
