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
** contract (SolveFK/SolveIK entry points and RoboDK’s robot_T parameter layout).
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
**   https://github.com/RoboDK/Plug-In-Interface/tree/master/robotextensions/samplekinematics
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
**         world_T_tcp = world_T_base * base_T_robot * FK(DH, q) * robot_T_flange * flange_T_tcp
**       and inverted accordingly for IK.
** - DH parameters are read from robot_T (as configured in the .robot model)
**   and must match the convention assumed by the solver (pay attention to
**   any RoboDK-specific coupling/remapping such as CRX J2/J3 coupling, and
**   any “analytic IK convention” conversions used in the derivation).
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
/****************************************************************************
**
** Fanuc CRX (CRX-10iA family) — Custom Inverse Kinematics for RoboDK
**
** (Header comment omitted here for brevity; keep your existing banner if desired)
**
****************************************************************************/

#define _USE_MATH_DEFINES
#include <cassert>
#include <cmath>
#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <vector>
#include <numeric>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "crx_kinematics.h"
#include "angle_conversions_inline.h"

namespace {

// ──────────────────────────────────────────────────────────────────────────────
// Constants (prefer constexpr over macros)
// ──────────────────────────────────────────────────────────────────────────────

static constexpr double TWO_PI  = 2.0 * M_PI;
static constexpr double HALF_PI = 0.5 * M_PI;
static constexpr double F64_EPS = 1e-12;

static constexpr int kPoseRows  = 4;
static constexpr int kPoseCols  = 4;
static constexpr int kPoseElems = 16;

static constexpr int CRX_DOF_COUNT = 6;

static constexpr int CRX_ROBOT_STRIDE              = 20;
static constexpr int CRX_ROBOT_BASE_XYZWPR_ROW     = 9;
static constexpr int CRX_ROBOT_TOOL_XYZWPR_ROW     = 28;
static constexpr int CRX_ROBOT_JOINT_LIM_LOWER_ROW = 30;
static constexpr int CRX_ROBOT_JOINT_LIM_UPPER_ROW = 31;
static constexpr int CRX_ROBOT_JOINT_SENSES_ROW    = 3;
static constexpr int CRX_ROBOT_JOINT_SENSES_COL    = 4;
static constexpr int CRX_ROBOT_DH_BASE_ROW         = 10;
static constexpr int CRX_ROBOT_DOF_ROW             = 1;
static constexpr int CRX_ROBOT_DOF_COL             = 1;

static constexpr int CRX_DH_ARG_ELEMS  = 4;
// Each solution slot: CRX_DOF_COUNT joint values + CRX_DOF_COUNT reserved words = 12.
static constexpr int CRX_SOLUTION_STRIDE = 12;
static_assert(CRX_SOLUTION_STRIDE == 2 * CRX_DOF_COUNT,
              "CRX_SOLUTION_STRIDE must be 2 * CRX_DOF_COUNT");

static constexpr int CRX_J2_INDEX                = 1;
static constexpr int CRX_J3_INDEX                = 2;
static constexpr int CRX_DH_PRISMATIC_FLAG_INDEX = 4;
static constexpr int CRX_DH_THETA_INDEX          = 2;
static constexpr int CRX_DH_D_INDEX              = 3;

static constexpr int    CRX_IK_Q_SAMPLES    = 1440;
static constexpr int    CRX_REFINE_MAX_ITER = 100;
static constexpr double CRX_REFINE_XTOL     = 1e-12;
static constexpr double CRX_ROOT_Z_TOL      = 1e-14;

static constexpr double CRX_SOLUTION_ATOL_DEG = 1e-3;
static constexpr double CRX_SOLUTION_ATOL_RAD = angle_conv::DegToRad(CRX_SOLUTION_ATOL_DEG);
static constexpr double CRX_SOLUTION_ATOL_MM  = 1e-4;
static constexpr double CRX_SOLUTION_ATOL_MM2 = CRX_SOLUTION_ATOL_MM * CRX_SOLUTION_ATOL_MM;

static constexpr double CRX_LIMIT_TOL_DEG = 1e-2;
static constexpr double CRX_LIMIT_TOL_RAD = angle_conv::DegToRad(CRX_LIMIT_TOL_DEG);

static constexpr double CRX_RIGHT_ANGLE_SNAP_TOL = 1e-5;
static constexpr double CRX_DH_CONVENTION_TOL    = 1e-5;
static constexpr double CRX_TRIANGLE_NEG_TOL     = 1e-9;

static constexpr std::size_t kMaxSolutions = 32;

// ──────────────────────────────────────────────────────────────────────────────
// robot_T flat-array accessors
// ──────────────────────────────────────────────────────────────────────────────

static inline const real_T* iRobot_At(const robot_T *r, int row, int col = 0) {
    return reinterpret_cast<const real_T*>(r) + row * CRX_ROBOT_STRIDE + col;
}
static inline const real_T* iRobot_BaseXYZWPR(const robot_T *r)             { return iRobot_At(r, CRX_ROBOT_BASE_XYZWPR_ROW); }
static inline const real_T* iRobot_ToolXYZWPR(const robot_T *r)             { return iRobot_At(r, CRX_ROBOT_TOOL_XYZWPR_ROW); }
static inline const real_T* iRobot_JointLimLower(const robot_T *r)          { return iRobot_At(r, CRX_ROBOT_JOINT_LIM_LOWER_ROW); }
static inline const real_T* iRobot_JointLimUpper(const robot_T *r)          { return iRobot_At(r, CRX_ROBOT_JOINT_LIM_UPPER_ROW); }
static inline const real_T* iRobot_JointSenses(const robot_T *r)            { return iRobot_At(r, CRX_ROBOT_JOINT_SENSES_ROW, CRX_ROBOT_JOINT_SENSES_COL); }
static inline const real_T* iRobot_DHM_JointId(const robot_T *r, int joint) { return iRobot_At(r, CRX_ROBOT_DH_BASE_ROW + joint); }

static inline int iRobot_nDOFs(const robot_T *r) {
    return static_cast<int>(*iRobot_At(r, CRX_ROBOT_DOF_ROW, CRX_ROBOT_DOF_COL));
}

// ──────────────────────────────────────────────────────────────────────────────
// Eigen type aliases
// ──────────────────────────────────────────────────────────────────────────────

using PoseMatRT      = Eigen::Matrix<real_T, kPoseRows, kPoseCols, Eigen::ColMajor>;
using PoseIsoRT      = Eigen::Transform<real_T, 3, Eigen::Isometry>;
using PoseMapConstRT = Eigen::Map<const PoseMatRT>;
using PoseMapRT      = Eigen::Map<PoseMatRT>;

using Vec6  = Eigen::Matrix<double, CRX_DOF_COUNT, 1>;
using Vec6r = Eigen::Matrix<real_T, CRX_DOF_COUNT, 1>;
using Mat3  = Eigen::Matrix3d;
using Vec3  = Eigen::Vector3d;

// ──────────────────────────────────────────────────────────────────────────────
// Pose helpers
// ──────────────────────────────────────────────────────────────────────────────

static inline PoseIsoRT PoseArrayToIsometry(const real_T pose[kPoseElems]) {
    PoseIsoRT out;
    out.matrix() = PoseMapConstRT(pose);
    return out;
}

// FIX: avoid `PoseMapRT(pose) = ...` (most-vexing parse / parameter redeclare)
static inline void IsometryToPoseArray(const PoseIsoRT &iso, real_T pose[kPoseElems]) {
    PoseMapRT pose_map(pose);
    pose_map = iso.matrix();
}

// FANUC/RoboDK WPR: Rz(R)*Ry(P)*Rx(W), translation mm, angles rad.
static inline PoseIsoRT XYZWPR_ToIsometry(const real_T xyzwpr[6]) {
    using AA = Eigen::AngleAxis<real_T>;
    PoseIsoRT T = PoseIsoRT::Identity();
    T.linear() =
        (AA(xyzwpr[5], Eigen::Matrix<real_T,3,1>::UnitZ()) *
         AA(xyzwpr[4], Eigen::Matrix<real_T,3,1>::UnitY()) *
         AA(xyzwpr[3], Eigen::Matrix<real_T,3,1>::UnitX())).toRotationMatrix();
    T.translation() << xyzwpr[0], xyzwpr[1], xyzwpr[2];
    return T;
}

static inline PoseIsoRT DHM_FromRad(real_T alpha, real_T a, real_T theta, real_T d) {
    const real_T ca = std::cos(alpha), sa = std::sin(alpha);
    const real_T ct = std::cos(theta), st = std::sin(theta);
    PoseIsoRT T = PoseIsoRT::Identity();
    T.linear() << ct,      -st,      0.0,
                  st * ca,  ct * ca, -sa,
                  st * sa,  ct * sa,  ca;
    T.translation() << a, -d * sa, d * ca;
    return T;
}

static inline PoseIsoRT FixedJ6ToToolIsometryFk() { return PoseIsoRT::Identity(); }

static inline PoseIsoRT FixedJ6ToToolIsometryAnalytic() {
    PoseIsoRT T = PoseIsoRT::Identity();
    T.linear() << 1.0,  0.0,  0.0,
                  0.0, -1.0,  0.0,
                  0.0,  0.0, -1.0;
    return T;
}

static inline void XYZWPR_2_Pose(const real_T xyzwpr[CRX_DOF_COUNT], real_T pose[kPoseElems]) {
    IsometryToPoseArray(XYZWPR_ToIsometry(xyzwpr), pose);
}

// FIX: avoid `PoseMapRT(out).noalias() = ...` (most-vexing parse)
static inline void Pose_Mult(const real_T A[kPoseElems], const real_T B[kPoseElems], real_T out[kPoseElems]) {
    PoseMapRT out_map(out);
    out_map.noalias() = PoseMapConstRT(A) * PoseMapConstRT(B);
}

// ──────────────────────────────────────────────────────────────────────────────
// Angle utilities — all radians
// ──────────────────────────────────────────────────────────────────────────────

static inline double WrapRad2Pi(double a) {
    double w = std::fmod(a, TWO_PI);
    return (w < 0.0) ? w + TWO_PI : w;
}
static inline double WrapRadPi(double a) {
    double w = std::fmod(a + M_PI, TWO_PI);
    if (w < 0.0) w += TWO_PI;
    return w - M_PI;
}

static inline double NormalizeRadKeepSignedPi(double a) {
    while (a >  M_PI) a -= TWO_PI;
    while (a < -M_PI) a += TWO_PI;
    return a;
}
static inline void NormalizeVecKeepSignedPi(Vec6 &q) {
    q = q.unaryExpr([](double x){ return NormalizeRadKeepSignedPi(x); });
}
static inline void NormalizeUserSolutionDomains(Vec6 &q) {
    for (int i = 0; i < CRX_DOF_COUNT; ++i) {
        if (i == CRX_J3_INDEX) continue;
        q[i] = NormalizeRadKeepSignedPi(q[i]);
    }
}

static inline double SnapToRightAngleFamily(double a, double tol = CRX_RIGHT_ANGLE_SNAP_TOL) {
    static const std::array<double, 5> refs = {-M_PI, -HALF_PI, 0.0, HALF_PI, M_PI};
    for (double r : refs)
        if (std::abs(WrapRadPi(a - r)) <= tol) return r;
    return a;
}
static inline double AngleDiffAbs(double a, double b) { return std::abs(WrapRadPi(a - b)); }

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

static inline void ReadJointLimitsRad(const robot_T *r, Vec6 &lo_rad, Vec6 &hi_rad) {
    const double k = angle_conv::DegToRad(1.0);
    const real_T *lo = iRobot_JointLimLower(r);
    const real_T *hi = iRobot_JointLimUpper(r);
    for (int i = 0; i < CRX_DOF_COUNT; ++i) {
        lo_rad[i] = static_cast<double>(lo[i]) * k;
        hi_rad[i] = static_cast<double>(hi[i]) * k;
    }
}

static inline double NormalizeJointSense(real_T s) {
    if (!std::isfinite(s) || std::abs(s) <= F64_EPS) return 1.0;
    return (s >= 0.0) ? 1.0 : -1.0;
}
static inline std::array<double, CRX_DOF_COUNT> ReadNormalizedJointSenses(const robot_T *r) {
    const real_T *raw = iRobot_JointSenses(r);
    std::array<double, CRX_DOF_COUNT> out{};
    for (int i = 0; i < CRX_DOF_COUNT; ++i) out[i] = NormalizeJointSense(raw[i]);
    return out;
}

static inline bool ClampToLimits(Vec6& q, const Vec6& lo, const Vec6& hi, double tol_rad) {
    for (int i = 0; i < CRX_DOF_COUNT; ++i) {
        if (q[i] < lo[i] - tol_rad || q[i] > hi[i] + tol_rad) return false;
        q[i] = std::min(hi[i], std::max(lo[i], q[i]));
    }
    return true;
}

// ──────────────────────────────────────────────────────────────────────────────
// Vec6 distance helpers (rad domain)
// ──────────────────────────────────────────────────────────────────────────────

static inline double WrappedDist2Rad(const Vec6 &a, const Vec6 &b) {
    double acc = 0.0;
    for (int i = 0; i < CRX_DOF_COUNT; ++i) {
        const double d = WrapRadPi(a[i] - b[i]);
        acc += d * d;
    }
    return acc;
}
static inline double WrappedMaxAbsDiffRad(const Vec6 &a, const Vec6 &b) {
    double mx = 0.0;
    for (int i = 0; i < CRX_DOF_COUNT; ++i)
        mx = std::max(mx, std::abs(WrapRadPi(a[i] - b[i])));
    return mx;
}
static inline double MaxAbsDiffRadDirect(const Vec6 &a, const Vec6 &b) {
    return (a - b).cwiseAbs().maxCoeff();
}

// ──────────────────────────────────────────────────────────────────────────────
// Solution storage — rad→deg conversion at write time only
// ──────────────────────────────────────────────────────────────────────────────

static inline void StoreSolution(real_T *joints_all, int id, const Vec6 &user_rad) {
    real_T *dst = joints_all + CRX_SOLUTION_STRIDE * id;
    std::fill(dst, dst + CRX_SOLUTION_STRIDE, static_cast<real_T>(0.0));
    RadVecToDegArray(user_rad, dst);
}

// ──────────────────────────────────────────────────────────────────────────────
// FK core
// ──────────────────────────────────────────────────────────────────────────────

static inline void BuildJointPoseInputRad(const real_T *dh, double model_rad,
                                          real_T rx_tx_rz_tz[CRX_DH_ARG_ELEMS]) {
    rx_tx_rz_tz[0]                  = static_cast<real_T>(SnapToRightAngleFamily(dh[0]));
    rx_tx_rz_tz[1]                  = dh[1];
    rx_tx_rz_tz[CRX_DH_THETA_INDEX]  = static_cast<real_T>(SnapToRightAngleFamily(dh[CRX_DH_THETA_INDEX]));
    rx_tx_rz_tz[CRX_DH_D_INDEX]      = dh[CRX_DH_D_INDEX];

    if (dh[CRX_DH_PRISMATIC_FLAG_INDEX] == 0.0)
        rx_tx_rz_tz[CRX_DH_THETA_INDEX] += static_cast<real_T>(model_rad);
    else
        rx_tx_rz_tz[CRX_DH_D_INDEX]     += static_cast<real_T>(model_rad);
}

static int SolveFKCore(const real_T *joints_user_deg,
                       real_T pose_out[kPoseElems],
                       real_T *joint_poses_out,
                       int max_poses,
                       bool check_limits,
                       const robot_T *ptr_robot) {
    const int nDOFs = iRobot_nDOFs(ptr_robot);
    if (nDOFs != CRX_DOF_COUNT) return -1;
    if (joint_poses_out != nullptr && max_poses < nDOFs + 1) return -1;

    Vec6 joints_rad;
    DegArrayToRadVec(joints_user_deg, joints_rad);

    const auto senses = ReadNormalizedJointSenses(ptr_robot);

    Vec6 lo_rad, hi_rad;
    if (check_limits) ReadJointLimitsRad(ptr_robot, lo_rad, hi_rad);

    real_T pose_base_local[kPoseElems];
    real_T *pose_base = (joint_poses_out != nullptr)
                      ? (joint_poses_out + kPoseElems * 0)
                      : pose_base_local;

    real_T pose_tool[kPoseElems], pose_j6_tool[kPoseElems];
    XYZWPR_2_Pose(iRobot_BaseXYZWPR(ptr_robot), pose_base);
    XYZWPR_2_Pose(iRobot_ToolXYZWPR(ptr_robot), pose_tool);
    IsometryToPoseArray(FixedJ6ToToolIsometryFk(), pose_j6_tool);

    real_T *last_pose = pose_base;
    real_T next_pose_local[kPoseElems];

    const double sensed_j2_rad = joints_rad[CRX_J2_INDEX] * senses[CRX_J2_INDEX];

    for (int i = 0; i < nDOFs; ++i) {
        const double sensed_rad = joints_rad[i] * senses[i];

        if (check_limits && (sensed_rad < lo_rad[i] || sensed_rad > hi_rad[i])) return -2;

        double model_rad = sensed_rad;
        if (i == CRX_J3_INDEX) model_rad = -sensed_j2_rad + sensed_rad;

        real_T rx_tx_rz_tz[CRX_DH_ARG_ELEMS];
        BuildJointPoseInputRad(iRobot_DHM_JointId(ptr_robot, i), model_rad, rx_tx_rz_tz);

        real_T T_i[kPoseElems];
        IsometryToPoseArray(DHM_FromRad(rx_tx_rz_tz[0], rx_tx_rz_tz[1],
                                        rx_tx_rz_tz[2], rx_tx_rz_tz[3]), T_i);

        real_T *next_pose = (joint_poses_out != nullptr)
                          ? (joint_poses_out + kPoseElems * (i + 1))
                          : next_pose_local;
        Pose_Mult(last_pose, T_i, next_pose);
        last_pose = next_pose;
    }

    real_T pose_flange[kPoseElems];
    Pose_Mult(last_pose,   pose_j6_tool, pose_flange);
    Pose_Mult(pose_flange, pose_tool,    pose_out);
    return 1;
}

static bool IsFkRoundtripValid(const Vec6 &user_rad,
                               const PoseIsoRT &target_pose,
                               const Eigen::Quaterniond &target_quat,
                               const robot_T *ptr_robot) {
    real_T user_deg[CRX_DOF_COUNT];
    real_T fk_pose[kPoseElems];
    RadVecToDegArray(user_rad, user_deg);
    if (SolveFKCore(user_deg, fk_pose, nullptr, 0, false, ptr_robot) != 1) return false;

    const PoseIsoRT T_fk = PoseArrayToIsometry(fk_pose);
    const Vec3 dp = T_fk.translation().cast<double>() - target_pose.translation().cast<double>();
    if (!std::isfinite(dp.squaredNorm()) || dp.squaredNorm() > CRX_SOLUTION_ATOL_MM2) return false;

    const Eigen::Quaterniond q_fk(T_fk.linear().cast<double>());
    const double ang_err = q_fk.angularDistance(target_quat);
    return std::isfinite(ang_err) && ang_err <= CRX_SOLUTION_ATOL_RAD;
}


// ──────────────────────────────────────────────────────────────────────────────
// IK — analytic CRX
// ──────────────────────────────────────────────────────────────────────────────

struct CrxParams {
    std::array<double, CRX_DOF_COUNT> alpha_rad{};
    std::array<double, CRX_DOF_COUNT> a{};
    std::array<double, CRX_DOF_COUNT> d{};
    std::array<double, CRX_DOF_COUNT> theta0_rad{};
    double a2 = 0.0, r4 = 0.0, r5 = 0.0, r6 = 0.0;
};

static bool ReadCrxParams(const robot_T *r, CrxParams &p) {
    for (int i = 0; i < CRX_DOF_COUNT; ++i) {
        const real_T *dh = iRobot_DHM_JointId(r, i);
        if (dh[4] != 0.0) return false; // prismatic not supported
        p.alpha_rad[i]  = SnapToRightAngleFamily(dh[0]);
        p.a[i]          = dh[1];
        p.theta0_rad[i] = SnapToRightAngleFamily(dh[2]);
        p.d[i]          = dh[3];
    }
    p.a2 = p.a[2]; p.r4 = p.d[3]; p.r5 = p.d[4]; p.r6 = p.d[5];
    return true;
}

static bool ConvertRoboDkDhToAnalyticIkConvention(CrxParams &p, double &pre_shift_z) {
    pre_shift_z = 0.0;
    const double tol = CRX_DH_CONVENTION_TOL;

    const bool alpha_ok =
        AngleDiffAbs(p.alpha_rad[0],  0.0    ) < tol &&
        AngleDiffAbs(p.alpha_rad[1], -HALF_PI) < tol &&
        AngleDiffAbs(p.alpha_rad[2],  0.0    ) < tol &&
        AngleDiffAbs(p.alpha_rad[3], -HALF_PI) < tol &&
        AngleDiffAbs(p.alpha_rad[4],  HALF_PI) < tol &&
        AngleDiffAbs(p.alpha_rad[5], -HALF_PI) < tol;

    const bool theta_ok =
        AngleDiffAbs(p.theta0_rad[0],  0.0    ) < tol &&
        AngleDiffAbs(p.theta0_rad[1], -HALF_PI) < tol &&
        AngleDiffAbs(p.theta0_rad[2],  0.0    ) < tol &&
        AngleDiffAbs(p.theta0_rad[3],  0.0    ) < tol &&
        AngleDiffAbs(p.theta0_rad[4],  0.0    ) < tol &&
        AngleDiffAbs(p.theta0_rad[5],  0.0    ) < tol;

    const bool a_ok =
        std::abs(p.a[0]) <= F64_EPS &&
        std::abs(p.a[1]) <= F64_EPS &&
        p.a[2]  > 0.0               &&
        std::abs(p.a[3]) <= F64_EPS &&
        std::abs(p.a[4]) <= F64_EPS &&
        std::abs(p.a[5]) <= F64_EPS;

    const bool d_ok =
        p.d[0]  > 0.0                &&
        std::abs(p.d[1]) <= F64_EPS  &&
        std::abs(p.d[2]) <= F64_EPS  &&
        p.d[3]  > 0.0                &&
        p.d[4] <= -F64_EPS           && // Pass-1 Fix #2
        p.d[5]  > 0.0;

    if (!(alpha_ok && theta_ok && a_ok && d_ok)) return false;

    pre_shift_z    = p.d[0];
    p.d[0]         = 0.0;
    p.alpha_rad[2] = M_PI;
    p.d[3]         = -p.d[3];
    p.d[4]         = -p.d[4];
    p.d[5]         = -p.d[5];
    p.a2 = p.a[2]; p.r4 = p.d[3]; p.r5 = p.d[4]; p.r6 = p.d[5];
    return true;
}

static inline std::array<double, CRX_DOF_COUNT> CoupledThetaRad(const Vec6 &j, const CrxParams &p) {
    return { j[0] + p.theta0_rad[0],
             j[1] + p.theta0_rad[1],
             j[1] + j[2] + p.theta0_rad[2],
             j[3] + p.theta0_rad[3],
             j[4] + p.theta0_rad[4],
             j[5] + p.theta0_rad[5] };
}

static inline PoseIsoRT ForwardFromSolverJoints(const Vec6 &j, const CrxParams &p) {
    const auto theta = CoupledThetaRad(j, p);
    PoseIsoRT T = PoseIsoRT::Identity();
    for (int i = 0; i < CRX_DOF_COUNT; ++i)
        T = T * DHM_FromRad(static_cast<real_T>(p.alpha_rad[i]),
                            static_cast<real_T>(p.a[i]),
                            static_cast<real_T>(theta[i]),
                            static_cast<real_T>(p.d[i]));
    return T * FixedJ6ToToolIsometryAnalytic();
}

static inline bool IsPoseConsistent(const Vec6 &j, const PoseIsoRT &T_target, const CrxParams &p) {
    const PoseIsoRT T_fk = ForwardFromSolverJoints(j, p);
    const Vec3 dp = T_fk.translation().cast<double>() - T_target.translation().cast<double>();
    if (!std::isfinite(dp.squaredNorm()) || dp.squaredNorm() > CRX_SOLUTION_ATOL_MM2) return false;
    const double ang_err = Eigen::Quaterniond(T_fk.linear().cast<double>())
                               .angularDistance(Eigen::Quaterniond(T_target.linear().cast<double>()));
    return std::isfinite(ang_err) && ang_err <= CRX_SOLUTION_ATOL_RAD;
}

// Specialized hot-path basis build for ConstructPlane(O4, UnitZ()).
static inline bool ConstructPlane_O4_UnitZ(const Vec3 &O4, Mat3 &R_plane) {
    constexpr double kPlaneEps2 = 1e-18;

    const double n2 = O4.squaredNorm();
    if (!std::isfinite(n2) || n2 <= F64_EPS * F64_EPS) return false;

    const Vec3 x = O4 * (1.0 / std::sqrt(n2));

    // Prefer y close to projected +Z; fall back to robust orthogonal.
    Vec3 y = Vec3::UnitZ() - x.z() * x;
    double y2 = y.squaredNorm();
    if (!std::isfinite(y2) || y2 <= kPlaneEps2) {
        y = x.unitOrthogonal();
        y2 = y.squaredNorm();
        if (!std::isfinite(y2) || y2 <= kPlaneEps2) return false;
    } else {
        y *= (1.0 / std::sqrt(y2));
    }

    Vec3 z = x.cross(y);
    const double z2 = z.squaredNorm();
    if (!std::isfinite(z2) || z2 <= kPlaneEps2) return false;
    z *= (1.0 / std::sqrt(z2));

    // Defensive re-orthogonalization against numeric drift.
    y = z.cross(x);

    R_plane.col(0) = x;
    R_plane.col(1) = y;
    R_plane.col(2) = z;

#ifndef NDEBUG
    assert(std::abs(R_plane.determinant() - 1.0) < 1e-9 &&
           "ConstructPlane_O4_UnitZ: R_plane is not a rotation matrix");
#endif
    return true;
}

static inline bool FindThirdTriangleCorner(double AB, double AC, double BC,
                                           double &x, double &y_abs) {
    if (AB <= F64_EPS) return false;
    x = (AC*AC - BC*BC + AB*AB) / (2.0 * AB);
    const double y2 = AC*AC - x*x;
    if (y2 < -CRX_TRIANGLE_NEG_TOL) return false;
    y_abs = std::sqrt(std::max(0.0, y2));
    return true;
}

static inline PoseIsoRT JointTransformRad(const CrxParams &p, int id, double q_rad) {
    return DHM_FromRad(static_cast<real_T>(p.alpha_rad[id]),
                       static_cast<real_T>(p.a[id]),
                       static_cast<real_T>(p.theta0_rad[id] + q_rad),
                       static_cast<real_T>(p.d[id]));
}

static bool DetermineJointValues(const Vec3 &O3,
                                 const Vec3 &O4,
                                 const Vec3 &O5,
                                 const PoseIsoRT &T06_target,
                                 const CrxParams &p,
                                 Vec6 &joints_rad) {
    const Vec3 O6 = T06_target.translation().cast<double>();

    const double J1 = std::atan2(O4.y(), O4.x());
    const Mat3 R_L1_L0 = JointTransformRad(p, 0, J1).inverse().linear().cast<double>();

    const Vec3 O_1_3 = R_L1_L0 * O3;
    const double J2  = std::atan2(O_1_3.x(), O_1_3.z());

    const Vec3 O_1_4 = R_L1_L0 * O4;
    const double J3  = std::atan2(O_1_4.z() - O_1_3.z(), O_1_4.x() - O_1_3.x());

    const PoseIsoRT T_L2_L1 = JointTransformRad(p, 1, J2).inverse();
    const PoseIsoRT T_L3_L2 = JointTransformRad(p, 2, J2 + J3).inverse();
    const PoseIsoRT T_L3_L0 = T_L3_L2 * T_L2_L1 * PoseIsoRT(R_L1_L0);

    const Vec3 O_3_5 = (T_L3_L0 * O5.cast<real_T>()).template head<3>().cast<double>();
    const double J4  = std::atan2(O_3_5.x(), O_3_5.z());

    const PoseIsoRT T_L4_L0 = JointTransformRad(p, 3, J4).inverse() * T_L3_L0;

    const Vec3 O_4_6 = (T_L4_L0 * O6.cast<real_T>()).template head<3>().cast<double>();
    const double J5  = std::atan2(O_4_6.x(), -O_4_6.z());

    const PoseIsoRT T_L5_L0 = JointTransformRad(p, 4, J5).inverse() * T_L4_L0;
    const Vec3 tool_x_in_L5 = (T_L5_L0.linear() * T06_target.linear()).cast<double>().col(0);
    const double J6          = std::atan2(-tool_x_in_L5.z(), tool_x_in_L5.x());

    if (!(std::isfinite(J1) && std::isfinite(J2) && std::isfinite(J3) &&
          std::isfinite(J4) && std::isfinite(J5) && std::isfinite(J6)))
        return false;

    joints_rad << NormalizeRadKeepSignedPi(J1), NormalizeRadKeepSignedPi(J2),
                  NormalizeRadKeepSignedPi(J3), NormalizeRadKeepSignedPi(J4),
                  NormalizeRadKeepSignedPi(J5), NormalizeRadKeepSignedPi(J6);

    return true;
}

// ─────────────────────────── Circle sweep ───────────────────────────────────

struct CircleEvalContext {
    Mat3   target_R      = Mat3::Identity();
    Vec3   target_t      = Vec3::Zero();
    Vec3   O5            = Vec3::Zero();
    double min_d04_sq    = 0.0;
    double max_d04_sq    = 0.0;
    bool   has_min_bound = false;
};

struct CircleEvaluation {
    Vec3   O4       = Vec3::Zero();
    Vec3   O3_up    = Vec3::Zero();
    Vec3   O3_down  = Vec3::Zero();
    double dot_up   = std::numeric_limits<double>::quiet_NaN();
    double dot_down = std::numeric_limits<double>::quiet_NaN();
    bool   valid    = false;
};

static bool EvaluateCircle_cs(double cq, double sq,
                              const CircleEvalContext &ctx,
                              const CrxParams &p,
                              CircleEvaluation &eval,
                              Mat3 &R_plane_out) {
    eval.valid = false;

    const Vec3 O4_local(p.r5 * cq, p.r5 * sq, p.r6);
    eval.O4.noalias() = ctx.target_R * O4_local;
    eval.O4 += ctx.target_t;

    const double d04_sq = eval.O4.squaredNorm();
    if (!std::isfinite(d04_sq) || d04_sq <= F64_EPS * F64_EPS ||
        d04_sq > ctx.max_d04_sq ||
        (ctx.has_min_bound && d04_sq < ctx.min_d04_sq))
        return false;

    if (!ConstructPlane_O4_UnitZ(eval.O4, R_plane_out)) return false;

    const double d04 = std::sqrt(d04_sq);
    double x = 0.0, y = 0.0;
    if (!FindThirdTriangleCorner(d04, p.a2, -p.r4, x, y)) return false;

    eval.O3_up   = R_plane_out * Vec3(x,  y, 0.0);
    eval.O3_down = R_plane_out * Vec3(x, -y, 0.0);

    const Vec3 Z5 = ctx.O5 - eval.O4;
    const double z5_sq = Z5.squaredNorm();
    if (!std::isfinite(z5_sq) || z5_sq <= F64_EPS * F64_EPS) return false;
    const double inv_z5 = 1.0 / std::sqrt(z5_sq);

    const Vec3 U34_up = eval.O4 - eval.O3_up;
    const Vec3 U34_dn = eval.O4 - eval.O3_down;

    const double u_up_sq = U34_up.squaredNorm();
    const double u_dn_sq = U34_dn.squaredNorm();
    if (!std::isfinite(u_up_sq) || u_up_sq <= F64_EPS * F64_EPS) return false;
    if (!std::isfinite(u_dn_sq) || u_dn_sq <= F64_EPS * F64_EPS) return false;

    eval.dot_up   = U34_up.dot(Z5) * (1.0 / std::sqrt(u_up_sq)) * inv_z5;
    eval.dot_down = U34_dn.dot(Z5) * (1.0 / std::sqrt(u_dn_sq)) * inv_z5;
    eval.valid    = std::isfinite(eval.dot_up) && std::isfinite(eval.dot_down);
    return eval.valid;
}

static bool EvaluateCircle(double q,
                           const CircleEvalContext &ctx,
                           const CrxParams &p,
                           CircleEvaluation &eval,
                           Mat3 &R_plane_out) {
    const double cq = std::cos(q);
    const double sq = std::sin(q);
    return EvaluateCircle_cs(cq, sq, ctx, p, eval, R_plane_out);
}

static inline bool HasBracket(double fa, double fb) {
    if (!(std::isfinite(fa) && std::isfinite(fb))) return false;
    if (std::abs(fa) <= CRX_ROOT_Z_TOL || std::abs(fb) <= CRX_ROOT_Z_TOL) return true;
    return fa * fb < 0.0;
}

template <typename EvalFn>
static bool RefineZeroBisection(double qa, double qb, double fa, double fb,
                                EvalFn eval_fn, double &root_out) {
    if (!(std::isfinite(fa) && std::isfinite(fb))) return false;
    if (std::abs(fa) <= CRX_ROOT_Z_TOL) { root_out = WrapRad2Pi(qa); return true; }
    if (std::abs(fb) <= CRX_ROOT_Z_TOL) { root_out = WrapRad2Pi(qb); return true; }
    if (fa * fb > 0.0) return false;

    double a = qa, b = qb;
    for (int i = 0; i < CRX_REFINE_MAX_ITER; ++i) {
        const double m  = 0.5 * (a + b);
        const double fm = eval_fn(m);
        if (!std::isfinite(fm)) return false;
        if (std::abs(fm) <= CRX_ROOT_Z_TOL || (b - a) < CRX_REFINE_XTOL) {
            root_out = WrapRad2Pi(m);
            return true;
        }
        if (fa * fm <= 0.0) { b = m; fb = fm; }
        else                { a = m; fa = fm; }
    }
    root_out = WrapRad2Pi(0.5 * (a + b));
    return true;
}

static void DedupSolutions(const std::vector<Vec6> &in, std::vector<Vec6> &out) {
    out.clear();
    out.reserve(std::min<std::size_t>(kMaxSolutions, in.size()));
    for (Vec6 s : in) {
        NormalizeVecKeepSignedPi(s);
        const bool dup = std::any_of(out.begin(), out.end(), [&](const Vec6& u){
            return WrappedMaxAbsDiffRad(s, u) <= CRX_SOLUTION_ATOL_RAD;
        });
        if (!dup) {
            if (out.size() >= kMaxSolutions) break;
            out.push_back(s);
        }
    }
}

static void DualSolutionRad(const Vec6 &s, Vec6 &d) {
    d = s;
    d[0] = s[0] - M_PI;
    d[1] = -s[1];
    d[2] = M_PI - s[2];
    d[3] = s[3] - M_PI;
    d[4] = s[4];
    d[5] = s[5];
    NormalizeUserSolutionDomains(d);
}

static void AddDualSolutions(const std::vector<Vec6> &in, std::vector<Vec6> &out) {
    out.clear();
    out.reserve(std::min<std::size_t>(kMaxSolutions, in.size() * 2u));
    for (const Vec6 &s : in) {
        if (out.size() >= kMaxSolutions) return;
        out.push_back(s);
    }
    Vec6 d = Vec6::Zero();
    for (const Vec6 &s : in) {
        if (out.size() >= kMaxSolutions) return;
        DualSolutionRad(s, d);
        NormalizeVecKeepSignedPi(d);
        out.push_back(d);
    }
}

template <typename EmitFn>
static bool ForEachSignedPiVariant(const Vec6 &q, EmitFn emit) {
    constexpr double kPiFlipTol = 1e-8;

    std::vector<int> flip_ids;
    flip_ids.reserve(CRX_DOF_COUNT);
    for (int i = 0; i < CRX_DOF_COUNT; ++i)
        if (std::abs(std::abs(q[i]) - M_PI) <= kPiFlipTol)
            flip_ids.push_back(i);

    const std::uint32_t variants = static_cast<std::uint32_t>(1u << flip_ids.size());
    for (std::uint32_t mask = 0; mask < variants; ++mask) {
        Vec6 v = q;
        for (std::size_t bit = 0; bit < flip_ids.size(); ++bit) {
            if ((mask & (1u << bit)) == 0u) continue;
            const int j = flip_ids[bit];
            v[j] = (q[j] >= 0.0) ? -M_PI : M_PI;
        }
        if (!emit(v)) return false;
    }
    return true;
}

template <typename EmitFn>
static bool ForEachCandidateVariant(const Vec6 &q, EmitFn emit) {
    if (!ForEachSignedPiVariant(q, [&](const Vec6 &base) { return emit(base); })) return false;

    Vec6 dual = Vec6::Zero();
    DualSolutionRad(q, dual);
    return ForEachSignedPiVariant(dual, [&](const Vec6 &dvar) { return emit(dvar); });
}

struct Candidate {
    Vec6   user_rad = Vec6::Zero();
    double dist2    = 0.0;
};

// ─────────────────────────── Closed-form CRX IK ─────────────────────────────

static void SolveCrxIk(const PoseIsoRT &T06_target, const CrxParams &p, std::vector<Vec6> &solutions_out) {
    solutions_out.clear();
    solutions_out.reserve(kMaxSolutions);

    CircleEvalContext ctx;
    ctx.target_R = T06_target.linear().cast<double>();
    ctx.target_t = T06_target.translation().cast<double>();
    ctx.O5       = ctx.target_R * Vec3(0.0, 0.0, p.r6) + ctx.target_t;

    const double L1 = std::abs(p.a2), L2 = std::abs(p.r4);
    const double reach_eps = 1e-6 * std::max(1.0, L1 + L2);
    const double max_d04 = L1 + L2 + reach_eps;
    const double min_d04 = std::abs(L1 - L2) - reach_eps;
    ctx.max_d04_sq    = max_d04 * max_d04;
    ctx.has_min_bound = min_d04 > 0.0;
    ctx.min_d04_sq    = ctx.has_min_bound ? min_d04 * min_d04 : 0.0;

    std::vector<Vec6> sols;
    sols.reserve(kMaxSolutions);

    const auto eval_up = [&](double q_eval) -> double {
        CircleEvaluation tmp; Mat3 rp;
        return EvaluateCircle(WrapRad2Pi(q_eval), ctx, p, tmp, rp)
               ? tmp.dot_up : std::numeric_limits<double>::quiet_NaN();
    };
    const auto eval_down = [&](double q_eval) -> double {
        CircleEvaluation tmp; Mat3 rp;
        return EvaluateCircle(WrapRad2Pi(q_eval), ctx, p, tmp, rp)
               ? tmp.dot_down : std::numeric_limits<double>::quiet_NaN();
    };

    Vec6 q = Vec6::Zero();
    const auto try_store = [&](const Vec3 &O3_cand, const CircleEvaluation &re) {
        if (sols.size() >= kMaxSolutions) return;
        if (DetermineJointValues(O3_cand, re.O4, ctx.O5, T06_target, p, q)) {
            NormalizeVecKeepSignedPi(q);
            sols.push_back(q);
        }
    };

    const double q_step = TWO_PI / static_cast<double>(CRX_IK_Q_SAMPLES);
    const double c_step = std::cos(q_step);
    const double s_step = std::sin(q_step);

    CircleEvaluation prev_eval, cur_eval, root_eval;
    Mat3 prev_rp, cur_rp, root_rp;

    double prev_q = 0.0;
    double cq = 1.0;
    double sq = 0.0;

    bool prev_ok = EvaluateCircle_cs(cq, sq, ctx, p, prev_eval, prev_rp);

    for (int i = 1; i <= CRX_IK_Q_SAMPLES; ++i) {
        const double cur_q = q_step * static_cast<double>(i);

        const double cq_new = cq * c_step - sq * s_step;
        const double sq_new = sq * c_step + cq * s_step;
        cq = cq_new;
        sq = sq_new;

        const bool cur_ok = EvaluateCircle_cs(cq, sq, ctx, p, cur_eval, cur_rp);

        if (prev_ok && cur_ok) {
            if (HasBracket(prev_eval.dot_up, cur_eval.dot_up)) {
                double root = 0.0;
                if (RefineZeroBisection(prev_q, cur_q,
                                        prev_eval.dot_up, cur_eval.dot_up, eval_up, root) &&
                    EvaluateCircle(root, ctx, p, root_eval, root_rp))
                    try_store(root_eval.O3_up, root_eval);
            }
            if (HasBracket(prev_eval.dot_down, cur_eval.dot_down)) {
                double root = 0.0;
                if (RefineZeroBisection(prev_q, cur_q,
                                        prev_eval.dot_down, cur_eval.dot_down, eval_down, root) &&
                    EvaluateCircle(root, ctx, p, root_eval, root_rp))
                    try_store(root_eval.O3_down, root_eval);
            }
        }
        prev_q    = cur_q;
        prev_eval = cur_eval;
        prev_ok   = cur_ok;

        if (sols.size() >= kMaxSolutions) break;
    }

    std::vector<Vec6> dual_sols;
    AddDualSolutions(sols, dual_sols);
    DedupSolutions(dual_sols, solutions_out);
}

} // namespace

// ──────────────────────────────────────────────────────────────────────────────
// Public entry points
// ──────────────────────────────────────────────────────────────────────────────

int SolveFK(const real_T *joints, real_T pose[16], const robot_T *ptr_robot) {
    return SolveFKCore(joints, pose, nullptr, 0, true, ptr_robot);
}

int SolveFK_CAD(const real_T *joints, real_T pose[16], real_T *joint_poses, int max_poses,
                const robot_T *ptr_robot) {
    return SolveFKCore(joints, pose, joint_poses, max_poses, false, ptr_robot);
}

int SolveIK(const real_T pose[16],
            real_T *joints,
            real_T *joints_all,
            int max_solutions,
            const real_T *joints_approx,
            const robot_T *ptr_robot) {
    if (iRobot_nDOFs(ptr_robot) != CRX_DOF_COUNT) return -1;
    if (max_solutions <= 0) return 0;

    CrxParams params;
    if (!ReadCrxParams(ptr_robot, params)) return -1;

    const PoseIsoRT T_base = XYZWPR_ToIsometry(iRobot_BaseXYZWPR(ptr_robot));
    const PoseIsoRT T_tool = XYZWPR_ToIsometry(iRobot_ToolXYZWPR(ptr_robot));
    const PoseIsoRT T_pose = PoseArrayToIsometry(pose);
    const Eigen::Quaterniond q_pose_target(T_pose.linear().cast<double>());

    const PoseIsoRT T06_target_raw = T_base.inverse() * T_pose * T_tool.inverse();

    double pre_shift_z = 0.0;
    if (!ConvertRoboDkDhToAnalyticIkConvention(params, pre_shift_z)) return -1;

    PoseIsoRT T06_target = T06_target_raw;
    if (std::abs(pre_shift_z) > F64_EPS) {
        PoseIsoRT T_shift = PoseIsoRT::Identity();
        T_shift.translation() << 0.0, 0.0, -pre_shift_z;
        T06_target = T_shift * T06_target_raw;
    }

    std::vector<Vec6> solver_solutions;
    SolveCrxIk(T06_target, params, solver_solutions);

    // Limits converted once in rad
    Vec6 lo_rad, hi_rad;
    ReadJointLimitsRad(ptr_robot, lo_rad, hi_rad);

    // If no solutions: try approximate roundtrip if provided
    if (solver_solutions.empty()) {
        if (joints_approx != nullptr) {
            Vec6 approx_rad = Vec6::Zero();
            DegArrayToRadVec(joints_approx, approx_rad);
            if (ClampToLimits(approx_rad, lo_rad, hi_rad, CRX_LIMIT_TOL_RAD) &&
                IsFkRoundtripValid(approx_rad, T_pose, q_pose_target, ptr_robot)) {
                RadVecToDegArray(approx_rad, joints);
                if (joints_all != nullptr) StoreSolution(joints_all, 0, approx_rad);
                return 1;
            }
        }
        return 0;
    }

    // Approx converted once at entry
    Vec6 approx_rad = Vec6::Zero();
    const bool has_approx = (joints_approx != nullptr);
    if (has_approx) { DegArrayToRadVec(joints_approx, approx_rad); NormalizeVecKeepSignedPi(approx_rad); }

    std::vector<Candidate> cands;
    cands.reserve(kMaxSolutions);

    auto is_dup_direct = [&](const Vec6& u) {
        return std::any_of(cands.begin(), cands.end(), [&](const Candidate& c){
            return MaxAbsDiffRadDirect(u, c.user_rad) <= CRX_SOLUTION_ATOL_RAD;
        });
    };

    for (Vec6 sol : solver_solutions) {
        NormalizeUserSolutionDomains(sol);

        const bool complete = ForEachCandidateVariant(sol, [&](Vec6 user_rad) {
            if (!ClampToLimits(user_rad, lo_rad, hi_rad, CRX_LIMIT_TOL_RAD)) return true;

            if (!IsFkRoundtripValid(user_rad, T_pose, q_pose_target, ptr_robot)) return true;

            // Keep signed-pi variants distinct for deterministic branch matching.
            if (is_dup_direct(user_rad)) return true;

            const double dist2 = has_approx ? WrappedDist2Rad(user_rad, approx_rad)
                                            : static_cast<double>(cands.size());

            if (cands.size() < kMaxSolutions) {
                cands.push_back(Candidate{user_rad, dist2});
            } else if (has_approx) {
                // keep best-k when approx provided
                auto worst_it = std::max_element(cands.begin(), cands.end(),
                    [](const Candidate& a, const Candidate& b){ return a.dist2 < b.dist2; });
                if (worst_it != cands.end() && dist2 < worst_it->dist2) *worst_it = Candidate{user_rad, dist2};
            } // else: preserve first-k behavior
            return true;
        });

        if (!complete) break;
    }

    // Optionally insert approx candidate if valid and not duplicate
    if (has_approx) {
        Vec6 approx_cand = approx_rad;
        if (ClampToLimits(approx_cand, lo_rad, hi_rad, CRX_LIMIT_TOL_RAD) &&
            IsFkRoundtripValid(approx_cand, T_pose, q_pose_target, ptr_robot) &&
            !is_dup_direct(approx_cand)) {

            if (cands.size() < kMaxSolutions) {
                cands.push_back(Candidate{approx_cand, 0.0});
            } else {
                auto worst_it = std::max_element(cands.begin(), cands.end(),
                    [](const Candidate& a, const Candidate& b){ return a.dist2 < b.dist2; });
                if (worst_it != cands.end() && 0.0 < worst_it->dist2) *worst_it = Candidate{approx_cand, 0.0};
            }
        }
    }

    if (cands.empty()) return 0;

    std::sort(cands.begin(), cands.end(),
              [](const Candidate& a, const Candidate& b){ return a.dist2 < b.dist2; });

    const int n = std::min<int>(max_solutions, static_cast<int>(cands.size()));

    RadVecToDegArray(cands[0].user_rad, joints);
    if (joints_all != nullptr)
        for (int s = 0; s < n; ++s)
            StoreSolution(joints_all, s, cands[s].user_rad);

    return n;
}