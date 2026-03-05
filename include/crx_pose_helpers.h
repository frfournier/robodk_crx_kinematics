#pragma once

#include "crx_math_helpers.h"
#include "crx_types.h"

namespace crx {

inline auto PoseArrayToIsometry(const real_T pose[kPoseElementCount])
    -> PoseIsoRT {
  PoseIsoRT out;
  out.matrix() = PoseMapConstRT(pose);
  return out;
}

inline void IsometryToPoseArray(const PoseIsoRT &iso,
                                real_T pose[kPoseElementCount]) {
  PoseMapRT pose_map(pose);
  pose_map = iso.matrix();
}

inline auto XYZWPR_ToIsometry(const real_T xyzwpr[6]) -> PoseIsoRT {
  using AA = Eigen::AngleAxis<real_T>;
  PoseIsoRT T = PoseIsoRT::Identity();
  T.linear() = (AA(xyzwpr[5], Eigen::Matrix<real_T, 3, 1>::UnitZ()) *
                AA(xyzwpr[4], Eigen::Matrix<real_T, 3, 1>::UnitY()) *
                AA(xyzwpr[3], Eigen::Matrix<real_T, 3, 1>::UnitX()))
                   .toRotationMatrix();
  T.translation() << xyzwpr[0], xyzwpr[1], xyzwpr[2];
  return T;
}

inline auto DHM_FromRad(real_T alpha, real_T a, real_T theta, real_T d)
    -> PoseIsoRT {
  double alpha_sin = 0.0;
  double alpha_cos = 1.0;
  SinCos(static_cast<double>(alpha), alpha_sin, alpha_cos);
  const real_T sa = static_cast<real_T>(alpha_sin);
  const real_T ca = static_cast<real_T>(alpha_cos);

  double theta_sin = 0.0;
  double theta_cos = 1.0;
  SinCos(static_cast<double>(theta), theta_sin, theta_cos);
  const real_T st = static_cast<real_T>(theta_sin);
  const real_T ct = static_cast<real_T>(theta_cos);

  PoseIsoRT T = PoseIsoRT::Identity();
  T.linear() << ct, -st, 0.0, st * ca, ct * ca, -sa, st * sa, ct * sa, ca;
  T.translation() << a, -d * sa, d * ca;
  return T;
}

inline auto FixedJ6ToToolIsometryFk() -> PoseIsoRT {
  return PoseIsoRT::Identity();
}

} // namespace crx
