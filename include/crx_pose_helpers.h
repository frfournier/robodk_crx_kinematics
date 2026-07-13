#pragma once

#include "crx_math_helpers.h"
#include "crx_types.h"

namespace crx {

inline auto DHM_FromRad(Scalar alpha, Scalar a, Scalar theta, Scalar d)
    -> PoseIsoRT {
  double alpha_sin = 0.0;
  double alpha_cos = 1.0;
  SinCos(static_cast<double>(alpha), alpha_sin, alpha_cos);
  const Scalar sa = static_cast<Scalar>(alpha_sin);
  const Scalar ca = static_cast<Scalar>(alpha_cos);

  double theta_sin = 0.0;
  double theta_cos = 1.0;
  SinCos(static_cast<double>(theta), theta_sin, theta_cos);
  const Scalar st = static_cast<Scalar>(theta_sin);
  const Scalar ct = static_cast<Scalar>(theta_cos);

  PoseIsoRT T = PoseIsoRT::Identity();
  T.linear() << ct, -st, 0.0, st * ca, ct * ca, -sa, st * sa, ct * sa, ca;
  T.translation() << a, -d * sa, d * ca;
  return T;
}

inline auto FixedJ6ToToolIsometryFk() -> PoseIsoRT {
  return PoseIsoRT::Identity();
}

} // namespace crx
