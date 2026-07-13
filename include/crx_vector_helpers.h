#pragma once

#include "crx_math_helpers.h"
#include "crx_types.h"

namespace crx {

inline void NormalizeVecKeepSignedPi(Vec6 &q) {
  q = q.unaryExpr([](double x) { return NormalizeRadKeepSignedPi(x); });
}

inline void NormalizeUserSolutionDomains(Vec6 &q) {
  const double joint3_raw = q[kJoint3Index];
  q = q.unaryExpr([](double x) { return NormalizeRadKeepSignedPi(x); });
  q[kJoint3Index] = joint3_raw;
}

inline auto ClampToLimits(Vec6 &q, const Vec6 &lo, const Vec6 &hi,
                          double tol_rad) -> bool {
  const auto out_of_bounds = ((q.array() < (lo.array() - tol_rad)) ||
                              (q.array() > (hi.array() + tol_rad)))
                                 .any();
  if (out_of_bounds)
    return false;
  q = q.cwiseMax(lo).cwiseMin(hi);
  return true;
}

inline auto WrappedDist2Rad(const Vec6 &a, const Vec6 &b) -> double {
  return (a - b).unaryExpr([](double x) { return WrapRadPi(x); }).squaredNorm();
}

inline auto MaxAbsDiffRadDirect(const Vec6 &a, const Vec6 &b) -> double {
  return (a - b).cwiseAbs().maxCoeff();
}

} // namespace crx
