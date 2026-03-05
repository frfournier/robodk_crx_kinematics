#pragma once

#include <algorithm>

#include "crx_math_helpers.h"
#include "crx_types.h"

namespace crx {

inline void DegArrayToRadVec(const real_T *j_deg, Vec6 &out_rad) {
  out_rad = Eigen::Map<const Vec6>(j_deg) * angle_conv::DegToRad(1.0);
}

inline void RadVecToDegArray(const Vec6 &v, real_T *out) {
  Eigen::Map<Vec6r> out_map(out);
  out_map = (v * angle_conv::RadToDeg(1.0)).template cast<real_T>();
}

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
  return (a - b)
      .unaryExpr([](double x) { return WrapRadPi(x); })
      .squaredNorm();
}

inline auto MaxAbsDiffRadDirect(const Vec6 &a, const Vec6 &b) -> double {
  return (a - b).cwiseAbs().maxCoeff();
}

inline void StoreSolution(real_T *all_solutions_deg, int solution_id,
                          const Vec6 &solution_user_rad) {
  real_T *solution_slot = all_solutions_deg + kSolutionStride * solution_id;
  std::fill(solution_slot, solution_slot + kSolutionStride,
            static_cast<real_T>(0.0));
  RadVecToDegArray(solution_user_rad, solution_slot);
}

} // namespace crx
