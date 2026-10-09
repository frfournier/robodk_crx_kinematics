#include "crx_vector_helpers.h"

#include "crx_math_helpers.h"
#include "crx_types.h"

namespace crx {

auto UserToCommand(const Vec6 &user) -> Vec6 {
  Vec6 command = user;
  command[kJoint3Index] += command[kJoint2Index];
  return command;
}

auto CommandToUser(const Vec6 &command) -> Vec6 {
  Vec6 user = command;
  user[kJoint3Index] -= user[kJoint2Index];
  return user;
}

void NormalizeVecKeepSignedPi(Vec6 &q) {
  q = q.unaryExpr([](double x) { return NormalizeRadKeepSignedPi(x); });
}

void NormalizeUserSolutionDomains(Vec6 &q) {
  const double joint3_raw = q[kJoint3Index];
  NormalizeVecKeepSignedPi(q);
  q[kJoint3Index] = joint3_raw;
}

auto ClampToLimits(Vec6 &q, const Vec6 &lo, const Vec6 &hi, double tol_rad)
    -> bool {
  const auto out_of_bounds = ((q.array() < (lo.array() - tol_rad)) ||
                              (q.array() > (hi.array() + tol_rad)))
                                 .any();
  if (out_of_bounds)
    return false;
  q = q.cwiseMax(lo).cwiseMin(hi);
  return true;
}

auto WrappedDist2Rad(const Vec6 &a, const Vec6 &b) -> double {
  return (a - b).unaryExpr([](double x) { return WrapRadPi(x); }).squaredNorm();
}

auto MaxAbsDiffRadDirect(const Vec6 &a, const Vec6 &b) -> double {
  return (a - b).cwiseAbs().maxCoeff();
}

} // namespace crx
