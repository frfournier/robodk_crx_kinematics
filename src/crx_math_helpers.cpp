#include "crx_math_helpers.h"
#include "crx_numerics.h"

#include <array>
#include <cmath>

namespace crx {

auto NormalizeRadKeepSignedPi(double a) -> double {
  // fmod is exactly the input inside one turn, including signed zero.
  double w = (a > -kTwoPi && a < kTwoPi) ? a : std::fmod(a, kTwoPi);
  if (w > angle_conv::kPi)
    w -= kTwoPi;
  else if (w < -angle_conv::kPi)
    w += kTwoPi;
  return w;
}

auto WrapRadPi(double a) -> double { return NormalizeRadKeepSignedPi(a); }

auto SnapToRightAngleFamily(double a, double tol) -> double {
  static const std::array<double, 5> refs = {-angle_conv::kPi, -kHalfPi, 0.0,
                                             kHalfPi, angle_conv::kPi};
  for (const double r : refs)
    if (std::abs(WrapRadPi(a - r)) <= tol)
      return r;
  return a;
}

auto AngleDiffAbs(double a, double b) -> double {
  return std::abs(WrapRadPi(a - b));
}

auto ApplyFanucJointCoupling(const Vec6 &decoupled) -> Vec6 {
  Vec6 coupled = decoupled;
  coupled[kJoint3Index] += coupled[kJoint2Index];
  return coupled;
}

auto RemoveFanucJointCoupling(const Vec6 &coupled) -> Vec6 {
  Vec6 decoupled = coupled;
  decoupled[kJoint3Index] -= decoupled[kJoint2Index];
  return decoupled;
}

void NormalizeVecKeepSignedPi(Vec6 &q) {
  q = q.unaryExpr([](double x) { return NormalizeRadKeepSignedPi(x); });
}

void NormalizeUserSolutionDomains(Vec6 &q) {
  const double joint3_raw = q[kJoint3Index];
  NormalizeVecKeepSignedPi(q);
  q[kJoint3Index] = joint3_raw;
}

} // namespace crx
