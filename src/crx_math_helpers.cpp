#include "crx_math_helpers.h"
#include "crx_numerics.h"

#include <array>
#include <cmath>

namespace crx {

void SinCos(double angle_rad, double &sin_out, double &cos_out) {
#if defined(__has_builtin)
#if __has_builtin(__builtin_sincos)
  __builtin_sincos(angle_rad, &sin_out, &cos_out);
  return;
#endif
#endif
  sin_out = std::sin(angle_rad);
  cos_out = std::cos(angle_rad);
}

auto WrapRad2Pi(double a) -> double {
  const double w = std::fmod(a, kTwoPi);
  return (w < 0.0) ? w + kTwoPi : w;
}

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

auto ClampCosineNearUnit(double value) -> double {
  if (!std::isfinite(value))
    return value;
  if (value > 1.0)
    return (value <= 1.0 + numerics::kCosineClampTolerance) ? 1.0 : value;
  if (value < -1.0)
    return (value >= -1.0 - numerics::kCosineClampTolerance) ? -1.0 : value;
  return value;
}

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

auto DHM_FromRad(Scalar alpha, Scalar a, Scalar theta, Scalar d) -> PoseIsoRT {
  // Modified DH: Tx(a) Rx(alpha) Tz(d) Rz(theta).
  const Scalar sa = std::sin(alpha);
  const Scalar ca = std::cos(alpha);
  const Scalar st = std::sin(theta);
  const Scalar ct = std::cos(theta);
  PoseIsoRT transform;
  transform.matrix() << ct, -st, 0.0, a, //
      ca * st, ca * ct, -sa, -d * sa,    //
      sa * st, sa * ct, ca, d * ca,      //
      0.0, 0.0, 0.0, 1.0;
  return transform;
}

} // namespace crx
