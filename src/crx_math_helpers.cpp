#include "crx_math_helpers.h"

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
  double w = std::fmod(a, kTwoPi);
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
    return (value <= 1.0 + kCosineClampTolerance) ? 1.0 : value;
  if (value < -1.0)
    return (value >= -1.0 - kCosineClampTolerance) ? -1.0 : value;
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

} // namespace crx
