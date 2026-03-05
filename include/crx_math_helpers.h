#pragma once

#define _USE_MATH_DEFINES
#include <array>
#include <cmath>

#include "crx_types.h"

#ifndef M_PI
#define M_PI 3.141592653589793238462643383279502884
#endif

#ifndef M_PI_2
#define M_PI_2 (0.5 * M_PI)
#endif

#ifndef M_2PI
#define M_2PI (2.0 * M_PI)
#endif

namespace angle_conv {

inline constexpr double kPi = M_PI;
inline constexpr double kRadPerDeg = kPi / 180.0;
inline constexpr double kDegPerRad = 180.0 / kPi;

inline constexpr double DegToRad(double deg) { return deg * kRadPerDeg; }

inline constexpr double RadToDeg(double rad) { return rad * kDegPerRad; }

} // namespace angle_conv

namespace crx {

inline constexpr double kTwoPi = M_2PI;
inline constexpr double kHalfPi = M_PI_2;
inline constexpr double kEpsilon = 1e-12;
inline constexpr double kCosineClampTolerance = 1.0e-9;
inline constexpr double kRightAngleSnapToleranceRad = 1e-5;

inline void SinCos(double angle_rad, double &sin_out, double &cos_out) {
#if defined(__has_builtin)
#if __has_builtin(__builtin_sincos)
  __builtin_sincos(angle_rad, &sin_out, &cos_out);
  return;
#endif
#endif
  sin_out = std::sin(angle_rad);
  cos_out = std::cos(angle_rad);
}

inline auto WrapRad2Pi(double a) -> double {
  const double w = std::fmod(a, kTwoPi);
  return (w < 0.0) ? w + kTwoPi : w;
}

inline auto NormalizeRadKeepSignedPi(double a) -> double {
  double w = std::fmod(a, kTwoPi);
  if (w > M_PI)
    w -= kTwoPi;
  else if (w < -M_PI)
    w += kTwoPi;
  return w;
}

inline auto WrapRadPi(double a) -> double { return NormalizeRadKeepSignedPi(a); }

inline auto ClampCosineNearUnit(double value) -> double {
  if (!std::isfinite(value))
    return value;
  if (value > 1.0)
    return (value <= 1.0 + kCosineClampTolerance) ? 1.0 : value;
  if (value < -1.0)
    return (value >= -1.0 - kCosineClampTolerance) ? -1.0 : value;
  return value;
}

inline auto
SnapToRightAngleFamily(double a, double tol = kRightAngleSnapToleranceRad)
    -> double {
  static const std::array<double, 5> refs = {-M_PI, -kHalfPi, 0.0, kHalfPi,
                                             M_PI};
  for (const double r : refs)
    if (std::abs(WrapRadPi(a - r)) <= tol)
      return r;
  return a;
}

inline auto AngleDiffAbs(double a, double b) -> double {
  return std::abs(WrapRadPi(a - b));
}

inline auto NormalizeJointSense(real_T s) -> double {
  if (!std::isfinite(s) || std::abs(s) <= kEpsilon)
    return 1.0;
  return (s >= 0.0) ? 1.0 : -1.0;
}

} // namespace crx
