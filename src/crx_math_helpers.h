#pragma once

namespace angle_conv {

inline constexpr double kPi = 3.141592653589793238462643383279502884;
inline constexpr double kRadPerDeg = kPi / 180.0;
inline constexpr double kDegPerRad = 180.0 / kPi;

inline constexpr double DegToRad(double deg) { return deg * kRadPerDeg; }

inline constexpr double RadToDeg(double rad) { return rad * kDegPerRad; }

} // namespace angle_conv

namespace crx {

inline constexpr double kTwoPi = 2.0 * angle_conv::kPi;
inline constexpr double kHalfPi = 0.5 * angle_conv::kPi;
inline constexpr double kEpsilon = 1e-12;
inline constexpr double kCosineClampTolerance = 1.0e-9;
inline constexpr double kRightAngleSnapToleranceRad = 1e-5;

void SinCos(double angle_rad, double &sin_out, double &cos_out);

auto WrapRad2Pi(double a) -> double;

auto NormalizeRadKeepSignedPi(double a) -> double;

auto WrapRadPi(double a) -> double;

auto ClampCosineNearUnit(double value) -> double;

auto SnapToRightAngleFamily(double a, double tol = kRightAngleSnapToleranceRad)
    -> double;

auto AngleDiffAbs(double a, double b) -> double;

} // namespace crx
