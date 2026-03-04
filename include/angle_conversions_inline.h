#pragma once

namespace angle_conv {

inline constexpr double kPi = 3.141592653589793238462643383279502884;
inline constexpr double kRadPerDeg = kPi / 180.0;
inline constexpr double kDegPerRad = 180.0 / kPi;

inline constexpr double DegToRad(double deg) {
    return deg * kRadPerDeg;
}

inline constexpr double RadToDeg(double rad) {
    return rad * kDegPerRad;
}

} // namespace angle_conv
