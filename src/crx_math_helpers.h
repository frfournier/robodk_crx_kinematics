#pragma once

#include "crx_numerics.h"

namespace crx {

void SinCos(double angle_rad, double &sin_out, double &cos_out);

auto WrapRad2Pi(double a) -> double;

auto NormalizeRadKeepSignedPi(double a) -> double;

auto WrapRadPi(double a) -> double;

auto ClampCosineNearUnit(double value) -> double;

auto SnapToRightAngleFamily(double a,
                            double tol = numerics::kRightAngleSnapToleranceRad)
    -> double;

auto AngleDiffAbs(double a, double b) -> double;

} // namespace crx
