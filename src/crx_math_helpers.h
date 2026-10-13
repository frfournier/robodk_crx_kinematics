#pragma once

#include "crx_numerics.h"
#include "crx_types.h"

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

// Fanuc J2/J3 coupling in radians, before joint senses are applied.
// Apply adds J2 to J3; remove subtracts J2 from J3.
// These conversions neither normalize turns nor apply command limits.
auto ApplyFanucJointCoupling(const Vec6 &decoupled) -> Vec6;
auto RemoveFanucJointCoupling(const Vec6 &coupled) -> Vec6;

void NormalizeVecKeepSignedPi(Vec6 &q);

void NormalizeUserSolutionDomains(Vec6 &q);

auto DHM_FromRad(Scalar alpha, Scalar a, Scalar theta, Scalar d) -> PoseIsoRT;

} // namespace crx
