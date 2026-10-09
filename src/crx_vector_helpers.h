#pragma once

#include "crx_types.h"

namespace crx {

// Radians before joint senses: command J3 = user J2 + decoupled user J3.
// These conversions neither normalize turns nor apply command limits.
auto UserToCommand(const Vec6 &user) -> Vec6;
auto CommandToUser(const Vec6 &command) -> Vec6;

void NormalizeVecKeepSignedPi(Vec6 &q);

void NormalizeUserSolutionDomains(Vec6 &q);

auto ClampToLimits(Vec6 &q, const Vec6 &lo, const Vec6 &hi, double tol_rad)
    -> bool;
auto WrappedDist2Rad(const Vec6 &a, const Vec6 &b) -> double;
auto MaxAbsDiffRadDirect(const Vec6 &a, const Vec6 &b) -> double;

} // namespace crx
