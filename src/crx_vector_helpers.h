#pragma once

#include "crx_types.h"

namespace crx {

// Radians before joint senses: command J3 = user J2 + decoupled user J3.
// These conversions neither normalize turns nor apply command limits.
auto UserToCommand(const Vec6 &user) -> Vec6;
auto CommandToUser(const Vec6 &command) -> Vec6;

void NormalizeVecKeepSignedPi(Vec6 &q);

void NormalizeUserSolutionDomains(Vec6 &q);

} // namespace crx
