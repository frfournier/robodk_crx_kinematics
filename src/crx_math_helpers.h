#pragma once

#include "crx_numerics.h"
#include "crx_types.h"

namespace crx {

auto NormalizeRadKeepSignedPi(double a) -> double;

auto WrapRadPi(double a) -> double;

auto AngleDiffAbs(double a, double b) -> double;

} // namespace crx
