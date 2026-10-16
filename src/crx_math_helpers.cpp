#include "crx_math_helpers.h"
#include "crx_numerics.h"

#include <cmath>

namespace crx {

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

auto AngleDiffAbs(double a, double b) -> double {
  return std::abs(WrapRadPi(a - b));
}

} // namespace crx
