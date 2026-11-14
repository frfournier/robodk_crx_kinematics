// SPDX-License-Identifier: Apache-2.0
// SPDX-FileCopyrightText: 2026 Centre Technologique en Aérospatiale

#include "crx_interval.h"

#include <algorithm>
#include <array>
#include <cfenv>
#include <cfloat>
#include <cmath>
#include <cstdint>
#include <limits>

#include <emmintrin.h>
#include <xmmintrin.h>

#ifdef __FAST_MATH__
#error "Enclosures require strict floating-point compilation"
#endif

static_assert(FLT_EVAL_METHOD == 0,
              "Excess intermediate precision unsupported");
constexpr int kBinary64Bytes = 8;
constexpr int kBinary64Precision = 53;
static_assert(sizeof(double) == kBinary64Bytes &&
                  std::numeric_limits<double>::is_iec559 &&
                  std::numeric_limits<double>::digits == kBinary64Precision,
              "Enclosures require IEEE binary64");

namespace crx::interval {
namespace {
constexpr double kInfinity = std::numeric_limits<double>::infinity();
constexpr double kPiLower = 0x1.921fb54442d18p+1;
constexpr double kPiUpper = 0x1.921fb54442d19p+1;
constexpr double kReductionLimit = 0x1p20;
constexpr double kMaximumReductionWidth = 3.0;
constexpr std::int64_t kQuarterTurnsPerPeriod = 4;
constexpr std::uint32_t kDaz = 0x0040;
constexpr std::uint32_t kFtz = 0x8000;
constexpr std::uint32_t kRounding = 0x6000;
constexpr std::uint32_t kExceptionMasks = 0x1f80;

auto Down(double value) -> double { return std::nextafter(value, -kInfinity); }

auto Up(double value) -> double { return std::nextafter(value, kInfinity); }

auto Checked(double lower, double upper) -> Interval {
  const Interval result{lower, upper};
  return result.IsValid() ? result : Interval{};
}

auto Usable(Interval a, Interval b) -> bool {
  return EnvironmentSupported() && a.IsValid() && b.IsValid();
}

auto IsZero(Interval value) -> bool { return value.lo == 0 && value.hi == 0; }

auto FullTrigonometry() -> SinCos { return {{-1, 1}, {-1, 1}, true}; }

auto ClampUnit(Interval value) -> Interval {
  return value.IsValid()
             ? Checked(std::max(-1.0, value.lo), std::min(1.0, value.hi))
             : Interval{};
}

// Every denominator below is an exactly represented small integer. Taylor
// degree 24 for sine (the even coefficient is zero), 23 for cosine, leaves
// remainders <= |x|^25/25! and |x|^24/24!. No alternating-series assumption.
auto TaylorSinCos(Interval x) -> SinCos {
  const Interval negative_square = -Square(x);
  Interval sine = x;
  Interval cosine{1.0};
  Interval sine_term = x;
  Interval cosine_term{1.0};
  constexpr int kTerms = 11;
  constexpr int kCosineRemainderOrder = (2 * kTerms) + 2;
  constexpr int kSineRemainderOrder = (2 * kTerms) + 3;
  for (int k = 1; k <= kTerms; ++k) {
    sine_term = (sine_term * negative_square) /
                Interval{static_cast<double>((2 * k) * ((2 * k) + 1))};
    cosine_term = (cosine_term * negative_square) /
                  Interval{static_cast<double>(((2 * k) - 1) * (2 * k))};
    sine = sine + sine_term;
    cosine = cosine + cosine_term;
  }
  const Interval magnitude{std::max(std::abs(x.lo), std::abs(x.hi))};
  Interval remainder{1.0};
  for (int k = 1; k <= kCosineRemainderOrder; ++k) {
    remainder = (remainder * magnitude) / Interval{static_cast<double>(k)};
  }
  if (!remainder.IsValid()) {
    return {};
  }
  cosine = cosine + Interval{-remainder.hi, remainder.hi};
  remainder = (remainder * magnitude) /
              Interval{static_cast<double>(kSineRemainderOrder)};
  if (!remainder.IsValid()) {
    return {};
  }
  sine = sine + Interval{-remainder.hi, remainder.hi};
  return {ClampUnit(sine), ClampUnit(cosine), false};
}

auto ReductionCentre(Interval angle, double period) -> double {
  // Used only to choose an integer, never as an enclosure endpoint. Any chosen
  // integer is algebraically valid; the reduced interval is checked afterward.
  const double midpoint = angle.lo + ((angle.hi - angle.lo) / 2);
  return std::nearbyint(midpoint / period);
}
} // namespace

auto EnvironmentSupported() -> bool {
  const auto control = _mm_getcsr();
  return (control & (kDaz | kFtz | kRounding)) == 0 &&
         (control & kExceptionMasks) == kExceptionMasks &&
         std::fegetround() == FE_TONEAREST;
}

auto Interval::IsValid() const -> bool {
  return std::isfinite(lo) && std::isfinite(hi) && lo <= hi;
}

auto Interval::ContainsZero() const -> bool {
  return IsValid() && lo <= 0 && hi >= 0;
}

auto Interval::IsPoint() const -> bool { return IsValid() && lo == hi; }

auto operator+(Interval a, Interval b) -> Interval {
  if (!Usable(a, b)) {
    return {};
  }
  if (IsZero(a)) {
    return b;
  }
  if (IsZero(b)) {
    return a;
  }
  return Checked(Down(a.lo + b.lo), Up(a.hi + b.hi));
}

auto operator-(Interval a) -> Interval {
  return EnvironmentSupported() && a.IsValid() ? Interval{-a.hi, -a.lo}
                                               : Interval{};
}

auto operator-(Interval a, Interval b) -> Interval {
  if (!Usable(a, b)) {
    return {};
  }
  if (IsZero(b)) {
    return a;
  }
  return Checked(Down(a.lo - b.hi), Up(a.hi - b.lo));
}

auto operator*(Interval a, Interval b) -> Interval {
  if (!Usable(a, b)) {
    return {};
  }
  if (IsZero(a) || IsZero(b)) {
    return Interval{0.0};
  }
  const std::array<double, 4> endpoints{a.lo * b.lo, a.lo * b.hi, a.hi * b.lo,
                                        a.hi * b.hi};
  const auto bounds = std::minmax_element(endpoints.begin(), endpoints.end());
  return Checked(Down(*bounds.first), Up(*bounds.second));
}

auto operator/(Interval a, Interval b) -> Interval {
  if (!Usable(a, b) || b.ContainsZero()) {
    return {};
  }
  if (IsZero(a)) {
    return Interval{0.0};
  }
  const std::array<double, 4> endpoints{a.lo / b.lo, a.lo / b.hi, a.hi / b.lo,
                                        a.hi / b.hi};
  const auto bounds = std::minmax_element(endpoints.begin(), endpoints.end());
  return Checked(Down(*bounds.first), Up(*bounds.second));
}

auto Square(Interval a) -> Interval {
  if (!EnvironmentSupported() || !a.IsValid()) {
    return {};
  }
  if (IsZero(a)) {
    return Interval{0.0};
  }
  const double first = a.lo * a.lo;
  const double second = a.hi * a.hi;
  const double lower =
      a.ContainsZero() ? 0.0 : std::max(0.0, Down(std::min(first, second)));
  return Checked(lower, Up(std::max(first, second)));
}

auto Sqrt(Interval a) -> Interval {
  if (!EnvironmentSupported() || !a.IsValid() || a.lo < 0) {
    return {};
  }
  if (IsZero(a)) {
    return Interval{0.0};
  }
  // Use the IEEE SSE2 primitive explicitly, not a libm error assumption.
  const double lower =
      _mm_cvtsd_f64(_mm_sqrt_sd(_mm_setzero_pd(), _mm_set_sd(a.lo)));
  const double upper =
      _mm_cvtsd_f64(_mm_sqrt_sd(_mm_setzero_pd(), _mm_set_sd(a.hi)));
  return Checked(std::max(0.0, Down(lower)), Up(upper));
}

auto Abs(Interval a) -> Interval {
  if (!EnvironmentSupported() || !a.IsValid()) {
    return {};
  }
  const double lower =
      a.ContainsZero() ? 0.0 : std::min(std::abs(a.lo), std::abs(a.hi));
  return {lower, std::max(std::abs(a.lo), std::abs(a.hi))};
}

auto Hull(Interval a, Interval b) -> Interval {
  return Usable(a, b) ? Interval{std::min(a.lo, b.lo), std::max(a.hi, b.hi)}
                      : Interval{};
}

auto Pi() -> Interval { return {kPiLower, kPiUpper}; }

auto EvaluateSinCos(Interval angle) -> SinCos {
  if (!EnvironmentSupported() || !angle.IsValid()) {
    return {};
  }
  if (std::max(std::abs(angle.lo), std::abs(angle.hi)) > kReductionLimit ||
      angle.hi - angle.lo > kMaximumReductionWidth) {
    return FullTrigonometry();
  }
  const double multiple = ReductionCentre(angle, kPiLower / 2);
  const Interval reduced =
      angle - (Interval{multiple} * (Pi() / Interval{2.0}));
  if (!reduced.IsValid() || reduced.lo < -2 || reduced.hi > 2) {
    return FullTrigonometry();
  }
  const auto evaluated = TaylorSinCos(reduced);
  const auto quadrant =
      ((static_cast<std::int64_t>(multiple) % kQuarterTurnsPerPeriod) +
       kQuarterTurnsPerPeriod) %
      kQuarterTurnsPerPeriod;
  switch (quadrant) {
  case 0:
    return evaluated;
  case 1:
    return {evaluated.cosine, -evaluated.sine, false};
  case 2:
    return {-evaluated.sine, -evaluated.cosine, false};
  default:
    return {-evaluated.cosine, evaluated.sine, false};
  }
}

auto EnclosePhase(Interval raw_angle) -> Phase {
  Phase result;
  result.trigonometry = EvaluateSinCos(raw_angle);
  if (!result.trigonometry.sine.IsValid() ||
      !result.trigonometry.cosine.IsValid()) {
    return result;
  }
  result.principal = {-kPiUpper, kPiUpper};
  result.principal_seam_uncertain = true;
  if (std::max(std::abs(raw_angle.lo), std::abs(raw_angle.hi)) <=
          kReductionLimit &&
      raw_angle.hi - raw_angle.lo <= kMaximumReductionWidth) {
    const double multiple = ReductionCentre(raw_angle, 2 * kPiLower);
    const Interval reduced =
        raw_angle - (Interval{multiple} * (Interval{2.0} * Pi()));
    if (reduced.IsValid() && reduced.lo >= -kPiLower &&
        reduced.hi <= kPiLower) {
      result.principal = reduced;
      result.principal_seam_uncertain = false;
    }
  }
  return result;
}
} // namespace crx::interval
