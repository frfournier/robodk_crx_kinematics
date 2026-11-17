// SPDX-License-Identifier: Apache-2.0
// SPDX-FileCopyrightText: 2026 Centre Technologique en Aérospatiale

#pragma once

#include <limits>

// Private binary64 enclosure primitives. Invalid intervals are failures, never
// empty-set/exclusion certificates. Arithmetic does not change caller FP modes.
namespace crx::interval {

struct Interval {
  double lo = std::numeric_limits<double>::quiet_NaN();
  double hi = std::numeric_limits<double>::quiet_NaN();

  constexpr Interval() = default;

  explicit constexpr Interval(double point) : lo(point), hi(point) {}

  constexpr Interval(double lower, double upper) : lo(lower), hi(upper) {}

  auto IsValid() const -> bool;
  auto ContainsZero() const -> bool;
  auto IsPoint() const -> bool;
};

// IEEE binary64, nearest rounding, masked exceptions, gradual underflow. The
// implementation is compiled strictly without contraction, reassociation or
// cross-TU LTO. Unsupported execution modes return invalid arithmetic results.
auto EnvironmentSupported() -> bool;

auto operator+(Interval a, Interval b) -> Interval;
auto operator-(Interval a, Interval b) -> Interval;
auto operator-(Interval a) -> Interval;
auto operator*(Interval a, Interval b) -> Interval;
auto operator/(Interval a, Interval b) -> Interval;

inline auto operator+=(Interval &a, Interval b) -> Interval & {
  a = a + b;
  return a;
}

inline auto operator-=(Interval &a, Interval b) -> Interval & {
  a = a - b;
  return a;
}

inline auto operator*=(Interval &a, Interval b) -> Interval & {
  a = a * b;
  return a;
}

inline auto operator/=(Interval &a, Interval b) -> Interval & {
  a = a / b;
  return a;
}

auto Square(Interval a) -> Interval;
auto Sqrt(Interval a) -> Interval;
auto Abs(Interval a) -> Interval;
auto Hull(Interval a, Interval b) -> Interval;

// Proven binary64 neighbours of mathematical pi, not a point approximation.
auto Pi() -> Interval;

struct SinCos {
  Interval sine;
  Interval cosine;
  // Broad or very large arguments conservatively return [-1,1]. This is a
  // sound enclosure, not a successful narrow range reduction.
  bool used_full_range = false;
};

// No dependence on libm sin/cos accuracy. Integer quadrant reduction uses Pi;
// Taylor polynomials include explicit Lagrange remainder enclosures.
auto EvaluateSinCos(Interval angle) -> SinCos;

struct Phase {
  Interval principal;
  SinCos trigonometry;
  // A seam cannot be represented by one narrow connected principal interval.
  // Preserve the complete [-pi,pi] enclosure instead of choosing a side.
  bool principal_seam_uncertain = false;
};

// Retains the exact represented raw angle through interval period reduction.
// In particular, an ordinary double remainder is not an exact encoder zero.
auto EnclosePhase(Interval raw_angle) -> Phase;

} // namespace crx::interval
