// SPDX-License-Identifier: Apache-2.0
// SPDX-FileCopyrightText: 2026 Centre Technologique en Aérospatiale

#include "crx_phase_catalogue.h"
#include "crx_numerics.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <utility>

#ifdef __FAST_MATH__
#error "Catalogue enclosures require strict floating-point compilation"
#endif

namespace crx::catalogue {
auto CircleDistance(double a, double b) -> interval::Interval {
  using interval::Interval;
  if (!interval::EnvironmentSupported() || !std::isfinite(a) ||
      !std::isfinite(b))
    return {};
  const Interval difference = Interval{a} - Interval{b};
  if (!difference.IsValid())
    return {};
  const double proposal =
      std::nearbyint(((difference.lo / 2) + (difference.hi / 2)) / kTwoPi);
  constexpr double kIntegerLimit = 0x1p40;
  if (!std::isfinite(proposal) || std::abs(proposal) > kIntegerLimit)
    return {};
  const auto integer = static_cast<std::int64_t>(proposal);
  const Interval period = Interval{2} * interval::Pi();
  const Interval residual =
      difference - period * Interval{static_cast<double>(integer)};
  const Interval range = Interval{1.5} * interval::Pi();
  const Interval magnitude = interval::Abs(residual);
  if (!magnitude.IsValid() || !range.IsValid() || magnitude.hi >= range.lo)
    return {};
  // |residual|<3pi/2. Other relative integers |j|>=2 are at distance
  // >=4pi-3pi/2=5pi/2, greater than the available j=0 candidate. Therefore
  // the exact minimum is among integer-1,integer,integer+1, even at a pi seam.
  double lower = interval::Pi().hi, upper = interval::Pi().hi;
  for (std::int64_t j = -1; j <= 1; ++j) {
    const Interval candidate = interval::Abs(
        difference - period * Interval{static_cast<double>(integer + j)});
    if (!candidate.IsValid())
      return {};
    lower = std::min(lower, candidate.lo);
    upper = std::min(upper, candidate.hi);
  }
  const Interval result{std::max(0.0, lower), upper};
  return result.IsValid() ? result : Interval{};
}

auto Assemble(const exact::Problem &problem, root::Certificate root_cover,
              std::vector<ball::Certificate> balls,
              std::uint32_t max_pair_tests) -> PhaseCatalogue {
  PhaseCatalogue result;
  auto &data = result.data_;
  data.root_cover = std::move(root_cover);
  data.balls = std::move(balls);
  if (!interval::EnvironmentSupported()) {
    data.failure = Failure::UnsupportedEnvironment;
    return result;
  }
  if (!problem.IsReady())
    return result;
  data.identity = problem.Data().identity;
  if (!data.root_cover.Matches(problem)) {
    data.failure = Failure::IdentityMismatch;
    return result;
  }
  if (!data.root_cover.IsComplete()) {
    data.failure = Failure::IncompleteRootCover;
    return result;
  }
  for (const auto &certificate : data.balls) {
    if (!certificate.IsValid()) {
      data.failure = Failure::InvalidBall;
      return result;
    }
    if (!certificate.Matches(problem)) {
      data.failure = Failure::IdentityMismatch;
      return result;
    }
  }
  data.expected_phases = 2 * data.root_cover.Values().simple_roots;
  if (data.balls.size() != data.expected_phases) {
    data.failure = Failure::CountMismatch;
    return result;
  }
  for (std::size_t i = 0; i < data.balls.size(); ++i) {
    for (std::size_t j = 0; j < i; ++j) {
      data.failed_first = i;
      data.failed_second = j;
      if (data.pair_tests >= max_pair_tests) {
        data.failure = Failure::PairWorkLimit;
        return result;
      }
      ++data.pair_tests;
      const auto &a = data.balls.at(i).Values(), &b = data.balls.at(j).Values();
      const auto radii = interval::Interval{a.existence_radius} +
                         interval::Interval{b.existence_radius};
      if (!radii.IsValid()) {
        data.failure = Failure::PhaseRangeUnavailable;
        return result;
      }
      bool separated = false, available = false;
      for (std::size_t coordinate = 0; coordinate < a.centre.size();
           ++coordinate) {
        const auto distance =
            CircleDistance(a.centre.at(coordinate), b.centre.at(coordinate));
        if (!distance.IsValid())
          continue;
        available = true;
        if (distance.lo > radii.hi) {
          separated = true;
          break;
        }
      }
      if (!separated) {
        data.failure = available ? Failure::BallsNotSeparated
                                 : Failure::PhaseRangeUnavailable;
        return result;
      }
    }
  }
  data.failed_first = data.failed_second = 0;
  data.coverage = Coverage::Complete;
  data.failure = Failure::None;
  result.complete_ = true;
  return result;
}
} // namespace crx::catalogue
