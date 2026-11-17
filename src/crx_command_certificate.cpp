// SPDX-License-Identifier: Apache-2.0
// SPDX-FileCopyrightText: 2026 Centre Technologique en Aérospatiale

#include "crx_command_certificate.h"
#include "crx_ball_certificate.h"
#include "crx_exact_problem.h"
#include "crx_interval.h"
#include "crx_phase_catalogue.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
#include <utility>
#include <vector>

#ifdef __FAST_MATH__
#error "Command certificates require strict floating-point compilation"
#endif

namespace crx::command {
namespace {
using interval::Interval;
// A deliberately conservative exact-integer guard. Large raw offsets are not
// silently reduced to a rounded phase, and unavailable ranges stay unresolved.
constexpr double kMaximumTurn = 0x1p40;
constexpr std::uint64_t kMaximumWork = std::uint64_t{1} << 20;
constexpr std::size_t kAxes = 6;
constexpr std::size_t kRawFields = 4;

auto Valid(const Box &box) -> bool {
  return std::all_of(box.begin(), box.end(),
                     [](Interval x) { return x.IsValid(); });
}

auto BaseBox(const exact::Problem &problem, const ball::Data &ball,
             Profile profile) -> Box {
  Box centre;
  Box result;
  const auto &model = problem.Data();
  for (std::size_t i = 0; i < kAxes; ++i) {
    centre.at(i) = Interval{model.diagonal.at(i)} *
                   (Interval{ball.centre.at(i)} - model.offset_unwrapped.at(i));
  }
  if (profile == Profile::DecoupledUser) {
    centre.at(2) = centre.at(2) - centre.at(1);
  }
  for (std::size_t i = 0; i < kAxes; ++i) {
    const double row_norm = profile == Profile::DecoupledUser && i == 2 ? 2 : 1;
    const auto radius = Interval{row_norm} * Interval{ball.existence_radius};
    result.at(i) = centre.at(i) + Interval{-radius.hi, radius.hi};
  }
  return result;
}

auto Range(PhaseRange &range) -> bool {
  std::uint64_t count = 1;
  for (std::size_t i = 0; i < kAxes; ++i) {
    const auto width =
        static_cast<std::uint64_t>(range.last.at(i) - range.first.at(i)) + 1;
    if (count > std::numeric_limits<std::uint64_t>::max() / width) {
      return false;
    }
    count *= width;
  }
  range.count = count;
  return true;
}

auto PrepareRange(const Box &base, const ContextData &context,
                  PhaseRange &range) -> bool {
  range.base = base;
  if (!Valid(base)) {
    return false;
  }
  const auto period = Interval{2} * interval::Pi();
  for (std::size_t i = 0; i < kAxes; ++i) {
    const auto lower =
        (Interval{context.lower.at(i).lo} - Interval{base.at(i).hi}) / period;
    const auto upper =
        (Interval{context.upper.at(i).hi} - Interval{base.at(i).lo}) / period;
    if (!lower.IsValid() || !upper.IsValid()) {
      return false;
    }
    const double first = std::ceil(lower.lo);
    const double last = std::floor(upper.hi);
    if (first > last) {
      range.available = true;
      range.empty = true;
      return true;
    }
    if (std::abs(first) > kMaximumTurn || std::abs(last) > kMaximumTurn) {
      return false;
    }
    range.first.at(i) = static_cast<std::int64_t>(first);
    range.last.at(i) = static_cast<std::int64_t>(last);
  }
  range.available = Range(range);
  return range.available;
}

auto Classify(const Box &box, const ContextData &context) -> Admission {
  bool inside = true;
  for (std::size_t i = 0; i < kAxes; ++i) {
    const auto value = box.at(i);
    const auto lower = context.lower.at(i);
    const auto upper = context.upper.at(i);
    if (value.hi < lower.lo || value.lo > upper.hi) {
      return Admission::No;
    }
    inside = inside && value.lo >= lower.hi && value.hi <= upper.lo;
  }
  return inside ? Admission::Yes : Admission::Undecided;
}

auto Cost(const Box &box, const ContextData &context) -> Interval {
  Interval cost{0};
  for (std::size_t i = 0; i < kAxes; ++i) {
    cost += Interval{context.raw.weights.at(i)} *
            interval::Square(box.at(i) - context.seed.at(i));
  }
  // Positive sums may be widened below zero by outward rounding.
  return cost.IsValid() ? Interval{std::max(0.0, cost.lo), cost.hi}
                        : Interval{};
}

auto Next(std::array<std::int64_t, kAxes> &turns, const PhaseRange &range)
    -> bool {
  for (std::size_t i = kAxes; i-- > 0;) {
    if (turns.at(i) < range.last.at(i)) {
      ++turns.at(i);
      return true;
    }
    turns.at(i) = range.first.at(i);
  }
  return false;
}

auto DisplayLess(const Lift &a, const Lift &b, bool seeded) -> bool {
  if (seeded && a.proposed_travel_squared != b.proposed_travel_squared) {
    return a.proposed_travel_squared < b.proposed_travel_squared;
  }
  if (a.phase != b.phase) {
    return a.phase < b.phase;
  }
  return a.turns < b.turns;
}

auto MinimumAdmissibleUpper(const Data &data)
    -> std::pair<std::size_t, double> {
  double upper = std::numeric_limits<double>::infinity();
  std::size_t best = data.lifts.size();
  for (std::size_t i = 0; i < data.lifts.size(); ++i) {
    const auto &lift = data.lifts.at(i);
    if (lift.admission == Admission::Yes && lift.travel_squared.hi < upper) {
      upper = lift.travel_squared.hi;
      best = i;
    }
  }
  return {best, upper};
}

void FindPossibleMinimizers(Data &data, double upper) {
  for (std::size_t i = 0; i < data.lifts.size(); ++i) {
    if (data.lifts.at(i).travel_squared.lo <= upper) {
      data.possible_minimizers.push_back(i);
    }
  }
}

void Rank(Data &data) {
  const bool seeded = data.context.Values().raw.has_seed;
  // A numerical display order only. Interval separation is never used as a
  // sort comparator: that relation is not a strict weak order.
  std::stable_sort(data.lifts.begin(), data.lifts.end(),
                   [seeded](const Lift &a, const Lift &b) {
                     return DisplayLess(a, b, seeded);
                   });
  if (!seeded) {
    return;
  }
  data.ranking = Ranking::Inconclusive;
  if (data.admission_complete && data.yes == 0) {
    data.ranking = Ranking::NoAdmissible;
    return;
  }
  if (!std::all_of(data.lifts.begin(), data.lifts.end(), [](const Lift &lift) {
        return lift.travel_squared.IsValid();
      })) {
    return;
  }
  const auto [best, upper] = MinimumAdmissibleUpper(data);
  if (best == data.lifts.size()) {
    return;
  }
  FindPossibleMinimizers(data, upper);
  data.ranking = Ranking::PossibleMinimizers;
  if (data.possible_minimizers.size() == 1 &&
      data.possible_minimizers.front() == best) {
    data.ranking = Ranking::Unique;
    // Make the strict winner visible even for capacity one. Keep audit indices
    // consistent with the final order, independently of the proposal sort.
    std::rotate(data.lifts.begin(),
                data.lifts.begin() + static_cast<std::ptrdiff_t>(best),
                data.lifts.begin() + static_cast<std::ptrdiff_t>(best + 1));
    data.possible_minimizers.front() = 0;
  }
}

auto ValidInput(const Input &input) -> bool {
  if (input.profile != Profile::Callback &&
      input.profile != Profile::DecoupledUser) {
    return false;
  }
  for (std::size_t i = 0; i < kAxes; ++i) {
    if (!std::isfinite(input.lower_degrees.at(i)) ||
        !std::isfinite(input.upper_degrees.at(i)) ||
        input.lower_degrees.at(i) > input.upper_degrees.at(i) ||
        !std::isfinite(input.seed_degrees.at(i)) ||
        !std::isfinite(input.weights.at(i)) || input.weights.at(i) <= 0) {
      return false;
    }
  }
  return true;
}

void RetainRawWords(ContextData &data) {
  const std::array<std::array<double, kAxes>, kRawFields> fields{
      data.raw.lower_degrees, data.raw.upper_degrees, data.raw.seed_degrees,
      data.raw.weights};
  for (std::size_t field = 0; field < fields.size(); ++field) {
    for (std::size_t i = 0; i < kAxes; ++i) {
      const double value = fields.at(field).at(i);
      std::memcpy(&data.raw_words.at(field * kAxes + i), &value, sizeof(value));
    }
  }
}

auto Preflight(const exact::Problem &problem,
               const std::vector<ball::Certificate> &balls, Data &data)
    -> Failure {
  data.ranges.resize(balls.size());
  for (std::size_t phase = 0; phase < balls.size(); ++phase) {
    auto &range = data.ranges.at(phase);
    if (!PrepareRange(BaseBox(problem, balls.at(phase).Values(),
                              data.context.Values().raw.profile),
                      data.context.Values(), range)) {
      return Failure::RangeUnavailable;
    }
    if (range.count >
        std::numeric_limits<std::uint64_t>::max() - data.box_lifts) {
      return Failure::RangeUnavailable;
    }
    data.box_lifts += range.count;
  }
  return Failure::None;
}

auto RecordLift(Data &data, Lift lift) -> bool {
  if (!Valid(lift.radians)) {
    return false;
  }
  lift.admission = Classify(lift.radians, data.context.Values());
  if (lift.admission == Admission::No) {
    ++data.no;
    return true;
  }
  if (lift.admission == Admission::Yes) {
    ++data.yes;
  } else {
    ++data.undecided;
  }
  if (data.context.Values().raw.has_seed) {
    lift.travel_squared = Cost(lift.radians, data.context.Values());
    double proposal = 0;
    const auto &context = data.context.Values();
    for (std::size_t i = 0; i < kAxes; ++i) {
      const auto value = lift.radians.at(i);
      const auto seed = context.seed.at(i);
      const double difference =
          (value.lo / 2 + value.hi / 2) - (seed.lo / 2 + seed.hi / 2);
      proposal += context.raw.weights.at(i) * difference * difference;
    }
    lift.proposed_travel_squared =
        std::isfinite(proposal) ? proposal
                                : std::numeric_limits<double>::infinity();
  }
  data.lifts.push_back(std::move(lift));
  return true;
}

auto Enumerate(Data &data) -> bool {
  const auto period = Interval{2} * interval::Pi();
  for (std::size_t phase = 0; phase < data.ranges.size(); ++phase) {
    const auto &range = data.ranges.at(phase);
    if (range.empty) {
      continue;
    }
    auto turns = range.first;
    do {
      Lift lift;
      lift.phase = phase;
      lift.turns = turns;
      for (std::size_t i = 0; i < kAxes; ++i) {
        lift.radians.at(i) =
            range.base.at(i) +
            Interval{static_cast<double>(turns.at(i))} * period;
      }
      if (!RecordLift(data, std::move(lift))) {
        return false;
      }
    } while (Next(turns, range));
  }
  return true;
}
} // namespace

auto Prepare(const exact::Problem &problem, const Input &input, Context &output)
    -> Failure {
  if (!interval::EnvironmentSupported()) {
    return Failure::UnsupportedEnvironment;
  }
  if (!problem.IsReady() || !ValidInput(input)) {
    return Failure::InvalidInput;
  }
  Context prepared;
  auto &data = prepared.data_;
  data.problem_identity = problem.Data().identity;
  data.raw = input;
  const auto conversion = interval::Pi() / Interval{180};
  RetainRawWords(data);
  for (std::size_t i = 0; i < kAxes; ++i) {
    data.lower.at(i) = Interval{input.lower_degrees.at(i)} * conversion;
    data.upper.at(i) = Interval{input.upper_degrees.at(i)} * conversion;
    data.seed.at(i) = Interval{input.seed_degrees.at(i)} * conversion;
  }
  if (!Valid(data.lower) || !Valid(data.upper) || !Valid(data.seed)) {
    return Failure::ArithmeticUnavailable;
  }
  prepared.ready_ = true;
  output = std::move(prepared);
  return Failure::None;
}

auto Certify(const exact::Problem &problem,
             const catalogue::PhaseCatalogue &phases, const Context &context,
             Options options) -> Certificate {
  Certificate result;
  auto &data = result.data_;
  data.context = context;
  if (context.IsReady() && context.Values().raw.has_seed) {
    data.ranking = Ranking::Inconclusive;
  }
  if (!interval::EnvironmentSupported()) {
    data.failure = Failure::UnsupportedEnvironment;
    return result;
  }
  if (!problem.IsReady() || !context.IsReady() ||
      options.max_lifts > kMaximumWork) {
    return result;
  }
  if (!context.Matches(problem) || !phases.Matches(problem)) {
    data.failure = Failure::IdentityMismatch;
    return result;
  }
  data.phase_complete = phases.IsComplete();
  if (!data.phase_complete) {
    data.failure = Failure::IncompletePhaseCover;
    return result;
  }
  data.failure = Preflight(problem, phases.Values().balls, data);
  if (data.failure != Failure::None) {
    return result;
  }
  if (data.box_lifts > options.max_lifts) {
    data.failure = Failure::WorkLimit;
    return result;
  }
  if (!Enumerate(data)) {
    data.failure = Failure::ArithmeticUnavailable;
    data.lifts.clear();
    data.yes = data.no = data.undecided = 0;
    return result;
  }
  data.failure = Failure::None;
  data.enumeration_complete = true;
  data.admission_complete = data.undecided == 0;
  Rank(data);
  data.returned_count = std::min(options.capacity, data.lifts.size());
  data.truncated = data.returned_count < data.lifts.size();
  return result;
}
} // namespace crx::command
