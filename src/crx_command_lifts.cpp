// SPDX-License-Identifier: Apache-2.0
// SPDX-FileCopyrightText: 2026 Centre Technologique en Aérospatiale
/**
 * @file crx_command_lifts.cpp
 * @brief Implementation of joint command lift utilities.
 */

#include "crx_command_lifts.h"

#include <Eigen/Core>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <limits>
#include <map>
#include <optional>
#include <queue>
#include <vector>

#include "crx_math_helpers.h"
#include "crx_numerics.h"
#include "crx_types.h"

namespace crx {
namespace {
using TurnVector = Eigen::Matrix<std::int64_t, kDofCount, 1>;

auto BoundaryTolerance(const Vec6 &boundary) -> Vec6 {
  return (boundary.cwiseAbs().cwiseMax(1.0) * numerics::kElementaryRoundoff)
      .cwiseMin(numerics::kJointPhaseToleranceRad);
}

struct CommandLimits {
  Vec6 lower_tolerance;
  Vec6 upper_tolerance;
  Vec6 accepted_lower;
  Vec6 accepted_upper;
};

auto PrepareLimits(const Vec6 &lower, const Vec6 &upper) -> CommandLimits {
  const Vec6 lower_tolerance = BoundaryTolerance(lower);
  const Vec6 upper_tolerance = BoundaryTolerance(upper);
  return {lower_tolerance, upper_tolerance, lower - lower_tolerance,
          upper + upper_tolerance};
}

auto ClampPreparedCommand(Vec6 &command, const Vec6 &lower, const Vec6 &upper,
                          const CommandLimits &limits) -> bool {
  if (!command.allFinite() ||
      ((command.array() < limits.accepted_lower.array()) ||
       (command.array() > limits.accepted_upper.array()))
          .any()) {
    return false;
  }
  command = command.cwiseMax(lower).cwiseMin(upper);
  return true;
}

auto CommandPhase(double angle) -> double {
  const double phase = WrapRadPi(angle);
  return numerics::Scalar(phase - angle_conv::kPi)
                 .isZero(numerics::kElementaryRoundoff)
             ? -angle_conv::kPi
             : phase;
}

auto SamePhase(const Vec6 &a, const Vec6 &b) -> bool {
  return (a - b)
      .unaryExpr([](double x) { return WrapRadPi(x); })
      .isZero(numerics::kJointPhaseToleranceRad);
}

auto ThreeDecimalPlaces(double value) -> double {
  if (value == 0.0 || !std::isfinite(value)) {
    return value == 0.0 ? 0.0 : value;
  }
  const double rounded = std::nearbyint(value * 1000.0) / 1000.0;
  return rounded == 0.0 ? 0.0 : rounded;
}

using CommandIdentity = std::array<double, kDofCount>;

auto OutputIdentity(const Vec6 &command) -> CommandIdentity {
  CommandIdentity identity{};
  for (int axis = 0; axis < kDofCount; ++axis) {
    identity.at(static_cast<std::size_t>(axis)) =
        ThreeDecimalPlaces(angle_conv::RadToDeg(command[axis]));
  }
  return identity;
}

auto SameOutputCommand(const Vec6 &a, const Vec6 &b) -> bool {
  for (int axis = 0; axis < kDofCount; ++axis) {
    if (ThreeDecimalPlaces(angle_conv::RadToDeg(a[axis])) !=
        ThreeDecimalPlaces(angle_conv::RadToDeg(b[axis]))) {
      return false;
    }
  }
  return true;
}

auto Better(const CommandLift &a, const CommandLift &b) -> bool {
  // Heap/sort ordering must be transitive. Approximate equality is only for
  // posture deduplication, never a pairwise tie rule in this comparator.
  if (a.travel_squared < b.travel_squared) {
    return true;
  }
  if (b.travel_squared < a.travel_squared) {
    return false;
  }
  if (a.preserved_seed != b.preserved_seed) {
    return a.preserved_seed;
  }
  for (int i = 0; i < kDofCount; ++i) {
    if (a.command_rad[i] < b.command_rad[i]) {
      return true;
    }
    if (b.command_rad[i] < a.command_rad[i]) {
      return false;
    }
  }
  return false;
}

struct LiftBox {
  Vec6 phase = Vec6::Zero();
  std::array<std::int64_t, kDofCount> lower{};
  std::array<std::int64_t, kDofCount> upper{};
  std::uint64_t count = 1;
};

auto BetterDuplicate(const CommandLift &candidate, const CommandLift &retained,
                     const Vec6 &lower, const Vec6 &upper) -> bool {
  // Within one controller-formatted identity, preserve exact box endpoints.
  // A strictly nearer command or a preserved seed still wins first.
  if (candidate.travel_squared != retained.travel_squared ||
      candidate.preserved_seed != retained.preserved_seed) {
    return Better(candidate, retained);
  }
  const auto endpoints = [&](const Vec6 &command) {
    return ((command.array() == lower.array()) ||
            (command.array() == upper.array()))
        .count();
  };
  const auto candidate_endpoints = endpoints(candidate.command_rad);
  const auto retained_endpoints = endpoints(retained.command_rad);
  return candidate_endpoints == retained_endpoints
             ? Better(candidate, retained)
             : candidate_endpoints > retained_endpoints;
}

auto PrepareBox(LiftBox &box, const Vec6 &lower, const Vec6 &upper,
                const CommandLimits &limits) -> bool {
  // Keep the subtraction order: moving the allowance before the phase can
  // change rounding at an integer turn boundary.
  const Vec6 lo =
      ((lower - box.phase - limits.lower_tolerance).array() / kTwoPi).ceil();
  const Vec6 hi =
      ((upper - box.phase + limits.upper_tolerance).array() / kTwoPi).floor();
  if (!lo.allFinite() || !hi.allFinite() ||
      lo.cwiseAbs().maxCoeff() > numerics::kMaxExactTurn ||
      hi.cwiseAbs().maxCoeff() > numerics::kMaxExactTurn) {
    return false;
  }
  // Every component must pass before converting any value to integer turns.
  Eigen::Map<TurnVector> lower_turns(box.lower.data());
  Eigen::Map<TurnVector> upper_turns(box.upper.data());
  lower_turns = lo.cast<std::int64_t>();
  upper_turns = hi.cast<std::int64_t>();
  if ((lo.array() > hi.array()).any()) {
    box.count = 0;
    return true;
  }
  for (std::size_t i = 0; i < box.lower.size(); ++i) {
    const auto count =
        static_cast<std::uint64_t>(box.upper[i] - box.lower[i]) + 1;
    if (box.count > std::numeric_limits<std::uint64_t>::max() / count) {
      return false;
    }
    box.count *= count;
  }
  return true;
}

auto HasRoundedAliases(const std::vector<LiftBox> &boxes, const Vec6 &lower,
                       const Vec6 &upper) -> bool {
  // Endpoint preference can replace a rounded duplicate by a lexicographically
  // later representative. A heap bound on the first visited member is then
  // insufficient to establish the exact prefix. Use exhaustive selection when
  // two geometric boxes could share a controller-formatted identity.
  const double output_bin_rad = angle_conv::DegToRad(0.001);
  const double magnitude =
      std::max({1.0, lower.cwiseAbs().maxCoeff(), upper.cwiseAbs().maxCoeff()});
  const double roundoff = magnitude * numerics::kElementaryRoundoff;
  // Lifting and degree conversion can merge phases farther apart than the
  // nominal output bin at large turns. Include their absolute roundoff scale,
  // and use exhaustive identity selection once that scale spans a whole bin.
  if (roundoff >= output_bin_rad) {
    return true;
  }
  for (std::size_t i = 0; i < boxes.size(); ++i) {
    for (std::size_t j = 0; j < i; ++j) {
      const Vec6 difference =
          (boxes.at(i).phase - boxes.at(j).phase).unaryExpr([](double value) {
            return WrapRadPi(value);
          });
      if (difference.cwiseAbs().maxCoeff() <= output_bin_rad + 2 * roundoff) {
        return true;
      }
    }
  }
  return false;
}

struct RankedProblem {
  const Vec6 &lower;
  const Vec6 &upper;
  const CommandLimits &limits;
  const Vec6 *seed;
  const std::function<bool(const Vec6 &)> &validate;
};

auto LiftCommand(const LiftBox &box,
                 const std::array<std::int64_t, kDofCount> &turns) -> Vec6 {
  const Vec6 values = Eigen::Map<const TurnVector>(turns.data()).cast<double>();
  return box.phase + kTwoPi * values;
}

auto CoversLifts(const LiftBox &covering, const LiftBox &covered) -> bool {
  if (!SamePhase(covering.phase, covered.phase)) {
    return false;
  }
  if (covered.count == 0) {
    return true;
  }
  if (covering.count == 0) {
    return false;
  }
  // Compare actual commands across the +/-pi phase cut. A representative
  // must retain every admitted turn, including exact joint-limit endpoints.
  return ((LiftCommand(covering, covering.lower) -
           LiftCommand(covered, covered.lower))
              .array() <= numerics::kCommandDedupToleranceRad)
             .all() &&
         ((LiftCommand(covered, covered.upper) -
           LiftCommand(covering, covering.upper))
              .array() <= numerics::kCommandDedupToleranceRad)
             .all();
}

struct RankedBox {
  LiftBox box;
  CommandLift first;
  std::size_t posture = 0;
  bool admitted = false;
};

auto FirstInBox(const LiftBox &box, std::size_t posture,
                const RankedProblem &problem) -> RankedBox {
  RankedBox node;
  node.box = box;
  node.posture = posture;
  node.first.turns = box.lower;
  if (problem.seed != nullptr) {
    auto next = box.lower;
    for (int axis = 0; axis < kDofCount; ++axis) {
      const auto i = static_cast<std::size_t>(axis);
      const double turn = std::clamp(
          std::floor(((*problem.seed)[axis] - box.phase[axis]) / kTwoPi),
          static_cast<double>(box.lower[i]), static_cast<double>(box.upper[i]));
      node.first.turns[i] = static_cast<std::int64_t>(turn);
      next[i] = std::min(node.first.turns[i] + 1, box.upper[i]);
    }
    const Vec6 first = LiftCommand(box, node.first.turns)
                           .cwiseMax(problem.lower)
                           .cwiseMin(problem.upper);
    const Vec6 second =
        LiftCommand(box, next).cwiseMax(problem.lower).cwiseMin(problem.upper);
    for (int axis = 0; axis < kDofCount; ++axis) {
      const double a = first[axis] - (*problem.seed)[axis];
      const double b = second[axis] - (*problem.seed)[axis];
      if (b * b < a * a) {
        node.first.turns[static_cast<std::size_t>(axis)] =
            next[static_cast<std::size_t>(axis)];
      }
    }
  }
  node.first.command_rad = LiftCommand(box, node.first.turns);
  node.admitted = ClampPreparedCommand(node.first.command_rad, problem.lower,
                                       problem.upper, problem.limits);
  // Even an inadmissible roundoff endpoint must bound its remaining box.
  node.first.command_rad =
      node.first.command_rad.cwiseMax(problem.lower).cwiseMin(problem.upper);
  if (problem.seed != nullptr) {
    node.first.travel_squared =
        (node.first.command_rad - *problem.seed).squaredNorm();
  }
  return node;
}

auto RankedPrefix(const std::vector<LiftBox> &boxes,
                  const RankedProblem &problem, const CommandLift &preserved,
                  std::size_t capacity, CommandSelection result)
    -> std::optional<CommandSelection> {
  // Unusual validators can reject arbitrarily many lifts. Keep the new work
  // storage bounded, and use the exhaustive implementation if it fills up.
  constexpr std::size_t kFrontierBudget = 4096;
  const auto later = [](const RankedBox &a, const RankedBox &b) {
    return Better(b.first, a.first);
  };
  std::priority_queue<RankedBox, std::vector<RankedBox>, decltype(later)>
      frontier(later);
  auto enqueue = [&](const LiftBox &box, std::size_t posture) {
    frontier.push(FirstInBox(box, posture, problem));
  };
  for (std::size_t i = 0; i < boxes.size(); ++i) {
    const auto &box = boxes[i];
    if (box.count == 0) {
      continue;
    }
    if (problem.seed != nullptr) {
      // A distant valid command can overflow travel even when the prefix is
      // finite. Preserve that failure policy through exhaustive fallback.
      const Vec6 low = LiftCommand(box, box.lower)
                           .cwiseMax(problem.lower)
                           .cwiseMin(problem.upper);
      const Vec6 high = LiftCommand(box, box.upper)
                            .cwiseMax(problem.lower)
                            .cwiseMin(problem.upper);
      const Vec6 farthest = (low - *problem.seed)
                                .cwiseAbs()
                                .cwiseMax((high - *problem.seed).cwiseAbs());
      if (!std::isfinite(farthest.squaredNorm())) {
        return std::nullopt;
      }
    }
    enqueue(box, i);
  }
  const auto retain = [&](CommandLift command) {
    if (problem.seed != nullptr) {
      command.travel_squared =
          (command.command_rad - *problem.seed).squaredNorm();
    }
    const auto duplicate = std::find_if(
        result.commands.begin(), result.commands.end(), [&](const auto &entry) {
          return SameOutputCommand(command.command_rad, entry.command_rad);
        });
    if (duplicate != result.commands.end()) {
      if (BetterDuplicate(command, *duplicate, problem.lower, problem.upper)) {
        *duplicate = command;
        std::make_heap(result.commands.begin(), result.commands.end(), Better);
      }
      return;
    }
    if (result.commands.size() < capacity + 1) {
      result.commands.push_back(command);
      std::push_heap(result.commands.begin(), result.commands.end(), Better);
    } else if (Better(command, result.commands.front())) {
      std::pop_heap(result.commands.begin(), result.commands.end(), Better);
      result.commands.back() = command;
      std::push_heap(result.commands.begin(), result.commands.end(), Better);
    }
  };
  if (result.seed_added) {
    retain(preserved);
  }
  std::vector<bool> feasible(boxes.size(), false);
  while (!frontier.empty()) {
    if (frontier.size() > kFrontierBudget) {
      return std::nullopt;
    }
    if (result.commands.size() > capacity &&
        (problem.seed == nullptr ||
         frontier.top().first.travel_squared >
             result.commands.front().travel_squared)) {
      // Process the entire equal-cost frontier: rounded sums can tie even
      // when an individual axis cost increases, changing lexicographic ties.
      result.counts_complete = false;
      break;
    }
    const RankedBox node = frontier.top();
    frontier.pop();
    if (node.admitted && problem.validate(node.first.command_rad)) {
      ++result.valid_lifts;
      if (!feasible[node.posture]) {
        feasible[node.posture] = true;
        ++result.feasible_postures;
      }
      if (result.seed_added &&
          SameOutputCommand(node.first.command_rad, preserved.command_rad)) {
        result.seed_matches_discovery = true;
      } else {
        retain(node.first);
      }
    }
    // Remove the visited point by partitioning into disjoint boxes. Earlier
    // coordinates are fixed to that point, so no visited-set is necessary.
    LiftBox remainder = node.box;
    for (std::size_t axis = 0; axis < remainder.lower.size(); ++axis) {
      const auto turn = node.first.turns[axis];
      if (turn > remainder.lower[axis]) {
        auto child = remainder;
        child.upper[axis] = turn - 1;
        enqueue(child, node.posture);
      }
      if (turn < remainder.upper[axis]) {
        auto child = remainder;
        child.lower[axis] = turn + 1;
        enqueue(child, node.posture);
      }
      remainder.lower[axis] = remainder.upper[axis] = turn;
    }
  }
  std::sort_heap(result.commands.begin(), result.commands.end(), Better);
  result.truncated = result.commands.size() > capacity;
  if (result.truncated) {
    result.commands.resize(capacity);
  }
  return result;
}
} // namespace

auto ClampCommandToLimits(Vec6 &command, const Vec6 &lower, const Vec6 &upper)
    -> bool {
  if (!lower.allFinite() || !upper.allFinite() ||
      (lower.array() > upper.array()).any()) {
    return false;
  }
  return ClampPreparedCommand(command, lower, upper,
                              PrepareLimits(lower, upper));
}

auto SelectCommandLifts(const std::vector<Vec6> &postures, const Vec6 &lower,
                        const Vec6 &upper, const Vec6 *seed, int capacity,
                        const std::function<bool(const Vec6 &)> &validate,
                        std::uint64_t work_limit, CommandSelectionMode mode)
    -> CommandSelection {
  CommandSelection result;
  if (!lower.allFinite() || !upper.allFinite() ||
      (lower.array() > upper.array()).any() ||
      (seed != nullptr && !seed->allFinite()) || !validate) {
    return result;
  }
  result.status = CommandSelectionStatus::Complete;
  if (capacity <= 0) {
    return result;
  }

  const CommandLimits limits = PrepareLimits(lower, upper);
  std::vector<LiftBox> boxes;
  for (const Vec6 &posture : postures) {
    if (!posture.allFinite()) {
      continue;
    }
    LiftBox box;
    box.phase = posture.unaryExpr(&CommandPhase);
    if (!validate(box.phase)) {
      continue;
    }
    if (!PrepareBox(box, lower, upper, limits)) {
      result.status = CommandSelectionStatus::RangeFailure;
      return result;
    }
    // Prefer a nearby representative covering all admitted turns. If neither
    // range contains the other, retain both rather than lose legal commands.
    if (std::any_of(boxes.begin(), boxes.end(), [&](const LiftBox &other) {
          return CoversLifts(other, box);
        })) {
      continue;
    }
    boxes.erase(std::remove_if(boxes.begin(), boxes.end(),
                               [&](const LiftBox &other) {
                                 if (!CoversLifts(box, other)) {
                                   return false;
                                 }
                                 result.box_lifts -= other.count;
                                 return true;
                               }),
                boxes.end());
    if (result.box_lifts >
        std::numeric_limits<std::uint64_t>::max() - box.count) {
      result.status = CommandSelectionStatus::RangeFailure;
      return result;
    }
    result.box_lifts += box.count;
    boxes.push_back(box);
  }
  result.geometric_postures = boxes.size();
  if (result.box_lifts > work_limit) {
    result.status = CommandSelectionStatus::WorkLimit;
    return result;
  }

  CommandLift preserved;
  if (seed != nullptr) {
    preserved.command_rad = *seed;
    preserved.preserved_seed = true;
    result.seed_added =
        ClampPreparedCommand(preserved.command_rad, lower, upper, limits) &&
        validate(preserved.command_rad);
    if (result.seed_added) {
      for (int i = 0; i < kDofCount; ++i) {
        const double phase = CommandPhase(preserved.command_rad[i]);
        const double turn =
            std::round((preserved.command_rad[i] - phase) / kTwoPi);
        if (std::abs(turn) > numerics::kMaxExactTurn) {
          result.status = CommandSelectionStatus::RangeFailure;
          return result;
        }
        preserved.turns[static_cast<std::size_t>(i)] =
            static_cast<std::int64_t>(turn);
      }
    }
  }
  const auto output_capacity = static_cast<std::size_t>(capacity);
  // A duplicate's preferred representative can move later in the ordering.
  // Retain all rounded identities in this bounded fallback, then rank them;
  // otherwise an evicted identity could become part of the final prefix.
  const bool retain_all_identities = HasRoundedAliases(boxes, lower, upper);
  if (mode == CommandSelectionMode::RankedPrefix && !retain_all_identities &&
      result.box_lifts > output_capacity + 1) {
    const auto ranked =
        RankedPrefix(boxes, {lower, upper, limits, seed, validate}, preserved,
                     output_capacity, result);
    if (ranked.has_value()) {
      return *ranked;
    }
    // A full frontier or nonfinite travel bound requests the exhaustive path.
    // Its validator-aware checks preserve RangeFailure versus WorkLimit.
  }
  bool unique_overflow = false;
  std::map<CommandIdentity, std::size_t> identities;
  auto retain = [&](CommandLift command) {
    if (seed != nullptr) {
      command.travel_squared = (command.command_rad - *seed).squaredNorm();
      if (!std::isfinite(command.travel_squared)) {
        return false;
      }
    }
    if (retain_all_identities) {
      const auto [entry, inserted] = identities.emplace(
          OutputIdentity(command.command_rad), result.commands.size());
      if (inserted) {
        result.commands.push_back(command);
      } else {
        auto &previous = result.commands.at(entry->second);
        if (BetterDuplicate(command, previous, lower, upper)) {
          previous = command;
        }
      }
      return true;
    }
    const auto duplicate = std::find_if(
        result.commands.begin(), result.commands.end(), [&](const auto &entry) {
          return SameOutputCommand(command.command_rad, entry.command_rad);
        });
    if (duplicate != result.commands.end()) {
      if (BetterDuplicate(command, *duplicate, lower, upper)) {
        *duplicate = command;
        std::make_heap(result.commands.begin(), result.commands.end(), Better);
      }
      return true;
    }
    if (result.commands.size() < output_capacity) {
      result.commands.push_back(command);
      std::push_heap(result.commands.begin(), result.commands.end(), Better);
    } else {
      unique_overflow = true;
      if (Better(command, result.commands.front())) {
        std::pop_heap(result.commands.begin(), result.commands.end(), Better);
        result.commands.back() = command;
        std::push_heap(result.commands.begin(), result.commands.end(), Better);
      }
    }
    return true;
  };

  if (result.seed_added && !retain(preserved)) {
    result.status = CommandSelectionStatus::RangeFailure;
    result.commands.clear();
    return result;
  }
  for (const LiftBox &box : boxes) {
    auto turns = box.lower;
    bool feasible = false;
    for (std::uint64_t index = 0; index < box.count; ++index) {
      CommandLift command;
      command.turns = turns;
      const Vec6 turn_values =
          Eigen::Map<const TurnVector>(turns.data()).cast<double>();
      command.command_rad = box.phase + kTwoPi * turn_values;
      if (ClampPreparedCommand(command.command_rad, lower, upper, limits) &&
          validate(command.command_rad)) {
        feasible = true;
        ++result.valid_lifts;
        const bool matches_seed =
            result.seed_added &&
            SameOutputCommand(command.command_rad, preserved.command_rad);
        if (matches_seed) {
          // Count seed preservation separately, never as another discovered
          // lift.
          result.seed_matches_discovery = true;
        } else if (!retain(command)) {
          result.status = CommandSelectionStatus::RangeFailure;
          result.commands.clear();
          return result;
        }
      }
      for (int i = kDofCount - 1; i >= 0; --i) {
        const auto axis = static_cast<std::size_t>(i);
        if (turns[axis] < box.upper[axis]) {
          ++turns[axis];
          break;
        }
        turns[axis] = box.lower[axis];
      }
    }
    if (feasible) {
      ++result.feasible_postures;
    }
  }
  if (retain_all_identities) {
    std::sort(result.commands.begin(), result.commands.end(), Better);
  } else {
    std::sort_heap(result.commands.begin(), result.commands.end(), Better);
  }
  result.truncated =
      unique_overflow || result.commands.size() > output_capacity;
  if (result.commands.size() > output_capacity) {
    result.commands.resize(output_capacity);
  }
  return result;
}
} // namespace crx
