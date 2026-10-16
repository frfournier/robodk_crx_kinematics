#include "crx_command_lifts.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>

#include "crx_math_helpers.h"
#include "crx_numerics.h"

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
      .isZero(numerics::kCommandDedupToleranceRad);
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
  std::vector<RankedBox> frontier;
  const auto later = [](const RankedBox &a, const RankedBox &b) {
    return Better(b.first, a.first);
  };
  auto enqueue = [&](const LiftBox &box, std::size_t posture) {
    frontier.push_back(FirstInBox(box, posture, problem));
    std::push_heap(frontier.begin(), frontier.end(), later);
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
         frontier.front().first.travel_squared >
             result.commands.front().travel_squared)) {
      // Process the entire equal-cost frontier: rounded sums can tie even
      // when an individual axis cost increases, changing lexicographic ties.
      result.counts_complete = false;
      break;
    }
    std::pop_heap(frontier.begin(), frontier.end(), later);
    const RankedBox node = frontier.back();
    frontier.pop_back();
    if (node.admitted && problem.validate(node.first.command_rad)) {
      ++result.valid_lifts;
      if (!feasible[node.posture]) {
        feasible[node.posture] = true;
        ++result.feasible_postures;
      }
      if (result.seed_added &&
          (node.first.command_rad - preserved.command_rad)
              .isZero(numerics::kCommandDedupToleranceRad)) {
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
  if (mode == CommandSelectionMode::RankedPrefix &&
      result.box_lifts > output_capacity + 1) {
    const auto ranked =
        RankedPrefix(boxes, {lower, upper, limits, seed, validate}, preserved,
                     output_capacity, result);
    if (ranked.has_value()) {
      return *ranked;
    }
  }
  auto retain = [&](CommandLift command) {
    if (seed != nullptr) {
      command.travel_squared = (command.command_rad - *seed).squaredNorm();
      if (!std::isfinite(command.travel_squared)) {
        return false;
      }
    }
    if (result.commands.size() < output_capacity) {
      result.commands.push_back(command);
      std::push_heap(result.commands.begin(), result.commands.end(), Better);
    } else if (Better(command, result.commands.front())) {
      std::pop_heap(result.commands.begin(), result.commands.end(), Better);
      result.commands.back() = command;
      std::push_heap(result.commands.begin(), result.commands.end(), Better);
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
            (command.command_rad - preserved.command_rad)
                .isZero(numerics::kCommandDedupToleranceRad);
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
  std::sort_heap(result.commands.begin(), result.commands.end(), Better);
  result.truncated = result.valid_lifts + static_cast<std::uint64_t>(
                                              result.seed_added &&
                                              !result.seed_matches_discovery) >
                     result.commands.size();
  return result;
}
} // namespace crx
