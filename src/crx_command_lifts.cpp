#include "crx_command_lifts.h"

#include <algorithm>
#include <cmath>
#include <limits>

#include "crx_math_helpers.h"
#include "crx_numerics.h"

namespace crx {
namespace {
using TurnVector = Eigen::Matrix<std::int64_t, kDofCount, 1>;

auto BoundaryTolerance(const Vec6 &boundary) -> Vec6 {
  return (boundary.cwiseAbs().cwiseMax(1.0) * numerics::kElementaryRoundoff)
      .cwiseMin(numerics::kJointPhaseToleranceRad);
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

auto PrepareBox(LiftBox &box, const Vec6 &lower, const Vec6 &upper) -> bool {
  const Vec6 lo =
      ((lower - box.phase - BoundaryTolerance(lower)).array() / kTwoPi).ceil();
  const Vec6 hi =
      ((upper - box.phase + BoundaryTolerance(upper)).array() / kTwoPi).floor();
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
} // namespace

auto ClampCommandToLimits(Vec6 &command, const Vec6 &lower, const Vec6 &upper)
    -> bool {
  if (!command.allFinite() || !lower.allFinite() || !upper.allFinite() ||
      (lower.array() > upper.array()).any()) {
    return false;
  }
  if (((command.array() < (lower - BoundaryTolerance(lower)).array()) ||
       (command.array() > (upper + BoundaryTolerance(upper)).array()))
          .any()) {
    return false;
  }
  command = command.cwiseMax(lower).cwiseMin(upper);
  return true;
}

auto SelectCommandLifts(const std::vector<Vec6> &postures, const Vec6 &lower,
                        const Vec6 &upper, const Vec6 *seed, int capacity,
                        const std::function<bool(const Vec6 &)> &validate,
                        std::uint64_t work_limit) -> CommandSelection {
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

  std::vector<LiftBox> boxes;
  for (const Vec6 &posture : postures) {
    if (!posture.allFinite()) {
      continue;
    }
    LiftBox box;
    box.phase = posture.unaryExpr(&CommandPhase);
    if (!validate(box.phase) ||
        std::any_of(boxes.begin(), boxes.end(), [&](const LiftBox &other) {
          return SamePhase(box.phase, other.phase);
        })) {
      continue;
    }
    if (!PrepareBox(box, lower, upper) ||
        result.box_lifts >
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
        ClampCommandToLimits(preserved.command_rad, lower, upper) &&
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
      if (ClampCommandToLimits(command.command_rad, lower, upper) &&
          validate(command.command_rad)) {
        feasible = true;
        ++result.valid_lifts;
        const bool matches_seed =
            result.seed_added && (command.command_rad - preserved.command_rad)
                                     .isZero(numerics::kJointPhaseToleranceRad);
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
