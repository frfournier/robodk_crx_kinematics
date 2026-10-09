#pragma once

#include <array>
#include <cstdint>
#include <functional>
#include <vector>

#include "crx_types.h"

namespace crx {

enum class CommandSelectionStatus {
  Complete,
  InvalidInput,
  RangeFailure,
  WorkLimit
};

struct CommandLift {
  Vec6 command_rad = Vec6::Zero();
  std::array<std::int64_t, kDofCount> turns{};
  double travel_squared = 0.0;
  bool preserved_seed = false;
};

struct CommandSelection {
  CommandSelectionStatus status = CommandSelectionStatus::InvalidInput;
  std::size_t geometric_postures = 0;
  std::size_t feasible_postures = 0;
  std::uint64_t box_lifts = 0;
  std::uint64_t valid_lifts = 0;
  bool seed_added = false;
  bool seed_matches_discovery = false;
  bool truncated = false;
  std::vector<CommandLift> commands;
};

// Only floating-point roundoff may be clamped, never the old 0.01 degree band.
auto ClampCommandToLimits(Vec6 &command, const Vec6 &lower, const Vec6 &upper)
    -> bool;

// All finite lifts are visited, FK-validated, and counted. A top-K heap limits
// output storage, not discovery. Failure publishes no partial command list.
// The validator consumes command coordinates and must not apply joint limits.
auto SelectCommandLifts(const std::vector<Vec6> &postures, const Vec6 &lower,
                        const Vec6 &upper, const Vec6 *seed, int capacity,
                        const std::function<bool(const Vec6 &)> &validate,
                        std::uint64_t work_limit = 1048576) -> CommandSelection;

} // namespace crx
