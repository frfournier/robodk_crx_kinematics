// SPDX-License-Identifier: Apache-2.0
// SPDX-FileCopyrightText: 2026 Centre Technologique en Aérospatiale

#pragma once

#include <cstdint>

namespace crx::canonical {

// Per invocation/evaluation, not per target. A target can nominate many roots
// and each refinement can evaluate several backtracking trials.
struct DiscoveryCounters {
  std::uint64_t cramer_runs = 0;
  std::uint64_t cramer_evaluations = 0;
  std::uint64_t cramer_guard_rejections = 0;
  std::uint64_t cramer_initial_rejections = 0;
  std::uint64_t cramer_successes = 0;
  std::uint64_t cramer_recovery_failures = 0;
};

#ifdef CRXKIN_DISCOVERY_COUNTERS
inline thread_local DiscoveryCounters discovery_counters;
#define CRXKIN_COUNT_DISCOVERY(event)                                          \
  (++crx::canonical::discovery_counters.event)
#else
// No counter storage, branch or function call enters ordinary DLL builds.
#define CRXKIN_COUNT_DISCOVERY(event) ((void)0)
#endif

} // namespace crx::canonical
