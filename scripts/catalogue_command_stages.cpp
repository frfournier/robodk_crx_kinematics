// SPDX-License-Identifier: Apache-2.0
// SPDX-FileCopyrightText: 2026 Centre Technologique en Aérospatiale

#include "catalogue_command_stages.h"

// Reuse the actual stage bodies, rather than copying their arithmetic or
// adding timing hooks to production. This TU alone supplies command functions
// in the opt-in benchmark; do not link crx_command_certificate into that
// target.
#include "../src/crx_command_certificate.cpp"

#ifdef CRXKIN_DISCOVERY_COUNTERS
#error "Do not time a counter-instrumented benchmark"
#endif

namespace crx::benchmark {
auto PrepareRanges(const exact::Problem &problem,
                   const catalogue::PhaseCatalogue &phases,
                   const command::Context &context) -> command::Data {
  command::Data data;
  data.context = context;
  data.phase_complete = phases.IsComplete();
  if (!data.phase_complete || !context.Matches(problem) ||
      !phases.Matches(problem)) {
    data.failure = command::Failure::IncompletePhaseCover;
    return data;
  }
  data.failure = command::Preflight(problem, phases.Values().balls, data);
  return data;
}

auto Enumerate(command::Data &data, std::uint64_t budget) -> bool {
  if (data.failure != command::Failure::None || data.box_lifts > budget) {
    return false;
  }
  if (!command::Enumerate(data)) {
    return false;
  }
  data.enumeration_complete = true;
  data.admission_complete = data.undecided == 0;
  return true;
}

void Rank(command::Data &data) {
  if (data.enumeration_complete) {
    command::Rank(data);
  }
}
} // namespace crx::benchmark
