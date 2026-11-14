// SPDX-License-Identifier: Apache-2.0
// SPDX-FileCopyrightText: 2026 Centre Technologique en Aérospatiale

#include "exceptional_stages.h"

// Benchmark-only access to the actual shipped recovery boundary. This object
// supplies Build to the opt-in driver too, so the implementation is compiled
// once, without runtime hooks, extra DLL exports or copied recovery arithmetic.
#include "../src/crx_catalogue_discovery.cpp"

namespace crx::benchmark {
auto PrepareExceptional(const exact::Problem &problem) -> ExceptionalContext {
  ExceptionalContext context;
  context.problem = problem;
  if (!problem.IsReady())
    return context;
  const auto &data = problem.Data();
  const canonical::Lengths lengths{
      catalogue::Midpoint(data.lengths.a), catalogue::Midpoint(data.lengths.b),
      catalogue::Midpoint(data.lengths.c), catalogue::Midpoint(data.lengths.r)};
  for (int i = 0; i < 3; ++i) {
    context.target.translation()(i) =
        catalogue::Midpoint(data.target.translation(i));
    for (int j = 0; j < 3; ++j)
      context.target.linear()(i, j) =
          catalogue::Midpoint(data.target.rotation(i, j));
  }
  context.ready =
      canonical::PrepareWristCircle(lengths, context.target, context.circle) ==
      canonical::PreparationStatus::Ready;
  return context;
}

auto RecoverExceptional(const ExceptionalContext &context,
                        const root::Certificate &roots,
                        catalogue::Options options, ExceptionalScratch &scratch)
    -> bool {
  if (!context.ready || !roots.Matches(context.problem) || roots.IsComplete())
    return false;
  catalogue::RecoverExceptional(context.problem, roots, context.circle,
                                context.target, options, scratch.result,
                                scratch.balls);
  return true;
}
} // namespace crx::benchmark
