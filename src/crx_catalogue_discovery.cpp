// SPDX-License-Identifier: Apache-2.0
// SPDX-FileCopyrightText: 2026 Centre Technologique en Aérospatiale

#include "crx_discovery.h"
#include "crx_phase_catalogue.h"

#include <array>
#include <cstddef>
#include <utility>
#include <vector>

namespace crx::catalogue {
namespace {
auto Midpoint(interval::Interval x) -> double {
  return (x.lo / 2) + (x.hi / 2);
}
} // namespace

auto Build(const exact::Problem &problem, Options options) -> BuildResult {
  BuildResult result;
  auto roots = root::Certify(problem, options.roots);
  std::vector<ball::Certificate> balls;
  if (!problem.IsReady() || !interval::EnvironmentSupported() ||
      (roots.IsComplete() && roots.Values().simple_roots == 0)) {
    result.catalogue =
        Assemble(problem, std::move(roots), {}, options.max_pair_tests);
    return result;
  }
  const auto &data = problem.Data();
  const canonical::Lengths lengths{
      Midpoint(data.lengths.a), Midpoint(data.lengths.b),
      Midpoint(data.lengths.c), Midpoint(data.lengths.r)};
  PoseIsoRT target = PoseIsoRT::Identity();
  for (int i = 0; i < 3; ++i) {
    target.translation()(i) = Midpoint(data.target.translation(i));
    for (int j = 0; j < 3; ++j)
      target.linear()(i, j) = Midpoint(data.target.rotation(i, j));
  }
  // No command bounds/capacity are passed to canonical discovery. Soundness
  // comes from exact-problem balls, never from this numerical tolerance.
  const auto discovery =
      canonical::DiscoverJointCandidates(lengths, target, {1e-12, 1e-12});
  result.numerical_status = static_cast<int>(discovery.status);
  result.nominations = static_cast<std::uint32_t>(discovery.count);
  for (std::size_t i = 0; i < discovery.count; ++i) {
    if (result.attempted_balls >= options.max_candidate_certificates) {
      result.candidate_budget_exhausted = true;
      break;
    }
    ++result.attempted_balls;
    ball::Centre centre;
    for (std::size_t j = 0; j < centre.size(); ++j)
      centre.at(j) = discovery.candidates.at(i).joints(static_cast<int>(j));
    auto attempt = ball::Certify(problem, centre);
    result.ball_failures.push_back(attempt.failure);
    if (attempt.IsCertified()) {
      ++result.certified_balls;
      balls.push_back(std::move(attempt.certificate));
    }
  }
  result.catalogue = Assemble(problem, std::move(roots), std::move(balls),
                              options.max_pair_tests);
  return result;
}
} // namespace crx::catalogue
