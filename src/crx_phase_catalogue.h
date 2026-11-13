// SPDX-License-Identifier: Apache-2.0
// SPDX-FileCopyrightText: 2026 Centre Technologique en Aérospatiale

#pragma once

#include "crx_ball_certificate.h"
#include "crx_root_certificate.h"
#include "crx_solver.h"

#include <cstddef>
#include <cstdint>
#include <vector>

namespace crx::catalogue {
enum class Failure {
  None,
  InvalidInput,
  UnsupportedEnvironment,
  IdentityMismatch,
  IncompleteRootCover,
  InvalidBall,
  CountMismatch,
  BallsNotSeparated,
  PhaseRangeUnavailable,
  PairWorkLimit
};

struct Data {
  exact::Identity identity;
  Coverage coverage = Coverage::Inconclusive;
  Failure failure = Failure::InvalidInput;
  root::Certificate root_cover;
  std::vector<ball::Certificate> balls;
  std::uint32_t expected_phases = 0;
  std::uint32_t pair_tests = 0;
  std::size_t failed_first = 0;
  std::size_t failed_second = 0;
};

class PhaseCatalogue {
public:
  auto IsComplete() const -> bool { return complete_; }

  // Nominal pose infeasibility only: no commands, limits or ranking implied.
  auto IsInfeasible() const -> bool { return complete_ && data_.balls.empty(); }

  auto Values() const -> const Data & { return data_; }

  auto Matches(const exact::Problem &problem) const -> bool {
    return problem.IsReady() && data_.identity == problem.Data().identity;
  }

private:
  friend auto Assemble(const exact::Problem &, root::Certificate,
                       std::vector<ball::Certificate>, std::uint32_t)
      -> PhaseCatalogue;
  Data data_;
  bool complete_ = false;
};

// Bounds min_k |a-b-2*pi*k| using exact represented centres and mathematical
// pi. A rounded reduction only proposes an integer, then enclosures validate
// the retained candidates. Invalid means unavailable, never phase equality.
auto CircleDistance(double a, double b) -> interval::Interval;

// Only matching sealed exact root/ball certificates are accepted. Complete
// requires exactly 2r sound, pairwise disjoint existence balls modulo turns.
// Counting is before limits, command lifting and output-prefix truncation.
auto Assemble(const exact::Problem &problem, root::Certificate root_cover,
              std::vector<ball::Certificate> balls,
              std::uint32_t max_pair_tests = 120) -> PhaseCatalogue;

struct Options {
  root::Options roots;
  std::uint32_t max_candidate_certificates = 512;
  std::uint32_t max_pair_tests = 120;
};

struct BuildResult {
  PhaseCatalogue catalogue;
  std::uint32_t nominations = 0;
  std::uint32_t attempted_balls = 0;
  std::uint32_t certified_balls = 0;
  // Raw canonical numerical discovery status, nomination only.
  int numerical_status = -1;
  bool candidate_budget_exhausted = false;
  std::vector<ball::Failure> ball_failures;
};

// Opt-in unlimited canonical discovery for this normalized exact problem.
// Rounded midpoint transforms/lengths nominate only. The assembler remains
// authoritative even if discovery is empty, duplicated, partial or inaccurate.
auto Build(const exact::Problem &problem, Options options = {}) -> BuildResult;
} // namespace crx::catalogue
