// SPDX-License-Identifier: Apache-2.0
// SPDX-FileCopyrightText: 2026 Centre Technologique en Aérospatiale

#pragma once

#include "crx_exact_problem.h"
#include "crx_interval.h"
#include "crx_phase_catalogue.h"

#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace crx::command {
using Box = std::array<interval::Interval, 6>;

// Callback coordinates are already coupled. DecoupledUser explicitly applies
// user[2]=callback[2]-callback[1]; its limits must be supplied in that
// convention.
enum class Profile : std::uint32_t { Callback = 0, DecoupledUser = 1 };
enum class Admission { No, Yes, Undecided };
enum class Ranking {
  NotAttempted,
  Inconclusive,
  Unique,
  PossibleMinimizers,
  NoAdmissible
};
enum class Failure {
  None,
  InvalidInput,
  UnsupportedEnvironment,
  IdentityMismatch,
  IncompletePhaseCover,
  RangeUnavailable,
  WorkLimit,
  ArithmeticUnavailable
};

struct Input {
  std::array<double, 6> lower_degrees{};
  std::array<double, 6> upper_degrees{};
  std::array<double, 6> seed_degrees{};
  std::array<double, 6> weights{1, 1, 1, 1, 1, 1};
  Profile profile = Profile::Callback;
  bool has_seed = false;
};

struct ContextData {
  exact::Identity problem_identity;
  Input raw;
  // Exact binary64 provenance, including unused seed values. No finite hash.
  std::array<std::uint64_t, 24> raw_words{};
  Box lower, upper, seed;
};

class Context {
public:
  auto IsReady() const -> bool { return ready_; }

  auto Values() const -> const ContextData & { return data_; }

  auto Matches(const exact::Problem &problem) const -> bool {
    return ready_ && problem.IsReady() &&
           data_.problem_identity == problem.Data().identity;
  }

private:
  friend auto Prepare(const exact::Problem &, const Input &, Context &)
      -> Failure;
  ContextData data_;
  bool ready_ = false;
};

// Raw degree values define mathematical pi/180 conversions. Failure leaves
// output unchanged. No limits are reconstructed from rounded radian inputs.
auto Prepare(const exact::Problem &problem, const Input &input, Context &output)
    -> Failure;

struct PhaseRange {
  Box base;
  std::array<std::int64_t, 6> first{}, last{};
  std::uint64_t count = 0;
  bool available = false;
  bool empty = false;
};

struct Lift {
  std::size_t phase = 0;
  std::array<std::int64_t, 6> turns{};
  Box radians;
  Admission admission = Admission::Undecided;
  interval::Interval travel_squared;
  // Numerical display proposal only; never used to certify a cost comparison.
  double proposed_travel_squared = 0;
};

struct Options {
  std::uint64_t max_lifts = std::uint64_t{1} << 20;
  // Applied after enumeration, admission and global ranking. All proof rows
  // remain available for auditing; returned_count describes the display prefix.
  std::size_t capacity = 512;
};

struct Data {
  Context context;
  Failure failure = Failure::InvalidInput;
  Ranking ranking = Ranking::NotAttempted;
  bool phase_complete = false;
  bool enumeration_complete = false;
  bool admission_complete = false;
  std::uint64_t box_lifts = 0;
  std::uint64_t yes = 0, no = 0, undecided = 0;
  std::vector<PhaseRange> ranges;
  // No rows are omitted from yes/undecided. Certified No rows are counted.
  std::vector<Lift> lifts;
  // Contains every possible minimizer, including boundary-uncertain lifts.
  // Multiple entries do not assert equality of exact travel costs.
  std::vector<std::size_t> possible_minimizers;
  std::size_t returned_count = 0;
  bool truncated = false;
};

class Certificate {
public:
  auto Values() const -> const Data & { return data_; }

  auto IsCommandInfeasible() const -> bool {
    return data_.admission_complete && data_.yes == 0;
  }

  auto HasUniqueNearest() const -> bool {
    return data_.ranking == Ranking::Unique;
  }

private:
  friend auto Certify(const exact::Problem &, const catalogue::PhaseCatalogue &,
                      const Context &, Options) -> Certificate;
  Data data_;
};

// All phase balls and all possible turns are checked before limiting output.
// Range/work failures cannot certify command infeasibility or global nearest.
// Weighted squared travel is in radians in the selected command convention.
auto Certify(const exact::Problem &problem,
             const catalogue::PhaseCatalogue &phases, const Context &context,
             Options options = {}) -> Certificate;
} // namespace crx::command
