// SPDX-License-Identifier: Apache-2.0
// SPDX-FileCopyrightText: 2026 Centre Technologique en Aérospatiale

#pragma once

#include "crx_exact_problem.h"

#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace crx::root {
using Interval = interval::Interval;
using Coefficients = std::array<Interval, 9>;

enum class Status {
  Complete,
  Inconclusive,
  InvalidInput,
  UnsupportedEnvironment,
  ArithmeticUnavailable
};
enum class Kind { Exclusion, SimpleRoot, Unresolved };
enum class Reason {
  None,
  ChartBoundary,
  DepthLimit,
  WorkLimit,
  LeafLimit,
  SplitNotSeparated,
  ZeroPolynomial,
  ZeroDerivative,
  ZeroDeterminant,
  ArithmeticUnavailable
};

struct Options {
  // Node work is shared by both charts and all attempts. Fixed degree-eight
  // polynomial/initial Bernstein construction is bounded separately by
  // attempts.
  std::uint32_t max_nodes = 8192;
  std::uint32_t max_depth = 40;
  std::uint32_t max_leaves = 4096;
  std::uint32_t max_chart_attempts = 8;
};

struct Leaf {
  Kind kind = Kind::Unresolved;
  Reason reason = Reason::None;
  Interval tangent{-1, 1};
  // An outward unwrapped phase descriptor. Exact chart semantics are
  // shift*pi/8 + opposite*pi + 2*atan(t); no rounded pi defines a chart.
  Interval phase;
  std::uint32_t depth = 0;
  std::array<Interval, 2> endpoint_values{};
  Coefficients residual_bernstein{};
  std::array<Interval, 8> derivative_bernstein{};
  std::array<Interval, 5> determinant_bernstein{};
  int derivative_sign = 0;
  int determinant_sign = 0;
};

struct Partition {
  Status status = Status::InvalidInput;
  std::vector<Leaf> leaves;
  std::uint32_t nodes = 0;
  std::uint32_t max_depth = 0;
  std::uint32_t roots = 0;
  std::uint32_t exclusions = 0;
  std::uint32_t unresolved = 0;
};

// Polynomial utilities: they prove statements about these input coefficient
// enclosures only. Their results cannot be submitted as an IK certificate.
// Fixed slots retain zero/uncertain leading coefficients and degree drops.
auto Evaluate(const Coefficients &power, double t) -> Interval;
auto Bernstein(const Coefficients &power, std::size_t degree, Interval domain,
               Coefficients &output) -> bool;
auto PartitionPolynomial(const Coefficients &power,
                         const std::array<Interval, 5> &determinant,
                         Options options = {}) -> Partition;

struct Data {
  exact::Identity identity;
  Status status = Status::InvalidInput;
  std::uint32_t chart_shift = 0; // Exact multiple of mathematical pi/8.
  std::uint32_t attempts = 0;
  std::uint32_t nodes = 0; // Includes unsuccessful attempts.
  // Failure can precede leaf construction. Empty partitions then mean the
  // whole circle is unresolved, never excluded.
  bool entire_circle_unresolved = true;
  std::array<exact::Polynomials, 2> polynomials;
  std::array<Partition, 2> partitions;
  std::uint32_t simple_roots = 0;
};

class Certificate {
public:
  auto IsComplete() const -> bool { return complete_; }

  auto Values() const -> const Data & { return data_; }

  auto Matches(const exact::Problem &problem) const -> bool {
    return problem.IsReady() && data_.identity == problem.Data().identity;
  }

private:
  friend auto Certify(const exact::Problem &, Options) -> Certificate;
  Data data_;
  bool complete_ = false;
};

// The sealed certificate constructs its own coefficients from the one exact
// Problem. Complete means a full circle cover with simple roots and nonzero
// determinant at every root. Reconstruction/catalogue soundness and command
// admissibility are separate obligations. Failed prepared-target covers retain
// either leaf domains or an explicit whole-circle-unresolved marker.
auto Certify(const exact::Problem &problem, Options options = {})
    -> Certificate;
} // namespace crx::root
