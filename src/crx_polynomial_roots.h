#pragma once

#include <array>
#include <complex>
#include <cstddef>
#include <limits>

#include "crx_canonical.h"

namespace crx::canonical {

enum class RootStatus {
  Candidates,
  ConstantPolynomial,
  ZeroPolynomial,
  InvalidInput,
  UnresolvedLeadingCoefficient,
  NumericalRangeFailure,
  BalanceWorkLimit,
  EigenWorkLimit
};

struct RootOptions {
  int max_balance_sweeps = 32;
  int max_eigen_iterations = 256;
  // A relative leading coefficient below this threshold is unresolved, never
  // trimmed. Zero disables this heuristic for numerical experiments only.
  double leading_relative_tolerance =
      64.0 * std::numeric_limits<double>::epsilon();
};

struct RootCandidate {
  // Preserve complex eigenvalues exactly as returned, without real-root
  // filtering, imaginary-part cleanup, clustering, or multiplicity claims.
  std::complex<double> value{};
  double relative_residual = 0.0;
};

struct RootResult {
  RootStatus status = RootStatus::InvalidInput;
  int represented_degree = -1;
  int balance_sweeps = 0;
  std::size_t count = 0;
  std::array<RootCandidate, kCoefficientCount - 1> candidates{};
};

// Numerical candidates for the stored, ascending coefficients only. Exact
// degree zeros do not establish degree of the uncertain geometric polynomial.
// Count is zero on failures; callers must inspect status, never infer that a
// target is unreachable. Candidates/constant cases still require chart
// endpoint, coefficient uncertainty, real-root accounting, incidence and FK
// checks. Degree one is solved directly; degrees 2..8 use fixed-size
// EigenSolver with eigenvalues only and bounded power-of-two diagonal
// similarity balancing.
auto FindRootCandidates(const Coefficients &coefficients,
                        const RootOptions &options = {}) -> RootResult;

} // namespace crx::canonical
