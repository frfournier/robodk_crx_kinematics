#include "crx_allocation_probe.h"
#include "crx_canonical.h"
#include "crx_numerics.h"
#include "crx_polynomial_roots.h"
#include "crx_root_tests.h"

#include <cmath>
#include <cstddef>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <limits>

namespace {
using namespace crx::canonical;

void Require(bool condition, const char *message) {
  if (!condition) {
    std::fprintf(stderr, "Polynomial test failed: %s\n", message);
    std::exit(EXIT_FAILURE);
  }
}

auto StatusName(RootStatus status) -> const char * {
  switch (status) {
  case RootStatus::Candidates:
    return "Candidates";
  case RootStatus::ConstantPolynomial:
    return "ConstantPolynomial";
  case RootStatus::ZeroPolynomial:
    return "ZeroPolynomial";
  case RootStatus::InvalidInput:
    return "InvalidInput";
  case RootStatus::UnresolvedLeadingCoefficient:
    return "UnresolvedLeadingCoefficient";
  case RootStatus::NumericalRangeFailure:
    return "NumericalRangeFailure";
  case RootStatus::BalanceWorkLimit:
    return "BalanceWorkLimit";
  case RootStatus::EigenWorkLimit:
    return "EigenWorkLimit";
  }
  return "Unknown";
}

void ExpectStatus(const Coefficients &p, RootStatus status,
                  const RootOptions &options = {}) {
  const auto result = FindRootCandidates(p, options);
  Require(result.status == status, StatusName(status));
  Require(result.count == 0, "failure publishes no candidates");
}
} // namespace

void RunPolynomialRootTests() {
  // Python checks degree 1..8 roots, scaling, multiplicity and residuals
  // through RunRootProbe, which also guards each solve against allocation.
  // This degree-eight polynomial is only needed for the work-limit test.
  Coefficients polynomial{1};
  for (int degree = 1; degree <= 8; ++degree) {
    const double root = 0.25 * degree;
    for (int i = degree; i > 0; --i) {
      const auto index = static_cast<std::size_t>(i);
      polynomial[index] = polynomial[index - 1] - root * polynomial[index];
    }
    polynomial[0] *= -root;
  }
  const auto subnormal =
      FindRootCandidates({std::numeric_limits<double>::denorm_min(), 1});
  Require(
      subnormal.status == RootStatus::Candidates && subnormal.count == 1 &&
          std::fpclassify(subnormal.candidates[0].value.real()) ==
              FP_SUBNORMAL &&
          subnormal.candidates[0].value.real() < 0,
      "representable subnormal root survives normalization and range checks");
  const auto balanced = FindRootCandidates({100, -10000.01, 1});
  Require(balanced.status == RootStatus::Candidates &&
              balanced.balance_sweeps > 1,
          "disparate roots exercise balancing");
  RootOptions one_sweep;
  one_sweep.max_balance_sweeps = 1;
  ExpectStatus({100, -10000.01, 1}, RootStatus::BalanceWorkLimit, one_sweep);
  ExpectStatus({std::numeric_limits<double>::infinity()},
               RootStatus::InvalidInput);
  ExpectStatus({std::numeric_limits<double>::quiet_NaN()},
               RootStatus::InvalidInput);
  RootOptions options;
  options.max_balance_sweeps = 0;
  ExpectStatus({1, 0, 1}, RootStatus::BalanceWorkLimit, options);
  options = {};
  options.max_eigen_iterations = 0;
  ExpectStatus({1, 0, 1}, RootStatus::EigenWorkLimit, options);
  options.max_eigen_iterations = 1;
  ExpectStatus(polynomial, RootStatus::EigenWorkLimit, options);
  options.max_eigen_iterations = -1;
  ExpectStatus({1, 1}, RootStatus::InvalidInput, options);
  options = {};
  options.leading_relative_tolerance = -1;
  ExpectStatus({1, 1}, RootStatus::InvalidInput, options);
  options.leading_relative_tolerance = 0;
  ExpectStatus({1, std::numeric_limits<double>::denorm_min()},
               RootStatus::NumericalRangeFailure, options);
}

auto RunRootProbe(int argc, char **argv) -> int {
  if (argc < 3 || argc > 11 || std::strcmp(argv[1], "--roots") != 0) {
    std::fputs("Usage: crx_canonical_tests --roots c0 [c1 ... c8]\n", stderr);
    return 2;
  }
  Coefficients coefficients{};
  for (int i = 2; i < argc; ++i) {
    char *end = nullptr;
    const double value = std::strtod(argv[i], &end);
    if (end == argv[i] || *end != '\0' || !std::isfinite(value)) {
      return 2;
    }
    coefficients[static_cast<std::size_t>(i - 2)] = value;
  }
  const auto allocations = crx::test::AllocationCount();
  const bool previous = Eigen::internal::is_malloc_allowed();
  Eigen::internal::set_is_malloc_allowed(false);
  const auto result = FindRootCandidates(coefficients);
  Eigen::internal::set_is_malloc_allowed(previous);
  Require(crx::test::AllocationCount() == allocations, "root probe allocated");
  std::printf("{\"status\":\"%s\",\"degree\":%d,\"roots\":[",
              StatusName(result.status), result.represented_degree);
  for (std::size_t i = 0; i < result.count; ++i) {
    const auto &candidate = result.candidates[i];
    std::printf("%s[%.17g,%.17g,%.17g]", i == 0 ? "" : ",",
                candidate.value.real(), candidate.value.imag(),
                candidate.relative_residual);
  }
  std::puts("]}");
  return 0;
}
