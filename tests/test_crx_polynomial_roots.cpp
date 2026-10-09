#include "crx_allocation_probe.h"
#include "crx_canonical.h"
#include "crx_polynomial_roots.h"
#include "crx_root_tests.h"

#include <array>
#include <cmath>
#include <complex>
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
  // Every fixed-size instantiation, including first construction, is called
  // inside the allocation guards in main. Match roots one-to-one.
  Coefficients polynomial{1};
  for (int degree = 1; degree <= 8; ++degree) {
    const double root = 0.25 * degree;
    for (int i = degree; i > 0; --i) {
      const auto index = static_cast<std::size_t>(i);
      polynomial[index] = polynomial[index - 1] - root * polynomial[index];
    }
    polynomial[0] *= -root;
    for (const int exponent : {-500, 0, 500}) {
      Coefficients scaled{};
      for (std::size_t i = 0; i < scaled.size(); ++i) {
        scaled[i] = -std::scalbn(polynomial[i], exponent);
      }
      const auto result = FindRootCandidates(scaled);
      Require(result.status == RootStatus::Candidates,
              "factored polynomial converges");
      Require(result.count == static_cast<std::size_t>(degree),
              "candidate count matches degree");
      std::array<bool, 8> used{};
      for (int expected = 1; expected <= degree; ++expected) {
        bool matched = false;
        for (std::size_t i = 0; i < result.count; ++i) {
          if (!used[i] &&
              std::abs(result.candidates[i].value - 0.25 * expected) < 1e-7) {
            used[i] = true;
            matched = true;
            break;
          }
        }
        Require(matched, "separated real root present exactly once");
      }
      for (std::size_t i = 0; i < result.count; ++i) {
        Require(result.candidates[i].relative_residual < 1e-13,
                "relative residual");
      }
    }
    Coefficients monomial{};
    monomial[static_cast<std::size_t>(degree)] = 1;
    const auto zeros = FindRootCandidates(monomial);
    Require(zeros.status == RootStatus::Candidates &&
                zeros.count == static_cast<std::size_t>(degree),
            "zero root repetitions");
    for (std::size_t i = 0; i < zeros.count; ++i) {
      Require(zeros.candidates[i].value == std::complex<double>{},
              "exact zero eigenvalue");
    }
  }
  const auto complex = FindRootCandidates({1e-20, 0, 1});
  Require(complex.status == RootStatus::Candidates && complex.count == 2,
          "near-real complex pair");
  Require(std::abs(complex.candidates[0].value.imag()) > 9e-11 &&
              std::abs(complex.candidates[1].value.imag()) > 9e-11,
          "small imaginary parts preserved");
  const auto balanced = FindRootCandidates({100, -10000.01, 1});
  Require(balanced.status == RootStatus::Candidates &&
              balanced.balance_sweeps > 1,
          "disparate roots exercise balancing");
  RootOptions one_sweep;
  one_sweep.max_balance_sweeps = 1;
  ExpectStatus({100, -10000.01, 1}, RootStatus::BalanceWorkLimit, one_sweep);
  ExpectStatus({}, RootStatus::ZeroPolynomial);
  ExpectStatus({3}, RootStatus::ConstantPolynomial);
  ExpectStatus({1, 0, 1e-20}, RootStatus::UnresolvedLeadingCoefficient);
  Require(FindRootCandidates({1, 0, 1e-20}).represented_degree == 2,
          "small leading term retained");
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
  ExpectStatus({std::numeric_limits<double>::denorm_min(), 1e300},
               RootStatus::NumericalRangeFailure);
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
