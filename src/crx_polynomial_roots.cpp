#include "crx_polynomial_roots.h"
#include "crx_canonical.h"

#include <algorithm>
#include <cmath>
#include <complex>
#include <cstddef>

#include <Eigen/Core>
#include <Eigen/Eigenvalues>

namespace crx::canonical {
namespace {

auto IsFinite(const std::complex<double> &value) -> bool {
  return std::isfinite(value.real()) && std::isfinite(value.imag());
}

// Evaluate the componentwise relative residual without large powers of the
// root. For |z|>1 evaluate z^-degree P(z) in the reciprocal variable instead.
auto RelativeResidual(const Coefficients &coefficients, int degree,
                      const std::complex<double> &root) -> double {
  const double magnitude = std::abs(root);
  const bool reciprocal = magnitude > 1.0;
  const std::complex<double> argument = reciprocal ? 1.0 / root : root;
  const double absolute_argument = std::abs(argument);
  std::complex<double> value{};
  double bound = 0.0;
  for (int step = 0; step <= degree; ++step) {
    const auto index =
        static_cast<std::size_t>(reciprocal ? step : degree - step);
    value = value * argument + coefficients[index];
    bound = bound * absolute_argument + std::abs(coefficients[index]);
  }
  // P(z)=z^d at z=0 has both numerator and bound exactly zero.
  return bound == 0.0 ? std::abs(value) : std::abs(value) / bound;
}

auto IsUsableScale(double original, double scaled) -> bool {
  return std::isfinite(scaled) && (original == 0.0 || scaled != 0.0);
}

template <int Degree>
auto Balance(Eigen::Matrix<double, Degree, Degree> &matrix, int sweep_limit,
             int &sweeps) -> RootStatus {
  constexpr double kRequiredImprovement = 0.95;
  for (int sweep = 0; sweep < sweep_limit; ++sweep) {
    ++sweeps;
    bool changed = false;
    for (int i = 0; i < Degree; ++i) {
      double row_norm = 0.0;
      double column_norm = 0.0;
      for (int j = 0; j < Degree; ++j) {
        if (i != j) {
          row_norm += std::abs(matrix(i, j));
          column_norm += std::abs(matrix(j, i));
        }
      }
      if (!std::isfinite(row_norm + column_norm)) {
        return RootStatus::NumericalRangeFailure;
      }
      if (row_norm == 0.0 || column_norm == 0.0) {
        continue;
      }
      // Minimize r/2^e + c*2^e without forming r/c, which may overflow.
      const int exponent = (std::ilogb(row_norm) - std::ilogb(column_norm)) / 2;
      const double improved =
          std::scalbn(row_norm, -exponent) + std::scalbn(column_norm, exponent);
      if (exponent == 0 ||
          improved >= kRequiredImprovement * (row_norm + column_norm)) {
        continue;
      }
      for (int j = 0; j < Degree; ++j) {
        if (i == j) {
          continue;
        }
        const double row_entry = std::scalbn(matrix(i, j), -exponent);
        const double column_entry = std::scalbn(matrix(j, i), exponent);
        if (!IsUsableScale(matrix(i, j), row_entry) ||
            !IsUsableScale(matrix(j, i), column_entry)) {
          return RootStatus::NumericalRangeFailure;
        }
        matrix(i, j) = row_entry;
        matrix(j, i) = column_entry;
      }
      changed = true;
    }
    if (!changed) {
      return RootStatus::Candidates;
    }
  }
  return RootStatus::BalanceWorkLimit;
}

template <int Degree>
void SolveCompanion(const Coefficients &coefficients,
                    const RootOptions &options, RootResult &result) {
  using Matrix = Eigen::Matrix<double, Degree, Degree>;
  Matrix companion = Matrix::Zero();
  for (int row = 0; row < Degree; ++row) {
    const double coefficient = coefficients[static_cast<std::size_t>(row)];
    const double monic = -coefficient / coefficients[Degree];
    if (!IsUsableScale(coefficient, monic)) {
      result.status = RootStatus::NumericalRangeFailure;
      return;
    }
    companion(row, Degree - 1) = monic;
    if (row > 0) {
      companion(row, row - 1) = 1.0;
    }
  }
  result.status =
      Balance(companion, options.max_balance_sweeps, result.balance_sweeps);
  if (result.status != RootStatus::Candidates) {
    return;
  }
  if (options.max_eigen_iterations == 0) {
    result.status = RootStatus::EigenWorkLimit;
    return;
  }
  Eigen::EigenSolver<Matrix> solver;
  solver.setMaxIterations(options.max_eigen_iterations);
  solver.compute(companion, false);
  if (solver.info() != Eigen::Success) {
    result.status = solver.info() == Eigen::NoConvergence
                        ? RootStatus::EigenWorkLimit
                        : RootStatus::NumericalRangeFailure;
    return;
  }
  for (int i = 0; i < Degree; ++i) {
    const std::complex<double> root = solver.eigenvalues()[i];
    if (!IsFinite(root)) {
      result.status = RootStatus::NumericalRangeFailure;
      return;
    }
    const double residual = RelativeResidual(coefficients, Degree, root);
    if (!std::isfinite(residual)) {
      result.status = RootStatus::NumericalRangeFailure;
      return;
    }
    result.candidates[static_cast<std::size_t>(i)] = {root, residual};
  }
  // Publish a complete list only after all numerical checks succeed.
  result.count = Degree;
}

} // namespace

auto FindRootCandidates(const Coefficients &coefficients,
                        const RootOptions &options) -> RootResult {
  RootResult result;
  if (options.max_balance_sweeps < 0 || options.max_eigen_iterations < 0 ||
      !std::isfinite(options.leading_relative_tolerance) ||
      options.leading_relative_tolerance < 0.0 ||
      options.leading_relative_tolerance > 1.0) {
    return result;
  }
  double scale = 0.0;
  for (const double coefficient : coefficients) {
    if (!std::isfinite(coefficient)) {
      return result;
    }
    scale = std::max(scale, std::abs(coefficient));
  }
  const int degree = RepresentedDegree(coefficients);
  result.represented_degree = degree;
  if (degree <= 0) {
    result.status = degree < 0 ? RootStatus::ZeroPolynomial
                               : RootStatus::ConstantPolynomial;
    return result;
  }
  if (std::abs(coefficients[static_cast<std::size_t>(degree)]) / scale <
      options.leading_relative_tolerance) {
    result.status = RootStatus::UnresolvedLeadingCoefficient;
    return result;
  }
  // Common power-of-two scaling controls residual evaluation without changing
  // monic ratios. Reject coefficients that disappear during normalization.
  const int exponent = std::ilogb(scale);
  Coefficients normalized{};
  for (std::size_t i = 0; i < coefficients.size(); ++i) {
    normalized[i] = std::scalbn(coefficients[i], -exponent);
    if (!IsUsableScale(coefficients[i], normalized[i])) {
      result.status = RootStatus::NumericalRangeFailure;
      return result;
    }
  }
  result.status = RootStatus::Candidates;
  if (degree == 1) {
    const double root = -normalized[0] / normalized[1];
    if (!IsUsableScale(normalized[0], root)) {
      result.status = RootStatus::NumericalRangeFailure;
      return result;
    }
    const double residual = RelativeResidual(normalized, degree, root);
    if (!std::isfinite(residual)) {
      result.status = RootStatus::NumericalRangeFailure;
      return result;
    }
    result.candidates[0] = {{root, 0.0}, residual};
    result.count = 1;
    return result;
  }
  switch (degree) {
  case 2:
    SolveCompanion<2>(normalized, options, result);
    break;
  case 3:
    SolveCompanion<3>(normalized, options, result);
    break;
  case 4:
    SolveCompanion<4>(normalized, options, result);
    break;
  case 5:
    SolveCompanion<5>(normalized, options, result);
    break;
  case 6:
    SolveCompanion<6>(normalized, options, result);
    break;
  case 7:
    SolveCompanion<7>(normalized, options, result);
    break;
  case 8:
    SolveCompanion<8>(normalized, options, result);
    break;
  default:
    result.status = RootStatus::InvalidInput;
    break;
  }
  return result;
}

} // namespace crx::canonical
