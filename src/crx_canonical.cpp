// SPDX-License-Identifier: Apache-2.0
// SPDX-FileCopyrightText: 2026 Centre Technologique en Aérospatiale
/**
 * @file crx_canonical.cpp
 * @brief Implementation of canonical kinematic solution utilities.
 */

#include "crx_canonical.h"
#include "crx_numerics.h"
#include "crx_scalar_kernels.h"
#include "crx_types.h"

#include <Eigen/Core>
#include <cmath>
#include <cstddef>
#include <limits>

namespace crx::canonical {
namespace {

auto IsRigidTarget(const PoseIsoRT &target) -> bool {
  if (!target.matrix().allFinite() ||
      !(target.matrix().row(3) - Eigen::RowVector4d(0, 0, 0, 1))
           .isZero(numerics::kUnitTolerance)) {
    return false;
  }
  const Mat3 gram = target.linear().transpose() * target.linear();
  return (gram - Mat3::Identity()).isZero(numerics::kUnitTolerance) &&
         numerics::Scalar(target.linear().determinant() - 1.0)
             .isZero(numerics::kUnitTolerance);
}

auto ResidualForAxis(const WristCircle &circle, const Vec3 &axis) -> double {
  return scalar::ResidualForAxis(circle.lengths, circle.p, axis);
}

auto ConstructCoefficients(const WristCircle &circle, const Vec3 &v,
                           const Vec3 &w) -> Coefficients {
  return scalar::ConstructCoefficients(circle.lengths, circle.p, v, w);
}

} // namespace

auto PrepareWristCircle(const Lengths &lengths, const PoseIsoRT &target,
                        WristCircle &output) -> PreparationStatus {
  const Eigen::Array4d physical{lengths.a, lengths.b, lengths.c, lengths.r};
  if (!physical.allFinite() || numerics::IsRepresentedZero(lengths.a) ||
      numerics::IsRepresentedZero(lengths.b) || !IsRigidTarget(target)) {
    return PreparationStatus::InvalidInput;
  }
  WristCircle prepared;
  prepared.length_scale = physical.abs().maxCoeff();
  // Keep division rather than a reciprocal that can overflow for tiny scales.
  const Eigen::Array4d normalized =
      physical / Eigen::Array4d::Constant(prepared.length_scale);
  if ((!physical.unaryExpr(&numerics::IsRepresentedZero) &&
       normalized.unaryExpr(&numerics::IsRepresentedZero))
          .any()) {
    return PreparationStatus::NumericalRangeFailure;
  }
  prepared.lengths = {normalized[0], normalized[1], normalized[2],
                      normalized[3]};
  prepared.v = target.linear().col(0);
  prepared.w = target.linear().col(1);
  // Divide before subtracting to avoid overflow in otherwise representable
  // large-scale problems. Do not form physical powers of L.
  prepared.p = target.translation() / prepared.length_scale -
               prepared.lengths.c * target.linear().col(2);
  if (!prepared.p.allFinite() || !std::isfinite(prepared.p.squaredNorm())) {
    return PreparationStatus::NumericalRangeFailure;
  }
  output = prepared;
  return PreparationStatus::Ready;
}

auto EvaluateResidual(const WristCircle &circle, double angle) -> double {
  if (!std::isfinite(angle)) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  const Vec3 axis = circle.v * std::cos(angle) + circle.w * std::sin(angle);
  return ResidualForAxis(circle, axis);
}

auto EvaluateResidualDerivative(const WristCircle &circle, double angle)
    -> double {
  if (!std::isfinite(angle)) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  const Vec3 axis = circle.v * std::cos(angle) + circle.w * std::sin(angle);
  // Differentiate the same denominator-cleared sphere residual with respect
  // to wrist phase. Keeping the derivative in vector form avoids expanding a
  // second, larger trigonometric polynomial just for root polishing.
  const Vec3 axis_derivative =
      -circle.v * std::sin(angle) + circle.w * std::cos(angle);
  const Vec3 x = circle.p + circle.lengths.r * axis;
  const Vec3 x_derivative = circle.lengths.r * axis_derivative;
  const Vec3 n = Vec3::UnitZ().cross(x);
  const Vec3 n_derivative = Vec3::UnitZ().cross(x_derivative);
  const double a_squared = circle.lengths.a * circle.lengths.a;
  const double k =
      (a_squared + x.squaredNorm() - (circle.lengths.b * circle.lengths.b)) / 2;
  const double k_derivative = x.dot(x_derivative);
  const Vec3 axis_cross_n = axis.cross(n);
  const Vec3 axis_cross_n_derivative =
      axis_derivative.cross(n) + axis.cross(n_derivative);
  const double determinant = x.dot(axis_cross_n);
  const double determinant_derivative =
      x_derivative.dot(axis_cross_n) + x.dot(axis_cross_n_derivative);
  const double x_axis = x.dot(axis);
  const double x_axis_derivative =
      x_derivative.dot(axis) + x.dot(axis_derivative);
  const Vec3 n_cross_x = n.cross(x);
  const Vec3 n_cross_x_derivative =
      n_derivative.cross(x) + n.cross(x_derivative);
  const Vec3 numerator = k * axis_cross_n + x_axis * n_cross_x;
  const Vec3 numerator_derivative =
      k_derivative * axis_cross_n + k * axis_cross_n_derivative +
      x_axis_derivative * n_cross_x + x_axis * n_cross_x_derivative;
  return (2 * numerator.dot(numerator_derivative)) -
         (2 * a_squared * determinant * determinant_derivative);
}

auto BuildHalfAnglePolynomial(const WristCircle &circle, double chart_angle,
                              HalfAnglePolynomial &output)
    -> PreparationStatus {
  if (!std::isfinite(chart_angle)) {
    return PreparationStatus::InvalidInput;
  }
  const double cosine = std::cos(chart_angle);
  const double sine = std::sin(chart_angle);
  // Rotate the phase basis so t=0 is the requested chart center. The half-
  // angle chart omits its antipodal phase (t=infinity), checked separately.
  const Vec3 v = circle.v * cosine + circle.w * sine;
  const Vec3 w = -circle.v * sine + circle.w * cosine;
  HalfAnglePolynomial polynomial;
  polynomial.coefficients = ConstructCoefficients(circle, v, w);
  polynomial.chart_angle = chart_angle;
  polynomial.omitted_point_residual = ResidualForAxis(circle, -v);
  const Eigen::Map<const Eigen::Matrix<double, kCoefficientCount, 1>> mapped(
      polynomial.coefficients.data());
  if (!std::isfinite(polynomial.omitted_point_residual) ||
      !mapped.allFinite()) {
    return PreparationStatus::NumericalRangeFailure;
  }
  output = polynomial;
  return PreparationStatus::Ready;
}

auto RepresentedDegree(const Coefficients &coefficients) -> int {
  for (std::size_t i = coefficients.size(); i > 0; --i) {
    if (!numerics::IsRepresentedZero(coefficients[i - 1])) {
      return static_cast<int>(i - 1);
    }
  }
  return -1;
}

auto EvaluatePolynomial(const Coefficients &coefficients, double argument)
    -> PolynomialValue {
  PolynomialValue result;
  for (std::size_t i = coefficients.size(); i > 0; --i) {
    result.derivative = (result.derivative * argument) + result.value;
    result.value = (result.value * argument) + coefficients[i - 1];
  }
  return result;
}

} // namespace crx::canonical
