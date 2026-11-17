// SPDX-License-Identifier: Apache-2.0
// SPDX-FileCopyrightText: 2026 Centre Technologique en Aérospatiale
/**
 * @file crx_canonical.cpp
 * @brief Implementation of canonical kinematic solution utilities.
 */

#include "crx_canonical.h"
#include "crx_numerics.h"
#include "crx_types.h"

#include <Eigen/Core>
#include <cmath>
#include <cstddef>
#include <limits>

namespace crx::canonical {
namespace {

// Vector sizes express the algebraic degree; multiplication cannot silently
// truncate terms, including for the intermediate quartics in appendix G4.
template <int N> using Polynomial = Eigen::Matrix<double, N, 1>;

template <int N, int M>
auto Multiply(const Polynomial<N> &left, const Polynomial<M> &right)
    -> Polynomial<N + M - 1> {
  Polynomial<N + M - 1> result = Polynomial<N + M - 1>::Zero();
  for (int i = 0; i < N; ++i) {
    result.template segment<M>(i) += left[i] * right;
  }
  return result;
}

constexpr int kQuadraticSize = 3;
constexpr int kQuarticSize = 5;

auto Square(const Polynomial<kQuadraticSize> &polynomial)
    -> Polynomial<kQuarticSize> {
  const double cross01 = polynomial.x() * polynomial.y();
  const double cross02 = polynomial.x() * polynomial.z();
  const double cross12 = polynomial.y() * polynomial.z();
  // Reuse symmetric products while preserving the convolution's accumulation
  // order, especially the cancellation in the middle coefficient.
  return {polynomial.x() * polynomial.x(), cross01 + cross01,
          (cross02 + (polynomial.y() * polynomial.y())) + cross02,
          cross12 + cross12, polynomial.z() * polynomial.z()};
}

auto MultiplyByHalfAngleDenominator(
    const Polynomial<kQuadraticSize> &polynomial) -> Polynomial<kQuarticSize> {
  // Q(t) = 1 + t^2: only the overlapping middle coefficient needs addition.
  return {polynomial.x(), polynomial.y(), polynomial.x() + polynomial.z(),
          polynomial.y(), polynomial.z()};
}

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
  const Vec3 x = circle.p + circle.lengths.r * axis;
  const Vec3 n = Vec3::UnitZ().cross(x);
  const double a_squared = circle.lengths.a * circle.lengths.a;
  const double b_squared = circle.lengths.b * circle.lengths.b;
  const double k = (a_squared + x.squaredNorm() - b_squared) / 2.0;
  const Vec3 axis_cross_n = axis.cross(n);
  const double determinant = x.dot(axis_cross_n);
  // Cramer's rule gives elbow = numerator/determinant. Clearing that
  // denominator from |elbow|^2=a^2 leaves this residual; its zeros nominate
  // wrist phases whose elbow also lies on the upper-arm sphere.
  const Vec3 numerator = k * axis_cross_n + x.dot(axis) * n.cross(x);
  return numerator.squaredNorm() - (a_squared * determinant * determinant);
}

auto ConstructCoefficients(const WristCircle &circle, const Vec3 &v,
                           const Vec3 &w) -> Coefficients {
  // Set t = tan((angle - chart_angle)/2), so an axis phase has numerator
  // v*(1-t^2) + w*(2t) and denominator Q=1+t^2. The quadratic polynomials
  // below are these numerators (or squared distances) with Q cleared; the
  // final quartic products clear the remaining denominators in the elbow
  // sphere residual, yielding a degree-at-most-eight equation in t.
  const Polynomial<3> kQ{1.0, 0.0, 1.0};
  const double a_squared = circle.lengths.a * circle.lengths.a;
  const double b_squared = circle.lengths.b * circle.lengths.b;
  const double r = circle.lengths.r;
  const double p_squared = circle.p.squaredNorm();
  const double pz = circle.p.z();
  const double pv = circle.p.dot(v);
  const Polynomial<3> d{pv, 2.0 * circle.p.dot(w), -pv};
  const Polynomial<3> uz{v.z(), 2.0 * w.z(), -v.z()};
  const Polynomial<3> z = pz * kQ + r * uz;
  // Form endpoint distances from vectors, not pÂ²+rÂ² +/- 2r(p.v). Near a
  // folded equal-link arm those large scalar terms cancel and erase the
  // closely spaced roots before the eigensolver even sees the polynomial.
  const Polynomial<3> s{(circle.p + r * v).squaredNorm(),
                        4.0 * r * circle.p.dot(w),
                        (circle.p - r * v).squaredNorm()};
  const Polynomial<3> g = d + r * kQ;
  const double arm_difference = (circle.lengths.a - circle.lengths.b) *
                                (circle.lengths.a + circle.lengths.b);
  const Polynomial<3> a_hat = s - arm_difference * kQ;
  const Polynomial<3> sphere = s + arm_difference * kQ;
  const Polynomial<5> d_hat =
      (4.0 * a_squared) * MultiplyByHalfAngleDenominator(s) - Square(sphere);
  const Polynomial<5> first_factor =
      Square(a_hat) - (4.0 * b_squared) * Square(z);
  const Polynomial<9> first = Multiply(first_factor, Square(g));
  const Polynomial<3> second_factor =
      (2.0 * pz) * g + ((r * r) - p_squared) * uz;
  const Polynomial<9> second = Multiply(d_hat, Multiply(uz, second_factor));
  Coefficients result{};
  Eigen::Map<Polynomial<9>> mapped(result.data());
  mapped = 0.25 * (first + second);
  return result;
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
  auto &invariants = prepared.residual;
  invariants.p_v = prepared.p.dot(prepared.v);
  invariants.p_w = prepared.p.dot(prepared.w);
  invariants.v_z = prepared.v.z();
  invariants.w_z = prepared.w.z();
  invariants.p_squared_plus_r_squared =
      prepared.p.squaredNorm() + (prepared.lengths.r * prepared.lengths.r);
  invariants.height = prepared.p.z();
  invariants.a_squared = prepared.lengths.a * prepared.lengths.a;
  invariants.b_squared = prepared.lengths.b * prepared.lengths.b;
  invariants.arm_difference = (prepared.lengths.a - prepared.lengths.b) *
                              (prepared.lengths.a + prepared.lengths.b);
  // The scalar identity uses a unit phase axis. Accepted near-rigid inputs
  // outside ordinary basis roundoff keep their original geometric residual.
  constexpr double kScalarBasisTolerance = 8 * numerics::kEpsilon;
  invariants.unit_basis =
      std::abs(prepared.v.squaredNorm() - 1.0) <= kScalarBasisTolerance &&
      std::abs(prepared.w.squaredNorm() - 1.0) <= kScalarBasisTolerance &&
      std::abs(prepared.v.dot(prepared.w)) <= kScalarBasisTolerance;
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

namespace {
auto TryBivariateResidual(const WristCircle &circle, double cosine, double sine,
                          ResidualEvaluation &output) -> bool {
  const auto &invariants = circle.residual;
  constexpr double kCancellationFraction = 0.125;
  if (invariants.unit_basis) {
    const double m = (invariants.p_v * cosine) + (invariants.p_w * sine);
    const double u = (invariants.v_z * cosine) + (invariants.w_z * sine);
    const double r = circle.lengths.r;
    const double s = invariants.p_squared_plus_r_squared + (2 * r * m);
    const double z = invariants.height + (r * u);
    const double g = m + r;
    if (!std::isfinite(s) || !std::isfinite(g) ||
        !(std::abs(s) > kCancellationFraction *
                            (std::abs(invariants.p_squared_plus_r_squared) +
                             (2 * std::abs(r * m)))) ||
        !(std::abs(g) > kCancellationFraction * (std::abs(m) + std::abs(r)))) {
      return false;
    }
    const double k = (s + invariants.arm_difference) / 2;
    const double j = (s - invariants.arm_difference) / 2;
    const double e = (k * k) - (invariants.a_squared * s);
    const double p_minus_r = invariants.p_squared_plus_r_squared - (2 * r * r);
    const double t = (p_minus_r * u) - (2 * invariants.height * g);
    const double v = (j * j) - (invariants.b_squared * z * z);
    const double value = (e * u * t) + (g * g * v);
    const double e_scale = (k * k) + std::abs(invariants.a_squared * s);
    const double t_scale =
        std::abs(p_minus_r * u) + (2 * std::abs(invariants.height * g));
    const double v_scale = (j * j) + (invariants.b_squared * z * z);
    const double value_scale =
        (e_scale * std::abs(u) * t_scale) + (g * g * v_scale);
    if (!std::isfinite(value_scale) || !std::isfinite(value) ||
        !(std::abs(value) > kCancellationFraction * value_scale)) {
      return false;
    }
    // Avoid derivative/guard work when a cancelled value already requires the
    // geometric route. The dispatch conditions are otherwise unchanged.
    const double dm = (-invariants.p_v * sine) + (invariants.p_w * cosine);
    const double du = (-invariants.v_z * sine) + (invariants.w_z * cosine);
    const double ds = 2 * r * dm;
    const double dz = r * du;
    const double dk = ds / 2;
    const double de = (2 * k * dk) - (invariants.a_squared * ds);
    const double dt = (p_minus_r * du) - (2 * invariants.height * dm);
    const double dv = (2 * j * dk) - (2 * invariants.b_squared * z * dz);
    const double derivative = (de * u * t) + (e * du * t) + (e * u * dt) +
                              (2 * g * dm * v) + (g * g * dv);
    const double de_scale =
        (2 * std::abs(k * dk)) + std::abs(invariants.a_squared * ds);
    const double dt_scale =
        std::abs(p_minus_r * du) + (2 * std::abs(invariants.height * dm));
    const double dv_scale =
        (2 * std::abs(j * dk)) + (2 * invariants.b_squared * std::abs(z * dz));
    const double derivative_scale = (de_scale * std::abs(u) * t_scale) +
                                    (e_scale * std::abs(du) * t_scale) +
                                    (e_scale * std::abs(u) * dt_scale) +
                                    (2 * std::abs(g * dm) * v_scale) +
                                    (g * g * dv_scale);
    // A conservative cancellation dispatch, not a proved error bound. Near
    // zeros, folded distances, and base-axis cancellations, retain the vector
    // evaluation rather than letting scalar subtraction manufacture a root.
    if (std::isfinite(derivative_scale) && std::isfinite(derivative) &&
        std::abs(derivative) > kCancellationFraction * derivative_scale) {
      output = {value, derivative, false};
      return true;
    }
  }
  return false;
}

auto GeometricResidualPair(const WristCircle &circle, double cosine,
                           double sine) -> ResidualEvaluation {
  const Vec3 axis = circle.v * cosine + circle.w * sine;
  // Differentiate the same denominator-cleared sphere residual with respect
  // to wrist phase. Keeping the derivative in vector form avoids expanding a
  // second, larger trigonometric polynomial just for root polishing.
  const Vec3 axis_derivative = -circle.v * sine + circle.w * cosine;
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
  return {numerator.squaredNorm() - (a_squared * determinant * determinant),
          (2 * numerator.dot(numerator_derivative)) -
              (2 * a_squared * determinant * determinant_derivative),
          true};
}
} // namespace

auto EvaluateResidualPair(const WristCircle &circle, double angle)
    -> ResidualEvaluation {
  if (!std::isfinite(angle)) {
    const double nan = std::numeric_limits<double>::quiet_NaN();
    return {nan, nan, true};
  }
  const double cosine = std::cos(angle);
  const double sine = std::sin(angle);
  ResidualEvaluation result;
  if (TryBivariateResidual(circle, cosine, sine, result)) {
    return result;
  }
  return GeometricResidualPair(circle, cosine, sine);
}

auto EvaluateCramerElbowResidual(const WristCircle &circle, double angle,
                                 CramerEvaluation &output) -> CramerStatus {
  if (!std::isfinite(angle)) {
    return CramerStatus::NumericalRangeFailure;
  }
  const double cosine = std::cos(angle);
  const double sine = std::sin(angle);
  const Vec3 u = (circle.v * cosine) + (circle.w * sine);
  const Vec3 du = (-circle.v * sine) + (circle.w * cosine);
  const Vec3 x = circle.p + (circle.lengths.r * u);
  const Vec3 dx = circle.lengths.r * du;
  const double s = x.squaredNorm();
  if (!std::isfinite(s) ||
      s <= numerics::kDirectionTolerance * numerics::kDirectionTolerance) {
    return CramerStatus::NumericalRangeFailure;
  }
  const double horizontal_dot = (x.x() * u.x()) + (x.y() * u.y());
  const double horizontal_squared = (x.x() * x.x()) + (x.y() * x.y());
  const double delta = (x.z() * horizontal_dot) - (u.z() * horizontal_squared);
  if (!std::isfinite(delta) ||
      !(std::abs(delta) >= numerics::kCramerGuard * s)) {
    return CramerStatus::GuardRejected;
  }
  const double g = horizontal_dot + (x.z() * u.z());
  const double dg_horizontal =
      (dx.x() * u.x()) + (x.x() * du.x()) + (dx.y() * u.y()) + (x.y() * du.y());
  const double dg = dg_horizontal + (dx.z() * u.z()) + (x.z() * du.z());
  const double d_horizontal_squared = 2 * ((x.x() * dx.x()) + (x.y() * dx.y()));
  const double ddelta = (dx.z() * horizontal_dot) + (x.z() * dg_horizontal) -
                        (du.z() * horizontal_squared) -
                        (u.z() * d_horizontal_squared);
  const double k = (circle.residual.arm_difference + s) / 2;
  const double dk = x.dot(dx);
  const double coefficient = (x.z() * g) - (k * u.z());
  const double dcoefficient =
      (dx.z() * g) + (x.z() * dg) - (dk * u.z()) - (k * du.z());
  // Scalar triple-product identities, with N.z and Delta expressed using
  // horizontal components. Avoid subtracting z*z from |x|^2 near the axis.
  const Vec3 numerator(coefficient * x.x(), coefficient * x.y(),
                       (k * horizontal_dot) - (g * horizontal_squared));
  const Vec3 dnumerator((dcoefficient * x.x()) + (coefficient * dx.x()),
                        (dcoefficient * x.y()) + (coefficient * dx.y()),
                        (dk * horizontal_dot) + (k * dg_horizontal) -
                            (dg * horizontal_squared) -
                            (g * d_horizontal_squared));
  CramerEvaluation result;
  result.elbow = numerator / delta;
  const Vec3 delbow = (dnumerator - (result.elbow * ddelta)) / delta;
  result.value = result.elbow.squaredNorm() - circle.residual.a_squared;
  result.derivative = 2 * result.elbow.dot(delbow);
  if (!result.elbow.allFinite() || !delbow.allFinite() ||
      !std::isfinite(result.value) || !std::isfinite(result.derivative)) {
    return CramerStatus::NumericalRangeFailure;
  }
  output = result;
  return CramerStatus::Ready;
}

auto EvaluateResidualDerivative(const WristCircle &circle, double angle)
    -> double {
  return EvaluateResidualPair(circle, angle).derivative;
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
