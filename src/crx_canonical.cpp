#include "crx_canonical.h"

#include <algorithm>
#include <cmath>
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

auto IsRigidTarget(const PoseIsoRT &target) -> bool {
  constexpr double kRotationTolerance =
      128.0 * std::numeric_limits<double>::epsilon();
  if (!target.matrix().allFinite() || target.matrix()(3, 0) != 0.0 ||
      target.matrix()(3, 1) != 0.0 || target.matrix()(3, 2) != 0.0 ||
      target.matrix()(3, 3) != 1.0) {
    return false;
  }
  const Mat3 gram = target.linear().transpose() * target.linear();
  return (gram - Mat3::Identity()).cwiseAbs().maxCoeff() <=
             kRotationTolerance &&
         std::abs(target.linear().determinant() - 1.0) <= kRotationTolerance;
}

auto ResidualForAxis(const WristCircle &circle, const Vec3 &axis) -> double {
  const Vec3 x = circle.p + circle.lengths.r * axis;
  const Vec3 n = Vec3::UnitZ().cross(x);
  const double a_squared = circle.lengths.a * circle.lengths.a;
  const double b_squared = circle.lengths.b * circle.lengths.b;
  const double k = (a_squared + x.squaredNorm() - b_squared) / 2.0;
  const Vec3 axis_cross_n = axis.cross(n);
  const double determinant = x.dot(axis_cross_n);
  const Vec3 numerator = k * axis_cross_n + x.dot(axis) * n.cross(x);
  return numerator.squaredNorm() - a_squared * determinant * determinant;
}

auto ConstructCoefficients(const WristCircle &circle, const Vec3 &v,
                           const Vec3 &w) -> Coefficients {
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
  // Form endpoint distances from vectors, not p²+r² +/- 2r(p.v). Near a
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
      (4.0 * a_squared) * Multiply(s, kQ) - Multiply(sphere, sphere);
  const Polynomial<5> first_factor =
      Multiply(a_hat, a_hat) - (4.0 * b_squared) * Multiply(z, z);
  const Polynomial<9> first = Multiply(first_factor, Multiply(g, g));
  const Polynomial<3> second_factor = (2.0 * pz) * g + (r * r - p_squared) * uz;
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
  if (!physical.allFinite() || lengths.a == 0.0 || lengths.b == 0.0 ||
      !IsRigidTarget(target)) {
    return PreparationStatus::InvalidInput;
  }
  WristCircle prepared;
  prepared.length_scale = physical.abs().maxCoeff();
  // Keep division rather than a reciprocal that can overflow for tiny scales.
  const Eigen::Array4d normalized =
      physical / Eigen::Array4d::Constant(prepared.length_scale);
  if (((physical != 0.0) && (normalized == 0.0)).any()) {
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

auto BuildHalfAnglePolynomial(const WristCircle &circle, double chart_angle,
                              HalfAnglePolynomial &output)
    -> PreparationStatus {
  if (!std::isfinite(chart_angle)) {
    return PreparationStatus::InvalidInput;
  }
  const double cosine = std::cos(chart_angle);
  const double sine = std::sin(chart_angle);
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
    if (coefficients[i - 1] != 0.0) {
      return static_cast<int>(i - 1);
    }
  }
  return -1;
}

auto EvaluatePolynomial(const Coefficients &coefficients, double argument)
    -> PolynomialValue {
  PolynomialValue result;
  for (std::size_t i = coefficients.size(); i > 0; --i) {
    result.derivative = result.derivative * argument + result.value;
    result.value = result.value * argument + coefficients[i - 1];
  }
  return result;
}

} // namespace crx::canonical
