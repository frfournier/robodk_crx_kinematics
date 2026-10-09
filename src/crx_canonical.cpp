#include "crx_canonical.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace crx::canonical {
namespace {

// Array sizes express the algebraic degree; multiplication cannot silently
// truncate terms, including for the intermediate quartics in appendix G4.
template <std::size_t N> using Polynomial = std::array<double, N>;

template <std::size_t N>
auto Add(const Polynomial<N> &left, const Polynomial<N> &right,
         double right_scale = 1.0) -> Polynomial<N> {
  Polynomial<N> result{};
  for (std::size_t i = 0; i < N; ++i) {
    result[i] = left[i] + right_scale * right[i];
  }
  return result;
}

template <std::size_t N>
auto Scale(const Polynomial<N> &polynomial, double factor) -> Polynomial<N> {
  Polynomial<N> result{};
  for (std::size_t i = 0; i < N; ++i) {
    result[i] = polynomial[i] * factor;
  }
  return result;
}

template <std::size_t N, std::size_t M>
auto Multiply(const Polynomial<N> &left, const Polynomial<M> &right)
    -> Polynomial<N + M - 1> {
  Polynomial<N + M - 1> result{};
  for (std::size_t i = 0; i < N; ++i) {
    for (std::size_t j = 0; j < M; ++j) {
      result[i + j] += left[i] * right[j];
    }
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
  constexpr Polynomial<3> kQ{1.0, 0.0, 1.0};
  const double a_squared = circle.lengths.a * circle.lengths.a;
  const double b_squared = circle.lengths.b * circle.lengths.b;
  const double r = circle.lengths.r;
  const double p_squared = circle.p.squaredNorm();
  const double pz = circle.p.z();
  const double pv = circle.p.dot(v);
  const Polynomial<3> d{pv, 2.0 * circle.p.dot(w), -pv};
  const Polynomial<3> uz{v.z(), 2.0 * w.z(), -v.z()};
  const auto z = Add(Scale(kQ, pz), uz, r);
  const auto s = Add(Scale(kQ, p_squared + r * r), d, 2.0 * r);
  const auto g = Add(d, kQ, r);
  const auto a_hat = Add(s, kQ, b_squared - a_squared);
  const auto sphere = Add(s, kQ, a_squared - b_squared);
  const auto d_hat = Add(Scale(Multiply(s, kQ), 4.0 * a_squared),
                         Multiply(sphere, sphere), -1.0);
  const auto first =
      Multiply(Add(Multiply(a_hat, a_hat), Multiply(z, z), -4.0 * b_squared),
               Multiply(g, g));
  const auto second = Multiply(
      d_hat, Multiply(uz, Add(Scale(g, 2.0 * pz), uz, r * r - p_squared)));
  return Scale(Add(first, second), 0.25);
}

} // namespace

auto PrepareWristCircle(const Lengths &lengths, const PoseIsoRT &target,
                        WristCircle &output) -> PreparationStatus {
  const std::array<double, 4> physical{lengths.a, lengths.b, lengths.c,
                                       lengths.r};
  if (!std::all_of(physical.begin(), physical.end(),
                   [](double value) { return std::isfinite(value); }) ||
      lengths.a == 0.0 || lengths.b == 0.0 || !IsRigidTarget(target)) {
    return PreparationStatus::InvalidInput;
  }
  WristCircle prepared;
  prepared.length_scale = std::max({std::abs(lengths.a), std::abs(lengths.b),
                                    std::abs(lengths.c), std::abs(lengths.r)});
  std::array<double, 4> normalized{};
  for (std::size_t i = 0; i < physical.size(); ++i) {
    normalized[i] = physical[i] / prepared.length_scale;
    if (physical[i] != 0.0 && normalized[i] == 0.0) {
      return PreparationStatus::NumericalRangeFailure;
    }
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
  if (!std::isfinite(polynomial.omitted_point_residual) ||
      !std::all_of(polynomial.coefficients.begin(),
                   polynomial.coefficients.end(),
                   [](double value) { return std::isfinite(value); })) {
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
