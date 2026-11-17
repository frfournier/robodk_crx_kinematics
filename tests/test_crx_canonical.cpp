// SPDX-License-Identifier: Apache-2.0
// SPDX-FileCopyrightText: 2026 Centre Technologique en Aérospatiale
/**
 * @file test_crx_canonical.cpp
 * @brief Tests for crx canonical.
 */

#include "crx_allocation_probe.h"
#include "crx_canonical.h"
#include "crx_incidence_tests.h"
#include "crx_joint_recovery_tests.h"
#include "crx_numerics.h"
#include "crx_polynomial_roots.h"
#include "crx_root_tests.h"

#include <algorithm>
#include <cmath>
#include <complex>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <new>

#if !defined(EIGEN_RUNTIME_NO_MALLOC) || defined(EIGEN_NO_DEBUG)
#error Canonical allocation tests require Eigen malloc guards and assertions.
#endif

namespace {
using crx::PoseIsoRT;
using crx::Vec3;
using namespace crx::canonical;
constexpr double kPi = 3.14159265358979323846;
constexpr std::array<Lengths, 6> kCapturedLengths{{{410, 430, 145, 130},
                                                   {540, 540, 160, 150},
                                                   {710, 540, 160, 150},
                                                   {710, 540, 160, 150},
                                                   {950, 750, 180, 185},
                                                   {950, 750, 180, 185}}};

void Require(bool condition, const char *message) {
  if (!condition) {
    std::fprintf(stderr, "Canonical test failed: %s\n", message);
    std::exit(EXIT_FAILURE);
  }
}

void Near(double actual, double expected, const char *message,
          double tolerance = 5e-11) {
  const double scale = std::max({1.0, std::abs(actual), std::abs(expected)});
  if (!std::isfinite(actual) || !std::isfinite(expected) ||
      std::abs(actual - expected) > tolerance * scale) {
    std::fprintf(stderr, "%s: actual=%.17g expected=%.17g\n", message, actual,
                 expected);
    std::exit(EXIT_FAILURE);
  }
}

auto Rotation(const Vec3 &axis, double angle) -> PoseIsoRT {
  PoseIsoRT result = PoseIsoRT::Identity();
  result.linear() = Eigen::AngleAxisd(angle, axis).toRotationMatrix();
  return result;
}

auto Translation(const Vec3 &offset) -> PoseIsoRT {
  PoseIsoRT result = PoseIsoRT::Identity();
  result.translation() = offset;
  return result;
}

auto Prepare(const Lengths &lengths, const PoseIsoRT &target) -> WristCircle {
  WristCircle circle;
  Require(PrepareWristCircle(lengths, target, circle) ==
              PreparationStatus::Ready,
          "prepare synthetic canonical target");
  return circle;
}

auto PolynomialFor(const WristCircle &circle, double chart = 0.0)
    -> HalfAnglePolynomial {
  HalfAnglePolynomial polynomial;
  Require(BuildHalfAnglePolynomial(circle, chart, polynomial) ==
              PreparationStatus::Ready,
          "construct half-angle polynomial");
  return polynomial;
}

// Independent scalar identities from appendix G2. The implementation evaluates
// cross products; these references use the compact and old triangle
// expressions.
auto CompactResidual(const WristCircle &circle, double angle) -> double {
  const Vec3 u = circle.v * std::cos(angle) + circle.w * std::sin(angle);
  const double a2 = circle.lengths.a * circle.lengths.a;
  const double b2 = circle.lengths.b * circle.lengths.b;
  const double r = circle.lengths.r;
  const double p2 = circle.p.squaredNorm();
  const double d = circle.p.dot(u);
  const double s = p2 + r * r + 2 * r * d;
  const double z = circle.p.z() + r * u.z();
  const double g = d + r;
  const double a0 = s + b2 - a2;
  const double d0 = 4 * a2 * s - (s + a2 - b2) * (s + a2 - b2);
  return ((a0 * a0 - 4 * b2 * z * z) * g * g +
          d0 * u.z() * (2 * circle.p.z() * g - (p2 - r * r) * u.z())) /
         4;
}

auto TriangleResidual(const WristCircle &circle, double angle) -> double {
  const Vec3 u = circle.v * std::cos(angle) + circle.w * std::sin(angle);
  const Vec3 x = circle.p + circle.lengths.r * u;
  const Vec3 triangle_w = circle.p - x;
  const double h = triangle_w.dot(x);
  const double s = x.squaredNorm();
  const double z = x.z();
  const double pz = circle.p.z();
  const double a2 = circle.lengths.a * circle.lengths.a;
  const double b2 = circle.lengths.b * circle.lengths.b;
  const double a0 = s + b2 - a2;
  const double d0 = 4 * a2 * s - (s + a2 - b2) * (s + a2 - b2);
  const double k = circle.p.squaredNorm() - circle.lengths.r * circle.lengths.r;
  return (a0 * a0 - 4 * b2 * z * z) * h * h +
         d0 * (-s * pz * pz + 2 * pz * z * (s + h) - k * z * z);
}

void CheckCharts(const WristCircle &circle) {
  for (const double chart : {0.0, kPi / 2, kPi / 4, -kPi / 4, 0.7, -1.3, kPi}) {
    const auto polynomial = PolynomialFor(circle, chart);
    Near(polynomial.coefficients.back(), polynomial.omitted_point_residual,
         "leading coefficient equals omitted-point residual");
    Near(polynomial.omitted_point_residual,
         EvaluateResidual(circle, chart + kPi),
         "omitted point agrees with geometric residual");
    for (const double tau :
         {-1e8, -7.0, -1.0, -0.125, 0.0, 0.375, 1.0, 9.0, 1e8}) {
      const double angle = chart + 2 * std::atan(tau);
      const double q = 1 + tau * tau;
      const double residual = EvaluateResidual(circle, angle);
      Near(EvaluatePolynomial(polynomial.coefficients, tau).value /
               (q * q * q * q),
           residual, "half-angle polynomial agrees with vector residual");
      Near(CompactResidual(circle, angle), residual,
           "compact residual identity");
      Near(TriangleResidual(circle, angle),
           4 * circle.lengths.r * circle.lengths.r * residual,
           "triangle identity without dividing by radial length");
      if (!crx::numerics::Scalar(tau).isZero(crx::numerics::kEpsilon)) {
        auto reciprocal = polynomial.coefficients;
        std::reverse(reciprocal.begin(), reciprocal.end());
        const double sigma = 1 / tau;
        const double qs = 1 + sigma * sigma;
        Near(EvaluatePolynomial(reciprocal, sigma).value / (qs * qs * qs * qs),
             residual, "reciprocal chart near the half-angle pole");
      }
    }
  }
}

void TestResidualsAndCanonicalWitnesses() {
  for (int sample = 0; sample < 128; ++sample) {
    const double t = static_cast<double>(sample) / 19;
    // Synthetic signed geometry, not dimensions of an approved RoboDK model.
    const Lengths lengths{sample % 2 == 0 ? 2.0 : -2.0, 1.5,
                          sample % 3 == 0 ? -0.25 : 0.25,
                          sample % 5 == 0 ? 0.0 : 0.375};
    auto target = Rotation(Vec3::UnitZ(), t) * Rotation(Vec3::UnitY(), -t / 3) *
                  Rotation(Vec3::UnitX(), t / 7);
    target.translation() = Vec3(3 * std::sin(t), 2 * std::cos(t), t / 3);
    CheckCharts(Prepare(lengths, target));

    // Independent canonical transform product (appendix G1) constructs a known
    // incidence witness. Native production FK does not supply the pose.
    PoseIsoRT fixed = PoseIsoRT::Identity();
    fixed.linear() << 0, 0, 1, 0, -1, 0, 1, 0, 0;
    const auto shoulder =
        Rotation(Vec3::UnitZ(), t) * Rotation(Vec3::UnitY(), t / 2);
    const auto elbow = shoulder * Translation(Vec3(0, 0, lengths.a)) *
                       Rotation(Vec3::UnitY(), -t / 3) *
                       Rotation(Vec3::UnitX(), -t / 5);
    const PoseIsoRT flange =
        elbow * Translation(Vec3(lengths.b, -lengths.r, 0)) *
        Rotation(Vec3::UnitY(), -t / 7) * Translation(Vec3(lengths.c, 0, 0)) *
        Rotation(Vec3::UnitX(), -t / 11) * fixed;
    const auto circle = Prepare(lengths, flange);
    const Vec3 fifth_axis = elbow.linear().col(1);
    const double root_angle =
        std::atan2(fifth_axis.dot(circle.w), fifth_axis.dot(circle.v));
    Near(EvaluateResidual(circle, root_angle), 0,
         "canonical FK incidence witness");
    CheckCharts(circle);
  }
}

struct ResidualOracle {
  double value = 0;
  double derivative = 0;
  double derivative_term_scale = 0;
  Vec3 elbow = Vec3::Zero();
};

auto ComplexStepResidual(const WristCircle &circle, double angle,
                         bool divide_by_determinant = false) -> ResidualOracle {
  // Differentiate the independent cross-product geometry with a complex
  // step. Products are bilinear: Eigen's conjugating complex dot/cross
  // operations are inappropriate for an analytic complex-step extension.
  using Complex = std::complex<double>;
  using Vector = Eigen::Matrix<Complex, 3, 1>;
  constexpr double kStep = 1e-24;
  const Complex phase{angle, kStep};
  const Vector axis = circle.v.cast<Complex>() * std::cos(phase) +
                      circle.w.cast<Complex>() * std::sin(phase);
  const Vector x = circle.p.cast<Complex>() + circle.lengths.r * axis;
  const auto cross = [](const Vector &left, const Vector &right) -> Vector {
    return {left.y() * right.z() - left.z() * right.y(),
            left.z() * right.x() - left.x() * right.z(),
            left.x() * right.y() - left.y() * right.x()};
  };
  const Vector n = cross(Vector(0, 0, 1), x);
  const auto product = [](const Vector &left, const Vector &right) {
    return left.cwiseProduct(right).sum();
  };
  const double a2 = circle.lengths.a * circle.lengths.a;
  const double b2 = circle.lengths.b * circle.lengths.b;
  const Complex k = (a2 + product(x, x) - b2) / 2.0;
  const Complex determinant = product(x, cross(axis, n));
  const Vector numerator = k * cross(axis, n) + product(x, axis) * cross(n, x);
  Complex residual =
      product(numerator, numerator) - a2 * determinant * determinant;
  double term_scale =
      2 * a2 * std::abs(determinant.real() * determinant.imag() / kStep);
  for (Eigen::Index i = 0; i < 3; ++i) {
    term_scale +=
        2 * std::abs(numerator[i].real() * numerator[i].imag() / kStep);
  }
  Vec3 elbow = Vec3::Zero();
  if (divide_by_determinant) {
    const Vector quotient = numerator / determinant;
    residual = product(quotient, quotient) - a2;
    term_scale = 0;
    for (Eigen::Index i = 0; i < 3; ++i) {
      elbow[i] = quotient[i].real();
      term_scale +=
          2 * std::abs(quotient[i].real() * quotient[i].imag() / kStep);
    }
  }
  return {residual.real(), residual.imag() / kStep, term_scale, elbow};
}

void TestBivariateResiduals() {
  std::size_t scalar_count = 0;
  std::size_t fallback_count = 0;
  for (const auto &asset : kCapturedLengths) {
    for (const double scale : {1e-100, 1.0, 1e100}) {
      for (const double sign : {-1.0, 1.0}) {
        const Lengths lengths{asset.a * scale, sign * asset.b * scale,
                              asset.c * scale, sign * asset.r * scale};
        for (int sample = 0; sample < 32; ++sample) {
          const double angle = static_cast<double>(sample) / 7 - 2;
          auto target = Rotation(Vec3::UnitZ(), angle / 3) *
                        Rotation(Vec3::UnitY(), angle / 5) *
                        Rotation(Vec3::UnitX(), angle / 7);
          target.translation() =
              scale * Vec3(300 * std::sin(angle), 400 * std::cos(angle),
                           200 + 100 * angle);
          const auto circle = Prepare(lengths, target);
          const auto result = EvaluateResidualPair(circle, angle);
          scalar_count += !result.used_geometric_fallback;
          fallback_count += result.used_geometric_fallback;
          const double expected_value = EvaluateResidual(circle, angle);
          const auto derivative_oracle = ComplexStepResidual(circle, angle);
          const double expected_derivative = derivative_oracle.derivative;
          // Relative agreement for values separated from zero; the absolute
          // branch budgets roundoff in dimensionless normalized geometry.
          const auto agrees = [](double actual, double expected) {
            const double tolerance = std::abs(expected) > 1e-6
                                         ? 1e-13 * std::abs(expected)
                                         : 256 * crx::numerics::kEpsilon;
            return std::isfinite(actual) && std::isfinite(expected) &&
                   std::abs(actual - expected) <= tolerance;
          };
          Require(agrees(result.value, expected_value),
                  "bivariate value agrees with cross-product geometry");
          // A derivative close to zero relative to its contributing terms is
          // cancellation-conditioned, even when its absolute value is 1e-5.
          // Budget absolute agreement by 64 eps times those terms; otherwise
          // require the specified 1e-13 relative agreement.
          const double derivative_tolerance =
              std::abs(expected_derivative) >
                      0.125 * derivative_oracle.derivative_term_scale
                  ? 1e-13 * std::abs(expected_derivative)
                  : 64 * crx::numerics::kEpsilon *
                        derivative_oracle.derivative_term_scale;
          const bool derivative_agrees =
              std::isfinite(result.derivative) &&
              std::isfinite(expected_derivative) &&
              std::abs(result.derivative - expected_derivative) <=
                  derivative_tolerance;
          if (!derivative_agrees) {
            std::fprintf(
                stderr,
                "derivative mismatch: sample=%d a=%.17g b=%.17g "
                "scale=%.17g fallback=%d actual=%.17g expected=%.17g\n",
                sample, asset.a, lengths.b, scale,
                static_cast<int>(result.used_geometric_fallback),
                result.derivative, expected_derivative);
          }
          Require(derivative_agrees,
                  "bivariate derivative agrees with complex-step oracle");
        }
      }
    }
  }
  Require(scalar_count > 0 && fallback_count > 0,
          "both residual evaluation routes exercised");
  for (const double epsilon : {1e-3, 1e-6, 1e-9, 1e-12, 1e-16}) {
    for (const bool folded : {false, true}) {
      PoseIsoRT target = PoseIsoRT::Identity();
      // Exactly orthonormal phase basis v=ez, w=ex.
      target.linear() << 0, 1, 0, 0, 0, 1, 1, 0, 0;
      target.translation() = Vec3(epsilon, epsilon, folded ? -0.25 : 0.75);
      const auto circle = Prepare({1, 1, 0, 0.25}, target);
      const auto result = EvaluateResidualPair(circle, 0);
      const double squared = epsilon * epsilon;
      const double expected_value =
          folded ? 2 * squared * squared * (squared - 2)
                 : squared * (2 * squared - 1) * (2 * squared - 1) / 2;
      const double expected_derivative =
          folded ? epsilon * squared * (3 * squared - 4) / 2
                 : epsilon * (12 * squared * squared + 24 * squared + 1) / 8;
      Require(result.used_geometric_fallback,
              "near-axis cancellation retains geometric evaluation");
      Require(result.value != 0 && result.derivative != 0,
              "nonzero near-axis residual and derivative are retained");
      Near(result.value / expected_value, 1,
           "near-axis residual relative accuracy", 1e-13);
      Near(result.derivative / expected_derivative, 1,
           "near-axis derivative relative accuracy", 1e-13);
    }
  }
  auto roundoff = PoseIsoRT::Identity();
  roundoff.linear()(0, 0) += 16 * crx::numerics::kEpsilon;
  const auto circle = Prepare({1, 1, 0, 0.25}, roundoff);
  Require(EvaluateResidualPair(circle, 0.3).used_geometric_fallback,
          "admitted nonunit phase basis keeps geometric residual");
  const auto invalid =
      EvaluateResidualPair(circle, std::numeric_limits<double>::quiet_NaN());
  Require(!std::isfinite(invalid.value) && !std::isfinite(invalid.derivative),
          "nonfinite phase cannot manufacture a root");
}

void TestCompactCramerResiduals() {
  std::size_t ready_count = 0;
  std::size_t rejected_count = 0;
  for (const auto &asset : kCapturedLengths) {
    for (const double scale : {1e-100, 1.0, 1e100}) {
      for (const double sign : {-1.0, 1.0}) {
        const Lengths lengths{asset.a * scale, sign * asset.b * scale,
                              asset.c * scale, sign * asset.r * scale};
        for (int sample = 0; sample < 64; ++sample) {
          const double angle = static_cast<double>(sample) / 11 - 3;
          auto target = Rotation(Vec3::UnitZ(), angle / 3) *
                        Rotation(Vec3::UnitY(), angle / 5) *
                        Rotation(Vec3::UnitX(), angle / 7);
          target.translation() =
              scale * Vec3(300 * std::sin(angle), 400 * std::cos(angle),
                           200 + 100 * angle);
          const auto circle = Prepare(lengths, target);
          const Vec3 u =
              circle.v * std::cos(angle) + circle.w * std::sin(angle);
          const Vec3 x = circle.p + circle.lengths.r * u;
          const double determinant = x.dot(u.cross(Vec3::UnitZ().cross(x)));
          CramerEvaluation actual;
          actual.value = 123;
          const auto status =
              EvaluateCramerElbowResidual(circle, angle, actual);
          if (std::abs(determinant) <
              crx::numerics::kCramerGuard * x.squaredNorm()) {
            Require(status == CramerStatus::GuardRejected &&
                        actual.value == 123,
                    "Cramer guard preserves the original chart and output");
            ++rejected_count;
            continue;
          }
          Require(status == CramerStatus::Ready,
                  "well-conditioned compact Cramer evaluation succeeds");
          ++ready_count;
          const auto oracle = ComplexStepResidual(circle, angle, true);
          // Division amplifies errors by the chart's conditioning ratio.
          // This is an independent numerical check, not a certified bound.
          const double roundoff = 128 * crx::numerics::kEpsilon *
                                  (1 + x.squaredNorm() / std::abs(determinant));
          Require(std::abs(actual.value - oracle.value) <=
                      roundoff * std::max(1.0, std::abs(oracle.value)),
                  "compact Cramer quotient agrees with cross-product oracle");
          Require(std::abs(actual.derivative - oracle.derivative) <=
                      roundoff * std::max(1.0, oracle.derivative_term_scale),
                  "compact Cramer derivative agrees with complex-step oracle");
          Require((actual.elbow - oracle.elbow).cwiseAbs().maxCoeff() <=
                      roundoff *
                          std::max(1.0, oracle.elbow.cwiseAbs().maxCoeff()),
                  "compact Cramer elbow agrees with geometric witness");
        }
      }
    }
  }
  auto target = PoseIsoRT::Identity();
  target.translation() = Vec3(1, 0, 0);
  const auto circle = Prepare({1, 1, 0, 0.25}, target);
  CramerEvaluation untouched;
  untouched.value = 123;
  Require(EvaluateCramerElbowResidual(circle, 0, untouched) ==
                  CramerStatus::GuardRejected &&
              untouched.value == 123,
          "vertical exceptional chart is retained for incidence recovery");
  ++rejected_count;
  Require(ready_count > 0 && rejected_count > 0,
          "Cramer accepted and rejected charts exercised");
  Require(EvaluateCramerElbowResidual(
              circle, std::numeric_limits<double>::quiet_NaN(), untouched) ==
                  CramerStatus::NumericalRangeFailure &&
              untouched.value == 123,
          "nonfinite Cramer phase leaves output unchanged");
  target.translation() = Vec3(-0.25, 0, 0);
  const auto coincident = Prepare({1, 1, 0, 0.25}, target);
  Require(EvaluateCramerElbowResidual(coincident, 0, untouched) ==
                  CramerStatus::NumericalRangeFailure &&
              untouched.value == 123,
          "zero wrist point cannot enter the Cramer chart");
  const double guard = crx::numerics::kCramerGuard;
  const double boundary = 2 * guard / (1 + std::sqrt(1 - 4 * guard * guard));
  for (const double sign : {-1.0, 1.0}) {
    for (const double fraction : {1 - 1e-8, 1 + 1e-8}) {
      target.translation() = Vec3(0.75, 0, sign * fraction * boundary);
      const auto near_guard = Prepare({1, 1, 0, 0.25}, target);
      const auto status = EvaluateCramerElbowResidual(near_guard, 0, untouched);
      Require(status == (fraction < 1 ? CramerStatus::GuardRejected
                                      : CramerStatus::Ready),
              "both signed Cramer conditioning boundaries preserve dispatch");
    }
  }
}

void TestFoldedEndpointCoefficients() {
  // With equal unit links, p=(x,0,0), v=ex and w=ey, the residual at
  // u=+/-ex is |p+r*u|^6/4. This independent scalar identity checks tiny
  // coefficients that an absolute residual tolerance would accept as zero.
  for (const double scale : {1e-120, 1.0, 1e120}) {
    const Lengths lengths{scale, scale, 0, 0.375 * scale};
    for (const double offset : {0x1p-10, 0x1p-20, 0x1p-40}) {
      for (const double side : {-1.0, 1.0}) {
        PoseIsoRT target = PoseIsoRT::Identity();
        target.translation().x() = side * (lengths.r - offset * scale);
        const auto circle = Prepare(lengths, target);
        const auto polynomial = PolynomialFor(circle);
        const double distance = circle.p.x() - side * circle.lengths.r;
        const double squared = distance * distance;
        const double expected = 0.25 * squared * squared * squared;
        const double actual = side < 0 ? polynomial.coefficients.front()
                                       : polynomial.coefficients.back();
        Require(actual > 0, "near-folded endpoint coefficient stays nonzero");
        Near(actual / expected, 1.0,
             "near-folded endpoint retains relative coefficient accuracy",
             64 * crx::numerics::kEpsilon);
      }
    }
  }
}

void TestNormalization() {
  const Lengths lengths{-4, 3, -0.5, 1.25};
  auto target = Rotation(Vec3::UnitX(), 0.3) * Rotation(Vec3::UnitY(), -0.9);
  target.translation() = Vec3(2, -3, 7);
  const auto reference = PolynomialFor(Prepare(lengths, target));
  for (const double factor : {-1e250, -1.0, -1e-250, 1e-250, 1.0, 1e250}) {
    PoseIsoRT scaled = target;
    scaled.translation() *= factor;
    const Lengths scaled_lengths{lengths.a * factor, lengths.b * factor,
                                 lengths.c * factor, lengths.r * factor};
    const auto circle = Prepare(scaled_lengths, scaled);
    Near(circle.length_scale / std::abs(factor), 4,
         "length normalization scale");
    const auto polynomial = PolynomialFor(circle);
    for (std::size_t i = 0; i < kCoefficientCount; ++i) {
      Near(polynomial.coefficients[i], reference.coefficients[i],
           "uniform signed length scaling preserves normalized coefficients");
    }
  }
}

void TestDegeneratePolynomials() {
  const auto coefficients_match = [](const Coefficients &a,
                                     const Coefficients &b) {
    using Vector = Eigen::Matrix<double, kCoefficientCount, 1>;
    return (Eigen::Map<const Vector>(a.data()) -
            Eigen::Map<const Vector>(b.data()))
        .isZero(crx::numerics::kEpsilon);
  };
  PoseIsoRT target = PoseIsoRT::Identity();
  auto circle = Prepare({1, 1, 0, 1}, target);
  const Coefficients constant_residual{0.25, 0, 1, 0, 1.5, 0, 1, 0, 0.25};
  Require(
      coefficients_match(PolynomialFor(circle).coefficients, constant_residual),
      "constant residual gives Q^4/4 in the finite chart");

  target.translation().x() = 1;
  circle = Prepare({1, 1, 0, 1}, target);
  auto polynomial = PolynomialFor(circle);
  Require(coefficients_match(polynomial.coefficients,
                             Coefficients{16, 0, 0, 0, 0, 0, 0, 0, 0}),
          "exact degree-zero polynomial with omitted-point zero");
  Require(RepresentedDegree(polynomial.coefficients) == 0 &&
              crx::numerics::Scalar(polynomial.omitted_point_residual)
                  .isZero(crx::numerics::kEpsilon),
          "constant polynomial still requires pi incidence");
  Require(FindRootCandidates(polynomial.coefficients).status ==
              RootStatus::ConstantPolynomial,
          "no finite candidates does not remove the omitted-point root");
  // a=1/2,b=r=1,p=ex: P=(4+(3/4)Q)^2, an exact degree-four case.
  circle = Prepare({0.5, 1, 0, 1}, target);
  polynomial = PolynomialFor(circle);
  Require(coefficients_match(
              polynomial.coefficients,
              Coefficients{22.5625, 0, 7.125, 0, 0.5625, 0, 0, 0, 0}),
          "exact degree-four coefficient cancellation");
  Require(RepresentedDegree(polynomial.coefficients) == 4,
          "degree four retained");

  target.translation().setZero();
  circle = Prepare({1, -0.5, 0, 0}, target);
  polynomial = PolynomialFor(circle);
  Require(RepresentedDegree(polynomial.coefficients) == -1 &&
              crx::numerics::Scalar(polynomial.omitted_point_residual)
                  .isZero(crx::numerics::kEpsilon),
          "origin zero polynomial is represented, never declared feasible");
  // Vertical O4 is another exceptional incidence domain; scalar zeros alone
  // do not provide a reconstruction or an infeasibility certificate.
  target.translation().z() = 0.5;
  circle = Prepare({1, 1, 0, 0}, target);
  CheckCharts(circle);
  Require(RepresentedDegree(PolynomialFor(circle).coefficients) == -1,
          "vertical zero polynomial");
}

void TestPolynomialUtilities() {
  for (std::size_t degree = 0; degree < kCoefficientCount; ++degree) {
    Coefficients coefficients{};
    coefficients[degree] = 2;
    Require(RepresentedDegree(coefficients) == static_cast<int>(degree),
            "every represented degree from zero through eight");
    for (const double argument : {-2.0, 0.0, 0.5, 3.0}) {
      const auto evaluated = EvaluatePolynomial(coefficients, argument);
      Near(evaluated.value, 2 * std::pow(argument, static_cast<int>(degree)),
           "Horner value");
      const double derivative =
          degree == 0 ? 0
                      : 2 * static_cast<double>(degree) *
                            std::pow(argument, static_cast<int>(degree - 1));
      Near(evaluated.derivative, derivative, "Horner derivative");
    }
  }
  Coefficients tiny{1};
  tiny.back() = std::numeric_limits<double>::denorm_min();
  Require(RepresentedDegree(tiny) == 8,
          "never trim a tiny nonzero coefficient");
}

void TestInvalidAndRangeFailures() {
  const double infinity = std::numeric_limits<double>::infinity();
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const PoseIsoRT identity = PoseIsoRT::Identity();
  WristCircle output;
  output.length_scale = 123;
  for (const Lengths lengths :
       {Lengths{0, 1, 0, 0}, Lengths{1, 0, 0, 0}, Lengths{infinity, 1, 0, 0},
        Lengths{1, 1, nan, 0}}) {
    Require(PrepareWristCircle(lengths, identity, output) ==
                PreparationStatus::InvalidInput,
            "invalid geometry rejected");
    Near(output.length_scale, 123, "prepare failure leaves output intact",
         crx::numerics::kEpsilon);
  }
  for (int kind = 0; kind < 5; ++kind) {
    PoseIsoRT invalid = identity;
    if (kind == 0)
      invalid.linear()(0, 0) = -1; // Reflection.
    if (kind == 1)
      invalid.linear()(0, 0) = 1.00000001;
    if (kind == 2)
      invalid.translation().x() = infinity;
    if (kind == 3)
      invalid.matrix()(3, 0) = 256 * crx::numerics::kEpsilon;
    if (kind == 4)
      invalid.linear()(0, 1) = nan;
    Require(PrepareWristCircle({1, 1, 0, 0}, invalid, output) ==
                PreparationStatus::InvalidInput,
            "invalid canonical target rejected without projection");
  }
  auto roundoff = identity;
  roundoff.linear()(0, 0) += std::numeric_limits<double>::epsilon();
  const auto preserved = Prepare({1, 1, 0, 0}, roundoff);
  Require(preserved.v.x() > 1.0, "roundoff is not projected to the identity");
  Near(preserved.v.x(), roundoff.linear()(0, 0), "roundoff preserved",
       crx::numerics::kEpsilon);
  // Unit-scale row: accept ordinary rounding, reject values outside the
  // existing 128-epsilon rigid-transform budget on either side.
  for (int column = 0; column < 4; ++column) {
    for (double sign : {-1.0, 1.0}) {
      for (double ulps : {1.0, 64.0, 256.0}) {
        auto perturbed = identity;
        perturbed.matrix()(3, column) += sign * ulps * crx::numerics::kEpsilon;
        const auto expected = ulps < 128 ? PreparationStatus::Ready
                                         : PreparationStatus::InvalidInput;
        Require(PrepareWristCircle({1, 1, 0, 0}, perturbed, output) == expected,
                "homogeneous row uses the unit roundoff budget");
      }
    }
  }

  Require(PrepareWristCircle({1e-300, 1e300, 0, 0}, identity, output) ==
              PreparationStatus::NumericalRangeFailure,
          "normalization underflow is unresolved");
  auto far = identity;
  far.translation().x() = 1e300;
  Require(PrepareWristCircle({1e-100, 1e-100, 0, 0}, far, output) ==
              PreparationStatus::NumericalRangeFailure,
          "normalization overflow is unresolved");
  far.translation().x() = 1e60;
  const auto huge_circle = Prepare({1, 1, 0, 0}, far);
  HalfAnglePolynomial untouched;
  untouched.chart_angle = 123;
  Require(BuildHalfAnglePolynomial(huge_circle, 0, untouched) ==
              PreparationStatus::NumericalRangeFailure,
          "coefficient overflow is unresolved");
  Near(untouched.chart_angle, 123, "coefficient failure leaves output intact",
       crx::numerics::kEpsilon);
  Require(BuildHalfAnglePolynomial(Prepare({1, 1, 0, 0}, identity), infinity,
                                   untouched) ==
              PreparationStatus::InvalidInput,
          "nonfinite chart rejected");
  Require(!std::isfinite(EvaluateResidual(huge_circle, nan)),
          "nonfinite angle cannot report a root");
}

class EigenAllocationGuard {
public:
  EigenAllocationGuard() : previous_(Eigen::internal::is_malloc_allowed()) {
    Eigen::internal::set_is_malloc_allowed(false);
  }

  ~EigenAllocationGuard() { Eigen::internal::set_is_malloc_allowed(previous_); }

  EigenAllocationGuard(const EigenAllocationGuard &) = delete;
  auto operator=(const EigenAllocationGuard &)
      -> EigenAllocationGuard & = delete;
  EigenAllocationGuard(EigenAllocationGuard &&) = delete;
  auto operator=(EigenAllocationGuard &&) -> EigenAllocationGuard & = delete;

private:
  bool previous_;
};
} // namespace

auto main(int argc, char **argv) -> int {
  if (argc > 1) {
    if (std::strcmp(argv[1], "--discover") == 0 ||
        std::strcmp(argv[1], "--discover-vertical") == 0 ||
        std::strcmp(argv[1], "--discover-general") == 0 ||
        std::strcmp(argv[1], "--diagnose-discovery") == 0) {
      return RunDiscoveryProbe(argc, argv);
    }
    return RunRootProbe(argc, argv);
  }
  // Positive controls: prove both allocation replacements are actually
  // active.
  const auto before_probe = crx::test::AllocationCount();
  void *plain = ::operator new(16);
  void *aligned = ::operator new[](64, std::align_val_t{64});
  ::operator delete(plain);
  ::operator delete[](aligned, std::align_val_t{64});
  Require(crx::test::AllocationCount() == before_probe + 2,
          "allocation probe active");
  const auto before_kernel = crx::test::AllocationCount();
  {
    const EigenAllocationGuard guard;
    // Include first construction, changing geometry/charts, and error paths.
    TestResidualsAndCanonicalWitnesses();
    TestBivariateResiduals();
    TestCompactCramerResiduals();
    TestFoldedEndpointCoefficients();
    TestNormalization();
    TestDegeneratePolynomials();
    TestPolynomialUtilities();
    TestInvalidAndRangeFailures();
    RunPolynomialRootTests();
    RunIncidenceTests();
    RunIncidenceReferenceTests();
    RunJointRecoveryTests();
  }
  Require(crx::test::AllocationCount() == before_kernel,
          "canonical component made a C++ heap allocation");
  std::puts(
      "Canonical residual/coefficient/root/incidence checks passed; zero C++ "
      "allocations; Eigen guard active.");
  return 0;
}
