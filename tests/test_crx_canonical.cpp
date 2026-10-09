#include "crx_allocation_probe.h"
#include "crx_canonical.h"
#include "crx_incidence_tests.h"
#include "crx_joint_recovery_tests.h"
#include "crx_polynomial_roots.h"
#include "crx_root_tests.h"

#include <algorithm>
#include <cmath>
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
  for (const double chart : {0.0, 0.7, -1.3, kPi}) {
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
      if (tau != 0.0) {
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
    // incidence witness. No scanner or native production FK supplies the pose.
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
  PoseIsoRT target = PoseIsoRT::Identity();
  auto circle = Prepare({1, 1, 0, 1}, target);
  const Coefficients constant_residual{0.25, 0, 1, 0, 1.5, 0, 1, 0, 0.25};
  Require(PolynomialFor(circle).coefficients == constant_residual,
          "constant residual gives Q^4/4 in the finite chart");

  target.translation().x() = 1;
  circle = Prepare({1, 1, 0, 1}, target);
  auto polynomial = PolynomialFor(circle);
  Require(polynomial.coefficients == Coefficients{16, 0, 0, 0, 0, 0, 0, 0, 0},
          "exact degree-zero polynomial with omitted-point zero");
  Require(RepresentedDegree(polynomial.coefficients) == 0 &&
              polynomial.omitted_point_residual == 0,
          "constant polynomial still requires pi incidence");
  Require(FindRootCandidates(polynomial.coefficients).status ==
              RootStatus::ConstantPolynomial,
          "no finite candidates does not remove the omitted-point root");
  // a=1/2,b=r=1,p=ex: P=(4+(3/4)Q)^2, an exact degree-four case.
  circle = Prepare({0.5, 1, 0, 1}, target);
  polynomial = PolynomialFor(circle);
  Require(polynomial.coefficients ==
              Coefficients{22.5625, 0, 7.125, 0, 0.5625, 0, 0, 0, 0},
          "exact degree-four coefficient cancellation");
  Require(RepresentedDegree(polynomial.coefficients) == 4,
          "degree four retained");

  target.translation().setZero();
  circle = Prepare({1, -0.5, 0, 0}, target);
  polynomial = PolynomialFor(circle);
  Require(RepresentedDegree(polynomial.coefficients) == -1 &&
              polynomial.omitted_point_residual == 0,
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
    Require(output.length_scale == 123, "prepare failure leaves output intact");
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
      invalid.matrix()(3, 0) = 1e-16;
    if (kind == 4)
      invalid.linear()(0, 1) = nan;
    Require(PrepareWristCircle({1, 1, 0, 0}, invalid, output) ==
                PreparationStatus::InvalidInput,
            "invalid canonical target rejected without projection");
  }
  auto roundoff = identity;
  roundoff.linear()(0, 0) += std::numeric_limits<double>::epsilon();
  const auto preserved = Prepare({1, 1, 0, 0}, roundoff);
  Require(preserved.v.x() == roundoff.linear()(0, 0),
          "roundoff is not projected");

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
  Require(untouched.chart_angle == 123,
          "coefficient failure leaves output intact");
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
