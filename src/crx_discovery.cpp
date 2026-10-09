#include "crx_discovery.h"

#include "crx_canonical.h"
#include "crx_joint_recovery.h"
#include "crx_polynomial_roots.h"
#include "crx_types.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>

namespace crx::canonical {
namespace {
constexpr double kPi = 3.14159265358979323846;
constexpr double kEpsilon = std::numeric_limits<double>::epsilon();
constexpr double kDuplicateTolerance = 1e-10;
// Numerical handoff thresholds, not coefficient-error enclosures or root
// certificates. FK acceptance uses the caller's physical pose tolerances.
constexpr double kRootResidualTolerance = 1e-10;
constexpr double kRootSlopeTolerance = 1e-8;
constexpr double kNearRealTolerance = 1e-6;
constexpr double kPolishRadius = 0.05;

struct ElbowResidual {
  double value = 0;
  double derivative = 0;
};

// Differentiate the unsquared wrist constraint on the two sphere/base-plane
// intersections. Using f/f' on the squared elimination polynomial loses useful
// digits near a double root. This calculation is only for polishing; acceptance
// remains in RecoverJointCandidates, including its independent full-pose FK.
auto EvaluateElbowResidual(const WristCircle &circle, double angle,
                           ElbowResidual &output) -> bool {
  const Vec3 u = circle.v * std::cos(angle) + circle.w * std::sin(angle);
  const Vec3 du = -circle.v * std::sin(angle) + circle.w * std::cos(angle);
  const Vec3 x = circle.p + circle.lengths.r * u;
  const Vec3 dx = circle.lengths.r * du;
  const double distance = x.norm();
  if (!std::isfinite(distance) || distance <= 256 * kEpsilon) {
    return false;
  }
  const Vec3 radial = x / distance;
  const double dd = radial.dot(dx);
  const Vec3 dradial = (dx - radial * dd) / distance;
  const Vec3 vertical = Vec3::UnitZ() - radial.z() * radial;
  const double vertical_norm = vertical.norm();
  if (vertical_norm <= 256 * kEpsilon) {
    return false;
  }
  const Vec3 perpendicular = vertical / vertical_norm;
  const Vec3 dvertical = -dradial.z() * radial - radial.z() * dradial;
  const Vec3 dperpendicular =
      (dvertical - perpendicular * perpendicular.dot(dvertical)) /
      vertical_norm;
  const double a2 = circle.lengths.a * circle.lengths.a;
  const double arm_difference = (circle.lengths.a - circle.lengths.b) *
                                (circle.lengths.a + circle.lengths.b);
  const double along = (arm_difference + distance * distance) / (2 * distance);
  const double dalong = (x.dot(dx) - along * dd) / distance;
  const double height2 = a2 - along * along;
  if (height2 <= std::sqrt(kEpsilon) * std::max(a2, along * along)) {
    // Near tangency differentiate the sphere residual after solving the
    // unsquared wrist plane for the signed elbow height. sqrt(height²) and
    // its derivative are ill-conditioned here, even for a simple root.
    const double slope = perpendicular.dot(u);
    if (std::abs(slope) <= 256 * kEpsilon) {
      return false;
    }
    const Vec3 center = (along - distance) * radial;
    const Vec3 dcenter = (dalong - dd) * radial + (along - distance) * dradial;
    const double value = center.dot(u);
    const double derivative = dcenter.dot(u) + center.dot(du);
    const double dslope = dperpendicular.dot(u) + perpendicular.dot(du);
    const double height = -value / slope;
    const double dheight = -(derivative + height * dslope) / slope;
    output = {height * height - height2,
              2 * (height * dheight + along * dalong)};
    return std::isfinite(output.value) && std::isfinite(output.derivative);
  }
  const double height = std::sqrt(height2);
  const double dheight = -along * dalong / height;
  ElbowResidual best{std::numeric_limits<double>::infinity(), 0};
  double best_step = std::numeric_limits<double>::infinity();
  for (const double sign : {-1.0, 1.0}) {
    const Vec3 forearm = along * radial + sign * height * perpendicular - x;
    const Vec3 dforearm =
        dalong * radial + along * dradial +
        sign * (dheight * perpendicular + height * dperpendicular) - dx;
    const double value = forearm.dot(u);
    const double derivative = dforearm.dot(u) + forearm.dot(du);
    if (!std::isfinite(value) || !std::isfinite(derivative)) {
      return false;
    }
    // Near a multiple root, the other elbow can have a smaller residual but
    // an almost flat, nonzero constraint. Prefer the nearest Newton correction
    // instead of getting trapped polishing that incompatible elbow branch.
    const double step = derivative == 0
                            ? std::numeric_limits<double>::infinity()
                            : std::abs(value / derivative);
    if (step < best_step) {
      best = {value, derivative};
      best_step = step;
    }
  }
  output = best;
  return std::isfinite(best_step);
}

auto OutsideArmShell(const WristCircle &circle, double angle, double radius)
    -> bool {
  const Vec3 u = circle.v * std::cos(angle) + circle.w * std::sin(angle);
  const double distance = (circle.p + circle.lengths.r * u).norm();
  const double a = std::abs(circle.lengths.a);
  const double b = std::abs(circle.lengths.b);
  const double margin = 8192 * kEpsilon * std::max({a, b, distance}) +
                        std::abs(circle.lengths.r) * radius;
  return std::isfinite(distance) &&
         (distance > a + b + margin || distance < std::abs(a - b) - margin);
}

auto PolishAndRecover(const Lengths &lengths, const PoseIsoRT &target,
                      const WristCircle &circle, double angle,
                      const PoseTolerance &tolerance, int budget, double radius)
    -> JointRecovery {
  const double initial_angle = angle;
  for (int iteration = 0; iteration <= budget; ++iteration) {
    const auto recovered =
        RecoverJointCandidates(lengths, target, angle, tolerance);
    if (recovered.status == RecoveryStatus::Candidates ||
        recovered.status == RecoveryStatus::NumericalRangeFailure) {
      return recovered;
    }
    if (OutsideArmShell(circle, initial_angle, radius)) {
      JointRecovery empty;
      empty.status = RecoveryStatus::NoCandidate;
      return empty;
    }
    ElbowResidual residual;
    if (iteration == budget ||
        !EvaluateElbowResidual(circle, angle, residual) ||
        std::abs(residual.derivative) <= 256 * kEpsilon) {
      break;
    }
    double step = std::clamp(residual.value / residual.derivative, -0.25, 0.25);
    bool improved = false;
    for (int backtrack = 0; backtrack < 8; ++backtrack) {
      const double trial = std::remainder(angle - step, 2 * kPi);
      ElbowResidual next;
      if (std::abs(std::remainder(trial - initial_angle, 2 * kPi)) <= radius &&
          EvaluateElbowResidual(circle, trial, next) &&
          std::abs(next.value) < std::abs(residual.value)) {
        angle = trial;
        improved = true;
        break;
      }
      step *= 0.5;
    }
    if (!improved) {
      break;
    }
  }
  JointRecovery unresolved;
  unresolved.status = RecoveryStatus::NeedsRefinement;
  return unresolved;
}

// Bounded Horner evaluation at the real projection. Reverse the polynomial
// for |t|>1, so powers cannot overflow. The angular slope flags ill-conditioned
// real roots; a small projected residual also catches multiple real roots that
// the eigensolver split into complex pairs (not just tiny imaginary parts).
auto RealProjection(const Coefficients &coefficients, double t)
    -> PolynomialValue {
  const Eigen::Map<const Eigen::Matrix<double, kCoefficientCount, 1>> mapped(
      coefficients.data());
  const double scale = mapped.cwiseAbs().maxCoeff();
  const bool reverse = std::abs(t) > 1;
  const double argument = reverse ? 1 / t : t;
  PolynomialValue evaluation;
  double bound = 0;
  for (std::size_t i = 0; i < coefficients.size(); ++i) {
    const double coefficient =
        coefficients[reverse ? i : coefficients.size() - 1 - i] / scale;
    evaluation.derivative = evaluation.derivative * argument + evaluation.value;
    evaluation.value = evaluation.value * argument + coefficient;
    bound = bound * std::abs(argument) + std::abs(coefficient);
  }
  // Include coefficient-scale uncertainty even at t=0 where the pointwise
  // bound could vanish. A simple exactly-zero root still has a usable slope.
  bound = std::max(1.0, bound);
  return {evaluation.value / bound,
          evaluation.derivative * (1 + argument * argument) / (2 * bound)};
}

auto AppendUnique(const JointRecovery &recovered, JointDiscovery &output)
    -> bool {
  for (std::size_t i = 0; i < recovered.count; ++i) {
    const auto &candidate = recovered.candidates[i];
    bool duplicate = false;
    for (std::size_t j = 0; j < output.count; ++j) {
      const double difference = (candidate.joints - output.candidates[j].joints)
                                    .unaryExpr([](double angle) {
                                      return std::remainder(angle, 2 * kPi);
                                    })
                                    .cwiseAbs()
                                    .maxCoeff();
      duplicate = duplicate || difference <= kDuplicateTolerance;
    }
    if (!duplicate) {
      if (output.count == output.candidates.size()) {
        return false;
      }
      output.candidates[output.count++] = candidate;
    }
  }
  return true;
}

struct GeometricPhases {
  std::array<WristPhase, 8> values{};
  std::size_t count = 0;
};

// These are analytic crossings, not an angular scan. They restore exact
// geometric boundary/multiple roots that are poorly conditioned after the
// degree-eight elimination: u.z=0, x.u=0, and the two arm-shell boundaries.
auto BoundaryPhases(const WristCircle &circle) -> GeometricPhases {
  GeometricPhases result;
  const auto append = [&](double cosine, double sine, double rhs) {
    // Solve A*cos(theta)+B*sin(theta)=C directly on the unit circle.
    // Scale before forming the discriminant, so small coefficients do not
    // underflow when squared. A=B=0 has no isolated crossing to nominate.
    const double scale = std::max(std::abs(cosine), std::abs(sine));
    if (scale == 0) {
      return;
    }
    const double a = cosine / scale, b = sine / scale;
    const double norm = std::hypot(a, b);
    if (std::abs(rhs) > scale * norm + 64 * kEpsilon) {
      return;
    }
    const double along = std::clamp((rhs / scale) / norm, -1.0, 1.0);
    const double across =
        std::sqrt((1.0 - std::abs(along)) * (1.0 + std::abs(along)));
    const double ax = a / norm, ay = b / norm;
    result.values[result.count++] = {along * ax - across * ay,
                                     along * ay + across * ax};
    if (across > 0) {
      result.values[result.count++] = {along * ax + across * ay,
                                       along * ay - across * ax};
    }
  };
  append(circle.v.z(), circle.w.z(), 0);
  const double pv = circle.p.dot(circle.v), pw = circle.p.dot(circle.w);
  append(pv, pw, -circle.lengths.r);
  for (const double reach :
       {std::abs(circle.lengths.a) + std::abs(circle.lengths.b),
        std::abs(std::abs(circle.lengths.a) - std::abs(circle.lengths.b))}) {
    append(2 * circle.lengths.r * pv, 2 * circle.lengths.r * pw,
           reach * reach - circle.p.squaredNorm() -
               circle.lengths.r * circle.lengths.r);
  }
  return result;
}

auto DiscoverInChart(const Lengths &lengths, const PoseIsoRT &target,
                     const WristCircle &circle, const PoseTolerance &tolerance,
                     const DiscoveryOptions &options, double chart)
    -> JointDiscovery {
  JointDiscovery result;
  HalfAnglePolynomial polynomial;
  if (BuildHalfAnglePolynomial(circle, chart, polynomial) !=
      PreparationStatus::Ready) {
    result.status = RecoveryStatus::NumericalRangeFailure;
    return result;
  }
  const auto roots = FindRootCandidates(polynomial.coefficients, options.roots);
  if (roots.status != RootStatus::Candidates &&
      roots.status != RootStatus::ConstantPolynomial) {
    result.status = roots.status == RootStatus::InvalidInput
                        ? RecoveryStatus::InvalidInput
                    : roots.status == RootStatus::NumericalRangeFailure
                        ? RecoveryStatus::NumericalRangeFailure
                        : RecoveryStatus::NeedsRefinement;
    return result;
  }
  JointDiscovery accepted;
  const auto boundary_phases = BoundaryPhases(circle);
  for (std::size_t i = 0; i < boundary_phases.count; ++i) {
    const auto boundary = RecoverJointCandidates(
        lengths, target, boundary_phases.values[i], tolerance);
    if (boundary.status == RecoveryStatus::Candidates &&
        !AppendUnique(boundary, accepted)) {
      result.status = RecoveryStatus::NeedsRefinement;
      return result;
    }
  }
  for (std::size_t i = 0; i <= roots.count; ++i) {
    double angle = polynomial.chart_angle + kPi;
    double radius = kPolishRadius;
    if (i == roots.count) {
      const Eigen::Map<const Eigen::Matrix<double, kCoefficientCount, 1>>
          mapped(polynomial.coefficients.data());
      const double scale = mapped.cwiseAbs().maxCoeff();
      if (std::abs(polynomial.omitted_point_residual) >
          128 * kEpsilon * scale) {
        continue;
      }
    } else {
      const auto &root = roots.candidates[i];
      const double relative_imaginary =
          std::abs(root.value.imag()) / (1 + std::abs(root.value.real()));
      const auto projected =
          RealProjection(polynomial.coefficients, root.value.real());
      // Multiple real roots can split into conjugate eigenvalues. Their real
      // projections are starting guesses only: unsquared geometry and full FK
      // below, not the imaginary part or polynomial slope, decide acceptance.
      if (relative_imaginary > 64 * kEpsilon) {
        if (relative_imaginary > kNearRealTolerance &&
            std::abs(projected.value) > kRootResidualTolerance) {
          continue;
        }
      }
      if (root.relative_residual > kRootResidualTolerance) {
        result.status = RecoveryStatus::NeedsRefinement;
        return result;
      }
      angle = polynomial.chart_angle + 2 * std::atan(root.value.real());
      const bool ill_conditioned =
          relative_imaginary > 64 * kEpsilon ||
          std::abs(projected.derivative) <= kRootSlopeTolerance;
      for (std::size_t j = 0; j < roots.count; ++j) {
        if (i == j || ill_conditioned) {
          continue;
        }
        const double other = polynomial.chart_angle +
                             2 * std::atan(roots.candidates[j].value.real());
        radius = std::min(
            radius, 0.25 * std::abs(std::remainder(angle - other, 2 * kPi)));
      }
    }
    auto recovered = PolishAndRecover(lengths, target, circle,
                                      std::remainder(angle, 2 * kPi), tolerance,
                                      options.max_polish_iterations, radius);
    if (recovered.status == RecoveryStatus::NeedsRefinement) {
      for (std::size_t j = 0; j < boundary_phases.count; ++j) {
        const auto &boundary = boundary_phases.values[j];
        // An angle is needed only for a failed root's proximity check; the
        // recovery itself continues to use the original circle coordinates.
        const double boundary_angle =
            std::atan2(boundary.sine, boundary.cosine);
        if (std::abs(std::remainder(boundary_angle - angle, 2 * kPi)) <=
            radius) {
          const auto trial =
              RecoverJointCandidates(lengths, target, boundary, tolerance);
          if (trial.status == RecoveryStatus::Candidates) {
            recovered = trial;
            break;
          }
        }
      }
    }
    if (recovered.status != RecoveryStatus::Candidates &&
        recovered.status != RecoveryStatus::NoCandidate) {
      result.status = recovered.status;
      return result;
    }
    if (!AppendUnique(recovered, accepted)) {
      result.status = RecoveryStatus::NeedsRefinement;
      return result;
    }
  }
  accepted.status = accepted.count == 0 ? RecoveryStatus::NoCandidate
                                        : RecoveryStatus::Candidates;
  return accepted;
}
} // namespace

auto DiscoverJointCandidates(const Lengths &lengths, const PoseIsoRT &target,
                             const PoseTolerance &tolerance,
                             const DiscoveryOptions &options)
    -> JointDiscovery {
  JointDiscovery result;
  if (!std::isfinite(tolerance.position) || tolerance.position <= 0 ||
      !std::isfinite(tolerance.orientation) || tolerance.orientation <= 0 ||
      tolerance.orientation >= kPi || options.max_polish_iterations < 0 ||
      options.max_polish_iterations > 64) {
    return result;
  }
  WristCircle circle;
  const auto preparation = PrepareWristCircle(lengths, target, circle);
  if (preparation != PreparationStatus::Ready) {
    result.status = preparation == PreparationStatus::InvalidInput
                        ? RecoveryStatus::InvalidInput
                        : RecoveryStatus::NumericalRangeFailure;
    return result;
  }
  // Re-express the same polynomial in another half-angle chart when roots
  // near the omitted point make geometric polishing ill-conditioned.
  const double closest_wrist =
      std::atan2(circle.p.dot(circle.w), circle.p.dot(circle.v)) +
      (circle.lengths.r >= 0 ? kPi : 0.0);
  const Vec3 closest =
      circle.p + circle.lengths.r * (circle.v * std::cos(closest_wrist) +
                                     circle.w * std::sin(closest_wrist));
  const double first_chart =
      closest.norm() < 0.01 * std::min(std::abs(circle.lengths.a),
                                       std::abs(circle.lengths.b))
          ? closest_wrist
          : 0.0;
  for (const double chart :
       {first_chart, kPi / 2, kPi / 4, -kPi / 4, closest_wrist}) {
    result =
        DiscoverInChart(lengths, target, circle, tolerance, options, chart);
    if (result.status != RecoveryStatus::NeedsRefinement) {
      return result;
    }
  }
  return result;
}
} // namespace crx::canonical
