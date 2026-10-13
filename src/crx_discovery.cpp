#include "crx_discovery.h"
#include "crx_numerics.h"

#include "crx_canonical.h"
#include "crx_joint_recovery.h"
#include "crx_polynomial_roots.h"
#include "crx_types.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>

namespace crx::canonical {
namespace {
struct ElbowResidual {
  double value = 0;
  double derivative = 0;
  Vec3 center = Vec3::Zero();
  Vec3 side = Vec3::Zero();
};

// Alternate elimination order: intersect the two arm spheres with the wrist
// plane and polish the remaining base-plane constraint. Unlike azimuth from
// x.xy(), the unsquared cross-product residual is smooth at the base axis.
auto EvaluateWristPlaneResidual(const WristCircle &circle, double angle,
                                double sign, ElbowResidual &output) -> bool {
  const Vec3 u = circle.v * std::cos(angle) + circle.w * std::sin(angle);
  const Vec3 du = -circle.v * std::sin(angle) + circle.w * std::cos(angle);
  const Vec3 x = circle.p + circle.lengths.r * u;
  const Vec3 dx = circle.lengths.r * du;
  const double distance = x.norm();
  if (distance <= numerics::kDirectionTolerance) {
    return false;
  }
  const Vec3 radial = x / distance;
  const double dd = radial.dot(dx);
  const Vec3 dradial = (dx - radial * dd) / distance;
  const double dot = radial.dot(u);
  const double ddot = dradial.dot(u) + radial.dot(du);
  const Vec3 transverse = u - dot * radial;
  const double sine = transverse.norm();
  if (sine <= numerics::kDirectionTolerance) {
    return false;
  }
  const Vec3 direction = transverse / sine;
  const Vec3 dtransverse = du - ddot * radial - dot * dradial;
  const double dsine = direction.dot(dtransverse);
  const Vec3 ddirection = (dtransverse - direction * dsine) / sine;
  const double a = circle.lengths.a;
  const double b = circle.lengths.b;
  const double along =
      ((a - b) * (a + b) + distance * distance) / (2 * distance);
  const double dalong = (x.dot(dx) - along * dd) / distance;
  const double across = (x.dot(u) - along * dot) / sine;
  const double dacross =
      (dx.dot(u) + x.dot(du) - dalong * dot - along * ddot - across * dsine) /
      sine;
  const Vec3 center = along * radial + across * direction;
  const Vec3 dcenter = dalong * radial + along * dradial + dacross * direction +
                       across * ddirection;
  const double height2 = a * a - center.squaredNorm();
  if (height2 <= numerics::kFourthRootEpsilon * a * a) {
    return false;
  }
  const double height = std::sqrt(height2);
  const double dheight = -center.dot(dcenter) / height;
  const Vec3 normal = radial.cross(direction);
  const Vec3 dnormal = dradial.cross(direction) + radial.cross(ddirection);
  const Vec3 elbow = center + sign * height * normal;
  const Vec3 delbow = dcenter + sign * (dheight * normal + height * dnormal);
  output = {x.cross(elbow).z(), (dx.cross(elbow) + x.cross(delbow)).z(), center,
            sign * normal};
  return std::isfinite(output.value) && std::isfinite(output.derivative);
}

// Differentiate the unsquared wrist constraint on the two sphere/base-plane
// intersections. Using f/f' on the squared elimination polynomial loses useful
// digits near a double root. This calculation is only for polishing; acceptance
// remains in RecoverJointCandidates, including its independent full-pose FK.
auto EvaluateElbowResidual(const WristCircle &circle, double angle, double sign,
                           ElbowResidual &output) -> bool {
  const Vec3 u = circle.v * std::cos(angle) + circle.w * std::sin(angle);
  const Vec3 du = -circle.v * std::sin(angle) + circle.w * std::cos(angle);
  const Vec3 x = circle.p + circle.lengths.r * u;
  const Vec3 dx = circle.lengths.r * du;
  const double distance = x.norm();
  if (!std::isfinite(distance) || distance <= numerics::kDirectionTolerance) {
    return false;
  }
  const Vec3 radial = x / distance;
  const double dd = radial.dot(dx);
  const Vec3 dradial = (dx - radial * dd) / distance;
  const Vec3 vertical = Vec3::UnitZ() - radial.z() * radial;
  const double vertical_norm = vertical.norm();
  if (vertical_norm <= numerics::kDirectionTolerance) {
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
  if (height2 <= numerics::kSqrtEpsilon * std::max(a2, along * along)) {
    // Near tangency differentiate the sphere residual after solving the
    // unsquared wrist plane for the signed elbow height. sqrt(heightÂ²) and
    // its derivative are ill-conditioned here, even for a simple root.
    const double slope = perpendicular.dot(u);
    if (std::abs(slope) <= numerics::kDirectionTolerance) {
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
  const Vec3 center = along * radial;
  const Vec3 forearm = center + sign * height * perpendicular - x;
  const Vec3 dforearm =
      dalong * radial + along * dradial +
      sign * (dheight * perpendicular + height * dperpendicular) - dx;
  output = {forearm.dot(u), dforearm.dot(u) + forearm.dot(du), center,
            sign * perpendicular};
  return std::isfinite(output.value) && std::isfinite(output.derivative);
}

auto OutsideArmShell(const WristCircle &circle, double angle, double radius)
    -> bool {
  const Vec3 u = circle.v * std::cos(angle) + circle.w * std::sin(angle);
  const double distance = (circle.p + circle.lengths.r * u).norm();
  const double a = std::abs(circle.lengths.a);
  const double b = std::abs(circle.lengths.b);
  const double margin = numerics::kShellRoundoff * std::max({a, b, distance}) +
                        std::abs(circle.lengths.r) * radius;
  return std::isfinite(distance) &&
         (distance > a + b + margin || distance < std::abs(a - b) - margin);
}

// A successful reconstruction of the other elbow must not stop this branch's
// Newton solve. The separating plane is meaningful only for distinct elbows;
// the tangency chart leaves side zero and keeps its existing joint recovery.
void KeepElbowBranch(const WristCircle &circle, const ElbowResidual &branch,
                     JointRecovery &recovered) {
  std::size_t count = 0;
  for (std::size_t i = 0; i < recovered.count; ++i) {
    const auto &candidate = recovered.candidates[i];
    const auto &q = candidate.joints;
    const double shoulder = std::sin(q[1]);
    const Vec3 elbow =
        circle.lengths.a * Vec3(std::cos(q[0]) * shoulder,
                                std::sin(q[0]) * shoulder, std::cos(q[1]));
    if ((elbow - branch.center).dot(branch.side) >= 0) {
      recovered.candidates[count++] = candidate;
    }
  }
  recovered.count = count;
  if (count == 0 && recovered.status == RecoveryStatus::Candidates) {
    recovered.status = RecoveryStatus::NeedsRefinement;
  }
}

struct PolishedRecovery {
  RecoveryStatus status = RecoveryStatus::NeedsRefinement;
  std::size_t count = 0;
  bool used_singular_representatives = false;
  std::array<JointCandidate, 2 * JointRecovery::kRefinementCapacity>
      candidates{};
};

template <typename Recovery, typename Output>
auto AppendUnique(const Recovery &recovered, Output &output) -> bool {
  output.used_singular_representatives =
      output.used_singular_representatives ||
      recovered.used_singular_representatives;
  for (std::size_t i = 0; i < recovered.count; ++i) {
    const auto &candidate = recovered.candidates[i];
    bool duplicate = false;
    for (std::size_t j = 0; j < output.count; ++j) {
      const double difference =
          (candidate.joints - output.candidates[j].joints)
              .unaryExpr([](double angle) {
                return std::remainder(angle, 2 * angle_conv::kPi);
              })
              .cwiseAbs()
              .maxCoeff();
      duplicate = duplicate || difference <= numerics::kJointPhaseToleranceRad;
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

auto PolishAndRecover(const Lengths &lengths, const PoseIsoRT &target,
                      const WristCircle &circle, double initial_angle,
                      const PoseTolerance &tolerance, int budget, double radius,
                      bool ill_conditioned) -> PolishedRecovery {
  PolishedRecovery accepted;
  const auto direct =
      RecoverJointCandidates(lengths, target, initial_angle, tolerance);
  if (direct.status == RecoveryStatus::NumericalRangeFailure) {
    return {RecoveryStatus::NumericalRangeFailure};
  }
  // A well-conditioned root already has its own disjoint polishing window.
  // Preserve direct recovery there. For a multiple-root nomination, a single
  // recovered elbow is insufficient: follow both branches unless direct
  // incidence already recovered both elbows (and their two base postures).
  if (direct.status == RecoveryStatus::Candidates &&
      (!ill_conditioned || direct.count == JointRecovery::kDirectCapacity)) {
    accepted.status = RecoveryStatus::Candidates;
    if (!AppendUnique(direct, accepted)) {
      return {};
    }
    return accepted;
  }
  if (direct.status != RecoveryStatus::Candidates &&
      OutsideArmShell(circle, initial_angle, radius)) {
    accepted.status = RecoveryStatus::NoCandidate;
    return accepted;
  }
  // Keep both the geometric chart and the elbow sign fixed throughout every
  // Newton solve and line search. A split double root can nominate the same
  // phase twice; choosing the nearest correction each time loses one elbow.
  for (const auto evaluate :
       {EvaluateWristPlaneResidual, EvaluateElbowResidual}) {
    JointRecovery cached_correction;
    double corrected_angle = 0;
    bool have_correction = false;
    for (const double sign : {-1.0, 1.0}) {
      double angle = initial_angle;
      bool found = false;
      for (int iteration = 0; iteration <= budget; ++iteration) {
        ElbowResidual residual;
        const bool regular = evaluate(circle, angle, sign, residual);
        auto recovered =
            iteration == 0
                ? direct
                : RecoverJointCandidates(lengths, target, angle, tolerance);
        if (regular) {
          KeepElbowBranch(circle, residual, recovered);
        }
        if (recovered.status == RecoveryStatus::NumericalRangeFailure) {
          return {RecoveryStatus::NumericalRangeFailure};
        }
        if (recovered.status == RecoveryStatus::Candidates) {
          if (!AppendUnique(recovered, accepted)) {
            return {};
          }
          found = true;
          break;
        }
        if (iteration == budget || !regular ||
            std::abs(residual.derivative) <= numerics::kDirectionTolerance) {
          break;
        }
        double step = std::clamp(residual.value / residual.derivative,
                                 -numerics::kPolishMaxStepRad,
                                 numerics::kPolishMaxStepRad);
        bool improved = false;
        for (int backtrack = 0; backtrack < numerics::kPolishBacktracks;
             ++backtrack) {
          const double trial = std::remainder(angle - step, kTwoPi);
          ElbowResidual next;
          if (std::abs(std::remainder(trial - initial_angle, kTwoPi)) <=
                  radius &&
              evaluate(circle, trial, sign, next) &&
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
      if (found) {
        continue;
      }
      // Preserve each branch's scalar progress before full-pose correction.
      const double remaining =
          radius - std::abs(std::remainder(angle - initial_angle, kTwoPi));
      if (remaining > 0) {
        // Both fixed-sign solves can stall at the identical phase. Full-pose
        // correction already tries both elbows; reuse that exact calculation
        // before selecting this sign. Nearby phases must still be corrected
        // separately, regardless of the joint deduplication tolerance.
        if (!have_correction ||
            !numerics::IsRepresentedZero(angle - corrected_angle)) {
          cached_correction = RefineJointCandidates(
              lengths, target, angle, tolerance, budget, remaining, false);
          corrected_angle = angle;
          have_correction = true;
        }
        auto corrected = cached_correction;
        ElbowResidual residual;
        if (evaluate(circle, angle, sign, residual)) {
          KeepElbowBranch(circle, residual, corrected);
        }
        if (corrected.status == RecoveryStatus::Candidates &&
            !AppendUnique(corrected, accepted)) {
          return {};
        }
      }
    }
    if (accepted.count > 0) {
      accepted.status = RecoveryStatus::Candidates;
      return accepted;
    }
  }
  const auto fallback = RefineJointCandidates(lengths, target, initial_angle,
                                              tolerance, budget, radius);
  accepted.status = fallback.status;
  if (!AppendUnique(fallback, accepted)) {
    return {};
  }
  return accepted;
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

struct GeometricPhases {
  std::array<WristPhase, kBoundaryPhaseCapacity> values{};
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
    if (numerics::IsRepresentedZero(scale)) {
      return;
    }
    const double a = cosine / scale, b = sine / scale;
    const double norm = std::hypot(a, b);
    if (std::abs(rhs) > scale * norm + numerics::kElementaryRoundoff) {
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
  const double closest_angle =
      std::atan2(circle.p.dot(circle.w), circle.p.dot(circle.v)) +
      (circle.lengths.r >= 0 ? angle_conv::kPi : 0.0);
  const Vec3 closest_point =
      circle.p + circle.lengths.r * (circle.v * std::cos(closest_angle) +
                                     circle.w * std::sin(closest_angle));
  if (closest_point.norm() <
      numerics::kFoldedChartArmFraction *
          std::min(std::abs(circle.lengths.a), std::abs(circle.lengths.b))) {
    const auto folded = RefineJointCandidates(
        lengths, target, closest_angle, tolerance,
        options.max_polish_iterations, numerics::kPolishRadiusRad);
    if (folded.status == RecoveryStatus::Candidates &&
        !AppendUnique(folded, accepted)) {
      result.status = RecoveryStatus::NeedsRefinement;
      return result;
    }
  }
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
    double angle = polynomial.chart_angle + angle_conv::kPi;
    double radius = numerics::kPolishRadiusRad;
    bool ill_conditioned = true;
    if (i == roots.count) {
      const Eigen::Map<const Eigen::Matrix<double, kCoefficientCount, 1>>
          mapped(polynomial.coefficients.data());
      const double scale = mapped.cwiseAbs().maxCoeff();
      if (std::abs(polynomial.omitted_point_residual) >
          numerics::kUnitTolerance * scale) {
        continue;
      }
    } else {
      const auto &root = roots.candidates[i];
      bool repeated_projection = false;
      for (std::size_t j = 0; j < i; ++j) {
        repeated_projection =
            repeated_projection ||
            (numerics::IsRepresentedZero(root.value.real() -
                                         roots.candidates[j].value.real()) &&
             numerics::IsRepresentedZero(
                 std::abs(root.value.imag()) -
                 std::abs(roots.candidates[j].value.imag())));
      }
      // A conjugate pair has the identical real nomination, radius and
      // conditioning. PolishAndRecover already follows both elbow signs.
      if (repeated_projection) {
        continue;
      }
      const double relative_imaginary =
          std::abs(root.value.imag()) / (1 + std::abs(root.value.real()));
      const auto projected =
          RealProjection(polynomial.coefficients, root.value.real());
      // Multiple real roots can split into conjugate eigenvalues. Their real
      // projections are starting guesses only: unsquared geometry and full FK
      // below, not the imaginary part or polynomial slope, decide acceptance.
      if (relative_imaginary > numerics::kElementaryRoundoff) {
        if (relative_imaginary > numerics::kNearRealTolerance &&
            std::abs(projected.value) > numerics::kRootProjectionTolerance) {
          continue;
        }
      }
      if (root.relative_residual > numerics::kRootResidualTolerance) {
        result.status = RecoveryStatus::NeedsRefinement;
        return result;
      }
      angle = polynomial.chart_angle + 2 * std::atan(root.value.real());
      ill_conditioned =
          relative_imaginary > numerics::kElementaryRoundoff ||
          std::abs(projected.derivative) <= numerics::kRootSlopeTolerance;
      for (std::size_t j = 0; j < roots.count; ++j) {
        if (i == j || ill_conditioned) {
          continue;
        }
        const double other = polynomial.chart_angle +
                             2 * std::atan(roots.candidates[j].value.real());
        radius = std::min(radius, numerics::kNeighborRadiusFraction *
                                      std::abs(std::remainder(
                                          angle - other, 2 * angle_conv::kPi)));
      }
    }
    auto recovered = PolishAndRecover(
        lengths, target, circle, std::remainder(angle, 2 * angle_conv::kPi),
        tolerance, options.max_polish_iterations, radius, ill_conditioned);
    if (recovered.status == RecoveryStatus::NeedsRefinement) {
      for (std::size_t j = 0; j < boundary_phases.count; ++j) {
        const auto &boundary = boundary_phases.values[j];
        // An angle is needed only for a failed root's proximity check; the
        // recovery itself continues to use the original circle coordinates.
        const double boundary_angle =
            std::atan2(boundary.sine, boundary.cosine);
        if (std::abs(std::remainder(boundary_angle - angle,
                                    2 * angle_conv::kPi)) <= radius) {
          const auto trial =
              RecoverJointCandidates(lengths, target, boundary, tolerance);
          if (trial.status == RecoveryStatus::Candidates) {
            recovered = {};
            recovered.status = trial.status;
            if (!AppendUnique(trial, recovered)) {
              result.status = RecoveryStatus::NeedsRefinement;
              return result;
            }
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
      tolerance.orientation >= angle_conv::kPi ||
      options.max_polish_iterations < 0 ||
      options.max_polish_iterations > numerics::kMaxPolishIterations) {
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
  // Below binary64 resolution, a rounded zero residual is not evidence of
  // the requested accuracy. The 1e-30 position/orientation regression can
  // otherwise depend on which equivalent FK evaluation happens to round to
  // zero. Preserve the explicit refinement outcome at this known scale.
  if (tolerance.position < numerics::kEpsilon * circle.length_scale ||
      tolerance.orientation < numerics::kEpsilon) {
    result.status = RecoveryStatus::NeedsRefinement;
    return result;
  }
  // Re-express the same polynomial in another half-angle chart when roots
  // near the omitted point make geometric polishing ill-conditioned.
  const double closest_wrist =
      std::atan2(circle.p.dot(circle.w), circle.p.dot(circle.v)) +
      (circle.lengths.r >= 0 ? angle_conv::kPi : 0.0);
  const Vec3 closest =
      circle.p + circle.lengths.r * (circle.v * std::cos(closest_wrist) +
                                     circle.w * std::sin(closest_wrist));
  const double first_chart =
      closest.norm() < numerics::kFoldedChartArmFraction *
                           std::min(std::abs(circle.lengths.a),
                                    std::abs(circle.lengths.b))
          ? closest_wrist
          : 0.0;
  for (const double chart :
       {first_chart, angle_conv::kPi / 2, angle_conv::kPi / 4,
        -angle_conv::kPi / 4, 0.0, closest_wrist}) {
    result =
        DiscoverInChart(lengths, target, circle, tolerance, options, chart);
    if (result.status != RecoveryStatus::NeedsRefinement) {
      const auto needs_accuracy = [&](const JointCandidate &candidate) {
        return candidate.position_error >
                   numerics::kAccurateCorrectionTolerance *
                       circle.length_scale ||
               candidate.orientation_error >
                   numerics::kAccurateCorrectionTolerance;
      };
      if (result.status == RecoveryStatus::Candidates &&
          options.max_polish_iterations > 0 &&
          std::any_of(result.candidates.begin(),
                      result.candidates.begin() + result.count,
                      needs_accuracy)) {
        JointDiscovery precise;
        precise.status = result.status;
        precise.used_singular_representatives =
            result.used_singular_representatives;
        for (std::size_t i = 0; i < result.count; ++i) {
          auto candidate = result.candidates[i];
          if (needs_accuracy(candidate)) {
            auto corrected = candidate;
            const double radius =
                candidate.position_error > numerics::kPoseCorrectionTolerance *
                                               circle.length_scale ||
                        candidate.orientation_error >
                            numerics::kPoseCorrectionTolerance
                    ? numerics::kPolishRadiusRad
                    : numerics::kAccuracyPostureRadiusRad;
            const Vec6 lower = candidate.joints.array() - radius;
            const Vec6 upper = candidate.joints.array() + radius;
            if (PolishJointCandidate(lengths, target, tolerance,
                                     std::min(options.max_polish_iterations,
                                              numerics::kAccuracyIterations),
                                     corrected, &lower, &upper, true) &&
                corrected.position_error <=
                    std::max(candidate.position_error,
                             numerics::kAccurateCorrectionTolerance *
                                 circle.length_scale) &&
                corrected.orientation_error <=
                    std::max(candidate.orientation_error,
                             numerics::kAccurateCorrectionTolerance)) {
              candidate = corrected;
            }
          }
          JointRecovery one;
          one.count = 1;
          one.candidates[0] = candidate;
          if (!AppendUnique(one, precise)) {
            JointDiscovery unresolved;
            unresolved.status = RecoveryStatus::NeedsRefinement;
            return unresolved;
          }
        }
        result = precise;
      }
      return result;
    }
  }
  return result;
}
} // namespace crx::canonical
