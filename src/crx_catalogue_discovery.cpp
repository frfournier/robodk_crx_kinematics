// SPDX-License-Identifier: Apache-2.0
// SPDX-FileCopyrightText: 2026 Centre Technologique en Aérospatiale

#include "crx_discovery.h"
#include "crx_incidence.h"
#include "crx_phase_catalogue.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <utility>
#include <vector>

namespace crx::catalogue {
namespace {
auto Midpoint(interval::Interval x) -> double {
  return (x.lo / 2) + (x.hi / 2);
}

auto TryBall(const exact::Problem &problem, const Vec6 &candidate,
             const Options &options, BuildResult &result,
             std::vector<ball::Certificate> &balls) -> bool {
  if (result.attempted_balls >= options.max_candidate_certificates) {
    result.candidate_budget_exhausted = true;
    return false;
  }
  ++result.attempted_balls;
  ball::Centre centre;
  for (std::size_t j = 0; j < centre.size(); ++j)
    centre.at(j) = candidate(static_cast<int>(j));
  auto attempt = ball::Certify(problem, centre);
  result.ball_failures.push_back(attempt.failure);
  if (!attempt.IsCertified())
    return false;
  ++result.certified_balls;
  balls.push_back(std::move(attempt.certificate));
  return true;
}

template <std::size_t N>
auto Straddles(const std::array<interval::Interval, N> &values) -> bool {
  double lower = std::numeric_limits<double>::infinity();
  double upper = -lower;
  for (const auto value : values) {
    if (!value.IsValid())
      return false;
    lower = std::min(lower, value.lo);
    upper = std::max(upper, value.hi);
  }
  return lower <= 0 && upper >= 0;
}

auto Diagnose(const canonical::WristCircle &circle, double phase,
              bool polynomial_ambiguity) -> BuildResult::Recovery {
  BuildResult::Recovery result;
  result.phase = phase;
  const Vec3 u = circle.v * std::cos(phase) + circle.w * std::sin(phase);
  const Vec3 x = circle.p + circle.lengths.r * u;
  const double a = circle.lengths.a, b = circle.lengths.b;
  const double scale = std::max({std::abs(a), std::abs(b), x.norm()});
  const double squared = scale * scale;
  const double s = x.squaredNorm();
  const double k = ((a - b) * (a + b) + s) / 2;
  result.delta_defect =
      std::abs(x.dot(u.cross(Vec3::UnitZ().cross(x)))) / squared;
  result.base_axis_defect = std::hypot(x.x(), x.y()) / scale;
  result.coincident_defect =
      std::max(x.norm() / scale, std::abs((a - b) * (a + b)) / squared);
  result.shell_defect = std::abs(a * a * s - k * k) / (squared * squared);
  result.vertical_defect = std::numeric_limits<double>::quiet_NaN();
  result.compatibility_defect = std::numeric_limits<double>::quiet_NaN();
  const auto elbows = canonical::FindElbowCandidates(a, b, x, u);
  for (std::size_t i = 0; i < elbows.count; ++i) {
    const auto &elbow = elbows.points.at(i);
    const double vertical =
        std::max(std::hypot(elbow.x(), elbow.y()), std::hypot(x.x(), x.y())) /
        scale;
    const double compatibility = std::abs((elbow - x).dot(u)) / scale;
    if (i == 0 || vertical < result.vertical_defect)
      result.vertical_defect = vertical;
    if (i == 0 || compatibility < result.compatibility_defect)
      result.compatibility_defect = compatibility;
  }
  // These scale-invariant numerical defects are dispatch diagnostics only.
  // The original constraints, full-pose FK and exact-problem balls remain
  // separate. Enclosing every polynomial coefficient around zero is weaker
  // than proving a zero polynomial, still less a positive-dimensional fibre.
  constexpr double tolerance = numerics::kGeometryTolerance;
  if (result.delta_defect <= tolerance)
    result.near_stratum |= NearDeterminantZero;
  if (result.base_axis_defect <= tolerance)
    result.near_stratum |= NearBaseAxis;
  if (result.coincident_defect <= tolerance)
    result.near_stratum |= NearCoincidentSpheres;
  if (result.vertical_defect <= tolerance)
    result.near_stratum |= NearVerticalArm;
  if (result.shell_defect <= tolerance)
    result.near_stratum |= NearArmShell;
  if (polynomial_ambiguity)
    result.near_stratum |= PolynomialEnclosesZero;
  return result;
}

void RecoverExceptional(const exact::Problem &problem,
                        const root::Certificate &roots,
                        const canonical::WristCircle &circle,
                        const PoseIsoRT &target, const Options &options,
                        BuildResult &result,
                        std::vector<ball::Certificate> &balls) {
  const auto &data = roots.Values();
  bool polynomial_ambiguity = true;
  for (const auto &chart : data.polynomials)
    for (const auto value : chart.residual)
      polynomial_ambiguity = polynomial_ambiguity && value.ContainsZero();
  const canonical::PoseTolerance tolerance{1e-12, 1e-12};
  const auto nominate = [&](double phase, std::uint32_t source) {
    if (!std::isfinite(phase))
      return;
    ++result.recovery_domains;
    phase = std::remainder(phase, 2 * angle_conv::kPi);
    for (auto &existing : result.recoveries) {
      if (std::abs(
              std::remainder(existing.phase - phase, 2 * angle_conv::kPi)) <=
          numerics::kJointPhaseToleranceRad) {
        existing.source |= source;
        return;
      }
    }
    if (result.recoveries.size() >= BuildResult::kRecoveryLimit) {
      result.recovery_budget_exhausted = true;
      return;
    }
    auto recovery = Diagnose(circle, phase, polynomial_ambiguity);
    recovery.source = source;
    recovery.numerical = canonical::RecoverPreparedJointCandidates(
        circle, target, phase, tolerance);
    if (recovery.numerical.status ==
            canonical::RecoveryStatus::NeedsRefinement ||
        (recovery.numerical.status == canonical::RecoveryStatus::NoCandidate &&
         (source & (DeterminantNomination | UnresolvedDomain)))) {
      recovery.refinement_attempted = true;
      canonical::JointRefiner refiner(circle, target, tolerance,
                                      numerics::kPolishIterations);
      recovery.numerical = refiner.Refine(phase, numerics::kPolishRadiusRad);
    }
    result.nominations += static_cast<std::uint32_t>(recovery.numerical.count);
    for (std::size_t i = 0; i < recovery.numerical.count; ++i) {
      const auto before = result.attempted_balls;
      if (TryBall(problem, recovery.numerical.candidates.at(i).joints, options,
                  result, balls))
        ++recovery.certified_balls;
      recovery.attempted_balls += result.attempted_balls - before;
    }
    result.recoveries.push_back(std::move(recovery));
  };
  // Q has degree at most four. Its rounded coefficient midpoints nominate
  // exceptional phases even when the octic eigensolve has no usable root.
  // Never treat numerical degree drops or real projections as exact equality.
  for (std::size_t chart = 0; chart < data.polynomials.size(); ++chart) {
    canonical::Coefficients coefficients{};
    bool valid = true;
    for (std::size_t i = 0; i < data.polynomials.at(chart).determinant.size();
         ++i) {
      const auto value = data.polynomials.at(chart).determinant.at(i);
      valid = valid && value.IsValid();
      coefficients.at(i) = Midpoint(value);
    }
    if (!valid)
      continue;
    const auto candidates = canonical::FindRootCandidates(coefficients);
    const double shift = (static_cast<double>(data.chart_shift) / 8 +
                          static_cast<double>(chart)) *
                         angle_conv::kPi;
    for (std::size_t i = 0; i < candidates.count; ++i) {
      const auto value = candidates.candidates.at(i).value;
      if (std::abs(value.real()) <= 1 &&
          std::abs(value.imag()) <= numerics::kFourthRootEpsilon)
        nominate(shift + 2 * std::atan(value.real()), DeterminantNomination);
    }
  }
  const auto boundaries = canonical::NominateWristBoundaries(circle);
  for (std::size_t i = 0; i < boundaries.count; ++i)
    nominate(std::atan2(boundaries.values.at(i).sine,
                        boundaries.values.at(i).cosine),
             BoundaryNomination);
  // An identically ambiguous eliminant has no isolated roots to nominate.
  // Four deterministic phase representatives supplement analytic boundaries;
  // they are not an angular coverage argument or a family parametrization.
  if (polynomial_ambiguity || data.entire_circle_unresolved)
    for (const double phase :
         {0.0, angle_conv::kPi / 2, angle_conv::kPi, -angle_conv::kPi / 2})
      nominate(phase, CardinalNomination);
  // Keep deterministic family representatives ahead of the potentially large
  // unresolved frontier, so leaf work cannot consume their recovery slots.
  for (const auto &partition : data.partitions)
    for (const auto &leaf : partition.leaves)
      if (leaf.kind == root::Kind::Unresolved && leaf.phase.IsValid() &&
          Straddles(leaf.residual_bernstein) &&
          Straddles(leaf.determinant_bernstein))
        nominate(Midpoint(leaf.phase), UnresolvedDomain);
}
} // namespace

auto Build(const exact::Problem &problem, Options options) -> BuildResult {
  BuildResult result;
  auto roots = root::Certify(problem, options.roots);
  std::vector<ball::Certificate> balls;
  if (!problem.IsReady() || !interval::EnvironmentSupported() ||
      (roots.IsComplete() && roots.Values().simple_roots == 0)) {
    result.catalogue =
        Assemble(problem, std::move(roots), {}, options.max_pair_tests);
    return result;
  }
  const auto &data = problem.Data();
  const canonical::Lengths lengths{
      Midpoint(data.lengths.a), Midpoint(data.lengths.b),
      Midpoint(data.lengths.c), Midpoint(data.lengths.r)};
  PoseIsoRT target = PoseIsoRT::Identity();
  for (int i = 0; i < 3; ++i) {
    target.translation()(i) = Midpoint(data.target.translation(i));
    for (int j = 0; j < 3; ++j)
      target.linear()(i, j) = Midpoint(data.target.rotation(i, j));
  }
  // No command bounds/capacity are passed to canonical discovery. Soundness
  // comes from exact-problem balls, never from this numerical tolerance.
  const auto discovery =
      canonical::DiscoverJointCandidates(lengths, target, {1e-12, 1e-12});
  result.numerical_status = static_cast<int>(discovery.status);
  result.nominations = static_cast<std::uint32_t>(discovery.count);
  for (std::size_t i = 0; i < discovery.count; ++i) {
    if (result.attempted_balls >= options.max_candidate_certificates) {
      result.candidate_budget_exhausted = true;
      break;
    }
    TryBall(problem, discovery.candidates.at(i).joints, options, result, balls);
  }
  if (!roots.IsComplete()) {
    canonical::WristCircle circle;
    if (canonical::PrepareWristCircle(lengths, target, circle) ==
        canonical::PreparationStatus::Ready)
      RecoverExceptional(problem, roots, circle, target, options, result,
                         balls);
  }
  result.catalogue = Assemble(problem, std::move(roots), std::move(balls),
                              options.max_pair_tests);
  return result;
}
} // namespace crx::catalogue
