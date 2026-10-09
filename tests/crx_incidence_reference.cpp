#include "crx_incidence_reference.h"

#include <algorithm>
#include <cmath>

#include <Eigen/Core>
#include <Eigen/SVD>

namespace crx::test::incidence_reference {
namespace {

auto ValidTolerance(double tolerance) -> bool {
  return std::isfinite(tolerance) && tolerance > 0.0 && tolerance < 1.0;
}

// Componentwise scale, with a dimensionless absolute floor for the normalized
// system. This is a numerical acceptance policy, not an interval error bound.
auto AffineError(const Mat3 &matrix, const Vec3 &rhs, const Vec3 &point)
    -> double {
  const Vec3 error = matrix * point - rhs;
  const Vec3 scale = matrix.cwiseAbs() * point.cwiseAbs() + rhs.cwiseAbs();
  if (!error.allFinite() || !scale.allFinite()) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  return (error.cwiseAbs().array() / scale.array().max(1.0)).maxCoeff();
}

auto SphereMatches(double a_squared, const Vec3 &point, double tolerance)
    -> bool {
  const double squared = point.squaredNorm();
  return std::isfinite(squared) && std::abs(squared - a_squared) <=
                                       tolerance * std::max(a_squared, squared);
}

} // namespace

auto FindElbowCandidates(double a, double b, const Vec3 &x, const Vec3 &u,
                         const IncidenceOptions &options) -> ElbowIncidence {
  ElbowIncidence result;
  constexpr double kUnitTolerance =
      128.0 * std::numeric_limits<double>::epsilon();
  if (!std::isfinite(a) || !std::isfinite(b) || a == 0.0 || b == 0.0 ||
      !x.allFinite() || !u.allFinite() ||
      !ValidTolerance(options.rank_relative_tolerance) ||
      !ValidTolerance(options.residual_relative_tolerance) ||
      std::abs(u.squaredNorm() - 1.0) > kUnitTolerance) {
    return result;
  }
  result.status = IncidenceStatus::NumericalRangeFailure;
  const double a_squared = a * a;
  const double b_squared = b * b;
  const double x_squared = x.squaredNorm();
  const double k = (a_squared + x_squared - b_squared) / 2.0;
  if (!std::isfinite(a_squared) || !std::isfinite(b_squared) ||
      !std::isfinite(x_squared) || !std::isfinite(k) || a_squared == 0.0 ||
      b_squared == 0.0 || (x_squared == 0.0 && !x.isZero(0.0))) {
    return result;
  }
  Mat3 matrix;
  matrix.row(0) = x.transpose();
  matrix.row(1) = u.transpose();
  matrix.row(2) = Vec3(-x.y(), x.x(), 0.0).transpose();
  const Vec3 rhs(k, x.dot(u), 0.0);
  if (!rhs.allFinite()) {
    return result;
  }
  const Eigen::JacobiSVD<Mat3, Eigen::NoQRPreconditioner> decomposition(
      matrix, Eigen::ComputeFullU | Eigen::ComputeFullV);
  if (decomposition.info() != Eigen::Success ||
      !decomposition.singularValues().allFinite()) {
    return result;
  }
  const Vec3 singular = decomposition.singularValues();
  const double cutoff = options.rank_relative_tolerance * singular[0];
  int rank = 0;
  for (int i = 0; i < 3; ++i) {
    if (singular[i] > 0.0) {
      if (singular[i] <= cutoff) {
        result.status = IncidenceStatus::UnresolvedRank;
        return result;
      }
      ++rank;
    }
  }
  // u is unit, hence a zero numerical rank indicates a numerical failure.
  if (rank == 0) {
    return result;
  }
  result.rank = rank;
  result.reciprocal_condition = singular[rank - 1] / singular[0];
  Vec3 center = Vec3::Zero();
  for (int i = 0; i < rank; ++i) {
    center += decomposition.matrixV().col(i) *
              (decomposition.matrixU().col(i).dot(rhs) / singular[i]);
  }
  if (!center.allFinite()) {
    return result;
  }
  const double affine_error = AffineError(matrix, rhs, center);
  if (!std::isfinite(affine_error)) {
    return result;
  }
  if (affine_error > options.residual_relative_tolerance) {
    result.status = IncidenceStatus::NoCandidate;
    return result;
  }
  const double center_squared = center.squaredNorm();
  const double slack = a_squared - center_squared;
  if (!std::isfinite(center_squared) || !std::isfinite(slack)) {
    return result;
  }
  const double slack_tolerance =
      options.residual_relative_tolerance * std::max(a_squared, center_squared);
  const double ambiguity_tolerance =
      slack_tolerance / result.reciprocal_condition;
  if (slack_tolerance == 0.0 || !std::isfinite(ambiguity_tolerance)) {
    return result;
  }
  if (rank == 3) {
    if (!SphereMatches(a_squared, center,
                       options.residual_relative_tolerance)) {
      result.status = std::abs(slack) <= ambiguity_tolerance
                          ? IncidenceStatus::UnresolvedBoundary
                          : IncidenceStatus::NoCandidate;
      return result;
    }
    result.points[0] = center;
    result.count = 1;
    result.status = IncidenceStatus::PointCandidates;
    return result;
  }
  if (slack < -ambiguity_tolerance) {
    result.status = IncidenceStatus::NoCandidate;
    return result;
  }
  if (slack <= ambiguity_tolerance) {
    result.status = IncidenceStatus::UnresolvedBoundary;
    return result;
  }
  const double radius = std::sqrt(slack);
  if (rank == 1) {
    result.center = center;
    result.radius = radius;
    result.basis[0] = decomposition.matrixV().col(1);
    result.basis[1] = decomposition.matrixV().col(2);
    result.status = IncidenceStatus::CircleCandidate;
    return result;
  }
  const Vec3 offset = radius * decomposition.matrixV().col(2);
  const std::array<Vec3, 2> points{center + offset, center - offset};
  for (const Vec3 &point : points) {
    if (!point.allFinite()) {
      return result;
    }
    const double error = AffineError(matrix, rhs, point);
    if (!std::isfinite(error)) {
      return result;
    }
    if (error > options.residual_relative_tolerance ||
        !SphereMatches(a_squared, point, options.residual_relative_tolerance)) {
      // Failure after a nominally consistent solve is unresolved roundoff,
      // not grounds to silently discard one branch.
      result.status = IncidenceStatus::UnresolvedBoundary;
      return result;
    }
  }
  result.points = points;
  result.count = points.size();
  result.status = IncidenceStatus::PointCandidates;
  return result;
}

} // namespace crx::test::incidence_reference
