#include "crx_incidence.h"
#include "crx_numerics.h"
#include "crx_types.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

namespace crx::canonical {
namespace {
// Roundoff-sized wrist-plane incompatibility produces a numerical candidate,
// not a definitive branch rejection. Joint recovery and full FK must still
// accept it at the caller's unchanged physical tolerances. Not calibration
// slack.

auto SphereMatches(double squared_length, const Vec3 &vector) -> bool {
  const double squared = vector.squaredNorm();
  return std::isfinite(squared) && std::abs(squared - squared_length) <=
                                       numerics::kIncidenceResidualTolerance *
                                           std::max(squared, squared_length);
}
} // namespace

auto FindElbowCandidates(double a, double b, const Vec3 &x, const Vec3 &u)
    -> ElbowCandidates {
  ElbowCandidates result;
  if (!std::isfinite(a) || !std::isfinite(b) ||
      numerics::IsRepresentedZero(a) || numerics::IsRepresentedZero(b) ||
      !x.allFinite() || !u.allFinite() ||
      std::abs(u.squaredNorm() - 1.0) > numerics::kUnitTolerance) {
    return result;
  }
  result.status = IncidenceStatus::NumericalRangeFailure;
  const double a_squared = a * a;
  const double b_squared = b * b;
  const double squared_distance = x.squaredNorm();
  if (!std::isfinite(a_squared) || !std::isfinite(b_squared) ||
      !std::isfinite(squared_distance) ||
      numerics::IsRepresentedZero(a_squared) ||
      numerics::IsRepresentedZero(b_squared) ||
      (numerics::IsRepresentedZero(squared_distance) &&
       !x.array().unaryExpr(&numerics::IsRepresentedZero).all())) {
    return result;
  }
  const double distance = std::sqrt(squared_distance);
  const double horizontal = std::hypot(x.x(), x.y());
  const double scale = std::max({std::abs(a), std::abs(b), distance});
  if (distance <= numerics::kDirectionTolerance * scale) {
    result.status = IncidenceStatus::NeedsRefinement;
    return result;
  }
  const Vec3 radial = x / distance;
  const double along =
      ((a - b) * (a + b) + squared_distance) / (2.0 * distance);
  const double height_squared = a_squared - along * along;
  const double height_scale = std::max(a_squared, along * along);
  const double height_tolerance =
      numerics::kIncidenceResidualTolerance * height_scale;
  if (!std::isfinite(along) || !std::isfinite(height_squared) ||
      !std::isfinite(height_tolerance) ||
      numerics::IsRepresentedZero(height_tolerance)) {
    return result;
  }
  if (height_squared <
      -numerics::kIncidenceAmbiguityFactor * height_tolerance) {
    result.status = IncidenceStatus::NoCandidate;
    return result;
  }
  if (horizontal <= numerics::kDirectionTolerance * scale) {
    // On the base axis, the base plane is determined by the elbow instead.
    // Intersect the horizontal arm-circle with the unsquared wrist plane.
    const double z = along * (x.z() / distance);
    const double wrist_horizontal = std::hypot(u.x(), u.y());
    const double rhs = (x.z() - z) * u.z();
    if (wrist_horizontal <= numerics::kDirectionTolerance) {
      result.status =
          std::abs(rhs) > numerics::kIncidenceResidualTolerance * scale
              ? IncidenceStatus::NoCandidate
              : IncidenceStatus::NeedsRefinement;
      return result;
    }
    const Vec3 direction(u.x() / wrist_horizontal, u.y() / wrist_horizontal, 0);
    const double along_plane = rhs / wrist_horizontal;
    const double remaining = height_squared - along_plane * along_plane;
    if (remaining < -numerics::kIncidenceAmbiguityFactor * height_tolerance) {
      result.status = IncidenceStatus::NoCandidate;
      return result;
    }
    const Vec3 center = Vec3(0, 0, z) + along_plane * direction;
    const Vec3 offset = std::sqrt(std::max(0.0, remaining)) *
                        Vec3(-direction.y(), direction.x(), 0);
    const std::array<Vec3, 2> points{center + offset, center - offset};
    const std::size_t count = remaining > 0 ? 2 : 1;
    for (std::size_t i = 0; i < count; ++i) {
      if (!SphereMatches(a_squared, points[i]) ||
          !SphereMatches(b_squared, points[i] - x) ||
          std::abs((points[i] - x).dot(u)) >
              numerics::kIncidenceResidualTolerance * scale) {
        result.status = IncidenceStatus::NeedsRefinement;
        return result;
      }
    }
    result.points = points;
    result.count = count;
    result.status = IncidenceStatus::PointCandidates;
    return result;
  }
  // Unit vector perpendicular to x within its vertical base plane. Avoid
  // products x.z()*x.x() and division by squared horizontal distance.
  const Vec3 perpendicular(-radial.z() * (x.x() / horizontal),
                           -radial.z() * (x.y() / horizontal),
                           horizontal / distance);
  const Vec3 center = along * radial;
  const double slope = perpendicular.dot(u);
  if (height_squared <= numerics::kFourthRootEpsilon * height_scale) {
    // At a straight/folded arm, sqrt(aÂ²-alongÂ²) loses the small elbow
    // displacement. Recover its signed value from the unsquared wrist plane
    // instead, then check both original spheres. No division by the height.
    const double residual = (center - x).dot(u);
    Vec3 elbow = center;
    if (std::abs(slope) > numerics::kDirectionTolerance) {
      elbow -= (residual / slope) * perpendicular;
    }
    if (std::abs(slope) > numerics::kDirectionTolerance ||
        height_squared <= 0) {
      if (SphereMatches(a_squared, elbow) &&
          SphereMatches(b_squared, elbow - x) &&
          std::abs((elbow - x).dot(u)) <=
              numerics::kIncidenceResidualTolerance * scale) {
        result.status = IncidenceStatus::PointCandidates;
        result.points[0] = elbow;
        result.count = 1;
      } else {
        const double sphere_error =
            std::max(std::abs(elbow.squaredNorm() - a_squared),
                     std::abs((elbow - x).squaredNorm() - b_squared));
        result.status =
            sphere_error > numerics::kIncidenceAmbiguityFactor *
                               numerics::kIncidenceResidualTolerance * scale *
                               scale
                ? IncidenceStatus::NoCandidate
                : IncidenceStatus::NeedsRefinement;
      }
      return result;
    }
  }
  const Vec3 offset = std::sqrt(height_squared) * perpendicular;
  const std::array<Vec3, 2> elbows{center + offset, center - offset};
  std::array<Vec3, 2> accepted{Vec3::Zero(), Vec3::Zero()};
  std::size_t count = 0;
  for (const Vec3 &elbow : elbows) {
    const Vec3 forearm = elbow - x;
    const double error = std::abs(forearm.dot(u));
    const double tolerance = numerics::kIncidenceResidualTolerance * scale;
    if (!elbow.allFinite() || !std::isfinite(error)) {
      return result;
    }
    if (!SphereMatches(a_squared, elbow) ||
        !SphereMatches(b_squared, forearm)) {
      result.status = IncidenceStatus::NeedsRefinement;
      return result;
    }
    if (error <= numerics::kIncidenceAmbiguityFactor * tolerance) {
      accepted[count++] = elbow;
    }
  }
  result.status = count == 0 ? IncidenceStatus::NoCandidate
                             : IncidenceStatus::PointCandidates;
  result.points = accepted;
  result.count = count;
  return result;
}

} // namespace crx::canonical
