#include "crx_incidence.h"
#include "crx_types.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

namespace crx::canonical {
namespace {
constexpr double kDirectionTolerance =
    256.0 * std::numeric_limits<double>::epsilon();
constexpr double kResidualTolerance =
    512.0 * std::numeric_limits<double>::epsilon();
// Roundoff-sized wrist-plane incompatibility produces a numerical candidate,
// not a definitive branch rejection. Joint recovery and full FK must still
// accept it at the caller's unchanged physical tolerances. Not calibration
// slack.
constexpr double kAmbiguityFactor = 8.0;

auto SphereMatches(double squared_length, const Vec3 &vector) -> bool {
  const double squared = vector.squaredNorm();
  return std::isfinite(squared) &&
         std::abs(squared - squared_length) <=
             kResidualTolerance * std::max(squared, squared_length);
}
} // namespace

auto FindElbowCandidates(double a, double b, const Vec3 &x, const Vec3 &u)
    -> ElbowCandidates {
  ElbowCandidates result;
  constexpr double kUnitTolerance =
      128.0 * std::numeric_limits<double>::epsilon();
  if (!std::isfinite(a) || !std::isfinite(b) || a == 0.0 || b == 0.0 ||
      !x.allFinite() || !u.allFinite() ||
      std::abs(u.squaredNorm() - 1.0) > kUnitTolerance) {
    return result;
  }
  result.status = IncidenceStatus::NumericalRangeFailure;
  const double a_squared = a * a;
  const double b_squared = b * b;
  const double squared_distance = x.squaredNorm();
  if (!std::isfinite(a_squared) || !std::isfinite(b_squared) ||
      !std::isfinite(squared_distance) || a_squared == 0.0 ||
      b_squared == 0.0 || (squared_distance == 0.0 && !x.isZero(0.0))) {
    return result;
  }
  const double distance = std::sqrt(squared_distance);
  const double horizontal = std::hypot(x.x(), x.y());
  const double scale = std::max({std::abs(a), std::abs(b), distance});
  if (distance <= kDirectionTolerance * scale) {
    result.status = IncidenceStatus::NeedsRefinement;
    return result;
  }
  const Vec3 radial = x / distance;
  const double along =
      ((a - b) * (a + b) + squared_distance) / (2.0 * distance);
  const double height_squared = a_squared - along * along;
  const double height_scale = std::max(a_squared, along * along);
  const double height_tolerance = kResidualTolerance * height_scale;
  if (!std::isfinite(along) || !std::isfinite(height_squared) ||
      !std::isfinite(height_tolerance) || height_tolerance == 0.0) {
    return result;
  }
  if (height_squared < -kAmbiguityFactor * height_tolerance) {
    result.status = IncidenceStatus::NoCandidate;
    return result;
  }
  if (horizontal <= kDirectionTolerance * scale) {
    // On the base axis, the base plane is determined by the elbow instead.
    // Intersect the horizontal arm-circle with the unsquared wrist plane.
    const double z = along * (x.z() / distance);
    const double wrist_horizontal = std::hypot(u.x(), u.y());
    const double rhs = (x.z() - z) * u.z();
    if (wrist_horizontal <= kDirectionTolerance) {
      result.status = std::abs(rhs) > kResidualTolerance * scale
                          ? IncidenceStatus::NoCandidate
                          : IncidenceStatus::NeedsRefinement;
      return result;
    }
    const Vec3 direction(u.x() / wrist_horizontal, u.y() / wrist_horizontal, 0);
    const double along_plane = rhs / wrist_horizontal;
    const double remaining = height_squared - along_plane * along_plane;
    if (remaining < -kAmbiguityFactor * height_tolerance) {
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
          std::abs((points[i] - x).dot(u)) > kResidualTolerance * scale) {
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
  if (height_squared <=
      std::sqrt(std::sqrt(std::numeric_limits<double>::epsilon())) *
          height_scale) {
    // At a straight/folded arm, sqrt(a²-along²) loses the small elbow
    // displacement. Recover its signed value from the unsquared wrist plane
    // instead, then check both original spheres. No division by the height.
    const double residual = (center - x).dot(u);
    Vec3 elbow = center;
    if (std::abs(slope) > kDirectionTolerance) {
      elbow -= (residual / slope) * perpendicular;
    }
    if (std::abs(slope) > kDirectionTolerance || height_squared <= 0) {
      if (SphereMatches(a_squared, elbow) &&
          SphereMatches(b_squared, elbow - x) &&
          std::abs((elbow - x).dot(u)) <= kResidualTolerance * scale) {
        result.status = IncidenceStatus::PointCandidates;
        result.points[0] = elbow;
        result.count = 1;
      } else {
        const double sphere_error =
            std::max(std::abs(elbow.squaredNorm() - a_squared),
                     std::abs((elbow - x).squaredNorm() - b_squared));
        result.status =
            sphere_error > kAmbiguityFactor * kResidualTolerance * scale * scale
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
    const double tolerance = kResidualTolerance * scale;
    if (!elbow.allFinite() || !std::isfinite(error)) {
      return result;
    }
    if (!SphereMatches(a_squared, elbow) ||
        !SphereMatches(b_squared, forearm)) {
      result.status = IncidenceStatus::NeedsRefinement;
      return result;
    }
    if (error <= kAmbiguityFactor * tolerance) {
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
