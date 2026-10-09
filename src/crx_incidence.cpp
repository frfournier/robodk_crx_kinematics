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
// A narrow ambiguity band prevents treating roundoff-sized incompatibility as
// a definitive branch rejection. This is not a calibration error allowance.
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
  if (distance <= kDirectionTolerance * scale ||
      horizontal <= kDirectionTolerance * scale) {
    result.status = IncidenceStatus::NeedsRefinement;
    return result;
  }
  const Vec3 radial = x / distance;
  const double along =
      (a_squared + squared_distance - b_squared) / (2.0 * distance);
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
  if (height_squared <= kAmbiguityFactor * height_tolerance) {
    result.status = IncidenceStatus::NeedsRefinement;
    return result;
  }
  // Unit vector perpendicular to x within its vertical base plane. Avoid
  // products x.z()*x.x() and division by squared horizontal distance.
  const Vec3 perpendicular(-radial.z() * (x.x() / horizontal),
                           -radial.z() * (x.y() / horizontal),
                           horizontal / distance);
  const Vec3 center = along * radial;
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
    if (error <= tolerance) {
      accepted[count++] = elbow;
    } else if (error <= kAmbiguityFactor * tolerance) {
      result.status = IncidenceStatus::NeedsRefinement;
      return result;
    }
  }
  result.status = count == 0 ? IncidenceStatus::NoCandidate
                             : IncidenceStatus::PointCandidates;
  result.points = accepted;
  result.count = count;
  return result;
}

} // namespace crx::canonical
