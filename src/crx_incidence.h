#pragma once

#include <array>
#include <cstddef>

#include "crx_types.h"

namespace crx::canonical {

enum class IncidenceStatus {
  PointCandidates,
  NoCandidate,
  NeedsRefinement,
  InvalidInput,
  NumericalRangeFailure
};

struct ElbowCandidates {
  IncidenceStatus status = IncidenceStatus::InvalidInput;
  std::size_t count = 0;
  std::array<Vec3, 2> points{Vec3::Zero(), Vec3::Zero()};
};

// Nominal CRX geometry only; normalized signed a,b must be nonzero, u unit.
// Intersect the two arm spheres with the base plane through x=O4. Test both
// elbows against (y-x).u=0 and the original arm lengths; no rank classification
// or continuous-family enumeration. Vertical/near-origin wrists, tangencies,
// and ambiguous compatibility return NeedsRefinement with no partial results.
// NoCandidate concerns this wrist point only, not target reachability.
// These numerical candidates still need joint recovery and full-pose FK checks.
// Joint offsets and family-preserving lengths belong in the coordinate bridge.
// Axis-geometry calibration requires calibrated FK/refinement, not looser
// checks.
auto FindElbowCandidates(double a, double b, const Vec3 &x, const Vec3 &u)
    -> ElbowCandidates;

} // namespace crx::canonical
