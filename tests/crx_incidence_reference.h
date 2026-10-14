#pragma once

#include <array>
#include <cstddef>
#include <limits>

#include "crx_types.h"

// Test-only SVD comparison for nominal CRX geometry. Broader classifications
// exercise the algebra; they do not define library support or production
// policy.
namespace crx::test::incidence_reference {

enum class IncidenceStatus {
  PointCandidates,
  CircleCandidate,
  NoCandidate,
  UnresolvedRank,
  UnresolvedBoundary,
  InvalidInput,
  NumericalRangeFailure
};

struct IncidenceOptions {
  double rank_relative_tolerance =
      256.0 * std::numeric_limits<double>::epsilon();
  double residual_relative_tolerance =
      512.0 * std::numeric_limits<double>::epsilon();
};

struct ElbowIncidence {
  IncidenceStatus status = IncidenceStatus::InvalidInput;
  // Numerical SVD rank, not an exact-rank certificate. -1 if not determined.
  int rank = -1;
  // Smallest retained singular value / largest, for conditioning diagnostics.
  double reciprocal_condition = 0.0;
  std::size_t count = 0;
  std::array<Vec3, 2> points{Vec3::Zero(), Vec3::Zero()};
  // CircleCandidate describes
  // y=center+radius*(basis[0]*cos(t)+basis[1]*sin(t)). It supplies no
  // representative, count, or global zero-polynomial solution.
  Vec3 center = Vec3::Zero();
  std::array<Vec3, 2> basis{Vec3::Zero(), Vec3::Zero()};
  double radius = 0.0;
};

// Appendix G6, for a single wrist-circle point: x=O4, u=unit fifth-axis vector,
// signed a,b nonzero. All lengths must already be normalized by a common scale.
// Solve [x^T; u^T; (ez cross x)^T] y = [(a^2+|x|^2-b^2)/2; x.u; 0],
// then intersect with |y|^2=a^2. Fixed-size SVD; no determinant division.
// Positive singular values at/below the relative rank threshold are unresolved;
// exact SVD zeros define the provisional nullspace. Near sphere tangencies and
// sphere mismatches within residual_tolerance/reciprocal_condition are
// unresolved. This conditioning band is a heuristic, not an error enclosure.
// NoCandidate is a numerical rejection at this supplied angle, never a
// geometric infeasibility certificate. Point/circle candidates require full
// incidence, angle and FK validation before any production use. Rounded
// coefficients/inputs carry no uncertainty enclosure.
auto FindElbowCandidates(double a, double b, const Vec3 &x, const Vec3 &u,
                         const IncidenceOptions &options = {})
    -> ElbowIncidence;

} // namespace crx::test::incidence_reference
