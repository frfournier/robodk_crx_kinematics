#pragma once

#include <array>
#include <cstddef>

#include "crx_canonical.h"
#include "crx_joint_recovery.h"
#include "crx_numerics.h"
#include "crx_polynomial_roots.h"
#include "crx_types.h"

namespace crx::canonical {

// Four analytic circle crossings, with at most two phases per crossing.
inline constexpr std::size_t kBoundaryPhaseCapacity = 4 * 2;

struct DiscoveryOptions {
  RootOptions roots{};
  int max_polish_iterations = numerics::kPolishIterations;
  // Optional coupled command box. Only J3/J6 nominate excluded phase domains;
  // final command limits and FK remain authoritative. Supply both or neither.
  const Vec6 *lower_limits = nullptr;
  const Vec6 *upper_limits = nullptr;
};

struct JointDiscovery {
  RecoveryStatus status = RecoveryStatus::InvalidInput;
  std::size_t count = 0;
  bool used_singular_representatives = false;
  std::size_t pruned_nominations = 0;
  // Bounded storage for roots and analytic boundary crossings after posture
  // deduplication. Capacity exhaustion returns NeedsRefinement, never
  // truncates.
  // Every polynomial nomination (including the omitted point), every boundary
  // phase, and the target-derived folded nomination can survive deduplication.
  // Each polynomial phase polishes both elbow branches independently.
  // Size from those work bounds, not an exact-geometric solution-count theorem.
  static constexpr std::size_t kCapacity =
      kCoefficientCount * 2 * JointRecovery::kRefinementCapacity +
      kBoundaryPhaseCapacity * JointRecovery::kDirectCapacity +
      JointRecovery::kRefinementCapacity;
  std::array<JointCandidate, kCapacity> candidates{};
};

// Target -> polynomial and analytic boundary crossings -> wrist angles ->
// polished, FK-validated postures. Repeated-root projections are only guesses;
// acceptance uses the original geometry and physical FK tolerances.
// No generating angle or joint seed is supplied. Fixed work/storage bounds;
// ambiguity or exhausted work returns NeedsRefinement with no partial list.
// Candidates/NoCandidate describe numerical discovery, NOT certified coverage
// or infeasibility. Only equivalent joint postures are deduplicated modulo 2pi.
// RoboDK mapping, final command limits/turns and fallback are outside this
// stage. A search with domain exclusions cannot authorize empty-search seed
// fallback or constrained correction: retry without domains first.
auto DiscoverJointCandidates(const Lengths &lengths, const PoseIsoRT &target,
                             const PoseTolerance &tolerance,
                             const DiscoveryOptions &options = {})
    -> JointDiscovery;

// Follow only the seed's elbow branch in a bounded wrist-phase window.
// No polynomial eigensolve or singular-family search. Failure is a local
// miss, never evidence of infeasibility; the caller must use full discovery.
// Returns canonical phases; command limits and turn selection are external.
auto DiscoverNearbyJointCandidates(const Lengths &lengths,
                                   const PoseIsoRT &target,
                                   const PoseTolerance &tolerance,
                                   const Vec6 &seed) -> JointRecovery;

} // namespace crx::canonical
