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
};

struct JointDiscovery {
  RecoveryStatus status = RecoveryStatus::InvalidInput;
  std::size_t count = 0;
  bool used_singular_representatives = false;
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
// RoboDK mapping, command limits/turns and fallback are outside this stage.
auto DiscoverJointCandidates(const Lengths &lengths, const PoseIsoRT &target,
                             const PoseTolerance &tolerance,
                             const DiscoveryOptions &options = {})
    -> JointDiscovery;

} // namespace crx::canonical
