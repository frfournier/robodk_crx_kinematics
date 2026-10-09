#pragma once

#include <array>
#include <cstddef>

#include "crx_canonical.h"
#include "crx_types.h"

namespace crx::canonical {

struct PoseTolerance {
  double position = 0.0;    // Same physical length unit as lengths and target.
  double orientation = 0.0; // Radians.
};

enum class RecoveryStatus {
  Candidates,
  NoCandidate,
  NeedsRefinement,
  InvalidInput,
  NumericalRangeFailure
};

struct JointCandidate {
  Vec6 joints = Vec6::Zero(); // Canonical radians, each in [-pi, pi].
  double position_error = 0.0;
  double orientation_error = 0.0;
};

struct JointRecovery {
  RecoveryStatus status = RecoveryStatus::InvalidInput;
  std::size_t count = 0;
  std::array<JointCandidate, 4> candidates{}; // Two elbows, two bases each.
};

// Unit-circle coordinates in the prepared wrist basis (v, w). Keeping these
// coordinates avoids angle -> trigonometry round trips for analytic crossings.
struct WristPhase {
  double cosine = 1.0;
  double sine = 0.0;
};

// Nominal canonical CRX only, for one supplied wrist-circle angle. Prepares
// the circle, reconstructs elbows, recovers both base branches without seed
// overrides or division by sin(q5), and checks the complete canonical FK pose.
// All candidates must pass before publishing a result. Failures have count=0.
// NoCandidate concerns this angle only. NeedsRefinement does not run a
// fallback. This does not discover roots, map asset/command coordinates, apply
// limits or enumerate turns; calibrated axis geometry requires a separate
// calibrated FK.
auto RecoverJointCandidates(const Lengths &lengths, const PoseIsoRT &target,
                            double wrist_angle, const PoseTolerance &tolerance)
    -> JointRecovery;

auto RecoverJointCandidates(const Lengths &lengths, const PoseIsoRT &target,
                            const WristPhase &phase,
                            const PoseTolerance &tolerance) -> JointRecovery;

} // namespace crx::canonical
