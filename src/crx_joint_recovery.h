#pragma once

#include <array>
#include <cstddef>

#include "crx_canonical.h"
#include "crx_types.h"

namespace crx::canonical {

struct PoseTolerance {
  double position = 0.0;    // Same physical length unit as lengths and target.
  double orientation = 0.0; // Radians.
  auto IsValid() const -> bool;
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
  static constexpr std::size_t kDirectCapacity = 2 * 2;
  static constexpr std::size_t kRefinementCapacity = 2 * kDirectCapacity;
  RecoveryStatus status = RecoveryStatus::InvalidInput;
  std::size_t count = 0;
  bool used_singular_representatives = false;
  // Two geometric charts, each with two elbows and two base branches.
  // Direct incidence recovery uses at most the first four entries.
  std::array<JointCandidate, kRefinementCapacity> candidates{};
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

// Prepared entry points require a Ready circle for this exact, unchanged
// target. Discovery shares it across all nominations; phase/tolerance gates
// still apply.
auto RecoverPreparedJointCandidates(const WristCircle &circle,
                                    const PoseIsoRT &target, double wrist_angle,
                                    const PoseTolerance &tolerance)
    -> JointRecovery;
auto RecoverPreparedJointCandidates(const WristCircle &circle,
                                    const PoseIsoRT &target,
                                    const WristPhase &phase,
                                    const PoseTolerance &tolerance)
    -> JointRecovery;

// Complete a recovered posture's opposite base branch, validating its rounded
// command vector against the same prepared target. This owns the base symmetry
// used by geometric discovery and by limit-constrained correction.
auto RecoverBaseFlip(const WristCircle &circle, const PoseIsoRT &target,
                     const PoseTolerance &tolerance,
                     const JointCandidate &first, JointCandidate &candidate)
    -> bool;

// Bounded full-pose correction of a geometric nomination. No random/joint-seed
// search: accepted postures must remain in the nominated wrist-phase window.
// Circle, target and tolerance must outlive the refiner and remain unchanged.
class JointRefiner {
public:
  // Same prepared-circle contract as recovery.
  JointRefiner(const WristCircle &circle, const PoseIsoRT &target,
               const PoseTolerance &tolerance, int budget)
      : circle_(circle), target_(target), tolerance_(tolerance),
        budget_(budget) {}

  auto Refine(double wrist_angle, double radius,
              bool singular_representatives = true) -> JointRecovery;

private:
  const WristCircle &circle_;
  const PoseIsoRT &target_;
  const PoseTolerance &tolerance_;
  int budget_;
  // Folded representative seeds depend only on this immutable target problem.
  // Cache their full-pose correction, then apply each nomination's own window.
  bool have_folded_ = false;
  JointRecovery folded_;
};

// Canonical command coordinates; optional limits constrain every trial.
// improve_accuracy uses a tighter stopping target, truncated singular
// directions and a damped step on stalled updates, within the same budget.
auto PolishJointCandidate(const Lengths &lengths, const PoseIsoRT &target,
                          const PoseTolerance &tolerance, int budget,
                          JointCandidate &candidate,
                          const Vec6 *lower = nullptr,
                          const Vec6 *upper = nullptr,
                          bool improve_accuracy = false) -> bool;

// Requires a Ready circle for this exact target, as above.
auto PolishPreparedJointCandidate(const WristCircle &circle,
                                  const PoseIsoRT &target,
                                  const PoseTolerance &tolerance, int budget,
                                  JointCandidate &candidate,
                                  const Vec6 *lower = nullptr,
                                  const Vec6 *upper = nullptr,
                                  bool improve_accuracy = false) -> bool;

} // namespace crx::canonical
