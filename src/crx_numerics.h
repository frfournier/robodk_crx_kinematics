#pragma once

#include <cmath>
#include <cstdint>
#include <limits>

#include <Eigen/Core>

namespace angle_conv {
inline constexpr double kPi = 3.141592653589793238462643383279502884;
inline constexpr double kRadPerDeg = kPi / 180.0;
inline constexpr double kDegPerRad = 180.0 / kPi;

inline constexpr auto DegToRad(double degrees) -> double {
  return degrees * kRadPerDeg;
}

inline constexpr auto RadToDeg(double radians) -> double {
  return radians * kDegPerRad;
}
} // namespace angle_conv

namespace crx {
inline constexpr double kTwoPi = 2 * angle_conv::kPi;
inline constexpr double kHalfPi = angle_conv::kPi / 2;

namespace numerics {
// Expose Eigen's absolute isZero and relative isApproxToConstant predicates
// for scalars as well as vectors. Always supply a named, unit-aware tolerance;
// isApproxToConstant(0) is not an absolute near-zero test.
inline auto Scalar(double value) -> Eigen::Matrix<double, 1, 1> {
  return Eigen::Matrix<double, 1, 1>{value};
}

// Representation checks for algebraic zero structure and loss of nonzero
// values through underflow. A tolerance would silently erase small
// coefficients/lengths, including denorm_min. Numerical decisions use Eigen's
// approximate predicates instead; do not emulate classification with isZero(0).
inline auto IsRepresentedZero(double value) -> bool {
  return std::fpclassify(value) == FP_ZERO;
}

// The supported x86-64 build uses IEEE binary64 throughout. These are rounding
// scales, not physical pose tolerances or certificates of complete discovery.
static_assert(sizeof(double) == 8 && std::numeric_limits<double>::is_iec559 &&
              std::numeric_limits<double>::digits == 53);
inline constexpr double kEpsilon = std::numeric_limits<double>::epsilon();
inline constexpr double kSqrtEpsilon =
    0x1p-26; // sqrt(2^-52), exact in binary64.
inline constexpr double kFourthRootEpsilon = 0x1p-13;
// 2026-10-09 comparison checks: +/-1/32/256-eps offsets around pi retain
// expected lifts; N1017 before/after DLL status/counts are unchanged.
inline constexpr double kElementaryRoundoff = 64 * kEpsilon;
// Signed 1/64/256-eps perturbations of homogeneous rows, joint senses,
// revolute flags and diagnostic characters verify this roundoff window
// (test_crx_canonical.cpp / test_crx_numerics.py, 2026-10-09).
inline constexpr double kUnitTolerance = 128 * kEpsilon;
inline constexpr double kDirectionTolerance = 256 * kEpsilon;
// 2026-10-09, 1017-pose sweep: 64*eps loses the folded-arm 1e-4-rad
// witness; 2048*eps also loses it by admitting an unpolishable branch.
// 512*eps is a tested compromise, not a formal forward-error bound.
inline constexpr double kIncidenceResidualTolerance = 512 * kEpsilon;
// Same sweep: 512*eps loses that witness; 16384*eps loses another posture.
// Retain 4096*eps for normalized back-substitution and posture-classification
// XY guards.
inline constexpr double kGeometryTolerance = 4096 * kEpsilon;
inline constexpr double kShellRoundoff = 2 * kGeometryTolerance;
inline constexpr double kIncidenceAmbiguityFactor = 8;

// Physical compatibility contracts (#12), not numerical tuning parameters.
inline constexpr double kSolutionPositionToleranceMm = 1e-4;
inline constexpr double kSolutionAngleToleranceRad = angle_conv::DegToRad(1e-3);
inline constexpr double kApproxFallbackPositionToleranceMm = 1e-2;
inline constexpr double kApproxFallbackAngleToleranceRad =
    angle_conv::DegToRad(1e-2);

// Sensitivity experiment N1017 (2026-10-09): six assets, 25 fixtures, 768
// random, 96 stress, 64 witness and 64 boundary poses; one-at-a-time threshold
// sweeps. Details/limitations: tests/README.md. Discardable harness/raw CSV:
// build/numerics. Phase: 1e-12 added near-duplicate candidates; 1e-9 merged
// polynomial candidates. 1e-10 is the rounded overlap of the stable count
// plateaus.
inline constexpr double kJointPhaseToleranceRad = 1e-10;
// Output posture identity only: merge numerical representatives at singular
// poses within 0.0001 degree per joint. Discovery, limit slack and FK accuracy
// retain their tighter, independent tolerances.
inline constexpr double kCommandDedupToleranceRad = angle_conv::DegToRad(1e-4);
// N1017: stable counts at 1e-10..1e-8. Lower settings alter chart/handoff
// decisions; retain the conservative lower edge, with strict FK still final.
inline constexpr double kRootResidualTolerance = 1e-10;
// Real-projection nomination has a roundoff budget distinct from the
// eigensolver's complex-root backward residual. The 20261010 mixed-singular
// holdout (indices 20/27) has genuine complex pairs with projected residuals
// 1e-11: the old shared 1e-10 threshold incorrectly required recovering them.
// Premerge 9082-pose sweep (2026-10-09): sampled 512/1024/2048 epsilon
// preserve all statuses and counts; 4096 epsilon loses three scaled holdout
// solves. Retain the lower end of that plateau; raw evidence: build/premerge.
inline constexpr double kRootProjectionTolerance = 512 * kEpsilon;
// N1017: 1e-9 loses 16 PERCH/ABBES-TABLE5 witnesses; sqrt(eps) matches
// 1e-8..1e-6. A repeated-root perturbation naturally has sqrt(eps) scale.
inline constexpr double kRootSlopeTolerance = kSqrtEpsilon;
// N1017: 1e-8..1e-4 had identical outcomes. Use 64*sqrt(eps), near the
// previous rounded 1e-6; it nominates guesses, never accepts a complex root.
inline constexpr double kNearRealTolerance = 64 * kSqrtEpsilon;
// N1017: .005/.01 rad lose a near-folded witness; .05.. .2 preserve counts.
// .05 rad is a rounded, bounded neighborhood, not an accuracy tolerance.
inline constexpr double kPolishRadiusRad = .05;
// Single-output tracking only: bound scalar work and command travel before
// falling back to complete discovery. These do not relax pose acceptance.
inline constexpr double kNearbyRadiusRad = .05;
inline constexpr int kNearbyIterations = 6;
// N1017: .1..1 rad preserve counts; retain the rounded .25-rad step cap.
// Premerge 9082-pose full-pose correction sweep: .125/.25/.5 all retain strict
// solves and independent FK validity; retain .25 as the bounded work policy.
inline constexpr double kPolishMaxStepRad = .25;
// Stop full-pose correction at sqrt(epsilon) in normalized coordinates only
// after both physical pose tolerances hold. The 9082-pose premerge sweep
// (2026-10-09, build/premerge/stopping) retained every solve and valid FK at
// 1e-9/sqrt(eps)/1e-7; near-family candidate counts vary. This avoids spending
// most mixed-singularity SVD updates polishing an already acceptable posture.
inline constexpr double kPoseCorrectionTolerance = kSqrtEpsilon;
// Improve already discovered postures after deduplication. Preserve nearby
// singular-family representatives with a small joint-space trust region;
// nominations that exhausted discovery's accuracy budget retain its radius.
// These are work/accuracy targets, never relaxed FK acceptance tolerances.
inline constexpr double kAccurateCorrectionTolerance = 1e-12;
inline constexpr double kAccuracyPostureRadiusRad = 1e-4;
inline constexpr int kAccuracyIterations = 6;
// Keep neighboring simple-root neighborhoods disjoint, with a half-gap left.
inline constexpr double kNeighborRadiusFraction = .25;
// N1017 folded-arm offsets 0.. .01 rad: .001.. .1 preserve counts;
// .0001 changes one. One percent is a rounded chart-switch scale.
// Premerge 9082-pose correction sweep: .001.. .05 preserve strict solves/FK;
// near-family counts vary. Retain the existing rounded one-percent scale.
inline constexpr double kFoldedChartArmFraction = .01;
// N1017: .8.. .95 preserve counts; .99/1 change one/two catalogues.
// Require a rounded 5% norm improvement to avoid marginal balancing work.
inline constexpr double kBalanceRequiredImprovement = .95;

// Physical geometry floors are scaled further by the model's arm lengths.
inline constexpr double kReferenceLengthMm = 1.0;
inline constexpr double kGeometryLengthFloorMm =
    kGeometryTolerance * kReferenceLengthMm;
inline constexpr double kDhLengthToleranceMm = kGeometryLengthFloorMm;
// N1017 nominal assets are unchanged at 4096*eps. The DH tilt micro-check
// showed that the old 1e-5-rad window erases .005346 mm over 540 mm at
// 9.9e-6 rad. Snap only roundoff; larger off-family tilts must request IK
// fallback. This bounds a single 540-mm lever's snap displacement below 5e-10
// mm.
inline constexpr double kRightAngleSnapToleranceRad = kGeometryTolerance;
// 1700 near-axis plane constructions (horizontal/vertical ratios 1e-16..1):
// largest determinant error 5.56e-16; the normalized geometry budget suffices.
inline constexpr double kPlaneOrthonormalityTolerance = kGeometryTolerance;
// Configuration labels intentionally have a dead band, independent of IK FK
// acceptance. This is a label policy, not a test of mathematical singularity.
// N1017: .010 changes ABBES-TABLE6's classification availability; .020
// changes 11 cases. Retain the rounded 1.1% arm-length exclusion band.
inline constexpr double kConfigShoulderRadiusFraction = .011;
// N1017: zero changes 12 cases; .1 degree changes one. Existing +/- .01
// degree API boundary regressions define this compatibility dead band.
inline constexpr double kConfigWristZeroRad = angle_conv::DegToRad(.01);

// Bounded work policies; finite lists do not certify branch completeness.
// Premerge 9082-pose sweep: eight loses a scaled holdout; 16/24/32/64 solve
// every reachable target. Retain 24; higher budgets change near-family
// candidate counts but do not improve measured solve rate. Not completeness.
inline constexpr int kPolishIterations = 24;
inline constexpr int kMaxPolishIterations = 64;
inline constexpr int kPolishBacktracks = 8;
inline constexpr int kBalanceSweeps = 32;
inline constexpr int kEigenIterations = 256;
inline constexpr std::uint64_t kCommandLiftBudget = std::uint64_t{1} << 20;
// Leave a sign bit of exact-integer headroom: differences of two admitted
// turns, and their inclusive interval size, remain exactly representable.
inline constexpr double kMaxExactTurn = static_cast<double>(
    (std::uint64_t{1} << (std::numeric_limits<double>::digits - 1)) - 1);
} // namespace numerics
} // namespace crx
