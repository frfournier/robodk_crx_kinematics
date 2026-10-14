#include "crx_allocation_probe.h"
#include "crx_joint_recovery_tests.h"

#include "crx_canonical.h"
#include "crx_discovery.h"
#include "crx_joint_recovery.h"
#include "crx_joint_step.h"
#include "crx_numerics.h"
#include "crx_types.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <string_view>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace {
using crx::Mat3;
using crx::PoseIsoRT;
using crx::Vec3;
using crx::Vec6;
using namespace crx::canonical;
constexpr double kPi = 3.14159265358979323846;

void Require(bool condition, const char *message) {
  if (!condition) {
    std::fprintf(stderr, "Joint recovery test failed: %s\n", message);
    std::exit(EXIT_FAILURE);
  }
}

auto Rotation(const Vec3 &axis, double angle) -> PoseIsoRT {
  PoseIsoRT result = PoseIsoRT::Identity();
  result.linear() = Eigen::AngleAxisd(angle, axis).toRotationMatrix();
  return result;
}

auto Translation(const Vec3 &offset) -> PoseIsoRT {
  PoseIsoRT result = PoseIsoRT::Identity();
  result.translation() = offset;
  return result;
}

struct Witness {
  PoseIsoRT pose = PoseIsoRT::Identity();
  Vec3 axis = Vec3::Zero();
};

// Independent homogeneous transform product. Runtime FK uses rotation/vector
// recurrences and does not call this test reference or production FK.
auto Forward(const Lengths &lengths, const Vec6 &q) -> Witness {
  PoseIsoRT fixed = PoseIsoRT::Identity();
  fixed.linear() << 0, 0, 1, 0, -1, 0, 1, 0, 0;
  const PoseIsoRT elbow =
      Rotation(Vec3::UnitZ(), q[0]) * Rotation(Vec3::UnitY(), q[1]) *
      Translation(Vec3(0, 0, lengths.a)) * Rotation(Vec3::UnitY(), -q[2]) *
      Rotation(Vec3::UnitX(), -q[3]);
  Witness witness;
  witness.axis = elbow.linear().col(1);
  witness.pose = elbow * Translation(Vec3(lengths.b, -lengths.r, 0)) *
                 Rotation(Vec3::UnitY(), -q[4]) *
                 Translation(Vec3(lengths.c, 0, 0)) *
                 Rotation(Vec3::UnitX(), -q[5]) * fixed;
  return witness;
}

auto Contains(const JointRecovery &result, const Vec6 &q) -> bool {
  for (std::size_t i = 0; i < result.count; ++i) {
    bool matches = true;
    for (int joint = 0; joint < 6; ++joint) {
      matches =
          matches &&
          std::abs(std::remainder(result.candidates[i].joints[joint] - q[joint],
                                  2.0 * kPi)) < 1e-8;
    }
    if (matches) {
      return true;
    }
  }
  return false;
}

void CheckCandidatePose(const Lengths &lengths, const PoseIsoRT &target,
                        const JointCandidate &candidate,
                        const PoseTolerance &tolerance) {
  const double scale = std::max({std::abs(lengths.a), std::abs(lengths.b),
                                 std::abs(lengths.c), std::abs(lengths.r)});
  Require(candidate.joints.allFinite() &&
              candidate.joints.cwiseAbs().maxCoeff() <= kPi,
          "finite principal joint angles");
  Require(candidate.position_error <= tolerance.position &&
              candidate.orientation_error <= tolerance.orientation,
          "reported full-pose residuals");
  const auto independently_rebuilt = Forward(lengths, candidate.joints).pose;
  Require(((independently_rebuilt.translation() - target.translation()) / scale)
                  .norm() < 1e-10,
          "independent full translation check");
  Require((independently_rebuilt.linear() - target.linear()).norm() < 1e-10,
          "independent full orientation check");
}

void CheckCandidates(const Lengths &lengths, const PoseIsoRT &target,
                     const JointRecovery &result,
                     const PoseTolerance &tolerance) {
  Require(result.status == RecoveryStatus::Candidates && result.count >= 2 &&
              result.count <= 4,
          "finite posture candidates");
  for (std::size_t i = 0; i < result.count; ++i) {
    const auto &candidate = result.candidates[i];
    CheckCandidatePose(lengths, target, candidate, tolerance);
    for (std::size_t j = 0; j < i; ++j) {
      double separation = 0;
      for (int joint = 0; joint < 6; ++joint) {
        separation = std::max(
            separation,
            std::abs(std::remainder(candidate.joints[joint] -
                                        result.candidates[j].joints[joint],
                                    2.0 * kPi)));
      }
      Require(separation > 1e-8, "base branches emitted without duplicates");
    }
  }
}

void CheckGeneratingWitness(const Lengths &lengths, const Vec6 &q) {
  const auto witness = Forward(lengths, q);
  WristCircle circle;
  Require(PrepareWristCircle(lengths, witness.pose, circle) ==
              PreparationStatus::Ready,
          "prepare generated target");
  const double angle =
      std::atan2(witness.axis.dot(circle.w), witness.axis.dot(circle.v));
  const PoseTolerance tolerance{circle.length_scale * 1e-9, 1e-9};
  const auto result =
      RecoverJointCandidates(lengths, witness.pose, angle, tolerance);
  CheckCandidates(lengths, witness.pose, result, tolerance);
  Require(Contains(result, q), "generating posture recovered without seed");
  const WristPhase phase{witness.axis.dot(circle.v),
                         witness.axis.dot(circle.w)};
  const auto direct =
      RecoverJointCandidates(lengths, witness.pose, phase, tolerance);
  CheckCandidates(lengths, witness.pose, direct, tolerance);
  Require(direct.count == result.count && Contains(direct, q),
          "direct circle coordinates recover the generating posture");
  for (std::size_t i = 0; i < result.count; ++i) {
    Require(Contains(direct, result.candidates[i].joints),
            "angle and circle-coordinate recovery retain both base branches");
  }
  Vec6 flipped = q;
  flipped[0] -= kPi;
  flipped[1] = -q[1];
  flipped[2] = kPi - q[2];
  flipped[3] -= kPi;
  Require(Contains(result, flipped), "base-flipped posture recovered");
}

void TestGeneratedAndZeroWristSine() {
  for (int sample = 1; sample <= 128; ++sample) {
    const double t = 0.15 + static_cast<double>(sample) / 23;
    const Lengths lengths{sample % 2 == 0 ? 2.0 : -2.0,
                          sample % 3 == 0 ? -1.5 : 1.5, 0.25, 0.375};
    Vec6 q;
    q << t, t / 2, t / 3, t / 5, t / 7, t / 11;
    CheckGeneratingWitness(lengths, q);
  }
  // q6 remains determined for the chosen wrist-circle witness, even when
  // sin(q5)=0. There is no seed input capable of overwriting q3 or q6.
  for (const double q5 : {0.0, 1e-14, -1e-14, kPi, -kPi}) {
    Vec6 q;
    q << 0.4, -0.7, 0.8, 0.6, q5, -1.2;
    CheckGeneratingWitness({2, 1.5, 0.25, 0.375}, q);
  }
  // Exercise rotation signs and branch cuts for every recovered joint. The
  // FK reference forms independent AngleAxis/homogeneous matrix products.
  for (int joint = 0; joint < 6; ++joint) {
    for (const double angle :
         {-kPi, -kPi + 1e-12, -kPi / 2, 0.0, kPi / 2, kPi - 1e-12, kPi}) {
      Vec6 q;
      q << 0.4, -0.7, 0.8, 0.6, -0.9, -1.2;
      q[joint] = angle;
      CheckGeneratingWitness({2, 1.5, 0.25, 0.375}, q);
    }
  }
  for (const double factor : {1e-200, -1.0, 1e200}) {
    Vec6 q;
    q << 0.4, -0.7, 0.8, 0.6, -0.9, -1.2;
    CheckGeneratingWitness(
        {2 * factor, 1.5 * factor, 0.25 * factor, 0.375 * factor}, q);
  }
}

void TestFourPosturesAndHandoffs() {
  const Lengths lengths{1, 1, 0.2, 0.25};
  PoseIsoRT target = PoseIsoRT::Identity();
  target.linear() << 0, -1, 0, 1, 0, 0, 0, 0, 1;
  target.translation() = Vec3(1, -0.25, 0.2);
  const PoseTolerance tolerance{1e-9, 1e-9};
  const auto result = RecoverJointCandidates(lengths, target, 0, tolerance);
  Require(result.count == 4, "two elbows times two base branches");
  CheckCandidates(lengths, target, result, tolerance);
  // The same wrist-circle point represented through the other atan endpoint.
  const auto periodic =
      RecoverJointCandidates(lengths, target, 2 * kPi, tolerance);
  CheckCandidates(lengths, target, periodic, tolerance);
  for (std::size_t i = 0; i < result.count; ++i) {
    Require(Contains(periodic, result.candidates[i].joints),
            "periodic wrist angle");
  }
  target.translation().x() = 0;
  const auto coincident = RecoverJointCandidates(lengths, target, 0, tolerance);
  Require(coincident.used_singular_representatives,
          "coincident arm spheres use the documented representative policy");
  CheckCandidates(lengths, target, coincident, tolerance);
  target.translation().x() = 2;
  const auto tangent = RecoverJointCandidates(lengths, target, 0, tolerance);
  Require(tangent.status == RecoveryStatus::Candidates && tangent.count == 2,
          "tangent elbow retains both base branches");
  CheckCandidates(lengths, target, tangent, tolerance);
  target.translation().x() = 3;
  const auto empty = RecoverJointCandidates(lengths, target, 0, tolerance);
  Require(empty.status == RecoveryStatus::NoCandidate && empty.count == 0,
          "no elbow at supplied angle");
}

void TestInvalidAndTightTolerance() {
  const Lengths lengths{2, 1.5, 0.25, 0.375};
  Vec6 q;
  q << 0.4, -0.7, 0.8, 0.6, -0.9, -1.2;
  const auto witness = Forward(lengths, q);
  WristCircle circle;
  Require(PrepareWristCircle(lengths, witness.pose, circle) ==
              PreparationStatus::Ready,
          "prepare tolerance witness");
  const double angle =
      std::atan2(witness.axis.dot(circle.w), witness.axis.dot(circle.v));
  for (const PoseTolerance tight :
       {PoseTolerance{1e-30, 1e-6}, PoseTolerance{1e-6, 1e-30}}) {
    const auto strict =
        RecoverJointCandidates(lengths, witness.pose, angle, tight);
    Require(strict.status == RecoveryStatus::NeedsRefinement &&
                strict.count == 0,
            "position and orientation gates independently enforce tolerances");
  }
  const double nan = std::numeric_limits<double>::quiet_NaN();
  for (const WristPhase phase :
       {WristPhase{0, 0}, WristPhase{1, 1}, WristPhase{nan, 0},
        WristPhase{0, std::numeric_limits<double>::infinity()}}) {
    const auto invalid_phase =
        RecoverJointCandidates(lengths, witness.pose, phase, {1e-9, 1e-9});
    Require(invalid_phase.status == RecoveryStatus::InvalidInput &&
                invalid_phase.count == 0,
            "non-unit or nonfinite circle coordinates rejected");
  }
  for (const PoseTolerance tolerance :
       {PoseTolerance{0, 1e-9}, PoseTolerance{1e-9, -1},
        PoseTolerance{nan, 1e-9}, PoseTolerance{1e-9, kPi}}) {
    Require(RecoverJointCandidates(lengths, witness.pose, angle, tolerance)
                    .status == RecoveryStatus::InvalidInput,
            "invalid pose tolerance rejected");
  }
  Require(
      RecoverJointCandidates(lengths, witness.pose, nan, {1e-9, 1e-9}).status ==
          RecoveryStatus::InvalidInput,
      "nonfinite wrist angle rejected");
  auto invalid = witness.pose;
  invalid.linear()(0, 0) += 0.01;
  Require(
      RecoverJointCandidates(lengths, invalid, angle, {1e-9, 1e-9}).status ==
          RecoveryStatus::InvalidInput,
      "invalid target rejected");
  invalid = PoseIsoRT::Identity();
  invalid.translation().x() = 1e200;
  Require(
      RecoverJointCandidates({1, 1, 0, 0}, invalid, 0, {1e-9, 1e-9}).status ==
          RecoveryStatus::NumericalRangeFailure,
      "numerical range failure propagated");
}

auto CheckDiscoveryWitness(const Lengths &lengths, const Vec6 &q, int sample)
    -> bool {
  const auto witness = Forward(lengths, q);
  const double scale = std::max({std::abs(lengths.a), std::abs(lengths.b),
                                 std::abs(lengths.c), std::abs(lengths.r)});
  const PoseTolerance tolerance{scale * 1e-9, 1e-9};
  const auto result = DiscoverJointCandidates(lengths, witness.pose, tolerance);
  if (result.status != RecoveryStatus::Candidates) {
    std::fprintf(stderr, "Discovery sample %d: status=%d\n", sample,
                 static_cast<int>(result.status));
  }
  Require(result.status == RecoveryStatus::Candidates && result.count >= 2,
          "target-only discovery succeeds");
  bool found = false;
  for (std::size_t i = 0; i < result.count; ++i) {
    const auto &candidate = result.candidates[i];
    double error = 0;
    for (int joint = 0; joint < 6; ++joint) {
      error =
          std::max(error, std::abs(std::remainder(
                              candidate.joints[joint] - q[joint], 2 * kPi)));
    }
    found = found || error < 1e-7;
    Require(candidate.position_error <= tolerance.position &&
                candidate.orientation_error <= tolerance.orientation,
            "discovered posture meets both pose tolerances");
    const auto independent = Forward(lengths, candidate.joints).pose;
    Require(((independent.translation() - witness.pose.translation()) / scale)
                        .norm() <= tolerance.position / scale &&
                (independent.linear() - witness.pose.linear()).norm() < 2e-9,
            "independent FK of every discovered posture");
    for (std::size_t j = 0; j < i; ++j) {
      double separation = 0;
      for (int joint = 0; joint < 6; ++joint) {
        separation = std::max(
            separation,
            std::abs(std::remainder(candidate.joints[joint] -
                                        result.candidates[j].joints[joint],
                                    2 * kPi)));
      }
      Require(separation > 1e-10, "no duplicate postures modulo full turns");
    }
  }
  Require(found, "target-only discovery recovers generating posture");
  DiscoveryOptions no_polishing;
  no_polishing.max_polish_iterations = 0;
  const auto unpolished =
      DiscoverJointCandidates(lengths, witness.pose, tolerance, no_polishing);
  return unpolished.status == RecoveryStatus::NeedsRefinement;
}

void CheckRefinementRotationVector(const Vec3 &axis, double angle) {
  constexpr double kRelativeTolerance = 16 * crx::numerics::kEpsilon;
  constexpr double kMatrixRoundoffTolerance = 1e-14;
  for (const double sign : {-1.0, 1.0}) {
    const Mat3 rotation =
        Eigen::AngleAxisd(sign * angle, axis).toRotationMatrix();
    const Vec3 actual = detail::RotationVector(rotation);
    Require((actual - sign * angle * axis).cwiseAbs().maxCoeff() <=
                kRelativeTolerance * angle,
            "rotation logarithm covers zero, series/chart boundaries and pi");
    const Vec3 perturbation(1e-15, -2e-15, 3e-15);
    Mat3 rounded = rotation;
    rounded.diagonal() += perturbation;
    const Eigen::AngleAxisd reference(rounded);
    Require(
        (detail::RotationVector(rounded) - reference.angle() * reference.axis())
                .norm() < kMatrixRoundoffTolerance,
        "rotation logarithm tolerates matrix roundoff");
  }
}

void TestRefinementRotationVector() {
  const std::array<Vec3, 4> axes{Vec3::UnitX(), Vec3::UnitY(), Vec3::UnitZ(),
                                 Vec3(1, -2, 3).normalized()};
  const double boundary = 2 * std::atan(crx::numerics::kFourthRootEpsilon);
  const auto allocations = crx::test::AllocationCount();
  const bool previous = Eigen::internal::is_malloc_allowed();
  Eigen::internal::set_is_malloc_allowed(false);
  for (const auto &axis : axes) {
    for (const double angle :
         {0.0, std::numeric_limits<double>::denorm_min(),
          std::numeric_limits<double>::min(), 1e-300, 1e-18, 1e-12,
          boundary * (1 - 1e-10), boundary, boundary * (1 + 1e-10), .1, kPi / 2,
          (2 * kPi / 3) - 1e-12, (2 * kPi / 3) + 1e-12, kPi - 1e-8, kPi}) {
      CheckRefinementRotationVector(axis, angle);
    }
  }
  Eigen::internal::set_is_malloc_allowed(previous);
  Require(allocations == crx::test::AllocationCount(),
          "rotation logarithm uses fixed storage");
}

void TestMinimumNormJointStep() {
  using Matrix6 = Eigen::Matrix<double, 6, 6>;
  Matrix6 left = Matrix6::Identity(), right = Matrix6::Identity();
  for (int i = 0; i < 5; ++i) {
    for (int j = i + 1; j < 6; ++j) {
      const double angle = .2 * (i + j + 1);
      left.applyOnTheLeft(
          i, j,
          Eigen::JacobiRotation<double>(std::cos(angle), std::sin(angle)));
      right.applyOnTheRight(i, j,
                            Eigen::JacobiRotation<double>(std::cos(2 * angle),
                                                          std::sin(2 * angle)));
    }
  }
  Vec6 error;
  error << .2, -.3, .5, -.7, 1.1, -1.3;
  for (int rank = 0; rank <= 6; ++rank) {
    Vec6 singular = Vec6::Zero(), reciprocal = Vec6::Zero();
    for (int i = 0; i < rank; ++i) {
      singular[i] = 1.0 / (i + 1);
      reciprocal[i] = i + 1;
    }
    const Matrix6 jacobian = left * singular.asDiagonal() * right.transpose();
    const Vec6 expected =
        right * reciprocal.asDiagonal() * left.transpose() * error;
    for (const bool accuracy : {false, true}) {
      const auto allocations = crx::test::AllocationCount();
      const bool previous = Eigen::internal::is_malloc_allowed();
      Eigen::internal::set_is_malloc_allowed(false);
      const Vec6 actual =
          detail::SolveMinimumNormStep(jacobian, error, accuracy);
      Eigen::internal::set_is_malloc_allowed(previous);
      Require(allocations == crx::test::AllocationCount() && actual.allFinite(),
              "minimum-norm step uses fixed storage at every rank");
      Require((actual - expected).norm() < 1e-12,
              "rank-revealing step matches known orthogonal factors");
    }
  }
  // A resolvable but nearly null direction is retained for geometric recovery
  // and deliberately truncated for local accuracy refinement.
  Matrix6 diagonal = Matrix6::Identity();
  diagonal(5, 5) = 1e-10;
  const Vec6 accurate = detail::SolveMinimumNormStep(diagonal, error, true);
  const Vec6 geometric = detail::SolveMinimumNormStep(diagonal, error, false);
  Require((accurate.head<5>() - error.head<5>()).norm() < 1e-12 &&
              accurate[5] == 0 && (diagonal * geometric - error).norm() < 1e-12,
          "accuracy truncation leaves observable directions unchanged");
}

void CheckRefinedBasePair(const Lengths &lengths, const Vec6 &q) {
  const auto witness = Forward(lengths, q);
  WristCircle circle;
  Require(PrepareWristCircle(lengths, witness.pose, circle) ==
              PreparationStatus::Ready,
          "prepare unequal-arm folded target");
  const double angle =
      std::atan2(witness.axis.dot(circle.w), witness.axis.dot(circle.v));
  const PoseTolerance tolerance{circle.length_scale *
                                    crx::numerics::kGeometryTolerance,
                                crx::numerics::kGeometryTolerance};
  JointRefiner refiner(lengths, witness.pose, tolerance,
                       crx::numerics::kPolishIterations);
  const auto allocations = crx::test::AllocationCount();
  const bool previous = Eigen::internal::is_malloc_allowed();
  Eigen::internal::set_is_malloc_allowed(false);
  const auto result =
      refiner.Refine(angle, crx::numerics::kPolishRadiusRad, false);
  Eigen::internal::set_is_malloc_allowed(previous);
  Require(allocations == crx::test::AllocationCount(),
          "refined base pair allocates no storage");
  constexpr std::size_t max_postures = 4;
  Require(result.status == RecoveryStatus::Candidates && result.count >= 2 &&
              result.count <= max_postures,
          "refined base pair contains finite postures");
  // Different elbow nominations can converge to the same corrected posture;
  // discovery deduplicates them after this local refinement stage.
  for (std::size_t i = 0; i < result.count; ++i) {
    CheckCandidatePose(lengths, witness.pose, result.candidates[i], tolerance);
  }
  Require(Contains(result, q), "refined generating posture retained");
  Vec6 flipped = q;
  flipped[0] -= kPi;
  flipped[1] = -q[1];
  flipped[2] = kPi - q[2];
  constexpr int wrist_roll = 3;
  flipped[wrist_roll] -= kPi;
  Require(Contains(result, flipped), "refined opposite base posture retained");
}

void TestRefinedBasePairs() {
  struct Sample {
    double upper, wrist, offset;
  };

  constexpr std::array samples{
      Sample{1.5, -.375, -.001}, Sample{1.5, -.375, .001},
      Sample{1.5, .375, -.001},  Sample{1.5, .375, .001},
      Sample{2.5, -.375, -.001}, Sample{2.5, -.375, .001},
      Sample{2.5, .375, -.001},  Sample{2.5, .375, .001}};
  const Vec6 generating =
      (Vec6() << .4, -.7, -kPi / 2, .6, -.9, -1.2).finished();
  constexpr double tool_length = .25;
  for (const double scale : {1e-3, 1.0, 1e3}) {
    for (const auto &sample : samples) {
      Vec6 q = generating;
      q[2] += sample.offset;
      CheckRefinedBasePair({sample.upper * scale, 2 * scale,
                            tool_length * scale, sample.wrist * scale},
                           q);
    }
  }
}

void TestFoldedNominationWindows() {
  for (const double scale : {1e-3, 1.0, 1e3}) {
    const Lengths lengths{2 * scale, 2 * scale, .25 * scale, .375 * scale};
    Vec6 q;
    q << .4, -.7, -kPi / 2, .6, -.9, -1.2;
    const auto target = Forward(lengths, q).pose;
    const PoseTolerance tolerance{scale * 1e-8, 1e-8};
    WristCircle circle;
    Require(PrepareWristCircle(lengths, target, circle) ==
                PreparationStatus::Ready,
            "folded target prepares");
    const double closest =
        std::atan2(circle.p.dot(circle.w), circle.p.dot(circle.v)) + kPi;
    JointRefiner refiner(lengths, target, tolerance,
                         crx::numerics::kPolishIterations);
    const auto allocations = crx::test::AllocationCount();
    const bool previous = Eigen::internal::is_malloc_allowed();
    Eigen::internal::set_is_malloc_allowed(false);
    const auto excluded = refiner.Refine(closest + .001, 1e-5);
    const auto admitted = refiner.Refine(closest, .05);
    const auto excluded_again = refiner.Refine(closest - .001, 1e-5);
    Eigen::internal::set_is_malloc_allowed(previous);
    Require(allocations == crx::test::AllocationCount(),
            "repeated folded nominations allocate no storage");
    Require(excluded.status == RecoveryStatus::NeedsRefinement &&
                excluded.count == 0 && excluded_again.count == 0,
            "each folded nomination enforces its own wrist window");
    Require(admitted.status == RecoveryStatus::Candidates &&
                admitted.count > 0 && admitted.used_singular_representatives,
            "a rejected window does not discard later folded representatives");
    for (std::size_t i = 0; i < admitted.count; ++i) {
      const auto actual = Forward(lengths, admitted.candidates[i].joints).pose;
      Require((actual.translation() - target.translation()).norm() <=
                      tolerance.position &&
                  (actual.linear() - target.linear()).norm() <=
                      2 * tolerance.orientation,
              "folded representative passes independent FK");
    }
  }
}

void TestBoundedPoseCorrection() {
  for (const double scale : {1e-100, 1.0, 1e100}) {
    const Lengths lengths{2 * scale, 1.5 * scale, .25 * scale, .375 * scale};
    Vec6 q;
    q << .4, -.7, .8, .6, -.9, -1.2;
    const auto target = Forward(lengths, q).pose;
    const PoseTolerance tolerance{scale * crx::numerics::kGeometryTolerance,
                                  crx::numerics::kGeometryTolerance};
    JointCandidate candidate;
    candidate.joints = q.array() + crx::numerics::kPolishMaxStepRad / 4;
    const auto allocations = crx::test::AllocationCount();
    const bool previous = Eigen::internal::is_malloc_allowed();
    Eigen::internal::set_is_malloc_allowed(false);
    const bool solved =
        PolishJointCandidate(lengths, target, tolerance,
                             crx::numerics::kPolishIterations, candidate);
    Eigen::internal::set_is_malloc_allowed(previous);
    Require(solved && allocations == crx::test::AllocationCount(),
            "bounded joint correction is allocation-free across scales");
    const auto actual = Forward(lengths, candidate.joints).pose;
    Require(((actual.translation() - target.translation()) / scale).norm() <=
                    crx::numerics::kGeometryTolerance &&
                (actual.linear() - target.linear()).norm() <=
                    2 * crx::numerics::kGeometryTolerance,
            "corrected candidate passes independent transform-product FK");
    Require(!PolishJointCandidate(lengths, target, tolerance, 0, candidate),
            "zero correction budget does no hidden work");
    JointCandidate accurate;
    accurate.joints = q.array() + 1e-5;
    const Vec6 nearby_lower = q.array() - 1e-4;
    const Vec6 nearby_upper = q.array() + 1e-4;
    Require(PolishJointCandidate(lengths, target, {scale * 1e-4, 1e-3}, 6,
                                 accurate, &nearby_lower, &nearby_upper, true),
            "local accuracy pass succeeds inside its posture window");
    const auto precise = Forward(lengths, accurate.joints).pose;
    Require(
        ((precise.translation() - target.translation()) / scale).norm() <
                1e-11 &&
            (precise.linear() - target.linear()).norm() < 1e-11 &&
            (accurate.joints - q).cwiseAbs().maxCoeff() < 1e-4,
        "accuracy target improves on loose acceptance without posture drift");
    const Vec6 excluded = q.array() + crx::numerics::kPolishMaxStepRad;
    Require(!PolishJointCandidate(lengths, target, tolerance,
                                  crx::numerics::kPolishIterations, candidate,
                                  &excluded, &excluded),
            "active bounds cannot publish an inaccurate fixed command");
  }
}

void TestSingularAccuracyCorrection() {
  for (const double scale : {1e-3, 1.0, 1e3}) {
    const Lengths lengths{2 * scale, 2 * scale, .25 * scale, .375 * scale};
    for (const double wrist : {0.0, 1e-8, .01}) {
      Vec6 q;
      q << .4, -.7, -kPi / 2, .6, wrist, -1.2;
      const auto target = Forward(lengths, q).pose;
      JointCandidate candidate;
      candidate.joints = q.array() + 1e-5;
      const Vec6 lower = candidate.joints.array() - 1e-4;
      const Vec6 upper = candidate.joints.array() + 1e-4;
      const auto allocations = crx::test::AllocationCount();
      const bool previous = Eigen::internal::is_malloc_allowed();
      Eigen::internal::set_is_malloc_allowed(false);
      const bool solved =
          PolishJointCandidate(lengths, target, {scale * 1e-4, 1e-3}, 6,
                               candidate, &lower, &upper, true);
      Eigen::internal::set_is_malloc_allowed(previous);
      Require(solved && allocations == crx::test::AllocationCount(),
              "singular accuracy correction uses fixed storage");
      const auto actual = Forward(lengths, candidate.joints).pose;
      Require(
          ((actual.translation() - target.translation()) / scale).norm() <
                  1e-11 &&
              (actual.linear() - target.linear()).norm() < 1e-11 &&
              (candidate.joints.array() >= lower.array()).all() &&
              (candidate.joints.array() <= upper.array()).all(),
          "singular refinement improves accuracy inside its posture window");
    }
  }
}

void TestTargetOnlyDiscovery() {
  int polished_cases = 0;
  for (int sample = 1; sample <= 128; ++sample) {
    const double t = 0.15 + static_cast<double>(sample) / 23;
    const Lengths lengths{sample % 2 == 0 ? 2.0 : -2.0,
                          sample % 3 == 0 ? -1.5 : 1.5, 0.25, 0.375};
    Vec6 q;
    q << t, t / 2, t / 3, t / 5, t / 7, t / 11;
    if (CheckDiscoveryWitness(lengths, q, sample)) {
      ++polished_cases;
    }
  }
  Require(polished_cases > 0,
          "geometric polishing resolves unpolished candidates");
  for (const double q6 :
       {-kPi / 2, -kPi / 2 + 1e-9, -kPi / 2 - 1e-9, kPi / 2, -1.2}) {
    Vec6 q;
    q << 0.4, -0.7, 0.8, 0.6, -0.9, q6;
    CheckDiscoveryWitness({2, 1.5, 0.25, 0.375}, q, 0);
  }
  for (const double factor : {1e-200, -1.0, 1e200}) {
    Vec6 q;
    q << 0.4, -0.7, 0.8, 0.6, -0.9, -1.2;
    CheckDiscoveryWitness(
        {2 * factor, 1.5 * factor, 0.25 * factor, 0.375 * factor}, q, -1);
  }
  Vec6 q;
  q << 0.4, -0.7, 0.8, 0.6, -0.9, -1.2;
  const Lengths lengths{2, 1.5, 0.25, 0.375};
  const auto target = Forward(lengths, q).pose;
  DiscoveryOptions options;
  options.roots.max_eigen_iterations = 0;
  const auto limited =
      DiscoverJointCandidates(lengths, target, {1e-9, 1e-9}, options);
  Require(limited.status == RecoveryStatus::NeedsRefinement &&
              limited.count == 0,
          "root work limit is not no-solution or partial success");
  options = {};
  options.max_polish_iterations = -1;
  Require(
      DiscoverJointCandidates(lengths, target, {1e-9, 1e-9}, options).status ==
          RecoveryStatus::InvalidInput,
      "invalid discovery budget");
  Require(DiscoverJointCandidates(lengths, target, {0, 1e-9}).status ==
              RecoveryStatus::InvalidInput,
          "invalid discovery tolerance");
  auto invalid = target;
  invalid.matrix()(3, 0) = 1;
  Require(DiscoverJointCandidates(lengths, invalid, {1e-9, 1e-9}).status ==
              RecoveryStatus::InvalidInput,
          "invalid discovery target");

  // Dyadic data keep the leading coefficient exactly zero: this exercises
  // the omitted point itself, not just a shifted chart around a nearby root.
  PoseIsoRT endpoint = PoseIsoRT::Identity();
  endpoint.linear() << 0, 0, -1, -1, 0, 0, 0, 1, 0;
  endpoint.translation() = Vec3(-1.25, 0.75, 1);
  const Lengths endpoint_lengths{1.25, 1, 2, 0.25};
  WristCircle circle;
  HalfAnglePolynomial polynomial;
  Require(PrepareWristCircle(endpoint_lengths, endpoint, circle) ==
                  PreparationStatus::Ready &&
              BuildHalfAnglePolynomial(circle, 0, polynomial) ==
                  PreparationStatus::Ready &&
              RepresentedDegree(polynomial.coefficients) == 7 &&
              crx::numerics::Scalar(polynomial.omitted_point_residual)
                  .isZero(crx::numerics::kEpsilon),
          "exact degree-seven chart with an omitted root");
  // End-to-end omitted-point recovery is checked by Python's
  // test_exact_omitted_point_is_not_lost through the allocation-guarded probe.
}
} // namespace

void RunJointRecoveryTests() {
  TestGeneratedAndZeroWristSine();
  TestFourPosturesAndHandoffs();
  TestInvalidAndTightTolerance();
  TestRefinementRotationVector();
  TestMinimumNormJointStep();
  TestFoldedNominationWindows();
  TestRefinedBasePairs();
  TestBoundedPoseCorrection();
  TestSingularAccuracyCorrection();
  TestTargetOnlyDiscovery();
}

auto RunDiscoveryProbe(int argc, char **argv) -> int {
  // Test-only canonical input, no model recognition or RoboDK bridge API.
  if (argc != 20) {
    return 2;
  }
  std::array<double, 18> values{};
  for (int i = 2; i < argc; ++i) {
    char *end = nullptr;
    const double value = std::strtod(argv[i], &end);
    if (end == argv[i] || *end != '\0' || !std::isfinite(value)) {
      return 2;
    }
    values[static_cast<std::size_t>(i - 2)] = value;
  }
  PoseIsoRT target = PoseIsoRT::Identity();
  for (int row = 0; row < 3; ++row) {
    for (int col = 0; col < 4; ++col) {
      target.matrix()(row, col) =
          values[static_cast<std::size_t>(4 + 4 * row + col)];
    }
  }
  const auto allocations = crx::test::AllocationCount();
  const bool previous = Eigen::internal::is_malloc_allowed();
  Eigen::internal::set_is_malloc_allowed(false);
  const auto result =
      DiscoverJointCandidates({values[0], values[1], values[2], values[3]},
                              target, {values[16], values[17]});
  Eigen::internal::set_is_malloc_allowed(previous);
  Require(crx::test::AllocationCount() == allocations,
          "discovery probe allocated");
  const char *status = "InvalidInput";
  switch (result.status) {
  case RecoveryStatus::Candidates:
    status = "Candidates";
    break;
  case RecoveryStatus::NoCandidate:
    status = "NoCandidate";
    break;
  case RecoveryStatus::NeedsRefinement:
    status = "NeedsRefinement";
    break;
  case RecoveryStatus::NumericalRangeFailure:
    status = "NumericalRangeFailure";
    break;
  case RecoveryStatus::InvalidInput:
    break;
  }
  std::printf("{\"status\":\"%s\",\"solutions\":[", status);
  for (std::size_t i = 0; i < result.count; ++i) {
    const auto &candidate = result.candidates[i];
    std::printf("%s[", i == 0 ? "" : ",");
    for (int joint = 0; joint < 6; ++joint) {
      std::printf("%s%.17g", joint == 0 ? "" : ",", candidate.joints[joint]);
    }
    std::printf(",%.17g,%.17g]", candidate.position_error,
                candidate.orientation_error);
  }
  std::printf("]");
  if (std::string_view(argv[1]) == "--diagnose-discovery") {
    WristCircle circle;
    const Lengths lengths{values[0], values[1], values[2], values[3]};
    if (PrepareWristCircle(lengths, target, circle) ==
        PreparationStatus::Ready) {
      std::printf(",\"charts\":[");
      int chart_index = 0;
      for (const double chart : {0.0, kPi / 2, kPi / 4, -kPi / 4}) {
        HalfAnglePolynomial polynomial;
        BuildHalfAnglePolynomial(circle, chart, polynomial);
        const auto roots = FindRootCandidates(polynomial.coefficients);
        std::printf("%s{\"chart\":%.17g,\"status\":%d,\"coefficients\":[",
                    chart_index++ == 0 ? "" : ",", chart,
                    static_cast<int>(roots.status));
        for (std::size_t i = 0; i < polynomial.coefficients.size(); ++i) {
          std::printf("%s%.17g", i == 0 ? "" : ",", polynomial.coefficients[i]);
        }
        std::printf("],\"roots\":[");
        for (std::size_t i = 0; i < roots.count; ++i) {
          const auto root = roots.candidates[i].value;
          const double angle = chart + 2 * std::atan(root.real());
          const auto recovery = RecoverJointCandidates(
              lengths, target, angle, {values[16], values[17]});
          const Vec3 u =
              circle.v * std::cos(angle) + circle.w * std::sin(angle);
          const Vec3 x = circle.p + circle.lengths.r * u;
          const double along =
              (circle.lengths.a * circle.lengths.a + x.squaredNorm() -
               circle.lengths.b * circle.lengths.b) /
              (2 * x.norm());
          std::printf("%s[%.17g,%.17g,%.17g,%d,%zu,%.17g,%.17g]",
                      i == 0 ? "" : ",", root.real(), root.imag(), angle,
                      static_cast<int>(recovery.status), recovery.count,
                      std::hypot(x.x(), x.y()),
                      circle.lengths.a * circle.lengths.a - along * along);
        }
        std::printf("]}");
      }
      std::printf("]");
    }
  }
  std::puts("}");
  return 0;
}
