#include "crx_joint_recovery_tests.h"

#include "crx_canonical.h"
#include "crx_joint_recovery.h"
#include "crx_types.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <cstdlib>
#include <limits>

#include <Eigen/Geometry>

namespace {
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
// recurrences and does not call this test reference or legacy production FK.
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

void CheckCandidates(const Lengths &lengths, const PoseIsoRT &target,
                     const JointRecovery &result,
                     const PoseTolerance &tolerance) {
  Require(result.status == RecoveryStatus::Candidates && result.count >= 2 &&
              result.count <= 4,
          "finite posture candidates");
  const double scale = std::max({std::abs(lengths.a), std::abs(lengths.b),
                                 std::abs(lengths.c), std::abs(lengths.r)});
  for (std::size_t i = 0; i < result.count; ++i) {
    const auto &candidate = result.candidates[i];
    Require(candidate.joints.allFinite() &&
                candidate.joints.cwiseAbs().maxCoeff() <= kPi,
            "finite principal joint angles");
    Require(candidate.position_error <= tolerance.position &&
                candidate.orientation_error <= tolerance.orientation,
            "reported full-pose residuals");
    const auto independently_rebuilt = Forward(lengths, candidate.joints).pose;
    Require(
        ((independently_rebuilt.translation() - target.translation()) / scale)
                .norm() < 1e-10,
        "independent full translation check");
    Require((independently_rebuilt.linear() - target.linear()).norm() < 1e-10,
            "independent full orientation check");
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
  for (const double horizontal : {0.0, 2.0}) {
    target.translation().x() = horizontal;
    const auto deferred = RecoverJointCandidates(lengths, target, 0, tolerance);
    Require(deferred.status == RecoveryStatus::NeedsRefinement &&
                deferred.count == 0,
            "vertical/tangent handoff retained");
  }
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
} // namespace

void RunJointRecoveryTests() {
  TestGeneratedAndZeroWristSine();
  TestFourPosturesAndHandoffs();
  TestInvalidAndTightTolerance();
}
