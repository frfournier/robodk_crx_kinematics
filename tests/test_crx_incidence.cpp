#include "crx_incidence_tests.h"

#include "crx_canonical.h"
#include "crx_incidence.h"
#include "crx_incidence_reference.h"
#include "crx_types.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <limits>

#include <Eigen/Geometry>

namespace {
using crx::PoseIsoRT;
using crx::Vec3;
using namespace crx::canonical;
namespace reference = crx::test::incidence_reference;

void Require(bool condition, const char *message) {
  if (!condition) {
    std::fprintf(stderr, "CRX elbow test failed: %s\n", message);
    std::exit(EXIT_FAILURE);
  }
}

void Near(double actual, double expected, const char *message) {
  Require(std::isfinite(actual) &&
              std::abs(actual - expected) <=
                  2e-11 * std::max({1.0, std::abs(actual), std::abs(expected)}),
          message);
}

void CheckWitness(double a, double b, const Vec3 &x, const Vec3 &u,
                  const Vec3 &y) {
  Near(y.squaredNorm(), a * a, "upper arm length");
  Near((y - x).squaredNorm(), b * b, "forearm length");
  Near(y.x() * x.y() - y.y() * x.x(), 0, "base plane");
  Near((y - x).dot(u), 0, "forearm perpendicular to wrist axis");
}

void ExpectStatus(double a, double b, const Vec3 &x, const Vec3 &u,
                  IncidenceStatus expected) {
  const auto result = FindElbowCandidates(a, b, x, u);
  Require(result.status == expected && result.count == 0,
          "non-candidate status with no partial output");
}

void TestBranchesAndFallback() {
  const Vec3 x(1, 0, 0);
  const Vec3 u = Vec3::UnitY();
  const auto two = FindElbowCandidates(1, -1, x, u);
  Require(two.status == IncidenceStatus::PointCandidates && two.count == 2,
          "both coplanar elbow branches retained");
  const auto oracle = reference::FindElbowCandidates(1, -1, x, u);
  Require(oracle.status == reference::IncidenceStatus::PointCandidates &&
              oracle.count == 2,
          "SVD reference agrees on two branches");
  for (const Vec3 &point : two.points) {
    CheckWitness(1, -1, x, u, point);
    Near(point.x(), 0.5, "elbow plane");
    Near(std::abs(point.z()), std::sqrt(0.75), "elbow height");
    Require(std::min((point - oracle.points[0]).norm(),
                     (point - oracle.points[1]).norm()) < 1e-12,
            "reference elbow match");
  }
  Require((two.points[0] - two.points[1]).norm() > 1, "no duplicate branch");
  ExpectStatus(1, 1, x, Vec3::UnitZ(), IncidenceStatus::NoCandidate);
  ExpectStatus(1, 1, Vec3(3, 0, 0), u, IncidenceStatus::NoCandidate);
  ExpectStatus(1, 0.5, Vec3(0.1, 0, 0), u, IncidenceStatus::NoCandidate);
  for (const Vec3 wrist :
       {Vec3::Zero().eval(), Vec3(0, 0, 1), Vec3(1e-15, 0, 1)}) {
    ExpectStatus(1, 1, wrist, u, IncidenceStatus::NeedsRefinement);
  }
  for (const double reach : {2.0 - 1e-14, 2.0, 2.0 + 1e-14}) {
    ExpectStatus(1, 1, Vec3(reach, 0, 0), u, IncidenceStatus::NeedsRefinement);
  }
  // One compatible branch and one ambiguous branch must not publish a partial
  // list. A clearly incompatible pair must not become a false root.
  const double tilt = 4e-13;
  ExpectStatus(1, 1, x, Vec3(std::sqrt(3.0) * tilt, 1, tilt),
               IncidenceStatus::NeedsRefinement);
  ExpectStatus(1, 1, x, Vec3(1e-10, 1, 0), IncidenceStatus::NoCandidate);
  // The scalar-zero counterexample is deferred, never accepted. Its detailed
  // inconsistency classification belongs only to the offline SVD reference.
  const Vec3 vertical(0, 0, 2);
  ExpectStatus(1, 1, vertical, Vec3::UnitZ(), IncidenceStatus::NeedsRefinement);
  Require(
      reference::FindElbowCandidates(1, 1, vertical, Vec3::UnitZ()).status ==
          reference::IncidenceStatus::NoCandidate,
      "reference rejects inconsistent scalar zero");
}

auto Rotation(const Vec3 &axis, double angle) -> PoseIsoRT {
  PoseIsoRT pose = PoseIsoRT::Identity();
  pose.linear() = Eigen::AngleAxisd(angle, axis).toRotationMatrix();
  return pose;
}

auto Translation(const Vec3 &offset) -> PoseIsoRT {
  PoseIsoRT pose = PoseIsoRT::Identity();
  pose.translation() = offset;
  return pose;
}

void TestCanonicalWitnesses() {
  for (int sample = 1; sample <= 128; ++sample) {
    const double t = 0.15 + static_cast<double>(sample) / 23;
    const Lengths lengths{sample % 2 == 0 ? 2.0 : -2.0,
                          sample % 3 == 0 ? -1.5 : 1.5, 0.25, 0.375};
    PoseIsoRT fixed = PoseIsoRT::Identity();
    fixed.linear() << 0, 0, 1, 0, -1, 0, 1, 0, 0;
    const PoseIsoRT shoulder =
        Rotation(Vec3::UnitZ(), t) * Rotation(Vec3::UnitY(), t / 2);
    const PoseIsoRT elbow = shoulder * Translation(Vec3(0, 0, lengths.a)) *
                            Rotation(Vec3::UnitY(), -t / 3) *
                            Rotation(Vec3::UnitX(), -t / 5);
    const PoseIsoRT flange =
        elbow * Translation(Vec3(lengths.b, -lengths.r, 0)) *
        Rotation(Vec3::UnitY(), -t / 7) * Translation(Vec3(lengths.c, 0, 0)) *
        Rotation(Vec3::UnitX(), -t / 11) * fixed;
    WristCircle circle;
    Require(PrepareWristCircle(lengths, flange, circle) ==
                PreparationStatus::Ready,
            "prepare independent canonical FK witness");
    const Vec3 axis = elbow.linear().col(1);
    const double angle = std::atan2(axis.dot(circle.w), axis.dot(circle.v));
    const Vec3 u = circle.v * std::cos(angle) + circle.w * std::sin(angle);
    const Vec3 x = circle.p + circle.lengths.r * u;
    const auto result =
        FindElbowCandidates(circle.lengths.a, circle.lengths.b, x, u);
    Require(result.status == IncidenceStatus::PointCandidates &&
                result.count == 1,
            "all 128 generating elbows recovered without exception");
    const Vec3 expected = elbow.translation() / circle.length_scale;
    Near((result.points[0] - expected).norm(), 0,
         "independent FK elbow recovered");
    CheckWitness(circle.lengths.a, circle.lengths.b, x, u, result.points[0]);
    const auto oracle = reference::FindElbowCandidates(circle.lengths.a,
                                                       circle.lengths.b, x, u);
    if (sample == 55 &&
        oracle.status == reference::IncidenceStatus::UnresolvedBoundary) {
      continue; // Known SVD conditioning case; direct path must still recover
                // it.
    }
    Require(oracle.status == reference::IncidenceStatus::PointCandidates &&
                oracle.count == 1,
            "SVD reference regular witness");
    Near((result.points[0] - oracle.points[0]).norm(), 0,
         "direct/SVD agreement");
  }
}

void TestInvalidAndRange() {
  const Vec3 x(1, 0, 0);
  const Vec3 u = Vec3::UnitY();
  const double infinity = std::numeric_limits<double>::infinity();
  const double nan = std::numeric_limits<double>::quiet_NaN();
  for (const double a : {0.0, infinity, nan}) {
    ExpectStatus(a, 1, x, u, IncidenceStatus::InvalidInput);
  }
  ExpectStatus(1, 0, x, u, IncidenceStatus::InvalidInput);
  ExpectStatus(1, 1, Vec3(nan, 0, 0), u, IncidenceStatus::InvalidInput);
  for (const Vec3 invalid :
       {Vec3::Zero().eval(), Vec3(0, 2, 0), Vec3(nan, 0, 1)}) {
    ExpectStatus(1, 1, x, invalid, IncidenceStatus::InvalidInput);
  }
  for (const double a : {1e-200, 1e200}) {
    ExpectStatus(a, 1, x, u, IncidenceStatus::NumericalRangeFailure);
  }
  for (const double distance : {1e-200, 1e200}) {
    ExpectStatus(1, 1, Vec3(distance, 0, 0), u,
                 IncidenceStatus::NumericalRangeFailure);
  }
}
} // namespace

void RunIncidenceTests() {
  TestBranchesAndFallback();
  TestCanonicalWitnesses();
  TestInvalidAndRange();
}
