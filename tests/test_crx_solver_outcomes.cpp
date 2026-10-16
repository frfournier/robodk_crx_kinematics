#include "crx_canonical.h"
#include "crx_discovery.h"
#include "crx_joint_recovery.h"
#include "crx_joint_trig.h"
#include "crx_math_helpers.h"
#include "crx_numerics.h"
#include "crx_robodk_adapter.h"
#include "crx_solver.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <limits>

namespace {
constexpr double kPi = 3.14159265358979323846;
using crx::IkStatus;
using crx::Vec6;

void Require(bool condition, const char *message) {
  if (!condition) {
    std::fprintf(stderr, "Solver outcome: %s\n", message);
    std::exit(EXIT_FAILURE);
  }
}

auto Model() -> crx::CrxModelData {
  crx::CrxModelData model;
  model.base_transform.translation().z() = -245;
  model.joint_senses = {1, 1, -1, -1, -1, -1};
  model.dh_rows = {{{0, 0, 0, 245, false},
                    {-kPi / 2, 0, -kPi / 2, 0, false},
                    {0, 540, 0, 0, false},
                    {-kPi / 2, 0, 0, 540, false},
                    {kPi / 2, 0, 0, -150, false},
                    {-kPi / 2, 0, 0, 160, false}}};
  model.lower_limits_rad.setConstant(-2 * kPi);
  model.upper_limits_rad.setConstant(2 * kPi);
  return model;
}

auto Target(const crx::CrxModelData &model, const Vec6 &q) -> crx::PoseIsoRT {
  crx::PoseIsoRT pose;
  Require(crx::SolveFkIsometry(model, q, pose, nullptr, false) == 1,
          "construct target by FK");
  return pose;
}

void CheckJointTrig() {
  const double infinity = std::numeric_limits<double>::infinity();
  const auto check = [](const crx::JointTrig::Values &angles) {
    const crx::JointTrig trig(angles);
    for (std::size_t joint = 0; joint < trig.sine.size(); ++joint) {
      const double angle = angles.at(joint);
      Require(std::abs(trig.sine[joint] - std::sin(angle)) <=
                      8 * std::numeric_limits<double>::epsilon() &&
                  std::abs(trig.cosine[joint] - std::cos(angle)) <=
                      8 * std::numeric_limits<double>::epsilon(),
              "batched joint trig agrees with scalar reference");
      Require(std::abs(trig.sine[joint] * trig.sine[joint] +
                       trig.cosine[joint] * trig.cosine[joint] - 1) <=
                  8 * std::numeric_limits<double>::epsilon(),
              "batched joint trig stays on the unit circle");
    }
  };
  // Mix lanes and include reduction boundaries, signed zero, and large turns.
  for (const double center :
       {0.0, kPi / 2, -kPi, 2 * kPi, 15.0, 1e6, 1e13, 1e14, 1e20, 1e300}) {
    const crx::JointTrig::Values angles{center,
                                        -center,
                                        std::nextafter(center, infinity),
                                        std::nextafter(center, -infinity),
                                        .375,
                                        -1.125};
    check(angles);
  }
  // Exercise every lane over the vector interval, including both sides of
  // quadrant boundaries where the result switches approximating polynomial.
  for (int sample = -1000; sample <= 1000; ++sample) {
    crx::JointTrig::Values angles{};
    for (std::size_t joint = 0; joint < angles.size(); ++joint) {
      angles.at(joint) = .014 * sample + .031 * static_cast<double>(joint);
    }
    check(angles);
  }
  for (int quadrant = -9; quadrant <= 9; ++quadrant) {
    const double angle = quadrant * kPi / 2;
    const crx::JointTrig::Values angles{angle,
                                        -angle,
                                        std::nextafter(angle, infinity),
                                        std::nextafter(angle, -infinity),
                                        angle + 1e-10,
                                        angle - 1e-10};
    check(angles);
  }
  crx::JointTrig::Values zeros{};
  zeros[1] = -0.0;
  const crx::JointTrig trig(zeros);
  Require(!std::signbit(trig.sine[0]) && std::signbit(trig.sine[1]) &&
              trig.cosine[0] == 1 && trig.cosine[1] == 1,
          "batched joint trig preserves signed zero");
  const crx::JointTrig::Values exceptional{
      infinity, -infinity, std::numeric_limits<double>::quiet_NaN(),
      .375,     -1.125,    0.0};
  const crx::JointTrig nonfinite(exceptional);
  for (std::size_t joint = 0; joint < 3; ++joint) {
    Require(std::isnan(nonfinite.sine[joint]) &&
                std::isnan(nonfinite.cosine[joint]),
            "exceptional trig inputs preserve scalar classification");
  }
  Require(nonfinite.sine[3] == std::sin(exceptional[3]) &&
              nonfinite.cosine[4] == std::cos(exceptional[4]),
          "exceptional trig lanes do not corrupt finite lanes");
}

void CheckGeneralForwardGeometry() {
  auto model = Model();
  model.base_transform.linear() =
      Eigen::AngleAxisd(.2, crx::Vec3::UnitZ()).toRotationMatrix();
  model.tool_transform.translation() = crx::Vec3(3, -2, 1);
  Vec6 q;
  q << .4, -.7, .8, .6, -.9, -1.2;
  for (std::size_t i = 0; i < model.dh_rows.size(); ++i) {
    const double index = static_cast<double>(i + 1);
    model.dh_rows[i] = {.13 * index, 10 * index, -.22 * index, 7 * index,
                        i == 4};
  }
  for (const double offset : {0.0, 3.0}) {
    model.dh_rows[4].d += offset;
    crx::PoseIsoRT actual;
    crx::JointPoseBuffer joints;
    Require(crx::SolveFkIsometry(model, q, actual, &joints, false) == 1,
            "general DH with a prismatic joint supports FK");
    crx::PoseIsoRT expected = model.base_transform;
    Require(joints[0].matrix().isApprox(expected.matrix(), 1e-12),
            "joint buffer starts at the model base");
    for (std::size_t i = 0; i < model.dh_rows.size(); ++i) {
      const auto &row = model.dh_rows[i];
      double motion = q[static_cast<Eigen::Index>(i)] * model.joint_senses[i];
      if (i == 2) {
        motion -= q[1] * model.joint_senses[1];
      }
      const double theta = row.theta0_rad + (row.is_prismatic ? 0 : motion);
      const double d = row.d + (row.is_prismatic ? motion : 0);
      expected = expected * Eigen::Translation3d(row.a, 0, 0) *
                 Eigen::AngleAxisd(row.alpha_rad, crx::Vec3::UnitX()) *
                 Eigen::Translation3d(0, 0, d) *
                 Eigen::AngleAxisd(theta, crx::Vec3::UnitZ());
      Require((joints[i + 1].matrix() - expected.matrix()).norm() < 1e-10,
              "every joint frame matches independent modified-DH factors");
    }
    expected = expected * model.tool_transform;
    Require((actual.matrix() - expected.matrix()).norm() < 1e-10,
            "FK uses this call's geometry and tool transform");
  }
}

void CheckMappings() {
  for (const auto status : {IkStatus::StrictCandidates, IkStatus::StrictSeed,
                            IkStatus::StrictLocal, IkStatus::ApproximateSeed}) {
    crx::IkResult result;
    result.status = status;
    Require(crx::IkResultToRoboDkReturn(result) == -1,
            "empty success rejected");
    result.solutions.push_back(Vec6::Zero());
    Require(crx::IkResultToRoboDkReturn(result) == 1, "success count");
    result.truncated = true;
    Require(crx::IkResultToRoboDkReturn(result) == 1,
            "truncation remains success");
  }
  for (const auto status :
       {IkStatus::NoCandidate, IkStatus::NoOutputRequested,
        IkStatus::InvalidInput, IkStatus::UnsupportedModel,
        IkStatus::Unresolved, IkStatus::NumericalRangeFailure,
        IkStatus::WorkLimit}) {
    crx::IkResult result;
    result.status = status;
    const int expected =
        status == IkStatus::NoCandidate || status == IkStatus::NoOutputRequested
            ? 0
            : -1;
    Require(crx::IkResultToRoboDkReturn(result) == expected,
            "failure/empty mapping");
  }
}

void CheckSolves() {
  auto model = Model();
  Vec6 q;
  q << .4, -.7, .8, .6, -.9, -1.2;
  const auto target = Target(model, q);
  const auto full = crx::SolveIkIsometry(model, target, &q, 4096);
  Require(full.status == IkStatus::StrictCandidates && !full.truncated &&
              !full.used_singular_representatives && full.solutions.size() > 1,
          "strict finite commands");
  const auto limited = crx::SolveIkIsometry(model, target, &q, 1);
  Require(limited.status == IkStatus::StrictSeed && !limited.truncated &&
              limited.solutions.size() == 1 &&
              limited.solutions[0].isApprox(q, crx::numerics::kEpsilon),
          "single-output exact seed returns without discovery");
  const Vec6 nearby = q.array() + .005;
  const auto tracked = crx::SolveIkIsometry(model, target, &nearby, 1);
  Require(tracked.status == IkStatus::StrictLocal && !tracked.truncated &&
              tracked.solutions.size() == 1 &&
              (Target(model, tracked.solutions[0]).matrix() - target.matrix())
                      .norm() < crx::numerics::kSolutionPositionToleranceMm,
          "nearby seed follows an algebraic branch with strict FK");
  const auto ranked = crx::SolveIkIsometry(model, target, &q, 2);
  Require(ranked.status == IkStatus::StrictCandidates && ranked.truncated &&
              ranked.solutions.size() == 2 &&
              ranked.solutions[0].isApprox(full.solutions[0],
                                           crx::numerics::kEpsilon),
          "multiple outputs keep ranked prefix and truncation metadata");
  model.lower_limits_rad.setConstant(-100 * kPi);
  model.upper_limits_rad.setConstant(100 * kPi);
  const auto work = crx::SolveIkIsometry(model, target, &q, 2);
  Require(work.status == IkStatus::WorkLimit && work.solutions.empty(),
          "work failure does not publish a seed or partial results");
  model.lower_limits_rad.setConstant(-1e20);
  model.upper_limits_rad.setConstant(1e20);
  const auto range = crx::SolveIkIsometry(model, target, &q, 2);
  Require(range.status == IkStatus::NumericalRangeFailure &&
              range.solutions.empty(),
          "command integer range failure is distinct from work exhaustion");
}

void CheckBoundaryStatusMetadata() {
  const auto model = Model();
  Vec6 seed;
  seed << 0, kPi / 2, 0, 0, 0, 0;
  for (const double delta : {0.00005, 0.005}) {
    auto target = Target(model, seed);
    target.translation().x() += delta;
    const auto result = crx::SolveIkIsometry(model, target, &seed, 64);
    Require(
        result.status == (delta < crx::numerics::kSolutionPositionToleranceMm
                              ? IkStatus::StrictCandidates
                              : IkStatus::Unresolved),
        "boundary correction must not mask unresolved discovery with a seed");
  }
}

void CheckSingularRepresentatives() {
  const auto model = Model();
  Vec6 seed;
  seed << 0, 0, kPi / 2, 0, 0, 0;
  const auto target = Target(model, seed);
  const auto result = crx::SolveIkIsometry(model, target, nullptr, 4096);
  Require(result.status == IkStatus::StrictCandidates &&
              !result.solutions.empty(),
          "vertical arm representatives are supported");
  Require(result.used_singular_representatives,
          "canonical free-base policy propagated");
}

void CheckPhaseDomains() {
  const auto model = Model();
  Vec6 user;
  user << .4, -.7, .8, .6, -.9, -1.2;
  const auto target = Target(model, user);
  const auto command = crx::ApplyFanucJointCoupling(user);
  const crx::canonical::Lengths lengths{540, 540, 160, 150};
  const crx::canonical::PoseTolerance tolerance{
      crx::numerics::kSolutionPositionToleranceMm,
      crx::numerics::kSolutionAngleToleranceRad};
  const auto full =
      crx::canonical::DiscoverJointCandidates(lengths, target, tolerance);
  Require(full.status == crx::canonical::RecoveryStatus::Candidates,
          "domain reference discovers regular target");
  for (const int axis : {2, 5}) {
    for (const double turns : {-2.0, 0.0, 2.0}) {
      Vec6 lower = Vec6::Constant(-2 * kPi), upper = -lower;
      lower[axis] = command[axis] + turns * 2 * kPi - .01;
      upper[axis] = command[axis] + turns * 2 * kPi + .01;
      crx::canonical::DiscoveryOptions options;
      options.lower_limits = &lower;
      options.upper_limits = &upper;
      const auto filtered = crx::canonical::DiscoverJointCandidates(
          lengths, target, tolerance, options);
      Require(filtered.status == crx::canonical::RecoveryStatus::Candidates &&
                  filtered.pruned_nominations > 0 &&
                  filtered.count < full.count,
              "each restricted J3/J6 domain prunes geometric nominations");
      for (std::size_t i = 0; i < full.count; ++i) {
        const auto &expected = full.candidates[i].joints;
        const double delta =
            std::remainder(expected[axis] - command[axis], 2 * kPi);
        if (std::abs(delta) > .01) {
          continue;
        }
        bool found = false;
        for (std::size_t j = 0; j < filtered.count; ++j) {
          const crx::Vec6 difference =
              (expected - filtered.candidates[j].joints)
                  .unaryExpr([](double angle) {
                    return std::remainder(angle, 2 * kPi);
                  });
          found = found || difference.isZero(1e-9);
        }
        Require(found, "domain sieve retains every feasible reference posture");
      }
    }
  }
  crx::canonical::DiscoveryOptions invalid;
  invalid.lower_limits = &command;
  Require(crx::canonical::DiscoverJointCandidates(lengths, target, tolerance,
                                                  invalid)
                  .status == crx::canonical::RecoveryStatus::InvalidInput,
          "partial domain box is invalid");
}
} // namespace

auto main() -> int {
  CheckJointTrig();
  CheckGeneralForwardGeometry();
  CheckMappings();
  CheckSolves();
  CheckBoundaryStatusMetadata();
  CheckSingularRepresentatives();
  CheckPhaseDomains();
  std::puts("Solver outcome and adapter checks passed");
  return EXIT_SUCCESS;
}
