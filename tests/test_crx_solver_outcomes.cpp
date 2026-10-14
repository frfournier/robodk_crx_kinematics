#include "crx_numerics.h"
#include "crx_robodk_adapter.h"
#include "crx_solver.h"

#include <cstdio>
#include <cstdlib>

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
                            IkStatus::ApproximateSeed}) {
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
  Require(limited.status == IkStatus::StrictCandidates && limited.truncated &&
              limited.solutions.size() == 1 &&
              limited.solutions[0].isApprox(full.solutions[0],
                                            crx::numerics::kEpsilon),
          "capacity keeps ranked prefix and records truncation");
  model.lower_limits_rad.setConstant(-100 * kPi);
  model.upper_limits_rad.setConstant(100 * kPi);
  const auto work = crx::SolveIkIsometry(model, target, &q, 1);
  Require(work.status == IkStatus::WorkLimit && work.solutions.empty(),
          "work failure does not publish a seed or partial results");
  model.lower_limits_rad.setConstant(-1e20);
  model.upper_limits_rad.setConstant(1e20);
  const auto range = crx::SolveIkIsometry(model, target, &q, 1);
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
} // namespace

auto main() -> int {
  CheckGeneralForwardGeometry();
  CheckMappings();
  CheckSolves();
  CheckBoundaryStatusMetadata();
  CheckSingularRepresentatives();
  std::puts("Solver outcome and adapter checks passed");
  return EXIT_SUCCESS;
}
