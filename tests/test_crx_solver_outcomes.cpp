#include "crx_numerics.h"
#include "crx_robodk_adapter.h"
#include "crx_solver.h"

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
  Require(crx::SolveIkIsometry(model, target, nullptr, 0).status ==
              IkStatus::NoOutputRequested,
          "zero capacity");
  auto invalid = target;
  invalid.translation().x() = std::numeric_limits<double>::quiet_NaN();
  Require(crx::SolveIkIsometry(model, invalid, &q, 64).status ==
              IkStatus::InvalidInput,
          "invalid target");
  auto unsupported = model;
  unsupported.dh_rows[0].a = 1;
  Require(crx::SolveIkIsometry(unsupported, target, &q, 64).status ==
              IkStatus::UnsupportedModel,
          "unsupported geometry");
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
  model = Model();
  auto far = target;
  far.translation().x() = 10000;
  const auto empty = crx::SolveIkIsometry(model, far, &q, 64);
  Require(empty.status == IkStatus::NoCandidate && empty.solutions.empty(),
          "empty numerical search");
}

void CheckEmptySearchPolicy() {
  const auto model = Model();
  Vec6 seed;
  seed << 0, kPi / 2, 0, 0, 0, 0;
  const auto target = Target(model, seed);
  for (const double delta : {0.00005, 0.005, 0.0099, 0.0101}) {
    auto moved = target;
    moved.translation().x() += delta;
    for (const bool seeded : {false, true}) {
      const auto found =
          crx::SolveIkIsometry(model, moved, seeded ? &seed : nullptr, 64);
#ifdef CRXKIN_POLYNOMIAL_COMPARISON
      const auto expected = IkStatus::Unresolved;
#else
      const auto expected = !seeded || delta > .01 ? IkStatus::NoCandidate
                            : delta < .0001        ? IkStatus::StrictSeed
                                                   : IkStatus::ApproximateSeed;
#endif
      Require(found.status == expected,
              "empty search versus unresolved seed policy");
      const bool accepted = expected == IkStatus::StrictSeed ||
                            expected == IkStatus::ApproximateSeed;
      Require(found.solutions.size() == (accepted ? 1U : 0U),
              "no partial result on handoff");
      if (accepted) {
        Require((found.solutions[0] - seed).norm() < 1e-14, "seed preserved");
      }
      Require(crx::IkResultToRoboDkReturn(found) ==
                  (expected == IkStatus::Unresolved ? -1
                   : accepted                       ? 1
                                                    : 0),
              "real solver outcome mapped by adapter");
    }
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
#ifdef CRXKIN_POLYNOMIAL_COMPARISON
  Require(result.used_singular_representatives,
          "canonical free-base policy propagated");
#endif
  for (const auto &q : result.solutions) {
    const auto pose = Target(model, q);
    Require((pose.translation() - target.translation()).norm() <= 1e-4 &&
                (pose.linear() - target.linear()).norm() <= 2.5e-5,
            "every singular representative satisfies strict full FK");
  }
}
} // namespace

auto main() -> int {
  CheckMappings();
  CheckSolves();
  CheckEmptySearchPolicy();
  CheckSingularRepresentatives();
  std::puts("Solver outcome and adapter checks passed");
  return EXIT_SUCCESS;
}
