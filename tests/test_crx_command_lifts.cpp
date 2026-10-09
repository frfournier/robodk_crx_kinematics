#include "crx_command_lifts.h"
#include "crx_math_helpers.h"
#include "crx_vector_helpers.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <random>

namespace {
using crx::CommandSelectionStatus;
using crx::Vec6;
constexpr double kTau = 6.283185307179586476925286766559;

void Require(bool test, const char *message) {
  if (!test) {
    std::fprintf(stderr, "Command selection: %s\n", message);
    std::exit(EXIT_FAILURE);
  }
}

auto Accept(const Vec6 &) -> bool { return true; }

void ExhaustiveReference() {
  std::mt19937 generator(17019);
  std::uniform_real_distribution<double> phase_value(-2.5, 2.5);
  std::uniform_real_distribution<double> seed_value(-12.0, 12.0);
  for (int trial = 0; trial < 200; ++trial) {
    Vec6 phase, seed, lo, hi;
    for (int axis = 0; axis < 6; ++axis) {
      phase[axis] = phase_value(generator);
      seed[axis] = seed_value(generator);
      lo[axis] = axis < 3 ? -9.0 : -3.0;
      hi[axis] = axis < 3 ? 10.0 : 3.0;
    }
    // Exhaustive bounded integer oracle: test commands directly, without
    // deriving the implementation's ceil/floor intervals.
    std::vector<Vec6> expected;
    for (int a = -2; a <= 2; ++a) {
      for (int b = -2; b <= 2; ++b) {
        for (int c = -2; c <= 2; ++c) {
          Vec6 q = phase;
          q[0] += kTau * a;
          q[1] += kTau * b;
          q[2] += kTau * c;
          if ((q.array() >= lo.array()).all() &&
              (q.array() <= hi.array()).all()) {
            expected.push_back(q);
          }
        }
      }
    }
    const bool seeded = trial % 2 == 0;
    std::sort(
        expected.begin(), expected.end(), [&](const Vec6 &a, const Vec6 &b) {
          if (seeded && (a - seed).squaredNorm() != (b - seed).squaredNorm()) {
            return (a - seed).squaredNorm() < (b - seed).squaredNorm();
          }
          return std::lexicographical_compare(a.data(), a.data() + 6, b.data(),
                                              b.data() + 6);
        });
    for (int capacity : {1, 7, 64}) {
      const auto actual = crx::SelectCommandLifts(
          {phase, phase + Vec6::Constant(kTau)}, lo, hi,
          seeded ? &seed : nullptr, capacity, [&](const Vec6 &q) {
            // This synthetic target consists of precisely one periodic posture.
            return (q - phase)
                       .unaryExpr(
                           [](double x) { return std::remainder(x, kTau); })
                       .cwiseAbs()
                       .maxCoeff() < 1e-10;
          });
      Require(actual.status == CommandSelectionStatus::Complete,
              "reference status");
      Require(actual.geometric_postures == 1 && actual.feasible_postures == 1,
              "phase deduplication");
      Require(actual.valid_lifts == expected.size(), "all lifts counted");
      Require(actual.commands.size() ==
                  std::min(expected.size(), static_cast<std::size_t>(capacity)),
              "capacity");
      Require(actual.truncated ==
                  (expected.size() > static_cast<std::size_t>(capacity)),
              "truncation metadata");
      for (std::size_t i = 0; i < actual.commands.size(); ++i) {
        Require((actual.commands[i].command_rad - expected[i])
                        .cwiseAbs()
                        .maxCoeff() < 1e-11,
                "finite reference ranking");
      }
    }
  }
}

void BoundariesAndFailures() {
  Vec6 phase = Vec6::Zero();
  phase[5] = -angle_conv::DegToRad(170.0);
  Vec6 lo = Vec6::Constant(-0.1), hi = Vec6::Constant(0.1);
  lo[5] = angle_conv::DegToRad(180.0);
  hi[5] = angle_conv::DegToRad(225.0);
  auto result = crx::SelectCommandLifts({phase}, lo, hi, nullptr, 64, Accept);
  Require(result.commands.size() == 1 && result.commands[0].turns[5] == 1,
          "shifted lift");
  Require(std::abs(result.commands[0].command_rad[5] -
                   angle_conv::DegToRad(190.0)) < 1e-12,
          "-170 to 190");

  phase[5] = angle_conv::kPi;
  lo[5] = -angle_conv::kPi;
  hi[5] = angle_conv::kPi;
  Vec6 seed = Vec6::Zero();
  result =
      crx::SelectCommandLifts({phase}, lo, hi, &seed, 64, [](const Vec6 &q) {
        return std::abs(std::abs(q[5]) - angle_conv::kPi) < 1e-12;
      });
  Require(result.commands.size() == 2, "both midpoint ties");
  Require(result.commands[0].command_rad[5] < 0,
          "lower command breaks exact tie");
  seed[5] = angle_conv::kPi;
  result = crx::SelectCommandLifts({phase}, lo, hi, &seed, 64, Accept);
  Require(result.commands.size() == 2 && result.commands[0].preserved_seed &&
              result.commands[0].turns[5] == 1,
          "seed turn metadata uses the same [-pi,pi) phase convention");

  result =
      crx::SelectCommandLifts({Vec6::Zero()}, Vec6::Constant(-100.0),
                              Vec6::Constant(100.0), nullptr, 1, Accept, 100);
  Require(result.status == CommandSelectionStatus::WorkLimit &&
              result.commands.empty(),
          "budget has no partial results");
  result = crx::SelectCommandLifts({Vec6::Zero()}, Vec6::Constant(-1e10),
                                   Vec6::Constant(1e10), nullptr, 1, Accept);
  Require(result.status == CommandSelectionStatus::RangeFailure &&
              result.commands.empty(),
          "checked count overflow");
  result = crx::SelectCommandLifts({Vec6::Zero()}, Vec6::Constant(-1e300),
                                   Vec6::Constant(1e300), nullptr, 1, Accept);
  Require(result.status == CommandSelectionStatus::RangeFailure &&
              result.commands.empty(),
          "checked integer conversion");
  // An empty early axis must not hide an unrepresentable later turn bound.
  Vec6 mixed_lower = Vec6::Zero(), mixed_upper = Vec6::Zero();
  mixed_lower[0] = mixed_upper[0] = 0.5;
  mixed_lower[5] = mixed_upper[5] = 1e300;
  result = crx::SelectCommandLifts({Vec6::Zero()}, mixed_lower, mixed_upper,
                                   nullptr, 1, Accept);
  Require(result.status == CommandSelectionStatus::RangeFailure &&
              result.commands.empty(),
          "all axes checked before accepting an empty lift box");
  mixed_lower[5] = mixed_upper[5] = 0.0;
  result = crx::SelectCommandLifts({Vec6::Zero()}, mixed_lower, mixed_upper,
                                   nullptr, 1, Accept);
  Require(result.status == CommandSelectionStatus::Complete &&
              result.commands.empty() && result.box_lifts == 0,
          "finite empty lift box");
  seed.setConstant(1e300);
  result = crx::SelectCommandLifts({Vec6::Zero()}, Vec6::Constant(-1.0),
                                   Vec6::Constant(1.0), &seed, 1, Accept);
  Require(result.status == CommandSelectionStatus::RangeFailure &&
              result.commands.empty(),
          "travel overflow");
  seed[0] = std::numeric_limits<double>::quiet_NaN();
  result = crx::SelectCommandLifts({Vec6::Zero()}, lo, hi, &seed, 1, Accept);
  Require(result.status == CommandSelectionStatus::InvalidInput,
          "nonfinite seed");

  Vec6 boundary = Vec6::Zero();
  boundary[0] = std::nextafter(1.0, 2.0);
  Require(crx::ClampCommandToLimits(boundary, Vec6::Zero(), Vec6::Ones()),
          "roundoff clamp");
  Require(boundary[0] == 1.0, "clamped boundary");
  boundary[0] = 1.00001;
  Require(!crx::ClampCommandToLimits(boundary, Vec6::Zero(), Vec6::Ones()),
          "no physical tolerance expansion");
}

void LateCandidatesAndSeed() {
  std::vector<Vec6> phases(80, Vec6::Constant(0.25));
  phases.insert(phases.end(), 80,
                Vec6::Constant(std::numeric_limits<double>::quiet_NaN()));
  for (int i = 0; i < 40; ++i) {
    Vec6 q = Vec6::Zero();
    q[0] = 0.01 * i;
    phases.push_back(q);
  }
  Vec6 seed = Vec6::Zero();
  seed[0] = 0.39;
  seed[5] = 0.01; // Not a target witness: selection cannot insert it.
  auto result = crx::SelectCommandLifts(
      phases, Vec6::Constant(-1.0), Vec6::Constant(1.0), &seed, 1,
      [](const Vec6 &q) { return q[5] == 0.0; });
  Require(result.valid_lifts == 40 && result.commands.size() == 1,
          "late valid phases survive invalid raw hits and 32 cap");
  Require(std::abs(result.commands[0].command_rad[0] - 0.39) < 1e-12,
          "late nearest survives cap");
  seed[5] = 0.0;
  result = crx::SelectCommandLifts(phases, Vec6::Constant(-1.0),
                                   Vec6::Constant(1.0), &seed, 64,
                                   [](const Vec6 &q) { return q[5] == 0.0; });
  Require(result.valid_lifts == 40 && result.commands.size() == 40,
          "seed deduplication");
  Require(result.seed_added && result.seed_matches_discovery &&
              result.commands.front().preserved_seed,
          "seed accounted separately");
  const Vec6 command = (Vec6() << 0.0, 1.0, -1.0, 0.0, 0.0, 0.0).finished();
  Require(crx::CommandToUser(command)[2] == -2.0, "coupling");
  Require(crx::UserToCommand(crx::CommandToUser(command)) == command,
          "command mapping inverse");
}

void CoupledMetricCounterexample() {
  // G7: equal command travel becomes [[2,1],[1,1]] in decoupled J2/J3.
  // Independent rounding of the decoupled seed (.49,.49) would choose (0,0).
  Vec6 user_seed = Vec6::Zero();
  user_seed[1] = user_seed[2] = 0.49 * kTau;
  const Vec6 seed = crx::UserToCommand(user_seed);
  Vec6 lo = Vec6::Zero(), hi = Vec6::Zero();
  hi[1] = kTau;
  hi[2] = 2.0 * kTau;
  const auto result = crx::SelectCommandLifts(
      {Vec6::Zero()}, lo, hi, &seed, 10, [](const Vec6 &q) {
        const Vec6 user = crx::CommandToUser(q);
        return user[2] >= -1e-12 && user[2] <= kTau + 1e-12 &&
               q.unaryExpr([](double x) { return std::remainder(x, kTau); })
                       .cwiseAbs()
                       .maxCoeff() < 1e-12;
      });
  Require(result.commands.size() == 4, "coupled finite catalogue");
  const Vec6 best = crx::CommandToUser(result.commands.front().command_rad);
  Require(best[1] == 0.0 && best[2] == kTau, "transported command metric");
  Require(std::abs(result.commands.front().travel_squared / (kTau * kTau) -
                   0.2405) < 1e-12,
          "cross-term reference cost");
}
} // namespace

auto main() -> int {
  ExhaustiveReference();
  BoundariesAndFailures();
  LateCandidatesAndSeed();
  CoupledMetricCounterexample();
  std::puts(
      "Command lift reference, capacity, boundary and failure checks passed");
}
