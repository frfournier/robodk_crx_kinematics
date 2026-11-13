// SPDX-License-Identifier: Apache-2.0
// SPDX-FileCopyrightText: 2026 Centre Technologique en Aérospatiale

#include "crx_phase_catalogue.h"

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdlib>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <utility>
#include <vector>

#include <xmmintrin.h>

namespace {
using crx::catalogue::Failure;
using crx::interval::Interval;

void Require(bool condition, const char *message) {
  if (!condition) {
    std::cerr << "Catalogue test failure: " << message << '\n';
    std::exit(1);
  }
}

auto Witness() -> crx::exact::Input {
  crx::exact::Input input;
  input.lengths = {1, .6, .2, .5};
  input.target.quaternion = {60, -7, 7, -19};
  input.target.position = {27.0 / 64, 15.0 / 32, 55.0 / 128};
  input.lower_commands.fill(-4);
  input.upper_commands.fill(4);
  return input;
}

void Print(Interval value) {
  if (!value.IsValid())
    std::cout << "null";
  else
    std::cout << '[' << value.lo << ',' << value.hi << ']';
}

template <typename Values> void PrintArray(const Values &values) {
  std::cout << '[';
  bool first = true;
  for (const auto value : values) {
    if (!first)
      std::cout << ',';
    first = false;
    std::cout << value;
  }
  std::cout << ']';
}

void Print(const crx::catalogue::BuildResult &built) {
  const auto &catalogue = built.catalogue;
  const auto &data = catalogue.Values();
  const auto &roots = data.root_cover.Values();
  std::cout << "{\"complete\":" << (catalogue.IsComplete() ? "true" : "false")
            << ",\"infeasible\":"
            << (catalogue.IsInfeasible() ? "true" : "false")
            << ",\"coverage\":" << static_cast<int>(data.coverage)
            << ",\"failure\":" << static_cast<int>(data.failure)
            << ",\"root_complete\":"
            << (data.root_cover.IsComplete() ? "true" : "false")
            << ",\"root_status\":" << static_cast<int>(roots.status)
            << ",\"entire_circle_unresolved\":"
            << (roots.entire_circle_unresolved ? "true" : "false")
            << ",\"roots\":" << roots.simple_roots
            << ",\"expected\":" << data.expected_phases
            << ",\"root_nodes\":" << roots.nodes
            << ",\"root_shift\":" << roots.chart_shift
            << ",\"pair_tests\":" << data.pair_tests
            << ",\"nominations\":" << built.nominations
            << ",\"attempted_balls\":" << built.attempted_balls
            << ",\"certified_balls\":" << built.certified_balls
            << ",\"numerical_status\":" << built.numerical_status
            << ",\"candidate_budget_exhausted\":"
            << (built.candidate_budget_exhausted ? "true" : "false")
            << ",\"schema\":" << data.identity.schema << ",\"words\":[";
  for (std::size_t i = 0; i < data.identity.words.size(); ++i) {
    if (i)
      std::cout << ',';
    std::cout << '"' << std::hex << data.identity.words.at(i) << std::dec
              << '"';
  }
  std::cout << "],\"ball_failures\":[";
  for (std::size_t i = 0; i < built.ball_failures.size(); ++i) {
    if (i)
      std::cout << ',';
    std::cout << static_cast<int>(built.ball_failures.at(i));
  }
  std::cout << "],\"unresolved\":[";
  bool first = true;
  for (std::size_t chart = 0; chart < 2; ++chart) {
    for (const auto &leaf : roots.partitions.at(chart).leaves) {
      if (leaf.kind != crx::root::Kind::Unresolved)
        continue;
      if (!first)
        std::cout << ',';
      first = false;
      std::cout << "{\"chart\":" << chart
                << ",\"reason\":" << static_cast<int>(leaf.reason) << ",\"t\":";
      Print(leaf.tangent);
      std::cout << ",\"phase\":";
      Print(leaf.phase);
      std::cout << '}';
    }
  }
  std::cout << "],\"balls\":[";
  first = true;
  for (const auto &certificate : data.balls) {
    if (!first)
      std::cout << ',';
    first = false;
    const auto &ball = certificate.Values();
    Require(certificate.IsValid() && ball.identity == data.identity,
            "unsealed/mismatched output ball");
    std::cout << "{\"failure\":0,\"centre\":";
    PrintArray(ball.centre);
    std::cout << ",\"constants\":";
    PrintArray(std::array<double, 11>{
        ball.existence_radius, ball.uniqueness_radius, ball.inverse_residual,
        ball.beta, ball.lipschitz, ball.eta, ball.pose_defect,
        ball.screw_norm_sum, ball.angular_norm_sum, ball.anchor_position_norm,
        static_cast<double>(ball.aspect_sign)});
    std::cout << ",\"preconditioner\":[";
    for (int i = 0; i < 6; ++i)
      for (int j = 0; j < 6; ++j) {
        if (i != 0 || j != 0)
          std::cout << ',';
        std::cout << ball.preconditioner(i, j);
      }
    std::cout << "],\"trace\":";
    Print(ball.target_relative_trace);
    std::cout << ",\"determinant\":";
    Print(ball.closed_form_determinant);
    std::cout << ",\"coarse_eta\":" << ball.coarse_eta
              << ",\"used_log_series\":"
              << (ball.used_log_series ? "true" : "false") << '}';
  }
  std::cout << "]}";
}

void CheckContracts() {
  const auto input = Witness();
  crx::exact::Problem problem;
  Require(crx::exact::Prepare(input, problem) == crx::exact::Status::Ready,
          "witness prepare");
  const auto built = crx::catalogue::Build(problem);
  const auto &catalogue = built.catalogue;
  if (!catalogue.IsComplete()) {
    std::cerr << "Witness nominations=" << built.nominations
              << " balls=" << built.certified_balls << " roots="
              << catalogue.Values().root_cover.Values().simple_roots
              << " failure=" << static_cast<int>(catalogue.Values().failure)
              << '\n';
  }
  Require(catalogue.IsComplete() && !catalogue.IsInfeasible() &&
              catalogue.Values().balls.size() == 16,
          "complete sixteen-phase witness");
  Require(catalogue.Values().pair_tests == 120, "all phase pairs checked");
  const auto &data = catalogue.Values();
  auto duplicate = data.balls;
  duplicate.back() = duplicate.front();
  Require(
      crx::catalogue::Assemble(problem, data.root_cover, std::move(duplicate))
              .Values()
              .failure == Failure::BallsNotSeparated,
      "duplicate ball reached Complete");
  auto missing = data.balls;
  missing.pop_back();
  Require(crx::catalogue::Assemble(problem, data.root_cover, std::move(missing))
                  .Values()
                  .failure == Failure::CountMismatch,
          "missing ball reached Complete");
  Require(
      crx::catalogue::Assemble(problem, data.root_cover, {}).Values().failure ==
          Failure::CountMismatch,
      "empty numerical search became infeasible");
  Require(crx::catalogue::Assemble(problem, data.root_cover, data.balls, 0)
                  .Values()
                  .failure == Failure::PairWorkLimit,
          "pair budget ignored");
  auto invalid = data.balls;
  invalid.front() = {};
  Require(crx::catalogue::Assemble(problem, data.root_cover, std::move(invalid))
                  .Values()
                  .failure == Failure::InvalidBall,
          "invalid certificate accepted");
  crx::root::Options no_roots;
  no_roots.max_nodes = 0;
  Require(crx::catalogue::Assemble(
              problem, crx::root::Certify(problem, no_roots), data.balls)
                  .Values()
                  .failure == Failure::IncompleteRootCover,
          "unresolved cover reached Complete by count");
  crx::catalogue::Options no_balls;
  no_balls.max_candidate_certificates = 0;
  const auto exhausted = crx::catalogue::Build(problem, no_balls);
  Require(!exhausted.catalogue.IsComplete() &&
              exhausted.candidate_budget_exhausted,
          "candidate budget ignored");
  auto changed = input;
  changed.lower_commands.fill(0);
  changed.upper_commands.fill(0);
  crx::exact::Problem constrained;
  Require(crx::exact::Prepare(changed, constrained) ==
              crx::exact::Status::Ready,
          "limit preparation");
  Require(crx::catalogue::Assemble(constrained, data.root_cover, data.balls)
                  .Values()
                  .failure == Failure::IdentityMismatch,
          "model identity mismatch");
  Require(crx::catalogue::Build(constrained).catalogue.IsComplete(),
          "phase count incorrectly constrained by limits");
  changed = input;
  changed.target.position = {20, 30, -40};
  crx::exact::Problem far;
  Require(crx::exact::Prepare(changed, far) == crx::exact::Status::Ready,
          "far prepare");
  const auto excluded = crx::catalogue::Build(far);
  Require(excluded.catalogue.IsComplete() &&
              excluded.catalogue.IsInfeasible() && excluded.nominations == 0,
          "zero-root exclusion did not prove nominal infeasibility");
  crx::exact::Input zero;
  crx::exact::Problem family;
  Require(crx::exact::Prepare(zero, family) == crx::exact::Status::Ready,
          "family prepare");
  Require(!crx::catalogue::Build(family).catalogue.IsComplete(),
          "zero polynomial became infeasible");
  const auto seam = crx::catalogue::CircleDistance(0, crx::kHalfPi * 2);
  Require(seam.IsValid() && seam.lo > 3,
          "pi seam made distinct centres coincide");
  const auto control = _mm_getcsr();
  _mm_setcsr(control | 0x8000U);
  const auto unsupported =
      crx::catalogue::Assemble(problem, data.root_cover, data.balls);
  _mm_setcsr(control);
  Require(!unsupported.IsComplete() &&
              unsupported.Values().failure == Failure::UnsupportedEnvironment,
          "unsupported controls");
  std::cout << "Catalogue witness/count/identity/overlap/budget/exclusion "
               "checks passed\n";
}

template <std::size_t N> auto Read(std::array<double, N> &values) -> bool {
  for (auto &value : values)
    std::cin >> value;
  return static_cast<bool>(std::cin);
}

auto ReadInput(crx::exact::Input &input) -> bool {
  if (!Read(input.lengths) || !(std::cin >> input.base_height) ||
      !Read(input.encoder_zero) || !Read(input.senses) ||
      !Read(input.lower_commands) || !Read(input.upper_commands))
    return false;
  for (auto *pose : {&input.target, &input.base, &input.tool})
    if (!Read(pose->quaternion) || !Read(pose->position) ||
        !Read(pose->input_rotation))
      return false;
  return true;
}

auto Batch() -> int {
  int mode = 0;
  while (std::cin >> mode) {
    crx::catalogue::Options options;
    if (!(std::cin >> options.roots.max_nodes >> options.roots.max_depth >>
          options.roots.max_leaves >> options.roots.max_chart_attempts >>
          options.max_candidate_certificates >> options.max_pair_tests))
      return 2;
    crx::exact::Input input;
    if (!ReadInput(input))
      return 2;
    crx::exact::Problem problem;
    if (crx::exact::Prepare(input,
                            mode == 1
                                ? crx::exact::FrameConvention::BasePose
                                : crx::exact::FrameConvention::ReferenceMap,
                            problem) != crx::exact::Status::Ready)
      return 2;
    Print(crx::catalogue::Build(problem, options));
    std::cout << '\n';
  }
  return std::cin.eof() ? 0 : 2;
}
} // namespace

auto main(int argc, char **argv) -> int {
  std::cout << std::setprecision(17);
  if (argc == 2 && std::strcmp(argv[1], "--batch") == 0)
    return Batch();
  if (argc == 2 && std::strcmp(argv[1], "--distance") == 0) {
    double a = 0, b = 0;
    while (std::cin >> a >> b) {
      Print(crx::catalogue::CircleDistance(a, b));
      std::cout << '\n';
    }
    return std::cin.eof() ? 0 : 2;
  }
  if (argc != 1)
    return 2;
  CheckContracts();
  return 0;
}
