// SPDX-License-Identifier: Apache-2.0
// SPDX-FileCopyrightText: 2026 Centre Technologique en Aérospatiale

#include "crx_phase_catalogue.h"

#include <array>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <iomanip>
#include <iostream>

namespace {
void Require(bool value, const char *message) {
  if (!value) {
    std::cerr << "Exceptional recovery failure: " << message << '\n';
    std::exit(1);
  }
}

void Number(double value) {
  if (std::isfinite(value))
    std::cout << value;
  else
    std::cout << "null";
}

template <typename Values> void Array(const Values &values) {
  std::cout << '[';
  bool first = true;
  for (const auto value : values) {
    if (!first)
      std::cout << ',';
    first = false;
    Number(value);
  }
  std::cout << ']';
}

void Print(const crx::catalogue::BuildResult &built) {
  const auto &data = built.catalogue.Values();
  std::cout << "{\"complete\":" << built.catalogue.IsComplete()
            << ",\"infeasible\":" << built.catalogue.IsInfeasible()
            << ",\"coverage\":" << static_cast<int>(data.coverage)
            << ",\"failure\":" << static_cast<int>(data.failure)
            << ",\"root_complete\":" << data.root_cover.IsComplete()
            << ",\"root_nodes\":" << data.root_cover.Values().nodes
            << ",\"attempted_balls\":" << built.attempted_balls
            << ",\"certified_balls\":" << built.certified_balls
            << ",\"candidate_budget_exhausted\":"
            << built.candidate_budget_exhausted
            << ",\"recovery_budget_exhausted\":"
            << built.recovery_budget_exhausted
            << ",\"recovery_domains\":" << built.recovery_domains
            << ",\"recoveries\":[";
  bool first = true;
  for (const auto &recovery : built.recoveries) {
    if (!first)
      std::cout << ',';
    first = false;
    std::cout << "{\"source\":" << recovery.source
              << ",\"near_stratum\":" << recovery.near_stratum
              << ",\"status\":" << static_cast<int>(recovery.numerical.status)
              << ",\"used_representatives\":"
              << recovery.numerical.used_singular_representatives
              << ",\"refined\":" << recovery.refinement_attempted
              << ",\"attempted_balls\":" << recovery.attempted_balls
              << ",\"certified_balls\":" << recovery.certified_balls
              << ",\"phase\":";
    Number(recovery.phase);
    std::cout << ",\"defects\":";
    Array(std::array<double, 6>{
        recovery.delta_defect, recovery.base_axis_defect,
        recovery.coincident_defect, recovery.vertical_defect,
        recovery.shell_defect, recovery.compatibility_defect});
    std::cout << ",\"candidates\":[";
    for (std::size_t i = 0; i < recovery.numerical.count; ++i) {
      if (i)
        std::cout << ',';
      const auto &candidate = recovery.numerical.candidates.at(i);
      std::cout << "{\"q\":";
      Array(candidate.joints);
      std::cout << ",\"position_error\":";
      Number(candidate.position_error);
      std::cout << ",\"orientation_error\":";
      Number(candidate.orientation_error);
      std::cout << '}';
    }
    std::cout << "]}";
  }
  std::cout << "],\"balls\":[";
  first = true;
  for (const auto &certificate : data.balls) {
    Require(certificate.IsValid(), "unsealed local ball");
    if (!first)
      std::cout << ',';
    first = false;
    const auto &ball = certificate.Values();
    std::cout << "{\"failure\":0,\"centre\":";
    Array(ball.centre);
    std::cout << ",\"constants\":";
    Array(std::array<double, 11>{
        ball.existence_radius, ball.uniqueness_radius, ball.inverse_residual,
        ball.beta, ball.lipschitz, ball.eta, ball.pose_defect,
        ball.screw_norm_sum, ball.angular_norm_sum, ball.anchor_position_norm,
        static_cast<double>(ball.aspect_sign)});
    std::cout << ",\"preconditioner\":[";
    for (int i = 0; i < 6; ++i)
      for (int j = 0; j < 6; ++j) {
        if (i || j)
          std::cout << ',';
        Number(ball.preconditioner(i, j));
      }
    std::cout << "],\"trace\":";
    Array(std::array<double, 2>{ball.target_relative_trace.lo,
                                ball.target_relative_trace.hi});
    std::cout << ",\"coarse_eta\":";
    Number(ball.coarse_eta);
    std::cout << ",\"used_log_series\":"
              << (ball.used_log_series ? "true" : "false") << '}';
  }
  std::cout << "]}\n";
}

auto Build(const crx::exact::Input &input, crx::catalogue::Options options = {})
    -> crx::catalogue::BuildResult {
  crx::exact::Problem problem;
  Require(crx::exact::Prepare(input, problem) == crx::exact::Status::Ready,
          "exact preparation");
  return crx::catalogue::Build(problem, options);
}

void Contracts() {
  crx::exact::Input input;
  input.lengths = {1, 1, 0, .25};
  input.target.quaternion = {.5, .5, .5, .5};
  input.target.position = {1, -.25, 0};
  crx::catalogue::Options options;
  options.roots.max_nodes = 256;
  const auto exceptional = Build(input, options);
  Require(!exceptional.catalogue.IsComplete() &&
              !exceptional.catalogue.IsInfeasible(),
          "exceptional recovery upgraded coverage");
  bool both = false;
  for (const auto &recovery : exceptional.recoveries)
    both = both ||
           ((recovery.near_stratum & crx::catalogue::NearDeterminantZero) &&
            recovery.numerical.count >= 4 && recovery.certified_balls >= 4);
  Require(both,
          "both exceptional elbow/base branches need separate local proofs");
  for (const std::array<double, 3> position :
       {std::array<double, 3>{0, 0, 0}, {0, 0, 2}}) {
    input.lengths = {1, 1, 0, 0};
    input.target.quaternion = {1, 0, 0, 0};
    input.target.position = position;
    const auto family = Build(input, options);
    Require(!family.recoveries.empty() && !family.catalogue.IsComplete() &&
                family.certified_balls == 0,
            "continuous family given an isolated-root certificate");
    bool retained = false;
    for (const auto &recovery : family.recoveries)
      retained = retained || (recovery.numerical.count > 0 &&
                              recovery.numerical.used_singular_representatives);
    Require(retained, "family numerical representatives lost");
  }
  input.lengths = {1, .6, .2, .5};
  input.target.quaternion = {60, -7, 7, -19};
  input.target.position = {27.0 / 64, 15.0 / 32, 55.0 / 128};
  const auto regular = Build(input);
  Require(regular.catalogue.IsComplete() && regular.recoveries.empty(),
          "complete regular cover used exceptional work");
  std::cout
      << "Exceptional witnesses, local proofs and family refusal passed\n";
}

auto Batch() -> int {
  crx::exact::Input input;
  while (std::cin >> input.lengths.front()) {
    for (std::size_t i = 1; i < input.lengths.size(); ++i)
      std::cin >> input.lengths.at(i);
    for (auto &value : input.target.quaternion)
      std::cin >> value;
    for (auto &value : input.target.position)
      std::cin >> value;
    crx::catalogue::Options options;
    std::cin >> options.roots.max_nodes >> options.max_candidate_certificates;
    if (!std::cin)
      return 2;
    Print(Build(input, options));
  }
  return std::cin.eof() ? 0 : 2;
}
} // namespace

auto main(int argc, char **argv) -> int {
  std::cout << std::setprecision(17);
  if (argc == 2 && std::strcmp(argv[1], "--batch") == 0)
    return Batch();
  if (argc != 1)
    return 2;
  Contracts();
  return 0;
}
