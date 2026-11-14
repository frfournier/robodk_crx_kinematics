// SPDX-License-Identifier: Apache-2.0
// SPDX-FileCopyrightText: 2026 Centre Technologique en Aérospatiale

#include "crx_branch_graph.h"

#include <cstdlib>
#include <iostream>

#include <xmmintrin.h>

namespace {
namespace branch = crx::branch;

void Require(bool condition, const char *message) {
  if (!condition) {
    std::cerr << "Branch graph: " << message << '\n';
    std::exit(EXIT_FAILURE);
  }
}

auto Input() -> branch::Input {
  branch::Input input;
  input.model.lengths = {1, .6, .2, .5};
  input.model.target.quaternion = {60, -7, 7, -19};
  input.model.target.position = {27.0 / 64, 15.0 / 32, 55.0 / 128};
  input.model.lower_commands.fill(-4);
  input.model.upper_commands.fill(4);
  input.limits.lower_degrees.fill(-180);
  input.limits.upper_degrees.fill(180);
  input.tasks = {{"first", input.model.target}, {"again", input.model.target}};
  input.connections = {{0, 1}};
  return input;
}

void Budgets(const branch::Data &data) {
  const auto &work = data.work;
  const auto &options = data.options;
  Require(data.vertices.size() <= options.max_vertices &&
              work.root_nodes <= options.max_root_nodes &&
              work.endpoint_certificates <= options.max_endpoint_certificates &&
              work.enumerated_lifts <= options.max_lifts &&
              work.pair_tests <= options.max_pair_tests &&
              work.path_attempts <= options.max_paths &&
              work.path_nodes <= options.max_path_nodes &&
              work.path_certificates <= options.max_path_certificates,
          "shared budget exceeded");
  for (const auto edge : data.edges) {
    const auto &attempt = data.attempts.at(edge);
    Require(attempt.certificate.IsComplete(), "uncertified edge");
    const auto &pair = data.pairs.at(attempt.pair);
    const auto &proof = attempt.certificate.Values();
    Require(
        proof.first.problem.Data().identity ==
                data.vertices.at(pair.first).root.problem.Data().identity &&
            proof.last.problem.Data().identity ==
                data.vertices.at(pair.last).root.problem.Data().identity &&
            proof.first.certificate.Values().centre ==
                data.vertices.at(pair.first).root.certificate.Values().centre &&
            proof.last.certificate.Values().centre ==
                data.vertices.at(pair.last).root.certificate.Values().centre,
        "edge attached to different endpoint roots");
  }
}

void Contracts() {
  const auto input = Input();
  branch::Options options;
  options.max_paths = 16;
  const auto graph = branch::Build(input, options);
  const auto &data = graph.Values();
  Budgets(data);
  Require(data.status == branch::Status::CertifiedTaskCoverage &&
              data.tasks.size() == 2 && data.vertices.size() == 32 &&
              data.edges.size() == 16 && data.components.size() == 16,
          "repeated target roots collapsed or did not connect");
  for (const auto &part : data.components) {
    Require(part.vertices.size() == 2 && part.covers_all_tasks,
            "component task coverage");
  }
  bool flip = false, same_sign_nonflip = false, opposite = false;
  for (const auto &pair : data.pairs) {
    const auto &relation = pair.relation;
    const auto &a = data.vertices.at(pair.first);
    const auto &b = data.vertices.at(pair.last);
    if (relation.base_flip_verified) {
      flip = true;
      Require(relation.shoulder_wall_required &&
                  !relation.singularity_required &&
                  relation.flip_inclusion.IsValid() &&
                  relation.flip_inclusion.hi <= relation.last_uniqueness_radius,
              "unverified base flip");
    }
    if (relation.singularity_required) {
      opposite = true;
    }
    if (a.phase != b.phase && !relation.singularity_required &&
        !relation.base_flip_verified && !relation.opposite_shoulder_signs) {
      same_sign_nonflip = true;
      Require(!relation.shoulder_wall_required,
              "same-sign non-flip incorrectly requires shoulder wall");
    }
  }
  Require(flip && same_sign_nonflip && opposite,
          "crossing-condition fixtures missing");

  options.max_paths = 0;
  const auto isolated = branch::Build(input, options);
  Budgets(isolated.Values());
  Require(isolated.Values().status == branch::Status::Inconclusive &&
              isolated.Values().edges.empty() &&
              isolated.Values().components.size() == 32,
          "same task/pose labels merged components without edges");

  options = {};
  options.max_paths = 16;
  options.max_root_nodes = 0;
  const auto partial = branch::Build(input, options);
  Budgets(partial.Values());
  Require(partial.Values().tasks.size() == 2 &&
              !partial.Values().tasks.front().phases.catalogue.IsComplete() &&
              !partial.Values().tasks.front().vertices_complete &&
              !partial.Values().vertices.empty(),
          "incomplete catalogue discarded partial roots/tasks");
  for (const auto &vertex : partial.Values().vertices) {
    Require(!vertex.has_command_lift && vertex.root.IsReady(),
            "partial root has an unsupported command-lift claim");
  }

  options = {};
  options.max_pair_tests = 3;
  options.max_paths = 0;
  const auto limited = branch::Build(input, options);
  Budgets(limited.Values());
  Require(limited.Values().connections.front().considered_pairs == 3 &&
              !limited.Values().connections.front().scan_complete &&
              limited.Values().connections.front().total_pairs == 256,
          "pair omissions not retained");

  options = {};
  options.max_vertices = 0;
  const auto empty = branch::Build(input, options);
  Budgets(empty.Values());
  Require(empty.Values().status == branch::Status::Inconclusive &&
              empty.Values().tasks.size() == 2 &&
              empty.Values().vertices.empty(),
          "vertex budget became an infeasibility claim");

  auto narrow = input;
  narrow.limits.lower_degrees.fill(0);
  narrow.limits.upper_degrees.fill(0);
  const auto infeasible = branch::Build(narrow);
  Require(infeasible.Values().status ==
                  branch::Status::CertifiedTaskInfeasible &&
              infeasible.Values().tasks.front().commands.IsCommandInfeasible(),
          "sealed point-task infeasibility not preserved");
  auto invalid = input;
  invalid.tasks.back().id = invalid.tasks.front().id;
  Require(branch::Build(invalid).Values().status ==
              branch::Status::InvalidInput,
          "duplicate task identity accepted");
  options = {};
  options.max_path_nodes = 65536;
  Require(branch::Build(input, options).Values().status ==
              branch::Status::InvalidInput,
          "unbounded graph option accepted");
  const auto original_mode = _mm_getcsr();
  _mm_setcsr(original_mode | (1U << 15U));
  const auto unsupported = branch::Build(input);
  _mm_setcsr(original_mode);
  Require(unsupported.Values().status ==
                  branch::Status::UnsupportedEnvironment &&
              unsupported.Values().edges.empty() &&
              unsupported.Values().work.endpoint_certificates == 0,
          "unsupported floating-point mode admitted graph proofs");
  std::cout << "Branch graph contracts passed\n";
}
} // namespace

auto main() -> int { Contracts(); }
