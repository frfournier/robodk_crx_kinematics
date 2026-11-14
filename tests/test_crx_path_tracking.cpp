// SPDX-License-Identifier: Apache-2.0
// SPDX-FileCopyrightText: 2026 Centre Technologique en Aérospatiale

#include "crx_ball_certificate.h"
#include "crx_command_certificate.h"
#include "crx_exact_problem.h"
#include "crx_interval.h"
#include "crx_path_certificate.h"
#include "crx_path_report.h"
#include "crx_path_tracking.h"
#include "crx_phase_catalogue.h"

#include <cstdlib>
#include <iostream>
#include <string>

namespace {
namespace path = crx::path;
using crx::interval::Interval;

void Require(bool condition, const char *message) {
  if (!condition) {
    std::cerr << "Path tracking: " << message << '\n';
    std::exit(EXIT_FAILURE);
  }
}

auto Source() -> crx::exact::Input {
  crx::exact::Input source;
  source.lengths = {1, .6, .2, .5};
  source.target.quaternion = {60, -7, 7, -19};
  source.target.position = {27.0 / 64, 15.0 / 32, 55.0 / 128};
  source.lower_commands.fill(-4);
  source.upper_commands.fill(4);
  source.base.quaternion.at(1) = -0.0;
  return source;
}

auto Context(const crx::exact::Problem &problem) -> crx::command::Context {
  crx::command::Input raw;
  raw.lower_degrees.fill(-720);
  raw.upper_degrees.fill(720);
  crx::command::Context context;
  Require(crx::command::Prepare(problem, raw, context) ==
              crx::command::Failure::None,
          "command context");
  return context;
}

auto End(const path::Prepared &prepared, const crx::ball::Centre &preferred)
    -> path::RootWitness {
  auto input = prepared.Values().source.source;
  input.target = prepared.Values().source.last;
  path::RootWitness result;
  Require(crx::exact::Prepare(input, prepared.Values().source.convention,
                              result.problem) == crx::exact::Status::Ready,
          "end prepare");
  auto point = crx::ball::Certify(result.problem, preferred);
  if (point.IsCertified()) {
    result.certificate = point.certificate;
    return result;
  }
  const auto proposed = path::Nominate(prepared, 1, preferred);
  for (const auto &centre : proposed.centres) {
    point = crx::ball::Certify(result.problem, centre);
    if (point.IsCertified()) {
      result.certificate = point.certificate;
      return result;
    }
  }
  Require(false, "end root nomination failed");
  return result;
}

void Partition(const path::TrackingCertificate &result) {
  const auto &data = result.Values();
  double before = 0;
  for (const auto &leaf : data.leaves) {
    Require(leaf.parameter.IsValid() && leaf.parameter.lo == before &&
                leaf.parameter.hi > before,
            "leaf partition gap/order");
    before = leaf.parameter.hi;
    if (leaf.IsAdmitted()) {
      Require(leaf.cell.IsAdmitted(), "unsealed admitted leaf");
    }
  }
  Require(before == 1, "partition omits final interval");
  Require(data.work.nodes <= data.options.max_nodes &&
              data.work.point_certificates <=
                  data.options.max_point_certificates,
          "work budget exceeded");
  for (const auto &link : data.links) {
    Require(link.certificate.IsValid(), "unsealed link");
  }
}

void Contracts() {
  path::Input input{Source(), Source().target};
  path::Prepared prepared;
  Require(path::Prepare(input, prepared) == path::Failure::None,
          "path prepare");
  const auto &problem = prepared.Values().first_problem;
  const auto built = crx::catalogue::Build(problem);
  Require(built.catalogue.IsComplete() &&
              built.catalogue.Values().balls.size() == 16,
          "complete fixture");
  const auto &balls = built.catalogue.Values().balls;
  const path::RootWitness first{problem, balls.front()};
  const auto limits = Context(problem);
  const auto constant = path::Track(prepared, first, first, limits);
  Require(constant.IsComplete() && constant.Values().leaves.size() == 1,
          "constant path completion");
  Partition(constant);
  auto displaced = first.certificate.Values().centre;
  displaced.at(0) += .2 * first.certificate.Values().uniqueness_radius;
  const auto displaced_proof = crx::ball::Certify(problem, displaced);
  Require(displaced_proof.IsCertified() &&
              4 * displaced_proof.certificate.Values().existence_radius >
                  displaced_proof.certificate.Values().uniqueness_radius,
          "terminal-cell reserve fixture");
  const path::RootWitness wide_root{problem, displaced_proof.certificate};
  path::TrackingOptions one_trial;
  one_trial.max_nodes = 1;
  one_trial.max_trials_per_node = 1;
  const auto terminal =
      path::Track(prepared, wide_root, wide_root, limits, one_trial);
  Require(terminal.IsComplete() &&
              terminal.Values().work.point_certificates == 1 &&
              terminal.Values().work.nearby_calls == 0,
          "valid terminal cell demanded an unused future reserve");
  Partition(terminal);
  auto generous = path::TrackingOptions{};
  generous.max_nodes = 65535;
  const auto short_with_budget =
      path::Track(prepared, first, first, limits, generous);
  Require(short_with_budget.IsComplete() &&
              short_with_budget.Values().leaves.capacity() < 64,
          "constant path eagerly allocated its full node ceiling");
  path::TrackingOptions zero;
  zero.max_nodes = 0;
  const auto no_work = path::Track(prepared, first, first, limits, zero);
  Require(!no_work.IsComplete() && no_work.Values().work.nodes == 0 &&
              no_work.Values().leaves.size() == 1,
          "zero node budget promoted");
  Partition(no_work);
  zero = {};
  zero.max_point_certificates = 0;
  const auto no_proofs = path::Track(prepared, first, first, limits, zero);
  Require(!no_proofs.IsComplete() &&
              no_proofs.Values().work.point_certificates == 0,
          "zero proof budget promoted");
  Partition(no_proofs);
  auto moved = input;
  moved.last.position.at(0) += 1e-12;
  path::Prepared small;
  Require(path::Prepare(moved, small) == path::Failure::None, "small prepare");
  const auto end = End(small, first.certificate.Values().centre);
  const auto forward = path::Track(small, first, end, limits);
  Require(forward.IsComplete(), "small target path completion");
  Partition(forward);
  auto reverse = moved;
  reverse.source.target = moved.last;
  reverse.last = moved.source.target;
  path::Prepared backward;
  Require(path::Prepare(reverse, backward) == path::Failure::None,
          "reverse prepare");
  const auto reversed = path::Track(backward, end, first, limits);
  Require(reversed.IsComplete(), "reverse completion");
  Partition(reversed);
  Require(path::Track(small, first, first, limits).Values().failure ==
              path::TrackingFailure::EndpointIdentity,
          "different endpoint targets identified");
  auto multiple = input;
  multiple.last.position.at(0) += 1e-6;
  path::Prepared subdivided;
  Require(path::Prepare(multiple, subdivided) == path::Failure::None,
          "subdivision prepare");
  const auto multiple_end = End(subdivided, first.certificate.Values().centre);
  const auto continued = path::Track(subdivided, first, multiple_end, limits);
  if (!continued.IsComplete()) {
    std::cerr << "Multi-cell result: nodes=" << continued.Values().work.nodes
              << " points=" << continued.Values().work.point_certificates
              << " leaves=" << continued.Values().leaves.size()
              << " reason=" << static_cast<int>(continued.Values().failure)
              << '\n';
  }
  Require(continued.IsComplete() && continued.Values().work.subdivisions > 0 &&
              continued.Values().leaves.size() > 1,
          "known admissible multi-cell lift failed");
  Partition(continued);
  Require(continued.Values().first.IsReady() &&
              continued.Values().last.IsReady() &&
              continued.Values().limits.IsReady(),
          "endpoint/limit ownership lost");
  moved.last.position.at(0) += 1e-4;
  path::Prepared broad;
  Require(path::Prepare(moved, broad) == path::Failure::None, "broad prepare");
  const auto broad_end = End(broad, first.certificate.Values().centre);
  path::TrackingOptions bounded;
  bounded.max_nodes = 15;
  bounded.max_depth = 8;
  bounded.max_point_certificates = 64;
  bounded.max_trials_per_node = 3;
  const auto partial = path::Track(broad, first, broad_end, limits, bounded);
  Require(!partial.IsComplete() &&
              partial.Values().uncovered_measure.IsValid() &&
              partial.Values().uncovered_measure.hi > 0,
          "budgeted broad path falsely complete");
  Partition(partial);
  bool opposite = false, same = false;
  for (std::size_t i = 1; i < balls.size(); ++i) {
    path::RootWitness other{problem, balls.at(i)};
    if (balls.at(i).Values().aspect_sign !=
            balls.front().Values().aspect_sign &&
        !opposite) {
      const auto blocked = path::Track(prepared, first, other, limits, bounded);
      Require(!blocked.IsComplete() &&
                  blocked.Values().failure ==
                      path::TrackingFailure::OppositeRegularSigns,
              "opposite regular signs certified");
      Partition(blocked);
      opposite = true;
    } else if (balls.at(i).Values().aspect_sign ==
                   balls.front().Values().aspect_sign &&
               !same) {
      const auto blocked = path::Track(prepared, first, other, limits, bounded);
      Require(!blocked.IsComplete(), "distinct constant branches joined");
      Partition(blocked);
      same = true;
    }
  }
  Require(opposite && same, "sign fixture incomplete");
  auto shifted = first;
  auto centre = first.certificate.Values().centre;
  centre.at(0) += 6.283185307179586;
  const auto turned = crx::ball::Certify(problem, centre);
  Require(turned.IsCertified(), "turned point witness");
  shifted.certificate = turned.certificate;
  const auto turns = path::Track(prepared, shifted, shifted, limits);
  Require(turns.IsComplete() && turns.Values()
                                        .leaves.front()
                                        .cell.Values()
                                        .point_witness.Values()
                                        .centre.at(0) > 3,
          "tracking erased real turn");
  Partition(turns);
}

void Reports() {
  path::Input input{Source(), Source().target};
  path::Prepared prepared;
  Require(path::Prepare(input, prepared) == path::Failure::None,
          "report source");
  const auto &problem = prepared.Values().first_problem;
  const auto built = crx::catalogue::Build(problem);
  Require(built.catalogue.IsComplete(), "report endpoint catalogue");
  const path::RootWitness first{problem,
                                built.catalogue.Values().balls.front()};
  const auto limits = Context(problem);
  for (double step : {0., 1e-6}) {
    auto moved = input;
    moved.last.position.at(0) += step;
    path::Prepared path;
    Require(path::Prepare(moved, path) == path::Failure::None, "report path");
    const auto end = End(path, first.certificate.Values().centre);
    const auto trace = path::Track(path, first, end, limits);
    Require(trace.IsComplete(), "report positive trace");
    Require(path::report::Write(std::cout, trace), "write report");
    std::cout << '\n';
    path::TrackingOptions bounded;
    bounded.max_nodes = 3;
    bounded.max_point_certificates = 8;
    if (step != 0) {
      const auto partial = path::Track(path, first, end, limits, bounded);
      Require(!partial.IsComplete(), "report budget control");
      Require(path::report::Write(std::cout, partial), "write partial");
      std::cout << '\n';
    }
  }
}
} // namespace

auto main(int argc, char **argv) -> int {
  if (argc == 2 && std::string(argv[1]) == "--witness") {
    Reports();
    return 0;
  }
  if (argc != 1) {
    return 2;
  }
  Contracts();
  return 0;
}
