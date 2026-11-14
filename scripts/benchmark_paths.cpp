// SPDX-License-Identifier: Apache-2.0
// SPDX-FileCopyrightText: 2026 Centre Technologique en Aérospatiale

// Opt-in native harness, linked to the shipped libraries. It adds no solver
// implementation, production timing hook, DLL export or instrumented kernel.
#include "crx_ball_certificate.h"
#include "crx_branch_graph.h"
#include "crx_branch_report.h"
#include "crx_command_certificate.h"
#include "crx_exact_problem.h"
#include "crx_interval.h"
#include "crx_path_certificate.h"
#include "crx_path_request.h"
#include "crx_path_tracking.h"
#include "crx_phase_catalogue.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <exception>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <ratio>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace {
namespace path = crx::path;
using Clock = std::chrono::steady_clock;

struct Cell {
  crx::interval::Interval parameter;
  crx::exact::Problem problem;
  crx::ball::Certificate point;
  bool admitted = false;
};

struct Case {
  std::size_t pair = 0;
  path::Prepared path;
  path::RootWitness first, last;
  crx::command::Context limits;
  path::TrackingOptions options;
  std::vector<double> parameters;
  std::vector<Cell> cells;
};

auto Cases(const crx::path::request::Query &query,
           const crx::branch::Graph &graph) -> std::vector<Case> {
  std::vector<Case> cases;
  const auto &data = graph.Values();
  for (const auto &attempt : data.attempts) {
    const auto &pair = data.pairs.at(attempt.pair);
    const auto &first = data.vertices.at(pair.first);
    const auto &last = data.vertices.at(pair.last);
    auto raw = query.input.model;
    raw.target = query.input.tasks.at(first.task).target;
    Case item;
    item.pair = attempt.pair;
    if (path::Prepare({raw, query.input.tasks.at(last.task).target,
                       query.input.convention},
                      item.path) != path::Failure::None) {
      throw std::runtime_error("Frozen path preparation changed");
    }
    item.first = first.root;
    item.last = last.root;
    item.limits = data.tasks.at(first.task).limits;
    item.options = attempt.certificate.Values().options;
    for (const auto &leaf : attempt.certificate.Values().leaves) {
      const auto s = leaf.parameter.lo / 2 + leaf.parameter.hi / 2;
      item.parameters.push_back(s);
      if (leaf.cell.HasLocalLift()) {
        Cell cell;
        cell.parameter = leaf.parameter;
        cell.point = leaf.cell.Values().point_witness;
        cell.admitted = leaf.cell.IsAdmitted();
        auto point_source = item.path.Values().source.source;
        std::size_t word = 29;
        const auto restore = [&cell, &word](auto &values) {
          for (auto &value : values) {
            const auto bits = cell.point.Values().identity.words.at(word++);
            std::memcpy(&value, &bits, sizeof(value));
          }
        };
        restore(point_source.target.quaternion);
        restore(point_source.target.position);
        restore(point_source.target.input_rotation);
        if (crx::exact::Prepare(point_source, query.input.convention,
                                cell.problem) != crx::exact::Status::Ready ||
            !cell.point.Matches(cell.problem)) {
          throw std::runtime_error("Frozen point provenance changed");
        }
        item.cells.push_back(std::move(cell));
      }
    }
    cases.push_back(std::move(item));
  }
  return cases;
}

auto Preprocess(const crx::path::request::Query &query) -> std::uint64_t {
  std::uint64_t checksum = 0;
  // Standalone per-endpoint work with the declared per-task options. This
  // deliberately excludes graph binding and is not an additive decomposition
  // of a graph query's shared, possibly exhausted budgets.
  for (const auto &task : query.input.tasks) {
    auto raw = query.input.model;
    raw.target = task.target;
    crx::exact::Problem problem;
    crx::command::Context limits;
    if (crx::exact::Prepare(raw, query.input.convention, problem) !=
            crx::exact::Status::Ready ||
        crx::command::Prepare(problem, query.input.limits, limits) !=
            crx::command::Failure::None) {
      continue;
    }
    const auto built = crx::catalogue::Build(problem, query.options.endpoint);
    crx::command::Options options;
    options.max_lifts = query.options.max_lifts;
    const auto commands =
        crx::command::Certify(problem, built.catalogue, limits, options);
    checksum += built.attempted_balls +
                built.catalogue.Values().root_cover.Values().nodes +
                commands.Values().yes + commands.Values().undecided;
  }
  return checksum;
}

auto Replay(const std::vector<Case> &cases) -> std::uint64_t {
  std::uint64_t checksum = 0;
  for (const auto &item : cases) {
    const auto result = path::Track(item.path, item.first, item.last,
                                    item.limits, item.options);
    checksum += result.Values().work.point_certificates +
                result.Values().leaves.size() + (result.IsComplete() ? 1U : 0U);
  }
  return checksum;
}

auto Cells(const std::vector<Case> &cases) -> std::uint64_t {
  std::uint64_t checksum = 0;
  for (const auto &item : cases) {
    for (const auto &cell : item.cells) {
      const auto result = path::Certify(item.path, cell.parameter, cell.problem,
                                        cell.point, item.limits);
      if (!result.certificate.HasLocalLift() ||
          result.IsCertified() != cell.admitted) {
        throw std::runtime_error("Frozen uniform cell result changed");
      }
      checksum += result.IsCertified() ? 2U : 1U;
    }
  }
  return checksum;
}

struct Sample {
  std::size_t pair = 0;
  double parameter = 0;
  crx::ball::Centre preferred{}, proposed{};
  bool ready = false, full = false;
  std::size_t nominations = 0;
  int status = -1;
};

auto Numerical(const std::vector<Case> &cases,
               std::vector<Sample> *audit = nullptr) -> std::uint64_t {
  std::uint64_t checksum = 0;
  for (const auto &item : cases) {
    auto preferred = item.first.certificate.Values().centre;
    for (const auto parameter : item.parameters) {
      const auto result = path::Nominate(item.path, parameter, preferred);
      if (audit != nullptr) {
        Sample sample;
        sample.pair = item.pair;
        sample.parameter = parameter;
        sample.preferred = preferred;
        sample.ready = result.ready;
        sample.full = result.used_full_discovery;
        sample.nominations = result.centres.size();
        sample.status = result.numerical_status;
        if (!result.centres.empty()) {
          sample.proposed = result.centres.front();
        }
        audit->push_back(sample);
      }
      if (!result.centres.empty()) {
        preferred = result.centres.front();
      }
      checksum += result.centres.size() + (result.ready ? 1U : 0U) +
                  (result.used_full_discovery ? 1U : 0U);
    }
  }
  return checksum;
}

void Scalar(std::ostream &out, double value) {
  if (value == 0 && std::signbit(value)) {
    out << "-0.0";
  } else if (std::isfinite(value)) {
    out << value;
  } else {
    out << "null";
  }
}

template <typename T> void Scalar(std::ostream &out, T value) { out << value; }

template <typename Values> void Array(std::ostream &out, const Values &values) {
  out << '[';
  bool first = true;
  for (const auto &value : values) {
    if (!first) {
      out << ',';
    }
    Scalar(out, value);
    first = false;
  }
  out << ']';
}

struct Timing {
  std::vector<double> microseconds;
  std::uint64_t checksum = 0;
};

template <typename Function>
void Measure(Timing &timing, std::uint32_t iterations, Function function) {
  std::uint64_t checksum = 0;
  const auto start = Clock::now();
  for (std::uint32_t i = 0; i < iterations; ++i) {
    checksum += function();
  }
  const auto end = Clock::now();
  timing.microseconds.push_back(
      std::chrono::duration<double, std::micro>(end - start).count() /
      iterations);
  timing.checksum += checksum;
}

auto Run(const std::filesystem::path &filename, bool audit_only,
         std::uint32_t iterations, std::uint32_t rounds) -> int {
  const auto query = path::request::Read(filename);
  const auto graph = crx::branch::Build(query.input, query.options);
  if (graph.Values().status == crx::branch::Status::InvalidInput) {
    throw std::runtime_error("Invalid benchmark graph");
  }
  const auto cases = Cases(query, graph);
  std::vector<Sample> samples;
  const std::array<std::uint64_t, 4> audit{Preprocess(query), Replay(cases),
                                           Cells(cases),
                                           Numerical(cases, &samples)};
  std::array<Timing, 5> timings;
  if (!audit_only) {
    for (std::uint32_t round = 0; round < rounds; ++round) {
      const auto run = [&](std::size_t stage) {
        switch (stage) {
        case 0:
          Measure(timings.at(stage), iterations,
                  [&query] { return Preprocess(query); });
          break;
        case 1:
          Measure(timings.at(stage), iterations, [&query] {
            const auto result = crx::branch::Build(query.input, query.options);
            return result.Values().work.path_nodes +
                   result.Values().edges.size();
          });
          break;
        case 2:
          Measure(timings.at(stage), iterations,
                  [&cases] { return Replay(cases); });
          break;
        case 3:
          Measure(timings.at(stage), iterations,
                  [&cases] { return Cells(cases); });
          break;
        case 4:
          Measure(timings.at(stage), iterations,
                  [&cases] { return Numerical(cases); });
          break;
        default:
          throw std::runtime_error("Unknown benchmark stage");
        }
      };
      for (std::size_t i = 0; i < timings.size(); ++i) {
        run(round % 2 == 0 ? i : timings.size() - 1 - i);
      }
    }
  }
  std::cout
      << std::setprecision(17) << std::boolalpha
      << "{\"schema\":1,\"audit_only\":" << audit_only
      << ",\"iterations\":" << iterations << ",\"rounds\":" << rounds
      << ",\"stage_order\":[\"standalone_endpoint_preprocessing\",\"whole_"
         "graph\","
         "\"frozen_path_replay\",\"uniform_cells_with_sealed_point_witnesses\","
         "\"numerical_tracking_at_frozen_leaf_midpoints\"]"
      << ",\"audit_checksums\":";
  Array(std::cout, audit);
  std::cout << ",\"timings\":[";
  for (std::size_t i = 0; i < timings.size(); ++i) {
    if (i != 0U) {
      std::cout << ',';
    }
    std::cout << "{\"microseconds_per_iteration\":";
    Array(std::cout, timings.at(i).microseconds);
    std::cout << ",\"checksum\":" << timings.at(i).checksum << '}';
  }
  std::cout << "],\"samples\":[";
  for (std::size_t i = 0; i < samples.size(); ++i) {
    if (i != 0U) {
      std::cout << ',';
    }
    const auto &sample = samples.at(i);
    std::cout << "{\"pair\":" << sample.pair
              << ",\"parameter\":" << sample.parameter << ",\"preferred\":";
    Array(std::cout, sample.preferred);
    std::cout << ",\"proposed\":";
    Array(std::cout, sample.proposed);
    std::cout << ",\"ready\":" << sample.ready << ",\"full\":" << sample.full
              << ",\"nominations\":" << sample.nominations
              << ",\"status\":" << sample.status << '}';
  }
  std::cout << "],\"graph\":";
  crx::branch::report::Write(std::cout, graph);
  std::cout << "}\n";
  return std::cout.good() ? 0 : 2;
}

template <typename Char> auto Main(int argc, Char **argv) -> int {
  try {
    if (argc != 3 && argc != 4) {
      throw std::runtime_error("Usage: crx_path_benchmark request.bin --audit "
                               "OR request.bin iterations rounds");
    }
    const auto argument = std::filesystem::path(argv[2]).string();
    const bool audit = argument == "--audit";
    std::uint32_t iterations = 0, rounds = 0;
    if (audit && argc != 3) {
      throw std::runtime_error("Audit mode accepts no timing arguments");
    }
    if (!audit) {
      if (argc != 4) {
        throw std::runtime_error("Timed mode requires iterations and rounds");
      }
      const auto repeat_argument = std::filesystem::path(argv[3]).string();
      const auto digits = [](const std::string &text) {
        return !text.empty() &&
               std::all_of(text.begin(), text.end(),
                           [](char c) { return c >= '0' && c <= '9'; });
      };
      if (!digits(argument) || !digits(repeat_argument)) {
        throw std::runtime_error("Repetition counts must be decimal integers");
      }
      const auto count = std::stoul(argument);
      const auto repeat = std::stoul(repeat_argument);
      if (count == 0 || count > 10000 || repeat == 0 || repeat > 15) {
        throw std::runtime_error("Benchmark repetition bounds exceeded");
      }
      iterations = static_cast<std::uint32_t>(count);
      rounds = static_cast<std::uint32_t>(repeat);
    }
    return Run(std::filesystem::path(argv[1]), audit, iterations, rounds);
  } catch (const std::exception &error) {
    std::cerr << "crx_path_benchmark: " << error.what() << '\n';
    return 2;
  }
}
} // namespace

#ifdef _WIN32
auto wmain(int argc, wchar_t **argv) -> int { return Main(argc, argv); }
#else
auto main(int argc, char **argv) -> int { return Main(argc, argv); }
#endif
