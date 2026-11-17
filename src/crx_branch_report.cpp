// SPDX-License-Identifier: Apache-2.0
// SPDX-FileCopyrightText: 2026 Centre Technologique en Aérospatiale

#include "crx_branch_report.h"
#include "crx_branch_graph.h"
#include "crx_exact_problem.h"
#include "crx_interval.h"
#include "crx_path_report.h"
#include "crx_root_certificate.h"

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <iomanip>
#include <ios>
#include <locale>
#include <ostream>
#include <string>

namespace crx::branch::report {
namespace {
using interval::Interval;

template <std::size_t N> struct Words {
  std::array<std::uint64_t, N> values{};
  std::size_t index = 0;

  void Real(double value) {
    std::memcpy(&values.at(index++), &value, sizeof(value));
  }

  template <typename Values> void Reals(const Values &source) {
    for (const auto value : source) {
      Real(value);
    }
  }

  void Pose(const exact::PoseInput &pose) {
    Reals(pose.quaternion);
    Reals(pose.position);
    Reals(pose.input_rotation);
  }
};

auto ModelWords(const exact::Input &source)
    -> std::array<std::uint64_t, exact::kIdentityWords> {
  Words<exact::kIdentityWords> words;
  words.Reals(source.lengths);
  words.Real(source.base_height);
  words.Reals(source.encoder_zero);
  words.Reals(source.senses);
  words.Reals(source.lower_commands);
  words.Reals(source.upper_commands);
  words.Pose(source.target);
  words.Pose(source.base);
  words.Pose(source.tool);
  return words.values;
}

void Value(std::ostream &out, double value) {
  if (value == 0 && std::signbit(value)) {
    out << "-0.0";
  } else if (std::isfinite(value)) {
    out << value;
  } else {
    out << "null";
  }
}

template <typename T> void Value(std::ostream &out, T value) { out << value; }

void Value(std::ostream &out, Interval value) {
  out << '[';
  Value(out, value.lo);
  out << ',';
  Value(out, value.hi);
  out << ']';
}

template <typename Values> void Array(std::ostream &out, const Values &values) {
  out << '[';
  bool first = true;
  for (const auto &value : values) {
    if (!first) {
      out << ',';
    }
    Value(out, value);
    first = false;
  }
  out << ']';
}

void String(std::ostream &out, const std::string &value) {
  out << '"';
  for (const auto c : value) {
    if (c == '"' || c == '\\') {
      out << '\\';
    }
    out << c;
  }
  out << '"';
}

void Roots(std::ostream &out, const root::Certificate &certificate) {
  const auto &data = certificate.Values();
  out << "{\"complete\":" << certificate.IsComplete()
      << ",\"status\":" << static_cast<int>(data.status)
      << ",\"entire_circle_unresolved\":" << data.entire_circle_unresolved
      << ",\"nodes\":" << data.nodes << ",\"attempts\":" << data.attempts
      << ",\"shift\":" << data.chart_shift
      << ",\"simple_roots\":" << data.simple_roots << ",\"charts\":[";
  for (std::size_t i = 0; i < 2; ++i) {
    if (i != 0U) {
      out << ',';
    }
    const auto &poly = data.polynomials.at(i);
    const auto &part = data.partitions.at(i);
    out << "{\"residual\":";
    Array(out, poly.residual);
    out << ",\"derivative\":";
    Array(out, poly.derivative);
    out << ",\"determinant\":";
    Array(out, poly.determinant);
    out << ",\"chart\":";
    Value(out, poly.chart);
    out << ",\"status\":" << static_cast<int>(part.status)
        << ",\"nodes\":" << part.nodes << ",\"max_depth\":" << part.max_depth
        << ",\"roots\":" << part.roots << ",\"exclusions\":" << part.exclusions
        << ",\"unresolved\":" << part.unresolved << ",\"leaves\":[";
    for (std::size_t j = 0; j < part.leaves.size(); ++j) {
      if (j != 0U) {
        out << ',';
      }
      const auto &leaf = part.leaves.at(j);
      out << "{\"kind\":" << static_cast<int>(leaf.kind)
          << ",\"reason\":" << static_cast<int>(leaf.reason) << ",\"tangent\":";
      Value(out, leaf.tangent);
      out << ",\"phase\":";
      Value(out, leaf.phase);
      out << ",\"depth\":" << leaf.depth << ",\"endpoints\":";
      Array(out, leaf.endpoint_values);
      out << ",\"residual_bernstein\":";
      Array(out, leaf.residual_bernstein);
      out << ",\"derivative_bernstein\":";
      Array(out, leaf.derivative_bernstein);
      out << ",\"determinant_bernstein\":";
      Array(out, leaf.determinant_bernstein);
      out << ",\"derivative_sign\":" << leaf.derivative_sign
          << ",\"determinant_sign\":" << leaf.determinant_sign << '}';
    }
    out << "]}";
  }
  out << "]}";
}

void Task(std::ostream &out, const TaskData &task) {
  out << "{\"id\":";
  String(out, task.source.id);
  Words<16> target_words;
  target_words.Pose(task.source.target);
  out << ",\"raw_target_words\":";
  Array(out, target_words.values);
  out << ",\"prepared\":" << task.prepared
      << ",\"preparation\":" << static_cast<int>(task.preparation)
      << ",\"limit_preparation\":" << static_cast<int>(task.limit_preparation)
      << ",\"input_schema\":" << task.problem.Data().identity.schema
      << ",\"input_words\":";
  Array(out, task.problem.Data().identity.words);
  out << ",\"vertices_complete\":" << task.vertices_complete
      << ",\"certified_infeasible\":" << task.certified_infeasible
      << ",\"vertices\":";
  Array(out, task.vertices);
  const auto &phases = task.phases.catalogue;
  out << ",\"phase_complete\":" << phases.IsComplete()
      << ",\"phase_failure\":" << static_cast<int>(phases.Values().failure)
      << ",\"nominations\":" << task.phases.nominations
      << ",\"point_attempts\":" << task.phases.attempted_balls
      << ",\"candidate_budget_exhausted\":"
      << task.phases.candidate_budget_exhausted
      << ",\"numerical_status\":" << task.phases.numerical_status
      << ",\"root_cover\":";
  Roots(out, phases.Values().root_cover);
  out << ",\"balls\":[";
  for (std::size_t i = 0; i < phases.Values().balls.size(); ++i) {
    if (i != 0U) {
      out << ',';
    }
    path::report::WritePoint(out, phases.Values().balls.at(i));
  }
  const auto &commands = task.commands.Values();
  out << "],\"command_failure\":" << static_cast<int>(commands.failure)
      << ",\"enumeration_complete\":" << commands.enumeration_complete
      << ",\"admission_complete\":" << commands.admission_complete
      << ",\"box_lifts\":" << commands.box_lifts << ",\"yes\":" << commands.yes
      << ",\"no\":" << commands.no << ",\"undecided\":" << commands.undecided
      << ",\"ranges\":[";
  for (std::size_t i = 0; i < commands.ranges.size(); ++i) {
    if (i != 0U) {
      out << ',';
    }
    const auto &range = commands.ranges.at(i);
    out << "{\"base\":";
    Array(out, range.base);
    out << ",\"first\":";
    Array(out, range.first);
    out << ",\"last\":";
    Array(out, range.last);
    out << ",\"count\":" << range.count << ",\"available\":" << range.available
        << ",\"empty\":" << range.empty << '}';
  }
  out << "]}";
}

void Relation(std::ostream &out, const branch::Relation &relation) {
  out << "{\"comparable\":" << relation.comparable
      << ",\"singularity_required\":" << relation.singularity_required
      << ",\"base_flip_verified\":" << relation.base_flip_verified
      << ",\"shoulder_wall_required\":" << relation.shoulder_wall_required
      << ",\"opposite_shoulder_signs\":" << relation.opposite_shoulder_signs
      << ",\"first_shoulder_sine\":";
  Value(out, relation.first_shoulder_sine);
  out << ",\"last_shoulder_sine\":";
  Value(out, relation.last_shoulder_sine);
  out << ",\"flip_turns\":";
  Array(out, relation.flip_turns);
  out << ",\"flip_image\":";
  Array(out, relation.flip_image);
  out << ",\"flip_inclusion\":";
  Value(out, relation.flip_inclusion);
  out << ",\"last_uniqueness_radius\":";
  Value(out, relation.last_uniqueness_radius);
  out << '}';
}

template <typename T>
auto Bytes(const std::vector<T> &values) -> std::uint64_t {
  return static_cast<std::uint64_t>(values.capacity()) * sizeof(T);
}
} // namespace

auto RetainedBytes(const Graph &graph) -> std::uint64_t {
  const auto &data = graph.Values();
  std::uint64_t bytes = sizeof(Graph) + Bytes(data.tasks) +
                        Bytes(data.vertices) + Bytes(data.connections) +
                        Bytes(data.pairs) + Bytes(data.attempts) +
                        Bytes(data.edges) + Bytes(data.components);
  for (const auto &task : data.tasks) {
    const auto &phases = task.phases.catalogue.Values();
    const auto &commands = task.commands.Values();
    bytes += task.source.id.capacity() + Bytes(task.vertices) +
             Bytes(phases.balls) + Bytes(task.phases.ball_failures) +
             Bytes(commands.ranges) + Bytes(commands.lifts) +
             Bytes(commands.possible_minimizers);
    for (const auto &partition : phases.root_cover.Values().partitions) {
      bytes += Bytes(partition.leaves);
    }
  }
  for (const auto &attempt : data.attempts) {
    bytes += Bytes(attempt.certificate.Values().leaves) +
             Bytes(attempt.certificate.Values().links);
  }
  for (const auto &part : data.components) {
    bytes += Bytes(part.vertices) + Bytes(part.admitted_tasks);
  }
  return bytes;
}

auto Write(std::ostream &out, const Graph &graph) -> bool {
  out.imbue(std::locale::classic());
  out << std::setprecision(17) << std::boolalpha;
  const auto &data = graph.Values();
  const auto &work = data.work;
  const auto &options = data.options;
  out << "{\"schema\":1,\"status\":" << static_cast<int>(data.status)
      << ",\"global_coverage\":\"Inconclusive\""
      << ",\"scope\":\"finite walk visiting supplied tasks in the certified "
         "subgraph; no exhaustive connectivity or dynamics claim\""
      << ",\"retained_payload_bytes\":" << RetainedBytes(graph)
      << ",\"convention\":" << static_cast<int>(data.convention)
      << ",\"profile\":" << static_cast<int>(data.raw_limits.profile)
      << ",\"raw_model_words\":";
  Array(out, ModelWords(data.raw_model));
  Words<24> limit_words;
  limit_words.Reals(data.raw_limits.lower_degrees);
  limit_words.Reals(data.raw_limits.upper_degrees);
  limit_words.Reals(data.raw_limits.seed_degrees);
  limit_words.Reals(data.raw_limits.weights);
  out << ",\"raw_limit_words\":";
  Array(out, limit_words.values);
  out << ",\"lower_degrees\":";
  Array(out, data.raw_limits.lower_degrees);
  out << ",\"upper_degrees\":";
  Array(out, data.raw_limits.upper_degrees);
  out << ",\"seed_degrees\":";
  Array(out, data.raw_limits.seed_degrees);
  out << ",\"weights\":";
  Array(out, data.raw_limits.weights);
  out << ",\"has_seed\":" << data.raw_limits.has_seed
      << ",\"options\":{\"max_vertices\":" << options.max_vertices
      << ",\"max_pair_tests\":" << options.max_pair_tests
      << ",\"max_paths\":" << options.max_paths
      << ",\"max_root_nodes\":" << options.max_root_nodes
      << ",\"max_endpoint_certificates\":" << options.max_endpoint_certificates
      << ",\"max_lifts\":" << options.max_lifts
      << ",\"max_path_nodes\":" << options.max_path_nodes
      << ",\"max_path_certificates\":" << options.max_path_certificates
      << ",\"endpoint\":{\"max_nodes\":" << options.endpoint.roots.max_nodes
      << ",\"max_depth\":" << options.endpoint.roots.max_depth
      << ",\"max_leaves\":" << options.endpoint.roots.max_leaves
      << ",\"max_chart_attempts\":" << options.endpoint.roots.max_chart_attempts
      << ",\"max_candidate_certificates\":"
      << options.endpoint.max_candidate_certificates
      << ",\"max_pair_tests\":" << options.endpoint.max_pair_tests
      << "},\"tracking\":{\"max_nodes\":" << options.tracking.max_nodes
      << ",\"max_depth\":" << options.tracking.max_depth
      << ",\"max_point_certificates\":"
      << options.tracking.max_point_certificates
      << ",\"max_trials_per_node\":" << options.tracking.max_trials_per_node
      << "}},\"work\":{\"root_nodes\":" << work.root_nodes
      << ",\"endpoint_certificates\":" << work.endpoint_certificates
      << ",\"enumerated_lifts\":" << work.enumerated_lifts
      << ",\"pair_tests\":" << work.pair_tests
      << ",\"path_attempts\":" << work.path_attempts
      << ",\"path_nodes\":" << work.path_nodes
      << ",\"path_certificates\":" << work.path_certificates
      << ",\"cell_checks\":" << work.cell_checks
      << ",\"subdivisions\":" << work.subdivisions << "},\"tasks\":[";
  for (std::size_t i = 0; i < data.tasks.size(); ++i) {
    if (i != 0U) {
      out << ',';
    }
    Task(out, data.tasks.at(i));
  }
  out << "],\"vertices\":[";
  for (std::size_t i = 0; i < data.vertices.size(); ++i) {
    if (i != 0U) {
      out << ',';
    }
    const auto &vertex = data.vertices.at(i);
    out << "{\"task\":" << vertex.task << ",\"phase\":" << vertex.phase
        << ",\"has_command_lift\":" << vertex.has_command_lift << ",\"turns\":";
    Array(out, vertex.turns);
    out << ",\"command_box\":";
    Array(out, vertex.command_box);
    out << ",\"canonical_box\":";
    Array(out, vertex.canonical_box);
    out << ",\"admission\":" << static_cast<int>(vertex.admission)
        << ",\"membership\":" << static_cast<int>(vertex.membership)
        << ",\"usable\":" << vertex.IsUsable() << ",\"inclusion\":";
    Value(out, vertex.inclusion);
    out << ",\"root\":";
    path::report::WritePoint(out, vertex.root.certificate);
    out << ",\"partial_admission\":";
    path::report::WriteCell(out, vertex.partial_admission);
    out << '}';
  }
  out << "],\"connections\":[";
  for (std::size_t i = 0; i < data.connections.size(); ++i) {
    if (i != 0U) {
      out << ',';
    }
    const auto &connection = data.connections.at(i);
    out << "{\"first\":" << connection.source.first
        << ",\"last\":" << connection.source.last
        << ",\"total_pairs\":" << connection.total_pairs
        << ",\"considered_pairs\":" << connection.considered_pairs
        << ",\"next_first\":" << connection.next_first
        << ",\"next_last\":" << connection.next_last
        << ",\"scan_complete\":" << connection.scan_complete << '}';
  }
  out << "],\"pairs\":[";
  for (std::size_t i = 0; i < data.pairs.size(); ++i) {
    if (i != 0U) {
      out << ',';
    }
    const auto &pair = data.pairs.at(i);
    out << "{\"connection\":" << pair.connection << ",\"first\":" << pair.first
        << ",\"last\":" << pair.last << ",\"attempted\":" << pair.attempted
        << ",\"attempt\":";
    if (pair.attempted) {
      out << pair.attempt;
    } else {
      out << "null";
    }
    out << ",\"relation\":";
    Relation(out, pair.relation);
    out << '}';
  }
  out << "],\"attempts\":[";
  for (std::size_t i = 0; i < data.attempts.size(); ++i) {
    if (i != 0U) {
      out << ',';
    }
    const auto &attempt = data.attempts.at(i);
    out << "{\"pair\":" << attempt.pair << ",\"path\":";
    path::report::Write(out, attempt.certificate);
    out << '}';
  }
  out << "],\"edges\":";
  Array(out, data.edges);
  out << ",\"components\":[";
  for (std::size_t i = 0; i < data.components.size(); ++i) {
    if (i != 0U) {
      out << ',';
    }
    const auto &part = data.components.at(i);
    out << "{\"vertices\":";
    Array(out, part.vertices);
    out << ",\"admitted_tasks\":";
    Array(out, part.admitted_tasks);
    out << ",\"covers_all_tasks\":" << part.covers_all_tasks << '}';
  }
  out << "]}";
  return out.good();
}
} // namespace crx::branch::report
