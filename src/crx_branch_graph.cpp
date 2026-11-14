// SPDX-License-Identifier: Apache-2.0
// SPDX-FileCopyrightText: 2026 Centre Technologique en Aérospatiale

#include "crx_branch_graph.h"
#include "crx_ball_certificate.h"
#include "crx_command_certificate.h"
#include "crx_exact_problem.h"
#include "crx_interval.h"
#include "crx_path_certificate.h"
#include "crx_path_tracking.h"
#include "crx_phase_catalogue.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <numeric>
#include <utility>
#include <vector>

#ifdef __FAST_MATH__
#error "Branch proof orchestration requires strict floating-point compilation"
#endif

namespace crx::branch {
namespace {
using interval::Interval;

auto Midpoint(Interval x) -> double { return x.lo / 2 + x.hi / 2; }

auto Sign(Interval x) -> int {
  return !x.IsValid() ? 0 : (x.lo > 0 ? 1 : (x.hi < 0 ? -1 : 0));
}

auto BallBox(const ball::Data &data) -> command::Box {
  command::Box box;
  for (std::size_t i = 0; i < box.size(); ++i) {
    box.at(i) = Interval{data.centre.at(i)} +
                Interval{-data.existence_radius, data.existence_radius};
  }
  return box;
}

auto Inclusion(const command::Box &box, const ball::Data &data) -> Interval {
  double upper = 0;
  for (std::size_t i = 0; i < box.size(); ++i) {
    const auto distance =
        interval::Abs(box.at(i) - Interval{data.centre.at(i)});
    if (!distance.IsValid()) {
      return {};
    }
    upper = std::max(upper, distance.hi);
  }
  return upper <= data.uniqueness_radius ? Interval{0, upper} : Interval{};
}

auto Valid(const Input &input, const Options &options) -> bool {
  const auto &root = options.endpoint.roots;
  const auto &tracking = options.tracking;
  if ((input.convention != exact::FrameConvention::BasePose &&
       input.convention != exact::FrameConvention::ReferenceMap) ||
      (input.limits.profile != command::Profile::Callback &&
       input.limits.profile != command::Profile::DecoupledUser) ||
      input.tasks.empty() || input.tasks.size() > 64 ||
      input.connections.size() > 256 || options.max_vertices > 4096 ||
      options.max_pair_tests > 65536 || options.max_paths > 4096 ||
      options.max_root_nodes > 65535 ||
      options.max_endpoint_certificates > 65535 || options.max_lifts > 65535 ||
      options.max_path_nodes > 65535 ||
      options.max_path_certificates > (1U << 20U) || root.max_nodes > 65535 ||
      root.max_depth > 50 || root.max_leaves < 2 || root.max_leaves > 65535 ||
      root.max_chart_attempts == 0 || root.max_chart_attempts > 8 ||
      options.endpoint.max_candidate_certificates > 512 ||
      options.endpoint.max_pair_tests > 65536 || tracking.max_nodes > 65535 ||
      tracking.max_depth > 48 ||
      tracking.max_point_certificates > (1U << 20U) ||
      tracking.max_trials_per_node == 0 || tracking.max_trials_per_node > 128) {
    return false;
  }
  for (std::size_t i = 0; i < input.tasks.size(); ++i) {
    const auto &id = input.tasks.at(i).id;
    if (id.empty() || id.size() > 256 ||
        !std::all_of(id.begin(), id.end(),
                     [](unsigned char c) { return c >= 32 && c <= 126; })) {
      return false;
    }
    for (std::size_t j = 0; j < i; ++j) {
      if (input.tasks.at(j).id == id) {
        return false;
      }
    }
  }
  return std::all_of(input.connections.begin(), input.connections.end(),
                     [&input](const Connection &connection) {
                       return connection.first < input.tasks.size() &&
                              connection.last < input.tasks.size();
                     });
}

auto Remaining(std::uint32_t limit, std::uint64_t used) -> std::uint32_t {
  return used < limit ? limit - static_cast<std::uint32_t>(used) : 0;
}

auto CanonicalBox(const exact::Problem &problem, command::Box box,
                  command::Profile profile) -> command::Box {
  if (profile == command::Profile::DecoupledUser) {
    box.at(2) += box.at(1);
  }
  for (std::size_t i = 0; i < box.size(); ++i) {
    box.at(i) = problem.Data().offset_unwrapped.at(i) +
                Interval{problem.Data().diagonal.at(i)} * box.at(i);
  }
  return box;
}

void Bind(Vertex &vertex, const ball::Certificate &source, Data &data) {
  if (!source.Matches(vertex.root.problem)) {
    return;
  }
  vertex.inclusion = Inclusion(vertex.canonical_box, source.Values());
  if (vertex.inclusion.IsValid()) {
    vertex.root.certificate = source;
    vertex.membership = Membership::LiftBoxInUniqueness;
    return;
  }
  if (data.work.endpoint_certificates >=
      data.options.max_endpoint_certificates) {
    return;
  }
  ball::Centre centre;
  for (std::size_t i = 0; i < centre.size(); ++i) {
    if (!vertex.canonical_box.at(i).IsValid()) {
      return;
    }
    centre.at(i) = Midpoint(vertex.canonical_box.at(i));
  }
  ++data.work.endpoint_certificates;
  const auto rebound = ball::Certify(vertex.root.problem, centre);
  if (!rebound.IsCertified()) {
    return;
  }
  vertex.inclusion =
      Inclusion(vertex.canonical_box, rebound.certificate.Values());
  if (!vertex.inclusion.IsValid()) {
    return;
  }
  vertex.root.certificate = rebound.certificate;
  vertex.membership = Membership::LiftBoxInUniqueness;
}

void AddVertex(Data &data, TaskData &task, Vertex vertex) {
  if (data.vertices.size() >= data.options.max_vertices) {
    return;
  }
  task.vertices.push_back(data.vertices.size());
  data.vertices.push_back(std::move(vertex));
}

void PrepareTask(const Input &input, std::size_t index, Data &data) {
  TaskData task;
  task.source = input.tasks.at(index);
  auto raw = input.model;
  raw.target = task.source.target;
  task.preparation = exact::Prepare(raw, input.convention, task.problem);
  if (task.preparation == exact::Status::Ready) {
    task.limit_preparation =
        command::Prepare(task.problem, input.limits, task.limits);
  }
  if (task.preparation != exact::Status::Ready ||
      task.limit_preparation != command::Failure::None) {
    data.tasks.push_back(std::move(task));
    return;
  }
  task.prepared = true;
  auto options = data.options.endpoint;
  options.roots.max_nodes =
      std::min(options.roots.max_nodes,
               Remaining(data.options.max_root_nodes, data.work.root_nodes));
  options.max_candidate_certificates =
      std::min(options.max_candidate_certificates,
               Remaining(data.options.max_endpoint_certificates,
                         data.work.endpoint_certificates));
  task.phases = catalogue::Build(task.problem, options);
  const auto &phases = task.phases.catalogue;
  data.work.root_nodes += phases.Values().root_cover.Values().nodes;
  data.work.endpoint_certificates += task.phases.attempted_balls;
  command::Options lift_options;
  lift_options.max_lifts =
      Remaining(data.options.max_lifts, data.work.enumerated_lifts);
  task.commands =
      command::Certify(task.problem, phases, task.limits, lift_options);
  const auto &commands = task.commands.Values();
  if (commands.enumeration_complete) {
    data.work.enumerated_lifts += commands.box_lifts;
  }
  task.certified_infeasible =
      phases.IsInfeasible() || task.commands.IsCommandInfeasible();
  const auto &balls = phases.Values().balls;
  if (commands.enumeration_complete) {
    for (const auto &lift : commands.lifts) {
      if (data.vertices.size() >= data.options.max_vertices) {
        break;
      }
      Vertex vertex;
      vertex.task = index;
      vertex.phase = lift.phase;
      vertex.has_command_lift = true;
      vertex.turns = lift.turns;
      vertex.command_box = lift.radians;
      vertex.canonical_box =
          CanonicalBox(task.problem, lift.radians, input.limits.profile);
      vertex.admission = lift.admission;
      vertex.root.problem = task.problem;
      if (lift.phase < balls.size()) {
        Bind(vertex, balls.at(lift.phase), data);
      }
      AddVertex(data, task, std::move(vertex));
    }
    task.vertices_complete = task.vertices.size() == commands.lifts.size();
  } else {
    // Partial phase roots remain useful, but have no exhaustive lift/turn
    // claim. Assemble can retain unvalidated balls after an incomplete root
    // cover, so validate every seal and its full exact identity here.
    path::Prepared constant;
    const bool prepared = path::Prepare({raw, raw.target, input.convention},
                                        constant) == path::Failure::None;
    for (std::size_t phase = 0; phase < balls.size(); ++phase) {
      if (data.vertices.size() >= data.options.max_vertices) {
        break;
      }
      const auto &ball = balls.at(phase);
      if (!ball.Matches(task.problem)) {
        continue;
      }
      Vertex vertex;
      vertex.task = index;
      vertex.phase = phase;
      vertex.root = {task.problem, ball};
      vertex.membership = Membership::SourceBall;
      vertex.canonical_box = BallBox(ball.Values());
      if (prepared) {
        const auto admitted = path::Certify(constant, Interval{0, 1},
                                            task.problem, ball, task.limits);
        vertex.partial_admission = admitted.certificate;
        if (admitted.IsCertified()) {
          vertex.admission = command::Admission::Yes;
        }
      }
      AddVertex(data, task, std::move(vertex));
    }
  }
  data.tasks.push_back(std::move(task));
}

auto Distance(const Vertex &first, const Vertex &last) -> double {
  if (!first.IsUsable() || !last.IsUsable()) {
    return std::numeric_limits<double>::infinity();
  }
  double distance = 0;
  for (std::size_t i = 0; i < 6; ++i) {
    distance = std::max(distance,
                        std::abs(first.root.certificate.Values().centre.at(i) -
                                 last.root.certificate.Values().centre.at(i)));
  }
  return distance;
}

void ScanPairs(Data &data, const Input &input) {
  for (std::size_t index = 0; index < input.connections.size(); ++index) {
    ConnectionData connection;
    connection.source = input.connections.at(index);
    const auto &first = data.tasks.at(connection.source.first).vertices;
    const auto &last = data.tasks.at(connection.source.last).vertices;
    connection.total_pairs =
        static_cast<std::uint64_t>(first.size()) * last.size();
    // Reflexive pairs do not connect distinct vertices and need no edge.
    if (connection.source.first == connection.source.last) {
      connection.total_pairs -= first.size();
    }
    std::size_t i = 0, j = 0;
    while (i < first.size() && !last.empty()) {
      if (first.at(i) == last.at(j)) {
        if (++j == last.size()) {
          j = 0;
          ++i;
        }
        continue;
      }
      if (data.work.pair_tests >= data.options.max_pair_tests) {
        break;
      }
      Pair pair;
      pair.connection = index;
      pair.first = first.at(i);
      pair.last = last.at(j);
      const auto &a = data.vertices.at(pair.first);
      const auto &b = data.vertices.at(pair.last);
      if (a.root.IsReady() && b.root.IsReady()) {
        pair.relation = Classify(a.root, b.root);
      }
      data.pairs.push_back(pair);
      ++data.work.pair_tests;
      ++connection.considered_pairs;
      if (++j == last.size()) {
        j = 0;
        ++i;
      }
    }
    connection.next_first = i;
    connection.next_last = j;
    connection.scan_complete =
        connection.considered_pairs == connection.total_pairs;
    data.connections.push_back(connection);
  }
}

void Connect(Data &data, const Input &input) {
  std::vector<std::size_t> order(data.pairs.size());
  std::iota(order.begin(), order.end(), std::size_t{0});
  // Distance only prioritizes nominations. It never accepts a link or proves
  // nearestness. Stable indices resolve all equal/infinite proposals.
  std::stable_sort(
      order.begin(), order.end(), [&data](std::size_t a, std::size_t b) {
        const auto &x = data.pairs.at(a);
        const auto &y = data.pairs.at(b);
        return Distance(data.vertices.at(x.first), data.vertices.at(x.last)) <
               Distance(data.vertices.at(y.first), data.vertices.at(y.last));
      });
  for (const auto index : order) {
    if (data.work.path_attempts >= data.options.max_paths) {
      break;
    }
    auto &pair = data.pairs.at(index);
    const auto &first = data.vertices.at(pair.first);
    const auto &last = data.vertices.at(pair.last);
    if (!first.IsUsable() || !last.IsUsable()) {
      continue;
    }
    auto raw = input.model;
    raw.target = input.tasks.at(first.task).target;
    path::Prepared prepared;
    if (path::Prepare({raw, input.tasks.at(last.task).target, input.convention},
                      prepared) != path::Failure::None) {
      continue;
    }
    auto options = data.options.tracking;
    options.max_nodes =
        std::min(options.max_nodes,
                 Remaining(data.options.max_path_nodes, data.work.path_nodes));
    options.max_point_certificates =
        std::min(options.max_point_certificates,
                 Remaining(data.options.max_path_certificates,
                           data.work.path_certificates));
    pair.attempted = true;
    pair.attempt = data.attempts.size();
    Attempt attempt;
    attempt.pair = index;
    attempt.certificate =
        path::Track(prepared, first.root, last.root,
                    data.tasks.at(first.task).limits, options);
    const auto &work = attempt.certificate.Values().work;
    ++data.work.path_attempts;
    data.work.path_nodes += work.nodes;
    data.work.path_certificates += work.point_certificates;
    data.work.cell_checks += work.cell_checks;
    data.work.subdivisions += work.subdivisions;
    if (attempt.certificate.IsComplete()) {
      data.edges.push_back(data.attempts.size());
    }
    data.attempts.push_back(std::move(attempt));
  }
}

void Components(Data &data) {
  std::vector<std::size_t> parent(data.vertices.size());
  std::iota(parent.begin(), parent.end(), std::size_t{0});
  const auto find = [&parent](std::size_t vertex) {
    while (parent.at(vertex) != vertex) {
      parent.at(vertex) = parent.at(parent.at(vertex));
      vertex = parent.at(vertex);
    }
    return vertex;
  };
  for (const auto edge : data.edges) {
    const auto &pair = data.pairs.at(data.attempts.at(edge).pair);
    parent.at(find(pair.last)) = find(pair.first);
  }
  std::vector<std::size_t> component(data.vertices.size(),
                                     data.vertices.size());
  for (std::size_t vertex = 0; vertex < data.vertices.size(); ++vertex) {
    const auto root = find(vertex);
    if (component.at(root) == data.vertices.size()) {
      component.at(root) = data.components.size();
      data.components.emplace_back();
    }
    auto &part = data.components.at(component.at(root));
    part.vertices.push_back(vertex);
    if (data.vertices.at(vertex).IsUsable()) {
      part.admitted_tasks.push_back(data.vertices.at(vertex).task);
    }
  }
  bool covered = false;
  for (auto &part : data.components) {
    auto &tasks = part.admitted_tasks;
    std::sort(tasks.begin(), tasks.end());
    tasks.erase(std::unique(tasks.begin(), tasks.end()), tasks.end());
    part.covers_all_tasks = tasks.size() == data.tasks.size();
    covered = covered || part.covers_all_tasks;
  }
  const bool infeasible = std::any_of(
      data.tasks.begin(), data.tasks.end(),
      [](const TaskData &task) { return task.certified_infeasible; });
  data.status = infeasible ? Status::CertifiedTaskInfeasible
                           : (covered ? Status::CertifiedTaskCoverage
                                      : Status::Inconclusive);
}
} // namespace

auto Classify(const path::RootWitness &first, const path::RootWitness &last)
    -> Relation {
  Relation result;
  if (!interval::EnvironmentSupported() || !first.IsReady() ||
      !last.IsReady() ||
      !exact::SameModel(first.problem.Data().identity,
                        last.problem.Data().identity)) {
    return result;
  }
  result.comparable = true;
  const auto &a = first.certificate.Values();
  const auto &b = last.certificate.Values();
  result.singularity_required = a.aspect_sign != b.aspect_sign;
  auto box = BallBox(a);
  result.first_shoulder_sine = interval::EvaluateSinCos(box.at(1)).sine;
  result.last_shoulder_sine = interval::EvaluateSinCos(BallBox(b).at(1)).sine;
  result.opposite_shoulder_signs =
      Sign(result.first_shoulder_sine) * Sign(result.last_shoulder_sine) == -1;
  result.shoulder_wall_required = result.opposite_shoulder_signs;
  if (first.problem.Data().identity != last.problem.Data().identity) {
    return result;
  }
  // Nominal canonical base flip (modulo exact 2*pi turns):
  // (q0+pi,-q1,pi-q2,q3+pi,q4,q5). Its entire image must lie in
  // the other root's uniqueness region for this identical exact target.
  box.at(0) += interval::Pi();
  box.at(1) = -box.at(1);
  box.at(2) = interval::Pi() - box.at(2);
  box.at(3) += interval::Pi();
  const auto period = Interval{2} * interval::Pi();
  for (std::size_t i = 0; i < box.size(); ++i) {
    const double proposal = std::nearbyint(
        (b.centre.at(i) - Midpoint(box.at(i))) / Midpoint(period));
    if (!std::isfinite(proposal) || std::abs(proposal) > 1099511627776.0) {
      return result;
    }
    result.flip_turns.at(i) = static_cast<std::int64_t>(proposal);
    box.at(i) += Interval{proposal} * period;
  }
  result.flip_image = box;
  result.flip_inclusion = Inclusion(box, b);
  result.last_uniqueness_radius = b.uniqueness_radius;
  result.base_flip_verified = result.flip_inclusion.IsValid();
  result.shoulder_wall_required =
      result.shoulder_wall_required || result.base_flip_verified;
  return result;
}

auto Build(const Input &input, Options options) -> Graph {
  Graph result;
  auto &data = result.data_;
  data.raw_model = input.model;
  data.raw_limits = input.limits;
  data.convention = input.convention;
  data.options = options;
  if (!Valid(input, options)) {
    return result;
  }
  if (!interval::EnvironmentSupported()) {
    data.status = Status::UnsupportedEnvironment;
    return result;
  }
  for (std::size_t i = 0; i < input.tasks.size(); ++i) {
    PrepareTask(input, i, data);
  }
  ScanPairs(data, input);
  Connect(data, input);
  Components(data);
  return result;
}
} // namespace crx::branch
