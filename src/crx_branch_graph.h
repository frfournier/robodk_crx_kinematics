// SPDX-License-Identifier: Apache-2.0
// SPDX-FileCopyrightText: 2026 Centre Technologique en Aérospatiale

#pragma once

#include "crx_path_tracking.h"

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace crx::branch {
// These are necessary crossing conditions, never existence or exclusion
// certificates for a connecting path. Same Jacobian sign alone implies none.
struct Relation {
  bool comparable = false;
  bool singularity_required = false;
  bool base_flip_verified = false;
  bool shoulder_wall_required = false;
  bool opposite_shoulder_signs = false;
  interval::Interval first_shoulder_sine, last_shoulder_sine;
  std::array<std::int64_t, 6> flip_turns{};
  command::Box flip_image;
  interval::Interval flip_inclusion;
  double last_uniqueness_radius = 0;
};

auto Classify(const path::RootWitness &first, const path::RootWitness &last)
    -> Relation;

struct Task {
  std::string id;
  exact::PoseInput target;
};

struct Connection {
  std::size_t first = 0, last = 0;
};

struct Input {
  exact::Input model;
  exact::FrameConvention convention = exact::FrameConvention::BasePose;
  command::Input limits;
  std::vector<Task> tasks;
  std::vector<Connection> connections;
};

struct Options {
  std::uint32_t max_vertices = 512;
  std::uint32_t max_pair_tests = 4096;
  std::uint32_t max_paths = 32;
  // Shared across all tasks or path attempts, respectively. Zero budgets
  // retain incomplete tasks/links and cannot establish absence of solutions.
  std::uint32_t max_root_nodes = 32768;
  std::uint32_t max_endpoint_certificates = 4096;
  std::uint32_t max_lifts = 4096;
  std::uint32_t max_path_nodes = 4096;
  std::uint32_t max_path_certificates = 32768;
  catalogue::Options endpoint;
  path::TrackingOptions tracking;
};

struct Work {
  std::uint64_t root_nodes = 0, endpoint_certificates = 0, enumerated_lifts = 0;
  std::uint64_t pair_tests = 0, path_attempts = 0;
  std::uint64_t path_nodes = 0, path_certificates = 0, cell_checks = 0;
  std::uint64_t subdivisions = 0;
};

enum class Membership { Unbound, SourceBall, LiftBoxInUniqueness };

struct Vertex {
  std::size_t task = 0, phase = 0;
  bool has_command_lift = false;
  std::array<std::int64_t, 6> turns{};
  command::Box command_box, canonical_box;
  command::Admission admission = command::Admission::Undecided;
  Membership membership = Membership::Unbound;
  path::RootWitness root;
  interval::Interval inclusion;
  // A partial canonical root has its own constant-target command proof.
  path::CellCertificate partial_admission;

  auto IsUsable() const -> bool {
    return membership != Membership::Unbound && root.IsReady() &&
           admission == command::Admission::Yes;
  }
};

struct TaskData {
  Task source;
  exact::Problem problem;
  command::Context limits;
  catalogue::BuildResult phases;
  command::Certificate commands;
  std::vector<std::size_t> vertices;
  exact::Status preparation = exact::Status::InvalidInput;
  command::Failure limit_preparation = command::Failure::InvalidInput;
  bool prepared = false;
  bool vertices_complete = false;
  bool certified_infeasible = false;
};

struct Pair {
  std::size_t connection = 0, first = 0, last = 0;
  Relation relation;
  bool attempted = false;
  std::size_t attempt = 0;
};

struct ConnectionData {
  Connection source;
  std::uint64_t total_pairs = 0, considered_pairs = 0;
  // The lexicographic scan stops at this cursor when its shared budget ends.
  std::size_t next_first = 0, next_last = 0;
  bool scan_complete = false;
};

struct Attempt {
  std::size_t pair = 0;
  path::TrackingCertificate certificate;
};

struct Component {
  std::vector<std::size_t> vertices;
  std::vector<std::size_t> admitted_tasks;
  bool covers_all_tasks = false;
};

enum class Status {
  InvalidInput,
  Inconclusive,
  CertifiedTaskCoverage,
  CertifiedTaskInfeasible,
  UnsupportedEnvironment
};

struct Data {
  exact::Input raw_model;
  command::Input raw_limits;
  exact::FrameConvention convention = exact::FrameConvention::BasePose;
  Options options;
  Work work;
  Status status = Status::InvalidInput;
  std::vector<TaskData> tasks;
  std::vector<Vertex> vertices;
  std::vector<ConnectionData> connections;
  std::vector<Pair> pairs;
  std::vector<Attempt> attempts;
  // Contains indices into attempts; each listed certificate is complete.
  std::vector<std::size_t> edges;
  std::vector<Component> components;
};

class Graph {
public:
  auto Values() const -> const Data & { return data_; }

private:
  friend auto Build(const Input &, Options) -> Graph;
  Data data_;
};

// At most 64 tasks, 256 requested connections, 4096 vertices, 65536 pair
// tests, 4096 path attempts, 65535 shared root/path nodes and 2^20 shared path
// point proofs. Endpoint roots/lifts always come from the declared exact raw
// target and raw degree limits. An edge needs a complete whole-path proof.
// Components describe this certified subgraph only. Positive task coverage
// means a finite walk can visit the supplied tasks; it asserts no exhaustive
// branch graph, global aspect connectivity, optimality or dynamics claim.
auto Build(const Input &input, Options options = {}) -> Graph;
} // namespace crx::branch
