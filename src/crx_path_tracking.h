// SPDX-License-Identifier: Apache-2.0
// SPDX-FileCopyrightText: 2026 Centre Technologique en Aérospatiale

#pragma once

#include "crx_path_certificate.h"

#include <cstddef>
#include <cstdint>
#include <vector>

namespace crx::path {
enum class TrackingFailure {
  None,
  InvalidInput,
  UnsupportedEnvironment,
  EndpointIdentity,
  OppositeRegularSigns,
  WorkLimit,
  ResolutionLimit,
  NumericalTarget,
  NoNumericalCandidate,
  PointProof,
  CellProof,
  InitialInclusion,
  JunctionInclusion,
  FinalInclusion,
  ContinuationReserve
};

struct RootWitness {
  exact::Problem problem;
  ball::Certificate certificate;

  auto IsReady() const -> bool { return certificate.Matches(problem); }
};

struct TrackingOptions {
  std::uint32_t max_nodes = 1023;
  std::uint32_t max_depth = 32;
  std::uint32_t max_point_certificates = 8192;
  // Includes the previous-centre trial, followed by numerical nominations.
  std::uint32_t max_trials_per_node = 17;
};

struct Work {
  std::uint64_t nodes = 0;
  std::uint64_t point_certificates = 0;
  std::uint64_t cell_checks = 0;
  std::uint64_t nearby_calls = 0;
  std::uint64_t full_discovery_calls = 0;
  std::uint64_t nominations = 0;
  std::uint64_t subdivisions = 0;
  std::uint32_t greatest_depth = 0;
};

// A rounded target and numerical candidates nominate only. The represented
// point target need not equal the exact path at the parameter: its sealed
// inverse/region witness is checked against the whole exact target family.
// This separate numerical TU retains the ordinary proposal compilation policy.
struct Nominations {
  bool ready = false;
  exact::Problem point_problem;
  std::vector<ball::Centre> centres;
  int numerical_status = -1;
  bool used_full_discovery = false;
};

auto Nominate(const Prepared &path, double parameter,
              const ball::Centre &preferred) -> Nominations;

// Rounded midpoint target used only to obtain a sealed point-region witness.
// The uniform cell checker continues to use the original exact path family.
auto PrepareNominationTarget(const Prepared &path, double parameter,
                             exact::Problem &output) -> bool;

struct PathLeaf {
  Interval parameter;
  std::uint32_t depth = 0;
  TrackingFailure failure = TrackingFailure::InvalidInput;
  Failure cell_failure = Failure::None;
  ball::Failure point_failure = ball::Failure::None;
  CellCertificate cell;

  auto IsAdmitted() const -> bool {
    return failure == TrackingFailure::None && cell.IsAdmitted();
  }
};

struct PathLink {
  std::size_t first = 0, second = 0;
  LinkCertificate certificate;
};

struct TrackingData {
  Identity path_identity;
  RootWitness first, last;
  command::Context limits;
  TrackingOptions options;
  Work work;
  TrackingFailure failure = TrackingFailure::InvalidInput;
  std::vector<PathLeaf> leaves;
  std::vector<PathLink> links;
  Interval uncovered_measure;
  Interval initial_inclusion, final_inclusion;
  double initial_uniqueness_radius = 0;
  double final_uniqueness_radius = 0;
};

class TrackingCertificate {
public:
  auto IsComplete() const -> bool { return complete_; }

  auto Values() const -> const TrackingData & { return data_; }

private:
  friend auto Track(const Prepared &, const RootWitness &, const RootWitness &,
                    const command::Context &, TrackingOptions)
      -> TrackingCertificate;
  TrackingData data_;
  bool complete_ = false;
};

// Bounded adaptive continuation of ONE supplied branch. Every retained leaf
// is admitted over its whole interval or explicitly unresolved. Completion
// requires a full partition, checked gluing and exact-target endpoint-root
// membership. No sampled clearance, numerical convergence, sign or count can
// seal this certificate. Failure never establishes path/IK infeasibility.
// max_nodes=0 or max_point_certificates=0 intentionally does no new proof work.
// Limits: nodes<=65535, depth<=48, point certificates<=2^20, trials<=128.
auto Track(const Prepared &path, const RootWitness &first,
           const RootWitness &last, const command::Context &limits,
           TrackingOptions options = {}) -> TrackingCertificate;
} // namespace crx::path
