// SPDX-License-Identifier: Apache-2.0
// SPDX-FileCopyrightText: 2026 Centre Technologique en Aérospatiale

#include "crx_path_tracking.h"
#include "crx_ball_certificate.h"
#include "crx_command_certificate.h"
#include "crx_exact_problem.h"
#include "crx_interval.h"
#include "crx_path_certificate.h"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <utility>
#include <vector>

#ifdef __FAST_MATH__
#error "Path proof orchestration requires strict floating-point compilation"
#endif

namespace crx::path {
namespace {
constexpr std::uint32_t kMaxNodes = 65535;
constexpr std::uint32_t kMaxDepth = 48;
constexpr std::uint32_t kMaxCertificates = 1U << 20U;
constexpr std::uint32_t kMaxTrials = 128;

auto Midpoint(Interval x) -> double { return x.lo / 2 + x.hi / 2; }

auto Valid(TrackingOptions options) -> bool {
  return options.max_nodes <= kMaxNodes && options.max_depth <= kMaxDepth &&
         options.max_point_certificates <= kMaxCertificates &&
         options.max_trials_per_node > 0 &&
         options.max_trials_per_node <= kMaxTrials;
}

auto Inclusion(const ball::Centre &first, double rho, const ball::Centre &next,
               double radius) -> Interval {
  double upper = 0;
  for (std::size_t i = 0; i < first.size(); ++i) {
    const auto distance =
        interval::Abs(Interval{first.at(i)} - Interval{next.at(i)});
    if (!distance.IsValid()) {
      return {};
    }
    upper = std::max(upper, distance.hi);
  }
  const auto bound = Interval{0, upper} + Interval{rho};
  return bound.IsValid() && bound.hi <= radius ? bound : Interval{};
}

struct Node {
  Interval parameter;
  std::uint32_t depth = 0;
};

struct State {
  const Prepared &path;
  const RootWitness &first;
  const RootWitness &last;
  const command::Context &limits;
  TrackingData &data;
  ball::Centre preferred;

  auto Trial(const Node &node, const exact::Problem &problem,
             const ball::Centre &centre, PathLeaf &leaf) -> bool {
    if (data.work.point_certificates >= data.options.max_point_certificates) {
      leaf.failure = TrackingFailure::WorkLimit;
      return false;
    }
    ++data.work.point_certificates;
    const auto point = ball::Certify(problem, centre);
    if (!point.IsCertified()) {
      leaf.failure = TrackingFailure::PointProof;
      leaf.point_failure = point.failure;
      return false;
    }
    ++data.work.cell_checks;
    const auto cell =
        Certify(path, node.parameter, problem, point.certificate, limits);
    leaf.cell = cell.certificate;
    leaf.cell_failure = cell.failure;
    leaf.point_failure = cell.point_failure;
    if (!cell.IsCertified()) {
      leaf.failure = TrackingFailure::CellProof;
      return false;
    }
    const auto &bounds = point.certificate.Values();
    // Refinement policy, not an exclusion theorem: reserve room for the next
    // uniqueness-ball inclusion. The final cell instead has its actual endpoint
    // inclusion checked below and needs no reserve for a following cell.
    const auto reserve =
        Interval{4} * Interval{cell.certificate.Values().existence_radius};
    if (node.parameter.hi < 1 &&
        (!reserve.IsValid() || reserve.hi > bounds.uniqueness_radius)) {
      leaf.failure = TrackingFailure::ContinuationReserve;
      return false;
    }
    if (node.parameter.lo == 0) {
      const auto &start = first.certificate.Values();
      const auto inclusion = Inclusion(start.centre, start.existence_radius,
                                       bounds.centre, bounds.uniqueness_radius);
      if (!inclusion.IsValid()) {
        leaf.failure = TrackingFailure::InitialInclusion;
        return false;
      }
      data.initial_inclusion = inclusion;
      data.initial_uniqueness_radius = bounds.uniqueness_radius;
    } else if (!data.leaves.empty() && data.leaves.back().IsAdmitted() &&
               data.leaves.back().parameter.hi == node.parameter.lo) {
      const auto glued = Glue(data.leaves.back().cell, cell.certificate);
      if (!glued.IsCertified()) {
        leaf.failure = TrackingFailure::JunctionInclusion;
        return false;
      }
    }
    if (node.parameter.hi == 1) {
      const auto &end = last.certificate.Values();
      const auto inclusion =
          Inclusion(bounds.centre, cell.certificate.Values().existence_radius,
                    end.centre, end.uniqueness_radius);
      if (!inclusion.IsValid()) {
        leaf.failure = TrackingFailure::FinalInclusion;
        return false;
      }
      data.final_inclusion = inclusion;
      data.final_uniqueness_radius = end.uniqueness_radius;
    }
    leaf.failure = TrackingFailure::None;
    return true;
  }

  auto Check(const Node &node) -> PathLeaf {
    PathLeaf leaf;
    leaf.parameter = node.parameter;
    leaf.depth = node.depth;
    Enclosure enclosed;
    const auto domain = Enclose(path, node.parameter, enclosed);
    if (domain != Failure::None) {
      leaf.failure = TrackingFailure::CellProof;
      leaf.cell_failure = domain;
      return leaf;
    }
    exact::Problem problem;
    if (!PrepareNominationTarget(path, Midpoint(node.parameter), problem)) {
      leaf.failure = TrackingFailure::NumericalTarget;
      return leaf;
    }
    if (Trial(node, problem, preferred, leaf)) {
      return leaf;
    }
    if (leaf.failure == TrackingFailure::WorkLimit ||
        data.options.max_trials_per_node == 1) {
      return leaf;
    }
    const auto proposed = Nominate(path, Midpoint(node.parameter), preferred);
    if (proposed.ready) {
      ++data.work.nearby_calls;
      data.work.full_discovery_calls += proposed.used_full_discovery ? 1 : 0;
      data.work.nominations += proposed.centres.size();
    }
    std::uint32_t tried = 1;
    for (const auto &centre : proposed.centres) {
      if (tried++ >= data.options.max_trials_per_node) {
        break;
      }
      if (Trial(node, proposed.point_problem, centre, leaf)) {
        return leaf;
      }
      if (leaf.failure == TrackingFailure::WorkLimit) {
        break;
      }
    }
    return leaf;
  }
};
} // namespace

auto Track(const Prepared &path, const RootWitness &first,
           const RootWitness &last, const command::Context &limits,
           TrackingOptions options) -> TrackingCertificate {
  TrackingCertificate result;
  auto &data = result.data_;
  data.options = options;
  if (!interval::EnvironmentSupported()) {
    data.failure = TrackingFailure::UnsupportedEnvironment;
    return result;
  }
  if (!path.IsReady() || !first.IsReady() || !last.IsReady() ||
      !limits.IsReady() || !Valid(options)) {
    return result;
  }
  auto end_source = path.Values().source.source;
  end_source.target = path.Values().source.last;
  exact::Problem end_problem;
  if (exact::Prepare(end_source, path.Values().source.convention,
                     end_problem) != exact::Status::Ready ||
      first.problem.Data().identity !=
          path.Values().first_problem.Data().identity ||
      last.problem.Data().identity != end_problem.Data().identity ||
      !exact::SameModel(limits.Values().problem_identity,
                        first.problem.Data().identity)) {
    data.failure = TrackingFailure::EndpointIdentity;
    return result;
  }
  data.path_identity = path.Values().identity;
  data.first = first;
  data.last = last;
  data.limits = limits;
  if (first.certificate.Values().aspect_sign !=
      last.certificate.Values().aspect_sign) {
    data.failure = TrackingFailure::OppositeRegularSigns;
    PathLeaf leaf;
    leaf.parameter = Interval{0, 1};
    leaf.failure = data.failure;
    data.leaves.push_back(leaf);
    data.uncovered_measure = Interval{1};
    return result;
  }
  std::vector<Node> pending{{Interval{0, 1}, 0}};
  State state{path,   first, last,
              limits, data,  first.certificate.Values().centre};
  while (!pending.empty()) {
    const auto node = pending.back();
    pending.pop_back();
    PathLeaf leaf;
    if (data.work.nodes >= options.max_nodes ||
        data.work.point_certificates >= options.max_point_certificates) {
      leaf.parameter = node.parameter;
      leaf.depth = node.depth;
      leaf.failure = TrackingFailure::WorkLimit;
    } else {
      ++data.work.nodes;
      data.work.greatest_depth = std::max(data.work.greatest_depth, node.depth);
      leaf = state.Check(node);
      if (!leaf.IsAdmitted() && node.depth < options.max_depth &&
          leaf.failure != TrackingFailure::WorkLimit) {
        const double mid = Midpoint(node.parameter);
        if (mid > node.parameter.lo && mid < node.parameter.hi) {
          ++data.work.subdivisions;
          pending.push_back({Interval{mid, node.parameter.hi}, node.depth + 1});
          pending.push_back({Interval{node.parameter.lo, mid}, node.depth + 1});
          continue;
        }
        leaf.failure = TrackingFailure::ResolutionLimit;
      }
    }
    if (leaf.IsAdmitted()) {
      if (!data.leaves.empty() && data.leaves.back().IsAdmitted()) {
        const auto glued = Glue(data.leaves.back().cell, leaf.cell);
        if (glued.IsCertified()) {
          data.links.push_back(
              {data.leaves.size() - 1, data.leaves.size(), glued.certificate});
        } else {
          leaf.failure = TrackingFailure::JunctionInclusion;
        }
      }
      if (leaf.IsAdmitted()) {
        state.preferred = leaf.cell.Values().point_witness.Values().centre;
      }
    }
    data.leaves.push_back(std::move(leaf));
  }
  data.uncovered_measure = Interval{0};
  bool complete =
      !data.leaves.empty() && data.leaves.front().parameter.lo == 0 &&
      data.leaves.back().parameter.hi == 1 &&
      data.initial_inclusion.IsValid() && data.final_inclusion.IsValid();
  double end = 0;
  for (const auto &leaf : data.leaves) {
    complete = complete && leaf.IsAdmitted() && leaf.parameter.lo == end;
    end = leaf.parameter.hi;
    if (!leaf.IsAdmitted()) {
      data.uncovered_measure +=
          Interval{leaf.parameter.hi} - Interval{leaf.parameter.lo};
    }
  }
  complete = complete && data.links.size() + 1 == data.leaves.size();
  data.failure = complete ? TrackingFailure::None : TrackingFailure::CellProof;
  if (!complete) {
    for (const auto &leaf : data.leaves) {
      if (!leaf.IsAdmitted()) {
        data.failure = leaf.failure;
        break;
      }
    }
  }
  result.complete_ = complete;
  return result;
}
} // namespace crx::path
