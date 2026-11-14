// SPDX-License-Identifier: Apache-2.0
// SPDX-FileCopyrightText: 2026 Centre Technologique en Aérospatiale

#include "crx_ball_certificate.h"
#include "crx_canonical.h"
#include "crx_discovery.h"
#include "crx_exact_problem.h"
#include "crx_interval.h"
#include "crx_path_certificate.h"
#include "crx_path_tracking.h"
#include "crx_types.h"

#include <Eigen/Core>
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

#include <Eigen/Geometry>

namespace crx::path {
namespace {
constexpr double kTurn = 6.283185307179586476925286766559;
constexpr double kMaximumTurn = 0x1p40;

auto Midpoint(Interval x) -> double { return x.lo / 2 + x.hi / 2; }

auto Finite(const ball::Centre &centre) -> bool {
  return std::all_of(centre.begin(), centre.end(),
                     [](double x) { return std::isfinite(x); });
}

auto Distance(const ball::Centre &first, const ball::Centre &second) -> double {
  double result = 0;
  for (std::size_t i = 0; i < first.size(); ++i) {
    result = std::max(result, std::abs(first.at(i) - second.at(i)));
  }
  return result;
}

auto PointSource(const Prepared &path, double parameter, exact::Problem &output)
    -> bool {
  auto source = path.Values().source.source;
  const auto &last = path.Values().source.last;
  for (std::size_t i = 0; i < source.target.quaternion.size(); ++i) {
    source.target.quaternion.at(i) =
        (1 - parameter) * source.target.quaternion.at(i) +
        parameter * last.quaternion.at(i);
  }
  for (std::size_t i = 0; i < source.target.position.size(); ++i) {
    source.target.position.at(i) =
        (1 - parameter) * source.target.position.at(i) +
        parameter * last.position.at(i);
  }
  // This approximate display matrix does not define the exact point rotation.
  // It is retained only as the ordinary input-deviation diagnostic provenance.
  double scale = 0;
  for (double value : source.target.quaternion) {
    scale = std::max(scale, std::abs(value));
  }
  if (!std::isfinite(scale) || scale <= 0) {
    return false;
  }
  const auto &q = source.target.quaternion;
  Eigen::Quaterniond quaternion(q.at(0) / scale, q.at(1) / scale,
                                q.at(2) / scale, q.at(3) / scale);
  quaternion.normalize();
  const Eigen::Matrix3d rotation = quaternion.toRotationMatrix();
  if (!rotation.allFinite()) {
    return false;
  }
  for (Eigen::Index r = 0; r < 3; ++r) {
    for (Eigen::Index c = 0; c < 3; ++c) {
      source.target.input_rotation.at(static_cast<std::size_t>(r * 3 + c)) =
          rotation(r, c);
    }
  }
  return exact::Prepare(source, path.Values().source.convention, output) ==
         exact::Status::Ready;
}

template <typename Candidates>
void Retain(const Candidates &discovery, const ball::Centre &preferred,
            std::vector<ball::Centre> &output) {
  for (std::size_t i = 0; i < discovery.count; ++i) {
    ball::Centre centre;
    bool ready = true;
    for (std::size_t j = 0; j < centre.size(); ++j) {
      const double phase =
          discovery.candidates.at(i).joints(static_cast<Eigen::Index>(j));
      const double turns = std::nearbyint((preferred.at(j) - phase) / kTurn);
      if (!std::isfinite(turns) || std::abs(turns) > kMaximumTurn) {
        ready = false;
        break;
      }
      centre.at(j) = phase + kTurn * turns;
    }
    if (ready && Finite(centre)) {
      output.push_back(centre);
    }
  }
}
} // namespace

auto PrepareNominationTarget(const Prepared &path, double parameter,
                             exact::Problem &output) -> bool {
  if (!interval::EnvironmentSupported() || !path.IsReady() ||
      !std::isfinite(parameter) || parameter < 0 || parameter > 1) {
    return false;
  }
  return PointSource(path, parameter, output);
}

auto Nominate(const Prepared &path, double parameter,
              const ball::Centre &preferred) -> Nominations {
  Nominations result;
  if (!interval::EnvironmentSupported() || !path.IsReady() ||
      !std::isfinite(parameter) || parameter < 0 || parameter > 1 ||
      !Finite(preferred)) {
    return result;
  }
  if (!PointSource(path, parameter, result.point_problem)) {
    return result;
  }
  const auto &data = result.point_problem.Data();
  const canonical::Lengths lengths{
      Midpoint(data.lengths.a), Midpoint(data.lengths.b),
      Midpoint(data.lengths.c), Midpoint(data.lengths.r)};
  PoseIsoRT target = PoseIsoRT::Identity();
  for (Eigen::Index r = 0; r < 3; ++r) {
    target.translation()(r) = Midpoint(data.target.translation(r));
    for (Eigen::Index c = 0; c < 3; ++c) {
      target.linear()(r, c) = Midpoint(data.target.rotation(r, c));
    }
  }
  Vec6 seed;
  for (std::size_t j = 0; j < preferred.size(); ++j) {
    seed(static_cast<Eigen::Index>(j)) = preferred.at(j);
  }
  const auto nearby = canonical::DiscoverNearbyJointCandidates(
      lengths, target, {1e-12, 1e-12}, seed);
  result.numerical_status = static_cast<int>(nearby.status);
  Retain(nearby, preferred, result.centres);
  if (result.centres.empty()) {
    result.used_full_discovery = true;
    const auto full =
        canonical::DiscoverJointCandidates(lengths, target, {1e-12, 1e-12});
    result.numerical_status = static_cast<int>(full.status);
    Retain(full, preferred, result.centres);
  }
  std::sort(result.centres.begin(), result.centres.end(),
            [&](const auto &a, const auto &b) {
              const double first = Distance(a, preferred),
                           second = Distance(b, preferred);
              return first == second ? a < b : first < second;
            });
  result.centres.erase(
      std::unique(result.centres.begin(), result.centres.end()),
      result.centres.end());
  result.ready = true;
  return result;
}
} // namespace crx::path
