// SPDX-License-Identifier: Apache-2.0
// SPDX-FileCopyrightText: 2026 Centre Technologique en Aérospatiale

#include "crx_verified_solver.h"
#include "crx_numerics.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace crx {
namespace {
auto ValidFrame(const PoseIsoRT &pose) -> bool {
  if (!pose.matrix().allFinite() || pose.matrix()(3, 0) != 0 ||
      pose.matrix()(3, 1) != 0 || pose.matrix()(3, 2) != 0 ||
      pose.matrix()(3, 3) != 1) {
    return false;
  }
  const Mat3 rotation = pose.linear();
  return (rotation.transpose() * rotation - Mat3::Identity()).norm() <=
             numerics::kUnitTolerance &&
         std::abs(rotation.determinant() - 1) <= numerics::kUnitTolerance;
}

auto ValidRaw(const command::Input &input) -> bool {
  if (input.profile != command::Profile::Callback &&
      input.profile != command::Profile::DecoupledUser) {
    return false;
  }
  for (std::size_t i = 0; i < input.lower_degrees.size(); ++i) {
    if (!std::isfinite(input.lower_degrees.at(i)) ||
        !std::isfinite(input.upper_degrees.at(i)) ||
        input.lower_degrees.at(i) > input.upper_degrees.at(i) ||
        !std::isfinite(input.seed_degrees.at(i)) ||
        !std::isfinite(input.weights.at(i)) || input.weights.at(i) <= 0) {
      return false;
    }
  }
  return true;
}

auto NominalCost(const Vec6 &value, const command::Input &raw) -> double {
  double result = 0;
  for (std::size_t i = 0; i < raw.weights.size(); ++i) {
    const double delta = value(static_cast<Eigen::Index>(i)) -
                         raw.seed_degrees.at(i) * angle_conv::kRadPerDeg;
    result += raw.weights.at(i) * delta * delta;
  }
  return std::isfinite(result) ? result
                               : std::numeric_limits<double>::infinity();
}

auto NumericalFallback(const CrxModelData &model, const PoseIsoRT &flange,
                       const command::Input &raw, std::size_t capacity)
    -> IkResult {
  if (capacity == 0) {
    IkResult empty;
    empty.status = IkStatus::NoOutputRequested;
    return empty;
  }
  auto bounded = model;
  Vec6 seed;
  for (std::size_t i = 0; i < raw.lower_degrees.size(); ++i) {
    const auto axis = static_cast<Eigen::Index>(i);
    bounded.lower_limits_rad(axis) =
        raw.lower_degrees.at(i) * angle_conv::kRadPerDeg;
    bounded.upper_limits_rad(axis) =
        raw.upper_degrees.at(i) * angle_conv::kRadPerDeg;
    seed(axis) = raw.seed_degrees.at(i) * angle_conv::kRadPerDeg;
  }
  const bool decoupled = raw.profile == command::Profile::DecoupledUser;
  if (decoupled) {
    // This rectangle encloses the callback parallelogram. Filter in the actual
    // user box after inverse shear; no completeness follows from this search.
    bounded.lower_limits_rad(2) += bounded.lower_limits_rad(1);
    bounded.upper_limits_rad(2) += bounded.upper_limits_rad(1);
    seed(2) += seed(1);
  }
  constexpr int kNumericalCapacity = 512;
  auto result = SolveIkIsometry(bounded, flange, raw.has_seed ? &seed : nullptr,
                                kNumericalCapacity);
  if (decoupled) {
    for (auto &value : result.solutions) {
      value(2) -= value(1);
    }
    result.solutions.erase(
        std::remove_if(result.solutions.begin(), result.solutions.end(),
                       [&raw](const Vec6 &value) {
                         for (std::size_t i = 0; i < raw.lower_degrees.size();
                              ++i) {
                           const double degrees =
                               value(static_cast<Eigen::Index>(i)) *
                               angle_conv::kDegPerRad;
                           if (degrees < raw.lower_degrees.at(i) ||
                               degrees > raw.upper_degrees.at(i)) {
                             return true;
                           }
                         }
                         return false;
                       }),
        result.solutions.end());
    if (result.solutions.empty() &&
        (result.status == IkStatus::StrictCandidates ||
         result.status == IkStatus::StrictSeed ||
         result.status == IkStatus::StrictLocal ||
         result.status == IkStatus::ApproximateSeed)) {
      result.status = IkStatus::NoCandidate;
    }
  }
  if (raw.has_seed) {
    std::stable_sort(result.solutions.begin(), result.solutions.end(),
                     [&raw](const Vec6 &a, const Vec6 &b) {
                       return NominalCost(a, raw) < NominalCost(b, raw);
                     });
  }
  result.truncated = result.truncated || result.solutions.size() > capacity;
  if (result.solutions.size() > capacity) {
    result.solutions.resize(capacity);
  }
  return result;
}
} // namespace

auto QueryVerified(const CrxModelData &model, const PoseIsoRT &target,
                   const PoseIsoRT &reference, const PoseIsoRT &tool,
                   const command::Input &raw, VerifiedOptions options)
    -> VerifiedQuery {
  VerifiedQuery result;
  result.raw = raw;
  if (!ValidRaw(raw) || !ValidFrame(target) || !ValidFrame(reference) ||
      !ValidFrame(tool)) {
    return result;
  }
  const PoseIsoRT flange = reference * target * tool.inverse();
  if (!flange.matrix().allFinite()) {
    return result;
  }
  result.input_valid = true;
  result.numerical =
      NumericalFallback(model, flange, raw, options.commands.capacity);
  if (!options.enabled) {
    return result;
  }
  result.attempted = true;
  const Eigen::Quaterniond quaternion(target.linear());
  result.preparation = exact::PrepareReference(model, target, quaternion,
                                               reference, tool, result.problem);
  if (result.preparation != exact::Status::Ready) {
    return result;
  }
  command::Context context;
  result.context_failure = command::Prepare(result.problem, raw, context);
  if (result.context_failure != command::Failure::None) {
    return result;
  }
  result.phases = catalogue::Build(result.problem, options.phases);
  result.commands = command::Certify(result.problem, result.phases.catalogue,
                                     context, options.commands);
  return result;
}
} // namespace crx
