// SPDX-License-Identifier: Apache-2.0
// SPDX-FileCopyrightText: 2026 Centre Technologique en Aérospatiale

#include "crx_path_certificate.h"
#include "crx_ball_certificate.h"
#include "crx_command_certificate.h"
#include "crx_exact_problem.h"
#include "crx_interval.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>

#ifdef __FAST_MATH__
#error "Path certificates require strict floating-point compilation"
#endif

namespace crx::path {
namespace {
auto Word(double value) -> std::uint64_t {
  std::uint64_t output = 0;
  static_assert(sizeof(output) == sizeof(value));
  std::memcpy(&output, &value, sizeof(output));
  return output;
}

auto ValidParameter(Interval value) -> bool {
  return value.IsValid() && value.lo >= 0 && value.hi <= 1;
}

auto SameModel(const Prepared &path, const exact::Identity &identity) -> bool {
  return exact::SameModel(path.Values().first_problem.Data().identity,
                          identity);
}

auto Failed(Failure failure, ball::Failure point = ball::Failure::None)
    -> Attempt {
  Attempt result;
  result.failure = failure;
  result.point_failure = point;
  return result;
}

void CommandSlacks(const exact::Problem &problem,
                   const command::Context &limits, CellData &output) {
  const auto &context = limits.Values();
  const auto &model = problem.Data();
  const auto &centre = output.point_witness.Values().centre;
  std::array<Interval, 6> commands;
  for (std::size_t i = 0; i < commands.size(); ++i) {
    commands.at(i) = Interval{model.diagonal.at(i)} *
                     (Interval{centre.at(i)} - model.offset_unwrapped.at(i));
  }
  if (context.raw.profile == command::Profile::DecoupledUser) {
    commands.at(2) = commands.at(2) - commands.at(1);
  }
  output.profile = context.raw.profile;
  output.commands_admitted = true;
  for (std::size_t i = 0; i < commands.size(); ++i) {
    output.limit_words.at(i) = Word(context.raw.lower_degrees.at(i));
    output.limit_words.at(i + commands.size()) =
        Word(context.raw.upper_degrees.at(i));
    const Interval lower = commands.at(i) - context.lower.at(i);
    const Interval upper = context.upper.at(i) - commands.at(i);
    const double row_norm =
        context.raw.profile == command::Profile::DecoupledUser && i == 2 ? 2
                                                                         : 1;
    output.slack_lipschitz.at(i) = row_norm;
    const Interval reach =
        Interval{row_norm} * Interval{output.existence_radius};
    if (!lower.IsValid() || !upper.IsValid() || !reach.IsValid()) {
      output.command_slack_lower.at(i) =
          std::numeric_limits<double>::quiet_NaN();
      output.commands_admitted = false;
      continue;
    }
    output.command_slack_lower.at(i) = std::min(lower.lo, upper.lo);
    output.commands_admitted = output.commands_admitted &&
                               lower.lo >= reach.hi && upper.lo >= reach.hi;
  }
}
} // namespace

auto Prepare(const Input &input, Prepared &output) -> Failure {
  if (!interval::EnvironmentSupported()) {
    return Failure::UnsupportedEnvironment;
  }
  Prepared result;
  auto &data = result.data_;
  const auto first =
      exact::Prepare(input.source, input.convention, data.first_problem);
  if (first != exact::Status::Ready) {
    return first == exact::Status::UnsupportedEnvironment
               ? Failure::UnsupportedEnvironment
               : Failure::InvalidInput;
  }
  exact::Input last = input.source;
  last.target = input.last;
  exact::Problem last_problem;
  if (exact::Prepare(last, input.convention, last_problem) !=
      exact::Status::Ready) {
    return Failure::InvalidInput;
  }
  data.source = input;
  data.identity.frame_convention = static_cast<std::uint32_t>(input.convention);
  const auto &words = data.first_problem.Data().identity.words;
  std::copy(words.begin(), words.end(), data.identity.words.begin());
  std::size_t index = words.size();
  for (double value : input.last.quaternion) {
    data.identity.words.at(index++) = Word(value);
  }
  for (double value : input.last.position) {
    data.identity.words.at(index++) = Word(value);
  }
  for (double value : input.last.input_rotation) {
    data.identity.words.at(index++) = Word(value);
  }
  for (const auto *quaternion :
       {&input.source.target.quaternion, &input.last.quaternion}) {
    for (double value : *quaternion) {
      data.quaternion_scale = std::max(data.quaternion_scale, std::abs(value));
    }
  }
  if (!std::isfinite(data.quaternion_scale) || data.quaternion_scale <= 0) {
    return Failure::InvalidInput;
  }
  result.ready_ = true;
  output = result;
  return Failure::None;
}

auto Enclose(const Prepared &path, Interval parameter, Enclosure &output)
    -> Failure {
  if (!interval::EnvironmentSupported()) {
    return Failure::UnsupportedEnvironment;
  }
  if (!path.IsReady() || !ValidParameter(parameter)) {
    return Failure::InvalidInput;
  }
  const auto &data = path.Values();
  std::array<Interval, 4> quaternion;
  std::array<Interval, 3> position;
  const Interval scale{data.quaternion_scale};
  Interval norm{0};
  for (std::size_t i = 0; i < quaternion.size(); ++i) {
    const Interval first =
        Interval{data.source.source.target.quaternion.at(i)} / scale;
    const Interval last = Interval{data.source.last.quaternion.at(i)} / scale;
    quaternion.at(i) = first + parameter * (last - first);
    norm += interval::Square(quaternion.at(i));
  }
  if (!norm.IsValid() || norm.lo <= 0) {
    return Failure::QuaternionDomain;
  }
  for (std::size_t i = 0; i < position.size(); ++i) {
    const Interval first{data.source.source.target.position.at(i)};
    const Interval last{data.source.last.position.at(i)};
    position.at(i) = first + parameter * (last - first);
  }
  Enclosure result;
  if (exact::EncloseTargets(data.source.source, data.source.convention,
                            quaternion, position,
                            result.target) != exact::Status::Ready) {
    return Failure::TargetEnclosure;
  }
  result.quaternion_norm_squared = norm;
  output = result;
  return Failure::None;
}

auto Certify(const Prepared &path, Interval parameter,
             const exact::Problem &point_problem,
             const ball::Certificate &point_witness,
             const command::Context &limits) -> Attempt {
  if (!interval::EnvironmentSupported()) {
    return Failed(Failure::UnsupportedEnvironment);
  }
  if (!path.IsReady() || !ValidParameter(parameter) ||
      !point_problem.IsReady() || !point_witness.IsValid() ||
      !limits.IsReady()) {
    return Failed(Failure::InvalidInput);
  }
  if (!point_witness.Matches(point_problem) ||
      !SameModel(path, point_problem.Data().identity) ||
      !SameModel(path, limits.Values().problem_identity)) {
    return Failed(Failure::IdentityMismatch);
  }
  Enclosure enclosure;
  const auto enclosed = Enclose(path, parameter, enclosure);
  if (enclosed != Failure::None) {
    return Failed(enclosed);
  }
  CellData data;
  data.path_identity = path.Values().identity;
  data.parameter = parameter;
  data.quaternion_norm_squared = enclosure.quaternion_norm_squared;
  data.point_witness = point_witness;
  data.target_enclosure = enclosure.target;
  const auto residual = ball::EncloseResidual(point_problem, point_witness,
                                              enclosure.target, data.residual);
  if (residual != ball::Failure::None) {
    return Failed(Failure::ResidualBound, residual);
  }
  const auto &bounds = point_witness.Values();
  const Interval rho = Interval{2} * Interval{data.residual.eta};
  const Interval beta_l = Interval{bounds.beta} * Interval{bounds.lipschitz};
  const Interval contraction = beta_l * Interval{bounds.uniqueness_radius};
  const Interval angular =
      Interval{bounds.uniqueness_radius} * Interval{bounds.angular_norm_sum};
  const Interval polynomial =
      beta_l * interval::Square(Interval{rho.hi}) / Interval{2} +
      Interval{data.residual.eta};
  if (!rho.IsValid() || !beta_l.IsValid() || !contraction.IsValid() ||
      !angular.IsValid() || !polynomial.IsValid() ||
      rho.hi > bounds.uniqueness_radius || contraction.hi >= 1 ||
      angular.hi >= interval::Pi().lo || polynomial.hi > rho.hi) {
    return Failed(Failure::RadiusInequality);
  }
  data.existence_radius = rho.hi;
  CommandSlacks(point_problem, limits, data);
  Attempt result;
  result.certificate.data_ = data;
  result.certificate.valid_ = true;
  result.failure =
      data.commands_admitted ? Failure::None : Failure::CommandSlack;
  return result;
}

auto Glue(const CellCertificate &first, const CellCertificate &second)
    -> LinkAttempt {
  LinkAttempt result;
  if (!interval::EnvironmentSupported()) {
    result.failure = Failure::UnsupportedEnvironment;
    return result;
  }
  if (!first.IsAdmitted() || !second.IsAdmitted()) {
    return result;
  }
  const auto &before = first.Values();
  const auto &after = second.Values();
  if (before.path_identity != after.path_identity ||
      before.profile != after.profile ||
      before.limit_words != after.limit_words) {
    result.failure = Failure::IdentityMismatch;
    return result;
  }
  if (before.parameter.hi != after.parameter.lo) {
    result.failure = Failure::JunctionMismatch;
    return result;
  }
  const auto &a = before.point_witness.Values();
  const auto &b = after.point_witness.Values();
  double distance = 0;
  for (std::size_t i = 0; i < a.centre.size(); ++i) {
    const auto difference =
        interval::Abs(Interval{a.centre.at(i)} - Interval{b.centre.at(i)});
    if (!difference.IsValid()) {
      result.failure = Failure::BallInclusion;
      return result;
    }
    distance = std::max(distance, difference.hi);
  }
  const Interval inclusion =
      Interval{distance} + Interval{before.existence_radius};
  if (!inclusion.IsValid() || inclusion.hi > b.uniqueness_radius) {
    result.failure = Failure::BallInclusion;
    return result;
  }
  auto &data = result.certificate.data_;
  data.path_identity = before.path_identity;
  data.first_parameter = before.parameter;
  data.second_parameter = after.parameter;
  data.centre_distance = Interval{0, distance};
  data.inclusion_bound = inclusion;
  data.next_uniqueness_radius = b.uniqueness_radius;
  result.certificate.valid_ = true;
  result.failure = Failure::None;
  return result;
}
} // namespace crx::path
