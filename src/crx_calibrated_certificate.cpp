// SPDX-License-Identifier: Apache-2.0
// SPDX-FileCopyrightText: 2026 Centre Technologique en Aérospatiale

#include "crx_calibrated_certificate.h"

#include <algorithm>
#include <cstring>
#include <limits>

namespace crx::calibrated {
namespace {
auto Failed(ball::Failure failure) -> Attempt { return {failure, {}}; }
} // namespace

auto PrepareProblem(const Model &model, const Frame &target, Problem &output)
    -> Status {
  if (!interval::EnvironmentSupported())
    return Status::UnsupportedEnvironment;
  if (!model.IsReady())
    return Status::InvalidInput;
  Problem result;
  const auto status =
      EncloseFrame(target, model.Data().source.length_scale, result.target_);
  if (status != Status::Ready)
    return status;
  result.model_ = model;
  result.source_target_ = target;
  result.identity_.model = model.Data().identity;
  std::size_t index = 0;
  for (const auto value : target.quaternion) {
    std::memcpy(&result.identity_.target.at(index++), &value, sizeof(value));
  }
  for (const auto value : target.position) {
    std::memcpy(&result.identity_.target.at(index++), &value, sizeof(value));
  }
  result.ready_ = true;
  output = result;
  return Status::Ready;
}

auto Certify(const Problem &problem, const Centre &centre,
             ball::Request request) -> Attempt {
  if (!interval::EnvironmentSupported())
    return Failed(ball::Failure::UnsupportedEnvironment);
  if (!problem.IsReady())
    return Failed(ball::Failure::InvalidInput);
  ball::Kinematics kinematics;
  const auto evaluated = EvaluateCentre(problem.Geometry(), centre, kinematics);
  if (evaluated != Status::Ready) {
    return Failed(evaluated == Status::InvalidInput
                      ? ball::Failure::InvalidInput
                      : ball::Failure::KinematicsUnavailable);
  }
  Attempt result;
  auto &data = result.certificate.data_;
  const auto checked = ball::detail::CheckKinematics(
      problem.Target(), centre, kinematics, request, data.bounds);
  if (checked != ball::Failure::None)
    return Failed(checked);
  data.identity = problem.Identity();
  data.command_jacobian_sign = data.bounds.aspect_sign;
  data.command_reserve_lower = std::numeric_limits<double>::infinity();
  const auto &input = problem.Geometry().Data().source;
  for (std::size_t i = 0; i < centre.size(); ++i) {
    data.command_centre.at(i) = input.senses.at(i) * centre.at(i);
    if (input.senses.at(i) == -1)
      data.command_jacobian_sign = -data.command_jacobian_sign;
    const Interval low = Interval{data.command_centre.at(i)} -
                         problem.Geometry().Data().lower_commands.at(i);
    const Interval high = problem.Geometry().Data().upper_commands.at(i) -
                          Interval{data.command_centre.at(i)};
    if (!low.IsValid() || !high.IsValid())
      return Failed(ball::Failure::ArithmeticUnavailable);
    data.command_reserve_lower =
        std::min({data.command_reserve_lower, low.lo, high.lo});
  }
  data.commands_admitted =
      data.command_reserve_lower >= data.bounds.existence_radius;
  result.certificate.valid_ = true;
  result.failure = ball::Failure::None;
  return result;
}

auto TryReuse(const Certificate &previous, const Problem &problem) -> Attempt {
  if (!interval::EnvironmentSupported())
    return Failed(ball::Failure::UnsupportedEnvironment);
  if (!previous.IsValid() || !problem.IsReady())
    return Failed(ball::Failure::InvalidInput);
  const auto &old = previous.Values();
  if (old.identity.model != problem.Identity().model)
    return Failed(ball::Failure::IdentityMismatch);
  return Certify(problem, old.bounds.centre,
                 {old.bounds.uniqueness_radius, &old.bounds.preconditioner,
                  old.bounds.aspect_sign});
}
} // namespace crx::calibrated
