// SPDX-License-Identifier: Apache-2.0
// SPDX-FileCopyrightText: 2026 Centre Technologique en Aérospatiale

#include "crx_calibrated_solver.h"

#include <algorithm>
#include <cmath>
#include <limits>

#include <Eigen/Geometry>
#include <Eigen/LU>

namespace crx::calibrated {
namespace {
using PointVector6 = Eigen::Matrix<double, 6, 1>;

auto Finite(const Centre &values) -> bool {
  return std::all_of(values.begin(), values.end(),
                     [](double value) { return std::isfinite(value); });
}

auto Finite(const PointPose &pose) -> bool {
  return pose.rotation.allFinite() && pose.translation.allFinite();
}

auto Trig(Angle angle) -> std::array<double, 2> {
  const double sine = std::sin(angle.radians), cosine = std::cos(angle.radians);
  switch (((angle.quarters % 4) + 4) % 4) {
  case 0:
    return {sine, cosine};
  case 1:
    return {cosine, -sine};
  case 2:
    return {-sine, -cosine};
  default:
    return {-cosine, sine};
  }
}

auto PointFrame(const Frame &input, double length_scale) -> PointPose {
  double maximum = 0;
  for (double value : input.quaternion)
    maximum = std::max(maximum, std::abs(value));
  Eigen::Quaterniond q(
      input.quaternion.at(0) / maximum, input.quaternion.at(1) / maximum,
      input.quaternion.at(2) / maximum, input.quaternion.at(3) / maximum);
  q.normalize();
  PointPose result;
  result.rotation = q.toRotationMatrix();
  for (std::size_t i = 0; i < input.position.size(); ++i)
    result.translation(static_cast<Eigen::Index>(i)) =
        input.position.at(i) / length_scale;
  return result;
}

auto Compose(const PointPose &left, const PointPose &right) -> PointPose {
  return {left.rotation * right.rotation,
          left.translation + left.rotation * right.translation};
}

auto Residual(const PointPose &actual, const PointPose &target)
    -> PointVector6 {
  const Eigen::Matrix3d relative =
      actual.rotation.transpose() * target.rotation;
  Eigen::Quaterniond q(relative);
  q.normalize();
  if (q.w() < 0)
    q.coeffs() = -q.coeffs();
  const double sine = q.vec().norm();
  Eigen::Vector3d angular;
  if (sine > std::numeric_limits<double>::epsilon())
    angular = (2 * std::atan2(sine, q.w()) / sine) * q.vec();
  else
    angular = 2 * q.vec();
  PointVector6 result;
  result.head<3>() =
      actual.rotation.transpose() * (target.translation - actual.translation);
  result.tail<3>() = angular;
  return result;
}

auto Valid(const CorrectionOptions &options) -> bool {
  return options.max_iterations <= 128 && options.max_backtracks <= 32 &&
         std::isfinite(options.maximum_step) && options.maximum_step > 0 &&
         std::isfinite(options.normalized_position_tolerance) &&
         options.normalized_position_tolerance >= 0 &&
         std::isfinite(options.orientation_tolerance) &&
         options.orientation_tolerance >= 0 &&
         std::isfinite(options.uniqueness_radius) &&
         options.uniqueness_radius >= 0;
}
} // namespace

auto ValidateCorrectionOptions(const CorrectionOptions &options) -> bool {
  return Valid(options);
}

auto PrepareSolver(const Model &model, Solver &output) -> Status {
  if (!model.IsReady())
    return Status::InvalidInput;
  Solver result;
  auto &data = result.data_;
  data.geometry = model;
  const auto &input = model.Data().source;
  data.base = PointFrame(input.base, input.length_scale);
  data.tool = PointFrame(input.tool, input.length_scale);
  if (!Finite(data.base) || !Finite(data.tool))
    return Status::ArithmeticFailure;
  for (std::size_t i = 0; i < input.rows.size(); ++i) {
    const auto &row = input.rows.at(i);
    const auto alpha = Trig(row.alpha), theta = Trig(row.theta);
    auto &fixed = data.fixed.at(i);
    fixed.rotation(1, 1) = alpha.at(1);
    fixed.rotation(1, 2) = -alpha.at(0);
    fixed.rotation(2, 1) = alpha.at(0);
    fixed.rotation(2, 2) = alpha.at(1);
    fixed.translation.x() = row.a / input.length_scale;
    data.theta_sine.at(i) = theta.at(0);
    data.theta_cosine.at(i) = theta.at(1);
    data.axial_distance.at(i) = row.d / input.length_scale;
    if (!Finite(fixed) || !std::isfinite(theta.at(0)) ||
        !std::isfinite(theta.at(1)) ||
        !std::isfinite(data.axial_distance.at(i)))
      return Status::ArithmeticFailure;
  }
  result.ready_ = true;
  output = result;
  return Status::Ready;
}

auto PrepareTarget(const Solver &solver, const Frame &target, Target &output)
    -> Status {
  if (!solver.IsReady())
    return Status::InvalidInput;
  Target result;
  const auto status =
      PrepareProblem(solver.Data().geometry, target, result.problem_);
  if (status != Status::Ready)
    return status;
  result.point_ =
      PointFrame(target, solver.Data().geometry.Data().source.length_scale);
  if (!Finite(result.point_))
    return Status::ArithmeticFailure;
  result.ready_ = true;
  output = result;
  return Status::Ready;
}

auto EvaluateNumerical(const Solver &solver, const Centre &increments,
                       PointKinematics &output) -> Status {
  if (!solver.IsReady() || !Finite(increments))
    return Status::InvalidInput;
  const auto &data = solver.Data();
  PointPose pose = data.base;
  std::array<Eigen::Vector3d, 6> axes, origins;
  for (std::size_t i = 0; i < increments.size(); ++i) {
    pose = Compose(pose, data.fixed.at(i));
    axes.at(i) = pose.rotation.col(2);
    origins.at(i) = pose.translation;
    const double sine = std::sin(increments.at(i)),
                 cosine = std::cos(increments.at(i));
    const double combined_sine =
        data.theta_sine.at(i) * cosine + data.theta_cosine.at(i) * sine;
    const double combined_cosine =
        data.theta_cosine.at(i) * cosine - data.theta_sine.at(i) * sine;
    Eigen::Matrix3d spin = Eigen::Matrix3d::Identity();
    spin(0, 0) = spin(1, 1) = combined_cosine;
    spin(0, 1) = -combined_sine;
    spin(1, 0) = combined_sine;
    pose.rotation = (pose.rotation * spin).eval();
    pose.translation += pose.rotation.col(2) * data.axial_distance.at(i);
  }
  PointKinematics result;
  result.pose = Compose(pose, data.tool);
  for (std::size_t i = 0; i < increments.size(); ++i) {
    const auto col = static_cast<Eigen::Index>(i);
    result.body_jacobian.block<3, 1>(0, col) =
        result.pose.rotation.transpose() *
        axes.at(i).cross(result.pose.translation - origins.at(i));
    result.body_jacobian.block<3, 1>(3, col) =
        result.pose.rotation.transpose() * axes.at(i);
  }
  if (!Finite(result.pose) || !result.body_jacobian.allFinite())
    return Status::ArithmeticFailure;
  output = result;
  return Status::Ready;
}

auto Correct(const Solver &solver, const Target &target,
             const Centre &seed_commands, CorrectionOptions options)
    -> Correction {
  Correction result;
  if (!solver.IsReady() || !target.IsReady() || !Finite(seed_commands) ||
      !Valid(options))
    return result;
  if (solver.Data().geometry.Data().identity !=
      target.Exact().Identity().model) {
    result.status = CorrectionStatus::ModelMismatch;
    return result;
  }
  const auto &input = solver.Data().geometry.Data().source;
  result.certificate_requested = options.certify;
  for (std::size_t i = 0; i < result.increments.size(); ++i) {
    result.increments.at(i) = input.senses.at(i) * seed_commands.at(i);
    result.commands.at(i) = seed_commands.at(i);
  }
  PointKinematics current;
  ++result.evaluations;
  if (EvaluateNumerical(solver, result.increments, current) != Status::Ready) {
    result.status = CorrectionStatus::ArithmeticFailure;
    return result;
  }
  for (;;) {
    const PointVector6 residual = Residual(current.pose, target.Point());
    if (!residual.allFinite()) {
      result.status = CorrectionStatus::ArithmeticFailure;
      return result;
    }
    result.normalized_position_error = residual.head<3>().norm();
    result.orientation_error = residual.tail<3>().norm();
    if (result.normalized_position_error <=
            options.normalized_position_tolerance &&
        result.orientation_error <= options.orientation_tolerance) {
      result.status = CorrectionStatus::Converged;
      result.numerical_limits_satisfied = true;
      for (std::size_t i = 0; i < result.commands.size(); ++i)
        result.numerical_limits_satisfied =
            result.numerical_limits_satisfied &&
            result.commands.at(i) >=
                solver.Data().geometry.Data().lower_commands.at(i).hi &&
            result.commands.at(i) <=
                solver.Data().geometry.Data().upper_commands.at(i).lo;
      if (options.certify) {
        result.certificate_attempted = true;
        result.local_proof = Certify(target.Exact(), result.increments,
                                     {options.uniqueness_radius, nullptr, 0});
      }
      return result;
    }
    if (result.iterations >= options.max_iterations) {
      result.status = CorrectionStatus::IterationBudget;
      return result;
    }
    const Eigen::FullPivLU<ball::PointMatrix6> factor(current.body_jacobian);
    if (!factor.isInvertible()) {
      result.status = CorrectionStatus::SingularStep;
      return result;
    }
    PointVector6 step = factor.solve(residual);
    if (!step.allFinite()) {
      result.status = CorrectionStatus::ArithmeticFailure;
      return result;
    }
    const double largest = step.cwiseAbs().maxCoeff();
    if (largest > options.maximum_step)
      step *= options.maximum_step / largest;
    const double old_cost = residual.norm();
    bool accepted = false;
    double fraction = 1;
    for (std::uint32_t back = 0; back <= options.max_backtracks; ++back) {
      Centre trial = result.increments;
      for (std::size_t i = 0; i < trial.size(); ++i)
        trial.at(i) += fraction * step(static_cast<Eigen::Index>(i));
      PointKinematics proposal;
      ++result.evaluations;
      if (EvaluateNumerical(solver, trial, proposal) == Status::Ready) {
        const PointVector6 next = Residual(proposal.pose, target.Point());
        if (next.allFinite() && next.norm() < old_cost) {
          result.increments = trial;
          current = proposal;
          for (std::size_t i = 0; i < trial.size(); ++i)
            result.commands.at(i) = input.senses.at(i) * trial.at(i);
          accepted = true;
          break;
        }
      }
      if (back < options.max_backtracks) {
        fraction *= .5;
        ++result.backtracks;
      }
    }
    if (!accepted) {
      result.status = CorrectionStatus::LineSearchFailure;
      return result;
    }
    ++result.iterations;
  }
}
} // namespace crx::calibrated
