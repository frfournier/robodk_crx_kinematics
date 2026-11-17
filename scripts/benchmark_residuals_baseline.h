// SPDX-License-Identifier: Apache-2.0
// SPDX-FileCopyrightText: 2026 Centre Technologique en Aérospatiale

#pragma once

// Benchmark-only copies of the Identification checkpoint's geometric kernels.
// These are excluded from the library and have no private-worktree dependency.
// canonical.cpp SHA-256:
// 199e601cecc4a9548b9dffac184701a538bc85668f11b1ac0da6d89a7a7fed92
#include "crx_canonical.h"
#include "crx_numerics.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace residual_baseline {
using namespace crx;
using namespace crx::canonical;

struct Circle {
  Lengths lengths{};
  Vec3 p = Vec3::Zero();
  Vec3 v = Vec3::Zero();
  Vec3 w = Vec3::Zero();
  double length_scale = 0;
};

inline auto Prepare(const Lengths &lengths, const PoseIsoRT &target,
                    Circle &output) -> PreparationStatus {
  const Eigen::Array4d physical{lengths.a, lengths.b, lengths.c, lengths.r};
  const Mat3 gram = target.linear().transpose() * target.linear();
  if (!physical.allFinite() || numerics::IsRepresentedZero(lengths.a) ||
      numerics::IsRepresentedZero(lengths.b) || !target.matrix().allFinite() ||
      !(target.matrix().row(3) - Eigen::RowVector4d(0, 0, 0, 1))
           .isZero(numerics::kUnitTolerance) ||
      !(gram - Mat3::Identity()).isZero(numerics::kUnitTolerance) ||
      !numerics::Scalar(target.linear().determinant() - 1)
           .isZero(numerics::kUnitTolerance)) {
    return PreparationStatus::InvalidInput;
  }
  Circle prepared;
  prepared.length_scale = physical.abs().maxCoeff();
  const Eigen::Array4d normalized =
      physical / Eigen::Array4d::Constant(prepared.length_scale);
  if ((!physical.unaryExpr(&numerics::IsRepresentedZero) &&
       normalized.unaryExpr(&numerics::IsRepresentedZero))
          .any()) {
    return PreparationStatus::NumericalRangeFailure;
  }
  prepared.lengths = {normalized[0], normalized[1], normalized[2],
                      normalized[3]};
  prepared.v = target.linear().col(0);
  prepared.w = target.linear().col(1);
  prepared.p = target.translation() / prepared.length_scale -
               prepared.lengths.c * target.linear().col(2);
  if (!prepared.p.allFinite() || !std::isfinite(prepared.p.squaredNorm())) {
    return PreparationStatus::NumericalRangeFailure;
  }
  output = prepared;
  return PreparationStatus::Ready;
}

inline auto Pair(const Circle &circle, double angle) -> ResidualEvaluation {
  if (!std::isfinite(angle)) {
    const double nan = std::numeric_limits<double>::quiet_NaN();
    return {nan, nan, true};
  }
  const Vec3 axis = circle.v * std::cos(angle) + circle.w * std::sin(angle);
  const Vec3 axis_derivative =
      -circle.v * std::sin(angle) + circle.w * std::cos(angle);
  const Vec3 x = circle.p + circle.lengths.r * axis;
  const Vec3 x_derivative = circle.lengths.r * axis_derivative;
  const Vec3 n = Vec3::UnitZ().cross(x);
  const Vec3 n_derivative = Vec3::UnitZ().cross(x_derivative);
  const double a_squared = circle.lengths.a * circle.lengths.a;
  const double k =
      (a_squared + x.squaredNorm() - circle.lengths.b * circle.lengths.b) / 2;
  const double k_derivative = x.dot(x_derivative);
  const Vec3 axis_cross_n = axis.cross(n);
  const Vec3 axis_cross_n_derivative =
      axis_derivative.cross(n) + axis.cross(n_derivative);
  const double determinant = x.dot(axis_cross_n);
  const double determinant_derivative =
      x_derivative.dot(axis_cross_n) + x.dot(axis_cross_n_derivative);
  const double x_axis = x.dot(axis);
  const double x_axis_derivative =
      x_derivative.dot(axis) + x.dot(axis_derivative);
  const Vec3 n_cross_x = n.cross(x);
  const Vec3 n_cross_x_derivative =
      n_derivative.cross(x) + n.cross(x_derivative);
  const Vec3 numerator = k * axis_cross_n + x_axis * n_cross_x;
  const Vec3 numerator_derivative =
      k_derivative * axis_cross_n + k * axis_cross_n_derivative +
      x_axis_derivative * n_cross_x + x_axis * n_cross_x_derivative;
  return {numerator.squaredNorm() - a_squared * determinant * determinant,
          2 * numerator.dot(numerator_derivative) -
              2 * a_squared * determinant * determinant_derivative,
          true};
}

inline auto Cramer(const Circle &circle, double angle, CramerEvaluation &output)
    -> bool {
  if (!std::isfinite(angle)) {
    return false;
  }
  const Vec3 u = circle.v * std::cos(angle) + circle.w * std::sin(angle);
  const Vec3 du = -circle.v * std::sin(angle) + circle.w * std::cos(angle);
  const Vec3 x = circle.p + circle.lengths.r * u;
  const Vec3 dx = circle.lengths.r * du;
  const Vec3 n = Vec3::UnitZ().cross(x);
  const Vec3 dn = Vec3::UnitZ().cross(dx);
  const double x_squared = x.squaredNorm();
  if (!std::isfinite(x_squared) ||
      x_squared <=
          numerics::kDirectionTolerance * numerics::kDirectionTolerance) {
    return false;
  }
  const double a_squared = circle.lengths.a * circle.lengths.a;
  const double arm_difference = (circle.lengths.a - circle.lengths.b) *
                                (circle.lengths.a + circle.lengths.b);
  const double k = (arm_difference + x_squared) / 2;
  const double dk = x.dot(dx);
  const double m = x.dot(u);
  const double dm = dx.dot(u) + x.dot(du);
  const Vec3 un = u.cross(n);
  const Vec3 dun = du.cross(n) + u.cross(dn);
  const Vec3 nx = n.cross(x);
  const Vec3 dnx = dn.cross(x) + n.cross(dx);
  const double delta = x.dot(un);
  const double ddelta = dx.dot(un) + x.dot(dun);
  if (!std::isfinite(delta) ||
      !(std::abs(delta) >= numerics::kCramerGuard * x_squared)) {
    return false;
  }
  const Vec3 elbow = (k * un + m * nx) / delta;
  const Vec3 delbow =
      (dk * un + k * dun + dm * nx + m * dnx - elbow * ddelta) / delta;
  output = {elbow.squaredNorm() - a_squared, 2 * elbow.dot(delbow), elbow};
  return elbow.allFinite() && delbow.allFinite() &&
         std::isfinite(output.value) && std::isfinite(output.derivative);
}
} // namespace residual_baseline
