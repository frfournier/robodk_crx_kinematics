// SPDX-License-Identifier: Apache-2.0
// SPDX-FileCopyrightText: 2026 Centre Technologique en Aérospatiale

#include "crx_calibrated_model.h"

#include <algorithm>
#include <cmath>
#include <cstring>

namespace crx::calibrated {
namespace {
using Matrix3 = exact::Matrix3;
constexpr std::size_t kJoints = 6;

template <typename Derived>
auto Valid(const Eigen::MatrixBase<Derived> &matrix) -> bool {
  for (Eigen::Index r = 0; r < matrix.rows(); ++r) {
    for (Eigen::Index c = 0; c < matrix.cols(); ++c) {
      if (!matrix(r, c).IsValid())
        return false;
    }
  }
  return true;
}

auto Valid(const Pose &pose) -> bool {
  return Valid(pose.rotation) && Valid(pose.translation);
}

template <std::size_t N>
auto Finite(const std::array<double, N> &values) -> bool {
  return std::all_of(values.begin(), values.end(),
                     [](double value) { return std::isfinite(value); });
}

auto Valid(const Frame &frame) -> bool {
  return Finite(frame.quaternion) && Finite(frame.position) &&
         std::any_of(frame.quaternion.begin(), frame.quaternion.end(),
                     [](double value) { return value != 0; });
}

auto Compose(const Pose &left, const Pose &right) -> Pose {
  return {left.rotation * right.rotation,
          left.translation + left.rotation * right.translation};
}

auto Trig(Angle angle) -> interval::SinCos {
  const auto value = interval::EvaluateSinCos(Interval{angle.radians});
  const auto quadrant = ((angle.quarters % 4) + 4) % 4;
  switch (quadrant) {
  case 0:
    return value;
  case 1:
    return {value.cosine, -value.sine, value.used_full_range};
  case 2:
    return {-value.sine, -value.cosine, value.used_full_range};
  default:
    return {-value.cosine, value.sine, value.used_full_range};
  }
}

auto Spin(const interval::SinCos &theta, Interval increment) -> Matrix3 {
  const auto motion = interval::EvaluateSinCos(increment);
  const auto sine = theta.sine * motion.cosine + theta.cosine * motion.sine;
  const auto cosine = theta.cosine * motion.cosine - theta.sine * motion.sine;
  Matrix3 result = Matrix3::Identity();
  result(0, 0) = cosine;
  result(0, 1) = -sine;
  result(1, 0) = sine;
  result(1, 1) = cosine;
  return result;
}

auto Skew(const Vector3 &axis) -> Matrix3 {
  Matrix3 result;
  result << Interval{0}, -axis.z(), axis.y(), axis.z(), Interval{0}, -axis.x(),
      -axis.y(), axis.x(), Interval{0};
  return result;
}

auto BuildDh(const PreparedData &data,
             const std::array<Interval, 6> &increments, Pose &output,
             std::array<Vector3, 6> *axes, std::array<Vector3, 6> *origins)
    -> bool {
  Pose pose = data.base;
  const Interval scale{data.source.length_scale};
  for (std::size_t i = 0; i < kJoints; ++i) {
    if (!increments.at(i).IsValid())
      return false;
    pose = Compose(pose, data.fixed.at(i));
    if (axes != nullptr)
      axes->at(i) = pose.rotation.col(2);
    if (origins != nullptr)
      origins->at(i) = pose.translation;
    pose.rotation =
        (pose.rotation * Spin(data.theta.at(i), increments.at(i))).eval();
    pose.translation +=
        pose.rotation.col(2) * (Interval{data.source.rows.at(i).d} / scale);
  }
  pose = Compose(pose, data.tool);
  if (!Valid(pose))
    return false;
  output = pose;
  return true;
}

auto ModelIdentity(const Input &input) -> Identity {
  Identity result;
  std::size_t index = 0;
  const auto word = [&](double value) {
    std::uint64_t bits = 0;
    static_assert(sizeof(bits) == sizeof(value));
    std::memcpy(&bits, &value, sizeof(bits));
    result.words.at(index++) = bits;
  };
  for (const auto &row : input.rows) {
    for (const auto &angle : {row.alpha, row.theta}) {
      result.words.at(index++) =
          static_cast<std::uint64_t>(static_cast<std::int64_t>(angle.quarters));
      word(angle.radians);
    }
    word(row.a);
    word(row.d);
  }
  for (const auto value : input.senses)
    word(value);
  for (const auto &frame : {input.base, input.tool}) {
    for (const auto value : frame.quaternion)
      word(value);
    for (const auto value : frame.position)
      word(value);
  }
  for (const auto *values : {&input.lower_commands, &input.upper_commands}) {
    for (const auto value : *values)
      word(value);
  }
  word(input.length_scale);
  result.words.at(index++) = static_cast<std::uint64_t>(input.limit_units);
  return result;
}
} // namespace

auto EncloseFrame(const Frame &input, double length_scale, Pose &output)
    -> Status {
  if (!interval::EnvironmentSupported())
    return Status::UnsupportedEnvironment;
  if (!Valid(input) || !std::isfinite(length_scale) || length_scale <= 0)
    return Status::InvalidInput;
  double scale = 0;
  for (const auto value : input.quaternion)
    scale = std::max(scale, std::abs(value));
  std::array<Interval, 4> q;
  for (std::size_t i = 0; i < q.size(); ++i)
    q.at(i) = Interval{input.quaternion.at(i)} / Interval{scale};
  const auto &[w, x, y, z] = q;
  const auto ww = interval::Square(w), xx = interval::Square(x);
  const auto yy = interval::Square(y), zz = interval::Square(z);
  const auto norm = ((ww + xx) + yy) + zz;
  if (!norm.IsValid() || norm.lo <= 0)
    return Status::ArithmeticFailure;
  const Interval two{2};
  Pose result;
  result.rotation << (((ww + xx) - yy) - zz) / norm,
      two * (x * y - w * z) / norm, two * (x * z + w * y) / norm,
      two * (x * y + w * z) / norm, (((ww - xx) + yy) - zz) / norm,
      two * (y * z - w * x) / norm, two * (x * z - w * y) / norm,
      two * (y * z + w * x) / norm, (((ww - xx) - yy) + zz) / norm;
  for (Eigen::Index i = 0; i < 3; ++i)
    result.translation(i) =
        Interval{input.position.at(static_cast<std::size_t>(i))} /
        Interval{length_scale};
  if (!Valid(result))
    return Status::ArithmeticFailure;
  output = result;
  return Status::Ready;
}

auto Prepare(const Input &input, Model &output) -> Status {
  if (!interval::EnvironmentSupported())
    return Status::UnsupportedEnvironment;
  if (!std::isfinite(input.length_scale) || input.length_scale <= 0 ||
      !Finite(input.lower_commands) || !Finite(input.upper_commands) ||
      !Valid(input.base) || !Valid(input.tool) ||
      (input.limit_units != LimitUnits::Radians &&
       input.limit_units != LimitUnits::Degrees))
    return Status::InvalidInput;
  for (std::size_t i = 0; i < kJoints; ++i) {
    const auto &row = input.rows.at(i);
    if (!std::isfinite(row.alpha.radians) ||
        !std::isfinite(row.theta.radians) || !std::isfinite(row.a) ||
        !std::isfinite(row.d) ||
        (input.senses.at(i) != -1 && input.senses.at(i) != 1) ||
        input.lower_commands.at(i) > input.upper_commands.at(i))
      return Status::InvalidInput;
  }
  Model result;
  auto &data = result.data_;
  data.identity = ModelIdentity(input);
  data.source = input;
  const Interval conversion = input.limit_units == LimitUnits::Degrees
                                  ? interval::Pi() / Interval{180}
                                  : Interval{1};
  for (std::size_t i = 0; i < kJoints; ++i) {
    data.lower_commands.at(i) = Interval{input.lower_commands.at(i)};
    data.upper_commands.at(i) = Interval{input.upper_commands.at(i)};
    if (input.limit_units == LimitUnits::Degrees) {
      data.lower_commands.at(i) *= conversion;
      data.upper_commands.at(i) *= conversion;
    }
    if (!data.lower_commands.at(i).IsValid() ||
        !data.upper_commands.at(i).IsValid())
      return Status::ArithmeticFailure;
  }
  if (EncloseFrame(input.base, input.length_scale, data.base) !=
          Status::Ready ||
      EncloseFrame(input.tool, input.length_scale, data.tool) != Status::Ready)
    return Status::ArithmeticFailure;
  for (std::size_t i = 0; i < kJoints; ++i) {
    const auto &row = input.rows.at(i);
    const auto alpha = Trig(row.alpha);
    auto &fixed = data.fixed.at(i);
    fixed.rotation = Matrix3::Identity();
    fixed.rotation(1, 1) = alpha.cosine;
    fixed.rotation(1, 2) = -alpha.sine;
    fixed.rotation(2, 1) = alpha.sine;
    fixed.rotation(2, 2) = alpha.cosine;
    fixed.translation = Vector3::Zero();
    fixed.translation.x() = Interval{row.a} / Interval{input.length_scale};
    data.theta.at(i) = Trig(row.theta);
    if (!Valid(fixed) || !data.theta.at(i).sine.IsValid() ||
        !data.theta.at(i).cosine.IsValid())
      return Status::ArithmeticFailure;
  }
  std::array<Interval, 6> zero;
  zero.fill(Interval{0});
  if (!BuildDh(data, zero, data.home, &data.axes, &data.origins))
    return Status::ArithmeticFailure;
  result.ready_ = true;
  output = result;
  return Status::Ready;
}

auto ForwardDh(const Model &model, const std::array<Interval, 6> &increments,
               Pose &output) -> Status {
  if (!interval::EnvironmentSupported())
    return Status::UnsupportedEnvironment;
  if (!model.IsReady())
    return Status::InvalidInput;
  Pose result;
  if (!BuildDh(model.Data(), increments, result, nullptr, nullptr))
    return Status::ArithmeticFailure;
  output = result;
  return Status::Ready;
}

auto ForwardPoe(const Model &model, const std::array<Interval, 6> &increments,
                Pose &output) -> Status {
  if (!interval::EnvironmentSupported())
    return Status::UnsupportedEnvironment;
  if (!model.IsReady())
    return Status::InvalidInput;
  Pose prefix{Matrix3::Identity(), Vector3::Zero()};
  const auto &data = model.Data();
  for (std::size_t i = 0; i < kJoints; ++i) {
    if (!increments.at(i).IsValid())
      return Status::InvalidInput;
    const auto trig = interval::EvaluateSinCos(increments.at(i));
    const Matrix3 skew = Skew(data.axes.at(i));
    Pose factor;
    factor.rotation = Matrix3::Identity() + trig.sine * skew +
                      (Interval{1} - trig.cosine) * (skew * skew);
    factor.translation =
        (Matrix3::Identity() - factor.rotation) * data.origins.at(i);
    prefix = Compose(prefix, factor);
  }
  const Pose result = Compose(prefix, data.home);
  if (!Valid(result))
    return Status::ArithmeticFailure;
  output = result;
  return Status::Ready;
}

auto EvaluateCentre(const Model &model, const Centre &centre,
                    ball::Kinematics &output) -> Status {
  if (!interval::EnvironmentSupported())
    return Status::UnsupportedEnvironment;
  if (!model.IsReady() || !Finite(centre))
    return Status::InvalidInput;
  std::array<Interval, 6> point;
  for (std::size_t i = 0; i < kJoints; ++i)
    point.at(i) = Interval{centre.at(i)};
  ball::Kinematics result;
  if (!BuildDh(model.Data(), point, result.anchor, &result.axes,
               &result.origins))
    return Status::ArithmeticFailure;
  for (std::size_t i = 0; i < kJoints; ++i) {
    const auto column = static_cast<Eigen::Index>(i);
    result.body_jacobian.block<3, 1>(0, column) =
        result.anchor.rotation.transpose() *
        result.axes.at(i).cross(result.anchor.translation -
                                result.origins.at(i));
    result.body_jacobian.block<3, 1>(3, column) =
        result.anchor.rotation.transpose() * result.axes.at(i);
  }
  if (!Valid(result.body_jacobian))
    return Status::ArithmeticFailure;
  output = result;
  return Status::Ready;
}
} // namespace crx::calibrated
