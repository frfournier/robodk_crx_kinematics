// SPDX-License-Identifier: Apache-2.0
// SPDX-FileCopyrightText: 2026 Centre Technologique en Aérospatiale

#include "crx_exact_problem.h"
#include "crx_interval.h"
#include "crx_interval_matrix.h"
#include "crx_numerics.h"
#include "crx_scalar_kernels.h"
#include "crx_types.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace crx::exact {
namespace {
using Quaternion = std::array<Interval, 4>;
constexpr int kSpaceDimension = Vector3::RowsAtCompileTime;
constexpr int kRotationEntries =
    Matrix3::RowsAtCompileTime * Matrix3::ColsAtCompileTime;
constexpr std::array<double, 6> kReferenceSenses{1, 1, -1, -1, -1, -1};

template <typename Container> auto Finite(const Container &values) -> bool {
  return std::all_of(values.begin(), values.end(),
                     [](double value) { return std::isfinite(value); });
}

template <typename Derived>
auto Valid(const Eigen::MatrixBase<Derived> &matrix) -> bool {
  for (Eigen::Index row = 0; row < matrix.rows(); ++row) {
    for (Eigen::Index col = 0; col < matrix.cols(); ++col) {
      if (!matrix(row, col).IsValid()) {
        return false;
      }
    }
  }
  return true;
}

auto ValidPoseInput(const PoseInput &input) -> bool {
  return Finite(input.quaternion) && Finite(input.position) &&
         Finite(input.input_rotation) &&
         std::any_of(input.quaternion.begin(), input.quaternion.end(),
                     [](double value) { return value != 0; });
}

auto Validate(const Input &input) -> bool {
  if (!Finite(input.lengths) || input.lengths.at(0) == 0 ||
      input.lengths.at(1) == 0 || !std::isfinite(input.base_height) ||
      !Finite(input.encoder_zero) || !Finite(input.senses) ||
      !Finite(input.lower_commands) || !Finite(input.upper_commands) ||
      !ValidPoseInput(input.target) || !ValidPoseInput(input.base) ||
      !ValidPoseInput(input.tool)) {
    return false;
  }
  for (std::size_t i = 0; i < input.senses.size(); ++i) {
    if ((input.senses.at(i) != 1 && input.senses.at(i) != -1) ||
        input.lower_commands.at(i) > input.upper_commands.at(i)) {
      return false;
    }
  }
  return true;
}

auto InputIdentity(const Input &input) -> Identity {
  Identity result;
  std::size_t index = 0;
  const auto append = [&result, &index](double value) {
    std::uint64_t bits = 0;
    static_assert(sizeof(bits) == sizeof(value));
    std::memcpy(&bits, &value, sizeof(bits));
    result.words.at(index++) = bits;
  };
  for (const double value : input.lengths) {
    append(value);
  }
  append(input.base_height);
  for (const auto *values : {&input.encoder_zero, &input.senses,
                             &input.lower_commands, &input.upper_commands}) {
    for (const double value : *values) {
      append(value);
    }
  }
  for (const auto *pose : {&input.target, &input.base, &input.tool}) {
    for (const double value : pose->quaternion) {
      append(value);
    }
    for (const double value : pose->position) {
      append(value);
    }
    for (const double value : pose->input_rotation) {
      append(value);
    }
  }
  return result;
}

auto ScaledQuaternion(const PoseInput &input) -> Quaternion {
  double scale = 0;
  for (const double value : input.quaternion) {
    scale = std::max(scale, std::abs(value));
  }
  Quaternion result;
  for (std::size_t i = 0; i < result.size(); ++i) {
    result.at(i) = Interval{input.quaternion.at(i)} / Interval{scale};
  }
  return result;
}

auto Conjugate(Quaternion q) -> Quaternion {
  for (std::size_t i = 1; i < q.size(); ++i) {
    q.at(i) = -q.at(i);
  }
  return q;
}

auto Product(const Quaternion &a, const Quaternion &b) -> Quaternion {
  const auto &[aw, ax, ay, az] = a;
  const auto &[bw, bx, by, bz] = b;
  return {((aw * bw) - (ax * bx)) - (ay * by) - (az * bz),
          ((aw * bx) + (ax * bw)) + (ay * bz) - (az * by),
          ((aw * by) - (ax * bz)) + (ay * bw) + (az * bx),
          ((aw * bz) + (ax * by)) - (ay * bx) + (az * bw)};
}

auto Rotation(const Quaternion &q) -> Matrix3 {
  const auto &[w, x, y, z] = q;
  const Interval ww = interval::Square(w);
  const Interval xx = interval::Square(x);
  const Interval yy = interval::Square(y);
  const Interval zz = interval::Square(z);
  const Interval norm = ((ww + xx) + yy) + zz;
  Matrix3 result;
  if (!norm.IsValid() || norm.lo <= 0) {
    return result;
  }
  const Interval two{2.0};
  result << (((ww + xx) - yy) - zz) / norm, two * ((x * y) - (w * z)) / norm,
      two * ((x * z) + (w * y)) / norm, two * ((x * y) + (w * z)) / norm,
      (((ww - xx) + yy) - zz) / norm, two * ((y * z) - (w * x)) / norm,
      two * ((x * z) - (w * y)) / norm, two * ((y * z) + (w * x)) / norm,
      (((ww - xx) - yy) + zz) / norm;
  return result;
}

auto Deviation(const PoseInput &input, const Matrix3 &rotation) -> Interval {
  interval::Matrix<kRotationEntries, 1> differences;
  for (std::size_t i = 0; i < differences.values.size(); ++i) {
    differences.values.at(i) =
        Interval{input.input_rotation.at(i)} -
        rotation(static_cast<Eigen::Index>(i / kSpaceDimension),
                 static_cast<Eigen::Index>(i % kSpaceDimension));
  }
  return interval::EuclideanNorm(differences);
}

auto DifferenceOverScale(double first, double second, double scale)
    -> Interval {
  if (first == second) {
    return Interval{0.0};
  }
  const Interval difference = Interval{first} - Interval{second};
  if (difference.IsValid()) {
    return difference / Interval{scale};
  }
  return (Interval{first} / Interval{scale}) -
         (Interval{second} / Interval{scale});
}

auto PoseSource(const PoseIsoRT &pose) -> PoseInput {
  PoseInput result;
  const Eigen::Quaterniond q(pose.linear());
  result.quaternion = {q.w(), q.x(), q.y(), q.z()};
  for (std::size_t i = 0; i < result.position.size(); ++i) {
    result.position.at(i) = pose.translation()(static_cast<Eigen::Index>(i));
  }
  for (std::size_t i = 0; i < result.input_rotation.size(); ++i) {
    result.input_rotation.at(i) =
        pose.matrix()(static_cast<Eigen::Index>(i / kSpaceDimension),
                      static_cast<Eigen::Index>(i % kSpaceDimension));
  }
  return result;
}

auto ValidHomogeneousRow(const PoseIsoRT &pose) -> bool {
  return pose.matrix().allFinite() && pose.matrix()(3, 0) == 0 &&
         pose.matrix()(3, 1) == 0 && pose.matrix()(3, 2) == 0 &&
         pose.matrix()(3, 3) == 1;
}
} // namespace

auto Prepare(const Input &input, Problem &output) -> Status {
  if (!interval::EnvironmentSupported()) {
    return Status::UnsupportedEnvironment;
  }
  if (!Validate(input)) {
    return Status::InvalidInput;
  }
  Problem result;
  auto &data = result.data_;
  data.identity = InputIdentity(input);
  for (const double length : input.lengths) {
    data.length_scale = std::max(data.length_scale, std::abs(length));
  }
  const Interval scale{data.length_scale};
  data.lengths = {Interval{input.lengths.at(0)} / scale,
                  Interval{input.lengths.at(1)} / scale,
                  Interval{input.lengths.at(2)} / scale,
                  Interval{input.lengths.at(3)} / scale};
  const Quaternion target = ScaledQuaternion(input.target);
  const Quaternion base = ScaledQuaternion(input.base);
  const Quaternion tool = ScaledQuaternion(input.tool);
  const Matrix3 target_rotation = Rotation(target);
  const Matrix3 base_rotation = Rotation(base);
  const Matrix3 tool_rotation = Rotation(tool);
  data.target.rotation =
      Rotation(Product(Product(Conjugate(base), target), Conjugate(tool)));
  Vector3 translation_difference;
  Vector3 tool_translation;
  for (Eigen::Index i = 0; i < kSpaceDimension; ++i) {
    const auto index = static_cast<std::size_t>(i);
    translation_difference(i) =
        DifferenceOverScale(input.target.position.at(index),
                            input.base.position.at(index), data.length_scale);
    tool_translation(i) = Interval{input.tool.position.at(index)} / scale;
  }
  data.target.translation = base_rotation.transpose() * translation_difference -
                            data.target.rotation * tool_translation;
  data.target.translation.z() -= Interval{input.base_height} / scale;
  data.v = data.target.rotation.col(0);
  data.w = data.target.rotation.col(1);
  data.p =
      data.target.translation - data.lengths.c * data.target.rotation.col(2);
  data.rotation_deviation = {Deviation(input.target, target_rotation),
                             Deviation(input.base, base_rotation),
                             Deviation(input.tool, tool_rotation)};
  for (std::size_t i = 0; i < data.diagonal.size(); ++i) {
    const Interval reference =
        i == 1 ? -(interval::Pi() / Interval{2.0}) : Interval{0.0};
    data.offset_unwrapped.at(i) =
        (Interval{input.encoder_zero.at(i)} - reference) *
        Interval{kReferenceSenses.at(i)};
    data.encoder_phase.at(i) =
        interval::EnclosePhase(data.offset_unwrapped.at(i));
    data.diagonal.at(i) = input.senses.at(i) * kReferenceSenses.at(i);
    if (!data.offset_unwrapped.at(i).IsValid() ||
        !data.encoder_phase.at(i).principal.IsValid()) {
      return Status::ArithmeticFailure;
    }
  }
  if (!data.lengths.a.IsValid() || !data.lengths.b.IsValid() ||
      !data.lengths.c.IsValid() || !data.lengths.r.IsValid() ||
      !Valid(data.p) || !Valid(data.target.rotation) ||
      !Valid(data.target.translation) ||
      !std::all_of(data.rotation_deviation.begin(),
                   data.rotation_deviation.end(),
                   [](Interval value) { return value.IsValid(); })) {
    return Status::ArithmeticFailure;
  }
  result.ready_ = true;
  output = result;
  return Status::Ready;
}

auto Prepare(const CrxModelData &model, const PoseIsoRT &target,
             const Eigen::Quaterniond &target_quaternion, Problem &output)
    -> Status {
  if (!interval::EnvironmentSupported()) {
    return Status::UnsupportedEnvironment;
  }
  if (!ValidHomogeneousRow(target) ||
      !ValidHomogeneousRow(model.base_transform) ||
      !ValidHomogeneousRow(model.tool_transform)) {
    return Status::InvalidInput;
  }
  constexpr std::array<double, 6> alpha{0,        -kHalfPi, 0,
                                        -kHalfPi, kHalfPi,  -kHalfPi};
  Input input;
  for (std::size_t i = 0; i < model.dh_rows.size(); ++i) {
    const auto &row = model.dh_rows.at(i);
    if (!std::isfinite(row.alpha_rad) || !std::isfinite(row.a) ||
        !std::isfinite(row.d) || !std::isfinite(row.theta0_rad) ||
        !std::isfinite(model.joint_senses.at(i))) {
      return Status::InvalidInput;
    }
    if (row.is_prismatic || row.alpha_rad != alpha.at(i) ||
        (model.joint_senses.at(i) != 1 && model.joint_senses.at(i) != -1) ||
        (i != 2 && row.a != 0) || ((i == 1 || i == 2) && row.d != 0)) {
      return Status::UnsupportedNominalModel;
    }
    input.encoder_zero.at(i) = row.theta0_rad;
    input.senses.at(i) = model.joint_senses.at(i);
    input.lower_commands.at(i) =
        model.lower_limits_rad(static_cast<Eigen::Index>(i));
    input.upper_commands.at(i) =
        model.upper_limits_rad(static_cast<Eigen::Index>(i));
  }
  input.lengths = {model.dh_rows.at(2).a, model.dh_rows.at(3).d,
                   model.dh_rows.at(5).d, -model.dh_rows.at(4).d};
  input.base_height = model.dh_rows.at(0).d;
  input.target = PoseSource(target);
  input.target.quaternion = {target_quaternion.w(), target_quaternion.x(),
                             target_quaternion.y(), target_quaternion.z()};
  input.base = PoseSource(model.base_transform);
  input.tool = PoseSource(model.tool_transform);
  return Prepare(input, output);
}

auto Prepare(const CrxModelData &model, const PoseIsoRT &target,
             Problem &output) -> Status {
  if (!interval::EnvironmentSupported()) {
    return Status::UnsupportedEnvironment;
  }
  if (!ValidHomogeneousRow(target)) {
    return Status::InvalidInput;
  }
  return Prepare(model, target, Eigen::Quaterniond(target.linear()), output);
}

auto BuildPolynomials(const Problem &problem, Interval chart,
                      Polynomials &output) -> Status {
  if (!interval::EnvironmentSupported()) {
    return Status::UnsupportedEnvironment;
  }
  if (!problem.IsReady() || !chart.IsValid()) {
    return Status::InvalidInput;
  }
  const auto &data = problem.Data();
  const auto trig = interval::EvaluateSinCos(chart);
  const Vector3 v = (data.v * trig.cosine) + (data.w * trig.sine);
  const Vector3 w = (-data.v * trig.sine) + (data.w * trig.cosine);
  Polynomials result;
  result.identity = data.identity;
  result.chart = chart;
  result.residual = scalar::ConstructCoefficients(data.lengths, data.p, v, w);
  result.determinant =
      scalar::ConstructDeterminantCoefficients(data.lengths, data.p, v, w);
  for (std::size_t i = 0; i < result.derivative.size(); ++i) {
    result.derivative.at(i) =
        Interval{static_cast<double>(i + 1)} * result.residual.at(i + 1);
  }
  const Vector3 axis = -v;
  result.omitted_residual = scalar::ResidualForAxis(data.lengths, data.p, axis);
  const Vector3 x = data.p + data.lengths.r * axis;
  const Vector3 n = Vector3::UnitZ().cross(x);
  result.omitted_determinant = x.dot(axis.cross(n));
  const auto valid = [](const auto &values) {
    return std::all_of(values.begin(), values.end(),
                       [](Interval value) { return value.IsValid(); });
  };
  if (!valid(result.residual) || !valid(result.derivative) ||
      !valid(result.determinant) || !result.omitted_residual.IsValid() ||
      !result.omitted_determinant.IsValid()) {
    return Status::ArithmeticFailure;
  }
  output = result;
  return Status::Ready;
}

auto ForwardCanonical(const Problem &problem, const std::array<Interval, 6> &q,
                      Pose &output) -> Status {
  if (!interval::EnvironmentSupported()) {
    return Status::UnsupportedEnvironment;
  }
  if (!problem.IsReady() || !std::all_of(q.begin(), q.end(), [](Interval x) {
        return x.IsValid();
      })) {
    return Status::InvalidInput;
  }
  std::array<interval::SinCos, 6> trig;
  for (std::size_t i = 0; i < trig.size(); ++i) {
    trig.at(i) = interval::EvaluateSinCos(q.at(i));
  }
  const Matrix3 r1 = scalar::BaseRotation(trig.at(0).sine, trig.at(0).cosine);
  const Matrix3 r2 =
      scalar::RotateLocalY(r1, trig.at(1).sine, trig.at(1).cosine);
  const Matrix3 r3 =
      scalar::RotateLocalY(r2, -trig.at(2).sine, trig.at(2).cosine);
  const Matrix3 r4 =
      scalar::RotateLocalX(r3, -trig.at(3).sine, trig.at(3).cosine);
  const Matrix3 r5 =
      scalar::RotateLocalY(r4, -trig.at(4).sine, trig.at(4).cosine);
  const Matrix3 r6 =
      scalar::RotateLocalX(r5, -trig.at(5).sine, trig.at(5).cosine);
  const auto result =
      scalar::ForwardPose(problem.Data().lengths, r2, r4, r5, r6);
  if (!Valid(result.rotation) || !Valid(result.translation)) {
    return Status::ArithmeticFailure;
  }
  output = result;
  return Status::Ready;
}

auto ForwardCommands(const Problem &problem,
                     const std::array<Interval, 6> &command, Pose &output)
    -> Status {
  if (!interval::EnvironmentSupported()) {
    return Status::UnsupportedEnvironment;
  }
  if (!problem.IsReady()) {
    return Status::InvalidInput;
  }
  std::array<Interval, 6> q;
  for (std::size_t i = 0; i < q.size(); ++i) {
    q.at(i) = (Interval{problem.Data().diagonal.at(i)} * command.at(i)) +
              problem.Data().offset_unwrapped.at(i);
  }
  return ForwardCanonical(problem, q, output);
}
} // namespace crx::exact
