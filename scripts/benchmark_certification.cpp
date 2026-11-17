// SPDX-License-Identifier: Apache-2.0
// SPDX-FileCopyrightText: 2026 Centre Technologique en Aérospatiale

#define NOMINMAX
#include <windows.h>

#include "crx_canonical.h"
#include "crx_exact_problem.h"
#include "crx_interval_matrix.h"
#include "crx_scalar_kernels.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <random>
#include <string>
#include <vector>

namespace {
using Interval = crx::interval::Interval;
using Status = crx::exact::Status;
enum class Operation {
  Add,
  Multiply,
  Divide,
  SquareRoot,
  Trig,
  Matrix,
  Norm,
  PrepareCircle,
  Convolution,
  Forward,
  PrepareExact,
  Bundle,
  CommandForward
};
constexpr std::array<const char *, 13> kOperations{
    "add",          "multiply",       "divide",
    "sqrt",         "trig_pair",      "matrix3_product",
    "norm3",        "circle_prepare", "coefficient_convolution",
    "canonical_fk", "exact_prepare",  "polynomial_bundle",
    "command_fk"};

struct Case {
  crx::exact::Input input;
  crx::canonical::Lengths lengths;
  crx::PoseIsoRT target =
      crx::PoseIsoRT::Identity(); // Independently supplied numerical canonical
                                  // target.
  crx::canonical::WristCircle circle;
  crx::exact::Problem problem;
  crx::exact::Polynomials polynomial;
  crx::exact::Pose forward, command_forward;
  Status prepare_status = Status::InvalidInput;
  Status bundle_status = Status::InvalidInput;
  Status forward_status = Status::InvalidInput;
  Status command_status = Status::InvalidInput;
  std::array<Interval, 6> joints;
  std::array<double, 6> centre{};
  double chart = 0;
  Interval a, b;
  double scalar_a = 0, scalar_b = 0;
};

struct Result {
  double consumed = 0;
  bool valid = false;
};

volatile double checksum = 0;

auto Consume(Interval value) -> Result {
  return value.IsValid() ? Result{value.lo + value.hi, true} : Result{};
}

auto Consume(double value) -> Result {
  return std::isfinite(value) ? Result{value, true} : Result{};
}

auto Combine(Result first, Result second) -> Result {
  return {first.consumed + second.consumed, first.valid && second.valid};
}

template <typename T, std::size_t N>
auto Consume(const std::array<T, N> &values) -> Result {
  Result result{0, true};
  for (const auto value : values) {
    result = Combine(result, Consume(value));
  }
  return result;
}

template <typename Derived>
auto Consume(const Eigen::MatrixBase<Derived> &matrix) -> Result {
  Result result{0, true};
  for (Eigen::Index row = 0; row < matrix.rows(); ++row) {
    for (Eigen::Index col = 0; col < matrix.cols(); ++col) {
      result = Combine(result, Consume(matrix(row, col)));
    }
  }
  return result;
}

template <typename T> auto Consume(const crx::scalar::Pose<T> &pose) -> Result {
  return Combine(Consume(pose.rotation), Consume(pose.translation));
}

template <std::size_t N>
auto Read(std::istream &stream, std::array<double, N> &values) -> bool {
  for (double &value : values) {
    if (!(stream >> value)) {
      return false;
    }
  }
  return true;
}

auto Read(std::istream &stream, crx::exact::Input &input) -> bool {
  if (!Read(stream, input.lengths) || !(stream >> input.base_height) ||
      !Read(stream, input.encoder_zero) || !Read(stream, input.senses) ||
      !Read(stream, input.lower_commands) ||
      !Read(stream, input.upper_commands)) {
    return false;
  }
  for (auto *pose : {&input.target, &input.base, &input.tool}) {
    if (!Read(stream, pose->quaternion) || !Read(stream, pose->position) ||
        !Read(stream, pose->input_rotation)) {
      return false;
    }
  }
  return true;
}

__declspec(noinline) auto Numerical(const Case &item, Operation operation)
    -> Result {
  const double a = item.scalar_a;
  const double b = item.scalar_b;
  switch (operation) {
  case Operation::Add:
    return Consume(a + b);
  case Operation::Multiply:
    return Consume(a * b);
  case Operation::Divide:
    return b != 0 ? Consume(a / b) : Result{};
  case Operation::SquareRoot:
    return Consume(std::sqrt(std::abs(a)));
  case Operation::Trig:
    return Consume(std::sin(a) + std::cos(a));
  case Operation::Matrix: {
    const crx::Mat3 product =
        item.target.linear() * item.target.linear().transpose();
    return Consume(product);
  }
  case Operation::Norm: {
    const crx::Vec3 vector(a, b, item.centre[2]);
    return Consume(vector.norm());
  }
  case Operation::PrepareCircle: {
    crx::canonical::WristCircle circle;
    const auto status =
        crx::canonical::PrepareWristCircle(item.lengths, item.target, circle);
    return status == crx::canonical::PreparationStatus::Ready
               ? Combine(Combine(Consume(circle.p), Consume(circle.v)),
                         Combine(Consume(circle.w),
                                 Consume(std::array<double, 5>{
                                     circle.lengths.a, circle.lengths.b,
                                     circle.lengths.c, circle.lengths.r,
                                     circle.length_scale})))
               : Result{};
  }
  case Operation::Convolution: {
    const auto coefficients = crx::scalar::ConstructCoefficients(
        item.circle.lengths, item.circle.p, item.circle.v, item.circle.w);
    return Consume(coefficients);
  }
  case Operation::Forward: {
    std::array<double, 6> sine{}, cosine{};
    for (std::size_t i = 0; i < sine.size(); ++i) {
      sine.at(i) = std::sin(item.centre.at(i));
      cosine.at(i) = std::cos(item.centre.at(i));
    }
    const auto r1 = crx::scalar::BaseRotation(sine[0], cosine[0]);
    const auto r2 = crx::scalar::RotateLocalY(r1, sine[1], cosine[1]);
    const auto r3 = crx::scalar::RotateLocalY(r2, -sine[2], cosine[2]);
    const auto r4 = crx::scalar::RotateLocalX(r3, -sine[3], cosine[3]);
    const auto r5 = crx::scalar::RotateLocalY(r4, -sine[4], cosine[4]);
    const auto r6 = crx::scalar::RotateLocalX(r5, -sine[5], cosine[5]);
    return Consume(
        crx::scalar::ForwardPose(item.circle.lengths, r2, r4, r5, r6));
  }
  default:
    return {};
  }
}

__declspec(noinline) auto Enclosed(const Case &item, Operation operation)
    -> Result {
  switch (operation) {
  case Operation::Add:
    return Consume(item.a + item.b);
  case Operation::Multiply:
    return Consume(item.a * item.b);
  case Operation::Divide:
    return Consume(item.a / item.b);
  case Operation::SquareRoot:
    return Consume(crx::interval::Sqrt(crx::interval::Abs(item.a)));
  case Operation::Trig: {
    const auto trig = crx::interval::EvaluateSinCos(item.a);
    return Consume(trig.sine + trig.cosine);
  }
  case Operation::Matrix: {
    const crx::exact::Matrix3 product =
        item.problem.Data().target.rotation *
        item.problem.Data().target.rotation.transpose();
    return Consume(product);
  }
  case Operation::Norm: {
    crx::interval::Matrix<3, 1> vector;
    vector.values = {item.a, item.b, item.joints[2]};
    return Consume(crx::interval::EuclideanNorm(vector));
  }
  case Operation::Convolution: {
    const auto &data = item.problem.Data();
    const auto coefficients = crx::scalar::ConstructCoefficients(
        data.lengths, data.p, data.v, data.w);
    return Consume(coefficients);
  }
  case Operation::Forward: {
    crx::exact::Pose pose;
    const auto status =
        crx::exact::ForwardCanonical(item.problem, item.joints, pose);
    return status == Status::Ready ? Consume(pose) : Result{};
  }
  case Operation::PrepareExact: {
    crx::exact::Problem problem;
    const auto status = crx::exact::Prepare(item.input, problem);
    return status == Status::Ready
               ? Combine(
                     Consume(problem.Data().target),
                     Combine(
                         Consume(problem.Data().p),
                         Combine(Consume(problem.Data().offset_unwrapped),
                                 Consume(problem.Data().rotation_deviation))))
               : Result{};
  }
  case Operation::Bundle: {
    crx::exact::Polynomials polynomial;
    const auto status = crx::exact::BuildPolynomials(
        item.problem, Interval{item.chart}, polynomial);
    return status == Status::Ready
               ? Combine(Consume(polynomial.residual),
                         Combine(Consume(polynomial.derivative),
                                 Combine(Consume(polynomial.determinant),
                                         Consume(std::array<Interval, 2>{
                                             polynomial.omitted_residual,
                                             polynomial.omitted_determinant}))))
               : Result{};
  }
  case Operation::CommandForward: {
    crx::exact::Pose pose;
    const auto status =
        crx::exact::ForwardCommands(item.problem, item.joints, pose);
    return status == Status::Ready ? Consume(pose) : Result{};
  }
  default:
    return {};
  }
}

void Bound(std::ostream &stream, std::size_t index, const char *field,
           std::size_t component, Interval value) {
  stream << index << ',' << field << ',' << component << ',' << value.lo << ','
         << value.hi << '\n';
}

template <std::size_t N>
void Bounds(std::ostream &stream, std::size_t index, const char *field,
            const std::array<Interval, N> &values) {
  for (std::size_t i = 0; i < N; ++i) {
    Bound(stream, index, field, i, values.at(i));
  }
}

template <typename Derived>
void Bounds(std::ostream &stream, std::size_t index, const char *field,
            const Eigen::MatrixBase<Derived> &matrix) {
  std::size_t i = 0;
  for (Eigen::Index row = 0; row < matrix.rows(); ++row) {
    for (Eigen::Index col = 0; col < matrix.cols(); ++col) {
      Bound(stream, index, field, i++, matrix(row, col));
    }
  }
}

void Snapshot(std::ostream &stream, std::size_t index, const Case &item) {
  Bound(stream, index, "scalar_trig_full_range", 0,
        Interval{crx::interval::EvaluateSinCos(item.a).used_full_range ? 1.0
                                                                       : 0.0});
  Bound(stream, index, "prepare_status", 0,
        Interval{static_cast<double>(item.prepare_status)});
  Bound(stream, index, "bundle_status", 0,
        Interval{static_cast<double>(item.bundle_status)});
  Bound(stream, index, "fk_status", 0,
        Interval{static_cast<double>(item.forward_status)});
  Bound(stream, index, "command_status", 0,
        Interval{static_cast<double>(item.command_status)});
  if (!item.problem.IsReady()) {
    return;
  }
  const auto &data = item.problem.Data();
  Bounds(stream, index, "lengths",
         std::array<Interval, 4>{data.lengths.a, data.lengths.b, data.lengths.c,
                                 data.lengths.r});
  Bounds(stream, index, "rotation", data.target.rotation);
  Bounds(stream, index, "translation", data.target.translation);
  Bounds(stream, index, "center", data.p);
  Bounds(stream, index, "deviation", data.rotation_deviation);
  Bounds(stream, index, "offset", data.offset_unwrapped);
  std::size_t uncertain = 0;
  for (const auto &phase : data.encoder_phase) {
    uncertain += phase.principal_seam_uncertain ? 1 : 0;
  }
  Bound(stream, index, "uncertain_principals", 0,
        Interval{static_cast<double>(uncertain)});
  if (item.bundle_status == Status::Ready) {
    Bounds(stream, index, "residual", item.polynomial.residual);
    Bounds(stream, index, "derivative", item.polynomial.derivative);
    Bounds(stream, index, "determinant", item.polynomial.determinant);
  }
  if (item.forward_status == Status::Ready) {
    Bounds(stream, index, "fk_rotation", item.forward.rotation);
    Bounds(stream, index, "fk_translation", item.forward.translation);
  }
  if (item.command_status == Status::Ready) {
    Bounds(stream, index, "command_rotation", item.command_forward.rotation);
    Bounds(stream, index, "command_translation",
           item.command_forward.translation);
  }
}
} // namespace

auto main(int argc, char **argv) -> int {
  if (argc != 6) {
    std::cerr << "input timings.csv bounds.csv repeats rounds\n";
    return 2;
  }
  const int repeats = std::stoi(argv[4]), rounds = std::stoi(argv[5]);
  if (repeats < 1 || rounds < 1) {
    return 2;
  }
  std::ifstream input(argv[1]);
  std::size_t count = 0;
  input >> count;
  if (!input || count == 0) {
    return 2;
  }
  std::vector<Case> cases(count);
  for (auto &item : cases) {
    if (!Read(input, item.input)) {
      return 2;
    }
    item.lengths = {item.input.lengths[0], item.input.lengths[1],
                    item.input.lengths[2], item.input.lengths[3]};
    for (int row = 0; row < 4; ++row) {
      for (int col = 0; col < 4; ++col) {
        input >> item.target.matrix()(row, col);
      }
    }
    for (std::size_t i = 0; i < item.joints.size(); ++i) {
      input >> item.joints.at(i).lo >> item.joints.at(i).hi;
      item.centre.at(i) = (item.joints.at(i).lo + item.joints.at(i).hi) / 2;
    }
    input >> item.chart >> item.a.lo >> item.a.hi >> item.b.lo >> item.b.hi;
    item.scalar_a = (item.a.lo + item.a.hi) / 2;
    item.scalar_b = (item.b.lo + item.b.hi) / 2;
    item.prepare_status = crx::exact::Prepare(item.input, item.problem);
    item.bundle_status = crx::exact::BuildPolynomials(
        item.problem, Interval{item.chart}, item.polynomial);
    item.forward_status =
        crx::exact::ForwardCanonical(item.problem, item.joints, item.forward);
    item.command_status = crx::exact::ForwardCommands(item.problem, item.joints,
                                                      item.command_forward);
    if (!input || item.prepare_status != Status::Ready ||
        crx::canonical::PrepareWristCircle(item.lengths, item.target,
                                           item.circle) !=
            crx::canonical::PreparationStatus::Ready) {
      std::cerr << "Benchmark corpus needs finite, defined exact/canonical "
                   "preparations\n";
      return 2;
    }
  }
  DWORD_PTR process_mask = 0, system_mask = 0;
  if (!GetProcessAffinityMask(GetCurrentProcess(), &process_mask,
                              &system_mask) ||
      process_mask == 0) {
    return 2;
  }
  const DWORD_PTR cpu = process_mask & (~process_mask + 1);
  if (SetThreadAffinityMask(GetCurrentThread(), cpu) == 0) {
    return 2;
  }
  std::cerr << "Thread affinity mask " << cpu << '\n';
  std::ofstream timings(argv[2]), bounds(argv[3]);
  timings << std::setprecision(17) << "case,operation,path,round,us,valid\n";
  bounds << std::setprecision(17) << "case,field,component,lo,hi\n";
  for (std::size_t i = 0; i < cases.size(); ++i) {
    Snapshot(bounds, i, cases.at(i));
  }
  std::vector<std::size_t> order(count);
  std::iota(order.begin(), order.end(), 0);
  std::mt19937 random(20261113);
  for (int round = -1; round < rounds; ++round) {
    std::shuffle(order.begin(), order.end(), random);
    for (const auto index : order) {
      for (std::size_t op = 0; op < kOperations.size(); ++op) {
        const auto operation = static_cast<Operation>(op);
        for (int side = 0; side < 2; ++side) {
          const bool enclosed =
              (static_cast<int>(index + op) + round + side) % 2 == 0;
          if ((enclosed && operation == Operation::PrepareCircle) ||
              (!enclosed &&
               op >= static_cast<std::size_t>(Operation::PrepareExact))) {
            continue;
          }
          // Volatile indirect calls prevent invariant numerical work from being
          // hoisted or removed. Both paths include this same harness overhead.
          Result (*volatile function)(const Case &, Operation) =
              enclosed ? &Enclosed : &Numerical;
          const auto expected = function(cases.at(index), operation);
          Result result;
          const int iterations = round < 0 ? 3 : repeats;
          const auto begin = std::chrono::steady_clock::now();
          for (int i = 0; i < iterations; ++i) {
            result = function(cases.at(index), operation);
            checksum += result.consumed;
          }
          const auto end = std::chrono::steady_clock::now();
          if (result.valid != expected.valid ||
              result.consumed != expected.consumed) {
            return 3;
          }
          if (round < 0) {
            continue;
          }
          const double us =
              std::chrono::duration<double, std::micro>(end - begin).count() /
              iterations;
          timings << index << ',' << kOperations.at(op) << ','
                  << (enclosed ? "interval" : "double") << ',' << round << ','
                  << us << ',' << result.valid << '\n';
        }
      }
    }
    std::cerr << "Round " << round + 1 << '/' << rounds << " complete\n";
  }
  std::cerr << "Checksum " << checksum << '\n';
  return timings && bounds ? 0 : 2;
}
