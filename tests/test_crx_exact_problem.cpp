// SPDX-License-Identifier: Apache-2.0
// SPDX-FileCopyrightText: 2026 Centre Technologique en Aérospatiale

#include "crx_allocation_probe.h"
#include "crx_exact_problem.h"
#include "crx_numerics.h"

#include <array>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <new>
#include <string>

#include <xmmintrin.h>

namespace {
using crx::exact::Interval;
using crx::exact::Status;

void Require(bool condition, const char *message) {
  if (!condition) {
    std::cerr << "Exact problem test failure: " << message << '\n';
    std::exit(EXIT_FAILURE);
  }
}

auto Contains(Interval interval, double value) -> bool {
  return interval.IsValid() && interval.lo <= value && value <= interval.hi;
}

void CheckContracts() {
  crx::exact::Input input;
  input.lengths = {1.25, -.8, .2, .25};
  input.target.position = {.4, -.2, .8};
  input.encoder_zero = {0, -crx::kHalfPi, 0, 0, 0, 0};
  crx::exact::Problem problem;
  Require(crx::exact::Prepare(input, problem) == Status::Ready,
          "ordinary preparation");
  const auto identity = problem.Data().identity;
  auto bad = input;
  bad.target.quaternion = {0, 0, 0, 0};
  Require(crx::exact::Prepare(bad, problem) == Status::InvalidInput &&
              problem.IsReady() && problem.Data().identity == identity,
          "failed preparation changed output");
  bad = input;
  bad.encoder_zero[0] = std::nextafter(0.0, 1.0);
  crx::exact::Problem different;
  Require(crx::exact::Prepare(bad, different) == Status::Ready &&
              different.Data().identity != identity,
          "raw zero identity snapped");
  bad.encoder_zero[0] = 1e20;
  Require(crx::exact::Prepare(bad, different) == Status::Ready &&
              different.Data().encoder_phase[0].trigonometry.used_full_range &&
              different.Data().encoder_phase[0].principal_seam_uncertain,
          "huge raw zero falsely reduced");
  crx::exact::Polynomials polynomial;
  Require(crx::exact::BuildPolynomials(problem, Interval{0.0}, polynomial) ==
              Status::Ready,
          "polynomial construction");
  polynomial.identity.words[0] = 42;
  Require(crx::exact::BuildPolynomials(problem, {}, polynomial) ==
                  Status::InvalidInput &&
              polynomial.identity.words[0] == 42,
          "invalid chart changed output");
  const auto control = _mm_getcsr();
  _mm_setcsr(control | 0x8000U);
  const auto rejected = crx::exact::Prepare(input, problem);
  _mm_setcsr(control);
  Require(rejected == Status::UnsupportedEnvironment &&
              problem.Data().identity == identity,
          "unsafe caller environment accepted");
  // Independent zero-polynomial stratum: preserve every zero slot, without
  // claiming a continuous fibre or inferring its dimension.
  input = {};
  Require(crx::exact::Prepare(input, problem) == Status::Ready &&
              crx::exact::BuildPolynomials(problem, Interval{0.0},
                                           polynomial) == Status::Ready,
          "zero-polynomial preparation");
  for (auto coefficient : polynomial.residual) {
    Require(coefficient.IsPoint() && coefficient.lo == 0,
            "zero slot trimmed or fabricated");
  }
  for (auto coefficient : polynomial.determinant) {
    Require(coefficient.IsPoint() && coefficient.lo == 0,
            "zero determinant slot");
  }
}

void CheckAdapter() {
  crx::CrxModelData model;
  constexpr std::array<double, 6> alpha{
      0, -crx::kHalfPi, 0, -crx::kHalfPi, crx::kHalfPi, -crx::kHalfPi};
  for (std::size_t i = 0; i < alpha.size(); ++i) {
    model.dh_rows.at(i).alpha_rad = alpha.at(i);
    model.joint_senses.at(i) = 1;
  }
  model.dh_rows[0].d = .3;
  model.dh_rows[1].theta0_rad = -crx::kHalfPi;
  model.dh_rows[2].a = 1.25;
  model.dh_rows[3].d = -.8;
  model.dh_rows[4].d = -.25;
  model.dh_rows[5].d = .2;
  model.lower_limits_rad.setConstant(-4);
  model.upper_limits_rad.setConstant(4);
  crx::exact::Problem problem;
  const auto target = crx::PoseIsoRT::Identity();
  Require(crx::exact::Prepare(model, target, problem) == Status::Ready,
          "nominal model adapter");
  model.dh_rows[0].alpha_rad = std::nextafter(0.0, 1.0);
  Require(crx::exact::Prepare(model, target, problem) ==
              Status::UnsupportedNominalModel,
          "calibrated twist silently snapped");
  model.dh_rows[0].alpha_rad = 0;
  model.dh_rows[0].a = std::nextafter(0.0, 1.0);
  Require(crx::exact::Prepare(model, target, problem) ==
              Status::UnsupportedNominalModel,
          "off-pattern length silently snapped");
  model.dh_rows[0].a = 0;
  const Eigen::Quaterniond supplied(1, 0, 1, 0);
  Require(crx::exact::Prepare(model, target, supplied, problem) ==
                  Status::Ready &&
              Contains(problem.Data().target.rotation(0, 0), 0) &&
              Contains(problem.Data().target.rotation(0, 2), 1) &&
              Contains(problem.Data().target.rotation(2, 0), -1),
          "supplied target quaternion ignored");
}

auto ReadDouble(double &output) -> bool {
  std::string token;
  if (!(std::cin >> token)) {
    return false;
  }
  char *end = nullptr;
  output = std::strtod(token.c_str(), &end);
  return end != token.c_str() && *end == '\0';
}

template <std::size_t N> auto Read(std::array<double, N> &values) -> bool {
  for (double &value : values) {
    if (!ReadDouble(value)) {
      return false;
    }
  }
  return true;
}

auto Read(crx::exact::Input &input) -> bool {
  if (!Read(input.lengths) || !ReadDouble(input.base_height) ||
      !Read(input.encoder_zero) || !Read(input.senses) ||
      !Read(input.lower_commands) || !Read(input.upper_commands)) {
    return false;
  }
  for (auto *pose : {&input.target, &input.base, &input.tool}) {
    if (!Read(pose->quaternion) || !Read(pose->position) ||
        !Read(pose->input_rotation)) {
      return false;
    }
  }
  return true;
}

auto Frame(const crx::exact::PoseInput &input) -> crx::PoseIsoRT {
  auto pose = crx::PoseIsoRT::Identity();
  for (std::size_t i = 0; i < input.input_rotation.size(); ++i) {
    pose.matrix()(static_cast<Eigen::Index>(i / 3),
                  static_cast<Eigen::Index>(i % 3)) =
        input.input_rotation.at(i);
  }
  for (std::size_t i = 0; i < input.position.size(); ++i) {
    pose.translation()[static_cast<Eigen::Index>(i)] = input.position.at(i);
  }
  return pose;
}

auto PrepareAdapter(const crx::exact::Input &input,
                    crx::exact::Problem &problem) -> Status {
  crx::CrxModelData model;
  constexpr std::array<double, 6> alpha{
      0, -crx::kHalfPi, 0, -crx::kHalfPi, crx::kHalfPi, -crx::kHalfPi};
  model.base_transform = Frame(input.base);
  model.tool_transform = Frame(input.tool);
  for (std::size_t i = 0; i < alpha.size(); ++i) {
    model.dh_rows.at(i).alpha_rad = alpha.at(i);
    model.dh_rows.at(i).theta0_rad = input.encoder_zero.at(i);
    model.joint_senses.at(i) = input.senses.at(i);
    model.lower_limits_rad[static_cast<Eigen::Index>(i)] =
        input.lower_commands.at(i);
    model.upper_limits_rad[static_cast<Eigen::Index>(i)] =
        input.upper_commands.at(i);
  }
  model.dh_rows[0].d = input.base_height;
  model.dh_rows[2].a = input.lengths[0];
  model.dh_rows[3].d = input.lengths[1];
  model.dh_rows[4].d = -input.lengths[3];
  model.dh_rows[5].d = input.lengths[2];
  const auto &q = input.target.quaternion;
  return crx::exact::Prepare(model, Frame(input.target),
                             Eigen::Quaterniond(q[0], q[1], q[2], q[3]),
                             problem);
}

void Print(Interval value) {
  if (value.IsValid()) {
    std::cout << '[' << value.lo << ',' << value.hi << ']';
  } else {
    std::cout << "null";
  }
}

template <std::size_t N> void Print(const std::array<Interval, N> &values) {
  std::cout << '[';
  for (std::size_t i = 0; i < N; ++i) {
    if (i) {
      std::cout << ',';
    }
    Print(values.at(i));
  }
  std::cout << ']';
}

template <typename Derived>
void Print(const Eigen::MatrixBase<Derived> &matrix) {
  std::cout << '[';
  for (Eigen::Index row = 0; row < matrix.rows(); ++row) {
    for (Eigen::Index col = 0; col < matrix.cols(); ++col) {
      if (row || col) {
        std::cout << ',';
      }
      Print(matrix(row, col));
    }
  }
  std::cout << ']';
}

auto Batch() -> int {
  std::string operation;
  while (std::cin >> operation) {
    crx::exact::Input input;
    Interval chart;
    std::array<Interval, 6> joints;
    if (!Read(input) || !ReadDouble(chart.lo) || !ReadDouble(chart.hi)) {
      return 2;
    }
    for (auto &joint : joints) {
      if (!ReadDouble(joint.lo) || !ReadDouble(joint.hi)) {
        return 2;
      }
    }
    if (operation != "canonical" && operation != "command" &&
        operation != "adapter") {
      return 2;
    }
    crx::exact::Problem problem;
    crx::exact::Polynomials polynomial;
    crx::exact::Pose forward;
    const auto allocations = crx::test::AllocationCount();
    const bool previous = Eigen::internal::is_malloc_allowed();
    Eigen::internal::set_is_malloc_allowed(false);
    const auto status = operation == "adapter"
                            ? PrepareAdapter(input, problem)
                            : crx::exact::Prepare(input, problem);
    const auto polynomial_status =
        crx::exact::BuildPolynomials(problem, chart, polynomial);
    const auto fk_status =
        operation == "canonical"
            ? crx::exact::ForwardCanonical(problem, joints, forward)
            : crx::exact::ForwardCommands(problem, joints, forward);
    Eigen::internal::set_is_malloc_allowed(previous);
    Require(crx::test::AllocationCount() == allocations,
            "exact arithmetic allocated");
    std::cout << "{\"status\":" << static_cast<int>(status)
              << ",\"polynomial_status\":"
              << static_cast<int>(polynomial_status)
              << ",\"fk_status\":" << static_cast<int>(fk_status);
    if (problem.IsReady()) {
      const auto &data = problem.Data();
      std::cout << ",\"identity\":[";
      for (std::size_t i = 0; i < data.identity.words.size(); ++i) {
        if (i) {
          std::cout << ',';
        }
        std::cout << '"' << std::hex << data.identity.words.at(i) << std::dec
                  << '"';
      }
      std::cout << "],\"scale\":" << data.length_scale << ",\"lengths\":";
      Print(std::array<Interval, 4>{data.lengths.a, data.lengths.b,
                                    data.lengths.c, data.lengths.r});
      std::cout << ",\"rotation\":";
      Print(data.target.rotation);
      std::cout << ",\"translation\":";
      Print(data.target.translation);
      std::cout << ",\"center\":";
      Print(data.p);
      std::cout << ",\"deviation\":";
      Print(data.rotation_deviation);
      std::cout << ",\"offset\":";
      Print(data.offset_unwrapped);
      std::cout << ",\"principal\":[";
      for (std::size_t i = 0; i < data.encoder_phase.size(); ++i) {
        if (i) {
          std::cout << ',';
        }
        Print(data.encoder_phase.at(i).principal);
      }
      std::cout << "],\"seams\":[";
      for (std::size_t i = 0; i < data.encoder_phase.size(); ++i) {
        if (i) {
          std::cout << ',';
        }
        std::cout << (data.encoder_phase.at(i).principal_seam_uncertain
                          ? "true"
                          : "false");
      }
      std::cout << ']';
    }
    if (polynomial_status == Status::Ready) {
      std::cout << ",\"residual\":";
      Print(polynomial.residual);
      std::cout << ",\"derivative\":";
      Print(polynomial.derivative);
      std::cout << ",\"determinant\":";
      Print(polynomial.determinant);
      std::cout << ",\"omitted\":";
      Print(std::array<Interval, 2>{polynomial.omitted_residual,
                                    polynomial.omitted_determinant});
    }
    if (fk_status == Status::Ready) {
      std::cout << ",\"fk_rotation\":";
      Print(forward.rotation);
      std::cout << ",\"fk_translation\":";
      Print(forward.translation);
    }
    std::cout << "}\n";
  }
  return std::cin.eof() ? 0 : 2;
}
} // namespace

auto main(int argc, char **argv) -> int {
  std::cout << std::setprecision(17);
  if (argc == 2 && std::strcmp(argv[1], "--batch") == 0) {
    return Batch();
  }
  if (argc != 1) {
    return 2;
  }
  const auto before = crx::test::AllocationCount();
  void *(*volatile allocate)(std::size_t) = &::operator new;
  void *memory = allocate(1);
  Require(crx::test::AllocationCount() == before + 1,
          "allocation counter inactive");
  ::operator delete(memory);
  CheckContracts();
  CheckAdapter();
  std::cout << "Exact target/model contracts passed\n";
  return 0;
}
