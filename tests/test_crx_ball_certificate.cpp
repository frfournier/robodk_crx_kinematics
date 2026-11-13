// SPDX-License-Identifier: Apache-2.0
// SPDX-FileCopyrightText: 2026 Centre Technologique en Aérospatiale

#include "crx_allocation_probe.h"
#include "crx_ball_certificate.h"
#include "crx_discovery.h"
#include "crx_numerics.h"

#include <array>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <limits>
#include <new>
#include <string>

#include <xmmintrin.h>

namespace {
using crx::ball::Centre;
using crx::ball::Failure;
using crx::interval::Interval;

void Require(bool condition, const char *message) {
  if (!condition) {
    std::cerr << "Ball certificate: " << message << '\n';
    std::exit(EXIT_FAILURE);
  }
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

void Print(double value) {
  if (std::isfinite(value)) {
    std::cout << value;
  } else {
    std::cout << "null";
  }
}

void Print(Interval value) {
  std::cout << '[';
  Print(value.lo);
  std::cout << ',';
  Print(value.hi);
  std::cout << ']';
}

template <typename T, std::size_t N>
void Print(const std::array<T, N> &values) {
  std::cout << '[';
  for (std::size_t i = 0; i < values.size(); ++i) {
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

auto Checked(const crx::exact::Problem &problem, const Centre &centre,
             crx::ball::Request request = {}) -> crx::ball::Attempt {
  const auto allocations = crx::test::AllocationCount();
  const bool previous = Eigen::internal::is_malloc_allowed();
  Eigen::internal::set_is_malloc_allowed(false);
  const auto attempt = crx::ball::Certify(problem, centre, request);
  Eigen::internal::set_is_malloc_allowed(previous);
  Require(crx::test::AllocationCount() == allocations, "certificate allocated");
  Require(attempt.IsCertified() == (attempt.failure == Failure::None),
          "success without sealed certificate");
  if (attempt.IsCertified()) {
    Require(attempt.certificate.Matches(problem), "wrong identity");
  }
  return attempt;
}

void PrintResult(const crx::exact::Problem &problem, const Centre &centre,
                 crx::ball::Request request = {},
                 const crx::ball::Attempt *cached = nullptr) {
  const auto attempt =
      cached != nullptr ? *cached : Checked(problem, centre, request);
  crx::ball::Kinematics kinematics;
  const auto kstatus = crx::ball::EvaluateCentre(problem, centre, kinematics);
  std::cout << "{\"failure\":" << static_cast<int>(attempt.failure)
            << ",\"kinematics_status\":" << static_cast<int>(kstatus)
            << ",\"centre\":";
  Print(centre);
  std::array<double, crx::exact::kIdentityWords> raw_input{};
  for (std::size_t i = 0; i < raw_input.size(); ++i) {
    const auto bits = problem.Data().identity.words.at(i);
    std::memcpy(&raw_input.at(i), &bits, sizeof(bits));
  }
  std::cout << ",\"input\":";
  Print(raw_input);
  std::cout << ",\"schema\":" << problem.Data().identity.schema;
  std::cout << ",\"target_rotation\":";
  Print(problem.Data().target.rotation);
  std::cout << ",\"target_translation\":";
  Print(problem.Data().target.translation);
  if (kstatus == Failure::None) {
    std::cout << ",\"rotation\":";
    Print(kinematics.anchor.rotation);
    std::cout << ",\"translation\":";
    Print(kinematics.anchor.translation);
    std::cout << ",\"axes\":[";
    for (std::size_t i = 0; i < centre.size(); ++i) {
      if (i) {
        std::cout << ',';
      }
      Print(kinematics.axes.at(i));
    }
    std::cout << "],\"origins\":[";
    for (std::size_t i = 0; i < centre.size(); ++i) {
      if (i) {
        std::cout << ',';
      }
      Print(kinematics.origins.at(i));
    }
    std::cout << "],\"jacobian\":";
    Print(kinematics.body_jacobian);
    std::cout << ",\"closed_determinant\":";
    Print(kinematics.closed_form_determinant);
  }
  if (attempt.IsCertified()) {
    const auto &data = attempt.certificate.Values();
    std::cout << ",\"constants\":";
    Print(std::array<double, 11>{
        data.existence_radius, data.uniqueness_radius, data.inverse_residual,
        data.beta, data.lipschitz, data.eta, data.pose_defect,
        data.screw_norm_sum, data.angular_norm_sum, data.anchor_position_norm,
        static_cast<double>(data.aspect_sign)});
    std::cout << ",\"trace\":";
    Print(data.target_relative_trace);
    std::cout << ",\"preconditioner\":";
    Print(data.preconditioner);
    std::cout << ",\"coarse_eta\":";
    Print(data.coarse_eta);
    std::cout << ",\"used_log_series\":"
              << (data.used_log_series ? "true" : "false");
  }
  std::cout << "}\n";
}

auto Batch() -> int {
  int requested = 0;
  while (std::cin >> requested) {
    crx::exact::Input input;
    Centre centre;
    double radius = 0;
    crx::ball::PointMatrix6 inverse;
    if (!Read(input) || !Read(centre) || !ReadDouble(radius)) {
      return 2;
    }
    for (Eigen::Index row = 0; row < inverse.rows(); ++row) {
      for (Eigen::Index col = 0; col < inverse.cols(); ++col) {
        if (!ReadDouble(inverse(row, col))) {
          return 2;
        }
      }
    }
    crx::exact::Problem problem;
    const auto prepared =
        (requested == 3 || requested == 5)
            ? crx::exact::Prepare(
                  input, crx::exact::FrameConvention::ReferenceMap, problem)
            : crx::exact::Prepare(input, problem);
    if (prepared != crx::exact::Status::Ready) {
      std::cout << "{\"preparation_failure\":" << static_cast<int>(prepared)
                << "}\n";
      continue;
    }
    if (requested == 2 || requested == 5) {
      crx::exact::Input changed;
      if (!Read(changed)) {
        return 2;
      }
      const auto prior = Checked(problem, centre);
      crx::exact::Problem next;
      const auto next_status =
          requested == 5
              ? crx::exact::Prepare(
                    changed, crx::exact::FrameConvention::ReferenceMap, next)
              : crx::exact::Prepare(changed, next);
      Require(next_status == crx::exact::Status::Ready,
              "invalid reuse target in batch");
      const auto allocations = crx::test::AllocationCount();
      const bool previous = Eigen::internal::is_malloc_allowed();
      Eigen::internal::set_is_malloc_allowed(false);
      const auto attempt = crx::ball::TryReuse(prior.certificate, next);
      Eigen::internal::set_is_malloc_allowed(previous);
      Require(crx::test::AllocationCount() == allocations, "reuse allocated");
      PrintResult(next, centre, {}, &attempt);
    } else {
      PrintResult(problem, centre,
                  {radius, requested == 1 ? &inverse : nullptr});
    }
  }
  return std::cin.eof() ? 0 : 2;
}

auto Witness(bool print) -> int {
  crx::PoseIsoRT target = crx::PoseIsoRT::Identity();
  target.linear() << 3239.0 / 4059, 2182.0 / 4059, 1106.0 / 4059,
      -2378.0 / 4059, 3239.0 / 4059, 574.0 / 4059, -574.0 / 4059,
      -1106.0 / 4059, 3863.0 / 4059;
  target.translation() << 27.0 / 64, 15.0 / 32, 55.0 / 128;
  const auto discovered = crx::canonical::DiscoverJointCandidates(
      {1, .6, .2, .5}, target, {1e-10, 1e-10});
  Require(discovered.status == crx::canonical::RecoveryStatus::Candidates &&
              discovered.count == 16,
          "sixteen-solution witness discovery");
  crx::exact::Input input;
  input.lengths = {1, .6, .2, .5};
  const Eigen::Quaterniond quaternion(target.linear());
  input.target.quaternion = {quaternion.w(), quaternion.x(), quaternion.y(),
                             quaternion.z()};
  for (std::size_t i = 0; i < input.target.position.size(); ++i) {
    input.target.position.at(i) = target.translation()(static_cast<int>(i));
  }
  crx::exact::Problem problem;
  Require(crx::exact::Prepare(input, problem) == crx::exact::Status::Ready,
          "witness exact preparation");
  for (std::size_t i = 0; i < discovered.count; ++i) {
    Centre centre;
    for (std::size_t j = 0; j < centre.size(); ++j) {
      centre.at(j) = discovered.candidates.at(i).joints(static_cast<int>(j));
    }
    const auto result = Checked(problem, centre);
    if (!result.IsCertified()) {
      std::cerr << "Witness " << i << " failed "
                << static_cast<int>(result.failure) << '\n';
    }
    Require(result.IsCertified(), "witness candidate not certified");
    crx::exact::Problem other_convention;
    Require(
        crx::exact::Prepare(input, crx::exact::FrameConvention::ReferenceMap,
                            other_convention) == crx::exact::Status::Ready &&
            crx::ball::TryReuse(result.certificate, other_convention).failure ==
                Failure::IdentityMismatch,
        "frame convention identity lost");
    Require(Checked(problem, centre,
                    {0, nullptr, -result.certificate.Values().aspect_sign})
                    .failure == Failure::AspectDisagreement,
            "aspect disagreement accepted");
    crx::ball::Kinematics retained;
    Require(crx::ball::EvaluateCentre(problem, centre, retained) ==
                Failure::None,
            "centre evaluation failed");
    const auto first = retained.anchor.translation.x();
    Centre invalid = centre;
    invalid.at(0) = std::numeric_limits<double>::infinity();
    Require(crx::ball::EvaluateCentre(problem, invalid, retained) ==
                    Failure::InvalidInput &&
                retained.anchor.translation.x().lo == first.lo &&
                retained.anchor.translation.x().hi == first.hi,
            "failed centre evaluation changed output");
    for (auto &value : centre) {
      value += 1e-9;
    }
    Require(Checked(problem, centre).IsCertified(),
            "perturbed witness not certified");
    if (print) {
      PrintResult(problem, centre);
    }
    const auto zero = crx::ball::PointMatrix6::Zero().eval();
    Require(Checked(problem, centre, {0, &zero}).failure ==
                Failure::InverseResidual,
            "zero preconditioner accepted");
    Require(Checked(problem, centre, {-1, nullptr}).failure ==
                Failure::InvalidInput,
            "negative radius accepted");
    Require(!Checked(problem, centre, {1e6, nullptr}).IsCertified(),
            "large radius accepted");
    auto different_input = input;
    different_input.target.position.at(0) += 1e-6;
    crx::exact::Problem different;
    Require(crx::exact::Prepare(different_input, different) ==
                    crx::exact::Status::Ready &&
                !result.certificate.Matches(different),
            "mismatched target certificate reused");
  }
  const auto control = _mm_getcsr();
  _mm_setcsr(control | 0x8000U);
  const auto rejected = crx::ball::Certify(problem, {});
  _mm_setcsr(control);
  Require(rejected.failure == Failure::UnsupportedEnvironment,
          "unsafe environment accepted");
  return 0;
}
} // namespace

namespace {
void CheckReferenceAdapter() {
  crx::CrxModelData model;
  model.base_transform.translation().z() = -.5;
  model.joint_senses = {1, 1, -1, -1, -1, -1};
  model.dh_rows = {{{0, 0, 0, .5, false},
                    {-crx::kHalfPi, 0, -crx::kHalfPi, 0, false},
                    {0, 1, 0, 0, false},
                    {-crx::kHalfPi, 0, 0, .6, false},
                    {crx::kHalfPi, 0, 0, -.5, false},
                    {-crx::kHalfPi, 0, 0, .2, false}}};
  model.lower_limits_rad.setConstant(-4);
  model.upper_limits_rad.setConstant(4);
  crx::PoseIsoRT target = crx::PoseIsoRT::Identity();
  target.translation() << .4, -.2, .5;
  crx::PoseIsoRT reference = crx::PoseIsoRT::Identity();
  reference.translation() << .2, .3, -.1;
  crx::PoseIsoRT tool = crx::PoseIsoRT::Identity();
  tool.translation() << -.1, .2, .05;
  const Eigen::Quaterniond supplied(2, .25, -.5, .75);
  crx::exact::Problem problem;
  Require(crx::exact::PrepareReference(model, target, supplied, reference, tool,
                                       problem) == crx::exact::Status::Ready &&
              problem.Data().identity.schema == 2,
          "reference adapter failed");
  const auto identity = problem.Data().identity;
  double stored = 0;
  const auto bits = identity.words.at(29);
  std::memcpy(&stored, &bits, sizeof(stored));
  Require(stored == supplied.w(),
          "reference adapter replaced source quaternion");
  const double tiny = std::numeric_limits<double>::denorm_min();
  auto bad = model;
  bad.base_transform.linear()(0, 1) = tiny;
  Require(crx::exact::PrepareReference(bad, target, supplied, reference, tool,
                                       problem) ==
                  crx::exact::Status::UnsupportedNominalModel &&
              problem.Data().identity == identity,
          "reference adapter snapped base rotation");
  bad = model;
  bad.base_transform.translation().x() = tiny;
  Require(crx::exact::PrepareReference(bad, target, supplied, reference, tool,
                                       problem) ==
              crx::exact::Status::UnsupportedNominalModel,
          "reference adapter snapped base translation");
  bad = model;
  bad.tool_transform.translation().x() = tiny;
  Require(crx::exact::PrepareReference(bad, target, supplied, reference, tool,
                                       problem) ==
              crx::exact::Status::UnsupportedNominalModel,
          "reference adapter ignored native tool");
  auto invalid_reference = reference;
  invalid_reference.matrix()(3, 0) = .1;
  Require(crx::exact::PrepareReference(model, target, supplied,
                                       invalid_reference, tool, problem) ==
                  crx::exact::Status::InvalidInput &&
              problem.Data().identity == identity,
          "reference adapter accepted nonhomogeneous frame");
  crx::exact::Input input;
  Require(crx::exact::Prepare(input,
                              static_cast<crx::exact::FrameConvention>(99),
                              problem) == crx::exact::Status::InvalidInput &&
              problem.Data().identity == identity,
          "unknown frame convention accepted");
}
} // namespace

auto main(int argc, char **argv) -> int {
  std::cout << std::setprecision(17);
  if (argc == 2 && std::strcmp(argv[1], "--batch") == 0) {
    return Batch();
  }
  if (argc == 2 && std::strcmp(argv[1], "--witness") == 0) {
    return Witness(true);
  }
  if (argc != 1) {
    return 2;
  }
  const auto count = crx::test::AllocationCount();
  void *(*volatile allocate)(std::size_t) = &::operator new;
  void *memory = allocate(1);
  Require(crx::test::AllocationCount() == count + 1,
          "allocation counter inactive");
  ::operator delete(memory);
  const auto adapter_count = crx::test::AllocationCount();
  const bool malloc_allowed = Eigen::internal::is_malloc_allowed();
  Eigen::internal::set_is_malloc_allowed(false);
  CheckReferenceAdapter();
  Eigen::internal::set_is_malloc_allowed(malloc_allowed);
  Require(crx::test::AllocationCount() == adapter_count,
          "reference adapter allocated");
  const int result = Witness(false);
  std::cout << "Ball witness and failed-hypothesis checks passed\n";
  return result;
}
