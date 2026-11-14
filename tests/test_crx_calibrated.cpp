// SPDX-License-Identifier: Apache-2.0
// SPDX-FileCopyrightText: 2026 Centre Technologique en Aérospatiale

#include "crx_calibrated_certificate.h"
#include "crx_calibrated_model.h"
#include "crx_calibrated_solver.h"

#include <cfenv>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <limits>
#include <string>

#include <xmmintrin.h>

namespace {
namespace cal = crx::calibrated;
using cal::Interval;

void Require(bool condition, const char *message) {
  if (!condition) {
    std::cerr << "Calibrated model: " << message << '\n';
    std::exit(EXIT_FAILURE);
  }
}

auto Read(double &value) -> bool {
  std::string token;
  if (!(std::cin >> token))
    return false;
  char *end = nullptr;
  value = std::strtod(token.c_str(), &end);
  return end != token.c_str() && *end == '\0';
}

template <std::size_t N> auto Read(std::array<double, N> &values) -> bool {
  for (auto &value : values)
    if (!Read(value))
      return false;
  return true;
}

auto Read(cal::Input &input, cal::Centre &centre) -> bool {
  std::uint32_t units = 0;
  if (!Read(input.length_scale) || !(std::cin >> units))
    return false;
  input.limit_units = static_cast<cal::LimitUnits>(units);
  for (auto &row : input.rows) {
    if (!(std::cin >> row.alpha.quarters) || !Read(row.alpha.radians) ||
        !Read(row.a) || !(std::cin >> row.theta.quarters) ||
        !Read(row.theta.radians) || !Read(row.d))
      return false;
  }
  return Read(input.senses) && Read(input.base.quaternion) &&
         Read(input.base.position) && Read(input.tool.quaternion) &&
         Read(input.tool.position) && Read(input.lower_commands) &&
         Read(input.upper_commands) && Read(centre);
}

void Print(double value) {
  if (std::isfinite(value))
    std::cout << value;
  else
    std::cout << "null";
}

void Print(Interval value) {
  std::cout << '[';
  Print(value.lo);
  std::cout << ',';
  Print(value.hi);
  std::cout << ']';
}

void Print(const cal::Centre &values) {
  std::cout << '[';
  for (std::size_t i = 0; i < values.size(); ++i) {
    if (i != 0)
      std::cout << ',';
    Print(values.at(i));
  }
  std::cout << ']';
}

template <typename Derived>
void Print(const Eigen::MatrixBase<Derived> &matrix) {
  std::cout << '[';
  for (Eigen::Index r = 0; r < matrix.rows(); ++r) {
    if (r != 0)
      std::cout << ',';
    std::cout << '[';
    for (Eigen::Index c = 0; c < matrix.cols(); ++c) {
      if (c != 0)
        std::cout << ',';
      Print(matrix(r, c));
    }
    std::cout << ']';
  }
  std::cout << ']';
}

void Print(const cal::Pose &pose) {
  std::cout << "{\"rotation\":";
  Print(pose.rotation);
  std::cout << ",\"translation\":";
  Print(pose.translation);
  std::cout << '}';
}

void Print(const cal::PointPose &pose) {
  std::cout << "{\"rotation\":";
  Print(pose.rotation);
  std::cout << ",\"translation\":";
  Print(pose.translation);
  std::cout << '}';
}

void Print(const cal::Attempt &attempt) {
  std::cout << ",\"failure\":" << static_cast<int>(attempt.failure)
            << ",\"certified\":" << (attempt.IsCertified() ? "true" : "false");
  if (!attempt.IsCertified())
    return;
  const auto &data = attempt.certificate.Values();
  const auto &bounds = data.bounds;
  std::cout << ",\"admitted\":" << (data.commands_admitted ? "true" : "false")
            << ",\"local_sign\":" << bounds.aspect_sign
            << ",\"command_sign\":" << data.command_jacobian_sign
            << ",\"rho\":";
  Print(bounds.existence_radius);
  std::cout << ",\"radius\":";
  Print(bounds.uniqueness_radius);
  std::cout << ",\"beta\":";
  Print(bounds.beta);
  std::cout << ",\"lipschitz\":";
  Print(bounds.lipschitz);
  std::cout << ",\"eta\":";
  Print(bounds.eta);
  std::cout << ",\"inverse_residual\":";
  Print(bounds.inverse_residual);
  std::cout << ",\"screw_sum\":";
  Print(bounds.screw_norm_sum);
  std::cout << ",\"angular_sum\":";
  Print(bounds.angular_norm_sum);
  std::cout << ",\"position_norm\":";
  Print(bounds.anchor_position_norm);
  std::cout << ",\"command_reserve\":";
  Print(data.command_reserve_lower);
  std::cout << ",\"preconditioner\":";
  Print(bounds.preconditioner);
}

auto Batch(bool certificates, bool corrections = false) -> int {
  std::cout << std::setprecision(17);
  std::size_t count = 0;
  if (!(std::cin >> count) || count > 10000)
    return 2;
  for (std::size_t i = 0; i < count; ++i) {
    cal::Input input;
    cal::Centre centre;
    if (!Read(input, centre))
      return 2;
    cal::Frame target;
    cal::CorrectionOptions correction_options;
    crx::ball::Request request;
    crx::ball::PointMatrix6 inverse;
    if (certificates || corrections) {
      if (!Read(target.quaternion) || !Read(target.position))
        return 2;
    }
    if (certificates) {
      int supplied = 0;
      if (!Read(request.uniqueness_radius) || !(std::cin >> supplied))
        return 2;
      for (Eigen::Index r = 0; r < 6; ++r)
        for (Eigen::Index c = 0; c < 6; ++c)
          if (!Read(inverse(r, c)))
            return 2;
      if (supplied != 0)
        request.preconditioner = &inverse;
    }
    if (corrections) {
      int certify = 0;
      if (!(std::cin >> correction_options.max_iterations >>
            correction_options.max_backtracks) ||
          !Read(correction_options.maximum_step) ||
          !Read(correction_options.normalized_position_tolerance) ||
          !Read(correction_options.orientation_tolerance) ||
          !Read(correction_options.uniqueness_radius) || !(std::cin >> certify))
        return 2;
      correction_options.certify = certify != 0;
    }
    cal::Model model;
    const auto prepared = cal::Prepare(input, model);
    std::cout << "{\"status\":" << static_cast<int>(prepared);
    if (certificates && prepared == cal::Status::Ready) {
      cal::Problem problem;
      const auto problem_status = cal::PrepareProblem(model, target, problem);
      std::cout << ",\"problem_status\":" << static_cast<int>(problem_status);
      if (problem_status == cal::Status::Ready)
        Print(cal::Certify(problem, centre, request));
    }
    if (corrections && prepared == cal::Status::Ready) {
      cal::Solver solver;
      const auto numerical_status = cal::PrepareSolver(model, solver);
      std::cout << ",\"solver_status\":" << static_cast<int>(numerical_status);
      if (numerical_status == cal::Status::Ready) {
        cal::PointKinematics point;
        if (cal::EvaluateNumerical(solver, centre, point) ==
            cal::Status::Ready) {
          std::cout << ",\"point_pose\":";
          Print(point.pose);
          std::cout << ",\"point_jacobian\":";
          Print(point.body_jacobian);
        }
        cal::Target prepared_target;
        const auto target_status =
            cal::PrepareTarget(solver, target, prepared_target);
        std::cout << ",\"problem_status\":" << static_cast<int>(target_status);
        if (target_status == cal::Status::Ready) {
          cal::Centre commands;
          for (std::size_t j = 0; j < commands.size(); ++j)
            commands.at(j) = input.senses.at(j) * centre.at(j);
          const auto correction = cal::Correct(solver, prepared_target,
                                               commands, correction_options);
          std::cout << ",\"correction_status\":"
                    << static_cast<int>(correction.status)
                    << ",\"iterations\":" << correction.iterations
                    << ",\"backtracks\":" << correction.backtracks
                    << ",\"evaluations\":" << correction.evaluations
                    << ",\"numerical_limits\":"
                    << (correction.numerical_limits_satisfied ? "true"
                                                              : "false")
                    << ",\"certificate_attempted\":"
                    << (correction.certificate_attempted ? "true" : "false")
                    << ",\"increments\":";
          Print(correction.increments);
          std::cout << ",\"commands\":";
          Print(correction.commands);
          std::cout << ",\"position_error\":";
          Print(correction.normalized_position_error);
          std::cout << ",\"orientation_error\":";
          Print(correction.orientation_error);
          Print(correction.local_proof);
        }
      }
    }
    if (prepared == cal::Status::Ready) {
      std::cout << ",\"lower_bounds\":[";
      for (std::size_t j = 0; j < 6; ++j) {
        if (j != 0)
          std::cout << ',';
        Print(model.Data().lower_commands.at(j));
      }
      std::cout << "],\"upper_bounds\":[";
      for (std::size_t j = 0; j < 6; ++j) {
        if (j != 0)
          std::cout << ',';
        Print(model.Data().upper_commands.at(j));
      }
      std::cout << ']';
      cal::Pose dh, poe;
      crx::ball::Kinematics kinematics;
      std::array<Interval, 6> increments;
      for (std::size_t j = 0; j < increments.size(); ++j)
        increments.at(j) = Interval{centre.at(j)};
      const auto dh_status = cal::ForwardDh(model, increments, dh);
      const auto poe_status = cal::ForwardPoe(model, increments, poe);
      const auto kin_status = cal::EvaluateCentre(model, centre, kinematics);
      std::cout << ",\"dh_status\":" << static_cast<int>(dh_status)
                << ",\"poe_status\":" << static_cast<int>(poe_status)
                << ",\"kinematics_status\":" << static_cast<int>(kin_status);
      if (dh_status == cal::Status::Ready && poe_status == cal::Status::Ready &&
          kin_status == cal::Status::Ready) {
        std::cout << ",\"dh\":";
        Print(dh);
        std::cout << ",\"poe\":";
        Print(poe);
        std::cout << ",\"home\":";
        Print(model.Data().home);
        std::cout << ",\"jacobian\":";
        Print(kinematics.body_jacobian);
        std::cout << ",\"axes\":[";
        for (std::size_t j = 0; j < 6; ++j) {
          if (j != 0)
            std::cout << ',';
          Print(model.Data().axes.at(j));
        }
        std::cout << "],\"origins\":[";
        for (std::size_t j = 0; j < 6; ++j) {
          if (j != 0)
            std::cout << ',';
          Print(model.Data().origins.at(j));
        }
        std::cout << ']';
      }
    }
    std::cout << "}\n";
  }
  return 0;
}

auto TargetAt(const cal::Model &model, const cal::Centre &centre)
    -> cal::Frame {
  crx::ball::Kinematics kinematics;
  Require(cal::EvaluateCentre(model, centre, kinematics) == cal::Status::Ready,
          "target proposal FK");
  Eigen::Matrix3d rotation;
  cal::Frame target;
  for (Eigen::Index r = 0; r < 3; ++r) {
    for (Eigen::Index c = 0; c < 3; ++c) {
      const auto value = kinematics.anchor.rotation(r, c);
      rotation(r, c) = value.lo / 2 + value.hi / 2;
    }
    const auto value = kinematics.anchor.translation(r);
    target.position.at(static_cast<std::size_t>(r)) =
        (value.lo / 2 + value.hi / 2) * model.Data().source.length_scale;
  }
  const Eigen::Quaterniond quaternion(rotation);
  target.quaternion = {quaternion.w(), quaternion.x(), quaternion.y(),
                       quaternion.z()};
  return target;
}

void CertificateTests() {
  cal::Input input;
  const std::array<std::int32_t, 6> alpha{0, -1, 2, -1, 1, -1};
  for (std::size_t i = 0; i < 6; ++i) {
    input.rows.at(i).alpha.quarters = alpha.at(i);
    input.rows.at(i).alpha.radians = (static_cast<double>(i) - 2.5) * .00012;
    input.rows.at(i).theta.radians = (2.5 - static_cast<double>(i)) * .00018;
  }
  input.rows.at(1).theta.quarters = -1;
  input.rows.at(0).d = .24;
  input.rows.at(2).a = .54;
  input.rows.at(3).d = .54;
  input.rows.at(4).d = -.15;
  input.rows.at(5).d = .16;
  input.lower_commands.fill(-4);
  input.upper_commands.fill(4);
  input.senses.at(0) = -1;
  input.length_scale = .54;
  cal::Model model;
  Require(cal::Prepare(input, model) == cal::Status::Ready,
          "prepare regular calibrated model");
  const cal::Centre centre{.3, -.7, .4, .8, -.6, .2};
  auto target = TargetAt(model, centre);
  cal::Problem problem;
  Require(cal::PrepareProblem(model, target, problem) == cal::Status::Ready,
          "prepare calibrated exact target");
  const auto attempt = cal::Certify(problem, centre);
  if (!attempt.IsCertified())
    std::cerr << "Calibrated local failure "
              << static_cast<int>(attempt.failure) << '\n';
  Require(attempt.IsCertified(), "certify regular calibrated root");
  Require(attempt.certificate.Matches(problem),
          "calibrated raw identity binding");
  Require(attempt.certificate.Values().commands_admitted,
          "whole existence ball in command bounds");
  Require(attempt.certificate.Values().command_jacobian_sign ==
              -attempt.certificate.Values().bounds.aspect_sign,
          "local command sign includes sense parity");
  Require(
      !attempt.certificate.Values().bounds.closed_form_determinant.IsValid(),
      "calibrated certificate has no nominal determinant formula");
  const auto repeated = cal::TryReuse(attempt.certificate, problem);
  Require(repeated.IsCertified(),
          "recheck calibrated certificate at same target");
  for (auto &value : target.quaternion)
    value = -value;
  cal::Problem opposite;
  Require(cal::PrepareProblem(model, target, opposite) == cal::Status::Ready,
          "prepare equivalent raw target");
  Require(!attempt.certificate.Matches(opposite),
          "equivalent quaternion signs retain distinct raw identity");
  Require(cal::TryReuse(attempt.certificate, opposite).IsCertified(),
          "full target recheck handles quaternion sign");
  auto tight = input;
  tight.upper_commands.at(1) = centre.at(1);
  cal::Model tight_model;
  Require(cal::Prepare(tight, tight_model) == cal::Status::Ready,
          "prepare boundary-limited model");
  cal::Problem tight_problem;
  Require(cal::PrepareProblem(tight_model, target, tight_problem) ==
              cal::Status::Ready,
          "prepare boundary-limited problem");
  const auto boundary = cal::Certify(tight_problem, centre);
  Require(boundary.IsCertified() &&
              !boundary.certificate.Values().commands_admitted,
          "root proof does not imply uncertain boundary admission");
  Require(cal::TryReuse(attempt.certificate, tight_problem).failure ==
              crx::ball::Failure::IdentityMismatch,
          "limit identity change cannot reuse old problem silently");
  auto degree_input = input;
  degree_input.limit_units = cal::LimitUnits::Degrees;
  cal::Model degree_model;
  Require(cal::Prepare(degree_input, degree_model) == cal::Status::Ready,
          "prepare raw degree limits with retained provenance");
  cal::Problem degree_problem;
  Require(cal::PrepareProblem(degree_model, target, degree_problem) ==
              cal::Status::Ready,
          "prepare degree-bounded exact problem");
  Require(cal::TryReuse(attempt.certificate, degree_problem).failure ==
              crx::ball::Failure::IdentityMismatch,
          "different bound units cannot reuse a raw model identity");
  const auto degree_proof = cal::Certify(degree_problem, centre);
  Require(degree_proof.IsCertified() &&
              !degree_proof.certificate.Values().commands_admitted,
          "four-degree bounds do not become four-radian admission");
  auto changed = input;
  changed.rows.at(2).a += .0001;
  cal::Model changed_model;
  Require(cal::Prepare(changed, changed_model) == cal::Status::Ready,
          "prepare changed geometry");
  cal::Problem changed_problem;
  Require(cal::PrepareProblem(changed_model, target, changed_problem) ==
              cal::Status::Ready,
          "prepare changed geometry target");
  Require(cal::TryReuse(attempt.certificate, changed_problem).failure ==
              crx::ball::Failure::IdentityMismatch,
          "calibrated geometry change cannot reuse old proof");
  crx::ball::PointMatrix6 zero = crx::ball::PointMatrix6::Zero();
  Require(!cal::Certify(problem, centre, {0, &zero, 0}).IsCertified(),
          "reject invalid represented inverse");
  Require(!cal::Certify(problem, centre, {-1, nullptr, 0}).IsCertified(),
          "reject negative requested radius");
  cal::Input singular;
  singular.rows.at(0).a = 1;
  cal::Model singular_model;
  Require(cal::Prepare(singular, singular_model) == cal::Status::Ready,
          "prepare continuous coaxial model");
  const cal::Centre zero_centre{};
  cal::Problem singular_problem;
  Require(cal::PrepareProblem(singular_model,
                              TargetAt(singular_model, zero_centre),
                              singular_problem) == cal::Status::Ready,
          "prepare continuous fibre target");
  Require(!cal::Certify(singular_problem, zero_centre).IsCertified(),
          "continuous fibre has no false unique-root ball");
  cal::Solver solver;
  Require(cal::PrepareSolver(model, solver) == cal::Status::Ready,
          "prepare numerical calibrated model");
  cal::Target prepared_target;
  Require(cal::PrepareTarget(solver, target, prepared_target) ==
              cal::Status::Ready,
          "prepare numerical exact target");
  cal::Centre seed;
  for (std::size_t i = 0; i < seed.size(); ++i)
    seed.at(i) = input.senses.at(i) * centre.at(i) + .001;
  const auto corrected = cal::Correct(solver, prepared_target, seed);
  Require(corrected.status == cal::CorrectionStatus::Converged,
          "correct calibrated seed");
  Require(corrected.local_proof.IsCertified() &&
              corrected.local_proof.certificate.Values().commands_admitted,
          "corrected calibrated result has an admitted exact local root");
  Require(corrected.iterations > 0 && corrected.iterations <= 24 &&
              corrected.evaluations <= 1 + 24 * 13,
          "bounded correction work");
  cal::CorrectionOptions numerical_options;
  numerical_options.certify = false;
  const auto numerical =
      cal::Correct(solver, prepared_target, seed, numerical_options);
  Require(numerical.status == cal::CorrectionStatus::Converged &&
              !numerical.certificate_attempted &&
              !numerical.local_proof.IsCertified(),
          "numerical-only correction reports no proof");
  Require(std::memcmp(numerical.commands.data(), corrected.commands.data(),
                      sizeof(seed)) == 0 &&
              numerical.iterations == corrected.iterations &&
              numerical.evaluations == corrected.evaluations,
          "proof enablement does not change numerical correction");
  cal::CorrectionOptions zero_budget;
  zero_budget.max_iterations = 0;
  const auto exhausted =
      cal::Correct(solver, prepared_target, seed, zero_budget);
  Require(exhausted.status == cal::CorrectionStatus::IterationBudget &&
              exhausted.evaluations == 1 && !exhausted.certificate_attempted,
          "zero correction budget remains inconclusive");
  cal::Solver changed_solver;
  Require(cal::PrepareSolver(changed_model, changed_solver) ==
              cal::Status::Ready,
          "prepare changed numerical model");
  Require(cal::Correct(changed_solver, prepared_target, seed).status ==
              cal::CorrectionStatus::ModelMismatch,
          "stale prepared target cannot cross calibrated model identities");
  cal::CorrectionOptions loose;
  loose.normalized_position_tolerance = .1;
  loose.orientation_tolerance = .1;
  const auto approximate = cal::Correct(solver, prepared_target, seed, loose);
  Require(
      approximate.status == cal::CorrectionStatus::Converged &&
          !approximate.local_proof.IsCertified(),
      "loose numerical dispatch cannot fabricate an exact root certificate");
}

void UnitTests() {
  cal::Input input;
  input.rows.at(0).a = 2;
  input.rows.at(2).d = 3;
  input.rows.at(4).alpha.quarters = 1;
  input.rows.at(4).d = 4;
  input.length_scale = 2;
  cal::Model model;
  Require(cal::Prepare(input, model) == cal::Status::Ready,
          "prepare known chain");
  const auto identity = model.Data().identity;
  cal::Centre centre{};
  crx::ball::Kinematics kinematics;
  Require(cal::EvaluateCentre(model, centre, kinematics) == cal::Status::Ready,
          "evaluate known chain");
  const std::array<double, 3> expected{1, -2, 1.5};
  for (std::size_t j = 0; j < 3; ++j) {
    const auto value =
        kinematics.anchor.translation(static_cast<Eigen::Index>(j));
    Require(value.lo <= expected.at(j) && value.hi >= expected.at(j),
            "known exact modified-DH translation");
  }
  Require(!kinematics.closed_form_determinant.IsValid(),
          "no nominal determinant formula on calibrated model");
  const auto invalid = [&](cal::Input bad) {
    Require(cal::Prepare(bad, model) == cal::Status::InvalidInput,
            "reject invalid input");
    Require(model.IsReady() && model.Data().identity == identity,
            "atomic preparation failure");
  };
  auto bad = input;
  bad.senses.at(0) = 0;
  invalid(bad);
  bad = input;
  bad.senses.at(0) = 2;
  invalid(bad);
  bad = input;
  bad.length_scale = 0;
  invalid(bad);
  bad = input;
  bad.base.quaternion.fill(0);
  invalid(bad);
  bad = input;
  bad.rows.at(3).alpha.radians = std::numeric_limits<double>::infinity();
  invalid(bad);
  bad = input;
  bad.lower_commands.at(1) = 1;
  invalid(bad);
  bad = input;
  bad.tool.position.at(2) = std::numeric_limits<double>::quiet_NaN();
  invalid(bad);
  auto changed = input;
  changed.rows.at(0).theta.radians = -0.0;
  cal::Model other;
  Require(cal::Prepare(changed, other) == cal::Status::Ready &&
              other.Data().identity != identity,
          "raw signed zero belongs to model identity");
  changed = input;
  changed.senses.at(0) = -1;
  Require(cal::Prepare(changed, other) == cal::Status::Ready &&
              other.Data().identity != identity,
          "command senses belong to identity");
  auto saved = kinematics.anchor.translation;
  centre.at(1) = std::numeric_limits<double>::quiet_NaN();
  Require(cal::EvaluateCentre(model, centre, kinematics) ==
              cal::Status::InvalidInput,
          "reject nonfinite centre");
  for (Eigen::Index j = 0; j < 3; ++j)
    Require(kinematics.anchor.translation(j).lo == saved(j).lo &&
                kinematics.anchor.translation(j).hi == saved(j).hi,
            "atomic kinematics failure");
  const int old_rounding = std::fegetround();
  Require(std::fesetround(FE_DOWNWARD) == 0,
          "set directed rounding for negative test");
  const auto rounding_status = cal::Prepare(input, model);
  Require(std::fesetround(old_rounding) == 0, "restore caller rounding");
  Require(rounding_status == cal::Status::UnsupportedEnvironment &&
              model.Data().identity == identity,
          "unsupported rounding fails atomically");
  const unsigned old_control = _mm_getcsr();
  _mm_setcsr(old_control | (1U << 15U));
  const auto flush_status = cal::Prepare(input, model);
  _mm_setcsr(old_control);
  Require(flush_status == cal::Status::UnsupportedEnvironment &&
              model.Data().identity == identity,
          "flush-to-zero fails atomically");
}
} // namespace

auto main(int argc, char **argv) -> int {
  if (argc == 2 && std::string(argv[1]) == "--batch")
    return Batch(false);
  if (argc == 2 && std::string(argv[1]) == "--certificates")
    return Batch(true);
  if (argc == 2 && std::string(argv[1]) == "--correct")
    return Batch(false, true);
  if (argc != 1)
    return 2;
  UnitTests();
  CertificateTests();
  return 0;
}
