// SPDX-License-Identifier: Apache-2.0
// SPDX-FileCopyrightText: 2026 Centre Technologique en Aérospatiale

#pragma once

#include "crx_calibrated_certificate.h"

#include <limits>

namespace crx::calibrated {
struct PointPose {
  Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();
  Eigen::Vector3d translation = Eigen::Vector3d::Zero();
};

struct PointKinematics {
  PointPose pose;
  ball::PointMatrix6 body_jacobian = ball::PointMatrix6::Zero();
};

struct NumericalData {
  Model geometry;
  PointPose base;
  PointPose tool;
  std::array<PointPose, 6> fixed;
  std::array<double, 6> theta_sine{};
  std::array<double, 6> theta_cosine{};
  std::array<double, 6> axial_distance{};
};

class Solver {
public:
  auto IsReady() const -> bool { return ready_; }

  auto Data() const -> const NumericalData & { return data_; }

private:
  friend auto PrepareSolver(const Model &, Solver &) -> Status;
  NumericalData data_;
  bool ready_ = false;
};

class Target {
public:
  auto IsReady() const -> bool { return ready_; }

  auto Exact() const -> const Problem & { return problem_; }

  auto Point() const -> const PointPose & { return point_; }

private:
  friend auto PrepareTarget(const Solver &, const Frame &, Target &) -> Status;
  Problem problem_;
  PointPose point_;
  bool ready_ = false;
};

auto PrepareSolver(const Model &model, Solver &output) -> Status;
auto PrepareTarget(const Solver &solver, const Frame &target, Target &output)
    -> Status;

// Rounded numerical proposals, not enclosures or exact root claims. Physical
// increments x are used here; Correct accepts coupled command radians instead.
auto EvaluateNumerical(const Solver &solver, const Centre &increments,
                       PointKinematics &output) -> Status;

enum class CorrectionStatus {
  Converged,
  InvalidInput,
  ModelMismatch,
  ArithmeticFailure,
  IterationBudget,
  SingularStep,
  LineSearchFailure
};

struct CorrectionOptions {
  std::uint32_t max_iterations = 24;
  std::uint32_t max_backtracks = 12;
  double maximum_step = .25; // Infinity norm in physical joint radians.
  double normalized_position_tolerance = 1e-13;
  double orientation_tolerance =
      1e-13; // Rotation-vector Euclidean norm, radians.
  bool certify = true;
  double uniqueness_radius = 0; // Zero requests the checked automatic proposal.
};

auto ValidateCorrectionOptions(const CorrectionOptions &options) -> bool;

struct Correction {
  CorrectionStatus status = CorrectionStatus::InvalidInput;
  Centre increments{};
  Centre commands{};
  std::uint32_t iterations = 0;
  std::uint32_t backtracks = 0;
  std::uint32_t evaluations = 0;
  double normalized_position_error = std::numeric_limits<double>::quiet_NaN();
  double orientation_error = std::numeric_limits<double>::quiet_NaN();
  bool numerical_limits_satisfied = false;
  bool certificate_requested = false;
  bool certificate_attempted = false;
  Attempt local_proof{ball::Failure::NotRequested, {}};
};

// Work is bounded by the iteration/backtrack options. No phase wrapping,
// nominal symmetry substitution or global nearest/coverage claim occurs.
// Converged means only a numerical dispatch test; local_proof and its separate
// command-admission result determine any certified calibrated IK claim.
auto Correct(const Solver &solver, const Target &target,
             const Centre &seed_commands, CorrectionOptions options = {})
    -> Correction;
} // namespace crx::calibrated
