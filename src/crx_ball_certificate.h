// SPDX-License-Identifier: Apache-2.0
// SPDX-FileCopyrightText: 2026 Centre Technologique en Aérospatiale

#pragma once

#include "crx_exact_problem.h"

#include <array>

#include <Eigen/Core>

namespace crx::ball {
using Interval = interval::Interval;
using Vector3 = exact::Vector3;
using Matrix3 = exact::Matrix3;
using Matrix6 = Eigen::Matrix<Interval, 6, 6>;
using PointMatrix6 = Eigen::Matrix<double, 6, 6>;
using Centre = std::array<double, 6>;

enum class Failure {
  None,
  InvalidInput,
  UnsupportedEnvironment,
  KinematicsUnavailable,
  InvalidPreconditioner,
  InverseResidual,
  UncertainDeterminantSign,
  TargetLogDomain,
  RadiusInequality,
  ArithmeticUnavailable,
  AspectDisagreement,
  IdentityMismatch,
  NotStrict,
  NotRequested,
  UnsupportedExactModel
};

struct Kinematics {
  exact::Pose anchor;
  std::array<Vector3, 6> axes;
  std::array<Vector3, 6> origins;
  Matrix6 body_jacobian;
  Interval closed_form_determinant;
};

// All lengths/poses are in Problem's normalized canonical coordinates. Axes
// and origins describe the exact rebased screws at the represented centre.
// Unsupported arithmetic leaves output unchanged.
auto EvaluateCentre(const exact::Problem &problem, const Centre &centre,
                    Kinematics &output) -> Failure;

struct Request {
  // Zero selects an automatically proposed radius. A positive represented
  // radius is checked as given. Negative/nonfinite values are invalid.
  double uniqueness_radius = 0;
  // Optional represented point preconditioner, never assumed to be an inverse.
  // The pointer is used only during the call and is not retained.
  const PointMatrix6 *preconditioner = nullptr;
  // Optional numerical cross-check; zero means no supplied expectation.
  int expected_aspect = 0;
};

struct Data {
  exact::Identity identity;
  Centre centre{};
  PointMatrix6 preconditioner = PointMatrix6::Zero();
  double existence_radius = 0;
  double uniqueness_radius = 0;
  double inverse_residual = 0;
  double beta = 0;
  double lipschitz = 0;
  double eta = 0;
  double coarse_eta = 0;
  bool used_log_series = false;
  double pose_defect = 0;
  double screw_norm_sum = 0;
  double angular_norm_sum = 0;
  double anchor_position_norm = 0;
  Interval target_relative_trace;
  Interval closed_form_determinant;
  int aspect_sign = 0; // Body Jacobian in canonical coordinates, not commands.
};

struct Attempt;

class Certificate {
public:
  auto IsValid() const -> bool { return valid_; }

  auto Values() const -> const Data & { return data_; }

  auto Matches(const exact::Problem &problem) const -> bool {
    return valid_ && problem.IsReady() &&
           data_.identity == problem.Data().identity;
  }

private:
  friend auto Certify(const exact::Problem &, const Centre &, Request)
      -> Attempt;
  Data data_;
  bool valid_ = false;
};

struct Attempt {
  Failure failure = Failure::InvalidInput;
  Certificate certificate;

  auto IsCertified() const -> bool {
    return failure == Failure::None && certificate.IsValid();
  }
};

// On success, exactly one solution of this exact nominal pose problem lies
// within the closed infinity ball R, and it lies within rho. This is no
// command-limit, global ranking, infeasibility or completeness certificate.
// A failed check is inconclusive. The implementation follows the assembled
// rebased anchored-log theorem, not a formal verification of this C++ code.
auto Certify(const exact::Problem &problem, const Centre &centre,
             Request request = {}) -> Attempt;

// Recheck a new target at the previous centre/preconditioner and uniqueness
// radius. Every non-target identity word must agree. Recompute all proof
// inequalities, including the new rho<=old R reserve. Failure is inconclusive.
auto TryReuse(const Certificate &previous, const exact::Problem &problem)
    -> Attempt;
} // namespace crx::ball
