// SPDX-License-Identifier: Apache-2.0
// SPDX-FileCopyrightText: 2026 Centre Technologique en Aérospatiale

#include "crx_ball_certificate.h"
#include "crx_exact_problem.h"
#include "crx_interval.h"
#include "crx_interval_matrix.h"
#include "crx_scalar_kernels.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>

#include <Eigen/Core>
#include <Eigen/LU>

namespace crx::ball {
namespace {
constexpr std::size_t kJoints = 6;
constexpr int kSpace = 3;

auto Finite(const Centre &centre) -> bool {
  return std::all_of(centre.begin(), centre.end(),
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

auto Norm(const Vector3 &vector) -> Interval {
  interval::Matrix<kSpace, 1> value;
  for (int i = 0; i < kSpace; ++i) {
    value.At(static_cast<std::size_t>(i), 0) = vector(i);
  }
  return interval::EuclideanNorm(value);
}

template <typename Derived>
auto RowSum(const Eigen::MatrixBase<Derived> &matrix) -> Interval {
  Interval result{0};
  for (Eigen::Index row = 0; row < matrix.rows(); ++row) {
    Interval sum{0};
    for (Eigen::Index col = 0; col < matrix.cols(); ++col) {
      sum = sum + interval::Abs(matrix(row, col));
    }
    if (!sum.IsValid()) {
      return {};
    }
    result.hi = std::max(result.hi, sum.hi);
  }
  return result;
}

auto Enclose(const PointMatrix6 &matrix) -> Matrix6 {
  Matrix6 result;
  for (Eigen::Index row = 0; row < result.rows(); ++row) {
    for (Eigen::Index col = 0; col < result.cols(); ++col) {
      result(row, col) = Interval{matrix(row, col)};
    }
  }
  return result;
}

auto InverseProposal(const Matrix6 &jacobian) -> PointMatrix6 {
  PointMatrix6 midpoint;
  for (Eigen::Index row = 0; row < midpoint.rows(); ++row) {
    for (Eigen::Index col = 0; col < midpoint.cols(); ++col) {
      const auto value = jacobian(row, col);
      midpoint(row, col) = (value.lo / 2) + (value.hi / 2);
    }
  }
  // This computation only proposes represented binary64 values. Every use
  // below validates their action against the enclosed exact Jacobian.
  return midpoint.fullPivLu().inverse();
}

auto DeterminantSign(Matrix6 matrix) -> int {
  int sign = 1;
  for (Eigen::Index col = 0; col < matrix.cols(); ++col) {
    Eigen::Index pivot = col;
    double magnitude = -1;
    for (Eigen::Index row = col; row < matrix.rows(); ++row) {
      const auto value = matrix(row, col);
      if (!value.IsValid()) {
        return 0;
      }
      const double lower = interval::Abs(value).lo;
      if (lower > magnitude) {
        magnitude = lower;
        pivot = row;
      }
    }
    if (matrix(pivot, col).ContainsZero()) {
      return 0;
    }
    if (pivot != col) {
      matrix.row(pivot).swap(matrix.row(col));
      sign = -sign;
    }
    const auto diagonal = matrix(col, col);
    sign *= diagonal.lo > 0 ? 1 : -1;
    for (Eigen::Index row = col + 1; row < matrix.rows(); ++row) {
      const Interval multiplier = matrix(row, col) / diagonal;
      for (Eigen::Index k = col + 1; k < matrix.cols(); ++k) {
        matrix(row, k) = matrix(row, k) - multiplier * matrix(col, k);
        if (!matrix(row, k).IsValid()) {
          return 0;
        }
      }
      matrix(row, col) = Interval{0};
    }
  }
  return sign;
}

auto RegionalLipschitz(double radius, double screw_sum, double position_norm)
    -> Interval {
  const Interval pi = interval::Pi();
  const Interval pi2 = interval::Square(pi);
  const Interval pi3 = pi2 * pi;
  const Interval root2 = interval::Sqrt(Interval{2});
  const Interval reach =
      Interval{radius} * Interval{screw_sum} + Interval{position_norm};
  const Interval lambda =
      (pi / Interval{2}) * (Interval{position_norm} + reach);
  const Interval b = pi / Interval{2} + (pi2 / Interval{8}) * lambda;
  const Interval c = (root2 * pi2) / Interval{8} +
                     ((root2 * pi3) / Interval{16} +
                      (Interval{3} * root2 * pi2) / Interval{16}) *
                         lambda +
                     pi2 / Interval{8};
  return b * interval::Square(Interval{1} + reach) * (c + Interval{1}) *
         interval::Square(Interval{screw_sum});
}

// For ||H||_rowsum=d<1, log(I+H)=sum (-1)^(n+1)H^n/n. The tail after N
// terms has row-sum norm <= d^(N+1)/((N+1)(1-d)). Each extracted twist
// coordinate has error <= that norm; multiplying by the represented X gives
// a tail <= ||X||_rowsum times it. The trace check selects the principal SE(3)
// logarithm. This directly bounds ||X Log(g)||, avoiding the extra beta factor
// of the coarser 3*pi*beta*||g-I|| bound when the centre is slightly displaced.
auto LogResidual(const Matrix3 &rotation, const Vector3 &translation,
                 const Matrix6 &x, double x_norm, double defect_norm)
    -> Interval {
  if (defect_norm >= .5) {
    return {};
  }
  using Matrix4 = Eigen::Matrix<Interval, 4, 4>;
  Matrix4 h = Matrix4::Zero();
  h.block<3, 3>(0, 0) = rotation - Matrix3::Identity();
  h.block<3, 1>(0, 3) = translation;
  Matrix4 power = h;
  Matrix4 sum = Matrix4::Zero();
  constexpr int kTerms = 6;
  for (int n = 1; n <= kTerms; ++n) {
    const Matrix4 term = power / Interval{static_cast<double>(n)};
    sum = (n % 2) ? (sum + term).eval() : (sum - term).eval();
    if (n != kTerms) {
      power = (power * h).eval();
    }
  }
  Eigen::Matrix<Interval, 6, 1> logarithm;
  logarithm << sum(0, 3), sum(1, 3), sum(2, 3), sum(2, 1), sum(0, 2), sum(1, 0);
  const Eigen::Matrix<Interval, 6, 1> preconditioned = x * logarithm;
  if (!Valid(preconditioned)) {
    return {};
  }
  double norm = 0;
  for (Eigen::Index i = 0; i < preconditioned.rows(); ++i) {
    norm = std::max(norm, interval::Abs(preconditioned(i)).hi);
  }
  Interval tail_power{1};
  for (int n = 0; n <= kTerms; ++n) {
    tail_power = tail_power * Interval{defect_norm};
  }
  const Interval tail =
      tail_power / (Interval{static_cast<double>(kTerms + 1)} *
                    (Interval{1} - Interval{defect_norm}));
  return Interval{norm} + Interval{x_norm} * tail;
}

auto ClosedDeterminant(const exact::Lengths &lengths,
                       const std::array<interval::SinCos, kJoints> &trig)
    -> Interval {
  const Interval c2 = trig.at(2).cosine, s2 = trig.at(2).sine;
  const Interval c3 = trig.at(3).cosine, s3 = trig.at(3).sine;
  const Interval a = lengths.a, b = lengths.b, r = lengths.r;
  const Interval alpha = c2 * b * ((b * c2) - (r * s2 * s3));
  const Interval beta = c2 * c3 * s3 * r * a;
  const Interval gamma = b * r * c2 * c2 * s3 + b * b * c2 * s2 + b * a * c2 -
                         r * a * s2 * s3 - b * r * s3;
  return a * (alpha * trig.at(1).cosine * trig.at(4).sine +
              beta * trig.at(1).sine * trig.at(4).cosine +
              gamma * trig.at(1).sine * trig.at(4).sine);
}

auto Failed(Failure failure) -> Attempt { return {failure, {}}; }
} // namespace

auto EvaluateCentre(const exact::Problem &problem, const Centre &centre,
                    Kinematics &output) -> Failure {
  if (!interval::EnvironmentSupported()) {
    return Failure::UnsupportedEnvironment;
  }
  if (!problem.IsReady() || !Finite(centre)) {
    return Failure::InvalidInput;
  }
  std::array<interval::SinCos, kJoints> trig;
  for (std::size_t i = 0; i < trig.size(); ++i) {
    trig.at(i) = interval::EvaluateSinCos(Interval{centre.at(i)});
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
  const auto &lengths = problem.Data().lengths;
  Kinematics result;
  result.anchor = scalar::ForwardPose(lengths, r2, r4, r5, r6);
  const Vector3 elbow = lengths.a * r2.col(2);
  const Vector3 wrist =
      elbow + r4 * Vector3(lengths.b, -lengths.r, Interval{0});
  result.axes = {Vector3::UnitZ(), r1.col(1),  -r2.col(1),
                 -r3.col(0),       -r4.col(1), -r5.col(0)};
  result.origins = {Vector3::Zero(), Vector3::Zero(),          elbow, elbow,
                    wrist,           result.anchor.translation};
  for (std::size_t joint = 0; joint < kJoints; ++joint) {
    const auto col = static_cast<Eigen::Index>(joint);
    result.body_jacobian.block<kSpace, 1>(0, col) =
        result.anchor.rotation.transpose() *
        result.axes.at(joint).cross(result.anchor.translation -
                                    result.origins.at(joint));
    result.body_jacobian.block<kSpace, 1>(kSpace, col) =
        result.anchor.rotation.transpose() * result.axes.at(joint);
  }
  result.closed_form_determinant = ClosedDeterminant(lengths, trig);
  if (!Valid(result.anchor.rotation) || !Valid(result.anchor.translation) ||
      !Valid(result.body_jacobian) ||
      !result.closed_form_determinant.IsValid()) {
    return Failure::KinematicsUnavailable;
  }
  output = result;
  return Failure::None;
}

auto Certify(const exact::Problem &problem, const Centre &centre,
             Request request) -> Attempt {
  if (!interval::EnvironmentSupported()) {
    return Failed(Failure::UnsupportedEnvironment);
  }
  if (!problem.IsReady() || !Finite(centre) ||
      !std::isfinite(request.uniqueness_radius) ||
      request.uniqueness_radius < 0 || request.expected_aspect < -1 ||
      request.expected_aspect > 1) {
    return Failed(Failure::InvalidInput);
  }
  Kinematics kinematics;
  const auto status = EvaluateCentre(problem, centre, kinematics);
  if (status != Failure::None) {
    return Failed(status);
  }
  const PointMatrix6 inverse = request.preconditioner != nullptr
                                   ? *request.preconditioner
                                   : InverseProposal(kinematics.body_jacobian);
  if (!inverse.allFinite()) {
    return Failed(Failure::InvalidPreconditioner);
  }
  const Matrix6 x = Enclose(inverse);
  const Matrix6 error = Matrix6::Identity() - x * kinematics.body_jacobian;
  const Interval error_norm = RowSum(error);
  if (!error_norm.IsValid()) {
    return Failed(Failure::ArithmeticUnavailable);
  }
  if (error_norm.hi >= 1) {
    return Failed(Failure::InverseResidual);
  }
  const auto x_norm = RowSum(x);
  const Interval beta =
      Interval{x_norm.hi} / (Interval{1} - Interval{error_norm.hi});
  if (!x_norm.IsValid() || !beta.IsValid() || beta.hi <= 0) {
    return Failed(Failure::ArithmeticUnavailable);
  }
  const int sign = DeterminantSign(x);
  if (sign == 0) {
    return Failed(Failure::UncertainDeterminantSign);
  }
  const auto determinant = kinematics.closed_form_determinant;
  const int closed_sign =
      determinant.ContainsZero() ? 0 : (determinant.lo > 0 ? 1 : -1);
  if ((closed_sign != 0 && closed_sign != sign) ||
      (request.expected_aspect != 0 && request.expected_aspect != sign)) {
    return Failed(Failure::AspectDisagreement);
  }
  const auto &target = problem.Data().target;
  const Matrix3 relative_rotation =
      kinematics.anchor.rotation.transpose() * target.rotation;
  const Vector3 relative_translation =
      kinematics.anchor.rotation.transpose() *
      (target.translation - kinematics.anchor.translation);
  const Interval trace = relative_rotation.trace();
  if (!trace.IsValid()) {
    return Failed(Failure::ArithmeticUnavailable);
  }
  if (trace.lo < 1) {
    return Failed(Failure::TargetLogDomain);
  }
  Eigen::Matrix<Interval, kSpace, kSpace + 1> defect;
  defect.block<kSpace, kSpace>(0, 0) = relative_rotation - Matrix3::Identity();
  defect.col(kSpace) = relative_translation;
  const Interval defect_norm = RowSum(defect);
  const Interval coarse_eta = Interval{3} * interval::Pi() * Interval{beta.hi} *
                              Interval{defect_norm.hi};
  // A^-1=(XA)^-1 X. Even a small inverse residual requires this Neumann
  // factor; ||X Log(g)|| alone would be unsound for a scaled preconditioner.
  const Interval series_eta =
      LogResidual(relative_rotation, relative_translation, x, x_norm.hi,
                  defect_norm.hi) /
      (Interval{1} - Interval{error_norm.hi});
  const bool used_series = series_eta.IsValid() && coarse_eta.IsValid() &&
                           series_eta.hi < coarse_eta.hi;
  const Interval eta = used_series ? series_eta : coarse_eta;
  Interval screw_sum{0}, angular_sum{0};
  for (std::size_t joint = 0; joint < kJoints; ++joint) {
    const auto omega = Norm(kinematics.axes.at(joint));
    const Vector3 velocity =
        kinematics.origins.at(joint).cross(kinematics.axes.at(joint));
    screw_sum = screw_sum + omega + Norm(velocity);
    angular_sum = angular_sum + omega;
  }
  const Interval p0 = Norm(kinematics.anchor.translation);
  if (!defect_norm.IsValid() || !eta.IsValid() || !screw_sum.IsValid() ||
      !angular_sum.IsValid() || !p0.IsValid()) {
    return Failed(Failure::ArithmeticUnavailable);
  }
  double radius = request.uniqueness_radius;
  if (radius == 0) {
    // The proposal need not itself be an enclosure. The actual chosen
    // represented radius and all theorem inequalities are verified below.
    constexpr double kProposalCap = 1e-3;
    const Interval cap_l = RegionalLipschitz(kProposalCap, screw_sum.hi, p0.hi);
    const Interval product = Interval{beta.hi} * cap_l;
    if (!product.IsValid() || product.hi <= 0) {
      return Failed(Failure::ArithmeticUnavailable);
    }
    radius = std::min(kProposalCap, .5 / product.hi);
  }
  if (!std::isfinite(radius) || radius <= 0) {
    return Failed(Failure::RadiusInequality);
  }
  const Interval l = RegionalLipschitz(radius, screw_sum.hi, p0.hi);
  const Interval rho = Interval{2} * Interval{eta.hi};
  const Interval domain = Interval{radius} * Interval{angular_sum.hi};
  if (!l.IsValid() || !rho.IsValid() || !domain.IsValid()) {
    return Failed(Failure::ArithmeticUnavailable);
  }
  const Interval beta_l = Interval{beta.hi} * Interval{l.hi};
  const Interval contraction = beta_l * Interval{radius};
  const Interval discriminant = Interval{2} * beta_l * Interval{eta.hi};
  const Interval radius_polynomial =
      (beta_l * interval::Square(Interval{rho.hi})) / Interval{2} +
      Interval{eta.hi};
  if (!contraction.IsValid() || !discriminant.IsValid() ||
      !radius_polynomial.IsValid()) {
    return Failed(Failure::ArithmeticUnavailable);
  }
  if (domain.hi >= interval::Pi().lo || contraction.hi >= 1 ||
      discriminant.hi > 1 || rho.hi > radius || radius_polynomial.hi > rho.hi) {
    return Failed(Failure::RadiusInequality);
  }
  Attempt result;
  auto &data = result.certificate.data_;
  data.identity = problem.Data().identity;
  data.centre = centre;
  data.preconditioner = inverse;
  data.existence_radius = rho.hi;
  data.uniqueness_radius = radius;
  data.inverse_residual = error_norm.hi;
  data.beta = beta.hi;
  data.lipschitz = l.hi;
  data.eta = eta.hi;
  data.coarse_eta = coarse_eta.hi;
  data.used_log_series = used_series;
  data.pose_defect = defect_norm.hi;
  data.screw_norm_sum = screw_sum.hi;
  data.angular_norm_sum = angular_sum.hi;
  data.anchor_position_norm = p0.hi;
  data.target_relative_trace = trace;
  data.closed_form_determinant = determinant;
  data.aspect_sign = sign;
  result.certificate.valid_ = true;
  result.failure = Failure::None;
  return result;
}

auto TryReuse(const Certificate &previous, const exact::Problem &problem)
    -> Attempt {
  if (!interval::EnvironmentSupported()) {
    return Failed(Failure::UnsupportedEnvironment);
  }
  if (!previous.IsValid() || !problem.IsReady()) {
    return Failed(Failure::InvalidInput);
  }
  const auto &old = previous.Values();
  const auto &identity = problem.Data().identity;
  if (!exact::SameModel(old.identity, identity))
    return Failed(Failure::IdentityMismatch);
  return Certify(problem, old.centre,
                 {old.uniqueness_radius, &old.preconditioner, old.aspect_sign});
}

auto EncloseResidual(const exact::Problem &problem, const Certificate &witness,
                     const exact::Pose &targets, ResidualBounds &output)
    -> Failure {
  if (!interval::EnvironmentSupported())
    return Failure::UnsupportedEnvironment;
  if (!problem.IsReady() || !witness.IsValid() || !Valid(targets.rotation) ||
      !Valid(targets.translation))
    return Failure::InvalidInput;
  if (!witness.Matches(problem))
    return Failure::IdentityMismatch;
  const auto &bounds = witness.Values();
  Kinematics kinematics;
  const auto evaluated = EvaluateCentre(problem, bounds.centre, kinematics);
  if (evaluated != Failure::None)
    return evaluated;
  const Matrix3 rotation =
      kinematics.anchor.rotation.transpose() * targets.rotation;
  const Vector3 translation =
      kinematics.anchor.rotation.transpose() *
      (targets.translation - kinematics.anchor.translation);
  const Interval trace = rotation.trace();
  if (!trace.IsValid())
    return Failure::ArithmeticUnavailable;
  if (trace.lo < 1)
    return Failure::TargetLogDomain;
  Eigen::Matrix<Interval, kSpace, kSpace + 1> defect;
  defect.block<kSpace, kSpace>(0, 0) = rotation - Matrix3::Identity();
  defect.col(kSpace) = translation;
  const auto norm = RowSum(defect);
  const auto inverse = Enclose(bounds.preconditioner);
  const auto inverse_norm = RowSum(inverse);
  if (!norm.IsValid() || !inverse_norm.IsValid())
    return Failure::ArithmeticUnavailable;
  const Interval coarse =
      Interval{3} * interval::Pi() * Interval{bounds.beta} * Interval{norm.hi};
  const Interval series =
      LogResidual(rotation, translation, inverse, inverse_norm.hi, norm.hi) /
      (Interval{1} - Interval{bounds.inverse_residual});
  const bool use_series =
      series.IsValid() && coarse.IsValid() && series.hi < coarse.hi;
  const auto eta = use_series ? series : coarse;
  if (!coarse.IsValid() || !eta.IsValid())
    return Failure::ArithmeticUnavailable;
  ResidualBounds result;
  result.eta = eta.hi;
  result.coarse_eta = coarse.hi;
  result.pose_defect = norm.hi;
  result.used_log_series = use_series;
  result.relative_trace = trace;
  output = result;
  return Failure::None;
}
} // namespace crx::ball
