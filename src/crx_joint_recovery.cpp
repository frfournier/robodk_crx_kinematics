#include "crx_joint_recovery.h"
#include "crx_canonical.h"
#include "crx_incidence.h"
#include "crx_numerics.h"
#include "crx_types.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>

#include <Eigen/Cholesky>
#include <Eigen/LU>
#include <Eigen/SVD>

namespace crx::canonical {
namespace {

auto FlangeConvention() -> Mat3 {
  Mat3 result;
  result << 0, 0, 1, 0, -1, 0, 1, 0, 0;
  return result;
}

auto Angle(double sine, double cosine, double &angle) -> bool {
  const double magnitude = std::hypot(sine, cosine);
  if (!std::isfinite(magnitude) || magnitude <= numerics::kGeometryTolerance) {
    return false;
  }
  angle = std::atan2(sine, cosine);
  return true;
}

auto NormalizePhase(double sine, double cosine, WristPhase &phase) -> bool {
  const double magnitude = std::hypot(sine, cosine);
  if (!std::isfinite(magnitude) || magnitude <= numerics::kGeometryTolerance) {
    return false;
  }
  phase = {cosine / magnitude, sine / magnitude};
  return true;
}

auto UndoBase(const WristPhase &base, const Vec3 &vector) -> Vec3 {
  return {base.cosine * vector.x() + base.sine * vector.y(),
          -base.sine * vector.x() + base.cosine * vector.y(), vector.z()};
}

auto UndoShoulder(const WristPhase &beta, const Vec3 &vector) -> Vec3 {
  return {beta.cosine * vector.x() - beta.sine * vector.z(), vector.y(),
          beta.sine * vector.x() + beta.cosine * vector.z()};
}

auto RecoverAtBase(const WristPhase &base, const Lengths &lengths,
                   const Mat3 &rotation, const Vec3 &x, const Vec3 &u,
                   const Vec3 &y, Vec6 &q, bool nomination = false) -> bool {
  const Vec3 upper = UndoBase(base, y) / lengths.a;
  const Vec3 lower = UndoBase(base, x - y) / lengths.b;
  WristPhase beta;
  if (!upper.allFinite() || !lower.allFinite() ||
      (!nomination && (std::abs(upper.y()) > numerics::kGeometryTolerance ||
                       std::abs(lower.y()) > numerics::kGeometryTolerance)) ||
      !NormalizePhase(-lower.z(), lower.x(), beta)) {
    return false;
  }
  const Vec3 u3 = UndoShoulder(beta, UndoBase(base, u));
  WristPhase wrist;
  if ((!nomination && std::abs(u3.x()) > numerics::kGeometryTolerance) ||
      !NormalizePhase(-u3.z(), u3.y(), wrist)) {
    return false;
  }
  const Vec3 v3 = UndoShoulder(beta, UndoBase(base, rotation.col(2)));
  const Vec3 v4(v3.x(), wrist.cosine * v3.y() - wrist.sine * v3.z(),
                wrist.sine * v3.y() + wrist.cosine * v3.z());
  if (!nomination && std::abs(v4.y()) > numerics::kGeometryTolerance) {
    return false;
  }
  // Ry(-q5) leaves the second column of R4 unchanged. Only its two dot
  // products with the target are needed for q6, not R5 or a wrist matrix.
  const Vec3 wrist_axis(
      -base.sine * wrist.cosine - base.cosine * beta.sine * wrist.sine,
      base.cosine * wrist.cosine - base.sine * beta.sine * wrist.sine,
      -beta.cosine * wrist.sine);
  q[0] = std::atan2(base.sine, base.cosine);
  q[3] = std::atan2(wrist.sine, wrist.cosine);
  if (!Angle(upper.x(), upper.z(), q[1]) || !Angle(v4.z(), v4.x(), q[4]) ||
      !Angle(wrist_axis.dot(rotation.col(0)), -wrist_axis.dot(rotation.col(1)),
             q[5])) {
    return false;
  }
  q[2] = std::remainder(q[1] - std::atan2(beta.sine, beta.cosine),
                        2.0 * angle_conv::kPi);
  return q.allFinite();
}

// Recompute from the recovered joint vector, not the elbow/wrist intermediates.
auto ForwardPose(const Lengths &lengths, const Vec6 &q) -> PoseIsoRT {
  const Mat3 r1 = Eigen::AngleAxisd(q[0], Vec3::UnitZ()).toRotationMatrix();
  const Mat3 r2 = r1 * Eigen::AngleAxisd(q[1], Vec3::UnitY());
  const Mat3 r3 = r2 * Eigen::AngleAxisd(-q[2], Vec3::UnitY());
  const Mat3 r4 = r3 * Eigen::AngleAxisd(-q[3], Vec3::UnitX());
  const Mat3 r5 = r4 * Eigen::AngleAxisd(-q[4], Vec3::UnitY());
  PoseIsoRT pose = PoseIsoRT::Identity();
  pose.linear() =
      r5 * Eigen::AngleAxisd(-q[5], Vec3::UnitX()) * FlangeConvention();
  pose.translation() = lengths.a * r2.col(2) +
                       r4 * Vec3(lengths.b, -lengths.r, 0) +
                       lengths.c * r5.col(0);
  return pose;
}

auto MapIncidenceStatus(IncidenceStatus status) -> RecoveryStatus {
  switch (status) {
  case IncidenceStatus::PointCandidates:
    return RecoveryStatus::Candidates;
  case IncidenceStatus::NoCandidate:
    return RecoveryStatus::NoCandidate;
  case IncidenceStatus::NeedsRefinement:
    return RecoveryStatus::NeedsRefinement;
  case IncidenceStatus::InvalidInput:
    return RecoveryStatus::InvalidInput;
  case IncidenceStatus::NumericalRangeFailure:
    return RecoveryStatus::NumericalRangeFailure;
  }
  return RecoveryStatus::InvalidInput;
}
} // namespace

auto PolishJointCandidate(const Lengths &lengths, const PoseIsoRT &target,
                          const PoseTolerance &tolerance, int budget,
                          JointCandidate &candidate, const Vec6 *lower,
                          const Vec6 *upper, bool improve_accuracy) -> bool {
  WristCircle circle;
  if (PrepareWristCircle(lengths, target, circle) != PreparationStatus::Ready ||
      budget <= 0 || budget > numerics::kMaxPolishIterations ||
      !candidate.joints.allFinite() || !std::isfinite(tolerance.position) ||
      !std::isfinite(tolerance.orientation) || tolerance.position <= 0 ||
      tolerance.orientation <= 0 || tolerance.orientation >= angle_conv::kPi ||
      (lower == nullptr) != (upper == nullptr)) {
    return false;
  }
  if (lower != nullptr && (!lower->allFinite() || !upper->allFinite() ||
                           (lower->array() > upper->array()).any())) {
    return false;
  }
  const auto &l = circle.lengths;
  const Vec3 position = target.translation() / circle.length_scale;
  auto residual = [&](const Vec6 &q) -> Vec6 {
    const auto pose = ForwardPose(l, q);
    const Eigen::AngleAxisd rotation(target.linear() *
                                     pose.linear().transpose());
    Vec6 error;
    error.head<3>() = position - pose.translation();
    error.tail<3>() = rotation.angle() * rotation.axis();
    return error;
  };
  Vec6 q = candidate.joints;
  if (lower != nullptr && upper != nullptr) {
    q = q.cwiseMax(*lower).cwiseMin(*upper);
  }
  Vec6 error = residual(q);
  for (int iteration = 0; iteration < budget; ++iteration) {
    if (error.isZero(improve_accuracy ? numerics::kAccurateCorrectionTolerance
                                      : numerics::kPoseCorrectionTolerance) &&
        error.head<3>().norm() * circle.length_scale <= tolerance.position &&
        error.tail<3>().norm() <= tolerance.orientation) {
      break;
    }
    const Mat3 r1 = Eigen::AngleAxisd(q[0], Vec3::UnitZ()).toRotationMatrix();
    const Mat3 r2 = r1 * Eigen::AngleAxisd(q[1], Vec3::UnitY());
    const Mat3 r3 = r2 * Eigen::AngleAxisd(-q[2], Vec3::UnitY());
    const Mat3 r4 = r3 * Eigen::AngleAxisd(-q[3], Vec3::UnitX());
    const Mat3 r5 = r4 * Eigen::AngleAxisd(-q[4], Vec3::UnitY());
    const Vec3 elbow = l.a * r2.col(2);
    const Vec3 wrist = elbow + r4 * Vec3(l.b, -l.r, 0);
    const Vec3 flange = wrist + l.c * r5.col(0);
    const std::array<Vec3, 6> axes{Vec3::UnitZ(), r1.col(1),  -r2.col(1),
                                   -r3.col(0),    -r4.col(1), -r5.col(0)};
    const std::array<Vec3, 6> origins{Vec3::Zero(), Vec3::Zero(), elbow,
                                      elbow,        wrist,        flange};
    Eigen::Matrix<double, 6, 6> jacobian;
    for (int joint = 0; joint < 6; ++joint) {
      jacobian.block<3, 1>(0, joint) =
          axes[joint].cross(flange - origins[joint]);
      jacobian.block<3, 1>(3, joint) = axes[joint];
    }
    auto solve = [&]() -> Vec6 {
      const Eigen::PartialPivLU<Eigen::Matrix<double, 6, 6>> lu(jacobian);
      if (lu.rcond() > numerics::kSqrtEpsilon) {
        const Vec6 correction = lu.solve(error);
        if (correction.allFinite() &&
            (jacobian * correction - error).norm() <=
                numerics::kGeometryTolerance * error.norm()) {
          return correction;
        }
      }
      auto svd =
          jacobian.jacobiSvd<Eigen::ComputeFullU | Eigen::ComputeFullV>();
      if (improve_accuracy) {
        svd.setThreshold(numerics::kSqrtEpsilon);
      }
      return svd.solve(error);
    };
    Vec6 step = solve();
    if (lower != nullptr && upper != nullptr) {
      bool active = false;
      for (int joint = 0; joint < 6; ++joint) {
        if ((q[joint] <= (*lower)[joint] && step[joint] < 0) ||
            (q[joint] >= (*upper)[joint] && step[joint] > 0)) {
          jacobian.col(joint).setZero();
          active = true;
        }
      }
      if (active) {
        step = solve();
      }
    }
    if (!step.allFinite()) {
      return false;
    }
    const auto advance = [&](Vec6 correction) {
      const double largest = correction.cwiseAbs().maxCoeff();
      if (largest > numerics::kPolishMaxStepRad) {
        correction *= numerics::kPolishMaxStepRad / largest;
      }
      for (int backtrack = 0; backtrack < numerics::kPolishBacktracks;
           ++backtrack) {
        Vec6 trial = q + correction;
        if (lower != nullptr && upper != nullptr) {
          trial = trial.cwiseMax(*lower).cwiseMin(*upper);
        }
        const Vec6 next = residual(trial);
        if (next.allFinite() && next.squaredNorm() < error.squaredNorm()) {
          q = trial;
          error = next;
          return true;
        }
        correction *= .5;
      }
      return false;
    };
    if (!advance(step)) {
      if (!improve_accuracy) {
        break;
      }
      // Near a multiple root, the Newton direction can be dominated by a
      // nearly null joint motion. A damped least-squares descent gives the
      // next iteration a usable local linearization without changing gates.
      Eigen::Matrix<double, 6, 6> normal = jacobian.transpose() * jacobian;
      normal.diagonal().array() +=
          std::max(numerics::kEpsilon, .001 * error.norm());
      const Vec6 damped = normal.ldlt().solve(jacobian.transpose() * error);
      if (!damped.allFinite() || !advance(damped)) {
        break;
      }
    }
  }
  candidate.position_error = error.head<3>().norm() * circle.length_scale;
  // AngleAxis drives the update, but can round a tiny symmetric matrix defect
  // to zero. Use the same full-matrix closure gate as direct recovery so an
  // unrealistically tight caller tolerance cannot publish that posture.
  candidate.orientation_error =
      2 * std::asin(std::min(
              1.0, (ForwardPose(l, q).linear() - target.linear()).norm() /
                       (2 * std::sqrt(2.0))));
  if (!error.allFinite() || candidate.position_error > tolerance.position ||
      candidate.orientation_error > tolerance.orientation) {
    return false;
  }
  candidate.joints =
      q.unaryExpr([](double angle) { return std::remainder(angle, kTwoPi); });
  return true;
}

auto RefineJointCandidates(const Lengths &lengths, const PoseIsoRT &target,
                           double wrist_angle, const PoseTolerance &tolerance,
                           int budget, double radius,
                           bool singular_representatives) -> JointRecovery {
  JointRecovery result;
  result.status = RecoveryStatus::NeedsRefinement;
  WristCircle circle;
  if (budget <= 0 ||
      PrepareWristCircle(lengths, target, circle) != PreparationStatus::Ready) {
    return result;
  }
  Vec3 u = circle.v * std::cos(wrist_angle) + circle.w * std::sin(wrist_angle);
  const Vec3 x = circle.p + circle.lengths.r * u;
  const double a = circle.lengths.a, b = circle.lengths.b;
  const double distance = x.norm();
  const bool folded =
      singular_representatives &&
      distance < numerics::kFoldedChartArmFraction *
                     std::min(std::abs(a), std::abs(b)) &&
      numerics::Scalar(std::abs(a))
          .isApproxToConstant(std::abs(b), numerics::kDirectionTolerance);
  std::array<Vec3, 4> elbows;
  std::size_t elbow_count = 2;
  Vec3 folded_point = x;
  Vec3 reconstruction = x;
  if (folded) {
    // All split nominations for this exceptional neighbourhood use the same
    // target-derived representative seeds. Root-dependent seeds produce an
    // arbitrary cloud of indistinguishable near-family postures.
    const double closest =
        std::atan2(circle.p.dot(circle.w), circle.p.dot(circle.v)) +
        (circle.lengths.r >= 0 ? angle_conv::kPi : 0.0);
    u = circle.v * std::cos(closest) + circle.w * std::sin(closest);
    Vec3 direction = Vec3::UnitZ().cross(u);
    if (direction.isZero(numerics::kDirectionTolerance)) {
      direction = Vec3::UnitX();
    } else {
      direction.normalize();
    }
    elbows[0] = a * direction;
    elbows[1] = -a * direction;
    reconstruction.setZero();
    folded_point = circle.p + circle.lengths.r * u;
    const double folded_distance = folded_point.norm();
    if (folded_distance > numerics::kDirectionTolerance) {
      const Vec3 radial = folded_point / folded_distance;
      Vec3 perpendicular = Vec3::UnitZ() - radial.z() * radial;
      if (!perpendicular.isZero(numerics::kDirectionTolerance)) {
        perpendicular.normalize();
        const double along = ((a - b) * (a + b) + folded_point.squaredNorm()) /
                             (2 * folded_distance);
        const Vec3 offset =
            std::sqrt(std::max(0.0, a * a - along * along)) * perpendicular;
        elbows[2] = along * radial + offset;
        elbows[3] = along * radial - offset;
        elbow_count = elbows.size();
      }
    }
  } else {
    const auto found = FindElbowCandidates(a, b, x, u);
    if (found.count > 0) {
      elbows[0] = found.points[0];
      elbows[1] = found.points[1];
      if (found.count == 1) {
        elbows[1] = elbows[0];
      }
    } else {
      if (distance <= numerics::kDirectionTolerance) {
        return result;
      }
      const Vec3 radial = x / distance;
      const double along =
          ((a - b) * (a + b) + x.squaredNorm()) / (2 * distance);
      Vec3 perpendicular = Vec3::UnitZ() - radial.z() * radial;
      if (perpendicular.isZero(numerics::kDirectionTolerance)) {
        return result;
      }
      perpendicular.normalize();
      const Vec3 offset =
          std::sqrt(std::max(0.0, a * a - along * along)) * perpendicular;
      elbows[0] = along * radial + offset;
      elbows[1] = along * radial - offset;
    }
  }
  for (std::size_t i = 0; i < elbow_count; ++i) {
    const Vec3 &elbow = elbows[i];
    const Vec3 point = i < 2 ? reconstruction : folded_point;
    const Vec3 base_point =
        std::hypot(point.x(), point.y()) > std::hypot(elbow.x(), elbow.y())
            ? point
            : elbow;
    WristPhase base;
    NormalizePhase(base_point.y(), base_point.x(), base);
    for (const auto &phase : {base, WristPhase{-base.cosine, -base.sine}}) {
      JointCandidate candidate;
      if (!RecoverAtBase(phase, circle.lengths, target.linear(), point, u,
                         elbow, candidate.joints, true) ||
          !PolishJointCandidate(lengths, target, tolerance, budget,
                                candidate)) {
        continue;
      }
      const Vec6 &q = candidate.joints;
      const Mat3 r4 = (Eigen::AngleAxisd(q[0], Vec3::UnitZ()) *
                       Eigen::AngleAxisd(q[1] - q[2], Vec3::UnitY()) *
                       Eigen::AngleAxisd(-q[3], Vec3::UnitX()))
                          .toRotationMatrix();
      const double corrected =
          std::atan2(r4.col(1).dot(circle.w), r4.col(1).dot(circle.v));
      if (std::abs(std::remainder(corrected - wrist_angle, kTwoPi)) <= radius) {
        result.candidates[result.count++] = candidate;
      }
    }
  }
  if (result.count > 0) {
    result.status = RecoveryStatus::Candidates;
    result.used_singular_representatives = folded;
  }
  return result;
}

auto RecoverJointCandidates(const Lengths &lengths, const PoseIsoRT &target,
                            double wrist_angle, const PoseTolerance &tolerance)
    -> JointRecovery {
  if (!std::isfinite(wrist_angle)) {
    return {};
  }
  return RecoverJointCandidates(
      lengths, target, WristPhase{std::cos(wrist_angle), std::sin(wrist_angle)},
      tolerance);
}

auto RecoverJointCandidates(const Lengths &lengths, const PoseIsoRT &target,
                            const WristPhase &phase,
                            const PoseTolerance &tolerance) -> JointRecovery {
  JointRecovery result;
  if (!std::isfinite(phase.cosine) || !std::isfinite(phase.sine) ||
      std::abs(phase.cosine * phase.cosine + phase.sine * phase.sine - 1.0) >
          numerics::kUnitTolerance ||
      !std::isfinite(tolerance.position) ||
      !std::isfinite(tolerance.orientation) || tolerance.position <= 0.0 ||
      tolerance.orientation <= 0.0 ||
      tolerance.orientation >= angle_conv::kPi) {
    return result;
  }
  WristCircle circle;
  const auto preparation = PrepareWristCircle(lengths, target, circle);
  if (preparation != PreparationStatus::Ready) {
    result.status = preparation == PreparationStatus::InvalidInput
                        ? RecoveryStatus::InvalidInput
                        : RecoveryStatus::NumericalRangeFailure;
    return result;
  }
  const Vec3 u = circle.v * phase.cosine + circle.w * phase.sine;
  const Vec3 x = circle.p + circle.lengths.r * u;
  const auto elbows =
      FindElbowCandidates(circle.lengths.a, circle.lengths.b, x, u);
  result.status = MapIncidenceStatus(elbows.status);
  if (result.status != RecoveryStatus::Candidates) {
    return result;
  }
  const Vec3 target_position = target.translation() / circle.length_scale;
  decltype(result.candidates) candidates{};
  std::size_t count = 0;
  bool used_singular_representatives = elbows.used_singular_representatives;
  for (std::size_t elbow = 0; elbow < elbows.count; ++elbow) {
    // At a vertical wrist point the elbow fixes the base plane. If both arm
    // points are on-axis, use base=0 as a deterministic singular-family
    // representative; the wrist angles still come from the target rotation.
    const Vec3 base_point =
        std::hypot(x.x(), x.y()) >
                std::hypot(elbows.points[elbow].x(), elbows.points[elbow].y())
            ? x
            : elbows.points[elbow];
    WristPhase base;
    if (!NormalizePhase(base_point.y(), base_point.x(), base)) {
      used_singular_representatives = true;
    }
    for (const WristPhase &base_phase :
         {base, WristPhase{-base.cosine, -base.sine}}) {
      JointCandidate candidate;
      if (!RecoverAtBase(base_phase, circle.lengths, target.linear(), x, u,
                         elbows.points[elbow], candidate.joints)) {
        result.status = RecoveryStatus::NeedsRefinement;
        return result;
      }
      const PoseIsoRT forward = ForwardPose(circle.lengths, candidate.joints);
      candidate.position_error =
          (forward.translation() - target_position).norm() *
          circle.length_scale;
      // Chordal rotation distance gives 2*asin(||R-Rtarget||F/(2*sqrt(2))).
      // Unlike acos(trace), this retains resolution near zero rotation error.
      candidate.orientation_error =
          2.0 *
          std::asin(std::min(1.0, (forward.linear() - target.linear()).norm() /
                                      (2.0 * std::sqrt(2.0))));
      if (!forward.matrix().allFinite() ||
          !std::isfinite(candidate.position_error) ||
          !std::isfinite(candidate.orientation_error)) {
        result.status = RecoveryStatus::NumericalRangeFailure;
        return result;
      }
      if (candidate.position_error > tolerance.position ||
          candidate.orientation_error > tolerance.orientation) {
        result.status = RecoveryStatus::NeedsRefinement;
        return result;
      }
      candidates[count++] = candidate;
    }
  }
  result.candidates = candidates;
  result.count = count;
  result.used_singular_representatives = used_singular_representatives;
  return result;
}

} // namespace crx::canonical
