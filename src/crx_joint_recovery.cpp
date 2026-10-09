#include "crx_joint_recovery.h"
#include "crx_canonical.h"
#include "crx_incidence.h"
#include "crx_types.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>

namespace crx::canonical {
namespace {
constexpr double kPi = 3.14159265358979323846;
constexpr double kGeometryTolerance =
    4096.0 * std::numeric_limits<double>::epsilon();

auto FlangeConvention() -> Mat3 {
  Mat3 result;
  result << 0, 0, 1, 0, -1, 0, 1, 0, 0;
  return result;
}

auto Angle(double sine, double cosine, double &angle) -> bool {
  const double magnitude = std::hypot(sine, cosine);
  if (!std::isfinite(magnitude) || magnitude <= kGeometryTolerance) {
    return false;
  }
  angle = std::atan2(sine, cosine);
  return true;
}

auto NormalizePhase(double sine, double cosine, WristPhase &phase) -> bool {
  const double magnitude = std::hypot(sine, cosine);
  if (!std::isfinite(magnitude) || magnitude <= kGeometryTolerance) {
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
                   const Vec3 &y, Vec6 &q) -> bool {
  const Vec3 upper = UndoBase(base, y) / lengths.a;
  const Vec3 lower = UndoBase(base, x - y) / lengths.b;
  WristPhase beta;
  if (!upper.allFinite() || !lower.allFinite() ||
      std::abs(upper.y()) > kGeometryTolerance ||
      std::abs(lower.y()) > kGeometryTolerance ||
      !NormalizePhase(-lower.z(), lower.x(), beta)) {
    return false;
  }
  const Vec3 u3 = UndoShoulder(beta, UndoBase(base, u));
  WristPhase wrist;
  if (std::abs(u3.x()) > kGeometryTolerance ||
      !NormalizePhase(-u3.z(), u3.y(), wrist)) {
    return false;
  }
  const Vec3 v3 = UndoShoulder(beta, UndoBase(base, rotation.col(2)));
  const Vec3 v4(v3.x(), wrist.cosine * v3.y() - wrist.sine * v3.z(),
                wrist.sine * v3.y() + wrist.cosine * v3.z());
  if (std::abs(v4.y()) > kGeometryTolerance) {
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
  q[2] = std::remainder(q[1] - std::atan2(beta.sine, beta.cosine), 2.0 * kPi);
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
          128.0 * std::numeric_limits<double>::epsilon() ||
      !std::isfinite(tolerance.position) ||
      !std::isfinite(tolerance.orientation) || tolerance.position <= 0.0 ||
      tolerance.orientation <= 0.0 || tolerance.orientation >= kPi) {
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
  std::array<JointCandidate, 4> candidates{};
  std::size_t count = 0;
  for (std::size_t elbow = 0; elbow < elbows.count; ++elbow) {
    // At a vertical wrist point the elbow fixes the base plane. If both arm
    // points are on-axis, use base=0 as a deterministic singular-family
    // representative; the wrist angles still come from the target rotation.
    const Vec3 base_point = std::hypot(x.x(), x.y()) > kGeometryTolerance
                                ? x
                                : elbows.points[elbow];
    WristPhase base;
    NormalizePhase(base_point.y(), base_point.x(), base);
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
  return result;
}

} // namespace crx::canonical
