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

auto Rx(double angle) -> Mat3 {
  const double c = std::cos(angle);
  const double s = std::sin(angle);
  Mat3 result;
  result << 1, 0, 0, 0, c, -s, 0, s, c;
  return result;
}

auto Ry(double angle) -> Mat3 {
  const double c = std::cos(angle);
  const double s = std::sin(angle);
  Mat3 result;
  result << c, 0, s, 0, 1, 0, -s, 0, c;
  return result;
}

auto Rz(double angle) -> Mat3 {
  const double c = std::cos(angle);
  const double s = std::sin(angle);
  Mat3 result;
  result << c, -s, 0, s, c, 0, 0, 0, 1;
  return result;
}

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

auto RecoverAtBase(double base, const Lengths &lengths, const Mat3 &rotation,
                   const Vec3 &x, const Vec3 &u, const Vec3 &y, Vec6 &q)
    -> bool {
  q[0] = std::remainder(base, 2.0 * kPi);
  const Mat3 base_rotation = Rz(q[0]);
  const Vec3 upper = (base_rotation.transpose() * y) / lengths.a;
  const Vec3 lower = (base_rotation.transpose() * (x - y)) / lengths.b;
  double beta = 0.0;
  if (!upper.allFinite() || !lower.allFinite() ||
      std::abs(upper.y()) > kGeometryTolerance ||
      std::abs(lower.y()) > kGeometryTolerance ||
      !Angle(upper.x(), upper.z(), q[1]) ||
      !Angle(-lower.z(), lower.x(), beta)) {
    return false;
  }
  q[2] = std::remainder(q[1] - beta, 2.0 * kPi);
  const Mat3 r3 = base_rotation * Ry(q[1] - q[2]);
  const Vec3 u3 = r3.transpose() * u;
  if (std::abs(u3.x()) > kGeometryTolerance || !Angle(-u3.z(), u3.y(), q[3])) {
    return false;
  }
  const Mat3 r4 = r3 * Rx(-q[3]);
  const Vec3 v4 = r4.transpose() * rotation.col(2);
  if (std::abs(v4.y()) > kGeometryTolerance || !Angle(v4.z(), v4.x(), q[4])) {
    return false;
  }
  const Mat3 r5 = r4 * Ry(-q[4]);
  const Mat3 wrist = r5.transpose() * rotation * FlangeConvention().transpose();
  return Angle(wrist(1, 2), wrist(1, 1), q[5]) && q.allFinite();
}

// Recompute from the recovered joint vector, not the elbow/wrist intermediates.
auto ForwardPose(const Lengths &lengths, const Vec6 &q) -> PoseIsoRT {
  const Mat3 r1 = Rz(q[0]);
  const Mat3 r2 = r1 * Ry(q[1]);
  const Mat3 r3 = r2 * Ry(-q[2]);
  const Mat3 r4 = r3 * Rx(-q[3]);
  const Mat3 r5 = r4 * Ry(-q[4]);
  PoseIsoRT pose = PoseIsoRT::Identity();
  pose.linear() = r5 * Rx(-q[5]) * FlangeConvention();
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
  JointRecovery result;
  if (!std::isfinite(wrist_angle) || !std::isfinite(tolerance.position) ||
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
  const Vec3 u =
      circle.v * std::cos(wrist_angle) + circle.w * std::sin(wrist_angle);
  const Vec3 x = circle.p + circle.lengths.r * u;
  const auto elbows =
      FindElbowCandidates(circle.lengths.a, circle.lengths.b, x, u);
  result.status = MapIncidenceStatus(elbows.status);
  if (result.status != RecoveryStatus::Candidates) {
    return result;
  }
  const double base = std::atan2(x.y(), x.x());
  const Vec3 target_position = target.translation() / circle.length_scale;
  std::array<JointCandidate, 4> candidates{};
  std::size_t count = 0;
  for (std::size_t elbow = 0; elbow < elbows.count; ++elbow) {
    for (const double base_angle : {base, base + kPi}) {
      JointCandidate candidate;
      if (!RecoverAtBase(base_angle, circle.lengths, target.linear(), x, u,
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
