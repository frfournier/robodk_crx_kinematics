// SPDX-License-Identifier: Apache-2.0
// SPDX-FileCopyrightText: 2026 Centre Technologique en Aérospatiale

#include "crx_certified_solver.h"
#include "crx_ball_certificate.h"
#include "crx_exact_problem.h"
#include "crx_interval.h"
#include "crx_numerics.h"
#include "crx_solver.h"
#include "crx_types.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <utility>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace crx {
namespace {
auto ValidFrame(const PoseIsoRT &pose) -> bool {
  if (!pose.matrix().allFinite() || pose.matrix()(3, 0) != 0 ||
      pose.matrix()(3, 1) != 0 || pose.matrix()(3, 2) != 0 ||
      pose.matrix()(3, 3) != 1) {
    return false;
  }
  const Mat3 rotation = pose.linear();
  return (rotation.transpose() * rotation - Mat3::Identity()).norm() <=
             numerics::kUnitTolerance &&
         std::abs(rotation.determinant() - 1) <= numerics::kUnitTolerance;
}

auto Nomination(const PoseIsoRT &target, const PoseIsoRT &reference,
                const PoseIsoRT &tool, PoseIsoRT &flange) -> bool {
  if (!ValidFrame(target) || !ValidFrame(reference) || !ValidFrame(tool)) {
    return false;
  }
  flange = reference * target * tool.inverse();
  return flange.matrix().allFinite();
}

auto PreparationFailure(exact::Status status) -> ball::Failure {
  switch (status) {
  case exact::Status::Ready:
    return ball::Failure::None;
  case exact::Status::InvalidInput:
    return ball::Failure::InvalidInput;
  case exact::Status::UnsupportedNominalModel:
    return ball::Failure::UnsupportedExactModel;
  case exact::Status::UnsupportedEnvironment:
    return ball::Failure::UnsupportedEnvironment;
  case exact::Status::ArithmeticFailure:
    return ball::Failure::ArithmeticUnavailable;
  }
  return ball::Failure::InvalidInput;
}

auto CanonicalCentre(const exact::Problem &problem, const Vec6 &command,
                     ball::Centre &centre) -> bool {
  const auto &data = problem.Data();
  for (std::size_t i = 0; i < centre.size(); ++i) {
    const auto angle =
        interval::Interval{data.diagonal.at(i)} *
            interval::Interval{command(static_cast<Eigen::Index>(i))} +
        data.offset_unwrapped.at(i);
    if (!angle.IsValid()) {
      return false;
    }
    // A represented point proposal, never a claim that the rounded command
    // maps exactly to this centre. Certify records and checks this point.
    centre.at(i) = (angle.lo / 2) + (angle.hi / 2);
    if (!std::isfinite(centre.at(i))) {
      return false;
    }
  }
  return true;
}

auto ReuseReserve(const ball::Certificate &certificate,
                  const ball::Centre &nomination) -> bool {
  double distance = 0;
  const auto &data = certificate.Values();
  for (std::size_t i = 0; i < nomination.size(); ++i) {
    const auto difference =
        interval::Abs(interval::Interval{nomination.at(i)} -
                      interval::Interval{data.centre.at(i)});
    if (!difference.IsValid()) {
      return false;
    }
    distance = std::max(distance, difference.hi);
  }
  const auto used =
      interval::Interval{distance} + interval::Interval{data.existence_radius};
  return used.IsValid() && used.hi <= data.uniqueness_radius;
}

auto Attach(const exact::Problem &problem, exact::Status preparation,
            const Vec6 &command, int expected_aspect, bool strict,
            CertificateOptions options) -> CertificateAttachment {
  CertificateAttachment result;
  if (!options.enabled) {
    result.attempt.failure = ball::Failure::NotRequested;
    return result;
  }
  if (!strict) {
    result.attempt.failure = ball::Failure::NotStrict;
    return result;
  }
  if (preparation != exact::Status::Ready) {
    result.attempt.failure = PreparationFailure(preparation);
    return result;
  }
  result.length_scale = problem.Data().length_scale;
  for (std::size_t i = 0; i < result.rotation_deviation_upper.size(); ++i) {
    result.rotation_deviation_upper.at(i) =
        problem.Data().rotation_deviation.at(i).hi;
  }
  ball::Centre centre;
  if (!CanonicalCentre(problem, command, centre)) {
    result.attempt.failure = ball::Failure::ArithmeticUnavailable;
    return result;
  }
  if (options.previous != nullptr) {
    auto reuse = ball::TryReuse(*options.previous, problem);
    result.reuse_failure = reuse.failure;
    if (reuse.IsCertified() && ReuseReserve(reuse.certificate, centre) &&
        (expected_aspect == 0 ||
         expected_aspect == reuse.certificate.Values().aspect_sign)) {
      result.attempt = std::move(reuse);
      result.reused = true;
      return result;
    }
    if (result.reuse_failure == ball::Failure::None) {
      result.reuse_failure =
          expected_aspect != 0 &&
                  expected_aspect != reuse.certificate.Values().aspect_sign
              ? ball::Failure::AspectDisagreement
              : ball::Failure::RadiusInequality;
    }
  }
  result.attempt =
      ball::Certify(problem, centre, {0, nullptr, expected_aspect});
  return result;
}

auto Strict(IkStatus status) -> bool {
  return status == IkStatus::StrictCandidates ||
         status == IkStatus::StrictSeed || status == IkStatus::StrictLocal;
}
} // namespace

auto SolveIkCertified(const CrxModelData &model, const PoseIsoRT &target,
                      const PoseIsoRT &reference, const PoseIsoRT &tool,
                      const Vec6 *seed, int capacity,
                      CertificateOptions options) -> CertifiedIkResult {
  CertifiedIkResult output;
  PoseIsoRT flange;
  if (!Nomination(target, reference, tool, flange)) {
    return output;
  }
  output.result = SolveIkIsometry(model, flange, seed, capacity);
  exact::Problem problem;
  auto preparation = exact::Status::InvalidInput;
  if (options.enabled && Strict(output.result.status) &&
      !output.result.solutions.empty()) {
    const Eigen::Quaterniond source(target.linear());
    preparation = exact::PrepareReference(model, target, source, reference,
                                          tool, problem);
  }
  output.certificates.reserve(output.result.solutions.size());
  for (const auto &command : output.result.solutions) {
    output.certificates.push_back(Attach(problem, preparation, command, 0,
                                         Strict(output.result.status),
                                         options));
  }
  return output;
}

auto IdentifyIkCertified(const CrxModelData &model, const PoseIsoRT &target,
                         const PoseIsoRT &reference, const PoseIsoRT &tool,
                         int capacity, CertificateOptions options)
    -> CertifiedFibreResult {
  CertifiedFibreResult output;
  PoseIsoRT flange;
  if (!Nomination(target, reference, tool, flange)) {
    return output;
  }
  output.result = IdentifyIkIsometry(model, flange, capacity);
  exact::Problem problem;
  auto preparation = exact::Status::InvalidInput;
  if (options.enabled && !output.result.solutions.empty()) {
    const Eigen::Quaterniond source(target.linear());
    preparation = exact::PrepareReference(model, target, source, reference,
                                          tool, problem);
  }
  output.certificates.reserve(output.result.solutions.size());
  for (auto &solution : output.result.solutions) {
    auto attached = Attach(problem, preparation, solution.joints,
                           solution.regular ? solution.aspect_sign : 0,
                           Strict(output.result.status), options);
    if (attached.attempt.IsCertified()) {
      solution.regular = true;
      solution.aspect_sign = attached.attempt.certificate.Values().aspect_sign;
    }
    output.certificates.push_back(std::move(attached));
  }
  return output;
}
} // namespace crx
