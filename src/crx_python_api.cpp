// SPDX-License-Identifier: Apache-2.0
// SPDX-FileCopyrightText: 2026 Centre Technologique en Aérospatiale
/**
 * @file crx_python_api.cpp
 * @brief Implementation of Python C ABI entry points.
 */

#include "crx_python_api.h"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "crx_certified_solver.h"
#include "crx_kinematics.h"
#include "crx_numerics.h"
#include "crx_solver.h"
#include "crx_types.h"

struct crx_v1_handle {
  crx::CrxModelData model;
};

struct crx_v1_certificate_handle {
  crx::CertificateAttachment attachment;
};

namespace {
static_assert(static_cast<int>(crx::ball::Failure::UnsupportedExactModel) ==
              CRX_CERT_UNSUPPORTED_EXACT_MODEL);
constexpr int kConfigSize = 3;
constexpr int kModelCount = 7;
constexpr int kSpatialSize = 3;
constexpr int kWristAddressCount = 3;
using PoseRow = Eigen::Matrix<double, crx::kPoseRowCount, crx::kPoseColCount,
                              Eigen::RowMajor>;
using JacobianRow =
    Eigen::Matrix<double, crx::kDofCount, crx::kDofCount, Eigen::RowMajor>;
using LimitsRow = Eigen::Matrix<double, 2, crx::kDofCount, Eigen::RowMajor>;
using SolutionsRow =
    Eigen::Matrix<double, Eigen::Dynamic, crx::kDofCount, Eigen::RowMajor>;
using IntVector = Eigen::Matrix<std::int32_t, Eigen::Dynamic, 1>;
using WristAddressRow =
    Eigen::Matrix<std::int32_t, 1, kWristAddressCount, Eigen::RowMajor>;
using WristAddresses = Eigen::Matrix<std::int32_t, Eigen::Dynamic,
                                     kWristAddressCount, Eigen::RowMajor>;

struct ModelSpec {
  const char *name;
  double height;
  double a;
  double b;
  double offset;
  double flange;
  std::array<double, crx::kDofCount> lower;
  std::array<double, crx::kDofCount> upper;
};

// Nominal geometry and total joint travel follow FANUC's CRX specifications.
// Joint limits are set to the default in ROBOGUIDE from the base 9.40 image
constexpr std::array<ModelSpec, kModelCount> kModels{{
    {"CRX-3iA",
     161,
     280,
     280,
     111,
     123,
     {-200.000, -179.990, -206.000, -190.000, -179.990, -225.000},
     {200.000, 179.990, 386.000, 190.000, 179.990, 225.000}},
    {"CRX-5iA",
     185,
     410,
     430,
     130,
     145,
     {-200.000, -179.990, -225.000, -190.000, -179.990, -225.000},
     {200.000, 179.990, 410.000, 190.000, 179.990, 225.000}},
    {"CRX-10iA",
     245,
     540,
     540,
     150,
     160,
     {-190.000, -179.990, -195.000, -190.000, -179.990, -225.000},
     {190.000, 179.990, 375.000, 190.000, 179.990, 225.000}},
    {"CRX-10iA-L",
     245,
     710,
     540,
     150,
     160,
     {-179.990, -179.990, -270.000, -190.000, -179.990, -225.000},
     {179.990, 179.990, 270.000, 190.000, 179.990, 225.000}},
    {"CRX-20iA-L",
     245,
     710,
     540,
     150,
     160,
     {-179.990, -179.990, -270.000, -190.000, -179.990, -225.000},
     {179.990, 179.990, 270.000, 190.000, 179.990, 225.000}},
    {"CRX-25iA",
     370,
     950,
     750,
     185,
     180,
     {-179.990, -179.990, -270.000, -190.000, -179.990, -225.000},
     {179.990, 179.990, 270.000, 190.000, 179.990, 225.000}},
    {"CRX-30iA",
     370,
     950,
     750,
     185,
     180,
     {-179.990, -179.990, -270.000, -190.000, -179.990, -225.000},
     {179.990, 179.990, 270.000, 190.000, 179.990, 225.000}},
}};

auto BuildModel(const ModelSpec &spec) -> crx::CrxModelData {
  crx::CrxModelData model;
  model.base_transform.translation().z() = -spec.height;
  model.joint_senses = {1, 1, -1, -1, -1, -1};
  model.dh_rows = {{{0, 0, 0, spec.height, false},
                    {-crx::kHalfPi, 0, -crx::kHalfPi, 0, false},
                    {0, spec.a, 0, 0, false},
                    {-crx::kHalfPi, 0, 0, spec.b, false},
                    {crx::kHalfPi, 0, 0, -spec.offset, false},
                    {-crx::kHalfPi, 0, 0, spec.flange, false}}};
  model.lower_limits_rad =
      Eigen::Map<const crx::Vec6>(spec.lower.data()) * angle_conv::kRadPerDeg;
  model.upper_limits_rad =
      Eigen::Map<const crx::Vec6>(spec.upper.data()) * angle_conv::kRadPerDeg;
  return model;
}

template <typename Function>
auto Guard(const Function &function) noexcept -> std::int32_t {
  try {
    return function();
  } catch (...) {
    return CRX_INTERNAL_ERROR;
  }
}

auto ReadCommand(const double *joints, crx::Vec6 &command) -> bool {
  if (joints == nullptr) {
    return false;
  }
  command = Eigen::Map<const crx::Vec6>(joints) * angle_conv::kRadPerDeg;
  return command.allFinite();
}

auto Status(crx::IkStatus status) -> std::int32_t {
  switch (status) {
  case crx::IkStatus::StrictCandidates:
    return CRX_STRICT_CANDIDATES;
  case crx::IkStatus::StrictSeed:
    return CRX_STRICT_SEED;
  case crx::IkStatus::StrictLocal:
    return CRX_STRICT_LOCAL;
  case crx::IkStatus::ApproximateSeed:
    return CRX_APPROXIMATE_SEED;
  case crx::IkStatus::NoCandidate:
    return CRX_NO_CANDIDATE;
  case crx::IkStatus::NoOutputRequested:
    return CRX_NO_OUTPUT_REQUESTED;
  case crx::IkStatus::InvalidInput:
    return CRX_INVALID_INPUT;
  case crx::IkStatus::UnsupportedModel:
    return CRX_UNSUPPORTED_MODEL;
  case crx::IkStatus::Unresolved:
    return CRX_UNRESOLVED;
  case crx::IkStatus::NumericalRangeFailure:
    return CRX_NUMERICAL_RANGE_FAILURE;
  case crx::IkStatus::WorkLimit:
    return CRX_WORK_LIMIT;
  }
  return CRX_INVALID_INPUT;
}

auto CopyIdentifiedSolutions(const crx::FibreResult &identified,
                             std::int32_t capacity, double *solutions,
                             std::int32_t *aspect_signs, std::int32_t *regular,
                             std::int32_t *wrist_addresses,
                             std::int32_t *address_valid,
                             crx_v1_fibre_result *result) -> bool {
  if (identified.solutions.size() > static_cast<std::size_t>(capacity)) {
    return false;
  }
  Eigen::Map<SolutionsRow> solution_output(solutions, capacity, crx::kDofCount);
  Eigen::Map<IntVector> aspect_output(aspect_signs, capacity);
  Eigen::Map<IntVector> regular_output(regular, capacity);
  Eigen::Map<WristAddresses> address_output(wrist_addresses, capacity,
                                            kWristAddressCount);
  Eigen::Map<IntVector> address_valid_output(address_valid, capacity);
  for (std::size_t i = 0; i < identified.solutions.size(); ++i) {
    const auto &solution = identified.solutions.at(i);
    const auto row = static_cast<Eigen::Index>(i);
    solution_output.row(row) =
        solution.joints.transpose() * angle_conv::kDegPerRad;
    aspect_output(row) = solution.aspect_sign;
    regular_output(row) = solution.regular ? 1 : 0;
    address_output.row(row) =
        Eigen::Map<const WristAddressRow>(solution.wrist_address.data());
    address_valid_output(row) = solution.wrist_address_valid ? 1 : 0;
  }
  *result = {
      Status(identified.status),
      static_cast<std::int32_t>(identified.solutions.size()),
      static_cast<std::int32_t>(identified.truncated),
      static_cast<std::int32_t>(identified.used_singular_representatives)};
  return true;
}

auto IdentifyWithMetadata(const crx_v1_handle *handle, const double *pose,
                          std::int32_t capacity, double *solutions,
                          std::int32_t *aspect_signs, std::int32_t *regular,
                          std::int32_t *wrist_addresses,
                          std::int32_t *address_valid,
                          crx_v1_fibre_result *result,
                          crx_v1_fibre_metadata *metadata) -> std::int32_t {
  if (result == nullptr) {
    return CRX_INVALID_ARGUMENT;
  }
  *result = {CRX_INVALID_INPUT, 0, 0, 0};
  return Guard([&]() -> std::int32_t {
    if (handle == nullptr || pose == nullptr || solutions == nullptr ||
        aspect_signs == nullptr || regular == nullptr ||
        wrist_addresses == nullptr || address_valid == nullptr ||
        capacity <= 0) {
      return CRX_INVALID_ARGUMENT;
    }
    crx::PoseIsoRT target;
    target.matrix() = Eigen::Map<const PoseRow>(pose);
    const auto identified =
        crx::IdentifyIkIsometry(handle->model, target, capacity);
    if (!CopyIdentifiedSolutions(identified, capacity, solutions, aspect_signs,
                                 regular, wrist_addresses, address_valid,
                                 result)) {
      return CRX_INTERNAL_ERROR;
    }
    if (metadata != nullptr) {
      metadata->coverage = static_cast<std::int32_t>(identified.coverage);
    }
    return CRX_OK;
  });
}

auto ValidCertificateOptions(const crx_v1_certificate_options *options)
    -> bool {
  return options != nullptr && options->struct_size == sizeof(*options) &&
         options->version == 1 && options->reserved == 0 &&
         (options->flags &
          ~static_cast<std::uint32_t>(CRX_CERTIFICATION_DISABLED)) == 0;
}

using PendingCertificates =
    std::vector<std::unique_ptr<crx_v1_certificate_handle>>;

auto StageCertificates(
    const std::vector<crx::CertificateAttachment> &attachments)
    -> PendingCertificates {
  PendingCertificates pending;
  pending.reserve(attachments.size());
  for (const auto &attachment : attachments) {
    if (attachment.attempt.IsCertified()) {
      auto handle = std::make_unique<crx_v1_certificate_handle>();
      handle->attachment = attachment;
      pending.push_back(std::move(handle));
    } else {
      pending.push_back(nullptr);
    }
  }
  return pending;
}

void CommitCertificates(
    PendingCertificates &pending,
    const std::vector<crx::CertificateAttachment> &attachments,
    crx_v1_certificate_handle **output, std::int32_t *failures) {
  for (std::size_t i = 0; i < pending.size(); ++i) {
    output[i] = pending.at(i).release();
    failures[i] = static_cast<std::int32_t>(attachments.at(i).attempt.failure);
  }
}

auto CertificateRecord(const crx_v1_certificate_handle &handle)
    -> crx_v1_ball_certificate {
  crx_v1_ball_certificate result{};
  result.struct_size = sizeof(result);
  result.version = 1;
  const auto &attachment = handle.attachment;
  const auto &data = attachment.attempt.certificate.Values();
  result.identity_schema = data.identity.schema;
  result.aspect_sign = data.aspect_sign;
  result.reused = attachment.reused ? 1 : 0;
  result.used_log_series = data.used_log_series ? 1 : 0;
  result.reuse_failure = static_cast<std::int32_t>(attachment.reuse_failure);
  std::copy(data.centre.begin(), data.centre.end(),
            result.canonical_center_rad);
  result.existence_radius_rad = data.existence_radius;
  result.uniqueness_radius_rad = data.uniqueness_radius;
  result.beta = data.beta;
  result.lipschitz = data.lipschitz;
  result.eta = data.eta;
  result.coarse_eta = data.coarse_eta;
  result.inverse_residual = data.inverse_residual;
  result.pose_defect = data.pose_defect;
  result.screw_norm_sum = data.screw_norm_sum;
  result.angular_norm_sum = data.angular_norm_sum;
  result.anchor_position_norm = data.anchor_position_norm;
  result.length_scale = attachment.length_scale;
  std::copy(attachment.rotation_deviation_upper.begin(),
            attachment.rotation_deviation_upper.end(),
            result.rotation_deviation_upper);
  result.target_relative_trace[0] = data.target_relative_trace.lo;
  result.target_relative_trace[1] = data.target_relative_trace.hi;
  result.closed_form_determinant[0] = data.closed_form_determinant.lo;
  result.closed_form_determinant[1] = data.closed_form_determinant.hi;
  for (int row = 0; row < crx::kDofCount; ++row) {
    for (int col = 0; col < crx::kDofCount; ++col) {
      result.preconditioner[row * crx::kDofCount + col] =
          data.preconditioner(row, col);
    }
  }
  std::copy(data.identity.words.begin(), data.identity.words.end(),
            result.input_words);
  return result;
}
} // namespace

extern "C" {
std::uint32_t crx_v1_abi_version(void) { return 1; }

const char *crx_v1_library_version(void) {
  return CRX_KINEMATICS_VERSION_STRING;
}

const char *crx_v1_model_name(std::int32_t model_id) {
  return model_id >= 0 && model_id < kModelCount
             ? kModels.at(static_cast<std::size_t>(model_id)).name
             : nullptr;
}

std::int32_t crx_v1_create(std::int32_t model_id, const double *limits_deg,
                           crx_v1_handle **output) {
  if (output == nullptr) {
    return CRX_INVALID_ARGUMENT;
  }
  *output = nullptr;
  return Guard([&]() -> std::int32_t {
    if (crx_v1_model_name(model_id) == nullptr) {
      return CRX_INVALID_ARGUMENT;
    }
    auto handle = std::make_unique<crx_v1_handle>();
    handle->model = BuildModel(kModels.at(static_cast<std::size_t>(model_id)));
    if (limits_deg != nullptr) {
      const Eigen::Map<const LimitsRow> limits(limits_deg);
      if (!limits.allFinite() ||
          (limits.row(0).array() > limits.row(1).array()).any()) {
        return CRX_INVALID_ARGUMENT;
      }
      handle->model.lower_limits_rad =
          limits.row(0).transpose() * angle_conv::kRadPerDeg;
      handle->model.upper_limits_rad =
          limits.row(1).transpose() * angle_conv::kRadPerDeg;
    }
    *output = handle.release();
    return CRX_OK;
  });
}

void crx_v1_destroy(crx_v1_handle *handle) {
  const std::unique_ptr<crx_v1_handle> owned(handle);
}

std::int32_t crx_v1_limits(const crx_v1_handle *handle, double *limits_deg) {
  return Guard([&]() -> std::int32_t {
    if (handle == nullptr || limits_deg == nullptr) {
      return CRX_INVALID_ARGUMENT;
    }
    Eigen::Map<LimitsRow> limits(limits_deg);
    limits.row(0) =
        handle->model.lower_limits_rad.transpose() * angle_conv::kDegPerRad;
    limits.row(1) =
        handle->model.upper_limits_rad.transpose() * angle_conv::kDegPerRad;
    return CRX_OK;
  });
}

std::int32_t crx_v1_fk(const crx_v1_handle *handle, const double *joints_deg,
                       double *pose) {
  return Guard([&]() -> std::int32_t {
    crx::Vec6 command;
    if (handle == nullptr || pose == nullptr ||
        !ReadCommand(joints_deg, command)) {
      return CRX_INVALID_ARGUMENT;
    }
    crx::PoseIsoRT result;
    const int status =
        crx::SolveFkIsometry(handle->model, command, result, nullptr, true);
    if (status != 1) {
      return status == -2 ? CRX_JOINT_LIMIT : CRX_INVALID_ARGUMENT;
    }
    Eigen::Map<PoseRow> output(pose);
    output = result.matrix();
    return CRX_OK;
  });
}

std::int32_t crx_v1_configuration(const crx_v1_handle *handle,
                                  const double *joints_deg,
                                  std::int32_t *config) {
  return Guard([&]() -> std::int32_t {
    crx::Vec6 command;
    if (handle == nullptr || config == nullptr ||
        !ReadCommand(joints_deg, command)) {
      return CRX_INVALID_ARGUMENT;
    }
    crx::ArmPosture posture;
    if (!crx::ClassifyArmPosture(handle->model, command, posture)) {
      return CRX_CONFIG_UNDEFINED;
    }
    Eigen::Map<Eigen::Matrix<std::int32_t, kConfigSize, 1>> output(config);
    output << static_cast<std::int32_t>(posture.rear),
        static_cast<std::int32_t>(posture.lower_arm),
        static_cast<std::int32_t>(posture.flip);
    return CRX_OK;
  });
}

std::int32_t crx_v1_jacobian(const crx_v1_handle *handle,
                             const double *joints_deg, double *jacobian) {
  return Guard([&]() -> std::int32_t {
    crx::Vec6 command;
    if (handle == nullptr || jacobian == nullptr ||
        !ReadCommand(joints_deg, command)) {
      return CRX_INVALID_ARGUMENT;
    }
    crx::PoseIsoRT flange;
    crx::JointPoseBuffer frames;
    const int status =
        crx::SolveFkIsometry(handle->model, command, flange, &frames, true);
    if (status != 1) {
      return status == -2 ? CRX_JOINT_LIMIT : CRX_INVALID_ARGUMENT;
    }
    JacobianRow result;
    for (int joint = 0; joint < crx::kDofCount; ++joint) {
      const auto index = static_cast<std::size_t>(joint);
      const auto &frame = frames.at(index + 1U);
      const crx::Vec3 axis =
          handle->model.joint_senses.at(index) * frame.linear().col(2);
      result.block<kSpatialSize, 1>(0, joint) =
          axis.cross(flange.translation() - frame.translation());
      result.block<kSpatialSize, 1>(kSpatialSize, joint) = axis;
    }
    if (!result.allFinite()) {
      return CRX_INTERNAL_ERROR;
    }
    Eigen::Map<JacobianRow> output(jacobian);
    output = result;
    return CRX_OK;
  });
}

std::int32_t crx_v1_ik(const crx_v1_handle *handle, const double *pose,
                       const double *seed_deg, std::int32_t capacity,
                       double *solutions, crx_v1_ik_result *result) {
  if (result == nullptr) {
    return CRX_INVALID_ARGUMENT;
  }
  *result = {CRX_INVALID_INPUT, 0, 0, 0};
  return Guard([&]() -> std::int32_t {
    if (handle == nullptr || pose == nullptr || solutions == nullptr ||
        capacity <= 0) {
      return CRX_INVALID_ARGUMENT;
    }
    crx::Vec6 seed;
    if (seed_deg != nullptr && !ReadCommand(seed_deg, seed)) {
      return CRX_INVALID_ARGUMENT;
    }
    crx::PoseIsoRT target;
    target.matrix() = Eigen::Map<const PoseRow>(pose);
    const auto solved = crx::SolveIkIsometry(
        handle->model, target, seed_deg == nullptr ? nullptr : &seed, capacity);
    if (solved.solutions.size() > static_cast<std::size_t>(capacity)) {
      return CRX_INTERNAL_ERROR;
    }
    Eigen::Map<SolutionsRow> output(solutions, capacity, crx::kDofCount);
    for (std::size_t i = 0; i < solved.solutions.size(); ++i) {
      output.row(static_cast<Eigen::Index>(i)) =
          solved.solutions.at(i).transpose() * angle_conv::kDegPerRad;
    }
    *result = {Status(solved.status),
               static_cast<std::int32_t>(solved.solutions.size()),
               static_cast<std::int32_t>(solved.truncated),
               static_cast<std::int32_t>(solved.used_singular_representatives)};
    return CRX_OK;
  });
}

std::int32_t crx_v1_identify(const crx_v1_handle *handle, const double *pose,
                             std::int32_t capacity, double *solutions,
                             std::int32_t *aspect_signs, std::int32_t *regular,
                             std::int32_t *wrist_addresses,
                             std::int32_t *address_valid,
                             crx_v1_fibre_result *result) {
  return IdentifyWithMetadata(handle, pose, capacity, solutions, aspect_signs,
                              regular, wrist_addresses, address_valid, result,
                              nullptr);
}

std::int32_t crx_v1_identify2(const crx_v1_handle *handle, const double *pose,
                              std::int32_t capacity, double *solutions,
                              std::int32_t *aspect_signs, std::int32_t *regular,
                              std::int32_t *wrist_addresses,
                              std::int32_t *address_valid,
                              crx_v1_fibre_result *result,
                              crx_v1_fibre_metadata *metadata) {
  if (result != nullptr) {
    *result = {CRX_INVALID_INPUT, 0, 0, 0};
  }
  if (metadata == nullptr || metadata->struct_size != sizeof(*metadata) ||
      metadata->version != 1 || metadata->reserved != 0) {
    return CRX_INVALID_ARGUMENT;
  }
  return IdentifyWithMetadata(handle, pose, capacity, solutions, aspect_signs,
                              regular, wrist_addresses, address_valid, result,
                              metadata);
}

void crx_v1_certificate_destroy(crx_v1_certificate_handle *certificate) {
  delete certificate;
}

std::int32_t
crx_v1_certificate_read(const crx_v1_certificate_handle *certificate,
                        crx_v1_ball_certificate *output) {
  if (certificate == nullptr || output == nullptr ||
      output->struct_size != sizeof(*output) || output->version != 1 ||
      output->reserved != 0) {
    return CRX_INVALID_ARGUMENT;
  }
  return Guard([&]() -> std::int32_t {
    if (!certificate->attachment.attempt.IsCertified()) {
      return CRX_INVALID_ARGUMENT;
    }
    *output = CertificateRecord(*certificate);
    return CRX_OK;
  });
}

std::int32_t crx_v1_ik_certified(const crx_v1_handle *handle,
                                 const double *pose, const double *reference,
                                 const double *tool, const double *seed_deg,
                                 std::int32_t capacity, double *solutions,
                                 crx_v1_ik_result *result,
                                 crx_v1_certificate_handle **certificates,
                                 std::int32_t *failures,
                                 const crx_v1_certificate_options *options,
                                 const crx_v1_certificate_handle *reuse) {
  if (result == nullptr) {
    return CRX_INVALID_ARGUMENT;
  }
  *result = {CRX_INVALID_INPUT, 0, 0, 0};
  if (!ValidCertificateOptions(options)) {
    return CRX_INVALID_ARGUMENT;
  }
  return Guard([&]() -> std::int32_t {
    if (handle == nullptr || pose == nullptr || reference == nullptr ||
        tool == nullptr || solutions == nullptr || certificates == nullptr ||
        failures == nullptr || capacity <= 0) {
      return CRX_INVALID_ARGUMENT;
    }
    crx::Vec6 seed;
    if (seed_deg != nullptr && !ReadCommand(seed_deg, seed)) {
      return CRX_INVALID_ARGUMENT;
    }
    crx::PoseIsoRT source, ref, tcp;
    source.matrix() = Eigen::Map<const PoseRow>(pose);
    ref.matrix() = Eigen::Map<const PoseRow>(reference);
    tcp.matrix() = Eigen::Map<const PoseRow>(tool);
    const crx::CertificateOptions requested{
        (options->flags & CRX_CERTIFICATION_DISABLED) == 0,
        reuse == nullptr ? nullptr : &reuse->attachment.attempt.certificate};
    const auto solved = crx::SolveIkCertified(
        handle->model, source, ref, tcp, seed_deg == nullptr ? nullptr : &seed,
        capacity, requested);
    const auto count = solved.result.solutions.size();
    if (count > static_cast<std::size_t>(capacity) ||
        count != solved.certificates.size()) {
      return CRX_INTERNAL_ERROR;
    }
    auto pending = StageCertificates(solved.certificates);
    Eigen::Map<SolutionsRow> output(solutions, capacity, crx::kDofCount);
    for (std::size_t i = 0; i < count; ++i) {
      output.row(static_cast<Eigen::Index>(i)) =
          solved.result.solutions.at(i).transpose() * angle_conv::kDegPerRad;
    }
    CommitCertificates(pending, solved.certificates, certificates, failures);
    *result = {
        Status(solved.result.status), static_cast<std::int32_t>(count),
        static_cast<std::int32_t>(solved.result.truncated),
        static_cast<std::int32_t>(solved.result.used_singular_representatives)};
    return CRX_OK;
  });
}

std::int32_t crx_v1_identify_certified(
    const crx_v1_handle *handle, const double *pose, const double *reference,
    const double *tool, std::int32_t capacity, double *solutions,
    std::int32_t *aspect_signs, std::int32_t *regular,
    std::int32_t *wrist_addresses, std::int32_t *address_valid,
    crx_v1_fibre_result *result, crx_v1_fibre_metadata *metadata,
    crx_v1_certificate_handle **certificates, std::int32_t *failures,
    const crx_v1_certificate_options *options) {
  if (result == nullptr) {
    return CRX_INVALID_ARGUMENT;
  }
  *result = {CRX_INVALID_INPUT, 0, 0, 0};
  if (!ValidCertificateOptions(options) || metadata == nullptr ||
      metadata->struct_size != sizeof(*metadata) || metadata->version != 1 ||
      metadata->reserved != 0) {
    return CRX_INVALID_ARGUMENT;
  }
  return Guard([&]() -> std::int32_t {
    if (handle == nullptr || pose == nullptr || reference == nullptr ||
        tool == nullptr || solutions == nullptr || aspect_signs == nullptr ||
        regular == nullptr || wrist_addresses == nullptr ||
        address_valid == nullptr || certificates == nullptr ||
        failures == nullptr || capacity <= 0) {
      return CRX_INVALID_ARGUMENT;
    }
    crx::PoseIsoRT source, ref, tcp;
    source.matrix() = Eigen::Map<const PoseRow>(pose);
    ref.matrix() = Eigen::Map<const PoseRow>(reference);
    tcp.matrix() = Eigen::Map<const PoseRow>(tool);
    const auto identified = crx::IdentifyIkCertified(
        handle->model, source, ref, tcp, capacity,
        {(options->flags & CRX_CERTIFICATION_DISABLED) == 0, nullptr});
    if (identified.result.solutions.size() >
            static_cast<std::size_t>(capacity) ||
        identified.result.solutions.size() != identified.certificates.size()) {
      return CRX_INTERNAL_ERROR;
    }
    auto pending = StageCertificates(identified.certificates);
    if (!CopyIdentifiedSolutions(identified.result, capacity, solutions,
                                 aspect_signs, regular, wrist_addresses,
                                 address_valid, result)) {
      return CRX_INTERNAL_ERROR;
    }
    CommitCertificates(pending, identified.certificates, certificates,
                       failures);
    metadata->coverage = static_cast<std::int32_t>(identified.result.coverage);
    return CRX_OK;
  });
}
} // extern "C"
