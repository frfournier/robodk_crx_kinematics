// SPDX-License-Identifier: Apache-2.0
// SPDX-FileCopyrightText: 2026 Centre Technologique en Aérospatiale

#include "crx_calibrated_api.h"
#include "crx_calibrated_certificate.h"
#include "crx_calibrated_model.h"
#include "crx_calibrated_solver.h"
#include "crx_interval.h"
#include "crx_python_api.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <iterator>
#include <limits>
#include <memory>

namespace cal = crx::calibrated;

struct crx_cal_v1_handle {
  cal::Solver solver;
};

namespace {
struct StoredAttempt {
  cal::Centre seed{};
  cal::Correction correction;
};
} // namespace

struct crx_cal_v1_query {
  cal::ProblemIdentity identity;
  cal::LimitUnits units = cal::LimitUnits::Radians;
  std::unique_ptr<StoredAttempt[]> rows;
  crx_cal_v1_summary summary{};
};

namespace {
static_assert(sizeof(crx_cal_v1_model_input) == 528);
static_assert(sizeof(crx_cal_v1_target) == 80);
static_assert(sizeof(crx_cal_v1_options) == 64);
static_assert(sizeof(crx_cal_v1_summary) == 88);
static_assert(sizeof(crx_cal_v1_identity) == 656);
static_assert(sizeof(crx_cal_v1_result) == 608);
static_assert(CRX_CAL_V1_MODEL_WORDS == cal::kModelIdentityWords);

template <typename Function>
auto Guard(const Function &function) noexcept -> std::int32_t {
  try {
    return function();
  } catch (...) {
    return CRX_INTERNAL_ERROR;
  }
}

template <typename T> auto Layout(const T *value) -> bool {
  return value != nullptr && value->struct_size == sizeof(T) &&
         value->version == 1 &&
         std::all_of(std::begin(value->reserved), std::end(value->reserved),
                     [](std::uint32_t word) { return word == 0; });
}

template <typename T> auto Record() -> T {
  T value{};
  value.struct_size = sizeof(T);
  value.version = 1;
  return value;
}

auto Error(cal::Status status) -> std::int32_t {
  switch (status) {
  case cal::Status::Ready:
    return CRX_OK;
  case cal::Status::InvalidInput:
    return CRX_INVALID_ARGUMENT;
  case cal::Status::UnsupportedEnvironment:
    return CRX_CAL_UNSUPPORTED_ENVIRONMENT;
  case cal::Status::ArithmeticFailure:
    return CRX_CAL_ARITHMETIC_UNAVAILABLE;
  }
  return CRX_INTERNAL_ERROR;
}

auto Input(const crx_cal_v1_model_input &source) -> cal::Input {
  cal::Input result;
  for (std::size_t i = 0; i < 6; ++i) {
    auto &row = result.rows.at(i);
    row.alpha = {source.alpha_quarters[i], source.dh[i][0]};
    row.a = source.dh[i][1];
    row.theta = {source.theta_quarters[i], source.dh[i][2]};
    row.d = source.dh[i][3];
    result.senses.at(i) = source.senses[i];
    result.lower_commands.at(i) = source.lower_commands[i];
    result.upper_commands.at(i) = source.upper_commands[i];
  }
  std::copy_n(source.base_quaternion, 4, result.base.quaternion.begin());
  std::copy_n(source.base_position, 3, result.base.position.begin());
  std::copy_n(source.tool_quaternion, 4, result.tool.quaternion.begin());
  std::copy_n(source.tool_position, 3, result.tool.position.begin());
  result.length_scale = source.length_scale;
  result.limit_units = static_cast<cal::LimitUnits>(source.limit_units);
  return result;
}

auto Frame(const crx_cal_v1_target &source) -> cal::Frame {
  cal::Frame result;
  std::copy_n(source.quaternion, 4, result.quaternion.begin());
  std::copy_n(source.position, 3, result.position.begin());
  return result;
}

auto Options(const crx_cal_v1_options *source, cal::CorrectionOptions &output)
    -> bool {
  cal::CorrectionOptions result;
  if (source != nullptr) {
    if (!Layout(source) ||
        (source->flags &
         ~static_cast<std::uint32_t>(CRX_CAL_DISABLE_CERTIFICATION)) != 0)
      return false;
    result.max_iterations = source->max_iterations;
    result.max_backtracks = source->max_backtracks;
    result.maximum_step = source->maximum_step;
    result.normalized_position_tolerance =
        source->normalized_position_tolerance;
    result.orientation_tolerance = source->orientation_tolerance;
    result.uniqueness_radius = source->uniqueness_radius;
    result.certify = (source->flags & CRX_CAL_DISABLE_CERTIFICATION) == 0;
  }
  if (!cal::ValidateCorrectionOptions(result))
    return false;
  output = result;
  return true;
}

auto Evaluate(const crx_cal_v1_handle *handle, const double *commands,
              cal::PointKinematics &output) -> std::int32_t {
  if (handle == nullptr || commands == nullptr)
    return CRX_INVALID_ARGUMENT;
  if (!crx::interval::EnvironmentSupported())
    return CRX_CAL_UNSUPPORTED_ENVIRONMENT;
  cal::Centre increments;
  const auto &input = handle->solver.Data().geometry.Data().source;
  for (std::size_t i = 0; i < increments.size(); ++i) {
    if (!std::isfinite(commands[i]))
      return CRX_INVALID_ARGUMENT;
    increments.at(i) = input.senses.at(i) * commands[i];
  }
  return Error(cal::EvaluateNumerical(handle->solver, increments, output));
}

auto Result(const StoredAttempt &record, std::uint32_t index)
    -> crx_cal_v1_result {
  auto output = Record<crx_cal_v1_result>();
  const auto &correction = record.correction;
  output.seed_index = index;
  output.status = static_cast<std::int32_t>(correction.status);
  output.certificate_failure =
      static_cast<std::int32_t>(correction.local_proof.failure);
  output.flags =
      (correction.certificate_requested ? CRX_CAL_PROOF_REQUESTED : 0) |
      (correction.certificate_attempted ? CRX_CAL_PROOF_ATTEMPTED : 0) |
      (correction.numerical_limits_satisfied ? CRX_CAL_NUMERICAL_LIMITS : 0);
  output.iterations = correction.iterations;
  output.backtracks = correction.backtracks;
  output.evaluations = correction.evaluations;
  std::copy(record.seed.begin(), record.seed.end(), output.seed_commands_rad);
  std::copy(correction.commands.begin(), correction.commands.end(),
            output.commands_rad);
  std::copy(correction.increments.begin(), correction.increments.end(),
            output.increments_rad);
  output.normalized_position_error = correction.normalized_position_error;
  output.orientation_error = correction.orientation_error;
  const double missing = std::numeric_limits<double>::quiet_NaN();
  output.existence_radius = output.uniqueness_radius = output.beta =
      output.lipschitz = missing;
  output.eta = output.inverse_residual = output.coarse_eta =
      output.pose_defect = missing;
  output.screw_norm_sum = output.angular_norm_sum =
      output.anchor_position_norm = missing;
  output.command_reserve_lower = missing;
  std::fill(std::begin(output.preconditioner), std::end(output.preconditioner),
            missing);
  if (correction.local_proof.IsCertified()) {
    const auto &data = correction.local_proof.certificate.Values();
    const auto &bounds = data.bounds;
    output.flags |= CRX_CAL_LOCAL_PROOF;
    if (data.commands_admitted)
      output.flags |= CRX_CAL_COMMANDS_ADMITTED;
    if (bounds.used_log_series)
      output.flags |= CRX_CAL_LOG_SERIES;
    output.local_jacobian_sign = bounds.aspect_sign;
    output.command_jacobian_sign = data.command_jacobian_sign;
    output.existence_radius = bounds.existence_radius;
    output.uniqueness_radius = bounds.uniqueness_radius;
    output.beta = bounds.beta;
    output.lipschitz = bounds.lipschitz;
    output.eta = bounds.eta;
    output.inverse_residual = bounds.inverse_residual;
    output.coarse_eta = bounds.coarse_eta;
    output.pose_defect = bounds.pose_defect;
    output.screw_norm_sum = bounds.screw_norm_sum;
    output.angular_norm_sum = bounds.angular_norm_sum;
    output.anchor_position_norm = bounds.anchor_position_norm;
    output.command_reserve_lower = data.command_reserve_lower;
    for (Eigen::Index r = 0; r < 6; ++r)
      for (Eigen::Index c = 0; c < 6; ++c)
        output.preconditioner[r * 6 + c] = bounds.preconditioner(r, c);
  }
  return output;
}
} // namespace

extern "C" {
std::uint32_t crx_cal_v1_abi_version(void) { return 1; }

std::int32_t crx_cal_v1_options_default(crx_cal_v1_options *output) {
  if (!Layout(output))
    return CRX_INVALID_ARGUMENT;
  auto result = Record<crx_cal_v1_options>();
  const cal::CorrectionOptions defaults;
  result.max_iterations = defaults.max_iterations;
  result.max_backtracks = defaults.max_backtracks;
  result.maximum_step = defaults.maximum_step;
  result.normalized_position_tolerance = defaults.normalized_position_tolerance;
  result.orientation_tolerance = defaults.orientation_tolerance;
  result.uniqueness_radius = defaults.uniqueness_radius;
  *output = result;
  return CRX_OK;
}

std::int32_t crx_cal_v1_create(const crx_cal_v1_model_input *input,
                               crx_cal_v1_handle **output) {
  if (!Layout(input) || output == nullptr)
    return CRX_INVALID_ARGUMENT;
  return Guard([&]() -> std::int32_t {
    cal::Model model;
    const auto status = cal::Prepare(Input(*input), model);
    if (status != cal::Status::Ready)
      return Error(status);
    auto result = std::make_unique<crx_cal_v1_handle>();
    const auto numerical = cal::PrepareSolver(model, result->solver);
    if (numerical != cal::Status::Ready)
      return Error(numerical);
    *output = result.release();
    return CRX_OK;
  });
}

void crx_cal_v1_destroy(crx_cal_v1_handle *handle) { delete handle; }

std::int32_t crx_cal_v1_fk(const crx_cal_v1_handle *handle,
                           const double *commands_rad, double *pose16) {
  if (pose16 == nullptr)
    return CRX_INVALID_ARGUMENT;
  return Guard([&]() -> std::int32_t {
    cal::PointKinematics point;
    const auto code = Evaluate(handle, commands_rad, point);
    if (code != CRX_OK)
      return code;
    const double scale =
        handle->solver.Data().geometry.Data().source.length_scale;
    std::array<double, 16> output{};
    for (Eigen::Index r = 0; r < 3; ++r) {
      for (Eigen::Index c = 0; c < 3; ++c)
        output.at(static_cast<std::size_t>(r * 4 + c)) =
            point.pose.rotation(r, c);
      output.at(static_cast<std::size_t>(r * 4 + 3)) =
          point.pose.translation(r) * scale;
    }
    output.at(15) = 1;
    if (!std::all_of(output.begin(), output.end(),
                     [](double value) { return std::isfinite(value); }))
      return CRX_CAL_ARITHMETIC_UNAVAILABLE;
    std::copy(output.begin(), output.end(), pose16);
    return CRX_OK;
  });
}

std::int32_t crx_cal_v1_jacobian(const crx_cal_v1_handle *handle,
                                 const double *commands_rad,
                                 double *jacobian36) {
  if (jacobian36 == nullptr)
    return CRX_INVALID_ARGUMENT;
  return Guard([&]() -> std::int32_t {
    cal::PointKinematics point;
    const auto code = Evaluate(handle, commands_rad, point);
    if (code != CRX_OK)
      return code;
    const auto &input = handle->solver.Data().geometry.Data().source;
    std::array<double, 36> output{};
    for (Eigen::Index r = 0; r < 6; ++r)
      for (Eigen::Index c = 0; c < 6; ++c)
        output.at(static_cast<std::size_t>(r * 6 + c)) =
            point.body_jacobian(r, c) *
            input.senses.at(static_cast<std::size_t>(c)) *
            (r < 3 ? input.length_scale : 1);
    if (!std::all_of(output.begin(), output.end(),
                     [](double value) { return std::isfinite(value); }))
      return CRX_CAL_ARITHMETIC_UNAVAILABLE;
    std::copy(output.begin(), output.end(), jacobian36);
    return CRX_OK;
  });
}

std::int32_t crx_cal_v1_query_create(const crx_cal_v1_handle *handle,
                                     const crx_cal_v1_target *target,
                                     const double *seeds,
                                     std::uint32_t seed_count,
                                     const crx_cal_v1_options *options,
                                     crx_cal_v1_query **output) {
  cal::CorrectionOptions request;
  if (handle == nullptr || !Layout(target) || output == nullptr ||
      seed_count > CRX_CAL_V1_MAX_SEEDS ||
      (seed_count != 0 && seeds == nullptr))
    return CRX_INVALID_ARGUMENT;
  if (!crx::interval::EnvironmentSupported())
    return CRX_CAL_UNSUPPORTED_ENVIRONMENT;
  if (!Options(options, request))
    return CRX_INVALID_ARGUMENT;
  for (std::size_t i = 0; i < static_cast<std::size_t>(seed_count) * 6; ++i)
    if (!std::isfinite(seeds[i]))
      return CRX_INVALID_ARGUMENT;
  return Guard([&]() -> std::int32_t {
    cal::Target prepared;
    const auto status =
        cal::PrepareTarget(handle->solver, Frame(*target), prepared);
    if (status != cal::Status::Ready)
      return Error(status);
    auto result = std::make_unique<crx_cal_v1_query>();
    result->identity = prepared.Exact().Identity();
    result->units = handle->solver.Data().geometry.Data().source.limit_units;
    result->summary = Record<crx_cal_v1_summary>();
    auto &summary = result->summary;
    summary.coverage = CRX_COVERAGE_INCONCLUSIVE;
    summary.seed_count = seed_count;
    summary.length_scale =
        handle->solver.Data().geometry.Data().source.length_scale;
    summary.owned_payload_bytes =
        sizeof(crx_cal_v1_query) +
        static_cast<std::uint64_t>(seed_count) * sizeof(StoredAttempt);
    if (seed_count != 0)
      result->rows = std::make_unique<StoredAttempt[]>(seed_count);
    for (std::uint32_t i = 0; i < seed_count; ++i) {
      auto &row = result->rows[i];
      std::copy_n(seeds + static_cast<std::size_t>(i) * 6, 6, row.seed.begin());
      row.correction =
          cal::Correct(handle->solver, prepared, row.seed, request);
      const auto &correction = row.correction;
      if (correction.status == cal::CorrectionStatus::Converged)
        ++summary.numerical_count;
      if (correction.numerical_limits_satisfied)
        ++summary.numerical_inside_count;
      if (correction.local_proof.IsCertified()) {
        ++summary.local_count;
        if (correction.local_proof.certificate.Values().commands_admitted)
          ++summary.admitted_count;
      }
      summary.iterations += correction.iterations;
      summary.backtracks += correction.backtracks;
      summary.evaluations += correction.evaluations;
    }
    *output = result.release();
    return CRX_OK;
  });
}

void crx_cal_v1_query_destroy(crx_cal_v1_query *query) { delete query; }

std::int32_t crx_cal_v1_summary_read(const crx_cal_v1_query *query,
                                     crx_cal_v1_summary *output) {
  if (query == nullptr || !Layout(output))
    return CRX_INVALID_ARGUMENT;
  *output = query->summary;
  return CRX_OK;
}

std::int32_t crx_cal_v1_identity_read(const crx_cal_v1_query *query,
                                      crx_cal_v1_identity *output) {
  if (query == nullptr || !Layout(output))
    return CRX_INVALID_ARGUMENT;
  auto result = Record<crx_cal_v1_identity>();
  result.model_schema = query->identity.model.schema;
  result.model_word_count = CRX_CAL_V1_MODEL_WORDS;
  std::copy(query->identity.model.words.begin(),
            query->identity.model.words.end(), result.model_words);
  std::copy(query->identity.target.begin(), query->identity.target.end(),
            result.target_words);
  result.length_scale = query->summary.length_scale;
  result.limit_units = static_cast<std::uint32_t>(query->units);
  *output = result;
  return CRX_OK;
}

std::int32_t crx_cal_v1_result_read(const crx_cal_v1_query *query,
                                    std::uint32_t index,
                                    crx_cal_v1_result *output) {
  if (query == nullptr || !Layout(output) || output->reserved_counter != 0 ||
      index >= query->summary.seed_count)
    return CRX_INVALID_ARGUMENT;
  *output = Result(query->rows[index], index);
  return CRX_OK;
}
} // extern "C"
