// SPDX-License-Identifier: Apache-2.0
// SPDX-FileCopyrightText: 2026 Centre Technologique en Aérospatiale

#include "crx_catalogue_api.h"
#include "crx_api_handles.h"
#include "crx_numerics.h"
#include "crx_verified_solver.h"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <memory>
#include <utility>

struct crx_v1_catalogue_handle {
  crx::VerifiedQuery query;
};

namespace {
static_assert(sizeof(crx_v1_catalogue_options) == 56);
static_assert(sizeof(crx_v1_catalogue_summary) == 1008);
static_assert(sizeof(crx_v1_catalogue_command) == 200);
static_assert(sizeof(crx_v1_catalogue_range) == 224);
static_assert(sizeof(crx_v1_catalogue_chart) == 448);
static_assert(sizeof(crx_v1_catalogue_leaf) == 456);
static_assert(static_cast<int>(crx::command::Ranking::NoAdmissible) ==
              CRX_RANK_NO_ADMISSIBLE);
static_assert(static_cast<int>(crx::command::Failure::ArithmeticUnavailable) ==
              CRX_COMMAND_ARITHMETIC_UNAVAILABLE);
static_assert(static_cast<int>(crx::catalogue::Failure::PairWorkLimit) ==
              CRX_PHASE_PAIR_WORK_LIMIT);
static_assert(static_cast<int>(crx::root::Reason::ArithmeticUnavailable) ==
              CRX_ROOT_ARITHMETIC_FAILURE);

template <typename Function>
auto Guard(const Function &function) noexcept -> std::int32_t {
  try {
    return function();
  } catch (...) {
    return CRX_INTERNAL_ERROR;
  }
}

template <typename Record> auto Negotiated(const Record *record) -> bool {
  return record != nullptr && record->struct_size == sizeof(Record) &&
         record->version == 1 && record->reserved == 0;
}

template <typename Record> auto MakeRecord() -> Record {
  Record result{};
  result.struct_size = sizeof(result);
  result.version = 1;
  return result;
}

auto ValidOptions(const crx_v1_catalogue_options *options) -> bool {
  constexpr std::uint64_t budget = std::uint64_t{1} << 20;
  return Negotiated(options) && options->flags <= CRX_CATALOGUE_DISABLED &&
         options->profile <= CRX_COMMAND_DECOUPLED_USER &&
         options->capacity <= 512 && options->max_nodes <= budget &&
         options->max_depth <= 50 && options->max_leaves >= 2 &&
         options->max_leaves <= budget && options->max_chart_attempts >= 1 &&
         options->max_chart_attempts <= 8 &&
         options->max_candidate_certificates <= budget &&
         options->max_pair_tests <= budget && options->max_lifts <= budget;
}

void Copy(crx::interval::Interval value, double (&output)[2]) {
  output[0] = value.lo;
  output[1] = value.hi;
}

template <std::size_t N>
void Copy(const std::array<crx::interval::Interval, N> &values,
          double (&output)[N][2]) {
  for (std::size_t i = 0; i < N; ++i) {
    Copy(values.at(i), output[i]);
  }
}

auto HasRootStage(const crx_v1_catalogue_handle *handle) -> bool {
  return handle != nullptr && handle->query.attempted &&
         handle->query.problem.IsReady() &&
         handle->query.context_failure == crx::command::Failure::None;
}

auto Summary(const crx::VerifiedQuery &query) -> crx_v1_catalogue_summary {
  auto out = MakeRecord<crx_v1_catalogue_summary>();
  const auto &phases = query.phases.catalogue;
  const auto &roots = phases.Values().root_cover.Values();
  const auto &commands = query.commands.Values();
  out.coverage = query.attempted ? static_cast<int>(phases.Values().coverage)
                                 : CRX_COVERAGE_NOT_ATTEMPTED;
  out.preparation = query.attempted ? static_cast<int>(query.preparation) + 1
                                    : CRX_QUERY_NOT_PREPARED;
  out.context_failure = static_cast<int>(query.context_failure);
  out.phase_failure = static_cast<int>(phases.Values().failure);
  out.command_failure = static_cast<int>(commands.failure);
  out.ranking = static_cast<int>(commands.ranking);
  out.phase_count = static_cast<std::uint32_t>(phases.Values().balls.size());
  out.root_status = static_cast<std::uint32_t>(roots.status);
  out.root_count = roots.simple_roots;
  out.chart_shift = roots.chart_shift;
  out.chart_attempts = roots.attempts;
  out.root_nodes = roots.nodes;
  out.entire_circle_unresolved = roots.entire_circle_unresolved;
  for (std::size_t i = 0; i < 2; ++i) {
    out.root_leaf_count[i] =
        static_cast<std::uint32_t>(roots.partitions.at(i).leaves.size());
  }
  out.nominal_infeasible = phases.IsInfeasible();
  out.enumeration_complete = commands.enumeration_complete;
  out.admission_complete = commands.admission_complete;
  out.command_infeasible = query.commands.IsCommandInfeasible();
  out.returned_count = static_cast<std::uint32_t>(commands.returned_count);
  out.truncated = commands.truncated;
  out.has_seed = query.raw.has_seed;
  out.profile = static_cast<std::uint32_t>(query.raw.profile);
  out.numerical_status = static_cast<int>(query.numerical.status);
  out.numerical_count =
      static_cast<std::uint32_t>(query.numerical.solutions.size());
  out.numerical_truncated = query.numerical.truncated;
  out.used_singular_representatives =
      query.numerical.used_singular_representatives;
  out.box_lifts = commands.box_lifts;
  out.yes = commands.yes;
  out.no = commands.no;
  out.undecided = commands.undecided;
  out.command_count = commands.lifts.size();
  if (query.problem.IsReady()) {
    const auto &prepared = query.problem.Data();
    out.identity_schema = prepared.identity.schema;
    std::copy(prepared.identity.words.begin(), prepared.identity.words.end(),
              out.input_words);
    out.length_scale = prepared.length_scale;
    for (std::size_t i = 0; i < 3; ++i) {
      out.rotation_deviation_upper[i] = prepared.rotation_deviation.at(i).hi;
    }
  }
  const std::array<std::array<double, 6>, 4> raw{
      query.raw.lower_degrees, query.raw.upper_degrees, query.raw.seed_degrees,
      query.raw.weights};
  for (std::size_t field = 0; field < raw.size(); ++field) {
    for (std::size_t axis = 0; axis < 6; ++axis) {
      const double value = raw.at(field).at(axis);
      std::memcpy(&out.raw_words[field * 6 + axis], &value, sizeof(value));
    }
  }
  return out;
}
} // namespace

std::int32_t crx_v1_catalogue_query(
    const crx_v1_handle *model, const double *pose, const double *reference,
    const double *tool, const double *limits_deg, const double *seed_deg,
    const double *weights, const crx_v1_catalogue_options *options,
    crx_v1_catalogue_handle **output) {
  if (output == nullptr) {
    return CRX_INVALID_ARGUMENT;
  }
  *output = nullptr;
  if (!ValidOptions(options) || model == nullptr || pose == nullptr ||
      reference == nullptr || tool == nullptr ||
      (options->profile == CRX_COMMAND_DECOUPLED_USER &&
       limits_deg == nullptr)) {
    return CRX_INVALID_ARGUMENT;
  }
  return Guard([&]() -> std::int32_t {
    crx::command::Input raw;
    raw.profile = static_cast<crx::command::Profile>(options->profile);
    raw.has_seed = seed_deg != nullptr;
    for (std::size_t i = 0; i < raw.lower_degrees.size(); ++i) {
      raw.lower_degrees.at(i) =
          limits_deg == nullptr ? model->lower_degrees.at(i) : limits_deg[i];
      raw.upper_degrees.at(i) = limits_deg == nullptr
                                    ? model->upper_degrees.at(i)
                                    : limits_deg[i + 6];
      if (seed_deg != nullptr)
        raw.seed_degrees.at(i) = seed_deg[i];
      if (weights != nullptr)
        raw.weights.at(i) = weights[i];
    }
    using PoseRow = Eigen::Matrix<double, 4, 4, Eigen::RowMajor>;
    crx::PoseIsoRT source, ref, tcp;
    source.matrix() = Eigen::Map<const PoseRow>(pose);
    ref.matrix() = Eigen::Map<const PoseRow>(reference);
    tcp.matrix() = Eigen::Map<const PoseRow>(tool);
    crx::VerifiedOptions requested;
    requested.enabled = options->flags == 0;
    requested.phases.roots = {options->max_nodes, options->max_depth,
                              options->max_leaves, options->max_chart_attempts};
    requested.phases.max_candidate_certificates =
        options->max_candidate_certificates;
    requested.phases.max_pair_tests = options->max_pair_tests;
    requested.commands = {options->max_lifts, options->capacity};
    auto owned = std::make_unique<crx_v1_catalogue_handle>();
    owned->query =
        crx::QueryVerified(model->model, source, ref, tcp, raw, requested);
    if (!owned->query.input_valid)
      return CRX_INVALID_ARGUMENT;
    *output = owned.release();
    return CRX_OK;
  });
}

void crx_v1_catalogue_destroy(crx_v1_catalogue_handle *query) { delete query; }

std::int32_t crx_v1_catalogue_read(const crx_v1_catalogue_handle *handle,
                                   crx_v1_catalogue_summary *output) {
  if (handle == nullptr || !Negotiated(output))
    return CRX_INVALID_ARGUMENT;
  return Guard([&]() -> std::int32_t {
    *output = Summary(handle->query);
    return CRX_OK;
  });
}

std::int32_t crx_v1_catalogue_phase(const crx_v1_catalogue_handle *handle,
                                    std::uint32_t index,
                                    crx_v1_ball_certificate *output) {
  if (handle == nullptr || !Negotiated(output) ||
      index >= handle->query.phases.catalogue.Values().balls.size())
    return CRX_INVALID_ARGUMENT;
  return Guard([&]() -> std::int32_t {
    crx_v1_certificate_handle local;
    local.attachment.attempt.failure = crx::ball::Failure::None;
    local.attachment.attempt.certificate =
        handle->query.phases.catalogue.Values().balls.at(index);
    local.attachment.length_scale = handle->query.problem.Data().length_scale;
    for (std::size_t i = 0; i < 3; ++i)
      local.attachment.rotation_deviation_upper.at(i) =
          handle->query.problem.Data().rotation_deviation.at(i).hi;
    return crx_v1_certificate_read(&local, output);
  });
}

std::int32_t
crx_v1_catalogue_command_read(const crx_v1_catalogue_handle *handle,
                              std::uint64_t index,
                              crx_v1_catalogue_command *output) {
  if (handle == nullptr || !Negotiated(output) || output->reserved2 != 0 ||
      index >= handle->query.commands.Values().lifts.size())
    return CRX_INVALID_ARGUMENT;
  return Guard([&]() -> std::int32_t {
    const auto &data = handle->query.commands.Values();
    const auto &lift = data.lifts.at(static_cast<std::size_t>(index));
    auto out = MakeRecord<crx_v1_catalogue_command>();
    out.admission = static_cast<int>(lift.admission);
    out.phase = static_cast<std::uint32_t>(lift.phase);
    out.possible_minimizer = std::binary_search(
        data.possible_minimizers.begin(), data.possible_minimizers.end(),
        static_cast<std::size_t>(index));
    out.cost_available = lift.travel_squared.IsValid();
    std::copy(lift.turns.begin(), lift.turns.end(), out.turns);
    Copy(lift.radians, out.radians);
    Copy(lift.travel_squared, out.travel_squared);
    out.proposed_travel_squared = lift.proposed_travel_squared;
    *output = out;
    return CRX_OK;
  });
}

std::int32_t crx_v1_catalogue_range_read(const crx_v1_catalogue_handle *handle,
                                         std::uint32_t phase,
                                         crx_v1_catalogue_range *output) {
  if (handle == nullptr || !Negotiated(output) ||
      phase >= handle->query.commands.Values().ranges.size())
    return CRX_INVALID_ARGUMENT;
  return Guard([&]() -> std::int32_t {
    const auto &range = handle->query.commands.Values().ranges.at(phase);
    auto out = MakeRecord<crx_v1_catalogue_range>();
    out.phase = phase;
    out.available = range.available;
    out.empty = range.empty;
    out.count = range.count;
    std::copy(range.first.begin(), range.first.end(), out.first);
    std::copy(range.last.begin(), range.last.end(), out.last);
    Copy(range.base, out.base);
    *output = out;
    return CRX_OK;
  });
}

std::int32_t crx_v1_catalogue_chart_read(const crx_v1_catalogue_handle *handle,
                                         std::uint32_t chart,
                                         crx_v1_catalogue_chart *output) {
  if (!HasRootStage(handle) || !Negotiated(output) || chart >= 2)
    return CRX_INVALID_ARGUMENT;
  return Guard([&]() -> std::int32_t {
    const auto &data =
        handle->query.phases.catalogue.Values().root_cover.Values();
    const auto &polynomial = data.polynomials.at(chart);
    const auto &partition = data.partitions.at(chart);
    auto out = MakeRecord<crx_v1_catalogue_chart>();
    out.chart = chart;
    out.status = static_cast<int>(partition.status);
    out.leaf_count = static_cast<std::uint32_t>(partition.leaves.size());
    out.nodes = partition.nodes;
    out.roots = partition.roots;
    out.exclusions = partition.exclusions;
    out.unresolved = partition.unresolved;
    out.max_depth = partition.max_depth;
    Copy(polynomial.chart, out.descriptor);
    Copy(polynomial.residual, out.residual);
    Copy(polynomial.derivative, out.derivative);
    Copy(polynomial.determinant, out.determinant);
    Copy(polynomial.omitted_residual, out.omitted_residual);
    Copy(polynomial.omitted_determinant, out.omitted_determinant);
    *output = out;
    return CRX_OK;
  });
}

std::int32_t crx_v1_catalogue_leaf_read(const crx_v1_catalogue_handle *handle,
                                        std::uint32_t chart,
                                        std::uint32_t index,
                                        crx_v1_catalogue_leaf *output) {
  if (!HasRootStage(handle) || !Negotiated(output) || chart >= 2)
    return CRX_INVALID_ARGUMENT;
  const auto &leaves = handle->query.phases.catalogue.Values()
                           .root_cover.Values()
                           .partitions.at(chart)
                           .leaves;
  if (index >= leaves.size())
    return CRX_INVALID_ARGUMENT;
  return Guard([&]() -> std::int32_t {
    const auto &leaf = leaves.at(index);
    auto out = MakeRecord<crx_v1_catalogue_leaf>();
    out.chart = chart;
    out.index = index;
    out.kind = static_cast<int>(leaf.kind);
    out.reason = static_cast<int>(leaf.reason);
    out.depth = leaf.depth;
    out.derivative_sign = leaf.derivative_sign;
    out.determinant_sign = leaf.determinant_sign;
    Copy(leaf.tangent, out.tangent);
    Copy(leaf.phase, out.phase);
    Copy(leaf.endpoint_values, out.endpoints);
    Copy(leaf.residual_bernstein, out.residual_bernstein);
    Copy(leaf.derivative_bernstein, out.derivative_bernstein);
    Copy(leaf.determinant_bernstein, out.determinant_bernstein);
    *output = out;
    return CRX_OK;
  });
}

std::int32_t crx_v1_catalogue_numerical(const crx_v1_catalogue_handle *handle,
                                        std::uint32_t index,
                                        double *command_deg) {
  if (handle == nullptr || command_deg == nullptr ||
      index >= handle->query.numerical.solutions.size())
    return CRX_INVALID_ARGUMENT;
  return Guard([&]() -> std::int32_t {
    const auto &value = handle->query.numerical.solutions.at(index);
    for (std::size_t i = 0; i < 6; ++i)
      command_deg[i] =
          value(static_cast<Eigen::Index>(i)) * angle_conv::kDegPerRad;
    return CRX_OK;
  });
}
