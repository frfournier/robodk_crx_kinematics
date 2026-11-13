// SPDX-License-Identifier: Apache-2.0
// SPDX-FileCopyrightText: 2026 Centre Technologique en Aérospatiale

#define NOMINMAX
#include <windows.h>

#include "crx_ball_certificate.h"
#include "crx_exact_problem.h"
#include "crx_numerics.h"
#include "crx_python_api.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <random>
#include <string>
#include <vector>

namespace {
enum class Operation {
  Prepare,
  Kinematics,
  Certify,
  Reuse,
  ApiDisabled,
  ApiEnabled,
  ApiReuse,
  ApiParent
};
constexpr std::array<const char *, 8> kNames{
    "prepare",      "kinematics",  "certify",   "reuse",
    "api_disabled", "api_enabled", "api_reuse", "api_parent"};

struct Api {
  HMODULE module = nullptr;
  decltype(&crx_v1_create) create = nullptr;
  decltype(&crx_v1_destroy) destroy = nullptr;
  decltype(&crx_v1_ik) numerical = nullptr;
  decltype(&crx_v1_ik_certified) ik = nullptr;
  decltype(&crx_v1_certificate_read) read = nullptr;
  decltype(&crx_v1_certificate_destroy) release = nullptr;
};

struct Case {
  int model_id = -1;
  crx::CrxModelData model;
  crx::PoseIsoRT previous_target, target, reference, tool;
  crx::ball::Centre previous_centre{}, centre{};
  std::array<double, 6> seed{};
  std::array<double, 12> limits{};
  std::array<double, 16> previous_row{}, target_row{}, reference_row{},
      tool_row{};
  crx::exact::Problem previous_problem, problem;
  crx::ball::Attempt previous;
  crx_v1_handle *handle = nullptr;
  crx_v1_handle *parent_handle = nullptr;
  crx_v1_certificate_handle *cache = nullptr;
};

struct Outcome {
  double consumed = 0;
  int failure = 0;
  int status = -1;
  int count = -1;
  bool certified = false;
  bool reused = false;
  int reuse_failure = -1;
  int truncated = 0;
  int singular_representatives = 0;
};

volatile double checksum = 0;

[[noreturn]] void Fail(const char *message) {
  std::cerr << message << '\n';
  std::exit(2);
}

template <std::size_t N>
auto Read(std::istream &input, std::array<double, N> &values) -> bool {
  for (double &value : values)
    input >> value;
  return static_cast<bool>(input);
}

auto ReadPose(std::istream &input, crx::PoseIsoRT &pose,
              std::array<double, 16> &row) -> bool {
  if (!Read(input, row))
    return false;
  for (int i = 0; i < 4; ++i)
    for (int j = 0; j < 4; ++j)
      pose.matrix()(i, j) = row.at(4 * i + j);
  return true;
}

auto ReadCase(std::istream &input, Case &item) -> bool {
  input >> item.model_id;
  std::array<double, 4> lengths{};
  double height = 0;
  std::array<double, 6> zero{}, senses{};
  if (!Read(input, lengths) || !(input >> height) || !Read(input, zero) ||
      !Read(input, senses) || !Read(input, item.limits))
    return false;
  const std::array<double, 6> alpha{
      0, -crx::kHalfPi, 0, -crx::kHalfPi, crx::kHalfPi, -crx::kHalfPi};
  item.model.base_transform.translation().z() = -height;
  for (std::size_t i = 0; i < 6; ++i) {
    auto &dh = item.model.dh_rows.at(i);
    dh.alpha_rad = alpha.at(i);
    dh.theta0_rad = zero.at(i);
    item.model.joint_senses.at(i) = senses.at(i);
    item.model.lower_limits_rad(static_cast<int>(i)) =
        item.limits.at(i) * angle_conv::kRadPerDeg;
    item.model.upper_limits_rad(static_cast<int>(i)) =
        item.limits.at(i + 6) * angle_conv::kRadPerDeg;
  }
  item.model.dh_rows.at(0).d = height;
  item.model.dh_rows.at(2).a = lengths.at(0);
  item.model.dh_rows.at(3).d = lengths.at(1);
  item.model.dh_rows.at(4).d = -lengths.at(3);
  item.model.dh_rows.at(5).d = lengths.at(2);
  return ReadPose(input, item.previous_target, item.previous_row) &&
         ReadPose(input, item.target, item.target_row) &&
         ReadPose(input, item.reference, item.reference_row) &&
         ReadPose(input, item.tool, item.tool_row) &&
         Read(input, item.previous_centre) && Read(input, item.centre) &&
         Read(input, item.seed);
}

auto Prepare(const Case &item, bool previous, crx::exact::Problem &problem)
    -> crx::exact::Status {
  const auto &target = previous ? item.previous_target : item.target;
  return crx::exact::PrepareReference(item.model, target,
                                      Eigen::Quaterniond(target.linear()),
                                      item.reference, item.tool, problem);
}

auto Consume(const crx::ball::Data &data) -> double {
  double value =
      data.beta + data.lipschitz + data.eta + data.coarse_eta +
      data.inverse_residual + data.existence_radius + data.uniqueness_radius +
      data.pose_defect + data.screw_norm_sum + data.angular_norm_sum +
      data.anchor_position_norm + data.aspect_sign +
      data.target_relative_trace.lo + data.target_relative_trace.hi +
      data.closed_form_determinant.lo + data.closed_form_determinant.hi +
      static_cast<double>(data.used_log_series) + data.identity.schema;
  std::uint64_t bits = 0;
  for (const auto word : data.identity.words)
    bits = (bits << 1) ^ (bits >> 63) ^ word;
  for (const double angle : data.centre)
    value += angle;
  for (int i = 0; i < 6; ++i)
    for (int j = 0; j < 6; ++j)
      value += data.preconditioner(i, j);
  return value + static_cast<double>(bits & 0xffffU);
}

auto FromAttempt(const crx::ball::Attempt &attempt, bool reuse = false)
    -> Outcome {
  return {attempt.IsCertified() ? Consume(attempt.certificate.Values()) : 0,
          static_cast<int>(attempt.failure),
          -1,
          -1,
          attempt.IsCertified(),
          reuse && attempt.IsCertified(),
          reuse ? static_cast<int>(attempt.failure) : -1};
}

template <typename Derived>
auto ConsumeMatrix(const Eigen::MatrixBase<Derived> &matrix) -> double {
  double result = 0;
  for (Eigen::Index row = 0; row < matrix.rows(); ++row)
    for (Eigen::Index col = 0; col < matrix.cols(); ++col)
      result += matrix(row, col).lo + matrix(row, col).hi;
  return result;
}

auto ConsumeRecord(const crx_v1_ball_certificate &data) -> double {
  double result = data.existence_radius_rad + data.uniqueness_radius_rad +
                  data.beta + data.lipschitz + data.eta + data.coarse_eta +
                  data.inverse_residual + data.pose_defect +
                  data.screw_norm_sum + data.angular_norm_sum +
                  data.anchor_position_norm + data.length_scale +
                  data.aspect_sign + data.identity_schema + data.reused +
                  data.used_log_series + data.reuse_failure;
  for (const auto value : data.canonical_center_rad)
    result += value;
  for (const auto value : data.preconditioner)
    result += value;
  for (const auto value : data.rotation_deviation_upper)
    result += value;
  for (const auto value : data.target_relative_trace)
    result += value;
  for (const auto value : data.closed_form_determinant)
    result += value;
  std::uint64_t bits = 0;
  for (const auto word : data.input_words)
    bits = (bits << 1) ^ (bits >> 63) ^ word;
  return result + static_cast<double>(bits & 0xffffU);
}

auto CallApi(const Api &api, const Case &item, bool disabled, bool reuse,
             bool previous, crx_v1_ball_certificate *record = nullptr,
             std::array<double, 6> *commands = nullptr) -> Outcome {
  std::array<double, 6> solution{};
  crx_v1_ik_result metadata{};
  crx_v1_certificate_handle *proof = nullptr;
  int failure = -1;
  const crx_v1_certificate_options options{sizeof(options), 1,
                                           disabled ? 1U : 0U, 0};
  const int error = api.ik(
      item.handle, (previous ? item.previous_row : item.target_row).data(),
      item.reference_row.data(), item.tool_row.data(), item.seed.data(), 1,
      solution.data(), &metadata, &proof, &failure, &options,
      reuse ? item.cache : nullptr);
  if (error != 0)
    Fail("certified API infrastructure error");
  double consumed = 0;
  for (const auto value : solution)
    consumed += value;
  Outcome result{consumed,         metadata.count ? failure : -1,
                 metadata.status,  metadata.count,
                 proof != nullptr, false};
  result.truncated = metadata.truncated;
  result.singular_representatives = metadata.used_singular_representatives;
  result.consumed += metadata.status + metadata.count + metadata.truncated +
                     metadata.used_singular_representatives + failure;
  if (proof != nullptr) {
    crx_v1_ball_certificate data{};
    data.struct_size = sizeof(data);
    data.version = 1;
    const int read = api.read(proof, &data);
    api.release(proof);
    if (read != 0)
      Fail("certificate snapshot read failed");
    result.reused = data.reused != 0;
    result.reuse_failure = reuse ? data.reuse_failure : -1;
    result.consumed += ConsumeRecord(data);
    if (record != nullptr)
      *record = data;
  }
  if (commands != nullptr)
    *commands = solution;
  return result;
}

auto CallParent(const Api &parent, const Case &item,
                std::array<double, 6> *commands = nullptr) -> Outcome {
  // The old C ABI accepts flange poses. Include ordinary raw-frame composition
  // here so its workload is comparable with the new raw-frame entry point.
  const crx::PoseIsoRT flange =
      item.reference * item.target * item.tool.inverse();
  std::array<double, 16> row{};
  for (int i = 0; i < 4; ++i)
    for (int j = 0; j < 4; ++j)
      row.at(4 * i + j) = flange.matrix()(i, j);
  std::array<double, 6> solution{};
  crx_v1_ik_result metadata{};
  if (parent.numerical(item.parent_handle, row.data(), item.seed.data(), 1,
                       solution.data(), &metadata) != 0)
    Fail("parent numerical API infrastructure error");
  double consumed = metadata.status + metadata.count + metadata.truncated +
                    metadata.used_singular_representatives;
  for (const auto value : solution)
    consumed += value;
  if (commands != nullptr)
    *commands = solution;
  Outcome result{consumed, -1, metadata.status, metadata.count, false, false};
  result.truncated = metadata.truncated;
  result.singular_representatives = metadata.used_singular_representatives;
  return result;
}

__declspec(noinline) auto Run(const Case &item, Operation operation,
                              const Api &api, const Api &parent) -> Outcome {
  switch (operation) {
  case Operation::Prepare: {
    crx::exact::Problem problem;
    const auto status = Prepare(item, false, problem);
    double consumed = 0;
    if (problem.IsReady()) {
      const auto &data = problem.Data();
      consumed = data.length_scale + ConsumeMatrix(data.target.rotation) +
                 ConsumeMatrix(data.target.translation) +
                 ConsumeMatrix(data.p) + ConsumeMatrix(data.v) +
                 ConsumeMatrix(data.w) + data.identity.schema;
      for (const auto value :
           {data.lengths.a, data.lengths.b, data.lengths.c, data.lengths.r})
        consumed += value.lo + value.hi;
      for (const auto value : data.rotation_deviation)
        consumed += value.lo + value.hi;
      for (const auto value : data.offset_unwrapped)
        consumed += value.lo + value.hi;
      for (const auto value : data.diagonal)
        consumed += value;
      for (const auto &value : data.encoder_phase) {
        consumed +=
            value.principal.lo + value.principal.hi +
            value.trigonometry.sine.lo + value.trigonometry.sine.hi +
            value.trigonometry.cosine.lo + value.trigonometry.cosine.hi +
            value.principal_seam_uncertain + value.trigonometry.used_full_range;
      }
      std::uint64_t bits = 0;
      for (const auto word : data.identity.words)
        bits = (bits << 1) ^ (bits >> 63) ^ word;
      consumed += static_cast<double>(bits & 0xffffU);
    }
    return {consumed, static_cast<int>(status)};
  }
  case Operation::Kinematics: {
    crx::ball::Kinematics output;
    const auto status =
        crx::ball::EvaluateCentre(item.problem, item.centre, output);
    double consumed = 0;
    if (status == crx::ball::Failure::None) {
      consumed = ConsumeMatrix(output.body_jacobian) +
                 ConsumeMatrix(output.anchor.rotation) +
                 ConsumeMatrix(output.anchor.translation) +
                 output.closed_form_determinant.lo +
                 output.closed_form_determinant.hi;
      for (const auto &value : output.axes)
        consumed += ConsumeMatrix(value);
      for (const auto &value : output.origins)
        consumed += ConsumeMatrix(value);
    }
    return {consumed, static_cast<int>(status)};
  }
  case Operation::Certify:
    return FromAttempt(crx::ball::Certify(item.problem, item.centre));
  case Operation::Reuse:
    return FromAttempt(
        crx::ball::TryReuse(item.previous.certificate, item.problem), true);
  case Operation::ApiDisabled:
    return CallApi(api, item, true, false, false);
  case Operation::ApiEnabled:
    return CallApi(api, item, false, false, false);
  case Operation::ApiReuse:
    return CallApi(api, item, false, true, false);
  case Operation::ApiParent:
    return CallParent(parent, item);
  }
  Fail("unknown operation");
}

void WriteRecord(std::ostream &output, std::size_t index, const char *operation,
                 const Outcome &outcome, const crx_v1_ball_certificate *data) {
  output << index << ',' << operation << ',' << outcome.failure << ','
         << outcome.status << ',' << outcome.count << ',' << outcome.certified
         << ',' << outcome.reused << ',' << outcome.reuse_failure << ','
         << (data != nullptr ? data->identity_schema : 0);
  if (data != nullptr) {
    const std::array<double, 14> constants{
        data->existence_radius_rad,
        data->uniqueness_radius_rad,
        data->inverse_residual,
        data->beta,
        data->lipschitz,
        data->eta,
        data->coarse_eta,
        data->pose_defect,
        data->screw_norm_sum,
        data->angular_norm_sum,
        data->anchor_position_norm,
        data->length_scale,
        static_cast<double>(data->aspect_sign),
        static_cast<double>(data->used_log_series)};
    for (const auto value : constants)
      output << ',' << value;
    for (const auto value : data->canonical_center_rad)
      output << ',' << value;
    for (const auto value : data->preconditioner)
      output << ',' << value;
    for (const auto value : data->input_words)
      output << ',' << std::hex << value << std::dec;
    for (const auto value : data->target_relative_trace)
      output << ',' << value;
    for (const auto value : data->closed_form_determinant)
      output << ',' << value;
    for (const auto value : data->rotation_deviation_upper)
      output << ',' << value;
  } else {
    for (int i = 0; i < 140; ++i)
      output << ',';
  }
  output << '\n';
}

auto Record(const crx::ball::Attempt &attempt, double scale)
    -> crx_v1_ball_certificate {
  crx_v1_ball_certificate result{};
  const auto &data = attempt.certificate.Values();
  result.identity_schema = data.identity.schema;
  std::copy(data.centre.begin(), data.centre.end(),
            result.canonical_center_rad);
  std::copy(data.identity.words.begin(), data.identity.words.end(),
            result.input_words);
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
  result.length_scale = scale;
  result.aspect_sign = data.aspect_sign;
  result.used_log_series = data.used_log_series ? 1 : 0;
  result.target_relative_trace[0] = data.target_relative_trace.lo;
  result.target_relative_trace[1] = data.target_relative_trace.hi;
  result.closed_form_determinant[0] = data.closed_form_determinant.lo;
  result.closed_form_determinant[1] = data.closed_form_determinant.hi;
  for (int i = 0; i < 6; ++i)
    for (int j = 0; j < 6; ++j)
      result.preconditioner[i * 6 + j] = data.preconditioner(i, j);
  return result;
}

template <typename Derived>
void Bounds(std::ostream &stream, std::size_t index, const std::string &field,
            const Eigen::MatrixBase<Derived> &matrix) {
  int component = 0;
  for (Eigen::Index row = 0; row < matrix.rows(); ++row)
    for (Eigen::Index col = 0; col < matrix.cols(); ++col)
      stream << index << ',' << field << ',' << component++ << ','
             << matrix(row, col).lo << ',' << matrix(row, col).hi << '\n';
}
} // namespace

auto main(int argc, char **argv) -> int {
  if (argc != 9) {
    std::cerr
        << "dll parent-dll input timings certificates domains repeats rounds\n";
    return 2;
  }
  const int repeats = std::stoi(argv[7]), rounds = std::stoi(argv[8]);
  if (repeats < 1 || rounds < 1)
    return 2;
  Api api;
  api.module = LoadLibraryA(argv[1]);
  if (api.module == nullptr)
    return 2;
  api.create = reinterpret_cast<decltype(api.create)>(
      GetProcAddress(api.module, "crx_v1_create"));
  api.destroy = reinterpret_cast<decltype(api.destroy)>(
      GetProcAddress(api.module, "crx_v1_destroy"));
  api.ik = reinterpret_cast<decltype(api.ik)>(
      GetProcAddress(api.module, "crx_v1_ik_certified"));
  api.read = reinterpret_cast<decltype(api.read)>(
      GetProcAddress(api.module, "crx_v1_certificate_read"));
  api.release = reinterpret_cast<decltype(api.release)>(
      GetProcAddress(api.module, "crx_v1_certificate_destroy"));
  if (!api.create || !api.destroy || !api.ik || !api.read || !api.release)
    return 2;
  Api parent;
  parent.module = LoadLibraryA(argv[2]);
  if (!parent.module)
    return 2;
  parent.create = reinterpret_cast<decltype(parent.create)>(
      GetProcAddress(parent.module, "crx_v1_create"));
  parent.destroy = reinterpret_cast<decltype(parent.destroy)>(
      GetProcAddress(parent.module, "crx_v1_destroy"));
  parent.numerical = reinterpret_cast<decltype(parent.numerical)>(
      GetProcAddress(parent.module, "crx_v1_ik"));
  if (!parent.create || !parent.destroy || !parent.numerical)
    return 2;
  std::ifstream input(argv[3]);
  std::size_t count = 0;
  input >> count;
  if (!input || count == 0)
    return 2;
  std::vector<Case> cases(count);
  for (auto &item : cases) {
    if (!ReadCase(input, item) ||
        Prepare(item, true, item.previous_problem) !=
            crx::exact::Status::Ready ||
        Prepare(item, false, item.problem) != crx::exact::Status::Ready)
      return 2;
    item.previous =
        crx::ball::Certify(item.previous_problem, item.previous_centre);
    if (item.model_id >= 0) {
      if (api.create(item.model_id, item.limits.data(), &item.handle) != 0)
        return 2;
      if (parent.create(item.model_id, item.limits.data(),
                        &item.parent_handle) != 0)
        return 2;
      const crx_v1_certificate_options options{sizeof(options), 1, 0, 0};
      crx_v1_ik_result metadata{};
      std::array<double, 6> solution{};
      int failure = 0;
      if (api.ik(item.handle, item.previous_row.data(),
                 item.reference_row.data(), item.tool_row.data(),
                 item.seed.data(), 1, solution.data(), &metadata, &item.cache,
                 &failure, &options, nullptr) != 0)
        return 2;
    }
  }
  DWORD_PTR process_mask = 0, system_mask = 0;
  if (!GetProcessAffinityMask(GetCurrentProcess(), &process_mask,
                              &system_mask) ||
      !process_mask)
    return 2;
  const DWORD_PTR cpu = process_mask & (~process_mask + 1);
  if (!SetThreadAffinityMask(GetCurrentThread(), cpu))
    return 2;
  std::cerr << "Thread affinity mask " << cpu << '\n';
  std::ofstream timing(argv[4]), proofs(argv[5]), domains(argv[6]);
  const auto directory = std::filesystem::path(argv[6]).parent_path();
  std::ofstream bounds(directory / "enclosures.csv"),
      sources(directory / "inputs.csv"), commands(directory / "commands.csv");
  bounds << std::setprecision(17) << "case,field,component,lo,hi\n";
  sources << "case,schema";
  for (int i = 0; i < 77; ++i)
    sources << ",w" << i;
  sources << '\n';
  commands << std::setprecision(17)
           << "case,operation,status,count,truncated,singular_representatives,"
              "q0,q1,q2,q3,q4,q5\n";
  timing << std::setprecision(17)
         << "case,operation,round,us,failure,status,count,certified,reused,"
            "reuse_failure\n";
  proofs << std::setprecision(17)
         << "case,operation,failure,status,count,certified,reused,reuse_"
            "failure,schema";
  for (const auto name :
       {"rho", "radius", "inverse_residual", "beta", "lipschitz", "eta",
        "coarse_eta", "defect", "screw_sum", "angular_sum", "position_norm",
        "scale", "sign", "series"})
    proofs << ',' << name;
  for (int i = 0; i < 6; ++i)
    proofs << ",q" << i;
  for (int i = 0; i < 36; ++i)
    proofs << ",x" << i;
  for (int i = 0; i < 77; ++i)
    proofs << ",w" << i;
  for (const auto name : {"trace_lo", "trace_hi", "det_lo", "det_hi",
                          "dev_target", "dev_reference", "dev_tool"})
    proofs << ',' << name;
  proofs << '\n';
  domains << std::setprecision(17)
          << "case,previous_valid,api_previous_valid,rotation_width,"
             "translation_width,jacobian_width\n";
  for (std::size_t i = 0; i < count; ++i) {
    const auto &item = cases.at(i);
    crx::ball::Kinematics kinematics;
    if (crx::ball::EvaluateCentre(item.problem, item.centre, kinematics) !=
        crx::ball::Failure::None)
      return 2;
    sources << i << ',' << item.problem.Data().identity.schema;
    for (const auto word : item.problem.Data().identity.words)
      sources << ',' << std::hex << word << std::dec;
    sources << '\n';
    Bounds(bounds, i, "target_rotation", item.problem.Data().target.rotation);
    Bounds(bounds, i, "target_translation",
           item.problem.Data().target.translation);
    Bounds(bounds, i, "rotation", kinematics.anchor.rotation);
    Bounds(bounds, i, "translation", kinematics.anchor.translation);
    Bounds(bounds, i, "jacobian", kinematics.body_jacobian);
    for (std::size_t joint = 0; joint < 6; ++joint) {
      Bounds(bounds, i, "axis" + std::to_string(joint),
             kinematics.axes.at(joint));
      Bounds(bounds, i, "origin" + std::to_string(joint),
             kinematics.origins.at(joint));
    }
    double rw = 0, pw = 0, jw = 0;
    for (int row = 0; row < 3; ++row) {
      pw = std::max(pw, kinematics.anchor.translation(row).hi -
                            kinematics.anchor.translation(row).lo);
      for (int col = 0; col < 3; ++col)
        rw = std::max(rw, kinematics.anchor.rotation(row, col).hi -
                              kinematics.anchor.rotation(row, col).lo);
    }
    for (int row = 0; row < 6; ++row)
      for (int col = 0; col < 6; ++col)
        jw = std::max(jw, kinematics.body_jacobian(row, col).hi -
                              kinematics.body_jacobian(row, col).lo);
    domains << i << ',' << item.previous.IsCertified() << ','
            << (item.cache != nullptr) << ',' << rw << ',' << pw << ',' << jw
            << '\n';
    for (const auto op : {Operation::Certify, Operation::Reuse}) {
      const auto attempt =
          op == Operation::Certify
              ? crx::ball::Certify(item.problem, item.centre)
              : crx::ball::TryReuse(item.previous.certificate, item.problem);
      auto data = Record(attempt, item.problem.Data().length_scale);
      for (std::size_t j = 0; j < 3; ++j)
        data.rotation_deviation_upper[j] =
            item.problem.Data().rotation_deviation.at(j).hi;
      WriteRecord(proofs, i, kNames.at(static_cast<int>(op)),
                  FromAttempt(attempt, op == Operation::Reuse),
                  attempt.IsCertified() ? &data : nullptr);
    }
    if (item.handle != nullptr) {
      std::array<double, 6> baseline{}, candidate{};
      const auto before =
          CallApi(api, item, true, false, false, nullptr, &baseline);
      for (const auto op : {Operation::ApiDisabled, Operation::ApiEnabled,
                            Operation::ApiReuse, Operation::ApiParent}) {
        crx_v1_ball_certificate data{};
        const auto result =
            op == Operation::ApiParent
                ? CallParent(parent, item, &candidate)
                : CallApi(api, item, op == Operation::ApiDisabled,
                          op == Operation::ApiReuse, false, &data, &candidate);
        if (result.status != before.status || result.count != before.count ||
            result.truncated != before.truncated ||
            result.singular_representatives !=
                before.singular_representatives ||
            candidate != baseline)
          return 3;
        if (result.certified &&
            (data.identity_schema != item.problem.Data().identity.schema ||
             !std::equal(item.problem.Data().identity.words.begin(),
                         item.problem.Data().identity.words.end(),
                         data.input_words))) {
          std::cerr << "API/component identity mismatch on case " << i << '\n';
          return 3;
        }
        commands << i << ',' << kNames.at(static_cast<int>(op)) << ','
                 << result.status << ',' << result.count << ','
                 << result.truncated << ',' << result.singular_representatives;
        for (const auto value : candidate)
          commands << ',' << value;
        commands << '\n';
        WriteRecord(proofs, i, kNames.at(static_cast<int>(op)), result,
                    result.certified ? &data : nullptr);
      }
    }
  }
  std::vector<std::size_t> order(count);
  std::iota(order.begin(), order.end(), 0);
  proofs.flush();
  domains.flush();
  bounds.flush();
  sources.flush();
  commands.flush();
  std::mt19937 random(20261113);
  for (int round = -1; round < rounds; ++round) {
    std::shuffle(order.begin(), order.end(), random);
    for (const auto index : order) {
      std::array<int, 8> operations{0, 1, 2, 3, 4, 5, 6, 7};
      std::shuffle(operations.begin(), operations.end(), random);
      for (const auto op : operations) {
        const auto &item = cases.at(index);
        if (op >= 4 && item.handle == nullptr)
          continue;
        Outcome (*volatile function)(const Case &, Operation, const Api &,
                                     const Api &) = &Run;
        const auto operation = static_cast<Operation>(op);
        const auto expected = function(item, operation, api, parent);
        Outcome result;
        const int iterations = round < 0 ? 3 : repeats;
        const auto start = std::chrono::steady_clock::now();
        for (int iteration = 0; iteration < iterations; ++iteration) {
          result = function(item, operation, api, parent);
          checksum += result.consumed;
        }
        const auto stop = std::chrono::steady_clock::now();
        if (result.failure != expected.failure ||
            result.status != expected.status ||
            result.count != expected.count ||
            result.truncated != expected.truncated ||
            result.singular_representatives !=
                expected.singular_representatives ||
            result.certified != expected.certified ||
            result.reused != expected.reused ||
            result.reuse_failure != expected.reuse_failure ||
            result.consumed != expected.consumed ||
            !std::isfinite(result.consumed))
          return 3;
        if (round >= 0)
          timing << index << ',' << kNames.at(op) << ',' << round << ','
                 << std::chrono::duration<double, std::micro>(stop - start)
                            .count() /
                        iterations
                 << ',' << result.failure << ',' << result.status << ','
                 << result.count << ',' << result.certified << ','
                 << result.reused << ',' << result.reuse_failure << '\n';
      }
    }
    std::cerr << "Round " << round + 1 << '/' << rounds << " complete\n";
  }
  for (auto &item : cases) {
    if (item.cache)
      api.release(item.cache);
    if (item.handle)
      api.destroy(item.handle);
    if (item.parent_handle)
      parent.destroy(item.parent_handle);
  }
  FreeLibrary(parent.module);
  FreeLibrary(api.module);
  std::cerr << "Checksum " << checksum << '\n';
  return timing && proofs && domains && bounds && sources && commands ? 0 : 2;
}
