// SPDX-License-Identifier: Apache-2.0
// SPDX-FileCopyrightText: 2026 Centre Technologique en Aérospatiale

#define NOMINMAX
#include <windows.h>

#include "crx_calibrated_api.h"
#include "crx_calibrated_solver.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <random>
#include <string>
#include <type_traits>
#include <vector>

namespace cal = crx::calibrated;

namespace {
enum class Operation {
  PrepareModel,
  PrepareSolver,
  PrepareTarget,
  Kinematics,
  CorrectDisabled,
  CorrectEnabled,
  ApiPrepare,
  ApiDisabled,
  ApiEnabled
};
constexpr std::array<const char *, 9> kNames{
    "prepare_model", "prepare_solver",   "prepare_target",
    "kinematics",    "correct_disabled", "correct_enabled",
    "api_prepare",   "api_disabled",     "api_enabled"};

[[noreturn]] void Fail(const char *message) {
  std::cerr << message << '\n';
  std::exit(2);
}

template <typename T> auto Record() -> T {
  T output{};
  output.struct_size = sizeof(T);
  output.version = 1;
  return output;
}

struct Api {
  HMODULE module = nullptr;
  decltype(&crx_cal_v1_abi_version) version = nullptr;
  decltype(&crx_cal_v1_create) create = nullptr;
  decltype(&crx_cal_v1_destroy) destroy = nullptr;
  decltype(&crx_cal_v1_query_create) query = nullptr;
  decltype(&crx_cal_v1_query_destroy) release = nullptr;
  decltype(&crx_cal_v1_summary_read) summary = nullptr;
  decltype(&crx_cal_v1_identity_read) identity = nullptr;
  decltype(&crx_cal_v1_result_read) row = nullptr;
};

template <typename T> auto Function(HMODULE module, const char *name) -> T {
  const auto address = GetProcAddress(module, name);
  if (address == nullptr)
    Fail("Required calibrated benchmark symbol missing");
  return reinterpret_cast<T>(address);
}

auto Load(const std::filesystem::path &path) -> Api {
  Api result;
  const auto full = std::filesystem::canonical(path);
  result.module = LoadLibraryW(full.c_str());
  if (result.module == nullptr)
    Fail("Could not load calibrated runtime");
  std::array<wchar_t, 32768> loaded{};
  if (GetModuleFileNameW(result.module, loaded.data(),
                         static_cast<DWORD>(loaded.size())) == 0 ||
      !std::filesystem::equivalent(full, std::filesystem::path(loaded.data())))
    Fail("Loaded module path differs from requested calibrated runtime");
  result.version = Function<decltype(result.version)>(result.module,
                                                      "crx_cal_v1_abi_version");
  result.create =
      Function<decltype(result.create)>(result.module, "crx_cal_v1_create");
  result.destroy =
      Function<decltype(result.destroy)>(result.module, "crx_cal_v1_destroy");
  result.query = Function<decltype(result.query)>(result.module,
                                                  "crx_cal_v1_query_create");
  result.release = Function<decltype(result.release)>(
      result.module, "crx_cal_v1_query_destroy");
  result.summary = Function<decltype(result.summary)>(
      result.module, "crx_cal_v1_summary_read");
  result.identity = Function<decltype(result.identity)>(
      result.module, "crx_cal_v1_identity_read");
  result.row =
      Function<decltype(result.row)>(result.module, "crx_cal_v1_result_read");
  if (result.version() != 1)
    Fail("Unsupported calibrated runtime ABI");
  return result;
}

struct Case {
  std::string name;
  std::uint32_t operations = 0;
  crx_cal_v1_model_input input{};
  crx_cal_v1_target target{};
  crx_cal_v1_options options{};
  std::vector<cal::Centre> seeds;
  cal::Model model, scratch_model;
  cal::Solver solver, scratch_solver;
  cal::Target problem, scratch_target;
  crx_cal_v1_handle *handle = nullptr;
  std::vector<cal::Correction> corrections;
  std::vector<cal::PointKinematics> points;
  std::vector<crx_cal_v1_result> records;
  crx_cal_v1_summary summary = Record<crx_cal_v1_summary>();
  crx_cal_v1_identity identity = Record<crx_cal_v1_identity>();
  std::int32_t code = 0;
};

template <typename T> auto Read(std::istream &stream, T &output) -> bool {
  static_assert(std::is_trivially_copyable_v<T>);
  stream.read(reinterpret_cast<char *>(&output), sizeof(T));
  return static_cast<bool>(stream);
}

auto Input(const crx_cal_v1_model_input &source) -> cal::Input {
  cal::Input output;
  for (std::size_t i = 0; i < 6; ++i) {
    output.rows.at(i) = {{source.alpha_quarters[i], source.dh[i][0]},
                         source.dh[i][1],
                         {source.theta_quarters[i], source.dh[i][2]},
                         source.dh[i][3]};
    output.senses.at(i) = source.senses[i];
    output.lower_commands.at(i) = source.lower_commands[i];
    output.upper_commands.at(i) = source.upper_commands[i];
  }
  std::copy_n(source.base_quaternion, 4, output.base.quaternion.begin());
  std::copy_n(source.base_position, 3, output.base.position.begin());
  std::copy_n(source.tool_quaternion, 4, output.tool.quaternion.begin());
  std::copy_n(source.tool_position, 3, output.tool.position.begin());
  output.length_scale = source.length_scale;
  output.limit_units = static_cast<cal::LimitUnits>(source.limit_units);
  return output;
}

auto Target(const crx_cal_v1_target &source) -> cal::Frame {
  cal::Frame output;
  std::copy_n(source.quaternion, 4, output.quaternion.begin());
  std::copy_n(source.position, 3, output.position.begin());
  return output;
}

auto Options(const Case &item, bool proofs) -> cal::CorrectionOptions {
  cal::CorrectionOptions output;
  output.max_iterations = item.options.max_iterations;
  output.max_backtracks = item.options.max_backtracks;
  output.maximum_step = item.options.maximum_step;
  output.normalized_position_tolerance =
      item.options.normalized_position_tolerance;
  output.orientation_tolerance = item.options.orientation_tolerance;
  output.uniqueness_radius = item.options.uniqueness_radius;
  output.certify = proofs;
  return output;
}

auto ReadCases(const std::filesystem::path &path, const Api &api)
    -> std::vector<Case> {
  std::ifstream stream(path, std::ios::binary);
  std::array<char, 8> magic{};
  std::uint32_t count = 0;
  if (!Read(stream, magic) || std::string(magic.data(), 7) != "CRXCAL1" ||
      !Read(stream, count) || count == 0 || count > 10000)
    Fail("Invalid calibrated benchmark packet");
  std::vector<Case> output(count);
  for (auto &item : output) {
    std::uint32_t length = 0, seeds = 0;
    if (!Read(stream, length) || length == 0 || length > 256)
      Fail("Invalid case name length");
    item.name.resize(length);
    stream.read(item.name.data(), length);
    if (!Read(stream, item.operations) || !Read(stream, item.input) ||
        !Read(stream, item.target) || !Read(stream, item.options) ||
        !Read(stream, seeds) || seeds > CRX_CAL_V1_MAX_SEEDS)
      Fail("Invalid calibrated case packet");
    item.seeds.resize(seeds);
    for (auto &seed : item.seeds)
      if (!Read(stream, seed))
        Fail("Incomplete calibrated seeds");
    if (cal::Prepare(Input(item.input), item.model) != cal::Status::Ready ||
        cal::PrepareSolver(item.model, item.solver) != cal::Status::Ready ||
        cal::PrepareTarget(item.solver, Target(item.target), item.problem) !=
            cal::Status::Ready ||
        !cal::ValidateCorrectionOptions(Options(item, true)))
      Fail("Calibrated case preparation failed");
    if (api.create(&item.input, &item.handle) != 0 || item.handle == nullptr)
      Fail("Calibrated API model preparation failed");
    item.corrections.resize(seeds);
    item.points.resize(seeds);
    item.records.assign(seeds, Record<crx_cal_v1_result>());
  }
  if (stream.peek() != std::char_traits<char>::eof())
    Fail("Extra calibrated packet bytes");
  return output;
}

void Run(Case &item, Operation operation, const Api &api) {
  item.code = 0;
  switch (operation) {
  case Operation::PrepareModel:
    item.code =
        static_cast<int>(cal::Prepare(Input(item.input), item.scratch_model));
    return;
  case Operation::PrepareSolver:
    item.code =
        static_cast<int>(cal::PrepareSolver(item.model, item.scratch_solver));
    return;
  case Operation::PrepareTarget:
    item.code = static_cast<int>(cal::PrepareTarget(
        item.solver, Target(item.target), item.scratch_target));
    return;
  case Operation::Kinematics:
    for (std::size_t i = 0; i < item.seeds.size(); ++i) {
      cal::Centre increments;
      for (std::size_t j = 0; j < increments.size(); ++j)
        increments.at(j) = item.input.senses[j] * item.seeds.at(i).at(j);
      item.code |= static_cast<int>(
          cal::EvaluateNumerical(item.solver, increments, item.points.at(i)));
    }
    return;
  case Operation::CorrectDisabled:
  case Operation::CorrectEnabled:
    for (std::size_t i = 0; i < item.seeds.size(); ++i)
      item.corrections.at(i) =
          cal::Correct(item.solver, item.problem, item.seeds.at(i),
                       Options(item, operation == Operation::CorrectEnabled));
    return;
  case Operation::ApiPrepare: {
    crx_cal_v1_handle *handle = nullptr;
    item.code = api.create(&item.input, &handle);
    api.destroy(handle);
    return;
  }
  case Operation::ApiDisabled:
  case Operation::ApiEnabled: {
    auto options = item.options;
    options.flags =
        operation == Operation::ApiDisabled ? CRX_CAL_DISABLE_CERTIFICATION : 0;
    crx_cal_v1_query *query = nullptr;
    const double *seeds =
        item.seeds.empty() ? nullptr : item.seeds.front().data();
    item.code = api.query(item.handle, &item.target, seeds,
                          static_cast<std::uint32_t>(item.seeds.size()),
                          &options, &query);
    if (item.code == 0) {
      item.code |= api.summary(query, &item.summary);
      item.code |= api.identity(query, &item.identity);
      for (std::size_t i = 0; i < item.records.size(); ++i)
        item.code |=
            api.row(query, static_cast<std::uint32_t>(i), &item.records.at(i));
    }
    api.release(query);
    return;
  }
  }
}

struct Outcome {
  std::uint64_t numeric = 1469598103934665603ULL;
  std::uint64_t proof = 1469598103934665603ULL;
  std::uint64_t iterations = 0, backtracks = 0, evaluations = 0;
  std::uint32_t converged = 0, local = 0, admitted = 0;

  bool operator==(const Outcome &other) const {
    return numeric == other.numeric && proof == other.proof &&
           iterations == other.iterations && backtracks == other.backtracks &&
           evaluations == other.evaluations && converged == other.converged &&
           local == other.local && admitted == other.admitted;
  }
};

void Mix(std::uint64_t &hash, std::uint64_t word) {
  hash ^= word;
  hash *= 1099511628211ULL;
}

void Mix(std::uint64_t &hash, double value) {
  std::uint64_t word = 0;
  std::memcpy(&word, &value, sizeof(word));
  Mix(hash, word);
}

auto Flatten(const cal::Correction &source) -> crx_cal_v1_result {
  auto output = Record<crx_cal_v1_result>();
  output.status = static_cast<std::int32_t>(source.status);
  output.certificate_failure =
      static_cast<std::int32_t>(source.local_proof.failure);
  output.iterations = source.iterations;
  output.backtracks = source.backtracks;
  output.evaluations = source.evaluations;
  output.normalized_position_error = source.normalized_position_error;
  output.orientation_error = source.orientation_error;
  std::copy(source.commands.begin(), source.commands.end(),
            output.commands_rad);
  std::copy(source.increments.begin(), source.increments.end(),
            output.increments_rad);
  if (source.local_proof.IsCertified()) {
    const auto &data = source.local_proof.certificate.Values();
    const auto &bounds = data.bounds;
    output.flags = CRX_CAL_LOCAL_PROOF |
                   (data.commands_admitted ? CRX_CAL_COMMANDS_ADMITTED : 0);
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

void Accumulate(Outcome &outcome, const crx_cal_v1_result &row) {
  Mix(outcome.numeric, static_cast<std::uint64_t>(row.status));
  for (double value : row.commands_rad)
    Mix(outcome.numeric, value);
  for (double value : row.increments_rad)
    Mix(outcome.numeric, value);
  Mix(outcome.numeric, row.normalized_position_error);
  Mix(outcome.numeric, row.orientation_error);
  Mix(outcome.numeric, static_cast<std::uint64_t>(row.iterations));
  Mix(outcome.numeric, static_cast<std::uint64_t>(row.backtracks));
  Mix(outcome.numeric, static_cast<std::uint64_t>(row.evaluations));
  outcome.iterations += row.iterations;
  outcome.backtracks += row.backtracks;
  outcome.evaluations += row.evaluations;
  outcome.converged += row.status == CRX_CAL_CONVERGED ? 1 : 0;
  const bool local = (row.flags & CRX_CAL_LOCAL_PROOF) != 0;
  outcome.local += local ? 1 : 0;
  outcome.admitted += (row.flags & CRX_CAL_COMMANDS_ADMITTED) != 0 ? 1 : 0;
  Mix(outcome.proof, static_cast<std::uint64_t>(row.certificate_failure));
  if (local) {
    for (double value :
         {row.existence_radius, row.uniqueness_radius, row.beta, row.lipschitz,
          row.eta, row.inverse_residual, row.coarse_eta, row.pose_defect,
          row.screw_norm_sum, row.angular_norm_sum, row.anchor_position_norm,
          row.command_reserve_lower})
      Mix(outcome.proof, value);
    for (double value : row.preconditioner)
      Mix(outcome.proof, value);
    Mix(outcome.proof, static_cast<std::uint64_t>(row.local_jacobian_sign));
  }
}

auto Capture(const Case &item, Operation operation) -> Outcome {
  if (item.code != 0)
    Fail("Calibrated benchmark operation failed");
  Outcome result;
  if (operation == Operation::CorrectDisabled ||
      operation == Operation::CorrectEnabled) {
    for (const auto &row : item.corrections)
      Accumulate(result, Flatten(row));
  } else if (operation == Operation::ApiDisabled ||
             operation == Operation::ApiEnabled) {
    if (item.summary.coverage != CRX_COVERAGE_INCONCLUSIVE ||
        item.summary.seed_count != item.seeds.size() ||
        item.identity.model_schema != item.model.Data().identity.schema ||
        !std::equal(item.model.Data().identity.words.begin(),
                    item.model.Data().identity.words.end(),
                    item.identity.model_words))
      Fail("Calibrated API snapshot identity or coverage mismatch");
    for (const auto &row : item.records)
      Accumulate(result, row);
    if (result.converged != item.summary.numerical_count ||
        result.local != item.summary.local_count ||
        result.admitted != item.summary.admitted_count ||
        result.evaluations != item.summary.evaluations)
      Fail("Calibrated API summary mismatch");
  } else if (operation == Operation::Kinematics) {
    for (const auto &point : item.points) {
      for (Eigen::Index r = 0; r < 3; ++r) {
        Mix(result.numeric, point.pose.translation(r));
        for (Eigen::Index c = 0; c < 3; ++c)
          Mix(result.numeric, point.pose.rotation(r, c));
      }
      for (Eigen::Index r = 0; r < 6; ++r)
        for (Eigen::Index c = 0; c < 6; ++c)
          Mix(result.numeric, point.body_jacobian(r, c));
    }
  } else if (operation == Operation::PrepareModel) {
    for (auto word : item.scratch_model.Data().identity.words)
      Mix(result.numeric, word);
  } else if (operation == Operation::PrepareSolver) {
    for (auto word : item.scratch_solver.Data().geometry.Data().identity.words)
      Mix(result.numeric, word);
  } else if (operation == Operation::PrepareTarget) {
    for (auto word : item.scratch_target.Exact().Identity().target)
      Mix(result.numeric, word);
  }
  return result;
}

void PrintNumber(std::ostream &stream, double value) {
  if (std::isfinite(value))
    stream << value;
  else
    stream << "nan";
}

void WriteRow(std::ostream &stream, const Case &item, Operation operation,
              std::size_t seed, const crx_cal_v1_result &row) {
  stream << item.name << ',' << kNames.at(static_cast<std::size_t>(operation))
         << ',' << seed << ',' << row.status << ',' << row.iterations << ','
         << row.backtracks << ',' << row.evaluations << ','
         << ((row.flags & CRX_CAL_LOCAL_PROOF) != 0) << ','
         << ((row.flags & CRX_CAL_COMMANDS_ADMITTED) != 0) << ','
         << row.certificate_failure;
  for (double value : row.commands_rad) {
    stream << ',';
    PrintNumber(stream, value);
  }
  for (double value : row.increments_rad) {
    stream << ',';
    PrintNumber(stream, value);
  }
  for (double value :
       {row.normalized_position_error, row.orientation_error,
        row.existence_radius, row.uniqueness_radius, row.beta, row.lipschitz,
        row.eta, row.inverse_residual, row.screw_norm_sum, row.angular_norm_sum,
        row.anchor_position_norm, row.command_reserve_lower}) {
    stream << ',';
    PrintNumber(stream, value);
  }
  stream << ',' << row.local_jacobian_sign << ',' << row.command_jacobian_sign;
  for (double value : row.preconditioner) {
    stream << ',';
    PrintNumber(stream, value);
  }
  stream << '\n';
}

struct Configuration {
  std::size_t item;
  Operation operation;
  Outcome expected;
};

auto Configurations(std::vector<Case> &cases, const Api &api,
                    std::ostream &audit) -> std::vector<Configuration> {
  audit << std::setprecision(17);
  audit << "case,operation,seed,status,iterations,backtracks,evaluations,local,"
           "admitted,failure";
  for (const auto *prefix : {"command", "increment"})
    for (int j = 0; j < 6; ++j)
      audit << ',' << prefix << j;
  audit << ",position_error,orientation_error,rho,radius,beta,lipschitz,eta,"
           "inverse_residual,screw_sum,angular_sum,position_norm,command_"
           "reserve,local_sign,command_sign";
  for (int j = 0; j < 36; ++j)
    audit << ",inverse" << j;
  audit << '\n';
  std::vector<Configuration> output;
  for (std::size_t i = 0; i < cases.size(); ++i) {
    auto &item = cases.at(i);
    std::array<Outcome, 9> outcomes;
    for (std::size_t op = 0; op < kNames.size(); ++op) {
      const auto operation = static_cast<Operation>(op);
      Run(item, operation, api);
      outcomes.at(op) = Capture(item, operation);
      if ((item.operations & (1U << op)) != 0)
        output.push_back({i, operation, outcomes.at(op)});
      if (operation == Operation::ApiEnabled ||
          operation == Operation::ApiDisabled) {
        for (std::size_t j = 0; j < item.records.size(); ++j)
          WriteRow(audit, item, operation, j, item.records.at(j));
      } else if (operation == Operation::CorrectDisabled ||
                 operation == Operation::CorrectEnabled) {
        for (std::size_t j = 0; j < item.corrections.size(); ++j)
          WriteRow(audit, item, operation, j, Flatten(item.corrections.at(j)));
      }
    }
    if (outcomes.at(4).numeric != outcomes.at(5).numeric ||
        outcomes.at(7).numeric != outcomes.at(8).numeric)
      Fail("Enabling local proofs changed calibrated numerical outputs/work");
  }
  return output;
}

volatile std::uint64_t sink = 0;

auto Timed(Case &item, Operation operation, const Api &api,
           std::uint64_t repeats) -> double {
  const auto before = std::chrono::steady_clock::now();
  for (std::uint64_t i = 0; i < repeats; ++i)
    Run(item, operation, api);
  const auto after = std::chrono::steady_clock::now();
  return std::chrono::duration<double, std::nano>(after - before).count();
}

void Measure(std::vector<Case> &cases, const Api &api,
             const std::vector<Configuration> &configs, std::uint32_t rounds,
             double minimum_ms, std::uint32_t seed, std::ostream &output) {
  std::vector<std::uint64_t> repeats(configs.size(), 1);
  for (std::size_t i = 0; i < configs.size(); ++i) {
    const auto &config = configs.at(i);
    auto &item = cases.at(config.item);
    while (Timed(item, config.operation, api, repeats.at(i)) <
               minimum_ms * 1e6 &&
           repeats.at(i) < (1U << 20U))
      repeats.at(i) *= 2;
    if (!(Capture(item, config.operation) == config.expected))
      Fail("Warmup changed calibrated outcome");
  }
  std::vector<std::size_t> order(configs.size());
  std::iota(order.begin(), order.end(), 0);
  std::mt19937 rng(seed);
  output << "round,case,operation,repeats,elapsed_ns,ns_per_call,numeric_hash,"
            "proof_hash,converged,local,admitted,iterations,backtracks,"
            "evaluations\n";
  output << std::setprecision(17);
  for (std::uint32_t round = 0; round < rounds; ++round) {
    std::shuffle(order.begin(), order.end(), rng);
    for (const auto index : order) {
      const auto &config = configs.at(index);
      auto &item = cases.at(config.item);
      const double elapsed =
          Timed(item, config.operation, api, repeats.at(index));
      const auto observed = Capture(item, config.operation);
      if (!(observed == config.expected))
        Fail("Timed calibrated outcome/work changed");
      sink = observed.numeric ^ observed.proof;
      output << round << ',' << item.name << ','
             << kNames.at(static_cast<std::size_t>(config.operation)) << ','
             << repeats.at(index) << ',' << elapsed << ','
             << elapsed / static_cast<double>(repeats.at(index)) << ','
             << observed.numeric << ',' << observed.proof << ','
             << observed.converged << ',' << observed.local << ','
             << observed.admitted << ',' << observed.iterations << ','
             << observed.backtracks << ',' << observed.evaluations << '\n';
    }
    output.flush();
  }
}
} // namespace

auto main(int argc, char **argv) -> int {
  if (argc != 5 && argc != 8)
    Fail("Usage: benchmark_calibrated verify|measure packet DLL output-prefix "
         "[rounds min-ms random-seed]");
  const bool measure = std::string(argv[1]) == "measure";
  if ((!measure && std::string(argv[1]) != "verify") || (measure && argc != 8))
    Fail("Invalid calibrated benchmark mode");
  DWORD_PTR process_mask = 0, system_mask = 0;
  if (!GetProcessAffinityMask(GetCurrentProcess(), &process_mask,
                              &system_mask) ||
      process_mask == 0)
    Fail("Could not read process CPU affinity");
  const DWORD_PTR bit = process_mask & (~process_mask + 1);
  if (SetThreadAffinityMask(GetCurrentThread(), bit) == 0)
    Fail("Could not pin benchmark thread");
  const auto api = Load(std::filesystem::path(argv[3]));
  auto cases = ReadCases(std::filesystem::path(argv[2]), api);
  const std::string prefix(argv[4]);
  std::ofstream audit(prefix + "-audit.csv");
  if (!audit)
    Fail("Could not create calibrated audit output");
  const auto configs = Configurations(cases, api, audit);
  audit.close();
  std::cout << "Verified " << cases.size() << " cases and " << configs.size()
            << " selected configurations; process affinity " << process_mask
            << ", timing thread " << bit << '\n';
  if (measure) {
    const auto rounds = static_cast<std::uint32_t>(std::stoul(argv[5]));
    const double minimum_ms = std::stod(argv[6]);
    const auto seed = static_cast<std::uint32_t>(std::stoul(argv[7]));
    if (rounds == 0 || rounds > 100 || !std::isfinite(minimum_ms) ||
        minimum_ms < 1 || minimum_ms > 1000)
      Fail("Invalid calibrated timing configuration");
    std::ofstream timing(prefix + "-timing.csv");
    if (!timing)
      Fail("Could not create calibrated timing output");
    Measure(cases, api, configs, rounds, minimum_ms, seed, timing);
  }
  for (auto &item : cases)
    api.destroy(item.handle);
  FreeLibrary(api.module);
  return 0;
}
