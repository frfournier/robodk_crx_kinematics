// SPDX-License-Identifier: Apache-2.0
// SPDX-FileCopyrightText: 2026 Centre Technologique en Aérospatiale

#define NOMINMAX
#include <windows.h>

#include "catalogue_command_stages.h"
#include "crx_catalogue_api.h"
#include "crx_discovery.h"
#include "crx_exact_problem.h"
#include "crx_phase_catalogue.h"

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstddef>
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
#include <utility>
#include <vector>

#ifdef CRXKIN_DISCOVERY_COUNTERS
#error "The timed driver must not contain discovery instrumentation"
#endif

namespace {
enum class Operation {
  Prepare,
  Roots,
  Nominate,
  Balls,
  Assemble,
  Context,
  Ranges,
  Enumerate,
  Rank,
  Commands,
  ApiDisabled,
  ApiEnabled,
  Parent
};
constexpr std::array<const char *, 13> kNames{
    "exact_prepare",   "root_cover",       "numerical_nomination",
    "candidate_balls", "phase_assembly",   "command_context",
    "turn_ranges",     "turn_enumeration", "travel_ranking",
    "command_full",    "api_disabled",     "api_enabled",
    "parent_ik"};
constexpr std::size_t kAxes = 6;
volatile std::uint64_t checksum = 0;

[[noreturn]] void Fail(const char *message) {
  std::cerr << message << '\n';
  std::exit(2);
}

struct Api {
  HMODULE module = nullptr;
  decltype(&crx_v1_create) create = nullptr;
  decltype(&crx_v1_destroy) destroy = nullptr;
  decltype(&crx_v1_ik) ik = nullptr;
  decltype(&crx_v1_catalogue_query) query = nullptr;
  decltype(&crx_v1_catalogue_read) read = nullptr;
  decltype(&crx_v1_catalogue_destroy) release = nullptr;
};

template <typename Function>
auto Symbol(HMODULE module, const char *name) -> Function {
  const auto address = GetProcAddress(module, name);
  if (address == nullptr)
    Fail(name);
  return reinterpret_cast<Function>(address);
}

auto Load(const std::filesystem::path &path, bool candidate) -> Api {
  Api api;
  api.module = LoadLibraryW(path.c_str());
  if (api.module == nullptr)
    Fail("Cannot load benchmark DLL");
  api.create = Symbol<decltype(api.create)>(api.module, "crx_v1_create");
  api.destroy = Symbol<decltype(api.destroy)>(api.module, "crx_v1_destroy");
  api.ik = Symbol<decltype(api.ik)>(api.module, "crx_v1_ik");
  if (candidate) {
    api.query =
        Symbol<decltype(api.query)>(api.module, "crx_v1_catalogue_query");
    api.read = Symbol<decltype(api.read)>(api.module, "crx_v1_catalogue_read");
    api.release =
        Symbol<decltype(api.release)>(api.module, "crx_v1_catalogue_destroy");
  }
  return api;
}

struct Case {
  int id = 0, mode = 2, model_id = -1, capacity = 64;
  crx::exact::Input input;
  crx::command::Input raw;
  crx::catalogue::Options options;
  crx::command::Options commands;
  std::array<double, 12> original_limits{}, query_limits{};
  std::array<double, 16> target{}, reference{}, tool{}, flange{};
  crx_v1_handle *handle = nullptr, *parent = nullptr;
  crx::exact::Problem problem;
  crx::root::Certificate roots;
  crx::canonical::JointDiscovery nominations;
  std::vector<crx::ball::Certificate> balls;
  std::uint32_t attempted_balls = 0;
  crx::catalogue::PhaseCatalogue phases;
  crx::command::Context context;
  crx::command::Data ranges, enumerated, ranked;
  crx::command::Certificate complete_commands;
  bool enumerate_ready = false;
};

struct Scratch {
  crx::exact::Problem problem;
  crx::root::Certificate roots, assembly_roots;
  crx::canonical::JointDiscovery nominations;
  std::vector<crx::ball::Certificate> balls;
  crx::catalogue::PhaseCatalogue phases;
  crx::command::Context context;
  crx::command::Data commands;
  crx::command::Certificate certificate;
  crx_v1_catalogue_summary summary{};
  crx_v1_ik_result numerical{};
  std::array<double, 512 * kAxes> numerical_values{};
  int status = 0;
};

struct Outcome {
  int status = 0, coverage = -1, ranking = -1;
  std::uint64_t roots = 0, nodes = 0, leaves = 0, depth = 0, phases = 0;
  std::uint64_t nominations = 0, balls = 0, box_lifts = 0, yes = 0,
                undecided = 0;
  std::uint64_t commands = 0, digest = 0;
};

template <std::size_t N>
auto Read(std::istream &input, std::array<double, N> &values) -> bool {
  for (auto &value : values)
    input >> value;
  return static_cast<bool>(input);
}

auto ReadInput(std::istream &stream, crx::exact::Input &input) -> bool {
  if (!Read(stream, input.lengths) || !(stream >> input.base_height) ||
      !Read(stream, input.encoder_zero) || !Read(stream, input.senses) ||
      !Read(stream, input.lower_commands) ||
      !Read(stream, input.upper_commands))
    return false;
  for (auto *pose : {&input.target, &input.base, &input.tool}) {
    if (!Read(stream, pose->quaternion) || !Read(stream, pose->position) ||
        !Read(stream, pose->input_rotation))
      return false;
  }
  return true;
}

void Rows(const crx::exact::PoseInput &input, std::array<double, 16> &row) {
  row.fill(0);
  row.back() = 1;
  for (std::size_t i = 0; i < 3; ++i) {
    row.at(4 * i + 3) = input.position.at(i);
    for (std::size_t j = 0; j < 3; ++j)
      row.at(4 * i + j) = input.input_rotation.at(3 * i + j);
  }
}

auto Nominate(const crx::exact::Problem &problem)
    -> crx::canonical::JointDiscovery {
  const auto &data = problem.Data();
  const auto midpoint = [](crx::interval::Interval x) {
    return (x.lo / 2) + (x.hi / 2);
  };
  const crx::canonical::Lengths lengths{
      midpoint(data.lengths.a), midpoint(data.lengths.b),
      midpoint(data.lengths.c), midpoint(data.lengths.r)};
  crx::PoseIsoRT target = crx::PoseIsoRT::Identity();
  for (int i = 0; i < 3; ++i) {
    target.translation()(i) = midpoint(data.target.translation(i));
    for (int j = 0; j < 3; ++j)
      target.linear()(i, j) = midpoint(data.target.rotation(i, j));
  }
  return crx::canonical::DiscoverJointCandidates(lengths, target,
                                                 {1e-12, 1e-12});
}

auto CertifyBalls(const Case &item) -> std::vector<crx::ball::Certificate> {
  std::vector<crx::ball::Certificate> balls;
  const auto count = std::min(
      item.nominations.count,
      static_cast<std::size_t>(item.options.max_candidate_certificates));
  for (std::size_t i = 0; i < count; ++i) {
    crx::ball::Centre centre;
    for (std::size_t j = 0; j < kAxes; ++j)
      centre.at(j) = item.nominations.candidates.at(i).joints(
          static_cast<Eigen::Index>(j));
    auto attempt = crx::ball::Certify(item.problem, centre);
    if (attempt.IsCertified())
      balls.push_back(std::move(attempt.certificate));
  }
  return balls;
}

auto Packet(std::istream &input, Case &item) -> bool {
  int profile = 0, seeded = 0;
  if (!(input >> item.id >> item.mode >> item.model_id >> profile >> seeded >>
        item.capacity))
    return false;
  auto &roots = item.options.roots;
  if (!(input >> roots.max_nodes >> roots.max_depth >> roots.max_leaves >>
        roots.max_chart_attempts >> item.options.max_candidate_certificates >>
        item.options.max_pair_tests >> item.commands.max_lifts))
    Fail("Invalid stage budget packet");
  item.raw.profile = static_cast<crx::command::Profile>(profile);
  item.raw.has_seed = seeded != 0;
  item.commands.capacity = static_cast<std::size_t>(item.capacity);
  if (item.capacity < 0 || item.capacity > 512 ||
      !Read(input, item.original_limits) || !ReadInput(input, item.input) ||
      !Read(input, item.raw.lower_degrees) ||
      !Read(input, item.raw.upper_degrees) ||
      !Read(input, item.raw.seed_degrees) || !Read(input, item.raw.weights))
    Fail("Invalid stage input packet");
  return true;
}

void Prepare(Case &item, const Api &api, const Api &parent) {
  const auto convention = item.mode == 1
                              ? crx::exact::FrameConvention::BasePose
                              : crx::exact::FrameConvention::ReferenceMap;
  if (crx::exact::Prepare(item.input, convention, item.problem) !=
      crx::exact::Status::Ready)
    Fail("Stage input preparation failed");
  if (crx::command::Prepare(item.problem, item.raw, item.context) !=
      crx::command::Failure::None)
    Fail("Stage command context failed");
  item.roots = crx::root::Certify(item.problem, item.options.roots);
  if (!item.roots.IsComplete() || item.roots.Values().simple_roots != 0) {
    item.nominations = Nominate(item.problem);
    item.attempted_balls = static_cast<std::uint32_t>(std::min(
        item.nominations.count,
        static_cast<std::size_t>(item.options.max_candidate_certificates)));
    item.balls = CertifyBalls(item);
  }
  item.phases = crx::catalogue::Assemble(item.problem, item.roots, item.balls,
                                         item.options.max_pair_tests);
  const auto assembled = crx::catalogue::Build(item.problem, item.options);
  if (assembled.catalogue.Values().coverage != item.phases.Values().coverage ||
      assembled.certified_balls != item.balls.size() ||
      assembled.nominations != item.nominations.count)
    Fail("Stage/full discovery outcomes differ");
  item.complete_commands = crx::command::Certify(item.problem, item.phases,
                                                 item.context, item.commands);
  item.ranges =
      crx::benchmark::PrepareRanges(item.problem, item.phases, item.context);
  item.enumerated = item.ranges;
  item.enumerate_ready =
      crx::benchmark::Enumerate(item.enumerated, item.commands.max_lifts);
  item.ranked = item.enumerated;
  crx::benchmark::Rank(item.ranked);
  const auto &complete = item.complete_commands.Values();
  if (item.enumerate_ready &&
      (complete.yes != item.ranked.yes ||
       complete.undecided != item.ranked.undecided ||
       complete.ranking != item.ranked.ranking ||
       complete.possible_minimizers != item.ranked.possible_minimizers ||
       complete.lifts.size() != item.ranked.lifts.size()))
    Fail("Stage/full command outcomes differ");
  for (std::size_t i = 0; item.enumerate_ready && i < complete.lifts.size();
       ++i) {
    const auto &a = complete.lifts.at(i), &b = item.ranked.lifts.at(i);
    if (a.phase != b.phase || a.turns != b.turns ||
        a.admission != b.admission ||
        (a.travel_squared.IsValid() &&
         (a.travel_squared.lo != b.travel_squared.lo ||
          a.travel_squared.hi != b.travel_squared.hi)))
      Fail("Stage/full ordered command data differ");
  }
  if (item.model_id >= 0 && item.mode == 2) {
    if (api.create(item.model_id, item.original_limits.data(), &item.handle) !=
            0 ||
        parent.create(item.model_id, item.original_limits.data(),
                      &item.parent) != 0)
      Fail("Cannot create benchmark model");
    Rows(item.input.target, item.target);
    Rows(item.input.base, item.reference);
    Rows(item.input.tool, item.tool);
    using Row = Eigen::Matrix<double, 4, 4, Eigen::RowMajor>;
    crx::PoseIsoRT target, reference, tool;
    target.matrix() = Eigen::Map<const Row>(item.target.data());
    reference.matrix() = Eigen::Map<const Row>(item.reference.data());
    tool.matrix() = Eigen::Map<const Row>(item.tool.data());
    const crx::PoseIsoRT flange = reference * target * tool.inverse();
    Eigen::Map<Row>(item.flange.data()) = flange.matrix();
    std::copy(item.raw.lower_degrees.begin(), item.raw.lower_degrees.end(),
              item.query_limits.begin());
    std::copy(item.raw.upper_degrees.begin(), item.raw.upper_degrees.end(),
              item.query_limits.begin() + kAxes);
  }
}

auto Eligible(const Case &item, Operation op) -> bool {
  if (op == Operation::Nominate)
    return !item.roots.IsComplete() || item.roots.Values().simple_roots != 0;
  if (op == Operation::Balls)
    return item.attempted_balls > 0;
  if (op == Operation::Ranges)
    return item.phases.IsComplete();
  if (op == Operation::Enumerate || op == Operation::Rank)
    return item.enumerate_ready;
  if (op == Operation::ApiDisabled || op == Operation::ApiEnabled)
    return item.handle != nullptr;
  if (op == Operation::Parent)
    return item.capacity > 0 && item.parent != nullptr &&
           item.raw.profile == crx::command::Profile::Callback &&
           item.original_limits == item.query_limits;
  return true;
}

void Reset(const Case &item, Operation op, Scratch &scratch) {
  scratch.status = 0;
  if (op == Operation::Roots)
    scratch.roots = {};
  if (op == Operation::Balls || op == Operation::Assemble) {
    std::vector<crx::ball::Certificate>().swap(scratch.balls);
  }
  if (op == Operation::Assemble) {
    scratch.assembly_roots = item.roots;
    scratch.balls = item.balls;
    scratch.phases = {};
  }
  if (op == Operation::Ranges)
    scratch.commands = {};
  if (op == Operation::Enumerate)
    scratch.commands = item.ranges;
  if (op == Operation::Rank)
    scratch.commands = item.enumerated;
  if (op == Operation::Commands)
    scratch.certificate = {};
  scratch.summary = {};
}

void CallApi(const Case &item, const Api &api, bool enabled, Scratch &scratch) {
  crx_v1_catalogue_options options{};
  options.struct_size = sizeof(options);
  options.version = 1;
  options.flags = enabled ? 0 : CRX_CATALOGUE_DISABLED;
  options.profile = static_cast<std::uint32_t>(item.raw.profile);
  options.capacity = static_cast<std::uint32_t>(item.capacity);
  options.max_nodes = item.options.roots.max_nodes;
  options.max_depth = item.options.roots.max_depth;
  options.max_leaves = item.options.roots.max_leaves;
  options.max_chart_attempts = item.options.roots.max_chart_attempts;
  options.max_candidate_certificates = item.options.max_candidate_certificates;
  options.max_pair_tests = item.options.max_pair_tests;
  options.max_lifts = item.commands.max_lifts;
  crx_v1_catalogue_handle *handle = nullptr;
  scratch.status =
      api.query(item.handle, item.target.data(), item.reference.data(),
                item.tool.data(), item.query_limits.data(),
                item.raw.has_seed ? item.raw.seed_degrees.data() : nullptr,
                item.raw.weights.data(), &options, &handle);
  if (scratch.status != 0 || handle == nullptr)
    Fail("Native query failed");
  scratch.summary.struct_size = sizeof(scratch.summary);
  scratch.summary.version = 1;
  scratch.status = api.read(handle, &scratch.summary);
  api.release(handle);
  if (scratch.status != 0)
    Fail("Native query summary failed");
}

void Run(const Case &item, Operation op, const Api &api, const Api &parent,
         Scratch &scratch) {
  switch (op) {
  case Operation::Prepare:
    scratch.status = static_cast<int>(crx::exact::Prepare(
        item.input,
        item.mode == 1 ? crx::exact::FrameConvention::BasePose
                       : crx::exact::FrameConvention::ReferenceMap,
        scratch.problem));
    break;
  case Operation::Roots:
    scratch.roots = crx::root::Certify(item.problem, item.options.roots);
    break;
  case Operation::Nominate:
    scratch.nominations = Nominate(item.problem);
    break;
  case Operation::Balls:
    scratch.balls = CertifyBalls(item);
    break;
  case Operation::Assemble:
    scratch.phases = crx::catalogue::Assemble(
        item.problem, std::move(scratch.assembly_roots),
        std::move(scratch.balls), item.options.max_pair_tests);
    break;
  case Operation::Context:
    scratch.status = static_cast<int>(
        crx::command::Prepare(item.problem, item.raw, scratch.context));
    break;
  case Operation::Ranges:
    scratch.commands =
        crx::benchmark::PrepareRanges(item.problem, item.phases, item.context);
    break;
  case Operation::Enumerate:
    scratch.status =
        crx::benchmark::Enumerate(scratch.commands, item.commands.max_lifts)
            ? 0
            : 1;
    break;
  case Operation::Rank:
    crx::benchmark::Rank(scratch.commands);
    break;
  case Operation::Commands:
    scratch.certificate = crx::command::Certify(item.problem, item.phases,
                                                item.context, item.commands);
    break;
  case Operation::ApiDisabled:
    CallApi(item, api, false, scratch);
    break;
  case Operation::ApiEnabled:
    CallApi(item, api, true, scratch);
    break;
  case Operation::Parent:
    scratch.status = parent.ik(
        item.parent, item.flange.data(),
        item.raw.has_seed ? item.raw.seed_degrees.data() : nullptr,
        item.capacity, scratch.numerical_values.data(), &scratch.numerical);
    break;
  }
}

void Mix(std::uint64_t &digest, double value) {
  std::uint64_t bits = 0;
  std::memcpy(&bits, &value, sizeof(bits));
  digest = ((digest << 7) | (digest >> 57)) ^ bits;
}

void Consume(std::uint64_t &digest, const crx::ball::Certificate &certificate) {
  const auto &data = certificate.Values();
  for (const auto value : data.centre)
    Mix(digest, value);
  for (int i = 0; i < 6; ++i)
    for (int j = 0; j < 6; ++j)
      Mix(digest, data.preconditioner(i, j));
  for (const auto value :
       {data.existence_radius, data.uniqueness_radius, data.beta,
        data.lipschitz, data.eta, data.inverse_residual, data.pose_defect})
    Mix(digest, value);
  for (const auto word : data.identity.words)
    digest ^= word;
}

auto Describe(const Case &item, Operation op, const Scratch &scratch)
    -> Outcome {
  Outcome out;
  out.status = scratch.status;
  const auto &root = op == Operation::Roots ? scratch.roots.Values()
                     : op == Operation::Assemble
                         ? scratch.phases.Values().root_cover.Values()
                         : item.roots.Values();
  out.roots = root.simple_roots;
  out.nodes = root.nodes;
  for (const auto &part : root.partitions) {
    out.leaves += part.leaves.size();
    out.depth = std::max(out.depth, static_cast<std::uint64_t>(part.max_depth));
  }
  if (op == Operation::Roots || op == Operation::Assemble) {
    for (const auto &part : root.partitions)
      for (const auto &leaf : part.leaves) {
        for (const auto value : leaf.residual_bernstein) {
          Mix(out.digest, value.lo);
          Mix(out.digest, value.hi);
        }
        for (const auto value : leaf.derivative_bernstein) {
          Mix(out.digest, value.lo);
          Mix(out.digest, value.hi);
        }
        for (const auto value : leaf.determinant_bernstein) {
          Mix(out.digest, value.lo);
          Mix(out.digest, value.hi);
        }
      }
  }
  if (op == Operation::Roots)
    out.status = static_cast<int>(root.status);
  const auto &phases = op == Operation::Assemble ? scratch.phases : item.phases;
  out.coverage = static_cast<int>(phases.Values().coverage);
  out.phases = phases.Values().balls.size();
  out.nominations = op == Operation::Nominate ? scratch.nominations.count
                                              : item.nominations.count;
  out.balls = op == Operation::Balls ? scratch.balls.size() : item.balls.size();
  if (op == Operation::Balls)
    for (const auto &ball : scratch.balls)
      Consume(out.digest, ball);
  if (op == Operation::Assemble)
    for (const auto &ball : scratch.phases.Values().balls)
      Consume(out.digest, ball);
  if (op == Operation::Nominate)
    for (std::size_t i = 0; i < scratch.nominations.count; ++i) {
      const auto &candidate = scratch.nominations.candidates.at(i);
      for (int j = 0; j < 6; ++j)
        Mix(out.digest, candidate.joints(j));
    }
  if (op == Operation::Nominate)
    out.status = static_cast<int>(scratch.nominations.status);
  if (op == Operation::Assemble)
    out.status = static_cast<int>(scratch.phases.Values().failure);
  const auto &commands = op == Operation::Commands
                             ? scratch.certificate.Values()
                         : (op == Operation::Ranges ||
                            op == Operation::Enumerate || op == Operation::Rank)
                             ? scratch.commands
                             : item.complete_commands.Values();
  out.ranking = static_cast<int>(commands.ranking);
  out.box_lifts = commands.box_lifts;
  out.yes = commands.yes;
  out.undecided = commands.undecided;
  out.commands = commands.lifts.size();
  if (op == Operation::Commands || op == Operation::Ranges)
    out.status = static_cast<int>(commands.failure);
  if (op == Operation::Commands || op == Operation::Ranges ||
      op == Operation::Enumerate || op == Operation::Rank) {
    for (const auto &lift : commands.lifts) {
      for (const auto value : lift.radians) {
        Mix(out.digest, value.lo);
        Mix(out.digest, value.hi);
      }
      Mix(out.digest, lift.travel_squared.lo);
      Mix(out.digest, lift.travel_squared.hi);
    }
  }
  if (op == Operation::ApiDisabled || op == Operation::ApiEnabled) {
    if (op == Operation::ApiEnabled &&
        (scratch.summary.identity_schema !=
             item.problem.Data().identity.schema ||
         !std::equal(item.problem.Data().identity.words.begin(),
                     item.problem.Data().identity.words.end(),
                     scratch.summary.input_words))) {
      Fail("Native/stage exact identity mismatch");
    }
    out.coverage = scratch.summary.coverage;
    out.ranking = scratch.summary.ranking;
    out.phases = scratch.summary.phase_count;
    out.roots = scratch.summary.root_count;
    out.yes = scratch.summary.yes;
    out.undecided = scratch.summary.undecided;
    out.commands = scratch.summary.command_count;
    out.box_lifts = scratch.summary.box_lifts;
    out.digest = 0;
    for (const auto word : scratch.summary.input_words)
      out.digest ^= word;
  }
  if (op == Operation::Parent) {
    out.status = scratch.numerical.status;
    out.commands = static_cast<std::uint64_t>(scratch.numerical.count);
    out.digest = 0;
    for (int i = 0; i < scratch.numerical.count * static_cast<int>(kAxes); ++i)
      Mix(out.digest, scratch.numerical_values.at(static_cast<std::size_t>(i)));
  }
  return out;
}

auto Equal(const Outcome &a, const Outcome &b) -> bool {
  return a.status == b.status && a.coverage == b.coverage &&
         a.ranking == b.ranking && a.roots == b.roots && a.nodes == b.nodes &&
         a.leaves == b.leaves && a.depth == b.depth && a.phases == b.phases &&
         a.nominations == b.nominations && a.balls == b.balls &&
         a.box_lifts == b.box_lifts && a.yes == b.yes &&
         a.undecided == b.undecided && a.commands == b.commands &&
         a.digest == b.digest;
}

void Write(std::ostream &stream, int id, std::size_t op, const Outcome &out) {
  stream << id << ',' << kNames.at(op) << ',' << out.status << ','
         << out.coverage << ',' << out.ranking << ',' << out.roots << ','
         << out.nodes << ',' << out.leaves << ',' << out.depth << ','
         << out.phases << ',' << out.nominations << ',' << out.balls << ','
         << out.box_lifts << ',' << out.yes << ',' << out.undecided << ','
         << out.commands << ',' << out.digest << '\n';
}
} // namespace

auto main(int argc, char **argv) -> int {
  if (argc != 6 && argc != 8)
    Fail("Usage: benchmark_catalogues --verify|--time packets candidate.dll "
         "parent.dll output [rounds minimum_ms]");
  const bool timing = std::string(argv[1]) == "--time";
  if (!timing && std::string(argv[1]) != "--verify")
    Fail("Expected explicit verification or timing mode");
  const int rounds = argc == 8 ? std::stoi(argv[6]) : 7;
  const double minimum_ms = argc == 8 ? std::stod(argv[7]) : 5;
  if (rounds < 1 || rounds > 30 || !(minimum_ms >= 1 && minimum_ms <= 100))
    Fail("Invalid timing budget");
  if (!crx::interval::EnvironmentSupported())
    Fail("Unsupported FP environment");
  const Api api = Load(std::filesystem::absolute(argv[3]), true),
            parent = Load(std::filesystem::absolute(argv[4]), false);
  std::ifstream input(argv[2]);
  std::vector<Case> cases;
  for (;;) {
    Case item;
    if (!Packet(input, item))
      break;
    Prepare(item, api, parent);
    cases.push_back(std::move(item));
  }
  if (!input.eof() || cases.empty())
    Fail("Invalid or empty packet file");
  const auto output = std::filesystem::path(argv[5]);
  std::filesystem::create_directories(output);
  std::ofstream outcomes(output / "outcomes.csv"),
      samples(output / "timings.csv");
  outcomes
      << "case,operation,status,coverage,ranking,roots,nodes,leaves,depth,"
         "phases,nominations,balls,box_lifts,yes,undecided,commands,digest\n";
  samples << "case,operation,round,iterations,total_ns,us\n"
          << std::setprecision(17);
  std::vector<std::array<Outcome, kNames.size()>> expected(cases.size());
  Scratch scratch;
  for (std::size_t index = 0; index < cases.size(); ++index) {
    const auto &item = cases.at(index);
    for (std::size_t op = 0; op < kNames.size(); ++op) {
      const auto operation = static_cast<Operation>(op);
      if (!Eligible(item, operation))
        continue;
      Reset(item, operation, scratch);
      Run(item, operation, api, parent, scratch);
      expected.at(index).at(op) = Describe(item, operation, scratch);
      if (operation == Operation::ApiEnabled &&
          (scratch.summary.coverage !=
               static_cast<int>(item.phases.Values().coverage) ||
           scratch.summary.ranking !=
               static_cast<int>(item.complete_commands.Values().ranking) ||
           scratch.summary.yes != item.complete_commands.Values().yes))
        Fail("Native/component proof outcomes differ");
      Write(outcomes, item.id, op, expected.at(index).at(op));
    }
  }
  outcomes.flush();
  if (timing) {
    std::mt19937 random(20261113);
    std::vector<std::pair<std::size_t, std::size_t>> order;
    for (std::size_t i = 0; i < cases.size(); ++i)
      for (std::size_t op = 0; op < kNames.size(); ++op)
        if (Eligible(cases.at(i), static_cast<Operation>(op)))
          order.emplace_back(i, op);
    for (int round = -1; round < rounds; ++round) {
      std::shuffle(order.begin(), order.end(), random);
      for (const auto [index, op] : order) {
        const auto &item = cases.at(index);
        const auto operation = static_cast<Operation>(op);
        double ns = 0;
        int iterations = 0;
        const double budget_ns = round < 0 ? 0 : minimum_ms * 1e6;
        do {
          Reset(item, operation,
                scratch); // Copies/cleanup excluded from isolated stage costs.
          std::atomic_signal_fence(std::memory_order_seq_cst);
          const auto start = std::chrono::steady_clock::now();
          Run(item, operation, api, parent, scratch);
          const auto stop = std::chrono::steady_clock::now();
          std::atomic_signal_fence(std::memory_order_seq_cst);
          ns += std::chrono::duration<double, std::nano>(stop - start).count();
          const auto actual =
              Describe(item, operation,
                       scratch); // Consumption/validation outside timing.
          if (!Equal(actual, expected.at(index).at(op)))
            Fail("Timed outcome changed");
          checksum ^= actual.digest;
          ++iterations;
        } while (ns < budget_ns && iterations < 16384);
        if (round >= 0)
          samples << item.id << ',' << kNames.at(op) << ',' << round << ','
                  << iterations << ',' << ns << ',' << ns / iterations / 1000
                  << '\n';
      }
      std::cerr << "Round " << round + 1 << '/' << rounds << " complete\n";
    }
  }
  for (auto &item : cases) {
    if (item.handle != nullptr)
      api.destroy(item.handle);
    if (item.parent != nullptr)
      parent.destroy(item.parent);
  }
  FreeLibrary(api.module);
  FreeLibrary(parent.module);
  std::cout << "Verified " << cases.size() << " cases; timing=" << timing
            << "; checksum=" << checksum << '\n';
  return outcomes && samples ? 0 : 2;
}
