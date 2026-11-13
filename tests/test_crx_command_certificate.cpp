// SPDX-License-Identifier: Apache-2.0
// SPDX-FileCopyrightText: 2026 Centre Technologique en Aérospatiale

#include "crx_command_certificate.h"
#include "crx_exact_problem.h"
#include "crx_interval.h"
#include "crx_phase_catalogue.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdlib>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <limits>
#include <type_traits>

#include <xmmintrin.h>

namespace {
using crx::command::Failure;
using crx::command::Ranking;
using crx::interval::Interval;

void Require(bool condition, const char *message) {
  if (!condition) {
    std::cerr << "Command certificate test failure: " << message << '\n';
    std::exit(1);
  }
}

auto Witness() -> crx::exact::Input {
  crx::exact::Input input;
  input.lengths = {1, .6, .2, .5};
  input.target.quaternion = {60, -7, 7, -19};
  input.target.position = {27.0 / 64, 15.0 / 32, 55.0 / 128};
  input.lower_commands.fill(-4);
  input.upper_commands.fill(4);
  return input;
}

void Contracts() {
  auto input = Witness();
  crx::exact::Problem problem;
  Require(crx::exact::Prepare(input, problem) == crx::exact::Status::Ready,
          "prepare witness");
  const auto phases = crx::catalogue::Build(problem).catalogue;
  Require(phases.IsComplete(), "phase cover");
  crx::command::Input raw;
  raw.lower_degrees.fill(-180);
  raw.upper_degrees.fill(180);
  raw.has_seed = true;
  crx::command::Context context;
  Require(crx::command::Prepare(problem, raw, context) == Failure::None,
          "raw context");
  const auto result = crx::command::Certify(problem, phases, context);
  const auto &data = result.Values();
  Require(data.failure == Failure::None && data.phase_complete &&
              data.enumeration_complete && data.admission_complete &&
              data.yes == 16 && data.undecided == 0 && data.lifts.size() == 16,
          "full single-turn command cover");
  Require(result.HasUniqueNearest() && data.possible_minimizers.size() == 1 &&
              data.possible_minimizers.front() == 0,
          "strict nearest witness");
  for (const auto capacity :
       {std::size_t{0}, std::size_t{1}, std::size_t{64}, std::size_t{512}}) {
    crx::command::Options options;
    options.capacity = capacity;
    const auto limited =
        crx::command::Certify(problem, phases, context, options);
    Require(limited.Values().yes == data.yes &&
                limited.Values().lifts.size() == data.lifts.size() &&
                limited.HasUniqueNearest() &&
                limited.Values().returned_count ==
                    std::min(capacity, data.lifts.size()),
            "capacity changed global proof");
  }
  crx::command::Options budget;
  budget.max_lifts = 0;
  const auto exhausted =
      crx::command::Certify(problem, phases, context, budget);
  Require(exhausted.Values().failure == Failure::WorkLimit &&
              !exhausted.Values().enumeration_complete &&
              !exhausted.IsCommandInfeasible() &&
              !exhausted.HasUniqueNearest() &&
              exhausted.Values().ranges.size() == 16 &&
              exhausted.Values().lifts.empty(),
          "budget created global claim");
  crx::catalogue::Options none;
  none.max_candidate_certificates = 0;
  const auto incomplete = crx::catalogue::Build(problem, none).catalogue;
  Require(
      crx::command::Certify(problem, incomplete, context).Values().failure ==
          Failure::IncompletePhaseCover,
      "missing phase accepted");
  auto changed = input;
  changed.encoder_zero.at(0) = 1e-20;
  crx::exact::Problem other;
  Require(crx::exact::Prepare(changed, other) == crx::exact::Status::Ready,
          "changed input");
  Require(crx::command::Certify(other, phases, context).Values().failure ==
              Failure::IdentityMismatch,
          "raw identity ignored");
  raw.lower_degrees.fill(0);
  raw.upper_degrees.fill(0);
  Require(crx::command::Prepare(problem, raw, context) == Failure::None,
          "point limits");
  const auto excluded = crx::command::Certify(problem, phases, context);
  Require(excluded.IsCommandInfeasible() &&
              excluded.Values().ranking == Ranking::NoAdmissible &&
              !phases.IsInfeasible(),
          "command and pose infeasibility confused");
  const auto words = context.Values().raw_words;
  raw.weights.at(0) = 0;
  Require(crx::command::Prepare(problem, raw, context) ==
                  Failure::InvalidInput &&
              context.Values().raw_words == words,
          "invalid prepare mutated output");
  raw.weights.at(0) = std::numeric_limits<double>::infinity();
  Require(crx::command::Prepare(problem, raw, context) == Failure::InvalidInput,
          "infinite weight");
  raw.weights.at(0) = 1;
  raw.seed_degrees.at(0) = std::numeric_limits<double>::quiet_NaN();
  Require(crx::command::Prepare(problem, raw, context) == Failure::InvalidInput,
          "nonfinite seed");
  raw.seed_degrees.at(0) = 0;
  raw.profile = static_cast<crx::command::Profile>(2);
  Require(crx::command::Prepare(problem, raw, context) == Failure::InvalidInput,
          "unknown convention");
  raw.profile = crx::command::Profile::Callback;
  raw.lower_degrees.fill(1e20);
  raw.upper_degrees.fill(1e20);
  Require(crx::command::Prepare(problem, raw, context) == Failure::None,
          "finite large limits");
  const auto huge = crx::command::Certify(problem, phases, context);
  Require(huge.Values().failure == Failure::RangeUnavailable &&
              !huge.IsCommandInfeasible() && !huge.HasUniqueNearest(),
          "unavailable integer range became empty");
  const auto control = _mm_getcsr();
  _mm_setcsr(control | 0x8000U);
  const auto unsupported = crx::command::Certify(problem, phases, context);
  _mm_setcsr(control);
  Require(unsupported.Values().failure == Failure::UnsupportedEnvironment &&
              !unsupported.IsCommandInfeasible(),
          "unsupported environment");
  std::cout << "Command cover/admission/ranking/capacity/budget/identity "
               "checks passed\n";
}

void Print(Interval value) {
  if (value.IsValid())
    std::cout << '[' << value.lo << ',' << value.hi << ']';
  else
    std::cout << "null";
}

template <typename Values> void PrintArray(const Values &values) {
  std::cout << '[';
  bool first = true;
  for (const auto value : values) {
    if (!first)
      std::cout << ',';
    first = false;
    if constexpr (std::is_same_v<typename Values::value_type, Interval>)
      Print(value);
    else
      std::cout << value;
  }
  std::cout << ']';
}

void Print(const crx::command::Certificate &certificate,
           const crx::catalogue::PhaseCatalogue &phases) {
  const auto &data = certificate.Values();
  std::cout << "{\"failure\":" << static_cast<int>(data.failure)
            << ",\"ranking\":" << static_cast<int>(data.ranking)
            << ",\"phase_complete\":" << data.phase_complete
            << ",\"enumeration_complete\":" << data.enumeration_complete
            << ",\"admission_complete\":" << data.admission_complete
            << ",\"infeasible\":" << certificate.IsCommandInfeasible()
            << ",\"unique\":" << certificate.HasUniqueNearest()
            << ",\"box_lifts\":" << data.box_lifts << ",\"yes\":" << data.yes
            << ",\"no\":" << data.no << ",\"undecided\":" << data.undecided
            << ",\"returned_count\":" << data.returned_count
            << ",\"truncated\":" << data.truncated << ",\"minimizers\":";
  PrintArray(data.possible_minimizers);
  const auto &context = data.context.Values();
  std::cout << ",\"words\":[";
  for (std::size_t i = 0; i < context.raw_words.size(); ++i) {
    if (i)
      std::cout << ',';
    std::cout << '"' << std::hex << context.raw_words.at(i) << std::dec << '"';
  }
  std::cout << "],\"lower\":";
  PrintArray(context.lower);
  std::cout << ",\"upper\":";
  PrintArray(context.upper);
  std::cout << ",\"seed\":";
  PrintArray(context.seed);
  std::cout << ",\"balls\":[";
  for (std::size_t i = 0; i < phases.Values().balls.size(); ++i) {
    if (i)
      std::cout << ',';
    const auto &ball = phases.Values().balls.at(i).Values();
    std::cout << "{\"centre\":";
    PrintArray(ball.centre);
    std::cout << ",\"rho\":" << ball.existence_radius << '}';
  }
  std::cout << "],\"ranges\":[";
  for (std::size_t i = 0; i < data.ranges.size(); ++i) {
    if (i)
      std::cout << ',';
    const auto &range = data.ranges.at(i);
    std::cout << "{\"base\":";
    PrintArray(range.base);
    std::cout << ",\"first\":";
    PrintArray(range.first);
    std::cout << ",\"last\":";
    PrintArray(range.last);
    std::cout << ",\"count\":" << range.count
              << ",\"available\":" << range.available
              << ",\"empty\":" << range.empty << '}';
  }
  std::cout << "],\"lifts\":[";
  for (std::size_t i = 0; i < data.lifts.size(); ++i) {
    if (i)
      std::cout << ',';
    const auto &lift = data.lifts.at(i);
    std::cout << "{\"phase\":" << lift.phase << ",\"turns\":";
    PrintArray(lift.turns);
    std::cout << ",\"radians\":";
    PrintArray(lift.radians);
    std::cout << ",\"admission\":" << static_cast<int>(lift.admission)
              << ",\"cost\":";
    Print(lift.travel_squared);
    std::cout << '}';
  }
  std::cout << "]}\n";
}

template <std::size_t N> auto Read(std::array<double, N> &values) -> bool {
  for (auto &value : values)
    std::cin >> value;
  return static_cast<bool>(std::cin);
}

auto Batch() -> int {
  int profile = 0, seeded = 0, mode = 0;
  while (std::cin >> profile >> seeded >> mode) {
    crx::command::Input raw;
    raw.profile = static_cast<crx::command::Profile>(profile);
    raw.has_seed = seeded != 0;
    crx::command::Options options;
    crx::catalogue::Options discovery;
    crx::exact::Input input;
    if (!(std::cin >> options.max_lifts >> options.capacity >>
          discovery.max_candidate_certificates) ||
        !Read(raw.lower_degrees) || !Read(raw.upper_degrees) ||
        !Read(raw.seed_degrees) || !Read(raw.weights) || !Read(input.lengths) ||
        !(std::cin >> input.base_height) || !Read(input.encoder_zero) ||
        !Read(input.senses) || !Read(input.lower_commands) ||
        !Read(input.upper_commands))
      return 2;
    for (auto *pose : {&input.target, &input.base, &input.tool})
      if (!Read(pose->quaternion) || !Read(pose->position) ||
          !Read(pose->input_rotation))
        return 2;
    crx::exact::Problem problem;
    if (crx::exact::Prepare(input,
                            mode == 1
                                ? crx::exact::FrameConvention::BasePose
                                : crx::exact::FrameConvention::ReferenceMap,
                            problem) != crx::exact::Status::Ready)
      return 2;
    crx::command::Context context;
    const auto status = crx::command::Prepare(problem, raw, context);
    if (status != Failure::None) {
      std::cout << "{\"prepare_failure\":" << static_cast<int>(status) << "}\n";
      continue;
    }
    const auto phases = crx::catalogue::Build(problem, discovery).catalogue;
    Print(crx::command::Certify(problem, phases, context, options), phases);
  }
  return std::cin.eof() ? 0 : 2;
}
} // namespace

auto main(int argc, char **argv) -> int {
  std::cout << std::boolalpha << std::setprecision(17);
  if (argc == 2 && std::strcmp(argv[1], "--batch") == 0)
    return Batch();
  if (argc != 1)
    return 2;
  Contracts();
  return 0;
}
