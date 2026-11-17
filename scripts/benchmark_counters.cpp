// SPDX-License-Identifier: Apache-2.0
// SPDX-FileCopyrightText: 2026 Centre Technologique en Aérospatiale

// Untimed diagnostic companion to benchmark_solvers.cpp. Reads the identical
// serialized model/pose/seed input; emits one counter record per target/mode.
#include "crx_discovery_counters.h"
#include "crx_robodk_adapter.h"
#include "crx_solver.h"
#include "crx_types.h"

#include <array>
#include <cstddef>
#include <exception>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

namespace {
constexpr std::size_t kRobotSize = 93 * 20;
using Robot = std::array<double, kRobotSize>;

auto Run(int argc, char **argv) -> int {
  if (argc != 4) {
    std::cerr << "input.txt counters.csv capacity\n";
    return 2;
  }
  const int capacity = std::stoi(argv[3]);
  std::ifstream input(argv[1]);
  std::size_t model_count = 0, case_count = 0;
  if (!(input >> model_count >> case_count) || model_count == 0 ||
      case_count == 0 || capacity <= 0) {
    return 2;
  }
  std::vector<crx::CrxModelData> models(model_count);
  for (auto &model : models) {
    Robot robot{};
    for (auto &value : robot) {
      input >> value;
    }
    if (!input || !crx::BuildModelFromRoboDkRobot(robot.data(), model)) {
      return 2;
    }
  }
  std::ofstream output(argv[2]);
  output << "case,mode,status,count,cramer_runs,cramer_evaluations,"
            "cramer_guard_rejections,cramer_initial_rejections,"
            "cramer_successes,cramer_recovery_failures\n";
  for (std::size_t index = 0; index < case_count; ++index) {
    std::size_t model_index = 0;
    input >> model_index;
    crx::PoseIsoRT target;
    crx::Vec6 seed;
    for (int column = 0; column < 4; ++column) {
      for (int row = 0; row < 4; ++row) {
        input >> target.matrix()(row, column);
      }
    }
    for (int axis = 0; axis < crx::kDofCount; ++axis) {
      input >> seed[axis];
    }
    seed *= angle_conv::kRadPerDeg;
    if (!input || model_index >= models.size()) {
      return 2;
    }
    for (const bool seeded : {false, true}) {
      crx::canonical::discovery_counters = {};
      const auto result = crx::SolveIkIsometry(
          models.at(model_index), target, seeded ? &seed : nullptr, capacity);
      const auto &count = crx::canonical::discovery_counters;
      if (count.cramer_runs != count.cramer_initial_rejections +
                                   count.cramer_successes +
                                   count.cramer_recovery_failures) {
        return 3;
      }
      output << index << ',' << (seeded ? "nearby_seed" : "no_seed") << ','
             << static_cast<int>(result.status) << ','
             << result.solutions.size() << ',' << count.cramer_runs << ','
             << count.cramer_evaluations << ',' << count.cramer_guard_rejections
             << ',' << count.cramer_initial_rejections << ','
             << count.cramer_successes << ',' << count.cramer_recovery_failures
             << '\n';
    }
  }
  return output ? 0 : 2;
}
} // namespace

auto main(int argc, char **argv) -> int {
  try {
    return Run(argc, argv);
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 2;
  }
}
