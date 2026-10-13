// Windows x64: compare complete DLL calls on identical inputs in native loops.
#include <algorithm>
#include <array>
#include <chrono>
#include <cstddef>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <random>
#include <string>
#include <vector>

#define NOMINMAX
#include <windows.h>

namespace {
using SolveIk = int (*)(const double *, double *, double *, int, const double *,
                        const void *);
using SolveFk = int (*)(const double *, double *, const void *);
constexpr int kDof = 6;
constexpr int kPoseSize = 16;
constexpr int kStride = 12;
constexpr int kRobotStride = 20;
// Includes the bounded diagnostic name at row 90 (59 doubles).
constexpr int kRobotRows = 93;
constexpr int kWarmupCalls = 3;
constexpr unsigned kOrderSeed = 20261009;
using Robot = std::array<double, kRobotRows * kRobotStride>;

struct Case {
  std::size_t model = 0;
  std::array<double, kPoseSize> pose{};
  std::array<double, kDof> seed{};
};

volatile int checksum = 0;

auto Fail(const char *message) -> int {
  std::cerr << message << '\n';
  return 2;
}

auto Run(int argc, char **argv) -> int {
  if (argc != 9 && argc != 10) {
    std::cerr << "scanner.dll polynomial.dll input timings.csv solutions.csv "
                 "repeats rounds capacity [fk]\n";
    return 2;
  }
  const int repeats = std::stoi(argv[6]), rounds = std::stoi(argv[7]);
  const int capacity = std::stoi(argv[8]);
  const bool fk = argc == 10 && std::string(argv[9]) == "fk";
  if (argc == 10 && !fk) {
    return Fail("Unknown benchmark mode");
  }
  if (repeats < 1 || rounds < 1 || capacity < 1) {
    return 2;
  }
  std::array<HMODULE, 2> libraries{};
  std::array<SolveIk, 2> solves{};
  std::array<SolveFk, 2> forwards{};
  for (std::size_t side = 0; side < libraries.size(); ++side) {
    libraries[side] = LoadLibraryA(argv[side + 1]);
    if (libraries[side] == nullptr) {
      return Fail("LoadLibrary failed");
    }
    solves[side] =
        reinterpret_cast<SolveIk>(GetProcAddress(libraries[side], "SolveIK"));
    if (solves[side] == nullptr) {
      return Fail("SolveIK export missing");
    }
    forwards[side] =
        reinterpret_cast<SolveFk>(GetProcAddress(libraries[side], "SolveFK"));
    if (forwards[side] == nullptr) {
      return Fail("SolveFK export missing");
    }
  }
  DWORD_PTR process_mask = 0, system_mask = 0;
  if (!GetProcessAffinityMask(GetCurrentProcess(), &process_mask,
                              &system_mask)) {
    return Fail("Cannot inspect process affinity");
  }
  const DWORD_PTR cpu = process_mask & (~process_mask + 1);
  if (SetThreadAffinityMask(GetCurrentThread(), cpu) == 0) {
    return Fail("Cannot pin benchmark thread");
  }
  std::cerr << "Thread affinity mask " << cpu << '\n';
  std::ifstream input(argv[3]);
  std::size_t model_count = 0, case_count = 0;
  input >> model_count >> case_count;
  std::vector<Robot> models(model_count);
  for (auto &model : models) {
    for (double &value : model) {
      input >> value;
    }
  }
  std::vector<Case> cases(case_count);
  for (auto &item : cases) {
    input >> item.model;
    for (double &value : item.pose) {
      input >> value;
    }
    for (double &value : item.seed) {
      input >> value;
    }
    if (item.model >= model_count) {
      return Fail("Invalid model index");
    }
  }
  if (!input || cases.empty()) {
    return Fail("Invalid/empty input");
  }
  std::ofstream timings(argv[4]), solutions(argv[5]);
  timings << std::setprecision(17) << "case,mode,path,round,us,count\n";
  solutions << std::setprecision(17) << "case,mode,path,solution";
  for (int component = 1; component <= (fk ? kPoseSize : kDof); ++component) {
    solutions << ",q" << component;
  }
  solutions << '\n';
  std::vector<std::size_t> order(case_count * 2);
  std::iota(order.begin(), order.end(), 0);
  std::mt19937 generator(kOrderSeed);
  std::vector<double> all(static_cast<std::size_t>(capacity) * kStride);
  for (int round = -1; round < rounds; ++round) {
    std::shuffle(order.begin(), order.end(), generator);
    for (const auto key : order) {
      const auto index = key / 2;
      const bool seeded = key % 2 != 0;
      if (fk && seeded) {
        continue;
      }
      const auto &item = cases[index];
      for (int turn = 0; turn < 2; ++turn) {
        const auto side =
            (key + static_cast<std::size_t>(round + 1 + turn)) % 2;
        std::array<double, kDof> best{};
        std::array<double, kPoseSize> pose{};
        int count = 0, first_count = 0;
        const int iterations = round < 0 ? kWarmupCalls : repeats;
        const auto begin = std::chrono::steady_clock::now();
        for (int iteration = 0; iteration < iterations; ++iteration) {
          count =
              fk ? forwards[side](item.seed.data(), pose.data(),
                                  models[item.model].data())
                 : solves[side](item.pose.data(), best.data(), all.data(),
                                capacity, seeded ? item.seed.data() : nullptr,
                                models[item.model].data());
          if (iteration == 0) {
            first_count = count;
          } else if (count != first_count) {
            return Fail("Nondeterministic return count");
          }
        }
        const auto end = std::chrono::steady_clock::now();
        checksum = count; // Consume an observable result outside the timer.
        if (count > capacity) {
          return Fail("DLL exceeded output capacity");
        }
        if (round < 0) {
          continue;
        }
        const double us =
            std::chrono::duration<double, std::micro>(end - begin).count() /
            iterations;
        const char *path = side == 0 ? "legacy" : "polynomial";
        const char *mode = fk ? "fk" : seeded ? "nearby_seed" : "no_seed";
        timings << index << ',' << mode << ',' << path << ',' << round << ','
                << us << ',' << count << '\n';
        if (round == 0 && count > 0) {
          // -1 identifies the best-command output, separately from the list.
          for (int solution = fk ? 0 : -1; solution < count; ++solution) {
            solutions << index << ',' << mode << ',' << path << ',' << solution;
            for (int joint = 0; joint < (fk ? kPoseSize : kDof); ++joint) {
              const auto axis = static_cast<std::size_t>(joint);
              solutions
                  << ','
                  << (fk ? pose[axis]
                      : solution < 0
                          ? best[axis]
                          : all[static_cast<std::size_t>(solution) * kStride +
                                axis]);
            }
            solutions << '\n';
          }
        }
      }
    }
    std::cerr << "Round " << round + 1 << '/' << rounds << " complete\n";
  }
  for (auto library : libraries) {
    FreeLibrary(library);
  }
  std::cerr << "Checksum " << checksum << '\n';
  return timings && solutions ? 0 : 2;
}
} // namespace

auto main(int argc, char **argv) -> int { return Run(argc, argv); }
