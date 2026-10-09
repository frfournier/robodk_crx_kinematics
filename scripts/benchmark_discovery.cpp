// Windows-only snapshot harness. Time the production DLL and the current
// polynomial component in native loops, without Python/process startup cost.
#include "crx_discovery.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <random>
#include <vector>

#define NOMINMAX
#include <windows.h>

namespace {
using SolveIk = int (*)(const double *, double *, double *, int, const double *,
                        const void *);
using Robot = std::array<double, 32 * 20>;
constexpr int kCapacity = 64;
constexpr double kRadians = 3.14159265358979323846 / 180;

struct Case {
  std::size_t model = 0;
  crx::canonical::Lengths lengths;
  crx::PoseIsoRT canonical = crx::PoseIsoRT::Identity();
  std::array<double, 16> production{};
};

struct ProductionResult {
  int count = 0;
  std::array<double, 6> best{};
  std::array<double, 12 * kCapacity> joints{};
};

volatile double checksum = 0;
} // namespace

auto main(int argc, char **argv) -> int {
  if (argc != 7) {
    std::cerr << "DLL input timings.csv solutions.csv repeats rounds\n";
    return 2;
  }
  const int repeats = std::stoi(argv[5]);
  const int rounds = std::stoi(argv[6]);
  if (repeats < 1 || rounds < 1) {
    return 2;
  }
  HMODULE library = LoadLibraryA(argv[1]);
  if (library == nullptr) {
    return 2;
  }
  const auto solve =
      reinterpret_cast<SolveIk>(GetProcAddress(library, "SolveIK"));
  if (solve == nullptr) {
    FreeLibrary(library);
    return 2;
  }
  DWORD_PTR process_mask = 0, system_mask = 0;
  if (GetProcessAffinityMask(GetCurrentProcess(), &process_mask,
                             &system_mask)) {
    const DWORD_PTR cpu = process_mask & (~process_mask + 1);
    if (SetThreadAffinityMask(GetCurrentThread(), cpu) != 0) {
      std::cerr << "Thread affinity mask " << cpu << '\n';
    }
  }
  std::ifstream input(argv[2]);
  std::size_t model_count = 0, case_count = 0;
  input >> model_count >> case_count;
  std::vector<Robot> models(model_count);
  for (auto &model : models) {
    for (double &v : model) {
      input >> v;
    }
  }
  std::vector<Case> cases(case_count);
  for (auto &item : cases) {
    input >> item.model >> item.lengths.a >> item.lengths.b >> item.lengths.c >>
        item.lengths.r;
    for (int r = 0; r < 4; ++r) {
      for (int c = 0; c < 4; ++c) {
        input >> item.canonical.matrix()(r, c);
      }
    }
    for (double &v : item.production) {
      input >> v;
    }
    if (item.model >= model_count) {
      return 2;
    }
  }
  if (!input || cases.empty()) {
    return 2;
  }
  std::ofstream timings(argv[3]), solutions(argv[4]);
  if (!timings || !solutions) {
    return 2;
  }
  timings << std::setprecision(17) << "case,path,round,us,status,count\n";
  solutions << std::setprecision(17) << "case,path,q1,q2,q3,q4,q5,q6\n";
  const crx::canonical::PoseTolerance tolerance{1e-4, 1e-3 * kRadians};
  std::vector<std::size_t> order(case_count);
  std::iota(order.begin(), order.end(), 0);
  std::mt19937 generator(20261009);
  for (int round = -1; round < rounds; ++round) {
    std::shuffle(order.begin(), order.end(), generator);
    for (const auto index : order) {
      const auto &item = cases[index];
      // Alternate which implementation is timed first; same target and thread.
      for (int side = 0; side < 2; ++side) {
        const bool polynomial =
            (static_cast<int>(index) + round + side) % 2 == 0;
        ProductionResult production;
        crx::canonical::JointDiscovery discovered;
        const int iterations = round < 0 ? 3 : repeats;
        double consumed = 0;
        const auto begin = std::chrono::steady_clock::now();
        for (int iteration = 0; iteration < iterations; ++iteration) {
          if (polynomial) {
            discovered = crx::canonical::DiscoverJointCandidates(
                item.lengths, item.canonical, tolerance);
            consumed += static_cast<double>(discovered.count) +
                        static_cast<double>(discovered.status);
          } else {
            production.count =
                solve(item.production.data(), production.best.data(),
                      production.joints.data(), kCapacity, nullptr,
                      models[item.model].data());
            consumed += production.count;
          }
        }
        const auto end = std::chrono::steady_clock::now();
        checksum += consumed;
        if (round < 0) {
          continue;
        }
        const double us =
            std::chrono::duration<double, std::micro>(end - begin).count() /
            iterations;
        const int status = polynomial ? static_cast<int>(discovered.status)
                           : production.count > 0  ? 0
                           : production.count == 0 ? 1
                                                   : 3;
        const auto count =
            polynomial
                ? discovered.count
                : static_cast<std::size_t>(std::max(0, production.count));
        const char *path = polynomial ? "polynomial" : "production";
        timings << index << ',' << path << ',' << round << ',' << us << ','
                << status << ',' << count << '\n';
        if (round == 0) {
          for (std::size_t i = 0; i < count; ++i) {
            solutions << index << ',' << path;
            for (int joint = 0; joint < 6; ++joint) {
              const double value =
                  polynomial
                      ? discovered.candidates[i].joints[joint]
                      : production.joints[12 * i +
                                          static_cast<std::size_t>(joint)] *
                            kRadians;
              solutions << ',' << value;
            }
            solutions << '\n';
          }
        }
      }
    }
    std::cerr << "Round " << round + 1 << '/' << rounds << " complete\n";
  }
  FreeLibrary(library);
  std::cerr << "Checksum " << checksum << '\n';
  return timings && solutions ? 0 : 2;
}
