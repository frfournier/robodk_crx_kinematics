// SPDX-License-Identifier: Apache-2.0
// SPDX-FileCopyrightText: 2026 Centre Technologique en Aérospatiale

// Component timings only. Complete DLL latency is measured separately.
#include "benchmark_residuals_baseline.h"
#include "crx_canonical.h"

#include <Eigen/Geometry>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <random>
#include <ratio>
#include <stdexcept>
#include <string>
#include <vector>

#define NOMINMAX
#include <windows.h>

namespace {
using namespace crx;
using namespace crx::canonical;

struct Item {
  std::size_t model = 0;
  std::string group;
  Lengths lengths;
  PoseIsoRT target = PoseIsoRT::Identity();
  double angle = 0;
  WristCircle current;
  residual_baseline::Circle baseline;
};

auto MakeItems() -> std::vector<Item> {
  constexpr std::array<Lengths, 6> kModels{{{410, 430, 145, 130},
                                            {540, 540, 160, 150},
                                            {710, 540, 160, 150},
                                            {710, 540, 160, 150},
                                            {950, 750, 180, 185},
                                            {950, 750, 180, 185}}};
  std::vector<Item> result;
  for (std::size_t model = 0; model < kModels.size(); ++model) {
    const auto &geometry = kModels.at(model);
    for (const double scale : {1e-100, 1.0, 1e100}) {
      for (const double sign : {-1.0, 1.0}) {
        for (int sample = 0; sample < 32; ++sample) {
          Item item;
          item.model = model;
          item.group = "generic";
          item.lengths = {geometry.a * scale, sign * geometry.b * scale,
                          geometry.c * scale, sign * geometry.r * scale};
          item.angle = static_cast<double>(sample) / 7 - 2;
          item.target.linear() =
              (Eigen::AngleAxisd(item.angle / 3, Vec3::UnitZ()) *
               Eigen::AngleAxisd(item.angle / 5, Vec3::UnitY()) *
               Eigen::AngleAxisd(item.angle / 7, Vec3::UnitX()))
                  .toRotationMatrix();
          item.target.translation() =
              scale * Vec3(300 * std::sin(item.angle),
                           400 * std::cos(item.angle), 200 + 100 * item.angle);
          result.push_back(item);
        }
      }
    }
    for (const double normal : {-1.0, 1.0}) {
      for (const double height : {0.0, 0.2}) {
        Item item;
        item.model = model;
        item.group = "vertical";
        item.lengths = geometry;
        item.target.linear().diagonal() = Vec3(1, normal, normal);
        item.target.translation() =
            Vec3(0.7 * geometry.a, 0.1 * geometry.a,
                 normal * geometry.c + height * geometry.a);
        item.angle = 0.7;
        result.push_back(item);
      }
    }
  }
  for (const double epsilon : {1e-3, 1e-6, 1e-9, 1e-12, 1e-16}) {
    for (const bool folded : {false, true}) {
      Item item;
      item.group = folded ? "near_fold" : "near_axis";
      item.lengths = {1, 1, 0, 0.25};
      item.target.linear() << 0, 1, 0, 0, 0, 1, 1, 0, 0;
      item.target.translation() = Vec3(epsilon, epsilon, folded ? -0.25 : 0.75);
      result.push_back(item);
    }
  }
  for (auto &item : result) {
    if (PrepareWristCircle(item.lengths, item.target, item.current) !=
            PreparationStatus::Ready ||
        residual_baseline::Prepare(item.lengths, item.target, item.baseline) !=
            PreparationStatus::Ready) {
      throw std::runtime_error("Synthetic component target failed preparation");
    }
  }
  return result;
}

using Operation = double (*)(const Item &);

struct Method {
  const char *name;
  Operation operation;
};

const std::array<Method, 10> kMethods{{
    {"loop", [](const Item &item) { return item.angle; }},
    {"legacy_prepare",
     [](const Item &item) {
       residual_baseline::Circle circle;
       const auto status =
           residual_baseline::Prepare(item.lengths, item.target, circle);
       return circle.p.sum() + circle.v.sum() + circle.w.sum() +
              circle.length_scale + circle.lengths.a + circle.lengths.b +
              circle.lengths.c + circle.lengths.r + static_cast<int>(status);
     }},
    {"candidate_prepare",
     [](const Item &item) {
       WristCircle circle;
       const auto status =
           PrepareWristCircle(item.lengths, item.target, circle);
       const auto &data = circle.residual;
       return circle.p.sum() + circle.v.sum() + circle.w.sum() +
              circle.length_scale + circle.lengths.a + circle.lengths.b +
              circle.lengths.c + circle.lengths.r + data.p_v + data.p_w +
              data.v_z + data.w_z + data.p_squared_plus_r_squared +
              data.height + data.a_squared + data.b_squared +
              data.arm_difference + static_cast<int>(data.unit_basis) +
              static_cast<int>(status);
     }},
    {"legacy_pair",
     [](const Item &item) {
       const auto value = residual_baseline::Pair(item.baseline, item.angle);
       return value.value + value.derivative;
     }},
    {"candidate_pair",
     [](const Item &item) {
       const auto value = EvaluateResidualPair(item.current, item.angle);
       return value.value + value.derivative;
     }},
    {"legacy_derivative",
     [](const Item &item) {
       return residual_baseline::Pair(item.baseline, item.angle).derivative;
     }},
    {"candidate_derivative",
     [](const Item &item) {
       return EvaluateResidualDerivative(item.current, item.angle);
     }},
    {"legacy_cramer",
     [](const Item &item) {
       CramerEvaluation value;
       return residual_baseline::Cramer(item.baseline, item.angle, value)
                  ? value.value + value.derivative
                  : 0;
     }},
    {"candidate_cramer",
     [](const Item &item) {
       CramerEvaluation value;
       return EvaluateCramerElbowResidual(item.current, item.angle, value) ==
                      CramerStatus::Ready
                  ? value.value + value.derivative
                  : 0;
     }},
    {"coefficients",
     [](const Item &item) {
       HalfAnglePolynomial polynomial;
       const auto status =
           BuildHalfAnglePolynomial(item.current, item.angle, polynomial);
       return std::accumulate(polynomial.coefficients.begin(),
                              polynomial.coefficients.end(),
                              polynomial.omitted_point_residual) +
              static_cast<int>(status);
     }},
}};

volatile double checksum = 0;

void WriteInputs(const std::filesystem::path &output,
                 const std::vector<Item> &items) {
  std::ofstream layout(output / "layout.json");
  layout << "{\"legacy_circle_bytes\":" << sizeof(residual_baseline::Circle)
         << ",\"candidate_circle_bytes\":" << sizeof(WristCircle)
         << ",\"candidate_cramer_result_bytes\":" << sizeof(CramerEvaluation)
         << "}\n";
  std::ofstream input(output / "input.csv"), checks(output / "checks.csv");
  input << std::setprecision(17) << "case,model,group,a,b,c,r,angle";
  for (int row = 0; row < 4; ++row) {
    for (int column = 0; column < 4; ++column) {
      input << ",target_" << row << column;
    }
  }
  input << '\n';
  checks << std::setprecision(17)
         << "case,scalar_fallback,legacy_cramer_ready,candidate_cramer_ready,"
            "value_error,derivative_error\n";
  for (std::size_t i = 0; i < items.size(); ++i) {
    const auto &item = items.at(i);
    input << i << ',' << item.model << ',' << item.group << ','
          << item.lengths.a << ',' << item.lengths.b << ',' << item.lengths.c
          << ',' << item.lengths.r << ',' << item.angle;
    for (int row = 0; row < 4; ++row) {
      for (int column = 0; column < 4; ++column) {
        input << ',' << item.target.matrix()(row, column);
      }
    }
    input << '\n';
    const auto legacy = residual_baseline::Pair(item.baseline, item.angle);
    const auto candidate = EvaluateResidualPair(item.current, item.angle);
    CramerEvaluation first, second;
    const bool first_ready =
        residual_baseline::Cramer(item.baseline, item.angle, first);
    const bool second_ready =
        EvaluateCramerElbowResidual(item.current, item.angle, second) ==
        CramerStatus::Ready;
    checks << i << ',' << candidate.used_geometric_fallback << ','
           << first_ready << ',' << second_ready << ','
           << std::abs(candidate.value - legacy.value) << ','
           << std::abs(candidate.derivative - legacy.derivative) << '\n';
    if (first_ready != second_ready) {
      throw std::runtime_error(
          "Cramer chart outcomes differ on component inputs");
    }
  }
  if (!input || !checks || !layout) {
    throw std::runtime_error("Cannot write component inputs/checks");
  }
}
} // namespace

auto main(int argc, char **argv) -> int {
  if (argc != 4) {
    std::cerr << "output-directory repeats rounds\n";
    return 2;
  }
  try {
    const auto output = std::filesystem::path(argv[1]);
    const int repeats = std::stoi(argv[2]), rounds = std::stoi(argv[3]);
    if (repeats < 1 || rounds < 1 || !std::filesystem::is_directory(output)) {
      return 2;
    }
    DWORD_PTR process_mask = 0, system_mask = 0;
    if (!GetProcessAffinityMask(GetCurrentProcess(), &process_mask,
                                &system_mask)) {
      throw std::runtime_error("Cannot read affinity");
    }
    const DWORD_PTR cpu = process_mask & (~process_mask + 1);
    if (!SetThreadAffinityMask(GetCurrentThread(), cpu)) {
      throw std::runtime_error("Cannot pin component benchmark thread");
    }
    std::cerr << "Thread affinity mask " << cpu << '\n';
    const auto items = MakeItems();
    WriteInputs(output, items);
    std::ofstream timings(output / "timings.csv");
    timings << std::setprecision(17) << "case,path,round,ns\n";
    std::vector<std::size_t> order(items.size());
    std::iota(order.begin(), order.end(), 0);
    std::array<std::size_t, kMethods.size()> methods{};
    std::iota(methods.begin(), methods.end(), 0);
    std::mt19937 rng(20261113);
    for (int round = -1; round < rounds; ++round) {
      std::shuffle(order.begin(), order.end(), rng);
      for (const auto index : order) {
        std::shuffle(methods.begin(), methods.end(), rng);
        const auto &item = items.at(index);
        for (const auto method : methods) {
          const auto &operation = kMethods.at(method);
          const int count = round < 0 ? 3 : repeats;
          double consumed = 0;
          const auto start = std::chrono::steady_clock::now();
          for (int repetition = 0; repetition < count; ++repetition) {
            // Keep reads in the measured loop; record loop overhead separately.
            std::atomic_signal_fence(std::memory_order_seq_cst);
            consumed += operation.operation(item);
          }
          const auto end = std::chrono::steady_clock::now();
          checksum += consumed;
          if (round >= 0) {
            const double elapsed =
                std::chrono::duration<double, std::nano>(end - start).count() /
                count;
            timings << index << ',' << operation.name << ',' << round << ','
                    << elapsed << '\n';
          }
        }
      }
    }
    if (!timings) {
      throw std::runtime_error("Cannot write component timings");
    }
    std::cerr << "Cases " << items.size() << ", checksum " << checksum << '\n';
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
  return 0;
}
