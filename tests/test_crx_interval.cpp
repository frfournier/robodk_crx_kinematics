// SPDX-License-Identifier: Apache-2.0
// SPDX-FileCopyrightText: 2026 Centre Technologique en Aérospatiale

#include "crx_interval.h"
#include "crx_interval_matrix.h"

#include <cfenv>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <limits>
#include <string>

#include <xmmintrin.h>

namespace {
using crx::interval::Interval;

void Require(bool condition, const char *message) {
  if (!condition) {
    std::fprintf(stderr, "Interval test failure: %s\n", message);
    std::exit(EXIT_FAILURE);
  }
}

auto Contains(Interval value, double point) -> bool {
  return value.IsValid() && value.lo <= point && point <= value.hi;
}

void CheckEnvironment() {
  using crx::interval::EnvironmentSupported;
  Require(EnvironmentSupported(), "default floating environment unsupported");
  const auto original = _mm_getcsr();
  for (const unsigned bit : {0x0040U, 0x8000U, 0x2000U, 0x4000U}) {
    _mm_setcsr(original | bit);
    const bool rejected =
        !EnvironmentSupported() && !(Interval{1.0} + Interval{2.0}).IsValid() &&
        !crx::interval::EvaluateSinCos(Interval{1.0}).sine.IsValid();
    _mm_setcsr(original);
    Require(rejected, "unsafe MXCSR accepted");
  }
  const int previous = std::fegetround();
  for (const int rounding : {FE_DOWNWARD, FE_UPWARD, FE_TOWARDZERO}) {
    Require(std::fesetround(rounding) == 0, "rounding mode setup failed");
    const bool rejected =
        !EnvironmentSupported() && !(Interval{1.0} * Interval{2.0}).IsValid();
    const int restored = std::fesetround(previous);
    Require(restored == 0 && rejected, "non-nearest arithmetic accepted");
  }
  Require(_mm_getcsr() == original, "arithmetic changed caller FP controls");
}

void CheckExceptionalArithmetic() {
  const double tiny = std::numeric_limits<double>::denorm_min();
  Require(Contains(Interval{tiny} * Interval{0.5}, 0) &&
              (Interval{tiny} * Interval{0.5}).hi >= tiny,
          "subnormal product not enclosed");
  Require(!(Interval{1.0} / Interval{-tiny, tiny}).IsValid(),
          "zero-containing division accepted");
  Require(!crx::interval::Sqrt({-tiny, 1}).IsValid(),
          "negative sqrt domain silently clipped");
  const double large = std::numeric_limits<double>::max();
  Require(!(Interval{large} + Interval{large}).IsValid(), "overflow accepted");
  Require(!(Interval{} + Interval{0.0}).IsValid(), "invalid input erased");
  Require(Contains(crx::interval::Sqrt({0, tiny}), 0),
          "sqrt zero endpoint lost");
  const auto square = crx::interval::Square({-2, 3});
  Require(square.lo == 0 && square.hi >= 9, "square range incorrect");
  // Exact product is 1-2^-54. A contracted/reassociated computation must not
  // move this value outside the two explicit primitive enclosures.
  const auto cancellation =
      (Interval{1 + 0x1p-27} * Interval{1 - 0x1p-27}) - Interval{1.0};
  Require(Contains(cancellation, -0x1p-54), "product rounding boundary erased");
}

void CheckTrigonometry() {
  const auto zero = crx::interval::EvaluateSinCos(Interval{-0.0});
  Require(zero.sine.IsPoint() && zero.sine.lo == 0 && zero.cosine.IsPoint() &&
              zero.cosine.lo == 1,
          "zero trigonometry");
  const auto huge = crx::interval::EnclosePhase(Interval{1e20});
  Require(huge.trigonometry.used_full_range && huge.principal_seam_uncertain &&
              huge.trigonometry.sine.lo == -1 && huge.trigonometry.sine.hi == 1,
          "large argument claimed a narrow reduction");
  const auto seam = crx::interval::EnclosePhase(crx::interval::Pi());
  Require(seam.principal_seam_uncertain && seam.principal.lo < 0 &&
              seam.principal.hi > 0,
          "principal seam chose a side");
  const auto ordinary = crx::interval::EnclosePhase(Interval{7.0});
  Require(!ordinary.principal_seam_uncertain && ordinary.principal.IsValid(),
          "ordinary phase reduction failed");
}

void PrintInterval(Interval value) {
  if (value.IsValid()) {
    std::printf("[%.17g,%.17g]", value.lo, value.hi);
  } else {
    std::printf("null");
  }
}

auto ReadDouble(double &output) -> bool {
  std::string token;
  if (!(std::cin >> token)) {
    return false;
  }
  char *end = nullptr;
  output = std::strtod(token.c_str(), &end);
  return end != token.c_str() && *end == '\0';
}

auto ReadInterval(Interval &output) -> bool {
  return ReadDouble(output.lo) && ReadDouble(output.hi);
}

auto MatrixProbe() -> bool {
  crx::interval::Matrix<2, 2> a, b;
  for (auto *matrix : {&a, &b}) {
    for (auto &value : matrix->values) {
      if (!ReadInterval(value)) {
        return false;
      }
    }
  }
  const auto product = crx::interval::Multiply(a, b);
  std::printf("{\"values\":[");
  for (std::size_t i = 0; i < product.values.size(); ++i) {
    if (i != 0) {
      std::printf(",");
    }
    PrintInterval(product.values.at(i));
  }
  std::printf(",");
  PrintInterval(crx::interval::MaxAbsNorm(a));
  std::printf(",");
  PrintInterval(crx::interval::RowSumNorm(a));
  std::puts("]}");
  return true;
}

auto RunBatch() -> int {
  std::string operation;
  while (std::cin >> operation) {
    if (operation == "matrix") {
      if (!MatrixProbe()) {
        return 2;
      }
      continue;
    }
    if (operation == "norm") {
      crx::interval::Matrix<3, 1> vector;
      for (auto &value : vector.values) {
        if (!ReadInterval(value)) {
          return 2;
        }
      }
      std::printf("{\"values\":[");
      PrintInterval(crx::interval::EuclideanNorm(vector));
      std::puts("]}");
      continue;
    }
    Interval a, b, result;
    if (!ReadInterval(a)) {
      return 2;
    }
    if (operation == "trig" || operation == "phase") {
      const auto phase = crx::interval::EnclosePhase(a);
      std::printf("{\"values\":[");
      PrintInterval(phase.trigonometry.sine);
      std::printf(",");
      PrintInterval(phase.trigonometry.cosine);
      std::printf(",");
      PrintInterval(phase.principal);
      std::printf("],\"full\":%s,\"seam\":%s}\n",
                  phase.trigonometry.used_full_range ? "true" : "false",
                  phase.principal_seam_uncertain ? "true" : "false");
      continue;
    }
    if (operation == "sqrt") {
      result = crx::interval::Sqrt(a);
    } else if (operation == "square") {
      result = crx::interval::Square(a);
    } else if (operation == "abs") {
      result = crx::interval::Abs(a);
    } else {
      if (!ReadInterval(b)) {
        return 2;
      }
      if (operation == "add") {
        result = a + b;
      } else if (operation == "sub") {
        result = a - b;
      } else if (operation == "mul") {
        result = a * b;
      } else if (operation == "div") {
        result = a / b;
      } else {
        return 2;
      }
    }
    std::printf("{\"values\":[");
    PrintInterval(result);
    std::puts("]}");
  }
  return std::cin.eof() ? 0 : 2;
}
} // namespace

auto main(int argc, char **argv) -> int {
  if (argc == 2 && std::strcmp(argv[1], "--batch") == 0) {
    return RunBatch();
  }
  if (argc != 1) {
    return 2;
  }
  CheckEnvironment();
  CheckExceptionalArithmetic();
  CheckTrigonometry();
  std::puts("Interval environment and exceptional arithmetic passed");
  return 0;
}
