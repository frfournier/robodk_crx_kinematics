// SPDX-License-Identifier: Apache-2.0
// SPDX-FileCopyrightText: 2026 Centre Technologique en Aérospatiale

#include "crx_root_certificate.h"

#include <array>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <limits>

#include <xmmintrin.h>

namespace {
using crx::interval::Interval;
using crx::root::Coefficients;
using crx::root::Kind;
using crx::root::Options;
using crx::root::Status;

void Require(bool value, const char *message) {
  if (!value) {
    std::cerr << "Root certificate test failure: " << message << '\n';
    std::exit(1);
  }
}

void Print(Interval value) {
  if (!value.IsValid())
    std::cout << "null";
  else
    std::cout << '[' << value.lo << ',' << value.hi << ']';
}

template <std::size_t N> void Print(const std::array<Interval, N> &values) {
  std::cout << '[';
  for (std::size_t i = 0; i < N; ++i) {
    if (i)
      std::cout << ',';
    Print(values.at(i));
  }
  std::cout << ']';
}

void Print(const crx::root::Partition &partition) {
  std::cout << "{\"status\":" << static_cast<int>(partition.status)
            << ",\"nodes\":" << partition.nodes
            << ",\"depth\":" << partition.max_depth
            << ",\"roots\":" << partition.roots
            << ",\"exclusions\":" << partition.exclusions
            << ",\"unresolved\":" << partition.unresolved << ",\"leaves\":[";
  for (std::size_t i = 0; i < partition.leaves.size(); ++i) {
    if (i)
      std::cout << ',';
    const auto &leaf = partition.leaves.at(i);
    std::cout << "{\"kind\":" << static_cast<int>(leaf.kind)
              << ",\"reason\":" << static_cast<int>(leaf.reason)
              << ",\"depth\":" << leaf.depth << ",\"t\":";
    Print(leaf.tangent);
    std::cout << ",\"phase\":";
    Print(leaf.phase);
    std::cout << ",\"endpoints\":";
    Print(leaf.endpoint_values);
    std::cout << ",\"p\":";
    Print(leaf.residual_bernstein);
    std::cout << ",\"d\":";
    Print(leaf.derivative_bernstein);
    std::cout << ",\"q\":";
    Print(leaf.determinant_bernstein);
    std::cout << ",\"derivative_sign\":" << leaf.derivative_sign
              << ",\"determinant_sign\":" << leaf.determinant_sign << '}';
  }
  std::cout << "]}";
}

void Print(const crx::root::Certificate &certificate) {
  const auto &data = certificate.Values();
  std::cout << "{\"complete\":" << (certificate.IsComplete() ? "true" : "false")
            << ",\"status\":" << static_cast<int>(data.status)
            << ",\"entire_circle_unresolved\":"
            << (data.entire_circle_unresolved ? "true" : "false")
            << ",\"shift\":" << data.chart_shift
            << ",\"attempts\":" << data.attempts << ",\"nodes\":" << data.nodes
            << ",\"roots\":" << data.simple_roots
            << ",\"schema\":" << data.identity.schema << ",\"words\":[";
  for (std::size_t i = 0; i < data.identity.words.size(); ++i) {
    if (i)
      std::cout << ',';
    std::cout << '"' << std::hex << data.identity.words.at(i) << std::dec
              << '"';
  }
  std::cout << "],\"polynomials\":[";
  for (std::size_t chart = 0; chart < 2; ++chart) {
    if (chart)
      std::cout << ',';
    const auto &polynomial = data.polynomials.at(chart);
    std::cout << "{\"p\":";
    Print(polynomial.residual);
    std::cout << ",\"d\":";
    Print(polynomial.derivative);
    std::cout << ",\"q\":";
    Print(polynomial.determinant);
    std::cout << ",\"omitted\":";
    Print(std::array<Interval, 2>{polynomial.omitted_residual,
                                  polynomial.omitted_determinant});
    std::cout << '}';
  }
  std::cout << "],\"partitions\":[";
  Print(data.partitions.front());
  std::cout << ',';
  Print(data.partitions.back());
  std::cout << "]}";
}

auto Polynomial(std::initializer_list<double> values) -> Coefficients {
  Coefficients result;
  result.fill(Interval{0});
  std::size_t i = 0;
  for (const auto value : values)
    result.at(i++) = Interval{value};
  return result;
}

auto Witness() -> crx::exact::Input {
  crx::exact::Input input;
  input.lengths = {1, .6, .2, .5};
  input.target.quaternion = {60, -7, 7, -19};
  input.target.position = {27.0 / 64, 15.0 / 32, 55.0 / 128};
  return input;
}

void CheckContracts() {
  std::array<Interval, 5> q;
  q.fill(Interval{0});
  q.front() = Interval{1};
  const auto simple = crx::root::PartitionPolynomial(Polynomial({0, 1}), q);
  Require(simple.status == Status::Complete && simple.roots == 1,
          "simple dyadic root");
  const auto three =
      crx::root::PartitionPolynomial(Polynomial({0, -.25, 0, 1}), q);
  Require(three.status == Status::Complete && three.roots == 3,
          "dyadic subdivision lost a root");
  const auto empty = crx::root::PartitionPolynomial(Polynomial({1}), q);
  Require(empty.status == Status::Complete && empty.roots == 0,
          "constant exclusion/degree drop");
  Options limited;
  limited.max_depth = 10;
  limited.max_nodes = 128;
  const auto repeated = crx::root::PartitionPolynomial(
      Polynomial({.0625, 0, -.5, 0, 1}), q, limited);
  Require(repeated.status == Status::Inconclusive && repeated.unresolved != 0,
          "multiple roots declared simple");
  const auto zero = crx::root::PartitionPolynomial(Polynomial({0}), q);
  Require(zero.status == Status::Inconclusive && zero.nodes == 1 &&
              zero.roots == 0,
          "zero polynomial excluded");
  q.front() = Interval{0};
  q.at(1) = Interval{1};
  const auto common =
      crx::root::PartitionPolynomial(Polynomial({0, 1}), q, limited);
  Require(common.status == Status::Inconclusive && common.unresolved != 0,
          "common determinant root accepted");
  q.front() = Interval{1};
  q.at(1) = Interval{0};
  limited.max_nodes = 0;
  const auto exhausted =
      crx::root::PartitionPolynomial(Polynomial({0, -.25, 0, 1}), q, limited);
  Require(exhausted.nodes == 0 && exhausted.leaves.size() == 1 &&
              exhausted.status == Status::Inconclusive,
          "work frontier lost");
  auto bad = Polynomial({1});
  bad.front() = Interval{};
  Require(crx::root::PartitionPolynomial(bad, q).status == Status::InvalidInput,
          "invalid coefficient excluded");
  crx::exact::Problem problem;
  Require(crx::exact::Prepare(Witness(), problem) == crx::exact::Status::Ready,
          "witness preparation");
  const auto circle = crx::root::Certify(problem);
  if (!circle.IsComplete()) {
    std::cerr << "witness unresolved nodes=" << circle.Values().nodes
              << " roots=" << circle.Values().simple_roots << '\n';
  }
  Require(circle.IsComplete() && circle.Values().simple_roots == 8 &&
              circle.Matches(problem),
          "eight-root wrist cover");
  Options no_work;
  no_work.max_nodes = 0;
  const auto unresolved = crx::root::Certify(problem, no_work);
  Require(!unresolved.IsComplete() && unresolved.Values().nodes == 0 &&
              unresolved.Values().partitions.front().unresolved == 1 &&
              unresolved.Values().partitions.back().unresolved == 1,
          "circle work budget/frontier");
  const auto control = _mm_getcsr();
  _mm_setcsr(control | 0x8000U);
  const auto unsupported = crx::root::Certify(problem);
  _mm_setcsr(control);
  Require(!unsupported.IsComplete() &&
              unsupported.Values().status == Status::UnsupportedEnvironment,
          "FTZ certificate");
  std::cout << "Root cover contracts and eight-root witness passed\n";
}

template <std::size_t N> auto Read(std::array<double, N> &values) -> bool {
  for (auto &value : values)
    std::cin >> value;
  return static_cast<bool>(std::cin);
}

auto ReadInput(crx::exact::Input &input) -> bool {
  if (!Read(input.lengths) || !(std::cin >> input.base_height) ||
      !Read(input.encoder_zero) || !Read(input.senses) ||
      !Read(input.lower_commands) || !Read(input.upper_commands))
    return false;
  for (auto *pose : {&input.target, &input.base, &input.tool})
    if (!Read(pose->quaternion) || !Read(pose->position) ||
        !Read(pose->input_rotation))
      return false;
  return true;
}

auto Batch() -> int {
  int mode = 0;
  while (std::cin >> mode) {
    Options options;
    if (!(std::cin >> options.max_nodes >> options.max_depth >>
          options.max_leaves >> options.max_chart_attempts))
      return 2;
    if (mode == 0) {
      Coefficients p;
      std::array<Interval, 5> q;
      for (auto &value : p)
        std::cin >> value.lo >> value.hi;
      for (auto &value : q)
        std::cin >> value.lo >> value.hi;
      if (!std::cin)
        return 2;
      Print(crx::root::PartitionPolynomial(p, q, options));
    } else if (mode == 1 || mode == 2) {
      crx::exact::Input input;
      if (!ReadInput(input))
        return 2;
      crx::exact::Problem problem;
      const auto convention = mode == 1
                                  ? crx::exact::FrameConvention::BasePose
                                  : crx::exact::FrameConvention::ReferenceMap;
      if (crx::exact::Prepare(input, convention, problem) !=
          crx::exact::Status::Ready)
        return 2;
      Print(crx::root::Certify(problem, options));
    } else
      return 2;
    std::cout << '\n';
  }
  return std::cin.eof() ? 0 : 2;
}
} // namespace

auto main(int argc, char **argv) -> int {
  std::cout << std::setprecision(17);
  if (argc == 2 && std::strcmp(argv[1], "--batch") == 0)
    return Batch();
  if (argc != 1)
    return 2;
  CheckContracts();
  return 0;
}
