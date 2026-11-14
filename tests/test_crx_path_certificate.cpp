// SPDX-License-Identifier: Apache-2.0
// SPDX-FileCopyrightText: 2026 Centre Technologique en Aérospatiale

#include "crx_ball_certificate.h"
#include "crx_command_certificate.h"
#include "crx_exact_problem.h"
#include "crx_interval.h"
#include "crx_path_certificate.h"
#include "crx_phase_catalogue.h"

#include <Eigen/Core>
#include <array>
#include <cfenv>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <string>

namespace {
namespace path = crx::path;
using crx::interval::Interval;

void Require(bool condition, const char *message) {
  if (!condition) {
    std::cerr << "Path certificate: " << message << '\n';
    std::exit(EXIT_FAILURE);
  }
}

auto Source() -> crx::exact::Input {
  crx::exact::Input result;
  result.lengths = {1, .6, .2, .5};
  result.target.quaternion = {60, -7, 7, -19};
  result.target.position = {27.0 / 64, 15.0 / 32, 55.0 / 128};
  result.lower_commands.fill(-4);
  result.upper_commands.fill(4);
  return result;
}

auto Read(double &value) -> bool {
  std::string token;
  if (!(std::cin >> token)) {
    return false;
  }
  char *last = nullptr;
  value = std::strtod(token.c_str(), &last);
  return last != token.c_str() && *last == '\0';
}

template <std::size_t N> auto Read(std::array<double, N> &values) -> bool {
  for (auto &value : values) {
    if (!Read(value)) {
      return false;
    }
  }
  return true;
}

auto Read(crx::exact::PoseInput &pose) -> bool {
  return Read(pose.quaternion) && Read(pose.position) &&
         Read(pose.input_rotation);
}

auto Read(crx::exact::Input &input) -> bool {
  return Read(input.lengths) && Read(input.base_height) &&
         Read(input.encoder_zero) && Read(input.senses) &&
         Read(input.lower_commands) && Read(input.upper_commands) &&
         Read(input.target) && Read(input.base) && Read(input.tool);
}

void Print(double value) {
  if (std::isfinite(value)) {
    std::cout << value;
  } else {
    std::cout << "null";
  }
}

void Print(Interval value) {
  std::cout << '[';
  Print(value.lo);
  std::cout << ',';
  Print(value.hi);
  std::cout << ']';
}

template <typename T, std::size_t N>
void Print(const std::array<T, N> &values) {
  std::cout << '[';
  for (std::size_t i = 0; i < N; ++i) {
    if (i) {
      std::cout << ',';
    }
    Print(values.at(i));
  }
  std::cout << ']';
}

template <typename Derived>
void PrintMatrix(const Eigen::MatrixBase<Derived> &matrix) {
  std::cout << '[';
  for (Eigen::Index r = 0; r < matrix.rows(); ++r) {
    for (Eigen::Index c = 0; c < matrix.cols(); ++c) {
      if (r || c) {
        std::cout << ',';
      }
      Print(matrix(r, c));
    }
  }
  std::cout << ']';
}

void PrintBall(const crx::exact::Problem &problem,
               const crx::ball::Attempt &point) {
  std::cout << "{\"failure\":" << static_cast<int>(point.failure);
  if (point.IsCertified()) {
    const auto &b = point.certificate.Values();
    std::array<double, crx::exact::kIdentityWords> input{};
    for (std::size_t i = 0; i < input.size(); ++i) {
      const auto bits = problem.Data().identity.words.at(i);
      std::memcpy(&input.at(i), &bits, sizeof(bits));
    }
    std::cout << ",\"input\":";
    Print(input);
    std::cout << ",\"schema\":" << problem.Data().identity.schema
              << ",\"centre\":";
    Print(b.centre);
    std::cout << ",\"constants\":";
    Print(std::array<double, 11>{
        b.existence_radius, b.uniqueness_radius, b.inverse_residual, b.beta,
        b.lipschitz, b.eta, b.pose_defect, b.screw_norm_sum, b.angular_norm_sum,
        b.anchor_position_norm, static_cast<double>(b.aspect_sign)});
    std::cout << ",\"preconditioner\":";
    PrintMatrix(b.preconditioner);
    std::cout << ",\"coarse_eta\":";
    Print(b.coarse_eta);
    std::cout << ",\"used_log_series\":" << b.used_log_series << ",\"trace\":";
    Print(b.target_relative_trace);
  }
  std::cout << '}';
}

void Probe(const path::Input &input, const crx::exact::PoseInput &point_target,
           const crx::ball::Centre &centre, Interval parameter,
           const crx::command::Input &raw, crx::ball::Request request = {}) {
  path::Prepared prepared;
  const auto ready = path::Prepare(input, prepared);
  path::Enclosure enclosed;
  const auto enclosure = path::Enclose(prepared, parameter, enclosed);
  auto point_source = input.source;
  point_source.target = point_target;
  crx::exact::Problem problem;
  const auto point_ready =
      crx::exact::Prepare(point_source, input.convention, problem);
  const auto point = crx::ball::Certify(problem, centre, request);
  crx::command::Context limits;
  const auto context = crx::command::Prepare(problem, raw, limits);
  const auto attempt =
      path::Certify(prepared, parameter, problem, point.certificate, limits);
  std::cout << "{\"preparation_failure\":" << static_cast<int>(ready)
            << ",\"enclosure_failure\":" << static_cast<int>(enclosure)
            << ",\"point_preparation\":" << static_cast<int>(point_ready)
            << ",\"context_failure\":" << static_cast<int>(context)
            << ",\"failure\":" << static_cast<int>(attempt.failure)
            << ",\"point_failure\":" << static_cast<int>(attempt.point_failure)
            << ",\"local\":" << attempt.certificate.HasLocalLift()
            << ",\"admitted\":" << attempt.certificate.IsAdmitted();
  std::cout << ",\"point\":";
  PrintBall(problem, point);
  if (enclosure == path::Failure::None) {
    std::cout << ",\"rotation\":";
    PrintMatrix(enclosed.target.rotation);
    std::cout << ",\"translation\":";
    PrintMatrix(enclosed.target.translation);
    std::cout << ",\"quaternion_norm\":";
    Print(enclosed.quaternion_norm_squared);
  }
  if (attempt.certificate.HasLocalLift()) {
    const auto &data = attempt.certificate.Values();
    std::cout << ",\"rho\":";
    Print(data.existence_radius);
    std::cout << ",\"eta\":";
    Print(data.residual.eta);
    std::cout << ",\"coarse_eta\":";
    Print(data.residual.coarse_eta);
    std::cout << ",\"defect\":";
    Print(data.residual.pose_defect);
    std::cout << ",\"used_log_series\":" << data.residual.used_log_series
              << ",\"trace\":";
    Print(data.residual.relative_trace);
    std::cout << ",\"slacks\":";
    Print(data.command_slack_lower);
    std::cout << ",\"slack_lipschitz\":";
    Print(data.slack_lipschitz);
  }
  std::cout << "}\n";
}

auto Batch() -> int {
  int convention = 0;
  while (std::cin >> convention) {
    path::Input input;
    input.convention = static_cast<crx::exact::FrameConvention>(convention);
    crx::exact::PoseInput point;
    crx::ball::Centre centre;
    double lo = 0, hi = 0, radius = 0;
    int profile = 0, use_inverse = 0;
    crx::command::Input limits;
    std::array<double, 36> raw_inverse{};
    if (!Read(input.source) || !Read(input.last) || !Read(point) ||
        !Read(centre) || !Read(lo) || !Read(hi) || !Read(radius) ||
        !Read(limits.lower_degrees) || !Read(limits.upper_degrees) ||
        !(std::cin >> profile >> use_inverse) || !Read(raw_inverse)) {
      return 2;
    }
    limits.profile = static_cast<crx::command::Profile>(profile);
    crx::ball::PointMatrix6 inverse;
    for (Eigen::Index r = 0; r < 6; ++r) {
      for (Eigen::Index c = 0; c < 6; ++c) {
        inverse(r, c) = raw_inverse.at(static_cast<std::size_t>(r * 6 + c));
      }
    }
    Probe(input, point, centre, Interval{lo, hi}, limits,
          {radius, (use_inverse != 0) ? &inverse : nullptr, 0});
  }
  return std::cin.eof() ? 0 : 2;
}

void Witnesses() {
  path::Input input{Source(), Source().target};
  path::Prepared prepared;
  Require(path::Prepare(input, prepared) == path::Failure::None,
          "witness path");
  const auto built = crx::catalogue::Build(prepared.Values().first_problem);
  Require(built.catalogue.IsComplete(), "witness cover");
  crx::command::Input limits;
  limits.lower_degrees.fill(-720);
  limits.upper_degrees.fill(720);
  for (const auto &ball : built.catalogue.Values().balls) {
    Probe(input, input.source.target, ball.Values().centre, Interval{0, 1},
          limits);
  }
}

void Contracts() {
  path::Input input{Source(), Source().target};
  path::Prepared prepared;
  Require(path::Prepare(input, prepared) == path::Failure::None, "prepare");
  const auto &problem = prepared.Values().first_problem;
  const auto built = crx::catalogue::Build(problem);
  Require(built.catalogue.IsComplete() &&
              built.catalogue.Values().balls.size() == 16,
          "complete point witness fixture");
  crx::command::Input raw;
  raw.lower_degrees.fill(-720);
  raw.upper_degrees.fill(720);
  crx::command::Context limits;
  Require(crx::command::Prepare(problem, raw, limits) ==
              crx::command::Failure::None,
          "limits");
  path::Enclosure enclosure;
  Require(path::Enclose(prepared, Interval{0, 1}, enclosure) ==
                  path::Failure::None &&
              enclosure.quaternion_norm_squared.lo > 0,
          "constant path enclosure");
  for (const auto &witness : built.catalogue.Values().balls) {
    const auto whole =
        path::Certify(prepared, Interval{0, 1}, problem, witness, limits);
    Require(whole.IsCertified(), "constant admitted lift");
    const auto first =
        path::Certify(prepared, Interval{0, .5}, problem, witness, limits);
    const auto second =
        path::Certify(prepared, Interval{.5, 1}, problem, witness, limits);
    Require(first.IsCertified() && second.IsCertified() &&
                path::Glue(first.certificate, second.certificate).IsCertified(),
            "junction containment");
    const auto gap =
        path::Certify(prepared, Interval{.75, 1}, problem, witness, limits);
    Require(gap.IsCertified() &&
                path::Glue(first.certificate, gap.certificate).failure ==
                    path::Failure::JunctionMismatch,
            "parameter gap accepted");
    auto narrow = raw;
    narrow.lower_degrees.fill(-.001);
    narrow.upper_degrees.fill(.001);
    crx::command::Context restricted;
    Require(crx::command::Prepare(problem, narrow, restricted) ==
                crx::command::Failure::None,
            "narrow limits");
    const auto outside =
        path::Certify(prepared, Interval{0, 1}, problem, witness, restricted);
    Require(outside.failure == path::Failure::CommandSlack &&
                outside.certificate.HasLocalLift() &&
                !outside.certificate.IsAdmitted(),
            "local lift confused with command admission");
    auto decoupled = raw;
    decoupled.profile = crx::command::Profile::DecoupledUser;
    crx::command::Context user;
    Require(crx::command::Prepare(problem, decoupled, user) ==
                crx::command::Failure::None,
            "user limits");
    const auto user_cell =
        path::Certify(prepared, Interval{0, 1}, problem, witness, user);
    Require(user_cell.IsCertified() &&
                user_cell.certificate.Values().slack_lipschitz.at(2) == 2 &&
                user_cell.certificate.Values().slack_lipschitz.at(1) == 1,
            "decoupled slack constants");
  }
  const auto &witness = built.catalogue.Values().balls.front();
  auto moved = input;
  moved.last.position.at(0) += 1e-12;
  path::Prepared small;
  Require(path::Prepare(moved, small) == path::Failure::None &&
              path::Certify(small, Interval{0, 1}, problem, witness, limits)
                  .IsCertified(),
          "small affine target lift");
  auto reversed = moved;
  reversed.source.target = moved.last;
  reversed.last = input.source.target;
  path::Prepared backward;
  Require(path::Prepare(reversed, backward) == path::Failure::None &&
              path::Certify(backward, Interval{0, 1}, problem, witness, limits)
                  .IsCertified(),
          "reversed target lift");
  moved.last.position.at(0) += .01;
  path::Prepared broad;
  Require(path::Prepare(moved, broad) == path::Failure::None &&
              !path::Certify(broad, Interval{0, 1}, problem, witness, limits)
                   .IsCertified(),
          "broad target lift incorrectly certified");
  auto vanished = input;
  for (auto &value : vanished.last.quaternion) {
    value = -value;
  }
  path::Prepared pole;
  Require(path::Prepare(vanished, pole) == path::Failure::None &&
              path::Enclose(pole, Interval{0, 1}, enclosure) ==
                  path::Failure::QuaternionDomain,
          "zero quaternion parameter accepted");
  const auto retained = prepared.Values().identity;
  auto invalid = input;
  invalid.last.quaternion.fill(0);
  Require(path::Prepare(invalid, prepared) == path::Failure::InvalidInput &&
              prepared.Values().identity == retained,
          "failed preparation changed output");
  const auto translation = enclosure.target.translation.x();
  Require(path::Enclose(prepared, Interval{-.1, 1}, enclosure) ==
                  path::Failure::InvalidInput &&
              enclosure.target.translation.x().lo == translation.lo &&
              enclosure.target.translation.x().hi == translation.hi,
          "failed enclosure changed output");
  const auto &other = built.catalogue.Values().balls.at(1);
  const auto first =
      path::Certify(prepared, Interval{0, .5}, problem, witness, limits);
  const auto second =
      path::Certify(prepared, Interval{.5, 1}, problem, other, limits);
  Require(first.IsCertified() && second.IsCertified() &&
              path::Glue(first.certificate, second.certificate).failure ==
                  path::Failure::BallInclusion,
          "distinct constant roots linked");
  auto changed = input;
  changed.source.lengths.at(0) += .01;
  path::Prepared different;
  Require(path::Prepare(changed, different) == path::Failure::None &&
              path::Certify(different, Interval{0, 1}, problem, witness, limits)
                      .failure == path::Failure::IdentityMismatch,
          "stale model witness accepted");
  const int rounding = std::fegetround();
  Require(std::fesetround(FE_UPWARD) == 0, "directed mode setup");
  const auto unsupported = path::Enclose(prepared, Interval{0, 1}, enclosure);
  const auto unsupported_link =
      path::Glue(first.certificate, second.certificate);
  Require(std::fesetround(rounding) == 0, "rounding restore");
  Require(unsupported == path::Failure::UnsupportedEnvironment &&
              unsupported_link.failure == path::Failure::UnsupportedEnvironment,
          "directed mode accepted");
}
} // namespace

auto main(int argc, char **argv) -> int {
  std::cout << std::setprecision(17);
  if (argc == 2 && std::string(argv[1]) == "--batch") {
    return Batch();
  }
  if (argc == 2 && std::string(argv[1]) == "--witness") {
    Witnesses();
    return 0;
  }
  if (argc != 1) {
    return 2;
  }
  Contracts();
  return 0;
}
