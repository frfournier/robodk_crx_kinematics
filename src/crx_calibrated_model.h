// SPDX-License-Identifier: Apache-2.0
// SPDX-FileCopyrightText: 2026 Centre Technologique en Aérospatiale

#pragma once

#include "crx_ball_certificate.h"

#include <array>
#include <cstddef>
#include <cstdint>

namespace crx::calibrated {
using Interval = interval::Interval;
using Pose = exact::Pose;
using Vector3 = exact::Vector3;
using Centre = ball::Centre;

enum class Status {
  Ready,
  InvalidInput,
  UnsupportedEnvironment,
  ArithmeticFailure
};

// Mathematical angle = quarters*pi/2 + radians. Right angles are declared,
// never recognized by tolerance. The correction is the exact binary64 input.
struct Angle {
  std::int32_t quarters = 0;
  double radians = 0;
};

// Modified DH: Rx(alpha) Tx(a) Rz(theta + x) Tz(d), six revolute joints.
// x_j = senses_j * u_j; u is already a coupled RoboDK command in radians.
// A decoupled user-coordinate adapter must apply coupling before this boundary.
struct DhRow {
  Angle alpha;
  double a = 0;
  Angle theta;
  double d = 0;
};

struct Frame {
  // Homogeneous quaternion rotation, w,x,y,z, divided by its exact squared
  // norm. Nonzero finite quaternions need not be normalized. Positions, DH
  // lengths and length_scale all use the same declared length unit.
  std::array<double, 4> quaternion{1, 0, 0, 0};
  std::array<double, 3> position{};
};

enum class LimitUnits : std::uint32_t { Radians = 0, Degrees = 1 };

struct Input {
  std::array<DhRow, 6> rows;
  std::array<double, 6> senses{1, 1, 1, 1, 1, 1};
  Frame base;
  Frame tool;
  // Raw declared bounds in limit_units; commands/increments themselves use
  // radians throughout the core. Degree provenance is retained exactly.
  std::array<double, 6> lower_commands{};
  std::array<double, 6> upper_commands{};
  double length_scale = 1;
  LimitUnits limit_units = LimitUnits::Radians;
};

inline constexpr std::size_t kModelIdentityWords = 70;

struct Identity {
  std::uint32_t schema =
      2; // Separate type/domain from nominal exact::Identity.
  std::array<std::uint64_t, kModelIdentityWords> words{};

  auto operator==(const Identity &other) const -> bool {
    return schema == other.schema && words == other.words;
  }

  auto operator!=(const Identity &other) const -> bool {
    return !(*this == other);
  }
};

struct PreparedData {
  Identity identity;
  Input source;
  Pose base;
  Pose tool;
  std::array<Pose, 6> fixed;
  std::array<interval::SinCos, 6> theta;
  // Space screws (origin cross axis; axis), in normalized base/world units.
  // Their unit axes follow structurally from the DH rotations; no rounded
  // axis normalization defines the exact calibrated geometry.
  std::array<Vector3, 6> axes;
  std::array<Vector3, 6> origins;
  Pose home;
  // Radian enclosures of raw radians or raw degrees*pi/180. The original
  // values and unit tag remain in source/identity.
  std::array<Interval, 6> lower_commands;
  std::array<Interval, 6> upper_commands;
};

class Model {
public:
  auto IsReady() const -> bool { return ready_; }

  auto Data() const -> const PreparedData & { return data_; }

private:
  friend auto Prepare(const Input &, Model &) -> Status;
  PreparedData data_;
  bool ready_ = false;
};

// Preparation/FK are enclosures, never feasibility, local regularity or
// completeness certificates. All failures leave the output unchanged.
auto Prepare(const Input &input, Model &output) -> Status;
auto EncloseFrame(const Frame &input, double length_scale, Pose &output)
    -> Status;

// Independent DH and PoE expressions for the same declared model. Both take
// physical joint increments x, before applying senses/command limits. Neither
// expression uses the nominal orthogonal signature or nominal symmetries.
auto ForwardDh(const Model &model, const std::array<Interval, 6> &increments,
               Pose &output) -> Status;
auto ForwardPoe(const Model &model, const std::array<Interval, 6> &increments,
                Pose &output) -> Status;

// Rebased unit screws and body Jacobian at a represented increment centre.
// closed_form_determinant is unavailable: nominal aspect formulas do not
// apply to arbitrary calibrated DH geometry.
auto EvaluateCentre(const Model &model, const Centre &centre,
                    ball::Kinematics &output) -> Status;
} // namespace crx::calibrated
