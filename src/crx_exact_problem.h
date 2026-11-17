// SPDX-License-Identifier: Apache-2.0
// SPDX-FileCopyrightText: 2026 Centre Technologique en Aérospatiale

#pragma once

#include "crx_interval_eigen.h"
#include "crx_scalar_kernels.h"
#include "crx_types.h"

#include <array>
#include <cstddef>
#include <cstdint>

namespace crx::exact {
using Interval = interval::Interval;
using Vector3 = scalar::Vector3<Interval>;
using Matrix3 = scalar::Matrix3<Interval>;
using Pose = scalar::Pose<Interval>;

enum class Status {
  Ready,
  InvalidInput,
  UnsupportedNominalModel,
  UnsupportedEnvironment,
  ArithmeticFailure
};

struct PoseInput {
  // Exact represented components, w,x,y,z. No rounded normalization is used
  // to define the rotation: its homogeneous action is divided by |q|^2.
  std::array<double, 4> quaternion{1, 0, 0, 0};
  std::array<double, 3> position{};
  // Row-major input matrix, used only to report its Frobenius deviation.
  std::array<double, 9> input_rotation{1, 0, 0, 0, 1, 0, 0, 0, 1};
};

struct Input {
  // One nominal orthogonal CRX model, with exact represented physical lengths.
  // Reference twists are exact right angles, not arbitrary calibrated twists.
  std::array<double, 4> lengths{1, 1, 0, 0}; // a,b,c,r
  // BasePose adds this native height after the base pose. ReferenceMap
  // records the native height whose exact opposite base shift cancels it.
  double base_height = 0;
  // Raw DH encoder zeros and unit senses, before canonical mapping. The
  // reference theta is (0,-pi/2,0,0,0,0), interpreted mathematically.
  std::array<double, 6> encoder_zero{};
  std::array<double, 6> senses{1, 1, 1, 1, 1, 1};
  // Existing core's represented radian bounds are retained in identity;
  // preparation/FK does not certify limits or their original degree conversion.
  // A later admissibility context must retain the raw degree-limit provenance.
  std::array<double, 6> lower_commands{};
  std::array<double, 6> upper_commands{};
  PoseInput target;
  PoseInput base; // Forward base pose or forward reference map, per convention.
  PoseInput tool;
};

inline constexpr std::size_t kIdentityWords = 77;

struct Identity {
  std::uint32_t schema = 1;
  // All exact binary64 inputs, including raw zeros, frames and command bounds.
  // This strict input identity may distinguish equivalent quaternion signs or
  // scales; equality never relies on a collision-prone finite hash.
  std::array<std::uint64_t, kIdentityWords> words{};

  auto operator==(const Identity &other) const -> bool {
    return schema == other.schema && words == other.words;
  }

  auto operator!=(const Identity &other) const -> bool {
    return !(*this == other);
  }
};

struct Lengths {
  Interval a, b, c, r;
};

struct PreparedData {
  Identity identity;
  double length_scale = 0;
  Lengths lengths;
  Pose target;     // Canonical frame, translation divided by length_scale.
  Vector3 p, v, w; // Normalized wrist circle of that same exact target.
  std::array<Interval, 3>
      rotation_deviation; // target/base/tool Frobenius norms.
  std::array<Interval, 6> offset_unwrapped;
  std::array<interval::Phase, 6> encoder_phase;
  std::array<double, 6> diagonal{};
};

enum class FrameConvention : std::uint32_t {
  BasePose = 1,
  // Input.base is a forward reference map: F = reference*T*tool^-1.
  // The nominal native base-height translation cancels exactly; base_height
  // remains a raw model identity word. This is a distinct identity schema.
  ReferenceMap = 2
};

class Problem {
public:
  auto IsReady() const -> bool { return ready_; }

  auto Data() const -> const PreparedData & { return data_; }

private:
  friend auto Prepare(const Input &, FrameConvention, Problem &) -> Status;
  PreparedData data_;
  bool ready_ = false;
};

struct Polynomials {
  Identity identity;
  Interval chart;
  std::array<Interval, 9> residual;
  std::array<Interval, 8> derivative;
  std::array<Interval, 5> determinant;
  Interval omitted_residual;
  Interval omitted_determinant;
};

// Ready means finite enclosures were constructed, never a root, feasibility,
// regularity or coverage certificate. All failures leave output unchanged.
auto Prepare(const Input &input, Problem &output) -> Status;
auto Prepare(const Input &input, FrameConvention convention, Problem &output)
    -> Status;

// Adapter for the existing nominal model. It accepts only its explicit
// orthogonal-twist encoding, zero off-pattern DH lengths, and exact unit
// senses. Models admitted only by numerical snapping remain unsupported here.
// Raw encoder zeros are retained without the numerical solver's tolerance
// snapping.
auto Prepare(const CrxModelData &model, const PoseIsoRT &target,
             Problem &output) -> Status;
// Preferred when the numerical problem already extracted target_quaternion:
// use those exact stored components, rather than recomputing another target.
auto Prepare(const CrxModelData &model, const PoseIsoRT &target,
             const Eigen::Quaterniond &target_quaternion, Problem &output)
    -> Status;

// Raw forward reference/TCP inputs, without a rounded flange conversion or
// rounded inverse translation. This adapter requires the native model's base
// pose to be exactly Translate(0,0,-base_height) and its tool to be identity,
// as in the built-in C ABI models. Other model/frame conventions use Prepare.
auto PrepareReference(const CrxModelData &model, const PoseIsoRT &target,
                      const Eigen::Quaterniond &target_quaternion,
                      const PoseIsoRT &reference, const PoseIsoRT &tool,
                      Problem &output) -> Status;

auto BuildPolynomials(const Problem &problem, Interval chart,
                      Polynomials &output) -> Status;

// FK in the same normalized canonical frame as Problem::target. No limit or
// root claim follows from enclosing FK. Command mapping retains raw-zero
// enclosures; principal seam uncertainty never chooses an arbitrary side.
auto ForwardCanonical(const Problem &problem, const std::array<Interval, 6> &q,
                      Pose &output) -> Status;
auto ForwardCommands(const Problem &problem,
                     const std::array<Interval, 6> &command, Pose &output)
    -> Status;
} // namespace crx::exact
