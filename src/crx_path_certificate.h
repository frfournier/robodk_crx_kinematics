// SPDX-License-Identifier: Apache-2.0
// SPDX-FileCopyrightText: 2026 Centre Technologique en Aérospatiale

#pragma once

#include "crx_ball_certificate.h"
#include "crx_command_certificate.h"

#include <array>
#include <cstdint>

namespace crx::path {
using Interval = interval::Interval;

enum class Failure {
  None,
  InvalidInput,
  UnsupportedEnvironment,
  IdentityMismatch,
  QuaternionDomain,
  TargetEnclosure,
  PointWitness,
  ResidualBound,
  RadiusInequality,
  CommandSlack,
  JunctionMismatch,
  BallInclusion
};

// Raw affine position and homogeneous-quaternion path:
// p(s)=(1-s)*source.target.position+s*last.position,
// q(s)=(1-s)*source.target.quaternion+s*last.quaternion, s in [0,1].
// Rotation is the homogeneous action of q(s), never a matrix interpolation or
// silently sign-adjusted slerp. Each certified interval proves q(s) nonzero.
struct Input {
  exact::Input source;
  exact::PoseInput last;
  exact::FrameConvention convention = exact::FrameConvention::BasePose;
};

inline constexpr std::size_t kIdentityWords = exact::kIdentityWords + 16;

struct Identity {
  std::uint32_t schema = 1;
  std::uint32_t frame_convention = 0;
  std::array<std::uint64_t, kIdentityWords> words{};

  auto operator==(const Identity &other) const -> bool {
    return schema == other.schema &&
           frame_convention == other.frame_convention && words == other.words;
  }

  auto operator!=(const Identity &other) const -> bool {
    return !(*this == other);
  }
};

struct PreparedData {
  Input source;
  Identity identity;
  exact::Problem first_problem;
  double quaternion_scale = 0;
};

class Prepared {
public:
  auto IsReady() const -> bool { return ready_; }

  auto Values() const -> const PreparedData & { return data_; }

private:
  friend auto Prepare(const Input &, Prepared &) -> Failure;
  PreparedData data_;
  bool ready_ = false;
};

// Ready means valid finite source provenance, not a globally nonzero path
// quaternion, root, admissibility or path-completion certificate.
auto Prepare(const Input &input, Prepared &output) -> Failure;

struct Enclosure {
  exact::Pose target;
  Interval quaternion_norm_squared;
};

// Encloses EVERY parameter in a closed subinterval of [0,1]. All failure
// outputs are atomic. A failed denominator/range check is inconclusive, not IK
// exclusion.
auto Enclose(const Prepared &path, Interval parameter, Enclosure &output)
    -> Failure;

struct CellData {
  Identity path_identity;
  command::Profile profile = command::Profile::Callback;
  std::array<std::uint64_t, 12> limit_words{};
  Interval parameter;
  Interval quaternion_norm_squared;
  ball::Certificate point_witness;
  exact::Pose target_enclosure;
  ball::ResidualBounds residual;
  double existence_radius = 0;
  std::array<double, 6> command_slack_lower{};
  std::array<double, 6> slack_lipschitz{};
  bool commands_admitted = false;
};

struct Attempt;

class CellCertificate {
public:
  auto HasLocalLift() const -> bool { return valid_; }

  auto IsAdmitted() const -> bool { return valid_ && data_.commands_admitted; }

  auto Values() const -> const CellData & { return data_; }

private:
  friend auto Certify(const Prepared &, Interval, const exact::Problem &,
                      const ball::Certificate &, const command::Context &)
      -> Attempt;
  CellData data_;
  bool valid_ = false;
};

struct Attempt {
  Failure failure = Failure::InvalidInput;
  ball::Failure point_failure = ball::Failure::None;
  CellCertificate certificate;

  auto IsCertified() const -> bool {
    return failure == Failure::None && certificate.IsAdmitted();
  }
};

// The sealed point witness supplies the fixed centre, verified inverse and
// rebased regional L/R bounds. Uniform target residuals, radius inequalities
// and actual command slacks are checked anew. Callback commands have slack
// constant1; the decoupled third user coordinate has row-sum constant2.
// This certifies one continuous lift, never all branches or global
// infeasibility.
auto Certify(const Prepared &path, Interval parameter,
             const exact::Problem &point_problem,
             const ball::Certificate &point_witness,
             const command::Context &limits) -> Attempt;

struct LinkData {
  Identity path_identity;
  Interval first_parameter, second_parameter;
  Interval centre_distance;
  Interval inclusion_bound;
  double next_uniqueness_radius = 0;
};

struct LinkAttempt;

class LinkCertificate {
public:
  auto IsValid() const -> bool { return valid_; }

  auto Values() const -> const LinkData & { return data_; }

private:
  friend auto Glue(const CellCertificate &, const CellCertificate &)
      -> LinkAttempt;
  LinkData data_;
  bool valid_ = false;
};

struct LinkAttempt {
  Failure failure = Failure::InvalidInput;
  LinkCertificate certificate;

  auto IsCertified() const -> bool {
    return failure == Failure::None && certificate.IsValid();
  }
};

// Requires contiguous cells of one exact path and identical raw limits/profile.
// ||c_first-c_next||_infinity+rho_first <= R_next glues their roots at the
// junction by uniqueness. Mere overlap of the two existence balls is
// insufficient.
auto Glue(const CellCertificate &first, const CellCertificate &second)
    -> LinkAttempt;
} // namespace crx::path
