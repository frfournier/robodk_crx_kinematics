// SPDX-License-Identifier: Apache-2.0
// SPDX-FileCopyrightText: 2026 Centre Technologique en Aérospatiale

#pragma once

#include "crx_calibrated_model.h"

namespace crx::calibrated {
struct ProblemIdentity {
  Identity model;
  std::array<std::uint64_t, 7> target{}; // Raw homogeneous quaternion/position.

  auto operator==(const ProblemIdentity &other) const -> bool {
    return model == other.model && target == other.target;
  }
};

class Problem {
public:
  auto IsReady() const -> bool { return ready_; }

  auto Geometry() const -> const Model & { return model_; }

  auto Target() const -> const Pose & { return target_; }

  auto SourceTarget() const -> const Frame & { return source_target_; }

  auto Identity() const -> const ProblemIdentity & { return identity_; }

private:
  friend auto PrepareProblem(const Model &, const Frame &, Problem &) -> Status;
  Model model_;
  Pose target_;
  Frame source_target_;
  ProblemIdentity identity_;
  bool ready_ = false;
};

auto PrepareProblem(const Model &model, const Frame &target, Problem &output)
    -> Status;

struct CertificateData {
  ProblemIdentity identity;
  ball::Bounds bounds;
  Centre command_centre{};
  // Lower bound on the minimum distance from the command centre to any raw
  // command limit. May be negative. This is not a heuristic clearance estimate.
  double command_reserve_lower = 0;
  bool commands_admitted = false;
  // Both signs are LOCAL Jacobian signs, never calibrated aspect invariants.
  int command_jacobian_sign = 0;
};

struct Attempt;

class Certificate {
public:
  auto IsValid() const -> bool { return valid_; }

  auto Values() const -> const CertificateData & { return data_; }

  auto Matches(const Problem &problem) const -> bool {
    return valid_ && problem.IsReady() && data_.identity == problem.Identity();
  }

private:
  friend auto Certify(const Problem &, const Centre &, ball::Request)
      -> Attempt;
  CertificateData data_;
  bool valid_ = false;
};

struct Attempt {
  ball::Failure failure = ball::Failure::InvalidInput;
  Certificate certificate;

  auto IsCertified() const -> bool {
    return failure == ball::Failure::None && certificate.IsValid();
  }
};

// Certifies one unique calibrated root in the increment-space infinity ball R,
// with an existence ball rho. commands_admitted separately checks every command
// throughout the existence ball against the exact declared bounds. A failed
// limit check cannot prove infeasibility. This never certifies global coverage,
// calibrated aspects, symmetry partners or nearest selection.
auto Certify(const Problem &problem, const Centre &centre,
             ball::Request request = {}) -> Attempt;
auto TryReuse(const Certificate &previous, const Problem &problem) -> Attempt;
} // namespace crx::calibrated
