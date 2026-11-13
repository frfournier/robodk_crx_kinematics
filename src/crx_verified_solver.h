// SPDX-License-Identifier: Apache-2.0
// SPDX-FileCopyrightText: 2026 Centre Technologique en Aérospatiale
#pragma once

#include "crx_command_certificate.h"
#include "crx_exact_problem.h"
#include "crx_phase_catalogue.h"
#include "crx_solver.h"

namespace crx {
struct VerifiedOptions {
  bool enabled = true;
  catalogue::Options phases;
  command::Options commands;
};

struct VerifiedQuery {
  bool input_valid = false;
  bool attempted = false;
  command::Input raw;
  exact::Status preparation = exact::Status::InvalidInput;
  exact::Problem problem;
  catalogue::BuildResult phases;
  command::Failure context_failure = command::Failure::InvalidInput;
  command::Certificate commands;
  // Numerical fallback in the selected command convention, never a proof.
  IkResult numerical;
};

// Raw frames have the same forward-reference contract as local certificates.
// Exact phase/command results are independent of the numerical fallback. All
// data are owned; no model/frame/input pointers survive this call.
auto QueryVerified(const CrxModelData &model, const PoseIsoRT &target,
                   const PoseIsoRT &reference, const PoseIsoRT &tool,
                   const command::Input &raw, VerifiedOptions options = {})
    -> VerifiedQuery;
} // namespace crx
