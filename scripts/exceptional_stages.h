// SPDX-License-Identifier: Apache-2.0
// SPDX-FileCopyrightText: 2026 Centre Technologique en Aérospatiale
#pragma once

#include "crx_discovery.h"
#include "crx_phase_catalogue.h"

namespace crx::benchmark {
struct ExceptionalContext {
  exact::Problem problem;
  canonical::WristCircle circle;
  PoseIsoRT target = PoseIsoRT::Identity();
  bool ready = false;
};

struct ExceptionalScratch {
  catalogue::BuildResult result;
  std::vector<ball::Certificate> balls;
};

auto PrepareExceptional(const exact::Problem &problem) -> ExceptionalContext;
auto RecoverExceptional(const ExceptionalContext &context,
                        const root::Certificate &roots,
                        catalogue::Options options, ExceptionalScratch &scratch)
    -> bool;
} // namespace crx::benchmark
