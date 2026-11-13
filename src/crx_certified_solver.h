// SPDX-License-Identifier: Apache-2.0
// SPDX-FileCopyrightText: 2026 Centre Technologique en Aérospatiale

#pragma once

#include "crx_ball_certificate.h"
#include "crx_solver.h"

#include <array>
#include <vector>

namespace crx {
struct CertificateOptions {
  bool enabled = true;
  const ball::Certificate *previous = nullptr;
};

struct CertificateAttachment {
  ball::Attempt attempt;
  bool reused = false;
  ball::Failure reuse_failure = ball::Failure::NotRequested;
  double length_scale = 0;
  std::array<double, 3> rotation_deviation_upper{};
};

struct CertifiedIkResult {
  IkResult result;
  std::vector<CertificateAttachment> certificates;
};

struct CertifiedFibreResult {
  FibreResult result;
  std::vector<CertificateAttachment> certificates;
};

// Opt-in paths leave the original numerical functions/layouts unchanged.
// reference maps the requested TCP reference into the native base; tool maps
// flange into TCP. Raw source frames define the exact target, while ordinary
// matrix composition only nominates the numerical flange target. Certificates
// record canonical centres/radian radii, not balls around rounded commands.
auto SolveIkCertified(const CrxModelData &model, const PoseIsoRT &target,
                      const PoseIsoRT &reference, const PoseIsoRT &tool,
                      const Vec6 *seed, int capacity,
                      CertificateOptions options = {}) -> CertifiedIkResult;
auto IdentifyIkCertified(const CrxModelData &model, const PoseIsoRT &target,
                         const PoseIsoRT &reference, const PoseIsoRT &tool,
                         int capacity, CertificateOptions options = {})
    -> CertifiedFibreResult;
} // namespace crx
