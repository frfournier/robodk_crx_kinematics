// SPDX-License-Identifier: Apache-2.0
// SPDX-FileCopyrightText: 2026 Centre Technologique en Aérospatiale
#pragma once

#include "crx_certified_solver.h"
#include "crx_types.h"

#include <array>

struct crx_v1_handle {
  crx::CrxModelData model;
  // Original binary64 degree provenance, before legacy rounded conversions.
  std::array<double, crx::kDofCount> lower_degrees{}, upper_degrees{};
};

struct crx_v1_certificate_handle {
  crx::CertificateAttachment attachment;
};
