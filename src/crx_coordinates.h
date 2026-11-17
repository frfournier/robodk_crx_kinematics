// SPDX-License-Identifier: Apache-2.0
// SPDX-FileCopyrightText: 2026 Centre Technologique en Aérospatiale

#pragma once

#include "crx_types.h"

namespace crx {

// Coupled command radians u and canonical radians q satisfy q = D*u + offset.
// D has entries +/-1. Coupling has already been applied at the caller boundary;
// applying it again here would change both FK and command limits.
struct CanonicalCommandMap {
  Vec6 diagonal = Vec6::Ones();
  Vec6 offset = Vec6::Zero();

  auto FromCommand(const Vec6 &command) const -> Vec6 {
    return diagonal.cwiseProduct(command) + offset;
  }

  auto ToCommand(const Vec6 &canonical) const -> Vec6 {
    return diagonal.cwiseProduct(canonical - offset);
  }

  void CanonicalBounds(const Vec6 &lower, const Vec6 &upper, Vec6 &mapped_lower,
                       Vec6 &mapped_upper) const {
    const Vec6 first = FromCommand(lower);
    const Vec6 second = FromCommand(upper);
    mapped_lower = first.cwiseMin(second);
    mapped_upper = first.cwiseMax(second);
  }
};

} // namespace crx
