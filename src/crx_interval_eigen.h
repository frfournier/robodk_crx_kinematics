// SPDX-License-Identifier: Apache-2.0
// SPDX-FileCopyrightText: 2026 Centre Technologique en Aérospatiale

#pragma once

#include "crx_interval.h"

#include <Eigen/Core>

// Fixed-size shared Eigen expressions use scalar Interval operations; no
// packetization or Eigen transcendental accuracy assumption enters a proof.
namespace Eigen {
template <>
struct NumTraits<crx::interval::Interval>
    : GenericNumTraits<crx::interval::Interval> {
  using Real = crx::interval::Interval;
  using NonInteger = Real;
  using Nested = Real;
  using Literal = Real;

  enum {
    IsComplex = 0,
    IsInteger = 0,
    IsSigned = 1,
    RequireInitialization = 1,
    ReadCost = 2,
    AddCost = 16,
    MulCost = 32
  };
};
} // namespace Eigen
