// SPDX-License-Identifier: Apache-2.0
// SPDX-FileCopyrightText: 2026 Centre Technologique en Aérospatiale
#pragma once

#include "crx_command_certificate.h"

// Benchmark-only stage access. These scratch data are not sealed certificates
// and never enter the DLL or its public ABI. The shipped implementation is
// compiled directly into a separate strict, non-LTO benchmark object.
namespace crx::benchmark {
auto PrepareRanges(const exact::Problem &, const catalogue::PhaseCatalogue &,
                   const command::Context &) -> command::Data;
auto Enumerate(command::Data &, std::uint64_t budget) -> bool;
void Rank(command::Data &);
} // namespace crx::benchmark
