// SPDX-License-Identifier: Apache-2.0
// SPDX-FileCopyrightText: 2026 Centre Technologique en Aérospatiale

#pragma once

#include "crx_path_tracking.h"

#include <iosfwd>

namespace crx::path::report {
// JSON1, classic numeric locale, round-trip binary64 decimals, null for
// unavailable bounds. Reports owned sealed evidence; does not perform proofs.
auto Write(std::ostream &output, const TrackingCertificate &path) -> bool;
auto WritePoint(std::ostream &output, const ball::Certificate &point) -> bool;
auto WriteCell(std::ostream &output, const CellCertificate &cell) -> bool;
} // namespace crx::path::report
