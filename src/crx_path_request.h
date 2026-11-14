// SPDX-License-Identifier: Apache-2.0
// SPDX-FileCopyrightText: 2026 Centre Technologique en Aérospatiale

#pragma once

#include "crx_branch_graph.h"

#include <filesystem>

namespace crx::path::request {
struct Query {
  branch::Input input;
  branch::Options options;
};

// Decode a bounded, versioned request. Geometry/proof work begins only after
// the entire packet has passed shape/trailing-byte checks. Throws on error.
auto Read(const std::filesystem::path &filename) -> Query;
} // namespace crx::path::request
