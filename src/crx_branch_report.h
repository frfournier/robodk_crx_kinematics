// SPDX-License-Identifier: Apache-2.0
// SPDX-FileCopyrightText: 2026 Centre Technologique en Aérospatiale

#pragma once

#include "crx_branch_graph.h"

#include <iosfwd>

namespace crx::branch::report {
// Source provenance and owned proof payload, including all unresolved tasks,
// scanned pairs and attempted paths. Global coverage always stays inconclusive.
auto Write(std::ostream &output, const Graph &graph) -> bool;

// sizeof-based retained payload plus dynamic vector capacities; excludes
// allocator overhead, transient workspaces, stream buffers and process RSS.
auto RetainedBytes(const Graph &graph) -> std::uint64_t;
} // namespace crx::branch::report
