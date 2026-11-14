// SPDX-License-Identifier: Apache-2.0
// SPDX-FileCopyrightText: 2026 Centre Technologique en Aérospatiale

#include "crx_branch_graph.h"
#include "crx_branch_report.h"
#include "crx_path_request.h"

#include <exception>
#include <filesystem>
#include <iostream>
#include <stdexcept>

namespace {
auto Run(const std::filesystem::path &filename) -> int {
  const auto query = crx::path::request::Read(filename);
  const auto graph = crx::branch::Build(query.input, query.options);
  if (graph.Values().status == crx::branch::Status::InvalidInput) {
    throw std::runtime_error("Invalid graph input or work bounds");
  }
  if (!crx::branch::report::Write(std::cout, graph)) {
    throw std::runtime_error("Cannot write graph report");
  }
  std::cout << '\n';
  return 0;
}

auto Main(int argc, const std::filesystem::path &argument) -> int {
  if (argc == 2 && argument == "--version") {
    std::cout << "crx_path_tool wire=1 report=1\n";
    return 0;
  }
  if (argc != 2 || argument == "--help") {
    std::cout
        << "CRX certified path backend. Use python -m crx_kinematics.paths "
           "REQUEST.json --backend crx_path_tool.exe [--output REPORT.json]\n";
    return argc == 2 ? 0 : 2;
  }
  try {
    return Run(argument);
  } catch (const std::exception &error) {
    std::cerr << "crx_path_tool: " << error.what() << '\n';
    return 2;
  }
}
} // namespace

#ifdef _WIN32
auto wmain(int argc, wchar_t **argv) -> int {
  return Main(argc, argc == 2 ? std::filesystem::path(argv[1])
                              : std::filesystem::path{});
}
#else
auto main(int argc, char **argv) -> int {
  return Main(argc, argc == 2 ? std::filesystem::path(argv[1])
                              : std::filesystem::path{});
}
#endif
