// SPDX-License-Identifier: Apache-2.0
// SPDX-FileCopyrightText: 2026 Centre Technologique en Aérospatiale

#include "crx_path_request.h"
#include "crx_branch_graph.h"
#include "crx_command_certificate.h"
#include "crx_exact_problem.h"

#include <array>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>

namespace crx::path::request {
namespace {
// Bounded private wire adapter for the Python JSON frontend. All geometry and
// certificate decisions remain in the production C++ modules. No test probe or
// caller-supplied proof flag is accepted by this executable.
class Reader {
public:
  explicit Reader(const std::filesystem::path &filename)
      : input_(filename, std::ios::binary) {
    if (!input_) {
      throw std::runtime_error("Cannot open request packet");
    }
    input_.seekg(0, std::ios::end);
    const auto size = input_.tellg();
    if (size < 8 || size > 1048576) {
      throw std::runtime_error("Request packet size is invalid");
    }
    input_.seekg(0);
    std::array<char, 8> magic{};
    input_.read(magic.data(), static_cast<std::streamsize>(magic.size()));
    if (std::string(magic.data(), magic.size()) != "CRXPATH1") {
      throw std::runtime_error("Unsupported request packet version");
    }
  }

  auto Unsigned() -> std::uint32_t {
    std::array<unsigned char, 4> bytes{};
    Read(bytes);
    std::uint32_t value = 0;
    for (std::size_t i = 0; i < bytes.size(); ++i) {
      value |= static_cast<std::uint32_t>(bytes.at(i)) << (8U * i);
    }
    return value;
  }

  auto Real() -> double {
    static_assert(sizeof(double) == 8 &&
                  std::numeric_limits<double>::is_iec559);
    std::array<unsigned char, 8> bytes{};
    Read(bytes);
    std::uint64_t bits = 0;
    for (std::size_t i = 0; i < bytes.size(); ++i) {
      bits |= static_cast<std::uint64_t>(bytes.at(i)) << (8U * i);
    }
    double value = 0;
    std::memcpy(&value, &bits, sizeof(value));
    return value;
  }

  template <std::size_t N> void Reals(std::array<double, N> &values) {
    for (auto &value : values) {
      value = Real();
    }
  }

  void Pose(crx::exact::PoseInput &pose) {
    Reals(pose.quaternion);
    Reals(pose.position);
    Reals(pose.input_rotation);
  }

  auto Text() -> std::string {
    const auto count = Unsigned();
    if (count == 0 || count > 256) {
      throw std::runtime_error("Task ID size is invalid");
    }
    std::string text(count, '\0');
    input_.read(text.data(), count);
    Check();
    return text;
  }

  void End() {
    if (input_.peek() != std::char_traits<char>::eof()) {
      throw std::runtime_error("Trailing request packet bytes");
    }
  }

private:
  template <std::size_t N> void Read(std::array<unsigned char, N> &bytes) {
    input_.read(reinterpret_cast<char *>(bytes.data()), N);
    Check();
  }

  void Check() {
    if (!input_) {
      throw std::runtime_error("Truncated request packet");
    }
  }

  std::ifstream input_;
};

} // namespace

auto Read(const std::filesystem::path &filename) -> Query {
  Reader reader(filename);
  const auto tasks = reader.Unsigned();
  const auto connections = reader.Unsigned();
  if (tasks == 0 || tasks > 64 || connections > 256) {
    throw std::runtime_error("Task/connection count is invalid");
  }
  Query result;
  auto &input = result.input;
  input.convention =
      static_cast<crx::exact::FrameConvention>(reader.Unsigned());
  input.limits.profile = static_cast<crx::command::Profile>(reader.Unsigned());
  reader.Reals(input.model.lengths);
  input.model.base_height = reader.Real();
  reader.Reals(input.model.encoder_zero);
  reader.Reals(input.model.senses);
  reader.Reals(input.model.lower_commands);
  reader.Reals(input.model.upper_commands);
  reader.Pose(input.model.target);
  reader.Pose(input.model.base);
  reader.Pose(input.model.tool);
  reader.Reals(input.limits.lower_degrees);
  reader.Reals(input.limits.upper_degrees);
  reader.Reals(input.limits.seed_degrees);
  reader.Reals(input.limits.weights);
  const auto has_seed = reader.Unsigned();
  if (has_seed > 1) {
    throw std::runtime_error("Seed flag is invalid");
  }
  input.limits.has_seed = has_seed != 0;
  auto &options = result.options;
  for (auto *value :
       {&options.max_vertices, &options.max_pair_tests, &options.max_paths,
        &options.max_root_nodes, &options.max_endpoint_certificates,
        &options.max_lifts, &options.max_path_nodes,
        &options.max_path_certificates, &options.endpoint.roots.max_nodes,
        &options.endpoint.roots.max_depth, &options.endpoint.roots.max_leaves,
        &options.endpoint.roots.max_chart_attempts,
        &options.endpoint.max_candidate_certificates,
        &options.endpoint.max_pair_tests, &options.tracking.max_nodes,
        &options.tracking.max_depth, &options.tracking.max_point_certificates,
        &options.tracking.max_trials_per_node}) {
    *value = reader.Unsigned();
  }
  for (std::uint32_t i = 0; i < tasks; ++i) {
    crx::branch::Task task;
    task.id = reader.Text();
    reader.Pose(task.target);
    input.tasks.push_back(std::move(task));
  }
  for (std::uint32_t i = 0; i < connections; ++i) {
    const auto first = reader.Unsigned();
    const auto last = reader.Unsigned();
    input.connections.push_back({first, last});
  }
  reader.End();
  return result;
}
} // namespace crx::path::request
