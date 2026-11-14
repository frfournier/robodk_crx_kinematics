// SPDX-License-Identifier: Apache-2.0
// SPDX-FileCopyrightText: 2026 Centre Technologique en Aérospatiale

#include "crx_path_report.h"
#include "crx_ball_certificate.h"
#include "crx_exact_problem.h"
#include "crx_path_certificate.h"
#include "crx_path_tracking.h"

#include <Eigen/Core>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <iomanip>
#include <ios>
#include <locale>
#include <ostream>

namespace crx::path::report {
namespace {
void Number(std::ostream &output, double value) {
  if (value == 0 && std::signbit(value)) {
    output << "-0.0";
  } else if (std::isfinite(value)) {
    output << value;
  } else {
    output << "null";
  }
}

void Number(std::ostream &output, std::uint64_t value) { output << value; }

void Number(std::ostream &output, Interval value) {
  output << '[';
  Number(output, value.lo);
  output << ',';
  Number(output, value.hi);
  output << ']';
}

template <typename T, std::size_t N>
void Array(std::ostream &output, const std::array<T, N> &values) {
  output << '[';
  for (std::size_t i = 0; i < N; ++i) {
    if (i) {
      output << ',';
    }
    Number(output, values.at(i));
  }
  output << ']';
}

template <typename Derived>
void Matrix(std::ostream &output, const Eigen::MatrixBase<Derived> &values) {
  output << '[';
  for (Eigen::Index r = 0; r < values.rows(); ++r) {
    for (Eigen::Index c = 0; c < values.cols(); ++c) {
      if (r || c) {
        output << ',';
      }
      Number(output, values(r, c));
    }
  }
  output << ']';
}

void Point(std::ostream &output, const ball::Certificate &certificate) {
  output << "{\"failure\":" << (certificate.IsValid() ? 0 : 1);
  if (certificate.IsValid()) {
    const auto &b = certificate.Values();
    std::array<double, exact::kIdentityWords> raw{};
    for (std::size_t i = 0; i < raw.size(); ++i) {
      const auto bits = b.identity.words.at(i);
      std::memcpy(&raw.at(i), &bits, sizeof(bits));
    }
    output << ",\"input\":";
    Array(output, raw);
    output << ",\"input_words\":";
    Array(output, b.identity.words);
    output << ",\"schema\":" << b.identity.schema << ",\"centre\":";
    Array(output, b.centre);
    output << ",\"constants\":";
    Array(output,
          std::array<double, 11>{b.existence_radius, b.uniqueness_radius,
                                 b.inverse_residual, b.beta, b.lipschitz, b.eta,
                                 b.pose_defect, b.screw_norm_sum,
                                 b.angular_norm_sum, b.anchor_position_norm,
                                 static_cast<double>(b.aspect_sign)});
    output << ",\"preconditioner\":";
    Matrix(output, b.preconditioner);
    output << ",\"coarse_eta\":";
    Number(output, b.coarse_eta);
    output << ",\"used_log_series\":" << b.used_log_series << ",\"trace\":";
    Number(output, b.target_relative_trace);
  }
  output << '}';
}

void Cell(std::ostream &output, const CellCertificate &certificate) {
  output << "{\"local\":" << certificate.HasLocalLift()
         << ",\"admitted\":" << certificate.IsAdmitted();
  if (certificate.HasLocalLift()) {
    const auto &data = certificate.Values();
    output << ",\"point\":";
    Point(output, data.point_witness);
    output << ",\"parameter\":";
    Number(output, data.parameter);
    output << ",\"rho\":";
    Number(output, data.existence_radius);
    output << ",\"eta\":";
    Number(output, data.residual.eta);
    output << ",\"coarse_eta\":";
    Number(output, data.residual.coarse_eta);
    output << ",\"defect\":";
    Number(output, data.residual.pose_defect);
    output << ",\"used_log_series\":" << data.residual.used_log_series
           << ",\"trace\":";
    Number(output, data.residual.relative_trace);
    output << ",\"rotation\":";
    Matrix(output, data.target_enclosure.rotation);
    output << ",\"translation\":";
    Matrix(output, data.target_enclosure.translation);
    output << ",\"quaternion_norm\":";
    Number(output, data.quaternion_norm_squared);
    output << ",\"slacks\":";
    Array(output, data.command_slack_lower);
    output << ",\"slack_lipschitz\":";
    Array(output, data.slack_lipschitz);
  }
  output << '}';
}
} // namespace

auto WritePoint(std::ostream &output, const ball::Certificate &point) -> bool {
  output.imbue(std::locale::classic());
  output << std::setprecision(17) << std::boolalpha;
  Point(output, point);
  return output.good();
}

auto WriteCell(std::ostream &output, const CellCertificate &cell) -> bool {
  output.imbue(std::locale::classic());
  output << std::setprecision(17) << std::boolalpha;
  Cell(output, cell);
  return output.good();
}

auto Write(std::ostream &output, const TrackingCertificate &path) -> bool {
  output.imbue(std::locale::classic());
  output << std::setprecision(17) << std::boolalpha;
  const auto &data = path.Values();
  output << "{\"schema\":1,\"complete\":" << path.IsComplete()
         << ",\"failure\":" << static_cast<int>(data.failure)
         << ",\"coverage\":\"one supplied lift; no global completeness claim\""
         << ",\"path_identity_schema\":" << data.path_identity.schema
         << ",\"convention\":" << data.path_identity.frame_convention
         << ",\"path_words\":";
  Array(output, data.path_identity.words);
  output << ",\"profile\":"
         << static_cast<int>(data.limits.Values().raw.profile);
  output << ",\"lower\":";
  Array(output, data.limits.Values().raw.lower_degrees);
  output << ",\"upper\":";
  Array(output, data.limits.Values().raw.upper_degrees);
  output << ",\"first\":";
  Point(output, data.first.certificate);
  output << ",\"last\":";
  Point(output, data.last.certificate);
  output << ",\"options\":{\"max_nodes\":" << data.options.max_nodes
         << ",\"max_depth\":" << data.options.max_depth
         << ",\"max_point_certificates\":"
         << data.options.max_point_certificates
         << ",\"max_trials_per_node\":" << data.options.max_trials_per_node
         << '}';
  output << ",\"work\":{\"nodes\":" << data.work.nodes
         << ",\"point_certificates\":" << data.work.point_certificates
         << ",\"cell_checks\":" << data.work.cell_checks
         << ",\"nearby_calls\":" << data.work.nearby_calls
         << ",\"full_discovery_calls\":" << data.work.full_discovery_calls
         << ",\"nominations\":" << data.work.nominations
         << ",\"subdivisions\":" << data.work.subdivisions
         << ",\"greatest_depth\":" << data.work.greatest_depth << '}';
  output << ",\"uncovered_measure\":";
  Number(output, data.uncovered_measure);
  output << ",\"initial_inclusion\":";
  Number(output, data.initial_inclusion);
  output << ",\"initial_uniqueness_radius\":";
  Number(output, data.initial_uniqueness_radius);
  output << ",\"final_inclusion\":";
  Number(output, data.final_inclusion);
  output << ",\"final_uniqueness_radius\":";
  Number(output, data.final_uniqueness_radius);
  output << ",\"leaves\":[";
  for (std::size_t i = 0; i < data.leaves.size(); ++i) {
    if (i != 0U) {
      output << ',';
    }
    const auto &leaf = data.leaves.at(i);
    output << "{\"parameter\":";
    Number(output, leaf.parameter);
    output << ",\"depth\":" << leaf.depth
           << ",\"accepted\":" << leaf.IsAdmitted()
           << ",\"failure\":" << static_cast<int>(leaf.failure)
           << ",\"cell_failure\":" << static_cast<int>(leaf.cell_failure)
           << ",\"point_failure\":" << static_cast<int>(leaf.point_failure)
           << ",\"cell\":";
    Cell(output, leaf.cell);
    output << '}';
  }
  output << "],\"links\":[";
  for (std::size_t i = 0; i < data.links.size(); ++i) {
    if (i != 0U) {
      output << ',';
    }
    const auto &link = data.links.at(i);
    const auto &bounds = link.certificate.Values();
    output << "{\"first\":" << link.first << ",\"second\":" << link.second
           << ",\"certified\":" << link.certificate.IsValid()
           << ",\"inclusion\":";
    Number(output, bounds.inclusion_bound);
    output << ",\"next_radius\":";
    Number(output, bounds.next_uniqueness_radius);
    output << '}';
  }
  output << "]}";
  return output.good();
}
} // namespace crx::path::report
