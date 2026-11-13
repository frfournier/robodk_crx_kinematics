// SPDX-License-Identifier: Apache-2.0
// SPDX-FileCopyrightText: 2026 Centre Technologique en Aérospatiale

#pragma once

#include <array>
#include <cstddef>

#include "crx_types.h"

// Canonical orthogonal-axis geometry and direct half-angle polynomial kernels.
// Model adaptation and complete IK solving are handled by their callers.
namespace crx::canonical {

struct Lengths {
  double a = 0.0;
  double b = 0.0;
  double c = 0.0;
  double r = 0.0;
};

enum class PreparationStatus { Ready, InvalidInput, NumericalRangeFailure };

struct ResidualInvariants {
  // Rows of (p.u, u.z) = L (cos(angle), sin(angle)), prepared once per target.
  double p_v = 0.0;
  double p_w = 0.0;
  double v_z = 0.0;
  double w_z = 0.0;
  double p_squared_plus_r_squared = 0.0;
  double height = 0.0;
  double a_squared = 0.0;
  double b_squared = 0.0;
  double arm_difference = 0.0;
  bool unit_basis = false;
};

struct WristCircle {
  Lengths lengths{}; // Divided by length_scale, including signed lengths.
  Vec3 p = Vec3::Zero();
  Vec3 v = Vec3::Zero();
  Vec3 w = Vec3::Zero();
  double length_scale = 0.0;
  ResidualInvariants residual{};
};

struct ResidualEvaluation {
  double value = 0.0;
  double derivative = 0.0;
  bool used_geometric_fallback = false;
};

enum class CramerStatus { Ready, GuardRejected, NumericalRangeFailure };

struct CramerEvaluation {
  double value = 0.0;
  double derivative = 0.0;
  Vec3 elbow = Vec3::Zero();
};

inline constexpr std::size_t kCoefficientCount = 9;
using Coefficients = std::array<double, kCoefficientCount>;

struct HalfAnglePolynomial {
  // Ascending powers: P(t) = (1+t*t)^4 f(chart_angle + 2 atan(t)).
  // Never discard a small leading coefficient. This is a rounded polynomial,
  // not an enclosure or a certificate for the original target.
  Coefficients coefficients{};
  double chart_angle = 0.0;
  // Separately evaluated at u = -v_chart; no tan(pi/2) or sin(pi) evaluation.
  double omitted_point_residual = 0.0;
};

struct PolynomialValue {
  double value = 0.0;
  double derivative = 0.0;
};

// Lengths and translation share one unit. a,b must be nonzero; c,r may be zero.
// L=max(|a|,|b|,|c|,|r|). Input must already be in canonical coordinates.
// Rotation and homogeneous-row roundoff within 128*epsilon is accepted without
// projection; larger orthogonality/determinant and bottom-row errors are
// rejected. Failure leaves output unchanged. Ready means prepared, never IK
// feasibility.
auto PrepareWristCircle(const Lengths &lengths, const PoseIsoRT &target,
                        WristCircle &output) -> PreparationStatus;

// Requires a prepared circle. A nonfinite result is numerical failure, never an
// exclusion. Residuals are dimensionless (physical f / L^6), separate from FK
// position/orientation tolerances. A zero does not certify exceptional
// incidence.
auto EvaluateResidual(const WristCircle &circle, double angle) -> double;
auto EvaluateResidualDerivative(const WristCircle &circle, double angle)
    -> double;

// Joint residual/derivative evaluation using cached bivariate invariants.
// Cancellation and admitted nonunit bases retain the geometric route. The
// dispatch is numerical, not an enclosure, exclusion or root certificate.
auto EvaluateResidualPair(const WristCircle &circle, double angle)
    -> ResidualEvaluation;

// Compact scalar Cramer numerator and its phase derivative. Uses the same
// conditioning margin as discovery and leaves output unchanged on failure.
// The returned quotient |N/Delta|^2-a^2 equals f/Delta^2 algebraically; forming
// it through the elbow retains accuracy near cancellation of f's factors.
auto EvaluateCramerElbowResidual(const WristCircle &circle, double angle,
                                 CramerEvaluation &output) -> CramerStatus;

// Direct degree-at-most-eight scalar convolution, with no interpolation or
// truncation. Supports shifted charts. Failure leaves output unchanged.
auto BuildHalfAnglePolynomial(const WristCircle &circle, double chart_angle,
                              HalfAnglePolynomial &output) -> PreparationStatus;

// These utilities operate on the represented coefficients, not exact geometry.
// Degree -1 means all nine stored coefficients are exactly zero; even then the
// geometric zero-polynomial/continuous-fibre decision remains unresolved.
auto RepresentedDegree(const Coefficients &coefficients) -> int;
auto EvaluatePolynomial(const Coefficients &coefficients, double argument)
    -> PolynomialValue;

} // namespace crx::canonical
