#pragma once

#include <array>
#include <cstddef>

#include "crx_types.h"

// Internal comparison kernel for appendix G2/G4 in
// notes/crx-implementation-review.md. Not a robot-model adapter or IK solver.
namespace crx::canonical {

struct Lengths {
  double a = 0.0;
  double b = 0.0;
  double c = 0.0;
  double r = 0.0;
};

enum class PreparationStatus { Ready, InvalidInput, NumericalRangeFailure };

struct WristCircle {
  Lengths lengths{}; // Divided by length_scale, including signed lengths.
  Vec3 p = Vec3::Zero();
  Vec3 v = Vec3::Zero();
  Vec3 w = Vec3::Zero();
  double length_scale = 0.0;
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
// Rotation roundoff within 128*epsilon is accepted without projection; larger
// orthogonality/determinant errors and non-affine bottom rows are rejected.
// Failure leaves output unchanged. Ready means prepared, never IK feasibility.
auto PrepareWristCircle(const Lengths &lengths, const PoseIsoRT &target,
                        WristCircle &output) -> PreparationStatus;

// Requires a prepared circle. A nonfinite result is numerical failure, never an
// exclusion. Residuals are dimensionless (physical f / L^6), separate from FK
// position/orientation tolerances. A zero does not certify exceptional
// incidence.
auto EvaluateResidual(const WristCircle &circle, double angle) -> double;

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
