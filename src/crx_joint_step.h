#pragma once

#include "crx_types.h"

namespace crx::canonical::detail {

// Rotation logarithm for refinement, including the small-angle limit.
// Acceptance still uses full-matrix orientation closure.
auto RotationVector(const Mat3 &rotation) -> Vec3;

// Minimum-norm correction of the normalized six-joint pose Jacobian.
// Accuracy refinement discards near-null directions using sqrt(epsilon).
// Fixed storage, including rank-deficient and zero Jacobians.
auto SolveMinimumNormStep(const Eigen::Matrix<double, 6, 6> &jacobian,
                          const Vec6 &error, bool improve_accuracy) -> Vec6;

} // namespace crx::canonical::detail
