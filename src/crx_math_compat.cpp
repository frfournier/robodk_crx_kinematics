// SPDX-License-Identifier: Apache-2.0
// SPDX-FileCopyrightText: 2026 Centre Technologique en Aérospatiale
/**
 * @file crx_math_compat.cpp
 * @brief Implementation of math compatibility helpers.
 */

#include <cmath>

#if defined(_WIN32) && defined(__clang__)
extern "C" void sincos(double value, double *sine, double *cosine) {
  *sine = std::sin(value);
  *cosine = std::cos(value);
}
#endif
