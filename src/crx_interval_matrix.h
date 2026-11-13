// SPDX-License-Identifier: Apache-2.0
// SPDX-FileCopyrightText: 2026 Centre Technologique en Aérospatiale

#pragma once

#include "crx_interval.h"

#include <algorithm>
#include <array>
#include <cstddef>

namespace crx::interval {

template <std::size_t Rows, std::size_t Columns> struct Matrix {
  std::array<Interval, Rows * Columns> values{};

  auto At(std::size_t row, std::size_t column) -> Interval & {
    return values.at((row * Columns) + column);
  }

  auto At(std::size_t row, std::size_t column) const -> const Interval & {
    return values.at((row * Columns) + column);
  }

  auto IsValid() const -> bool {
    return std::all_of(values.begin(), values.end(),
                       [](Interval value) { return value.IsValid(); });
  }
};

template <std::size_t Rows, std::size_t Inner, std::size_t Columns>
auto Multiply(const Matrix<Rows, Inner> &left,
              const Matrix<Inner, Columns> &right) -> Matrix<Rows, Columns> {
  Matrix<Rows, Columns> result;
  for (std::size_t row = 0; row < Rows; ++row) {
    for (std::size_t col = 0; col < Columns; ++col) {
      Interval sum{0.0};
      for (std::size_t k = 0; k < Inner; ++k) {
        sum = sum + (left.At(row, k) * right.At(k, col));
      }
      result.At(row, col) = sum;
    }
  }
  return result;
}

template <std::size_t Rows, std::size_t Columns>
auto Transpose(const Matrix<Rows, Columns> &input) -> Matrix<Columns, Rows> {
  Matrix<Columns, Rows> output;
  for (std::size_t row = 0; row < Rows; ++row) {
    for (std::size_t col = 0; col < Columns; ++col) {
      output.At(col, row) = input.At(row, col);
    }
  }
  return output;
}

// Bounds the entrywise max norm (vector infinity norm when Columns=1).
template <std::size_t Rows, std::size_t Columns>
auto MaxAbsNorm(const Matrix<Rows, Columns> &input) -> Interval {
  if (!EnvironmentSupported() || !input.IsValid()) {
    return {};
  }
  double upper = 0;
  for (const auto value : input.values) {
    upper = std::max(upper, Abs(value).hi);
  }
  return {0, upper};
}

// Induced infinity norm: maximum row sum, not entrywise maximum.
template <std::size_t Rows, std::size_t Columns>
auto RowSumNorm(const Matrix<Rows, Columns> &input) -> Interval {
  if (!EnvironmentSupported() || !input.IsValid()) {
    return {};
  }
  double upper = 0;
  for (std::size_t row = 0; row < Rows; ++row) {
    Interval sum{0.0};
    for (std::size_t col = 0; col < Columns; ++col) {
      sum = sum + Abs(input.At(row, col));
    }
    if (!sum.IsValid()) {
      return {};
    }
    upper = std::max(upper, sum.hi);
  }
  return {0, upper};
}

template <std::size_t Size>
auto EuclideanNorm(const Matrix<Size, 1> &input) -> Interval {
  Interval sum{0.0};
  for (const auto value : input.values) {
    sum = sum + Square(value);
  }
  if (!sum.IsValid()) {
    return {};
  }
  // The mathematical sum of squares is nonnegative, even if outward rounding
  // of a zero lower sum introduced a negative subnormal endpoint.
  return Sqrt({std::max(0.0, sum.lo), sum.hi});
}
} // namespace crx::interval
