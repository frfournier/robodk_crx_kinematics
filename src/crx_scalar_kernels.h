// SPDX-License-Identifier: Apache-2.0
// SPDX-FileCopyrightText: 2026 Centre Technologique en Aérospatiale

#pragma once

#include <array>

#include <Eigen/Core>
#include <Eigen/Geometry>

// Shared expression trees for numerical and enclosed evaluation. Keep the
// ordinary double accumulation order and vector endpoint distances intact.
namespace crx::scalar {
template <typename T> using Vector3 = Eigen::Matrix<T, 3, 1>;
template <typename T> using Matrix3 = Eigen::Matrix<T, 3, 3>;
template <typename T, int N> using Polynomial = Eigen::Matrix<T, N, 1>;

template <typename T, int N, int M>
auto Multiply(const Polynomial<T, N> &left, const Polynomial<T, M> &right)
    -> Polynomial<T, N + M - 1> {
  Polynomial<T, N + M - 1> result = Polynomial<T, N + M - 1>::Zero();
  for (int i = 0; i < N; ++i) {
    result.template segment<M>(i) += left[i] * right;
  }
  return result;
}

template <typename T>
auto Square(const Polynomial<T, 3> &polynomial) -> Polynomial<T, 5> {
  const T cross01 = polynomial.x() * polynomial.y();
  const T cross02 = polynomial.x() * polynomial.z();
  const T cross12 = polynomial.y() * polynomial.z();
  return {polynomial.x() * polynomial.x(), cross01 + cross01,
          (cross02 + (polynomial.y() * polynomial.y())) + cross02,
          cross12 + cross12, polynomial.z() * polynomial.z()};
}

template <typename T>
auto MultiplyByHalfAngleDenominator(const Polynomial<T, 3> &polynomial)
    -> Polynomial<T, 5> {
  return {polynomial.x(), polynomial.y(), polynomial.x() + polynomial.z(),
          polynomial.y(), polynomial.z()};
}

template <typename T, typename Geometry>
auto ResidualForAxis(const Geometry &lengths, const Vector3<T> &p,
                     const Vector3<T> &axis) -> T {
  const Vector3<T> x = p + lengths.r * axis;
  const Vector3<T> n = Vector3<T>::UnitZ().cross(x);
  const T a_squared = lengths.a * lengths.a;
  const T b_squared = lengths.b * lengths.b;
  const T k = (a_squared + x.squaredNorm() - b_squared) / T{2.0};
  const Vector3<T> axis_cross_n = axis.cross(n);
  const T determinant = x.dot(axis_cross_n);
  const Vector3<T> numerator = k * axis_cross_n + x.dot(axis) * n.cross(x);
  return numerator.squaredNorm() - (a_squared * determinant * determinant);
}

template <typename T, typename Geometry>
auto ConstructCoefficients(const Geometry &lengths, const Vector3<T> &p,
                           const Vector3<T> &v, const Vector3<T> &w)
    -> std::array<T, 9> {
  const Polynomial<T, 3> q{T{1.0}, T{0.0}, T{1.0}};
  const T a_squared = lengths.a * lengths.a;
  const T b_squared = lengths.b * lengths.b;
  const T r = lengths.r;
  const T p_squared = p.squaredNorm();
  const T pz = p.z();
  const T pv = p.dot(v);
  const Polynomial<T, 3> d{pv, T{2.0} * p.dot(w), -pv};
  const Polynomial<T, 3> uz{v.z(), T{2.0} * w.z(), -v.z()};
  const Polynomial<T, 3> z = pz * q + r * uz;
  // Do not replace endpoint vector norms by cancelling scalar distances.
  const Polynomial<T, 3> s{(p + r * v).squaredNorm(), T{4.0} * r * p.dot(w),
                           (p - r * v).squaredNorm()};
  const Polynomial<T, 3> g = d + r * q;
  const T difference = (lengths.a - lengths.b) * (lengths.a + lengths.b);
  const Polynomial<T, 3> a_hat = s - difference * q;
  const Polynomial<T, 3> sphere = s + difference * q;
  const Polynomial<T, 5> d_hat =
      (T{4.0} * a_squared) * MultiplyByHalfAngleDenominator(s) - Square(sphere);
  const Polynomial<T, 5> first_factor =
      Square(a_hat) - (T{4.0} * b_squared) * Square(z);
  const Polynomial<T, 9> first = Multiply(first_factor, Square(g));
  const Polynomial<T, 3> second_factor =
      (T{2.0} * pz) * g + ((r * r) - p_squared) * uz;
  const Polynomial<T, 9> second = Multiply(d_hat, Multiply(uz, second_factor));
  std::array<T, 9> result{};
  Eigen::Map<Polynomial<T, 9>> mapped(result.data());
  mapped = T{0.25} * (first + second);
  return result;
}

// q(t)^2 Delta(theta) with theta=alpha+2 atan(t). Unit exact phase axes
// imply Delta=h(m+r)-(|p|^2+r*m)U; no interpolation or degree trimming.
template <typename T, typename Geometry>
auto ConstructDeterminantCoefficients(const Geometry &lengths,
                                      const Vector3<T> &p, const Vector3<T> &v,
                                      const Vector3<T> &w) -> std::array<T, 5> {
  const Polynomial<T, 3> q{T{1.0}, T{0.0}, T{1.0}};
  const Polynomial<T, 3> m{p.dot(v), T{2.0} * p.dot(w), -p.dot(v)};
  const Polynomial<T, 3> u{v.z(), T{2.0} * w.z(), -v.z()};
  const Polynomial<T, 3> g = m + lengths.r * q;
  const Polynomial<T, 3> second = p.squaredNorm() * q + lengths.r * m;
  const Polynomial<T, 5> polynomial =
      p.z() * Multiply(g, q) - Multiply(second, u);
  std::array<T, 5> result{};
  Eigen::Map<Polynomial<T, 5>>(result.data()) = polynomial;
  return result;
}

template <typename T> struct Pose {
  Matrix3<T> rotation;
  Vector3<T> translation;
};

template <typename T> auto BaseRotation(T sine, T cosine) -> Matrix3<T> {
  Matrix3<T> result;
  result << cosine, -sine, T{0.0}, sine, cosine, T{0.0}, T{0.0}, T{0.0},
      (T{1.0} - cosine) + cosine;
  return result;
}

template <typename T>
auto RotateLocalX(const Matrix3<T> &rotation, T sine, T cosine) -> Matrix3<T> {
  Matrix3<T> result;
  result.col(0) = ((T{1.0} - cosine) + cosine) * rotation.col(0);
  result.col(1) = cosine * rotation.col(1) + sine * rotation.col(2);
  result.col(2) = -sine * rotation.col(1) + cosine * rotation.col(2);
  return result;
}

template <typename T>
auto RotateLocalY(const Matrix3<T> &rotation, T sine, T cosine) -> Matrix3<T> {
  Matrix3<T> result;
  result.col(0) = cosine * rotation.col(0) - sine * rotation.col(2);
  result.col(1) = ((T{1.0} - cosine) + cosine) * rotation.col(1);
  result.col(2) = sine * rotation.col(0) + cosine * rotation.col(2);
  return result;
}

// Canonical FK's final assembly, also used by ordinary joint recovery. r2,
// r4,r5,r6 are the existing joint rotations, not recovered elbow intermediates.
template <typename T, typename Geometry>
auto ForwardPose(const Geometry &lengths, const Matrix3<T> &r2,
                 const Matrix3<T> &r4, const Matrix3<T> &r5,
                 const Matrix3<T> &r6) -> Pose<T> {
  Pose<T> pose;
  pose.rotation.col(0) = r6.col(2);
  pose.rotation.col(1) = -r6.col(1);
  pose.rotation.col(2) = r6.col(0);
  pose.translation = lengths.a * r2.col(2) +
                     r4 * Vector3<T>(lengths.b, -lengths.r, T{0.0}) +
                     lengths.c * r5.col(0);
  return pose;
}
} // namespace crx::scalar
