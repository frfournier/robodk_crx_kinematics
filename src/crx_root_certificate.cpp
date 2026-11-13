// SPDX-License-Identifier: Apache-2.0
// SPDX-FileCopyrightText: 2026 Centre Technologique en Aérospatiale

#include "crx_root_certificate.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <utility>
#include <vector>

#ifdef __FAST_MATH__
#error "Root enclosures require strict floating-point compilation"
#endif

namespace crx::root {
namespace {
constexpr std::size_t kDegree = 8;
constexpr std::uint32_t kMaximumBudget = 1U << 20;

template <typename Values> auto Valid(const Values &values) -> bool {
  return std::all_of(values.begin(), values.end(),
                     [](Interval x) { return x.IsValid(); });
}

template <typename Values> auto Zero(const Values &values) -> bool {
  return std::all_of(values.begin(), values.end(),
                     [](Interval x) { return x.lo == 0 && x.hi == 0; });
}

auto Sign(Interval x) -> int {
  return !x.IsValid() ? 0 : (x.lo > 0 ? 1 : (x.hi < 0 ? -1 : 0));
}

template <typename Values> auto StrictSign(const Values &values) -> int {
  const int sign = Sign(values.front());
  return sign != 0 &&
                 std::all_of(values.begin(), values.end(),
                             [sign](Interval x) { return Sign(x) == sign; })
             ? sign
             : 0;
}

auto ValidOptions(Options options, bool circle) -> bool {
  return options.max_depth <= 50 && options.max_nodes <= kMaximumBudget &&
         options.max_leaves >= (circle ? 2U : 1U) &&
         options.max_leaves <= kMaximumBudget &&
         options.max_chart_attempts >= 1 && options.max_chart_attempts <= 8;
}

auto Choose(std::size_t n, std::size_t k) -> std::uint32_t {
  std::uint32_t value = 1;
  for (std::size_t j = 1; j <= k; ++j) {
    value = value * static_cast<std::uint32_t>(n + 1 - j) /
            static_cast<std::uint32_t>(j);
  }
  return value;
}

auto Power(Interval x, std::size_t n) -> Interval {
  Interval value{1};
  for (std::size_t j = 0; j < n; ++j)
    value *= x;
  return value;
}

// atan(x)=2 atan(x/(1+sqrt(1+x^2))), |x|<=1. After reduction |y|<.415.
// Absolute geometric tail after 32 odd terms bounds the entire omitted series;
// no libm accuracy or alternating-interval assumption is used.
auto AtanPoint(double x) -> Interval {
  if (!std::isfinite(x) || x < -1 || x > 1)
    return {};
  const Interval y =
      Interval{x} /
      (Interval{1} +
       interval::Sqrt(Interval{1} + interval::Square(Interval{x})));
  const Interval square = interval::Square(y);
  Interval term = y, sum{0};
  constexpr std::size_t kTerms = 32;
  for (std::size_t j = 0; j < kTerms; ++j) {
    sum += term / Interval{static_cast<double>(2 * j + 1)};
    term *= -square;
  }
  const Interval remainder =
      interval::Abs(term) /
      (Interval{static_cast<double>(2 * kTerms + 1)} * (Interval{1} - square));
  if (!remainder.IsValid())
    return {};
  return Interval{2} * (sum + Interval{-remainder.hi, remainder.hi});
}

auto Phase(Interval domain, std::uint32_t shift, bool opposite) -> Interval {
  const Interval left = AtanPoint(domain.lo), right = AtanPoint(domain.hi);
  if (!left.IsValid() || !right.IsValid())
    return {};
  const Interval alpha =
      interval::Pi() * Interval{static_cast<double>(shift)} / Interval{8};
  return alpha + (opposite ? interval::Pi() : Interval{0}) +
         Interval{2} * Interval{left.lo, right.hi};
}

template <std::size_t N>
auto Split(const std::array<Interval, N> &input, Interval s,
           std::array<Interval, N> &left, std::array<Interval, N> &right)
    -> bool {
  auto temporary = input;
  left.front() = input.front();
  right.back() = input.back();
  for (std::size_t row = 1; row < N; ++row) {
    for (std::size_t j = 0; j < N - row; ++j)
      temporary.at(j) =
          (Interval{1} - s) * temporary.at(j) + s * temporary.at(j + 1);
    left.at(row) = temporary.front();
    right.at(N - 1 - row) = temporary.at(N - 1 - row);
  }
  return Valid(left) && Valid(right);
}

struct Node {
  Leaf leaf;
};

auto Initial(const Coefficients &power, const std::array<Interval, 5> &q,
             Node &node) -> bool {
  Coefficients derivative{}, determinant{}, enclosed{};
  derivative.fill(Interval{0});
  determinant.fill(Interval{0});
  for (std::size_t j = 0; j < kDegree; ++j)
    derivative.at(j) = Interval{static_cast<double>(j + 1)} * power.at(j + 1);
  std::copy(q.begin(), q.end(), determinant.begin());
  if (!Bernstein(power, 8, {-1, 1}, node.leaf.residual_bernstein) ||
      !Bernstein(derivative, 7, {-1, 1}, enclosed))
    return false;
  std::copy_n(enclosed.begin(), 8, node.leaf.derivative_bernstein.begin());
  if (!Bernstein(determinant, 4, {-1, 1}, enclosed))
    return false;
  std::copy_n(enclosed.begin(), 5, node.leaf.determinant_bernstein.begin());
  return true;
}

void Retain(Partition &output, Leaf leaf) {
  switch (leaf.kind) {
  case Kind::Exclusion:
    ++output.exclusions;
    break;
  case Kind::SimpleRoot:
    ++output.roots;
    break;
  case Kind::Unresolved:
    ++output.unresolved;
    break;
  }
  output.max_depth = std::max(output.max_depth, leaf.depth);
  output.leaves.push_back(std::move(leaf));
}

auto Opposite(const exact::Polynomials &first) -> exact::Polynomials {
  auto result = first;
  // tan((theta-alpha)/2)=-1/t. Even fixed degrees give exactly
  // P_opp(t)=t^8 P(-1/t), Q_opp(t)=t^4 Q(-1/t), including degree drops/t=0.
  // This is the exact opposite circle chart, not alpha+rounded(binary64 pi).
  for (std::size_t j = 0; j < result.residual.size(); ++j)
    result.residual.at(j) =
        j % 2 == 0 ? first.residual.at(8 - j) : -first.residual.at(8 - j);
  for (std::size_t j = 0; j < result.derivative.size(); ++j)
    result.derivative.at(j) =
        Interval{static_cast<double>(j + 1)} * result.residual.at(j + 1);
  for (std::size_t j = 0; j < result.determinant.size(); ++j)
    result.determinant.at(j) =
        j % 2 == 0 ? first.determinant.at(4 - j) : -first.determinant.at(4 - j);
  result.chart = first.chart + interval::Pi(); // Descriptor only.
  result.omitted_residual = first.residual.front();
  result.omitted_determinant = first.determinant.front();
  return result;
}

auto UnresolvedBoundary(const exact::Polynomials &polynomials) -> Partition {
  Partition result;
  result.status = Status::Inconclusive;
  Leaf leaf;
  leaf.reason = Reason::ChartBoundary;
  leaf.endpoint_values = {Evaluate(polynomials.residual, -1),
                          Evaluate(polynomials.residual, 1)};
  Node initial;
  if (Initial(polynomials.residual, polynomials.determinant, initial)) {
    leaf.residual_bernstein = initial.leaf.residual_bernstein;
    leaf.derivative_bernstein = initial.leaf.derivative_bernstein;
    leaf.determinant_bernstein = initial.leaf.determinant_bernstein;
  }
  Retain(result, std::move(leaf));
  return result;
}
} // namespace

auto Evaluate(const Coefficients &power, double t) -> Interval {
  if (!interval::EnvironmentSupported() || !Valid(power) || !std::isfinite(t))
    return {};
  Interval result = power.back();
  for (std::size_t j = kDegree; j > 0; --j)
    result = result * Interval{t} + power.at(j - 1);
  return result;
}

auto Bernstein(const Coefficients &power, std::size_t degree, Interval domain,
               Coefficients &output) -> bool {
  if (!interval::EnvironmentSupported() || degree > kDegree || !Valid(power) ||
      !domain.IsValid() || domain.lo >= domain.hi)
    return false;
  const Interval a{domain.lo}, h = Interval{domain.hi} - a;
  Coefficients translated{}, result{};
  translated.fill(Interval{0});
  result.fill(Interval{0});
  for (std::size_t j = 0; j <= degree; ++j) {
    for (std::size_t i = j; i <= degree; ++i)
      translated.at(j) += power.at(i) *
                          Interval{static_cast<double>(Choose(i, j))} *
                          Power(a, i - j);
    translated.at(j) *= Power(h, j);
  }
  for (std::size_t k = 0; k <= degree; ++k)
    for (std::size_t j = 0; j <= k; ++j)
      result.at(k) +=
          translated.at(j) * (Interval{static_cast<double>(Choose(k, j))} /
                              Interval{static_cast<double>(Choose(degree, j))});
  if (!Valid(result))
    return false;
  output = result;
  return true;
}

auto PartitionPolynomial(const Coefficients &power,
                         const std::array<Interval, 5> &determinant,
                         Options options) -> Partition {
  Partition output;
  if (!interval::EnvironmentSupported()) {
    output.status = Status::UnsupportedEnvironment;
    return output;
  }
  if (!ValidOptions(options, false) || !Valid(power) || !Valid(determinant))
    return output;
  Node initial;
  if (!Initial(power, determinant, initial)) {
    output.status = Status::ArithmeticUnavailable;
    initial.leaf.reason = Reason::ArithmeticUnavailable;
    Retain(output, std::move(initial.leaf));
    return output;
  }
  std::vector<Node> stack;
  stack.push_back(std::move(initial));
  while (!stack.empty()) {
    auto node = std::move(stack.back());
    stack.pop_back();
    auto &leaf = node.leaf;
    if (output.nodes >= options.max_nodes) {
      leaf.reason = Reason::WorkLimit;
      Retain(output, std::move(leaf));
      continue;
    }
    ++output.nodes;
    leaf.endpoint_values = {Evaluate(power, leaf.tangent.lo),
                            Evaluate(power, leaf.tangent.hi)};
    if (StrictSign(leaf.residual_bernstein) != 0) {
      leaf.kind = Kind::Exclusion;
      Retain(output, std::move(leaf));
      continue;
    }
    leaf.derivative_sign = StrictSign(leaf.derivative_bernstein);
    leaf.determinant_sign = StrictSign(leaf.determinant_bernstein);
    const int left = Sign(leaf.endpoint_values.front()),
              right = Sign(leaf.endpoint_values.back());
    if (left != 0 && right == -left && leaf.derivative_sign != 0 &&
        leaf.determinant_sign != 0) {
      leaf.kind = Kind::SimpleRoot;
      Retain(output, std::move(leaf));
      continue;
    }
    if (Zero(power))
      leaf.reason = Reason::ZeroPolynomial;
    else if (Zero(leaf.derivative_bernstein))
      leaf.reason = Reason::ZeroDerivative;
    else if (Zero(determinant))
      leaf.reason = Reason::ZeroDeterminant;
    else if (leaf.depth >= options.max_depth)
      leaf.reason = Reason::DepthLimit;
    else if (output.leaves.size() + stack.size() + 2 > options.max_leaves)
      leaf.reason = Reason::LeafLimit;
    if (leaf.reason != Reason::None) {
      Retain(output, std::move(leaf));
      continue;
    }
    // A shared child endpoint must exclude zero. Otherwise an exact dyadic
    // root could remain at endpoints forever or accidentally be counted twice.
    constexpr std::array<double, 5> candidates{.5, .375, .625, .25, .75};
    double midpoint = 0;
    bool separated = false;
    for (const auto ratio : candidates) {
      const double point =
          leaf.tangent.lo + (leaf.tangent.hi - leaf.tangent.lo) * ratio;
      if (point > leaf.tangent.lo && point < leaf.tangent.hi &&
          Sign(Evaluate(power, point)) != 0) {
        midpoint = point;
        separated = true;
        break;
      }
    }
    if (!separated) {
      leaf.reason = Reason::SplitNotSeparated;
      Retain(output, std::move(leaf));
      continue;
    }
    const Interval ratio =
        (Interval{midpoint} - Interval{leaf.tangent.lo}) /
        (Interval{leaf.tangent.hi} - Interval{leaf.tangent.lo});
    Node first, second;
    first.leaf.tangent = {leaf.tangent.lo, midpoint};
    second.leaf.tangent = {midpoint, leaf.tangent.hi};
    first.leaf.depth = second.leaf.depth = leaf.depth + 1;
    if (!ratio.IsValid() || ratio.lo < 0 || ratio.hi > 1 ||
        !Split(leaf.residual_bernstein, ratio, first.leaf.residual_bernstein,
               second.leaf.residual_bernstein) ||
        !Split(leaf.derivative_bernstein, ratio,
               first.leaf.derivative_bernstein,
               second.leaf.derivative_bernstein) ||
        !Split(leaf.determinant_bernstein, ratio,
               first.leaf.determinant_bernstein,
               second.leaf.determinant_bernstein)) {
      leaf.reason = Reason::ArithmeticUnavailable;
      Retain(output, std::move(leaf));
      continue;
    }
    stack.push_back(std::move(second));
    stack.push_back(std::move(first));
  }
  output.status =
      output.unresolved == 0 ? Status::Complete : Status::Inconclusive;
  return output;
}

auto Certify(const exact::Problem &problem, Options options) -> Certificate {
  Certificate result;
  auto &data = result.data_;
  if (!interval::EnvironmentSupported()) {
    data.status = Status::UnsupportedEnvironment;
    return result;
  }
  if (!problem.IsReady() || !ValidOptions(options, true))
    return result;
  data.identity = problem.Data().identity;
  for (std::uint32_t attempt = 0; attempt < options.max_chart_attempts;
       ++attempt) {
    data.attempts = attempt + 1;
    const Interval alpha =
        interval::Pi() * Interval{static_cast<double>(attempt)} / Interval{8};
    exact::Polynomials first;
    const auto built = exact::BuildPolynomials(problem, alpha, first);
    if (built != exact::Status::Ready) {
      data.status = Status::ArithmeticUnavailable;
      data.entire_circle_unresolved = true;
      data.partitions = {};
      data.simple_roots = 0;
      return result;
    }
    data.chart_shift = attempt;
    data.polynomials.front() = std::move(first);
    data.polynomials.back() = Opposite(data.polynomials.front());
    data.entire_circle_unresolved = false;
    const bool boundary =
        Sign(Evaluate(data.polynomials.front().residual, -1)) == 0 ||
        Sign(Evaluate(data.polynomials.front().residual, 1)) == 0;
    data.simple_roots = 0;
    for (std::size_t chart = 0; chart < 2; ++chart) {
      auto limits = options;
      limits.max_nodes = options.max_nodes - data.nodes;
      limits.max_leaves = options.max_leaves / 2;
      const auto &polynomial = data.polynomials.at(chart);
      data.partitions.at(chart) =
          boundary ? UnresolvedBoundary(polynomial)
                   : PartitionPolynomial(polynomial.residual,
                                         polynomial.determinant, limits);
      auto &partition = data.partitions.at(chart);
      data.nodes += partition.nodes;
      data.simple_roots += partition.roots;
      for (auto &leaf : partition.leaves) {
        leaf.phase = Phase(leaf.tangent, attempt, chart != 0);
        if (!leaf.phase.IsValid()) {
          data.status = Status::ArithmeticUnavailable;
          return result;
        }
      }
    }
    if (data.partitions.front().status == Status::Complete &&
        data.partitions.back().status == Status::Complete) {
      data.status = Status::Complete;
      result.complete_ = true;
      return result;
    }
    if (data.nodes >= options.max_nodes)
      break;
  }
  data.status = Status::Inconclusive;
  return result;
}
} // namespace crx::root
