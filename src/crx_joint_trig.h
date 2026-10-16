#pragma once

#include <array>

namespace crx {

// Evaluate six independent angles before composing the dependent FK rotations.
// Plain storage keeps Eigen's ISA-dependent alignment out of the kernel ABI.
struct JointTrig {
  static constexpr int kCount = 6;
  using Values = std::array<double, kCount>;
  Values sine{};
  Values cosine{};

  explicit JointTrig(const Values &angles);
};

} // namespace crx
