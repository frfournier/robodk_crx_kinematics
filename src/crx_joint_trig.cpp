#include "crx_joint_trig.h"

#include <array>
#include <cmath>
#include <cstddef>

#include <Eigen/Core>
#include <Eigen/src/Core/arch/Default/GenericPacketMathTrig.h>

namespace crx {

JointTrig::JointTrig(const Values &angles) {
  // Keep Eigen packet types and their AVX alignment inside this translation
  // unit. Only plain doubles cross into the baseline-ISA solver.
  constexpr int kPacketSize = 4;
  constexpr int kBatchSize = 2 * kPacketSize;
  constexpr double kSmallAngleBound = 15.0;
  Eigen::Array<double, kBatchSize, 1> batch =
      Eigen::Array<double, kBatchSize, 1>::Zero();
  batch.head<kCount>() =
      Eigen::Map<const Eigen::Array<double, kCount, 1>>(angles.data());
  // Use the bounded argument reduction for ordinary joint commands. Preserve
  // the platform math library for large turns and exceptional inputs.
  if ((batch.abs() < kSmallAngleBound).all()) {
    // Eigen's interleaved packet evaluates sin(a), cos(a), sin(b), cos(b)
    // together. Separate Array::sin()/cos() calls repeat the same reduction
    // and both approximating polynomials. Keep this Eigen-internal dependency
    // here, covered by the scalar-reference tests when updating Eigen.
    for (std::size_t joint = 0; joint < sine.size(); joint += 2) {
      const auto packet = Eigen::internal::ploaddup<Eigen::internal::Packet4d>(
          &angles.at(joint));
      const auto values = Eigen::internal::psincos_selector(packet);
      std::array<double, kPacketSize> pair{};
      Eigen::internal::pstoreu(pair.data(), values);
      sine.at(joint) = pair.at(0);
      cosine.at(joint) = pair.at(1);
      sine.at(joint + 1) = pair.at(2);
      cosine.at(joint + 1) = pair.back();
    }
    return;
  }
  for (std::size_t joint = 0; joint < sine.size(); ++joint) {
    sine.at(joint) = std::sin(angles.at(joint));
    cosine.at(joint) = std::cos(angles.at(joint));
  }
}

} // namespace crx
