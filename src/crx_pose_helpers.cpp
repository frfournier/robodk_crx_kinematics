#include "crx_pose_helpers.h"

#include "crx_types.h"

namespace crx {

auto DHM_FromRad(Scalar alpha, Scalar a, Scalar theta, Scalar d) -> PoseIsoRT {
  // Modified DH: Tx(a) Rx(alpha) Tz(d) Rz(theta).
  PoseIsoRT transform(Eigen::Translation<Scalar, 3>(a, 0.0, 0.0));
  transform.rotate(Eigen::AngleAxis<Scalar>(alpha, Vec3::UnitX()));
  transform.translate(Vec3(0.0, 0.0, d));
  transform.rotate(Eigen::AngleAxis<Scalar>(theta, Vec3::UnitZ()));
  return transform;
}

} // namespace crx
