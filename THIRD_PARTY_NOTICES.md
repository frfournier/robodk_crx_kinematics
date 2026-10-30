# Third-Party Notices

## RoboDK API header

[`include/crx_kinematics.h`](include/crx_kinematics.h) contains the RoboDK
custom-kinematics API declarations and retains the upstream copyright and
Apache-2.0 license notice. Its SPDX metadata records `RoboDK Global` for
2015-2026 alongside this project’s copyright.

## Eigen (vendored tree based on 5.0.1)

This software uses the Eigen library, licensed under the Mozilla Public
License v2.0 (`MPL-2.0`). `EIGEN_MPL2_ONLY` restricts project compilation to
Eigen modules available under that license.

The vendored Eigen tree also contains files under other licenses. No single
license identifier is asserted for the entire tree; the upstream per-file
notices and `COPYING.*` files identify those terms.

The exact corresponding Eigen header source used to build this binary,
including any local modifications, is distributed with the binary in:

`licenses/Eigen/source/Eigen`

The upstream Eigen 5.0.1 release is available for reference from the official
repository:

https://gitlab.com/libeigen/eigen/-/tree/5.0.1

The original Eigen license and accompanying `COPYING.*` files are distributed
in `licenses/Eigen`.

The project defines `EIGEN_MPL2_ONLY` for all C++ compilation. The Eigen
sources are maintained separately under `third_party/eigen`; proprietary
project files include Eigen headers but do not incorporate Eigen source text.

Any modified Eigen files in the bundled corresponding source remain available
under their applicable upstream licenses, including MPL 2.0.
