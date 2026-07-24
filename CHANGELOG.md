# Release history

Release details are derived from the repository's commit history. Releases are
not tagged, so abbreviated commit references are provided for traceability.

## 0.3.1

- Modernized the development container from the newer RoboDK Docker interface,
  using the latest upstream RoboDK Linux installer at build time.
- Migrated the Python integration and test environment to the RoboDK API 6.0.1
  package while retaining RoboDK 6.0.6.26901 as the minimum supported native
  custom-kinematics host.
- Adopted the Apache License 2.0 and included the project license, third-party
  notices, and Eigen licensing materials beside packaged libraries.

## 0.3.0

- Added computation of the RoboDK kinematic configuration flags from a joint
  position: rear/front, lower/upper arm, and flip/non-flip (`bcc4ac7`,
  `ffa1c02`).
- Adopted the RoboDK 6.0.6 custom-kinematics ABI and exposed configuration
  results through `Joints2Config` as `[REAR, LOWERARM, FLIP]`
  (`16b3728`).
- Added fixture-based configuration coverage, including coupled joints,
  singular configurations, input validation, and preservation of IK branch
  behavior (`e8901cc`, `396fb05`, `f1a2d2b`).
- Added RoboDK integration tests that verify the returned configuration flags
  through the live RoboDK API (`cc311e1`).
- Documented RoboDK 6.0.6 as the minimum supported version (`672f5a5`).
- Decoupled the kinematics core from the RoboDK ABI and modernized the C++17,
  clang-tidy, formatting, CMake preset, and optimized Windows build workflows
  (`d65a340`, `054122c`, `67cc7e2`, `462df45`).

## 0.2.0

- Updated the RoboDK interoperability layer to read the robot name from the
  robot parameter block (`712246f`).
- Exposed `iRobot_Name` in the interoperability header and included the robot
  name in adapter validation warnings (`1a15594`).
- Improved silent installation for RoboDK 6 in the Docker environment
  (`29a9285`).

## 0.1.0

- Initial RoboDK 6 custom-kinematics library for the Fanuc CRX-10iA, providing
  forward kinematics, inverse kinematics, and CAD joint poses through a C ABI
  (`c935d9a`).
- Added conversion between RoboDK robot data and the solver model, including
  DH parameters, base and tool transforms, joint senses, joint limits, and
  coupled-joint handling (`c935d9a`, `146b87f`).
- Added numerical safeguards around angle wrapping, cosine limits, and
  workspace boundaries, along with solver performance improvements
  (`04d3f51`, `1995ba2`, `9837657`, `d9e4148`, `4d766d7`).
- Added pytest regression coverage using CRX-10iA reference fixtures,
  singularity and coupling cases, and FK/IK round-trip testing (`f86be14`,
  `0d8f774`, `b72f4f3`).
- Added Windows CMake and qmake build support, Eigen integration, and RoboDK
  robot and station assets for testing (`7717476`, `d862b82`, `73b2b0c`,
  `22affee`, `6f475aa`, `99e9e7f`).
