# Release history

Release details are recorded below, with GitHub release tags and abbreviated
commit references for traceability.

## 0.4.4

- Merge near-identical IK output postures within 0.0001 degree per joint,
  including numerical duplicates at the all-zero home pose. Preserve distinct
  command turns and legal joint-limit endpoints. Root discovery, joint-limit
  slack and FK acceptance tolerances are unchanged.
- Added regressions for singular home poses, exact seeds, ranked prefixes and
  boundary turns; verified one near-zero home solution on all six CRX assets
  through live RoboDK callbacks.

## 0.4.3

- Specialized FK and CAD frame evaluation for validated nominal CRX geometry,
  with consistent RoboDK fallback across FK, IK and configuration callbacks.
- Standardized internal joint vectors on coupled command radians, removing
  repeated J2/J3 conversions during discovery, limits, ranking and validation.
- Centralized base-branch symmetry in joint recovery, completing refined and
  constrained postures before command finalization.
- Added regressions for CRX model rejection, CAD frames and branch coverage.
- Updated fixture asset names and hash provenance for all CRX robots currently
  supported by RoboDK.
- Checked ranked prefixes exactly within the new solver and complete command
  catalogues against 0.4.2. Roundoff-equivalent singular representatives can
  merge or change lexicographic order.
- Benchmarked against `v0.3.1` (`2a00385`) using the Release configuration with
  clang-cl 22 on Windows 11 x64 / Core i7-12700H. Mean native callback speedups
  on common successful cases:

  - 1.47x for FK.
  - 1.38x for seeded eight-solution IK.
  - 2.69x for seeded single-solution IK.

  These timings exclude RoboDK process/API overhead.

- Seeded eight-solution IK passes at 100% on 3,929 reachable targets versus
  87.8% in 0.3.1. The tradeoff was a higher p95 across common successful poses:
  51.5 to 89.7 microseconds.

## 0.4.2

- Added AVX2/FMA batching for FK trigonometry in CMake and qmake builds,
  requiring an x86-64 CPU with AVX2 and FMA.
- Reduced IK calculations by reusing wrist geometry, refinement residuals,
  and pose-check rotations.
- Optimized polynomial coefficient construction by reusing symmetric products
  and specializing multiplication by the half-angle denominator.
- Enabled link-time optimization for CMake Release builds.
- Added a Linux Release preset and removed host-specific `-march=native` tuning.
- Made Eigen selection automatic and removed its build option.
- Fixed Linux C ABI exports and Eigen source copying on repeated qmake builds.
- Updated fixture regeneration checks to allow platform math-library roundoff
  and added regression coverage for polynomial coefficients near folded poses.

## 0.4.1

- Added a validated nearby-seed fast path for `max_solutions == 1`, with
  full-search fallback.
- Changed command-turn enumeration to follow ranking order and stop after
  finding enough valid outputs.
- Added J3/J6 joint-limit pruning of algebraic nominations, with unfiltered
  discovery fallback.
- Added independent FK, joint-limit, ranking, fallback and periodic-turn
  regressions.

## 0.4.0

- Integrated polynomial IK discovery and geometric refinement as the production
  solver, preserving the public C ABI (`4e74e9a`).
- Improved singularity and boundary recovery, command-turn handling, and solver
  throughput. Difficult singularities retain a measured latency tradeoff;
- Added independent geometry, scale, accuracy and allocation regressions, with
  comparisons against an external baseline snapshot.
- Added frozen stress corpora and captured asset references; corrected the ABBES-TABLE6 solution-4 configuration labels in the recorded fixtures.

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
