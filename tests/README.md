# Tests and fixtures

The test suite covers the native C ABI, fixture-based forward and inverse
kinematics, version metadata, and live RoboDK integration.

- `test_crx_kinematics.py` exercises the compiled library directly through
  `ctypes`.
- `test_crx_robolink_kinematics.py` validates the library through a live RoboDK
  instance and the Python RoboDK API.
- `test_version_metadata.py` checks that release versions agree across build
  and package metadata.

Run the suite from the repository root after building the library:

```powershell
uv sync --locked
uv run --locked pytest
```

Set `CRXKIN_LIBRARY_PATH` when the library is not in the default
`build/Release` location.

Live RoboDK tests are opt-in. After explicitly deploying the current Release DLL
using the installation instructions, run `uv run --locked pytest --run-robodk`.
The Windows integration setup checks that the deployed DLL matches the build and
skips on a missing or stale deployment; tests never copy files into RoboDK.
The default suite and CTest release gate skip live tests and run native checks.

## Fixture provenance

The parametrized CRX-10iA cases combine legacy regression observations from:

1. FANUC RoboGuide simulation results.
2. M. Abbes and G. Poisson,
   [“Geometric Approach for Inverse Kinematics of the FANUC CRX Collaborative Robot”](https://doi.org/10.3390/robotics13060091).

These observations and the callback model in `conftest.py` have not yet been
reconciled with the approved assets. They do not override asset geometry or
limits, or establish family support. `crx_assets.py` records the six authoritative
robot files and their Git LFS SHA-256 IDs; offline tests verify the bytes and live
configuration smoke tests use the same inventory. RoboDK model extraction and
canonical coordinate validation remain pending under roadmap milestone #19 A.

## Initial discovery gate

`test_crx_discovery.py` starts roadmap package 1 (#2). Its 64 deterministic
targets come from an independent NumPy product of elementary modified-DH
transforms in `crx_reference.py`, using the legacy callback model. No native FK
call or generating seed constructs or rescues these discovery targets. Each
generating posture is a known witness; this is not a complete branch catalogue.
Every returned pose is checked with the independent reference. A deletion test
checks that the posture comparator detects a missing branch and compares modulo
turns rather than configuration flags. Separate tests exercise selection with
unrelated seeds and demonstrate generating-seed masking.

Probes 8, 36, 42, and 57 reproduce missing witnesses against the pre-change DLL.
They are explicit strict expected failures for the missing-witness assertion
only. Invalid returned poses, no-solution results, and new missing witnesses
fail normally. When a recorded witness is recovered, strict XPASS requires
removing its exception. The underlying discovery/finalization cause remains to
be isolated; the tests do not attribute all losses to the angle scanner.

These four cases are required regression targets for the new implementation:
recover the generating posture without a seed, preserve independent FK checks,
and remove each strict expected-failure mark once production integration fixes
it. They are not accepted permanent limitations or grounds to weaken assertions.

The same reference checks all seven CAD frames, including synthetic base/tool
transforms and joint senses, short-buffer rejection, and trailing-buffer
sentinels. These are derived tests, not additional supported robots.

The first native storage change replaces the seven-frame and six-index vectors
with call-local fixed arrays. It preserves the C ABI and existing IK policy.
Whole-solver allocation instrumentation, stage counters (roots, geometric
postures, feasible postures, command lifts, returned rows), the asset coordinate
bridge, polynomial discovery, and asset-backed reference catalogues remain pending.
No timing or allocation guarantee follows from these regression checks.

Run this gate after building with:

```powershell
uv run --locked pytest tests/test_crx_discovery.py
```

## Canonical residual and coefficient kernel

`src/crx_canonical.h` and `src/crx_canonical.cpp` implement appendix G2/G4 of
[`notes/crx-implementation-review.md`](../notes/crx-implementation-review.md).
This internal component is compiled and tested, but production `SolveIK` does
not call it yet. Inputs must already use the canonical frame and one consistent
length unit; the six-asset adapter is still required before production use.

The kernel normalizes lengths and translation by
`L = max(abs(a), abs(b), abs(c), abs(r))`, accepts signed lengths with nonzero
`a,b`, and forms the vector residual and nine ascending half-angle coefficients
using bounded scalar convolution. It supports shifted charts, evaluates the
omitted point separately, and includes Horner value/derivative evaluation.
`RepresentedDegree` examines the stored coefficients without trimming tiny
terms; a stored zero polynomial is an unresolved geometric case, not evidence
of a feasible continuous family. Residual units are physical length to the sixth
power divided by `L^6`; production FK tolerances are unchanged.

The preparation policy rejects nonfinite inputs, zero arm lengths, non-affine
bottom rows, and rotations whose maximum Gram-matrix or determinant error exceeds
`128 * epsilon(double)`. Accepted rotation roundoff is retained without projection.
Normalization underflow to zero and detected nonfinite intermediate/output
values produce a numerical range failure. Successful preparation and coefficient
construction do not establish feasibility, exact degree, or a coefficient error
enclosure. Boundary/rank decisions, root isolation, shell arcs, and exceptional
incidence reconstruction remain pending.

CTest's `crx.canonical` target exercises 128 synthetic target/witness pairs with
four charts, comparing the vector residual, compact expression, triangle identity,
and polynomial evaluation. It also covers reciprocal-chart evaluation, canonical
chain witnesses, signed uniform scales from `1e-250` to `1e250`, degree-zero and
degree-four examples, zero polynomials, tiny leading coefficients, all represented
degrees, derivatives, invalid poses, and overflow/underflow status handling.
These are numerical derived tests, not additional robot models or certification.

The standalone native harness compiles the same kernel with
`EIGEN_RUNTIME_NO_MALLOC` and assertions enabled even in Release. It guards initial
construction, chart/geometry changes, and error paths, restoring the previous
Eigen allocation setting afterward. Test-only scalar, array, and aligned C++
allocation replacements count calls on the current thread; positive controls
check that interception is active. Passing demonstrates no observed C++ or
guarded Eigen allocations in these tests. Direct CRT allocations, whole-solver
allocations, concurrent stack use, and timing have not been measured.

Run the narrow native gate with the configured Windows toolchain:

```powershell
cmake --preset windows-clang-release-tidy
cmake --build --preset windows-clang-release-tidy --target crx_canonical_tests
ctest --preset windows-clang-release-tidy -R '^crx.canonical$' --output-on-failure
```

The full `windows-clang-release-verify` workflow runs both this native gate and
the existing Python regression suite. The qmake source list includes the kernel;
the native CTest harness is provided by the default CMake build.

## Polynomial root candidates and exact reference

`src/crx_polynomial_roots.h/.cpp` add an internal candidate backend for the
stored ascending polynomial coefficients. Degree one uses scalar division;
degrees two through eight use fixed-size companion matrices and supported
`Eigen::EigenSolver`, computing eigenvalues only. This avoids coupling to
`unsupported/Eigen/Polynomials` and keeps balancing, iteration budgets, and
failure reporting under local control. Production IK still uses the scanner.

Power-of-two coefficient normalization and diagonal similarity balancing reject
detected overflow or underflow to zero. Balancing defaults to at most 32 sweeps;
the eigensolver receives a 256-iteration limit. Separate statuses identify
invalid input, numerical range failure, each work limit, constant and zero
polynomials, and unresolved leading coefficients. The default leading-term
heuristic is `abs(leading)/max(abs(coefficients)) < 64*epsilon(double)`; it never
trims the coefficient or claims a lower geometric degree. A zero candidate count
must be interpreted with its status. Even a constant finite-chart polynomial
still requires checking the chart's separately stored omitted point.

Candidates retain all complex eigenvalues, including small imaginary parts and
numerical splitting of repeated roots. Each carries the componentwise residual
`abs(P(z))/sum(abs(c[k])*abs(z)^k)`, evaluated using the reciprocal
polynomial for `abs(z)>1` to avoid large powers. This diagnostic has no acceptance
threshold and does not establish reality, multiplicity, completeness, or geometric
feasibility.
The native harness guards first construction of every degree, scale and degree
changes, zero/repeated and complex roots, and numerical/work-limit paths against
C++ and Eigen allocations. These checks retain the allocation scope limits above.

`polynomial_reference.py` is an offline reference using standard-library
`Fraction`, exact square-free factorization, and Sturm sign counts. It returns
rational isolating intervals and multiplicities, or distinct zero-polynomial and
subdivision-limit exceptions. It certifies the supplied rational polynomial
only: converting a float to `Fraction` does not recover uncertain geometric
coefficients. Its subdivision budget does not bound rational arithmetic cost.

`test_polynomial_roots.py` checks known rational and irrational roots, close pairs,
repeated roots, complex-only and mixed cases, degree drops, and signed coefficient
scales. A test-only JSON probe in `crx_canonical_tests --roots c0 ... c8` enables
one-to-one native comparisons and missing/duplicate-candidate mutation checks.
CTest sets `CRXKIN_ROOT_PROBE_PATH` to its build's executable; direct pytest uses
that variable or `build/Release/crx_canonical_tests.exe`, skipping native probe
tests if the default executable is absent. Run the narrow Python gate after build:

```powershell
uv run --locked pytest tests/test_polynomial_roots.py
```

This partially addresses roadmap #8/#9/#11. Numerical acceptance policy,
the six-asset bridge, latency measurements and production integration remain
open. Formal root certification is deferred under the CRX scope decision below.

## Fixed-angle elbow incidence

`src/crx_incidence.h/.cpp` implement a direct nominal-CRX elbow reconstruction.
For normalized signed arm lengths `a,b`, wrist point `x`, and unit fifth axis `u`,
the two arm spheres intersect the vertical base plane in at most two elbows.
The implementation constructs both, then checks both arm lengths and the
unsquared wrist constraint `(y-x).u=0`. It has no SVD, numerical rank API, or
circle descriptor. This remains an internal candidate component; production IK
still uses the existing scanner.

Vertical/near-origin wrists, tangencies, ambiguous compatibility and failed
geometric rechecks return `NeedsRefinement`, with no partial candidate list.
That status is a handoff requirement, not an implemented refinement solver or
an unreachable-target result. `NoCandidate` is restricted to a clear numerical
rejection at this wrist point. The joint recovery component below performs the
next canonical step; integration into production IK remains pending.

Unit-axis tolerance is `128*epsilon(double)`, small-direction tolerance is
`256*epsilon(double)` times the largest arm/wrist length, and relative residual
tolerance is `512*epsilon(double)`. Sphere checks scale by squared lengths;
wrist perpendicularity scales by the largest arm/wrist length. An eightfold
ambiguity band defers marginal branch decisions without expanding acceptance.
These are roundoff heuristics, not physical calibration tolerances or certificates.

The earlier SVD implementation and its broader singular/circle cases live in
`tests/crx_incidence_reference.*` and `test_crx_incidence_reference.cpp`. CMake
links them only into the test executable, never the library or qmake target.
They provide an independent algebraic comparison, not production feature scope.
Its provisional SVD ranks and heuristic classifications are not exact oracles.
The scalar-zero counterexample is rejected by the reference and safely deferred
by the direct path. The direct path recovers all 128 synthetic canonical FK
witnesses, including sample 55 that the SVD may leave unresolved. Two-branch,
ambiguity, tangency, invalid-input and range checks also run under the existing
Eigen/C++ allocation guards in `crx.canonical`.

## Canonical joint recovery and full-pose validation

`src/crx_joint_recovery.h/.cpp` recover joint candidates for one supplied
wrist-circle angle. They prepare the normalized circle, reconstruct elbows,
and recover both base-angle branches using appendix G6. The result holds at
most four canonical postures (two elbows, two bases), with each joint in
`[-pi, pi]`. There is no seed input, division by `sin(q5)`, or singularity-based
override of J3/J6. Vertical, tangent and ambiguous cases keep the existing
`NeedsRefinement` handoff; no numerical fallback is implied.

Every recovered posture is checked by recomputing the complete canonical FK
from its joint vector. Callers must supply a positive position tolerance in
the target's physical length unit and an orientation tolerance in radians.
Position error is computed in normalized coordinates then rescaled; orientation
uses the rotation-matrix chordal distance converted to an angle, preserving
resolution near zero. Results include both errors. A failed reconstruction or
pose check publishes no partial list and does not widen the supplied tolerances.

The native allocation harness checks 128 generating postures and their base
flips against an independent homogeneous-transform FK implementation. Additional
cases cover four-posture output, duplicate prevention, periodic wrist angles,
signed lengths, length scales `1e-200` and `1e200`, and `q5=0`, near zero and
`+/-pi` with a nonzero J6. Position and orientation gates are independently
tested with tight tolerances, alongside invalid inputs and refinement/range
handoffs. These are synthetic nominal-CRX tests, not validation of the six
asset-coordinate mappings or calibrated robots.

This component consumes an angle; it does not discover or polish polynomial
roots. Asset/command-coordinate conversion, limits, turn selection, refinement
and production integration remain pending. The public C ABI is unchanged.

## CRX scope and calibration boundary

Support is limited to the six approved CRX assets. The coordinate bridge and
their reachable/limited configurations determine production requirements;
synthetic reference cases do not add supported robot models. General continuous
family enumeration and formal completeness certification are deferred. A clear
refinement/failure outcome remains necessary for unsupported configurations.

Joint zero offsets, base/tool frames and changes to the canonical lengths can
preserve the nominal geometry when incorporated correctly into the coordinate
bridge. Calibration of axis tilts or other offsets that break the assumed
parallelism/intersections can invalidate the polynomial as well as incidence.
Increasing incidence tolerances does not establish support for those models.

The intended calibrated path uses nominal CRX candidates as initial guesses for
bounded numerical correction against the actual calibrated FK, followed by
position/orientation and command-limit validation. It must retain genuine
calibration parameters rather than snap them to nominal geometry. That path,
its physical acceptance tolerances, and its singular-case behavior are not yet
implemented or validated. Canonical joint recovery and FK checks now have a
tested component; the immediate priorities are the asset-coordinate bridge and
connecting root discovery to validated postures, rather than more general
incidence classification.

## Fixture source

The committed CSV is the editable source of truth. The JSON is the generated
representation loaded by pytest:

```text
fixtures/CRX10iA-solutions.csv
              |
              v
fixtures/compile_fixtures.py
              |
              v
fixtures/CRX10iA-solutions.json
              |
              v
parametrized FK, IK, configuration, and RoboDK integration tests
```

## CSV schema

Each row in `fixtures/CRX10iA-solutions.csv` represents one joint solution.
Rows with the same `TEST CASE` share a target pose.

| Column | Description |
| --- | --- |
| `TEST CASE` | Test-case identifier used to group solutions |
| `SOLUTION ID` | Solution identifier within the test case |
| `TEST NAME` | Human-readable case name |
| `X`, `Y`, `Z` | Target TCP position in millimetres |
| `W`, `P`, `R` | Target TCP orientation in degrees |
| `J1`–`J6` | Joint angles in degrees |
| `FB`, `UD`, `TB` | Front/back, up/down, and turn/flip configuration letters |

The fixture configuration maps to RoboDK's `[REAR, LOWERARM, FLIP]` result as
follows:

| Fixture | RoboDK flag | False value | True value |
| --- | --- | --- | --- |
| `TB` | `REAR` | `T` | `B` |
| `UD` | `LOWERARM` | `U` | `D` |
| `FB` | `FLIP` | `N` | `F` |

## Generated JSON

`compile_fixtures.py` groups CSV rows by test case and stores the shared target
pose once:

```json
{
  "meta": {
    "robot_model": "CRX10iA",
    "units": {
      "position": "mm",
      "orientation": "deg",
      "joints": "deg"
    }
  },
  "test_cases": [
    {
      "id": 1,
      "name": "HOME",
      "target": {
        "xyz_mm": [540.0, -150.0, 380.0],
        "wpr_deg": [-180.0, 0.0, 0.0]
      },
      "solutions": [
        {
          "id": 1,
          "joints_deg": [0.0, 0.0, 0.0, 0.0, -90.0, 0.0],
          "config": {
            "FB": "N",
            "UD": "U",
            "TB": "T"
          }
        }
      ]
    }
  ]
}
```

## Regenerating fixtures

After editing the CSV, regenerate the JSON from the repository root:

```powershell
uv run python tests\fixtures\compile_fixtures.py
```

Review the generated JSON diff and run the fixture tests:

```powershell
uv run pytest tests\test_crx_kinematics.py -v
```

Fixture changes are intentional test-data changes and should be called out in
the release notes or change description.
