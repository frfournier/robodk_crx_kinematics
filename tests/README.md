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

## Polynomial RoboDK comparison DLL

Windows test builds also produce `crx_kinematics_polynomial.dll`. It exposes the
same C ABI and runs polynomial discovery through the existing base/tool
conversion, command-space limits, full-turn enumeration, seed ranking and result
packing. It never calls the legacy scanner: unresolved discovery, non-asset
joint senses and FK mismatches return `-1`; numerical no-candidate results use
the existing empty-search/seed policy. The production DLL and deployment target
still use the scanner as a temporary comparison baseline before cutover.

`test_crx_polynomial_adapter.py` compares both DLLs: all 64 legacy witnesses
(including the four production defects), all 25 fixture targets, random legal
commands for all six assets, transformed base/tool frames, restricted limits,
seed ranking, capacity and reserved-field/buffer boundaries. Fixture targets are
compared by postures and independently recomputed FK, not discovery ordering.
`CRXKIN_POLYNOMIAL_LIBRARY_PATH` overrides the comparison DLL location; CTest
sets it to the matching build and fails if it is absent. The additional
`crx.polynomial-api` CTest target runs the existing API regression module against
the comparison DLL. Both gates run in the Release verification workflow.

This is an integration comparison, not production cutover or a new completeness
guarantee. Calibrated geometry and remaining continuous-family cases are still
pending. The four strict production xfails remain until production switches;
the comparison DLL must recover those same witnesses without exceptions.

## Command turns, ranking and capacity (#17)

`crx.command-lifts` tests the private `crx_command_lifts` selector against 200
independently enumerated finite catalogues, with seeded and seedless orderings
and several capacities. Further cases cover shifted limits, both midpoint ties,
the G7 coupled-metric counterexample, invalid/repeated early candidates, a late
nearest candidate after more than 32 postures, seed preservation, clamped
roundoff, count/integer/travel overflow and explicit work exhaustion.

`test_crx_command_turns.py` runs against both DLLs through the C ABI. It covers
shifted and negative turns, coupled J3 limits, negative rear-J3 commands,
perturbed multiturn seeds, a 125-command exhaustive reference, capacities below,
equal to and above the eligible count, optional alternatives, reserved zeros,
and untouched buffer sentinels on success and failure. The six-asset random API
tests now sample the full captured command box, without the former restriction
to commands also legal in decoupled coordinates. Fixture catalogue checks use
4096 output slots and assert that capacity was not exhausted; small-capacity
tests independently require the correct ranked prefix.

The policy follows RoboDK's [sample header](https://github.com/RoboDK/Plug-In-Interface/blob/master/robotextensions/samplekinematics/samplekinematics.h):
chosen output included among alternatives, 12-double slots, caller capacity,
and `-1` generic-solver handoff. Its dummy IK example does not specify turn
enumeration, ranking, or a meaning for the six reserved fields. This library
enumerates all finite admissible turns within its explicit work budget, ranks
by equal-weight squared command travel, and keeps reserved fields zero.
Seedless order and exact computed-score ties use lexicographic command order;
a preserved strict-FK-valid seed takes precedence on a score tie.

Limits and distances use coupled RoboDK commands in radians, before joint
senses; internal FK joints remain decoupled. Integer coupling and signs preserve
the full-turn lattice. Phase deduplication uses `1e-10` radians. Endpoint clamps
allow only `min(1e-10, 64*epsilon*max(1,abs(boundary)))` radians, followed by full
FK validation. The former physical 0.01-degree limit band is removed.

The selector records unique geometric and feasible postures, box-lift count,
FK-valid discovered lift count, seed inclusion/matching, returned commands and
output truncation. Each retained command carries six internal signed turn
indices. Angle/root counts remain discovery's responsibility. Checked uint64
arithmetic and exactly representable turn bounds protect enumeration; a budget
of 1,048,576 box lifts prevents unbounded work. A top-K heap bounds retained
storage, while every lift is visited before success. Range/work failures expose
no partial results and map to `-1`, independently of output truncation.

The legacy scanner's raw 32-hit stop is removed: its 960 intervals bound the
number of raw candidates instead. Neither the geometric regular-case bound nor
configuration flags bound the number of commands. Continuous-family coverage,
global optimality, and the legacy approximate empty-search fallback remain
outside this command-selection guarantee. The selector uses dynamic storage;
the canonical allocation guards do not establish allocation-free ABI calls.

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

Near tangency, the wrist plane supplies a stable signed elbow height instead
of subtracting nearly equal squared arm lengths and taking a square root.
On the base axis, the horizontal arm circle is intersected with the wrist plane.
Coincident arm spheres and undetermined circles still return `NeedsRefinement`,
with no partial list. `NoCandidate` concerns only the supplied wrist point.

Unit-axis tolerance is `128*epsilon(double)`, small-direction tolerance is
`256*epsilon(double)` times the largest arm/wrist length, and relative residual
tolerance is `512*epsilon(double)`. Sphere checks scale by squared lengths;
wrist perpendicularity scales by the largest arm/wrist length. An eightfold
compatibility band retains roundoff-sized candidates for strict full-FK checks.
These are roundoff heuristics, not physical calibration tolerances or certificates.

The earlier SVD implementation and its broader singular/circle cases live in
`tests/crx_incidence_reference.*` and `test_crx_incidence_reference.cpp`. CMake
links them only into the test executable, never the library or qmake target.
They provide an independent algebraic comparison, not production feature scope.
Its provisional SVD ranks and heuristic classifications are not exact oracles.
The scalar-zero counterexample is rejected by both implementations.
The direct path recovers all 128 synthetic canonical FK
witnesses, including sample 55 that the SVD may leave unresolved. Two-branch,
ambiguity, tangency, invalid-input and range checks also run under the existing
Eigen/C++ allocation guards in `crx.canonical`.

## Canonical joint recovery and full-pose validation

`src/crx_joint_recovery.h/.cpp` recover joint candidates for one supplied
wrist-circle angle or unit sine/cosine pair. They prepare the normalized circle,
reconstruct elbows,
and recover both base-angle branches using appendix G6. The result holds at
most four canonical postures (two elbows, two bases), with each joint in
`[-pi, pi]`. There is no seed input, division by `sin(q5)`, or singularity-based
override of J3/J6. A vertical wrist uses its recovered elbow to determine the
base plane. When both arm points lie on the base axis, base=0 and its opposite
are deterministic representatives, not enumeration of the continuous family.

Recovery carries normalized sine/cosine pairs through scalar vector rotations,
then extracts the joint angles. J6 uses two target-axis dot products instead of
constructing a full wrist matrix. The independent FK check below still rebuilds
the pose from the emitted angles. Tests cover each joint's quadrants and branch
cuts, equivalent angle/coordinate inputs, and rejection of invalid unit pairs.

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

This component consumes a wrist-circle point; it does not discover or polish
polynomial roots itself. The discovery stage below supplies those points.

## Target-only polynomial discovery

`src/crx_discovery.h` declares discovery's options, result and entry point;
`src/crx_discovery.cpp` implements them and keeps polishing/boundary helpers
private. Recovery's header declares only recovery, without a root-solver
dependency. Ordinary math, pose and vector helpers likewise use matching
headers and `.cpp` files together in `src/`; `include/` contains only the public
RoboDK C ABI header. Only compile-time constants and the two `constexpr`
degree/radian conversions keep definitions in headers.

`DiscoverJointCandidates` connects the existing
coefficient, eigenvalue and joint-recovery components. It takes nominal canonical
lengths, a target pose and physical pose tolerances, with no generating angle or
joint seed. The model/coordinate oracle remains in Python; no new C++ robot-model
bridge or public ABI is introduced. Production `SolveIK` still uses the scanner.

The usual half-angle chart is zero; nearly folded arms start with a chart
centered on the wrist-circle point closest to the base origin. Up to five
bounded chart attempts address numerical conditioning; coefficients are never
trimmed. Small endpoint distances and equal-arm differences are formed without
cancelling large squared terms. Each chart's omitted point is checked separately.
Near-real complex projections and small slopes are starting guesses for original
geometry, not automatic rejection or acceptance. Analytic crossings of the
horizontal wrist axis, wrist plane and arm-shell boundaries retain multiple and
tangent roots. These are numerical methods, not formal root certification.
Boundary crossings solve `A*cos(theta)+B*sin(theta)=C` directly in circle
coordinates with a scaled discriminant. Tangencies nominate one point, disjoint
constraints nominate none, and a zero normal supplies no isolated crossing.
Roundoff-sized negative discriminants can nominate a tangent point for the
unchanged geometric/FK checks. Boundary recovery avoids inverse-trigonometric
round trips; an angle is formed only when an unresolved root needs a proximity
check against a boundary point.

Real candidates are polished against the unsquared elbow/wrist constraint with
an analytic derivative, at most 24 iterations by default (64 maximum), and eight
backtracking attempts per iteration. Movement stays within 0.05 radians; regular
roots also use a quarter of neighboring angular separation. Ill-conditioned
clusters retain the larger bounded polishing window. Elbow selection uses the
Newton correction instead of residual size alone, avoiding a flat incompatible
branch. A smooth signed-height sphere residual handles tangency. Acceptance uses
arm/wrist rechecks and independent full-pose FK calculation; tolerances are not
widened. Postures within `1e-10` radians in every joint modulo full turns are
deduplicated. The fixed buffer holds at most 36 numerical candidates; overflow
returns an explicit unresolved status instead of truncating or overrunning it.

`Candidates` and `NoCandidate` are numerical discovery outcomes, not proofs of
coverage or infeasibility. Ambiguity, exhausted work and exceptional incidence
return `NeedsRefinement` with no partial list. No fallback is implemented here.
The comparison DLL supplies RoboDK mapping and existing finalization. Revised
turn/limit semantics and production cutover remain pending.

Validation uses both the allocation-guarded native harness and the test-only
`--discover` probe, which receives only lengths/target/tolerances. Coverage:

- 128 synthetic generating postures from targets alone, signed/scaled geometry,
  half-angle endpoints, and a control proving geometric polishing is necessary.
- All 96 captured mixed-joint asset targets, checked by the Python FK oracle.
- All 64 legacy discovery witnesses, including 8, 36, 42 and 57, without seeds.
  Their production xfails remain until the new path is actually integrated.
- All recorded postures for ALL8, ONLY7, ABBES-TABLE4, ABBES-TABLE6 and
  WRIST_SING_NEAR_RG_LIMIT. ABBES-TABLE6 returns 16 distinct postures. ONLY7
  returns eight before command limits, as expected for this stage.
- All 25 fixture targets now require candidates without a scanner. The rounded
  MIN_Z target disagrees with its recorded posture by 0.00425 mm; exact target
  results are checked by strict independent FK, and a separate exact-FK target
  requires recovery of the recorded generating posture. Fixture data is unchanged.
- Straight/near-straight arms across all six assets, perturbed multiple roots,
  near-chart endpoints, and nearly folded arms are passing witness requirements.
- Zero wrist sine with nonzero J6, no-candidate versus zero-polynomial handoff,
  separate tight pose-tolerance failures, and missing/duplicate mutation checks.

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
implemented or validated. Target-only discovery now reaches validated canonical
postures, with coordinate evidence in the Python oracle. The immediate priorities
are command limits/turns, remaining singular families, and production cutover.

## Asset-coordinate oracle

`fixtures/crx_asset_frames.json` captures all six approved assets through RoboDK
6.0.6's nominal `JointPoses`, `SolveFK`, and `JointLimits` APIs. Each record keeps
the asset SHA-256, zero frames, six single-command probes, and 16 deterministic
multi-joint samples. Dimensions are derived from those frames, not a model-name
table. The `.robot` files and the solution CSV/JSON are unchanged.

The extraction, modified-DH derivation, screw-axis/home-transform comparisons,
and canonical FK oracle live in Python (`crx_asset_reference.py` and
`test_crx_asset_bridge.py`). They add no C++ model abstraction, runtime model
recognizer, or public API. For these six observed assets, canonical `q` equals
the RoboDK command vector in radians; the current C ABI's internal decoupled
coordinate satisfies `q3 = user2 + user3`. The base includes DH shoulder height
once, and the canonical flange convention requires no extra rotation for the
captured assets. Every captured link/flange is also compared with the existing
`SolveFK_CAD` callback. CAD FK bypasses limits, so this is not evidence for the
still-pending command/decoupled limit and turn-selection policy.

Regenerate the capture explicitly with:

```powershell
uv run python tests/capture_crx_assets.py --robodk-path 'C:/Program Files/RoboDK/bin/RoboDK.exe'
uv run pytest tests/test_crx_asset_bridge.py
```

The collector opens its own hidden, unsaved RoboDK instance, selects nominal
accuracy, and closes it afterward. It does not deploy a DLL, save assets, or
connect to hardware. API reference:
[RoboDK JointPoses](https://robodk.com/doc/en/PythonAPI/robodk.html#robodk.robolink.Item.JointPoses).
The installed custom DLL can participate in these API observations; its hash is
recorded, and bypass is explicitly **not** claimed. The independent Python
transform products check consistency of the observed nominal chain, not an
independent complete IK catalogue or end-to-end new-solver support. A capture
with the extension bypassed remains desirable before production cutover.

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
