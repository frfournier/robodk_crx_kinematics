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
