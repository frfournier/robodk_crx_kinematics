# Testing

From the repository root, run the full release gate:

```powershell
scripts\build_crx_kinematics_msvc.bat --verify
```

This discovers the Windows toolchain and runs formatting, build, clang-tidy,
and CTest. In an x64 Visual Studio Developer PowerShell, the equivalent is
`cmake --workflow --preset windows-clang-release-verify`.

For focused work after building:

```powershell
uv run --locked pytest tests/test_crx_kinematics.py
uv run --locked pytest tests/test_crx_discovery.py
uv run --locked pytest
```

Direct pytest uses `build/Release/` by default and skips unavailable native
binaries. CTest supplies paths to its matching build. Use these environment
variables to select another build:

| Variable | Binary |
| --- | --- |
| `CRXKIN_LIBRARY_PATH` | Production `crx_kinematics.dll` |
| `CRXKIN_BASELINE_LIBRARY_PATH` | Optional external baseline DLL |
| `CRXKIN_ROOT_PROBE_PATH` | `crx_canonical_tests.exe` |

## Where tests live

| Files | Coverage |
| --- | --- |
| `test_crx_kinematics.py` | Recorded FK/IK poses, configurations, C ABI, seed policy and continuity |
| `test_crx_discovery.py`, `test_crx_adapter.py` | Seed-free witnesses, external baseline coverage and asset adaptation |
| `test_baseline_comparison.py` | Comparison-report detection of gained/lost coverage and invalid commands |
| `test_crx_command_turns.py`, `test_crx_solver_outcomes.py`, `test_crx_numerics.py` | Shared command limits, ranking, buffers, fallback and numerical guards |
| `test_crx_solver_fixtures.py`, `test_crx_solver_family.py`, `test_crx_accuracy_stress.py` | Frozen corpus, independent holdouts and singularity/scale regressions |
| `test_polynomial_roots.py`, `test_crx_polynomial_discovery.py` | Native probe compared with independent root/geometry references |
| `test_crx_asset_bridge.py` | Captured asset frames and model mapping |
| `test_crx_robolink_kinematics.py` | Opt-in live integration and offline deployment-guard checks |
| `test_version_metadata.py` | Release metadata consistency |
| `test_*.cpp` | Internal invariants, metadata, option budgets and allocation guards; run through CTest |

Shared support has three roles:

- `conftest.py` owns DLL loading, the `kinematics_lib` fixture, the
  `canonical_probe` runner and the CRX-10iA robot fixture.
- `crx_test_support.py` owns robot-table/C ABI plumbing;
  it also shares the generated-pose assertions between corpus and holdout tests.
  `crx_fixture_support.py` loads recorded fixtures once and supplies pose
  comparisons for native and live tests. FK/configuration test each recorded
  solution; IK solves each target once and checks every expected solution.
- `crx_reference.py`, `crx_asset_reference.py`, `crx_incidence_reference.*` and
  `polynomial_reference.py` provide independent oracles. Keep them independent
  of production FK/IK.

Prefer Python for observable solver behavior. Root catalogues, seed thresholds,
turn enumeration, ranking and omitted-point discovery are checked there rather
than mirrored in C++. The native root/discovery probes still enforce allocation
guards on every Python-driven call. Keep native tests for internal metadata,
configurable work limits, exact floating-point boundaries and private kernels
that the C ABI cannot expose.

## Reliability and live integration

The generated solver corpus must pass without expected-failure exceptions.
The release reliability gate requires a built DLL, so missing binaries cannot
turn this gate into skipped tests. The independent scaled stress suite requires
every reachable target to solve within the same strict pose tolerances.

```powershell
uv run --locked pytest tests/test_crx_solver_fixtures.py --require-solver-reliability
```

Live RoboDK tests require an explicitly deployed matching DLL. They never
copy files into RoboDK. After deploying using the root README instructions:

```powershell
uv run --locked pytest tests/test_crx_robolink_kinematics.py --run-robodk
```

## Fixtures and investigation tools

`fixtures/CRX10iA-solutions.csv` is the editable source for recorded poses.
Each row describes one solution; rows sharing `TEST CASE` share a target.
Positions are millimetres and angles are degrees. Fixture `TB`, `UD`, `FB`
map to RoboDK `[REAR, LOWERARM, FLIP]` as `T/U/N = 0`, `B/D/F = 1`.
After an intentional CSV edit, regenerate and review the JSON:

```powershell
uv run --locked python tests/fixtures/compile_fixtures.py
uv run --locked pytest tests/test_crx_kinematics.py
```

Generated stress corpora have separate checks; omit `--check` only when
intentionally regenerating and reviewing their data:

```powershell
uv run --locked python tests/fixtures/generate_solver_cases.py --check
uv run --locked python tests/fixtures/generate_accuracy_stress.py --check
```

`capture_crx_assets.py` records live asset references. Benchmarks remain opt-in
under `scripts/benchmark_*.py`; they do not add timing work to normal tests.
Fixture and asset changes must be called out explicitly.

Detailed numerical rationale, fixture provenance, benchmark commands and past
results are preserved in [validation history](../notes/testing-validation-history.md).
That archive records results at the time of each investigation; this guide
is the entry point for running and maintaining the current suite.

## External baseline comparisons

Build the snapshot branch in a separate checkout using its Release preset and
keep its DLL outside the candidate output directory. Record the snapshot commit
alongside the DLL hash; ordinary tests need only the candidate build.

To compare recorded fixture coverage against a supplied snapshot:

```powershell
$env:CRXKIN_BASELINE_LIBRARY_PATH = "C:/path/to/baseline/crx_kinematics.dll"
uv run --locked pytest tests/test_crx_adapter.py -k baseline
```

A configured but missing baseline is an error. Without this variable, only the
external-baseline tests skip; independent candidate checks still run.

For native timing, accuracy and coverage comparisons:

```powershell
cmake --build --preset windows-clang-release-tidy --target crx_solver_benchmark
uv run --locked python scripts/benchmark_solvers.py --baseline-dll C:/path/to/baseline/crx_kinematics.dll --baseline-revision COMMIT --output build/baseline-comparison
```

The runner snapshots baseline and candidate DLLs under distinct names and records
their hashes, inputs and candidate compile commands. `comparison.csv` reports
lost/gained strict solves, witness coverage and count differences. Every returned
command is checked with independent FK and joint limits; matching counts alone
are not correctness evidence. Timing ratios use common successful targets.
`benchmark_optimization.py` accepts the same DLL arguments and optionally
`--require-identical` for experiments requiring unchanged ordered outputs.
