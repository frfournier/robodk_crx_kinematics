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
uv run --locked pytest tests/test_crx_discovery.py -k polynomial
uv run --locked pytest
```

Direct pytest uses `build/Release/` by default and skips unavailable native
binaries. CTest supplies paths to its matching build. Use these environment
variables to select another build:

| Variable | Binary |
| --- | --- |
| `CRXKIN_LIBRARY_PATH` | Production `crx_kinematics.dll` |
| `CRXKIN_POLYNOMIAL_LIBRARY_PATH` | Comparison `crx_kinematics_polynomial.dll` |
| `CRXKIN_ROOT_PROBE_PATH` | `crx_canonical_tests.exe` |

## Where tests live

| Files | Coverage |
| --- | --- |
| `test_crx_kinematics.py` | Recorded FK/IK poses, configurations, C ABI, seed policy and continuity |
| `test_crx_discovery.py`, `test_crx_polynomial_adapter.py` | Seed-free witnesses through both DLLs, comparison and asset adaptation |
| `test_crx_command_turns.py`, `test_crx_solver_outcomes.py`, `test_crx_numerics.py` | Shared command limits, ranking, buffers, fallback and numerical guards |
| `test_crx_solver_fixtures.py`, `test_crx_solver_family.py`, `test_crx_accuracy_stress.py` | Frozen corpus, independent holdouts and singularity/scale regressions |
| `test_polynomial_roots.py`, `test_crx_polynomial_discovery.py` | Native probe compared with independent root/geometry references |
| `test_crx_asset_bridge.py` | Captured asset frames and model mapping |
| `test_crx_robolink_kinematics.py` | Opt-in live integration and offline deployment-guard checks |
| `test_version_metadata.py` | Release metadata consistency |
| `test_*.cpp` | Internal invariants, metadata, option budgets and allocation guards; run through CTest |

Shared support has three roles:

- `conftest.py` owns DLL loading, the two-backend `solver_lib` fixture, the
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

The production scanner retains strict expected failures for recorded discovery
misses. Invalid returned poses are never expected failures. Keep exact defect
IDs in `fixtures/crx_solver_known_failures.json`; remove an exception when its
fix is confirmed.

The release gate also reruns the API module against the polynomial DLL and
runs the polynomial corpus with all known-failure exceptions disabled:

```powershell
uv run --locked pytest tests/test_crx_solver_fixtures.py --require-solver-reliability -k polynomial
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
