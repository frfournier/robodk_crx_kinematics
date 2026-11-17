# Identification foundations: validation and measurements

Measured 2026-11-13 on Windows x64. The implementation is a local feature result; no global IK completeness or exact local certificate is claimed. Coverage is always `NotAttempted`.

## Implementation and compatibility

- Derive command/canonical coordinates from the modified-DH angles, apply the affine map and inverse throughout FK, discovery, limits and identification, and evaluate aspect/shoulder formulas on canonical vectors.
- Support mixed senses, encoder-zero shifts and signed forearm links. Reduce large encoder zeros through their stored sine/cosine rotation before subtracting the reference angle. Independent static/motion transform products cover offsets through ±1e20 radians with ordinary command magnitudes.
- Compare residual/denominator/forearm signs separately, retaining the existing near-zero guards and numerical regularity policy.
- Preserve all 15 original DLL exports and both 16-byte legacy result layouts. Add only `crx_v1_identify2`, with a separate 16-byte `(size=16, version=1)` extension; Python falls back to the old symbol.
- Preserve overflow failure semantics and exact command-box representatives among equal-cost rounded duplicates. Keep all identity classes before ranking when aliases could reorder a prefix; include large-turn conversion roundoff. The existing lift-work ceiling remains 1,048,576 nominations. The alias fallback uses O(U) storage and O(N log U) identity selection; peak RAM at the ceiling was not measured.
- Preserve the frozen fixture JSON and approved assets. Historical benchmark sampling boxes are explicit in `tests/fixtures/benchmark_models.py`; current asset/catalogue checks use the current approved capture limits.

## Build and regression checks

| Check | Result |
| --- | --- |
| `cmake --workflow --preset windows-clang-release-verify` | All 5 CTest entries pass; format check and clang-tidy build included. |
| Release Python regression | 17,023 passed, 833 opt-in skips. |
| Release solver reliability | 8,640 passed. |
| Debug build and matching CTest | Native tests and solver reliability pass; Python: 17,018 passed, 833 skipped, 5 failures. |
| Unchanged-parent Debug diagnostic reproduction | Same 5 failures: stderr capture is empty in diagnostic assertions; 2 related cases pass. |
| Opt-in counter runner lint | Real compile-command clang-tidy check exits 0; diagnostics retained. |
| Export comparison | Original exports retained; sole addition is `crx_v1_identify2`. |
| ABI memory checks | Old sizes/symbols, guarded extension buffers, unsupported sizes/versions/reserved values and optional-symbol fallback pass. |
| Independent geometry | Six captured assets; signed/mixed-sense/scaled models; independently differentiated DH Jacobians; turns, transforms, offset and limit checks pass. |

The five Debug failures are `test_invalid_model_diagnostic_has_complete_name_buffer` and `test_diagnostic_character_roundoff` at -64, -1, 1 and 64 ulps. They reproduce on the unchanged parent with the same toolchain. Logging code and these assertions were left intact. Debug is not reported as a green gate. The opt-in skips require an external baseline selection or explicit live RoboDK deployment; no required DLL/probe test was skipped.

Baseline foundation failures were reproduced before repair: ranked travel overflow, corpus regeneration using refreshed asset limits, pandas grouping conversion, and a twelve-element test seed. The corpus fix pins historical sampling metadata rather than regenerating observations. The native boundary test now correctly counts five admitted lifts: the shifted phase has two, the exact phase three. Its former sixth lift lies outside the box by much more than roundoff. Exact endpoint and prefix assertions remain, with additional eviction and large-turn counterexamples.

## Method and identities

Immediate parent: `d7f0f3aa4e404079736873ccf7226932d62ff6cc`. Historical comparison: `59d93ecc978a0e124bee9e2801c74b0c9317e80d`.
Candidate production-source identity: `fffd97309ea894f9e20f70955c4d12a487444f1d3960dd7a1bf9b8bb04a319cb`. This identifies the measured working-tree source.
Candidate DLL SHA-256: `dcb3a2ef1bcdacb175d13a9e49c276a6792e0759c01c3702f86cfd8d390e5f06`.
Parent DLL SHA-256: `dab8b42025a0e8f67b471aac83f15dd8df07cefe7336acf9f3996187212c5fc8`.
Historical DLL SHA-256: `77f9b1bde240ec09e24edf04ff0e55708d60e8d3bacd20a753cb877631e33e0c`.

Hardware: 12th Gen Intel Core i7-12700H, Windows 11 build 26200. Visual Studio 2026 x64 environment, clang-cl/clang-tidy/clang-format 22.1.3, CMake 4.3.1-msvc1, Ninja 1.13.2, uv 0.13.4, CPython 3.14.8, NumPy 2.5.3 and pandas 3.0.6. Deployment was disabled. Vendored Eigen was copied unchanged into isolated checkouts; per-header content hashes and compile commands are retained with the experiments.

Paired, uninstrumented complete-DLL calls use one thread pinned to affinity mask 1, shuffled case/mode order, alternating DLL order, three warmups, nine rounds and eight calls per timed batch at capacity 512. Builds, tests and other benchmarks were idle during timing. Times below are per-pose medians of batch means; p95 describes variation across poses, not individual-call tail latency. All outputs and limits are checked independently against captured modified-DH transform products at 0.0001 mm / 0.001 degree.

The original corpus has 4,313 targets, including 3,929 reachable targets. The independent stress corpus has 3,024 reachable targets over six captured geometries and length scales 0.001, 1 and 1,000. Both seed modes are retained. Capacities and exact prefixes are also exercised by native/pytest regression suites; seeded capacity one has a separate local-search contract.

## Ordinary IK results

| Comparison / corpus | Seed mode | Strict solves before / after | Overall p50 / p95 / p99 / max µs, before → after | Mixed-singularity p95 µs, before → after |
| --- | --- | --- | --- | --- |
| parent-existing | no_seed | 3929 / 3929 | 19.46 / 106.26 / 168.16 / 291.06 → 19.88 / 99.78 / 154.99 / 266.65 | 181.70 → 166.34 |
| parent-existing | nearby_seed | 3929 / 3929 | 19.44 / 107.06 / 168.07 / 278.15 → 19.88 / 99.54 / 156.49 / 259.90 | 178.83 → 166.91 |
| parent-stress | no_seed | 3024 / 3024 | 20.69 / 124.70 / 178.74 / 375.26 → 21.15 / 117.80 / 153.94 / 299.34 | 180.39 → 150.96 |
| parent-stress | nearby_seed | 3024 / 3024 | 20.68 / 124.50 / 179.94 / 358.98 → 21.15 / 118.08 / 152.43 / 296.46 | 183.76 → 150.24 |
| historical-existing | no_seed | 3929 / 3929 | 17.25 / 90.01 / 143.94 / 250.24 → 19.15 / 97.42 / 149.89 / 256.16 | 151.99 → 162.35 |
| historical-existing | nearby_seed | 3929 / 3929 | 17.26 / 90.04 / 142.74 / 249.43 → 19.20 / 97.46 / 153.00 / 256.75 | 150.59 → 162.05 |
| historical-stress | no_seed | 3024 / 3024 | 18.23 / 104.98 / 138.07 / 300.45 → 20.79 / 116.48 / 148.96 / 298.52 | 134.58 → 147.36 |
| historical-stress | nearby_seed | 3024 / 3024 | 18.22 / 104.16 / 138.34 / 292.43 → 20.80 / 115.79 / 151.33 / 299.70 | 135.00 → 147.44 |

Every comparison has zero invalid candidate outputs, zero lost strict solves and zero lost known-witness hits. Against the immediate parent, both corpora have identical numeric ordered command arrays, including best commands: no count or representative change needs attribution to this feature. Witness-hit parity is not a completeness claim or a claim that every supplied witness was recovered.

Historical differences are inherited from the immediate parent. [Per-case explanations](identification-case-changes.csv) cover all 1,840 changed case/mode catalogues: 840 original and 1,000 stress. Of these, output counts differ in 218 and 410 case/modes respectively. Every final catalogue is identical to the corresponding immediate-parent catalogue. The table distinguishes controller-identity duplicate removal, ordering/sub-precision changes, and near-coalescent representatives; it records nearest-command distances without reducing turns modulo 360. There are no unmatched changes beyond the existing 0.009-degree witness-matching tolerance.

No latency envelope was specified. The measured parent and historical tradeoffs are supplied for acceptance; the historical comparison is not presented as a speedup caused by this feature.

## Untimed Cramer refinement counters

Counter executables are separate from the measured DLLs. Per-case input/mode coverage, output counts and the outcome partition are validated against the uninstrumented runs. Runs count refinement invocations; evaluations include backtracking trials; guard rejections count failed determinant gates. Initial rejection, successful recovery and recovery failure partition runs. Recovery failures here are failed Cramer attempts, not failed full IK queries.

| Corpus, both seed modes combined | Runs | Evaluations | Guard rejections | Initial rejections | Successful recoveries | Recovery failures |
| --- | --- | --- | --- | --- | --- | --- |
| existing (parent = candidate) | 5172 | 12016 | 4174 | 4174 | 798 | 200 |
| stress (parent = candidate) | 5614 | 11182 | 4870 | 4746 | 612 | 256 |

Historical `59d93ec` predates this Cramer path; historical Cramer counters are not applicable, rather than a missing zero-valued measurement.

## Identification endpoint cost

A separate deterministic 200-target sample uses four entries per asset/group where available, nine rounds, eight repeats, three warmups and a pinned thread. These are wall times through ctypes with preallocated output arrays, including call-boundary validation/error checking; they exclude wrapper result processing and are not pure C++ timings. Each path has 176 positive identification outcomes, the same statuses/counts, and zero invalid outputs.

| Path | p50 / p95 / p99 / max µs |
| --- | --- |
| baseline | 41.27 / 151.28 / 226.22 / 233.57 |
| candidate_legacy | 41.59 / 142.67 / 191.10 / 231.74 |
| candidate_extended | 40.74 / 143.54 / 193.26 / 245.15 |

The paired extended-minus-legacy per-target median delta has median -0.156 µs and p95 3.593 µs. Small negative deltas are measurement variation, not evidence of negative metadata cost.

## Reproduction

Build the parent and historical revision in separate checkouts using their checked-in Release presets and matching Eigen/toolchain inputs. Preserve their DLLs before rebuilding. From the candidate checkout:

```powershell
uv sync --locked
cmake --workflow --preset windows-clang-release-verify
cmake --build --preset windows-clang-release-tidy --target crx_solver_benchmark
cmake --preset windows-clang-release
cmake --build --preset windows-clang-release --target crx_solver_counters
uv run --locked python scripts/benchmark_accuracy.py --before PARENT_DLL --after build/Release/crx_kinematics.dll --corpus existing --rounds 9 --repeats 8 --capacity 512 --before-revision PARENT_SHA --output build/evidence/identification/parent-existing
uv run --locked python scripts/benchmark_accuracy.py --before PARENT_DLL --after build/Release/crx_kinematics.dll --corpus stress --rounds 9 --repeats 8 --capacity 512 --before-revision PARENT_SHA --output build/evidence/identification/parent-stress
uv run --locked python scripts/benchmark_identification.py --before PARENT_DLL --after build/Release/crx_kinematics.dll --output build/evidence/identification/endpoints --rounds 9 --repeats 8 --capacity 512
```

Replace the uppercase paths/identities with immutable artifacts; repeat both accuracy commands with the historical DLL/revision and fresh output directories. To collect counters, pass `--before-counters PARENT_COUNTER_EXE --after-counters CANDIDATE_COUNTER_EXE`; the parent probe must contain only the instrumentation additions and current probe/build registration, not the feature solver changes. The tool validates counters after timing on the identical serialized input. Retain source/binary/corpus hashes, compile commands, Eigen header hashes, environment manifests and raw CSV/JSON. The historical probe is not required because that revision has no Cramer path.

The isolated workspace retains the raw artifacts and exact executed commands. This document and the derived case table are the standalone review report; no private planning file is required by the implementation, builds, tests or reproduction tools.
