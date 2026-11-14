# Certified nominal paths and branch coverage

This feature implements uniform path cells, bounded continuation, a branch graph, a standalone native backend and a packaged JSON frontend. It is based on Catalogues commit `427307200057212c9523a907928dddd1340d577e` (draft PR #25). The ordinary nominal C ABI retains all 29 exports and their ordinals. Calibration and Exceptional fibres remain independent sibling features.

The final Release workflow passes all 14 CTest entries, including **17,267 Python regressions**, **833 opt-in skips** and **8,640 reliability checks**. The frozen 42-query workload passes independent proof audits. All 14,674 paired ordinary solves retain identical return counts, refinement counters and **337,498 ordered solution/best-command rows, bit for bit**.

The path feature is conservative and computationally expensive. It proves task visitation for 25/42 frozen queries; 17 remain Inconclusive. It completes 114/145 attempted paths. These are deterministic workload outcomes, not estimates of population success rates. Broad motions remain unresolved under the tested budgets. No exhaustive robot-space connectivity, closest branch, collision clearance, dynamics or executable trajectory is claimed.

See [usage and request schema](../path-coverage.md), [the runnable synthetic example](../examples/path-coverage.json), and [complete machine-readable results](certified-path-coverage-results.json). All fixtures are synthetic; no measured calibration or robot path data is used.

## Supported exact target family

For raw represented endpoint positions and homogeneous quaternions, define

`p(s) = (1-s)*p0 + s*p1`,
`q(s) = (1-s)*q0 + s*q1`,
`R(s) = homogeneous_quaternion_action(q(s)) / ||q(s)||^2`,

for real `s` in `[0,1]`. The implementation does not flip quaternion signs or
substitute a slerp or an interpolated matrix. Each certified parameter interval
encloses the quaternion and proves its squared norm strictly positive. A path
whose quaternion vanishes, or whose denominator cannot be bounded away from
zero, remains inconclusive on that interval.

`exact::EncloseTargets` uses the existing exact quaternion operations to map the
whole enclosed family through the declared base/reference and TCP transforms.
Both existing `BasePose` and `ReferenceMap` conventions retain their meanings.
Lengths and translations use the same normalization as the exact nominal
problem. `path::Identity` retains all 77 first-problem words, all 16 raw final-pose
words and the frame convention. Equality does not depend on a finite hash.

The enclosure evaluates `a+s*(b-a)`. This is algebraically the declared affine
path; it avoids losing the dependency between `s` and `1-s` for a constant or
nearly constant path. Both endpoint differences and all later operations are
outward interval operations. The quaternion endpoints share one constant scale;
this preserves their exact homogeneous rotation family.

## Uniform local-cell theorem

A sealed existing point certificate supplies one exact nominal model, a fixed
represented canonical joint centre `c`, a verified inverse bound `beta`, regional
Lipschitz bound `L`, uniqueness radius `R` and regular Jacobian sign. Its
non-target provenance must match the prepared path exactly.

Let `a=F(c)`. For every parameter in the closed cell, the target enclosure proves
`trace(R(a)^T R(T(s))) >= 1`, which selects a common principal rotation-log
domain. `ball::EncloseResidual` reuses the shipped interval matrix-log series
and its tail bound to enclose `||A^-1 Log(a^-1 T(s))|| <= eta`. It retains the
verified inverse-residual factor `1/(1-e)`; the sealed represented
preconditioner is not assumed to equal the exact inverse. The existing
`3*pi*beta*pose_defect` bound is the conservative fallback.

The cell rechecks, outwardly,

`R*sum ||omega_i|| < pi`,
`beta*L*R < 1`,
`rho = upper(2*eta) <= R`,
`beta*L*rho^2/2 + eta <= rho`.

The anchored derivative depends on `F` and `a`, not on the target parameter.
Consequently the same point-certificate derivative/region witness applies to
every `T(s)` in the cell. The uniform inequalities give one root for every
parameter, with a common uniqueness ball and a common existence ball. The
existing inverse estimate gives continuity of this root as `T(s)` varies
continuously (DKN-091/092). Samples do not discharge any of these hypotheses.

## Actual command slacks

The canonical centre is mapped back using the prepared exact unwrapped encoder
offsets and exact unit diagonal senses. Raw degree bounds come from the existing
sealed command context and are enclosed using mathematical `pi/180`. They are
not reconstructed from rounded radians or circle phases.

For coupled callback commands the map has infinity-row norm `1`, so each lower
and upper command slack must be at least `rho`. For decoupled user coordinates,
`user[2]=callback[2]-callback[1]` has row norm `2`; that coordinate needs slack at
least `2*rho`. Other coordinates retain constant `1`. Actual real turn numbers
are preserved. The raw 12 degree-limit words and profile bind admitted cells.

A successful local lift can be retained while whole-ball command admission
fails. Such a cell is not admitted and cannot establish a graph edge. A failed
slack or radius check never establishes path or IK infeasibility.

## Gluing

Adjacent admitted cells must describe the same exact path and limits and have
an identical shared parameter endpoint. The checker proves

`||c_first-c_next||_infinity + rho_first <= R_next`.

At the junction, the first root lies inside the next uniqueness ball. Since the
next cell also has a root there for the identical target, uniqueness identifies
the two roots. The continuous lifts therefore glue (DKN-093). Mere overlap of
existence balls is insufficient. The gluing result retains the enclosed
distance, inclusion bound and next uniqueness radius.

## Bounded continuation and endpoint identity

`src/crx_path_tracking.cpp` performs strict, bounded, left-first dyadic continuation. Ordinary proposal work is isolated in `src/crx_path_nomination.cpp` and uses the actual existing numerical discovery kernels. A rounded midpoint target only nominates an inverse/region witness; the path checker continues to enclose the original exact raw target family over the entire cell. Each numerical nomination must pass the point and uniform-cell proof checks.

The tracker owns both endpoint problems/certificates and the raw command context. The supplied first and last problems must equal the literal prepared endpoint identities. Initial membership proves `E_first` lies in the first cell uniqueness ball; final membership proves the final cell existence ball lies in `U_last`. Root balls for different nearby targets are never identified merely by overlap.

The proposal policy normally reserves `4*rho <= R` for a following cell. This is a refinement policy, not an exclusion theorem. A final cell instead checks actual endpoint inclusion and needs no reserve for a future cell. A native regression supplies a deliberately displaced, valid point witness with `4*rho > R`; its constant path completes using one node, one point proof and no numerical discovery. All actual radius, slack, gluing and endpoint obligations remain required.

An incomplete result retains the entire parameter partition, including unvisited pending intervals when work expires. Local/admitted cell facts are distinct from acceptance into a fully glued path. Partial positive cells cannot seal an edge. Opposite verified Jacobian signs prevent this regular-cell tracker from certifying a singular crossing; they never establish IK or path infeasibility.

## Branch graph and standalone delivery

`src/crx_branch_graph.cpp` builds endpoint catalogues natively from each declared raw target and the actual raw degree limits. A vertex represents a task, phase and real command lift. The command lift enclosure is transformed back to canonical coordinates and must lie wholly inside a matching exact-target uniqueness ball before it binds to a point root. Partial catalogues retain individually matching sealed roots and independent constant-target command-admission checks. Unbound and admission-undecided vertices remain unusable for edges.

Only complete `Track` certificates create edges. Connected components join vertices through those edges; equal task IDs or equal target poses never merge roots. Two repeated poses in the example retain 32 vertices in 16 certified components; zero path work leaves 32 isolated vertices. Distinct turns retain distinct components. A component covering all supplied task IDs proves a finite admissible continuous walk visiting those tasks, allowing revisits and any visitation order. Global coverage remains `Inconclusive`. Only an existing sealed nominal point/command exclusion at a supplied task can establish task-set infeasibility. Missing edges cannot.

For identical exact targets, the nominal base flip is `(q0+pi, -q1, pi-q2, q3+pi, q4, q5)` modulo exact full turns. It preserves the canonical FK: `O3` is unchanged and `Rz(pi)*Ry(-theta-pi)=Ry(theta)*Rx(pi)` cancels the added wrist half turn. The tool labels this relation only after the entire flipped source existence box, with enclosed mathematical pi/turn shifts, lies in the other root uniqueness ball. Independent 100-digit FK checks and rational interval image checks cover this relation. Opposite endpoint shoulder-sine signs separately imply a wall crossing by continuity. Arbitrary same-sign nonflip pairs imply no such wall. These crossing conditions neither certify a transition nor exclude one.

The production `crx_path_tool.exe` reads a bounded, versioned packet from `crx_kinematics.paths`; `scripts/crx_paths.py` is a thin repository entry point. The wheel contains the frontend, not a native executable. Raw-model requests run without any solver DLL. Named models use the existing DLL only for model provenance; endpoint catalogues are rebuilt from the declared raw quaternions. The frontend never substitutes matrix-rounded metadata targets. Raw model/target/limit words remain in reports even after failed preparation. Windows Unicode request paths and atomic report replacement are checked. No test executable, private notes or orchestration file is a runtime dependency.

A shared CMake `crx_runtime_licenses` target supplies the project notices and exact vendored Eigen headers to the DLL, standalone tool and opt-in harness. It avoids concurrent copies to one destination and supports building the tool without a DLL target dependency. The isolated wheel/backend check copied no solver DLL, removed project paths/CRX environment hints, verified all 430 copied Eigen header files and reproduced the 32-vertex/16-edge example.

## Independent validation

The public auditor `tests/audit_crx_paths.py` verifies every retained positive proof in all 42 corrected workload queries. Exact rational Bernstein tests cover every path parameter, rather than accepting a sample grid. A separate 192-bit dyadic interval implementation uses rational Taylor/Lagrange trigonometric bounds and differentiated homogeneous products for FK/Jacobians, independently of the native moved-axis construction. It audits inverse residuals and the Neumann factor, regional constants, matrix-log remainder, radius inequalities, actual degree conversions and command slacks.

The completed audit checks **2,801 distinct positive point proofs at 180 digits**, **3,738 positive uniform cells**, **3,573 junction inclusions**, all endpoint memberships, exact circle/root partitions and Sturm counts, phase-ball separation, **692 command lifts**, crossing-condition claims, components and work ceilings. Of 84 task instances, 74 have complete endpoint covers; incomplete tasks remain explicit. The 3,738 cells include partial-vertex admission checks; the path-replay timing stage covers the 3,698 local cells retained in attempted paths. The shipped backend graph equals the harness graph, and timed proof/sample outputs equal their independently audited snapshots.

Regression coverage includes sixteen constant roots, moving/reversed paths, both frame conventions and command profiles, length scales `1e-6/1/1e6`, nonidentity base/TCP, real turns, bad junctions/gaps, stale identities, unsupported FP modes, half-turn log boundaries, zero path quaternions, partial/omitted graph work, repeated-target component splits and same-sign nonflip pairs. The exact dyadic singular witness has a known feasible zero-joint solution and prevents false task infeasibility. A 120-digit-generated tangent-translation fixture has admitted endpoints but a whole middle-root enclosure outside a supplied joint limit. Setting only wrist `q4=0` is not universally singular for offset-wrist geometry; the approach fixture also fixes shoulder `q1=0`.

The frozen data contains 18 synthetic cases and 24 queries over the six unchanged approved asset geometries: constant/tiny/moving/broad for each. All 18 nonconstant asset cases have checked nonzero endpoint displacements and a common starting pose per model. The moving/budget synthetic cases isolate the fixed first catalogue branch using actual endpoint command boxes, avoiding substitution of an easier branch by the graph proposal order. No source CSV or RoboDK asset changed.

## Checks and reproducibility

All commands run from the isolated feature repository in the Visual Studio x64 environment, with the checked-in CMake presets and task-local uv cache. The final gate and relevant additional checks were:

```powershell
cmake --workflow --preset windows-clang-release-verify
cmake --build --preset windows-clang-debug
ctest --preset windows-clang-debug --output-on-failure
ctest --preset windows-clang-debug -R 'crx\.(path-tracking|branch-graph|path-certificate)$' --output-on-failure
uv run --offline pytest tests/test_crx_paths.py tests/test_crx_branch_graph.py
uv run --offline pytest tests/test_crx_branch_graph.py::test_frozen_asset_workload_retains_actual_motion_and_common_start_poses
uv run --offline python tests/fixtures/make_path_limit_interior.py
cmake --build --preset windows-clang-release-tidy --target crx_path_benchmark crx_solver_benchmark crx_solver_counters
uv build --offline --out-dir <package-artifacts>
```

The full Release result is 14/14 CTest entries, 17,267 Python passes/833 skips and 8,640 reliability passes. Matching Debug feature checks pass three native tests and 57 focused Python checks; the subsequently added workload-intent regression passes separately. The full Debug run has 17,261 passes/833 skips, 8,640 reliability passes and exactly the same five inherited diagnostic-capture failures as the actual recorded parent baseline:

- `tests/test_crx_kinematics.py::test_invalid_model_diagnostic_has_complete_name_buffer`
- `tests/test_crx_numerics.py::test_diagnostic_character_roundoff[-64]`
- `tests/test_crx_numerics.py::test_diagnostic_character_roundoff[-1]`
- `tests/test_crx_numerics.py::test_diagnostic_character_roundoff[1]`
- `tests/test_crx_numerics.py::test_diagnostic_character_roundoff[64]`

Those full Debug results predate the final terminal-reserve regression; that change passes the matching focused Debug checks and the final full Release gate. No inherited failure is waived in the Release gate. The 833 opt-in skips include unavailable/live integration checks; no RoboDK deployment or live robot test was requested. Qmake is unvalidated; the standalone tools use CMake.

Scoped clang-format and clang-tidy fixes were reviewed. The complete tidy-enabled build passes with visible nonfatal style advisories; no warning configuration was weakened or suppressed. The existing clang-cl/clang-tidy exception-forwarding workaround was extended to the new I/O/harness files. All 12 proof compile instances use strict/no-contraction/no-LTO flags; all nine optimized production proof objects contain no fused arithmetic instructions. Numerical nominations retain the normal compilation policy. Timed targets contain no discovery-counter instrumentation. DLL export names and ordinals are identical to the actual Catalogues parent.

Wheel and sdist byte checks cover all seven Python modules, typing marker, guides and example. Direct wheel import and the isolated raw-model CLI pass. The intentional new data files are the 42-query benchmark workload and the interior-limit JSON fixture with its regeneration script. The latter regenerates byte for byte; no existing asset or fixture CSV changed.

Run the full independent workload audit:

```powershell
uv run python tests/audit_crx_paths.py --dataset tests/fixtures/crx_path_queries.json `
  --harness build/Release/crx_path_benchmark.exe --backend build/Release/crx_path_tool.exe `
  --output build/path-audits
```

## Measurement identity and method

Measurements completed sequentially after every build, test and oracle process terminated. A single measurement lease prohibited concurrent work and source edits. The process and children were pinned to logical CPU mask `1` on a 12th Gen Intel(R) Core(TM) i7-12700H; `OPENBLAS_NUM_THREADS`, `MKL_NUM_THREADS` and `OMP_NUM_THREADS` were `1`. Feature stages used three iterations in each of nine rounds, reversing stage order on alternate rounds. Ordinary DLL calls used eight iterations and nine paired rounds at capacity 512. Source, binary, asset and input hashes were guarded before and after commands.

The native stages link shipped kernels directly: standalone endpoint preparation/catalogues/commands; whole graph construction; replay of the frozen attempted paths with their actual remaining budgets; uniform cell checks with existing sealed point witnesses; and numerical tracking at exactly the same retained leaf midpoints. Standalone preprocessing is not an additive breakdown of a shared-budget graph query. The numerical sampler seeds each next nomination from its previous result. Its retained-midpoint workload is distinct from the proof engine's extra adaptive internal nodes and point proofs, and supplies no between-sample certificate. Parse/serialization costs lie outside native timers. Empty stages can measure zero clock ticks and have no claimed work throughput.

| Identity | SHA256 / commit |
|---|---|
| Measured source snapshot | `117e23747c308d93d839007e18692a5d641eb497ed76a6dd52ad736e3751f34b` |
| Frozen query data | `37b99535b6880926c970a0ba5c6657a370cf0a0bc7598160d9c142f604c70045` |
| Candidate DLL | `0d5d25c6aceafc129882b7e8796ab29057bc11dff6f8fc25533565c5fde56cdb` |
| Standalone backend | `2fa45aaf417aacb4b5e31eef68619bc6de248720cf0c9dca920ff07ffc4a6885` |
| Native feature driver | `09a13245e5a4e50c12a6729087aedf206c3aba5b1bebbb1e5ea7b0f8455da549` |
| Actual parent commit | `427307200057212c9523a907928dddd1340d577e` |
| Actual parent DLL | `aca4d3d566131a20c6170be14fab60daac703595566dec71b6a1b0ff5471c38b` |

Subsequent delivery edits are this report and its compact result file; runtime/header/wrapper/driver inputs remain byte-identical to the measured snapshot. Source archives, compile commands, raw measurements and audits are retained locally, while the committed query data, public tools, full tables and compact results make the experiment reproducible from this repository. CPU frequency and unrelated system activity are not controlled; small ordinary timing ratios do not demonstrate a speed change.

## Feature outcomes and work

Across 42 queries: 25 positive finite-task coverage results, 17 Inconclusive, zero task-infeasible results; 114/145 attempted paths complete. There are 3,872 retained path leaves, 3,698 local path cells and 3,727 subdivisions. Missing/uncertain links are never interpreted as disconnectedness or unreachability.

The largest reported retained payload is 2,511,770 bytes. This is a native `sizeof`/container-capacity estimate, including string capacity alongside object sizes, rather than RSS. It excludes allocator overhead, temporary workspaces and JSON buffers. The regression for a one-cell query with `max_nodes=65535` verifies that the leaf vector does not eagerly reserve its full ceiling.

Hard ceilings: 64 tasks, 256 requested connections, 4,096 vertices, 65,536 pair tests, 4,096 path attempts, 65,535 shared root/path nodes and `2^20` shared path point proofs. Per-path depth is at most 48 and trials per node at most 128; endpoint root depths are at most 50. The report retains omitted pair counts/cursors and every unresolved parameter interval. All observed counts satisfy the corresponding query budgets.

| Query | Result | Complete/attempted paths | Retained leaves | Path nodes | Retained KiB |
|---|---|---:|---:|---:|---:|
| `synthetic_constant_16` | Task coverage | 16/16 | 16 | 16 | 572.1 |
| `synthetic_tiny_16` | Task coverage | 16/16 | 16 | 16 | 572.1 |
| `synthetic_moving` | Task coverage | 1/1 | 125 | 249 | 550.6 |
| `synthetic_budget` | Inconclusive | 0/1 | 3 | 3 | 108.3 |
| `synthetic_broad` | Inconclusive | 0/1 | 68 | 127 | 672.2 |
| `synthetic_zero_work` | Inconclusive | 0/16 | 16 | 0 | 573.2 |
| `synthetic_partial_cover` | Task coverage | 14/16 | 16 | 14 | 540.8 |
| `synthetic_no_points` | Inconclusive | 0/0 | 0 | 0 | 37.3 |
| `synthetic_turns` | Task coverage | 32/32 | 32 | 32 | 1251.6 |
| `synthetic_decoupled` | Task coverage | 16/16 | 16 | 16 | 572.1 |
| `synthetic_quaternion_zero` | Inconclusive | 0/1 | 30 | 31 | 498.2 |
| `synthetic_halfturn` | Inconclusive | 0/1 | 1 | 0 | 290.4 |
| `synthetic_frames` | Task coverage | 1/1 | 1 | 1 | 99.7 |
| `synthetic_near_singular_2` | Task coverage | 1/1 | 1 | 1 | 100.5 |
| `synthetic_near_singular_4` | Inconclusive | 0/0 | 0 | 0 | 33.1 |
| `synthetic_near_singular_6` | Inconclusive | 0/0 | 0 | 0 | 1928.5 |
| `synthetic_exact_singular` | Task coverage | 1/1 | 1 | 1 | 2002.7 |
| `synthetic_interior_limit` | Inconclusive | 0/1 | 24 | 31 | 139.6 |
| `crx_5ia_constant` | Task coverage | 1/1 | 1 | 1 | 189.0 |
| `crx_5ia_tiny` | Task coverage | 1/1 | 1 | 1 | 189.0 |
| `crx_5ia_moving` | Inconclusive | 0/1 | 513 | 1023 | 2452.9 |
| `crx_5ia_broad` | Inconclusive | 0/1 | 72 | 127 | 460.5 |
| `crx_10ia_constant` | Task coverage | 1/1 | 1 | 1 | 187.2 |
| `crx_10ia_tiny` | Task coverage | 1/1 | 1 | 1 | 187.2 |
| `crx_10ia_moving` | Inconclusive | 0/1 | 513 | 1023 | 2451.2 |
| `crx_10ia_broad` | Inconclusive | 0/1 | 72 | 127 | 458.8 |
| `crx_10ia_l_constant` | Task coverage | 1/1 | 1 | 1 | 186.8 |
| `crx_10ia_l_tiny` | Task coverage | 1/1 | 1 | 1 | 186.8 |
| `crx_10ia_l_moving` | Task coverage | 1/1 | 509 | 1017 | 2450.8 |
| `crx_10ia_l_broad` | Inconclusive | 0/1 | 72 | 127 | 458.4 |
| `crx_20ia_l_constant` | Task coverage | 1/1 | 1 | 1 | 186.8 |
| `crx_20ia_l_tiny` | Task coverage | 1/1 | 1 | 1 | 186.8 |
| `crx_20ia_l_moving` | Task coverage | 1/1 | 509 | 1017 | 2450.8 |
| `crx_20ia_l_broad` | Inconclusive | 0/1 | 72 | 127 | 458.4 |
| `crx_25ia_constant` | Task coverage | 1/1 | 1 | 1 | 186.8 |
| `crx_25ia_tiny` | Task coverage | 1/1 | 1 | 1 | 186.8 |
| `crx_25ia_moving` | Task coverage | 1/1 | 509 | 1017 | 2450.8 |
| `crx_25ia_broad` | Inconclusive | 0/1 | 72 | 127 | 458.4 |
| `crx_30ia_constant` | Task coverage | 1/1 | 1 | 1 | 186.8 |
| `crx_30ia_tiny` | Task coverage | 1/1 | 1 | 1 | 186.8 |
| `crx_30ia_moving` | Task coverage | 1/1 | 509 | 1017 | 2450.8 |
| `crx_30ia_broad` | Inconclusive | 0/1 | 72 | 127 | 458.4 |

## Native feature timings

Median milliseconds per complete stage invocation, not per point. `Cells` checks existing sealed point witnesses; `Numeric` is the separate retained-midpoint sampler. A dash means the stage has no corresponding logical work. All nine round values, p95s and throughput fields are in the compact result file.

| Query | Endpoint preprocessing | Whole graph | Frozen paths | Cells | Numeric |
|---|---:|---:|---:|---:|---:|
| `synthetic_constant_16` | 2.199 | 6.726 | 1.900 | 0.565 | 0.234 |
| `synthetic_tiny_16` | 2.312 | 6.733 | 1.963 | 0.566 | 0.243 |
| `synthetic_moving` | 2.262 | 58.312 | 55.437 | 4.399 | 1.852 |
| `synthetic_budget` | 2.264 | 3.050 | 0.664 | — | 0.043 |
| `synthetic_broad` | 2.283 | 35.798 | 31.481 | 2.073 | 1.010 |
| `synthetic_zero_work` | 2.249 | 5.103 | 0.222 | — | 0.234 |
| `synthetic_partial_cover` | 2.225 | 7.334 | 1.748 | 0.496 | 0.233 |
| `synthetic_no_points` | 2.254 | 0.661 | — | — | — |
| `synthetic_turns` | 2.219 | 15.707 | 3.891 | 1.124 | 0.462 |
| `synthetic_decoupled` | 2.236 | 7.326 | 1.928 | 0.570 | 0.233 |
| `synthetic_quaternion_zero` | 2.266 | 11.959 | 7.746 | 0.070 | 0.427 |
| `synthetic_halfturn` | 2.075 | 3.644 | 0.016 | — | 0.032 |
| `synthetic_frames` | 0.692 | 1.052 | 0.121 | 0.036 | 0.017 |
| `synthetic_near_singular_2` | 0.758 | 1.124 | 0.124 | 0.039 | 0.015 |
| `synthetic_near_singular_4` | 0.894 | 0.980 | — | — | — |
| `synthetic_near_singular_6` | 88.074 | 87.323 | — | — | — |
| `synthetic_exact_singular` | 88.106 | 88.236 | 0.112 | 0.034 | 0.015 |
| `synthetic_interior_limit` | 0.735 | 8.099 | 7.145 | 0.274 | 0.353 |
| `crx_5ia_constant` | 1.222 | 1.941 | 0.120 | 0.036 | 0.015 |
| `crx_5ia_tiny` | 1.145 | 1.804 | 0.114 | 0.035 | 0.015 |
| `crx_5ia_moving` | 1.219 | 212.722 | 212.348 | 18.042 | 7.710 |
| `crx_5ia_broad` | 1.180 | 28.397 | 26.584 | 1.929 | 1.065 |
| `crx_10ia_constant` | 1.103 | 1.787 | 0.111 | 0.033 | 0.015 |
| `crx_10ia_tiny` | 1.128 | 1.814 | 0.115 | 0.034 | 0.015 |
| `crx_10ia_moving` | 1.222 | 206.472 | 204.344 | 18.183 | 7.604 |
| `crx_10ia_broad` | 1.169 | 32.295 | 30.763 | 1.921 | 1.073 |
| `crx_10ia_l_constant` | 1.110 | 1.805 | 0.114 | 0.034 | 0.016 |
| `crx_10ia_l_tiny` | 1.125 | 1.770 | 0.114 | 0.034 | 0.015 |
| `crx_10ia_l_moving` | 1.139 | 210.476 | 208.270 | 17.775 | 7.433 |
| `crx_10ia_l_broad` | 1.153 | 28.134 | 26.535 | 1.920 | 1.069 |
| `crx_20ia_l_constant` | 1.108 | 1.758 | 0.113 | 0.034 | 0.014 |
| `crx_20ia_l_tiny` | 1.095 | 1.709 | 0.109 | 0.033 | 0.015 |
| `crx_20ia_l_moving` | 1.170 | 209.801 | 208.057 | 17.836 | 7.417 |
| `crx_20ia_l_broad` | 1.134 | 28.178 | 26.426 | 1.910 | 1.037 |
| `crx_25ia_constant` | 1.061 | 1.734 | 0.109 | 0.033 | 0.014 |
| `crx_25ia_tiny` | 1.099 | 1.712 | 0.111 | 0.033 | 0.015 |
| `crx_25ia_moving` | 1.193 | 205.510 | 203.491 | 17.725 | 7.524 |
| `crx_25ia_broad` | 1.140 | 27.491 | 25.604 | 1.918 | 1.038 |
| `crx_30ia_constant` | 1.130 | 1.804 | 0.115 | 0.034 | 0.015 |
| `crx_30ia_tiny` | 1.080 | 1.735 | 0.111 | 0.033 | 0.015 |
| `crx_30ia_moving` | 1.151 | 203.261 | 203.211 | 17.848 | 7.464 |
| `crx_30ia_broad` | 1.129 | 27.456 | 26.033 | 1.987 | 1.041 |

The fixed-branch synthetic moving query completes 125 cells/124 links using 249 nodes and 560 point certificates. Its frozen path replay takes 55.437 ms; uniform cells alone take 4.399 ms; numerical tracking at its 125 retained midpoints takes 1.852 ms. The restricted three-node/eight-point version remains Inconclusive. This separates uniform proof, adaptive proof work and sampled proposal costs.

For asset geometries, normalized translation `1e-6` completes on four models with 509 cells/1,017 nodes, taking about 203–210 ms for the whole graph. CRX-5iA/10iA hit the 1,023-node ceiling and remain Inconclusive, retaining 513 leaves. Every normalized `1e-3` broad case remains Inconclusive under 127 nodes. These conservative regional constants and budgets are not a practical guarantee for ordinary millimetre-scale motion planning. Constant/tiny motions and bounded subinterval proofs are the demonstrated positive cases.

## Ordinary solver comparison

Both original corpora use the actual committed Catalogues parent, the uninstrumented candidate DLL and separate matched counter executables. All 14,674 paired return counts/counter records and all 337,498 ordered solution/best-command rows match exactly in binary64. Strict solve counts, witness hits and position/angle-error distributions are identical.

| Corpus/mode | Paired poses | Parent p50 us | Candidate p50 us | Median paired candidate/parent | Strict solves | Witness hits |
|---|---:|---:|---:|---:|---:|---:|
| existing/no_seed | 4313 | 18.488 | 18.550 | 1.001471 | 3929 | 3726 |
| existing/nearby_seed | 4313 | 18.488 | 18.562 | 1.002259 | 3929 | 3726 |
| stress/no_seed | 3024 | 20.275 | 20.394 | 1.001878 | 3024 | 2994 |
| stress/nearby_seed | 3024 | 20.350 | 20.475 | 1.002717 | 3024 | 2994 |

Median paired ratios range from 1.00147 to 1.00272. This is a small shared-machine timing difference, not evidence of an ordinary solver optimization or material regression. Position/angle error quantiles, raw round values and exact-equality assertions are recorded in the compact results.

Reproduce one feature case, then the paired ordinary runs (supply the actual parent DLL/counter executable and save the candidate source archive):

```powershell
uv run python scripts/benchmark_paths.py tests/fixtures/crx_path_queries.json `
  --case synthetic_moving --harness build/Release/crx_path_benchmark.exe `
  --iterations 3 --rounds 9 --output build/path-moving-timing.json
uv run python scripts/benchmark_accuracy.py --before <Catalogues-parent.dll> `
  --after build/Release/crx_kinematics.dll --corpus existing --rounds 9 --repeats 8 `
  --capacity 512 --before-counters <Catalogues-parent-counters.exe> `
  --after-counters build/Release/crx_solver_counters.exe --after-sources <sources.zip> `
  --compile-commands build/vscode/windows-clang-release-tidy/compile_commands.json `
  --output build/path-ordinary-existing
# Repeat with --corpus stress and a new output directory.
```

## Delivery boundaries

The supported target law is exact raw affine position plus nonvanishing homogeneous-quaternion interpolation. Numerical failures, zero/unbounded quaternion domains, exhausted budgets, missing roots/lifts and failed inclusions remain explicit. No singular crossing or continuous-family path certificate is implemented by this regular-cell module. A positive finite-task walk does not prove exhaustive branch connectivity; no nearest/shortest path or controller dynamics claim is made.

The proof arithmetic and checks are numerical C++ implementations of the stated theorems, with independent audits; the C++ program itself is not formally verified. Tests target Windows x64 using the pinned Python/toolchain. The user's unrelated working-tree edits and all six assets remain unchanged. No merge, RoboDK deployment or history rewrite is part of this delivery.
