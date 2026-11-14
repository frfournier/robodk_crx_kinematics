# Exceptional wrist recovery and fibre diagnostics

This feature extends the opt-in nominal catalogue query with bounded recovery
on unresolved determinant-zero domains and numerical stratum diagnostics.
Parent: Catalogues commit `427307200057212c9523a907928dddd1340d577e`, draft PR #25.
The implementation and measurements are complete. The results describe a
frozen prepublication checkpoint; committed-checkout and publication evidence
is recorded separately in its PR. This C++ implementation is not formally
verified.

## Geometry and reconstruction

Use normalized signed arm lengths `a,b`, wrist point `x`, unit fifth axis `u`
and elbow `y`. The original incidence relation is

```
|y|^2 = a^2, |y-x|^2 = b^2,
y . (Z cross x) = 0, (y-x) . u = 0.
```

Set `n=Z cross x`, `H=|n|^2`, `s=|x|^2`, `v=x cross n` and
`k=(a^2+s-b^2)/2`. For `H>0`, the orthogonal basis `(x,n,v)` gives

```
y = (k H x + t v)/(s H),
t^2 = H (a^2 s-k^2),
t (v.u) = H (s-k) (x.u).
```

When the radicand is nonnegative, retain both signs
`t=+/-sqrt(H(a^2 s-k^2))`. The last unsquared equation selects compatible
witnesses. Since `Delta=x.(u cross n)=-v.u`, determinant zero does not determine
an elbow uniquely. Dividing by Delta loses these branches. The existing stable
incidence chart uses a unit radial direction and a perpendicular direction in
the base plane to implement the same two sphere intersections. Near shell
tangencies it obtains the small signed displacement from the wrist equation,
then checks both original sphere equations. Recovery checks full canonical FK
and both base branches before retaining a configuration.

For `H=0`, write `x=d Z`. If `d!=0`, the sphere equations fix
`y_z=k/d`, with horizontal radius squared `a^2-y_z^2`. The wrist condition is
`y_h.u_h=(d-y_z)u_z`. A nonzero horizontal wrist direction gives two circle/line
intersections, a tangency, or no intersection. When both plane normals are
vertical and the numerical compatibility test passes, keep deterministic
antipodal representatives of the horizontal elbow circle. Guard the competing
wrist-plane construction against roundoff-sized direction vectors before
selecting it; normalization of nearly parallel vectors otherwise creates an
unstable apparent plane pair. Full sphere, wrist-plane and FK checks still apply.

At `x=0`, equal squared arm lengths leave `|y|=|a|, y.u=0`; horizontal antipodes
give existing numerical representatives. Unequal lengths or uncertain geometry
remain unresolved as appropriate. If both wrist and elbow are vertical, base
rotation is undetermined and recovery uses deterministic representatives. No
numerical proximity dispatch proves equality of the exact defining equations.

## Recovery and coverage policy

The sealed exact-problem root cover remains authoritative. Recovery runs only
when that cover is incomplete. It nominates midpoint roots of the enclosed
degree-at-most-four determinant polynomial, existing analytic wrist/base/shell
crossings, and midpoints of unresolved leaves whose residual and determinant
Bernstein hulls both straddle zero. When every residual coefficient interval
contains zero, or the whole circle is unresolved, four cardinal representatives
supplement the nominations. Cardinal representatives precede leaf work so a
large unresolved frontier cannot consume their slots.

All coefficient-midpoint degree decisions, real projections, angle reductions,
phase deduplication and candidate FK checks are numerical. They cannot certify
an exact common zero or count a family. At most 32 distinct nominated phases
are processed. Each uses the existing two-branch incidence recovery; uncertain
recovery and missed determinant/domain nominations can use bounded full-pose
refinement. Every attempt retains its final numerical status, refinement flag,
source flags and up to eight full FK-checked canonical configurations.

Each candidate may separately receive a local ball certificate against the
unchanged raw exact problem. The candidate-certificate budget is shared with
ordinary discovery. Exhaustion is reported explicitly; numerical witnesses
remain retained even when the local-ball budget is zero. Repeated nominations
can yield repeated or overlapping local balls, so attempt/ball counts are not
counts of distinct exact configurations. Their source and identity remain
available independently.

The extra numerical storage has a hard bound of 32 attempts times eight
configurations. Ordinary discovery can retain at most 368 nominations, so the
combined pre-certificate nomination bound is 624, not a solution-count theorem.
The shared candidate-certificate budget defaults to 512. Reading all optional
C recovery snapshots transfers at most 32 times 616 bytes (19,712 bytes), plus
the 40-byte summary; native/query and Python ownership overhead is separate.
Local balls and root witnesses remain subject to the existing certificate,
node/leaf/depth/chart budgets. Diagnostics are lazy in Python, so ordinary query
construction does not materialize those extra snapshots unless requested.

The root certificate and original unresolved domains are retained without
modification. The assembler rejects incomplete root covers before any attained
candidate count can establish completeness. Thus even four locally certified
exceptional configurations leave global coverage and nearest selection
inconclusive. A continuous fibre cannot receive an isolated-root ball proof:
its tangent gives a Jacobian null direction. Exact family recognition in rational
arithmetic and family parametrizations are explicitly deferred.
`ContinuousFibre` is reserved and never emitted by this feature.

## Diagnostic and API contract

Original options, summary, local-ball and other public ABI-1 layouts stay
unchanged. Optional `crx_v1_catalogue_strata_read` and
`crx_v1_catalogue_recovery_read` expose negotiated 40-byte and 616-byte records.
They require exact size/version1/reserved0, reject invalid indices and preserve
the entire output on failure. The query owns their immutable data and survives
model destruction. The new enum values map explicitly to numerical recovery
statuses, not solver acceptance or proof outcomes.

Python `query.strata` returns an owned `FibreStrata` containing immutable
`FibreRecovery` snapshots. New DLLs with no needed recovery return an empty
attempt tuple. DLLs lacking either optional endpoint return `None` while their
existing catalogue query remains usable. Lazy reads/closure share the existing
RLock; materialized snapshots survive closure. Arrays contain only the retained
candidate prefix. Counts, statuses, booleans, flag masks and numerical witness
arrays are validated before publication to the caller.

For a nominated phase, define `L=max(|a|,|b|,|x|)`. Numerical defects are:

| Diagnostic | Dimensionless magnitude |
| --- | --- |
| determinant zero | `abs(Delta)/L^2` |
| base axis | `hypot(x_x,x_y)/L` |
| coincident spheres | `max(|x|/L,abs(a^2-b^2)/L^2)` |
| vertical arm | minimum over retained elbows of `max(|x_h|,|y_h|)/L` |
| arm shell | `abs(a^2 s-k^2)/L^4` |
| compatibility | minimum over retained elbows of `abs((y-x).u)/L` |

Flags use `4096*DBL_EPSILON`; unavailable elbow-dependent defects are NaN.
`POLYNOMIAL_ENCLOSES_ZERO` reports coefficient interval containment, not exact
polynomial identity or an exhaustive fibre classification. Flags describe the
nominated phase; refinement can move a retained candidate within its bounded
window. Position errors use normalized length units, orientation radians.
Multiply position errors by `summary.length_scale` for physical units.

## Validated runtime checkpoint

The full Release workflow passes all 12 CTest entries: 17,234 Python regressions,
833 opt-in skips and 8,640 reliability checks. Matching Debug passes all ten
native suites and 211 focused arithmetic/proof/API tests, including 25 new
independent exceptional-recovery cases. The full Debug run has exactly the five
diagnostic-output capture failures already reproduced on the unchanged parent:
four `test_diagnostic_character_roundoff` parameters and
`test_invalid_model_diagnostic_has_complete_name_buffer`. Its remaining 17,229
Python tests and 8,640 reliability checks pass; no test is disabled or relaxed.
The imported RoboDK package emits its existing PendingDeprecationWarning.

All nine actual proof compile commands retain strict floating-point/no-LTO
options. Disassembly of all six optimized proof objects contains no fused
multiply-add/subtract opcodes. Existing proof arithmetic sources are unchanged.
The optional qmake build is unavailable and has not been validated.

The actual published parent DLL lacks both new diagnostic exports. The new
wrapper passes 72 compatibility cases against that DLL: six assets, both
coordinate profiles, enabled/disabled proof work and capacities 0/1/64. Full
numerical FK and native query ownership after model destruction are checked.
The wheel and Python source distribution build; all six module byte streams,
the typing marker and declared wrapper documentation match their source.
Direct import from the actual wheel and post-model-close diagnostic reads pass.
Native validation reports remain repository deliverables under the existing
Python-only source distribution policy.

Commands for this checkpoint, from the x64 developer environment:

```powershell
cmake --preset windows-clang-debug
cmake --build --preset windows-clang-debug
ctest --preset windows-clang-debug
cmake --workflow --preset windows-clang-release-verify
uv run --locked --offline pytest tests/test_crx_exceptional.py -q
uv build --offline
```

Direct pytest must select the matching DLL and
`CRXKIN_EXCEPTIONAL_PROBE_PATH`; CTest supplies every required probe itself.
The full Debug command returns failure for the five known capture checks;
the Release workflow is fully passing. No live RoboDK deployment was performed.

## Independent coverage and remaining evidence

The new native exceptional probe and independent Python tests cover both
determinant-zero elbow signs and both base representatives, separate local
proofs, on-axis noncoincident circles, vertical/free-base and coincident-sphere
families, shell tangencies, signed links, power-of-two unit scales, perturbed
common roots, zero/exhausted root and candidate budgets, numerical defect
cross-checks, record negotiation/canaries, optional-symbol fallback, concurrent
reads and post-model/query snapshot ownership.

The independent oracle derives raw quaternion geometry with rational arithmetic
and evaluates full elementary-transform products and local certificate constants
at 160 digits. A candidate's rounded full FK check is not exact acceptance.
Positive-dimensional examples explicitly require no false isolated-root proof.
The existing catalogue/root/command and solver suites remain regression gates.
The completed asset audit and measurements follow. No latency envelope was
supplied, so no performance acceptance or speedup claim is made.

## Six-asset exceptional and nearby audit

The cohort contains 64 captured or independently reconstructed modified-DH
targets per approved asset and eight deterministic nearby translations per
asset: 432 total. Exact rational isolation and 160–640-digit reconstruction
classify 366 finite and 66 exact exceptional outcomes. Separate 200-digit
transform products, defect evaluation and local-ball constant checks audit
every retained numerical witness and proof, including exceptional outcomes.
There are zero errors or unresolved oracle cases. Unresolved oracles never
count as agreement. All original inputs, raw identity words, full recovered
configurations, local proof constants and outcomes are retained.

| Asset | Cases | Finite / exceptional | Complete / inconclusive | Numerical witnesses | Recovery proof instances |
| --- | ---: | ---: | ---: | ---: | ---: |
| CRX-5iA | 72 | 61 / 11 | 57 / 15 | 108 | 62 |
| CRX-10iA | 72 | 61 / 11 | 57 / 15 | 112 | 66 |
| CRX-10iA-L | 72 | 61 / 11 | 57 / 15 | 108 | 62 |
| CRX-20iA-L | 72 | 61 / 11 | 57 / 15 | 108 | 62 |
| CRX-25iA | 72 | 61 / 11 | 57 / 15 | 108 | 62 |
| CRX-30iA | 72 | 61 / 11 | 57 / 15 | 108 | 62 |

Totals: 342 Complete and 90 Inconclusive, with 486 recovery attempts, 652
numerical witnesses and 376 recovery local-proof instances. Of 198 refinement
attempts, 102 yield numerical configurations and 96 remain unsuccessful.
All 66 exact exceptional outcomes and 24 finite outcomes remain inconclusive.
The cohort intentionally emphasizes exceptional and nearby targets; these
fractions are not estimates of production frequency. Repeated nominations
and overlapping local balls are retained, so witness/proof-instance counts
are not counts of distinct exact solutions.

The equal-link, free-base, coincident-sphere and shell contract examples use
explicit synthetic nominal models. They do not assert that any corrected
captured robot has equal arm lengths or measured calibration data. The six
approved asset hashes and fixture inventory are unchanged.

## Measurement method and scopes

All builds, tests and oracle workers were terminal before timing. Feature,
original-corpus and stress-corpus experiments ran strictly serially on the
same Windows x64 host (12th Gen Intel(R) Core(TM) i7-12700H). BLAS/MKL/OMP
thread limits were one. Feature runs inherited the recorded process affinity;
the existing ordinary native runner pins its timing thread. Toolchain:
Visual Studio 2026 Community 18.10.3, clang-cl/clang-tidy/clang-format 22.1.3,
CMake 4.3.1-msvc1, Ninja 1.13.2, uv 0.13.4, CPython 3.14.8, NumPy 2.5.3,
pandas 3.0.6, mpmath 1.3.0 and vendored Eigen 5.0.1.

The 148 configurations contain captured, independent, nearby and synthetic
exceptional/degenerate/budget cases, with capacities 1/64/512, both command
profiles and seeded/unseeded controls. All 1,923 eligible native operations
and 504 typed operations passed untimed verification before measurement.
Nine randomized rounds follow untimed verification and warmup. Each operation
uses actual call boundaries until at least 5 ms of measured work accumulates;
clock overhead is retained. Tables summarize per-configuration round medians,
then percentiles across the stated population. Native stages exclude input
copies, scratch reset/cleanup, checksum consumption and validation. Actual
C measurements include query, summary and destruction; model creation is
excluded. Typed measurements include query snapshots and native cache closure;
the separate with-strata operation also materializes lazy diagnostics.
Snapshot destruction and outcome/array validation remain outside clocks.

A separate opt-in stage object includes the actual shipped exceptional
implementation once. Its ordinary floating-point settings match numerical
discovery; shared proof libraries retain strict arithmetic. Recovery runs
with preparation/root coverage held out and ordinary nomination counts and
balls already prepared. Comparing zero remaining ball budget with the full
budget separates numerical recovery from added local proof work. The two
stages retain identical numerical/refinement counts. There are no runtime
timing hooks or extra benchmark DLL exports, and no counter instrumentation
in any timed target. These isolated costs need not sum exactly to an actual
query because compilation/call contexts and allocation scopes differ.

| Operation | Configurations | Median us | p95 us | p99 us | Max us |
| --- | ---: | ---: | ---: | ---: | ---: |
| exact_prepare | 148 | 13.398 | 13.624 | 13.863 | 13.900 |
| root_cover | 148 | 169.385 | 40779.690 | 49074.905 | 49491.400 |
| numerical_nomination | 147 | 11.486 | 17.474 | 18.749 | 19.687 |
| candidate_balls | 143 | 370.471 | 750.960 | 764.491 | 767.400 |
| phase_assembly | 148 | 5.026 | 20.553 | 20.977 | 21.283 |
| exceptional_numerical | 41 | 77.829 | 86.803 | 87.626 | 88.082 |
| exceptional_with_balls | 41 | 524.070 | 586.611 | 1112.944 | 1188.160 |
| command_full | 148 | 11.290 | 21.524 | 27.088 | 585.900 |

The 41 isolated exceptional configurations retain 216 attempts, 81
refinements (44 unsuccessful), 360 numerical witnesses and 160 local proof
instances with certificates enabled. These are benchmark configuration
counts, separate from the 432-target audit. Root work drives the large tail;
bounded failure work is included rather than omitted.

## Actual catalogue API costs

| Candidate coverage | Queries | Parent p50 us | Candidate p50 us | Parent p95 us | Candidate p95 us | Candidate p99 / max us |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| Complete | 96 | 596.300 | 598.928 | 991.987 | 1009.610 | 1166.957 / 1197.300 |
| Inconclusive | 30 | 41214.100 | 41780.350 | 41385.920 | 42010.865 | 42172.073 / 42226.100 |

Parent and candidate global coverage states match for all 126 public
configurations. Complete-query phase records match; all 30 inconclusive
configurations retain different local proof-record counts after recovery.
Additional records are not a distinct-solution or completeness claim.
The challenge-rich cohort is deliberately split by outcome; its combined
tail is not a production-frequency estimate.

| Typed operation | Queries | Median us | p95 us | Max us |
| --- | ---: | ---: | ---: | ---: |
| python_disabled | 126 | 166.844 | 220.946 | 241.200 |
| python_enabled | 126 | 1329.475 | 42010.950 | 42347.600 |
| python_enabled_with_strata | 126 | 1332.412 | 42266.675 | 42461.900 |
| python_parent_catalogue | 126 | 1334.888 | 41382.375 | 41708.800 |

## Ordinary solver regression and costs

The immediate parent comparison covers 4,313 original and 3,024 stress poses
in both seed modes at capacity 512, nine rounds and eight repeats. All
14,674 paired results and 337,498 ordered binary64 command rows, including
best rows, remain exactly identical. There are no added/removed rows, count
changes, lost strict solves, lost known witnesses or invalid outputs. Every
matched per-case refinement-counter record agrees, not merely totals.

| Corpus / seed / population | Strict solves / witness hits, each binary | Parent p50 / p95 us | Candidate p50 / p95 us | Parent p99 / max us | Candidate p99 / max us |
| --- | ---: | ---: | ---: | ---: | ---: |
| existing / no_seed / all_reachable | 3929 / 3726 | 18.663 / 94.410 | 18.688 / 94.250 | 148.081 / 261.100 | 146.892 / 247.588 |
| existing / no_seed / mixed_singular | 384 / 155 | 88.431 / 157.152 | 88.088 / 157.911 | 195.628 / 261.100 | 195.922 / 247.588 |
| existing / nearby_seed / all_reachable | 3929 / 3726 | 18.637 / 94.535 | 18.700 / 94.032 | 146.039 / 249.700 | 147.622 / 255.525 |
| existing / nearby_seed / mixed_singular | 384 / 155 | 88.337 / 158.469 | 88.369 / 160.510 | 197.588 / 249.700 | 194.637 / 255.525 |
| stress / no_seed / all_reachable | 3024 / 2994 | 20.656 / 114.534 | 20.675 / 115.321 | 151.677 / 306.700 | 149.524 / 291.700 |
| stress / no_seed / mixed_singular | 432 / 432 | 94.125 / 150.949 | 94.144 / 149.436 | 178.723 / 195.012 | 175.352 / 201.350 |
| stress / nearby_seed / all_reachable | 3024 / 2994 | 20.600 / 114.724 | 20.613 / 115.468 | 150.729 / 287.738 | 148.754 / 284.825 |
| stress / nearby_seed / mixed_singular | 432 / 432 | 93.669 / 147.496 | 93.269 / 148.470 | 176.614 / 211.512 | 177.654 / 196.188 |

Raw reports also retain challenging/base-axis timing and all error populations,
all output rows, paired case identities and independent untimed counters. Small
timing differences do not establish a speedup or a regression acceptance
threshold; no latency budget was supplied.

## Reproduction and frozen identities

From a clean checkout with declared locked dependencies and an immutable
parent binary/counter executable, build the opt-in drivers before timing:

```powershell
cmake --build --preset windows-clang-release-tidy --target crx_catalogue_benchmark crx_solver_benchmark crx_solver_counters
uv run --locked python scripts/validate_exceptional.py --library build/Release/crx_kinematics.dll --per-asset 64 --nearby 4 --jobs 3 --output build/evidence/exceptional-audit
uv run --locked python scripts/benchmark_catalogues.py --candidate build/Release/crx_kinematics.dll --parent PARENT_DLL --parent-revision 427307200057212c9523a907928dddd1340d577e --time --rounds 9 --minimum-ms 5 --output build/evidence/exceptional-feature
uv run --locked python scripts/benchmark_accuracy.py --before PARENT_DLL --after build/Release/crx_kinematics.dll --corpus existing --rounds 9 --repeats 8 --capacity 512 --before-counters PARENT_COUNTER_EXE --after-counters build/Release/crx_solver_counters.exe --output build/evidence/exceptional-existing
uv run --locked python scripts/benchmark_accuracy.py --before PARENT_DLL --after build/Release/crx_kinematics.dll --corpus stress --rounds 9 --repeats 8 --capacity 512 --before-counters PARENT_COUNTER_EXE --after-counters build/Release/crx_solver_counters.exe --output build/evidence/exceptional-stress
```

Run the audit and untimed verification before timing; stop all other builds,
tests, oracles and benchmarks for measured runs. Use fresh output directories.
The runner records full commands, seeds, binary/source/input hashes, CPU and
affinity metadata, compile commands, raw rounds and outcomes. The ordinary
runner additionally archives matching Eigen header hashes.

- Runtime source checkpoint: `26456dcde8e674d18e97043aee1160d82ee811af090861c2b1c21a13fcb763bb`.
- Measured runtime DLL SHA-256: `d1d77556b13323ef9bfb605bb4bf45bf9f9fad03e0fe9f8df4ec4ddad4b1e0a8`.
- Benchmark-target build identity: `ccbb9c15f8af7659da4c2af0138fbb0a0f063a2f3d41cf94eea102aa4275aa49`; the runtime DLL is byte-identical.
- Benchmark tool identity: `d5bb0aa8f7bd3d455e179868e0b6861687cabb9a7e208bae3389ff3edeb42528`.
- Parent commit: `427307200057212c9523a907928dddd1340d577e`; DLL SHA-256 `aca4d3d566131a20c6170be14fab60daac703595566dec71b6a1b0ff5471c38b`.
- Asset-audit manifest identity: `ca702440fc2f9f9524f0aa659d09be4ea9012431d477b8fe03ba050fc973e6b6`.
- Asset-case artifact SHA-256: `4c67b2383e43b54409cdf925253a1c9dbbefeee0f4cb92de3c9df62808b269d9`.

The only production-path difference after the runtime freeze is excluded
benchmark-target CMake registration. Final Release verification passes again
after those additions. The final report is updated after measurements; runtime,
wrapper, oracle and benchmark algorithms are unchanged. No asset, fixture or
vendored source edits, deployment, merge or history rewrite are included.
Exact continuous-family classification/parametrization remains deferred, and
a failed certificate or numerical search remains inconclusive.
