# Verified nominal catalogues and command ranking

The private proof core now constructs exact-problem wrist-root covers and can
assemble a complete nominal phase catalogue from matching local balls. The
reproducible witness has eight wrist roots and 16 distinct solution phases;
independent rational isolation and high-precision full reconstruction agree.
The command core checks raw-degree limits, full turn enumeration and nearest
ranking. An additive owned C/Python query now exposes these proofs, while model
creation retains original degree-limit provenance. The full 1,000-target-per-asset
audit and serial feature/ordinary measurements are complete. The feature remains
uncommitted pending publication authorization. Existing numerical APIs retain
their coverage policy; proof work is opt-in through the new query.

Of 6,000 audited targets, 5,947 completed phase coverage, command admission and
unique nearest selection; 53 remained inconclusive, including 30 exact exceptional
cases. No false claims or unresolved oracle outcomes occurred. Ordinary IK
preserved all 14,674 paired outcomes and all 337,498 ordered binary64 output rows.

Parent: Local certificates commit
`7a1d4c39eca249a2ef75ef440ee04e5b65a09ab9`, PR #24. Its raw exact-target and local
certificate contracts remain unchanged. No asset, fixture or vendored content
has changed. This C++ implementation is tested; it is not formally verified.

## Exact problem and opposite charts

The sealed `exact::Problem` defines one quaternion-rigid nominal target and
normalized model, retaining all 77 raw identity words and the frame schema.
`root::Certify` constructs its own enclosed residual P (nine slots), derivative
P' (eight slots) and determinant Q (five slots). Generic polynomial utility
results cannot be supplied as sealed IK root certificates.

A chart is defined mathematically by `theta=alpha+2 atan(t)`, `t in [-1,1]`.
The retries use `alpha=j*pi/8`, `j=0,...,7`, with outward pi/trigonometric bounds.
The opposite chart is defined by exactly `alpha+pi`. It is constructed by

```
P_opposite(t) = t^8 P(-1/t)
Q_opposite(t) = t^4 Q(-1/t).
```

Thus its coefficients are the reversed original coefficients with alternating
signs. The even fixed degrees make this identity valid through `t=0`, including
degree drops and the original chart's omitted point. No rounded binary64 sum
`alpha+pi` defines the opposite chart. The chart-shift integer and opposite bit
retain the exact meaning; the interval angle is only a descriptor.

The two closed half-circle charts cover the entire circle with disjoint open
interiors. Their shared endpoints must have strictly nonzero enclosed P values.
An uncertain endpoint triggers a shifted attempt, never an exclusion or a
double count. A simple root at a dyadic subdivision point is retained by
choosing a different certified non-root split point.

## Bernstein leaf proofs and budgets

For `P(a+(b-a)s)=sum_j c_j s^j`, the power-to-Bernstein relation is

```
b_k = sum(j=0..k) choose(k,j)/choose(n,j) * c_j.
```

Small integer binomial coefficients are exact; division and all remaining
operations are outward. Subsequent subintervals use interval de Casteljau
subdivision with an enclosed affine split parameter. Fixed degree 8/7/4 is
retained throughout; small or uncertain leading coefficients are never trimmed.

Each leaf is one of:

- Exclusion: all residual Bernstein coefficients have one strict sign.
- Simple root: endpoint P enclosures have strictly opposite signs, and both
  derivative and determinant Bernstein coefficients have strict constant signs.
- Unresolved: any unproved condition, retained with its tangent domain, phase
  descriptor and precise depth/work/leaf/split/degeneracy/arithmetic reason.

Bernstein basis functions are nonnegative and sum to one, proving exclusions.
Opposite endpoint signs give existence by continuity; nonzero derivative on the
whole leaf gives uniqueness and simplicity. The Q sign excludes the exceptional
determinant stratum on a root leaf. The chart transfer has positive derivative,
so every simple P root is a simple wrist root.

The default work budget is 8,192 visited nodes shared across all charts and
attempts, depth 40 and 4,096 retained leaves. Depth is capped at 50; requests
above the explicit integer work limits are rejected. The two charts share the
leaf allowance conservatively. Exhausted work retains the whole pending
frontier, including when the node budget is zero. A coefficient/environment
failure before subdivision carries `entire_circle_unresolved`; empty leaf
arrays never mean that an unavailable polynomial excluded the circle.

Phase descriptors use outward evaluation of `alpha+opposite*pi+2 atan(t)`.
The private atan uses `atan(x)=2 atan(x/(1+sqrt(1+x^2)))` for `|x|<=1`.
After reduction `|y|<0.415`; 32 odd series terms plus the absolute geometric
tail `|y|^65/(65*(1-y^2))` enclose the reduced atan. It relies on no libm
sin/cos/atan accuracy assumption. These are unwrapped ranges; a seam is retained.

## Catalogue assembly and its scope

The nominal reconstruction theorem on `a*b!=0` gives exactly two distinct
configuration phases for each simple wrist root with nonzero determinant. If
the complete circle cover has r root leaves, the whole nominal phase fibre has
exactly 2r elements, all regular. `catalogue::Assemble` accepts only matching
sealed root/ball certificates. It requires 2r sound balls, pairwise disjoint
modulo full turns, before setting `Coverage::Complete`.

For each pair, at least one canonical coordinate must satisfy

```
min_k |centre_a-centre_b-2*pi*k| > rho_a+rho_b.
```

The two existence radii are used separately. The period is mathematical 2*pi.
A numerical nearest integer only proposes a reduction; an enclosure then
validates a three-integer candidate set. At a pi seam the minimum remains
narrow instead of collapsing into a broad principal-angle interval containing
zero. Unsupported large reductions are unavailable, never equality proofs.

The local balls prove that there are at least 2r distinct exact solutions;
the root/reconstruction theorem bounds the total by 2r. Equality proves the
catalogue complete without trusting numerical root labels or count coincidence.
Missing, duplicate, mismatched or invalid balls and exhausted pair work remain
inconclusive. A zero-root full exclusion cover plus zero balls proves nominal
pose infeasibility. An empty numerical search alone cannot do so. Zero
polynomials and common/multiple roots retain unresolved strata.

The opt-in builder nominates from unlimited canonical discovery, using a
rounded midpoint target and lengths solely to propose centres. It supplies no
command limits or output capacity. Every centre still needs its exact-problem
ball. It can therefore fail to complete when numerical nomination or a ball
check fails; this does not alter the exact target or prove infeasibility.
Candidate and pair budgets are separate from the root-cover budget.

## Independent validation

The C++ native suites cover the eight-root/16-phase witness, exact dyadic roots,
multiple roots, common Q roots, constant/zero polynomials, invalid coefficients,
node/depth/leaf exhaustion, zero candidate/pair budgets, duplicate/missing balls,
raw identity changes, strict unsupported FP controls, zero-root exclusion and
phase coverage independent of command bounds.

The independent Python checker recomputes every leaf's Bernstein coefficients
from exact fractions or 200-digit chart arithmetic, validates endpoint/sign
proofs, checks a gap-free partition, and verifies all reported phase enclosures.
Exact rational Sturm isolation independently counts the original raw-target
polynomial's finite roots and its projective-infinity root. A degree-seven
residual with a simple omitted root is explicitly covered and reconstructed.

The full recovery oracle solves the original incidence equations for the elbow,
recovers canonical angles by elementary transform products, constructs the
opposite base configuration independently, and verifies the entire FK pose.
Every successful witness phase matches exactly one returned ball modulo turns;
all ball theorem constants and refined roots are checked independently. Signed,
scaled, raw-frame and unreachable examples are included.

Oracle root isolation and high-precision recovery have explicit unresolved
outcomes. Recovery escalates through 160/240/400/640 decimal digits and retains
all failed attempts; exact multiple/common-root strata are reported separately.
Fault tests force a lower-precision failure and verify a real higher-precision
reconstruction, then force complete exhaustion and require an unresolved result.
No unresolved oracle outcome is counted as agreement. The separate full 1,000-target-per-asset acceptance run is reported below.

The first identity test used unchanged zero default bounds by mistake. Its
fixture was corrected to change actual nonzero bounds; no acceptance condition
was weakened. A later review added explicit whole-circle retention for
pre-subdivision arithmetic failure and regressions ensuring it cannot become an
empty exclusion certificate.

The full Release format/tidy/build/CTest workflow passes all eleven entries:
17,209 Python regressions, 833 opt-in skips and 8,640 reliability checks.
Matching Debug passes all nine native suites and 186 focused arithmetic,
ball, linked-ABI, root-cover, phase and command tests. The new root/phase tests
account for 26 focused tests; command certificates add 18 and the public query
adds 67. All nine actual enclosure compile
commands are strict and non-LTO. Disassembly of the optimized interval, exact,
ball, root, phase and command objects contains no fused multiply-add instructions.
Structural/inherited lint diagnostics remain visible.

## Command mapping and original degree provenance

The actual current callback map in `crx_coordinates.h` is
`q=D*u_callback+offset`, with diagonal signs +/-1. Coupling has already occurred
at the callback boundary; its inverse infinity norm is 1. The decoupled user
bridge instead includes the shear `u_callback[2]=u_user[1]+u_user[2]`; the inverse
row sum for its third coordinate is 2. The appropriate bound must be derived
from the explicit requested convention, rather than applying coupling twice.

`command::Prepare` retains the 12 original represented degree bounds, six seed
values, six positive weights and the explicit profile/seed-presence flag,
together with the exact problem's complete identity. Their 24 raw binary64
encodings remain available; no finite hash replaces equality. Bounds and seed
are converted with an interval enclosing mathematical pi/180. Failure preserves
the previous context. Existing radian identity fields alone cannot reconstruct
the original degree provenance. Runtime model creation retains those original
inputs in its opaque private handle. The legacy limit getter retains its
existing rounded conversion policy.

For a canonical ball, inverse mapping uses `offset_unwrapped` from the sealed
exact problem, without numerical snapping or principal-angle reduction.
Callback coordinates use each ball's radius rho in every row. The explicit
decoupled profile uses 2*rho in the third row and rho elsewhere. These radii
are added to enclosed mapped centres, retaining raw-offset uncertainty.
Both maps are integer unimodular, so full turns still range over Z^6.

## Full turn enumeration and three-state admission

Let B be a phase's base command box and let L,U enclose its raw degree limits.
For each coordinate, every possibly admissible integer turn is retained in

```
ceil(lower_endpoint((L.lo-B.hi)/(2*pi))) <= m
m <= floor(upper_endpoint((U.hi-B.lo)/(2*pi))).
```

These conservative integer ranges include every feasible exact lift. A proven
empty coordinate excludes that phase's command box. Every converted integer
must be within 2^40 in magnitude; checked uint64 products and sums prevent count
overflow. At most 2^20 Cartesian lifts may be visited. All phase ranges are
preflighted before enumeration; budget/range failure retains unresolved range
records and produces neither command infeasibility nor a global nearest claim.

Each lifted box is B+2*pi*m, with the mathematical period enclosed outward.
It is Yes only when every coordinate lies inside the closed limits using
`box.lo>=L.hi` and `box.hi<=U.lo`. One coordinate with `box.hi<L.lo` or
`box.lo>U.hi` proves No. Everything else is Undecided; no tolerance moves an
uncertain boundary. Yes and Undecided rows remain available for audit, while No
rows are counted. Fully resolved zero admissible lifts proves command-box
infeasibility, which remains distinct from nominal pose infeasibility.

## Travel intervals and nearest selection

Each lifted box independently bounds `sum_i w_i*(u_i-seed_i)^2` using its own
transported radius and raw conversion uncertainty. This direct squared-distance
enclosure supplies a sufficient ordering test corresponding to the weighted
norm margin in R-19. It does not assume equal candidate radii. The C++ code and
the added R-18/R-19 reasoning are not formally verified.

A unique nearest claim requires a Yes row whose upper cost is strictly below
every other Yes or Undecided row's lower cost. A boundary-uncertain distant row
can therefore be excluded from nearest selection by a proven travel gap, even
while overall admission remains unresolved. Otherwise, when at least one Yes
row exists, all rows whose lower cost does not exceed the smallest Yes upper
cost form a possible-minimizer set. Multiple entries mean unresolved ordering,
not certified equality of exact costs. With no guaranteed admissible row,
ranking stays inconclusive. Overflow of travel arithmetic also leaves ranking
inconclusive while preserving independently established admission.

Numerical midpoint travel only proposes display order; the proof checks use
interval bounds. No nontransitive interval comparison enters `std::sort`.
The strict winner is placed first after proof. Requested capacity 0/1/64/512
affects only the displayed prefix after full enumeration and ranking; all proof
rows and counts remain available. There is no nearest claim without a seed.

The independent command suite reconstructs exact roots with the rational and
high-precision oracle, maps them through raw senses/zeros and both command
conventions, enumerates every feasible exact turn and checks command/cost
enclosures and global minima. It covers mixed senses and large offsets with
1,024 admissible lifts, boundary uncertainty, a two-element possible-minimizer
band, strict selection despite a distant uncertain boundary, signed-zero raw
provenance, empty limits, missing phases, integer/count/work exhaustion, huge
unreduced encoder zeros and travel overflow. Native checks additionally cover
nonfinite inputs, unchanged context on failure and unsupported FP controls.

## Additive runtime, ABI and Python query

`QueryVerified` validates raw rigid frames and constructs the same exact
forward-reference problem as local certificates. It invokes the sealed phase
and command assemblers separately from its explicitly numerical fallback.
The selected command convention and positive travel weights apply to both;
the numerical decoupled fallback searches a containing callback rectangle then
filters the actual user box. Its search makes no completeness claim. Capacity
zero requests proofs without numerical fallback. Approximate numerical seed
acceptance can coexist with a proof excluding the exact target, since those
predicates differ. No old entry point or public layout changed.

`include/crx_catalogue_api.h` adds an immutable query handle and negotiated
options/summary/command/range/chart/leaf records. Their Windows x64 sizes are
56/1008/200/224/448/456 bytes, with compile-time assertions and linked canary
tests. Options use explicit size/version/reserved negotiation. Unknown layouts,
profiles, budgets and invalid source inputs allocate no handle; failed reads
preserve output bytes. Query ownership is independent of the model. Root charts
retain complete fixed polynomial slots and leaf Bernstein witnesses, including
unresolved descriptors. Phase records reuse the existing 1136-byte ball snapshot
layout, without creating new reusable native ball handles.

The typed `CrxSolver.catalogue()` wrapper supplies owned read-only summary,
phase and command snapshots and lazy indexed audit reads. An RLock protects
the immutable native cache and its exactly-once finalizer. Model closure leaves
the query readable; query closure releases only the cache, preserving snapshots.
Malformed native records trigger cleanup. Older ABI-1 DLLs missing the optional
exports retain numerical fallback, `NotAttempted` coverage and an explicit lack
of original-degree provenance; no rounded getter is relabelled as raw input.

The 67 linked-query regressions independently audit exact rational root counts,
all reconstructed phase matches, exposed polynomial/leaf witnesses, ball bounds,
command boxes, feasible turns and weighted nearest selection. They also cover
raw custom limits whose legacy degree round trip loses bits, both command
profiles, multiple turns, capacity0/1/64/512, approximate fallback, root/candidate/
pair/lift exhaustion, command-versus-pose infeasibility, uncertain boundaries,
cost/range overflow, explicit nonidentity reference/tool/TCP composition on
three models, input immutability, negotiation canaries, invalid indices,
native read concurrency, model-independent ownership, cleanup on decode faults
and optional-export fallback. Existing local-certificate regressions still pass.

The first native integration build exposed a missing strict-library link in the
solver-outcomes test target, which compiles the entire runtime source list.
The target link was fixed; no check was disabled. CMake now links the runtime
and that test to the same root/phase/command libraries. Qmake lists the new
ordinary and strict sources but remains unvalidated because qmake is unavailable.

`uv build --offline` produces a wheel/source distribution. Every packaged Python
module was byte-compared to source; the typing marker and wrapper documentation
are included. Importing the wheel directly and running a complete nearest query
with model-independent lifetime passed. No native DLL is packaged.

## Exhaustive asset acceptance

`scripts/validate_catalogues.py` verifies the six approved asset/capture hashes,
generates captured and independent modified-DH targets, and uses exact rational
Sturm isolation plus 160/240/400/640-digit full reconstruction. Its independent
audit checks every claimed phase/command/nearest result and retains original
input bits, local balls, high-precision configurations and failed oracle
attempts. Unresolved oracle outcomes are never counted as agreement. Resume
requires identical corpus, DLL, runner, wrapper and oracle hashes. Independent
processes can isolate mpmath and native state for the long audit; no timing
measurements run during correctness validation.

| Captured asset | Targets | Finite oracle | Exact exceptional | Complete / unique nearest |
| --- | ---: | ---: | ---: | ---: |
| CRX-5iA | 1000 | 995 | 5 | 992 |
| CRX-10iA | 1000 | 995 | 5 | 986 |
| CRX-10iA-L | 1000 | 995 | 5 | 992 |
| CRX-20iA-L | 1000 | 995 | 5 | 993 |
| CRX-25iA | 1000 | 995 | 5 | 992 |
| CRX-30iA | 1000 | 995 | 5 | 992 |

All 6,000 targets were audited: 5,970 finite oracle cases and 30 exact
exceptional cases. There were no false phase/command/nearest claims and no
unresolved oracle outcomes. Every finite case resolved at 160 decimal digits;
fault regressions separately exercise precision escalation and exhaustion.
The native query completed phase coverage, admission and unique nearest
selection for 5,947 targets: 99.12% overall and 99.61% of finite oracle cases.

The remaining 53 native queries stayed inconclusive: all 30 exact exceptional
cases, 22 finite cases reaching the root depth/work bounds, and one finite
case with a complete root cover but insufficient certified local balls.
None was promoted to infeasibility or nearest selection. The full corpus
includes complete sixteen-phase queries; the independent explicit sixteen-
phase witness and projective-infinity/seam controls also pass.

| Root work over 6,000 queries | Median | p95 | Maximum |
| --- | ---: | ---: | ---: |
| Visited nodes | 20 | 36 | 8,192 |
| Retained leaves | 11 | 19 | 2,053 |
| Maximum depth | 5 | 10 (rounded up) | 40 |

The initial 18/192-case pilots, separate-process 96-case smoke run and
partial-16-to-96 resume check also passed. Resumed per-case records matched
the original records exactly, independent of completion order. Full audit
results, raw source words, local balls, high-precision configurations and
source/environment fingerprints were retained. The later three additional
public frame regressions change no runtime, wrapper or oracle code used by
the completed audit.

```powershell
uv run --locked python scripts/validate_catalogues.py --library build/Release/crx_kinematics.dll --output build/catalogue-validation --per-asset 1000 --jobs 3
```

## Serial measurement results

The timed corpus has 116 configurations over all six captured models,
including random/captured poses, capacities 1/64/512, unseeded and explicitly
decoupled requests, and private sixteen-phase/seam/infinity/degeneracy/work/
boundary/turn/cost controls. All 1,384 eligible native operations and 204
typed-wrapper configurations passed untimed checks first. Timings used nine
randomized rounds, warmup and at least 5 ms of measured calls per batch.
Reported percentiles use the per-configuration median across rounds.

All measurements ran serially after the correctness audit and builds had
finished. Timing binaries contain no discovery-counter instrumentation;
matched counters run separately. Strict command stage bodies are compiled
directly from the shipped implementation in an opt-in, non-LTO benchmark
object; no runtime timing hooks or duplicated proof arithmetic were added.
Copies, prior-result cleanup, output consumption and invariant checks sit
outside isolated stage clocks. Clock overhead is retained. These isolated
costs need not sum to end-to-end cost, which also includes fallback, framing,
ownership, summary and destruction work.

| Isolated stage | Configurations | p50 us | p95 us |
| --- | ---: | ---: | ---: |
| exact_prepare | 116 | 12.910 | 13.398 |
| root_cover | 116 | 158.802 | 39363.200 |
| numerical_nomination | 115 | 10.792 | 16.317 |
| candidate_balls | 112 | 357.404 | 726.063 |
| phase_assembly | 116 | 4.918 | 20.314 |
| command_context | 116 | 0.336 | 0.345 |
| turn_ranges | 93 | 5.680 | 11.480 |
| turn_enumeration | 92 | 5.780 | 11.379 |
| travel_ranking | 92 | 0.378 | 1.075 |
| command_full | 116 | 11.284 | 21.816 |

The challenge-rich population deliberately overrepresents singular controls.
Root proof exhaustion dominates its tail; it is separated below from
completed queries rather than presented as a production-frequency estimate.

| Actual native C API query + summary + destroy | Configurations | p50 us | p95 us |
| --- | ---: | ---: | ---: |
| Complete | 84 | 608.600 | 1075.324 |
| Inconclusive | 18 | 39963.600 | 40683.495 |
| Proof disabled, numerical fallback | 102 | 19.301 | 28.547 |

The typed Python query also includes frame/input validation, owned phase/
command snapshots and native cache closure. Its enabled p50/p95 is
1,322.600/42,167.125 us over all 102 configurations, including inconclusive
controls; disabled p50/p95 is 163.789/218.774 us. Enabled and disabled
numerical fallback arrays matched exactly. Snapshot-array destruction and
outcome validation were outside those clocks; solver creation was excluded.

### Ordinary IK against the committed parent

Baseline is Local certificates commit `7a1d4c39eca249a2ef75ef440ee04e5b65a09ab9`.
The ordinary complete-DLL experiment used capacity 512, nine randomized
rounds and eight repetitions, with independent FK/limit/witness checks.
It covers 4,313 original and 3,024 stress poses in unseeded and nearby-seed
modes: 14,674 paired case/mode results. All 337,498 ordered output rows,
including best rows, match as numeric values and exact IEEE binary64 words.
There are zero lost strict solves, lost witnesses, count changes or invalid
outputs. Every matched per-case refinement-counter record is identical.

| Corpus / mode | Parent p95 us | Candidate p95 us |
| --- | ---: | ---: |
| Original, unseeded | 94.487 | 93.930 |
| Original, nearby seed | 94.297 | 94.505 |
| Stress, unseeded | 114.503 | 114.112 |
| Stress, nearby seed | 113.771 | 114.441 |

These small timing differences are measurements, not a speedup or latency-
acceptance claim. The proof-query cost is reported independently of ordinary
IK. Raw timings, exact outputs, counters, source archives, compile commands
and binary hashes were frozen with the experiments.

```powershell
cmake --build --preset windows-clang-release-tidy --target crx_catalogue_benchmark crx_solver_benchmark crx_solver_counters
uv run --locked python scripts/benchmark_catalogues.py --candidate build/Release/crx_kinematics.dll --parent <committed-parent.dll> --parent-revision 7a1d4c39eca249a2ef75ef440ee04e5b65a09ab9 --output build/catalogue-benchmark --time --rounds 9
uv run --locked python scripts/benchmark_accuracy.py --before <committed-parent.dll> --after build/Release/crx_kinematics.dll --corpus existing --rounds 9 --repeats 8 --capacity 512 --before-counters <matched-parent-counter.exe> --after-counters build/Release/crx_solver_counters.exe --output build/catalogue-existing
# Repeat the accuracy command with --corpus stress and a new output directory.
```

Run timing with other tests, builds and benchmarks stopped. Omit `--time`
from the feature command for untimed verification. Existing output directories
are never overwritten.

Measured runtime DLL SHA-256:
`1d9b216e263ff5809b125360c5d56c173289770f9611ef147fe8aca55ec85b84`.
Its runtime source checkpoint is
`de8f2a7964d349cc7dcf8c146d408de430a08e503def40684faf0fc466f45cc5`;
the benchmark-target build configuration identity is
`cae6e370fbb11c4d1b409d1d07b4a305b99edc396009a536d62d615882e31bca`.
Only benchmark targets differ between those production-path mappings; the
runtime DLL is byte-identical. Final report and three extra frame tests were
written afterward; runtime, benchmark and oracle algorithms are unchanged.

## Reproduction and current limits

Use the supported x64 developer shell, pinned uv environment and CMake presets:

```powershell
uv sync --locked --offline
cmake --preset windows-clang-debug
cmake --build --preset windows-clang-debug --target crx_root_certificate_tests crx_phase_catalogue_tests crx_command_certificate_tests
ctest --preset windows-clang-debug -R '^crx\.(root-certificate|phase-catalogue|command-certificate)$' --output-on-failure
$env:CRXKIN_COVER_PROBE_PATH = "$PWD/build/Debug/crx_root_certificate_tests.exe"
$env:CRXKIN_CATALOGUE_PROBE_PATH = "$PWD/build/Debug/crx_phase_catalogue_tests.exe"
$env:CRXKIN_COMMAND_CERTIFICATE_PROBE_PATH = "$PWD/build/Debug/crx_command_certificate_tests.exe"
uv run --locked pytest tests/test_crx_root_certificate.py tests/test_crx_phase_catalogue.py tests/test_crx_command_certificate.py -v
Remove-Item Env:CRXKIN_COVER_PROBE_PATH,Env:CRXKIN_CATALOGUE_PROBE_PATH,Env:CRXKIN_COMMAND_CERTIFICATE_PROBE_PATH
cmake --workflow --preset windows-clang-release-verify
```

Root, phase and command proof translation units are separate strict-FP, non-LTO libraries.
Numerical discovery is nomination only. Full Debug's five inherited diagnostic-capture failures
remain a baseline limitation; a full Debug green claim is not made. Qmake has
not been exercised. No latency budget was supplied, so no performance
acceptance claim is made. Commit/push/draft-PR publication requires separate
user authorization; no merge or deployment is included.
