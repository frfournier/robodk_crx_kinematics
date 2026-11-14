# Calibrated IK implementation and validation

The separate calibrated model, bounded numerical correction, local certificates
and additive C/Python APIs are implemented and validated. Frozen native/Python
benchmarks and comparisons against the actual published Local certificates
parent are recorded below. The optional offline tolerance report is deferred
with measured evidence. The existing nominal solver's policy and public layouts
are unchanged; its local-ball checker is shared internally.
There is no calibrated completeness or infeasibility claim, and this C++ code is
not formally verified.

## Declared model

The calibrated model accepts six revolute modified-DH rows, with the convention

`A_i(x_i) = Rx(alpha_i) Tx(a_i) Rz(theta_i + x_i) Tz(d_i)`.

The full forward relation is

`F(x) = base A_1(x_1) ... A_6(x_6) tool`.

Every angle is declared as `quarters*pi/2 + radians`, where `quarters` is an
integer and `radians` is the exact represented binary64 correction. Quarter
turns are evaluated by exact permutation/sign changes of enclosed sine and
cosine. They are never inferred from a tolerance. Arbitrary represented radians
are supported by setting the quarter count to zero. The two declarations can
describe slightly different exact geometry when a binary64 approximation to
`pi/2` is used; neither declaration is silently snapped to the other.

Base and tool rotations use nonzero finite quaternions, in `w,x,y,z` order.
Their exact homogeneous quaternion action is divided by the squared quaternion
norm. No rounded quaternion normalization defines the exact model. Positions,
DH lengths and the positive represented `length_scale` use one declared length
unit. All positions and lengths are divided by that scale before evaluation.

The physical DH increments are `x_i = senses_i*u_i`, with exact senses `+1` or
`-1`. Here `u` is already a coupled RoboDK command in radians, matching the
existing native solver boundary. A user-coordinate adapter must apply any
decoupled-to-coupled map before this boundary. Applying a third-joint coupling
again would change the model. Joint bounds retain raw represented values and an
explicit degree/radian unit tag. Radian bounds are exact point intervals;
degree bounds are enclosed as `raw_degrees*(Pi()/180)`. Whole-ball command
admission uses those enclosures without silently substituting rounded radian
bounds. Numerical point membership is conservative at an uncertain endpoint.

This is a general six-revolute DH model. Nonorthogonal twists, off-pattern DH
offsets and arbitrary base/tool frames do not pass through the nominal CRX
signature recognizer. No nominal symmetry or aspect formula is assumed.

## DH-to-PoE derivation

Put `B_i = Rx(alpha_i) Tx(a_i)` and
`C_i = B_i Rz(theta_i) Tz(d_i)`. Let
`G_0 = base`, `G_i = G_(i-1) C_i`, and `M = G_6 tool`.
Let `Z = (0,0,0;0,0,1)` be the translation-first unit z-axis twist.

Since z rotations commute with z translations,

`A_i(x_i) = C_i exp(Z*x_i)`.

Conjugation gives
`G exp(Z*t) G^-1 = exp(Ad_G(Z)*t)`. Inserting the home prefixes successively
therefore yields

`F(x) = exp(S_1*x_1) ... exp(S_6*x_6) M`,
`S_i = Ad_(G_i)(Z)`.

The same screw can be read before the row's z rotation/translation:
`Ad_(G_i)(Z) = Ad_(G_(i-1) B_i)(Z)`, because `Rz(theta_i) Tz(d_i)` fixes `Z`.
The stored home axis and origin are thus

`omega_i = R_(G_(i-1)B_i) e_z`,
`o_i = p_(G_(i-1)B_i)`,
`S_i = (o_i cross omega_i; omega_i)`.

Each axis is mathematically unit length because the defining transform is a
product of exact rotations. The implementation encloses those components; it
does not normalize a rounded screw and substitute a different exact axis.
The PoE expression uses Rodrigues rotation and
`p(exp(S_i*t)) = (I-R(exp(S_i*t))) o_i`.

`ForwardDh` and `ForwardPoe` implement these separate expressions. The enclosed
centre evaluation follows the actual moved DH prefixes. If the resulting flange
pose is `(R,p)` and the moved joint line is `(omega_i,o_i)`, the body Jacobian is

`J_i = (R^T [omega_i cross (p-o_i)]; R^T omega_i)`.

The centre also retains those moved unit screws for subsequent rebased local
ball checks. The nominal closed-form determinant is deliberately unavailable.
Any later calibrated determinant sign must be a local label obtained from the
calibrated Jacobian; it cannot establish a nominal aspect or global coverage.

## Typed calibrated local certificates

The nominal and calibrated paths now share the same internal local-ball
inequality checker. The common `Bounds` contain only theorem bounds. The nominal
certificate still binds those bounds to its nominal exact-problem identity;
the calibrated certificate binds them to a distinct typed problem containing an
owned calibrated model and all seven raw target quaternion/position words.
The checker alone cannot manufacture either sealed certificate.

For the enclosed calibrated centre Jacobian `J`, a represented proposal `X` must
pass `e = ||I-XJ||_infinity < 1`. The verified inverse bound is
`beta = ||X||_infinity/(1-e)`. The represented preconditioner's determinant gives
the local Jacobian sign only after that inverse-residual check. The nominal
closed-form determinant cross-check remains active for nominal kinematics;
it is unavailable for arbitrary calibrated geometry.

Use the moved unit screws and anchor `a = F(x0)` to rebase
`F(x0+h) = exp(S_1^c*h_1) ... exp(S_6^c*h_6) a`.
The existing regional anchored-log Lipschitz bound is evaluated on that exact
rebased model. The residual bound `eta` includes the inverse-residual factor
`1/(1-e)`, including when the tighter matrix-log series is used. The checks are

`R*sum_i ||omega_i|| < pi`,
`beta*L*R < 1`,
`0 <= rho <= R`,
`beta*L*rho^2/2 + eta <= rho`.

Together with the enclosed target log-domain check, these establish one unique
calibrated root in the closed increment-space infinity ball `R`, and locate it
in the existence ball `rho`. A failed hypothesis is inconclusive. The regional
formula, preconditioner checks and residual arithmetic retain the existing
nominal operation order. No calibrated completeness theorem is inferred.

Command admission is separate. For `u0_i = senses_i*x0_i`, the minimum raw command
reserve is bounded below by outward evaluation of

`m = min_i (u0_i-lower_i, upper_i-u0_i)`.

The exact diagonal senses preserve the infinity radius, so `m >= rho` admits
the entire root-enclosing command ball. Limits apply to these real commands;
they are not reduced to phases. An uncertain boundary or negative reserve does
not disprove a calibrated root or establish global infeasibility. A local root
certificate can therefore be valid while `commands_admitted` is false.

The calibrated result retains separate Jacobian signs for physical increments
and command coordinates; the latter includes the product of the senses. Both
are local labels. Reuse requires the identical full raw model identity and
rechecks every theorem and command-admission bound at the new target. Equivalent
quaternion signs keep distinct raw target identities, even when a full recheck
can certify the same geometric target.

## Numerical correction

A separate numerical translation unit prepares rounded DH caches and target
poses for proposals. Those caches do not define the exact calibrated geometry.
The typed target retains the exact calibrated problem, and a full raw model
identity check rejects stale cache/target combinations.

Correction operates on physical joint increments and preserves real command
turns through the exact unit senses. A bounded Newton step uses the calibrated
body Jacobian and a body-frame translation/rotation-vector residual. The step's
infinity norm is capped; a bounded halving search accepts only a smaller
numerical residual. Default ceilings are 24 accepted updates, 12 halvings per
update and a 0.25-radian maximum step. Input ceilings are 128 updates and 32
halvings. Evaluation counts include the initial FK and every trial, and cannot
exceed `1 + max_iterations*(max_backtracks+1)`.

Convergence checks separate normalized position and orientation errors. The
default tolerances are `1e-13` in normalized length and radians respectively.
Neither tolerance defines exact acceptance. Numerical limit membership, exact
local-root certification and whole-ball command admission are separate facts.
Iteration exhaustion, a singular proposed step, arithmetic failure and failed
line search remain explicit inconclusive numerical outcomes. No phase wrapping
or nominal symmetry formula changes a corrected result.

Proof enablement does not participate in the correction stop rule. A requested
proof recomputes the exact calibrated enclosures at the final represented
increment centre. Tests require identical binary64 corrected commands and work
counts with proofs enabled and disabled. Loose numerical dispatch tolerances
cannot turn an approximate candidate into an exact-root certificate.

On each approved asset, a synthetic perturbation changes DH twists/zeros and
link offsets. Actual nominal discovery supplies a regular partner pair; each
seed is corrected separately and receives its own independently audited local
certificate. The tests additionally apply the nominal base flip to one
corrected result and verify that it misses the perturbed target. This directly
exercises broken calibrated symmetry. These selected regular pairs are not a
coverage audit or a measured production success rate.

## Identity and arithmetic boundary

The calibrated model has its own identity type and schema, separate from the
nominal exact-problem identity. Schema 2 has 70 words retaining all raw row angle
declarations, lengths, senses, frame quaternions/positions, limits, normalization
scale and the bound-unit tag.
Integer quarter declarations and binary64 bit patterns are recorded directly;
even signed-zero differences remain distinguishable. Identity equality does
not depend on a finite hash. Prepared models own their inputs.

Preparation and evaluation return `Ready`, `InvalidInput`,
`UnsupportedEnvironment` or `ArithmeticFailure`. Every failure leaves the output
unchanged. A ready enclosure is not a feasibility or root certificate. Negative
tests exercise reversed bounds, invalid senses, zero quaternions, nonfinite
geometry, invalid scales and nonfinite centres. Unsupported directed rounding
and flush-to-zero modes fail closed and preserve the prepared output.

The calibrated model and its native probe are compiled with the established
strict floating-point/no-LTO enclosure policy. The ordinary numerical solver
retains its existing optimization policy.

## C and Python API boundary

The additive `crx_cal_v1_` ABI has separate calibrated model/query handle types.
It does not reinterpret nominal handles or expose nominal certificate layouts
as calibrated certificates. Six C-compatible records negotiate exact size,
version 1 and zero reserved fields:

| Record | Windows x64 bytes | Purpose |
|---|---:|---|
| model input | 528 | Declared DH, quarters, senses, frames, raw limits/units, scale |
| target | 80 | Exact raw homogeneous quaternion and position |
| correction options | 64 | Work limits, numerical dispatch, optional local proof |
| summary | 88 | Per-seed counts/work and always-inconclusive coverage |
| identity | 656 | All raw calibrated model and target identity words |
| attempt result | 608 | Numerical outcome, separate proof/admission and local bounds |

The header passes a C compiler syntax/layout check, not only C++ compilation.
Creation/read/evaluation failures leave caller outputs unchanged. Inputs and
results use caller-owned nonoverlapping buffers; no array pointer is retained.
Model preparation owns its declared geometry. Synchronous query creation owns
every attempt and identity; query readers remain usable after model destruction.
Raw C callers must finish calls before destroying a handle. Python serializes
calls and closure, materializes owned snapshots, then destroys the native query.

The query accepts at most 512 proposed coupled-radian seeds. Every seed is
corrected independently; none is synthesized by a nominal symmetry formula.
The total numerical evaluation ceiling is
`seed_count*(1+max_iterations*(max_backtracks+1))`, at most 2,163,200 under the
supported input ceilings. Fixed-size native owned payload is 728 bytes plus
1,392 bytes per attempted seed on the checked Windows toolchain: 713,432 bytes
at 512 seeds. This excludes allocator overhead, caller buffers, transient
preparation/correction stack work and Python snapshot storage. The 608-byte
public reader layout is separate from that internal owned record size.

Summary counts are per-seed instances, including repeated seeds. Numerical
convergence, conservative point-limit membership, local proof and certified
whole-ball admission are separate. Coverage is always `Inconclusive`, including
empty queries and queries where every local attempt certifies. Numerical FK and
body Jacobian entry points make no root or limit-enforcement claim. Commands
are radians in C; public matrices are row major and Jacobian translation units
are millimetres per command radian.

`CalibratedSolver` is an optional NumPy API associated with a nominal `CrxSolver`
used only for seed proposals. Its `ik` requests bounded nominal discovery then
corrects each seed; `correct` accepts explicit proposed command-degree rows.
The complete declared `base*DH*tool` relation and target share one output frame;
no nominal reference/TCP transformation is silently composed into this model.
Explicit homogeneous quaternion frames, or recorded matrix-to-quaternion
conversions, define the exact rotations used by the proof checker.

With proofs enabled, Python `solutions` selects only attempts whose exact local
root is both certified and admitted. Other numerical candidates remain in
owned attempt diagnostics. Proof-disabled results are explicitly numerical.
All attempts preserve source indices and submitted degree seeds; raw increment
and command centres are retained in radians. Model/target identities compare
raw values, and equivalent quaternion signs can have distinct identities.
Arrays are owned read-only snapshots independent of caller target buffers.
Copying a live calibrated handle is rejected. Explicit calibrated correction
still works after closing the external nominal handle; automatic nominal-seed
discovery needs that nominal solver open.

The actual published Local certificates DLL, SHA-256
`6f47dce683c0d9e51de853cb1d47f15477a415b62edf4b28a8a656e947a48336`, was checked
with the new package on all six approved models. Nominal FK, seeded IK and local
certificates still work, and requesting missing calibrated exports raises
`NotImplementedError` explicitly. Imports do not load a native library.

## Independent validation

The independent oracle uses 110–160 decimal digits for modified-DH products,
PoE products and a body Jacobian obtained by differentiating homogeneous
transform products. It does not reuse the production cross-product Jacobian.
Tests cover all six approved captures, zero perturbations, deliberately generic
nonorthogonal chains, signed senses and already-coupled commands, nonidentity
frames, length scales `1e-6`, `1`, `1e6`, real turns, extreme quarter declarations,
quaternion scales `2^-500` to `2^500`, raw degree/radian limits, singularity
approach, continuous coaxial fibres, failed correction budgets and stale models.
Every positive proof in the frozen benchmark is independently checked at 160
digits: inverse residual/norm/sign, screw/anchor bounds, anchored logarithm,
regional constants, every radius inequality, full exact-pose root, containment
in the existence ball and admitted command bounds.

The 63 frozen synthetic cases produce 699 seed instances and 2,796 audit rows
(component correction and actual DLL, each with proofs disabled/enabled).
All 181 distinct positive proof records pass that independent audit. Identical
repeated evidence is checked once; the 512-seed control repeats one root.
Enabling proofs preserves every corrected command/increment bit and work count.
Python matches the DLL's command bits, work and local/admission flags. Automatic
Python discovery reproduces the ordered seeds frozen from the actual parent.
All 63 query snapshots retain `Coverage::Inconclusive`, including empty queries
and those with every supplied seed locally certified.

Final Release verification passes formatting, the clang-tidy build and all nine
CTest entries: **17,189 Python passes, 833 opt-in skips and 8,640 reliability
checks**. Matching full Debug has **17,184 passes, 833 skips and exactly five
inherited diagnostic-capture failures**; all native entries and 8,640 reliability
checks pass. The exact failed-name set matches the reproduced parent:

- `test_invalid_model_diagnostic_has_complete_name_buffer`;
- `test_diagnostic_character_roundoff[-64]`, `[-1]`, `[1]`, `[64]`.

No check was disabled or weakened. The focused calibrated suites contain 41
core tests, 24 C ABI tests and 26 public wrapper tests. They include size/version/
reserved canaries, zero/max-seed ownership, atomic failure outputs, proof/limit
separation, immutable identities, concurrent close/read, and optional-symbol
fallback. A real C translation unit compiles the public header and asserts all
six Windows x64 record sizes.

Eight proof compile-command instances retain strict floating point, disabled
contraction and no LTO. Five optimized proof objects were inspected without
fused arithmetic. Their runtime sources are byte-identical to that audited API
checkpoint. Static comparison preserves the nominal bound arithmetic sequence;
only typed sealing and the unavailable calibrated closed-determinant check
differ. Ordinary numerical optimization settings were retained.

Wheel/sdist checks compare all six Python modules byte for byte, `py.typed` and
the packaged guide. Import from the actual wheel executes the corrected nominal
10iA DH example, proposing eight seeds and retaining eight certified/admitted
local instances after both models close. These are per-seed example outcomes.
The guide's third-row twist is zero, checked directly against nominal FK.
Qmake and live RoboDK/deployment checks remain unvalidated; Windows CMake/Ninja
is the supported tested path. The 833 skips include external-baseline and live
RoboDK tests whose opt-in inputs were not supplied.

## Frozen measurement protocol

Measured 2026-11-14 02:16–02:20 UTC on 12th Gen Intel(R) Core(TM) i7-12700H, Windows x64,
clang-cl/LLVM 22.1.3, CMake 4.3.1, Ninja 1.13.2, Python 3.14.8, NumPy 2.5.3,
mpmath 1.3.0 and Eigen 5.0.1. The feature timing thread was pinned to CPU mask
`1` within process mask `1048575`; the ordinary harness also pins its thread.
OMP/OpenBLAS/MKL thread settings were `1`. All builds, tests and oracles were
terminal before timing; feature native, feature Python, existing-corpus and
stress-corpus runs were strictly serialized. Source/binary guards passed before
and after every phase. No product source changed during measurements.

There are nine randomized feature rounds, with repetition counts calibrated to
at least 5 ms per configuration, and nine alternating/randomized ordinary
rounds with eight repeated DLL calls per record. Tables report quantiles across
**per-case medians of nine rounds**, not pooled repetitions. Six-case feature
tails are descriptive interpolated quantiles, not production percentiles.
No statistical speedup claim is made from small timing differences.

Native component costs call the shipped model/solver/certificate libraries;
the harness alone has LTO disabled to preserve call boundaries. Public native
query costs dynamically load the explicitly identified DLL and include query
creation, summary/identity/all-result reads and destruction. Numeric correction
components use cached model/target preparation. Python costs include validation,
unit conversion, native calls and owned NumPy/result decoding. `python_ik`
additionally includes nominal discovery. Python model preparation includes
creation/close of the calibrated handle; the associated nominal handle is
already prepared. Caller inputs and caches are allocated before timed loops.
Counters run separately in matched untimed executables; no counter define is
present in either timed DLL or driver.

Measurement source identity: `22cee526ec0cd4f60fd10737e5507570ddbd3016ff14919df6e020bb70fb957a`.
Runtime API source checkpoint: `e7343f19ee81e433175604aba8b604e0fca7d4e1435af9e70cd59d7bff59feff`.
Candidate DLL SHA-256: `9769109083b4aa2ffb3161c7a65d2f31a4369833ad76a37a7a53b2303369261f`.
Actual parent commit: `7a1d4c39eca249a2ef75ef440ee04e5b65a09ab9` (Local certificates #24).
Actual parent DLL SHA-256: `6f47dce683c0d9e51de853cb1d47f15477a415b62edf4b28a8a656e947a48336`.
Documentation and final review records may subsequently change; these identities
describe the actual measured runtime/build, not a later report hash.

## Synthetic populations, outcomes and work

All six approved assets are represented in each ordinary feature group.
`zero`, `small` and `frames` use eight ordered actual-parent nominal proposals
per asset. The small perturbation adds alpha/theta corrections up to `2.5e-5`
radians and offsets up to `0.03 mm`; these are **synthetic declarations, not
measured calibration data**. Frame cases compose a declared nonidentity
reference and TCP with the captured geometry. Other six-asset groups use one
explicit proposal per model. Scaled lengths use a factor of `1e-3`, senses use
a different exact sign pattern, and turn controls retain a 720-degree first-axis
offset. Tight bounds are within `1e-12` degrees of the generating command.
Near-singular controls set the nominal wrist command to `1e-8` radians before
perturbation; this does not assert distance to a calibrated singularity.
The separate literal coaxial chain is singular.

Counts below are seed instances. They are neither distinct-root counts nor
production success rates. The distant seed can converge to an out-of-limit root;
that root remains locally valid and is not selected as admitted IK.

| Group | Queries | Seeds | Converged | Local proofs | Admitted | Max updates / evaluations |
|---|---:|---:|---:|---:|---:|---:|
| coaxial | 1 | 1 | 0 | 0 | 0 | 0 / 1 |
| empty | 1 | 0 | 0 | 0 | 0 | — |
| far_seed | 6 | 6 | 6 | 6 | 0 | 18 / 19 |
| frames | 6 | 48 | 48 | 48 | 48 | 2 / 3 |
| iteration_budget | 6 | 6 | 0 | 0 | 0 | 0 / 1 |
| near_singular | 6 | 6 | 6 | 6 | 6 | 3 / 4 |
| scaled | 6 | 6 | 6 | 6 | 6 | 3 / 4 |
| seed_cap | 1 | 512 | 512 | 512 | 512 | 0 / 1 |
| senses | 6 | 6 | 6 | 6 | 6 | 3 / 4 |
| small | 6 | 48 | 48 | 48 | 48 | 2 / 3 |
| tight_limits | 6 | 6 | 6 | 6 | 0 | 3 / 4 |
| turns | 6 | 6 | 6 | 6 | 6 | 3 / 4 |
| zero | 6 | 48 | 48 | 48 | 48 | 0 / 1 |

The six zero-iteration displaced-seed controls return `IterationBudget`; the
coaxial control returns `SingularStep`. No correction backtracks were needed in
this selected corpus; separate regressions exercise rejected steps and failure
budgets. The seed-cap control supplies 512 identical exact generating commands,
so convergence and proofs occur without an update. Its 512 local instances are
one repeated solution. Maximum observed updates/evaluations are 18/19.
Configured work remains bounded by `1 + max_iterations*(max_backtracks+1)`
per seed, with at most 512 seeds, 128 updates and 32 halvings.

The native query's measured logical owned payload is `728 + 1392*N` bytes:
728 for zero seeds, 2,120 for one and 713,432 for 512. This excludes allocator
overhead, caller buffers, transient stack data and Python snapshots; it is not
a peak-RSS measurement. Python returns independently owned bounded snapshots.

## Calibrated timing

Per-query medians in microseconds; preparation is excluded from correction/query columns.

| Group | Native proofs off / on | Python correction off / on |
|---|---:|---:|
| coaxial | 1.118 / 1.117 | 72.341 / 69.316 |
| empty | 1.123 / 1.124 | 53.655 / 53.022 |
| far_seed | 10.897 / 61.449 | 84.070 / 153.688 |
| frames | 11.872 / 442.934 | 169.453 / 684.878 |
| iteration_budget | 1.517 / 1.525 | 74.305 / 74.964 |
| near_singular | 2.975 / 53.027 | 76.905 / 144.905 |
| scaled | 2.981 / 52.893 | 76.612 / 146.385 |
| seed_cap | 336.544 / 26268.300 | 6254.500 / 36051.500 |
| senses | 2.939 / 52.893 | 75.534 / 148.182 |
| small | 11.921 / 428.044 | 169.798 / 663.622 |
| tight_limits | 2.971 / 52.733 | 77.463 / 143.679 |
| turns | 2.992 / 52.874 | 75.145 / 143.566 |
| zero | 3.842 / 398.212 | 163.488 / 639.572 |

Small-perturbation group (six models, eight seeds/query), including tails in microseconds:

| Operation | Median / p95 / p99 / maximum |
|---|---:|
| api_disabled | 11.921 / 12.109 / 12.151 / 12.162 |
| api_enabled | 428.044 / 434.177 / 435.575 / 435.925 |
| api_prepare | 37.680 / 38.468 / 38.572 / 38.598 |
| correct_disabled | 10.091 / 10.567 / 10.666 / 10.691 |
| correct_enabled | 426.559 / 437.927 / 438.310 / 438.406 |
| kinematics | 1.375 / 1.425 / 1.433 / 1.435 |
| prepare_model | 36.377 / 37.806 / 37.942 / 37.975 |
| prepare_solver | 0.214 / 0.217 / 0.218 / 0.218 |
| prepare_target | 0.992 / 1.021 / 1.025 / 1.026 |
| python_correct_disabled | 169.798 / 171.145 / 171.179 / 171.188 |
| python_correct_enabled | 663.622 / 694.987 / 700.717 / 702.150 |
| python_ik_disabled | 421.519 / 441.703 / 444.571 / 445.288 |
| python_ik_enabled | 980.100 / 1007.156 / 1012.591 / 1013.950 |
| python_prepare | 208.859 / 212.518 / 212.986 / 213.103 |

For this group, the paired native API proof-cost ratio has median 36.031; added cost has median 415.988 µs/query. The paired Python correction ratio has median 3.894. This is the cost of recomputing exact enclosures and checking each of eight local roots; proofs do not change numerical stopping or work.

Cold calibrated preparation amortized over repeated small-model queries (microseconds/query, medians across six models):

| Queries/model | Native prepare + proof query | Python prepare + proof correction |
|---|---:|---:|
| 1 | 465.757 | 872.481 |
| 10 | 431.831 | 684.508 |
| 100 | 428.422 | 665.710 |
| 1000 | 428.082 | 663.831 |

## Ordinary nominal API versus actual parent

Both original corpora were regenerated and checked against their frozen
fixtures: 4,313 existing poses and 3,024 independent stress poses, each without
and with a nearby seed. All 14,674 paired cases preserve counts and every field
of the separate matched counter probes. All 337,498 ordered returned-solution
and best-command rows are binary64-identical. The independent captured-model
pose, limit and witness checks therefore also remain identical.

Per-case timing quantiles (microseconds) include the complete public DLL solve
call, including adaptation, discovery, limits, FK acceptance, ranking and packing.

| Corpus / mode | Parent median / p95 / p99 / max | Candidate median / p95 / p99 / max | Paired candidate/parent median / p95 / p99 / max |
|---|---:|---:|---:|
| existing / no_seed | 19.087 / 95.200 / 147.974 / 266.975 | 19.012 / 95.362 / 148.088 / 254.350 | 0.998 / 1.018 / 1.032 / 1.099 |
| existing / nearby_seed | 19.113 / 95.517 / 148.428 / 269.625 | 19.087 / 95.905 / 148.680 / 257.125 | 0.999 / 1.019 / 1.035 / 1.102 |
| stress / no_seed | 20.538 / 115.951 / 151.057 / 298.363 | 20.500 / 115.742 / 148.252 / 289.962 | 0.999 / 1.021 / 1.037 / 1.089 |
| stress / nearby_seed | 20.587 / 114.616 / 148.091 / 288.350 | 20.550 / 114.405 / 147.740 / 290.925 | 0.999 / 1.019 / 1.031 / 1.066 |

The paired medians are about 0.999. Individual tail ratios vary; these data support preserved ordinary behavior and similar cost on this host, not a general speedup.

Reachable populations and quality (parent and candidate equal):

| Corpus / mode | Reachable poses | Strict solves | Witness hits / supplied | Invalid returned solutions | Position mm p50 / p95 / p99 / max | Angle degrees p50 / p95 / p99 / max |
|---|---:|---:|---:|---:|---:|---:|
| existing / no_seed | 3929 | 3929 | 3726 / 3983 | 0 | 5.89e-13 / 3.77e-07 / 1.97e-06 / 9.27e-05 | 5.31e-14 / 6.48e-08 / 4.37e-07 / 3.27e-06 |
| existing / nearby_seed | 3929 | 3929 | 3726 / 3983 | 0 | 5.88e-13 / 3.76e-07 / 1.99e-06 / 9.27e-05 | 5.3e-14 / 6.46e-08 / 4.37e-07 / 3.27e-06 |
| stress / no_seed | 3024 | 3024 | 2994 / 3024 | 0 | 9.33e-13 / 1.51e-07 / 1.51e-06 / 9.12e-05 | 5.53e-14 / 1.24e-07 / 4.86e-07 / 0.000309 |
| stress / nearby_seed | 3024 | 3024 | 2994 / 3024 | 0 | 9.37e-13 / 1.52e-07 / 1.51e-06 / 9.12e-05 | 5.53e-14 / 1.24e-07 / 4.86e-07 / 0.000309 |

Untimed refinement counters, identical for every paired case (totals across both seed modes):

| Corpus | Runs | Evaluations | Guard/initial rejections | Successes | Recovery failures |
|---|---:|---:|---:|---:|---:|
| existing | 5172 | 12016 | 4174 / 4174 | 798 | 200 |
| stress | 5614 | 11182 | 4870 / 4746 | 612 | 256 |

## Optional offline tolerance report: measured deferral

No DKN-262 admission report ships or enters production calls. A separate
untimed numerical survey evaluates the six frozen small synthetic models at the
same regular nominal generating command, at 160 and 220 decimal digits. All
reported binary64 summaries agree across precisions. For the specified
zero-centred DKN-165/262 constant family, `L(R) >= L(0)`, `b >= ||J0^-1||` and
the declared defect bound must exceed the actual anchored log defect. Thus the
estimated product `2*||J0^-1||^2*L(0)*actual_defect` is an optimistic lower
estimate for the necessary `2*b^2*L*e < 1` condition.

| Synthetic model | Inverse norm | Actual log defect | L at zero radius | Estimated necessary product |
|---|---:|---:|---:|---:|
| CRX-5iA:small | 11.41 | 0.0001663 | 1.8184e+07 | 787426 |
| CRX-10iA:small | 10.03 | 0.0001453 | 1.9543e+07 | 571291 |
| CRX-10iA-L:small | 11.16 | 0.0001143 | 1.5613e+07 | 444743 |
| CRX-20iA-L:small | 11.16 | 0.0001143 | 1.5613e+07 | 444743 |
| CRX-25iA:small | 10.15 | 9.894e-05 | 1.5663e+07 | 319237 |
| CRX-30iA:small | 10.15 | 9.894e-05 | 1.5663e+07 | 319237 |

Products of about 319,000–787,000 motivate deferring this optional tool in
favor of the implemented direct, a-posteriori calibrated-root checker. This
survey is **numerical evidence only**: it supplies no outward enclosures,
tolerance-box derivative budget, log-domain/radius check or limit-admission
hypotheses. It never returns “admitted,” “infeasible” or a rejection certificate.
Passing a necessary inequality would not establish admission. Other stronger
bound families are not ruled out. No measured calibration data is inferred.
The private survey reproducer and JSON evidence are frozen with measurement
inputs; all production functionality is independent of that survey.

## Reproduction and retained evidence

From an x64 Visual Studio Developer PowerShell, with the approved LFS payloads
and Eigen initialized, use the checked-in presets. Deployment stays disabled.
`$parentDll` below must identify the built published F04 commit above; the
parent counter executable must come from that same source/toolchain.

```powershell
uv sync --locked
cmake --workflow --preset windows-clang-release-verify
cmake --preset windows-clang-debug
cmake --build --preset windows-clang-debug
ctest --preset windows-clang-debug
cmake --build --preset windows-clang-release-tidy --target crx_calibrated_benchmark crx_solver_benchmark crx_solver_counters
$env:PYTHONPATH = "python;scripts;tests"
$env:OMP_NUM_THREADS = "1"
$env:OPENBLAS_NUM_THREADS = "1"
$env:MKL_NUM_THREADS = "1"
uv run --locked python scripts/benchmark_calibrated.py prepare --dataset build/calibrated-experiment --parent $parentDll --candidate build/Release/crx_kinematics.dll --driver build/Release/crx_calibrated_benchmark.exe
uv run --locked python scripts/benchmark_calibrated.py verify --dataset build/calibrated-experiment
# Stop all builds, tests and oracles before timing; keep sources/binaries fixed.
uv run --locked python scripts/benchmark_calibrated.py measure --dataset build/calibrated-experiment --rounds 9 --minimum-ms 5 --seed 20261113
uv run --locked python scripts/benchmark_accuracy.py --before $parentDll --after build/Release/crx_kinematics.dll --corpus existing --rounds 9 --repeats 8 --capacity 512 --before-counters $parentCounters --after-counters build/Release/crx_solver_counters.exe --output build/calibrated-ordinary-existing
uv run --locked python scripts/benchmark_accuracy.py --before $parentDll --after build/Release/crx_kinematics.dll --corpus stress --rounds 9 --repeats 8 --capacity 512 --before-counters $parentCounters --after-counters build/Release/crx_solver_counters.exe --output build/calibrated-ordinary-stress
```

Preparation freezes raw represented geometry, exact target declarations,
original degree seeds, native packets, source hashes and loaded binary hashes.
Verification runs outside timing and checks every distinct positive proof using
the tracked independent test oracle. The native driver checks proof-toggle
numeric hashes and stable work/outcomes after warm-up and each timed round.
The scripts require repository test dependencies; they do not depend on private
swarm orchestration. The immutable C/Python record/layout/ownership behavior is
documented in `include/crx_calibrated_api.h` and `docs/python-wrapper.md`.

Local raw evidence is retained under:

- `build/artifacts/calibrated-ik-certification/feature-cases-v2`: cases, packet,
  source/binary manifest, native audit, independent verification, nine-round
  native/Python CSVs and the tolerance survey;
- `build/swarm/artifacts/calibrated-ik-certification/measurement-inputs-22cee526ec0cd4f6`:
  source ZIP, actual DLLs, drivers, compile commands, strict/counter separation,
  full Release evidence and matched untimed counters;
- `build/swarm/artifacts/calibrated-ik-certification/measure-22cee526ec0cd4f6`:
  complete ordinary raw solutions/timings/counters/accuracy summaries, measurement
  plan and analysis JSON;
- `build/swarm/artifacts/calibrated-ik-certification/package-api-v2`: checked
  wheel, source distribution and direct-import verification.

These ignored raw artifacts are local evidence, not package runtime inputs.
This report embeds the principal measurements and limits so review does not
require access to them. All six approved assets and the user's unrelated root
edits remain unchanged. Publication remains a separate authorized step.
