# Local existence and uniqueness balls: implementation record

The private ball checker encloses nominal canonical FK and its body Jacobian
at a represented centre, validates a floating preconditioner, and checks a
regional anchored-log existence/uniqueness theorem. A successful payload
identifies one exact nominal problem and two closed infinity balls: the exact
pose root is inside radius `rho`, and is the only pose root inside radius `R`.
The payload does not certify command limits, nearest ranking or global coverage.
Every failed hypothesis is inconclusive.

The opt-in solver, C ABI and Python integration are implemented on Arithmetic
commit `d4a6cc7e2f8afe19b1359a5eeed219147bcc2f57` (PR #23). Legacy entry points
retain their numerical behavior and cost. Matching Debug/Release gates and paired
feature/solver measurements pass; results below identify their scope explicitly.
This document does not claim that the C++ implementation is
formally verified.

## Coordinates, norms and derivative

Use the same sealed exact nominal model/target as the arithmetic foundation:
all translations/lengths are divided by `L=max(|a|,|b|,|c|,|r|)`; target/frame
rotations are their homogeneous quaternion actions. Centres are canonical
radians. Raw command-map offsets and target/model identity remain those of
the exact problem. A local pose ball does not by itself transport command
limits, degree conversion or turns.

The existing base-pose convention retains identity schema 1. A second explicit
convention, schema 2, accepts the wrapper's raw forward reference map, target
and tool without computing a rounded flange pose or inverse-frame translation.
Writing their exact homogeneous quaternion actions as `Rref,Rt,Ru`, it encloses

```
R06 = Rref Rt Ru^T
p06/L = Rref (pt/L) + pref/L - R06 (pu/L).
```

The model adapter for this convention accepts only a native base transform
exactly `Translate(0,0,-base_height)` and an identity native tool, as in the
built-in models. Their native base-height pair cancels mathematically before
arithmetic; the raw height stays in identity. Component equality rejects even
nonzero subnormal off-pattern entries. Other nominal native frame conventions
use schema 1. All reference, target and tool quaternion/translation words
remain raw identity inputs, with each input-matrix rotation deviation reported.
The adapter honors an already extracted source target quaternion. It never
normalizes a composed point quaternion or substitutes a rounded flange target
as the exact object. Schema mismatch prevents cross-convention cache reuse.

Let the exact canonical FK at centre `qhat` be `M'=(Rhat,phat)`. The shared FK
expressions give joint axes `omega_j` and a point `p_j` on each axis in the
normalized base frame. Their rebased spatial screws are
`xi'_j=(p_j cross omega_j; omega_j)`. For each joint, enclose

```
Jb_linear_j  = Rhat^T (omega_j cross (phat-p_j))
Jb_angular_j = Rhat^T omega_j.
```

This is the body Jacobian of the anchored logarithm at its centre, not an
arbitrary six-entry pose residual. Recentring `q=qhat+w` yields the same FK
map from the rebased screws and fixed end pose `M'`. The exact map supplies
unit axes and a proper rotation; interval boxes contain that object without
requiring every point in the boxes to be a rotation or unit vector.

Joint coordinates and six twist coordinates use infinity norms. Matrix
operator bounds use the induced row-sum norm. Screw constants use the sum of
Euclidean translation/angular norms. No entrywise-max matrix norm is substituted
for an induced norm.

## Validated inverse and aspect

A floating full-pivot inverse of the midpoint Jacobian only proposes a matrix
`X` of represented binary64 entries. A caller may supply another proposal.
Enclose `E=I-X Jb`; require `e=||E||_infinity <1`. Then

```
||Jb^-1||_infinity <= beta = ||X||_infinity/(1-e).
```

The path `I-tE`, `0<=t<=1`, is nonsingular and starts at identity. Consequently
`det(X Jb)>0`, so their determinant signs agree. The sign of the represented
`X` is independently enclosed by pivoted elimination. Every used pivot must
exclude zero; uncertain pivots or unavailable arithmetic produce inconclusive.
The canonical body-Jacobian sign is recorded explicitly. Opt-in identification
uses it as the canonical aspect label only when the complete ball succeeds.
It is never relabelled as a determinant in raw command coordinates.
The checker also encloses the independent expanded nominal determinant at the
centre. A definite contradictory sign, or a supplied contradictory numerical
aspect expectation, makes the attempt inconclusive with an explicit reason.
An uncertain closed-form sign does not invalidate the separate Jacobian proof.

## Pose-log residual and the direct series refinement

Let `g=M'^-1 T*`. Require an enclosed lower bound `trace(rot g)>=1`, which
places its exact rotation at angle at most pi/2. With
`d=||g-I4||_rowsum`, the coarse bound is

```
eta_coarse = 3*pi*beta*d >= ||Jb^-1 Log(g)||_infinity.
```

All products and divisions that form bounds are outward. This sufficient bound
certifies polished candidates but was too conservative for the required
`1e-9` perturbations of the 16-solution witness. The implementation retains it
and also evaluates a stronger direct bound when `d<1/2`.

Write `H=g-I4`. The convergent matrix series is

```
Log(g) = sum(n=1..N) (-1)^(n+1) H^n/n + remainder
||remainder||_rowsum <= d^(N+1)/((N+1)*(1-d)), N=6.
```

Submultiplicativity and `1/n<=1/(N+1)` for the tail give that remainder bound
by a geometric series. The series is the principal homogeneous matrix
logarithm in this domain. Its top translation column and rotation-block
entries `(2,1),(0,2),(1,0)` give the six twist coordinates. Each omitted twist
coordinate is bounded in absolute value by the row-sum remainder. Hence

```
u >= ||X * twist(partial_Log(g))||_infinity
     + ||X||_infinity * tail
eta_series = u/(1-e) >= ||Jb^-1 Log(g)||_infinity.
```

The final Neumann factor is essential even when `X` is only a scaled inverse.
The independent oracle exercises `X=0.1*inverse_proposal` with inverse residual
above 0.8, and checks the exact preconditioned residual and enclosed root.
The checker uses the smaller available valid bound and records the choice and
the coarse value. It therefore applies the general anchored-log theorem with
a directly verified residual bound; it does not assert the more conservative
assembled recipe's `eta>=3*pi*beta*d` when using this refinement.

## Regional derivative and the two radii

Enclose upper constants

```
A  = sum_j (||omega_j||2 + ||p_j cross omega_j||2)
W  = sum_j ||omega_j||2
P0 = ||phat||2
P_R = R*A+P0
Lambda = (pi/2)*(P0+P_R)
B = pi/2+(pi^2/8)*Lambda
C = sqrt(2)*pi^2/8
    +(sqrt(2)*pi^3/16+3*sqrt(2)*pi^2/16)*Lambda+pi^2/8
L(R) = B*(1+P_R)^2*(C+1)*A^2.
```

These are the rebased anchored-log regional derivative constants. The checker
verifies `R*W<pi` for the whole ball's log domain, then

```
R>0, beta*L(R)*R<1
2*beta*L(R)*eta<=1
rho>=2*eta, rho<=R
beta*L(R)*rho^2/2+eta<=rho.
```

`rho` is the outward upper endpoint of `2*eta`; the final radius polynomial is
checked explicitly to account for rounding that radius upward. An automatic
radius proposal uses `min(1e-3, 0.5/(beta*L(1e-3)))`, then recomputes the actual
regional constant and verifies every inequality. The proposal itself is not
a certificate. A supplied positive represented `R` receives the same checks;
negative/nonfinite inputs are invalid. Invalid arithmetic, domain failures or
unsatisfied inequalities do not prove infeasibility.

The contraction/radius theorem gives an exact zero inside `rho` and uniqueness
inside `R`. Its regional derivative also remains invertible throughout `R`;
the body Jacobian has constant sign there because the log derivative factor
is invertible on the certified domain. This explains the canonical aspect
payload, separately from numerical dispatch/regularity policy.

## Checked target-change reuse

A prior sealed certificate can nominate its same canonical centre, represented
preconditioner and uniqueness radius for a changed target. The model identity
schema and every non-target raw word must agree: lengths, raw zeros/senses,
bounds, base/tool frames and their source matrices. Strict identity comparison
can conservatively reject physically equivalent inputs. All target words may
change, including their rotation-deviation source matrix.

Reuse recomputes every enclosure and theorem inequality for the new exact
target. In particular, its domain must pass and its new existence ball must fit
within the old uniqueness radius. This is the radius reserve; no cached inverse,
log estimate or target delta is accepted without revalidation. The successful
payload carries the new full target/model identity. This currently reuses a
centre and preconditioner, not cached derivative arithmetic; any performance
benefit remains to be measured. Model mismatch or a failed reserve requires
ordinary refinement/search, never an infeasibility conclusion.

## Opt-in solver and public contracts

`SolveIkCertified` and `IdentifyIkCertified` wrap the existing numerical solver
and return parallel attachment vectors. Ordinary matrix composition nominates
the numerical flange pose; schema-2 preparation encloses the original raw
reference, TCP target and tool for proof. The represented canonical centre is
the midpoint proposal from the enclosed raw encoder map of the returned
command. The checker certifies that explicit centre and records it. It does
not assert that a rounded degree command maps exactly to that centre.

Strict candidate, seed and local outcomes attempt certificates. Approximate
seed outcomes keep their numerical status and receive `NotStrict`, with no
proof handle. A failed proof keeps the numerical solution and its policy labels.
Only success upgrades opt-in identification's regularity and canonical aspect;
coverage stays `NotAttempted`. Neither successful balls nor failed checks prove
global completeness, infeasibility, command admissibility or nearest ranking.

Reuse checks the old immutable identity, target domain and radius inequalities
at the old centre. An additional outward association check requires the distance
between the new candidate centre and the old centre plus the new existence
radius to fit within the uniqueness radius. Aspect disagreement also rejects
reuse. The call then attempts fresh certification, retaining a precise reuse
failure in a successful fresh payload. Refinement and numerical fallback remain
the existing solver's responsibility. Revalidation currently recomputes FK and
derivative bounds; it is not a promise of a faster solve.

The additive C ABI exports `crx_v1_ik_certified`,
`crx_v1_identify_certified`, `crx_v1_certificate_read` and
`crx_v1_certificate_destroy`. Existing ABI-1 symbols and layouts are unchanged.
The new solve calls take raw row-major target/reference/tool matrices in mm.
Initialize options to `(sizeof(options),1,flags,0)`; only flags 0 and
`CRX_CERTIFICATION_DISABLED` are supported. Initialize each read record's
size/version/reserved to `(sizeof(record),1,0)`. Unsupported negotiation leaves
output arrays and options unchanged; the solve-result structure is initialized.
All proof handles are allocated before publishing output rows, so allocation
failure cannot publish a partial set of owned handles.

For each returned row, failure zero is equivalent to a non-null proof handle.
Handles own immutable attachments independently of solver lifetime. Destroy each
exactly once; null destruction is safe. Concurrent reads are permitted while
the handle remains alive; callers must coordinate destruction with readers and
reuse calls. The full source identity, canonical centre/radii, preconditioner,
inverse/log/derivative constants, sign, trace/determinant enclosures and input
rotation-deviation bounds are copied into the negotiated record. The 77 raw
identity words and schema are documented in the public header.

| Value | Failed check / outcome |
| --- | --- |
| 0 | Certified |
| 1 | Invalid input or unavailable prior certificate |
| 2 | Unsupported floating-point controls |
| 3 | Kinematics enclosure unavailable |
| 4 | Invalid preconditioner proposal |
| 5 | Inverse residual does not prove invertibility |
| 6 | Preconditioner determinant sign uncertain |
| 7 | Target rotation-log domain not established |
| 8 | Regional radius/reserve inequality not established |
| 9 | Required arithmetic enclosure unavailable |
| 10 | Aspect cross-check disagreement |
| 11 | Target/model identity mismatch |
| 12 | Numerical outcome is not strict |
| 13 | Certification not requested |
| 14 | Model/frame unsupported by the exact nominal adapter |

Python exposes `BallCertificate`, `CertificateFailure`,
`ik(..., certify=True, reuse=previous)` and `identify(..., certify=True)`.
`certificates` and `certificate_failures` are parallel to solution rows;
ordinary calls leave both as `None`. Identification's `regular_certified`
distinguishes successful proof rows from numerical regularity labels. Old DLLs
continue to support ordinary calls; requesting unavailable certificate exports
raises `LibraryError`.

Proof arrays are owned read-only snapshots. `close()` releases the native reuse
cache while leaving the snapshot readable; it does not invalidate the numerical
solver. Automatic finalization retains the DLL and its original destructor.
Reuse borrows under a lock, rejects a closed cache or another loaded DLL, and
validates raw native identity rather than trusting client snapshot contents.

```python
import numpy as np
from crx_kinematics import CrxSolver

q = np.array([20., 30., 40., 50., 60., 70.])
with CrxSolver("CRX-10iA") as solver:
    result = solver.ik(solver.fk(q), seed=q, certify=True)
    previous = result.certificates[0] if result.success else None
    try:
        if previous is not None:
            updated = solver.ik(solver.fk(q + 1e-6), seed=result.solutions[0],
                                certify=True, reuse=previous)
            # Each new proof owns a separate cache, even after successful reuse.
            for proof in updated.certificates:
                if proof is not None:
                    proof.close()
    finally:
        if previous is not None:
            previous.close()
```

## Validation at this checkpoint

The Debug native witness certifies all 16 candidates and all 16 centres
perturbed by `1e-9`, while rejecting bad inverse proposals, invalid radii,
unsafe FP controls and target-identity mismatches. An active replaceable C++
allocation counter and Eigen runtime malloc prohibition surround certificate
calls; direct CRT allocations are outside that counter's scope.

Eleven independent 200-digit Python tests pass. They differentiate elementary
transform products to check every body-Jacobian entry, independently refine
the exact quaternion-defined targets and place their roots inside the returned
balls, and recalculate all reported theorem inequalities and signs. Cases
include signed lengths, three physical scales, base/tool frames, scaled
preconditioners, log half-turns and singular limits. On an offset wrist, `q5=0`
alone can be regular; the singular-limit fixture uses `q2=q5=0` and independently
checks determinant collapse. No acceptance condition was weakened.

The reuse test independently refines the changed target's root inside its new
ball, rejects a target beyond the old reserve, and mutates each of all 61
non-target identity words to ensure it is rejected. Native checks reject a
contradictory aspect expectation and preserve centre-evaluation output on
invalid input. Debug and optimized Release exercise these same checks.
Three scale-parametrized raw-reference tests independently compose the exact
source quaternions/translations, refine their roots, verify target enclosures,
and check changed-target reuse. Native tests enforce exact native frame
admission, source quaternion retention, schema separation and no allocation.

The 25 linked-DLL API tests independently refine all 14 discovered regular roots
at a fixed command target and check each returned theorem payload. They cover
strict seed/local attachments, raw reference/tool transforms, full turns,
approximate-boundary exclusions, model lifetime and changed-model reuse,
concurrent native reads/finalization, old-DLL fallback, cross-DLL rejection,
size/version/reserved negotiation and output canaries. Injected malformed native
responses verify exactly-once cleanup after bad status, count, second record or
failure code. Capacity 1/64/512 regressions exercise genuinely truncated full-turn
prefixes, retain ordered commands and all numerical/address flags, and refine
the first/last successful proof independently. Native integration tests additionally prove that only a valid ball
upgrades small-scale numerical regularity and that unsupported exact frames
retain the existing numerical policy. Unsupported FTZ caller controls remain
inconclusive even when a previous certificate is supplied.

Matching Debug binaries pass all six native CTest entries and all 75 arithmetic,
ball and linked-DLL oracle/API tests. The full Release
format/tidy/build/CTest workflow passes all eight entries:
17,098 Python regressions, 833 opt-in skips and 8,640 reliability checks.
The actual optimized certificate compile commands have strict FP flags and no
LTO; object disassembly contains no fused multiply-add instructions. Structural
and inherited lint warnings remain visible. The full Debug regression suite was
not repeated; its previously reproduced diagnostic-capture limitations remain.

The first full run found seven asset checks failing because the newly created
worktree contained Git LFS pointers. Copying the six existing approved payloads
from the verified parent, checking their pointer OIDs/sizes and refreshing Git's
stat metadata restored the checkout. All seven asset checks and then the full
workflow pass; no tracked asset content or fixtures changed.

Commands used from the owner worktree with the supported x64 environment:

```powershell
uv sync --locked --offline
cmake --preset windows-clang-debug
cmake --build --preset windows-clang-debug
ctest --preset windows-clang-debug -R '^crx\.(ball|interval|exact|solver|command-lifts|canonical)$' --output-on-failure
$env:CRXKIN_LIBRARY_PATH = "$PWD/build/Debug/crx_kinematics.dll"
$env:CRXKIN_BALL_PROBE_PATH = "$PWD/build/Debug/crx_ball_tests.exe"
$env:CRXKIN_EXACT_PROBE_PATH = "$PWD/build/Debug/crx_exact_tests.exe"
$env:CRXKIN_INTERVAL_PROBE_PATH = "$PWD/build/Debug/crx_interval_tests.exe"
uv run --locked pytest tests/test_crx_ball_certificate.py tests/test_crx_exact_target.py tests/test_crx_interval.py tests/test_certified_api.py -v
Remove-Item Env:CRXKIN_LIBRARY_PATH
Remove-Item Env:CRXKIN_BALL_PROBE_PATH
Remove-Item Env:CRXKIN_EXACT_PROBE_PATH
Remove-Item Env:CRXKIN_INTERVAL_PROBE_PATH
cmake --workflow --preset windows-clang-release-verify
```

The CMake benchmark targets use the same optimized configuration. qmake's
explicit source list and strict non-LTO enclosure rule are wired for compatibility,
but qmake is unavailable and has not been exercised. No fixture/asset content,
vendored source, deployment or legacy public layout changed.

## Measurement protocol

The local feature corpus is constructed before numerical or proof outcomes are
known. It contains all six captured geometries at three physical scales
(`0.001`, `1`, `1000`) in eight groups: point, tiny target update, local target
update, reference/tool frames, near-singular limit, nominal singular limit,
signed link and a huge raw encoder zero. Six targets just outside maximum reach
exercise approximate numerical fallback. These are 150 private math inputs;
42 also use built-in public models (36 strict tracking/point inputs and six
approximate controls). Signed, scaled and huge-zero variants are explicitly
private-only. Nominal singular group names refer to command settings; the raw
binary64 encoder offsets remain part of the exact identity.

Private preparation, enclosed FK/body Jacobian, fresh certification and old-centre
revalidation are separate stages. The complete DLL cycle compares the parent
legacy C ABI, the new API disabled, enabled and supplied with a previous proof.
Parent calls include ordinary reference/target/tool composition; certificate calls
include native handle allocation, record copying and destruction. All four modes
must preserve every ordered command, status, count, truncation and singular-
representative flag. The parent is Arithmetic's immutable measured DLL, linked
to its reviewed source and published commit; the new operation has no old proof
implementation to compare against.

Each measurement is serial and pinned to one available hardware thread. Native
batches use three warmup calls, untimed equality checks, shuffled input/operation
order and complete output consumption through a noinline volatile indirect call.
The final feature run uses nine rounds of 64 repetitions; its quantiles are
across input medians of batch means, not individual-call tail latency. Ordinary
solver comparisons use both frozen corpora, both seed modes, nine rounds of
eight repetitions and capacity 512. Separate matched instrumented executables
collect refinement counters outside timing. No latency budget was supplied.

The independent 200-digit checker validates every prepared target/FK/Jacobian,
axis and origin enclosure, every successful private/API proof constant, raw
identity word, sign, rotation-deviation bound and refined exact root. It checks
all operation rows, all rounds and all command metadata. The first pilot found
an arbitrary-precision cancellation residue in two analytically zero Jacobian
entries at the maximum-reach controls. Cancelling adjacent Y rotations gives
their exact values `sin(q4)*sin(q3)` and its negative. Evaluating that identity
removed the oracle residue without a tolerance, production-code change or
excluded input. The failed pilot remains in the local evidence archive.

## Measured feature cost and outcomes

Measured on 12th Gen Intel(R) Core(TM) i7-12700H, Windows-11-10.0.26200-SP0, Windows x64;
Visual Studio 2026 x64 tool environment, clang-cl/clang-tidy 22.1.3, CMake
4.3.1-msvc1 and Ninja 1.13.2. Python 3.14.8, NumPy 2.5.3, pandas 3.0.6,
mpmath 1.3.0 and the unchanged vendored Eigen headers are recorded with the
raw runs. The following timings are microseconds under the protocol above.

| Operation | Inputs | Ready / proofs | Reused | p50 | p95 | p99 | Max |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| Private exact preparation | 150 | 150/150 | 0 | 13.755 | 15.938 | 16.105 | 16.127 |
| Private FK/body Jacobian | 150 | 150/150 | 0 | 18.112 | 21.441 | 21.622 | 21.748 |
| Private fresh ball | 150 | 90/150 | 0 | 43.423 | 46.480 | 47.387 | 47.584 |
| Private old-centre revalidation | 150 | 71/150 | 71 | 42.990 | 44.532 | 45.160 | 46.259 |
| Parent numerical C ABI | 42 | not requested | 0 | 0.362 | 9.418 | 9.596 | 9.688 |
| New C ABI, proof disabled | 42 | not requested | 0 | 0.545 | 9.561 | 9.826 | 9.922 |
| New C ABI, fresh proof | 42 | 23/42 | 0 | 59.237 | 62.806 | 63.240 | 63.500 |
| New C ABI, previous proof supplied | 42 | 23/42 | 17 | 59.221 | 105.507 | 106.962 | 107.575 |

Fresh public proof succeeds for 23 of the 36 strict inputs; 13 remain
inconclusive (seven radius checks and six inverse-residual checks). All six
approximate controls retain `ApproximateSeed` and `NotStrict`, with no proof.
Public numerical status is 30 `StrictSeed`, six `StrictLocal` and six
`ApproximateSeed` in every mode. No command or result flag changes.

| Input group | Private inputs | Fresh balls | Revalidated balls | Public inputs | Fresh public proofs | Public reuse |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| point | 18 | 18 | 18 | 6 | 6 | 6 |
| track_seed | 18 | 18 | 17 | 6 | 5 | 5 |
| track_local | 18 | 18 | 0 | 6 | 6 | 0 |
| frames | 18 | 18 | 18 | 6 | 6 | 6 |
| near_singular | 18 | 0 | 0 | 6 | 0 | 0 |
| singular | 18 | 0 | 0 | 6 | 0 | 0 |
| signed | 18 | 18 | 18 | 0 | 0 | 0 |
| large_zero | 18 | 0 | 0 | 0 | 0 | 0 |
| approximate | 6 | 0 | 0 | 6 | 0 | 0 |

There are 90 valid prior private certificates. Revalidation succeeds for 71;
19 fail the reserve/radius check. The other 60 inputs have no valid prior
certificate and report invalid prior input. There are 24 eligible public caches;
17 reuse successfully, six fail the old radius reserve and succeed with
a fresh proof, and one tiny-update input remains inconclusive after a fresh
attempt. These seven updates pay for both attempts. Reuse recomputes
the proof arithmetic, so the measured successful reuse has approximately
the fresh-proof cost, while its failed-reserve tail is higher. There is no
measured reuse speedup claim. Times for failed proofs are shown separately:

| Operation / outcome | Inputs | p50 | p95 | p99 | Max |
| --- | ---: | ---: | ---: | ---: | ---: |
| certify / proof | 90 | 43.583 | 45.289 | 45.556 | 45.997 |
| certify / no proof | 60 | 26.780 | 47.112 | 47.578 | 47.584 |
| reuse / proof | 71 | 43.452 | 44.775 | 45.574 | 46.259 |
| reuse / no proof | 79 | 0.025 | 43.881 | 44.548 | 44.664 |
| api_enabled / proof | 23 | 59.648 | 61.747 | 62.615 | 62.853 |
| api_enabled / no proof | 19 | 41.875 | 62.929 | 63.386 | 63.500 |
| api_reuse / proof | 23 | 59.694 | 106.023 | 107.246 | 107.575 |
| api_reuse / no proof | 19 | 41.958 | 66.102 | 93.555 | 100.419 |

Preparation is performed once per numerical request and shared by its
candidate attachments. Sealed prepared problems may also be reused directly
by the private checker. Public target updates prepare a new exact problem;
the current cache does not amortize derivative computations across calls.
The private preparation timing excludes ball construction, and each private
ball timing includes FK/Jacobian work. These overlapping stages should not
all be added together. Public timings include the full native client cycle.

The core uses fixed six-dimensional matrices and a six-term logarithm series;
it performs no adaptive proof search or unbounded subdivision. The native
allocation probes cover its stack-only work. Public calls allocate parallel
attachment vectors and one immutable cache per success, with memory linear
in the returned prefix. A negotiated C snapshot is 1136 bytes, excluding
allocator/native-cache and Python-array overhead. Snapshot/reuse ownership
cost is included in the complete API timing.

## Conditioning, radii and widths

The following distribution contains only the 90 successful fresh private
balls, out of all 150 attempted inputs. Inconclusive inputs are retained in
the outcome tables above. All values use normalized lengths and canonical
radians.

| Quantity | p50 | p95 | p99 | Max |
| --- | ---: | ---: | ---: | ---: |
| Existence radius rho | 3.93616e-13 | 5.07079e-12 | 1.0404e-11 | 1.7584e-11 |
| Uniqueness radius R | 3.43828e-07 | 6.27774e-06 | 1.2721e-05 | 1.2871e-05 |
| Inverse norm bound beta | 9.8076 | 91.2022 | 317.288 | 604.134 |
| Regional derivative bound L | 133551 | 428439 | 486726 | 495880 |
| Inverse residual e | 1.89484e-12 | 2.14202e-11 | 4.92145e-11 | 7.24828e-11 |

Widths below are maximum entry widths per input, aggregated over all 150
inputs, including huge-zero and singular-limit cases. Preparation and
centre evaluation remain finite on every input; broad bounds are allowed
and never converted into proof success.

| Enclosure width | p50 | p95 | Max |
| --- | ---: | ---: | ---: |
| rotation_width | 3.14193e-14 | 4.02359 | 5.32039 |
| translation_width | 3.04756e-14 | 6.75482 | 8.40155 |
| jacobian_width | 1.05249e-13 | 145.415 | 276.348 |

## Ordinary solver comparison against Arithmetic

The 4,313 original and 3,024 stress targets each run in both seed modes.
All 14,674 case/mode pairs retain exactly equal ordered binary64 command
values, including the best command, without reducing angles modulo turns.
There are zero lost strict solves, lost witness case/modes, count changes
or invalid outputs. The six separate refinement counters match exactly.
The original corpus retains 3,929 strict solves and 3,726 witness hits in
each mode; stress retains 3,024 strict solves and 2,994 witness hits.

| Corpus / mode / population | Inputs | Parent p50 | New p50 | Parent p95 | New p95 | Parent p99 | New p99 | Parent max | New max |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| existing / no_seed / all inputs | 4313 | 18.312 | 18.288 | 92.075 | 91.665 | 141.368 | 142.699 | 248.588 | 246.225 |
| existing / no_seed / all reachable | 3929 | 18.775 | 18.762 | 94.767 | 94.687 | 145.585 | 146.711 | 248.588 | 246.225 |
| existing / no_seed / mixed singular | 384 | 89.256 | 89.462 | 158.566 | 157.612 | 202.899 | 199.650 | 248.588 | 246.225 |
| existing / nearby_seed / all inputs | 4313 | 18.312 | 18.337 | 91.487 | 91.410 | 142.567 | 143.683 | 249.262 | 248.838 |
| existing / nearby_seed / all reachable | 3929 | 18.800 | 18.788 | 94.750 | 95.057 | 146.561 | 148.325 | 249.262 | 248.838 |
| existing / nearby_seed / mixed singular | 384 | 88.900 | 88.575 | 156.646 | 157.147 | 194.943 | 195.806 | 249.262 | 248.838 |
| stress / no_seed / all inputs | 3024 | 20.512 | 20.381 | 113.930 | 115.399 | 148.653 | 148.610 | 294.375 | 294.913 |
| stress / no_seed / all reachable | 3024 | 20.512 | 20.381 | 113.930 | 115.399 | 148.653 | 148.610 | 294.375 | 294.913 |
| stress / no_seed / mixed singular | 432 | 93.188 | 93.662 | 146.009 | 148.337 | 176.721 | 175.745 | 198.863 | 194.713 |
| stress / nearby_seed / all inputs | 3024 | 20.475 | 20.400 | 114.402 | 115.080 | 149.665 | 147.427 | 287.113 | 288.000 |
| stress / nearby_seed / all reachable | 3024 | 20.475 | 20.400 | 114.402 | 115.080 | 149.665 | 147.427 | 287.113 | 288.000 |
| stress / nearby_seed / mixed singular | 432 | 93.763 | 93.925 | 149.373 | 144.664 | 175.214 | 179.432 | 198.787 | 200.438 |

Ordinary timing is approximately unchanged. Individual tails vary in both
directions; no speedup, equivalence margin or latency acceptance is inferred
from these measurements. Certification is optional additional work whose
measured cost and inconclusive rate are stated above.

## Artifact identities and reproduction

Arithmetic parent: commit `d4a6cc7e2f8afe19b1359a5eeed219147bcc2f57`, PR #23. Its timing DLL
has SHA-256 `896af586ba64bac9932487ccdbc6a37d0ea9552d3e40bac67b4b65567e9fa2a1` and measured source identity
`d9ac41f90154558eb6191b79d6a891fc0a7aa0cc287c9c405250cae224f5ac86`. Arithmetic's fresh committed DLL
has a separate build-path-dependent identity recorded in its validation/PR;
the committed source and measured production source are content-equivalent.

This uncommitted Local certificates checkpoint has production identity
`6014bc5f4275b7d67300db606166a2ed28458b1fd87dd1ff697b15a7da09dbe0`. This is SHA-256 of the compact sorted JSON mapping
from production paths to raw SHA-256 values; it is not a Git commit. Tests,
documentation and benchmark sources are hashed separately in the archive.

| Artifact | SHA-256 |
| --- | --- |
| candidate.dll | `653126970987bfdf1e9adc161c978bfa960d561478b065bff990cc157bac3ca1` |
| feature-benchmark.exe | `b433f203d243fb3adc57302d50f9f8d0689d303585ad04f006e22ec324e4945b` |
| solver-benchmark.exe | `98b73e43b8c2ba05d6ed12e271044bd8a2a6d1a2f3a4ade5b98dda91fde409b5` |
| candidate-counters.exe | `860a5b57bfd1a0f53a6dcdbfc1d111a9492c0cc6dae790215adec4c4031aa318` |
| sources.zip | `b0469a2b414206cee6d47f13d059adecddad45dd3ca61e5c7b2797d135a15081` |
| compile_commands.json | `b3dd48f7436cad6899ded02a3b0c588ed495a643be8e89d3dbab8abcba613713` |
| feature corpus.json | `2c166373c3504f88b9ebf9ecd99a0e05597351fc14bc432c0e0354b92d99d62e` |
| existing corpus.json | `e582fae793d0c91717a557409c240787ac9a36e6d80c382d5f56b3483e9269e0` |
| stress corpus.json | `d0dc2f4003ff4dfe46fd8f224d578f8b5ab445d3e03b5117a28169332cf83041` |

The measured source ZIP is immutable. The final handoff also carries this
completed report and three additional capacity-prefix regressions; production
code and benchmark source remain equal to the measured checkpoint. Raw timings, enclosure/proof/input/command
CSVs, counter records, source/binary/compile-command manifests and independent
analysis are retained in the shared local evidence archive. This report is
the standalone repository summary; its reproduction commands need no private
orchestration document. From an x64 developer shell after the gates above:

```powershell
cmake --build --preset windows-clang-release-tidy --target crx_local_certificate_benchmark crx_solver_benchmark crx_solver_counters
uv run --locked python scripts/benchmark_local_certificates.py --parent-dll PARENT_DLL --rounds 9 --repeats 64 --output build/evidence/local-feature
uv run --locked python scripts/benchmark_accuracy.py --before PARENT_DLL --after build/Release/crx_kinematics.dll --before-counters PARENT_COUNTERS --after-counters build/Release/crx_solver_counters.exe --corpus existing --rounds 9 --repeats 8 --capacity 512 --output build/evidence/local-existing
uv run --locked python scripts/benchmark_accuracy.py --before PARENT_DLL --after build/Release/crx_kinematics.dll --before-counters PARENT_COUNTERS --after-counters build/Release/crx_solver_counters.exe --corpus stress --rounds 9 --repeats 8 --capacity 512 --output build/evidence/local-stress
```

`PARENT_DLL` and `PARENT_COUNTERS` must be a matched immutable Arithmetic
build. The default feature driver snapshots source and binaries and validates
the math independently; the accuracy driver checks the frozen corpora against
their generators and validates all returned configurations. Keep the runs
serial and pause builds, tests and other numeric workloads during timing.

This feature establishes local exact-pose existence, uniqueness and canonical
regularity under the stated hypotheses. Complete target catalogues, original
degree-limit admission, exact nearest-command ranking, calibrated models and
path coverage remain separate work. Its C++ code is validated by these tests
and audits; it is not itself formally verified.
