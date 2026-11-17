# Wrist residual and exact vertical discovery

This change caches per-target scalar invariants, shares residual/derivative
evaluation, simplifies Cramer elbow polishing, and nominates wrist phases
directly for exactly vertical tool targets. Candidates still pass existing
unsquared incidence recovery and full forward-kinematics acceptance. Public C
ABI layouts and command-lift handling are unchanged. Coverage remains
`NotAttempted`; numerical discovery does not establish completeness or exclude
unreachable targets.

## Coordinates and residual

All quantities below use the existing canonical coordinates divided by the
largest absolute link length. Let `p` be the wrist-circle centre, `v,w` the
target's first two rotation columns, and

```
u(theta) = v cos(theta) + w sin(theta)
x = p + r u
m = p.u                         U = u_z
P0 = |p|^2 + r^2                h = p_z
s = P0 + 2 r m                  z = h + r U
g = m + r                       d = (a-b)(a+b)
k = (s+d)/2                     j = (s-d)/2
E = k^2 - a^2 s
T = (|p|^2-r^2) U - 2 h g
V = j^2 - b^2 z^2
F(m,U) = E U T + g^2 V
```

For an orthonormal phase basis this is the existing denominator-cleared
Cramer sphere residual `|N|^2-a^2 Delta^2`. It is a bivariate quartic.
Differentiate its displayed factors using `m'=-p.v sin(theta)+p.w cos(theta)`
and `U'=-v_z sin(theta)+w_z cos(theta)` to compute the residual and derivative
together. `ResidualInvariants` caches the target-only dot products, heights,
squared lengths, and signed arm difference. The private circle grows from
112 to 192 bytes on the supported Windows x64 build and introduces no allocation.

Scalar subtraction can erase a nonzero residual near folded arms or the base
axis. `EvaluateResidualPair` retains the geometric cross-product calculation
when the basis differs from unit roundoff, arithmetic is nonfinite, or
contributing-term cancellation exceeds the guarded fraction. The `0.125`
dispatch fraction is a numerical policy, **not a proved error bound**. Inner
distance/projection guards run before the derivative work. The original plain
residual and vector endpoint coefficient convolution remain available; no
nine-sample interpolation or new complex-circle eigensolver is introduced.

## Cramer chart

The elbow numerator can be evaluated without constructing cross products.
Define `M=x_x u_x+x_y u_y`, `H=x_x^2+x_y^2`, and `g=x.u`. Then

```
Delta = x_z M - U H
C = x_z g - k U
N = (C x_x, C x_y, k M - g H)
elbow = N / Delta
elbow' = (N' - elbow Delta') / Delta
value = |elbow|^2-a^2
value' = 2 elbow.elbow'
```

These component identities do not require a unit phase axis. Horizontal
components avoid the cancellation of calculating `H` as `|x|^2-x_z^2`.
The original conditioning requirement `|Delta| >= 1e-3 |x|^2` and the
point-size floor remain. A failed evaluation leaves the caller's output
unchanged and reports guard rejection or numerical range failure. Evaluating
the quotient through the elbow preserves the useful near-root residual; it
does not substitute the cancelled factored numerator for that quotient.

## Vertical phase nomination

When the target normal is exactly vertical and both phase basis vectors have
exactly zero vertical components, `U=0` and

```
F = (m+r)^2 [(s-d)^2/4 - b^2 h^2].
```

For nonzero `r`, the three linear phase equations are

```
m = -r
m = (d-P0-2 b h)/(2 r)
m = (d-P0+2 b h)/(2 r).
```

Each equation intersects the phase unit circle in at most two points. The
implementation passes their sine/cosine coordinates directly to existing
incidence recovery, retaining planar elbows, base flips, and coincident
branches. It avoids an angle/trigonometric round trip. Existing joint
deduplication and the final accuracy improvement still apply.

Small nonzero normal components take the general chart route. Zero radius,
zero phase amplitude, unsuitable basis roundoff, nonfinite right sides,
rejected or unresolved nominations, and singular representatives also retain
general solving. Partial results from a declined fast path are replaced by
the general search. An empty floating phase list is not a rigorous exclusion.

Finite vertical counts need not belong to `{0,8,12,16}`. For
`(a,b,c,r)=(1,1,8/27,5/18)` and wrist centre `(1/10,0,1/40)`, the independent
half-angle numerator is proportional to

```
-(8t^2+17)^2 (2303t^2-12097) (10657t^2+25057).
```

Its two simple real phases each recover two base branches: four configurations
modulo full turns. With explicit closed `[-180,180]` degree command limits,
the downward target admits both J5 endpoints and therefore eight command
outputs. Configuration count and admissible command count are separate checks.

## Independent validation

`tests/crx_vertical_reference.py` intersects horizontal circles using exact
`Fraction` discriminants of the represented binary64 input. It reconstructs
elbows and frames independently and checks forward kinematics. It neither
imports production code nor infers a family from approximate multiplicity.
The vertical suite covers six captured asset geometries, signed links and
offsets, scales from `1e-100` to `1e100`, both normals, random yaw, zero height,
tangencies, arm shells, coincident branches, and retained general-route cases.
The expanded focused suite passes 284 tests.

Native residual checks cover 1,152 signed/scaled cases plus explicit
cancellation examples; Cramer checks cover 2,304 cases and guard boundaries.
Independent geometric/complex-step oracles use a relative `1e-13` criterion
away from zero and a contributing-term binary64 absolute bound at cancellation.
No existing FK tolerance was loosened. Native probes run with Eigen allocation
disabled and check the process allocation counter.

The final Release workflow passes all five CTest entries: 17,313 Python regression
tests, 833 opt-in skips, and 8,640 solver-reliability checks. The six
benchmark-checker tests also pass separately; they reject missing witnesses,
duplicated postures, nonfinite joints, inconsistent counts, and wrong FK.
The skips cover external snapshot comparisons and live RoboDK integration.
No deployment was requested or performed. Existing clang-tidy diagnostics,
including straight-line function variable-count warnings, remain visible.

Reproduction on the configured Windows x64 toolchain:

```powershell
cmake --workflow --preset windows-clang-release-verify
cmake --build --preset windows-clang-release-tidy --target crx_discovery_benchmark crx_residual_benchmark crx_solver_benchmark crx_solver_counters
uv run pytest tests/test_benchmark_vertical.py
```

## Performance evidence

Measurements use native paired batches on one pinned thread, alternating order,
three warmups, nine rounds, eight repeats, and capacity 512. Each pose's statistic
is the median of its round batch means; quantiles span poses and are not
individual-call latency tails. Validation occurs outside timing. Independent
captured modified-DH transforms validate complete DLL outputs, command limits,
and witness retention. Untimed counter executables remain separate from DLLs.

The residual/Cramer stage uses source identity
`353aa5bc4a7a191c80ca07c6b52ffc64625ded25f36435bc4a8c93e244a90247` and DLL SHA-256
`f9a2b302a51d8196b46d6f0b905f5b6564978f83cd7f914e4ce72de3c8864256`.
Its 1,186-case component experiment (256 repeats) measured combined residual
median `35.16 -> 26.56 ns`, but p95 `35.55 -> 42.19 ns`. Cramer median/p95
improved from `37.11/37.50` to `28.91/29.30 ns`. Preparation rose from
`15.63` to `20.31 ns`; geometric fallbacks account for 592 component cases.
Whole-solver unseeded p95 was approximately unchanged on both frozen corpora:
`98.93 -> 99.39 us` and `116.95 -> 117.06 us`. All strict solves, witness hits,
and counts were retained. The 144 changed case/mode arrays differ by less than
`3.1e-11` command degrees, without modulo-turn equivalence.

The vertical stage uses source identity
`aa4956db938fc662f3329cdf8ab4b3bd68fee0b3a4d4e9cec06e088cd05ef6f1` and DLL SHA-256
`1cceb315c5b4528f794c8a43f7d5490012a195f57a48ffc4eb261b9e6c9524a5`.
Its independent specialized corpus has 192 regular vertical targets, 24
vertical boundary targets, 48 nearby tilted targets, 96 generic targets and
96 exact coplanar targets. All 1,936 supplied witnesses pass the captured-DH
reference before either DLL runs. Explicit benchmark-only full-turn command
limits do not modify catalogues or fixtures.

In the same candidate executable, enabling vertical nomination reduced regular
vertical discovery p50/p95 from `16.11/21.41` to `3.98/5.33 us`.
It was used for all 192 regular vertical targets and 12 boundary targets;
all nearby, generic, coplanar and family controls retained the general route.
All specialized configuration counts agree with the independent oracle and
the general route; no general posture is lost. Complete-DLL unseeded vertical
p50/p95 against the residual/Cramer stage is `23.04/29.81 -> 9.24/14.43 us`.
These vertical quantiles span all 192 finite-oracle targets: 176 have solutions
and 16 are unreachable in the independent oracle. All 1,648 vertical witness
configurations are retained. Nearby controls perturb a known configuration's
fifth joint by `+/-1e-7` radian and recompute independent FK; all 48 solve and
retain their supplied witnesses. Among 24 boundary/family controls, 14 produce
strict solutions and ten remain empty on both prerequisite and candidate DLLs.
This delivery makes no full-coverage claim for those general-route families.

Both frozen corpora retain every strict solve, witness and output count against
both prerequisite checkpoints. Against residual/Cramer, all 14,674 frozen
case/mode command arrays are numerically identical. Against Identification,
144 differ only below `3.1e-11` command degrees. The specialized comparisons
add 112 small roundoff changes against residual/Cramer, and 178 changes against
Identification. Of the latter, 120 are below `1e-9` degree; the remaining 58
case/modes cover 31 ill-conditioned coplanar targets and exactly reproduce the
earlier residual/Cramer DLL arrays. Their largest direct command difference is
`3.693e-6` degrees. Independent screw-axis Jacobians, with translation scaled by
the largest link, give smallest singular values at most `1.133e-8` for every
changed posture. This is numerical conditioning evidence, not a singularity
certificate. Independent FK errors for those cases are at most `5.20e-11 mm`
and `1.13e-11` degrees. Counts, strict solves and witnesses are retained.
Every changed case/mode is listed in
[wrist-discovery-case-changes.csv](wrist-discovery-case-changes.csv), including
the direct command distance and the conditioning/FK evidence where applicable.

These measurements do not imply a general whole-solver speedup. No latency
budget or performance acceptance was supplied.

## Optional coplanar quartic decision

Defer the optional quartic runtime path. Exact `Fraction` evaluation of
`(p.v) w_z-(p.w) v_z` on the represented prepared geometry finds one rank-loss
target among 4,313 original inputs, already vertical, and none among the 3,024
stress inputs. There are **zero eligible nonvertical targets in either recorded
workload**. Even making every eligible complete call free would save zero
measured total cost there. Small determinants are not counted as exact zero.

The constructed specialized corpus contains all 96 intended nonvertical
coplanar controls. Their complete-DLL cost is about 30.4% of that deliberately
concentrated corpus's unseeded cost, so a workload dominated by explicitly
constructed coplanar targets could justify revisiting the decision. The
current delivery retains general discovery for them. No quartic runtime code
or unproved exact-zero predicate is shipped.

The frequency tool also records the normal-based triple-product predicate
separately: rounded near-orthogonal columns need not satisfy `v cross w=n`
exactly. An exact runtime dispatch needs its own verified hypotheses; the
corrected workload has no disagreements between these two tests. The measurement uses
exact rational products of the represented values and does not assume that
Windows `long double` is wider than `double`.

## Verification and reproduction record

The complete paired experiments run without overlapping builds, tests or other
timing runs on a 12th Gen Intel Core i7-12700H, Windows x64, clang-cl 22.1.3.
The commands below use immutable DLLs identified by the hashes above and the
Identification report. The directories retain input corpora, model tables,
native timing/solution CSVs, independent validation, metadata, source ZIPs and
compile commands. Full-corpus counter probes are separately matched artifacts.

```powershell
# Run twice, with the residual/Cramer and Identification DLLs respectively.
uv run python scripts/benchmark_vertical.py --before BASELINE.dll --after VERTICAL.dll --output EXPERIMENTS/specialized-reachable-cramer --rounds 9 --repeats 8 --capacity 512
# Use specialized-reachable-identification and --skip-component for the second run.

# Run each corpus against both immutable baseline DLLs.
uv run python scripts/benchmark_accuracy.py --before BASELINE.dll --after VERTICAL.dll --corpus existing --rounds 9 --repeats 8 --capacity 512 --before-counters BASELINE-COUNTERS.exe --after-counters VERTICAL-COUNTERS.exe --output EXPERIMENTS/existing-cramer
# Repeat with --corpus stress; use stress-cramer, existing-identification,
# and stress-identification for their respective outputs.
uv run python scripts/benchmark_coplanar_frequency.py --experiments EXPERIMENTS

cmake --preset windows-clang-debug
cmake --build --preset windows-clang-debug
ctest --preset windows-clang-debug -R '^crx.canonical$'
# Set CRXKIN_LIBRARY_PATH and CRXKIN_ROOT_PROBE_PATH to this Debug build.
uv run pytest tests/test_crx_vertical.py tests/test_crx_polynomial_discovery.py tests/test_benchmark_vertical.py -q
git diff --check
git status --short
```

The matching Debug canonical CTest and 415 focused Python tests pass. A full
Debug suite is not claimed green: five diagnostic stderr-capture failures were
previously reproduced on the unchanged parent. Release is the full gate.

The first vertical Release run failed because a new test-probe JSON field
changed an existing exact-response check. A separate `--discover-vertical`
mode now exposes dispatch metadata while `--discover` preserves its response.
The existing status distinction tests pass without weaker assertions. The
first specialized pilot also rejected a `1.44e-10` radian pair under a new
checker-only `1e-9` threshold; the checker now uses discovery's existing
`1e-10` deduplication contract. The production threshold was unchanged, both
pilot logs are retained, and the complete independent set checks pass.

An earlier specialized corpus labelled nearby tilted targets reachable without
supplied FK witnesses. It was replaced by the joint-perturbation construction
above and both affected paired experiments were rerun. All 48 nearby controls
now have independently checked witnesses and solve on both baseline and
candidate DLLs. The earlier exploratory outputs are retained separately; the
specialized values in this report use the corrected corpus.

## Frozen-corpus timing detail

All values are microseconds. Each cell is baseline → candidate.

| Baseline | Corpus | Seed | Group | p50 | p95 | p99 | max |
|---|---|---|---|---:|---:|---:|---:|
| cramer | existing | no_seed | all_reachable | 19.625 → 19.837 | 98.645 → 99.265 | 154.010 → 154.511 | 261.000 → 262.762 |
| cramer | existing | no_seed | mixed_singular | 92.537 → 93.213 | 163.929 → 166.054 | 208.539 → 214.369 | 261.000 → 262.762 |
| cramer | existing | nearby_seed | all_reachable | 19.600 → 19.875 | 98.930 → 99.620 | 155.902 → 154.864 | 260.988 → 261.650 |
| cramer | existing | nearby_seed | mixed_singular | 92.862 → 92.769 | 165.516 → 165.021 | 211.257 → 206.072 | 260.988 → 261.650 |
| cramer | stress | no_seed | all_reachable | 21.250 → 20.962 | 117.984 → 118.291 | 155.156 → 152.210 | 295.788 → 311.962 |
| cramer | stress | no_seed | mixed_singular | 96.362 → 96.269 | 153.015 → 151.050 | 179.543 → 183.525 | 199.387 → 199.175 |
| cramer | stress | nearby_seed | all_reachable | 21.225 → 21.038 | 118.263 → 118.022 | 154.944 → 155.613 | 314.625 → 309.200 |
| cramer | stress | nearby_seed | mixed_singular | 96.950 → 97.075 | 152.710 → 152.390 | 183.018 → 182.860 | 208.150 → 205.475 |
| identification | existing | no_seed | all_reachable | 19.688 → 19.587 | 99.540 → 99.507 | 158.134 → 156.345 | 260.387 → 261.700 |
| identification | existing | no_seed | mixed_singular | 92.981 → 92.781 | 167.194 → 167.389 | 208.361 → 204.865 | 260.387 → 261.700 |
| identification | existing | nearby_seed | all_reachable | 19.663 → 19.587 | 99.800 → 99.382 | 155.552 → 156.142 | 264.600 → 265.438 |
| identification | existing | nearby_seed | mixed_singular | 92.988 → 93.287 | 165.714 → 167.373 | 212.947 → 212.164 | 264.600 → 265.438 |
| identification | stress | no_seed | all_reachable | 20.994 → 21.262 | 117.875 → 118.485 | 155.558 → 153.633 | 295.000 → 303.238 |
| identification | stress | no_seed | mixed_singular | 96.312 → 96.287 | 155.326 → 153.293 | 182.435 → 182.376 | 205.575 → 203.588 |
| identification | stress | nearby_seed | all_reachable | 20.988 → 21.275 | 118.760 → 118.955 | 153.787 → 153.540 | 302.725 → 301.300 |
| identification | stress | nearby_seed | mixed_singular | 96.569 → 97.537 | 151.253 → 153.196 | 184.131 → 186.543 | 202.113 → 204.312 |

The original all-reachable subset retains 3,929 strict solves and 3,726 witness
hits in each mode; stress retains 3,024 strict solves and 2,994 witness hits.
The slower tails shown above remain part of the performance review.
