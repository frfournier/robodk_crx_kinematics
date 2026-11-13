# Interval arithmetic and exact nominal target foundations

The private certification foundation now evaluates the existing geometric
residual, direct polynomial convolution and canonical FK expressions with
either `double` or outward intervals. It defines one exact rigid target and
nominal model from stored inputs, and encloses their wrist polynomials.
`Ready` means finite enclosures were constructed. It certifies no IK root,
regularity, infeasibility, command admissibility, or completeness. Existing C ABI
results and numerical acceptance remain unchanged.

## Floating-point contract

`Interval` contains finite ordered endpoints. Invalid operands, a divisor
containing zero, a negative square-root domain, unsupported FP controls, and
unavailable finite bounds fail closed. Invalid is an arithmetic failure, never
an empty-set certificate. An otherwise finite mathematical result may still
be unavailable near binary64 overflow because outward widening needs an
infinite endpoint.

For each endpoint of `+`, `-`, `*`, `/`, and square root, nearest binary64
arithmetic followed by one outward `nextafter` contains the exact result.
Multiplication/division evaluate every endpoint extremum. Exact zero identities
are preserved. Square handles zero-crossing boxes with a nonnegative lower
bound. Euclidean norms use enclosed squares and a justified nonnegative domain;
entrywise maximum and induced row-sum norms are distinct functions.

The interval and exact-preparation translation units are compiled with strict
FP semantics, disabled contraction and no LTO, independently of ordinary
numerical optimization. On the validated clang-cl toolchain their command lines
include `/fp:strict`, `/clang:-ffp-model=strict`,
`/clang:-ffp-contract=off`, and `-fno-lto`. The runtime checks nearest rounding,
gradual underflow (FTZ/DAZ disabled), and masked SSE exceptions. It changes no
caller control mode; normal arithmetic can set sticky exception flags. The
source rejects excess intermediate precision and non-IEEE binary64 builds.
The compiler behavior is described by the [Clang FP controls](https://clang.llvm.org/docs/UsersManual.html#controlling-floating-point-behavior)
and [Microsoft `/fp` documentation](https://learn.microsoft.com/en-us/cpp/build/reference/fp-specify-floating-point-behavior).

Square roots use the SSE2 `sqrtsd` primitive explicitly. Sin/cos use no libm
accuracy assumption. The exact pi bracket is

```
[0x1.921fb54442d18p+1, 0x1.921fb54442d19p+1].
```

Integer-quadrant reduction encloses `x-k*pi/2`. The floating estimate only
chooses an exactly represented integer; it need not choose the nearest exact
quadrant. The reduced box is checked before evaluation. Eleven Taylor recurrence
terms give sine through degree 23 and cosine through degree 22. Including their
zero next coefficients, the Lagrange remainders are bounded by
`|x|^25/25!` and `|x|^24/24!`, evaluated outward too. Reduced magnitudes above
two, broad arguments, or magnitudes above `2^20` use the conservative `[-1,1]`
range. This is an explicit precision limitation, not a narrow reduction.

## One exact target and model

For nonzero represented quaternion components `q=(w,x,y,z)`, define

```
R*(q) = A(q)/(w^2+x^2+y^2+z^2),
```

where `A` is the homogeneous quadratic quaternion rotation action. This is a
single mathematical rotation. The code encloses component scaling by the
largest absolute stored component, products, squared norm, and division. It
does not first normalize to a rounded point quaternion and call that point
exact. Scaling avoids overflowing the raw squared norm and retains tiny
components conservatively.

Let `B,T,U` denote the exact base, world target, and tool poses. Rotations compose
through the quaternion expression `conj(q_B)*q_T*conj(q_U)`, evaluated as
enclosures without rounding its result to another point quaternion. With
`L=max(|a|,|b|,|c|,|r|)`, the normalized canonical target is

```
R_06 = R_B^T R_T R_U^T
p_06/L = R_B^T (p_T-p_B)/L - R_06 p_U/L - (base_height/L) zhat.
```

The difference/scale evaluation avoids overflowing cancellable absolute
translations, and also handles a large difference when division first makes
it representable. Every physical length ratio is enclosed. The wrist-circle
centre is `p=p_06/L-(c/L) R_06 zhat`.

The preferred model adapter takes the numerical problem's already stored
`target_quaternion` components. Base/tool quaternion extraction makes explicit
binary64 input choices. The certificate target is their homogeneous rigid
action, while the original matrices remain available through enclosed
Frobenius deviations `||R_in-R*||_F`. Final numerical acceptance continues to
use the original input pose. An interval matrix box need not consist entirely
of rotations; it contains the one rigid object defined by these inputs.

The model is the nominal orthogonal CRX family. Its fixed twists are ideal
right angles. The DH adapter accepts only the explicit nominal twist encoding,
zero off-pattern lengths, and exact unit senses. Calibrated twists, small
off-pattern dimensions, and approximate gains are not silently snapped into
an exact certificate. An encoded nominal `-kHalfPi` twist selects the ideal
quarter turn. An encoder zero at the same binary64 value remains the exact
stored real angle and is treated differently on purpose.

For coupled command radians `u`, the exact coordinate map is

```
q_i = (sense_ref_i*sense_i) u_i
      + sense_ref_i (theta0_i-theta_ref_i),
sense_ref = (1,1,-1,-1,-1,-1)
theta_ref = (0,-pi/2,0,0,0,0).
```

Raw encoder zeros are enclosed without tolerance snapping or ordinary libm
remainder. The nominal stored `-binary64(pi/2)` therefore retains its small
difference from mathematical `-pi/2`. Principal seams return a full connected
`[-pi,pi]` enclosure and an uncertainty flag. FK uses the unwrapped offset;
it does not choose a seam side or substitute the broad principal diagnostic
for a narrow periodic expression. Very large raw zeros can produce broad FK
enclosures, as required by the declared trig limitation.

The immutable prepared object contains a schema and all 77 raw binary64 words:
lengths, base height, raw zeros, senses, represented radian bounds, and the
three quaternion/translation/input-matrix records. Equality compares the words,
not a finite hash. Equivalent quaternion scales/signs can have different strict
input identities. The stored radian bounds do not certify original degree-limit
conversion; a later admissibility context must retain that provenance.

## Direct scalar polynomial enclosures

All following quantities use normalized lengths. For the true exact target,
`v,w` are orthonormal phase axes. Put

```
u(theta)=v cos(theta)+w sin(theta)
x=p+r u, n=zhat cross x, k=(a^2+|x|^2-b^2)/2
Delta=x.(u cross n)
N=k (u cross n)+(x.u)(n cross x)
f=|N|^2-a^2 Delta^2.
```

The physical residual scales by `L^6`; the determinant scales by `L^2`.
With chart origin `alpha`, `t=tan((theta-alpha)/2)` and `q0=1+t^2`, direct
convolution constructs all coefficients of

```
P_alpha(t)=q0^4 f(alpha+2 atan(t))      # nine slots, degree <=8
P'_alpha(t)                           # eight slots, degree <=7
Q_alpha(t)=q0^2 Delta(alpha+2 atan(t))  # five slots, degree <=4.
```

The shared numerical/enclosed implementation keeps the original vector endpoint
distances when forming squared-distance coefficients. It uses neither sampled
interpolation nor truncation. For the determinant, writing `m=p.u`, `h=p_z`,
`U=u_z` gives the exact identity

```
Delta=h(m+r)-(|p|^2+r*m)U.
```

Each factor has degree at most one in phase, hence the five-slot half-angle
convolution. Derivative coefficients are enclosed integer multiples of `P`.
The omitted phase uses `u=-v_alpha` directly, evaluating both residual and
determinant without an infinite tangent or a rounded `sin(pi)`.

No small leading coefficient is trimmed. A degree drop, a zero-straddling
leading enclosure, and an identically zero represented polynomial remain
distinct from proof of root coverage or a continuous fibre. For example,
`a=b=1`, `r=1/4`, `p=(1/4,0,0)`, `R=I` gives
`f=r^6(1+cos(theta))^4` and constant `P=16*r^6=1/256`.
Its omitted phase is still a root of `f`, and `Delta` vanishes identically.
All nine slots and the omitted-point record are retained.

## Validation and measurement record

The current integrated Release workflow passes all seven CTest entries:
17,062 Python regressions, 833 opt-in skips and 8,640 reliability checks.
Matching Debug interval/exact/canonical native tests and all 39 focused Python
oracle tests pass. The native exact harness disables Eigen allocation and checks
the active replaceable C++ allocation counter around preparation, polynomial
construction and FK. Direct CRT allocations are outside that counter's scope.

The independent oracle uses exact `Fraction` quaternion matrices and frame
compositions, an independent vector-polynomial Cramer numerator, and 200-digit
elementary transform products. Tests cover all raw identity words, six captured
asset ratios with signed/scaled geometry, extreme quaternion components and
scale/sign, overflow-avoiding normalization, raw offsets/seams, interior trig
extrema, chart intervals, omitted phases, degree drops, and FK boxes. mpmath
1.3.0 is a pinned development-only dependency; no runtime interval library was
added. Existing fixture and robot assets are unchanged.

Reproduction uses the supported Windows x64 clang-cl presets:

```powershell
cmake --workflow --preset windows-clang-release-verify
cmake --preset windows-clang-debug
cmake --build --preset windows-clang-debug --target crx_interval_tests crx_exact_tests crx_canonical_tests
ctest --preset windows-clang-debug -R '^crx.(exact|interval|canonical)$'
$env:CRXKIN_INTERVAL_PROBE_PATH = "$PWD/build/Debug/crx_interval_tests.exe"
$env:CRXKIN_EXACT_PROBE_PATH = "$PWD/build/Debug/crx_exact_tests.exe"
uv run --locked pytest tests/test_crx_interval.py tests/test_crx_exact_target.py -q
cmake --build --preset windows-clang-release-tidy --target crx_certification_benchmark crx_solver_benchmark crx_solver_counters
```

The full Debug Python suite was not repeated for this change. Identification's
unchanged Debug baseline has five diagnostic-output capture failures; the focused
Debug arithmetic checks above pass. No test was disabled or tolerance weakened.
The Release tidy build retains visible inherited and structural diagnostics;
passing the workflow does not mean a warning-free build. qmake was not exercised.

## Frozen measurement identity and method

Measured on 2026-11-13, Windows 11 x64, Intel Core i7-12700H, clang-cl/clang-tidy
22.1.3 (Visual Studio 2026 Community 18), CMake 4.3.1-msvc1, Ninja 1.13.2,
uv 0.13.4, CPython 3.14.8, NumPy 2.5.3, Pandas 3.0.6, Eigen 5.0.1 and mpmath
1.3.0. Windows x64 is the validated platform. No latency budget was supplied;
these measurements establish costs rather than performance acceptance.

- Candidate production-source identity: `d9ac41f90154558eb6191b79d6a891fc0a7aa0cc287c9c405250cae224f5ac86`.
- Candidate DLL SHA-256: `896af586ba64bac9932487ccdbc6a37d0ea9552d3e40bac67b4b65567e9fa2a1`.
- Identity method: SHA-256 of compact sorted JSON mapping every production
  source/configuration path to its raw-byte SHA-256; tests, documentation and
  benchmarks have separately recorded hashes. This is a working-tree identity,
  not a Git commit. The branch starts at Identification commit
  `a018c691b3defaedb4be56cd6de43c84edd4a05e`
  ([PR #21](https://github.com/frfournier/robodk_crx_kinematics/pull/21)).
- Paired full-DLL baseline is Identification's frozen measured source
  `fffd97309ea894f9e20f70955c4d12a487444f1d3960dd7a1bf9b8bb04a319cb`,
  DLL `dcb3a2ef1bcdacb175d13a9e49c276a6792e0759c01c3702f86cfd8d390e5f06`.
  Its published content differs only in seven Git line-ending normalizations.
  Discovery is a sibling feature; its optimization is not a proof prerequisite.

Timing ran serially after all builds/tests finished, pinned to affinity mask 1.
Both harnesses use three warmups, nine rounds, alternating path order, and a
fixed shuffled input order. Each pose's reported time is the median of round
batch means; p50/p95/p99/max below span poses, not individual-call tails.
Feature batches use 64 repeats and complete output consumption behind volatile
indirect calls. Their times include call/consumption overhead and clock
quantization; very small double times are not isolated instruction latencies.
The four-repeat integration pilot rejected zero-duration clock samples; the
64-repeat pilot and final nine-round run passed independent containment checks.

## Arithmetic cost, width and domain results

The 144 feature inputs cover six captured asset geometries, three scales
(0.001, 1, 1000), and eight groups of 18: point, narrow joint box, base/tool
frames, signed lengths, encoder seam, large encoder, quaternion scaling/large
scalar trig, and zero-crossing division. They are constructed finite nominal
inputs, with no IK-success selection. Unsupported/extreme-domain behavior is
tested separately. Fractions and 200-digit transform products independently
check every frame/coefficient output and three FK box samples on every input.

The double rows evaluate point inputs using ordinary numerical arithmetic;
the interval rows enclose their corresponding boxes with the shared expression
trees. Full exact model/frame preparation does more work than numerical wrist
circle preparation; those are distinct tasks and no speed ratio is inferred.
All times are microseconds.

| Operation | Double p50 | Double p95 | Interval p50 | Interval p95 | Interval p99 | Interval max |
|---|---:|---:|---:|---:|---:|---:|
| add | 0.004687 | 0.004687 | 0.017188 | 0.018750 | 0.018750 | 0.018750 |
| multiply | 0.004687 | 0.004687 | 0.020313 | 0.020313 | 0.020313 | 0.020313 |
| divide | 0.004687 | 0.004687 | 0.020313 | 0.021875 | 0.021875 | 0.021875 |
| sqrt | 0.004687 | 0.004687 | 0.018750 | 0.018750 | 0.018750 | 0.018750 |
| trig_pair | 0.012500 | 0.014063 | 1.965625 | 1.984141 | 2.049016 | 2.401562 |
| matrix3_product | 0.009375 | 0.009375 | 0.671875 | 0.695078 | 0.704688 | 0.706250 |
| norm3 | 0.004687 | 0.004687 | 0.079687 | 0.081250 | 0.081250 | 0.081250 |
| coefficient_convolution | 0.082812 | 0.084375 | 4.270313 | 4.322969 | 4.360234 | 4.371875 |
| canonical_fk | 0.064062 | 0.073203 | 13.969531 | 14.334766 | 14.537500 | 15.787500 |
| circle_prepare | 0.029687 | 0.029687 | — | — | — | — |
| exact_prepare | — | — | 13.577344 | 39.287422 | 40.042828 | 40.289062 |
| polynomial_bundle | — | — | 9.235156 | 13.061797 | 13.748906 | 13.985938 |
| command_fk | — | — | 14.087500 | 14.238516 | 14.553187 | 14.651563 |

Preparation, polynomial bundles, canonical FK and command FK returned finite
enclosures on all 144 inputs (0 arithmetic failures). Division deliberately
rejected 18/144 zero-crossing boxes; the corresponding double midpoint was zero
and also rejected. Other measured operations had zero failures. Finite `Ready`
results do not establish any IK certificate.

Principal-angle uncertainty occurred on 36/144 inputs (all encoder-seam and
large-encoder cases). Scalar trig returned its full `[-1,1]` range on 18/144
inputs, all in the large-trig group. Raw large encoder FK also uses full-range
trig internally: its resulting rotation entries have width 144. This deliberately
broad enclosure reflects dependency accumulation, not rotation-coordinate
precision. No performance gain is claimed from these broad cases.

Each cell below is the median / maximum, across that group's 18 poses, of the
largest component enclosure width for the field. Lengths/translation and `P`
use the declared normalized units; rotations/derivative/determinant use the
corresponding normalized expressions. The maximum width does not bound a
physical error or a root radius.

| Input group | Target R width | P width | P' width | Canonical FK R width | Command FK R width |
|---|---:|---:|---:|---:|---:|
| point | 7.883e-15 / 8.660e-15 | 7.941e-13 / 4.325e-12 | 3.247e-12 / 2.164e-11 | 2.864e-14 / 4.163e-14 | 3.170e-14 / 4.552e-14 |
| box | 8.271e-15 / 9.326e-15 | 7.005e-13 / 1.655e-12 | 3.148e-12 / 7.313e-12 | 1.474e-08 / 1.877e-08 | 1.474e-08 / 1.877e-08 |
| frames | 2.176e-14 / 2.731e-14 | 1.291e-12 / 2.472e-11 | 5.297e-12 / 8.141e-11 | 2.975e-14 / 3.553e-14 | 3.308e-14 / 4.052e-14 |
| signed | 8.160e-15 / 9.215e-15 | 2.721e-13 / 7.532e-12 | 1.236e-12 / 3.385e-11 | 2.831e-14 / 3.131e-14 | 3.109e-14 / 3.575e-14 |
| encoder_seam | 8.216e-15 / 8.993e-15 | 6.619e-13 / 3.659e-12 | 2.804e-12 / 1.355e-11 | 3.103e-14 / 3.908e-14 | 6.178e-14 / 9.837e-14 |
| large_encoder | 8.105e-15 / 9.104e-15 | 8.233e-13 / 7.326e-12 | 3.689e-12 / 2.933e-11 | 3.025e-14 / 3.453e-14 | 1.440e+02 / 1.440e+02 |
| quaternion_scale_and_large_trig | 8.049e-15 / 9.104e-15 | 3.827e-13 / 1.132e-12 | 1.593e-12 / 5.004e-12 | 2.875e-14 / 3.952e-14 | 3.247e-14 / 4.419e-14 |
| division_seam | 7.883e-15 / 9.548e-15 | 7.159e-13 / 2.853e-12 | 2.897e-12 / 1.427e-11 | 2.826e-14 / 4.263e-14 | 3.275e-14 / 4.807e-14 |

Raw per-case widths for normalized translations, wrist centres, determinant
coefficients and both FK translations are retained with all rotation/polynomial
widths. Group timing, domain statuses, and broad-range diagnostics are retained
separately so exceptional cases cannot silently improve ordinary-input figures.

## Complete numerical solver comparison

The frozen original corpus contains 4,313 targets; the scaled stress corpus
contains 3,024. Each was measured with and without a nearby seed, eight repeats
per round, capacity 512, the same DLLs and independently rebuilt asset-frame FK
acceptance (position <= 1e-4 mm; rotation <= 1e-3 degrees). Counter probes run
separately from timing and agree with uninstrumented output counts.

Across all 14,674 case/mode pairs, every ordered numeric command array is exactly
equal to the baseline, including the selected best command. There are zero lost
strict solves, lost known witnesses, count changes or invalid outputs, and no
modulo-turn matching was used. No changed-case CSV is needed. Candidate and
baseline errors are consequently identical. Reachable strict solves / witness
hits in each mode are 3,929 / 3,726 on the original corpus and 3,024 / 2,994 on
the stress corpus. The original corpus also includes 384 unreachable targets.

Below, `mixed` is the fixed mixed-singular group; `reachable` is all targets
classified reachable by the corpus. Paired values are baseline -> candidate,
in microseconds. No optimization or speedup is asserted for this refactoring.

| Corpus | Mode | Group | p50 | p95 | p99 | max |
|---|---|---|---:|---:|---:|---:|
| existing | no_seed | all_reachable | 19.637 -> 19.587 | 99.763 -> 98.712 | 154.105 -> 154.935 | 262.925 -> 264.762 |
| existing | no_seed | mixed_singular | 92.550 -> 92.419 | 165.157 -> 165.210 | 212.940 -> 210.316 | 262.925 -> 264.762 |
| existing | nearby_seed | all_reachable | 19.613 -> 19.650 | 99.407 -> 98.845 | 153.016 -> 154.216 | 265.425 -> 261.475 |
| existing | nearby_seed | mixed_singular | 92.831 -> 92.731 | 165.005 -> 164.552 | 204.678 -> 208.318 | 265.425 -> 261.475 |
| stress | no_seed | all_reachable | 21.137 -> 21.156 | 117.209 -> 116.954 | 152.561 -> 152.542 | 298.538 -> 298.375 |
| stress | no_seed | mixed_singular | 96.613 -> 95.294 | 151.831 -> 151.437 | 185.059 -> 179.787 | 199.800 -> 199.450 |
| stress | nearby_seed | all_reachable | 21.113 -> 21.200 | 118.189 -> 117.541 | 155.731 -> 150.039 | 293.625 -> 299.200 |
| stress | nearby_seed | mixed_singular | 95.725 -> 95.844 | 152.328 -> 148.824 | 180.185 -> 180.300 | 201.037 -> 195.562 |

## Reproduction and evidence

After the validated build commands above, run each benchmark in a fresh output
directory while other builds/tests/benchmarks are idle:

```powershell
uv run --locked python scripts/benchmark_certification.py --rounds 9 --repeats 64 --output build/evidence/arithmetic-components
uv run --locked python scripts/benchmark_accuracy.py --before PARENT_DLL --after build/Release/crx_kinematics.dll --corpus existing --rounds 9 --repeats 8 --capacity 512 --before-counters PARENT_COUNTERS --after-counters build/Release/crx_solver_counters.exe --output build/evidence/arithmetic-existing
uv run --locked python scripts/benchmark_accuracy.py --before PARENT_DLL --after build/Release/crx_kinematics.dll --corpus stress --rounds 9 --repeats 8 --capacity 512 --before-counters PARENT_COUNTERS --after-counters build/Release/crx_solver_counters.exe --output build/evidence/arithmetic-stress
```

`benchmark_certification.py` also accepts `--runner` and `--compile-commands` to
measure archived binaries. `benchmark_accuracy.py` accepts `--after-sources`,
`--before-revision`, `--after-revision` and `--compile-commands` for provenance.
The actual measurement used immutable DLLs, feature runner, matched counter
probes, source ZIP and compile-command snapshot. Raw timing/solution/error/count
CSVs, corpus copies, per-case widths and statuses, manifests/hashes, strict
command lines, Release/Debug logs and optimized object disassembly are retained
under `build/swarm/artifacts/certification-foundations/review-d9ac41f90154558e/`.
The durable tables and mathematics here can be reviewed independently of those
local files. Optimized interval/exact objects contain no fused multiply-add
instructions; the interval object contains two explicit `sqrtsd` instructions.

The implementation remains private. It supplies enclosures and exact nominal
input identity to later certificate/catalogue work; it adds no public ABI entry
point, proof outcome, root isolator or calibrated-model certificate.
