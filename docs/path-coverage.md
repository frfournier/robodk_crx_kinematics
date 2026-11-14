# Certified nominal path coverage

The path tool builds a graph of actual command lifts at a finite set of task
poses. An edge requires a proof over the entire raw pose path, command-limit
admission, checked continuity between all proof cells and membership in the
declared endpoint roots. Unknown catalogues, omitted pairs, failed checks and
uncovered parameter intervals remain in the report.

Build with the Windows toolchain and run the included synthetic example:

```powershell
cmake --preset windows-clang-release
cmake --build --preset windows-clang-release
uv run python -m crx_kinematics.paths docs/examples/path-coverage.json `
  --backend build/Release/crx_path_tool.exe --output build/path-report.json
```

The example uses a synthetic one-millimetre-scale nominal model and two
identical poses with distinct task IDs. Its 32 command vertices form 16
separate certified components, each visiting both task IDs. Identical target
poses never merge different roots automatically. This fixture is not a
calibration or measured robot validation.

`uv run python scripts/crx_paths.py` is an equivalent repository entry point.
The Python wheel includes `crx_kinematics.paths`; supply the separately built
native executable and its generated license/source bundle using `--backend` or
`CRXKIN_PATH_BACKEND`. Named-model
requests also require a catalogue-capable solver DLL, selected by `--dll` or
the wrapper's existing library lookup. No test executable is a runtime
dependency. Output is UTF-8 JSON; `--output` replaces its destination only after
a successful native run and valid complete JSON output.

## Requests

Schema 1 accepts `model`, `tasks`, `limits_degrees`, and optional `connections`,
`convention`, `profile`, `reference`, `tool`, `seed_degrees`, `weights`, `options`,
`endpoint` and `tracking`. Unknown fields, duplicate JSON fields and nonfinite
values are rejected. Task IDs must be unique printable ASCII strings of 1 to
256 characters. There are at most 64 tasks and 256 requested connections.
Each task contains `id` and a `target` pose. Connections are pairs of declared
task IDs; the default connects adjacent tasks in input order. The coverage
claim permits a walk that visits all tasks in any order, including revisits.

A pose contains a raw `[w,x,y,z]` `quaternion` and a `position_mm` vector. The
defaults are the identity quaternion and zero translation. An optional
row-major `input_rotation` records a supplied matrix for deviation diagnostics;
it does not define the exact rotation. All finite JSON numbers are interpreted
as the exact real values of their parsed binary64 representations.

For each requested connection, translation and the **raw quaternion** vary
affinely in the real parameter `s` from zero to one. Rotation uses the
homogeneous quaternion action divided by its squared norm. Quaternion signs
are preserved: there is no sign correction, slerp or matrix interpolation. A
path whose quaternion vanishes cannot receive a complete certificate. Every
positive cell proves a nonzero denominator over its entire interval.

`model` can be a built-in model name such as `"CRX-10iA"`. Its convention must
be `"reference_map"`, which is also the named-model default. Optional top-level
`reference` and `tool` are raw poses in the wrapper's forward reference/TCP
convention. The DLL supplies the nominal model provenance only. Endpoint
catalogues are rebuilt natively from the declared raw target quaternions;
rounded matrix-to-quaternion targets from the metadata query are not reused.

Alternatively, `model` is a raw nominal-model object with required
`lengths_mm: [a,b,c,r]`, `lower_commands_radians` and `upper_commands_radians`.
Optional fields are `base_height_mm`, `encoder_zero_radians`, unit `senses`,
`base` and `tool`. This route needs no solver DLL. Its default convention is
`"base_pose"`; `"reference_map"` makes `model.base` the forward reference map
instead. Native represented radian bounds are retained as model provenance.
The path's actual limit box always comes from the explicit raw
`limits_degrees.lower` and `.upper`, converted with mathematical pi/180.

`profile` is `"callback"` by default: callback commands already contain the
robot's coupling. `"decoupled_user"` instead defines user[2] as
callback[2]-callback[1], and applies the supplied limits in those user
coordinates. Commands retain real turns; they are never wrapped to admit an
otherwise invalid path. Seed and positive weights are retained as request
provenance; this tool makes no nearest-path or travel-optimality claim.

## Budgets and reports

`options` sets graph-wide ceilings. Defaults are 512 vertices, 4096 pair tests,
32 path attempts, 32768 root nodes, 4096 endpoint point proofs, 4096 enumerated
lifts, 4096 path nodes and 32768 path point proofs. The corresponding JSON keys
are `max_vertices`, `max_pair_tests`, `max_paths`, `max_root_nodes`,
`max_endpoint_certificates`, `max_lifts`, `max_path_nodes` and
`max_path_certificates`. Work budgets are shared across the entire graph.

`endpoint` sets per-task `max_nodes`, `max_depth`, `max_leaves`,
`max_chart_attempts`, `max_candidate_certificates` and `max_pair_tests`.
`tracking` sets per-path `max_nodes`, `max_depth`, `max_point_certificates` and
`max_trials_per_node`. Shared remaining work further restricts each query.
Hard bounds reject excessive requests before proof work. Zero work budgets
retain incomplete results. A small successful query allocates according to
its retained work, rather than reserving every permitted path node.

Report `status` values are 0 InvalidInput, 1 Inconclusive, 2
CertifiedTaskCoverage, 3 CertifiedTaskInfeasible and 4 UnsupportedEnvironment.
Task coverage is positive evidence that a finite continuous admissible walk
in this certified subgraph can visit every supplied task. Task infeasibility
requires an existing sealed nominal point or exhaustive command-lift exclusion
at a supplied task. Missing edges and failed numerical/proof work cannot
establish infeasibility. `global_coverage` always remains `"Inconclusive"`.

The report includes exact raw model/target/limit words even when preparation
fails, prepared identity words, raw degree limits, point proof
constants, endpoint catalogue partitions, command ranges, vertices, scanned
pairs, attempted paths, certified edges and components. Each incomplete
connection retains its pair count and scan cursor. Every attempted path keeps
its full admitted/unresolved parameter partition. A vertex without a complete
catalogue or command enumeration can retain a matching sealed partial root,
with an independent constant-target command-admission proof. Such a vertex
makes no exhaustive root or turn claim.
`vertices_complete` means every enumerated candidate lift was retained; an
unbound or admission-undecided vertex remains unusable for graph edges.

Certified opposite Jacobian signs require a singularity along any continuous
connecting lift; this regular-cell tracker consequently remains inconclusive
for that crossing. A nominal base-flip relation is reported only after its
entire interval image is included in the other exact-target root's uniqueness
ball. Opposite certified shoulder-sine signs separately imply a shoulder-wall
crossing. Arbitrary same-sign pairs imply no such base-flip relation. These
necessary crossing conditions do not certify a connecting path or its absence.

`retained_payload_bytes` counts the graph object and retained vector capacities
using native `sizeof` values. It excludes allocator overhead, temporary
workspaces, JSON buffers and process RSS. The certificates concern nominal
kinematics and the stated position limits. Collision clearance, actuator
velocity/acceleration, calibration errors and executable robot trajectories
require their own models and checks.

## Native stage measurements

The opt-in harness links the same shipped libraries and accepts the same raw
request through the frontend's bounded wire adapter. Build and run its untimed
audit before collecting performance numbers:

```powershell
cmake --build --preset windows-clang-release --target crx_path_benchmark
uv run python scripts/benchmark_paths.py docs/examples/path-coverage.json `
  --harness build/Release/crx_path_benchmark.exe --audit --output build/path-audit.json
uv run python scripts/benchmark_paths.py docs/examples/path-coverage.json `
  --harness build/Release/crx_path_benchmark.exe --iterations 3 --rounds 5 `
  --output build/path-timing.json
```

The five stages are standalone endpoint preprocessing, whole graph construction,
replay of the graph's frozen attempted paths, uniform cell checking with existing
sealed point witnesses, and numerical tracking at exactly the retained leaf
midpoints of those paths. Endpoint preprocessing uses the declared per-task
options; it excludes graph binding and is not an additive breakdown of the
graph's shared-budget work. Frozen path replay retains each attempt's actual
remaining budgets. Numerical tracking starts at the same first root and feeds
each ordinary nomination to the next sample. It makes no between-sample
admission, continuity or coverage claim. Its sample workload is distinct from
the adaptive proof engine's additional internal nodes and point proofs.

Each measurement includes native work counts, all numerical sample proposals,
the owned graph evidence, request/packet/executable hashes, repeated stage
durations and checksums. Stage order reverses in alternate rounds. Parse and
serialization costs are outside the native stage timings. Independent oracle
audits and correctness runs belong outside serialized measurement intervals.

The frozen workload in `tests/fixtures/crx_path_queries.json` contains 42
synthetic queries, including all six approved asset model geometries, broad
motions, near-singular proposals and restricted budgets. Select an individual
case with `--case`, for example:

```powershell
uv run python scripts/benchmark_paths.py tests/fixtures/crx_path_queries.json `
  --case synthetic_moving --harness build/Release/crx_path_benchmark.exe `
  --audit --output build/path-moving-audit.json
```

Its model provenance and asset hashes identify the nominal geometry. Target
proposals use independent high-precision FK and are synthetic data. They do not
constitute sensor calibration or measured path accuracy.

Run the independent workload audit separately from timing:

```powershell
uv run python tests/audit_crx_paths.py `
  --dataset tests/fixtures/crx_path_queries.json `
  --harness build/Release/crx_path_benchmark.exe `
  --backend build/Release/crx_path_tool.exe --output build/path-audits
```

It checks retained positive point proofs with 180-digit independent FK and
Jacobians, every positive uniform cell with rational interval arithmetic,
gluing and endpoint membership, exact root counts/partitions, command-lift
enumeration, crossing-condition claims and graph components. It also compares
the harness's owned graph to the shipped standalone backend. Unresolved leaves
remain unresolved when their proof bounds are unavailable.
