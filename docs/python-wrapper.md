# Python API

The Python layer is a uv package using the native `uv_build` backend. From
the repository root, run `uv sync --locked` to install it in the project
environment, then run Python or your scripts with `uv run --locked`.
For a runtime-only environment with DLL discovery, use
`uv sync --locked --no-dev --extra robodk`.

NumPy is the only required Python runtime dependency. Supply the DLL separately;
Windows x64 and the native library's AVX2/FMA requirements apply.

```python
import numpy as np
from crx_kinematics import CrxSolver

q = np.array([23, 60, -70, -37, 51, 83], dtype=np.float64)
with CrxSolver("CRX-10iA", dll_path=r"build/Release/crx_kinematics.dll") as solver:
    pose = solver.fk(q)                       # (4, 4)
    configuration = solver.configuration(q) # (3,), int32
    result = solver.ik(pose, seed=q)          # one nearby solution
    alternatives = solver.ik(pose, seed=q, max_solutions=64)
    identified = solver.identify(pose, max_solutions=64)
    J = solver.jacobian(q)                    # (6, 6)
    twist = J @ np.ones(6)                    # 1 degree/s on each command
    print(result.status.name, result.solutions)
    print(identified.aspect_signs, identified.wrist_addresses)
```

Without `dll_path`, the wrapper calls `robodk.robolink.getPathRoboDK()` and loads
`robotextensions/crx_kinematics.dll` beside the discovered executable. It does
not connect to or launch RoboDK. An explicit path always wins; errors never
cause fallback to another DLL. Importing the package has no native side effects.

## Models and coordinates

Supported names: `CRX-3iA`, `CRX-5iA`, `CRX-10iA`, `CRX-10iA-L`, `CRX-20iA-L`,
`CRX-25iA`, and `CRX-30iA`. Nominal geometry and total joint travel follow
FANUC's CRX specifications. FANUC publishes total travel spans rather than
command endpoints; asymmetric endpoints preserve the approved RoboDK asset's
midpoint where available. CRX-3iA endpoints are centered on zero. These values
are not calibration data.

Joint vectors use RoboDK command degrees. **Do not add or subtract J2 from J3.**
The solver already uses command coordinates; limits independently bound those
commands. Full turns and asymmetric limits are preserved.

Supply `joint_limits=np.array([lower, upper])`, shape `(2, 6)`, to override
limits at construction. FK and Jacobians enforce limits; configuration
classification accepts finite commands without limit checks. Constructor
settings are copied and properties return copies. Construct a new solver to
change settings. Model identity, limits, DLL path, native version, and ABI
version are available as properties.

## Tool, reference, and Jacobian

`tool` is flange-to-TCP and `reference` is base-to-reference. Both default to
identity. Transforms use millimetres and bottom row `[0,0,0,1]`. Rigid-transform
validation accepts float64 roundoff within `128 * eps`, matching the native
gate. It never projects or repairs a rotation. Float32 inputs must still meet
that tolerance; construct arbitrary rotations in float64.

```python
tool = np.eye(4)
tool[:3, 3] = [35, -24, 92]
reference = np.eye(4)
reference[:3, 3] = [-120, 80, 45]

with CrxSolver("CRX-10iA", tool=tool, reference=reference) as solver:
    target = solver.fk(q)
    result = solver.ik(target, seed=q)
    J_reference = solver.jacobian(q)
```

FK returns `inverse(reference) @ base_to_flange @ tool`; IK reverses that
transformation. Model-internal base/flange offsets are already included.

The analytic geometric Jacobian describes velocity at the **TCP origin**:

```text
J @ joint_velocity_deg_s = [vx_mm_s, vy_mm_s, vz_mm_s, wx_rad_s, wy_rad_s, wz_rad_s]
```

Both vectors are expressed in the fixed reference frame. Angular rows are
angular velocity components, not Euler derivatives. Tool offsets affect
linear velocity; reference translations do not move its evaluation point away
from the TCP. Singularities return finite, possibly rank-deficient matrices.

## IK outcomes and configurations

`ik(pose, seed=None, max_solutions=1)` returns an `IKResult`:

| Field | Meaning |
| --- | --- |
| `solutions` | Owned float64 `(N, 6)` command degrees |
| `configurations` | Owned int32 `(N, 3)`, `[REAR, LOWERARM, FLIP]` |
| `configuration_valid` | Boolean `(N,)`; undefined rows contain `[-1,-1,-1]` |
| `status` | `IKStatus`, preserving the native numerical outcome |
| `success`, `is_approximate` | Solution presence and approximate-seed outcome |
| `truncated` | Native search established that more valid commands exist |
| `used_singular_representatives` | Discovery used singular-family representatives |
| `position_residuals_mm` | TCP position residual per solution |
| `orientation_residuals_deg` | TCP rotation-angle residual per solution |

`identify(pose, max_solutions=64)` performs full, unseeded discovery and returns
the same selected command solutions with aligned `aspect_signs`, `regular`,
`wrist_addresses`, and `wrist_address_valid` arrays. Address rows are
`[arc, zero_based_root_rank, base_flip_bit]`; invalid rows contain `[-1,-1,-1]`.
Arcs follow increasing wrist-circle angle in `[-π, π)`, ranks are zero-based,
and the flip bit is zero for positive shoulder side and one for negative side.
Address validity describes an unambiguous numerical assignment and does not
certify exhaustive root discovery. It requires a native DLL exporting the
additive ABI 1 identification function; older ABI 1 DLLs continue to support
the existing methods, while `identify()` raises `NotImplementedError`.

`aspect_signs` uses the canonical Jacobian determinant, evaluated after mapping
the command vector into canonical radians. With reference senses
`s = (1,1,-1,-1,-1,-1)`, actual senses `sigma`, DH encoder offsets `theta0`,
and reference offsets `t = (0,-pi/2,0,0,0,0)`, each coordinate obeys
`q[i] = s[i]*sigma[i]*u[i] + s[i]*(theta0[i]-t[i])`.
Offsets equivalent modulo whole turns are represented by the principal angle
of the stored encoder rotation; admitted encoder roundoff near the reference
offset snaps to zero. Large encoder zeros use sine/cosine argument reduction
before subtracting the reference angle, avoiding accumulated binary64 period
error and loss of the reference shift.
The inverse is `u[i] = s[i]*sigma[i]*(q[i]-offset[i])`.
Commands are already coupled at this boundary. If a caller starts with
decoupled coordinates, its coupling map composes before this map; applying
coupling again inside the solver would change the model.

For the body or geometric Jacobian,
`det(J_command) = det(J_canonical) * product(sigma)`: the reference sense
product is one and fixed rigid base/tool adjoints have determinant one.
Controller `[REAR,LOWERARM,FLIP]` triples are noninjective and need not
distinguish all IK solutions. Base-flip partners have equal aspect signs.
On positive nominal geometry, away from singularities and modulo whole turns,
the sign describes the regular aspect; it is not a global branch identifier
for arbitrary signed or calibrated models. Current regularity is numerical:
`abs(det) > 1e-10*max(1,abs(a),abs(b),abs(r))**3`.
Here `a` is the upper-arm length, `b` the signed forearm length, and `r`
the wrist offset, in millimetres.
No local existence/uniqueness certificate is attached.

Wrist addresses are local to the particular target and wrist-circle basis.
Crossing the reference-angle seam or a fold can change arcs/ranks. At the
shoulder-side wall `sin(q[1])=0`, the flip bit is undefined and the address
remains invalid. These labels keep their numerical ambiguity guards.

`FibreIdentificationResult.coverage` is a `Coverage` enum. The additive
`crx_v1_identify2` endpoint currently returns `NOT_ATTEMPTED` for every outcome.
Coverage describes the internal nominal phase catalogue before limits, turn
expansion or output capacity. It is independent of `truncated`, command
admissibility and nearest-command claims. Only a later certified catalogue
implementation may report `COMPLETE`; an empty numerical search proves no
infeasibility. Older DLLs with only `crx_v1_identify` keep working, with
conservative `NOT_ATTEMPTED` coverage.

The extension uses a separate 16-byte `crx_v1_fibre_metadata` record:
`uint32 struct_size=16`, `uint32 version=1`, `int32 coverage`,
`uint32 reserved=0`. Supported size/version pairs are explicit; unknown
sizes, versions or nonzero reserved fields return invalid argument without
writing array outputs or extension bytes. Existing result structs stay
16 bytes and all existing symbols retain their signatures.

With a seed and capacity one, IK may return a strictly validated nearby
solution before enumerating other branches. It is not guaranteed globally
closest. Larger requests rank discovered solutions by squared command travel
from the seed; without a seed, ordering is lexicographic. There is no implicit
current-joint state. `truncated=False` does not certify complete discovery.
Solutions whose six joint values round to the same three-decimal-place degree
values are returned once; returned values retain full solver precision.
At equal travel, duplicate representatives prefer an exact command-box endpoint
after preserving an exact seed; this does not admit an out-of-limit command.

Approximate seed fallback is preserved with `status=APPROXIMATE_SEED`.
Native strict flange tolerances are 0.0001 mm and 0.001 degree; approximate
fallback tolerances are 0.01 mm and 0.01 degree. Residuals describe the requested
TCP; a long tool can amplify flange orientation error into a larger TCP position
error. Callers requiring strict results must inspect status and can imp
their own TCP residual bounds.

Empty/numerically unresolved searches return `(0,6)` solutions and distinct
statuses: `NO_CANDIDATE`, `UNRESOLVED`, `NUMERICAL_RANGE_FAILURE`, or `WORK_LIMIT`.
An empty search is not proof of unreachability. Configuration can be undefined
at an arm singularity without invalidating a solution. Standalone
`configuration()` raises `ConfigurationError` in that case.

## NumPy memory and errors

Inputs must have exactly the documented shapes. Real integer/floating arrays
are normalized to native-endian, aligned, C-contiguous float64 buffers. Arrays
that already meet those requirements are passed through without copying;
other layouts and dtypes are converted. Complex, boolean, string, object, and
nonfinite inputs are rejected. Read-only arrays are accepted, and inputs are
never modified. Do not mutate an input while a solver call is running because
ctypes releases the GIL during native computation.

The ctypes signatures use `numpy.ctypeslib.ndpointer` to check dtype,
dimensions, layout, alignment, and output writeability. Outputs own their data,
do not view native allocations, and survive `close()`. Calls and closure are
serialized per instance. Separate instances may run concurrently; ctypes
releases the GIL during native computation. Copying a solver with `copy.copy`
or `copy.deepcopy` is rejected; construct another instance for a separate handle.

Malformed input raises `ValueError`/`TypeError`; out-of-limit FK/Jacobian
commands raise `JointLimitError`; loading/ABI problems raise `LibraryError`;
native infrastructure/contract failures raise `NativeError`. A closed solver
raises `ClosedSolverError`. `close()` is idempotent. Context managers are
supported, with garbage collection as a cleanup fallback.

The additive C API in `include/crx_python_api.h` uses row-major matrices and
six-value IK rows. RoboDK retains column-major poses and twelve-value IK records.
DLLs without ABI 1 are rejected; no deployment or legacy fallback occurs.

## Distribution and testing

```powershell
uv sync --locked
uv build
uv run --locked pytest tests/test_python_wrapper.py
```

`uv sync` installs this package in editable mode. `uv build` produces a wheel
and source distribution under `dist/`, using `uv_build` rather than setuptools.
The module root remains `python/`; `py.typed` is included automatically as
package data. Runtime dependencies, the optional `robodk` extra, and development
dependencies are tracked in `uv.lock`.

To consume the package from another uv project, add this repository as a local
dependency, for example `uv add C:/path/to/robodk_crx_kinematics`. Alternatively,
add the built wheel with `uv add C:/path/to/crx_kinematics-0.5.0-py3-none-any.whl`.
To install a wheel into an existing virtual environment, use `uv pip install`
with the wheel path.

Build the native DLL first using repository CMake presets. The wheel/sdist
contain the Python package, metadata, and license, with no DLL or runtime test
dependencies. CTest selects the matching DLL explicitly. Live RoboDK tests
remain opt-in and require an already deployed matching DLL.
