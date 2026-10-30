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

With a seed and capacity one, IK may return a strictly validated nearby
solution before enumerating other branches. It is not guaranteed globally
closest. Larger requests rank discovered solutions by squared command travel
from the seed; without a seed, ordering is lexicographic. There is no implicit
current-joint state. `truncated=False` does not certify complete discovery.
Solutions whose six joint values round to the same three-decimal-place degree
values are returned once; returned values retain full solver precision.

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
