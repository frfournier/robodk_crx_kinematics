# FANUC CRX custom kinematics for RoboDK

**Current release:** `0.4.1` ([changelog](CHANGELOG.md))

C++17 forward and inverse kinematics for FANUC CRX robots with a non-spherical
wrist. Targets RoboDK **6.0.6.26901+**; Windows x64 and Linux Ubuntu 24.04 are the tested platform.

## Build and use

Requires Git LFS, Visual Studio 2022/2026 with C++ and Clang tools, CMake/Ninja,
and [uv](https://docs.astral.sh/uv/).

```powershell
git clone https://github.com/frfournier/robodk_crx_kinematics.git
cd robodk_crx_kinematics
git lfs pull
scripts\build_crx_kinematics_msvc.bat
```

The script configures the toolchain and Python dependencies. CMake uses local
Eigen headers or downloads them on the first build.

1. Close RoboDK and copy `build/Release/crx_kinematics.dll` into
   `<RoboDK>/bin/robotextensions/`.
2. Restart RoboDK and open the matching `*-Custom.robot` from [assets](assets).
   **These provided robot assets are required for the library to work.**
3. Use RoboDK's normal FK/IK tools; `assets/crx10ia-test-station.rdk` provides
   a CRX-10iA example.

Builds do not deploy automatically. Validate motion before using it on hardware.

## Solver logic

FK composes the robot model's modified-DH transforms, joint senses, and base/tool
frames. IK reduces the geometry to one angle around a wrist circle, reconstructs
joint postures, and checks them against the complete FK pose.

IK finds candidates with polynomial roots and geometric refinement, applies
limits to coupled RoboDK commands, and ranks valid full turns by squared command
travel from the seed. Without a seed, ordering is lexicographic. When the turn
catalogue exceeds the requested capacity, ordered selection can stop after
establishing the ranked prefix and whether additional valid commands exist.
It retains the same range/work guards and strict FK checks on returned commands.
Complete coverage of singular families is not guaranteed.

Restricted J6 and coupled J3 limits exclude incompatible wrist-angle domains
before branch reconstruction. The exclusions allow for geometric refinement,
pose tolerance and roundoff; ambiguous boundaries stay in the search. A pruned
search without a validated result retries full discovery before constrained
correction or seed fallback.

With `max_solutions == 1` and `joints_approx`, IK first checks the seed and then
follows its elbow branch in a bounded wrist-angle neighborhood. It returns the
first nearby command passing the existing strict FK and joint-limit checks,
without enumerating other roots or turns. This result need not be the globally
closest solution. A local miss runs the ranked search above. Requests for
multiple solutions and calls without a seed retain ranked selection across
the discovered feasible branches.

## RoboDK callback contract

The public [C ABI](include/crx_kinematics.h) follows RoboDK's
[samplekinematics interface](https://github.com/RoboDK/Plug-In-Interface/blob/master/robotextensions/samplekinematics/samplekinematics.h).
RoboDK passes robot parameters through `ptr_robot`; poses are column-major
4x4 arrays of 16 doubles, translations are millimetres, and CRX joints are degrees.

| Callback | Role |
| --- | --- |
| `SolveFK` | Joint values to pose, with limit checks |
| `SolveFK_CAD` | Pose and intermediate link poses for display, without limit checks |
| `SolveIK` | Target pose to chosen joints and optional alternatives |
| `Joints2Config` | `[REAR, LOWERARM, FLIP]` flags |

`SolveIK` returns a solution count up to `max_solutions`, `0` for no solution,
or `-1` to request RoboDK's generic solver. `joints_approx` is an optional seed.
Each `joints_all` entry occupies **12 doubles**; this library writes six command
angles followed by six zeros, with the chosen `joints` first.

The sample defines FK returns as `1` for success, `0` for no solution, and `-1`
for generic fallback. This implementation currently uses `-2` for FK limit
violations.

## Verify

```powershell
scripts\build_crx_kinematics_msvc.bat --verify
```

Runs formatting, build, clang-tidy, and native/Python tests. Live RoboDK tests
are opt-in; see [tests](tests/README.md). Public declarations are in `include/`;
implementation and private headers are in `src/`.

## Sources

- [Abbes & Poisson, 2024](https://doi.org/10.3390/robotics13060091): geometric CRX inverse kinematics.
- [Daniel Cranston's CRX kinematics](https://github.com/danielcranston/crx_kinematics): implementation reference.
- [IK-Geo](https://github.com/rpiRobotics/ik-geo): geometric subproblem decomposition and one-dimensional search reference.
- [RoboDK samplekinematics](https://github.com/RoboDK/Plug-In-Interface/tree/master/robotextensions/samplekinematics): plugin interface and integration sample.

[Apache-2.0](LICENSE). See [third-party notices](THIRD_PARTY_NOTICES.md).
