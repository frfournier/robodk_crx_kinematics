# Tooling and dependency review — 2026-10-09

The Windows x64 installation passes the release gate after the updates below.
The native solver, public ABI, RoboDK assets, and existing review documents were
not changed. Application release metadata remains 0.3.1. All changes were committed
locally; nothing was pushed or deployed.

## Installed tools and version decisions

| Component | Initially observed | Result and decision |
| --- | --- | --- |
| Visual Studio Community | 2026, 18.8.2 | Retained; complete installation, x64 development environment works. Documentation now allows 2022 or 2026. |
| clang-cl / clang-format / clang-tidy | 22.1.3, bundled with Visual Studio | Retained as one matched toolchain; clean compilation, formatting, and lint execution verified. |
| CMake / CTest | 4.3.1-msvc1, bundled with Visual Studio | Retained; project minimum raised from 3.20 to 3.25 to match preset schema/workflows. |
| Ninja | 1.13.2, bundled with Visual Studio | Retained; build execution verified. |
| uv | 0.12.22 | Installed patch upgrade to 0.12.24 using `uv self update 0.12.24`. This is a machine-level update, recorded here rather than a binary in Git. |
| Python | Runtime 3.14.7; project pin `3.14` | Installed and pinned 3.14.8; kept the existing minor series. |
| Eigen | Local headers identify as `5.0.1-dev+master`; fallback requested 5.0.1 | Local tree preserved. Official stable 5.0.1 archive is now checksum-pinned and independently build-tested. |
| RoboDK application | 6.0.6.26901 at `C:/Program Files/RoboDK/bin/RoboDK.exe` | Retained; matches the project's documented interface version. File version inspected; live application behavior not validated in this audit. |
| Git / Git LFS / GitHub CLI | 2.55.0.windows.3 / 3.7.1 / 2.97.0 | Executables respond; local commits work. No application-specific reason to change them was established. |
| Docker client / Compose | 29.7.2 / 5.3.1 | Compose configuration parses; Linux engine pipe is unavailable. No container build or runtime validation. |
| qmake | Not found on PATH | Compatibility build not executed; Windows CMake/Ninja remains the validated path. |

The native tools are not on an ordinary PowerShell PATH. This is an environment
activation requirement, not a missing installation. The build script locates a
Visual Studio installation containing all three required components: x64 C++,
Clang, and CMake tools. It initializes `vcvars64.bat` and puts that installation's
LLVM, CMake, and Ninja directories first on PATH.

Newer standalone [CMake 4.4.4](https://github.com/Kitware/CMake/releases/tag/v4.4.4)
and [LLVM 23.1.3](https://github.com/llvm/llvm-project/releases/tag/llvmorg-23.1.3)
were identified. They were not installed over the matched Visual Studio tools:
the installed bundle meets the project's requirements and passed the clean gate.
[Python 3.15.0](https://www.python.org/downloads/release/python-3150/) is a new minor
release; this audit takes the maintenance update within the existing 3.14 series.

## Python dependencies

Versions were checked against the official PyPI JSON endpoints, then installed
and tested one dependency at a time. The declared minimum and lockfile were
updated together. Transitive packages remain at their existing compatible pins.

| Package | Initial installed/locked version | New declared minimum and locked version | Commit |
| --- | --- | --- | --- |
| [Hypothesis](https://pypi.org/project/hypothesis/6.168.5/) | 6.161.2 | 6.168.5 | `23aef3c` |
| [NumPy](https://pypi.org/project/numpy/2.5.3/) | 2.5.1 | 2.5.3 | `c81abf8` |
| [pandas](https://pypi.org/project/pandas/3.0.6/) | 3.0.5 | 3.0.6 | `5130ecf` |
| [pytest](https://pypi.org/project/pytest/9.1.1/) | 9.1.1 | 9.1.1 | `c9255a0` |
| [RoboDK Python API](https://pypi.org/project/robodk/6.0.2/) | 6.0.1 | 6.0.2 | `ce9cd05` |

The pytest change aligns its declared minimum, formerly 9.0.2, with the installed
and tested version. The Python API package and RoboDK desktop application have
different version numbers; updating the package does not upgrade the application.

## Tooling fixes and commits

| Commit | Change |
| --- | --- |
| `7d49e4b` | Pin Python to the installed and tested 3.14.8 patch release. |
| `6f771da` | Stop ignoring `uv.lock` and commit the baseline dependency resolution before individual upgrades. |
| `0ba8a5d` | Require `--run-robodk` for live tests; remove automatic DLL copying; add missing/stale/matching deployment guard tests. |
| `c6218ba` | Use checked-in release presets in the build script, validate tool discovery, align CMake minimum, run CTest with `uv run --locked`, and correct Windows setup documentation. |
| `920e8a5` | Checksum-pin the Eigen archive and consume only headers, fixing the fresh-checkout configure failure. |
| This audit commit | Check environment synchronization against the lockfile in `--check` and record findings and validation. |

The previously ignored lockfile meant that fresh clones could not reproduce the
locally tested environment. It is now tracked. The build wrapper offers:

- `scripts\build_crx_kinematics_msvc.bat --check`: report tools and Python packages,
  require an environment synchronized with the lockfile, and check dependencies.
- `scripts\build_crx_kinematics_msvc.bat`: synchronize dependencies, configure and
  build `windows-clang-release-tidy`.
- `scripts\build_crx_kinematics_msvc.bat --verify`: synchronize dependencies and
  run the complete release workflow.

The default test gate now intentionally skips live RoboDK tests. Previously, its
setup silently copied the Release DLL into RoboDK. Live tests now require explicit
opt-in and an already deployed matching DLL. This audit performed no deployment.

## Eigen provenance and fresh-checkout validation

`third_party/` is ignored, and no `.gitmodules` is tracked. The local Eigen
`.git` file points to an unavailable parent repository's submodule storage, so its
Git revision cannot be established. Its files were not edited or replaced.

The official GitLab tags API resolves 5.0.1 to
`bc3b39870ecb690a623a3f49149a358b95c5781d`. The
[official archive](https://gitlab.com/libeigen/eigen/-/archive/5.0.1/eigen-5.0.1.tar.gz)
has SHA-256:

`e9c326dc8c05cd1e044c71f30f1b2e34a6161a3b6ecf445d56b53ff1669e3dec`

Testing with `CRXKIN_USE_VENDORED_EIGEN=OFF` exposed an unnecessary upstream
C-language configuration that selected `clang.exe` with MSVC-style flags and
failed. The corrected fallback downloads Eigen without adding its upstream build
project. Both local and fetched headers use the same imported interface target.
This uses the documented
[FetchContent SOURCE_SUBDIR behavior](https://cmake.org/cmake/help/latest/module/FetchContent.html#command:fetchcontent_makeavailable).
No vendored source edit was needed.

## Validation record

Commands ran from the repository root. Native commands used the Visual Studio
2026 x64 development environment. `UV_CACHE_DIR` was set to `build/uv-cache`;
most pytest runs used `-o cache_dir=build/pytest-cache`. Logs and scratch validation
scripts remain in ignored `build/`.

| Command or check | Result |
| --- | --- |
| `Get-Command`; tool `--version` commands; `vswhere -all -products * -format json` | Located and inventoried installed tools; plain-shell CMake/Ninja lookup failed until developer environment activation. |
| Official PyPI/GitHub/GitLab metadata queries via `Invoke-RestMethod` | Verified dependency releases, uv/CMake/LLVM releases, and the Eigen tag. |
| `git submodule status`; `git -C third_party/eigen rev-parse HEAD` / `status --short` / `describe --tags --always` | Initial sandbox Git shell failed; local Eigen Git commands independently found an invalid repository reference. No tracked submodule configuration exists. |
| `uv self update 0.12.24`; `uv --version` | Upgrade and installed version verified. |
| `uv python list --all-versions --only-downloads`; `uv python install 3.14.8`; `uv python pin 3.14.8`; `uv run --locked python --version` | Runtime installed, pinned, and executed. |
| `uv sync --locked` | Existing environment synchronized successfully. |
| `uv add "PACKAGE>=VERSION" --upgrade-package "PACKAGE==VERSION"`, for each package above | Targeted upgrade succeeded; each package has a separate commit. |
| `uv pip check`, after each package upgrade | All installed packages compatible. |
| `uv run --locked pytest --ignore=tests/test_crx_robolink_kinematics.py`, baseline and after each package update | 1,321 passed, one known expected failure each time; existing RoboDK import deprecation warning. |
| `uv run --locked pytest`, after deployment guard change and in final fresh environment | 1,324 passed, 298 intentionally skipped live cases, one expected failure, one existing warning. |
| `uv run --locked python build/validate_dependency_tools.py` | pandas fixture regeneration is semantically identical to committed JSON; Hypothesis search and RoboDK pose math checks pass. No fixture changes. |
| `cmake --build --preset windows-clang-release-tidy --clean-first` | Actual clean native compile/link and clang-tidy execution pass. Existing source diagnostics remain visible. |
| `cmake --workflow --preset windows-clang-release-verify` and wrapper `--verify` | Configure, formatting, build/lint, and CTest pass; deployment remains off. |
| `cmake --preset windows-clang-release-tidy -DCRXKIN_USE_VENDORED_EIGEN=OFF`, then format-check/build and matching CTest presets | Initially failed in Eigen's C compiler setup; passed after header-only fix, using the checked archive. |
| Wrapper `--check` | Passes for installed tools and synchronized environment. |
| Wrapper `--bad-option` | Expected exit code 2. |
| Wrapper `--check` with `UV_PROJECT_ENVIRONMENT=build/tooling-missing-environment` | Correctly rejects missing environment and does not create it. |
| `uv sync --locked` and `uv run --locked --no-sync pytest` with `UV_PROJECT_ENVIRONMENT=build/tooling-fresh-venv` | Fresh dependency installation and complete default suite pass independently of `.venv`. |
| `docker version`; `docker compose version`; `docker compose -f .docker/compose.yaml config --quiet` | Client/Compose versions obtained; base Compose config passes; Docker Linux engine unavailable. |
| `git diff --check`; staged diffs; `git status --short` | Reviewed task changes; unrelated review documents remain untracked and untouched. |

Early sandbox runs could not launch Ninja correctly or access pytest temporary
directories. They were stopped or failed, then rerun successfully with the required
filesystem/process access. A workspace pytest temporary-directory retry also
encountered access restrictions. uv encountered transient Windows cache rename
warnings and recovered automatically. These are recorded separately from actual
test failures; no tests or lint rules were weakened to address them.

## Remaining limits

- Live RoboDK integration is intentionally unverified. The 298 skips are explicit,
  not evidence of RoboDK runtime success. Running it requires `--run-robodk` and
  a separately authorized deployment.
- The local Eigen copy's exact revision remains unknown. The release archive is
  verified independently; this does not certify provenance of the local tree.
- Linux/qmake and Docker builds were not run. Their local Eigen supply still
  needs attention because `third_party` is not tracked; the CMake fallback fixes
  Windows fresh-checkout builds, not the Dockerfile's direct COPY requirement.
- `scripts/install.bat` remains a historical bootstrap script containing Git
  configuration and commit operations; it was reviewed but not executed.
- Visual Studio 2022, Python 3.12/3.13, and the minimum CMake 3.25 were not executed.
  Validation used the versions in the inventory above.
- Existing clang-tidy warnings, the RoboDK import deprecation warning, and the
  known ABBES-TABLE6 SOL4 fixture anomaly remain outside this tooling update.
