# FANUC CRX custom kinematics for RoboDK

**Current release:** `0.3.0` ([release history](CHANGELOG.md))

This project adds deterministic forward and inverse kinematics for FANUC CRX
robots in RoboDK. It is intended for RoboDK users who need reliable posture
enumeration for the CRX non-spherical wrist.

## Requirements

- RoboDK **6.0.6.26901**
- A matching 64-bit library for your operating system:
  - Windows: `crx_kinematics.dll`
  - Linux: `libcrx_kinematics.so`

RoboDK 6.0.5 and earlier use a different custom-kinematics interface and are
not supported. Windows x64 is the primary validated platform.

## Install in RoboDK

1. Close every RoboDK instance.
2. Build the library for your platform using the instructions below.
3. Copy the library into RoboDK's `bin/robotextensions` directory:
   - Windows: `<RoboDK>\bin\robotextensions\crx_kinematics.dll`
   - Linux: `<RoboDK>/bin/robotextensions/libcrx_kinematics.so`
4. Restart RoboDK. Robot-extension libraries are loaded at startup.
5. Open the custom robot model for your CRX from [`assets`](assets):
   - `Fanuc-CRX-5iA-Custom.robot`
   - `Fanuc-CRX-10iA-Custom.robot`
   - `Fanuc-CRX-10iA-L-Custom.robot`
   - `Fanuc-CRX-30iA-Custom.robot`

The included `assets/crx10ia-test-station.rdk` station can be used for a quick
CRX-10iA check.

> Custom kinematics affect robot motion and program generation. Validate
> generated programs in RoboDK and on the real controller using your normal
> safety process.

## Compile on Windows

### Prerequisites

- Git with Git LFS
- Visual Studio 2022 with:
  - Desktop development with C++
  - C++ Clang tools for Windows
  - CMake and Ninja

Clone the repository and initialize Eigen:

```powershell
git clone --recurse-submodules https://github.com/frfournier/robodk_crx_kinematics.git
cd robodk_crx_kinematics
git lfs pull
```

Build the Release DLL:

```powershell
scripts\build_crx_kinematics_msvc.bat
```

Output:

```text
build\Release\crx_kinematics.dll
```

The script finds Visual Studio, initializes its x64 environment, and builds
with `clang-cl`, CMake, and Ninja.

If the repository was cloned without submodules, run this first:

```powershell
git submodule update --init --recursive
```

## Compile on Linux

### Prerequisites

- A 64-bit Linux system
- Git with Git LFS
- A C++17 compiler, qmake, and make

On Ubuntu or Debian:

```bash
sudo apt update
sudo apt install --yes build-essential git git-lfs qt5-qmake qtbase5-dev
```

Clone and build:

```bash
git clone --recurse-submodules https://github.com/frfournier/robodk_crx_kinematics.git
cd robodk_crx_kinematics
git lfs pull
mkdir -p build/linux-release
cd build/linux-release
qmake ../../crxkinematics.pro "CONFIG+=release"
make -j"$(nproc)"
```

Output:

```text
build/Release/libcrx_kinematics.so
```

The Linux container uses the same qmake build path. Windows x64 currently
receives the project's full regression coverage.

## Developer verification

Tests use [uv](https://docs.astral.sh/uv/) and Python 3.12 or newer.
See the [test documentation](tests/README.md) for suite organization, fixture
provenance, and fixture regeneration instructions.

### Windows release gate

Run these commands from an x64 Visual Studio Developer PowerShell:

```powershell
git submodule update --init --recursive
uv sync
cmake --workflow --preset windows-clang-release-verify
```

### Linux tests

After building the Linux library and installing `uv`, run from the repository
root:

```bash
uv sync
CRXKIN_LIBRARY_PATH="$PWD/build/Release/libcrx_kinematics.so" \
  uv run pytest
```

Do not run `scripts/install.bat` for a normal build. It is a repository
bootstrap script that modifies Git configuration and creates commits.

## How it works

The solver implements the geometric method described by M. Abbes and
G. Poisson in
[“Geometric Approach for Inverse Kinematics of the FANUC CRX Collaborative Robot”](https://doi.org/10.3390/robotics13060091).
It reduces the six-axis inverse-kinematics problem to one-dimensional root
finding, enumerates valid postures, and verifies candidates through forward
kinematics.

The library implements RoboDK's current custom-kinematics callbacks:

| Callback | Purpose |
| --- | --- |
| `SolveFK` | Forward kinematics with joint-limit validation |
| `SolveFK_CAD` | Forward kinematics and intermediate CAD joint poses |
| `SolveIK` | Ranked inverse-kinematics solutions |
| `Joints2Config` | RoboDK `[REAR, LOWERARM, FLIP]` configuration flags |

The implementation is based in part on Daniel Cranston's
[CRX kinematics package](https://github.com/danielcranston/crx_kinematics)
and RoboDK's
[custom kinematics sample](https://github.com/RoboDK/Plug-In-Interface/tree/master/robotextensions/samplekinematics).

## License

This project is licensed under the
[Apache License 2.0](LICENSE).

## Notes

- RoboDK interface positions use millimetres and joint angles use degrees.
- Internal calculations use radians.
- Robot geometry, base/tool adapters, joint limits, and joint coupling are read
  from the RoboDK robot model.
- The project license, third-party notices, and Eigen licensing files are copied
  beside every build; see [`THIRD_PARTY_NOTICES.md`](THIRD_PARTY_NOTICES.md).
- FANUC and RoboDK are trademarks of their respective owners. This project is
  not affiliated with or endorsed by FANUC or RoboDK.
- The software is provided “AS IS”, without warranty of any kind.
