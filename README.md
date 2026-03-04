# Fanuc CRX (CRX-10iA family) — Custom Inverse Kinematics for RoboDK

## Overview

This repository provides a deterministic inverse-kinematics (IK) solver for the FANUC CRX collaborative robot family (6R serial arm with a **non-spherical wrist**). The solver is compiled as a RoboDK custom kinematics library, following the Plug-In-Interface "robotextensions/samplekinematics" contract with `SolveFK`/`SolveIK` entry points and RoboDK's `robot_T` parameter layout.

## Algorithmic Basis

The IK method follows the fully geometric approach described by:

> **M. Abbes, G. Poisson**, "Geometric Approach for Inverse Kinematics of the FANUC CRX Collaborative Robot", **Robotics 13(6):91, 2024**.  
> DOI: https://doi.org/10.3390/robotics13060091

**Key insight:** The solver reduces the 6-DoF IK to a 1-D root-finding problem over a scalar function derived from geometric constraints of the CRX architecture. It enumerates all valid postures (typically 8/12/16 solutions when they exist) and remains robust near singularities by validating candidate solutions against forward-kinematics (FK) consistency checks.

## Reference Implementation & Provenance

This RoboDK-oriented implementation is inspired by the open-source CRX FK/IK package by Daniel Cranston (MIT):

- **Original CRX Kinematics Package**: https://github.com/danielcranston/crx_kinematics

**RoboDK Integration Target**:

- **RoboDK Custom Kinematics Sample**: https://github.com/RoboDK/Plug-In-Interface/tree/master/robotextensions/samplekinematics

## Conventions & Units (RoboDK Integration Notes)

- **Pose representation**: 4×4 homogeneous transform matrices (row-major in the RoboDK C API); position in millimeters
- **Joint angles**: Revolute joints; angles interpreted/returned per RoboDK's expected custom kinematics units (typically degrees at the interface boundary; internal computation uses radians — conversions are explicit)
- **Base and Tool adaptation**:
  - RoboDK provides a *robot base adaptor* and a *tool flange adaptor* through `robot_T`
  - These must be applied consistently:
    ```
    world_T_tcp = world_T_base × base_T_robot × FK(DH, q) × robot_T_flange × flange_T_tcp
    ```
  - IK inverts this relationship accordingly
- **DH parameters**: Read from `robot_T` (configured in the `.robot` model)
  - Must match the convention assumed by the solver
  - Pay attention to RoboDK-specific coupling/remapping (e.g., CRX J2/J3 coupling)
  - Account for any "analytic IK convention" conversions used in the derivation

## Numerical Behavior & Validation

- **Solution enumeration**: The solver scans/solves the 1-D constraint and back-substitutes remaining joints
- **Candidate validation**: Each candidate is validated by FK reconstruction:
  - `pose_error(position, orientation) ≤ tolerance`
  - Rejects spurious roots and handles near-singular cases safely
- **Deduplication**: Solutions can be deduplicated by an angular threshold to remove numerical duplicates created by root-finding tolerances

## Safety & Legal

- This implementation is provided **"AS IS"**, without warranty of any kind
- FANUC® and RoboDK® are trademarks of their respective owners
- This project is **not affiliated with or endorsed** by FANUC or RoboDK

---

## Development

# Development Guide: robodk_crx_kinematics

This document provides step-by-step instructions for installing, building, and testing the CRX kinematics solver.

## Prerequisites

- **Windows** (x64)
- **Git** with Git LFS support
- **Visual Studio** (Community, Professional, or Enterprise) with C++ build tools
- **Python 3.13+**
- One of the following build systems:
  - **QMake** (Qt 5.x or later)
  - **CMake** (3.16 or later)

## Installation

The installation process sets up Git LFS for tracking RoboDK assets and pulls the Eigen 5.0.1 library as a git submodule.

### Step 1: Run the Install Script

From the repository root, execute:

```bash
scripts\install.bat
```

This script performs the following actions:

1. **Initializes Git LFS**: Sets up Git Large File Storage tracking for `*.rdk`, `*.sld`, and `*.robot` files
2. **Commits LFS configuration**: Records the `.gitattributes` file
3. **Adds Eigen submodule**: Clones Eigen 5.0.1 from the official GitLab repository into `third_party/eigen`
4. **Pins the Eigen version**: Checks out and records Eigen 5.0.1 commit
5. **Commits submodule pin**: Records the submodule configuration in git

**What gets created:**
- `.gitattributes` – LFS configuration
- `.gitmodules` – Submodule manifest
- `third_party/eigen/` – Eigen library source (linked as submodule)

### Step 2: Verify Installation

Confirm the following directories and files exist:

```
third_party/eigen/
  ├── Eigen/
  ├── CMakeLists.txt
  ├── README.md
  └── signature_of_eigen3_matrix_library

.gitattributes
.gitmodules
```

## Building

Choose one of the two build approaches below. Both require Visual Studio to be installed.

### Option A: CMake Build (Recommended)

CMake is the modern build system and integrates seamlessly with Visual Studio.

#### Step 1: Run the CMake Build Script

From the repository root:

```bash
scripts\build_crx_kinematics_msvc.bat
```

#### What This Does

1. **Auto-detects Visual Studio** using `vswhere.exe`
2. **Initializes MSVC environment** via `vcvars64.bat`
3. **Discovers CMake** from PATH or common installation locations
4. **Configures the build** using Visual Studio 17 2022 generator (x64)
5. **Builds in Release mode**
6. **Verifies output**: Checks that `build\Release\crx_kinematics.dll` was created

#### Output

- **DLL**: `build\Release\crx_kinematics.dll`
- **Build artifacts**: `build\Release\` and `build\Debug\`

### Option B: QMake Build

QMake uses Qt's build system with nmake.

#### Step 1: Run the QMake Build Script

From the repository root:

```bash
scripts\build_crx_kinematics_qmake.bat
```

#### What This Does

1. **Auto-detects Visual Studio** using `vswhere.exe`
2. **Initializes MSVC environment** via `vcvars64.bat`
3. **Discovers qmake** from:
   - Existing QMAKE_EXE environment variable
   - PATH
   - Conda installation: `%USERPROFILE%\AppData\Local\miniconda3\Library\bin\qmake.exe`
   - Qt standard installation directories
4. **Runs qmake** with Release configuration
5. **Compiles using nmake**
6. **Verifies output**: Checks that `build\Release\crx_kinematics.dll` was created

#### Output

- **DLL**: `build\Release\crx_kinematics.dll`
- **Build artifacts**: Created by qmake in `build\`

#### QMake Discovery

If qmake is not found automatically, set the environment variable before running the script:

```bash
set QMAKE_EXE=C:\Path\To\Qt\bin\qmake.exe
scripts\build_crx_kinematics_qmake.bat
```

### Troubleshooting Builds

| Issue | Solution |
|-------|----------|
| `vswhere.exe not found` | Install Visual Studio Community/Professional with C++ workload |
| `CMake not found` | Install CMake or add it to PATH |
| `qmake.exe not found` | Install Qt or set `QMAKE_EXE` environment variable |
| `vcvars64.bat not found` | Install Visual Studio C++ build tools |
| DLL not found after build | Verify `.pro` file (for QMake) or `CMakeLists.txt` generates correct output name |

## Testing

Testing uses **Astral uv** as the Python environment and package manager, with **pytest** as the test runner.

### Prerequisites for Testing

1. **Python 3.13+** installed on your system
2. **Astral uv** installed globally

Install uv if not already installed:

```bash
pip install uv
```

Or using Astral's installer: https://docs.astral.sh/uv/getting-started/installation/

### Test Data: Excel to JSON Compilation

The test suite uses **parametrized test cases** based on ground truth data compiled from two authoritative sources:

1. **RoboGuide** — FANUC's official robot simulation software
2. **Abbes & Poisson (2024)** — The academic paper describing the geometric IK approach

#### Data Flow

```
CRX10iA-solutions.xlsx (RoboGuide + Paper ground truth)
            ↓
       compile_fixtures.py (conversion script)
            ↓
    CRX10iA-solutions.json (pytest parametrization source)
            ↓
    pytest loads JSON → generates test cases
            ↓
    Tests validate IK/FK against ground truth
```

#### Excel Structure

The `tests/fixtures/CRX10iA-solutions.xlsx` file contains:

| Column | Description |
|--------|-------------|
| `TEST CASE` | Test case ID (grouped by target pose) |
| `TEST NAME` | Human-readable test description |
| `X, Y, Z` | Target TCP position (mm) |
| `W, P, R` | Target TCP orientation — Roll, Pitch, Yaw (degrees) |
| `J1–J6` | Joint angles (degrees) for each solution |
| `SOLUTION ID` | Sequential ID within each test case |
| `FB, UD, TB` | Robot configuration (Front/Back, Up/Down, Tool/Base elbow) |

#### JSON Structure

The compiled `tests/fixtures/CRX10iA-solutions.json` groups solutions by test case:

```json
{
  "meta": {
    "robot_model": "CRX10iA",
    "units": {
      "position": "mm",
      "orientation": "deg",
      "joints": "deg"
    }
  },
  "test_cases": [
    {
      "id": 1,
      "name": "Home position",
      "target": {
        "xyz_mm": [x, y, z],
        "wpr_deg": [w, p, r]
      },
      "solutions": [
        {
          "id": 1,
          "joints_deg": [j1, j2, j3, j4, j5, j6],
          "config": {"FB": "F", "UD": "U", "TB": "T"}
        },
        ...
      ]
    },
    ...
  ]
}
```

#### Compilation & Test Parametrization

To compile the Excel file to JSON:

```bash
python tests/fixtures/compile_fixtures.py
```

The pytest suite then:
1. **Loads** `CRX10iA-solutions.json`
2. **Parametrizes** test functions with each test case
3. **Validates** IK solutions against the ground truth data
4. **Checks** that FK evaluates to the target pose within tolerance

This ensures the solver is tested against **real robot data** from both simulation and mathematical derivation.

### Step 1: Install Test Dependencies

From the repository root:

```bash
uv sync
```

### Step 2: Run Tests

Execute the test suite:

```bash
uv run pytest
```

Or with verbose output:

```bash
uv run pytest -v
```

Or run a specific test file:

```bash
uv run pytest tests/test_crx_kinematics.py -v
```

### Test Configuration

Test behavior is configured in `pyproject.toml` under `[tool.pytest.ini_options]`:

- **Test discovery**: `tests/` directory
- **Logging**: INFO level to console
- **Output**: Quiet mode (`-q` flag)

### Example Test Run Output

```
INFO Starting kinematics tests
test_crx_kinematics.py::test_forward_kinematics PASSED
test_crx_kinematics.py::test_inverse_kinematics[test_case_0] PASSED
test_crx_kinematics.py::test_inverse_kinematics[test_case_1] PASSED
test_crx_kinematics.py::test_inverse_kinematics[test_case_2] PASSED

====== 3 passed in 0.42s ======
```

## Complete Workflow

Here's the complete sequence to set up, build, and test:

```bash
# 1. Install dependencies and Eigen submodule
scripts\install.bat

# 2. Build the kinematics library (choose one)
scripts\build_crx_kinematics_msvc.bat      # CMake-based build
# OR
scripts\build_crx_kinematics_qmake.bat     # QMake-based build

# 3. Install Python test dependencies
uv sync

# 4. Run tests
uv run pytest
```

## Additional Resources

- **Eigen Library**: https://eigen.tuxfamily.org/
- **RoboDK**: https://robodk.com/
- **Pytest Docs**: https://docs.pytest.org/
- **Astral uv Docs**: https://docs.astral.sh/uv/
- **Qt/QMake Docs**: https://doc.qt.io/qt-5/qmake-manual.html (if using QMake)
- **CMake Docs**: https://cmake.org/documentation/ (if using CMake)

## Project Structure

```
robodk_crx_kinematics/
├── include/                           # C++ headers
├── src/                               # C++ source
├── third_party/                       # External dependencies
│   └── eigen/                         # Eigen 5.0.1 (git submodule)
├── tests/                             # Python tests
│   ├── test_crx_kinematics.py
│   ├── conftest.py
│   └── fixtures/
├── scripts/                           # Build and install scripts
│   ├── install.bat # Local deploy of configuration and vendoring
│   ├── build_crx_kinematics_msvc.bat
│   └── build_crx_kinematics_qmake.bat
├── build/                             # Build output (generated - not git checked)
├── CMakeLists.txt                     # CMake configuration
├── crxkinematics.pro                  # QMake project file
└── pyproject.toml                     # Python project metadata
```

## License

TBD