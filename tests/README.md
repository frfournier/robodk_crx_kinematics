# Tests and fixtures

The test suite covers the native C ABI, fixture-based forward and inverse
kinematics, version metadata, and live RoboDK integration.

- `test_crx_kinematics.py` exercises the compiled library directly through
  `ctypes`.
- `test_crx_robolink_kinematics.py` validates the library through a live RoboDK
  instance and the Python RoboDK API.
- `test_version_metadata.py` checks that release versions agree across build
  and package metadata.

Run the suite from the repository root after building the library:

```powershell
uv sync
uv run pytest
```

Set `CRXKIN_LIBRARY_PATH` when the library is not in the default
`build/Release` location.

## Fixture provenance

The parametrized CRX-10iA cases combine ground-truth data from:

1. FANUC RoboGuide simulation results.
2. M. Abbes and G. Poisson,
   [“Geometric Approach for Inverse Kinematics of the FANUC CRX Collaborative Robot”](https://doi.org/10.3390/robotics13060091).

The committed CSV is the editable source of truth. The JSON is the generated
representation loaded by pytest:

```text
fixtures/CRX10iA-solutions.csv
              |
              v
fixtures/compile_fixtures.py
              |
              v
fixtures/CRX10iA-solutions.json
              |
              v
parametrized FK, IK, configuration, and RoboDK integration tests
```

## CSV schema

Each row in `fixtures/CRX10iA-solutions.csv` represents one joint solution.
Rows with the same `TEST CASE` share a target pose.

| Column | Description |
| --- | --- |
| `TEST CASE` | Test-case identifier used to group solutions |
| `SOLUTION ID` | Solution identifier within the test case |
| `TEST NAME` | Human-readable case name |
| `X`, `Y`, `Z` | Target TCP position in millimetres |
| `W`, `P`, `R` | Target TCP orientation in degrees |
| `J1`–`J6` | Joint angles in degrees |
| `FB`, `UD`, `TB` | Front/back, up/down, and turn/flip configuration letters |

The fixture configuration maps to RoboDK's `[REAR, LOWERARM, FLIP]` result as
follows:

| Fixture | RoboDK flag | False value | True value |
| --- | --- | --- | --- |
| `TB` | `REAR` | `T` | `B` |
| `UD` | `LOWERARM` | `U` | `D` |
| `FB` | `FLIP` | `N` | `F` |

## Generated JSON

`compile_fixtures.py` groups CSV rows by test case and stores the shared target
pose once:

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
      "name": "HOME",
      "target": {
        "xyz_mm": [540.0, -150.0, 380.0],
        "wpr_deg": [-180.0, 0.0, 0.0]
      },
      "solutions": [
        {
          "id": 1,
          "joints_deg": [0.0, 0.0, 0.0, 0.0, -90.0, 0.0],
          "config": {
            "FB": "N",
            "UD": "U",
            "TB": "T"
          }
        }
      ]
    }
  ]
}
```

## Regenerating fixtures

After editing the CSV, regenerate the JSON from the repository root:

```powershell
uv run python tests\fixtures\compile_fixtures.py
```

Review the generated JSON diff and run the fixture tests:

```powershell
uv run pytest tests\test_crx_kinematics.py -v
```

Fixture changes are intentional test-data changes and should be called out in
the release notes or change description.
