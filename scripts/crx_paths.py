# SPDX-License-Identifier: Apache-2.0
# SPDX-FileCopyrightText: 2026 Centre Technologique en Aérospatiale
"""Repository entry point; implementation is also included in the Python wheel."""

from crx_kinematics.paths import main


if __name__ == "__main__":
    raise SystemExit(main())
