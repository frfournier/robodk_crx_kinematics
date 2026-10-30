# SPDX-License-Identifier: Apache-2.0
# SPDX-FileCopyrightText: 2026 Centre Technologique en Aérospatiale
"""Typed NumPy wrapper for the CRX solver DLL; importing has no native side effects."""

from ._native import (
    ClosedSolverError,
    ConfigurationError,
    JointLimitError,
    LibraryError,
    NativeError,
)
from ._solver import CrxSolver, FibreIdentificationResult, IKResult, IKStatus

__all__ = [
    "CrxSolver", "FibreIdentificationResult", "IKResult", "IKStatus",
    "ClosedSolverError",
    "ConfigurationError", "JointLimitError", "LibraryError", "NativeError",
]
