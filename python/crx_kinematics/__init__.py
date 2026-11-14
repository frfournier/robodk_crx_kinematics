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
from ._solver import Coverage, CrxSolver, FibreIdentificationResult, IKResult, IKStatus
from ._certificates import BallCertificate, CertificateFailure
from ._calibrated import (
    CalibratedAttempt, CalibratedBallCertificate, CalibratedFrame, CalibratedGeometry,
    CalibratedIdentity, CalibratedOptions, CalibratedResult, CalibratedSolver, CalibratedStatus,
)

__all__ = [
    "BallCertificate",
    "CertificateFailure",
    "CalibratedAttempt",
    "CalibratedBallCertificate",
    "CalibratedFrame",
    "CalibratedGeometry",
    "CalibratedIdentity",
    "CalibratedOptions",
    "CalibratedResult",
    "CalibratedSolver",
    "CalibratedStatus",
    "ClosedSolverError",
    "ConfigurationError",
    "Coverage",
    "CrxSolver",
    "FibreIdentificationResult",
    "IKResult",
    "IKStatus",
    "JointLimitError",
    "LibraryError",
    "NativeError",
]
