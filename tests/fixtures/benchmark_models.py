# SPDX-License-Identifier: Apache-2.0
# SPDX-FileCopyrightText: 2026 Centre Technologique en Aérospatiale
"""Immutable command boxes for the v1 solver and accuracy benchmark corpora.

These are historical experiment inputs, not current robot catalogue defaults.
The frozen cases predate the asset-limit update in ef16299. Preserve their
sampling boxes from 036e0d24c2a1195f6bde9d896eeb5e3f304792ee while deriving
geometry from the independently captured frames. Current asset tests continue
to use the current capture's limits.
"""

from fixtures.generate_discovery_cases import make_corpus

_LIMITS = {
    "Fanuc-CRX-5iA-Custom.robot": (
        [-200., -180., -310., -190., -180., -225.],
        [200., 180., 310., 190., 180., 225.],
    ),
    "Fanuc-CRX-10iA-Custom.robot": (
        [-190., -179.99, -195., -190., -179.99, -225.],
        [190., 179.99, 375., 190., 179.99, 225.],
    ),
    "Fanuc-CRX-10iA-L-Custom.robot": (
        [-180., -180., -360., -190., -180., -190.],
        [180., 180., 430., 190., 180., 190.],
    ),
    "Fanuc-CRX-20iA-L-Custom.robot": (
        [-180., -180., -270., -190., -180., -225.],
        [180., 180., 270., 190., 180., 225.],
    ),
    "Fanuc-CRX-25iA-Custom.robot": (
        [-180., -180., -270., -190., -180., -225.],
        [180., 180., 270., 190., 180., 225.],
    ),
    "Fanuc-CRX-30iA-Custom.robot": (
        [-180., -180., -270., -190., -180., -225.],
        [180., 180., 270., 190., 180., 225.],
    ),
}


def benchmark_models(seed):
    """Return captured geometry with the frozen v1 command sampling limits."""
    models, cases = make_corpus(0, 0, seed)
    assert {model["name"] for model in models} == _LIMITS.keys()
    for model in models:
        model["robot"][[30, 31], :6] = _LIMITS[model["name"]]
    return models, cases
