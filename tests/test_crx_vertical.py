# SPDX-License-Identifier: Apache-2.0
# SPDX-FileCopyrightText: 2026 Centre Technologique en Aérospatiale
"""Complete numerical vertical sets, exact discriminants, and retained fallbacks."""

import json
from pathlib import Path

import numpy as np
import pytest
from crx_asset_reference import canonical_forward, derive_model
from crx_kinematics import Coverage, CrxSolver
from crx_reference import rotation
from crx_vertical_reference import vertical_reference

ASSETS = json.loads((Path(__file__).parent / "fixtures/crx_asset_frames.json").read_text())["assets"]


def _lengths(asset):
    dh = derive_model(asset)["dh"]
    return np.array([dh[2, 1], dh[3, 3], dh[5, 3], -dh[4, 3]])


def _target(lengths, center, normal, yaw=0.):
    pose = np.eye(4)
    cosine, sine = np.cos(yaw), np.sin(yaw)
    pose[:3, :3] = [[cosine, -normal*sine, 0], [sine, normal*cosine, 0], [0, 0, normal]]
    pose[:3, 3] = np.asarray(center) + [0, 0, normal*lengths[2]]
    return pose


def _probe(canonical_probe, lengths, target, scale=1., general=False):
    return canonical_probe("--discover-general" if general else "--discover-vertical", [
        *lengths, *target[:3, :].flat, scale*1e-7, 1e-10,
    ])


def _check_set(result, lengths, target, oracle, position_tolerance, joint_tolerance=1e-7):
    actual = np.asarray(result["solutions"], dtype=float).reshape(-1, 8)[:, :6]
    for q in actual:
        pose = canonical_forward(lengths, q)
        assert np.linalg.norm(pose[:3, 3] - target[:3, 3]) <= position_tolerance
        assert np.linalg.norm(pose[:3, :3] - target[:3, :3]) <= 3e-10
    if oracle["family"]:
        assert not result["vertical_closed_form"]
        return
    expected = np.asarray(oracle["solutions"], dtype=float).reshape(-1, 6)
    assert len(actual) == len(expected), (result, expected)
    if not len(expected):
        return
    assert result["status"] == "Candidates"
    distance = np.max(np.abs((expected[:, None, :] - actual[None, :, :] + np.pi)
                             % (2*np.pi) - np.pi), axis=2)
    assert np.max(np.min(distance, axis=0)) < joint_tolerance
    assert np.max(np.min(distance, axis=1)) < joint_tolerance
    # Compare the complete wrist-phase set as world axes, independently of
    # joint-six chart/angle conventions and base-flip labels.
    axes = np.array([(rotation(2, q[0]) @ rotation(1, q[1]-q[2])
                      @ rotation(0, -q[3]))[:3, 1] for q in actual])
    expected_axes = np.array(oracle["axes"])
    phase_distance = np.linalg.norm(expected_axes[:, None, :] - axes[None, :, :], axis=2)
    assert np.max(np.min(phase_distance, axis=0)) < joint_tolerance
    assert np.max(np.min(phase_distance, axis=1)) < joint_tolerance


@pytest.mark.parametrize("asset", ASSETS, ids=lambda asset: asset["asset"])
@pytest.mark.parametrize("scale", [.001, 1., 1000.])
@pytest.mark.parametrize("sign", [-1., 1.])
@pytest.mark.parametrize("normal", [-1., 1.])
@pytest.mark.parametrize("height", [0., .3])
def test_vertical_asset_sets(canonical_probe, asset, scale, sign, normal, height):
    lengths = _lengths(asset)*scale
    lengths[[1, 3]] *= sign
    target = _target(lengths, [0.7*lengths[0], 0.1*lengths[0], height*lengths[0]], normal, .7)
    oracle = vertical_reference(lengths, target)
    result = _probe(canonical_probe, lengths, target, scale)
    assert result["vertical_closed_form"], result
    _check_set(result, lengths, target, oracle, scale*1e-7)


@pytest.mark.parametrize("asset", ASSETS, ids=lambda asset: asset["asset"])
@pytest.mark.parametrize("normal", [-1., 1.])
@pytest.mark.parametrize("sample", range(8))
def test_random_vertical_sets(canonical_probe, asset, normal, sample):
    lengths = _lengths(asset)
    rng = np.random.default_rng(20261113 + sample)
    center = lengths[0]*rng.uniform([.1, -.8, -1.8], [1.7, .8, 1.8])
    target = _target(lengths, center, normal, float(rng.uniform(-np.pi, np.pi)))
    oracle = vertical_reference(lengths, target)
    _check_set(_probe(canonical_probe, lengths, target), lengths, target, oracle, 1e-7)


@pytest.mark.parametrize("normal", [-1., 1.])
@pytest.mark.parametrize(("lengths", "center"), [
    ([1., 1., 2., .5], [.5, 0., .5]),  # Tangent T phase on the base axis.
    ([1.5, 1., 2., .5], [1.5, -.5, 2.]),  # Outer arm shell at the T phases.
    ([1.5, 1., 2., .5], [.5, -.5, 0.]),  # Inner arm shell, unequal links.
    ([1., 1., 2., .5], [1., 0., .5]),  # T and V-minus phases coincide.
    ([1., 1., 2., .5], [1., 0., 0.]),  # Zero height and merged V phases.
    ([-1., 1., 2., .5], [1., 0., .3]),  # Signed canonical upper arm.
], ids=["tangent", "outer-shell", "inner-shell", "coincident-branches", "zero-height", "signed-upper-arm"])
def test_vertical_boundaries(canonical_probe, normal, lengths, center):
    target = _target(lengths, center, normal)
    oracle = vertical_reference(lengths, target)
    result = _probe(canonical_probe, lengths, target)
    _check_set(result, lengths, target, oracle, 1e-7, 1e-6)


@pytest.mark.parametrize("normal", [-1., 1.])
@pytest.mark.parametrize(("lengths", "center"), [
    ([1.25, 1., 2., .75], [0., 0., 0.]),  # Zero-amplitude continuous wrist phase.
    ([1., 1., 2., .5], [.5, 0., 0.]),  # Coincident arm spheres at a T phase.
    ([1., 1., 2., .5], [0., 0., 4.]),  # Zero-amplitude excluded geometry.
], ids=["zero-amplitude-family", "coincident-spheres", "zero-amplitude-empty"])
def test_vertical_exceptional_fallback(canonical_probe, normal, lengths, center):
    target = _target(lengths, center, normal)
    result = _probe(canonical_probe, lengths, target)
    assert not result["vertical_closed_form"]
    _check_set(result, lengths, target, vertical_reference(lengths, target), 1e-7)


@pytest.mark.parametrize("normal", [-1., 1.])
@pytest.mark.parametrize("tilt", [-1e-6, -1e-12, -1e-16, 1e-16, 1e-12, 1e-6])
def test_near_vertical_retains_general_solving(canonical_probe, normal, tilt):
    lengths = [1., 1., 8/27, 5/18]
    target = _target(lengths, [.1, 0., .025], normal)
    target[:3, :3] = rotation(1, tilt)[:3, :3] @ target[:3, :3]
    result = _probe(canonical_probe, lengths, target)
    general = _probe(canonical_probe, lengths, target, general=True)
    assert not result["vertical_closed_form"]
    assert result["status"] == general["status"] == "Candidates"
    assert len(result["solutions"]) == 4
    np.testing.assert_array_equal(result["solutions"], general["solutions"])
    for row in result["solutions"]:
        actual = canonical_forward(lengths, row[:6])
        assert np.linalg.norm(actual[:3, 3] - target[:3, 3]) <= 1e-7
        assert np.linalg.norm(actual[:3, :3] - target[:3, :3]) <= 3e-10


@pytest.mark.parametrize("normal", [-1., 1.])
@pytest.mark.parametrize("offset", [-0., 0., -1e-320, 1e-320])
def test_vertical_offset_degeneracy_retains_general_route(canonical_probe, normal, offset):
    lengths = [1., 1., 2., offset]
    target = _target(lengths, [1., 0., .3], normal)
    result = _probe(canonical_probe, lengths, target)
    general = _probe(canonical_probe, lengths, target, general=True)
    assert not result["vertical_closed_form"]
    assert result["status"] == general["status"]
    np.testing.assert_array_equal(result["solutions"], general["solutions"])


@pytest.mark.parametrize("normal", [-1., 1.])
@pytest.mark.parametrize("component", [(0, 2), (2, 0)])
def test_admitted_rotation_roundoff_is_not_exact_vertical(canonical_probe, normal, component):
    lengths = [1., 1., 8/27, 5/18]
    target = _target(lengths, [.1, 0., .025], normal)
    target[component] = 16*np.finfo(float).eps
    result = _probe(canonical_probe, lengths, target)
    general = _probe(canonical_probe, lengths, target, general=True)
    assert not result["vertical_closed_form"]
    assert result["status"] == general["status"] == "Candidates"
    np.testing.assert_array_equal(result["solutions"], general["solutions"])


@pytest.mark.parametrize("normal", [-1., 1.])
def test_four_configuration_vertical_fibre(canonical_probe, kinematics_lib, normal):
    lengths = [1., 1., 8/27, 5/18]
    target = _target(lengths, [.1, 0., .025], normal)
    oracle = vertical_reference(lengths, target)
    assert len(oracle["solutions"]) == 4
    assert len(oracle["axes"]) == 2
    result = _probe(canonical_probe, lengths, target)
    assert result["vertical_closed_form"]
    _check_set(result, lengths, target, oracle, 1e-7)
    general = _probe(canonical_probe, lengths, target, general=True)
    assert not general["vertical_closed_form"]
    _check_set(general, lengths, target, oracle, 1e-7)
    with CrxSolver("CRX-10iA", dll_path=kinematics_lib._name,
                   joint_limits=[[-180.]*6, [180.]*6]) as solver:
        physical = target.copy()
        physical[:3, 3] *= 540
        identified = solver.identify(physical, max_solutions=512)
        assert identified.coverage == Coverage.NOT_ATTEMPTED
        assert not identified.truncated and identified.regular.all()
        assert len(identified.solutions) == (4 if normal > 0 else 8)
