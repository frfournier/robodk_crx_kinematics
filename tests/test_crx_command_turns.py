"""Command limits/turns through the unchanged RoboDK sample ABI (#17).

Shifted bounds are synthetic tests of the legacy CRX callback model; the asset
API tests separately exercise the captured six-asset command limits.
"""

import ctypes
import itertools

import numpy as np
import pytest

from conftest import _set_row
from crx_reference import reference_fk
from test_crx_kinematics import _to_c_array


@pytest.fixture(params=["kinematics_lib"])
def command_lib(request):
    return request.getfixturevalue(request.param)


def target_for_command(robot, command):
    user = np.array(command, dtype=float)
    user[2] -= user[1]
    return reference_fk(robot, user)[0]


def solve(lib, robot, target, seed=None, capacity=64, alternatives=True):
    best = (ctypes.c_double * 6)(*([12345.] * 6))
    output = (ctypes.c_double * (12 * max(0, capacity) + 12))(
        *([12345.] * (12 * max(0, capacity) + 12)))
    count = lib.SolveIK(
        _to_c_array(target.ravel(order="F")), best,
        output if alternatives else None, capacity,
        _to_c_array(seed) if seed is not None else None, ctypes.byref(robot),
    )
    if count <= 0:
        assert list(best) == [12345.] * 6
        assert list(output) == [12345.] * len(output)
        return count, [], []
    assert count <= capacity
    commands = [list(output[12*i:12*i+6]) for i in range(count)] if alternatives else []
    if alternatives:
        assert list(best) == commands[0]
        for i in range(count):
            assert list(output[12*i+6:12*i+12]) == [0.] * 6
        assert list(output[12*count:]) == [12345.] * (len(output) - 12*count)
        for q in commands:
            assert np.all(np.array(q) >= np.array(robot.data[30][:6]) - 1e-10)
            assert np.all(np.array(q) <= np.array(robot.data[31][:6]) + 1e-10)
            fk = target_for_command(robot, q)
            assert np.linalg.norm(fk[:3, 3] - target[:3, 3]) <= 1e-4
            assert np.linalg.norm(fk[:3, :3] - target[:3, :3]) <= 2.5e-5
    return count, list(best), commands


def restrict(robot, command, width=.05):
    _set_row(robot, 30, np.array(command) - width)
    _set_row(robot, 31, np.array(command) + width)


@pytest.mark.parametrize("angle", [190., -190., 550., -550., 180., -180.])
def test_shifted_j6_seed_free(command_lib, crx_10ia, angle):
    witness = [25., -35., 35., 15., 25., angle]
    target = target_for_command(crx_10ia, witness)
    restrict(crx_10ia, witness)
    count, best, _ = solve(command_lib, crx_10ia, target)
    assert count == 1
    np.testing.assert_allclose(best, witness, atol=1e-5, rtol=0)


@pytest.mark.parametrize("command", [
    [25., 120., -100., 15., 25., 40.],
    [25., -120., 250., 15., 25., 40.],
    [25., 480., -460., 15., 25., 40.],
])
def test_coupled_j3_limits_and_fk(command_lib, crx_10ia, command):
    target = target_for_command(crx_10ia, command)
    restrict(crx_10ia, command)
    pose = (ctypes.c_double * 16)()
    assert command_lib.SolveFK(_to_c_array(command), pose, ctypes.byref(crx_10ia)) == 1
    np.testing.assert_allclose(np.array(pose).reshape(4, 4, order="F"), target, atol=1e-9, rtol=0)
    count, best, _ = solve(command_lib, crx_10ia, target)
    assert count == 1
    np.testing.assert_allclose(best, command, atol=1e-5, rtol=0)
    invalid = list(command)
    invalid[2] += 1.
    assert command_lib.SolveFK(_to_c_array(invalid), pose, ctypes.byref(crx_10ia)) == -2


def test_negative_dual_j3(command_lib, crx_10ia):
    # Base flip in decoupled coordinates: [J1+180,-J2,180-J3,J4-180,J5,J6].
    # This rear posture has an admissible negative J3 command representative.
    witness = [-155., 35., -175., -165., 25., 40.]
    target = target_for_command(crx_10ia, witness)
    restrict(crx_10ia, witness)
    count, best, _ = solve(command_lib, crx_10ia, target)
    assert count == 1
    np.testing.assert_allclose(best, witness, atol=1e-5, rtol=0)


@pytest.mark.parametrize("seeded", [False, True])
def test_exhaustive_turn_catalogue_and_capacity(command_lib, crx_10ia, seeded):
    witness = np.array([25., -35., 35., 15., 25., 40.])
    target = target_for_command(crx_10ia, witness)
    restrict(crx_10ia, witness)
    # Three independent command axes, including coupled J2/J3, yield 5^3 lifts.
    for axis in (1, 2, 5):
        crx_10ia.data[30][axis] = witness[axis] - 720.05
        crx_10ia.data[31][axis] = witness[axis] + 720.05
    seed = witness + [0., 650., -640., 0., 0., 690.] if seeded else None
    count, _, full = solve(command_lib, crx_10ia, target, seed, capacity=1024)
    # Narrow J1/J4/J5 bounds isolate the witness posture. The expected commands
    # come from the witness, independently of the solver's returned catalogue.
    expected = []
    lower, upper = np.array(crx_10ia.data[30][:6]), np.array(crx_10ia.data[31][:6])
    for phase in [witness]:
        for turns in itertools.product(range(-3, 4), repeat=3):
            q = phase.copy()
            q[[1, 2, 5]] += 360. * np.array(turns)
            if np.all(q >= lower) and np.all(q <= upper):
                expected.append(q)
    assert any(np.max(np.abs(q - witness)) < 1e-5 for q in full)
    assert count == len(expected) == 125
    key = (lambda q: (float(np.sum((q - seed)**2)), *q)) if seeded else (lambda q: tuple(q))
    expected.sort(key=key)
    if seeded:
        # Degree/radian conversion can reorder floating-point score ties.
        # Check the complete command set and sorted travel costs separately;
        # exact midpoint tie ordering is checked by the native reference tests.
        np.testing.assert_allclose(sorted(full), sorted(map(list, expected)), atol=1e-5, rtol=0)
        costs = [np.sum((np.array(q) - seed)**2) for q in full]
        np.testing.assert_allclose(costs, [key(q)[0] for q in expected], atol=1e-7, rtol=0)
    else:
        np.testing.assert_allclose(full, expected, atol=1e-5, rtol=0)
    for capacity in (1, 7, 32, count, count + 1):
        actual_count, best, actual = solve(command_lib, crx_10ia, target, seed, capacity)
        assert actual_count == min(count, capacity)
        np.testing.assert_allclose(actual, full[:capacity], atol=1e-9, rtol=0)
        _, best_only, _ = solve(command_lib, crx_10ia, target, seed, capacity, alternatives=False)
        assert best_only == best


def test_multiturn_seed_for_neighboring_target(command_lib, crx_10ia):
    witness = np.array([25., -35., 35., 15., 25., 550.])
    target = target_for_command(crx_10ia, witness)
    restrict(crx_10ia, witness)
    crx_10ia.data[30][5], crx_10ia.data[31][5] = -900., 900.
    seed = witness.copy()
    seed[5] += 2.
    count, best, full = solve(command_lib, crx_10ia, target, seed)
    assert count == 5
    np.testing.assert_allclose(best, witness, atol=1e-5, rtol=0)
    distances = [np.sum((np.array(q) - seed)**2) for q in full]
    assert distances == sorted(distances)


def test_limit_failures_and_work_limit_do_not_write(command_lib, crx_10ia):
    witness = np.array([25., -35., 35., 15., 25., 40.])
    target = target_for_command(crx_10ia, witness)
    restrict(crx_10ia, witness)
    crx_10ia.data[30][5], crx_10ia.data[31][5] = 41., 42.
    assert solve(command_lib, crx_10ia, target)[0] == 0
    for bound in (10000., 1e100):
        _set_row(crx_10ia, 30, [-bound] * 6)
        _set_row(crx_10ia, 31, [bound] * 6)
        assert solve(command_lib, crx_10ia, target, witness, capacity=1)[0] == -1
    assert solve(command_lib, crx_10ia, target, capacity=0)[0] == 0
