"""Independent exact numerical, scope and final-admission checks; no ROS I/O."""
from copy import deepcopy
from dataclasses import replace
from pathlib import Path
import ast
import inspect
import math
from types import SimpleNamespace

import numpy as np
import pytest

from alicia_flexible_grasp.grasp.gripper_geometry import GripperGeometry
from alicia_flexible_grasp.robot.observation_path_guard import (
    SerialUrdfFk, ObservationPathError, observation_endpoint_support_bound)
from alicia_grasp_modes.observation_endpoint_precompute import PreparedEndpointSupportBound
from alicia_grasp_modes import reachable_observation_views as module
from test_reachable_observation_exposure_tiers import synthetic


@pytest.fixture
def contract():
    work = Path.cwd()
    xml = (work/'src/real-arm/alicia_d_descriptions/urdf/alicia_duo_with_gripper.urdf').read_text()
    fk = SerialUrdfFk(xml, ['Joint%d' % i for i in range(1, 7)])
    gripper = GripperGeometry(.05, .002, np.array([.0434, .0286, .06]),
                              np.array([.1175, .155, .0774]), .003)
    return fk, np.array([.01, .02, .03, .04, .05, .06]), np.array([0., 0., 1.]), 0., gripper, .05


def original(contract, q):
    fk, errors, normal, offset, gripper, opening = contract
    return observation_endpoint_support_bound(fk, q, errors, normal, offset, gripper, opening)


def exact(a, b):
    assert a == b
    # Include the sign bit and binary float representation, rather than an epsilon comparison.
    for key in a:
        if isinstance(a[key], float):
            assert a[key].hex() == b[key].hex(), key


@pytest.mark.parametrize('opening', [0., .025, .05])
@pytest.mark.parametrize('error', [0., .01, .5, math.pi])
@pytest.mark.parametrize('normal', [[0., 0., 1.], [.6, 0., .8], [0., 0., 1.0000005]])
def test_all_fields_exact_for_extremes_and_random_joint_configurations(contract, opening, error, normal):
    fk, _, _, _, gripper, _ = contract
    contract = fk, np.full(6, error), np.array(normal), -.0123, gripper, opening
    bound = PreparedEndpointSupportBound(*contract)
    rng = np.random.RandomState(731)
    qs = [fk.limits[:, 0], fk.limits[:, 1], fk.limits.mean(axis=1)]
    qs += [rng.uniform(fk.limits[:, 0], fk.limits[:, 1]) for _ in range(9)]
    for q in qs:
        exact(bound(q), original(contract, q))


@pytest.mark.parametrize('delta', [-1e-9, -1e-12, 0., 1e-12, 1e-9])
def test_three_mm_gate_boundary_preserves_original_comparison(contract, delta):
    fk, errors, normal, _, gripper, opening = contract
    q = fk.limits.mean(axis=1)
    low = original(contract, q)['minimum_support_clearance_lower_bound_m']
    contract = fk, errors, normal, .003-low+delta, gripper, opening
    a = original(contract, q); b = PreparedEndpointSupportBound(*contract)(q)
    exact(a, b)
    if delta < 0.: assert not b['ok']
    if delta > 0.: assert b['ok']


@pytest.mark.parametrize('fault', ['errors_negative', 'errors_large', 'errors_nan', 'errors_shape',
    'normal_zero', 'normal_not_unit', 'normal_nan', 'offset_nan', 'offset_inf',
    'opening_negative', 'opening_large', 'opening_nan', 'clearance_below_three_mm'])
def test_invalid_contract_cannot_prepare(contract, fault):
    fk, errors, normal, offset, gripper, opening = contract
    if fault == 'errors_negative': errors[0] = -1e-15
    elif fault == 'errors_large': errors[0] = math.pi+1e-12
    elif fault == 'errors_nan': errors[0] = float('nan')
    elif fault == 'errors_shape': errors = [0.]
    elif fault == 'normal_zero': normal[:] = 0.
    elif fault == 'normal_not_unit': normal *= 1.000002
    elif fault == 'normal_nan': normal[0] = float('nan')
    elif fault == 'offset_nan': offset = float('nan')
    elif fault == 'offset_inf': offset = float('inf')
    elif fault == 'opening_negative': opening = -1e-15
    elif fault == 'opening_large': opening = .0500000001
    elif fault == 'opening_nan': opening = float('nan')
    elif fault == 'clearance_below_three_mm': gripper = replace(gripper, support_clearance_m=.002999)
    changed = fk, errors, normal, offset, gripper, opening
    with pytest.raises(ObservationPathError): original(changed, fk.limits.mean(axis=1))
    with pytest.raises(ObservationPathError): PreparedEndpointSupportBound(*changed)


@pytest.mark.parametrize('fault', ['lower', 'upper', 'nan', 'infinity', 'shape'])
def test_proposed_q_always_revalidates_actual_joint_limits(contract, fault):
    q = contract[0].limits.mean(axis=1)
    if fault == 'lower': q[0] = contract[0].limits[0, 0]-1e-10
    elif fault == 'upper': q[0] = contract[0].limits[0, 1]+1e-10
    elif fault == 'nan': q[0] = float('nan')
    elif fault == 'infinity': q[0] = float('inf')
    else: q = [0.]
    bound = PreparedEndpointSupportBound(*contract)
    with pytest.raises(ObservationPathError): original(contract, q)
    with pytest.raises(ObservationPathError): bound(q)


def test_snapshot_owns_model_cad_and_plane_and_new_request_refreshes(contract):
    bound = PreparedEndpointSupportBound(*contract)
    fk, errors, normal, offset, gripper, opening = contract
    q = fk.limits.mean(axis=1); expected = original(contract, q)
    errors[:] = .1; normal[:] = [.6, 0., .8]
    fk.chain[0][0][0, 3] += .1
    fk.model_sha256 = 'f'*64
    gripper.finger_size_xyz_m.setflags(write=True); gripper.finger_size_xyz_m[0] += .01
    exact(bound(q), expected)
    changed = PreparedEndpointSupportBound(*contract)(q)
    exact(changed, original(contract, q))
    assert changed != expected
    # Returned collections are fresh evidence and cannot mutate later receipts.
    returned = bound(q); returned['joint_names'][0] = 'tampered'
    returned['goal_joint_positions_rad'][0] += .1
    exact(bound(q), expected)


def test_complete_admission_does_not_trust_prepared_proposal_receipt(synthetic, monkeypatch):
    args, observed, _ = synthetic
    prepared_calls = []; final_calls = []
    def prepare(*_args):
        prepared_calls.append(True)
        return lambda q: {'ok': True, 'minimum_support_clearance_lower_bound_m': 99.}
    def original_reject(*args):
        final_calls.append(args[1].copy())
        return {'ok': False, 'minimum_support_clearance_lower_bound_m': -.01}
    monkeypatch.setattr(module, 'PreparedEndpointSupportBound', prepare)
    monkeypatch.setattr(module, 'observation_endpoint_support_bound', original_reject)
    rows, report = module.generate_reachable_observation_views(**args)
    assert prepared_calls == [True] and len(final_calls) == 4
    assert not rows and report['candidate_count'] == 0
    assert all(not row['accepted'] for row in report['directions'])


def test_preparation_consumes_original_budget_before_any_solver(synthetic, monkeypatch):
    args, observed, _ = synthetic
    now = [0.]; calls = []
    def prepare(*_args):
        calls.append(True); now[0] = 2.
        return lambda q: {'ok': True}
    monkeypatch.setattr(module, 'PreparedEndpointSupportBound', prepare)
    rows, report = module.generate_reachable_observation_views(**dict(args, clock=lambda: now[0]))
    assert calls == [True] and not observed['calls'] and not rows
    assert report['status'] == 'budget_exhausted' and report['elapsed_sec'] == 2.


def test_each_generator_request_constructs_its_own_snapshot(synthetic, monkeypatch):
    args, _, _ = synthetic; calls = []
    real = module.PreparedEndpointSupportBound
    def prepare(*a):
        result = real(*a); calls.append(result); return result
    monkeypatch.setattr(module, 'PreparedEndpointSupportBound', prepare)
    for _ in range(2):
        rows, _report = module.generate_reachable_observation_views(**args)
        assert len(rows) == 4
    assert len(calls) == 2 and calls[0] is not calls[1]


def test_final_original_gate_and_constraint_only_prepared_scope():
    tree = ast.parse(inspect.getsource(module.generate_reachable_observation_views))
    funcs = {node.name: node for node in ast.walk(tree) if isinstance(node, ast.FunctionDef)}
    names = lambda node: [n.func.id for n in ast.walk(node)
                          if isinstance(n, ast.Call) and isinstance(n.func, ast.Name)]
    assert 'observation_endpoint_support_bound' in names(funcs['complete_endpoint'])
    assert 'prepared_support_bound' not in names(funcs['complete_endpoint'])
    assert 'prepared_support_bound' in names(funcs['inequalities'])
    assert 'observation_endpoint_support_bound' not in names(funcs['inequalities'])
