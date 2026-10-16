"""GEN11 held-guide counterexample; nominal IK is an offline reconstruction."""
from copy import deepcopy
import json
import math
from pathlib import Path

import numpy as np

from alicia_flexible_grasp.robot.held_axes_prediction import predict_held_axes_endpoint
from alicia_flexible_grasp.robot.held_pregrasp_selection import select_held_pregrasp_endpoint
from alicia_flexible_grasp.robot.stationary_following import SerialUrdfFk, ARM_NAMES, SDK_QUANTUM_RAD as Q


def recorded():
    data = json.loads((Path(__file__).parent/'fixtures/held_guide_gen11_reconstructed.json').read_text())
    fk = SerialUrdfFk((Path(__file__).resolve().parents[3]/data['model_path']).read_text(), ARM_NAMES)
    assert fk.model_sha256 == data['model_sha256']
    return data, fk


def test_gen11_pairwise_guide_qualifies_in_original_hard_contract():
    data, fk = recorded(); old = data['failed_approach_prediction']
    before = deepcopy(old)
    sdk = (np.asarray(old['initial_sdk_counts'])-2048)*Q
    counts, report = predict_held_axes_endpoint(fk, sdk, old['virtual_start_positions_rad'],
        old['fixed_goal_pose'], position_tolerance_m=.006, orientation_tolerance_rad=math.radians(5))
    assert report['success'] and report['predicted_position_error_m'] <= .006
    assert report['predicted_orientation_error_rad'] <= math.radians(5)
    delta = counts-old['initial_sdk_counts']
    assert delta[1] == delta[4] == 0 and max(abs(delta)) <= 32
    assert report['execution_authorized'] is False and old == before


def test_gen11_real_selector_produces_quantized_valid_pregrasp_and_approach():
    data, fk = recorded(); before = deepcopy(data)
    command, report = select_held_pregrasp_endpoint(fk, data['source_sample'], *data['fixed_poses'],
        data['reconstructed_nominal_rad'], position_tolerance_m=.010,
        orientation_tolerance_rad=math.radians(5))
    for key, tolerance in [('predicted_pregrasp', .010), ('predicted_approach', .006)]:
        assert report[key]['predicted_position_error_m'] <= tolerance
        assert report[key]['predicted_orientation_error_rad'] <= math.radians(5)
        assert max(abs(np.asarray(report[key]['proposed_delta_counts']))) <= 32
        assert report[key]['proposed_delta_counts'][1] == report[key]['proposed_delta_counts'][4] == 0
    np.testing.assert_allclose(command/Q, np.rint(command/Q), atol=1e-12)
    assert report['reselected'] and report['execution_authorized'] is False
    assert len(report['trials']) == 3
    assert [t['normalized_fallback'] for t in report['trials']] == [False, False, True]
    assert data == before


import pytest
from types import SimpleNamespace as NS
from alicia_flexible_grasp.robot import held_pregrasp_selection as selector_module
from alicia_flexible_grasp.robot.pregrasp_compensation import remaining_budget_guide


@pytest.mark.parametrize('fault', ['none', 'position', 'angle', 'nonfinite_angle', 'three_axes',
                                  'spent_steps', 'spent_counts', 'held', 'sdk', 'urdf', 'cost'])
def test_joint_neighbourhood_is_bounded_and_retains_every_original_constraint(fault):
    sdk = np.full(6, 2048)
    if fault == 'sdk': sdk[0] = 4095
    measured = (sdk-2048)*Q
    calls = dict(fk=0, residual=0)
    def fk(command):
        calls['fk'] += 1
        if fault == 'urdf' and command[0] > measured[0]:
            raise ValueError('command outside synthetic URDF joint limit')
    def residual(q):
        calls['residual'] += 1
        delta = np.rint((np.asarray(q)-measured)/Q).astype(int)
        at_pair = delta[0] == delta[2] == 1
        if fault == 'three_axes': at_pair = at_pair and delta[3] == 1
        if at_pair:
            return (.0061 if fault == 'position' else .0055,
                    float('nan') if fault == 'nonfinite_angle' else
                    math.radians(5.01 if fault == 'angle' else 4.))
        return .007, math.radians(4.)
    guide, error = remaining_budget_guide(fk, residual, measured, sdk, sdk,
        maximum_steps=12, steps=12 if fault == 'spent_steps' else 0, maximum_step_counts=4,
        maximum_total_counts=0 if fault == 'spent_counts' else 32,
        held_axes=(0, 1, 4) if fault == 'held' else (1, 4), response_gains=np.ones(6),
        tolerances=(.006, math.radians(5)), cost=(lambda q: 100.) if fault == 'cost' else None)
    assert guide[1] == guide[4] == 0 and max(abs(guide)) <= 32
    assert calls['fk'] <= 4*33+96 and calls['residual'] <= 1+4*33+96
    if fault == 'none':
        assert guide.tolist() == [1, 0, 1, 0, 0, 0] and error == .0055
    else:
        assert not np.any(guide) and error == .007
    if fault == 'cost':
        assert calls['fk'] == 4*33  # No new search in the separate contact merit policy.


def test_already_qualified_original_guide_has_no_neighbourhood_calls():
    calls = []
    def fk(q): calls.append(q)
    residual = lambda q: (.005 if not np.any(q) else .0055, .02)
    sdk = np.full(6, 2048)
    guide, error = remaining_budget_guide(fk, residual, np.zeros(6), sdk, sdk,
        maximum_steps=12, steps=0, maximum_step_counts=4, maximum_total_counts=32,
        held_axes=(1, 4), response_gains=np.ones(6), tolerances=(.006, math.radians(5)))
    assert not np.any(guide) and error == .005
    assert len(calls) == 4*33


@pytest.mark.parametrize('where', ['objective', 'constraints', 'post_quantization'])
def test_normalized_fallback_propagates_deadline_without_more_search(monkeypatch, where):
    from test_held_pregrasp_selection import recorded_case, select
    fk, case = recorded_case()
    calls = []; expired = [False]
    def deadline():
        if expired[0]: raise RuntimeError('absolute candidate deadline consumed')
    def optimizer(fun, seed, **kwargs):
        calls.append(seed.copy())
        if len(calls) == 3:
            expired[0] = True
            if where == 'objective': fun(seed)
            elif where == 'constraints': kwargs['constraints'][0]['fun'](seed)
        return NS(success=False, message='unchanged seed', nit=1, x=seed.copy())
    monkeypatch.setattr(selector_module, 'minimize', optimizer)
    with pytest.raises(RuntimeError, match='absolute candidate deadline'):
        select(fk, case, deadline_check=deadline)
    assert len(calls) == 3
