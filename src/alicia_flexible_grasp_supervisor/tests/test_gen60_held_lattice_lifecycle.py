"""GEN60 lifecycle contracts using frozen solver outputs, never live feedback.

Real FK is retained for screens. Predictor exception/success substitutions in
the lifecycle tests exercise ownership and error propagation only; they are
not physical reachability evidence. The separate lattice suite owns actual
recorded qualification and unchanged geometric acceptance tests.
"""
from copy import deepcopy
import hashlib
import json
import math
from pathlib import Path
from types import SimpleNamespace as NS
from unittest import mock

import numpy as np
import pytest
from tf.transformations import quaternion_from_matrix

from alicia_flexible_grasp.robot import held_pregrasp_selection as module
from alicia_flexible_grasp.robot.endpoint_correction import sdk_counts
from alicia_flexible_grasp.robot.held_axes_prediction import HeldAxesPredictionError
from alicia_flexible_grasp.robot.stationary_following import ARM_NAMES, SerialUrdfFk, SDK_QUANTUM_RAD as Q

FIXTURE_SHA = 'bdae6b6347322bbcc92c495f4e175f3da3860c1d9d559a5cf00960b83d041e97'
MODEL_SHA = '7812d6b69e854ed9f37b4e3cbd813ecd53dab3504065b2d04d287abc6fd852e0'
GUIDE = 'PREGRASP_HELD_AXES_GUIDE_DOES_NOT_REACH_FIXED_GOAL'


@pytest.fixture(scope='module')
def frozen():
    raw = (Path(__file__).parent/'fixtures/gen59_held_lattice_inputs.json').read_bytes()
    assert hashlib.sha256(raw).hexdigest() == FIXTURE_SHA
    data = json.loads(raw)
    root = Path(__file__).resolve().parents[3]
    xml = (root/'deployment/2026-10-08/runtime/candidate_models/expanded_metric_20260929/'
                'alicia_expanded_metric_candidate.urdf').read_text()
    assert hashlib.sha256(xml.encode()).hexdigest() == data['model_sha256'] == MODEL_SHA
    return xml, data['rows'][0]


def call(fk, case, **kwargs):
    options = dict(position_tolerance_m=case['position_tolerance_m'],
                   orientation_tolerance_rad=case['orientation_tolerance_rad'])
    options.update(kwargs)
    return module.select_held_pregrasp_endpoint(fk, case['sample'],
        case['pregrasp_goal'], case['approach_goal'], case['nominal_command_rad'], **options)


def install_frozen_solver(monkeypatch, case, state, faults=None):
    """Supply the stored three continuous outputs, without another solve."""
    def solver(fun, seed, **kwargs):
        index = state['solver_calls']
        state['solver_calls'] += 1
        state['trial_index'] = index
        row = case['original_trials'][index]
        result = np.asarray(row['solver_endpoint']).copy()
        if faults and index in faults:
            result[0] = faults[index]
        # The normalized caller uses radian units for the four guide variables.
        unit = kwargs['bounds'][6][1]/32.
        result[6:] *= unit
        return NS(x=result, success=row['solver_success'], nit=row['iterations'],
                  message=row['solver_message'])
    monkeypatch.setattr(module, 'minimize', solver)


def failure(code=GUIDE):
    return HeldAxesPredictionError(code, dict(code=code, proposed_delta_counts=[0]*6,
        scope='controlled_test_failure', execution_authorized=False))


@pytest.mark.parametrize('fault', ['model', 'joint_order', 'sdk_nan', 'actual_inf',
    'nominal_shape', 'following_too_large', 'position_tolerance', 'orientation_tolerance'])
def test_invalid_input_cannot_reach_solver_or_lattice(frozen, monkeypatch, fault):
    xml, original = frozen
    fk, case = SerialUrdfFk(xml, ARM_NAMES), deepcopy(original)
    options = {}
    if fault == 'model': case['sample']['model_sha256'] = 'unbound model'
    elif fault == 'joint_order': case['sample']['joint_names'] = list(reversed(ARM_NAMES))
    elif fault == 'sdk_nan': case['sample']['sdk_positions_rad'][0] = float('nan')
    elif fault == 'actual_inf': case['sample']['accepted_positions_rad'][2] = float('inf')
    elif fault == 'nominal_shape': case['nominal_command_rad'] = [0.]*5
    elif fault == 'following_too_large': case['sample']['sdk_positions_rad'][0] += .1
    elif fault == 'position_tolerance': options['position_tolerance_m'] = .020001
    elif fault == 'orientation_tolerance': options['orientation_tolerance_rad'] = math.radians(10)+1e-6
    blocked = mock.Mock(side_effect=AssertionError('invalid input entered recovery'))
    for name in ('minimize', 'product', 'predict_held_axes_endpoint'):
        monkeypatch.setattr(module, name, blocked)
    with pytest.raises(ValueError): call(fk, case, **options)
    blocked.assert_not_called()


def test_qualified_original_keeps_source_identity_and_never_enters_fallback(frozen, monkeypatch):
    xml, original = frozen
    fk, case = SerialUrdfFk(xml, ARM_NAMES), deepcopy(original)
    nominal = (sdk_counts(case['nominal_command_rad'])-2048)*Q
    bias = np.asarray(case['sample']['sdk_positions_rad'])-case['sample']['accepted_positions_rad']
    target = fk(nominal-bias)
    goal = np.r_[target[:3, 3], quaternion_from_matrix(target)].tolist()
    case['pregrasp_goal'] = goal
    case['approach_goal'] = list(goal)
    before = deepcopy(case)
    blocked = mock.Mock(side_effect=AssertionError('qualified original entered search'))
    monkeypatch.setattr(module, 'minimize', blocked)
    monkeypatch.setattr(module, 'product', blocked)
    checks = mock.Mock()
    command, report = call(fk, case, deadline_check=checks)
    np.testing.assert_array_equal(command, nominal)
    assert report['reselected'] is False and report['success'] is True
    assert 'quantized_neighborhood' not in report and 'trials' not in report
    assert checks.call_count == 2
    blocked.assert_not_called()
    for source, report_key in [('sdk_stamp_ns', 'source_sdk_stamp_ns'),
                               ('accepted_stamp_ns', 'source_accepted_stamp_ns'), ('epoch_ns', 'epoch_ns')]:
        assert report[report_key] == case['sample'][source]
    for key in ('execution_authorized', 'trajectory_checked', 'future_following_verified',
                'hardware_response_verified', 'globally_optimal'):
        assert report[key] is False
    assert case == before


@pytest.mark.parametrize('code', ['PREGRASP_OUTSIDE_LOCAL_CAPTURE_RANGE',
    'PREGRASP_HELD_AXES_GUIDE_INVALID', 'PREGRASP_FOLLOWING_OUTSIDE_CONTRACT'])
def test_original_nonrecoverable_error_propagates_without_any_solver(frozen, monkeypatch, code):
    xml, case = frozen
    error = failure(code)
    monkeypatch.setattr(module, 'predict_held_axes_endpoint', mock.Mock(side_effect=error))
    blocked = mock.Mock(side_effect=AssertionError('nonrecoverable error searched'))
    monkeypatch.setattr(module, 'minimize', blocked)
    monkeypatch.setattr(module, 'product', blocked)
    with pytest.raises(HeldAxesPredictionError) as caught:
        call(SerialUrdfFk(xml, ARM_NAMES), case)
    assert caught.value is error
    assert 'pregrasp_endpoint_selection' not in error.evidence
    blocked.assert_not_called()


@pytest.mark.parametrize('trial_index', [0, 1, 2])
@pytest.mark.parametrize('code', ['PREGRASP_OUTSIDE_LOCAL_CAPTURE_RANGE',
    'PREGRASP_HELD_AXES_GUIDE_INVALID', 'PREGRASP_FOLLOWING_OUTSIDE_CONTRACT'])
def test_one_different_trial_error_preserves_original_failure_without_lattice(
        frozen, monkeypatch, trial_index, code):
    xml, case = frozen
    state = dict(solver_calls=0, trial_index=None)
    install_frozen_solver(monkeypatch, case, state)
    original = failure(); before = deepcopy(original.evidence)
    def predictor(*args, **kwargs):
        if state['trial_index'] is None: raise original
        raise failure(code if state['trial_index'] == trial_index else GUIDE)
    monkeypatch.setattr(module, 'predict_held_axes_endpoint', predictor)
    blocked = mock.Mock(side_effect=AssertionError('mixed rejection entered lattice'))
    monkeypatch.setattr(module, 'product', blocked)
    with pytest.raises(HeldAxesPredictionError) as caught:
        call(SerialUrdfFk(xml, ARM_NAMES), case)
    assert caught.value is original and state['solver_calls'] == 3
    report = original.evidence['pregrasp_endpoint_selection']
    assert report['original_failure']['evidence'] == before
    assert [r['rejection'] for r in report['trials']] == [
        code if i == trial_index else GUIDE for i in range(3)]
    assert 'quantized_neighborhood' not in report
    json.dumps(original.evidence, allow_nan=False)  # No parent/back-reference cycle.
    blocked.assert_not_called()


@pytest.mark.parametrize('trial_index', [0, 1, 2])
def test_nonfinite_trial_is_not_reclassified_as_recoverable(frozen, monkeypatch, trial_index):
    xml, case = frozen
    state = dict(solver_calls=0, trial_index=None)
    install_frozen_solver(monkeypatch, case, state, {trial_index: float('nan')})
    original = failure()
    def predictor(*args, **kwargs):
        if state['trial_index'] is None: raise original
        raise failure()
    monkeypatch.setattr(module, 'predict_held_axes_endpoint', predictor)
    blocked = mock.Mock(side_effect=AssertionError('nonfinite output entered lattice'))
    monkeypatch.setattr(module, 'product', blocked)
    with pytest.raises(HeldAxesPredictionError) as caught:
        call(SerialUrdfFk(xml, ARM_NAMES), case)
    assert caught.value is original
    report = original.evidence['pregrasp_endpoint_selection']
    assert report['trials'][trial_index]['rejection'] == 'NONFINITE_SOLVER_RESULT'
    assert 'quantized_neighborhood' not in report
    blocked.assert_not_called()


class OwnershipExpired(ValueError):
    pass


@pytest.mark.parametrize('where', ['before_lattice', 'before_screen', 'after_screen_fk',
    'before_full_verify', 'after_rejected_predictor', 'after_qualified_predictor'])
def test_deadline_and_ownership_expiry_always_escape_recovery(frozen, monkeypatch, where):
    xml, case = frozen
    state = dict(solver_calls=0, trial_index=None, predictor_calls=0, products=0,
                 completed_products=0, phase='original', expired=False, screen_fk=0)
    install_frozen_solver(monkeypatch, case, state)
    error = OwnershipExpired('deadline / ownership revoked at '+where)
    real_product = module.product
    def product(*args, **kwargs):
        state['products'] += 1
        state['phase'] = 'screen'
        if where == 'before_screen': state['expired'] = True
        for item in real_product(*args, **kwargs): yield item
        state['completed_products'] += 1
        if state['completed_products'] == 3:
            state['phase'] = 'full_verify'
            if where == 'before_full_verify': state['expired'] = True
    monkeypatch.setattr(module, 'product', product)
    class ObservableFk(SerialUrdfFk):
        def __call__(self, q):
            answer = super().__call__(q)
            if state['phase'] == 'screen':
                state['screen_fk'] += 1
                if where == 'after_screen_fk': state['expired'] = True
            return answer
    def success(sdk, actual):
        return np.asarray(sdk_counts(sdk)), dict(predicted_command_positions_rad=list(sdk),
            predicted_positions_rad=list(actual), predicted_position_error_m=0.,
            predicted_orientation_error_rad=0., execution_authorized=False,
            hardware_response_verified=False, future_following_verified=False)
    def predictor(fk, sdk, actual, goal, **kwargs):
        state['predictor_calls'] += 1
        if state['predictor_calls'] <= 4:
            if state['predictor_calls'] == 4 and where == 'before_lattice': state['expired'] = True
            raise failure()
        if where == 'after_rejected_predictor':
            state['expired'] = True
            raise failure()
        if where == 'after_qualified_predictor':
            if state['predictor_calls'] == 6: state['expired'] = True
            return success(sdk, actual)
        raise AssertionError('deadline should have fired before full predictor')
    monkeypatch.setattr(module, 'predict_held_axes_endpoint', predictor)
    def deadline():
        if state['expired']: raise error
    with pytest.raises(OwnershipExpired) as caught:
        call(ObservableFk(xml, ARM_NAMES), case, deadline_check=deadline)
    assert caught.value is error
    assert state['solver_calls'] == 3
    if where == 'before_lattice': assert state['products'] == 0
    elif where == 'before_screen': assert state['screen_fk'] == 0
    elif where == 'after_screen_fk': assert state['screen_fk'] > 0
    elif where == 'before_full_verify': assert state['predictor_calls'] == 4
    elif where == 'after_rejected_predictor': assert state['predictor_calls'] == 5
    elif where == 'after_qualified_predictor': assert state['predictor_calls'] == 6


@pytest.mark.parametrize('error_type', [ValueError, RuntimeError])
def test_non_predictor_failure_during_lattice_verify_is_not_a_rejected_trial(
        frozen, monkeypatch, error_type):
    xml, case = frozen
    state = dict(solver_calls=0, trial_index=None, calls=0)
    install_frozen_solver(monkeypatch, case, state)
    error = error_type('source/model owner failed during final verification')
    def predictor(*args, **kwargs):
        state['calls'] += 1
        if state['calls'] <= 4: raise failure()
        raise error
    monkeypatch.setattr(module, 'predict_held_axes_endpoint', predictor)
    with pytest.raises(error_type) as caught:
        call(SerialUrdfFk(xml, ARM_NAMES), case)
    assert caught.value is error and state['calls'] == 5
