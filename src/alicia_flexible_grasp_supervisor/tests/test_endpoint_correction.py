from copy import deepcopy
import math

import numpy as np
import pytest

from test_stationary_following import XML, NAMES, message
from alicia_flexible_grasp.robot.stationary_following import (
    SerialUrdfFk, SDK_QUANTUM_RAD as Q, stationary_following_error,
)
from alicia_flexible_grasp.robot.endpoint_correction import EndpointCorrection, sdk_counts


def sample(fk, sdk, actual, at=10.):
    times = np.linspace(at-.45, at-.05, 5)
    return stationary_following_error(
        fk, [message(t, 'sdk_transmitted', sdk) for t in times],
        [message(t+.001, 'sdk_measured', actual, True) for t in times],
        now_sec=at, epoch_ns=1_000_000_000)


def fixture():
    fk = SerialUrdfFk(XML.replace('0.05 0 0', '0.10 0 0'), NAMES)
    goal = np.zeros(6)
    bias = np.array([0, -7, -10, 0, -8, 0]) * Q
    initial = sample(fk, goal, goal+bias)
    return fk, goal, bias, initial


def test_fixed_bias_plant_converges_without_moving_goal_or_erasing_sdk_gap():
    fk, goal, bias, initial = fixture()
    correction = EndpointCorrection(fk, initial)
    current = initial
    for index in range(6):
        code, step, evidence = correction.evaluate(current)
        if step is None:
            break
        assert set(np.flatnonzero(np.array(step.target_counts)-step.baseline_counts)) == {1,2,4}
        assert np.max(np.abs(np.array(step.target_counts)-step.baseline_counts)) <= 4
        correction.committed(step, 11.+index*3)
        current = sample(fk, step.positions, np.asarray(step.positions)+bias, 12.+index*3)
    assert code == 'MEASURED_ENDPOINT_CONVERGED'
    assert evidence['fixed_goal_position_error_m'] <= .006
    assert evidence['sdk_following_error_m'] > .006
    assert evidence['fixed_goal_counts'] == [2048]*6
    assert not evidence['grasp_or_calibration_success']
    assert np.array_equal(correction.goal, goal)


@pytest.mark.parametrize('response', [[0,0,0,0,0,0], [0,3,1,0,0,0],
                                      [0,-2,-2,0,-2,0], [3,4,4,0,4,0]])
def test_stall_partial_reverse_and_uncommanded_response_cannot_integrate(response):
    fk, goal, bias, initial = fixture()
    correction = EndpointCorrection(fk, initial)
    _, step, _ = correction.evaluate(initial)
    correction.committed(step, 11.)
    after = sample(fk, step.positions, goal+bias+np.array(response)*Q, 12.)
    for _ in range(2):
        with pytest.raises(ValueError, match='NO_BOUNDED_DIRECTIONAL_RESPONSE'):
            correction.evaluate(after)
    assert correction.steps == 1
    assert correction.expected == step.target_counts  # no implied rollback
    evidence = correction.last_evidence
    assert evidence['current_sdk_counts'] == list(step.target_counts)
    assert evidence['measured_counts'] == sdk_counts(after['accepted_positions_rad']).tolist()
    assert evidence['measurement_stamp_sec'] == after['stamp_sec']
    assert evidence['last_step_response']['measured_delta_counts'] == response
    assert evidence['last_step_response']['bounded_directional_response'] is False
    assert evidence['sdk_following_error_m'] == after['position_error_m']
    assert evidence['fixed_goal_position_error_m'] == pytest.approx(float(np.linalg.norm(
        fk(goal)[:3, 3] - fk(after['accepted_positions_rad'])[:3, 3])))


def test_invalid_sample_cannot_replace_last_valid_measurement():
    fk, _, _, initial = fixture()
    correction = EndpointCorrection(fk, initial)
    correction.evaluate(initial)
    evidence = deepcopy(correction.last_evidence)
    wrong = deepcopy(initial)
    wrong['epoch_ns'] += 1
    with pytest.raises(ValueError, match='CONTEXT_CHANGED'):
        correction.evaluate(wrong)
    assert correction.last_evidence == evidence


def test_small_geometric_error_does_not_override_failed_axis_response():
    fk, goal, bias, initial = fixture()
    correction = EndpointCorrection(fk, initial)
    _, step, _ = correction.evaluate(initial)
    correction.committed(step, 11.)
    # Two joints respond but Joint5 does not. Geometry can nevertheless fall
    # within tolerance; a good scalar residual must not authorize another step.
    after = sample(fk, step.positions, goal+bias+np.array([0,4,4,0,0,0])*Q, 12.)
    with pytest.raises(ValueError, match='NO_BOUNDED_DIRECTIONAL_RESPONSE'):
        correction.evaluate(after)
    assert correction.last_evidence['fixed_goal_position_error_m'] <= .006
    assert not correction.last_evidence['last_step_response']['bounded_directional_response']
    assert correction.steps == 1


@pytest.mark.parametrize('field,value', [
    ('epoch_ns',2_000_000_000), ('model_sha256','other'),
    ('sdk_positions_rad',[Q]*6), ('stamp_sec',float('nan')),
    ('stamp_sec',41.), ('sdk_stamp_ns',9_000_000_000),
    ('accepted_stamp_ns',10_100_000_000), ('stationary_window_start_ns',9_990_000_000)])
def test_bad_or_changed_evidence_cannot_propose_motion(field,value):
    fk, _, _, initial = fixture()
    correction = EndpointCorrection(fk, initial)
    wrong = deepcopy(initial)
    wrong[field] = value
    with pytest.raises(ValueError): correction.evaluate(wrong)


def test_repeat_evaluation_is_not_another_command_and_commit_is_exactly_once():
    fk, _, _, initial = fixture()
    correction = EndpointCorrection(fk, initial)
    _, first, _ = correction.evaluate(initial)
    _, second, _ = correction.evaluate(initial)
    assert first == second and correction.steps == 0
    correction.committed(first, 11.)
    with pytest.raises(ValueError): correction.committed(first, 11.)


def test_response_must_be_entirely_after_completed_command():
    fk, _, bias, initial = fixture()
    correction = EndpointCorrection(fk, initial)
    _, step, _ = correction.evaluate(initial)
    correction.committed(step, 11.)
    with pytest.raises(ValueError, match='NOT_POST_COMMAND'):
        correction.evaluate(sample(fk,step.positions,np.asarray(step.positions)+bias,11.1))


@pytest.mark.parametrize('position,orientation', [(.007,.01),(.006,.1),(0,.01),(.006,float('nan'))])
def test_contact_tolerances_cannot_be_relaxed(position,orientation):
    fk, _, _, initial = fixture()
    with pytest.raises(ValueError):
        EndpointCorrection(fk,initial,position_tolerance_m=position,
                           orientation_tolerance_rad=orientation)


def test_integer_codec_round_trips_every_physical_count():
    for word in range(4096):
        assert tuple(sdk_counts([(word-2048)*Q]*6)) == (word,)*6


def test_slow_but_directional_response_cannot_exceed_total_offset_budget():
    fk, goal, bias, initial = fixture()
    correction = EndpointCorrection(fk, initial, position_tolerance_m=.0001)
    current = initial
    for index in range(3):
        _, step, _ = correction.evaluate(current)
        assert max(abs(a-b) for a,b in zip(step.target_counts, correction.goal_counts)) <= 12
        correction.committed(step, 11.+index*3)
        delta = np.asarray(step.target_counts) - step.baseline_counts
        measured = np.asarray(current['accepted_positions_rad']) + np.sign(delta)*2*Q
        current = sample(fk, step.positions, measured, 12.+index*3)
    with pytest.raises(ValueError, match='TOTAL_BUDGET'):
        correction.evaluate(current)
    assert correction.steps == 3


def test_quantization_floor_does_not_create_blind_one_count_integral_steps():
    fk, goal, _, _ = fixture()
    current = sample(fk, goal, np.array([0,-2,-2,0,-2,0])*Q)
    correction = EndpointCorrection(fk, current, position_tolerance_m=.0001)
    with pytest.raises(ValueError, match='QUANTIZATION_FLOOR'):
        correction.evaluate(current)
    assert correction.steps == 0


def test_step_budget_is_independent_of_joint_offset_budget():
    fk, _, _, initial = fixture()
    correction = EndpointCorrection(fk, initial)
    correction.steps = 6
    with pytest.raises(ValueError, match='STEP_BUDGET'):
        correction.evaluate(initial)


def test_wire_limit_cannot_be_crossed_to_correct_an_endpoint():
    fk = SerialUrdfFk(XML.replace('upper="3.14"', 'upper="3.141592653589793"'), NAMES)
    goal = np.array([0., 2047*Q, 0., 0., 0., 0.])
    initial = sample(fk, goal, goal-np.array([0,10,0,0,0,0])*Q)
    correction = EndpointCorrection(fk, initial, position_tolerance_m=.0001)
    with pytest.raises(ValueError, match='TOTAL_BUDGET'):
        correction.evaluate(initial)


def test_completion_after_episode_deadline_cannot_authorize_a_new_step():
    fk, _, _, initial = fixture()
    correction = EndpointCorrection(fk, initial)
    _, step, _ = correction.evaluate(initial)
    with pytest.raises(ValueError, match='COMMIT_INVALID'):
        correction.committed(step, 40.001)
    assert correction.steps == 0
