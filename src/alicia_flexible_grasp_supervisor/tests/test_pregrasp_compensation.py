from copy import deepcopy
import json
import math
from pathlib import Path

import numpy as np
import pytest

from test_endpoint_correction import sample
from alicia_flexible_grasp.robot.pregrasp_compensation import PregraspCompensation, pose_matrix
from alicia_flexible_grasp.robot.stationary_following import ARM_NAMES, SerialUrdfFk, SDK_QUANTUM_RAD as Q


def recorded_fixture():
    record = json.loads((Path(__file__).parent/'fixtures/pregrasp_following_20260919.json').read_text())
    fk = SerialUrdfFk(record['robot_description'], ARM_NAMES)
    assert fk.model_sha256 == record['model_sha256']
    sdk = (np.array(record['initial_sdk_counts'])-2048)*Q
    actual = (np.array(record['initial_measured_counts'])-2048)*Q
    goal = record['goal_position_m']+record['goal_quaternion_xyzw']
    return fk,sdk,actual,goal


def test_recorded_pretrim_pose_converges_under_explicit_other_axes_response_hypothesis():
    fk,sdk,actual,goal = recorded_fixture()
    initial = sample(fk,sdk,actual)
    correction = PregraspCompensation(fk,initial,goal)
    original_goal = correction.goal.copy()
    held = sdk[1]
    for i in range(13):
        code,step,evidence = correction.evaluate(sample(fk,sdk,actual,10+i*3))
        if step is None:
            break
        delta = np.array(step.target_counts)-step.baseline_counts
        assert delta[1] == 0 and max(abs(delta)) <= 4
        assert max(abs(np.array(step.target_counts)-correction.initial_counts)) <= 32
        correction.committed(step,11+i*3)
        sdk = np.array(step.positions)
        actual += delta*Q  # hypothetical responsive plant, not a physical replay
        assert sdk[1] == held
    assert code == 'PREGRASP_MEASURED_CONVERGED'
    assert evidence['position_error_m'] == pytest.approx(.00566169967,abs=1e-8)
    assert evidence['orientation_error_rad'] < math.radians(5)
    assert correction.steps == 8
    assert evidence['real_grasp_success'] is False
    assert np.array_equal(original_goal,correction.goal)
    assert not np.array_equal(sdk,actual)  # never pretend the encoder equals the command


@pytest.mark.parametrize('kind',['stalled','partial','reverse','overshoot','held_joint_moves'])
def test_response_failure_stops_without_another_step_or_rollback(kind):
    fk,sdk,actual,goal = recorded_fixture()
    c = PregraspCompensation(fk,sample(fk,sdk,actual),goal)
    _,step,_ = c.evaluate(sample(fk,sdk,actual))
    delta = np.array(step.target_counts)-step.baseline_counts
    response = delta.copy()
    if kind == 'stalled': response[:] = 0
    if kind == 'partial': response[2] = 0
    if kind == 'reverse': response = -response
    if kind == 'overshoot': response = response*2
    if kind == 'held_joint_moves': response[1] = 3
    c.committed(step,11.)
    for _ in range(2):
        with pytest.raises(ValueError,match=('FOLLOWING_OUTSIDE_CONTRACT' if kind == 'reverse'
                                             else 'NO_BOUNDED_DIRECTIONAL_RESPONSE')):
            c.evaluate(sample(fk,step.positions,actual+response*Q,12.))
    assert c.steps == 1 and c.expected == step.target_counts
    if kind != 'reverse':
        assert c.last_evidence['last_step_response']['measured_delta_counts'] == response.tolist()


@pytest.mark.parametrize('field,value',[
    ('epoch_ns',2_000_000_000),('model_sha256','changed'),('stamp_sec',float('nan')),
    ('stamp_sec',71.),('sdk_stamp_ns',1),('accepted_stamp_ns',11_000_000_000),
    ('sdk_positions_rad',[0.]*6),('stationary_window_start_ns',9_990_000_000)])
def test_changed_or_stale_sample_cannot_authorize_step(field,value):
    fk,sdk,actual,goal = recorded_fixture()
    initial=sample(fk,sdk,actual); c=PregraspCompensation(fk,initial,goal)
    wrong=deepcopy(initial);wrong[field]=value
    with pytest.raises(ValueError):c.evaluate(wrong)
    assert c.steps == 0


def test_feedback_window_must_follow_entire_completed_command():
    fk,sdk,actual,goal = recorded_fixture();c=PregraspCompensation(fk,sample(fk,sdk,actual),goal)
    _,step,_=c.evaluate(sample(fk,sdk,actual));c.committed(step,11.)
    with pytest.raises(ValueError,match='RESPONSE_NOT_POST_COMMAND'):
        c.evaluate(sample(fk,step.positions,actual,11.1))


def test_already_inside_tolerance_requires_no_command():
    fk,sdk,_,goal=recorded_fixture();initial=sample(fk,sdk,sdk)
    c=PregraspCompensation(fk,initial,goal)
    code,step,evidence=c.evaluate(initial)
    assert code=='PREGRASP_MEASURED_CONVERGED' and step is None and c.steps==0


@pytest.mark.parametrize('p,a',[(.0061,.05),(.006,.09),(0,.05),(.006,float('nan'))])
def test_tolerance_cannot_be_relaxed(p,a):
    fk,sdk,actual,goal=recorded_fixture()
    with pytest.raises(ValueError):PregraspCompensation(fk,sample(fk,sdk,actual),goal,
        position_tolerance_m=p,orientation_tolerance_rad=a)


def test_step_budget_prevents_implicit_retry():
    fk,sdk,actual,goal=recorded_fixture();c=PregraspCompensation(fk,sample(fk,sdk,actual),goal)
    c.steps=12
    with pytest.raises(ValueError,match='STEP_BUDGET'):c.evaluate(sample(fk,sdk,actual))


def test_commit_is_exactly_once():
    fk,sdk,actual,goal=recorded_fixture();c=PregraspCompensation(fk,sample(fk,sdk,actual),goal)
    _,step,_=c.evaluate(sample(fk,sdk,actual));c.committed(step,11.)
    with pytest.raises(ValueError,match='COMMIT_INVALID'):c.committed(step,11.)


def test_goal_outside_local_capture_range_is_rejected():
    fk,sdk,actual,goal=recorded_fixture();goal[0]+=.1
    c=PregraspCompensation(fk,sample(fk,sdk,actual),goal)
    with pytest.raises(ValueError,match='OUTSIDE_LOCAL_CAPTURE_RANGE'):c.evaluate(sample(fk,sdk,actual))


@pytest.mark.parametrize('goal',[[0.]*7,[0.]*6,[0.,0.,0.,0.,0.,0.,float('nan')]])
def test_invalid_visual_pose_is_rejected(goal):
    with pytest.raises(ValueError):pose_matrix(goal)
