from copy import deepcopy
import json
import math
from pathlib import Path

import numpy as np
import pytest

from test_endpoint_correction import sample
from alicia_flexible_grasp.robot.pregrasp_compensation import PregraspCompensation, pose_matrix
from alicia_flexible_grasp.robot.stationary_following import ARM_NAMES, SerialUrdfFk, SDK_QUANTUM_RAD as Q


def recorded_fixture(day='20260919'):
    record = json.loads((Path(__file__).parent/('fixtures/pregrasp_following_'+day+'.json')).read_text())
    fk = SerialUrdfFk(record['robot_description'], ARM_NAMES)
    assert fk.model_sha256 == record['model_sha256']
    sdk = (np.array(record['initial_sdk_counts'])-2048)*Q
    actual = (np.array(record['initial_measured_counts'])-2048)*Q
    goal = record['goal_position_m']+record['goal_quaternion_xyzw']
    return fk,sdk,actual,goal


def test_recorded_wrist_pose_converges_under_explicit_four_axes_response_hypothesis():
    fk,sdk,actual,goal = recorded_fixture('20260920_wrist')
    initial = sample(fk,sdk,actual)
    correction = PregraspCompensation(fk,initial,goal)
    original_goal = correction.goal.copy()
    held = sdk[[1,4]].copy()
    for i in range(13):
        code,step,evidence = correction.evaluate(sample(fk,sdk,actual,10+i*3))
        if step is None:
            break
        delta = np.array(step.target_counts)-step.baseline_counts
        assert delta[1] == delta[4] == 0 and max(abs(delta)) <= 4
        assert max(abs(np.array(step.target_counts)-correction.initial_counts)) <= 32
        correction.committed(step,11+i*3)
        sdk = np.array(step.positions)
        actual += delta*Q  # hypothetical responsive plant, not a physical replay
        assert np.array_equal(sdk[[1,4]],held)
    assert code == 'PREGRASP_MEASURED_CONVERGED'
    assert evidence['position_error_m'] <= .006
    assert evidence['orientation_error_rad'] < math.radians(5)
    assert correction.steps == 6
    assert evidence['real_grasp_success'] is False
    assert np.array_equal(original_goal,correction.goal)
    assert not np.array_equal(sdk,actual)  # never pretend the encoder equals the command


@pytest.mark.parametrize('kind',['stalled','reverse','overshoot','held_joint_moves'])
def test_response_failure_stops_without_another_step_or_rollback(kind):
    fk,sdk,actual,goal = recorded_fixture()
    c = PregraspCompensation(fk,sample(fk,sdk,actual),goal)
    _,step,_ = c.evaluate(sample(fk,sdk,actual))
    delta = np.array(step.target_counts)-step.baseline_counts
    response = delta.copy()
    if kind == 'stalled': response[:] = 0
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


def test_live_joint5_target_inside_response_band_is_not_credited_as_four_counts():
    fk,sdk,actual,goal=recorded_fixture('20260920')
    c=PregraspCompensation(fk,sample(fk,sdk,actual),goal)
    _,step,_=c.evaluate(sample(fk,sdk,actual))
    delta=np.asarray(step.target_counts)-step.baseline_counts
    assert step.baseline_counts[4]==1137 and step.measured_counts[4]==1140
    assert delta[4]==0  # 1137+4=1141 is only one count from 1140
    assert max(abs(delta))<=4 and delta[1]==0


@pytest.mark.parametrize('response_counts',[0,1])
def test_partial_progress_holds_weak_axis_without_aborting_responsive_axes(response_counts):
    fk,sdk,actual,goal=recorded_fixture()
    c=PregraspCompensation(fk,sample(fk,sdk,actual),goal)
    _,first,_=c.evaluate(sample(fk,sdk,actual));delta=np.array(first.target_counts)-first.baseline_counts
    assert delta[2]==4
    delta[2]=response_counts
    c.committed(first,11.)
    after=actual+delta*Q
    _,second,report=c.evaluate(sample(fk,first.positions,after,13.))
    assert second is not None
    assert report['last_step_response']['partial_bounded_response']
    assert report['last_step_response']['newly_held_joints']==['Joint3']
    assert c.held_axes=={1,2,4}
    assert second.target_counts[2]==first.target_counts[2]
    assert report['position_error_m']<c.residual(actual)[0]
    # A late movement of the held axis is still monitored, not masked.
    second_delta=np.array(second.target_counts)-second.baseline_counts
    second_delta[2]=3
    c.committed(second,14.)
    with pytest.raises(ValueError,match='NO_BOUNDED_DIRECTIONAL_RESPONSE'):
        c.evaluate(sample(fk,second.positions,after+second_delta*Q,16.))


def test_exact_recorded_partial_response_can_continue_without_commanding_joint5_again():
    from alicia_flexible_grasp.robot.endpoint_correction import CorrectionStep
    fk,sdk,actual,goal=recorded_fixture('20260920')
    record=json.loads((Path(__file__).parent/'fixtures/pregrasp_following_20260920.json').read_text())
    c=PregraspCompensation(fk,sample(fk,sdk,actual),goal)
    _,proposal,_=c.evaluate(sample(fk,sdk,actual))
    target=tuple((np.array(proposal.baseline_counts)+record['recorded_command_delta_counts']).tolist())
    # Seed the exact previously admitted historical command, not a claim that
    # the revised search would issue the old ineffective Joint5 command.
    issued=CorrectionStep(proposal.baseline_counts,target,proposal.measured_counts,
                          proposal.sdk_stamp_ns,proposal.accepted_stamp_ns)
    c.proposed=issued;c.committed(issued,11.)
    after=actual+np.array(record['recorded_measured_delta_counts'])*Q
    code,step,report=c.evaluate(sample(fk,issued.positions,after,13.))
    assert code=='PREGRASP_STEP_PROPOSED'
    assert report['position_error_m']==pytest.approx(.016547458311,abs=1e-10)
    assert report['held_joints']==['Joint2','Joint5']
    assert step.target_counts[4]==1141
    assert c.response_gains[2]==.75


@pytest.mark.parametrize('joint3_loses_one_count',[False,True])
def test_live_pose_hypothesis_converges_with_stalled_joint5_and_partial_joint3(joint3_loses_one_count):
    fk,sdk,actual,goal=recorded_fixture('20260920')
    c=PregraspCompensation(fk,sample(fk,sdk,actual),goal)
    immutable=c.goal.copy();initial_sdk=sdk.copy();held5=None
    for i in range(c.MAX_STEPS+1):
        code,step,e=c.evaluate(sample(fk,sdk,actual,10+i*3))
        if step is None:break
        delta=np.asarray(step.target_counts)-step.baseline_counts
        if 4 in c.held_axes:
            assert delta[4]==0
            held5=step.target_counts[4] if held5 is None else held5
            assert step.target_counts[4]==held5
        c.committed(step,11+i*3);sdk=np.array(step.positions)
        delta[4]=0
        if joint3_loses_one_count and abs(delta[2])==4:delta[2]=int(np.sign(delta[2])*3)
        actual+=delta*Q
        assert max(abs(sdk-initial_sdk))<=32*Q+1e-12
        assert max(abs(sdk-actual))<=.035
    assert code=='PREGRASP_MEASURED_CONVERGED'
    assert e['position_error_m']<=.006 and e['orientation_error_rad']<=math.radians(5)
    assert np.array_equal(c.goal,immutable) and 4 in c.held_axes
    assert c.steps<=12


def test_older_pose_with_two_held_joints_stops_at_bound_without_relaxing_tolerance():
    fk,sdk,actual,goal=recorded_fixture('20260919')
    c=PregraspCompensation(fk,sample(fk,sdk,actual),goal)
    with pytest.raises(ValueError,match='NO_BOUNDED_IMPROVING_STEP'):
        for i in range(c.MAX_STEPS+1):
            _,step,_=c.evaluate(sample(fk,sdk,actual,10+i*3))
            assert step is not None
            delta=np.asarray(step.target_counts)-step.baseline_counts
            assert delta[1]==delta[4]==0
            c.committed(step,11+i*3);sdk=np.asarray(step.positions);actual+=delta*Q
    assert c.steps==8
    assert .006 < c.last_evidence['position_error_m'] < .0066
    assert c.tolerances[0]==.006


def test_held_wrist_movement_is_not_hidden_by_omitting_it_from_commands():
    fk,sdk,actual,goal=recorded_fixture('20260920_wrist')
    c=PregraspCompensation(fk,sample(fk,sdk,actual),goal)
    _,step,_=c.evaluate(sample(fk,sdk,actual))
    response=np.asarray(step.target_counts)-step.baseline_counts
    assert response[4]==0
    response[4]=3
    c.committed(step,11.)
    with pytest.raises(ValueError,match='NO_BOUNDED_DIRECTIONAL_RESPONSE'):
        c.evaluate(sample(fk,step.positions,actual+response*Q,13.))
