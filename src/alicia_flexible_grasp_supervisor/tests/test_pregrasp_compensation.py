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


def replay_september24_to_last_sample(probe_enabled):
    """Replay only actually recorded commands/responses; no synthetic success."""
    record = json.loads((Path(__file__).parent /
        'fixtures/pregrasp_following_20260924_actual.json').read_text())
    fk, sdk, actual, goal = recorded_fixture('20260924_actual')
    c = PregraspCompensation(fk, sample(fk, sdk, actual), goal,
                            joint2_response_probe_enabled=probe_enabled)
    rows = record['actual_stationary_steps']
    for i, row in enumerate(rows[:-1]):
        sdk = (np.array(row['sdk_counts'])-2048)*Q
        actual = (np.array(row['measured_counts'])-2048)*Q
        _, step, evidence = c.evaluate(sample(fk, sdk, actual, 10.+4*i))
        assert evidence['position_error_m'] == pytest.approx(row['position_error_m'])
        assert list(step.target_counts) == rows[i+1]['sdk_counts']
        assert evidence['step_kind'] == 'bounded_cartesian_correction'
        c.committed(step, 12.+4*i)
    last = rows[-1]
    sdk = (np.array(last['sdk_counts'])-2048)*Q
    actual = (np.array(last['measured_counts'])-2048)*Q
    return c, sample(fk, sdk, actual, 42.), record


def test_september24_actual_failure_stays_failure_with_probe_disabled():
    c, final, _ = replay_september24_to_last_sample(False)
    with pytest.raises(ValueError, match='NO_BOUNDED_IMPROVING_STEP'):
        c.evaluate(final)
    assert c.last_evidence['position_error_m'] == pytest.approx(.006667775399362672)
    assert c.last_evidence['saturated_joints'] == ['Joint3']
    assert not c.last_evidence['real_grasp_success']


def test_september24_recovery_is_one_isolated_bounded_proposal_not_success():
    c, final, _ = replay_september24_to_last_sample(True)
    code, step, evidence = c.evaluate(final)
    assert code == 'PREGRASP_STEP_PROPOSED'
    assert (np.array(step.target_counts)-step.baseline_counts).tolist() == [0,4,0,0,0,0]
    assert evidence['step_kind'] == 'joint2_response_probe'
    assert evidence['position_error_m'] > .006
    assert evidence['predicted_position_error_m'] < .006
    assert not evidence['real_grasp_success']
    assert not c.joint2_response_probe_used  # A proposal is not an issued command.
    origin, started, goal = c.initial_counts, c.started, c.goal.copy()
    c.committed(step, 44.)
    assert c.steps == 9 and c.joint2_response_probe_used
    assert c.initial_counts == origin and c.started == started
    np.testing.assert_array_equal(c.goal, goal)
    assert not np.any(c._joint2_response_probe(
        np.array(final['accepted_positions_rad']), np.array(step.measured_counts), .0067))


@pytest.mark.parametrize('response_counts', [2, 3, 4])
def test_september24_only_hypothetical_fresh_joint2_response_can_converge(response_counts):
    c, final, _ = replay_september24_to_last_sample(True)
    _, step, _ = c.evaluate(final)
    c.committed(step, 44.)
    # No actual measurement exists for the new command; explicitly hypothetical.
    actual = np.array(final['accepted_positions_rad'])
    actual[1] += response_counts*Q
    code, next_step, report = c.evaluate(sample(c.fk, np.array(step.positions), actual, 46.))
    assert code == 'PREGRASP_MEASURED_CONVERGED' and next_step is None
    assert report['position_error_m'] <= .006 and report['steps_completed'] == 9
    assert report['joint2_response_probe_used'] and not report['real_grasp_success']


@pytest.mark.parametrize('response_counts', [0, 1, -2, 7])
def test_joint2_probe_without_bounded_response_stops_without_accumulation(response_counts):
    c, final, _ = replay_september24_to_last_sample(True)
    _, step, _ = c.evaluate(final)
    c.committed(step, 44.)
    actual = np.array(final['accepted_positions_rad'])
    actual[1] += response_counts*Q
    failure = ('FOLLOWING_OUTSIDE_CONTRACT' if response_counts == -2
               else 'NO_BOUNDED_DIRECTIONAL_RESPONSE')
    for _ in range(2):
        with pytest.raises(ValueError, match=failure):
            c.evaluate(sample(c.fk, np.array(step.positions), actual, 46.))
    assert c.steps == 9 and c.expected == step.target_counts
    assert c.joint2_response_probe_used


def test_joint2_probe_cannot_extend_step_time_or_total_limits():
    c, final, _ = replay_september24_to_last_sample(True)
    c.steps = c.MAX_STEPS
    with pytest.raises(ValueError, match='STEP_BUDGET'):
        c.evaluate(final)
    c, final, _ = replay_september24_to_last_sample(True)
    final['stamp_sec'] = c.started + c.MAX_SECONDS + .001
    with pytest.raises(ValueError, match='STALE_OR_EXPIRED'):
        c.evaluate(final)
    c, final, _ = replay_september24_to_last_sample(True)
    # The desired positive direction has no remaining original-box allowance.
    c.initial_counts = tuple(np.array(c.expected)-np.array([0,32,0,0,0,0]))
    actual = np.array(final['accepted_positions_rad'])
    assert not np.any(c._joint2_response_probe(actual, np.array(final['accepted_positions_rad'])/Q+2048,
                                             c.residual(actual)[0]))


@pytest.mark.parametrize('invalid', ['true', 1, None])
def test_joint2_probe_configuration_requires_boolean(invalid):
    fk,sdk,actual,goal = recorded_fixture('20260924_actual')
    with pytest.raises(ValueError, match='PROBE_CONFIG_INVALID'):
        PregraspCompensation(fk, sample(fk,sdk,actual), goal,
                            joint2_response_probe_enabled=invalid)


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
    assert correction.steps <= correction.MAX_STEPS
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


def test_recorded_eight_step_actual_response_preserves_failure_at_7_303_mm():
    # Actual post-command encoder samples, including delayed J1 response and
    # two counts of uncommanded J2 drift; no repeatable-plant assumption.
    from alicia_flexible_grasp.robot.endpoint_correction import CorrectionStep
    record = json.loads((Path(__file__).parent /
        'fixtures/pregrasp_following_20260920_held_actual.json').read_text())
    fk, sdk, actual, goal = recorded_fixture('20260920_held_actual')
    c = PregraspCompensation(fk, sample(fk, sdk, actual), goal)
    rows = record['actual_stationary_steps']
    for i, row in enumerate(rows[:-1]):
        sdk = (np.array(row['sdk_counts']) - 2048) * Q
        actual = (np.array(row['measured_counts']) - 2048) * Q
        _, step, evidence = c.evaluate(sample(fk, sdk, actual, 10. + 4*i))
        assert evidence['position_error_m'] == pytest.approx(row['position_error_m'])
        # Replay the historical commands explicitly: the new search must not
        # claim these measured responses belong to its different J6 commands.
        issued = CorrectionStep(step.baseline_counts, tuple(rows[i+1]['sdk_counts']),
            step.measured_counts, step.sdk_stamp_ns, step.accepted_stamp_ns)
        assert issued.target_counts[1] == c.initial_counts[1]
        assert issued.target_counts[4] == c.initial_counts[4]
        c.proposed = issued
        c.committed(issued, 12. + 4*i)
    last = rows[-1]
    sdk = (np.array(last['sdk_counts']) - 2048) * Q
    actual = (np.array(last['measured_counts']) - 2048) * Q
    for _ in range(2):
        with pytest.raises(ValueError, match='NO_BOUNDED_IMPROVING_STEP'):
            c.evaluate(sample(fk, sdk, actual, 42.))
    assert c.steps == 8
    assert c.last_evidence['position_error_m'] == pytest.approx(.007303032813870364)
    assert c.last_evidence['held_joints'] == ['Joint1', 'Joint2', 'Joint5', 'Joint6']
    assert c.expected[2] - c.initial_counts[2] == 32
    assert c.tolerances[0] == .006 and c.last_evidence['real_grasp_success'] is False


def test_budget_guide_avoids_spending_joint6_in_wrong_direction():
    fk, sdk, actual, goal = recorded_fixture('20260920_held_actual')
    c = PregraspCompensation(fk, sample(fk, sdk, actual), goal)
    _, step, report = c.evaluate(sample(fk, sdk, actual))
    delta = np.asarray(step.target_counts)-step.baseline_counts
    # Historical myopic first step was [4,0,4,4,0,-4], then J6 did not move.
    assert delta.tolist() == [4, 0, 4, 4, 0, 4]
    assert report['predicted_position_error_m'] < report['position_error_m']
    guide = report['conditional_budget_guide']
    assert guide['delta_counts'][5] > 0
    assert guide['predicted_position_error_m'] < .006
    assert not guide['physical_convergence_proven'] and not guide['path_authorized']
    assert not guide['globally_optimal']


def test_held_actual_start_converges_only_under_explicit_repeatable_response_hypothesis():
    fk, sdk, actual, goal = recorded_fixture('20260920_held_actual')
    c = PregraspCompensation(fk, sample(fk, sdk, actual), goal)
    original = c.goal.copy()
    errors = []
    for i in range(c.MAX_STEPS+1):
        code, step, report = c.evaluate(sample(fk, sdk, actual, 10.+4*i))
        errors.append(report['position_error_m'])
        if step is None:
            break
        delta = np.asarray(step.target_counts)-step.baseline_counts
        assert delta[1] == delta[4] == 0
        assert max(abs(delta)) <= 4
        assert max(abs(np.asarray(step.target_counts)-c.initial_counts)) <= 32
        c.committed(step, 12.+4*i)
        sdk = np.asarray(step.positions)
        actual += delta*Q  # hypothetical; NOT the archived measured responses
    assert code == 'PREGRASP_MEASURED_CONVERGED'
    assert errors[-1] <= .006 and all(a > b for a, b in zip(errors, errors[1:]))
    assert report['orientation_error_rad'] <= math.radians(5)
    assert np.array_equal(c.goal, original) and not report['real_grasp_success']


def test_guide_uses_remaining_original_box_steps_gains_and_held_axes():
    fk, sdk, actual, goal = recorded_fixture('20260920_held_actual')
    c = PregraspCompensation(fk, sample(fk, sdk, actual), goal)
    c.steps = 11
    c.expected = tuple(np.asarray(c.initial_counts)+[28, 0, 32, 30, 0, 0])
    c.held_axes.add(5)
    c.response_gains[2] = .5
    guide, error = c._remaining_budget_guide(actual)
    assert max(abs(guide)) <= 4
    assert guide[1] == guide[4] == guide[5] == 0
    assert guide[2] <= 0 and guide[3] <= 2
    assert max(abs(np.asarray(c.expected)+guide-c.initial_counts)) <= 32
    assert error <= c.residual(actual)[0]


def test_recorded_budget_guided_seven_steps_reach_original_gate_with_actual_encoders():
    record = json.loads((Path(__file__).parent /
        'fixtures/pregrasp_following_20260920_budget_actual.json').read_text())
    fk, sdk, actual, goal = recorded_fixture('20260920_budget_actual')
    c = PregraspCompensation(fk, sample(fk, sdk, actual), goal)
    rows = record['actual_stationary_steps']
    for i, row in enumerate(rows):
        sdk = (np.array(row['sdk_counts'])-2048)*Q
        actual = (np.array(row['measured_counts'])-2048)*Q
        code, step, report = c.evaluate(sample(fk, sdk, actual, 10.+5*i))
        assert report['position_error_m'] == pytest.approx(row['position_error_m'])
        assert report['held_joints'] == row['held_joints']
        if i == len(rows)-1:
            assert step is None and code == 'PREGRASP_MEASURED_CONVERGED'
        else:
            assert list(step.target_counts) == rows[i+1]['sdk_counts']
            c.committed(step, 13.+5*i)
    assert c.steps == 7
    assert report['position_error_m'] == pytest.approx(.005903179294131822)
    assert report['orientation_error_rad'] == pytest.approx(.06341813378899798)
    assert not report['real_grasp_success']  # subsequent vision stage failed
