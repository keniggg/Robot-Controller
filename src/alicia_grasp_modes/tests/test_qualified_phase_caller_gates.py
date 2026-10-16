"""Early planning has no authority to skip the existing reached-view gates.

Physical observation submission is an inert sentinel. The first-view CAD gate
and the original first/supplement caller control flow execute unchanged.
"""
from unittest import mock
from types import SimpleNamespace as NS

import pytest
import test_grasp_task_sequence as sequence
from test_observe_return_supplement_task import context, prepare, supplement, install, invoke


@pytest.mark.parametrize('fault', ['envelope', 'range'])
def test_first_view_reached_gate_still_prevents_contact_after_planning_started(fault):
    fixture = sequence.GraspTaskSequenceTest()
    n, plan, actual = fixture._measured_observation_fixture(z=.13414 if fault == 'envelope' else .20)
    base = sequence.grasp_task_node
    plan.poses[0] = base.deepcopy(actual.pose)
    plan.plan_id = sequence.compute_plan_id(plan)
    n._freeze_execution_plan(plan)
    n._near_field_active = False
    n._position_only_execute_globally_enabled = lambda: False
    n._execution_checkpoint = mock.Mock(return_value=True)
    n._command_gripper_position = mock.Mock(return_value=True)
    n._wait_for_motion_settle = mock.Mock(return_value=True)
    n._current_observation_view_reusable = mock.Mock(return_value=False)
    # This sentinel represents only completed observation motion. The exact
    # qualified dual-ACK/phase construction is independently tested with MRO.
    def observation_done(*args, **kwargs):
        n._near_field_active = True
        return True
    n._plan_and_execute_pose = mock.Mock(side_effect=observation_done)
    n._wait_for_fresh_observation_camera_target_range = mock.Mock(
        return_value=base.PlanValidationResult(False, 'RGBD_INVALID', 'original range rejection'))
    n._maybe_execute_observation_camera_retreat = mock.Mock(return_value=None)
    n._maybe_rebind_near_field_grasp6d_plan = mock.Mock(side_effect=AssertionError('contact planning reached'))
    n._ensure_qualified_station_phase = mock.Mock(side_effect=AssertionError('failed range cannot reuse phase'))
    strict = mock.Mock(side_effect=AssertionError('contact motion reached'))
    with mock.patch.object(base.rospy.Time, 'now', return_value=base.rospy.Time.from_sec(10.)), \
            mock.patch.object(base.rospy, 'get_param', side_effect=lambda name, default=None: default):
        result = n._execute_grasp6d_plan(dict(near_field_replan_enabled=True,
            near_field_replan_required=True, observation_reuse_position_tolerance_m=0.,
            measured_endpoint_check_enabled=False), {'open_position_m':.05}, .05,
            mock.Mock(), object(), object(), None, plan, strict_execute_pose=strict)
    assert result is False and n._near_field_active is True
    assert n._plan_and_execute_pose.call_count == 1
    assert not strict.called and not n._maybe_rebind_near_field_grasp6d_plan.called
    assert not n._ensure_qualified_station_phase.called
    if fault == 'envelope':
        assert n._last_measured_observation_envelope_sample['code'] == 'OBSERVATION_SUPPORT_COLLISION'
        assert not n._wait_for_fresh_observation_camera_target_range.called
    else:
        assert n._wait_for_fresh_observation_camera_target_range.call_count == 1


@pytest.mark.parametrize('fault', ['envelope', 'range'])
def test_supplement_reached_gate_still_prevents_contact_after_planning_started(context, monkeypatch, fault):
    c = context
    calls, bound = prepare(c, monkeypatch)
    c.now[0] += 2_000_000_000
    plan, report = supplement(c, 2)
    install(c, plan, report)
    original = c.node._plan_and_execute_pose
    def observation_done(*args, **kwargs):
        result = original(*args, **kwargs)
        c.node._near_field_active = True
        return result
    c.node._plan_and_execute_pose = observation_done
    c.node._ensure_qualified_station_phase = mock.Mock(side_effect=AssertionError('failed gate cannot reuse phase'))
    if fault == 'envelope': c.node._validate_measured_observation_envelope = lambda *args: False
    else: c.node._wait_for_fresh_observation_camera_target_range = lambda *args: NS(ok=False, code='RGBD_INVALID', reason='')
    assert invoke(c, c.plan) == {'failed': True}
    assert len([event for event in calls if event[0] == 'move']) == 1
    assert c.node._near_field_active is True  # start_cb finally owns deactivation.
    assert not c.node._ensure_qualified_station_phase.called
    assert not c.replans
