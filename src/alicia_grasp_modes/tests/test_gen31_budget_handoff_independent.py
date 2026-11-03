"""One real producer publication/poll crosses the task's original wait.

Only transport, the prior stationary observation and clocks are inert test
fixtures. No station context, target association or receipt gate is mocked.
"""
from collections import deque
from copy import deepcopy
from types import SimpleNamespace as NS
from unittest.mock import Mock
import threading

from test_observation_measurement_remote import context, at
from test_station_compute_wait_remote import station, clock
from test_station_compute_work_handoff import next_row
import test_gen31_observation_search_progress as producer
import test_station_compute_wait_task as task_fixture
from test_task_modes import module as task_module
from alicia_grasp_modes.rgb_observation_source import digest
from alicia_grasp_modes.observation_search_failure import CODE
from alicia_flexible_grasp.robot.observation_view_gateway import _camera_configuration_contract


def receiving_task(c, row, monkeypatch):
    # The producer fixture's prior wire plan supplies the actual stationary
    # track, but its originally unused embedded geometry is empty. Populate
    # that prior observation from the same measured ObjectPose, not a grasp.
    plan = deepcopy(c.node._observe_return_prefix)
    plan.object_geometry.header = deepcopy(plan.header)
    plan.object_geometry.valid = True
    plan.object_geometry.label = 'unknown_small'
    plan.object_geometry.target_track_id = plan.target_track_id
    plan.object_geometry.pose_base = deepcopy(row['object'].pose_base.pose)
    monkeypatch.setattr(task_module.rospy, 'get_master', lambda: NS(
        getParam=lambda name: (1, 'inert shared parameter store', deepcopy(c.params.get(name)))))
    monkeypatch.setattr(task_module.rospy, 'loginfo', lambda *a: None)
    monkeypatch.setattr(task_module.rospy, 'logwarn_throttle', lambda *a: None)
    node = task_module.ModeAwareGraspTaskNode.__new__(task_module.ModeAwareGraspTaskNode)
    node._initialize_mode_boundary()
    node.active = False
    node.set_state = lambda *a, **k: None
    assert node.reset_mode_cb(None).success
    node._mode_bound_selection = deepcopy(node._mode_selection)
    node.active = True
    node.stage = task_fixture.base.GraspStages.PLAN_PREGRASP
    node._bound_execution_plan = deepcopy(plan)
    node._bound_execution_plan_id = plan.plan_id
    node._bound_execution_plan_digest = node._execution_plan_digest(plan)
    node.latest_grasp6d_plan = deepcopy(plan)
    node._execution_authority_revoked = False
    node._observe_return_session = deepcopy(c.node._observe_return_session)
    node._observation_measurement_request = deepcopy(c.node._measurement_remote_request)
    node._published_near_field_phase = deepcopy(c.phase)
    node._published_near_field_phase.header.frame_id = 'base_link'
    node._near_field_active = True
    node._near_field_phase_id = c.binding['phase_id']
    node._near_field_phase_deadline_sec = c.phase.deadline.to_sec()
    node._following_evidence_lock = threading.RLock()
    node._following_epoch_ns = c.binding['control_epoch_ns']
    node._following_accepted_history = deque(maxlen=128)
    node._following_sdk_history = deque(maxlen=128)
    return NS(node=node, plan=plan, params=c.params)


def task_row(task, row, now_ns):
    # row_at advances the copied outer header; a real ObjectPose carries the
    # same current stamp on pose_base too. Complete that wire fixture here.
    row = deepcopy(row)
    row['object'].pose_base.header = deepcopy(row['object'].header)
    ns = row['status']['source_stamp_ns']
    for stamp in (ns-10_000_000, ns+10_000_000, now_ns):
        task_fixture.feedback(task, stamp)
    task_fixture.deliver(task, row)
    assert task.node._station_task_record is not None
    assert task.node._station_task_record['contract'] == row['status']['station_compute_wait']
    assert not task.node._station_task_retired


def test_actual_published_budget_survives_next_poll_and_task_wait(station, monkeypatch):
    c = station
    # Establish this real shared camera contract before any station source is
    # admitted. Existing planner-only fixtures intentionally use a dummy hash.
    c.params['/camera'] = dict(fx=500., fy=500., width=24, height=24)
    c.params['/grasp'] = dict(target_observation_validity_sec=.8)
    c.binding['camera_configuration_sha256'] = digest(
        _camera_configuration_contract(c.params['/camera']))
    n = c.node
    prepared, row = producer.real_station_prepared(c, monkeypatch)
    scoped = n._station_observation_search_context(prepared)
    assert scoped is not None
    task = receiving_task(c, row, monkeypatch)
    task_row(task, row, n._ros_source_clock_ns())
    source = c.phase.deadline.to_nsec()-2_000_000_000
    while row['status']['source_stamp_ns'] < source:
        ns = min(source, row['status']['source_stamp_ns']+400_000_000)
        row = next_row(c, ns, row['status']['station_compute_wait'])
        task_row(task, row, n._ros_source_clock_ns())
    scoped['progress'].start(scoped['token'], 'view')
    scoped['progress'].finish(scoped['token'], 'view', elapsed_sec=1.64,
                              code='OBSERVATION_PLANNING_TIMEOUT')
    clock(c, source+850_000_000)
    task_fixture.feedback(task, n._ros_source_clock_ns())
    assert n._station_compute_poll() and n._station_compute_pending
    assert task.node._station_task_record['pending']
    budget = n._station_observation_search_reserve(scoped)
    assert not budget['station_can_start']
    scoped.update(budget=budget, station_budget_exhausted=True)
    result = n._publish_station_observation_search_exhausted(prepared,
        dict(station_observation_search=scoped, observation_variants=(object(), object())))
    assert result['status'] == CODE
    preview = deepcopy(n.preview_rich_plan_pub.messages[-1])
    epoch, count = n.target_instance_epoch, len(n.preview_rich_plan_pub.messages)
    n._station_compute_poll()
    assert n.target_instance_epoch == epoch and not n._station_compute_retired
    assert len(n.preview_rich_plan_pub.messages) == count
    assert n._observe_return_paused and not n._unknown_observation_planning_active()
    assert n._station_compute_snapshot_policy() is None
    assert not n._station_compute_work_certificate(prepared.ticket)
    # Deliver actual post-poll output through both task subscribers, including
    # the pending invalid geometry which formerly became a false TARGET_LOST.
    task.node.target_geometry_cb(deepcopy(n.latest_object_geometry))
    task.node.grasp6d_preview_plan_cb(preview)
    assert task.node.latest_grasp6d_preview_plan == preview
    frozen = deepcopy(task.node._bound_execution_plan)
    contract = deepcopy(task.node._station_task_record['contract'])
    states = []
    task.node.set_state = lambda *args: states.append(args)
    task.node._set_near_field_preview_stream = Mock(return_value=True)
    task.node._freeze_execution_plan = Mock(side_effect=AssertionError('failure cannot freeze'))
    move, execute = Mock(), Mock()
    cfg = dict(near_field_replan_enabled=True, near_field_replan_required=True,
               near_field_strategy='single_snapshot_direct')
    assert task.node._maybe_rebind_near_field_grasp6d_plan(
        cfg, {}, task.plan, move_pose=move, strict_execute_pose=execute) is None
    assert states[-1][0] == task_fixture.base.GraspStages.FAILED
    assert states[-1][1].startswith(CODE+':') and 'TARGET_LOST' not in states[-1][1]
    task.node._set_near_field_preview_stream.assert_called_once_with(cfg, False)
    task.node._freeze_execution_plan.assert_not_called()
    move.assert_not_called()
    execute.assert_not_called()
    assert task.node._bound_execution_plan == frozen
    assert task.node._station_task_record['contract'] == contract
    assert not task.node._execution_authority_revoked
    assert not task.node._station_task_retired
