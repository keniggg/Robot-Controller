"""G43 geometry through Task, original service handler and continuous kernel.

The frozen plan and URDF are historical. The short MoveIt curve, service
transport, controller and feedback providers are explicitly simulated. These
tests prove the software handoff and its rejection boundaries, never a new
physical response or reconstruction of the unrecorded historical trajectory.
"""
from copy import deepcopy
import hashlib
import json
from pathlib import Path
import time
from types import SimpleNamespace as NS
from unittest.mock import Mock

import pytest
import rospy
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import RobotTrajectory
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

from alicia_flexible_grasp.grasp import contact_execution as execution
from alicia_flexible_grasp.robot import contact_linear_compensation as linear
from alicia_flexible_grasp.robot.observation_path_guard import (
    FrozenObservationScene, wire_digest)
from alicia_flexible_grasp.robot.stationary_following import ARM_NAMES, SDK_QUANTUM_RAD as Q
from test_contact_execution import make_task, module as task
from test_contact_linear_compensation import gateway_case
from test_gen44_current_stage_contract import g43_case, route


PATH_FIXTURE_SHA = '95c72478c8f24da8953ec990c42b73323f1ab2b309432d473073dba1063e59fd'


@pytest.fixture(autouse=True)
def no_external_boundaries(monkeypatch):
    import socket
    class BoundaryError(BaseException):
        pass
    def reject(*args, **kwargs):
        raise BoundaryError('GEN44 chain forbids external '+repr(args))
    for name in ('connect', 'connect_ex', 'sendto', 'bind', 'listen'):
        monkeypatch.setattr(socket.socket, name, reject)
    monkeypatch.setattr(socket, 'create_connection', reject)
    for name in ('init_node', 'Publisher', 'Subscriber', 'Service', 'ServiceProxy',
                 'wait_for_service', 'set_param', 'delete_param'):
        monkeypatch.setattr(rospy, name, reject)


def reconstructed_chain(monkeypatch, *, bad_path=False):
    c = g43_case()
    raw = (Path(__file__).parent/'fixtures/gen44_reconstructed_contact_path.json').read_bytes()
    assert hashlib.sha256(raw).hexdigest() == PATH_FIXTURE_SHA
    path = json.loads(raw)
    assert path['source_fixture_sha256'] == 'da66a49b1141e943e1b9e734621400a4130c17443c09516fabc9bc1e8a4c50f0'
    assert path['plan_id'] == c.plan.plan_id and path['model_sha256'] == c.fk.model_sha256
    assert path['historical_MoveIt_trajectory'] is False
    assert path['hardware_executed'] is False
    assert path['planner_and_response_provider_simulated'] is True
    nominal = RobotTrajectory()
    nominal.joint_trajectory.joint_names = list(ARM_NAMES)
    points = path['bad_positions_rad'] if bad_path else path['nominal_positions_rad']
    for index, values in enumerate(points):
        nominal.joint_trajectory.points.append(JointTrajectoryPoint(positions=values,
            velocities=[0.]*6, accelerations=[0.]*6,
            time_from_start=rospy.Duration(index*path['waypoint_interval_sec'])))
    if not bad_path:
        assert wire_digest(nominal) == path['nominal_wire_sha256']

    # Reuse only inert boundary construction; restore the original context
    # and sampled whole-arm checker methods before any request is handled.
    gateway, planner, _, _ = gateway_case(monkeypatch, 2)
    del gateway._validate_pregrasp_context
    del gateway._contact_command_collision_check
    node, _, clock, params = make_task(monkeypatch)
    cfg = route()
    params.update({'/robot_description': c.xml, '/grasp': cfg,
        '/grasp_6d/remote/gripper_geometry': {},
        '/robot/observation_tracking_contract_enabled': True,
        '/alicia_d_driver_node/endpoint_feedback_trim_enabled': False,
        '/robot/strict_execution_controller_sync_max_feedback_age_sec': .5})
    monkeypatch.setattr(rospy, 'get_param', lambda key, default=None: params.get(key, default))
    monkeypatch.setattr(rospy, 'get_time', lambda: clock['now'])
    sample = deepcopy(path['sample'])
    epoch = sample['epoch_ns']
    node._following_epoch_ns = epoch
    clock['now'] = sample['accepted_stamp_ns']*1e-9+.02
    joints = JointState()
    joints.header.frame_id = 'sdk_measured'
    joints.header.stamp = rospy.Time(sample['accepted_stamp_ns']//10**9,
                                    sample['accepted_stamp_ns']%10**9)
    joints.name = list(ARM_NAMES)+['right_finger']
    joints.position = list(sample['accepted_positions_rad'])+[path['opening_m']]
    node.latest_accepted_joint_state = joints
    authority = dict(manual=False)
    gateway._manual_control_active = lambda: authority['manual']
    gateway._pregrasp_task_stage = task.GraspStages.APPROACH_TARGET
    gateway._control_reference_epoch_ns = epoch
    gateway._observation_contexts = NS(revocation=3)
    # Observation-task liveness and actuation status are transport providers;
    # all original epoch, digest, manual, config and deadline checks run.
    gateway._require_live_observation_task = Mock()
    gateway._fresh_actuation_status = lambda: 'CONFIRMED: explicit offline provider'
    gateway.manipulator_group = 'manipulator'
    context = dict(plan=deepcopy(c.plan), scene=FrozenObservationScene.from_contact_plan(c.plan),
        epoch=epoch, revocation=3, model=c.xml, pose_index=2,
        config=deepcopy(cfg), deadline=time.monotonic()+120.)
    def admit_context(*args):
        gateway._compensation_visual_context = context
        return context
    gateway._pregrasp_context.side_effect = admit_context
    gateway._endpoint_following_snapshot.side_effect = lambda *a, **k: deepcopy(sample)
    gateway._observation_measured_opening = lambda: path['opening_m']
    gateway._observation_controller_hold = lambda _: JointTrajectoryPoint(
        positions=sample['sdk_positions_rad'], velocities=[0.]*6, accelerations=[0.]*6)
    planner._plan_cartesian_from_start_state.return_value = (nominal, 'explicit reconstructed provider')
    # Geometry equality is checked by the actual trajectory digest; retiming
    # is an identity provider and cannot replace the checked command curve.
    planner._same_joint_path_geometry = lambda a, b: wire_digest(a) == wire_digest(b)
    events = []
    kernels = []
    original_kernel = linear.validate_observation_trajectory
    def kernel(*args, **kwargs):
        try:
            result = original_kernel(*args, **kwargs)
        except Exception as exc:
            kernels.append(dict(ok=False, reason=str(exc), trajectory_sha256=wire_digest(args[0])))
            raise
        kernels.append(dict(result, ok=True))
        return result
    monkeypatch.setattr(linear, 'validate_observation_trajectory', kernel)
    def discovery(name, timeout):
        assert name == '/supervisor/move_contact_grasp_pose' and timeout == 2.
        # Bounded virtual transport delay, without sleeping or renewing data.
        clock['now'] += .01
        events.append(('service_discovered', name))
    def proxy(name, service_type, persistent=False):
        if name == '/supervisor/move_contact_grasp_pose':
            assert persistent is False
            def request(plan, execute):
                events.append(('service_request', bool(execute)))
                return gateway.handle_contact_grasp_pose(NS(plan=plan, execute=execute))
            return request
        assert name == '/check_state_validity'
        assert persistent is True
        def validity(request):
            events.append(('simulated_MoveIt_state_validity', tuple(request.robot_state.joint_state.position[:6])))
            return NS(valid=True)
        validity.close = lambda: events.append(('simulated_MoveIt_connection_closed',))
        return validity
    # The whole-arm service is explicit simulation; the continuous finite-CAD
    # checks above are the actual production kernel, including bad-path tests.
    def discovery_all(name, timeout):
        if name == '/check_state_validity':
            assert timeout == 2.
            return None
        return discovery(name, timeout)
    monkeypatch.setattr(rospy, 'wait_for_service', discovery_all)
    monkeypatch.setattr(rospy, 'ServiceProxy', proxy)
    pose = PoseStamped(); pose.header.frame_id = 'base_link'; pose.pose = deepcopy(c.plan.poses[2])
    move = node._contact_linear_service(c.plan, 2, cfg)
    return NS(c=c, path=path, task=node, gateway=gateway, planner=planner, sample=sample,
        clock=clock, params=params, authority=authority, context=context,
        events=events, kernels=kernels, move=move, pose=pose)


def test_task_hands_current_safe_pose_to_original_fresh_full_path_service_once(monkeypatch):
    r = reconstructed_chain(monkeypatch)
    actual = r.c.fk(r.sample['accepted_positions_rad'])
    assert not execution.plan_aperture_evaluator(r.c.plan, {}, r.path['opening_m'], 1)(actual)['ok']
    assert execution.plan_current_stage_cad_evaluator(r.c.plan, {}, r.path['opening_m'], 1)(actual)['ok']
    checked = r.move(r.pose, False)
    assert checked.success, checked.message
    assert r.events.count(('simulated_MoveIt_connection_closed',)) == 1
    r.planner._execute_cached_pose_plan.assert_not_called()
    r.gateway._ensure_trajectory_controllers_started.assert_not_called()
    assert len(r.kernels) == 2 and all(row['ok'] for row in r.kernels)
    assert r.move(r.pose, True).success
    assert r.task._last_measured_aperture_sample['required_next_path_contract'] == 'fresh_compensated_contact_linear'
    assert not r.task._last_measured_aperture_sample['rigid_residual_projection_diagnostic']['ok']
    assert len(r.kernels) == 3 and r.kernels[-1]['interval_checks'] > 0
    assert r.kernels[-1]['trajectory_sha256'] == r.path['command_wire_sha256']
    assert not r.move(r.pose, True).success
    assert r.planner._execute_cached_pose_plan.call_count == 1
    assert r.gateway._contact_linear_cache is None
    assert r.gateway._compensation_visual_context is None


def test_original_continuous_path_collision_cannot_be_bypassed_by_safe_current_cad(monkeypatch):
    r = reconstructed_chain(monkeypatch, bad_path=True)
    assert r.task._validate_measured_aperture(r.c.plan, route(), 1, 'current safe before bad future provider')
    response = r.move(r.pose, False)
    assert not response.success
    failure = json.loads(response.message)
    assert failure['validation_stage'] == 'nominal_continuous_geometry'
    assert 'support clearance' in failure['reason']
    assert r.kernels[-1]['ok'] is False
    r.planner._execute_cached_pose_plan.assert_not_called()
    r.gateway._ensure_trajectory_controllers_started.assert_not_called()
    assert r.gateway._contact_linear_cache is None


@pytest.mark.parametrize('fault', ['manual', 'target_digest', 'revocation', 'feedback'])
def test_fresh_service_rejects_real_context_or_feedback_changes_after_planning(monkeypatch, fault):
    r = reconstructed_chain(monkeypatch)
    assert r.move(r.pose, False).success
    if fault == 'manual':
        r.authority['manual'] = True
    elif fault == 'target_digest':
        r.context['plan'].poses[2].position.x += .001
    elif fault == 'revocation':
        r.gateway._observation_contexts.revocation += 1
    else:
        r.sample['accepted_positions_rad'][0] += Q
    response = r.move(r.pose, True)
    assert not response.success
    failure = json.loads(response.message)
    expected = {'manual': 'manual_control_active', 'target_digest': 'plan_digest_changed',
                'revocation': 'revocation', 'feedback': 'FEEDBACK_CHANGED_REPLAN_REQUIRED'}[fault]
    assert expected in failure['reason']
    r.planner._execute_cached_pose_plan.assert_not_called()
    r.gateway._ensure_trajectory_controllers_started.assert_not_called()
    assert r.gateway._contact_linear_cache is None
    assert r.gateway._compensation_visual_context is None


@pytest.mark.parametrize('fault', ['delay', 'out_of_order_stamp'])
def test_task_requires_fresh_feedback_before_delivering_cached_motion(monkeypatch, fault):
    r = reconstructed_chain(monkeypatch)
    assert r.move(r.pose, False).success
    if fault == 'delay':
        r.clock['now'] += .6
    else:
        r.task.latest_accepted_joint_state.header.stamp = deepcopy(r.c.plan.header.stamp)
        r.task.latest_accepted_joint_state.header.stamp.secs -= 1
    before = len([event for event in r.events if event == ('service_request', True)])
    with pytest.raises(task._MeasuredApertureRejected):
        r.move(r.pose, True)
    assert r.task._last_measured_aperture_sample['code'] == 'MEASURED_APERTURE_EVIDENCE_INVALID'
    assert len([event for event in r.events if event == ('service_request', True)]) == before
    r.planner._execute_cached_pose_plan.assert_not_called()


def test_after_simulated_path_final_measured_width_still_blocks_close(monkeypatch):
    r = reconstructed_chain(monkeypatch)
    assert r.move(r.pose, False).success
    assert r.move(r.pose, True).success
    sample = r.task.latest_accepted_joint_state
    sample.header.stamp = rospy.Time.from_sec(r.clock['now'])
    # This is an explicit future encoder provider, not inferred actual motion.
    sample.position = r.path['nominal_positions_rad'][-1]+[.040]
    close = Mock()
    guard = r.task._guard_measured_aperture_action(r.c.plan, route(), 2,
        'simulated fresh final width before close', close)
    with pytest.raises(task._MeasuredApertureRejected):
        guard()
    report = r.task._last_measured_aperture_sample
    assert report['code'] == 'MEASURED_APERTURE_INSUFFICIENT'
    assert report['projected_contact_is_measured'] is True
    assert 'rigid_residual_projection_diagnostic' not in report
    close.assert_not_called()
