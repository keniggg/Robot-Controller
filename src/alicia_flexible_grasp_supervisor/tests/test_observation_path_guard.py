"""Offline continuous commanded-path checks: no ROS node or service calls."""
from copy import deepcopy
from pathlib import Path
from types import SimpleNamespace as NS

import numpy as np
import pytest
import rospy
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

from alicia_flexible_grasp.robot.observation_path_guard import (
    CommittedObservationContexts, FrozenObservationScene, ObservationPathError,
    SerialUrdfFk, segment_coefficients, validate_observation_trajectory,
    _polynomial, _derivative_bound,
)
from alicia_flexible_grasp.grasp.gripper_geometry import GripperGeometry
from alicia_flexible_grasp.grasp.rich_plan_integrity import compute_plan_id
from alicia_flexible_grasp_supervisor.msg import Grasp6DPlan
from test_observation_branch_hint import offline_robot_description


def model_xml(two=False):
    second = ('<joint name="J2" type="revolute"><parent link="one"/><child link="two"/>'
              '<origin xyz="0 0 .1"/><axis xyz="1 0 0"/></joint>' if two else '')
    return ('<robot name="test"><joint name="J1" type="revolute">'
            '<parent link="base_link"/><child link="one"/><axis xyz="0 1 0"/>'
            '</joint>'+second+'<joint name="tip" type="fixed"><parent link="'+
            ('two' if two else 'one')+'"/><child link="tool0"/>'
            '<origin xyz="0 0 .4"/></joint></robot>')


def gripper():
    return GripperGeometry(.05, .002, np.array([.0434, .0286, .06]),
                           np.array([.1175, .155, .0774]), .003)


def scene():
    return FrozenObservationScene('test', 'frozen', 10**9, np.array([0., 0., 1.]),
                                  0., np.array([5., 5., 5.]), np.eye(3), np.array([.02]*3))


def point(q, when=0., v=None, a=None):
    p = JointTrajectoryPoint()
    p.positions = list(q)
    p.velocities = [] if v is None else list(v)
    p.accelerations = [] if a is None else list(a)
    p.time_from_start = rospy.Duration.from_sec(when)
    return p


def trajectory(points, names=('J1',)):
    result = RobotTrajectory()
    result.joint_trajectory.joint_names = list(names)
    result.joint_trajectory.points = points
    return result


def check(plan, frozen=None, desired=None, **kwargs):
    names = plan.joint_trajectory.joint_names
    fk = SerialUrdfFk(model_xml(len(names)==2), names)
    desired = desired or point(plan.joint_trajectory.points[0].positions,
                               v=[0.]*len(names), a=[0.]*len(names))
    return validate_observation_trajectory(plan, frozen or scene(), fk,
                                            gripper(), .05, desired, **kwargs)


@pytest.mark.parametrize('degree', [1, 3, 5])
def test_noetic_spline_endpoint_derivatives(degree):
    first = point([.2], v=[.3] if degree>=3 else None, a=[.4] if degree==5 else None)
    second = point([.8], v=[-.2] if degree>=3 else None, a=[-.1] if degree==5 else None)
    c = segment_coefficients(first, second, 2., 1)
    assert _polynomial(c, 0.)[0] == pytest.approx(.2)
    assert _polynomial(c, 1.)[0] == pytest.approx(.8)
    if degree>=3:
        derivative = c[:, 1:]*np.arange(1, 6)
        assert _polynomial(derivative, 0.)[0]/2 == pytest.approx(.3)
        assert _polynomial(derivative, 1.)[0]/2 == pytest.approx(-.2)
    if degree==5:
        acceleration = c[:, 2:]*np.arange(2, 6)*np.arange(1, 5)
        assert _polynomial(acceleration, 0.)[0]/4 == pytest.approx(.4)
        assert _polynomial(acceleration, 1.)[0]/4 == pytest.approx(-.1)


def test_bernstein_derivative_bound_contains_all_dense_samples():
    rng = np.random.RandomState(23)
    for _ in range(30):
        c = rng.normal(size=(3, 6))*4
        lo, hi = sorted(rng.uniform(0, 1, 2))
        bound = _derivative_bound(c, lo, hi)
        actual = np.array([_polynomial(c[:, 1:]*np.arange(1, 6), t)
                           for t in np.linspace(lo, hi, 301)])
        assert np.all(np.max(np.abs(actual), axis=0) <= bound + 1e-10)


@pytest.mark.parametrize('two', [False, True])
def test_safe_single_and_simultaneous_multijoint_paths(two):
    count = 2 if two else 1
    plan = trajectory([point([0.]*count, 0., [0.]*count, [0.]*count),
                       point([.2]*count, 2., [0.]*count, [0.]*count)],
                      ['J1', 'J2'] if two else ['J1'])
    report = check(plan)
    assert report['certified_minimum_clearance_m'] >= .003
    assert report['bridge'] == 'stationary_desired_to_first_positive_time_waypoint'
    assert len(report['trajectory_sha256']) == 64


@pytest.mark.parametrize('quintic', [False, True])
def test_spline_midsegment_collision_with_safe_knots_is_rejected(quintic):
    # Bridge is safe; the second segment's equal endpoint q values hide a
    # large curved excursion. A q-linear/waypoint-only check misses it.
    a = [0.] if quintic else None
    plan = trajectory([point([0.], 0., [0.], a), point([0.], 1., [2.], a),
                       point([0.], 5., [-2.], a)])
    with pytest.raises(ObservationPathError, match='support clearance'):
        check(plan)


def test_bridge_uses_first_positive_time_point_not_dropped_zero_point():
    plan = trajectory([point([0.], 0., [0.], [0.]), point([0.], 1., [12.], [0.])])
    with pytest.raises(ObservationPathError, match='support clearance'):
        check(plan)


def test_target_obb_collision_in_middle_is_rejected():
    plan = trajectory([point([-.5], 0.), point([.5], 2.)])
    frozen = scene()
    frozen = FrozenObservationScene(frozen.plan_id, frozen.digest, frozen.source_ns,
        frozen.normal, frozen.offset, np.array([-.0393, .0003, .4-.09344]),
        np.eye(3), np.array([.015]*3))
    with pytest.raises(ObservationPathError, match='target OBB'):
        check(plan, frozen)


@pytest.mark.parametrize('failure', ['moving', 'missing_acc', 'nonzero_stamp', 'budget', 'depth', 'bad_time', 'duplicate_names'])
def test_missing_or_unresolved_evidence_fails_closed(failure):
    plan = trajectory([point([0.], 0., [0.], [0.]), point([.2], 2., [0.], [0.])])
    desired, kwargs = point([0.], v=[0.], a=[0.]), {}
    if failure=='moving': desired.velocities=[.01]
    if failure=='missing_acc': desired.accelerations=[]
    if failure=='nonzero_stamp': plan.joint_trajectory.header.stamp=rospy.Time(4)
    if failure=='budget': kwargs['max_checks']=0
    if failure=='depth': kwargs.update(max_depth=0); plan.joint_trajectory.points[-1].positions=[1.]
    if failure=='bad_time': plan.joint_trajectory.points[-1].time_from_start=rospy.Duration(0)
    if failure=='duplicate_names': plan.joint_trajectory.joint_names=['J1','J1']
    with pytest.raises((ObservationPathError, ValueError)):
        check(plan, desired=desired, **kwargs)


def test_production_urdf_fk_known_aligned_pose_and_joint_permutation():
    root = Path(__file__).resolve().parents[2]
    xml = (root/'real-arm/alicia_d_descriptions/urdf/alicia_duo_with_gripper.urdf').read_text()
    names = ['Joint%d'%i for i in range(1,7)]
    q = [-1.9113400617,.5645049299,-.2699806187,-.0076699039,-.2285631374,-.0015339808]
    expected = [-.07928442582838044,-.22449173683672835,.1460659223532479]
    fk = SerialUrdfFk(xml, names)
    np.testing.assert_allclose(fk(q)[:3,3], expected, atol=1e-12)
    reverse = SerialUrdfFk(xml, names[::-1])
    np.testing.assert_allclose(reverse(q[::-1]), fk(q), atol=1e-12)


def rich_plan():
    result = Grasp6DPlan()
    result.header.frame_id='base_link'; result.header.stamp=rospy.Time(10)
    result.valid=True; result.diagnostic='FAR_FIELD_OBSERVATION_PLAN'
    result.target_track_id='g0-t1'; result.refinement_status='NOT_EVALUATED'
    result.candidate_source='tabletop_geometry'; result.candidate_source_lineage=['tabletop_geometry']
    result.required_open_width_m=.04; result.model_choice='test'
    result.poses=[]
    for _ in range(4):
        p=Pose(); p.position.z=.4; p.orientation.w=1.; result.poses.append(p)
    geometry=result.object_geometry
    geometry.valid=True; geometry.header=deepcopy(result.header)
    geometry.target_track_id=result.target_track_id; geometry.source_mode='instance_mask'
    geometry.pose_base.position.z=.1; geometry.pose_base.orientation.w=1.
    geometry.size_xyz_m.x=.03; geometry.size_xyz_m.y=.02; geometry.size_xyz_m.z=.02
    geometry.support_normal_base.z=1.
    result.plan_id=compute_plan_id(result)
    return result


def target_for(plan):
    p=PoseStamped(); p.header=deepcopy(plan.header);p.pose=deepcopy(plan.poses[0]);return p


def test_committed_source_pose_geometry_frozen_and_new_plan_does_not_retarget():
    plan=rich_plan(); cache=CommittedObservationContexts();cache.ingest(plan)
    cache.set_active(True)
    target=target_for(plan); frozen,token=cache.capture(target, 10., 120.)
    plan.object_geometry.support_offset_m=.2
    assert frozen.offset==0.
    new=rich_plan();new.header.stamp=rospy.Time(11);new.object_geometry.header=deepcopy(new.header)
    new.plan_id=compute_plan_id(new);cache.ingest(new)
    cache.validate_capture(frozen,token)
    assert cache.capture(target,11.,120.)[0] is frozen


@pytest.mark.parametrize('change', ['stamp','frame','zero_quaternion','age','future'])
def test_context_rejects_unbound_corrected_or_stale_requests(change):
    plan=rich_plan();cache=CommittedObservationContexts();cache.ingest(plan);p=target_for(plan)
    cache.set_active(True)
    now=10.
    if change=='stamp':p.header.stamp=rospy.Time(11)
    if change=='frame':p.header.frame_id='camera_link'
    if change=='zero_quaternion':p.pose.orientation.w=0.
    if change=='age':now=131.
    if change=='future':now=9.
    with pytest.raises(ObservationPathError):cache.capture(p,now,120.)


def test_ambiguous_geometry_before_capture_or_tombstone_is_rejected():
    plan=rich_plan();cache=CommittedObservationContexts();cache.ingest(plan)
    cache.set_active(True)
    target=target_for(plan)
    other=deepcopy(plan);other.object_geometry.support_offset_m=.001;other.plan_id=compute_plan_id(other)
    cache.ingest(other)
    with pytest.raises(ObservationPathError):cache.capture(target,10.,120.)
    cache=CommittedObservationContexts();cache.ingest(plan);cache.set_active(True)
    frozen,token=cache.capture(target,10.,120.)
    cache.ingest(Grasp6DPlan())
    with pytest.raises(ObservationPathError):cache.validate_capture(frozen,token)


@pytest.mark.parametrize('prefix', [False, True])
def test_final_retimed_validation_failure_never_calls_execute(prefix):
    from test_observation_branch_hint import fixture, check as plan_check
    from test_moveit_planner_pose_feedback import make_pose
    planner, manipulator, _, _ = fixture()
    assert plan_check(planner)[0]
    planner.strict_execution_retime_enabled=True
    planner._retime_strict_execution_plan=lambda p:(deepcopy(p),'retimed')
    planner.observation_path_guard_required=True
    seen=[]
    def reject(path):
        seen.append(path)
        raise ObservationPathError('synthetic continuous collision')
    planner.observation_path_validator=reject
    method=planner.execute_cached_observation_prefix if prefix else planner.execute_cached_strict_pose
    ok,message=method(make_pose())
    assert not ok and 'synthetic continuous collision' in message
    assert seen and not manipulator.executed_plans
    assert planner._last_pose_plan is None
    assert not message.startswith('execute failed from cached plan (strict pose):')


def gateway_fixture(monkeypatch):
    import threading
    import test_motion_gateway_controller_start as gateway_tests
    from control_msgs.msg import JointTrajectoryControllerState
    from sensor_msgs.msg import JointState
    module = gateway_tests.motion_gateway_node
    gateway = module.MotionGateway.__new__(module.MotionGateway)
    gateway.joint_names = ['Joint%d'%i for i in range(1,7)] + ['right_finger']
    gateway._gui_direct_mode = False
    gateway._observation_context_lock = threading.RLock()
    gateway._observation_context_condition = threading.Condition(gateway._observation_context_lock)
    gateway._observation_contexts = CommittedObservationContexts()
    gateway._observation_contexts.set_active(True)
    gateway._observation_task_sub = NS(get_num_connections=lambda: 1)
    gateway._observation_task_state_stamp_ns = 0
    frozen = rich_plan(); gateway._observation_contexts.ingest(frozen)
    context = gateway._observation_contexts.capture(target_for(frozen), 10., 120.)
    state = JointTrajectoryControllerState()
    state.header.stamp = rospy.Time.from_sec(10.3)
    state.joint_names = gateway.joint_names[:-1]
    state.desired.positions = [0.]*6
    state.desired.velocities = [0.]*6
    state.desired.accelerations = [0.]*6
    gateway._trajectory_controller_state = state
    gateway._observation_sync_hold_evidence = {
        'names': tuple(state.joint_names), 'positions': tuple(state.desired.positions),
        'end_sec': 10.1}
    feedback = JointState()
    feedback.header.stamp = rospy.Time.from_sec(10.3)
    feedback.header.frame_id = 'sdk_measured'
    feedback.name = gateway.joint_names
    feedback.position = [0.]*6 + [.04975]
    gateway._observation_joint_feedback = feedback
    xml = (Path(__file__).resolve().parents[2]/
           'real-arm/alicia_d_descriptions/urdf/alicia_duo_with_gripper.urdf').read_text()
    params = {'/robot_description': xml, '/grasp': {'plan_validity_sec': 120.},
              '/alicia_controller/constraints': {name: {'trajectory': .12, 'goal': .035}
                                                  for name in gateway.joint_names[:-1]},
              '/grasp_6d/remote/gripper_geometry': {}}
    monkeypatch.setattr(module.rospy, 'get_param', lambda name, default=None: params.get(name, default))
    monkeypatch.setattr(module.rospy, 'get_time', lambda: 10.4)
    planner = NS(manipulator=NS(get_planning_frame=lambda:'base_link',
                                get_end_effector_link=lambda:'tool0'),
                 _observation_branch_signature=lambda:('model-signature', gateway.joint_names[:-1]))
    submitted = trajectory([point([0.]*6, 0., [0.]*6, [0.]*6),
                            point([.01]*6, 1., [0.]*6, [0.]*6)], gateway.joint_names[:-1])
    from alicia_flexible_grasp.robot.observation_path_guard import wire_digest
    monkeypatch.setattr(module, 'validate_observation_trajectory',
                        lambda path, *_args, **_kwargs: {'trajectory_sha256':wire_digest(path)})
    return module, gateway, context, planner, submitted, params


def test_gateway_uses_accepted_feedback_and_completed_hold(monkeypatch):
    module, gateway, context, planner, path, params = gateway_fixture(monkeypatch)
    report = gateway._validate_frozen_observation_path(context, path, planner)
    assert report['model_group_frame_tool_sha256'] == 'model-signature'
    assert report['trajectory_sha256']


def test_gateway_final_path_requires_live_tracking_allowance(monkeypatch):
    module, gateway, context, planner, path, params = gateway_fixture(monkeypatch)
    calls = []
    def proof(submitted, *_args, **kwargs):
        calls.append(kwargs['joint_error_bounds_rad'])
        return {'trajectory_sha256': module.wire_digest(submitted)}
    monkeypatch.setattr(module, 'validate_observation_trajectory', proof)
    gateway._validate_frozen_observation_path(context, path, planner)
    np.testing.assert_allclose(calls[0], [.12+np.pi/4096.]*6)
    del params['/alicia_controller/constraints']
    with pytest.raises(ObservationPathError, match='constraints unavailable'):
        gateway._validate_frozen_observation_path(context, path, planner)
    assert len(calls) == 1


@pytest.mark.parametrize('invalid', ['heartbeat','stale','missing_finger','nan_gap','oversize_gap',
                                    'no_hold_provenance','before_hold_end','different_hold',
                                    'moving_desired','missing_acceleration','manual','revoked'])
def test_gateway_missing_real_feedback_or_completed_hold_fails_closed(monkeypatch, invalid):
    module, gateway, context, planner, path, params = gateway_fixture(monkeypatch)
    state, feedback = gateway._trajectory_controller_state, gateway._observation_joint_feedback
    if invalid=='heartbeat': feedback.header.frame_id=''
    if invalid=='stale': feedback.header.stamp=rospy.Time(1)
    if invalid=='missing_finger': feedback.name[-1]='unrelated'
    if invalid=='nan_gap': feedback.position[-1]=float('nan')
    if invalid=='oversize_gap': feedback.position[-1]=.051
    if invalid=='no_hold_provenance': gateway._observation_sync_hold_evidence=None
    if invalid=='before_hold_end': state.header.stamp=rospy.Time(10)
    if invalid=='different_hold': state.desired.positions[0]=.01
    if invalid=='moving_desired': state.desired.velocities[0]=.01
    if invalid=='missing_acceleration': state.desired.accelerations=[]
    if invalid=='manual': gateway._gui_direct_mode=True
    if invalid=='revoked': gateway._observation_contexts.clear()
    with pytest.raises(ObservationPathError):
        gateway._validate_frozen_observation_path(context, path, planner)


@pytest.mark.parametrize('change', ['config','model','desired','opening','path','context','model_signature', 'constraints'])
def test_gateway_rechecks_inputs_after_path_proof(monkeypatch, change):
    module, gateway, context, planner, path, params = gateway_fixture(monkeypatch)
    def mutate(submitted, *_args, **_kwargs):
        if change=='config':params['/grasp_6d/remote/gripper_geometry']['support_clearance_m']=.004
        if change=='model':params['/robot_description']+=' '
        if change=='desired':gateway._trajectory_controller_state.desired.positions[0]=.002
        if change=='opening':gateway._observation_joint_feedback.position[-1]=.0495
        if change=='path':submitted.joint_trajectory.points[-1].positions[0]+=.1
        if change=='context':gateway._observation_contexts.clear()
        if change=='model_signature':planner._observation_branch_signature=lambda:('changed',())
        if change=='constraints':params['/alicia_controller/constraints']['Joint2']['trajectory']=.13
        return {'trajectory_sha256':'proof'}
    monkeypatch.setattr(module, 'validate_observation_trajectory', mutate)
    with pytest.raises(ObservationPathError):
        gateway._validate_frozen_observation_path(context, path, planner)


@pytest.mark.parametrize('atomic', [False, True])
def test_gateway_path_rejection_does_not_issue_failure_hold(monkeypatch, atomic):
    import test_motion_gateway_controller_start as gateway_tests
    module = gateway_tests.motion_gateway_node
    monkeypatch.setattr(module.rospy, 'get_param', lambda _name, default=None: default)
    gateway = gateway_tests.MotionGatewayControllerStartTest().make_gateway()
    gateway.planner.execute_cached_strict_pose = lambda _target: (
        False, 'strict cached execute blocked: OBSERVATION_PATH_INVALID: synthetic collision')
    handler = gateway.handle_pose_strict_plan_execute if atomic else gateway.handle_pose_strict_execute
    result = handler(NS(target='target', execute=True))
    assert not result.success
    assert gateway.failure_holds == 0


def test_unchanged_urdf_limits_apply_inside_spline_not_only_waypoints():
    xml=model_xml().replace('<axis xyz="0 1 0"/>',
        '<axis xyz="0 1 0"/><limit lower="-.2" upper=".2"/>')
    fk=SerialUrdfFk(xml, ['J1'])
    plan=trajectory([point([0.],0.,[0.]),point([0.],1.,[2.])])
    with pytest.raises(ObservationPathError, match='URDF joint limits'):
        validate_observation_trajectory(plan,scene(),fk,gripper(),.05,
                                        point([0.],v=[0.],a=[0.]))


def test_operation_context_parameter_precedence_matches_task(monkeypatch):
    module,gateway,context,planner,path,params=gateway_fixture(monkeypatch)
    gateway._planner_lock=__import__('threading').RLock()
    planner.ready=True;planner.strict_execution_max_joint_velocity_rad_s=.08
    gateway._observation_planner=planner;gateway.planner=NS()
    params['/grasp']['plan_validity_sec']=.1
    params['/grasp_6d/plan_validity_sec']=120.
    gateway._observation_contexts.set_active(False)
    gateway._observation_contexts.set_active(True)
    calls=[]
    result=gateway._with_observation_planner(
        NS(target=target_for(rich_plan()),execute=True),lambda req:calls.append(req))
    assert not result.success and 'stale or future' in result.message
    assert calls==[]


def test_active_task_reuses_frozen_scene_for_derived_pose_without_readmitting_ttl():
    plan=rich_plan();cache=CommittedObservationContexts();cache.ingest(plan)
    cache.set_active(True)
    frozen,token=cache.capture(target_for(plan),10.,120.)
    derived=target_for(plan);derived.pose.position.x+=.02
    replacement=deepcopy(plan);replacement.object_geometry.support_offset_m=.05
    replacement.plan_id=compute_plan_id(replacement);cache.ingest(replacement)
    recovered,after=cache.capture(derived,200.,120.)
    assert recovered is frozen and recovered.offset==0. and after==token
    cache.validate_capture(frozen,token)
    cache.set_active(False)
    with pytest.raises(ObservationPathError):cache.validate_capture(frozen,token)
    with pytest.raises(ObservationPathError):cache.capture(derived,200.,120.)
    # Another task is a fresh admission, not an indefinite replay lease.
    cache.set_active(True)
    with pytest.raises(ObservationPathError,match='stale'):
        cache.capture(derived,200.,120.)


def test_inactive_manual_track_change_and_tombstone_cannot_reactivate_capture():
    plan=rich_plan();cache=CommittedObservationContexts();cache.ingest(plan)
    with pytest.raises(ObservationPathError):cache.capture(target_for(plan),10.,120.)
    cache.set_active(True);frozen,token=cache.capture(target_for(plan),10.,120.)
    replacement=deepcopy(plan)
    replacement.target_track_id='g0-t2';replacement.object_geometry.target_track_id='g0-t2'
    replacement.plan_id=compute_plan_id(replacement);cache.ingest(replacement)
    with pytest.raises(ObservationPathError):cache.validate_capture(frozen,token)
    cache.ingest(plan);cache.set_active(True)  # repeated active cannot undo revocation
    with pytest.raises(ObservationPathError):cache.capture(target_for(plan),10.,120.)


def test_same_source_different_plan_payload_is_ambiguous_before_capture():
    plan=rich_plan();cache=CommittedObservationContexts();cache.ingest(plan)
    other=deepcopy(plan);other.poses[2].position.y=.1;other.plan_id=compute_plan_id(other)
    assert other.plan_id!=plan.plan_id
    cache.ingest(other);cache.set_active(True)
    with pytest.raises(ObservationPathError,match='unique'):
        cache.capture(target_for(other),10.,120.)


def test_republished_identical_plan_transport_sequence_is_not_geometry_conflict():
    plan = rich_plan()
    plan.header.seq = 5
    repeated = deepcopy(plan)
    repeated.header.seq = 6  # ROS Publisher increments only this transport counter.
    cache = CommittedObservationContexts()
    cache.ingest(plan)
    first = next(iter(cache.entries.values()))
    token = cache.revocation
    cache.ingest(repeated)
    assert next(iter(cache.entries.values())) is first
    assert cache.revocation == token
    assert plan.header.seq == 5 and repeated.header.seq == 6
    assert FrozenObservationScene.from_plan(repeated).digest == first.digest
    cache.set_active(True)
    assert cache.capture(target_for(repeated), 10., 120.)[0] is first


@pytest.mark.parametrize('field', ['pose', 'normal', 'track', 'diagnostic', 'width'])
def test_transport_sequence_normalization_never_hides_real_payload_changes(field):
    original = rich_plan()
    repeated = deepcopy(original)
    repeated.header.seq = 17
    if field == 'pose':
        repeated.poses[0].position.x += .001
    elif field == 'normal':
        repeated.object_geometry.support_normal_base.x = .001
        repeated.object_geometry.support_normal_base.z = float(np.sqrt(1.-.001**2))
    elif field == 'track':
        repeated.target_track_id = 'g0-t2'
        repeated.object_geometry.target_track_id = 'g0-t2'
    elif field == 'diagnostic':
        # A nested header is evidence, not the outer publisher's counter.
        repeated.object_geometry.header.seq = 17
    else:
        repeated.required_open_width_m += .001
    repeated.plan_id = compute_plan_id(repeated)
    cache = CommittedObservationContexts()
    cache.ingest(original)
    cache.ingest(repeated)
    assert next(iter(cache.entries.values())) is None


@pytest.mark.parametrize('event', ['inactive','future','disconnected','manual','track_change','tombstone'])
def test_gateway_active_lifecycle_revokes_before_final_submission(monkeypatch,event):
    from alicia_flexible_grasp_supervisor.msg import GraspState
    module,gateway,context,planner,path,params=gateway_fixture(monkeypatch)
    if event in ('inactive','future'):
        msg=GraspState();msg.active=event=='future'
        msg.header.stamp=rospy.Time.from_sec(11. if event=='future' else 10.4)
        gateway._observation_task_state_cb(msg)
    if event=='disconnected':gateway._observation_task_sub=NS(get_num_connections=lambda:0)
    if event=='manual':gateway._gui_direct_mode=True
    if event=='track_change':
        replacement=rich_plan();replacement.target_track_id='g0-t2'
        replacement.object_geometry.target_track_id='g0-t2';replacement.plan_id=compute_plan_id(replacement)
        gateway._committed_observation_plan_cb(replacement)
    if event=='tombstone':gateway._committed_observation_plan_cb(Grasp6DPlan())
    with pytest.raises(ObservationPathError):gateway._validate_frozen_observation_path(context,path,planner)


def test_old_active_status_cannot_resurrect_a_finished_task(monkeypatch):
    from alicia_flexible_grasp_supervisor.msg import GraspState
    module,gateway,context,planner,path,params=gateway_fixture(monkeypatch)
    inactive=GraspState();inactive.header.stamp=rospy.Time.from_sec(10.4);inactive.active=False
    gateway._observation_task_state_cb(inactive)
    old=GraspState();old.header.stamp=rospy.Time.from_sec(10.3);old.active=True
    gateway._observation_task_state_cb(old)
    assert not gateway._observation_contexts.active


@pytest.mark.parametrize('stage', [6,7,8])
def test_terminal_stage_immediately_revokes_even_when_active_flag_true(monkeypatch,stage):
    from alicia_flexible_grasp_supervisor.msg import GraspState
    from alicia_flexible_grasp.grasp.grasp_state_machine import STATE_NAMES
    module,gateway,context,planner,path,params=gateway_fixture(monkeypatch)
    msg=GraspState();msg.header.stamp=rospy.Time.from_sec(10.4)
    msg.active=True;msg.stage=stage;msg.state=STATE_NAMES[stage]
    gateway._observation_task_state_cb(msg)
    with pytest.raises(ObservationPathError):gateway._observation_contexts.validate_capture(*context)
    assert not gateway._observation_contexts.active


def test_detected_disconnect_permanently_revokes_old_capture_on_reconnect(monkeypatch):
    from alicia_flexible_grasp_supervisor.msg import GraspState
    module,gateway,context,planner,path,params=gateway_fixture(monkeypatch)
    connection=[0];gateway._observation_task_sub=NS(get_num_connections=lambda:connection[0])
    with gateway._observation_context_lock:
        with pytest.raises(ObservationPathError):gateway._require_live_observation_task()
    connection[0]=1
    old=GraspState();old.header.stamp=rospy.Time.from_sec(10.3);old.active=True
    gateway._observation_task_state_cb(old)
    with pytest.raises(ObservationPathError):gateway._observation_contexts.validate_capture(*context)
    assert not gateway._observation_contexts.active


def test_first_admission_waits_for_real_active_callback_not_topic_rpc_order(monkeypatch):
    import threading
    from alicia_flexible_grasp_supervisor.msg import GraspState
    module,gateway,context,planner,path,params=gateway_fixture(monkeypatch)
    gateway._observation_contexts.set_active(False)
    active=GraspState();active.header.stamp=rospy.Time.from_sec(10.4);active.active=True;active.stage=3
    callback=threading.Timer(.02,lambda:gateway._observation_task_state_cb(active))
    callback.start()
    try:
        gateway._wait_for_active_observation_task(max_wait_sec=.5)
    finally:
        callback.join(1.)
    assert gateway._observation_contexts.active


def test_active_admission_wait_is_bounded_and_does_not_invent_authority(monkeypatch):
    module,gateway,context,planner,path,params=gateway_fixture(monkeypatch)
    gateway._observation_contexts.set_active(False)
    with pytest.raises(ObservationPathError):
        gateway._wait_for_active_observation_task(max_wait_sec=.02)
    assert not gateway._observation_contexts.active


@pytest.mark.parametrize('event', ['terminal', 'disconnect'])
def test_revoked_task_requires_real_inactive_then_active_lifecycle(monkeypatch,event):
    from alicia_flexible_grasp_supervisor.msg import GraspState
    module,gateway,context,planner,path,params=gateway_fixture(monkeypatch)
    now=[10.4];monkeypatch.setattr(module.rospy,'get_time',lambda:now[0])
    def callback(stamp,active,stage=3):
        now[0]=stamp
        msg=GraspState();msg.header.stamp=rospy.Time.from_sec(stamp)
        msg.active=active;msg.stage=stage
        gateway._observation_task_state_cb(msg)
    if event=='terminal':
        callback(10.4,True,7)
    else:
        gateway._observation_task_sub=NS(get_num_connections=lambda:0)
        with gateway._observation_context_lock:
            with pytest.raises(ObservationPathError):gateway._require_live_observation_task()
        gateway._observation_task_sub=NS(get_num_connections=lambda:1)
    gateway._committed_observation_plan_cb(rich_plan())
    callback(10.5,True)  # newer active, not merely an old transport replay
    assert not gateway._observation_contexts.active
    with gateway._observation_context_lock:
        with pytest.raises(ObservationPathError):gateway._require_live_observation_task()
    callback(10.6,False,7)  # the real task finally publishes terminal+inactive
    callback(10.7,True)
    gateway._committed_observation_plan_cb(rich_plan())
    with gateway._observation_context_lock:
        gateway._require_live_observation_task()
        new_context=gateway._observation_contexts.capture(target_for(rich_plan()),10.7,120.)
    assert new_context[1]!=context[1]
    with pytest.raises(ObservationPathError):gateway._observation_contexts.validate_capture(*context)


def test_source_expiring_during_real_callback_wait_is_not_readmitted(monkeypatch):
    import threading
    from alicia_flexible_grasp_supervisor.msg import GraspState
    module,gateway,context,planner,path,params=gateway_fixture(monkeypatch)
    gateway._planner_lock=threading.RLock()
    planner.ready=True;planner.strict_execution_max_joint_velocity_rad_s=.08
    gateway._observation_planner=planner;gateway.planner=NS()
    gateway._observation_contexts.set_active(False)
    params['/grasp']['plan_validity_sec']=.5
    now=[10.4];monkeypatch.setattr(module.rospy,'get_time',lambda:now[0])
    def notify_real_active():
        now[0]=10.6
        active=GraspState();active.header.stamp=rospy.Time.from_sec(10.6)
        active.active=True;active.stage=3
        gateway._observation_task_state_cb(active)
    timer=threading.Timer(.02,notify_real_active);timer.start()
    calls=[]
    try:
        result=gateway._with_observation_planner(
            NS(target=target_for(rich_plan()),execute=True),lambda req:calls.append(req))
    finally:
        timer.join(1.)
    assert not result.success and 'stale or future' in result.message
    assert calls==[] and gateway._observation_contexts.active_scene is None
