"""One-view software replay with explicit device and transport simulators.

The real producer, snapshot fuser, geometry, remote acceptance and task gates
consume one analytic RGB-D scene. MoveIt/MuJoCo and controller/SDK/FK providers
are explicit simulators. The unified case starts with a bound observation plan;
initial search, origin capture and physical grasp feasibility are not covered.
"""
from copy import deepcopy
from types import SimpleNamespace as NS
import json
import threading
from collections import deque
from unittest.mock import Mock

import numpy as np
import pytest

from test_unknown_stream import rgb_interest_node, module as producer_module
from test_unknown_pending_identity import publication, consumer_for, deliver
from test_gen26_regular_box_inference_independent import ray_scene


class UnsimulatedExternalCall(BaseException):
    """Escape broad production RPC error handlers; a replay must stay offline."""


@pytest.fixture(autouse=True)
def forbid_external_transport(monkeypatch):
    import socket
    import rospy
    def forbidden(*args, **kwargs):
        raise UnsimulatedExternalCall('unmodeled network/ROS RPC attempted: '+repr(args))
    monkeypatch.setattr(socket.socket, 'connect', forbidden)
    monkeypatch.setattr(socket.socket, 'connect_ex', forbidden)
    monkeypatch.setattr(rospy, 'ServiceProxy', forbidden)
    monkeypatch.setattr(rospy, 'wait_for_service', forbidden)
    monkeypatch.setattr(rospy, 'set_param', forbidden)


def same_scene_producer(monkeypatch, params_override=None):
    scene = ray_scene(width=.030, length=.065)
    producer, clock, _, joint, params = rgb_interest_node(monkeypatch)
    params['/camera'].update({key: getattr(scene.K, key)
        for key in ('fx', 'fy', 'cx', 'cy', 'width', 'height')})
    params.update(deepcopy(params_override or {}))
    optical_to_link = np.array([[0., 0., 1.], [-1., 0., 0.], [0., -1., 0.]])
    T = scene.T.copy(); T[:3, :3] = scene.T[:3, :3] @ optical_to_link.T
    monkeypatch.setattr(producer_module, 'transform_matrix',
        lambda args: T.copy() if args[0] == 'base_link' else np.eye(4))
    rgb = np.full(scene.depth.shape+(3,), (60, 90, 190), np.uint8)
    rgb[scene.mask != 0] = (20, 20, 20)
    mount = np.eye(4)
    mount[:3, :3] = optical_to_link.T
    mount[2, 3] = -.08
    sources = []
    def feed():
        stamp = clock[0]-.1
        pair = (producer.bridge.cv2_to_imgmsg(rgb, encoding='bgr8'),
                producer.bridge.cv2_to_imgmsg(scene.depth, encoding='16UC1'))
        for message in pair:
            message.header.stamp = producer_module.rospy.Time.from_sec(stamp)
            message.header.frame_id = producer.camera_frame
        joint(stamp-.05); joint(stamp+.05)
        producer.latest = pair
        producer.process(None)
        sources.append(pair)
        return pair
    for _ in range(3):
        feed(); clock[0] += .2
    row = publication(producer)
    assert row['object'].detected, row['status']
    assert producer.tracker.anchor is not None and not producer.tracker.lost
    return NS(scene=scene, producer=producer, clock=clock, joint=joint, params=params,
              feed=feed, sources=sources, rgb=rgb, T_link=T, mount=mount, ready=row)


def test_same_ray_pixels_are_accepted_by_real_producer(monkeypatch):
    c = same_scene_producer(monkeypatch)
    node = consumer_for(c.producer, c.clock, c.params)
    deliver(node, c.ready)
    assert node.latest_object.detected
    assert node.latest_object.header.stamp == c.ready['object'].header.stamp
    assert node._current_stream_target_identity().track_id
    assert c.ready['status']['source_stamp_ns'] == c.sources[-1][0].header.stamp.to_nsec()


def _stamp(ns):
    return producer_module.rospy.Time(*divmod(ns, 10**9))


def _geometry(c, row):
    from alicia_flexible_grasp.vision.object_geometry import estimate_object_geometry
    mask = c.producer.bridge.imgmsg_to_cv2(row['mask'], 'mono8')
    ys, xs = np.nonzero(mask)
    bbox = (int(xs.min()), int(ys.min()), int(xs.max()-xs.min()+1), int(ys.max()-ys.min()+1))
    geometry = estimate_object_geometry(c.scene.depth, c.scene.depth, mask, bbox,
        c.scene.K, .0001, c.scene.T, 'instance_mask', .35, .003, .001,
        60, 60, .004, .20, .20, None)
    assert geometry.ok, (geometry.failure_code, geometry.failure_reason)
    return geometry, mask, bbox


def arrived_same_scene(monkeypatch, order=('status', 'ack', 'object', 'mask'), *, execute_observation=True, params_override=None, perturb=False):
    from alicia_flexible_grasp.vision.target_observation import TargetObservation
    from alicia_flexible_grasp.robot.observation_path_guard import committed_plan_digest
    from alicia_flexible_grasp.grasp.rich_plan_integrity import compute_plan_id
    from test_remote_grasp6d_streaming import bound_surface_plan
    from test_task_modes import module as task_module, base as task_base
    from test_remote_modes import Publisher
    from std_msgs.msg import String
    c = same_scene_producer(monkeypatch, params_override)
    remote = consumer_for(c.producer, c.clock, c.params)
    remote._ros_source_clock_ns = lambda: producer_module.rospy.Time.now().to_nsec()
    deliver(remote, c.ready)
    identity = remote._current_stream_target_identity()
    geometry, mask, bbox = _geometry(c, c.ready)
    source_ns = c.ready['object'].header.stamp.to_nsec()
    observation = TargetObservation(identity, source_ns, 'base_link',
        geometry.object_points_base, geometry.support_normal_base, geometry.support_offset_m,
        bbox, mask.shape, min(bbox[0], bbox[1]), 'instance_mask', 'unknown_small')
    origin = bound_surface_plan(observation)
    origin.model_choice = 'unknown_tabletop'; origin.diagnostic = 'FAR_FIELD_OBSERVATION_PLAN'
    origin.contact_geometry_evidence = ''
    origin.candidate_source = 'measured_observation'
    origin.candidate_source_lineage = ['measured_observation']
    origin.has_candidate_model_width = False
    origin.candidate_width_m = origin.required_open_width_m = 0.
    from geometry_msgs.msg import Pose
    from test_remote_grasp6d_streaming import remote_node as original_remote
    origin.valid = True; origin.header.frame_id = 'base_link'
    origin.poses = [Pose()]
    tool = c.T_link @ np.linalg.inv(c.mount)
    from tf.transformations import quaternion_from_matrix
    for key, value in zip('xyz', tool[:3, 3]): setattr(origin.poses[0].position, key, value)
    for key, value in zip('xyzw', quaternion_from_matrix(tool)): setattr(origin.poses[0].orientation, key, value)
    origin.object_geometry = original_remote.geometry_estimate_to_message(geometry,
        stamp=origin.header.stamp, target_identity=identity)
    origin.plan_id = compute_plan_id(origin)
    session = dict(version=2, session_id='one-ray-station', selection=deepcopy(remote._mode_selection),
        phase='observing', stamp_ns=remote._ros_source_clock_ns(), target_epoch=identity.epoch,
        target_track_id=identity.track_id, origin_plan_id=origin.plan_id,
        origin_snapshot_stamp_ns=source_ns, view_plan_id=origin.plan_id, view_source_stamp_ns=source_ns,
        view_index=1, max_observations=1, deadline_ns=remote._ros_source_clock_ns()+60_000_000_000,
        return_verified_stamp_ns=0, view_request_id='initial:'+origin.plan_id,
        previous_gain_sha256='0'*64)
    remote._observe_return_prefix = deepcopy(origin)
    remote._observe_return_origin_prefix = deepcopy(origin)
    remote._observe_return_phase = 'observing'; remote._observe_return_paused = True
    remote._observe_return_saved = object(); remote._observe_return_session = deepcopy(session)
    remote._measurement_remote_ack_pub = Publisher()
    remote._measurement_remote_epoch_ns = c.producer._rgb_interest_epoch
    task = task_module.ModeAwareGraspTaskNode.__new__(task_module.ModeAwareGraspTaskNode)
    task._initialize_mode_boundary()
    task._mode_selection = deepcopy(session['selection']); task._mode_bound_selection = deepcopy(session['selection'])
    task._mode_ready = True; task.active = True; task.stage = 3
    task._observe_return_session = deepcopy(session)
    task._observe_return_origin_plan = deepcopy(origin)
    task._observe_return_origin_proof = dict(epoch_ns=c.producer._rgb_interest_epoch)
    task._following_evidence_lock = threading.RLock(); task._following_epoch_ns = c.producer._rgb_interest_epoch
    task._following_accepted_history = deque(maxlen=128); task._following_sdk_history = deque(maxlen=128)
    task.latest_obj = deepcopy(c.ready['object']); task.latest_obj_time = task.latest_obj.header.stamp
    task.latest_target_geometry = None
    task._near_field_active = False; task._near_field_phase_id = 0
    task._freeze_execution_plan(origin)
    task._near_field_phase_started_sec = task._near_field_phase_deadline_sec = 0.
    states, phases, requests = [], [], []
    c.injected_events=[]
    task.pub = NS(publish=lambda msg: states.append(deepcopy(msg)))
    task.near_field_pub = NS(publish=lambda msg: None)
    task.near_field_phase_pub = NS(publish=lambda msg: phases.append(deepcopy(msg)))
    c.producer._observation_measurement_ack_pub = Mock()
    def forward_acks():
        for raw in (c.producer._observation_measurement_ack_pub.publish.call_args.args[0],
                    remote._measurement_remote_ack_pub.messages[-1]):
            task._observation_measurement_ack_cb(deepcopy(raw))
    def publish(message):
        req = json.loads(message.data); requests.append(deepcopy(req))
        c.producer._observation_measurement_request_cb(message)
        remote._measurement_remote_request_cb(message)
        pending_acks=[]
        if req['state'] == 'requalifying':
            pending_acks=[deepcopy(c.producer._observation_measurement_ack_pub.publish.call_args.args[0]),
                deepcopy(remote._measurement_remote_ack_pub.messages[-1])]
            for _ in range(3):
                if hasattr(c,'advance'):c.advance(.2)
                else:c.clock[0]+=.2
                c.feed()
            row = publication(c.producer)
            row['ack'] = json.loads(c.producer._observation_measurement_ack_pub.publish.call_args.args[0].data)
            for channel in order:
                if channel == 'ack': remote._measurement_remote_ack_cb(String(data=json.dumps(row['ack'])))
                else: deliver(remote, row, (channel,))
            assert row['ack']['qualified'], row['status']
            task.obj_cb(deepcopy(row['object']))
            c.arrival_row = row
        forward_acks()
        if perturb and pending_acks:
            for pending in pending_acks:
                task._observation_measurement_ack_cb(pending)
            forward_acks()
            c.injected_events.append('qualified_then_late_pending_and_duplicate_ack')
    task._observation_measurement_pub = NS(publish=publish)
    c.clock[0] += .01
    cfg = dict(near_field_replan_enabled=True, near_field_replan_required=True,
        near_field_strategy='single_snapshot_direct', near_field_replan_timeout_sec=30.,
        plan_validity_sec=5., target_max_drift_m=.02,
        target_observation_validity_sec=1.5, unknown_observe_return_max_observations=1)
    # Actuator and arrival feedback are explicit simulation boundaries. The
    # original task binds/validates/submits the observation and owns the entire
    # begin -> settle -> finish handshake; calling begin is not counted as motion.
    motion = []
    def plan_motion(pose, execute):
        assert execute is False
        return NS(success=True, message='inert MoveIt boundary')
    def submit_motion(pose, execute):
        assert execute is True
        motion.append(deepcopy(pose))
        return NS(success=True, message='inert controller boundary')
    def settle(label):
        c.clock[0] += .4
        now_ns = producer_module.rospy.Time.now().to_nsec()
        for delta in np.linspace(.35, 0., 8): c.joint(c.clock[0]-float(delta))
        task._last_motion_settle_sample = dict(epoch_ns=c.producer._rgb_interest_epoch,
            stationary_window_start_ns=now_ns-350_000_000, stationary_window_end_ns=now_ns)
        return True
    task._position_only_execute_globally_enabled = lambda: False
    task._wait_for_motion_settle = settle
    c.motion = motion
    c.remote, c.task, c.identity, c.origin, c.session = remote, task, identity, origin, session
    c.initial_observation = observation
    c.states, c.phases, c.requests, c.cfg = states, phases, requests, cfg
    c.plan_motion, c.submit_motion, c.settle = plan_motion, submit_motion, settle
    if execute_observation:
        assert task._plan_and_execute_pose(task_base.GraspStages.MOVE_PREGRASP,
            '6D pregrasp', task_base.observation_plan_pose(origin), plan_motion,
            '6D pregrasp', execution_plan_id=origin.plan_id, execution_plan=origin,
            gcfg=cfg, execute_pose=submit_motion), states
    return c


def test_real_one_scene_measurement_reaches_task_contact_phase(monkeypatch):
    c = arrived_same_scene(monkeypatch)
    assert [r['state'] for r in c.requests] == ['executing', 'requalifying', 'qualified']
    sources = c.requests[-1]['source_stamps_ns']
    assert sources == [pair[0].header.stamp.to_nsec() for pair in c.sources[-3:]]
    assert c.task.latest_obj.header.stamp.to_nsec() == sources[-1]
    assert c.remote.latest_object.header.stamp.to_nsec() == sources[-1]
    assert len(c.phases) == 1 and c.phases[0].planning_stage == 'contact_replan'
    assert len(c.motion) == 1
    assert c.task._observe_return_session['max_observations'] == 1


def station_samples(c):
    """Real reached-phase callback, then new pixels at that same station."""
    from sensor_msgs.msg import JointState
    from alicia_flexible_grasp.vision.target_observation import TargetObservation
    from alicia_flexible_grasp.vision.multiview_surface import surface_view_from_observation, fused_surface_from_view
    from test_task_modes import base as task_base
    n, p, task = c.remote, c.producer, c.task
    geometry, mask, bbox = _geometry(c, c.arrival_row)
    ns = c.arrival_row['object'].header.stamp.to_nsec()
    reached = TargetObservation(c.identity, ns, 'base_link', geometry.object_points_base,
        geometry.support_normal_base, geometry.support_offset_m, bbox, mask.shape,
        min(bbox[0],bbox[1]), 'instance_mask', 'unknown_small')
    initial = surface_view_from_observation(c.initial_observation)
    n._observe_return_saved = (fused_surface_from_view(initial), initial)
    n._observe_return_initial_stamp = c.initial_observation.stamp_ns
    n._observe_return_view_uncertainty_m = .001
    n._latest_target_observation = reached
    n.near_field_state_cb(c.phases[-1])
    assert n._observe_return_phase == 'observing_contact'
    assert n._observe_return_reached_contact_context['source_stamp_ns'] == ns
    assert not n._observe_return_paused, n.latest_preview_rich_plan
    # Same mode policy as _stage_runtime_params: visibility remains diagnostic
    # after the qualified current-view transition (occlusion is expected).
    n.camera_visibility_gate_enabled = not n._direct_strategy_selected()
    n.camera_visibility_diagnostic_enabled = True
    p._station_compute_phase_cb(c.phases[-1])
    task.set_state(task_base.GraspStages.PLAN_PREGRASP, 'waiting for same-station contact computation')
    p._station_compute_state_cb(c.states[-1]); n.grasp_state_cb(c.states[-1])
    n.near_field_planning_snapshot_frames=3; n.planning_snapshot_frames=5
    n.planning_snapshot_timeout_sec=0.; n.planning_snapshot_max_age_sec=.35
    n.planning_snapshot_max_span_sec=12.; n.planning_snapshot_max_inference_latency_sec=3.5
    n.planning_mask_min_iou=.85; n.planning_mask_max_centroid_shift_px=5.
    n.planning_max_joint_delta_rad=.01; n.mask_erosion_px=0
    n.depth_mad_scale=3.5; n.depth_mad_absolute_floor_m=.002
    n.mask_internal_hole_max_area_px=0; n.rate_hz=3.
    n.target_projection_frame_convention='ros_camera_link'
    n.inference_coordinator._result_max_age_sec=15.
    n.frames.update_joints(np.zeros(6))
    c.station_rows=[]
    for i in range(3):
        interval=.2 if i==0 else getattr(c,'station_interval_sec',.2)
        if hasattr(c,'advance'):c.advance(interval)
        else:c.clock[0]+=interval
        pair=c.feed(); row=publication(p)
        assert 'station_compute_wait' in row['status'], row['status']
        for stamp,q in p._rgb_interest_feedback:
            msg=JointState();msg.header.stamp=_stamp(stamp);msg.header.frame_id='sdk_measured'
            msg.name=['Joint%d'%j for j in range(1,7)]+['right_finger']
            msg.position=list(q)+[.05]
            n.latest_joint_state=deepcopy(msg); n._station_compute_accepted_cb(msg)
            previous=getattr(task,'latest_accepted_joint_state',None)
            if previous is None or msg.header.stamp>previous.header.stamp:
                task.accepted_joint_cb(msg)
        n.frames.update_color(c.rgb,pair[0].header.stamp,pair[0].header.frame_id)
        n.frames.update_depth(c.scene.depth,pair[1].header.stamp,pair[1].header.frame_id)
        deliver(n,row)
        task._station_compute_task_mask_cb(deepcopy(row['mask']))
        task.obj_cb(deepcopy(row['object']))
        task._unknown_support_status_cb(NS(data=json.dumps(row['status'])))
        assert n._station_compute_reference is not None
        assert task._station_task_record is not None
        c.station_rows.append(row)
        admitted=n._poll_direct_snapshot()
        if i<2: assert not admitted
    assert admitted,getattr(n,'_direct_snapshot_last_rejection',None)
    c.ticket=n._stream_worker_ticket
    sample_stamps=[row['object'].header.stamp.to_nsec() for row in c.station_rows]
    c.snapshot_timing=dict(source_stamps_ns=sample_stamps,
        span_sec=(sample_stamps[-1]-sample_stamps[0])*1e-9,
        source_age_at_submission_sec=c.clock[0]-sample_stamps[-1]*1e-9,
        configured_result_max_age_sec=15.,configured_collection_span_sec=12.,
        configuration_source='src/alicia_flexible_grasp_supervisor/config/grasp_params.yaml')
    assert c.snapshot_timing['span_sec']<=1.000001
    assert 0<=c.snapshot_timing['source_age_at_submission_sec']<=.100001
    assert c.ticket is not None and n._station_compute_work_certificate(c.ticket)
    c.current_row=row
    return c.ticket


def same_source_contact(c):
    import hashlib
    import test_remote_grasp6d_streaming as f
    from alicia_flexible_grasp.vision.target_observation import TargetObservation
    from alicia_flexible_grasp.vision.rgbd_snapshot import FrozenRgbdSample
    from alicia_flexible_grasp.vision.multiview_surface import surface_view_from_observation, fused_surface_from_view
    from alicia_flexible_grasp.grasp.rich_plan_integrity import compute_plan_id
    node = c.remote
    node.tabletop_geometry_enabled = True
    node.tabletop_geometry_config = f.TabletopGeometryConfig(max_candidates=16)
    node.gripper_geometry = f.tabletop_gripper()
    node.gripper_tool_jaw_axis, node.gripper_tool_finger_length_axis = 'y', 'z'
    node.gripper_physical_open_width_m = .05
    node.grasp_config = dict(tool_approach_axis='z', pregrasp_distance_m=.08,
                            final_approach_offset_m=.015, lift_height_m=.05)
    node.candidate_min_downward_approach_cos = .65
    node.candidate_max_final_approach_lateral_m = .010
    node.soft_score_weights = f.SoftScoreWeights()
    node.target_absolute_sanity_distance_m = .15
    node.camera_visibility_gate_enabled = False
    node.camera_visibility_diagnostic_enabled = False
    f.configure_identity_handeye(node)
    from tf.transformations import quaternion_from_matrix
    node.handeye_translation_xyz = tuple(c.mount[:3,3])
    node.handeye_rotation_xyzw = tuple(quaternion_from_matrix(c.mount))
    node._cached_tool_from_camera = None
    node.camera_visibility_gate_enabled = True
    node.camera_visibility_diagnostic_enabled = True
    node.handeye_parent_frame='tool0';node.handeye_camera_frame='camera_link'
    node.camera_depth_scale=c.params['/camera']['depth_scale']
    c.scene.K.depth_scale=node.camera_depth_scale
    def snapshot_tf(base_frame,source_frame,stamp,timeout):
        from geometry_msgs.msg import TransformStamped
        assert base_frame in ('base_link','tool0') and source_frame=='camera_link'
        matrix=c.T_link if base_frame=='base_link' else c.mount
        assert stamp.to_nsec() in [pair[0].header.stamp.to_nsec() for pair in c.sources]
        msg=TransformStamped();msg.header.stamp=stamp;msg.header.frame_id=base_frame
        for key,value in zip('xyz',matrix[:3,3]):setattr(msg.transform.translation,key,value)
        for key,value in zip('xyzw',quaternion_from_matrix(matrix)):setattr(msg.transform.rotation,key,value)
        return msg
    node.tf_buffer=NS(lookup_transform=snapshot_tf)
    ticket = station_samples(c)
    snapshot=ticket.payload[0]
    ns=snapshot.stamp_ns
    assert snapshot.quality.valid_depth_points>0
    geometry,depth,transform=node._prepare_snapshot_geometry(snapshot,_stamp(ns))
    assert geometry.ok,(geometry.failure_code,geometry.failure_reason)
    assert np.count_nonzero(depth)>0
    assert np.allclose(transform,c.scene.T)
    obs=node._latest_target_observation
    registration=node._latest_registration_evidence.result
    planning = node._prepare_regular_box_snapshot(snapshot, geometry, transform)
    assert node._current_inferred_box() is not None, node._regular_box_snapshot['result']
    prepared = NS(geometry=planning,stamp=_stamp(ns),near_field=True,snapshot=snapshot,target_observation=obs)
    candidates, audit = node._generate_tabletop_candidates(planning, snapshot)
    assert candidates, audit
    c.prepared,c.contact_audit,c.registration=prepared,audit,registration
    return candidates


def test_same_producer_pixels_and_fused_depth_reach_original_geometry(monkeypatch):
    c=arrived_same_scene(monkeypatch)
    candidates=same_source_contact(c)
    assert candidates
    assert c.prepared.snapshot.quality.valid_depth_points>0
    assert c.prepared.geometry.ok and c.registration.ok
    assert c.prepared.target_observation.stamp_ns==c.current_row['object'].header.stamp.to_nsec()
    assert c.remote._current_inferred_box().source['stamp_ns']==c.prepared.target_observation.stamp_ns
    assert c.current_row['object'].header.stamp.to_nsec()>c.requests[-1]['source_stamps_ns'][-1]


def accepted_same_scene(c, monkeypatch, tmp_path):
    """Run real remote acceptance; only ROS planning services are simulated."""
    import test_remote_grasp6d_streaming as f
    from test_remote_modes import Publisher, base
    from alicia_flexible_grasp.grasp.contact_execution import plan_contact_evaluator
    from alicia_flexible_grasp.grasp.rich_plan_integrity import pose_values
    from tf.transformations import quaternion_matrix
    n=c.remote
    source=c.prepared
    n.candidate_frame_convention=base.PRODUCTION_CANDIDATE_FRAME_CONVENTION
    n.hybrid_merge_config=base.load_tabletop_geometry_config({},n.gripper_geometry)[2]
    estimator=base.FrozenSnapshotCandidatePoseEstimator(c.scene.T, source.stamp,
        source.snapshot.frame_id,raw_candidate_convention=n.candidate_frame_convention)
    candidates, audit=n._generate_tabletop_candidates(source.geometry,source.snapshot)
    prepared=base.PreparedPrediction(ticket=c.ticket,snapshot=source.snapshot,
        stamp=source.stamp,geometry=source.geometry,pose_estimator=estimator,
        graspnet_input=None,candidates=(),tabletop_candidates=candidates,
        remote_diagnostics={'tabletop_geometry':audit},remote_performance={},
        request_invalidation_generation=n._capture_geometry_generation(),
        graspnet_input_config=c.ticket.payload[1],model_choice='unknown_tabletop',
        near_field=True,near_field_phase_id=n._near_field_phase_id,
        near_field_phase_deadline_sec=n._near_field_phase_deadline_sec,
        target_observation=source.target_observation,
        refinement_binding=getattr(n,'_near_field_refinement_binding',None))
    n.gate_audit_enabled=True
    n.gate_audit_output_path=str(tmp_path/'gate_audit.json')
    n.mujoco_audit_output_path=str(tmp_path/'simulation.json')
    n.gate_audit_pub=Publisher()
    n.contact_sequence_precheck_enabled=True
    n.grasp_config['unknown_lift_after_close_enabled']=False
    calls=[]
    # Simulated physics is an explicit RPC boundary too. No HTTP transport is
    # permitted in this replay; its response is not evidence of a real grasp.
    from test_mujoco_digital_twin_client import _passing_response
    def simulated_twin(payload):
        calls.append(('inert_mujoco', deepcopy(payload)))
        response=_passing_response(payload['plan_id'])
        response['candidate_source']=payload['candidate_source']
        response['candidate_source_lineage']=payload['candidate_source_lineage']
        return response
    n._mujoco_client_factory=lambda *a,**kw:NS(simulate_grasp=simulated_twin)
    def proxy(name, service_type):
        if name == '/compute_ik':
            def unavailable(*args, **kwargs):
                raise RuntimeError('offline advisory IK provider unavailable')
            return unavailable
        if name == '/grasp_6d/request_plan':
            return lambda enabled: n.request_plan_cb(NS(trigger=enabled))
        if name not in ('/supervisor/check_pose_sequence_strict','/supervisor/check_contact_sequence',
                        '/supervisor/check_pose_strict'):
            raise UnsimulatedExternalCall('unmodeled ROS service '+name)
        def check(*args):
            calls.append((name,deepcopy(args)))
            if name.endswith('check_contact_sequence'):
                request=args[0];plan=request.plan
                opening=plan.required_open_width_m
                evaluator=plan_contact_evaluator(plan,{},opening)
                values=pose_values(plan.poses[2]);T=quaternion_matrix(values[3:]);T[:3,3]=values[:3]
                geometry=evaluator(T)
                assert geometry['ok'],geometry
                evidence=dict(plan_id=plan.plan_id,success=True,
                    scope='read_only_candidate_constant_bias_prediction',execution_authorized=False,
                    checked_stage_names=['pregrasp','approach','grasp'],
                    contact_prediction=dict(scope='constant_current_bias_prediction',
                        future_following_verified=False,execution_authorized=False,
                        nominal_geometry=geometry,predicted_command_geometry=deepcopy(geometry)))
                return NS(success=True,message='inert MoveIt and zero-bias SDK provider',
                    evidence_json=json.dumps(evidence),joint_path_cost=.1,joint_max_delta=.05,
                    failure_code='',failed_stage='')
            return NS(success=True,message='inert MoveIt boundary joint_path_cost=0.1 joint_max_delta=0.05',
                joint_path_cost=.1,joint_max_delta=.05,failure_code='',failed_stage='')
        return check
    monkeypatch.setattr(base.rospy,'wait_for_service',lambda *a,**kw:None)
    monkeypatch.setattr(base.rospy,'ServiceProxy',proxy)
    decision=n.inference_coordinator.complete(c.ticket,now_sec=c.clock[0],target_epoch=n.target_instance_epoch)
    assert decision.accepted,decision
    result=n._accept_prediction(prepared)
    if not n.latest_preview_rich_plan.valid:
        (tmp_path/'acceptance_failure.json').write_text(json.dumps(dict(result=result,
            runtimes={str(k):str(v) for k,v in n._stable_variant_runtime.items()},calls=calls),default=str,indent=2))
    assert n.latest_preview_rich_plan.valid,(result,getattr(n,'_active_gate_audit_report',None))
    c.accepted_prepared=prepared;c.acceptance=result;c.planning_calls=calls
    c.contact=deepcopy(n.latest_preview_rich_plan)
    c.remote_report=deepcopy(n._active_gate_audit_report)
    assert c.remote_report['plan_id']==c.contact.plan_id
    assert c.remote_report['observe_return']['phase']=='observation_contact'
    return c.contact


def test_same_scene_real_acceptance_publishes_original_contact_audit(monkeypatch,tmp_path):
    c=arrived_same_scene(monkeypatch)
    same_source_contact(c)
    plan=accepted_same_scene(c,monkeypatch,tmp_path)
    from alicia_flexible_grasp.grasp.contact_evidence import decode_contact_evidence
    evidence=decode_contact_evidence(plan,required=True)
    assert evidence['measurement_source']=='regular_box_model_inference'
    assert evidence['box_model_evidence']['source']['stamp_ns']==c.current_row['object'].header.stamp.to_nsec()
    assert plan.fused_view_count==3
    assert len(c.motion)==1
    assert c.planning_calls


def frozen_same_scene(c, monkeypatch):
    """Original published-file audit and task callback/admission/freeze chain."""
    from test_remote_modes import Publisher
    n,t=c.remote,c.task
    for message in n.gate_audit_pub.messages:
        t._observe_return_audit_cb(deepcopy(message))
    assert c.contact.plan_id in t._observe_return_reports
    t.grasp6d_preview_plan_cb(deepcopy(c.contact))
    t._frozen_contact_pub=Publisher()
    proof=t._station_contact_provenance(c.contact)
    assert proof.ok,proof
    copied,candidate=t._copy_near_field_preview_candidate(c.origin,
        c.origin.header.stamp.to_nsec()+1,c.cfg)
    assert copied.ok,copied
    frozen=t._maybe_rebind_near_field_grasp6d_plan(c.cfg,{},c.origin)
    assert frozen is not None,c.states
    assert frozen.plan_id==c.contact.plan_id!=c.origin.plan_id
    assert len(t._frozen_contact_pub.messages)==1
    c.frozen=frozen
    return frozen


def test_same_scene_original_task_admits_and_freezes_contact(monkeypatch,tmp_path):
    c=arrived_same_scene(monkeypatch)
    same_source_contact(c);accepted_same_scene(c,monkeypatch,tmp_path)
    frozen_same_scene(c,monkeypatch)
    assert len(c.motion)==1


def unified_replay(monkeypatch, tmp_path, order=('status','ack','object','mask'), *, perturb=False, fault=None):
    """Original task executor drives one observation, contact handoff and close.

    The seed observation plan/session is fixture input. Device services, FK and
    encoder stream are deterministic simulators; production task/admission/
    geometry consumers run with the explicitly configured fixture policy.
    """
    from test_task_modes import base
    from test_remote_modes import Publisher
    from alicia_flexible_grasp.grasp.rich_plan_integrity import pose_values
    from sensor_msgs.msg import JointState
    from geometry_msgs.msg import TransformStamped, PoseStamped
    from tf.transformations import quaternion_matrix
    import hashlib
    initial_params={'/robot_description':'<robot name="offline_simulator"/>',
        '/robot':{'strict_execution_controller_sync_max_feedback_age_sec':.5},
        '/handeye/parent_frame':'tool0','/grasp_6d/remote/gripper_geometry':{},
        '/tactile/simulate':True}
    c=arrived_same_scene(monkeypatch,order,execute_observation=False,params_override=initial_params,perturb=perturb)
    t,n,p=c.task,c.remote,c.producer
    if perturb:c.station_interval_sec=.5
    cfg=dict(c.cfg, final_visual_refine_enabled=False,
        unknown_lift_after_close_enabled=False, measured_endpoint_check_enabled=True,
        contact_endpoint_precision_enabled=True,pregrasp_cartesian_compensation_enabled=True,
        unknown_contact_cartesian_compensation_enabled=True)
    c.cfg=cfg
    c.params['/tactile/simulate']=True
    c.params['/robot_description']='<robot name="offline_simulator"/>'
    c.params['/robot']={'strict_execution_controller_sync_max_feedback_age_sec':.5}
    c.params['/handeye/parent_frame']='tool0'
    c.params['/grasp_6d/remote/gripper_geometry']={}
    c.device_calls=[];c.station_requests=[];c.motion_stamps=[]
    def set_runtime_parameter(name,value):
        if name!='/grasp_6d/runtime_execution_error':
            raise UnsimulatedExternalCall('unmodeled parameter write '+name)
        c.params[name]=deepcopy(value)
    monkeypatch.setattr(base.rospy,'set_param',set_runtime_parameter)
    original_wait=t._observe_return_condition.wait
    def condition_wait(timeout=None):
        c.clock[0]+=max(.001,float(timeout or .01))
        return original_wait(0.)
    monkeypatch.setattr(t._observe_return_condition,'wait',condition_wait)
    actual=deepcopy(base.observation_plan_pose(c.origin))
    q=[0.]*6;opening=[.05];transforms={}
    def register_pose(pose):
        nonlocal actual,q
        actual=deepcopy(pose)
        v=pose_values(actual.pose)
        # A unique simulated encoder cell for each commanded pose. FK rejects
        # any unknown vector, rather than returning one pose for arbitrary q.
        T=quaternion_matrix(v[3:]);T[:3,3]=v[:3]
        key=tuple(v[:3])+tuple(v[3:6]);q=list(key);transforms[key]=T
    register_pose(actual)
    # Source producer/registration station uses q=0 at the observed view.
    transforms[tuple([0.]*6)]=transforms[tuple(q)].copy();q=[0.]*6
    class SimulatedFk:
        def __init__(self,description,names):
            self.model_sha256=hashlib.sha256(description.encode()).hexdigest()
        def __call__(self,joints):
            return transforms[tuple(joints)].copy()
    monkeypatch.setattr(base,'SerialUrdfFk',SimulatedFk)
    last_feedback=[-1]
    def feedback():
        stamp_ns=base.rospy.Time.now().to_nsec()
        if stamp_ns<=last_feedback[0]:return t.latest_accepted_joint_state
        last_feedback[0]=stamp_ns
        msg=JointState();msg.header.frame_id='sdk_measured';msg.header.stamp=base.rospy.Time.now()
        msg.name=['Joint%d'%j for j in range(1,7)]+['right_finger'];msg.position=q+[opening[0]]
        t.accepted_joint_cb(msg);t.joint_cb(msg)
        sdk=deepcopy(msg);sdk.header.frame_id='sdk_transmitted'
        sdk.name=sdk.name[:6];sdk.position=sdk.position[:6];t.following_sdk_cb(sdk)
        actual.header.stamp=msg.header.stamp;actual.header.frame_id='base_link'
        return msg
    def transform(*args):
        result=TransformStamped();result.header=deepcopy(actual.header)
        result.transform.translation.x=actual.pose.position.x
        result.transform.translation.y=actual.pose.position.y
        result.transform.translation.z=actual.pose.position.z
        result.transform.rotation=deepcopy(actual.pose.orientation)
        return result
    t.tf_buffer=NS(lookup_transform=transform)
    def sleep(duration):
        c.clock[0]+=float(duration);feedback();c.joint(c.clock[0],q)
    monkeypatch.setattr(base.rospy,'sleep',sleep)
    def advance(duration):
        steps=max(1,round(duration/.05))
        for _ in range(steps):sleep(duration/steps)
    c.advance=advance
    del t._wait_for_motion_settle  # Use the original accepted stationary-window gate.
    feedback()
    original_feed=c.feed
    def feed_with_encoder():
        feedback()
        return original_feed()
    c.feed=feed_with_encoder
    old_publish=t.pub.publish
    def state(msg):
        old_publish(msg)
        p._station_compute_state_cb(msg);n.grasp_state_cb(msg)
    t.pub=NS(publish=state)
    t._frozen_contact_pub=Publisher()
    p._contact_station_ack_pub=Publisher();n._contact_station_ack_pub=Publisher()
    t._contact_station_acks={}
    def retirement(message):
        c.station_requests.append(json.loads(message.data))
        for actor in (p,n):
            actor._contact_station_request_cb(deepcopy(message))
            t._contact_station_ack_cb(deepcopy(actor._contact_station_ack_pub.messages[-1]))
    t._contact_station_pub=NS(publish=retirement)
    def motion(pose,execute):
        if execute:
            c.clock[0]+=.001
            c.motion_stamps.append(base.rospy.Time.now().to_nsec())
            c.device_calls.append(('motion',t._bound_execution_plan_id,deepcopy(pose)))
            if t._bound_execution_plan_id==c.origin.plan_id:
                # This is the observation simulator's original zero joint cell.
                actual.pose=deepcopy(pose.pose)
            else:register_pose(pose)
            feedback()
        return NS(success=True,message='inert device boundary')
    def gripper(value):
        c.device_calls.append(('gripper',t._bound_execution_plan_id,float(value)))
        opening[0]=float(value);feedback()
        return NS(success=True,message='inert gripper boundary')
    planning_proxy=[None]
    def proxy(name,kind):
        if name=='/grasp_6d/request_plan':
            def stream(enabled):
                if enabled and not hasattr(c,'contact'):
                    same_source_contact(c);accepted_same_scene(c,monkeypatch,tmp_path)
                    planning_proxy[0]=base.rospy.ServiceProxy
                    monkeypatch.setattr(base.rospy,'ServiceProxy',proxy)
                    for message in n.gate_audit_pub.messages:t._observe_return_audit_cb(deepcopy(message))
                    t.grasp6d_preview_plan_cb(deepcopy(c.contact))
                    if perturb:
                        from test_gen35_task_pending_handoff import historical_pending
                        parent=c.station_rows[0]['status']['station_compute_wait']
                        pending=historical_pending(NS(plan=c.contact),parent)
                        assert pending.header.stamp<c.contact.header.stamp
                        t.grasp6d_preview_plan_cb(pending)
                        assert t._station_contact_proposal==c.contact
                        t.grasp6d_preview_plan_cb(deepcopy(c.contact))
                        for message in n.gate_audit_pub.messages:t._observe_return_audit_cb(deepcopy(message))
                        duplicate=t.start_cb(NS(execute=True,plan_id=c.origin.plan_id))
                        assert duplicate.success is False
                        proof=t._station_contact_provenance(c.contact)
                        assert proof.ok,proof
                        copied,_=t._copy_near_field_preview_candidate(c.origin,c.origin.header.stamp.to_nsec()+1,cfg)
                        assert copied.ok,copied
                        c.injected_events+=['late_ancestor_clock_preview','duplicate_preview_audit','duplicate_start_rejected']
                    if fault=='same_source_conflict':
                        changed=deepcopy(c.current_row['object'])
                        changed.pose_base.pose.position.x+=.01
                        t.obj_cb(changed)
                        c.injected_events.append('same_source_geometry_conflict')
                return n.request_plan_cb(NS(trigger=enabled))
            return stream
        moves={'/supervisor/move_contact_approach':1,'/supervisor/move_contact_grasp_pose':2}
        comps={'/supervisor/compensate_pregrasp':0,'/supervisor/compensate_approach':1,
               '/supervisor/compensate_grasp_pose':2}
        if name in moves:
            def move(plan,execute):
                pose=PoseStamped();pose.header=deepcopy(plan.header);pose.pose=deepcopy(plan.poses[moves[name]])
                return motion(pose,execute)
            return move
        if name in comps:
            def compensate(plan,execute):
                assert execute is True
                i=comps[name];feedback()
                report=dict(code='PREGRASP_MEASURED_CONVERGED',plan_id=plan.plan_id,
                    fixed_goal_pose_index=i,fixed_goal_pose=list(pose_values(plan.poses[i])),
                    epoch_ns=t._following_epoch_ns,accepted_stamp_ns=t.latest_accepted_joint_state.header.stamp.to_nsec(),
                    model_sha256=SimulatedFk(c.params['/robot_description'],()).model_sha256)
                c.device_calls.append(('endpoint_provider',plan.plan_id,i))
                return NS(success=True,message=json.dumps(report))
            return compensate
        if name=='/alicia_d_driver_node/set_task_endpoint_precision':
            return lambda enabled:NS(success=True,message='inert precision lease')
        if planning_proxy[0]:return planning_proxy[0](name,kind)
        raise UnsimulatedExternalCall('unmodeled ROS service '+name)
    monkeypatch.setattr(base.rospy,'ServiceProxy',proxy)
    monkeypatch.setattr(base.rospy,'wait_for_service',lambda *a,**kw:None)
    result=base.GraspTaskNode._execute_grasp6d_plan(t,cfg,dict(open_position_m=.05,use_compliant_close=False,
        close_wait_sec=0.,open_wait_sec=0.),.05,motion,motion,gripper,None,c.origin,
        strict_execute_pose=motion)
    (tmp_path/'unified_result.json').write_text(json.dumps(dict(result=result,
        states=[str(s) for s in c.states],device_calls=c.device_calls,
        station_requests=c.station_requests,
        injected_events=c.injected_events,
        source_timing=getattr(c,'snapshot_timing',None),
        snapshot_valid_depth_points=(c.ticket.payload[0].quality.valid_depth_points if hasattr(c,'ticket') else None),
        latest_settle=getattr(t,'_last_motion_settle_sample',None),
        measured_contact=getattr(t,'_last_measured_contact_sample',None),
        observation_measurement_requests=c.requests,
        source_plan_ids=dict(observation=c.origin.plan_id,contact=getattr(getattr(c,'contact',None),'plan_id',None)),
        motion_stamps=c.motion_stamps),default=str,indent=2))
    c.execution_result=result
    return c


def test_unified_original_task_reaches_unverified_hold(monkeypatch,tmp_path):
    c=unified_replay(monkeypatch,tmp_path)
    assert c.states[-1].state=='HOLDING',c.states[-1]
    assert 'GRASP_HOLD_UNVERIFIED' in c.states[-1].message
    assert c.execution_result is False
    assert [r['state'] for r in c.station_requests]==['prepare','retire']
    assert sum(x[0]=='motion' and x[1]==c.origin.plan_id for x in c.device_calls)==1
    assert sum(x[0]=='motion' and x[1]==c.contact.plan_id for x in c.device_calls)==3
    assert len(c.task._frozen_contact_pub.messages)==1
    assert c.task._last_measured_contact_sample['ok'] is True
    settled=c.task._last_motion_settle_sample
    assert settled['accepted_sample_count']>=3
    assert settled['stationary_window_start_ns']>=c.motion_stamps[-1]
    assert settled['epoch_ns']==c.task._following_epoch_ns


@pytest.mark.parametrize('order', [('mask','object','ack','status'),('ack','status','mask','object')])
def test_unified_delay_reorder_duplicate_still_submits_one_observation(monkeypatch,tmp_path,order):
    c=unified_replay(monkeypatch,tmp_path,order,perturb=True)
    assert c.states[-1].state=='HOLDING',c.states[-1]
    assert 'GRASP_HOLD_UNVERIFIED' in c.states[-1].message
    assert sum(x[0]=='motion' and x[1]==c.origin.plan_id for x in c.device_calls)==1
    assert sum(x[0]=='motion' and x[1]==c.contact.plan_id for x in c.device_calls)==3
    assert len(c.task._frozen_contact_pub.messages)==1
    assert len(c.phases)==1 and c.session['max_observations']==1
    assert len(c.requests)==3
    assert c.injected_events==['qualified_then_late_pending_and_duplicate_ack',
        'late_ancestor_clock_preview','duplicate_preview_audit','duplicate_start_rejected']


def test_unified_real_source_conflict_blocks_contact_and_close(monkeypatch,tmp_path):
    c=unified_replay(monkeypatch,tmp_path,fault='same_source_conflict')
    assert c.states[-1].state=='FAILED',c.states[-1]
    assert c.task._execution_authority_revoked
    assert sum(x[0]=='motion' for x in c.device_calls)==1
    assert not any(x[0]=='gripper' and x[2]<.05 for x in c.device_calls)
    assert not c.task._frozen_contact_pub.messages
