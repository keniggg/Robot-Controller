#!/usr/bin/env python3
"""Finite RGB-interest view/path preview. Never submits a robot trajectory.

The input is recorded evidence, not a live motion lease. Candidate views use
the recorded model/pose and the strict planning-only RPC may inspect the live
MoveIt scene. Neither result can be reused as execution authority.
"""
import argparse
from copy import deepcopy
from dataclasses import asdict, fields
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import threading
import time
from types import SimpleNamespace

import numpy as np

from alicia_grasp_modes.rgb_observation_volume import RGBObservationVolume, SupportPlaneProvenance
from alicia_grasp_modes.selection import parse_selection
from record_rgb_observation import (bracket_feedback, camera_geometry_snapshot,
    digest, matrix, source_context_matches, Config)
from alicia_flexible_grasp.robot.observation_view_limits import (
    DEFAULT_OBSERVATION_VIEW_PARAMETERS, effective_observation_view_limits)


MOUNT = Path('/media/zhuyupei/SanStand')
ROOT = MOUNT/'实验记录/Robot-Controller/ros_log'
NAMES = tuple('Joint%d' % i for i in range(1, 7))


def read_interest_record(path):
    path = Path(path).resolve()
    document = json.loads(path.read_text())
    candidates = [row for row in document['rows']
                  if row.get('observation_interest_ready') is True
                  and row.get('source_context_unchanged') is True
                  and row.get('fresh_for_existing_frame_limit') is True
                  and row.get('motion_authorized') is False
                  and row.get('contact_geometry_authorized') is False]
    if not candidates:
        raise ValueError('no_accepted_recorded_rgb_interest')
    row = max(candidates, key=lambda value: value['source_stamp_ns'])
    source = Path(row['source_file']).resolve()
    if source.parent != path.parent or hashlib.sha256(source.read_bytes()).hexdigest() != row['source_file_sha256']:
        raise ValueError('recorded_source_artifact_mismatch')
    raw = row['interest_volume']
    arguments = {field.name: raw[field.name] for field in fields(RGBObservationVolume) if field.init}
    arguments['support_provenance'] = SupportPlaneProvenance(**arguments['support_provenance'])
    volume = RGBObservationVolume(**arguments)
    if (volume.source_stamp_ns != row['source_stamp_ns']
            or not np.allclose(volume.vertices_base_m, raw['vertices_base_m'], rtol=0., atol=1e-12)
            or not np.array_equal(volume.T_base_camera, row['T_base_camera'])):
        raise ValueError('recorded_interest_binding_mismatch')
    context = row['source_context']
    selection = parse_selection(row['selection'])
    track = row['rgb_track']
    limits = effective_observation_view_limits(context['observation_view_parameters'])
    recorded_config = Config(**{key:value for key,value in context['perception'].items()
                               if key in Config.__dataclass_fields__})
    if (digest(context) != row['source_id']
            or parse_selection(context['selection']) != selection
            or context['epoch_ns'] != row['epoch_ns']
            or not row['source_stamp_ns'] > max(selection['stamp_ns'], row['epoch_ns'])
            or not np.array_equal(context['tool_camera'], context['current_tool_camera'])
            or Path(volume.support_provenance.source_record_id).resolve() != source
            or track['confirmed'] is not True or track['source_id'] != row['source_id']
            or track['generation'] != selection['generation']
            or len(track['source_stamps_ns']) < 3
            or track['source_stamps_ns'][-1] != row['source_stamp_ns']
            or any(a >= b for a,b in zip(track['source_stamps_ns'],track['source_stamps_ns'][1:]))
            or tuple(row['rgb_candidates'][0]['bbox_xywh']) != tuple(volume.bbox_xywh)
            or row['rgb_reason'] != 'observation_region_found'
            or limits != row['observation_working_limits']
            or volume.working_depth_range_m != (limits['minimum_depth_m'],limits['maximum_depth_m'])
            or asdict(recorded_config) != row['config']
            or volume.foreground_max_m != recorded_config.foreground_max_m):
        raise ValueError('recorded_context_or_interest_identity_mismatch')
    with np.load(str(source), allow_pickle=False) as evidence:
        # Source NPZ key names are owned by record_rgb_observation.py.
        if (int(evidence['source_stamp_ns']) != row['source_stamp_ns']
                or not np.array_equal(evidence['T_base_camera'], row['T_base_camera'])
                or not np.array_equal(evidence['q_source'], row['q_source'])
                or not np.array_equal(evidence['T_tool_camera'], context['tool_camera'])):
            raise ValueError('recorded_raw_source_binding_mismatch')
        rgb, depth = evidence['color_bgr'], evidence['depth_raw']
        camera = context['camera']
        K = [camera[key] for key in ('fx','fy','cx','cy')]
        if (rgb.dtype != np.uint8 or rgb.shape != tuple(volume.image_shape)+(3,)
                or depth.shape != tuple(volume.image_shape)
                or tuple(volume.image_shape) != (camera['height'],camera['width'])
                or not np.array_equal(evidence['K'], K)
                or not np.array_equal(volume.camera_matrix, [[K[0],0.,K[2]],[0.,K[1],K[3]],[0.,0.,1.]])
                or float(evidence['depth_scale']) != camera['depth_scale']
                or hashlib.sha256(np.ascontiguousarray(rgb).tobytes()).hexdigest() != row['source_rgb_sha256']
                or hashlib.sha256(np.ascontiguousarray(depth).tobytes()).hexdigest() != row['source_depth_sha256']):
            raise ValueError('recorded_pixels_or_camera_geometry_mismatch')
        q, _, _ = bracket_feedback([row['accepted_before'],row['accepted_after']],
                                   row['source_stamp_ns'],row['epoch_ns'])
        if not np.array_equal(q, row['q_source']):
            raise ValueError('recorded_feedback_interpolation_mismatch')
    return row, volume


def local_joint_candidates(q, low, high, fk, mount):
    """Finite local FK candidates for every arm axis; no IK branch jumping."""
    from alicia_grasp_modes.rgb_observation_views import ObservationPoseCandidate
    q, low, high = (np.asarray(value, dtype=float) for value in (q, low, high))
    if (any(value.shape != (6,) or not np.all(np.isfinite(value)) for value in (q, low, high))
            or np.any(low >= high) or np.any(q < low) or np.any(q > high)):
        raise ValueError('invalid_recorded_joint_limits')
    result = []
    for degrees in (5., 10.):
        for axis in range(6):
            for direction in (-1., 1.):
                target = q.copy()
                target[axis] += math.radians(direction*degrees)
                if np.any(target < low) or np.any(target > high):
                    continue
                tool = fk(target)
                result.append(ObservationPoseCandidate(
                    candidate_id='joint%d_%+gdeg' % (axis+1, direction*degrees),
                    T_base_camera=tool @ mount,
                    joint_positions=tuple(target), T_base_tool=tool))
    return result


def trajectory_metrics(trajectory, start):
    from alicia_flexible_grasp.robot.observation_path_guard import (
        segment_coefficients, _polynomial_bounds)
    points = trajectory.points
    if tuple(trajectory.joint_names) != NAMES or not 2 <= len(points) <= 4096:
        raise ValueError('invalid_preview_trajectory')
    positions = np.asarray([point.positions for point in points], dtype=float)
    times = np.asarray([point.time_from_start.to_sec() for point in points])
    start = np.asarray(start, dtype=float)
    if (positions.shape != (len(points), 6) or start.shape != (6,)
            or not np.all(np.isfinite(positions)) or not np.all(np.isfinite(start))
            or not np.all(np.isfinite(times)) or times[0] < 0. or np.any(np.diff(times) <= 0.)):
        raise ValueError('invalid_preview_trajectory')
    curve = np.vstack((start, positions))
    # Waypoints alone can hide large cubic/quintic excursions. Reuse the
    # gateway's actual Noetic polynomial including the stationary bridge.
    first_positive = next((i for i,t in enumerate(times) if t > 0.), None)
    if first_positive is None:
        raise ValueError('preview_trajectory_has_no_positive_time')
    hold = SimpleNamespace(positions=start.tolist(),velocities=[0.]*6,accelerations=[0.]*6)
    segments = [(hold,points[first_positive],times[first_positive])]
    segments.extend((points[i],points[i+1],times[i+1]-times[i])
                    for i in range(first_positive,len(points)-1))
    lower, upper = start.copy(), start.copy()
    for first, second, duration in segments:
        polynomial = segment_coefficients(first,second,float(duration),6)
        # Fixed subdivision tightens the safe Bernstein enclosure; this is an
        # upper bound, not a measurement or proof of physical path tracking.
        for index in range(8):
            lo, hi = _polynomial_bounds(polynomial,index/8.,(index+1)/8.)
            lower,upper = np.minimum(lower,lo),np.maximum(upper,hi)
    return dict(duration_sec=float(times[-1]), point_count=len(points),
        max_joint_excursion_deg=np.degrees(np.max(np.abs(curve-start), axis=0)).tolist(),
        continuous_joint_excursion_bound_deg=np.degrees(np.maximum(np.abs(lower-start),np.abs(upper-start))).tolist(),
        continuous_bound_semantics='noetic_polynomial_and_stationary_bridge_bernstein_enclosure_not_physical_measurement',
        joint_travel_deg=np.degrees(np.sum(np.abs(np.diff(curve, axis=0)), axis=0)).tolist(),
        total_joint_travel_rad=float(np.sum(np.abs(np.diff(curve, axis=0)))),
        goal_positions_rad=positions[-1].tolist(), motion_authorized=False)


class PreviewRevocationGuard:
    """Latch transient changes, including move-away-and-back during an RPC."""
    def __init__(self, row):
        self.q = np.asarray(row['q_source'],float)
        self.epoch = row['epoch_ns']
        self.selection = parse_selection(row['selection'])
        self.lock = threading.Lock()
        self.reason = ''

    def revoke(self, reason):
        with self.lock:
            if not self.reason:
                self.reason = reason

    def check(self):
        with self.lock:
            reason = self.reason
        if reason:
            raise ValueError(reason+'_no_more_previews')

    def epoch_sample(self, epoch):
        if epoch != self.epoch:
            self.revoke('control_epoch_changed')

    def selection_sample(self, selection):
        try:
            if parse_selection(selection) != self.selection:
                self.revoke('selection_changed')
        except (TypeError,ValueError):
            self.revoke('selection_invalid')

    def joint_sample(self, message, now_ns):
        try:
            names, positions = list(message.name),list(message.position)
            named = dict(zip(names,positions))
            actual = np.asarray([named[name] for name in NAMES],float)
            source = message.header.stamp.to_nsec()
            if (len(names) != len(positions) or len(named) != len(names)
                    or message.header.frame_id != 'sdk_measured'
                    or source < self.epoch or not 0 <= now_ns-source <= 800_000_000
                    or not np.all(np.isfinite(actual)) or np.max(np.abs(actual-self.q)) > .004):
                self.revoke('pose_or_feedback_changed_since_record')
        except (KeyError,TypeError,ValueError,AttributeError):
            self.revoke('accepted_feedback_invalid')


def current_context(get_param, mount):
    """Read only the same geometry/configuration fields used by the recorder."""
    return dict(selection=parse_selection(get_param('/grasp_mode/selection')),
        camera=camera_geometry_snapshot(get_param('/camera')),
        perception=deepcopy(get_param('/unknown_perception',{})),handeye=deepcopy(get_param('/handeye',{})),
        model_sha256=hashlib.sha256(get_param('/robot_description').encode()).hexdigest(),
        observation_view_parameters={key:get_param('/grasp_6d/remote/'+key,value)
            for key,value in DEFAULT_OBSERVATION_VIEW_PARAMETERS.items()},
        tool_camera=mount.tolist(),current_tool_camera=mount.tolist())


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--record', required=True)
    parser.add_argument('--max-paths', type=int, default=3)
    args = parser.parse_args()
    if not 0 <= args.max_paths <= 3:
        raise ValueError('preview_path_budget_must_be_0_to_3')
    if not MOUNT.is_mount():
        raise RuntimeError('SanStand unavailable; no fallback')
    out = ROOT/('rgb_observation_preview_'+datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ'))
    out.mkdir(); (out/'ros').mkdir()
    os.environ['ROS_LOG_DIR'] = str(out/'ros')
    import rospy
    import tf2_ros
    from geometry_msgs.msg import PoseStamped
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Bool, Header, String
    from alicia_flexible_grasp_supervisor.msg import GraspState
    from tf.transformations import quaternion_from_matrix
    from alicia_flexible_grasp_supervisor.srv import SetTargetPose, SetTargetPoseRequest
    from alicia_flexible_grasp.robot.observation_path_guard import SerialUrdfFk
    from alicia_flexible_grasp.robot.observation_preview import decode_path_evidence
    from alicia_grasp_modes.rgb_observation_views import rank_rgb_observation_candidates
    rospy.init_node('rgb_observation_preview_readonly', anonymous=True, disable_signals=True)
    report = dict(record=str(Path(args.record).resolve()), motion_authorized=False,
        contact_geometry_authorized=False, robot_commands_sent=False,
        scope='recorded_interest_and_live_planning_only_not_execution_lease', paths=[])
    subs=[]
    try:
        row, volume = read_interest_record(args.record)
        report['record_sha256']=hashlib.sha256(Path(args.record).read_bytes()).hexdigest()
        guard=PreviewRevocationGuard(row)
        buffer=tf2_ros.Buffer(rospy.Duration(10))
        listener=tf2_ros.TransformListener(buffer)
        subs=[
            rospy.Subscriber('/alicia_d/accepted_joint_states',JointState,
                lambda m:guard.joint_sample(m,rospy.Time.now().to_nsec()),queue_size=50),
            rospy.Subscriber('/alicia_d/control_reference_epoch',Header,
                lambda m:guard.epoch_sample(m.stamp.to_nsec()),queue_size=2),
            rospy.Subscriber('/grasp_mode/selection',String,
                lambda m:guard.selection_sample(m.data),queue_size=2),
            rospy.Subscriber('/gui/joint_direct_mode',Bool,
                lambda m:guard.revoke('manual_control_active') if m.data else None,queue_size=2),
            rospy.Subscriber('/grasp/state',GraspState,
                lambda m:guard.revoke('active_grasp_owns_planner') if m.active else None,queue_size=2)]

        def require_unchanged():
            guard.check()
            if rospy.get_param('/gui/joint_direct_mode',False) is True:
                guard.revoke('manual_control_active'); guard.check()
            epoch=rospy.wait_for_message('/alicia_d/control_reference_epoch',Header,timeout=2.)
            guard.epoch_sample(epoch.stamp.to_nsec())
            task=rospy.wait_for_message('/grasp/state',GraspState,timeout=2.)
            if task.active:
                guard.revoke('active_grasp_owns_planner')
            joints=rospy.wait_for_message('/alicia_d/accepted_joint_states',JointState,timeout=2.)
            guard.joint_sample(joints,rospy.Time.now().to_nsec())
            mount_now=matrix(buffer.lookup_transform('tool0','camera_link',rospy.Time(0),rospy.Duration(.3)))
            live=current_context(rospy.get_param,mount_now)
            live['epoch_ns']=epoch.stamp.to_nsec()
            if not source_context_matches(row['source_context'],live):
                guard.revoke('live_geometry_or_owner_context_changed')
            guard.check()

        require_unchanged()
        description = rospy.get_param('/robot_description')
        if hashlib.sha256(description.encode()).hexdigest() != row['source_context']['model_sha256']:
            raise ValueError('recorded_robot_model_changed')
        fk = SerialUrdfFk(description, NAMES)
        q = np.asarray(row['q_source'])
        mount = np.asarray(row['source_context']['tool_camera'])
        if not np.allclose(fk(q) @ mount, row['T_base_camera'], rtol=0., atol=1e-6):
            raise ValueError('recorded_fk_tf_mismatch')
        low, high = (rospy.get_param('/robot/joint_'+key+'_rad')[:6] for key in ('min', 'max'))
        low = np.maximum(low, fk.limits[:, 0]); high = np.minimum(high, fk.limits[:, 1])
        candidates = local_joint_candidates(q, low, high, fk, mount)
        ranking = rank_rgb_observation_candidates(volume, candidates,
            current_T_base_camera=row['T_base_camera'], camera_matrix=volume.camera_matrix,
            image_shape=volume.image_shape, working_depth_range_m=volume.working_depth_range_m,
            current_joint_positions=q, current_T_base_tool=fk(q),
            roi_margin_px=row['observation_working_limits']['roi_margin_px'],
            acquisition_center_xy=(.5, .5),
            acquisition_radius_fraction=row['config']['center_radius_fraction'])
        report.update(source_stamp_ns=row['source_stamp_ns'], candidate_count=len(candidates),
                      ranking=ranking, model_sha256=fk.model_sha256)
        (out/'robot_description.urdf').write_text(description)
        index = {candidate.candidate_id: candidate for candidate in candidates}
        if args.max_paths:
            rospy.wait_for_service('/supervisor/check_observation_pose_strict', timeout=3.)
        proxy = rospy.ServiceProxy('/supervisor/check_observation_pose_strict', SetTargetPose)
        started = time.monotonic()
        for scored in ranking['ranked'][:args.max_paths]:
            if time.monotonic()-started > 40.:
                report['path_budget_reached'] = True
                break
            require_unchanged()
            candidate = index[scored['candidate_id']]
            target = PoseStamped()
            target.header.frame_id = 'base_link'
            target.header.stamp = rospy.Time.now()
            tool = np.asarray(candidate.T_base_tool)
            target.pose.position.x, target.pose.position.y, target.pose.position.z = tool[:3, 3]
            orientation = quaternion_from_matrix(tool)
            (target.pose.orientation.x, target.pose.orientation.y,
             target.pose.orientation.z, target.pose.orientation.w) = orientation
            # The only service used by this tool rejects execute=True itself.
            response = proxy(SetTargetPoseRequest(target=target, execute=False))
            entry = dict(candidate_id=scored['candidate_id'], success=bool(response.success),
                         complete_collision_scene_certified=False, motion_authorized=False)
            report['paths'].append(entry)
            # An RPC may complete after a user moved and returned. The guard
            # latches that event rather than comparing only these endpoints.
            try:
                require_unchanged()
            except Exception as exc:
                entry['context_valid_after_rpc']=False
                entry['rejection']=str(exc)
                raise
            entry['context_valid_after_rpc']=True
            token = next((part.split('=', 1)[1] for part in response.message.split()
                          if part.startswith('observation_path_evidence=')), None)
            entry['message'] = ' '.join(part for part in response.message.split()
                                       if not part.startswith('observation_path_evidence='))
            if response.success and token is None:
                raise ValueError('successful_preview_missing_serialized_path_evidence')
            if token:
                data, plan, hold = decode_path_evidence(token, target, fk)
                if data['reference_epoch_ns'] != row['epoch_ns']:
                    raise ValueError('planned_controller_reference_epoch_changed')
                if getattr(plan.multi_dof_joint_trajectory,'points',()):
                    raise ValueError('unsupported_multi_dof_preview_path')
                entry['path'] = trajectory_metrics(plan.joint_trajectory, hold.positions)
                entry['reference_epoch_ns'] = data['reference_epoch_ns']
                entry['evidence_file'] = str(out/(candidate.candidate_id+'_path.txt'))
                Path(entry['evidence_file']).write_text(token+'\n')
                endpoint = np.asarray(plan.joint_trajectory.points[-1].positions)
                entry['endpoint_branch_error_deg'] = np.degrees(np.abs(endpoint-np.asarray(candidate.joint_positions))).tolist()
            print(json.dumps(entry, ensure_ascii=False), flush=True)
    except Exception as exc:
        report['error'] = str(exc)
        raise
    finally:
        for sub in subs: sub.unregister()
        if not MOUNT.is_mount():
            raise RuntimeError('SanStand disappeared before preview evidence write')
        (out/'preview.json').write_text(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False)+'\n')
        print('RESULT '+str(out/'preview.json'), flush=True)


if __name__ == '__main__':
    main()
