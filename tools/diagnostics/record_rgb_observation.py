#!/usr/bin/env python3
"""Finite, read-only ROS experiment for initial RGB observation interests.

No publishers, service calls, parameter writes, target injection or motion.
Raw evidence and summaries require the configured SanStand experiment drive.
"""
import argparse
from collections import deque
from dataclasses import asdict, fields
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import sys
import threading
import time

import cv2
import numpy as np

from alicia_grasp_modes.rgb_observation_region import (
    CenterSearchPolicy, RgbObservationTracker, propose_rgb_observation_regions)
from alicia_grasp_modes.rgb_observation_volume import (
    SupportPlaneProvenance, build_rgb_observation_volume)
from alicia_grasp_modes.tabletop import Config, segment
from alicia_grasp_modes.rgb_observation_source import (
    digest, camera_geometry_snapshot, bracket_feedback, require_fresh_source,
    measured_support_coverage)
from alicia_grasp_modes.selection import parse_selection
from alicia_flexible_grasp.robot.observation_view_limits import (
    DEFAULT_OBSERVATION_VIEW_PARAMETERS, effective_observation_view_limits)
from alicia_flexible_grasp.vision.rgbd_rectification import (
    rectification_rgb_valid_mask, require_rectified_source_after_activation)

MOUNT = Path('/media/zhuyupei/SanStand')
ROOT = MOUNT/'实验记录/Robot-Controller/ros_log'
NAMES = tuple('Joint%d' % i for i in range(1, 7))


def implementation_metadata():
    paths = [Path(__file__).resolve(),
        Path(sys.modules[propose_rgb_observation_regions.__module__].__file__).resolve(),
        Path(sys.modules[build_rgb_observation_volume.__module__].__file__).resolve(),
        Path(sys.modules[bracket_feedback.__module__].__file__).resolve()]
    return dict(python_version=sys.version, executable=sys.executable,
                opencv_version=cv2.__version__, numpy_version=np.__version__,
                opencv_threads=cv2.getNumThreads(),
                implementation_sha256={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in paths})


def matrix(message):
    t = message.transform.translation
    r = message.transform.rotation
    from tf.transformations import quaternion_matrix
    quaternion = np.array([r.x, r.y, r.z, r.w], dtype=float)
    translation = np.array([t.x, t.y, t.z], dtype=float)
    if (not np.all(np.isfinite(quaternion)) or not np.all(np.isfinite(translation))
            or abs(float(np.linalg.norm(quaternion))-1.) > 1e-6):
        raise ValueError('invalid_source_transform')
    value = quaternion_matrix([r.x, r.y, r.z, r.w])
    value[:3, 3] = translation
    return value


def latest_bracketed_pair(options, joints, source_floor):
    """Choose the newest already bracketed pair inside the caller's age window.

    Waiting for feedback is input synchronization, not a rejected RGB sample.
    This never extrapolates joints or widens the existing 200 ms bracket.
    Conflicting feedback is a data error, not a reason to try an older frame.
    """
    for ns, pair in sorted(options, key=lambda row: row[0], reverse=True):
        if ns <= source_floor:
            continue
        try:
            bracket = bracket_feedback(joints, ns, source_floor)
        except ValueError as exc:
            if str(exc) != 'accepted_feedback_does_not_bracket_source':
                raise
            continue
        return ns, pair, bracket
    return None


def source_context_matches(before, after):
    """Digest all context fields; same source provenance is caller-owned here."""
    return digest(before) == digest(after)


def source_rgb_proposals(rgb, camera, stamp_ns, policy, depth_valid=None):
    """Use the frame's bound camera geometry, including its observed RGB domain."""
    require_rectified_source_after_activation(camera, stamp_ns)
    valid = rectification_rgb_valid_mask(camera, rgb.shape[:2])
    return propose_rgb_observation_regions(rgb, policy=policy, depth_valid=depth_valid,
        **({} if valid is None else {'rgb_valid': valid}))


def main():
    import rospy
    import tf2_ros
    from cv_bridge import CvBridge
    from sensor_msgs.msg import Image, JointState
    from std_msgs.msg import Header, String
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--duration', type=float, default=12.)
    parser.add_argument('--max-frames', type=int, default=12)
    args = parser.parse_args()
    if not 1 <= args.duration <= 60 or not 3 <= args.max_frames <= 60:
        raise ValueError('finite observation capture budget required')
    if not MOUNT.is_mount():
        raise RuntimeError('SanStand unavailable; no fallback')
    out = ROOT/('rgb_observation_bootstrap_'+datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ'))
    out.mkdir(); (out/'ros').mkdir()
    os.environ['ROS_LOG_DIR'] = str(out/'ros')
    rospy.init_node('rgb_observation_readonly', anonymous=True, disable_signals=True)
    cv2.setNumThreads(1)
    implementation = implementation_metadata()
    tf = tf2_ros.Buffer(rospy.Duration(10))
    listener = tf2_ros.TransformListener(tf)
    bridge = CvBridge(); lock = threading.Lock(); pairs = {}; qbuffer = deque(maxlen=150)
    runtime = {}; rows = []; seen = set()
    tracker = RgbObservationTracker()
    policy = CenterSearchPolicy(center_xy=(.5, .5), radius_fraction=.125)
    config_raw = rospy.get_param('/unknown_perception', {})
    cfg = Config(**{k:v for k,v in config_raw.items() if k in Config.__dataclass_fields__})
    selection = parse_selection(rospy.get_param('/grasp_mode/selection'))
    if selection['mode'] != 'unknown' or selection['strategy'] != 'observe_return':
        raise RuntimeError('current unknown observation selection required')

    def image_cb(msg, kind):
        with lock:
            ns = msg.header.stamp.to_nsec()
            pairs.setdefault(ns, {})[kind] = msg
            while len(pairs) > 16:
                del pairs[min(pairs)]

    def joint_cb(msg):
        with lock:
            if msg.header.frame_id != 'sdk_measured':
                return
            values = dict(zip(msg.name, msg.position))
            if (len(msg.name) != len(msg.position) or len(values) != len(msg.name)
                    or any(n not in values for n in NAMES)
                    or not np.all(np.isfinite([values[n] for n in NAMES]))):
                return
            qbuffer.append((msg.header.stamp.to_nsec(), [values[n] for n in NAMES]))

    def state_cb(msg, key):
        try:
            value = msg.stamp.to_nsec() if key == 'epoch' else json.loads(msg.data)
        except (AttributeError, TypeError, ValueError):
            return
        with lock:
            runtime[key] = value

    def source_context(stamp):
        with lock:
            epoch = runtime.get('epoch')
        return dict(epoch_ns=epoch,
            selection=parse_selection(rospy.get_param('/grasp_mode/selection')),
            camera=camera_geometry_snapshot(rospy.get_param('/camera')),
            perception=rospy.get_param('/unknown_perception', {}),
            # These are the existing RGB-D projection working limits used by
            # the planner/gateway, not an imposed observation-pose distance.
            observation_view_parameters={key:rospy.get_param('/grasp_6d/remote/'+key, value)
                for key,value in DEFAULT_OBSERVATION_VIEW_PARAMETERS.items()},
            handeye=rospy.get_param('/handeye', {}),
            model_sha256=hashlib.sha256(rospy.get_param('/robot_description').encode()).hexdigest(),
            # Installation is rigid; also reject a changed latest mount. The
            # projection itself always uses the source-stamped transform.
            tool_camera=matrix(tf.lookup_transform('tool0', 'camera_link', stamp,
                                                   rospy.Duration(.1))).tolist(),
            current_tool_camera=matrix(tf.lookup_transform('tool0', 'camera_link',
                rospy.Time(0), rospy.Duration(.1))).tolist())

    subs = [rospy.Subscriber('/supervisor/camera/'+name+'/image_raw', Image,
        lambda m,k=k:image_cb(m,k), queue_size=2, buff_size=2**24)
        for name,k in [('color','rgb'),('depth','depth')]]
    subs += [rospy.Subscriber('/alicia_d/accepted_joint_states', JointState, joint_cb, queue_size=50),
             rospy.Subscriber('/alicia_d/control_reference_epoch', Header, lambda m:state_cb(m,'epoch'),queue_size=2),
             rospy.Subscriber('/perception/unknown/detector_status', String, lambda m:state_cb(m,'detector'),queue_size=2)]
    started = time.monotonic()
    print('SESSION '+str(out), flush=True)
    try:
        while time.monotonic()-started < args.duration and len(rows) < args.max_frames and not rospy.is_shutdown():
            time.sleep(.05)
            if not MOUNT.is_mount():
                raise RuntimeError('SanStand disappeared')
            now = rospy.Time.now().to_nsec()
            with lock:
                options = [(ns,dict(pair)) for ns,pair in pairs.items()
                    if len(pair)==2 and ns > max(seen, default=selection['stamp_ns']) and 100_000_000 <= now-ns <= 500_000_000]
                joints = list(qbuffer); state = dict(runtime)
            if not options or 'epoch' not in state:
                continue
            matched = latest_bracketed_pair(options, joints,
                                           max(state['epoch'], selection['stamp_ns']))
            if matched is None:
                continue
            ns, pair, bracket = matched
            seen.add(ns)
            row = dict(source_stamp_ns=ns, observation_only=True, motion_authorized=False,
                       contact_geometry_authorized=False)
            rows.append(row)
            frame_started = time.monotonic()
            raw_evidence = None
            region_image = None
            try:
                if parse_selection(rospy.get_param('/grasp_mode/selection')) != selection:
                    raise RuntimeError('selection changed; experiment ends')
                if ns <= max(selection['stamp_ns'], state['epoch']):
                    raise ValueError('source_predates_selection_or_epoch')
                require_fresh_source(ns, rospy.Time.now().to_nsec())
                q, lo, hi = bracket
                stamp = pair['rgb'].header.stamp
                context = source_context(stamp)
                if (context['epoch_ns'] != state['epoch'] or context['selection'] != selection
                        or context['perception'] != config_raw
                        or context['tool_camera'] != context['current_tool_camera']):
                    raise ValueError('source_context_changed_at_acquisition')
                cam = context['camera']
                require_rectified_source_after_activation(cam, ns)
                limits = effective_observation_view_limits(context['observation_view_parameters'])
                raw = bridge.imgmsg_to_cv2(pair['depth'], 'passthrough')
                rgb = bridge.imgmsg_to_cv2(pair['rgb'], 'bgr8')
                if (pair['rgb'].header.frame_id != 'camera_link' or pair['depth'].header.frame_id != 'camera_link'
                        or pair['depth'].header.stamp.to_nsec() != ns
                        or raw.shape != rgb.shape[:2] or not cam.get('align_depth_to_color')
                        or cam.get('frame_id') != 'camera_link'
                        or raw.shape != (cam.get('height'), cam.get('width'))):
                    raise ValueError('source_pair_geometry_invalid')
                if pair['depth'].encoding in ('16UC1','mono16'):
                    if not np.isfinite(cam['depth_scale']) or cam['depth_scale'] <= 0.:
                        raise ValueError('invalid_runtime_depth_scale')
                    depth = raw.astype(np.float32)*cam['depth_scale']
                elif pair['depth'].encoding == '32FC1':
                    depth = raw.astype(np.float32)
                else:
                    raise ValueError('unsupported_depth_encoding')
                T = matrix(tf.lookup_transform('base_link','camera_link',stamp,rospy.Duration(.2)))
                X = np.asarray(context['tool_camera'])
                K = [cam[k] for k in ('fx','fy','cx','cy')]
                camera_binding = {k:cam.get(k) for k in ('fx','fy','cx','cy','depth_scale','width','height','frame_id','align_depth_to_color')}
                source_id = digest(context)
                file = out/(str(ns)+'_source.npz')
                # Retain in this bounded worker, then write only after source
                # age/context adjudication. No callback performs disk I/O.
                raw_evidence = dict(color_bgr=rgb, depth_raw=raw, depth_scale=cam['depth_scale'],
                    K=np.array(K), T_base_camera=T, T_tool_camera=X, q_source=q,
                    source_stamp_ns=np.int64(ns))
                row.update(source_file=str(file), camera=camera_binding, config=asdict(cfg),
                    source_id=source_id, selection=selection, epoch_ns=state['epoch'],
                    source_context=context,
                    source_rgb_sha256=hashlib.sha256(np.ascontiguousarray(rgb).tobytes()).hexdigest(),
                    source_depth_sha256=hashlib.sha256(np.ascontiguousarray(raw).tobytes()).hexdigest(),
                    observation_working_limits=limits,
                    observation_working_limits_semantics='existing_rgbd_projection_limits_not_target_pose_distance',
                    accepted_before=lo, accepted_after=hi, q_source=q.tolist(),
                    T_base_camera=T.tolist(), detector=state.get('detector'))
                valid = np.isfinite(depth)&(depth>=cfg.depth_min_m)&(depth<=cfg.depth_max_m)
                proposals = source_rgb_proposals(rgb, cam, ns, policy, depth_valid=valid)
                require_fresh_source(ns, rospy.Time.now().to_nsec())
                track = tracker.observe(proposals,source_id=source_id,generation=selection['generation'],
                    stamp_ns=ns,joints_rad=q)
                row.update(rgb_reason=proposals.reason, rgb_candidate_count=proposals.total_candidate_count,
                    rgb_candidates=[{f.name:getattr(c,f.name) for f in fields(c) if f.name!='mask'} for c in proposals.candidates],
                    rgb_track={f.name:getattr(track,f.name) for f in fields(track) if f.name!='region'})
                if track.region is not None:
                    region_image = track.region.mask.astype(np.uint8)*255
                try:
                    fitted = segment(depth,K,T,cfg)
                    plane = fitted.plane_base
                    metrics = fitted.metrics
                    # Retain actual coverage rather than recording a made-up passing value.
                    coverage = measured_support_coverage(depth, K, T, plane, cfg,
                                                        metrics['support_ring_outer_radius_px'])
                    quadrants = coverage['spatial_quadrants']
                    row['support_metrics']=dict(metrics, **coverage)
                    if track.confirmed:
                        provenance = SupportPlaneProvenance(source_stamp_ns=ns,
                            support_plane_base=tuple(plane),source_record_id=str(file),
                            inlier_count=int(round(metrics['support_sample_count']*metrics['support_inlier_ratio'])),
                            inlier_ratio=metrics['support_inlier_ratio'],fit_rmse_m=metrics['support_residual_m'],
                            spatial_quadrants=quadrants)
                        volume = build_rgb_observation_volume(bbox_xywh=track.region.bbox_xywh,
                            image_shape=depth.shape,camera_matrix=[[K[0],0,K[2]],[0,K[1],K[3]],[0,0,1]],
                            T_base_camera=T,support_plane_base=plane,support_provenance=provenance,
                            foreground_max_m=cfg.foreground_max_m,source_stamp_ns=ns,
                            working_depth_range_m=(limits['minimum_depth_m'],limits['maximum_depth_m']))
                        row['interest_volume']=asdict(volume)
                except ValueError as exc:
                    row['support_or_volume_error']=str(exc)
                row['source_context_unchanged']=source_context_matches(context, source_context(stamp))
                row['source_age_at_end_sec']=(rospy.Time.now().to_nsec()-ns)*1e-9
                row['fresh_for_existing_frame_limit']=0 <= row['source_age_at_end_sec'] <= .8
                row['observation_interest_ready']=bool(track.confirmed and row.get('interest_volume') and row['fresh_for_existing_frame_limit'] and row['source_context_unchanged'])
                if not row['fresh_for_existing_frame_limit']:
                    tracker.reset()
                if not row['source_context_unchanged']:
                    tracker.reset()
            except Exception as exc:
                row['error']=str(exc); row['observation_interest_ready']=False; tracker.reset()
                if isinstance(exc,RuntimeError):
                    raise
            finally:
                row['processing_wall_sec']=time.monotonic()-frame_started
                # Source acceptance above is diagnostic as of this timestamp,
                # never a lease usable after writing or by an execution caller.
                write_started = time.monotonic()
                if not MOUNT.is_mount():
                    raise RuntimeError('SanStand disappeared before evidence write')
                if raw_evidence is not None:
                    np.savez(file, **raw_evidence)
                    row['source_file_sha256']=hashlib.sha256(file.read_bytes()).hexdigest()
                    if not cv2.imwrite(str(out/(str(ns)+'_rgb.png')), raw_evidence['color_bgr']):
                        raise RuntimeError('RGB evidence write failed')
                    if region_image is not None and not cv2.imwrite(
                            str(out/(str(ns)+'_rgb_region.png')), region_image):
                        raise RuntimeError('region evidence write failed')
                row['evidence_write_wall_sec']=time.monotonic()-write_started
                (out/'result.json').write_text(json.dumps(dict(rows=rows,robot_commands_sent=False,
                    implementation=implementation,
                    motion_authorized=False,contact_geometry_authorized=False,
                    execution_integration_complete=False),ensure_ascii=False,indent=2,allow_nan=False)+'\n')
                print(json.dumps({k:row.get(k) for k in ('source_stamp_ns','rgb_reason','rgb_candidate_count','observation_interest_ready','support_or_volume_error','error','processing_wall_sec')},ensure_ascii=False),flush=True)
            time.sleep(max(0,.5-(time.monotonic()-frame_started)))
    finally:
        for sub in subs: sub.unregister()
        print('RESULT '+str(out/'result.json'),flush=True)

if __name__ == '__main__':
    main()
