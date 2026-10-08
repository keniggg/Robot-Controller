#!/usr/bin/env python3
"""J3-only fresh-board out-and-back diagnostic; never bootstrap/sync/enable.

Default invocation only prepares a current geometry report and sends no command.
--execute permits at most two gui_direct J3 commands: an SDK increment no larger
than 2 degrees (22 encoder words), then the original SDK word. Return is only
sent after fresh geometry/state evidence; failure sends no further command.
A 57-second wall-clock deadline includes acquisition and execution. Physical
response is observed within +/-4.8 degrees of the initial measured J3 position,
using the same absolute interval for both outbound and return. The 2-degree
limit describes the requested SDK increment, not a certified physical bound.
"""
import argparse
from collections import OrderedDict, deque
from datetime import datetime, timezone
import hashlib
import importlib.util
import json
import math
import os
import signal
from pathlib import Path
import sys
import threading
import time
import xmlrpc.client

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'src/alicia_flexible_grasp_supervisor/src'))

import numpy as np
import rosbag
import rosgraph
import rospy
import tf2_ros
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import TransformStamped
from moveit_msgs.msg import RobotState
from moveit_msgs.srv import GetStateValidity, GetStateValidityRequest
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import Bool, Header, String
from tf2_msgs.msg import TFMessage
from tf.transformations import quaternion_matrix
from alicia_flexible_grasp_supervisor.msg import GraspState
from alicia_flexible_grasp.robot.observation_path_guard import SerialUrdfFk
from alicia_flexible_grasp.robot.stationary_following import accepted_stationary_window
from alicia_flexible_grasp.grasp.gripper_geometry import (
    GripperGeometry, _stage_boxes, _box_corners)

NAMES = ['Joint%d' % n for n in range(1, 7)]+['right_finger']
MOUNT = Path('/media/zhuyupei/SanStand')
ARCHIVE = MOUNT/'实验记录/Robot-Controller/ros_log'
RGB = '/supervisor/camera/color/image_raw'
QUANTUM = 2*math.pi/4096
SPEC = importlib.util.spec_from_file_location('bounded_response_camera_model',
    ROOT/'src/real-arm/alicia_d_calibration/scripts/charuco_tracker.py')
CAMERA = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(CAMERA)


def write_json(path, value):
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2,
                               allow_nan=False)+'\n')


def transform(msg):
    p, q = msg.transform.translation, msg.transform.rotation
    value = quaternion_matrix([q.x, q.y, q.z, q.w])
    value[:3, 3] = [p.x, p.y, p.z]
    if not np.isfinite(value).all():
        raise ValueError('nonfinite transform')
    return value


def positions(msg):
    if len(msg.name) != len(msg.position) or len(set(msg.name)) != len(msg.name):
        raise ValueError('ambiguous joint vector')
    values = dict(zip(msg.name, msg.position))
    q = np.array([values[n] for n in NAMES], dtype=float)
    if not np.isfinite(q).all() or np.any(np.abs(q[:6]) > math.pi) or not 0 <= q[6] <= .05:
        raise ValueError('invalid joint vector')
    return q


def wire_words(q):
    # C++ std::lround for the positive encoded range, not Python bankers round.
    return [int(math.floor((float(v)+math.pi)/QUANTUM+.5)) for v in q[:6]]


def validate_joint3_envelope(q, bounds):
    """Reject an escape from the original measured interval, including on return."""
    if bounds is None:
        raise RuntimeError('fixed J3 response interval has not been initialized')
    value = float(q[2])
    if not math.isfinite(value) or not bounds[0] <= value <= bounds[1]:
        raise RuntimeError('J3 response outside original absolute measured interval')


def validate_joint3_command(target, reference, baseline, label, count):
    """Validate actual publication sequence/word bounds independently of path planning."""
    if count not in (0, 1) or label != ('outbound', 'return_original_sdk')[count]:
        raise RuntimeError('J3 two-command sequence exhausted or out of order')
    values = [np.asarray(v, dtype=float) for v in (target, reference, baseline)]
    if any(v.shape != (7,) or not np.isfinite(v).all() for v in values):
        raise RuntimeError('J3 command/reference/baseline must be finite seven-joint vectors')
    target, reference, baseline = values
    words, current_words, baseline_words = map(wire_words, values)
    if any(words[k] != current_words[k] or words[k] != baseline_words[k]
           for k in range(6) if k != 2):
        raise RuntimeError('J3 command would alter an unedited arm wire word')
    if target[6] != reference[6] or target[6] != baseline[6]:
        raise RuntimeError('J3 command would alter the gripper reference')
    step_words = abs(words[2]-current_words[2])
    if not 0 < step_words <= int(math.floor(math.radians(2.)/QUANTUM)):
        raise RuntimeError('J3 SDK increment must be 1..22 encoder words (at most 2 degrees)')
    if abs(target[2]-reference[2]) > math.radians(2.)+1e-12:
        raise RuntimeError('J3 unquantized SDK increment exceeds 2 degrees')
    if count == 0 and current_words != baseline_words:
        raise RuntimeError('J3 baseline SDK reference changed before outbound')
    if count == 1 and (words != baseline_words or not np.array_equal(target, baseline)):
        raise RuntimeError('J3 return must restore the exact original SDK reference')


def paired_board_candidates(rgb, quality, board, now_ns, model_fingerprint):
    """Exact, fresh source-time tuples; notification never extends source age."""
    options = []
    for ns, q in quality.items():
        frame = q.get('board_frame')
        if (ns in rgb and (ns, frame) in board and
                0 <= now_ns-ns <= 350_000_000 and
                q.get('pose_ok') is True and q.get('charuco_count', 0) >= 35 and
                q.get('edge_clearance_px', -1) >= 20 and
                isinstance(q.get('reprojection_rms_px'), (int, float)) and
                0 <= float(q['reprojection_rms_px']) <= 1.5 and
                not q.get('camera_model_change_latched') and not q.get('camera_model_error') and
                q.get('camera_model_fingerprint') == model_fingerprint and
                q.get('intrinsics_source') == 'realsense_runtime' and
                q.get('board_size') == [12,9] and q.get('dictionary_id') == 'DICT_5X5_100' and
                abs(float(q.get('square_length_m', 0))-.015) < 1e-10 and
                abs(float(q.get('marker_length_m', 0))-.01125) < 1e-10):
            options.append((ns, q, rgb[ns], board[(ns, frame)]))
    return options


class Diagnostic:
    def __init__(self, args, directory):
        self.args, self.out = args, directory
        # Import the matching math backend before any bounded fresh-pair wait.
        # No RealSense context, pipeline, device, or hardware API is created.
        self.camera_backend = __import__('pyrealsense2')
        self.lock = threading.RLock()
        self.pair_changed = threading.Condition(self.lock)
        self.pair_generation = 0
        self.bag_lock = threading.Lock()
        self.events = (directory/'events.jsonl').open('x', buffering=1)
        self.bag = rosbag.Bag(str(directory/'evidence.bag'), 'w')
        self.closed = False
        self.started_ns = rospy.Time.now().to_nsec()
        self.deadline = time.monotonic()+56
        self.bootstrap_phase = None
        self.publish_count = 0
        self.baseline_wire_q = None
        self.baseline_measured_q = None
        self.joint3_response_bounds = None
        self.state = {}
        self.epoch = None
        self.frozen_epoch = None
        self.fault = None
        self.accepted, self.reference = deque(maxlen=1000), deque(maxlen=1000)
        self.rgb, self.quality, self.board = OrderedDict(), OrderedDict(), OrderedDict()
        self.board_frame = None
        self.model_fingerprint = None
        self.expected_words = None
        self.expected_gripper = None
        self.buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10))
        self.listener = tf2_ros.TransformListener(self.buffer)
        # A publisher exists only in explicit execution mode; it never latches.
        self.publisher = (rospy.Publisher('/joint_commands', JointState, queue_size=1,
                                         latch=False) if args.execute else None)
        self.subs = []
        for topic, kind in [
            ('/alicia_d/accepted_joint_states', JointState),
            ('/alicia_d/sdk_command', JointState),
            ('/alicia_d/control_reference', JointState),
            ('/alicia_d/sdk_diagnostics', DiagnosticArray),
            ('/alicia_d/control_reference_epoch', Header),
            ('/alicia_d/actuation_status', String),
            ('/alicia_d/feedback_ready', Bool),
            ('/alicia_d/protection_latched', Bool),
            ('/gui/joint_direct_mode', Bool), ('/joint_commands', JointState),
            ('/grasp/state', GraspState), ('/charuco/quality', String),
            ('/tf', TFMessage), ('/tf_static', TFMessage), (RGB, Image),
            ('/supervisor/camera/depth/image_raw', Image)]:
            self.subs.append(rospy.Subscriber(topic, kind,
                lambda msg, topic=topic: self.receive(topic, msg),
                queue_size=300 if kind != Image else 4, buff_size=2**24))
        self.fk = SerialUrdfFk(rospy.get_param('/robot_description'), NAMES[:6])
        (directory/'robot_description.urdf').write_text(rospy.get_param('/robot_description'))
        self.gripper = GripperGeometry(.05, .002, np.array([.0434,.0286,.06]),
                                      np.array([.1175,.155,.0774]), .003)
        self.lo = np.asarray(rospy.get_param('/robot/joint_min_rad'))
        self.hi = np.asarray(rospy.get_param('/robot/joint_max_rad'))
        self.validity = rospy.ServiceProxy('/check_state_validity', GetStateValidity)

    def write_bag(self, topic, msg, stamp):
        # Slow external-disk writes must not block fresh camera/encoder caches.
        # rosbag is still serialized independently; no callback writes after close.
        with self.bag_lock:
            if not self.closed:
                self.bag.write(topic, msg, stamp)

    def event(self, name, **values):
        with self.lock:
            row = dict(event=name, wall_time=time.time(),
                       ros_stamp_ns=rospy.Time.now().to_nsec(), **values)
            payload = json.dumps(row, ensure_ascii=False, allow_nan=False)
            self.events.write(payload+'\n')
        self.write_bag('/bounded_response/events', String(data=payload), rospy.Time.now())
        print(payload, flush=True)

    def receive(self, topic, msg):
        received_stamp = rospy.Time.now()
        with self.lock:
            if self.closed:
                return
            now = time.monotonic()
            try:
                if topic == '/alicia_d/accepted_joint_states':
                    q = positions(msg)
                    self.accepted.append(msg)
                    self.state['accepted_received'] = now
                    if self.joint3_response_bounds is not None:
                        validate_joint3_envelope(q, self.joint3_response_bounds)
                elif topic == '/alicia_d/control_reference':
                    positions(msg)
                    self.reference.append(msg)
                    if self.expected_words is not None and (
                        wire_words(positions(msg)) != self.expected_words
                        or abs(positions(msg)[6]-self.expected_gripper) > 1e-10):
                        self.fault = 'unexpected transmitted SDK position or gripper word'
                elif topic == '/alicia_d/control_reference_epoch':
                    self.epoch = msg.stamp.to_nsec()
                    if self.frozen_epoch is not None and self.epoch != self.frozen_epoch:
                        self.fault = 'driver reference epoch changed; no automatic recovery'
                elif topic == '/joint_commands':
                    caller = getattr(msg, '_connection_header', {}).get('callerid', '')
                    if caller != rospy.get_name() and msg.header.frame_id.startswith('gui_direct'):
                        self.fault = 'external GUI command from '+caller
                elif topic == '/alicia_d/actuation_status':
                    self.state['status'] = msg.data
                    self.state['status_received'] = now
                    if msg.data.startswith(('UNCONFIRMED:','DISABLED:','OVERHEAT_BLOCKED:')):
                        self.fault='driver lost admissible actuation state: '+msg.data
                elif topic in ('/alicia_d/feedback_ready', '/alicia_d/protection_latched',
                               '/gui/joint_direct_mode'):
                    self.state[topic] = bool(msg.data)
                elif topic == '/grasp/state':
                    self.state['task_active'] = bool(msg.active)
                elif topic == RGB:
                    self.cache(self.rgb, msg.header.stamp.to_nsec(), msg, 12)
                elif topic == '/charuco/quality':
                    q = json.loads(msg.data)
                    if isinstance(q.get('source_stamp_ns'), int):
                        self.cache(self.quality, q['source_stamp_ns'], q, 40)
                elif topic == '/tf':
                    for t in msg.transforms:
                        if t.header.frame_id == 'camera_link':
                            self.cache(self.board, (t.header.stamp.to_nsec(), t.child_frame_id), t, 100)
                if topic in (RGB, '/charuco/quality', '/tf'):
                    self.pair_generation += 1
                    self.pair_changed.notify_all()
            except Exception as exc:
                self.fault = 'evidence callback: '+str(exc)
                self.pair_changed.notify_all()
        if topic != RGB:  # Raw RGB is saved only at geometry snapshots.
            try:
                self.write_bag(topic, msg, received_stamp)
            except Exception as exc:
                with self.pair_changed:
                    self.fault = 'evidence bag write: '+str(exc)
                    self.pair_changed.notify_all()

    @staticmethod
    def cache(cache, key, value, limit):
        cache[key] = value
        while len(cache) > limit:
            cache.popitem(last=False)

    def guard(self):
        with self.lock:
            if time.monotonic() > self.deadline:
                raise RuntimeError('experiment overall deadline exhausted')
            if rospy.is_shutdown() or self.fault:
                raise RuntimeError(self.fault or 'ROS shutdown')
            status=self.state.get('status', '')
            if not status.startswith('CONFIRMED:'):
                raise RuntimeError('unexpected driver state '+status+'; no recovery permitted')
            if time.monotonic()-self.state.get('status_received', 0) > .5:
                raise RuntimeError('stale actuation status')
            if self.state.get('/alicia_d/feedback_ready') is not True:
                raise RuntimeError('feedback not ready')
            if self.state.get('/alicia_d/protection_latched') is not False:
                raise RuntimeError('protection state unavailable/active')
            if self.state.get('/gui/joint_direct_mode') is not True:
                raise RuntimeError('GUI direct ownership unavailable')
            if self.state.get('task_active') is not False:
                raise RuntimeError('task state unavailable/active')
            if (not self.accepted or time.monotonic()-self.state.get('accepted_received', 0) > .3
                    or not self.reference or self.epoch is None):
                raise RuntimeError('fresh measured and wire reference evidence unavailable')
            if self.reference[-1].header.frame_id != 'sdk_manual':
                raise RuntimeError('wire reference is not in GUI direct mode')
            ref_ns = self.reference[-1].header.stamp.to_nsec()
            if ref_ns < self.epoch or not 0 <= rospy.Time.now().to_nsec()-ref_ns <= 300_000_000:
                raise RuntimeError('SDK source stamp stale/future/outside epoch')
            if self.accepted[-1].header.frame_id != 'sdk_measured':
                raise RuntimeError('accepted feedback is not SDK measured')
            if not 0 <= rospy.Time.now().to_nsec()-self.accepted[-1].header.stamp.to_nsec() <= 300_000_000:
                raise RuntimeError('accepted source stamp stale/future')
            if self.joint3_response_bounds is not None:
                validate_joint3_envelope(positions(self.accepted[-1]), self.joint3_response_bounds)

    def stationary(self, after_ns):
        with self.lock:
            return accepted_stationary_window(tuple(self.accepted),
                now_sec=rospy.Time.now().to_sec(), epoch_ns=self.epoch,
                after_ns=after_ns, duration_sec=.8,
                position_epsilon_rad=math.radians(.15), minimum_samples=6,
                maximum_age_sec=.3, maximum_gap_sec=.25)

    def await_initial(self):
        deadline = time.monotonic()+8
        last = 'waiting'
        while time.monotonic() < deadline:
            try:
                self.guard()
                result = self.stationary(max(self.started_ns, self.epoch))
                with self.lock:
                    refs = [m for m in self.reference if m.header.stamp.to_nsec() >=
                            result['stationary_window_start_ns']]
                    if len(refs) < 3 or (refs[-1].header.stamp-refs[0].header.stamp).to_sec() < .6:
                        raise RuntimeError('insufficient actually written SDK hold window')
                    if rospy.Time.now().to_nsec()-refs[-1].header.stamp.to_nsec() > 300_000_000:
                        raise RuntimeError('SDK hold evidence stale')
                    q = positions(refs[-1])
                    if any(wire_words(positions(m)) != wire_words(q) or
                           positions(m)[6] != q[6] for m in refs):
                        raise RuntimeError('SDK hold changed during baseline window')
                    self.frozen_epoch = self.epoch
                    self.expected_words, self.expected_gripper = wire_words(q), float(q[6])
                    return q, result
            except (RuntimeError, ValueError) as exc:
                last = str(exc)
                if self.fault:
                    raise
            time.sleep(.03)
        raise RuntimeError('initial evidence unavailable: '+last)

    def snapshot(self, label):
        # Wait for new evidence rather than missing a <30 ms fresh-pair window.
        # Four seconds bounds acquisition only; every candidate still has <=350 ms age.
        deadline = min(self.deadline, time.monotonic()+4.)
        while True:
            with self.lock:
                generation = self.pair_generation
            try:
                return self._snapshot_once(label)
            except RuntimeError as exc:
                if (not str(exc).startswith('fresh paired board quality/image/TF unavailable')
                        or time.monotonic() >= deadline):
                    raise
                self.guard()
                with self.pair_changed:
                    # A callback between the attempt and wait cannot be lost.
                    self.pair_changed.wait_for(
                        lambda: self.pair_generation != generation or self.closed or self.fault,
                        timeout=max(0., deadline-time.monotonic()))

    def _snapshot_once(self, label):
        self.guard()
        parameters = CAMERA.read_runtime_camera_parameters('/camera', rospy.get_name(),
                                                           rosgraph.get_master_uri())
        evidence = CAMERA.runtime_color_model_evidence(parameters, 'camera_link', RGB)
        model = CAMERA.RuntimeColorModel(evidence, sdk=self.camera_backend)
        if self.model_fingerprint is not None and model.fingerprint != self.model_fingerprint:
            raise RuntimeError('runtime camera model changed')
        with self.lock:
            options = paired_board_candidates(self.rgb, self.quality, self.board,
                rospy.Time.now().to_nsec(), model.fingerprint)
            if not options:
                raise RuntimeError('fresh paired board quality/image/TF unavailable; no old board pose fallback')
            ns, quality, image, board_msg = options[-1]
            if self.board_frame and quality['board_frame'] != self.board_frame:
                raise RuntimeError('board frame changed')
            closest = min(self.accepted, key=lambda m: abs(m.header.stamp.to_nsec()-ns))
            if abs(closest.header.stamp.to_nsec()-ns) > 100_000_000:
                raise RuntimeError('board/encoder time separation exceeds 100 ms')
            q = positions(closest)
        self.write_bag('/bounded_response/snapshot_rgb', image, rospy.Time.now())
        # Exact image-time base/tool TF and current live hand-eye. Never use Time(0) for board.
        stamp = rospy.Time(ns//1_000_000_000, ns % 1_000_000_000)
        base_tool = self.buffer.lookup_transform('base_link', 'tool0', stamp, rospy.Duration(.15))
        camera_tf = self.buffer.lookup_transform('tool0', 'camera_link', stamp, rospy.Duration(.15))
        a, x, b = transform(base_tool), transform(camera_tf), transform(board_msg)
        fk_error = float(np.linalg.norm(a[:3,3]-self.fk(q[:6])[:3,3]))
        if fk_error > .002:
            raise RuntimeError('image-time TF versus associated accepted FK differs >2 mm')
        y = a@x@b
        self.board_frame, self.model_fingerprint = quality['board_frame'], model.fingerprint
        geometry = dict(q=q, X=x, Y=y, model=model, source_stamp_ns=ns)
        row = dict(source_stamp_ns=ns, accepted_stamp_ns=closest.header.stamp.to_nsec(),
            quality=quality, camera_parameters=parameters, T_base_tool=a.tolist(),
            T_tool_camera=x.tolist(), T_camera_board=b.tolist(), T_base_board=y.tolist(),
            accepted_positions_rad=q.tolist(), fk_translation_match_m=fk_error)
        write_json(self.out/(label+'_geometry.json'), row)
        self.write_bag('/bounded_response/base_tool', base_tool, rospy.Time.now())
        self.write_bag('/bounded_response/tool_camera', camera_tf, rospy.Time.now())
        return geometry

    def path(self, measured, target, geometry):
        j = self.args.joint-1
        validate_joint3_envelope(measured, self.joint3_response_bounds)
        validate_joint3_envelope(target, self.joint3_response_bounds)
        if abs(target[j]-measured[j]) > math.radians(3.5):
            raise RuntimeError('wire target is more than 3.5 degrees from measured joint')
        # Re-prove the complete original absolute interval with each fresh scene.
        # Centering return reservations on its new measured start would expand it.
        ends = [np.array(target[:6])]
        for endpoint in self.joint3_response_bounds:
            r=np.array(measured[:6]); r[j]=endpoint; ends.append(r)
        y, x, model = geometry['Y'], geometry['X'], geometry['model']
        normal = y[:3,2].copy()
        if normal[2] < 0:
            normal *= -1
        d = -float(normal@y[:3,3])
        corners = np.array([[cx,cy,0,1] for cx in (0,.18) for cy in (0,.135)])
        smallest_clearance, smallest_edge = math.inf, math.inf
        for end in ends:
            for alpha in np.linspace(0,1,17):
                q = measured[:6]*(1-alpha)+end*alpha
                if np.any(q < self.lo) or np.any(q > self.hi):
                    raise RuntimeError('reserved path exceeds joint limits')
                tool = self.fk(q)
                clearance = min(float(np.min(_box_corners(c,tool[:3,:3],size)@normal+d))
                    for _,c,size in _stage_boxes(tool,self.gripper,float(measured[6]),'y','z'))
                camera_points = (np.linalg.inv(tool@x)@y@corners.T).T[:,:3]
                if np.min(camera_points[:,0]) < .12:
                    raise RuntimeError('reserved board projection too close/behind camera')
                optical = np.column_stack((-camera_points[:,1],-camera_points[:,2],camera_points[:,0]))
                uv = np.asarray([model.sdk.rs2_project_point_to_pixel(model.intrinsics,p.tolist())
                                 for p in optical])
                ci = model.evidence['color_intrinsics']
                edge = float(min(uv[:,0].min(),ci['width']-1-uv[:,0].max(),
                                 uv[:,1].min(),ci['height']-1-uv[:,1].max()))
                if not np.isfinite([clearance,edge]).all() or clearance < .04 or edge < 20:
                    raise RuntimeError('reserved path violates 40 mm plane / 20 px board margin')
                request = GetStateValidityRequest(group_name='alicia')
                request.robot_state = RobotState()
                request.robot_state.joint_state = JointState(name=NAMES,position=list(q)+[float(measured[6])])
                if not self.validity(request).valid:
                    raise RuntimeError('current MoveIt scene rejects reserved path')
                smallest_clearance=min(smallest_clearance,clearance)
                smallest_edge=min(smallest_edge,edge)
        return dict(minimum_plane_clearance_m=smallest_clearance,
                    minimum_full_board_edge_px=smallest_edge,
                    response_reserved_degrees=4.8,
                    response_absolute_joint3_range_rad=list(self.joint3_response_bounds),
                    reservation='fixed initial measured absolute J3 interval, both directions',
                    moveit_samples=17*len(ends))

    def driver_connection(self):
        class Transport(xmlrpc.client.Transport):
            def make_connection(self,host):
                connection=super().make_connection(host)
                connection.timeout=.5
                return connection
        code,detail,uri=rospy.get_master().lookupNode('/alicia_d_driver_node')
        if code!=1:
            raise RuntimeError('driver node unavailable')
        with xmlrpc.client.ServerProxy(uri,transport=Transport()) as api:
            code,detail,rows=api.getBusInfo(rospy.get_name())
        if not (code==1 and any(len(row)>=6 and row[1] in
                (rospy.get_name(),rospy.get_node_uri()) and row[2]=='i' and
                row[4]=='/joint_commands' and row[5] for row in rows)):
            raise RuntimeError('actual driver command connection not established')

    def stable_hold_recording(self, duration):
        deadline = time.monotonic()+duration
        while time.monotonic() < deadline:
            self.guard()
            time.sleep(.02)

    def execute_step(self, label, target):
        if not self.args.execute or self.publisher is None:
            raise RuntimeError('prepare-only process cannot publish')
        self.guard()
        self.stationary(max(self.started_ns,self.epoch))
        geometry = self.snapshot(label)
        with self.lock:
            measured = positions(self.accepted[-1])
        details = self.path(measured,target,geometry)
        self.driver_connection()
        self.guard()
        # At most three observations may replace one that expired during disk/TF work.
        # This never renews the overall deadline, changes geometry, or retries a command.
        for observation_attempt in range(3):
            suffix = '_precommand' if observation_attempt == 0 else '_precommand_retry%d' % (observation_attempt+1)
            fresh=self.snapshot(label+suffix)
            if (np.linalg.norm(fresh['Y'][:3,3]-geometry['Y'][:3,3])>.003 or
                    not np.allclose(fresh['X'],geometry['X'],rtol=0,atol=1e-10)):
                raise RuntimeError('scene or hand-eye changed during path evaluation')
            # Revalidate the planning-to-publication gap against the original measured start.
            with self.lock:
                self.guard()
                source_ns = fresh['source_stamp_ns']
                source_age_ns = rospy.Time.now().to_nsec()-source_ns
                if source_ns < self.frozen_epoch or source_age_ns < 0:
                    raise RuntimeError('precommand board source future or outside control epoch')
                if np.max(np.abs(positions(self.accepted[-1])[:6]-measured[:6])) > math.radians(.15):
                    raise RuntimeError('measured pose changed while evaluating reserved path')
                if source_age_ns > 350_000_000:
                    continue
                if self.publisher.get_num_connections() < 1:
                    raise RuntimeError('no joint command subscriber')
                msg = JointState()
                msg.header.stamp = rospy.Time.now()
                msg.header.frame_id = 'gui_direct'
                msg.name, msg.position = NAMES, target.tolist()
                msg.effort = [0.]*7; msg.effort[self.args.joint-1]=1.
                sent_ns = msg.header.stamp.to_nsec()
                # The next reference may still be an in-flight old keepalive; switch
                # expected words only after observing the first post-command target.
                validate_joint3_command(target, positions(self.reference[-1]),
                    self.baseline_wire_q, label, self.publish_count)
                prior_words = list(self.expected_words)
                self.expected_words = None
                self.publish_count += 1
                self.publisher.publish(msg)
                self.event('command',label=label,edited_joint=self.args.joint,
                    command_stamp_ns=sent_ns,starting_measured_rad=measured.tolist(),
                    target_rad=target.tolist(),expected_wire_words=wire_words(target),
                    precommand_observation_attempt=observation_attempt+1,
                    precommand_source_age_ms=source_age_ns/1e6,**details)
            break
        else:
            raise RuntimeError('precommand board source expired across three bounded acquisitions')
        deadline = time.monotonic()+12
        target_seen=False; last='no stationary directional response'; last_reference_ns=sent_ns-1
        j=self.args.joint-1
        sign=1 if target[j]>measured[j] else -1
        while time.monotonic()<deadline:
            self.guard()
            with self.lock:
                current=positions(self.accepted[-1])
                newrefs=[m for m in self.reference if m.header.stamp.to_nsec()>last_reference_ns]
                for reference in newrefs:
                    w=wire_words(positions(reference))
                    last_reference_ns=reference.header.stamp.to_nsec()
                    if w==wire_words(target):
                        target_seen=True
                    elif (target_seen or any(w[k]!=prior_words[k] for k in range(6) if k!=j)
                          or not min(prior_words[j],wire_words(target)[j])<=w[j]<=max(prior_words[j],wire_words(target)[j])):
                        raise RuntimeError('unexpected SDK target outside single-axis segment')
                    if abs(positions(reference)[6]-self.expected_gripper)>1e-10:
                        raise RuntimeError('gripper wire changed')
                if target_seen:
                    self.expected_words=wire_words(target)
            delta=current[:6]-measured[:6]
            validate_joint3_envelope(current, self.joint3_response_bounds)
            if abs(delta[j])>math.radians(4.8) or sign*delta[j]<-math.radians(.5):
                raise RuntimeError('joint response outside reserved direction/amplitude')
            if max(abs(np.delete(delta,j)))>math.radians(.4):
                raise RuntimeError('unedited joint moved >0.4 degrees with held targets')
            try:
                stationary=self.stationary(sent_ns)
            except ValueError as exc:
                last=str(exc)
            else:
                if (target_seen and sign*delta[j] >= .003 and
                        self.state.get('status','').startswith('CONFIRMED:')):
                    self.event('settled',label=label,actual_delta_deg=np.degrees(delta).tolist(),
                        nominal_residual_deg=np.degrees(target[:6]-current[:6]).tolist(),
                        stationary=stationary)
                    self.stable_hold_recording(3.)
                    return
            time.sleep(.02)
        raise RuntimeError('no bounded stationary response: '+last)

    def run(self):
        base, stationary = self.await_initial()
        self.baseline_wire_q = base.copy()
        geometry=self.snapshot('baseline')
        with self.lock:
            self.guard()
            measured=positions(self.accepted[-1])
            self.baseline_measured_q = measured.copy()
            self.joint3_response_bounds = (float(measured[2]-math.radians(4.8)),
                                           float(measured[2]+math.radians(4.8)))
        candidates=[]
        count=max(1,int(math.floor(math.radians(self.args.step_deg)/QUANTUM)))
        for direction in (1,-1):
            target=base.copy(); target[self.args.joint-1]+=direction*count*QUANTUM
            try:
                details=self.path(measured,target,geometry)
                candidates.append((details['minimum_full_board_edge_px'],target,details))
            except RuntimeError as exc:
                self.event('candidate_rejected',direction=direction,reason=str(exc))
        if not candidates:
            raise RuntimeError('no admissible bounded single-joint candidate')
        _,target,details=max(candidates,key=lambda x:x[0])
        plan=dict(schema=2,execute=self.args.execute,joint=self.args.joint,
            frozen_epoch_ns=self.frozen_epoch,source='fresh current process evidence',
            initial_measured_rad=self.baseline_measured_q.tolist(),
            response_absolute_joint3_range_rad=list(self.joint3_response_bounds),
            starting_sdk_rad=base.tolist(),starting_sdk_words=wire_words(base),
            outbound_sdk_rad=target.tolist(),outbound_sdk_words=wire_words(target),
            return_sdk_rad=base.tolist(),return_sdk_words=wire_words(base),
            stationary=stationary,geometry=details,
            recovery='none; no bootstrap/sync/enable path exists',
            bootstrap_pending=False,
            commanded_joint_delta_limit_deg=2., actual_response_reservation_deg=4.8,
            maximum_commands=2, overall_deadline_sec=57.,
            archive_policy='local_only',upload_enabled=False,delete_enabled=False)
        write_json(self.out/'plan.json',plan)
        if not self.args.execute:
            self.event('prepared_no_motion',plan=str(self.out/'plan.json'))
            return
        # A fresh execution process always regenerates this plan; no --plan input exists.
        self.execute_step('outbound',target)
        self.execute_step('return_original_sdk',base)
        return_stationary=self.stationary(max(self.started_ns,self.epoch))
        returned=self.snapshot('return_post_settle')
        delta=returned['Y'][:3,3]-geometry['Y'][:3,3]
        angle=math.degrees(math.acos(float(np.clip(
            (np.trace(geometry['Y'][:3,:3].T@returned['Y'][:3,:3])-1)/2,-1.,1.))))
        comparison=dict(kind='same_sdk_return_fixed_board_diagnostic_not_calibration',
            joint=self.args.joint,epoch_ns=self.frozen_epoch,
            baseline_source_stamp_ns=geometry['source_stamp_ns'],
            return_source_stamp_ns=returned['source_stamp_ns'],
            baseline_T_base_board=geometry['Y'].tolist(),
            return_T_base_board=returned['Y'].tolist(),
            delta_base_board_xyz_mm=(delta*1000).tolist(),
            displacement_base_board_mm=float(np.linalg.norm(delta)*1000),
            orientation_difference_deg=angle,
            baseline_accepted_positions_rad=geometry['q'].tolist(),
            return_accepted_positions_rad=returned['q'].tolist(),
            accepted_joint_delta_deg=np.degrees(returned['q'][:6]-geometry['q'][:6]).tolist(),
            accepted_gripper_delta_m=float(returned['q'][6]-geometry['q'][6]),
            original_sdk_words=wire_words(base),
            final_sdk_words=wire_words(positions(self.reference[-1])),
            handeye_unchanged=bool(np.allclose(geometry['X'],returned['X'],rtol=0,atol=1e-10)),
            return_stationary=return_stationary,transform_refitted=False,
            interpretation='One same-command return observation; does not certify broad-pose calibration.')
        write_json(self.out/'same_sdk_return_comparison.json',comparison)
        self.event('same_sdk_return_observed',**comparison)
        self.event('completed_same_sdk_return',original_words=wire_words(base),
            final_words=wire_words(positions(self.reference[-1])),
            final_measured_rad=positions(self.accepted[-1]).tolist())

    def close(self):
        with self.pair_changed:
            self.closed=True
            self.pair_changed.notify_all()
            self.events.close()
        with self.bag_lock:
            self.bag.close()
        for sub in self.subs:
            sub.unregister()


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execute',action='store_true')
    parser.add_argument('--step-deg',type=float,default=2.,
                        help='Requested SDK increment; exactly 2 degrees maximum, quantized down.')
    args=parser.parse_args()
    args.joint=3
    if not math.isfinite(args.step_deg) or not 2 <= args.step_deg <= 2:
        parser.error('J3 diagnostic permits exactly --step-deg 2 (22 wire counts)')
    if not os.path.ismount(str(MOUNT)) or not ARCHIVE.is_dir():
        raise RuntimeError('SanStand archive unavailable; no system-disk fallback')
    tag=datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')
    out=ARCHIVE/('bounded_joint3_response_'+tag+'_'+str(os.getpid()))
    out.mkdir()
    (out/'ros').mkdir()
    os.environ['ROS_LOG_DIR']=str(out/'ros')
    (out/'diagnostic_source.py').write_bytes(Path(__file__).read_bytes())
    rospy.init_node('bounded_joint3_response_diagnostic',anonymous=True,disable_signals=True)
    experiment=None
    def deadline_signal(signum,frame):
        raise RuntimeError('57-second overall deadline exhausted; no further command')
    signal.signal(signal.SIGALRM,deadline_signal)
    signal.setitimer(signal.ITIMER_REAL,57.)
    terminal=dict(status='failed',execute=args.execute,commands_authorized=2 if args.execute else 0)
    try:
        experiment=Diagnostic(args,out)
        experiment.run()
        terminal['status']='completed' if args.execute else 'prepared_no_motion'
    except BaseException as exc:
        terminal['reason']=str(exc)
        if experiment:
            experiment.event('no_further_commands',reason=str(exc),
                             recovery_attempted=False,stop_or_disable_sent=False)
        raise
    finally:
        signal.setitimer(signal.ITIMER_REAL,0)
        if experiment:
            terminal['commands_published'] = experiment.publish_count
            experiment.close()
        write_json(out/'terminal.json',terminal)
        print('SESSION '+str(out),flush=True)


if __name__=='__main__':
    main()
