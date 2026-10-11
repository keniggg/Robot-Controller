#!/usr/bin/env python3
"""Passively record exact detected RGB-D/mask frames plus continuous TF.

No publishers, services, motion, or enable commands. Sensor messages are kept
at their original source stamps; a keyframe is recorded only when RGB, depth,
mask and detection all agree exactly. This avoids a full-rate RGB bag rolling
away the very source frames needed to replay a grasp.

During a new observation-measurement handoff, a separate bag-only 5 Hz stream
also retains exact RGB-D pairs without requiring detection or a mask.
"""

import argparse
from collections import OrderedDict
import hashlib
import json
import os
from pathlib import Path
import shutil
import signal
import threading
import time

from run_recorded_grasp import atomic_json, classify_result, observe_task_state, utc_now

import rosbag
import rospy
from sensor_msgs.msg import Image, CameraInfo, JointState
from std_msgs.msg import String, Bool, Header, UInt8
from control_msgs.msg import (
    JointTrajectoryControllerState, FollowJointTrajectoryActionGoal,
    FollowJointTrajectoryActionResult,
)
from actionlib_msgs.msg import GoalStatusArray
from trajectory_msgs.msg import JointTrajectory
from rosgraph_msgs.msg import Log
from diagnostic_msgs.msg import DiagnosticArray
from tf2_msgs.msg import TFMessage
from alicia_flexible_grasp_supervisor.msg import (
    GraspState, Grasp6DPlan, NearFieldPlanningPhase, ObjectGeometry, ObjectPose,
)

SOURCES = {
    '/supervisor/camera/color/image_raw': Image,
    '/supervisor/camera/depth/image_raw': Image,
    '/perception/object_mask': Image,
    '/perception/object': ObjectPose,
}

MEASUREMENT_REQUEST_TOPIC = '/grasp_mode/observe_return_measurement_request'
OBSERVATION_RGBD_TOPICS = {
    '/supervisor/camera/color/image_raw': '/experiment/observation_rgbd/color',
    '/supervisor/camera/depth/image_raw': '/experiment/observation_rgbd/depth',
}


class ObservationRgbdWindow:
    """Bounded diagnostic sampling only; messages never become execution input.

    The caller owns the existing bag lock. Neither callbacks nor this class
    create a sensor message, change its header, or fill a missing component.
    """
    _BINDING = ('lease_id', 'selection', 'session_id', 'target_epoch', 'target_track_id',
        'origin_plan_id', 'origin_snapshot_stamp_ns', 'view_plan_id', 'view_source_stamp_ns',
        'view_index', 'plan_sha256', 'control_epoch_ns', 'deadline_ns', 'motion_kind')

    def __init__(self, minimum_stamp_ns):
        self.minimum_stamp_ns = int(minimum_stamp_ns)
        self.request = None
        self.pending = OrderedDict()
        self.seen_leases = set()
        self.last_request_stamp_ns = 0
        self.last_pair_stamp_ns = 0
        self.last_pair_wall = float('-inf')
        self.deadline_ns = 0
        self.deadline_wall = 0.
        self.count = 0

    def _close(self):
        self.request = None
        self.pending.clear()

    def active(self, now_ns, now_wall):
        if self.request is not None and (now_ns >= self.deadline_ns
                or now_wall >= self.deadline_wall or now_ns < self.request['stamp_ns']):
            self._close()
        return self.request is not None

    def update_request(self, data, now_ns, now_wall):
        self.active(now_ns, now_wall)
        try:
            request = json.loads(data)
            if (not isinstance(request, dict) or type(request.get('version')) is not int
                    or request['version'] != 1 or any(key not in request for key in self._BINDING)
                    or request.get('metric_geometry_authority') is not False
                    or request.get('motion_authorized') is not False):
                return False
            stamp, deadline = request['stamp_ns'], request['deadline_ns']
            selected = request['selection']
            if (type(stamp) is not int or type(deadline) is not int
                    or not self.minimum_stamp_ns <= stamp <= now_ns
                    or stamp <= self.last_request_stamp_ns
                    or not isinstance(selected, dict) or selected.get('mode') != 'unknown'
                    or selected.get('strategy') != 'observe_return'
                    or not isinstance(request.get('lease_id'), str) or not request['lease_id']
                    or not isinstance(request.get('request_id'), str) or not request['request_id']):
                return False
            state = request.get('state')
            if self.request is not None:
                if any(request[key] != self.request[key] for key in self._BINDING):
                    return False
                if state in ('qualified', 'revoked'):
                    self.last_request_stamp_ns = stamp
                    self._close()
                    return True
                if state != 'requalifying' or self.request['state'] != 'executing':
                    return False
            elif state != 'executing' or request['lease_id'] in self.seen_leases:
                return False
            if not stamp <= now_ns < deadline or deadline-stamp > 600_000_000_000:
                return False
            cutoff = deadline
            if state == 'requalifying':
                arrival, capture = request.get('arrival_stamp_ns'), request.get('capture_deadline_ns')
                if (type(arrival) is not int or type(capture) is not int
                        or not 0 < arrival <= stamp <= now_ns < capture <= deadline
                        or capture-arrival > 10_000_000_000):
                    return False
                cutoff = capture
            local_deadline = now_wall + (cutoff-now_ns)*1e-9
            if self.request is not None:
                local_deadline = min(local_deadline, self.deadline_wall)
            self.request = request
            self.seen_leases.add(request['lease_id'])
            self.deadline_ns, self.deadline_wall = cutoff, local_deadline
            self.last_request_stamp_ns = stamp
            # A transition cannot pair a pre-transition component with a later
            # source. Source matching below is still exact integer nanoseconds.
            self.pending.clear()
            return True
        except (KeyError, TypeError, ValueError, OverflowError):
            return False

    def offer(self, topic, message, now_ns, now_wall):
        if topic not in OBSERVATION_RGBD_TOPICS or not self.active(now_ns, now_wall):
            return None
        stamp = message.header.stamp.to_nsec()
        if (type(stamp) is not int or not self.request['stamp_ns'] <= stamp <= now_ns
                or stamp >= self.deadline_ns or stamp-self.last_pair_stamp_ns < 200_000_000
                or now_wall-self.last_pair_wall < .2):
            return None
        for key in tuple(self.pending):
            if now_wall-self.pending[key]['received'] > 1.:
                self.pending.pop(key)
        entry = self.pending.setdefault(stamp, dict(received=now_wall, messages={}))
        entry['messages'].setdefault(topic, message)
        while len(self.pending) > 16:
            self.pending.popitem(last=False)
        if len(entry['messages']) != 2:
            return None
        self.pending.pop(stamp, None)
        original = entry['messages']
        color, depth = [original[key] for key in OBSERVATION_RGBD_TOPICS]
        if ((color.width, color.height, color.header.frame_id)
                != (depth.width, depth.height, depth.header.frame_id)):
            return None
        self.last_pair_stamp_ns, self.last_pair_wall = stamp, now_wall
        self.count += 1
        metadata = dict(schema_version=1, recording_scope='diagnostic_only',
            execution_source=False, metric_geometry_authority=False, motion_authorized=False,
            pairing='exact_original_header_stamp_ns', source_stamp_ns=stamp,
            original_topics={destination: source for source, destination in OBSERVATION_RGBD_TOPICS.items()},
            measurement_request=self.request, sample_rate_limit_hz=5., pair_count=self.count)
        return [(destination, original[source]) for source, destination in OBSERVATION_RGBD_TOPICS.items()], metadata


def camera_runtime_metadata_snapshot(parameters, read_interval, previous_content=None,
                                     read_error=None):
    """Return changed parameter evidence, never an image-synchronized CameraInfo.

    Deduplication excludes the read interval so an unchanged profile is not
    rewritten every second. Preserve the complete namespace, including native
    and aligned geometry, distortion model, SDK provenance and depth units.
    Availability describes parameter content only, not a live-camera check.
    """
    profile = parameters.get('runtime_profile') if isinstance(parameters, dict) else None
    if read_error == 'parameter_read_pending':
        reason = 'parameter_read_pending'
    elif read_error:
        reason = 'parameter_read_failed'
    elif not isinstance(parameters, dict):
        reason = 'camera_parameter_missing_or_invalid'
    elif not isinstance(profile, dict):
        reason = 'active_profile_missing'
    elif profile.get('source') != 'realsense_active_profile':
        reason = 'active_profile_unavailable'
    elif (not isinstance(profile.get('geometry_intrinsics'), dict)
          or 'depth_scale_m_per_unit' not in profile):
        reason = 'active_geometry_missing'
    else:
        reason = None
    content = {
        'schema_version': 1,
        'source_parameter': '/camera',
        'status': 'unavailable' if reason else 'available',
        'unavailable_reason': reason,
        'read_error': read_error,
        'availability_scope': 'parameter_content_only; camera_liveness_not_checked',
        'time_semantics': 'parameter_read_interval; not_image_or_exposure_time',
        'image_time_association': 'none',
        'unit_conventions': {
            'intrinsics_focal_length_and_principal_point': 'pixels',
            'depth_scale': 'metres_per_depth_unit',
            'runtime_profile.depth_scale_m_per_unit': 'metres_per_depth_unit',
            'runtime_profile.depth_to_color_extrinsics.translation_m': 'metres',
        },
        'parameters': parameters,
    }
    try:
        serialized = json.dumps(content, sort_keys=True, allow_nan=False)
    except (TypeError, ValueError) as exc:
        # Malformed parameters must not stop recording valid sensor keyframes.
        content.update(status='unavailable', unavailable_reason='parameter_not_json_serializable',
                       read_error='%s: %s' % (type(exc).__name__, exc), parameters=None)
        serialized = json.dumps(content, sort_keys=True, allow_nan=False)
    if serialized == previous_content:
        return serialized, None
    payload = json.loads(serialized)
    payload['parameter_read_interval'] = read_interval
    return serialized, json.dumps(payload, sort_keys=True, allow_nan=False)


def read_camera_parameters():
    """Use a private read-only connection, never rospy's shared RPC lock."""
    caller_id = rospy.get_name()
    from rosgraph import get_master_uri
    from xmlrpc.client import ServerProxy

    # rospy.get_param shares one locked XMLRPC proxy per master URI. A stuck
    # background call on it could also block the existing foreground diagnostic
    # read. This connection belongs only to the camera metadata worker.
    with ServerProxy(get_master_uri()) as master:
        code, message, parameters = master.getParam(caller_id, '/camera')
    if code == 1:
        return parameters
    if code == -1:
        return None  # ROS parameter server reports a missing namespace.
    raise RuntimeError('camera getParam failed: %s' % message)


class CameraRuntimeMetadataReader:
    """One daemon isolates a possibly stuck, read-only parameter RPC.

    At most one request is outstanding. Polling never starts another worker;
    pending reads have no fabricated completion time and can recover later.
    """

    def __init__(self, read_parameter, read_clock):
        self._read_parameter = read_parameter
        self._read_clock = read_clock
        self._lock = threading.Lock()
        self._requested = threading.Event()
        self._ready = threading.Event()
        self._closed = False
        self._outstanding = False
        self._submitted = None
        self._started = None
        self._result = None
        self._worker = threading.Thread(
            target=self._run, name='camera-runtime-metadata-reader', daemon=True)
        self._worker.start()

    def request(self):
        with self._lock:
            if self._closed or self._outstanding:
                return False
            self._outstanding = True
            self._submitted = self._read_clock()
            self._started = None
            self._ready.clear()
            self._requested.set()
            return True

    def poll(self, wait_seconds=0.):
        # Only startup uses a bounded wait. Periodic polls are non-blocking.
        self._ready.wait(timeout=wait_seconds)
        with self._lock:
            if self._closed:
                return None
            if self._result is not None:
                result = self._result
                self._result = None
                self._outstanding = False
                self._ready.clear()
                return result
            if self._outstanding:
                return (None, {
                    'requested': self._submitted, 'started': self._started,
                    'finished': None, 'observed_pending': self._read_clock(),
                }, 'parameter_read_pending')
            return None

    def close(self):
        # Never join a potentially stuck RPC. The sole daemon cannot write the
        # bag, and discards any late result after recorder shutdown.
        with self._lock:
            self._closed = True
            self._requested.set()
            self._ready.set()

    def _run(self):
        while True:
            self._requested.wait()
            self._requested.clear()
            with self._lock:
                if self._closed:
                    return
                self._started = self._read_clock()
                started = self._started
                submitted = self._submitted
            parameters, read_error = None, None
            try:
                parameters = self._read_parameter()
            except Exception as exc:
                read_error = '%s: %s' % (type(exc).__name__, exc)
            finished = self._read_clock()
            with self._lock:
                if self._closed:
                    return
                self._result = (parameters, {
                    'requested': submitted, 'started': started, 'finished': finished,
                }, read_error)
                self._ready.set()


def archive_gate_audit(message_data, directory):
    """Freeze only the exact committed payload; never accept a replaced latest file."""
    reference = json.loads(message_data)
    expected = str(reference['report_sha256'])
    payload = Path(reference['report_path']).read_bytes()
    if hashlib.sha256(payload).hexdigest() != expected:
        raise ValueError('gate audit reference/payload hash mismatch')
    destination = directory / (expected + '.json')
    try:
        with destination.open('xb') as handle:
            handle.write(payload)
    except FileExistsError:
        if destination.read_bytes() != payload:
            raise ValueError('existing archive differs from committed payload')
    return destination


def main():
    from grasp_archive import begin_record, finalize_record, load_config, load_manifest

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True,
                        help='bag filename/root hint; standalone recordings get a unique child directory')
    parser.add_argument('--duration', type=float, default=600.)
    parser.add_argument('--record-dir', help='existing managed directory allocated by the wrapper')
    parser.add_argument('--archive-root', help='standalone record root (default: output parent)')
    parser.add_argument('--config', help='archive config (default: record root/archive_config.json)')
    parser.add_argument('--min-free-bytes', type=int, help='override startup free-space reserve')
    parser.add_argument('--defer-finalize', action='store_true',
                        help='wrapper seals all logs and finalizes after this process closes its bag')
    args = parser.parse_args()
    if not 0 < args.duration <= 1800:
        raise ValueError('duration must be in (0, 1800] seconds')
    if args.defer_finalize and not args.record_dir:
        parser.error('--defer-finalize requires --record-dir')
    requested_output = Path(args.output).expanduser().resolve()
    if requested_output.suffix != '.bag':
        parser.error('--output must name an original .bag recording')
    root = (Path(args.record_dir).expanduser().resolve().parent if args.record_dir else
            Path(args.archive_root).expanduser().resolve() if args.archive_root else requested_output.parent)
    config = load_config(args.config or root / 'archive_config.json')
    if args.min_free_bytes is not None:
        if args.min_free_bytes < 0:
            parser.error('--min-free-bytes must be nonnegative')
        config['min_free_bytes'] = args.min_free_bytes
    if args.record_dir:
        record_dir = Path(args.record_dir).expanduser().resolve()
        manifest = load_manifest(record_dir)
        if manifest.get('state') != 'recording':
            parser.error('--record-dir must be an active managed recording')
        if requested_output.parent != record_dir:
            parser.error('--output must be directly inside --record-dir')
    else:
        record_dir = begin_record(root, metadata={
            'owner_pid': os.getpid(), 'recorder': Path(__file__).name,
            'recording_duration_seconds': args.duration,
        }, config=config)
    output = record_dir / requested_output.name
    if output.exists():
        raise FileExistsError(output)
    audit_directory = output.parent / (output.stem + '_audits')
    audit_directory.mkdir(exist_ok=True)
    rospy.init_node('grasp_keyframe_recorder_readonly', anonymous=True, disable_signals=True)
    bag = rosbag.Bag(str(output), 'w', compression=rosbag.Compression.LZ4)
    lock = threading.Lock()
    callbacks_done = threading.Condition(lock)
    pending, written = OrderedDict(), OrderedDict()
    state = {'active_seen': False, 'terminal_at': None, 'count': 0, 'closed': False,
             'audit_callbacks': 0, 'audit_files': set()}
    outcome = {'started_at': utc_now(), 'active_seen': False,
               'minimum_stamp_ns': rospy.Time.now().to_nsec(), 'interrupted': False,
               'bag_closed': False, 'stop_reason': 'duration_elapsed'}
    observation_rgbd = ObservationRgbdWindow(outcome['minimum_stamp_ns'])
    signal_event = threading.Event()
    previous_handlers = {}

    def stop_recording(signum, _frame):
        outcome['interrupted'] = True
        outcome['stop_reason'] = 'signal_%s' % signal.Signals(signum).name
        signal_event.set()

    for signum in (signal.SIGINT, signal.SIGTERM):
        previous_handlers[signum] = signal.signal(signum, stop_recording)

    def source(topic, message):
        stamp = message.header.stamp.to_nsec()
        if stamp <= 0:
            return
        with lock:
            if state['closed'] or stamp in written:
                return
            now = time.monotonic()
            sampled = observation_rgbd.offer(topic, message, rospy.Time.now().to_nsec(), now)
            if sampled is not None:
                images, receipt = sampled
                received = rospy.Time.now()
                for name, original_message in images:
                    bag.write(name, original_message, received)
                bag.write('/experiment/observation_rgbd/metadata',
                          String(data=json.dumps(receipt, sort_keys=True, allow_nan=False)), received)
            entry = pending.setdefault(stamp, {'received': now, 'messages': {}})
            entry['messages'][topic] = message
            if len(entry['messages']) == len(SOURCES):
                received = rospy.Time.now()
                for name in SOURCES:
                    bag.write(name, entry['messages'][name], received)
                pending.pop(stamp)
                written[stamp] = None
                state['count'] += 1
                print('KEYFRAME stamp_ns=%d count=%d' % (stamp, state['count']), flush=True)
            # Bound memory for unmatched/full-rate camera frames. Never replace
            # missing frames with a nearest/latest sample.
            for key in list(pending):
                if now - pending[key]['received'] > 5.:
                    pending.pop(key)
            while len(pending) > 160:
                pending.popitem(last=False)
            while len(written) > 2048:
                written.popitem(last=False)

    def metadata(topic, message):
        with lock:
            if state['closed']:
                return
            bag.write(topic, message, rospy.Time.now())
            if topic == MEASUREMENT_REQUEST_TOPIC:
                observation_rgbd.update_request(message.data, rospy.Time.now().to_nsec(), time.monotonic())
            if isinstance(message, GraspState):
                observe_task_state(outcome, {
                    'stamp_ns': message.header.stamp.to_nsec(),
                    'state': message.state, 'active': bool(message.active),
                    'success': bool(message.success), 'message': message.message,
                }, outcome['minimum_stamp_ns'])
                if message.active:
                    state['active_seen'] = True
                elif state['active_seen'] and state['terminal_at'] is None:
                    state['terminal_at'] = time.monotonic()
            if topic == '/grasp_6d/gate_audit':
                state['audit_callbacks'] += 1
        # Outside the bag lock: disk copying must not block joint telemetry.
        if topic == '/grasp_6d/gate_audit':
            try:
                path = archive_gate_audit(message.data, audit_directory)
                with lock:
                    state['audit_files'].add(str(path.relative_to(record_dir)))
                print('AUDIT_ARCHIVED path=%s' % path, flush=True)
            except (OSError, KeyError, TypeError, ValueError) as exc:
                print('AUDIT_ARCHIVE_FAILED error=%s' % exc, flush=True)
            finally:
                with callbacks_done:
                    state['audit_callbacks'] -= 1
                    callbacks_done.notify_all()

    last_camera_content = None
    camera_reader = CameraRuntimeMetadataReader(
        # One namespace read preserves the camera node's atomic profile
        # replacement. This is passive parameter evidence, not calibration.
        read_camera_parameters,
        lambda: {'utc': utc_now(), 'ros_time_ns': rospy.Time.now().to_nsec()})

    def sample_camera_runtime_metadata(request=False, wait_seconds=0.):
        nonlocal last_camera_content
        if request:
            camera_reader.request()
        sample = camera_reader.poll(wait_seconds=wait_seconds)
        if sample is None:
            return
        parameters, read_interval, read_error = sample
        content, payload = camera_runtime_metadata_snapshot(
            parameters, read_interval, previous_content=last_camera_content,
            read_error=read_error)
        if payload is not None:
            metadata('/grasp_record/camera_runtime_metadata', String(data=payload))
            last_camera_content = content

    # Capture before subscribers/READY. A hung parameter service delays startup
    # by at most this wait; pending/unavailable metadata never gates keyframes.
    sample_camera_runtime_metadata(request=True, wait_seconds=.2)
    subscribers = []
    for topic, msg_type in SOURCES.items():
        subscribers.append(rospy.Subscriber(
            topic, msg_type, lambda m, t=topic: source(t, m),
            queue_size=4, buff_size=2**24))
    meta = [
        ('/supervisor/camera/color/camera_info', CameraInfo),
        ('/joint_states', JointState), ('/tf', TFMessage), ('/tf_static', TFMessage),
        # Actual accepted SDK packets only; driver receipt time is not a
        # synchronized device sampling clock. Heartbeat is recorded separately.
        ('/alicia_d/accepted_joint_states', JointState),
        # Keep input, successful quantized SDK output and measured feedback in
        # the same non-rolling attempt artifact. Heartbeat joint_states are not
        # independent hardware samples; SDK output is not measured motion.
        ('/joint_commands', JointState), ('/alicia_d/sdk_command', JointState),
        ('/alicia_d/control_reference', JointState),
        ('/alicia_d/control_reference_epoch', Header),
        ('/alicia_d/actuation_status', String),
        ('/alicia_d/motion_enabled', Bool),
        # Driver maps demonstration=true to torque-off. Record this input
        # passively so the experiment can audit the no-disable requirement.
        ('/demonstration', Bool),
        ('/alicia_d/protection_latched', Bool),
        ('/alicia_d/run_status', UInt8),
        ('/alicia_d/sdk_diagnostics', DiagnosticArray),
        ('/alicia_d/device_info', DiagnosticArray),
        ('/alicia_controller/state', JointTrajectoryControllerState),
        # The frozen-reference handoff uses this topic, not the action goal.
        ('/alicia_controller/command', JointTrajectory),
        ('/alicia_controller/follow_joint_trajectory/goal', FollowJointTrajectoryActionGoal),
        ('/alicia_controller/follow_joint_trajectory/result', FollowJointTrajectoryActionResult),
        ('/alicia_controller/follow_joint_trajectory/status', GoalStatusArray),
        ('/gui/joint_direct_mode', Bool),
        ('/grasp_6d/status', String),
        ('/grasp_6d/gate_audit', String),
        ('/perception/unknown/detector_status', String),
        ('/grasp_mode/observe_return_session', String),
        ('/grasp_mode/observe_return_ack', String),
        ('/grasp_mode/observe_return_measurement_request', String),
        ('/grasp_mode/observe_return_measurement_ack', String),
        ('/grasp_mode/contact_station_retirement_request', String),
        ('/grasp_mode/contact_station_retirement_ack', String),
        ('/grasp_mode/initial_preflight_request', String),
        ('/grasp_mode/initial_preflight_ack', String),
        ('/grasp_mode/initial_preflight_attempt', String),
        ('/rosout_agg', Log),
        ('/grasp/state', GraspState), ('/grasp/near_field_phase', NearFieldPlanningPhase),
        ('/grasp_6d/plan_enriched', Grasp6DPlan),
        ('/grasp_6d/preview_plan_enriched', Grasp6DPlan),
        ('/grasp_6d/object_geometry', ObjectGeometry),
    ]
    for topic, msg_type in meta:
        subscribers.append(rospy.Subscriber(
            topic, msg_type, lambda m, t=topic: metadata(t, m), queue_size=40))
    started = time.monotonic()
    last_parameter_sample = float('-inf')
    last_endpoint_payload = None
    last_space_warning = float('-inf')
    atomic_json(record_dir / 'recorder_ready.json', {
        'pid': os.getpid(), 'record_dir': str(record_dir), 'bag_path': str(output),
        'started_at': outcome['started_at'],
    })
    print('KEYFRAME_RECORDING_READY path=%s' % output, flush=True)
    try:
        while (not rospy.is_shutdown() and not signal_event.is_set()
               and time.monotonic() - started < args.duration):
            with lock:
                terminal_at = state['terminal_at']
            if terminal_at is not None and time.monotonic() - terminal_at >= 3.:
                outcome['stop_reason'] = 'observed_task_became_inactive'
                break
            # Poll completed camera reads without blocking callback recording.
            sample_camera_runtime_metadata()
            if time.monotonic() - last_parameter_sample >= 1.:
                last_parameter_sample = time.monotonic()
                camera_reader.request()
                # This diagnostic exists as a parameter, not a ROS topic.
                # Write a bag-only snapshot; do not publish or alter it.
                try:
                    payload = json.dumps(rospy.get_param(
                        '/grasp_6d/runtime_execution_error', {}), sort_keys=True)
                    if payload != last_endpoint_payload:
                        metadata('/experiment/runtime_execution_error_snapshot',
                                 String(data=payload))
                        last_endpoint_payload = payload
                except (rospy.ROSException, OSError, ValueError) as exc:
                    print('ENDPOINT_SNAPSHOT_FAILED error=%s' % exc, flush=True)
                free = shutil.disk_usage(record_dir).free
                reserve = int(config.get('min_free_bytes', 2 * 1024 ** 3))
                if free < reserve and time.monotonic() - last_space_warning >= 30.:
                    last_space_warning = time.monotonic()
                    print('RECORDING_DISK_SPACE_LOW free_bytes=%d reserve_bytes=%d; '
                          'next recording will be blocked; no arm command sent' % (free, reserve),
                          flush=True)
            rospy.sleep(.2)
        if rospy.is_shutdown() and not signal_event.is_set():
            outcome['interrupted'] = True
            outcome['stop_reason'] = 'ros_shutdown'
    except BaseException as exc:
        outcome['interrupted'] = True
        outcome['stop_reason'] = 'recorder_exception: %s: %s' % (type(exc).__name__, exc)
        raise
    finally:
        camera_reader.close()
        for subscriber in subscribers:
            subscriber.unregister()
        with callbacks_done:
            state['closed'] = True
            while state['audit_callbacks']:
                callbacks_done.wait()
            bag.close()
            outcome['bag_closed'] = True
            outcome['closed_files'] = [output.name] + sorted(state['audit_files'])
        outcome['finished_at'] = utc_now()
        outcome['keyframe_count'] = state['count']
        outcome['observation_rgbd_pair_count'] = observation_rgbd.count
        atomic_json(record_dir / 'recorder_outcome.json', outcome)
        print('KEYFRAME_RECORDING_FINISHED count=%d path=%s' % (
            state['count'], output), flush=True)
        for signum, handler in previous_handlers.items():
            signal.signal(signum, handler)
        if not args.defer_finalize:
            result, reason, evidence = classify_result(outcome, '')
            finalize_record(record_dir, result=result, failure_reason=reason, evidence=evidence,
                            closed_files=outcome['closed_files'] + [
                                'recorder_ready.json', 'recorder_outcome.json'])


if __name__ == '__main__':
    main()
