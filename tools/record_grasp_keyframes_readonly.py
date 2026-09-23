#!/usr/bin/env python3
"""Passively record exact detected RGB-D/mask frames plus continuous TF.

No publishers, services, motion, or enable commands. Sensor messages are kept
at their original source stamps; a keyframe is recorded only when RGB, depth,
mask and detection all agree exactly. This avoids a full-rate RGB bag rolling
away the very source frames needed to replay a grasp.
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
from std_msgs.msg import String, Bool, Header, UInt8, Float32MultiArray
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
        ('/alicia_d/temperatures_c', Float32MultiArray),
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
            if time.monotonic() - last_parameter_sample >= 1.:
                last_parameter_sample = time.monotonic()
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
