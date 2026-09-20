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
from pathlib import Path
import threading
import time

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
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True)
    parser.add_argument('--duration', type=float, default=600.)
    args = parser.parse_args()
    output = Path(args.output)
    if output.exists():
        raise FileExistsError(output)
    if not 0 < args.duration <= 1800:
        raise ValueError('duration must be in (0, 1800] seconds')
    audit_directory = output.parent / (output.stem + '_audits')
    audit_directory.mkdir(exist_ok=True)
    rospy.init_node('grasp_keyframe_recorder_readonly', anonymous=True)
    bag = rosbag.Bag(str(output), 'w', compression=rosbag.Compression.LZ4)
    lock = threading.Lock()
    pending, written = OrderedDict(), OrderedDict()
    state = {'active_seen': False, 'terminal_at': None, 'count': 0, 'closed': False}

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
                if message.active:
                    state['active_seen'] = True
                elif state['active_seen'] and state['terminal_at'] is None:
                    state['terminal_at'] = time.monotonic()
        # Outside the bag lock: disk copying must not block joint telemetry.
        if topic == '/grasp_6d/gate_audit':
            try:
                path = archive_gate_audit(message.data, audit_directory)
                print('AUDIT_ARCHIVED path=%s' % path, flush=True)
            except (OSError, KeyError, TypeError, ValueError) as exc:
                print('AUDIT_ARCHIVE_FAILED error=%s' % exc, flush=True)

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
    try:
        while not rospy.is_shutdown() and time.monotonic() - started < args.duration:
            with lock:
                terminal_at = state['terminal_at']
            if terminal_at is not None and time.monotonic() - terminal_at >= 3.:
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
            rospy.sleep(.2)
    finally:
        for subscriber in subscribers:
            subscriber.unregister()
        with lock:
            state['closed'] = True
            bag.close()
        print('KEYFRAME_RECORDING_FINISHED count=%d path=%s' % (
            state['count'], output), flush=True)


if __name__ == '__main__':
    main()
