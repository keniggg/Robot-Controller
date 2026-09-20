#!/usr/bin/env python3
"""Bounded, purely subscribing terminal view of one live grasp attempt.

No publisher, service, action client, or control authority. JointState receipt
age is not the age of an independent serial sample (driver heartbeat repeats
measurements). SDK targets prove a successful position write, not movement.
"""

import argparse
import math
import threading
import time

import rospy
from rosgraph_msgs.msg import Log
from sensor_msgs.msg import JointState
from control_msgs.msg import FollowJointTrajectoryActionGoal, FollowJointTrajectoryActionResult
from std_msgs.msg import String, Bool
from alicia_flexible_grasp_supervisor.msg import GraspState, ObjectPose


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--duration', type=float, default=1800.)
    args = parser.parse_args()
    if not math.isfinite(args.duration) or not 0 < args.duration <= 1800:
        raise ValueError('duration must be in (0, 1800]')
    rospy.init_node('grasp_attempt_watch_readonly', anonymous=True)
    lock = threading.Lock()
    cached, counts = {}, {}
    state = {'active_seen': False, 'terminal_at': None, 'last_grasp': None}

    def emit(kind, value):
        print('%.6f %s %s' % (rospy.get_time(), kind, value), flush=True)

    def joints(kind, msg):
        with lock:
            cached[kind] = (dict(zip(msg.name, msg.position)), time.monotonic())
            counts[kind] = counts.get(kind, 0) + 1

    def grasp(msg):
        value = (msg.state, msg.stage, msg.active, msg.success, msg.message)
        with lock:
            if value != state['last_grasp']:
                emit('GRASP_STATE', value)
                state['last_grasp'] = value
            if msg.active:
                state['active_seen'] = True
            elif state['active_seen'] and state['terminal_at'] is None:
                state['terminal_at'] = time.monotonic()

    def scalar(kind, msg):
        with lock:
            if cached.get(kind) != msg.data:
                emit(kind, msg.data)
                cached[kind] = msg.data

    def logs(msg):
        # Ordinary streamed target updates can generate many IDLE->IDLE trim
        # bookkeeping messages. Full rosout remains in the keyframe recorder;
        # keep this terminal view focused on real phase changes and failures.
        if 'Endpoint trim transition' in msg.msg and 'phase=IDLE->IDLE' in msg.msg:
            return
        if msg.level >= Log.WARN or any(word in msg.msg for word in (
            'Endpoint trim', 'retimed', 'settled', 'request_completed',
            'observation roll selected', 'observation selected', 'VALID_3D', 'precision lease',
            'Measured observation CAD envelope', 'Fixed-goal endpoint correction',
            'Captured reached-view measured surface',
        )):
            emit('ROSLOG', '%s %s' % (msg.name, msg.msg))

    def detection(msg):
        with lock:
            cached['object'] = (msg.detected, round(msg.confidence, 3),
                                round(msg.depth_m, 4), msg.header.stamp.to_nsec())

    def action_goal(msg):
        trajectory = msg.goal.trajectory
        points = trajectory.points
        emit('ACTION_GOAL', dict(id=msg.goal_id.id,
            joints=list(trajectory.joint_names),
            duration_sec=points[-1].time_from_start.to_sec() if points else None,
            first_positions=list(points[0].positions) if points else [],
            final_positions=list(points[-1].positions) if points else [],
            path_position_tolerances={t.name: t.position for t in msg.goal.path_tolerance}))

    def action_result(msg):
        emit('ACTION_RESULT', dict(id=msg.status.goal_id.id, state=msg.status.status,
            error_code=msg.result.error_code, reason=msg.result.error_string))

    subscribers = []
    for kind, topic in (('measured', '/joint_states'),
                        ('accepted', '/alicia_d/accepted_joint_states'),
                        ('input', '/joint_commands'),
                        ('sdk', '/alicia_d/sdk_command')):
        subscribers.append(rospy.Subscriber(
            topic, JointState, lambda m, k=kind: joints(k, m), queue_size=80))
    subscribers.extend((
        rospy.Subscriber('/grasp/state', GraspState, grasp, queue_size=20),
        rospy.Subscriber('/alicia_d/actuation_status', String,
                         lambda m: scalar('ACTUATION', m), queue_size=20),
        rospy.Subscriber('/gui/joint_direct_mode', Bool,
                         lambda m: scalar('GUI_DIRECT', m), queue_size=20),
        rospy.Subscriber('/perception/object', ObjectPose, detection, queue_size=1),
        rospy.Subscriber('/rosout_agg', Log, logs, queue_size=300),
        rospy.Subscriber('/alicia_controller/follow_joint_trajectory/goal',
                         FollowJointTrajectoryActionGoal, action_goal, queue_size=10),
        rospy.Subscriber('/alicia_controller/follow_joint_trajectory/result',
                         FollowJointTrajectoryActionResult, action_result, queue_size=10),
    ))
    started = time.monotonic()
    try:
        while not rospy.is_shutdown() and time.monotonic() - started < args.duration:
            with lock:
                summary = {'messages': dict(counts), 'object': cached.get('object')}
                for kind in ('measured', 'accepted', 'input', 'sdk'):
                    if kind in cached:
                        values, received = cached[kind]
                        summary[kind] = [round(values.get('Joint%d' % j, float('nan')),
                                               7) for j in range(1, 7)]
                        summary[kind + '_receipt_age_sec'] = round(
                            time.monotonic() - received, 3)
                terminal = state['terminal_at']
            emit('TELEMETRY', summary)
            if terminal is not None and time.monotonic() - terminal >= 3.:
                break
            rospy.sleep(2.)
    finally:
        for subscriber in subscribers:
            subscriber.unregister()
        emit('WATCH_FINISHED', 'read-only subscription ended; no control command')


if __name__ == '__main__':
    main()
