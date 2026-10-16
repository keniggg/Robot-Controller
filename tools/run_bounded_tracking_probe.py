#!/usr/bin/env python3
"""One explicit small, free-space tracking experiment via the strict gateway.

Default is planning ONLY. --execute submits once, never retries, compensates,
closes the gripper or sends enable/disable/stop requests. This is a diagnostic
experiment, not a 6D grasp or proof of calibration. Requires operator control
handoff and physically clear workspace before invocation.
"""
import argparse
from copy import deepcopy
import json
import math
from pathlib import Path
import threading
import time

import numpy as np
import rosbag
import rospy
from control_msgs.msg import (JointTrajectoryControllerState,
                             FollowJointTrajectoryActionGoal, FollowJointTrajectoryActionResult)
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import DisplayTrajectory
from rosgraph_msgs.msg import Log
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Header, String, UInt8, Float32MultiArray
from tf.transformations import quaternion_from_matrix

from alicia_flexible_grasp.robot.stationary_following import ARM_NAMES, SDK_QUANTUM_RAD, SerialUrdfFk
from alicia_flexible_grasp.robot.observation_path_guard import (
    FrozenObservationScene, observation_endpoint_support_bound,
)
from alicia_flexible_grasp.grasp.gripper_geometry import (
    GripperGeometry, ANALYTICAL_MAX_INNER_GAP_M, ANALYTICAL_JAW_CLEARANCE_EACH_SIDE_M,
    ANALYTICAL_FINGER_SIZE_XYZ_M, ANALYTICAL_PALM_SIZE_XYZ_M, ANALYTICAL_SUPPORT_CLEARANCE_M,
)
from alicia_flexible_grasp_supervisor.msg import Grasp6DPlan, GraspState
from alicia_flexible_grasp_supervisor.srv import SetJointCommand


def sdk_words(q):
    q = np.asarray(q, dtype=float)
    if q.shape != (6,) or not np.all(np.isfinite(q)) or np.any(np.abs(q) > math.pi):
        raise ValueError('invalid SDK arm positions')
    return np.clip(np.floor(((q*180/math.pi+180)/360*4096)+.5), 0, 4095).astype(int)


def validate_small_plan(plan, sdk, requested_pose, fk):
    """Admit one local two-waypoint strict IK plan, not a large alternate branch."""
    tr = plan.joint_trajectory
    if (set(tr.joint_names) != set(ARM_NAMES) or len(tr.joint_names) != 6
            or len(tr.points) != 2
            or getattr(plan.multi_dof_joint_trajectory, 'points', [])):
        raise ValueError('probe requires one two-point, six-arm-joint local path')
    indices = [list(tr.joint_names).index(j) for j in ARM_NAMES]
    positions = np.array([[p.positions[i] for i in indices] for p in tr.points])
    if (not np.all(np.isfinite(positions))
            or np.max(np.abs(positions-np.asarray(sdk))) > .020):
        raise ValueError('probe path leaves the 0.020 rad local neighborhood')
    delta = sdk_words(positions[-1])-sdk_words(sdk)
    if np.max(np.abs(delta)) > 4 or not np.any(delta):
        raise ValueError('probe terminal SDK increment must be nonzero and <=4 counts per axis')
    actual = fk(positions[-1])
    position_error = np.linalg.norm(actual[:3, 3]-requested_pose[:3, 3])
    angle = math.acos(float(np.clip(
        (np.trace(actual[:3, :3].T @ requested_pose[:3, :3])-1)/2, -1, 1)))
    if position_error > .0001 or angle > .002:
        raise ValueError('displayed plan does not match the requested strict pose')
    return {'terminal_sdk_delta_counts': delta.tolist(),
            'terminal_joint_positions_rad': positions[-1].tolist(),
            'path_max_from_sdk_rad': float(np.max(np.abs(positions-np.asarray(sdk)))),
            'ik_position_error_m': float(position_error), 'ik_orientation_error_rad': angle}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execute', action='store_true')
    parser.add_argument('--output', required=True, help='new diagnostic bag; never overwritten')
    args = parser.parse_args()
    if Path(args.output).exists():
        raise FileExistsError(args.output)
    rospy.init_node('bounded_tracking_probe', anonymous=True)
    lock = threading.RLock()
    latest, displays, closed = {}, [], False
    bag = rosbag.Bag(args.output, 'w')

    def receive(topic, msg):
        nonlocal closed
        with lock:
            if closed:
                return
            bag.write(topic, msg, rospy.Time.now())
            latest[topic] = deepcopy(msg)
            if topic == '/move_group/display_planned_path':
                displays.append(deepcopy(msg))

    topics = {
        '/alicia_d/sdk_command': JointState, '/alicia_d/accepted_joint_states': JointState,
        '/alicia_d/control_reference': JointState, '/alicia_d/control_reference_epoch': Header,
        '/gui/joint_direct_mode': Bool, '/grasp/state': GraspState,
        '/alicia_controller/state': JointTrajectoryControllerState,
        '/alicia_controller/follow_joint_trajectory/goal': FollowJointTrajectoryActionGoal,
        '/alicia_controller/follow_joint_trajectory/result': FollowJointTrajectoryActionResult,
        '/alicia_d/actuation_status': String, '/alicia_d/run_status': UInt8,
        '/alicia_d/temperatures_c': Float32MultiArray,
        '/move_group/display_planned_path': DisplayTrajectory,
        '/grasp_6d/plan_enriched': Grasp6DPlan, '/joint_commands': JointState, '/rosout_agg': Log,
    }
    subs = [rospy.Subscriber(topic, typ, lambda m, t=topic: receive(t, m), queue_size=80)
            for topic, typ in topics.items()]

    def fresh_arm(topic, frame):
        msg = deepcopy(latest.get(topic))
        if msg is None or msg.header.frame_id != frame:
            raise ValueError('missing '+frame)
        if not 0 <= rospy.get_time()-msg.header.stamp.to_sec() <= .5:
            raise ValueError('stale or future '+frame)
        if len(msg.name) != len(msg.position) or len(set(msg.name)) != len(msg.name):
            raise ValueError('ambiguous joint values')
        values = dict(zip(msg.name, msg.position))
        return np.array([values[j] for j in ARM_NAMES]), msg

    def ownership():
        mode = latest.get('/gui/joint_direct_mode')
        state = latest.get('/grasp/state')
        epoch = latest.get('/alicia_d/control_reference_epoch')
        if mode is None or mode.data or rospy.get_param('/gui/joint_direct_mode', False) is True:
            raise ValueError('manual control not released')
        if state is None or state.active:
            raise ValueError('explicit inactive task state required')
        if epoch is None or epoch.frame_id != 'sdk_reference_epoch' or epoch.stamp.to_nsec() <= 0:
            raise ValueError('no current control reference epoch')
        return epoch.stamp.to_nsec()

    try:
        deadline = time.monotonic()+8
        while time.monotonic() < deadline and not rospy.is_shutdown():
            with lock:
                if all(t in latest for t in ('/alicia_d/sdk_command', '/alicia_d/accepted_joint_states',
                                              '/gui/joint_direct_mode', '/alicia_d/control_reference_epoch',
                                              '/grasp_6d/plan_enriched', '/grasp/state')):
                    break
            time.sleep(.05)
        with lock:
            epoch = ownership()
            sdk, wire = fresh_arm('/alicia_d/sdk_command', 'sdk_transmitted')
            accepted, measured = fresh_arm('/alicia_d/accepted_joint_states', 'sdk_measured')
            committed = deepcopy(latest.get('/grasp_6d/plan_enriched'))
            if min(wire.header.stamp.to_nsec(), measured.header.stamp.to_nsec()) < epoch:
                raise ValueError('feedback predates current epoch')
        scene = FrozenObservationScene.from_plan(committed)
        if not 0 <= rospy.get_time()-scene.source_ns*1e-9 <= 120:
            raise ValueError('fresh committed scene required for diagnostic geometry')
        fk = SerialUrdfFk(rospy.get_param('/robot_description'), ARM_NAMES)
        gripper = GripperGeometry(ANALYTICAL_MAX_INNER_GAP_M, ANALYTICAL_JAW_CLEARANCE_EACH_SIDE_M,
                                 ANALYTICAL_FINGER_SIZE_XYZ_M, ANALYTICAL_PALM_SIZE_XYZ_M,
                                 ANALYTICAL_SUPPORT_CLEARANCE_M)
        # Large relative to this tiny test: includes initial following gap and
        # prospective step. Conditional geometry bound, not hardware dynamics.
        margin = observation_endpoint_support_bound(fk, sdk, [.05]*6, scene.normal,
                                                     scene.offset, gripper, .05)
        if not margin['ok']:
            raise ValueError('insufficient local support margin')
        away = fk(sdk)[:3, 3]-scene.center
        away /= np.linalg.norm(away)
        separation_offset = -float(away @ scene.center) - float(
            np.sum(np.abs(away @ scene.rotation)*scene.size/2))
        target_margin = observation_endpoint_support_bound(
            fk, sdk, [.05]*6, away, separation_offset, gripper, .05)
        if not target_margin['ok']:
            raise ValueError('insufficient local target OBB separation')
        q = sdk.copy()
        q[[1, 2, 4]] += 4*SDK_QUANTUM_RAD
        transform = fk(q)
        target = PoseStamped()
        target.header.frame_id, target.header.stamp = 'base_link', rospy.Time.now()
        for k, v in zip('xyz', transform[:3, 3]):
            setattr(target.pose.position, k, float(v))
        for k, v in zip('xyzw', quaternion_from_matrix(transform)):
            setattr(target.pose.orientation, k, float(v))
        with lock:
            bag.write('/diagnostic/requested_pose', target, rospy.Time.now())
            displays.clear()
        result = rospy.ServiceProxy('/supervisor/probe_joint_tracking', SetJointCommand)(q.tolist(), False)
        if not result.success:
            raise ValueError('strict preflight failed: '+result.message)
        time.sleep(.1)
        matches, rejections = [], []
        with lock:
            for display in displays:
                for plan in display.trajectory:
                    try:
                        matches.append(validate_small_plan(plan, sdk, transform, fk))
                    except (ValueError, IndexError, AttributeError) as exc:
                        rejections.append(str(exc))
        if len(matches) != 1:
            print('PROBE_PREFLIGHT_REJECTED', json.dumps(rejections), flush=True)
            raise ValueError('no unique displayed local strict plan')
        print('PROBE_PLANNED', json.dumps({'epoch_ns': epoch, 'scene_plan_id': scene.plan_id,
              'sdk_before': sdk.tolist(), 'accepted_before': accepted.tolist(),
              'support_bound': margin, 'target_separation_bound': target_margin,
              'plan': matches[0]}), flush=True)
        if args.execute:
            with lock:
                current_sdk, _ = fresh_arm('/alicia_d/sdk_command', 'sdk_transmitted')
                current_actual, _ = fresh_arm('/alicia_d/accepted_joint_states', 'sdk_measured')
                if (ownership() != epoch or not np.array_equal(sdk_words(current_sdk), sdk_words(sdk))
                        or np.max(np.abs(current_actual-accepted)) > SDK_QUANTUM_RAD+1e-12
                        or not 0 <= rospy.get_time()-scene.source_ns*1e-9 <= 120):
                    raise ValueError('source/ownership changed before probe execution')
            print('PROBE_EXECUTE_ONCE via bounded exact-joint gateway; fresh server planning, not grasp/start', flush=True)
            response = rospy.ServiceProxy('/supervisor/probe_joint_tracking', SetJointCommand)(q.tolist(), True)
            print('PROBE_SERVICE_RESULT', response, flush=True)
            deadline = time.monotonic()+6
            while time.monotonic() < deadline and not rospy.is_shutdown():
                time.sleep(.1)
            with lock:
                after_sdk, _ = fresh_arm('/alicia_d/sdk_command', 'sdk_transmitted')
                after_actual, _ = fresh_arm('/alicia_d/accepted_joint_states', 'sdk_measured')
            print('PROBE_MEASURED', json.dumps({
                'sdk_delta_counts': (sdk_words(after_sdk)-sdk_words(sdk)).tolist(),
                'accepted_delta_counts': ((after_actual-accepted)/SDK_QUANTUM_RAD).tolist(),
                'sdk_after': after_sdk.tolist(), 'accepted_after': after_actual.tolist(),
                'following_error_m': float(np.linalg.norm(fk(after_actual)[:3, 3]-fk(after_sdk)[:3, 3])),
                'diagnostic_only_not_grasp_success': True}), flush=True)
        else:
            print('PLANNING_ONLY: no execution service called', flush=True)
    finally:
        with lock:
            closed = True
            bag.close()
        for sub in subs:
            sub.unregister()


if __name__ == '__main__':
    main()
