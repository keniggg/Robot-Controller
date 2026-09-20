#!/usr/bin/env python3
"""One Joint2 +/-7-count free-space diagnostic; default plan-only, never a grasp.

Uses a new exact-audited scene and the gateway's one-per-handoff admission.
No raw serial, torque/stop/gripper commands, repeated step or implicit rollback.
"""
import argparse
from collections import deque
from copy import deepcopy
import json
from pathlib import Path
import threading
import time

import numpy as np
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header, String

from alicia_flexible_grasp.robot.stationary_following import (
    ARM_NAMES, SDK_QUANTUM_RAD, SerialUrdfFk, stationary_following_error,
)
from alicia_flexible_grasp.robot.endpoint_correction import sdk_counts
from alicia_flexible_grasp_supervisor.msg import Grasp6DPlan
from alicia_flexible_grasp_supervisor.srv import SetJointCommand, TriggerZero
from run_fresh_grasp6d_after_alignment import FreshPreviewRunner


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execute', action='store_true')
    parser.add_argument('--delta-counts', type=int, choices=(-7, 7), default=7,
                        help='one signed Joint2 increment; the existing seven-count bound is unchanged')
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    with Path(args.output).open('x') as output:
        rospy.init_node('joint_response_characterization_runner', anonymous=True)
        runner = FreshPreviewRunner()
        lock = threading.Lock()
        samples = {'sdk': deque(maxlen=160), 'accepted': deque(maxlen=160)}
        epoch = [None]
        def receive(kind, message):
            with lock:
                samples[kind].append(deepcopy(message))
        def receive_epoch(message):
            if message.frame_id == 'sdk_reference_epoch':
                with lock:
                    epoch[0] = message.stamp.to_nsec()
        subscribers = [
            rospy.Subscriber('/alicia_d/sdk_command', JointState, lambda m: receive('sdk', m)),
            rospy.Subscriber('/alicia_d/accepted_joint_states', JointState, lambda m: receive('accepted', m)),
            rospy.Subscriber('/alicia_d/control_reference_epoch', Header, receive_epoch),
            rospy.Subscriber('/grasp_6d/preview_plan_enriched', Grasp6DPlan, runner.on_preview),
            rospy.Subscriber('/grasp_6d/gate_audit', String, runner.on_gate_audit),
        ]
        report = {'execution_requested': args.execute, 'grasp_success': False}
        key = '/motion_gateway/enable_joint_response_characterization'
        existed, previous = rospy.has_param(key), rospy.get_param(key, False)
        enabled_here = False
        try:
            for service in ['/grasp_6d/request_plan', '/grasp/current_plan',
                            '/grasp_6d/replan_execution', '/supervisor/characterize_joint_response']:
                rospy.wait_for_service(service, timeout=15.)
            runner.request_plan = rospy.ServiceProxy('/grasp_6d/request_plan', TriggerZero)
            runner.plan_validity_sec = float(rospy.get_param('/grasp_6d/plan_validity_sec'))
            runner.minimum_stamp_ns = rospy.Time.now().to_nsec()
            report['minimum_scene_stamp_ns'] = runner.minimum_stamp_ns
            if not runner.request_plan(True).success:
                raise ValueError('fresh inference did not start')
            preview = runner.wait_for_preview(300.)
            if preview is None:
                raise ValueError('no new valid observation scene within 300s')
            if runner.wait_for_execution_authority(
                    rospy.ServiceProxy('/grasp/current_plan', TriggerZero),
                    rospy.ServiceProxy('/grasp_6d/replan_execution', TriggerZero),
                    preview, preview.plan_id, 60.) is None:
                raise ValueError('no exact committed scene authority')
            report['scene_plan_id'] = preview.plan_id
            description = rospy.get_param('/robot_description')
            fk = SerialUrdfFk(description, ARM_NAMES)
            def snapshot(after_ns=0):
                deadline = time.monotonic() + 3.
                last_error = 'no samples'
                while time.monotonic() < deadline:
                    with lock:
                        saved = deepcopy(samples)
                        reference_epoch = epoch[0]
                    try:
                        value = stationary_following_error(fk, list(saved['sdk']),
                            list(saved['accepted']), now_sec=rospy.get_time(),
                            epoch_ns=reference_epoch)
                        if value['stationary_window_start_ns'] > after_ns:
                            return value
                    except ValueError as exc:
                        last_error = str(exc)
                    rospy.sleep(.05)
                raise ValueError('stationary feedback unavailable: ' + last_error)
            initial = snapshot()
            baseline = sdk_counts(initial['sdk_positions_rad'])
            goal = baseline.copy()
            goal[1] += args.delta_counts
            if np.any(goal < 0) or np.any(goal > 4095):
                raise ValueError('target encoder range invalid')
            positions = ((goal-2048)*SDK_QUANTUM_RAD).tolist()
            report.update(before=initial, requested_delta_counts=(goal-baseline).tolist(),
                          baseline_counts=baseline.tolist(), goal_counts=goal.tolist())
            probe = rospy.ServiceProxy('/supervisor/characterize_joint_response', SetJointCommand)
            planned = probe(positions, False)
            report['planning'] = {'success': planned.success, 'message': planned.message}
            print('RESPONSE_PROBE_PLANNED ' + json.dumps(report), flush=True)
            if not planned.success:
                raise ValueError(planned.message)
            if args.execute:
                fresh = snapshot()
                if (fresh['epoch_ns'] != initial['epoch_ns']
                        or not np.array_equal(sdk_counts(fresh['sdk_positions_rad']), baseline)
                        or np.max(np.abs(sdk_counts(fresh['accepted_positions_rad'])
                                         - sdk_counts(initial['accepted_positions_rad']))) > 1
                        or rospy.get_param('/robot_description') != description):
                    raise ValueError('diagnostic reference/model changed')
                rospy.set_param(key, True)
                enabled_here = True
                report['submission_sec'] = rospy.get_time()
                result = probe(positions, True)
                completed_ns = rospy.Time.now().to_nsec()
                report['execution'] = {'success': result.success, 'message': result.message,
                                       'completed_sec': completed_ns*1e-9}
                rospy.sleep(.5)
                after = snapshot(after_ns=completed_ns)
                report['after'] = after
                report['measured_delta_counts'] = (
                    sdk_counts(after['accepted_positions_rad'])
                    - sdk_counts(initial['accepted_positions_rad'])).tolist()
                report['transmitted_delta_counts'] = (
                    sdk_counts(after['sdk_positions_rad'])-baseline).tolist()
                # Report actual physical evidence independently of service success.
                report['response_is_grasp_or_calibration_success'] = False
        except Exception as exc:
            report['error'] = str(exc)
        finally:
            if enabled_here:
                if existed:
                    rospy.set_param(key, previous)
                elif rospy.has_param(key):
                    rospy.delete_param(key)
            if runner.request_plan is not None:
                runner.stop_candidate_computation()
            for subscriber in subscribers:
                subscriber.unregister()
            json.dump(report, output, indent=2, allow_nan=False)
            print('RESPONSE_PROBE_RESULT ' + json.dumps(report), flush=True)


if __name__ == '__main__':
    main()
