#!/usr/bin/env python3
"""Explicit single fixed-goal diagnostic, default plan-only, never a grasp.

--execute temporarily opts into the gateway's already bounded <=30s episode.
No retries, rollback, gripper, torque, enable/disable or stop commands. The
gateway owns fresh scene/ownership/whole-path checks and the consumed-epoch
latch. Restoring the diagnostic opt-in parameter does not alter arm enable.
"""
import argparse
import json
from pathlib import Path

import rospy
from alicia_flexible_grasp_supervisor.srv import TriggerZero


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execute', action='store_true')
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    # Reserve the output before any request; never overwrite historical data.
    with Path(args.output).open('x') as handle:
        rospy.init_node('fixed_goal_endpoint_experiment', anonymous=True)
        key = '/motion_gateway/enable_endpoint_correction_experiments'
        existed = rospy.has_param(key)
        previous = rospy.get_param(key, False)
        report = {'execution_requested': args.execute, 'grasp_success': False,
                  'started_sec': rospy.get_time()}
        try:
            rospy.wait_for_service('/supervisor/refine_endpoint_feedback', timeout=5.)
            if args.execute:
                rospy.set_param(key, True)
            result = rospy.ServiceProxy('/supervisor/refine_endpoint_feedback', TriggerZero)(args.execute)
            report.update(service_success=result.success, message=result.message,
                          completed_sec=rospy.get_time())
        except Exception as exc:
            report.update(error=str(exc), completed_sec=rospy.get_time())
            raise
        finally:
            if args.execute:
                if existed:
                    rospy.set_param(key, previous)
                elif rospy.has_param(key):
                    rospy.delete_param(key)
            json.dump(report, handle, indent=2, allow_nan=False)
            handle.flush()
            print(json.dumps(report, allow_nan=False), flush=True)


if __name__ == '__main__':
    main()
