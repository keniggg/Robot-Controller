#!/usr/bin/env python3
"""Select the next grasp mode. Does not enable/disable or start the arm."""
import argparse
import rospy
from std_srvs.srv import SetBool
from alicia_grasp_modes.srv import SetGraspMode

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('mode', choices=('carton', 'unknown'))
    parser.add_argument('--strategy', choices=('two_stage', 'direct'))
    args = parser.parse_args()
    rospy.init_node('switch_grasp_mode', anonymous=True)
    if args.strategy:
        rospy.wait_for_service('/grasp_mode/select', timeout=10.)
        result = rospy.ServiceProxy('/grasp_mode/select', SetGraspMode)(args.mode, args.strategy)
    else:
        rospy.wait_for_service('/grasp_mode/set_unknown', timeout=10.)
        result = rospy.ServiceProxy('/grasp_mode/set_unknown', SetBool)(args.mode == 'unknown')
    print(('accepted: ' if result.success else 'rejected: ') + result.message)
    raise SystemExit(0 if result.success else 1)
