#!/usr/bin/env python3
import csv
import os
import rospy
from alicia_flexible_grasp_supervisor.msg import TactileState, GraspState


class DataLoggerNode:
    def __init__(self):
        out_dir = rospy.get_param('~log_dir', os.path.expanduser('~/alicia_grasp_logs'))
        os.makedirs(out_dir, exist_ok=True)
        self.path = os.path.join(out_dir, 'grasp_log.csv')
        self.fp = open(self.path, 'a', newline='', encoding='utf-8')
        self.writer = csv.writer(self.fp)
        if self.fp.tell() == 0:
            self.writer.writerow(['time', 'total_force', 'force_diff', 'grasp_state', 'message'])
        self.tactile = None
        self.grasp = None
        rospy.Subscriber('/tactile/state', TactileState, self.tactile_cb, queue_size=10)
        rospy.Subscriber('/grasp/state', GraspState, self.grasp_cb, queue_size=10)
        rospy.Timer(rospy.Duration(0.1), self.tick)

    def tactile_cb(self, msg): self.tactile = msg
    def grasp_cb(self, msg): self.grasp = msg

    def tick(self, _):
        if self.tactile or self.grasp:
            self.writer.writerow([
                rospy.Time.now().to_sec(),
                getattr(self.tactile, 'total_grip_force', 0.0) if self.tactile else 0.0,
                getattr(self.tactile, 'force_diff', 0.0) if self.tactile else 0.0,
                getattr(self.grasp, 'state_name', '') if self.grasp else '',
                getattr(self.grasp, 'message', '') if self.grasp else '',
            ])
            self.fp.flush()


if __name__ == '__main__':
    rospy.init_node('data_logger_node')
    DataLoggerNode()
    rospy.spin()
