#!/usr/bin/env python3
import rospy
from alicia_flexible_grasp_supervisor.msg import TactileState
from alicia_flexible_grasp_supervisor.srv import StartGrasp, StartGraspResponse, SetFloat
from alicia_flexible_grasp.grasp.compliant_grasp import CompliantGraspLogic

class CompliantGripperController:
    def __init__(self):
        fcfg = rospy.get_param('/force', {})
        gcfg = rospy.get_param('/gripper', {})
        self.logic = CompliantGraspLogic(
            fcfg.get('contact_threshold_mn',200), fcfg.get('target_force_mn',1500), fcfg.get('max_force_mn',4000),
            gcfg.get('close_step_fast_m',0.002), gcfg.get('close_step_slow_m',0.0007), gcfg.get('open_step_safe_m',0.003),
            gcfg.get('open_position_m',0.0), gcfg.get('close_limit_m',0.05))
        self.force = 0.0
        self.gripper_pos = float(gcfg.get('open_position_m',0.0))
        self.rate_hz = float(gcfg.get('command_rate_hz',25))
        rospy.Subscriber('/tactile/state', TactileState, self.tactile_cb, queue_size=1)
        rospy.Service('/supervisor/compliant_close', StartGrasp, self.handle_close)
        rospy.wait_for_service('/supervisor/set_gripper', timeout=10)
        self.set_gripper = rospy.ServiceProxy('/supervisor/set_gripper', SetFloat)

    def tactile_cb(self, msg):
        self.force = msg.total_grip_force_mn

    def handle_close(self, req):
        if not req.execute:
            return StartGraspResponse(False, 'execute=false')
        rate = rospy.Rate(self.rate_hz)
        start = rospy.Time.now()
        while not rospy.is_shutdown() and (rospy.Time.now()-start).to_sec() < 8.0:
            new_pos, state = self.logic.next_position(self.gripper_pos, self.force)
            self.gripper_pos = new_pos
            try:
                self.set_gripper(self.gripper_pos)
            except Exception as exc:
                return StartGraspResponse(False, 'set gripper failed: %s' % exc)
            if state == 'hold':
                return StartGraspResponse(True, 'target force reached')
            if state == 'over_force':
                return StartGraspResponse(False, 'over force, opened safe step')
            rate.sleep()
        return StartGraspResponse(False, 'timeout')

if __name__ == '__main__':
    rospy.init_node('compliant_gripper_controller')
    CompliantGripperController()
    rospy.spin()
