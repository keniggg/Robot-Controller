#!/usr/bin/env python3
import rospy
from alicia_flexible_grasp_supervisor.msg import ObjectPose, GraspState
from alicia_flexible_grasp_supervisor.srv import StartGrasp, StartGraspResponse, StopGrasp, StopGraspResponse, SetTargetPose, SetFloat
from alicia_flexible_grasp.grasp.grasp_state_machine import GraspStages, STATE_NAMES
from alicia_flexible_grasp.grasp.grasp_pose_generator import make_pregrasp_pose, make_lift_pose

class GraspTaskNode:
    def __init__(self):
        self.latest_obj = None
        self.active = False
        self.stage = GraspStages.IDLE
        self.pub = rospy.Publisher('/grasp/state', GraspState, queue_size=10)
        rospy.Subscriber('/perception/object', ObjectPose, self.obj_cb, queue_size=1)
        rospy.Service('/grasp/start', StartGrasp, self.start_cb)
        rospy.Service('/grasp/stop', StopGrasp, self.stop_cb)
        rospy.loginfo('GraspTaskNode ready')

    def obj_cb(self, msg):
        if msg.detected:
            self.latest_obj = msg

    def set_state(self, stage, message='', success=False):
        self.stage = stage
        msg = GraspState()
        msg.header.stamp = rospy.Time.now()
        msg.stage = int(stage)
        msg.state = STATE_NAMES.get(stage, 'UNKNOWN')
        msg.active = self.active
        msg.success = success
        msg.message = message
        self.pub.publish(msg)
        rospy.loginfo('[Grasp] %s %s', msg.state, message)

    def start_cb(self, req):
        if self.active:
            return StartGraspResponse(False, 'already active')
        self.active = True
        try:
            result = self.execute()
            self.active = False
            return StartGraspResponse(result, 'success' if result else 'failed')
        except Exception as exc:
            self.active = False
            self.set_state(GraspStages.FAILED, str(exc), False)
            return StartGraspResponse(False, str(exc))

    def stop_cb(self, req):
        self.active = False
        self.set_state(GraspStages.EMERGENCY_STOP if req.emergency else GraspStages.IDLE, 'stop requested')
        return StopGraspResponse(True, 'stop requested')

    def execute(self):
        rospy.wait_for_service('/supervisor/move_to_pose', timeout=10)
        rospy.wait_for_service('/supervisor/set_gripper', timeout=10)
        rospy.wait_for_service('/supervisor/compliant_close', timeout=10)
        move_pose = rospy.ServiceProxy('/supervisor/move_to_pose', SetTargetPose)
        set_gripper = rospy.ServiceProxy('/supervisor/set_gripper', SetFloat)
        close = rospy.ServiceProxy('/supervisor/compliant_close', StartGrasp)
        gcfg = rospy.get_param('/grasp', {})
        gripper_cfg = rospy.get_param('/gripper', {})

        self.set_state(GraspStages.SEARCH_OBJECT, 'waiting for object')
        t0 = rospy.Time.now()
        while self.latest_obj is None and (rospy.Time.now()-t0).to_sec() < 5.0 and self.active:
            rospy.sleep(0.05)
        if self.latest_obj is None:
            self.set_state(GraspStages.FAILED, 'no object')
            return False

        self.set_state(GraspStages.PLAN_PREGRASP, 'compute pregrasp')
        pre = make_pregrasp_pose(self.latest_obj.pose_base, gcfg.get('pregrasp_distance_m',0.08))
        self.set_state(GraspStages.MOVE_PREGRASP, 'moving')
        resp = move_pose(pre, True)
        if not resp.success:
            self.set_state(GraspStages.FAILED, resp.message)
            return False

        set_gripper(gripper_cfg.get('open_position_m',0.0))
        rospy.sleep(0.5)
        self.set_state(GraspStages.COMPLIANT_CLOSE, 'force-guided close')
        resp = close(True)
        if not resp.success:
            self.set_state(GraspStages.FAILED, resp.message)
            return False

        self.set_state(GraspStages.LIFT_OBJECT, 'lifting')
        lift = make_lift_pose(pre, gcfg.get('lift_height_m',0.05))
        resp = move_pose(lift, True)
        if not resp.success:
            self.set_state(GraspStages.FAILED, 'lift failed: '+resp.message)
            return False
        self.set_state(GraspStages.SUCCESS, 'grasp done', True)
        return True

if __name__ == '__main__':
    rospy.init_node('grasp_task_node')
    GraspTaskNode()
    rospy.spin()
