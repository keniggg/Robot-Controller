#!/usr/bin/env python3
import threading
import rospy
from std_msgs.msg import Bool
from alicia_flexible_grasp_supervisor.msg import ObjectPose, TactileState, GraspState
from alicia_flexible_grasp_supervisor.srv import StartGrasp, StartGraspResponse, StopGrasp, StopGraspResponse
from alicia_flexible_grasp.robot.moveit_planner import MoveItPlanner
from alicia_flexible_grasp.robot.gripper_commander import GripperCommander
from alicia_flexible_grasp.grasp.grasp_state_machine import GraspStates
from alicia_flexible_grasp.grasp.grasp_pose_generator import GraspPoseGenerator
from alicia_flexible_grasp.grasp.compliant_grasp import CompliantGraspController
from alicia_flexible_grasp.grasp.grasp_verifier import GraspVerifier


class GraspTaskNode:
    def __init__(self):
        self.object_pose = None
        self.tactile = None
        self.running = False
        self.stop_flag = False
        self.state = GraspStates.IDLE
        self.message = 'idle'
        self.planner = MoveItPlanner(rospy.get_param('~move_group', 'alicia'))
        self.gripper = GripperCommander('/gripper_control')
        self.pose_gen = GraspPoseGenerator(rospy.get_param('~pregrasp_distance_m', 0.08), rospy.get_param('~lift_height_m', 0.05))
        self.verifier = GraspVerifier(rospy.get_param('~hold_force_min_mn', 800.0))
        self.force_target = rospy.get_param('~target_force_mn', 1500.0)
        self.comp = CompliantGraspController(
            self.gripper, self.get_force,
            open_position=rospy.get_param('~open_position', 0.12),
            contact_threshold=rospy.get_param('~contact_threshold_mn', 200.0),
            target_force=self.force_target,
            max_force=rospy.get_param('~max_force_mn', 4000.0),
            close_step_fast=rospy.get_param('~close_step_fast', 0.003),
            close_step_slow=rospy.get_param('~close_step_slow', 0.001),
            open_step_safe=rospy.get_param('~open_step_safe', 0.005),
        )
        self.pub_state = rospy.Publisher('/grasp/state', GraspState, queue_size=10)
        rospy.Subscriber('/perception/object_pose_base', ObjectPose, self.object_cb, queue_size=1)
        rospy.Subscriber('/tactile/state', TactileState, self.tactile_cb, queue_size=10)
        rospy.Service('/grasp/start', StartGrasp, self.start_srv)
        rospy.Service('/grasp/stop', StopGrasp, self.stop_srv)
        rospy.Timer(rospy.Duration(0.1), self.timer_pub)

    def object_cb(self, msg):
        if msg.detected:
            self.object_pose = msg

    def tactile_cb(self, msg):
        self.tactile = msg

    def get_force(self):
        return 0.0 if self.tactile is None else self.tactile.total_grip_force

    def set_state(self, state, msg=''):
        self.state = state
        self.message = msg or GraspStates.NAMES.get(state, str(state))
        rospy.loginfo('grasp state: %s - %s', GraspStates.NAMES.get(state, state), self.message)

    def start_srv(self, req):
        if self.running:
            return StartGraspResponse(False, 'grasp task already running')
        self.stop_flag = False
        self.running = True
        threading.Thread(target=self.execute, daemon=True).start()
        return StartGraspResponse(True, 'grasp task started')

    def stop_srv(self, req):
        self.stop_flag = True
        if req.emergency:
            self.set_state(GraspStates.EMERGENCY_STOP, 'emergency stop requested')
        return StopGraspResponse(True, 'stop requested')

    def execute(self):
        try:
            self.set_state(GraspStates.SEARCH_OBJECT, 'waiting for detected object')
            t0 = rospy.Time.now().to_sec()
            while not rospy.is_shutdown() and not self.object_pose and rospy.Time.now().to_sec() - t0 < 5.0:
                if self.stop_flag: return
                rospy.sleep(0.05)
            if self.object_pose is None:
                self.set_state(GraspStates.FAILED, 'no object detected'); return
            obj_pose = self.object_pose.pose_base
            self.set_state(GraspStates.PLAN_PREGRASP, 'generate pregrasp pose')
            pre = self.pose_gen.pregrasp_from_object_pose(obj_pose)
            self.gripper.command(rospy.get_param('~open_position', 0.12))
            self.set_state(GraspStates.MOVE_PREGRASP, 'move to pregrasp')
            ok, msg = self.planner.move_to_pose(pre, execute=True)
            if not ok:
                self.set_state(GraspStates.FAILED, 'pregrasp planning failed: ' + msg); return
            self.set_state(GraspStates.APPROACH_OBJECT, 'move to grasp pose')
            grasp = self.pose_gen.grasp_pose_from_object_pose(obj_pose)
            ok, msg = self.planner.move_to_pose(grasp, execute=True)
            if not ok:
                self.set_state(GraspStates.FAILED, 'approach planning failed: ' + msg); return
            self.set_state(GraspStates.COMPLIANT_CLOSE, 'close gripper until target force')
            ok, msg, force, pos = self.comp.close_until_force(timeout=8.0)
            if not ok:
                self.set_state(GraspStates.FAILED, msg); return
            self.set_state(GraspStates.LIFT_OBJECT, 'lift object')
            lift = self.pose_gen.lift_pose_from_grasp_pose(grasp)
            ok, msg = self.planner.move_to_pose(lift, execute=True)
            if not ok:
                self.set_state(GraspStates.FAILED, 'lift failed: ' + msg); return
            self.set_state(GraspStates.GRASP_VERIFY, 'verify tactile hold')
            slip = self.tactile.slip_detected if self.tactile else False
            ok, msg = self.verifier.verify(self.get_force(), slip)
            self.set_state(GraspStates.SUCCESS if ok else GraspStates.FAILED, msg)
        finally:
            self.running = False

    def timer_pub(self, _):
        msg = GraspState()
        msg.header.stamp = rospy.Time.now()
        msg.state = self.state
        msg.state_name = GraspStates.NAMES.get(self.state, str(self.state))
        msg.running = self.running
        msg.success = self.state == GraspStates.SUCCESS
        msg.message = self.message
        msg.current_force = self.get_force()
        msg.target_force = self.force_target
        if self.object_pose:
            msg.object_pose_base = self.object_pose.pose_base
        self.pub_state.publish(msg)


if __name__ == '__main__':
    rospy.init_node('grasp_task_node')
    GraspTaskNode()
    rospy.spin()
