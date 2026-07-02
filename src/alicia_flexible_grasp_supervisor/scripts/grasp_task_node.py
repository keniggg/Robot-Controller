#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped
from alicia_flexible_grasp_supervisor.msg import ObjectPose, GraspState
from alicia_flexible_grasp_supervisor.srv import StartGrasp, StartGraspResponse, StopGrasp, StopGraspResponse, SetTargetPose, SetFloat
from alicia_flexible_grasp.grasp.grasp_state_machine import GraspStages, STATE_NAMES
from alicia_flexible_grasp.grasp.grasp_pose_generator import make_pregrasp_pose, make_lift_pose
try:
    import tf2_ros
except Exception:
    tf2_ros = None

class GraspTaskNode:
    def __init__(self):
        self.latest_obj = None
        self.active = False
        self.stage = GraspStages.IDLE
        self.tf_buffer = None
        self.tf_listener = None
        if tf2_ros is not None and bool(rospy.get_param('/handeye/use_tf', True)):
            self.tf_buffer = tf2_ros.Buffer()
            self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
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
        pregrasp_distance = float(gcfg.get('pregrasp_distance', gcfg.get('pregrasp_distance_m', 0.08)))
        final_offset = max(0.0, float(gcfg.get('final_approach_offset_m', 0.015)))
        pregrasp_mode = str(gcfg.get('pregrasp_offset_mode', 'base_z'))
        lift_height = float(gcfg.get('lift_height_m', 0.05))
        open_position = float(gripper_cfg.get('open_position_m', 0.0))

        self.set_state(GraspStages.SEARCH_OBJECT, 'waiting for object')
        t0 = rospy.Time.now()
        while self.latest_obj is None and (rospy.Time.now()-t0).to_sec() < 5.0 and self.active:
            rospy.sleep(0.05)
        if self.latest_obj is None:
            self.set_state(GraspStages.FAILED, 'no object')
            return False

        self.set_state(GraspStages.PLAN_PREGRASP, 'compute pregrasp')
        camera_pose = self._lookup_camera_pose_base()
        pre = make_pregrasp_pose(
            self.latest_obj.pose_base,
            pregrasp_distance,
            camera_pose=camera_pose,
            mode=pregrasp_mode,
        )
        self.set_state(GraspStages.MOVE_PREGRASP, 'moving')
        resp = move_pose(pre, True)
        if not resp.success:
            self.set_state(GraspStages.FAILED, resp.message)
            return False

        set_gripper(open_position)
        rospy.sleep(0.5)

        if not self.active:
            self.set_state(GraspStages.IDLE, 'stopped before target approach')
            return False

        camera_pose = self._lookup_camera_pose_base() or camera_pose
        target_obj = self.latest_obj
        approach = make_pregrasp_pose(
            target_obj.pose_base,
            final_offset,
            camera_pose=camera_pose,
            mode=pregrasp_mode,
        )
        self.set_state(GraspStages.APPROACH_TARGET, 'moving to target')
        resp = move_pose(approach, True)
        if not resp.success:
            self.set_state(GraspStages.FAILED, 'approach target failed: ' + resp.message)
            return False

        self.set_state(GraspStages.COMPLIANT_CLOSE, 'force-guided close')
        resp = close(True)
        if not resp.success:
            self.set_state(GraspStages.FAILED, resp.message)
            return False

        self.set_state(GraspStages.LIFT_OBJECT, 'lifting')
        lift = make_lift_pose(approach, lift_height)
        resp = move_pose(lift, True)
        if not resp.success:
            self.set_state(GraspStages.FAILED, 'lift failed: '+resp.message)
            return False
        self.set_state(GraspStages.SUCCESS, 'grasp done', True)
        return True

    def _lookup_camera_pose_base(self):
        tf_buffer = getattr(self, 'tf_buffer', None)
        if tf_buffer is None:
            return None
        base_frame = str(rospy.get_param('/handeye/base_frame', 'base_link'))
        camera_frame = str(rospy.get_param('/handeye/camera_frame', rospy.get_param('/camera/frame_id', 'camera_link')))
        timeout = float(rospy.get_param('/handeye/tf_timeout_sec', 0.2))
        try:
            transform = tf_buffer.lookup_transform(
                base_frame,
                camera_frame,
                rospy.Time(0),
                rospy.Duration(timeout),
            )
        except Exception as exc:
            rospy.logwarn_throttle(2.0, 'Grasp camera TF lookup failed: %s', exc)
            return None
        pose = PoseStamped()
        pose.header = transform.header
        pose.pose.position.x = transform.transform.translation.x
        pose.pose.position.y = transform.transform.translation.y
        pose.pose.position.z = transform.transform.translation.z
        pose.pose.orientation = transform.transform.rotation
        return pose

if __name__ == '__main__':
    rospy.init_node('grasp_task_node')
    GraspTaskNode()
    rospy.spin()
