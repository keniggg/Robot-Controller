#!/usr/bin/env python3
import rospy
import yaml
import numpy as np
from geometry_msgs.msg import Point
from tf.transformations import quaternion_matrix
import os
import tf2_ros
import geometry_msgs.msg
import tf_conversions


class ObjectPoseTransformer:
    def __init__(self):
        if not rospy.get_node_uri():
            rospy.init_node('object_pose_transformer', anonymous=True)

        # 加载标定变换矩阵
        self.object_detected = False  # Initialize to False
        self.transform_matrix = self.load_transform_from_yaml()
        self.latest_pose = None
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        self.publish_rate = rospy.get_param('~publish_rate', 10)  # 10Hz by default
        self.timer = rospy.Timer(rospy.Duration(1.0/self.publish_rate), self.publish_timer_callback)
        
        # 订阅检测到的物体位置
        rospy.Subscriber("/detected_object_position", Point, self.callback)
        rospy.loginfo("ObjectPoseTransformer initialized. Waiting for detected object position...")
    def publish_timer_callback(self, event):
        """Timer callback to continuously publish the object TF"""
        # Only publish if we have a valid detection
        if self.object_detected and self.latest_pose is not None:
            self.publish_object_tf(self.latest_pose)
        else:
            rospy.loginfo_throttle(10.0, "No object detected yet, waiting for detection...")

    def load_transform_from_yaml(self):
        """从 YAML 文件加载标定变换矩阵"""
        yaml_path = os.path.expanduser("~/.ros/easy_handeye/orbbec_handeyecalibration_eye_on_base.yaml")

        try:
            with open(yaml_path, 'r') as f:
                data = yaml.safe_load(f)
                trans = [
                    data['transformation']['x'],
                    data['transformation']['y'],
                    data['transformation']['z']
                ]
                quat = [
                    data['transformation']['qx'],
                    data['transformation']['qy'],
                    data['transformation']['qz'],
                    data['transformation']['qw']
                ]
                rospy.loginfo("Loaded calibration transformation from YAML.")
                return self._create_transform(trans, quat)
        except FileNotFoundError:
            rospy.logerr(f"YAML file not found at {yaml_path}. Please check the path.")
        except KeyError as e:
            rospy.logerr(f"Missing key in YAML file: {e}")
        except Exception as e:
            rospy.logerr(f"Failed to load transformation from YAML: {e}")
        rospy.signal_shutdown("Failed to load calibration data.")
        return np.identity(4)

    def _create_transform(self, trans, rot):
        """创建 4x4 齐次变换矩阵"""
        mat = quaternion_matrix(rot)
        mat[0:3, 3] = trans
        return mat

    def publish_object_tf(self, obj_pos):
        """发布物体的 TF 坐标"""
        t = geometry_msgs.msg.TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "base_link"  # 或 "base_link" 作为基坐标系
        t.child_frame_id = "detected_object"

        t.transform.translation.x = obj_pos[0]
        t.transform.translation.y = obj_pos[1]
        t.transform.translation.z = obj_pos[2]

        quat = tf_conversions.transformations.quaternion_from_euler(0, 0, 0)
        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]

        self.tf_broadcaster.sendTransform(t)
        # Add a log message but throttle it to avoid flooding the console
        # if self.object_detected:
        #     rospy.loginfo_throttle(5.0, "Publishing detected object TF at: x=%.3f y=%.3f z=%.3f", *obj_pos)
        # else:
        #     rospy.loginfo_throttle(5.0, "Publishing default object TF (no detection)")

    def callback(self, msg):
        """将 3D 点从相机光学坐标系转换到机械臂基坐标系"""
        try:
            # 验证输入数据
            if not all(hasattr(msg, attr) for attr in ['x', 'y', 'z']):
                rospy.logwarn("Invalid message received. Missing attributes.")
                return

            # 1. 物体在相机光学帧中的坐标（齐次坐标）
            obj_optical = np.array([msg.x, msg.y, msg.z, 1.0])

            # 2. 光学帧到相机本体帧的变换矩阵
            R_optical_to_link = np.array([
                [0, 0, 1, 0],
                [-1, 0, 0, 0],
                [0, -1, 0, 0],
                [0, 0, 0, 1]
            ])  # 旋转矩阵 + 无平移

            # # 3. 将物体从光学帧转换到相机本体帧
            # obj_link = np.dot(R_optical_to_link, obj_optical)
            obj_link = obj_optical.copy()
            # 4. 从相机本体帧转换到机械臂基坐标系
            obj_robot = np.dot(self.transform_matrix, obj_link)

            # 更新并记录结果
            self.latest_pose = obj_robot[:3]
            # self.publish_object_tf(self.latest_pose)
            self.object_detected = True

            # 限制日志输出频率
            rospy.loginfo_throttle(1.0, "Detected Object in Robot Frame: x=%.3f y=%.3f z=%.3f", *obj_robot[:3])
        except Exception as e:
            rospy.logerr(f"Error in callback: {e}")

    def get_latest_position(self):
        """返回物体在机械臂基坐标系中的最新位置"""
        return self.latest_pose


if __name__ == '__main__':
    try:
        ObjectPoseTransformer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass