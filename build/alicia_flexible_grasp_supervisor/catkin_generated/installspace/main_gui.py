#!/usr/bin/env python3
import sys
import rospy
from PyQt5 import QtWidgets, QtCore
from alicia_flexible_grasp_supervisor.msg import TactileState, GraspState, ObjectPose
from alicia_flexible_grasp_supervisor.srv import StartGrasp, StopGrasp
from alicia_duo_driver.msg import ArmJointState


class MainWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle('Alicia Flexible Grasp Supervisor')
        self.resize(1400, 850)
        self.robot_label = QtWidgets.QLabel('机械臂状态：等待 /arm_joint_state')
        self.tactile_label = QtWidgets.QLabel('电子皮肤：等待 /tactile/state')
        self.object_label = QtWidgets.QLabel('目标识别：等待 /perception/object_pose_base')
        self.grasp_label = QtWidgets.QLabel('抓取状态：IDLE')
        self.log = QtWidgets.QTextEdit(); self.log.setReadOnly(True)
        start_btn = QtWidgets.QPushButton('开始自主柔顺抓取')
        stop_btn = QtWidgets.QPushButton('停止')
        estop_btn = QtWidgets.QPushButton('急停')
        start_btn.clicked.connect(self.start_grasp)
        stop_btn.clicked.connect(lambda: self.stop_grasp(False))
        estop_btn.clicked.connect(lambda: self.stop_grasp(True))
        top = QtWidgets.QHBoxLayout()
        top.addWidget(start_btn); top.addWidget(stop_btn); top.addWidget(estop_btn)
        layout = QtWidgets.QVBoxLayout()
        layout.addLayout(top)
        layout.addWidget(self.robot_label)
        layout.addWidget(self.tactile_label)
        layout.addWidget(self.object_label)
        layout.addWidget(self.grasp_label)
        layout.addWidget(self.log)
        w = QtWidgets.QWidget(); w.setLayout(layout)
        self.setCentralWidget(w)
        rospy.Subscriber('/arm_joint_state', ArmJointState, self.robot_cb, queue_size=1)
        rospy.Subscriber('/tactile/state', TactileState, self.tactile_cb, queue_size=1)
        rospy.Subscriber('/perception/object_pose_base', ObjectPose, self.object_cb, queue_size=1)
        rospy.Subscriber('/grasp/state', GraspState, self.grasp_cb, queue_size=1)
        self.timer = QtCore.QTimer(); self.timer.timeout.connect(lambda: None); self.timer.start(50)

    def append(self, text):
        self.log.append(text)

    def robot_cb(self, msg):
        self.robot_label.setText('机械臂关节(rad)：J1={:.3f}, J2={:.3f}, J3={:.3f}, J4={:.3f}, J5={:.3f}, J6={:.3f}, gripper={:.3f}'.format(
            msg.joint1, msg.joint2, msg.joint3, msg.joint4, msg.joint5, msg.joint6, msg.gripper))

    def tactile_cb(self, msg):
        self.tactile_label.setText('电子皮肤：总力={:.1f} mN，左右差={:.1f} mN，接触 L/R={}/{}，滑移={}'.format(
            msg.total_grip_force, msg.force_diff, msg.left_contact, msg.right_contact, msg.slip_detected))

    def object_cb(self, msg):
        if msg.detected:
            p = msg.pose_base.position
            self.object_label.setText('目标：detected，base=({:.3f}, {:.3f}, {:.3f})，depth={:.3f} m'.format(p.x, p.y, p.z, msg.depth))
        else:
            self.object_label.setText('目标：未检测到 - {}'.format(msg.status))

    def grasp_cb(self, msg):
        self.grasp_label.setText('抓取状态：{} | 力 {:.1f}/{:.1f} mN | {}'.format(msg.state_name, msg.current_force, msg.target_force, msg.message))

    def start_grasp(self):
        try:
            rospy.wait_for_service('/grasp/start', timeout=2.0)
            resp = rospy.ServiceProxy('/grasp/start', StartGrasp)(True, '')
            self.append('start_grasp: {}'.format(resp.message))
        except Exception as exc:
            self.append('start_grasp failed: {}'.format(exc))

    def stop_grasp(self, emergency=False):
        try:
            rospy.wait_for_service('/grasp/stop', timeout=2.0)
            resp = rospy.ServiceProxy('/grasp/stop', StopGrasp)(emergency)
            self.append('stop_grasp: {}'.format(resp.message))
        except Exception as exc:
            self.append('stop_grasp failed: {}'.format(exc))


def main():
    rospy.init_node('alicia_supervisor_gui', anonymous=True, disable_signals=True)
    app = QtWidgets.QApplication(sys.argv)
    win = MainWindow(); win.show()
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
