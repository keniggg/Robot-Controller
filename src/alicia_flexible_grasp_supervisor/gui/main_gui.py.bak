#!/usr/bin/env python3
import sys
import rospy
from PyQt5 import QtWidgets, QtCore
from gui.widgets.camera_widget import CameraWidget
from gui.widgets.tactile_widget import TactileWidget
from gui.widgets.robot_state_widget import RobotStateWidget
from gui.widgets.joint_control_widget import JointControlWidget
from gui.widgets.cartesian_control_widget import CartesianControlWidget
from gui.widgets.grasp_task_widget import GraspTaskWidget
from gui.widgets.log_widget import LogWidget

class MainWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        title = rospy.get_param('/gui/window_title', 'Alicia-D 柔顺抓取上位机 v2')
        self.setWindowTitle(title)
        self.resize(1280, 820)
        tabs = QtWidgets.QTabWidget()
        self.setCentralWidget(tabs)

        # 首页布局：左侧实时摄像头，右侧机械臂/触觉/任务状态
        home = QtWidgets.QWidget(); h = QtWidgets.QHBoxLayout(home)
        self.camera = CameraWidget(rospy.get_param('/gui/camera_topic','/supervisor/camera/color/image_raw'))
        right = QtWidgets.QVBoxLayout()
        self.robot = RobotStateWidget(rospy.get_param('/gui/joint_state_topic','/joint_states'))
        self.tactile = TactileWidget(rospy.get_param('/gui/tactile_topic','/tactile/state'))
        self.grasp = GraspTaskWidget(rospy.get_param('/gui/grasp_state_topic','/grasp/state'))
        right.addWidget(self.robot); right.addWidget(self.tactile); right.addWidget(self.grasp)
        h.addWidget(self.camera, 3); h.addLayout(right, 2)
        tabs.addTab(home, '总览')

        tabs.addTab(CameraWidget(rospy.get_param('/gui/camera_topic','/supervisor/camera/color/image_raw')), '摄像头实时画面')
        tabs.addTab(JointControlWidget(), '关节/夹爪控制')
        tabs.addTab(CartesianControlWidget(), '笛卡尔空间控制')
        tabs.addTab(TactileWidget(rospy.get_param('/gui/tactile_topic','/tactile/state')), '电子皮肤力反馈')
        tabs.addTab(GraspTaskWidget(rospy.get_param('/gui/grasp_state_topic','/grasp/state')), '自主柔顺抓取')
        tabs.addTab(LogWidget(), '日志')


def main():
    rospy.init_node('alicia_supervisor_gui', anonymous=True, disable_signals=True)
    app = QtWidgets.QApplication(sys.argv)
    w = MainWindow(); w.show()
    timer = QtCore.QTimer(); timer.timeout.connect(lambda: None); timer.start(50)
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()
