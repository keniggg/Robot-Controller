#!/usr/bin/env python3
import os
import sys

# 让 ROS wrapper 运行时也能找到本包下的 gui/widgets 模块
_THIS_FILE = os.path.abspath(__file__)
_PKG_ROOT = os.path.abspath(os.path.join(os.path.dirname(_THIS_FILE), '..'))
if _PKG_ROOT not in sys.path:
    sys.path.insert(0, _PKG_ROOT)

import rospy
from PyQt5 import QtWidgets, QtCore
from alicia_flexible_grasp_supervisor.srv import SetJointCommand
from gui.widgets.camera_widget import CameraWidget
from gui.widgets.tactile_widget import TactileWidget
from gui.widgets.robot_state_widget import RobotStateWidget
from gui.widgets.joint_control_widget import JointControlWidget
from gui.widgets.cartesian_control_widget import CartesianControlWidget
from gui.widgets.tcp_calibration_widget import TcpCalibrationWidget
from gui.widgets.grasp6d_control_widget import Grasp6DControlWidget
from gui.widgets.perception_widget import PerceptionWidget
from gui.widgets.log_widget import LogWidget
from gui.theme import HudFrame, HudRoot, apply_app_theme

try:
    from controller_manager_msgs.srv import SwitchController, SwitchControllerRequest
except Exception:
    SwitchController = None
    SwitchControllerRequest = None

class MainWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        title = rospy.get_param('/gui/window_title', 'Alicia-D 柔顺抓取上位机 v2')
        self.setWindowTitle(title)
        self.resize(1320, 860)
        self.setMinimumSize(1120, 720)
        self.home_positions = rospy.get_param(
            '/gui/home_joint_positions',
            [-0.0015339808, -0.0138058271, -0.0076699039, -0.0030679616, -0.0107378655, -0.0061359232, 0.0498]
        )

        root = HudRoot()
        root.setObjectName('AppRoot')
        shell = QtWidgets.QVBoxLayout(root)
        shell.setContentsMargins(14, 14, 14, 14)
        shell.setSpacing(14)
        shell.addWidget(self._build_top_bar(title))

        tabs = QtWidgets.QTabWidget()
        tabs.setDocumentMode(True)
        shell.addWidget(tabs, 1)
        self.setCentralWidget(root)
        color_topic = rospy.get_param('/gui/camera_topic', rospy.get_param('/camera/color_topic', '/supervisor/camera/color/image_raw'))
        depth_topic = rospy.get_param('/gui/depth_topic', rospy.get_param('/camera/depth_topic', '/supervisor/camera/depth/image_raw'))

        # 首页布局：左侧实时摄像头，右侧机械臂/触觉/任务状态
        home = QtWidgets.QWidget()
        h = QtWidgets.QHBoxLayout(home)
        h.setContentsMargins(14, 14, 14, 14)
        h.setSpacing(14)
        self.camera = CameraWidget(color_topic, depth_topic)
        right = QtWidgets.QVBoxLayout()
        right.setSpacing(14)
        self.robot = RobotStateWidget(rospy.get_param('/gui/joint_state_topic','/joint_states'))
        self.tactile = TactileWidget(rospy.get_param('/gui/tactile_topic','/tactile/state'))
        self.grasp = Grasp6DControlWidget(
            rospy.get_param('/gui/grasp6d_status_topic', '/grasp_6d/status'),
            rospy.get_param('/gui/grasp6d_plan_topic', rospy.get_param('/grasp/grasp6d_plan_topic', '/grasp_6d/plan')),
            rospy.get_param('/gui/grasp_state_topic','/grasp/state'),
            compact=True,
        )
        right.addWidget(self.robot); right.addWidget(self.tactile); right.addWidget(self.grasp)
        h.addWidget(self.camera, 7); h.addLayout(right, 4)
        tabs.addTab(home, '总览监控')

        tabs.addTab(JointControlWidget(color_topic, depth_topic), '关节控制')
        tabs.addTab(CartesianControlWidget(color_topic, depth_topic), '笛卡尔控制')
        tabs.addTab(TcpCalibrationWidget(), 'TCP标定')
        tabs.addTab(PerceptionWidget(rospy.get_param('/gui/object_topic', '/perception/object'), color_topic, depth_topic), '目标识别')
        tabs.addTab(Grasp6DControlWidget(
            rospy.get_param('/gui/grasp6d_status_topic', '/grasp_6d/status'),
            rospy.get_param('/gui/grasp6d_plan_topic', rospy.get_param('/grasp/grasp6d_plan_topic', '/grasp_6d/plan')),
            rospy.get_param('/gui/grasp_state_topic','/grasp/state'),
        ), '6D抓取')
        tabs.addTab(TactileWidget(rospy.get_param('/gui/tactile_topic','/tactile/state')), '电子皮肤')
        tabs.addTab(LogWidget(), '日志')

    def _build_top_bar(self, title):
        bar = HudFrame()
        bar.setObjectName('TopBar')
        layout = QtWidgets.QHBoxLayout(bar)
        layout.setContentsMargins(18, 14, 18, 14)
        layout.setSpacing(18)

        text = QtWidgets.QVBoxLayout()
        text.setSpacing(4)
        app_title = QtWidgets.QLabel(title)
        app_title.setObjectName('AppTitle')
        subtitle = QtWidgets.QLabel('Alicia-D cybernetic grasp interface')
        subtitle.setObjectName('AppSubtitle')
        text.addWidget(app_title)
        text.addWidget(subtitle)
        layout.addLayout(text, 1)

        for label in ('ROS LINK',):
            chip = QtWidgets.QLabel(label)
            chip.setObjectName('StatusChip')
            layout.addWidget(chip)

        reset_btn = QtWidgets.QPushButton('复位')
        reset_btn.setObjectName('PrimaryButton')
        reset_btn.setMinimumWidth(124)
        reset_btn.clicked.connect(self.reset_home)
        layout.addWidget(reset_btn)

        for label in ('HUD v2',):
            chip = QtWidgets.QLabel(label)
            chip.setObjectName('StatusChip')
            layout.addWidget(chip)

        return bar

    def reset_home(self):
        reply = QtWidgets.QMessageBox.question(
            self,
            '复位确认',
            '将规划并执行机械臂回到当前记录的初始位置，是否继续？',
            QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No,
            QtWidgets.QMessageBox.No,
        )
        if reply != QtWidgets.QMessageBox.Yes:
            return
        self._switch_trajectory_controllers(True)
        try:
            rospy.wait_for_service('/supervisor/move_to_joints', timeout=2.0)
            srv = rospy.ServiceProxy('/supervisor/move_to_joints', SetJointCommand)
            res = srv([float(v) for v in self.home_positions], True)
            QtWidgets.QMessageBox.information(self, '复位结果', res.message)
        except Exception as exc:
            QtWidgets.QMessageBox.warning(self, '复位失败', str(exc))

    def _switch_trajectory_controllers(self, enabled):
        if SwitchController is None or SwitchControllerRequest is None:
            return
        try:
            rospy.wait_for_service('/controller_manager/switch_controller', timeout=0.5)
            srv = rospy.ServiceProxy('/controller_manager/switch_controller', SwitchController)
            req = SwitchControllerRequest()
            req.start_controllers = ['alicia_controller', 'hand_controller'] if enabled else []
            req.stop_controllers = [] if enabled else ['alicia_controller', 'hand_controller']
            req.strictness = SwitchControllerRequest.BEST_EFFORT
            req.start_asap = True
            req.timeout = 1.0
            srv(req)
        except Exception:
            pass


def main():
    rospy.init_node('alicia_supervisor_gui', anonymous=True, disable_signals=True)
    QtWidgets.QApplication.setAttribute(QtCore.Qt.AA_EnableHighDpiScaling, True)
    app = QtWidgets.QApplication(sys.argv)
    apply_app_theme(app)
    w = MainWindow(); w.show()
    timer = QtCore.QTimer(); timer.timeout.connect(lambda: None); timer.start(50)
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()
