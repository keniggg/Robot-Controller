from PyQt5 import QtWidgets, QtCore
import rospy
from sensor_msgs.msg import JointState
from alicia_flexible_grasp_supervisor.srv import SetJointCommand
from gui.widgets.camera_widget import CameraWidget
from gui.theme import metric_chip, panel

try:
    from controller_manager_msgs.srv import SwitchController, SwitchControllerRequest
except Exception:
    SwitchController = None
    SwitchControllerRequest = None

class JointControlWidget(QtWidgets.QWidget):
    state_signal = QtCore.pyqtSignal(object)

    def __init__(self, color_topic=None, depth_topic=None):
        super().__init__()
        self._alive = True
        self._subscriber = None
        self.names=['Joint1','Joint2','Joint3','Joint4','Joint5','Joint6','right_finger']
        self.pub=rospy.Publisher('/joint_commands', JointState, queue_size=10)
        self.sliders=[]; self.labels=[]
        self.current_state = None
        self.waypoints = []
        self._syncing_sliders = False
        self._pending_direct_publish = False
        self.controller_names = ['alicia_controller', 'hand_controller']
        layout = QtWidgets.QHBoxLayout(self)
        layout.setContentsMargins(14, 14, 14, 14)
        layout.setSpacing(14)
        controls = QtWidgets.QVBoxLayout()
        controls.setSpacing(0)
        frame, body = panel('关节 / 夹爪控制')
        controls.addWidget(frame)

        grid = QtWidgets.QGridLayout()
        grid.setHorizontalSpacing(14)
        grid.setVerticalSpacing(10)
        body.addLayout(grid)

        for row_index, name in enumerate(self.names):
            lab=QtWidgets.QLabel(name)
            lab.setMinimumWidth(92)
            val=metric_chip('0.000')
            val.setMinimumWidth(86)
            val.setAlignment(QtCore.Qt.AlignCenter)
            s=QtWidgets.QSlider(QtCore.Qt.Horizontal)
            s.setMinimum(-3140); s.setMaximum(3140)
            if name=='right_finger':
                s.setMinimum(0); s.setMaximum(50)
            s.valueChanged.connect(self.handle_slider_changed)
            s.sliderReleased.connect(self.publish_direct_if_realtime)
            grid.addWidget(lab, row_index, 0)
            grid.addWidget(s, row_index, 1)
            grid.addWidget(val, row_index, 2)
            self.sliders.append(s); self.labels.append(val)

        mode_row = QtWidgets.QHBoxLayout()
        self.realtime_direct = QtWidgets.QCheckBox('关节直控模式（滑条直接驱动机械臂）')
        self.realtime_direct.setChecked(self._default_direct_control_enabled())
        self.realtime_direct.toggled.connect(self.update_control_mode)
        self.command_conn_chip = metric_chip('/joint_commands 连接 0', accent=True)
        mode_row.addWidget(self.realtime_direct)
        mode_row.addWidget(self.command_conn_chip)
        body.addLayout(mode_row)

        actions = QtWidgets.QGridLayout()
        actions.setSpacing(10)
        sync_btn = QtWidgets.QPushButton('同步当前关节')
        self.plan_btn = QtWidgets.QPushButton('规划当前目标')
        self.exec_btn = QtWidgets.QPushButton('执行规划目标')
        add_btn = QtWidgets.QPushButton('添加示教点')
        clear_btn = QtWidgets.QPushButton('清空示教点')
        play_btn = QtWidgets.QPushButton('执行示教队列')
        self.plan_btn.setObjectName('PrimaryButton')
        self.exec_btn.setObjectName('PrimaryButton')
        play_btn.setObjectName('PrimaryButton')
        sync_btn.clicked.connect(self.sync_current_state)
        self.plan_btn.clicked.connect(lambda: self.call_moveit(False))
        self.exec_btn.clicked.connect(lambda: self.call_moveit(True))
        add_btn.clicked.connect(self.add_waypoint)
        clear_btn.clicked.connect(self.clear_waypoints)
        play_btn.clicked.connect(self.execute_waypoints)
        for index, button in enumerate([sync_btn, self.plan_btn, self.exec_btn, add_btn, clear_btn, play_btn]):
            actions.addWidget(button, index // 3, index % 3)
        body.addLayout(actions)

        self.waypoint_chip = metric_chip('示教点 0')
        self.status = QtWidgets.QLabel('等待操作')
        self.status.setObjectName('StateBanner')
        self.status.setWordWrap(True)
        body.addWidget(self.waypoint_chip)
        body.addWidget(self.status)
        controls.addStretch(1)
        layout.addLayout(controls, 5)

        if color_topic and depth_topic:
            self.camera_preview = CameraWidget(color_topic, depth_topic, compact=True, default_mode='color')
            layout.addWidget(self.camera_preview, 2)

        self.state_signal.connect(self.update_current_state)
        self._subscriber = rospy.Subscriber('/joint_states', JointState, self._emit_if_alive, queue_size=1)
        self.destroyed.connect(lambda *_: self._shutdown_ros())
        self.direct_timer = QtCore.QTimer(self)
        self.direct_timer.timeout.connect(self.flush_direct_publish)
        self.direct_timer.start(50)
        self.update_control_mode(self.realtime_direct.isChecked(), apply_controller_switch=False)
        QtCore.QTimer.singleShot(400, self._refresh_control_mode_later)

    def _refresh_control_mode_later(self):
        if not self.__dict__.get('_alive', False):
            return
        try:
            self.update_control_mode(self.realtime_direct.isChecked())
        except RuntimeError:
            self._shutdown_ros()

    def _emit_if_alive(self, msg):
        if not self.__dict__.get('_alive', False):
            return
        try:
            self.state_signal.emit(msg)
        except RuntimeError:
            self._shutdown_ros()

    def _shutdown_ros(self):
        self._alive = False
        timer = self.__dict__.get('direct_timer', None)
        if timer is not None:
            try:
                timer.stop()
            except Exception:
                pass
        subscriber = self.__dict__.get('_subscriber', None)
        if subscriber is not None and not self.__dict__.get('_subscriber_unregistered', False):
            try:
                subscriber.unregister()
            except Exception:
                pass
            self._subscriber_unregistered = True

    def closeEvent(self, event):
        self._shutdown_ros()
        super().closeEvent(event)

    def positions(self):
        pos=[]
        for name,s,lab in zip(self.names,self.sliders,self.labels):
            if name=='right_finger': v=s.value()/1000.0
            else: v=s.value()/1000.0
            lab.setText('%.3f'%v); pos.append(v)
        return pos

    def update_labels(self):
        self.positions()

    def handle_slider_changed(self):
        self.update_labels()
        if self.realtime_direct.isChecked() and not self._syncing_sliders:
            self._pending_direct_publish = True

    def publish_direct_if_realtime(self):
        if self.realtime_direct.isChecked() and not self._syncing_sliders:
            self.publish_direct('已发送关节直控目标')

    def flush_direct_publish(self):
        if not self.__dict__.get('_alive', False):
            return
        self.command_conn_chip.setText('/joint_commands 连接 %d' % self.pub.get_num_connections())
        if not self.realtime_direct.isChecked() or not self._pending_direct_publish:
            return
        self._pending_direct_publish = False
        self.publish_direct('已发送关节直控目标')

    def publish_direct(self, message='已显式发送到 /joint_commands'):
        pos = self.positions()
        msg=JointState(); msg.header.stamp=rospy.Time.now(); msg.name=self.names; msg.position=pos
        self.pub.publish(msg)
        connections = self.pub.get_num_connections()
        self.status.setText('%s；订阅连接数=%d' % (message, connections))

    def call_moveit(self, execute):
        if self.realtime_direct.isChecked():
            self.status.setText('当前是关节直控模式；如需规划，请先取消勾选直控模式')
            return
        try:
            rospy.wait_for_service('/supervisor/move_to_joints', timeout=1.0)
            srv = rospy.ServiceProxy('/supervisor/move_to_joints', SetJointCommand)
            res = srv(self.positions(), execute)
            self.status.setText(('执行结果：' if execute else '规划结果：') + res.message)
        except Exception as exc:
            self.status.setText(str(exc))

    def update_current_state(self, msg):
        self.current_state = msg

    def sync_current_state(self):
        if self.current_state is None:
            self.status.setText('还没有收到 /joint_states')
            return
        name_to_pos = dict(zip(self.current_state.name, self.current_state.position))
        fallback = list(self.current_state.position)
        for index, (name, slider) in enumerate(zip(self.names, self.sliders)):
            value = name_to_pos.get(name, fallback[index] if index < len(fallback) else 0.0)
            target = int(max(slider.minimum(), min(slider.maximum(), round(value * 1000.0))))
            self._syncing_sliders = True
            slider.blockSignals(True)
            slider.setValue(target)
            slider.blockSignals(False)
            self._syncing_sliders = False
        self.update_labels()
        self.status.setText('已同步当前 /joint_states 到目标滑杆')

    def add_waypoint(self):
        self.waypoints.append(self.positions())
        self.waypoint_chip.setText('示教点 %d' % len(self.waypoints))
        self.status.setText('已添加示教点 %d' % len(self.waypoints))

    def clear_waypoints(self):
        self.waypoints = []
        self.waypoint_chip.setText('示教点 0')
        self.status.setText('示教队列已清空')

    def execute_waypoints(self):
        if self.realtime_direct.isChecked():
            self.status.setText('当前是关节直控模式；如需执行示教队列，请先取消勾选直控模式')
            return
        if not self.waypoints:
            self.status.setText('示教队列为空')
            return
        try:
            rospy.wait_for_service('/supervisor/move_to_joints', timeout=1.0)
            srv = rospy.ServiceProxy('/supervisor/move_to_joints', SetJointCommand)
            for index, positions in enumerate(self.waypoints, start=1):
                res = srv(positions, True)
                if not res.success:
                    self.status.setText('示教点 %d 执行失败：%s' % (index, res.message))
                    return
            self.status.setText('示教队列执行完成，共 %d 点' % len(self.waypoints))
        except Exception as exc:
            self.status.setText(str(exc))

    def switch_trajectory_controllers(self, enabled):
        if SwitchController is None or SwitchControllerRequest is None:
            return False, 'controller_manager_msgs 不可用'
        try:
            rospy.wait_for_service('/controller_manager/switch_controller', timeout=0.35)
            srv = rospy.ServiceProxy('/controller_manager/switch_controller', SwitchController)
            req = SwitchControllerRequest()
            req.start_controllers = self.controller_names if enabled else []
            req.stop_controllers = [] if enabled else self.controller_names
            req.strictness = SwitchControllerRequest.BEST_EFFORT
            req.start_asap = True
            req.timeout = 1.0
            res = srv(req)
            action = '启动' if enabled else '暂停'
            if getattr(res, 'ok', False):
                return True, '已%s轨迹控制器' % action
            return False, '轨迹控制器%s请求未完成' % action
        except Exception as exc:
            return False, '轨迹控制器切换不可用：%s' % exc

    def update_control_mode(self, direct_enabled, apply_controller_switch=True):
        self.plan_btn.setEnabled(not direct_enabled)
        self.exec_btn.setEnabled(not direct_enabled)
        controller_msg = ''
        if apply_controller_switch:
            ok, controller_msg = self.switch_trajectory_controllers(not direct_enabled)
            controller_msg = '；' + controller_msg
        if direct_enabled:
            self.status.setText('关节直控模式：滑动滑条会直接连续发布 /joint_commands%s' % controller_msg)
        else:
            self._pending_direct_publish = False
            self.status.setText('规划模式：滑条只设目标，请使用“规划当前目标/执行规划目标”%s' % controller_msg)

    @staticmethod
    def _default_direct_control_enabled():
        return bool(rospy.get_param('/gui/default_joint_direct_control', False))
