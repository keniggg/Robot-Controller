from PyQt5 import QtWidgets, QtCore
import threading
import time

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String
from alicia_flexible_grasp_supervisor.srv import SetJointCommand
from gui.widgets.camera_widget import CameraWidget
from gui.theme import metric_chip, panel

try:
    from controller_manager_msgs.srv import SwitchController, SwitchControllerRequest
except Exception:
    SwitchController = None
    SwitchControllerRequest = None


class DirectControlRecovery:
    """Keep one explicit GUI target across a positive-enable handshake."""

    PROBE_DELTA_RAD = 0.025
    PROBE_REQUIRED_DELTA_RAD = 0.02

    ENABLE_REQUIRED_STATES = frozenset((
        'DISABLED',
        'UNCONFIRMED',
        'OVERHEAT_BLOCKED',
    ))

    def __init__(self, timeout_sec=2.0, enable_retry_sec=0.5):
        self.timeout_sec = max(0.1, float(timeout_sec))
        self.enable_retry_sec = max(0.1, float(enable_retry_sec))
        self._lock = threading.RLock()
        self._actuation_status = ''
        self._feedback_ready = False
        self._pending_target = None
        self._pending_deadline = 0.0
        self._pending_enable_attempted = False
        self._last_enable_request_time = float('-inf')
        self._awaiting_feedback_after_enable = False
        self._last_joint_feedback_time = float('-inf')
        self._latest_feedback_target = None
        self._sync_target_sent = False
        self._probe_target_sent = False

    @staticmethod
    def _state_name(status):
        return str(status or '').partition(':')[0].strip().upper()

    def update_actuation_status(self, status):
        with self._lock:
            self._actuation_status = str(status or '')

    def update_feedback_ready(self, ready):
        with self._lock:
            self._feedback_ready = bool(ready)

    def note_joint_feedback(self, target, now_sec):
        with self._lock:
            self._last_joint_feedback_time = float(now_sec)
            self._latest_feedback_target = list(target)

    def cancel(self):
        with self._lock:
            self._clear_pending()

    def submit(self, target, now_sec):
        """Record a user target and return (action, target, detail)."""
        now = float(now_sec)
        with self._lock:
            self._pending_target = list(target)
            self._pending_deadline = now + self.timeout_sec
            if (
                now - self._last_enable_request_time >=
                self.enable_retry_sec
            ):
                self._pending_enable_attempted = False
            return self._decide(now, allow_enable=True)

    def poll(self, now_sec):
        """Advance a pending handshake without retrying a rejected enable."""
        now = float(now_sec)
        with self._lock:
            if self._pending_target is None:
                return 'none', None, ''
            if now > self._pending_deadline:
                status = self._actuation_status or '状态未知'
                self._clear_pending()
                return (
                    'clear',
                    None,
                    '直控恢复超时；驱动状态=%s；已丢弃未确认滑条目标，'
                    '需要机械臂重新上电后再同步关节' % status,
                )
            return self._decide(now, allow_enable=True)

    def _decide(self, now, allow_enable):
        state = self._state_name(self._actuation_status)
        feedback_is_new_enough = (
            not self._awaiting_feedback_after_enable or
            self._last_joint_feedback_time > self._last_enable_request_time
        )
        if self._awaiting_feedback_after_enable:
            if (
                state == 'UNCONFIRMED' and
                str(self._actuation_status).partition(':')[2] ==
                'ENCODER_RESPONSE_TIMEOUT'
            ):
                self._clear_pending()
                return (
                    'clear',
                    None,
                    '有界验证步后未检测到真实关节运动；已丢弃完整滑条目标，'
                    '需要机械臂重新上电后再同步关节',
                )
            # A rejected positive-enable request leaves the state blocked.
            # Do not even publish the zero-motion synchronization target until
            # the driver has acknowledged a pending/confirmed enable state.
            if state not in ('PENDING', 'CONFIRMED'):
                if (
                    state in self.ENABLE_REQUIRED_STATES and
                    allow_enable and
                    not self._pending_enable_attempted
                ):
                    self._pending_enable_attempted = True
                    self._last_enable_request_time = now
                    self._sync_target_sent = False
                    self._probe_target_sent = False
                    return 'enable', None, self._actuation_status
                return 'wait', None, self._actuation_status
            if not self._feedback_ready or not feedback_is_new_enough:
                return 'wait', None, self._actuation_status
            if not self._sync_target_sent:
                if self._latest_feedback_target is None:
                    return 'wait', None, self._actuation_status
                self._sync_target_sent = True
                return (
                    'sync',
                    list(self._latest_feedback_target),
                    self._actuation_status,
                )
            reason = str(self._actuation_status or '').partition(':')[2]
            if state == 'CONFIRMED':
                target = list(self._pending_target)
                self._clear_pending()
                return 'publish', target, self._actuation_status
            if reason == 'COMMAND_SYNCHRONIZED' and not self._probe_target_sent:
                probe_target = self._bounded_probe_target()
                if probe_target is None:
                    self._clear_pending()
                    return (
                        'clear',
                        None,
                        '滑条目标的机械臂关节变化不足 0.02 rad，无法验证真实驱动；'
                        '已丢弃未确认目标',
                    )
                self._probe_target_sent = True
                return 'probe', probe_target, self._actuation_status
            return 'wait', None, self._actuation_status

        if state == 'CONFIRMED' and self._feedback_ready:
            target = list(self._pending_target)
            self._clear_pending()
            return 'publish', target, self._actuation_status

        if state == 'PENDING':
            reason = str(self._actuation_status or '').partition(':')[2]
            if reason == 'POSITIVE_ENABLE_REQUESTED':
                # A task failure may have requested positive enable before the
                # GUI target was created. Require feedback newer than this
                # explicit user action, then continue through sync + probe.
                self._awaiting_feedback_after_enable = True
                self._last_enable_request_time = now
                self._sync_target_sent = False
                self._probe_target_sent = False
            return 'wait', None, self._actuation_status

        if state in self.ENABLE_REQUIRED_STATES:
            if allow_enable and not self._pending_enable_attempted:
                self._pending_enable_attempted = True
                self._last_enable_request_time = now
                self._awaiting_feedback_after_enable = True
                self._sync_target_sent = False
                self._probe_target_sent = False
                return 'enable', None, self._actuation_status
            return 'wait', None, self._actuation_status

        if state in ('CONFIRMED', 'PENDING'):
            return 'wait', None, self._actuation_status

        # The actuation-status publisher is latched, so this is normally only
        # the short GUI startup window.  Queue the explicit user target until
        # the driver state arrives instead of silently dropping it.
        return 'wait', None, self._actuation_status or '等待驱动状态'

    def _bounded_probe_target(self):
        if (
            self._pending_target is None or
            self._latest_feedback_target is None or
            len(self._pending_target) != len(self._latest_feedback_target)
        ):
            return None
        arm_joint_count = min(6, len(self._pending_target))
        if arm_joint_count <= 0:
            return None
        deltas = [
            self._pending_target[index] - self._latest_feedback_target[index]
            for index in range(arm_joint_count)
        ]
        joint_index = max(range(arm_joint_count), key=lambda index: abs(deltas[index]))
        requested_delta = deltas[joint_index]
        if abs(requested_delta) < self.PROBE_REQUIRED_DELTA_RAD:
            return None
        probe_delta = min(abs(requested_delta), self.PROBE_DELTA_RAD)
        probe_target = list(self._latest_feedback_target)
        probe_target[joint_index] += (
            probe_delta if requested_delta > 0.0 else -probe_delta
        )
        return probe_target

    def _clear_pending(self):
        self._pending_target = None
        self._pending_deadline = 0.0
        self._pending_enable_attempted = False
        self._awaiting_feedback_after_enable = False
        self._sync_target_sent = False
        self._probe_target_sent = False


class JointControlWidget(QtWidgets.QWidget):
    state_signal = QtCore.pyqtSignal(object)

    def __init__(self, color_topic=None, depth_topic=None):
        super().__init__()
        self._alive = True
        self._subscriber = None
        self.names=['Joint1','Joint2','Joint3','Joint4','Joint5','Joint6','right_finger']
        self.pub=rospy.Publisher('/joint_commands', JointState, queue_size=10)
        self.enable_pub = rospy.Publisher('/demonstration', Bool, queue_size=1)
        self.sliders=[]; self.labels=[]
        self.current_state = None
        self.waypoints = []
        self._syncing_sliders = False
        self._pending_direct_publish = False
        self.direct_recovery = DirectControlRecovery(
            timeout_sec=rospy.get_param(
                '/gui/direct_control_recovery_timeout_sec',
                2.0,
            ),
            enable_retry_sec=rospy.get_param(
                '/gui/direct_control_enable_retry_sec',
                0.5,
            ),
        )
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
        self._actuation_subscriber = rospy.Subscriber(
            '/alicia_d/actuation_status',
            String,
            self._actuation_status_cb,
            queue_size=1,
        )
        self._feedback_subscriber = rospy.Subscriber(
            '/alicia_d/feedback_ready',
            Bool,
            self._feedback_ready_cb,
            queue_size=1,
        )
        self.destroyed.connect(lambda *_: self._shutdown_ros())
        self.direct_timer = QtCore.QTimer(self)
        self.direct_timer.timeout.connect(self.flush_direct_publish)
        self.direct_timer.start(50)
        self.update_control_mode(self.realtime_direct.isChecked(), apply_controller_switch=False)
        if not bool(rospy.get_param(
                '~preserve_controller_state_on_startup',
                False,
        )):
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
            name_to_pos = dict(zip(msg.name, msg.position))
            fallback = list(msg.position)
            feedback_target = [
                name_to_pos.get(
                    name,
                    fallback[index] if index < len(fallback) else 0.0,
                )
                for index, name in enumerate(self.names)
            ]
            self.direct_recovery.note_joint_feedback(
                feedback_target,
                time.monotonic(),
            )
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
        for name in ('_actuation_subscriber', '_feedback_subscriber'):
            subscriber = self.__dict__.get(name, None)
            marker = name + '_unregistered'
            if subscriber is None or self.__dict__.get(marker, False):
                continue
            try:
                subscriber.unregister()
            except Exception:
                pass
            self.__dict__[marker] = True

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
        if not self.realtime_direct.isChecked():
            return
        if self._pending_direct_publish:
            self._pending_direct_publish = False
            self.publish_direct('已发送关节直控目标')
            return
        action, target, detail = self.direct_recovery.poll(time.monotonic())
        self._apply_direct_recovery_action(action, target, detail)

    def publish_direct(self, message='已显式发送到 /joint_commands'):
        pos = self.positions()
        action, target, detail = self.direct_recovery.submit(
            pos,
            time.monotonic(),
        )
        self._apply_direct_recovery_action(action, target, detail, message)

    def _apply_direct_recovery_action(
        self,
        action,
        target,
        detail,
        published_message='已发送关节直控目标',
    ):
        if action == 'none':
            return
        if action == 'enable':
            # False is the existing positive torque-on request.  This path is
            # entered only after an explicit GUI slider target; it never sends
            # the opposite zero-torque/disable value and never replays a task
            # target.
            enable_msg = Bool()
            enable_msg.data = False
            self.enable_pub.publish(enable_msg)
            self.status.setText(
                '驱动状态 %s；已请求正向使能，等待新编码器反馈后自动发送本次滑条目标'
                % (detail or '未确认')
            )
            return
        if action == 'sync':
            self._publish_joint_target(target)
            self.status.setText(
                '已用最新实测关节姿态完成零运动同步；等待驱动确认后发送滑条目标'
            )
            return
        if action == 'probe':
            self._publish_joint_target(target)
            self.status.setText(
                '已沿滑条方向发送 0.025 rad 以内的有界验证步；'
                '只有编码器确认真实运动后才会发送完整目标'
            )
            return
        if action == 'publish':
            self._publish_joint_target(target)
            connections = self.pub.get_num_connections()
            self.status.setText('%s；订阅连接数=%d' % (published_message, connections))
            return
        if action == 'clear':
            # A positive-enable request clears the driver's retained command
            # before asking for torque-on. This bounds a failed recovery to
            # the small probe and never emits the opposite torque-off value.
            enable_msg = Bool()
            enable_msg.data = False
            self.enable_pub.publish(enable_msg)
            self.status.setText(detail)
            return
        if action == 'timeout':
            self.status.setText(detail)
            return
        if action == 'wait' and detail:
            self.status.setText('等待直控恢复：%s' % detail)

    def _publish_joint_target(self, target):
        msg = JointState()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = 'gui_direct'
        msg.name = self.names
        msg.position = list(target)
        self.pub.publish(msg)

    def _actuation_status_cb(self, msg):
        if self.__dict__.get('_alive', False):
            self.direct_recovery.update_actuation_status(msg.data)

    def _feedback_ready_cb(self, msg):
        if self.__dict__.get('_alive', False):
            self.direct_recovery.update_feedback_ready(msg.data)

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
            self.direct_recovery.cancel()
            self.status.setText('规划模式：滑条只设目标，请使用“规划当前目标/执行规划目标”%s' % controller_msg)

    @staticmethod
    def _default_direct_control_enabled():
        configured = rospy.get_param('/gui/default_joint_direct_control', False)
        return bool(rospy.get_param('~default_joint_direct_control', configured))
