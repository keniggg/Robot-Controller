from PyQt5 import QtWidgets, QtCore
import math
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


class DirectCommandGesture:
    """Compose one channel's intent; never restrict other admitted goals."""

    def __init__(self):
        self._held_target = None
        self._edited_index = None

    @property
    def edited_index(self):
        return self._edited_index

    @property
    def active(self):
        return self._held_target is not None

    def begin(self, baseline_target, edited_index):
        target = list(baseline_target)
        index = int(edited_index)
        if not 0 <= index < len(target):
            self.clear()
            return False
        self._held_target = target
        self._edited_index = index
        return True

    def compose(self, slider_target):
        current = list(slider_target)
        if (
            self._held_target is None or
            self._edited_index is None or
            len(self._held_target) != len(current)
        ):
            return current
        target = list(self._held_target)
        target[self._edited_index] = current[self._edited_index]
        return target

    def clear(self):
        self._held_target = None
        self._edited_index = None


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
        self._pending_edited_index = None
        self._pending_edited_value = None
        self._pending_rebase_unedited = True
        self._pending_deadline = 0.0
        self._pending_enable_attempted = False
        self._last_enable_request_time = float('-inf')
        self._awaiting_feedback_after_enable = False
        self._last_joint_feedback_time = float('-inf')
        self._latest_feedback_target = None
        self._sync_target_sent = False
        self._probe_target_sent = False
        self._last_action_edited_index = None

    @property
    def last_action_edited_index(self):
        with self._lock:
            return self._last_action_edited_index

    @property
    def pending_edit(self):
        """Expose handshake ownership without replacing it with another axis."""
        with self._lock:
            return self._pending_target is not None, self._pending_edited_index

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
            if self._pending_rebase_unedited:
                self._rebase_pending_target_to_feedback()

    def cancel(self):
        with self._lock:
            self._clear_pending()

    def submit(
        self,
        target,
        now_sec,
        edited_index=None,
        rebase_unedited=True,
    ):
        """Record a user target and return (action, target, detail)."""
        now = float(now_sec)
        with self._lock:
            self._pending_target = list(target)
            self._pending_edited_index = None
            self._pending_edited_value = None
            self._pending_rebase_unedited = bool(rebase_unedited)
            if edited_index is not None:
                index = int(edited_index)
                if 0 <= index < len(self._pending_target):
                    self._pending_edited_index = index
                    self._pending_edited_value = self._pending_target[index]
                    if self._pending_rebase_unedited:
                        self._rebase_pending_target_to_feedback()
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
                    '请再次移动目标关节滑条以发起新的有界恢复' % status,
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
                    '没有重发使能；请再次移动目标关节滑条重试',
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
                self._last_action_edited_index = None
                return (
                    'sync',
                    list(self._latest_feedback_target),
                    self._actuation_status,
                )
            reason = str(self._actuation_status or '').partition(':')[2]
            if state == 'CONFIRMED':
                target = list(self._pending_target)
                edited_index = self._pending_edited_index
                self._clear_pending()
                self._last_action_edited_index = edited_index
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
                self._last_action_edited_index = (
                    self._pending_edited_index
                )
                return 'probe', probe_target, self._actuation_status
            return 'wait', None, self._actuation_status

        if state == 'CONFIRMED' and self._feedback_ready:
            target = list(self._pending_target)
            edited_index = self._pending_edited_index
            self._clear_pending()
            self._last_action_edited_index = edited_index
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
            elif reason == 'COMMAND_SYNCHRONIZED':
                # A previous slider action may have completed the zero-motion
                # synchronization but then been discarded because its delta
                # was too small (or because it addressed only the gripper).
                # Bind this new explicit arm-slider action to the already
                # synchronized driver state. Require feedback newer than the
                # new action, then proceed directly to its bounded probe;
                # never leave COMMAND_SYNCHRONIZED as a terminal wait state.
                self._awaiting_feedback_after_enable = True
                self._last_enable_request_time = now
                self._sync_target_sent = True
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
        if (
            self._pending_edited_index is not None and
            0 <= int(self._pending_edited_index) < arm_joint_count
        ):
            joint_index = int(self._pending_edited_index)
        else:
            joint_index = max(
                range(arm_joint_count),
                key=lambda index: abs(deltas[index]),
            )
        requested_delta = deltas[joint_index]
        if abs(requested_delta) < self.PROBE_REQUIRED_DELTA_RAD:
            return None
        probe_delta = min(abs(requested_delta), self.PROBE_DELTA_RAD)
        probe_target = list(self._latest_feedback_target)
        probe_target[joint_index] += (
            probe_delta if requested_delta > 0.0 else -probe_delta
        )
        return probe_target

    def _rebase_pending_target_to_feedback(self):
        """Keep only the actively edited slider across autonomous arm motion."""
        if (
            self._pending_target is None or
            self._latest_feedback_target is None or
            self._pending_edited_index is None or
            len(self._pending_target) != len(self._latest_feedback_target)
        ):
            return
        index = int(self._pending_edited_index)
        if not 0 <= index < len(self._pending_target):
            return
        rebased = list(self._latest_feedback_target)
        rebased[index] = self._pending_edited_value
        self._pending_target = rebased

    def _clear_pending(self):
        self._pending_target = None
        self._pending_edited_index = None
        self._pending_edited_value = None
        self._pending_rebase_unedited = True
        self._pending_deadline = 0.0
        self._pending_enable_attempted = False
        self._awaiting_feedback_after_enable = False
        self._sync_target_sent = False
        self._probe_target_sent = False


class JointControlWidget(QtWidgets.QWidget):
    state_signal = QtCore.pyqtSignal(object)
    control_mode_signal = QtCore.pyqtSignal(bool)
    pose_target_changed = QtCore.pyqtSignal()

    def __init__(
        self,
        color_topic=None,
        depth_topic=None,
        show_feedback_angles=False,
        arm_only=False,
        calibration_mode=False,
        preserve_controller_state_on_startup=None,
    ):
        super().__init__()
        self._alive = True
        self._subscriber = None
        self.show_feedback_angles = bool(show_feedback_angles)
        self.calibration_mode = bool(calibration_mode)
        self.names=['Joint1','Joint2','Joint3','Joint4','Joint5','Joint6']
        if not arm_only:
            self.names.append('right_finger')
        self.pub=rospy.Publisher('/joint_commands', JointState, queue_size=10)
        self.enable_pub = rospy.Publisher('/demonstration', Bool, queue_size=1)
        self.control_mode_pub = rospy.Publisher('/gui/joint_direct_mode', Bool, queue_size=1, latch=True)
        self.sliders=[]; self.labels=[]; self.feedback_labels=[]
        self.current_state = None
        self.waypoints = []
        self._syncing_sliders = False
        self._pending_direct_publish = False
        self._pending_direct_edits = {}
        self._active_direct_slider_index = None
        self._direct_gesture = DirectCommandGesture()
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
        if self.calibration_mode:
            layout.setContentsMargins(0, 0, 0, 0)
            layout.setSpacing(8)
        else:
            layout.setContentsMargins(14, 14, 14, 14)
            layout.setSpacing(14)
        controls = QtWidgets.QVBoxLayout()
        controls.setSpacing(0)
        panel_title = '手眼标定姿态控制' if self.calibration_mode else '关节 / 夹爪控制'
        frame, body = panel(panel_title)
        if self.calibration_mode:
            body.setContentsMargins(10, 8, 10, 10)
            body.setSpacing(6)
        controls.addWidget(frame)

        grid = QtWidgets.QGridLayout()
        grid.setHorizontalSpacing(8 if self.calibration_mode else 14)
        grid.setVerticalSpacing(4 if self.calibration_mode else 10)
        body.addLayout(grid)

        row_offset = 0
        if self.show_feedback_angles:
            target_header = QtWidgets.QLabel('滑条目标')
            target_header.setObjectName('MutedLabel')
            feedback_header = QtWidgets.QLabel('实时角度')
            feedback_header.setObjectName('MutedLabel')
            feedback_header.setAlignment(QtCore.Qt.AlignCenter)
            grid.addWidget(target_header, 0, 2)
            grid.addWidget(feedback_header, 0, 3)
            row_offset = 1

        for row_index, name in enumerate(self.names):
            grid_row = row_index + row_offset
            lab=QtWidgets.QLabel(name)
            lab.setMinimumWidth(68 if self.calibration_mode else 92)
            val=metric_chip('0.000')
            val.setMinimumWidth(70 if self.calibration_mode else 86)
            val.setAlignment(QtCore.Qt.AlignCenter)
            s=QtWidgets.QSlider(QtCore.Qt.Horizontal)
            s.setMinimum(-3140); s.setMaximum(3140)
            if name=='right_finger':
                s.setMinimum(0); s.setMaximum(50)
            s.valueChanged.connect(self.handle_slider_changed)
            s.sliderPressed.connect(self.handle_slider_pressed)
            s.sliderReleased.connect(self.handle_slider_released)
            grid.addWidget(lab, grid_row, 0)
            grid.addWidget(s, grid_row, 1)
            grid.addWidget(val, grid_row, 2)
            if self.show_feedback_angles:
                feedback = metric_chip('--')
                feedback.setObjectName('JointFeedbackAngle')
                feedback.setMinimumWidth(78 if self.calibration_mode else 96)
                feedback.setAlignment(QtCore.Qt.AlignCenter)
                feedback.setToolTip('来自 /joint_states 的实时实测值')
                grid.addWidget(feedback, grid_row, 3)
                self.feedback_labels.append(feedback)
            self.sliders.append(s); self.labels.append(val)

        mode_row = QtWidgets.QHBoxLayout()
        direct_label = (
            '关节直控（滑条直接驱动）'
            if self.calibration_mode
            else '关节直控模式（滑条直接驱动机械臂）'
        )
        self.realtime_direct = QtWidgets.QCheckBox(direct_label)
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
        action_buttons = [sync_btn]
        if not self.calibration_mode:
            action_buttons.extend([
                self.plan_btn,
                self.exec_btn,
                add_btn,
                clear_btn,
                play_btn,
            ])
        for index, button in enumerate(action_buttons):
            actions.addWidget(button, index // 3, index % 3)
        body.addLayout(actions)

        self.waypoint_chip = metric_chip('示教点 0')
        self.status = QtWidgets.QLabel('等待操作')
        self.status.setObjectName('StateBanner')
        self.status.setWordWrap(True)
        if self.calibration_mode:
            self.status.setMaximumHeight(62)
        if not self.calibration_mode:
            body.addWidget(self.waypoint_chip)
        body.addWidget(self.status)
        controls.addStretch(1)
        layout.addLayout(controls, 5)

        if color_topic and depth_topic:
            self.camera_preview = CameraWidget(color_topic, depth_topic, compact=True, default_mode='color')
            layout.addWidget(self.camera_preview, 2)

        self.state_signal.connect(self.update_current_state)
        self._subscriber = rospy.Subscriber('/joint_states', JointState, self._emit_if_alive, queue_size=1)
        self.control_mode_signal.connect(self._apply_remote_control_mode)
        self._mode_subscriber = rospy.Subscriber('/gui/joint_direct_mode', Bool, self._control_mode_cb, queue_size=1)
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
        if preserve_controller_state_on_startup is None:
            preserve_controller_state_on_startup = bool(rospy.get_param(
                '~preserve_controller_state_on_startup',
                False,
            ))
        if not preserve_controller_state_on_startup:
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
        for name in ('_actuation_subscriber', '_feedback_subscriber', '_mode_subscriber'):
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

    def _current_feedback_target(self):
        if self.current_state is None:
            return None
        name_to_pos = dict(zip(
            self.current_state.name,
            self.current_state.position,
        ))
        fallback = list(self.current_state.position)
        target = []
        for index, name in enumerate(self.names):
            value = name_to_pos.get(
                name,
                fallback[index] if index < len(fallback) else None,
            )
            if value is None or not math.isfinite(float(value)):
                return None
            target.append(float(value))
        return target

    def _begin_direct_slider_gesture(self, edited_index):
        index = int(edited_index)
        # Mode entry already synchronizes the display. A different slider must
        # not erase targets the user has just given to other joints. Only the
        # edited channel is admitted by the driver's one-hot intent protocol.
        baseline = self.positions()
        if not self._direct_gesture.begin(baseline, index):
            return

    def _end_direct_slider_gesture(self):
        self._direct_gesture.clear()
        self._active_direct_slider_index = None

    def _queue_direct_edit(self, edited_index):
        if edited_index is None or self._syncing_sliders:
            return
        index = int(edited_index)
        if 0 <= index < len(self.names):
            # Coalesce repeated samples of THIS slider, not all sliders into
            # one global last-writer. Preserve each explicit channel's value.
            self._pending_direct_edits[index] = (self.positions(), time.monotonic())
            self._pending_direct_publish = True

    def handle_slider_changed(self, _value=None):
        sender = self.sender()
        if sender in self.sliders:
            self._active_direct_slider_index = self.sliders.index(sender)
            if (
                self.realtime_direct.isChecked() and
                self._direct_gesture.edited_index !=
                self._active_direct_slider_index
            ):
                # Keyboard/wheel changes do not emit sliderPressed, so start
                # their short gesture on the first valueChanged event.
                self._begin_direct_slider_gesture(
                    self._active_direct_slider_index
                )
        self.update_labels()
        if not self._syncing_sliders:
            self.pose_target_changed.emit()
        if self.realtime_direct.isChecked() and not self._syncing_sliders:
            self._queue_direct_edit(self._active_direct_slider_index)

    def handle_slider_pressed(self):
        sender = self.sender()
        if sender not in self.sliders:
            return
        self._active_direct_slider_index = self.sliders.index(sender)
        if self.realtime_direct.isChecked():
            self._begin_direct_slider_gesture(
                self._active_direct_slider_index
            )

    def handle_slider_released(self):
        if self.realtime_direct.isChecked() and not self._syncing_sliders:
            sender = self.sender()
            index = self.sliders.index(sender) if sender in self.sliders else self._active_direct_slider_index
            self._queue_direct_edit(index)
            self.flush_direct_publish()
        self._end_direct_slider_gesture()

    def flush_direct_publish(self):
        if not self.__dict__.get('_alive', False):
            return
        self.command_conn_chip.setText('/joint_commands 连接 %d' % self.pub.get_num_connections())
        if not self.realtime_direct.isChecked():
            return
        action, target, detail = self.direct_recovery.poll(time.monotonic())
        self._apply_direct_recovery_action(action, target, detail)
        if action in ('clear', 'timeout'):
            return
        while self._pending_direct_edits:
            pending, pending_index = self.direct_recovery.pending_edit
            if pending:
                # Updating the same explicit slider may replace its goal;
                # another slider waits for the existing bounded handshake.
                if pending_index not in self._pending_direct_edits:
                    break
                index = pending_index
            else:
                index = next(iter(self._pending_direct_edits))
            target, queued_at = self._pending_direct_edits.pop(index)
            if time.monotonic() - queued_at > self.direct_recovery.timeout_sec:
                self._pending_direct_edits.clear()
                self.status.setText('待发滑条输入已过期并清理，请重新操作目标滑条')
                break
            self.publish_direct(target=target, edited_index=index)
            if self.direct_recovery.pending_edit[0]:
                break
        self._pending_direct_publish = bool(self._pending_direct_edits)
        edited_index = self._direct_gesture.edited_index
        if edited_index is not None and not self.sliders[edited_index].isSliderDown():
            self._end_direct_slider_gesture()

    def publish_direct(self, message='已显式发送到 /joint_commands', target=None, edited_index=None):
        pos = list(target) if target is not None else self._direct_gesture.compose(self.positions())
        if edited_index is None:
            edited_index = self._direct_gesture.edited_index
            if edited_index is None:
                edited_index = self._active_direct_slider_index
        action, target, detail = self.direct_recovery.submit(
            pos,
            time.monotonic(),
            edited_index=edited_index,
            # Only the one-hot channel is a new target; the driver retains all
            # other explicitly admitted manual goals for concurrent motion.
            rebase_unedited=False,
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
            self._publish_joint_target(target, edited_index=None)
            self.status.setText(
                '已用最新实测关节姿态完成零运动同步；等待驱动确认后发送滑条目标'
            )
            return
        if action == 'probe':
            self._publish_joint_target(
                target,
                edited_index=self.direct_recovery.last_action_edited_index,
            )
            self.status.setText(
                '已沿滑条方向发送 0.025 rad 以内的有界验证步；'
                '只有编码器确认真实运动后才会发送完整目标'
            )
            return
        if action == 'publish':
            self._publish_joint_target(
                target,
                edited_index=self.direct_recovery.last_action_edited_index,
            )
            connections = self.pub.get_num_connections()
            self.status.setText('%s；订阅连接数=%d' % (published_message, connections))
            return
        if action == 'clear':
            self._pending_direct_edits.clear()
            self._pending_direct_publish = False
            # The failed handshake has already issued its one explicit
            # positive-enable request.  Do not create an orphan PENDING state
            # after discarding the user target; the next slider action owns
            # the next bounded recovery attempt.
            self.status.setText(detail)
            return
        if action == 'timeout':
            self._pending_direct_edits.clear()
            self._pending_direct_publish = False
            self.status.setText(detail)
            return
        if action == 'wait' and detail:
            self.status.setText('等待直控恢复：%s' % detail)

    def _publish_joint_target(self, target, edited_index=None):
        msg = JointState()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = (
            'gui_direct_sync'
            if edited_index is None
            else 'gui_direct'
        )
        msg.name = self.names
        msg.position = list(target)
        msg.effort = [0.0] * len(msg.name)
        if edited_index is not None:
            index = int(edited_index)
            if 0 <= index < len(msg.effort):
                # One-hot intent metadata lets the real driver distinguish
                # the one explicitly edited channel from the full GUI display
                # vector. It does NOT cancel previously edited joint goals.
                msg.effort[index] = 1.0
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
        if not self.realtime_direct.isChecked():
            self._refresh_sliders_from_feedback()
        if not self.show_feedback_angles:
            return
        name_to_pos = dict(zip(msg.name, msg.position))
        fallback = list(msg.position)
        for index, (name, label) in enumerate(zip(self.names, self.feedback_labels)):
            value = name_to_pos.get(
                name,
                fallback[index] if index < len(fallback) else None,
            )
            label.setText(self._format_feedback_value(name, value))

    def feedback_positions_degrees(self):
        """Return the latest six arm encoder angles in degrees."""
        if self.current_state is None:
            return None
        name_to_pos = dict(zip(self.current_state.name, self.current_state.position))
        fallback = list(self.current_state.position)
        values = []
        for index, name in enumerate(self.names[:6]):
            value = name_to_pos.get(
                name,
                fallback[index] if index < len(fallback) else None,
            )
            if value is None or not math.isfinite(float(value)):
                return None
            values.append(math.degrees(float(value)))
        return values

    @staticmethod
    def _format_feedback_value(name, value):
        if value is None:
            return '--'
        try:
            value = float(value)
        except (TypeError, ValueError):
            return '--'
        if not math.isfinite(value):
            return '--'
        if name == 'right_finger':
            return '%.3f m' % value
        return '%+.2f°' % math.degrees(value)

    def _refresh_sliders_from_feedback(self):
        if self.current_state is None:
            return
        values = dict(zip(self.current_state.name, self.current_state.position))
        self._syncing_sliders = True
        try:
            for name, slider in zip(self.names, self.sliders):
                value = values.get(name)
                if value is None or not math.isfinite(float(value)):
                    continue
                target = int(max(slider.minimum(), min(slider.maximum(), round(value * 1000.0))))
                previous = slider.blockSignals(True)
                try:
                    slider.setValue(target)
                finally:
                    slider.blockSignals(previous)
            self.update_labels()
        finally:
            self._syncing_sliders = False

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

    def _control_mode_cb(self, msg):
        if self.__dict__.get('_alive', False):
            self.control_mode_signal.emit(bool(msg.data))

    def _apply_remote_control_mode(self, enabled):
        if self.realtime_direct.isChecked() == enabled:
            return
        self.realtime_direct.blockSignals(True)
        self.realtime_direct.setChecked(enabled)
        self.realtime_direct.blockSignals(False)
        # Refresh this GUI's own latched publisher as well as the parameter:
        # otherwise a restarted driver can receive our pre-handoff mode even
        # though the checkbox followed the remote change. Its echo is a no-op
        # because the equality guard above sees the already updated checkbox.
        self.update_control_mode(enabled)

    def update_control_mode(self, direct_enabled, apply_controller_switch=True, publish_mode=True):
        del apply_controller_switch
        if publish_mode:
            rospy.set_param('/gui/joint_direct_mode', bool(direct_enabled))
            self.control_mode_pub.publish(Bool(data=bool(direct_enabled)))
        self.plan_btn.setEnabled(not direct_enabled)
        self.exec_btn.setEnabled(not direct_enabled)
        for slider in self.sliders:
            slider.setEnabled(bool(direct_enabled))
        if direct_enabled:
            self._refresh_sliders_from_feedback()
            self.status.setText('关节直控：滑条拥有最高控制权，自动运动已禁止')
        else:
            self._pending_direct_publish = False
            self._pending_direct_edits.clear()
            self.direct_recovery.cancel()
            self._end_direct_slider_gesture()
            self._refresh_sliders_from_feedback()
            self.status.setText('自动模式：允许自主运动，滑条实时显示实测关节角度')

    @staticmethod
    def _default_direct_control_enabled():
        configured = rospy.get_param('/gui/default_joint_direct_control', False)
        default = bool(rospy.get_param('~default_joint_direct_control', configured))
        return bool(rospy.get_param('/gui/joint_direct_mode', default))
