import json
import math
import threading

import rospy
from PyQt5 import QtCore, QtWidgets
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from alicia_flexible_grasp_supervisor.srv import CartesianJog, SetFloat, TcpCalibrationCommand
from gui.theme import metric_chip, panel


class TcpCalibrationWidget(QtWidgets.QWidget):
    status_signal = QtCore.pyqtSignal(dict)
    service_result_signal = QtCore.pyqtSignal(object)
    joint_signal = QtCore.pyqtSignal(float, float)

    def __init__(self):
        super().__init__()
        self._alive = True
        self._busy = False
        self._last_joint_stamp = 0.0
        self._current_gripper_m = 0.0
        self._motion_buttons = []

        root = QtWidgets.QHBoxLayout(self)
        root.setContentsMargins(14, 14, 14, 14)
        root.setSpacing(14)
        root.addWidget(self._build_motion_panel(), 3)
        root.addWidget(self._build_calibration_panel(), 2)

        self.status_signal.connect(self._apply_status)
        self.service_result_signal.connect(self._apply_service_result)
        self.joint_signal.connect(self._apply_joint_state)
        self._joint_subscriber = rospy.Subscriber('/joint_states', JointState, self._joint_callback, queue_size=5)
        self._status_subscriber = rospy.Subscriber('/tcp_calibration/status', String, self._status_callback, queue_size=2)
        self.destroyed.connect(self._shutdown_ros)

    def _build_motion_panel(self):
        frame, body = panel(
            'TCP 标定微动控制',
            'X/Y/Z 沿 base_link 平移；Rx/Ry/Rz 绕当前工具局部轴旋转。调整步长不会发布运动命令。',
        )
        body.setSpacing(9)

        step_grid = QtWidgets.QGridLayout()
        self.translation_step = self._spinbox(0.1, 20.0, 1.0, 0.1, 1, ' mm')
        self.rotation_step = self._spinbox(0.5, 15.0, 2.0, 0.5, 1, ' deg')
        step_grid.addWidget(QtWidgets.QLabel('平移步长'), 0, 0)
        step_grid.addWidget(self.translation_step, 0, 1)
        step_grid.addWidget(QtWidgets.QLabel('旋转步长'), 0, 2)
        step_grid.addWidget(self.rotation_step, 0, 3)
        body.addLayout(step_grid)

        motion_grid = QtWidgets.QGridLayout()
        motion_grid.setHorizontalSpacing(8)
        motion_grid.setVerticalSpacing(6)
        translation_header = QtWidgets.QLabel('基坐标平移  X / Y / Z')
        rotation_header = QtWidgets.QLabel('工具局部旋转  Rx / Ry / Rz')
        translation_header.setObjectName('MutedLabel')
        rotation_header.setObjectName('MutedLabel')
        translation_header.setAlignment(QtCore.Qt.AlignCenter)
        rotation_header.setAlignment(QtCore.Qt.AlignCenter)
        motion_grid.addWidget(translation_header, 0, 0, 1, 2)
        motion_grid.addWidget(rotation_header, 0, 2, 1, 2)
        axes = [('X+', 1, 0), ('X-', 1, 1), ('Y+', 2, 0), ('Y-', 2, 1), ('Z+', 3, 0), ('Z-', 3, 1),
                ('Rx+', 1, 2), ('Rx-', 1, 3), ('Ry+', 2, 2), ('Ry-', 2, 3), ('Rz+', 3, 2), ('Rz-', 3, 3)]
        for label, row, column in axes:
            button = QtWidgets.QPushButton(label)
            button.setObjectName('AxisButton')
            button.setCursor(QtCore.Qt.PointingHandCursor)
            button.setToolTip(self._axis_tooltip(label))
            button.clicked.connect(lambda _, axis=label: self._jog(axis))
            motion_grid.addWidget(button, row, column)
            self._motion_buttons.append(button)
        body.addLayout(motion_grid)

        gripper_title = QtWidgets.QLabel('探针夹持')
        gripper_title.setObjectName('PanelTitle')
        body.addWidget(gripper_title)

        gripper_grid = QtWidgets.QGridLayout()
        gripper_grid.setHorizontalSpacing(10)
        gripper_grid.setVerticalSpacing(6)
        self.gripper_current = metric_chip('当前开度 -- mm', accent=True)
        self.gripper_step = self._spinbox(0.1, 5.0, 0.5, 0.1, 1, ' mm')
        self.gripper_target = self._spinbox(0.0, 50.0, 49.8, 0.1, 1, ' mm')
        gripper_grid.addWidget(self.gripper_current, 0, 0, 1, 4)
        gripper_grid.addWidget(QtWidgets.QLabel('夹爪步长'), 1, 0)
        gripper_grid.addWidget(self.gripper_step, 1, 1)
        gripper_grid.addWidget(QtWidgets.QLabel('目标开度'), 1, 2)
        gripper_grid.addWidget(self.gripper_target, 1, 3)
        close_button = QtWidgets.QPushButton('夹紧 -')
        open_button = QtWidgets.QPushButton('张开 +')
        send_button = QtWidgets.QPushButton('发送目标开度')
        close_button.clicked.connect(lambda: self._step_gripper(-1.0))
        open_button.clicked.connect(lambda: self._step_gripper(1.0))
        send_button.clicked.connect(self._send_gripper_target)
        gripper_grid.addWidget(close_button, 2, 0)
        gripper_grid.addWidget(open_button, 2, 1)
        gripper_grid.addWidget(send_button, 2, 2, 1, 2)
        self._motion_buttons.extend([close_button, open_button, send_button])
        body.addLayout(gripper_grid)

        self.motion_status = QtWidgets.QLabel('机械臂未上电时可调整步长；运动命令将被反馈门控阻止。')
        self.motion_status.setObjectName('StateBanner')
        self.motion_status.setWordWrap(True)
        body.addWidget(self.motion_status)
        body.addStretch(1)
        return frame

    @staticmethod
    def _axis_tooltip(axis):
        axis_name = axis[:-1]
        direction = '正方向' if axis.endswith('+') else '负方向'
        if axis_name.startswith('R'):
            return '绕当前工具局部 %s 轴按右手定则向%s旋转一个旋转步长' % (axis_name[1:], direction)
        return '沿机械臂基坐标系 base_link 的 %s %s平移一个平移步长' % (axis_name, direction)

    def _build_calibration_panel(self):
        frame, body = panel(
            '固定点采样与求解',
            '每次让探针尖端触碰同一个固定点，停稳后采样；各样本应采用不同腕部姿态。',
        )
        metrics = QtWidgets.QGridLayout()
        self.sample_metric = metric_chip('样本 0 / 8', accent=True)
        self.span_metric = metric_chip('姿态跨度 --')
        self.rms_metric = metric_chip('RMS --')
        self.max_metric = metric_chip('MAX --')
        metrics.addWidget(self.sample_metric, 0, 0)
        metrics.addWidget(self.span_metric, 0, 1)
        metrics.addWidget(self.rms_metric, 1, 0)
        metrics.addWidget(self.max_metric, 1, 1)
        body.addLayout(metrics)

        self.solution_label = QtWidgets.QLabel('TCP Grasp_base xyz：尚未求解')
        self.solution_label.setObjectName('MetricChip')
        self.solution_label.setWordWrap(True)
        body.addWidget(self.solution_label)

        capture = QtWidgets.QPushButton('记录当前触点姿态')
        capture.setObjectName('PrimaryButton')
        capture.clicked.connect(lambda: self._calibration_command('capture'))
        undo = QtWidgets.QPushButton('撤销最后样本')
        undo.clicked.connect(lambda: self._calibration_command('undo'))
        solve = QtWidgets.QPushButton('求解 TCP')
        solve.clicked.connect(lambda: self._calibration_command('solve'))
        save = QtWidgets.QPushButton('保存结果')
        save.clicked.connect(lambda: self._calibration_command('save'))
        clear = QtWidgets.QPushButton('清空样本')
        clear.setObjectName('DangerButton')
        clear.clicked.connect(self._confirm_clear)
        body.addWidget(capture)
        row = QtWidgets.QHBoxLayout()
        row.addWidget(undo)
        row.addWidget(solve)
        body.addLayout(row)
        row2 = QtWidgets.QHBoxLayout()
        row2.addWidget(save)
        row2.addWidget(clear)
        body.addLayout(row2)

        self.calibration_status = QtWidgets.QLabel('等待 TCP 标定节点')
        self.calibration_status.setObjectName('StateBanner')
        self.calibration_status.setWordWrap(True)
        body.addWidget(self.calibration_status)
        body.addStretch(1)
        return frame

    @staticmethod
    def _spinbox(minimum, maximum, value, step, decimals, suffix):
        widget = QtWidgets.QDoubleSpinBox()
        widget.setRange(minimum, maximum)
        widget.setValue(value)
        widget.setSingleStep(step)
        widget.setDecimals(decimals)
        widget.setSuffix(suffix)
        widget.setMinimumWidth(120)
        return widget

    @staticmethod
    def jog_request(axis, translation_step_mm, rotation_step_deg):
        values = [0.0] * 6
        index_by_axis = {'X': 0, 'Y': 1, 'Z': 2, 'Rx': 3, 'Ry': 4, 'Rz': 5}
        sign = 1.0 if axis.endswith('+') else -1.0
        name = axis[:-1]
        if name not in index_by_axis:
            raise ValueError('unknown TCP jog axis: %s' % axis)
        step = float(translation_step_mm) / 1000.0 if index_by_axis[name] < 3 else math.radians(float(rotation_step_deg))
        values[index_by_axis[name]] = sign * step
        return tuple(values) + (True,)

    def _jog(self, axis):
        try:
            request = self.jog_request(axis, self.translation_step.value(), self.rotation_step.value())
        except ValueError as exc:
            self.motion_status.setText(str(exc))
            return
        self._start_call('点动 %s' % axis, self._call_cartesian_jog, request, require_feedback=True)

    @staticmethod
    def _call_cartesian_jog(request):
        rospy.wait_for_service('/supervisor/cartesian_jog', timeout=2.0)
        response = rospy.ServiceProxy('/supervisor/cartesian_jog', CartesianJog)(*request)
        return bool(response.success), str(response.message)

    def _step_gripper(self, direction):
        target_mm = max(0.0, min(50.0, self._current_gripper_m * 1000.0 + float(direction) * self.gripper_step.value()))
        self.gripper_target.setValue(target_mm)
        self._send_gripper_target()

    def _send_gripper_target(self):
        target_m = self.gripper_target.value() / 1000.0
        self._start_call('夹爪目标 %.1f mm' % (target_m * 1000.0), self._call_gripper, target_m, require_feedback=True)

    @staticmethod
    def _call_gripper(target_m):
        rospy.wait_for_service('/supervisor/set_gripper', timeout=2.0)
        response = rospy.ServiceProxy('/supervisor/set_gripper', SetFloat)(float(target_m))
        return bool(response.success), str(response.message)

    def _calibration_command(self, command):
        self._start_call('TCP %s' % command, self._call_calibration, command, require_feedback=(command == 'capture'))

    @staticmethod
    def _call_calibration(command):
        rospy.wait_for_service('/tcp_calibration/command', timeout=2.0)
        response = rospy.ServiceProxy('/tcp_calibration/command', TcpCalibrationCommand)(str(command))
        return bool(response.success), str(response.message), response

    def _confirm_clear(self):
        reply = QtWidgets.QMessageBox.question(
            self,
            '清空 TCP 样本',
            '确定清空全部 TCP 标定样本？机械臂不会移动。',
            QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No,
            QtWidgets.QMessageBox.No,
        )
        if reply == QtWidgets.QMessageBox.Yes:
            self._calibration_command('clear')

    def _start_call(self, label, callback, argument, require_feedback=False):
        if self._busy:
            self.motion_status.setText('上一条命令仍在执行，请等待完成')
            return
        if require_feedback and not self._joint_feedback_fresh():
            self.motion_status.setText('命令已阻止：没有新鲜的真机关节反馈，请先上电并确认 /joint_states')
            return
        self._set_busy(True)
        self.motion_status.setText('%s：处理中' % label)

        def run():
            try:
                result = callback(argument)
            except Exception as exc:
                result = (False, '%s 失败：%s' % (label, exc))
            if self._alive:
                self.service_result_signal.emit(result)

        threading.Thread(target=run, daemon=True).start()

    def _apply_service_result(self, result):
        self._set_busy(False)
        success = bool(result[0])
        message = str(result[1])
        self.motion_status.setText(message)
        if len(result) > 2:
            response = result[2]
            payload = {
                'message': message,
                'sample_count': int(response.sample_count),
                'tcp_translation_xyz_m': [
                    float(response.tcp_translation.x),
                    float(response.tcp_translation.y),
                    float(response.tcp_translation.z),
                ] if response.rms_error_m > 0.0 else None,
                'rms_error_m': float(response.rms_error_m),
                'max_error_m': float(response.max_error_m),
                'orientation_span_deg': float(response.orientation_span_deg),
            }
            self._apply_status(payload)
        if not success:
            self.calibration_status.setText(message)

    def _set_busy(self, busy):
        self._busy = bool(busy)
        for button in self._motion_buttons:
            button.setEnabled(not self._busy)

    def _joint_callback(self, msg):
        try:
            index = list(msg.name).index('right_finger')
            gripper = float(msg.position[index])
            stamp = msg.header.stamp.to_sec() if msg.header.stamp.to_sec() > 0.0 else rospy.get_time()
            self._last_joint_stamp = stamp
            self._current_gripper_m = gripper
            if self._alive:
                self.joint_signal.emit(gripper, max(0.0, rospy.get_time() - stamp))
        except Exception:
            return

    def _apply_joint_state(self, gripper_m, age_sec):
        self.gripper_current.setText('当前开度 %.1f mm' % (gripper_m * 1000.0))
        if not self.gripper_target.hasFocus() and not self._busy:
            self.gripper_target.setValue(gripper_m * 1000.0)
        self.gripper_current.setProperty('accent', 'true' if age_sec <= 1.0 else 'false')
        self.gripper_current.style().unpolish(self.gripper_current)
        self.gripper_current.style().polish(self.gripper_current)

    def _joint_feedback_fresh(self):
        return self._last_joint_stamp > 0.0 and rospy.get_time() - self._last_joint_stamp <= 1.0

    def _status_callback(self, msg):
        try:
            payload = json.loads(msg.data)
        except Exception:
            payload = {'message': str(msg.data)}
        if self._alive:
            self.status_signal.emit(payload)

    def _apply_status(self, payload):
        count = int(payload.get('sample_count', 0) or 0)
        minimum = int(payload.get('minimum_samples', 8) or 8)
        self.sample_metric.setText('样本 %d / %d' % (count, minimum))
        span = float(payload.get('orientation_span_deg', 0.0) or 0.0)
        self.span_metric.setText('姿态跨度 %.1f°' % span if span > 0.0 else '姿态跨度 --')
        rms = float(payload.get('rms_error_m', 0.0) or 0.0)
        maximum = float(payload.get('max_error_m', 0.0) or 0.0)
        self.rms_metric.setText('RMS %.2f mm' % (rms * 1000.0) if rms > 0.0 else 'RMS --')
        self.max_metric.setText('MAX %.2f mm' % (maximum * 1000.0) if maximum > 0.0 else 'MAX --')
        translation = payload.get('tcp_translation_xyz_m')
        if translation:
            self.solution_label.setText(
                'TCP Grasp_base xyz：%.4f, %.4f, %.4f m' % tuple(float(value) for value in translation)
            )
        self.calibration_status.setText(str(payload.get('message', '等待操作')))

    def _shutdown_ros(self, *_):
        self._alive = False
        for subscriber in (getattr(self, '_joint_subscriber', None), getattr(self, '_status_subscriber', None)):
            if subscriber is not None:
                try:
                    subscriber.unregister()
                except Exception:
                    pass
