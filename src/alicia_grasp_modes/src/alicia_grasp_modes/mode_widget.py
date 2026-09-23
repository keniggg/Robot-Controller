"""Two independent grasp choices embedded in the existing control panel."""
import json
import threading
import time

from PyQt5 import QtCore, QtWidgets
import rospy
from std_msgs.msg import String
from alicia_grasp_modes.srv import SetGraspMode
from gui.theme import panel


TARGET_NAMES = {'carton': 'Carton 纸盒', 'unknown': '未知小物体'}
STRATEGY_NAMES = {'direct': '一次性直接抓取', 'two_stage': '两阶段抓取'}


class GraspModeWidget(QtWidgets.QWidget):
    status_received = QtCore.pyqtSignal(str)
    request_finished = QtCore.pyqtSignal(bool, str)

    def __init__(self, parent=None):
        super().__init__(parent)
        self._alive = True
        self._dirty = False
        self._requesting = False
        self._busy = False
        self._last_receipt = 0.
        self._generation = 0
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        frame, body = panel('抓取方案')
        layout.addWidget(frame)
        form = QtWidgets.QFormLayout()
        self.target = QtWidgets.QComboBox()
        self.target.setObjectName('GraspTargetChoice')
        self.strategy = QtWidgets.QComboBox()
        self.strategy.setObjectName('GraspStrategyChoice')
        for value, label in TARGET_NAMES.items():
            self.target.addItem(label, value)
        for value in ('two_stage', 'direct'):
            self.strategy.addItem(STRATEGY_NAMES[value], value)
        form.addRow('抓取目标', self.target)
        form.addRow('执行方式', self.strategy)
        body.addLayout(form)
        self.description = QtWidgets.QLabel()
        self.description.setWordWrap(True)
        body.addWidget(self.description)
        self.status = QtWidgets.QLabel('等待当前方案状态')
        self.status.setWordWrap(True)
        body.addWidget(self.status)
        self.apply_button = QtWidgets.QPushButton('应用方案并重新识别')
        self.apply_button.setObjectName('PrimaryButton')
        self.apply_button.clicked.connect(self.apply_selection)
        body.addWidget(self.apply_button)
        self.target.currentIndexChanged.connect(self._edited)
        self.strategy.currentIndexChanged.connect(self._edited)
        self.status_received.connect(self._update_status)
        self.request_finished.connect(self._finish_request)
        self._subscriber = rospy.Subscriber('/grasp_mode/status', String,
            self._receive_status, queue_size=1)
        self._timer = QtCore.QTimer(self)
        self._timer.timeout.connect(self._refresh_controls)
        self._timer.start(500)
        self.destroyed.connect(lambda *_: self._shutdown_ros())
        self._describe()
        self._refresh_controls()

    def _edited(self, *_args):
        self._dirty = True
        self._describe()

    def _describe(self):
        if self.strategy.currentData() == 'direct':
            description = '当前视角规划 → 预抓取位 → 接近、闭爪、抬升。'
        else:
            description = '先到观察位 → 重新识别规划 → 接近、闭爪、抬升。'
        if self.target.currentData() == 'unknown':
            description += ' 请将单个物体放在画面中心。'
        self.description.setText(description)

    def _receive_status(self, message):
        if self._alive:
            try:
                self.status_received.emit(message.data)
            except RuntimeError:
                self._shutdown_ros()

    def _update_status(self, text):
        try:
            value = json.loads(text)
            mode, strategy = value['mode'], value['strategy']
            if mode not in TARGET_NAMES or strategy not in STRATEGY_NAMES:
                return
            generation = int(value['generation'])
        except (ValueError, KeyError, TypeError):
            return
        if generation < self._generation:
            return
        self._generation = generation
        self._last_receipt = time.monotonic()
        self._busy = bool(value.get('active') or value.get('start_reserved')
                          or value.get('state') in ('initializing', 'switching'))
        if not self._dirty:
            for combo, selected in ((self.target, mode), (self.strategy, strategy)):
                combo.blockSignals(True)
                combo.setCurrentIndex(combo.findData(selected))
                combo.blockSignals(False)
            self._describe()
        state = value.get('state', '')
        label = '当前方案'
        if state == 'switching':
            label = '正在切换'
        elif state == 'error':
            label = '切换失败，等待重新应用'
        detail = '；执行中' if value.get('active') or value.get('start_reserved') else ''
        if value.get('pending'):
            detail += '；任务结束后切换'
        if state == 'waiting_for_target':
            detail += '；等待目标'
        if state == 'error':
            detail += '；' + str(value.get('detail', ''))
        self.status.setText('%s：%s · %s%s' % (
            label, TARGET_NAMES[mode], STRATEGY_NAMES[strategy], detail))
        self._refresh_controls()

    def _refresh_controls(self):
        # The router publishes a heartbeat, so a dead connection cannot keep
        # the mode selectors looking usable indefinitely.
        connected = self._last_receipt > 0 and time.monotonic() - self._last_receipt < 4.
        enabled = connected and not self._busy and not self._requesting
        self.target.setEnabled(enabled)
        self.strategy.setEnabled(enabled)
        self.apply_button.setEnabled(enabled)
        if self._last_receipt and not connected:
            self.status.setText('方案状态连接中断，等待恢复')

    def apply_selection(self):
        if self._requesting or not self.apply_button.isEnabled():
            return
        mode, strategy = self.target.currentData(), self.strategy.currentData()
        self._requesting = True
        self._dirty = False
        self.status.setText('正在应用方案…')
        self._refresh_controls()
        def request():
            try:
                rospy.wait_for_service('/grasp_mode/select', timeout=3.)
                response = rospy.ServiceProxy('/grasp_mode/select', SetGraspMode)(mode, strategy)
                result = bool(response.success), str(response.message)
            except Exception as exc:
                result = False, str(exc)
            if self._alive:
                try:
                    self.request_finished.emit(*result)
                except RuntimeError:
                    pass
        threading.Thread(target=request, daemon=True).start()

    def _finish_request(self, success, message):
        self._requesting = False
        if not success:
            self._dirty = True
            self.status.setText('方案未应用：' + message)
        elif message == 'QUEUED':
            self.status.setText('方案已排队，将在当前任务结束后切换')
        self._refresh_controls()

    def _shutdown_ros(self):
        if not self._alive:
            return
        self._alive = False
        self._subscriber.unregister()
        self._timer.stop()

    def closeEvent(self, event):
        self._shutdown_ros()
        super().closeEvent(event)
