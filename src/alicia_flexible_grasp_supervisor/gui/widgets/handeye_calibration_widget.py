import json
import math
import threading
import time

from PyQt5 import QtCore, QtGui, QtWidgets
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_srvs.srv import Empty

from gui.theme import metric_chip, panel
from gui.widgets.joint_control_widget import JointControlWidget

try:
    from cv_bridge import CvBridge
    import cv2
except Exception:
    CvBridge = None
    cv2 = None

try:
    from easy_handeye_msgs.srv import (
        ComputeCalibration,
        ListAlgorithms,
        RemoveSample,
        SetAlgorithm,
        TakeSample,
    )
except Exception:
    ComputeCalibration = None
    ListAlgorithms = None
    RemoveSample = None
    SetAlgorithm = None
    TakeSample = None


def evaluate_charuco_quality(
    quality,
    now_sec,
    max_age_ms=500.0,
    min_corners=35,
    min_edge_clearance_px=12.0,
    max_reprojection_rms_px=1.5,
):
    """Evaluate one /charuco/quality payload without touching Qt or ROS."""
    if not quality:
        return False, '等待 ChArUco', {}
    stamp = float(quality.get('stamp') or 0.0)
    age_ms = (float(now_sec) - stamp) * 1000.0 if stamp > 0.0 else 1e9
    corners = int(quality.get('charuco_count') or 0)
    markers = int(quality.get('marker_count') or 0)
    edge_value = float(quality.get('edge_clearance_px', -1.0) or -1.0)
    reproj_value = float(quality.get('reprojection_rms_px', -1.0) or -1.0)
    detail = {
        'age_ms': age_ms,
        'corners': corners,
        'markers': markers,
        'edge': edge_value,
        'reproj': reproj_value,
    }
    if not bool(quality.get('pose_ok')):
        return False, '未识别到 ChArUco', detail
    if not math.isfinite(age_ms) or age_ms < 0.0 or age_ms > float(max_age_ms):
        return False, '质量数据已过期', detail
    if corners < int(min_corners):
        return False, '角点数量不足', detail
    if not math.isfinite(edge_value) or edge_value < float(min_edge_clearance_px):
        return False, '标定板距离图像边缘过近', detail
    if (
        not math.isfinite(reproj_value)
        or reproj_value < 0.0
        or reproj_value > float(max_reprojection_rms_px)
    ):
        return False, '重投影误差过大', detail
    return True, '质量合格', detail


def format_charuco_quality(ok, reason, detail):
    if not detail:
        return reason
    return (
        '%s | age %.0f ms | corners %d | markers %d | edge %.1f px | RMS %.3f px'
        % (
            '质量合格' if ok else reason,
            detail['age_ms'],
            detail['corners'],
            detail['markers'],
            detail['edge'],
            detail['reproj'],
        )
    )


class OnDemandRgbView(QtWidgets.QWidget):
    """RGB view that owns no ROS image subscription while switched off."""

    frame_signal = QtCore.pyqtSignal(object)

    def __init__(self, topic):
        super().__init__()
        self.topic = str(topic)
        self.bridge = CvBridge() if CvBridge else None
        self._alive = True
        self._subscriber = None
        self._last_pixmap = None

        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        self.status = metric_chip('图像已关闭', accent=True)
        self.status.setToolTip(self.topic)
        self.image_label = QtWidgets.QLabel('点击“开启图像”后订阅 RGB 图像')
        self.image_label.setObjectName('CameraFrame')
        self.image_label.setAlignment(QtCore.Qt.AlignCenter)
        self.image_label.setMinimumSize(360, 280)
        layout.addWidget(self.status)
        layout.addWidget(self.image_label, 1)

        self.frame_signal.connect(self._display_frame)
        self.destroyed.connect(lambda *_: self.shutdown())

    @property
    def streaming(self):
        return self._subscriber is not None

    def start(self):
        if not self._alive or self.streaming:
            return self.streaming
        if self.bridge is None or cv2 is None:
            self.status.setText('无法开启：cv_bridge / OpenCV 不可用')
            return False
        self._subscriber = rospy.Subscriber(
            self.topic,
            Image,
            self._image_cb,
            queue_size=1,
            buff_size=2 ** 24,
            tcp_nodelay=True,
        )
        self.status.setText('RGB 已开启，等待图像')
        self.image_label.setText('等待 %s ...' % self.topic)
        return True

    def stop(self):
        subscriber = self._subscriber
        self._subscriber = None
        if subscriber is not None:
            try:
                subscriber.unregister()
            except Exception:
                pass
        self._last_pixmap = None
        try:
            self.image_label.clear()
            self.image_label.setText('图像已关闭；当前没有 RGB 订阅')
            self.status.setText('图像已关闭')
        except RuntimeError:
            pass

    def _image_cb(self, msg):
        if not self._alive or not self.streaming:
            return
        try:
            bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
            self.frame_signal.emit(rgb)
        except Exception as exc:
            rospy.logwarn_throttle(2.0, 'Hand-eye RGB conversion failed: %s', exc)

    def _display_frame(self, rgb):
        try:
            if not self._alive or not self.streaming:
                return
            height, width, channels = rgb.shape
            image = QtGui.QImage(
                rgb.data,
                width,
                height,
                channels * width,
                QtGui.QImage.Format_RGB888,
            )
            self._last_pixmap = QtGui.QPixmap.fromImage(image.copy())
            self.status.setText('RGB %d x %d' % (width, height))
            self._render()
        except RuntimeError:
            self.shutdown()

    def _render(self):
        if self._last_pixmap is None:
            return
        self.image_label.setPixmap(self._last_pixmap.scaled(
            self.image_label.size(),
            QtCore.Qt.KeepAspectRatio,
            QtCore.Qt.FastTransformation,
        ))

    def resizeEvent(self, event):
        super().resizeEvent(event)
        self._render()

    def shutdown(self):
        if not self._alive:
            return
        self.stop()
        self._alive = False

    def closeEvent(self, event):
        self.shutdown()
        super().closeEvent(event)


class HandeyeCalibrationWidget(QtWidgets.QWidget):
    """Integrated manual-pose, ChArUco preview and easy_handeye page."""

    ui_event = QtCore.pyqtSignal(str, object)

    def __init__(self):
        super().__init__()
        self._alive = True
        self._quality_lock = threading.RLock()
        self._latest_quality = None
        self._can_take_sample = False
        self._stability_busy = False
        self._service_busy = False
        self._sample_list = None
        self._joint_records = {}

        self.calibration_namespace = str(rospy.get_param(
            '/gui/handeye/calibration_namespace',
            '/d405_cc200_recalib_20260809_eye_on_hand',
        )).rstrip('/')
        self.image_topic = str(rospy.get_param(
            '/gui/handeye/image_topic',
            '/charuco/result',
        ))
        self.quality_topic = str(rospy.get_param(
            '/gui/handeye/quality_topic',
            '/charuco/quality',
        ))
        self.max_quality_age_ms = float(rospy.get_param(
            '/gui/handeye/max_quality_age_ms', 500.0,
        ))
        self.min_charuco_corners = int(rospy.get_param(
            '/gui/handeye/min_charuco_corners', 35,
        ))
        self.min_edge_clearance_px = float(rospy.get_param(
            '/gui/handeye/min_edge_clearance_px', 12.0,
        ))
        self.max_reprojection_rms_px = float(rospy.get_param(
            '/gui/handeye/max_reprojection_rms_px', 1.5,
        ))
        self.stable_window_sec = float(rospy.get_param(
            '/gui/handeye/stable_window_sec', 2.0,
        ))
        self.stable_required_hits = int(rospy.get_param(
            '/gui/handeye/stable_required_hits', 4,
        ))

        self._build_ui()
        self.ui_event.connect(self._handle_ui_event)
        self._quality_subscriber = rospy.Subscriber(
            self.quality_topic,
            String,
            self._quality_cb,
            queue_size=1,
        )
        self.quality_timer = QtCore.QTimer(self)
        self.quality_timer.timeout.connect(self._refresh_quality_display)
        self.quality_timer.start(250)
        self.destroyed.connect(lambda *_: self._shutdown_ros())
        QtCore.QTimer.singleShot(300, self.refresh_samples)
        QtCore.QTimer.singleShot(2500, self.refresh_algorithms)

    def _build_ui(self):
        root = QtWidgets.QGridLayout(self)
        root.setContentsMargins(10, 10, 10, 10)
        root.setSpacing(10)
        root.setColumnStretch(0, 6)
        root.setColumnStretch(1, 5)
        root.setRowStretch(0, 5)
        root.setRowStretch(1, 3)

        self.joint_control = JointControlWidget(
            show_feedback_angles=True,
            arm_only=True,
            calibration_mode=True,
            preserve_controller_state_on_startup=True,
        )
        self.joint_control.pose_target_changed.connect(self._pose_changed)
        self.joint_control.setMinimumWidth(500)
        root.addWidget(self.joint_control, 0, 0)

        image_frame, image_body = panel('手眼标定 RGB / ChArUco')
        image_frame.setObjectName('HandeyeImagePanel')
        image_frame.setMinimumWidth(430)
        image_body.setContentsMargins(10, 8, 10, 10)
        image_body.setSpacing(6)
        image_controls = QtWidgets.QHBoxLayout()
        image_controls.addWidget(QtWidgets.QLabel('图像输出'))
        self.image_mode_selector = QtWidgets.QComboBox()
        self.image_mode_selector.setObjectName('HandeyeImageModeSelector')
        self.image_mode_selector.addItem('关闭图像', 'off')
        self.image_mode_selector.addItem('开启图像', 'on')
        self.image_mode_selector.setMinimumWidth(180)
        self.image_mode_selector.currentIndexChanged.connect(
            self._image_mode_changed
        )
        image_controls.addWidget(self.image_mode_selector)
        image_controls.addStretch(1)
        image_body.addLayout(image_controls)
        image_hint = QtWidgets.QLabel(
            '默认不订阅 RGB；开启后仅显示标注图，不需要深度图。'
        )
        image_hint.setObjectName('MutedLabel')
        image_hint.setWordWrap(True)
        image_body.addWidget(image_hint)
        self.image_view = OnDemandRgbView(self.image_topic)
        self.image_view.image_label.setMinimumSize(300, 170)
        self.image_view.setVisible(False)
        image_body.addWidget(self.image_view, 1)
        root.addWidget(image_frame, 0, 1)

        calibration_frame, calibration_body = panel('CC200 采样与标定')
        calibration_frame.setObjectName('HandeyeSamplingPanel')
        calibration_body.setContentsMargins(10, 8, 10, 10)
        calibration_body.setSpacing(6)
        board = metric_chip(
            'CC200 | 12 x 9 | DICT_5X5_100 | marker 11.25 mm',
            accent=True,
        )
        board.setToolTip('当前技术路线确认的标定板参数')

        self.quality_status = QtWidgets.QLabel('等待 /charuco/quality')
        self.quality_status.setObjectName('StateBanner')
        self.quality_status.setWordWrap(True)

        algorithm_row = QtWidgets.QHBoxLayout()
        algorithm_row.addWidget(QtWidgets.QLabel('算法'))
        self.algorithm_combo = QtWidgets.QComboBox()
        self.algorithm_combo.currentTextChanged.connect(self._set_algorithm)
        algorithm_row.addWidget(self.algorithm_combo, 1)
        self.refresh_algorithm_btn = QtWidgets.QPushButton('刷新算法')
        self.refresh_algorithm_btn.clicked.connect(self.refresh_algorithms)
        algorithm_row.addWidget(self.refresh_algorithm_btn)

        action_grid = QtWidgets.QGridLayout()
        self.stable_btn = QtWidgets.QPushButton('确认画面稳定')
        self.take_sample_btn = QtWidgets.QPushButton('采集样本')
        self.refresh_sample_btn = QtWidgets.QPushButton('刷新样本')
        self.remove_sample_btn = QtWidgets.QPushButton('删除选中样本')
        self.compute_btn = QtWidgets.QPushButton('计算标定')
        self.save_btn = QtWidgets.QPushButton('保存标定')
        self.take_sample_btn.setObjectName('PrimaryButton')
        self.compute_btn.setObjectName('PrimaryButton')
        self.save_btn.setObjectName('DangerButton')
        self.take_sample_btn.setEnabled(False)
        self.remove_sample_btn.setEnabled(False)
        self.compute_btn.setEnabled(False)
        self.save_btn.setEnabled(False)
        self.stable_btn.clicked.connect(self.check_stability)
        self.take_sample_btn.clicked.connect(self.take_sample)
        self.refresh_sample_btn.clicked.connect(self.refresh_samples)
        self.remove_sample_btn.clicked.connect(self.remove_selected_sample)
        self.compute_btn.clicked.connect(self.compute_calibration)
        self.save_btn.clicked.connect(self.save_calibration)
        for index, button in enumerate((
            self.stable_btn,
            self.take_sample_btn,
            self.refresh_sample_btn,
            self.remove_sample_btn,
            self.compute_btn,
            self.save_btn,
        )):
            action_grid.addWidget(button, index // 3, index % 3)

        self.sample_table = QtWidgets.QTableWidget(0, 3)
        self.sample_table.setHorizontalHeaderLabels((
            '样本',
            '采样时实测关节角度（deg）',
            'camera -> board 平移（m）',
        ))
        self.sample_table.horizontalHeader().setStretchLastSection(True)
        self.sample_table.horizontalHeader().setSectionResizeMode(
            1, QtWidgets.QHeaderView.Stretch,
        )
        self.sample_table.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectRows)
        self.sample_table.setSelectionMode(QtWidgets.QAbstractItemView.SingleSelection)
        self.sample_table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self.sample_table.setMinimumHeight(96)
        self.sample_table.itemSelectionChanged.connect(self._update_sample_buttons)

        self.calibration_output = QtWidgets.QPlainTextEdit()
        self.calibration_output.setReadOnly(True)
        self.calibration_output.setMaximumHeight(64)
        self.calibration_output.setPlaceholderText('计算结果将在这里显示；保存需要再次确认。')

        self.operation_status = QtWidgets.QLabel(
            '等待操作；移动任一关节滑条后需要重新确认画面稳定。'
        )
        self.operation_status.setObjectName('StateBanner')
        self.operation_status.setWordWrap(True)

        summary_row = QtWidgets.QHBoxLayout()
        summary_row.setSpacing(8)
        summary_row.addWidget(board, 2)
        summary_row.addWidget(self.quality_status, 5)
        calibration_body.addLayout(summary_row)

        lower_row = QtWidgets.QHBoxLayout()
        lower_row.setSpacing(10)
        control_column = QtWidgets.QVBoxLayout()
        control_column.setSpacing(6)
        control_column.addLayout(algorithm_row)
        control_column.addLayout(action_grid)
        control_column.addWidget(self.operation_status)
        sample_column = QtWidgets.QVBoxLayout()
        sample_column.setSpacing(6)
        sample_column.addWidget(self.sample_table, 1)
        sample_column.addWidget(self.calibration_output)
        lower_row.addLayout(control_column, 4)
        lower_row.addLayout(sample_column, 6)
        calibration_body.addLayout(lower_row, 1)
        root.addWidget(calibration_frame, 1, 0, 1, 2)

    def open_image(self):
        index = self.image_mode_selector.findData('on')
        if self.image_mode_selector.currentIndex() != index:
            self.image_mode_selector.setCurrentIndex(index)
            return
        self.image_view.setVisible(True)
        self.image_view.start()

    def close_image(self):
        index = self.image_mode_selector.findData('off')
        if self.image_mode_selector.currentIndex() != index:
            self.image_mode_selector.setCurrentIndex(index)
            return
        self.image_view.stop()
        self.image_view.setVisible(False)

    def _image_mode_changed(self):
        if self.image_mode_selector.currentData() == 'on':
            self.open_image()
        else:
            self.close_image()

    def _quality_cb(self, msg):
        if not self._alive:
            return
        try:
            quality = json.loads(msg.data)
        except Exception:
            return
        with self._quality_lock:
            self._latest_quality = quality

    def _quality_gate(self):
        with self._quality_lock:
            quality = dict(self._latest_quality) if self._latest_quality else None
        return evaluate_charuco_quality(
            quality,
            rospy.Time.now().to_sec(),
            self.max_quality_age_ms,
            self.min_charuco_corners,
            self.min_edge_clearance_px,
            self.max_reprojection_rms_px,
        )

    def _refresh_quality_display(self):
        ok, reason, detail = self._quality_gate()
        self.quality_status.setText(format_charuco_quality(ok, reason, detail))
        if self._can_take_sample and not ok:
            self._set_can_take_sample(False)
            self.operation_status.setText('画面质量已失效：%s；请重新确认稳定' % reason)

    def _pose_changed(self):
        self._set_can_take_sample(False)
        self.save_btn.setEnabled(False)
        self.operation_status.setText('关节目标已变化；到位后请重新确认画面稳定')

    def _set_can_take_sample(self, enabled):
        self._can_take_sample = bool(enabled)
        self.take_sample_btn.setEnabled(self._can_take_sample and not self._service_busy)

    def check_stability(self):
        if self._stability_busy:
            return
        self._set_can_take_sample(False)
        self._stability_busy = True
        self.stable_btn.setEnabled(False)
        self.operation_status.setText('正在检查 ChArUco 画面稳定性...')

        def worker():
            hits = 0
            last_text = '等待 ChArUco'
            deadline = time.monotonic() + self.stable_window_sec
            while time.monotonic() < deadline and not rospy.is_shutdown():
                ok, reason, detail = self._quality_gate()
                hits = hits + 1 if ok else 0
                last_text = format_charuco_quality(ok, reason, detail)
                self._emit_ui_event(
                    'stability_progress',
                    (hits, self.stable_required_hits, last_text),
                )
                if hits >= self.stable_required_hits:
                    self._emit_ui_event('stability_done', (True, last_text))
                    return
                time.sleep(0.25)
            self._emit_ui_event('stability_done', (False, last_text))

        threading.Thread(target=worker, daemon=True).start()

    def take_sample(self):
        if self._service_busy or not self._can_take_sample:
            return
        ok, reason, _detail = self._quality_gate()
        if not ok:
            self._set_can_take_sample(False)
            self.operation_status.setText('未采样：%s' % reason)
            return
        joint_degrees = self.joint_control.feedback_positions_degrees()
        self._run_service(
            'take_sample',
            self.calibration_namespace + '/take_sample',
            TakeSample,
            (),
            {'joint_degrees': joint_degrees},
        )

    def refresh_samples(self):
        if self._service_busy:
            return
        self._run_service(
            'sample_list',
            self.calibration_namespace + '/get_sample_list',
            TakeSample,
        )

    def remove_selected_sample(self):
        row = self.sample_table.currentRow()
        if row < 0 or self._service_busy:
            return
        reply = QtWidgets.QMessageBox.question(
            self,
            '删除样本确认',
            '将删除第 %d 个手眼标定样本，是否继续？' % (row + 1),
            QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No,
            QtWidgets.QMessageBox.No,
        )
        if reply != QtWidgets.QMessageBox.Yes:
            return
        self._run_service(
            'remove_sample',
            self.calibration_namespace + '/remove_sample',
            RemoveSample,
            (row,),
            {'removed_index': row},
        )

    def compute_calibration(self):
        if self._service_busy:
            return
        self._run_service(
            'compute',
            self.calibration_namespace + '/compute_calibration',
            ComputeCalibration,
        )

    def save_calibration(self):
        if self._service_busy or not self.save_btn.isEnabled():
            return
        reply = QtWidgets.QMessageBox.question(
            self,
            '保存标定确认',
            '这会写入当前 easy_handeye 标定结果。确认结果有效并保存吗？',
            QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No,
            QtWidgets.QMessageBox.No,
        )
        if reply != QtWidgets.QMessageBox.Yes:
            return
        self._run_service(
            'save',
            self.calibration_namespace + '/save_calibration',
            Empty,
        )

    def refresh_algorithms(self):
        if self._service_busy:
            return
        self._run_service(
            'algorithms',
            self.calibration_namespace + '/list_algorithms',
            ListAlgorithms,
        )

    def _set_algorithm(self, name):
        if not name or self._service_busy or SetAlgorithm is None:
            return
        self._run_service(
            'set_algorithm',
            self.calibration_namespace + '/set_algorithm',
            SetAlgorithm,
            (str(name),),
        )

    def _run_service(self, action, service_name, service_type, args=(), context=None):
        if service_type is None:
            self.operation_status.setText(
                'easy_handeye_msgs 不可用；请先构建并 source 当前工作区'
            )
            return
        self._service_busy = True
        self._update_action_buttons()
        self.operation_status.setText('正在执行：%s' % action)
        context = dict(context or {})

        def worker():
            try:
                rospy.wait_for_service(service_name, timeout=2.0)
                response = rospy.ServiceProxy(service_name, service_type)(*args)
                self._emit_ui_event(
                    'service_success',
                    (action, response, context),
                )
            except Exception as exc:
                self._emit_ui_event('service_error', (action, str(exc)))

        threading.Thread(target=worker, daemon=True).start()

    def _emit_ui_event(self, event_name, payload):
        try:
            if self._alive:
                self.ui_event.emit(event_name, payload)
        except RuntimeError:
            pass

    def _handle_ui_event(self, event_name, payload):
        if not self._alive:
            return
        if event_name == 'stability_progress':
            hits, required, text = payload
            self.operation_status.setText(
                '稳定性检查 %d/%d | %s' % (hits, required, text)
            )
            return
        if event_name == 'stability_done':
            ok, text = payload
            self._stability_busy = False
            self.stable_btn.setEnabled(True)
            self._set_can_take_sample(ok)
            self.operation_status.setText(
                ('稳定性已确认，可以采样 | ' if ok else '稳定性检查未通过 | ') + text
            )
            return
        if event_name == 'service_error':
            action, error = payload
            self._service_busy = False
            self._update_action_buttons()
            self.operation_status.setText('%s 失败：%s' % (action, error))
            return
        if event_name == 'service_success':
            action, response, context = payload
            self._service_busy = False
            self._handle_service_success(action, response, context)
            self._update_action_buttons()

    def _handle_service_success(self, action, response, context):
        if action in ('sample_list', 'take_sample', 'remove_sample'):
            samples = getattr(response, 'samples', None)
            if samples is None:
                self.operation_status.setText('%s 返回中没有样本列表' % action)
                return
            if action == 'take_sample':
                count = len(getattr(samples, 'hand_world_samples', []))
                if count > 0:
                    self._joint_records[count - 1] = context.get('joint_degrees')
                self._set_can_take_sample(False)
                self.save_btn.setEnabled(False)
                self.operation_status.setText(
                    '已保存样本 %d；移动到下一姿态后重新确认稳定' % count
                )
            elif action == 'remove_sample':
                self._shift_joint_records_after_remove(context['removed_index'])
                self.save_btn.setEnabled(False)
                self.operation_status.setText('已删除选中样本；需要重新计算标定')
            else:
                count = len(getattr(samples, 'hand_world_samples', []))
                self.save_btn.setEnabled(False)
                self.operation_status.setText('样本列表已刷新，共 %d 个' % count)
            self._display_samples(samples)
            return
        if action == 'compute':
            valid = bool(getattr(response, 'valid', False))
            calibration = getattr(response, 'calibration', None)
            if valid and calibration is not None:
                transform = calibration.transform.transform
                self.calibration_output.setPlainText(
                    'translation [m]: x=%+.6f y=%+.6f z=%+.6f\n'
                    'quaternion: x=%+.7f y=%+.7f z=%+.7f w=%+.7f'
                    % (
                        transform.translation.x,
                        transform.translation.y,
                        transform.translation.z,
                        transform.rotation.x,
                        transform.rotation.y,
                        transform.rotation.z,
                        transform.rotation.w,
                    )
                )
                self.save_btn.setEnabled(True)
                self.operation_status.setText('标定计算完成；保存前请检查结果')
            else:
                self.calibration_output.setPlainText('标定计算无有效结果')
                self.save_btn.setEnabled(False)
                self.operation_status.setText('标定计算失败：结果无效')
            return
        if action == 'save':
            self.save_btn.setEnabled(False)
            self.operation_status.setText('当前手眼标定结果已保存')
            return
        if action == 'algorithms':
            algorithms = list(getattr(response, 'algorithms', []))
            current = str(getattr(response, 'current_algorithm', ''))
            self.algorithm_combo.blockSignals(True)
            self.algorithm_combo.clear()
            self.algorithm_combo.addItems(algorithms)
            index = self.algorithm_combo.findText(current)
            if index >= 0:
                self.algorithm_combo.setCurrentIndex(index)
            self.algorithm_combo.blockSignals(False)
            self.operation_status.setText('标定算法已刷新：%s' % (current or '未设置'))
            return
        if action == 'set_algorithm':
            success = bool(getattr(response, 'success', False))
            self.save_btn.setEnabled(False)
            self.operation_status.setText(
                '标定算法已切换，需要重新计算' if success else '标定算法切换失败'
            )

    def _display_samples(self, samples):
        self._sample_list = samples
        hand_samples = list(getattr(samples, 'hand_world_samples', []))
        camera_samples = list(getattr(samples, 'camera_marker_samples', []))
        self.sample_table.setRowCount(len(hand_samples))
        for index in range(len(hand_samples)):
            self.sample_table.setItem(index, 0, QtWidgets.QTableWidgetItem(str(index + 1)))
            joints = self._joint_records.get(index)
            joint_text = '--'
            if joints:
                joint_text = '  '.join(
                    'J%d=%+.2f°' % (joint + 1, value)
                    for joint, value in enumerate(joints)
                )
            self.sample_table.setItem(index, 1, QtWidgets.QTableWidgetItem(joint_text))
            translation_text = '--'
            if index < len(camera_samples):
                translation = camera_samples[index].translation
                translation_text = 'x=%+.4f  y=%+.4f  z=%+.4f' % (
                    translation.x,
                    translation.y,
                    translation.z,
                )
            self.sample_table.setItem(
                index,
                2,
                QtWidgets.QTableWidgetItem(translation_text),
            )
        self.compute_btn.setEnabled(len(hand_samples) >= 3 and not self._service_busy)
        self._update_sample_buttons()

    def _shift_joint_records_after_remove(self, removed_index):
        shifted = {}
        for index, values in self._joint_records.items():
            if index < removed_index:
                shifted[index] = values
            elif index > removed_index:
                shifted[index - 1] = values
        self._joint_records = shifted

    def _update_sample_buttons(self):
        selected = self.sample_table.currentRow() >= 0
        self.remove_sample_btn.setEnabled(selected and not self._service_busy)
        sample_count = self.sample_table.rowCount()
        self.compute_btn.setEnabled(sample_count >= 3 and not self._service_busy)

    def _update_action_buttons(self):
        self.take_sample_btn.setEnabled(self._can_take_sample and not self._service_busy)
        self.refresh_sample_btn.setEnabled(not self._service_busy)
        self.refresh_algorithm_btn.setEnabled(not self._service_busy)
        self.algorithm_combo.setEnabled(not self._service_busy)
        self._update_sample_buttons()

    def _shutdown_ros(self):
        if not self._alive:
            return
        self._alive = False
        try:
            self.quality_timer.stop()
        except Exception:
            pass
        subscriber = self.__dict__.get('_quality_subscriber')
        if subscriber is not None:
            try:
                subscriber.unregister()
            except Exception:
                pass
        image_view = self.__dict__.get('image_view')
        if image_view is not None:
            image_view.shutdown()

    def closeEvent(self, event):
        self._shutdown_ros()
        super().closeEvent(event)
