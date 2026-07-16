from copy import deepcopy
import math
import threading
import time

from PyQt5 import QtWidgets, QtCore
import cv2
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
import numpy as np
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String
try:
    import tf2_ros
except Exception:
    tf2_ros = None
from alicia_flexible_grasp_supervisor.msg import ObjectPose
from alicia_flexible_grasp_supervisor.srv import CartesianJog, SetTargetPose, StartGrasp
from alicia_flexible_grasp.grasp.grasp_pose_generator import make_pregrasp_pose
from alicia_flexible_grasp.vision.model_selection import (
    normalize_model_profiles,
    resolve_yolo_model_path,
    select_yolo_model,
)
from gui.widgets.camera_widget import CameraWidget
from gui.theme import metric_chip, panel


def perception_grasp_action_mode(use_grasp6d_plan):
    if bool(use_grasp6d_plan):
        return {
            'observation_only': True,
            'show_legacy_pregrasp': False,
            'note': '当前为 6D 抓取模式：本页仅用于视觉观察/旧目标识别，抓取请使用“6D 抓取”页。',
        }
    return {
        'observation_only': False,
        'show_legacy_pregrasp': True,
        'note': '当前为传统目标点模式：可在本页规划预抓取、执行预抓取和启动抓取流程。',
    }


class PerceptionWidget(QtWidgets.QWidget):
    object_signal = QtCore.pyqtSignal(object)
    mask_signal = QtCore.pyqtSignal(object)
    detector_status_signal = QtCore.pyqtSignal(str)
    plan_result_signal = QtCore.pyqtSignal(int, bool, bool, str)
    grasp_result_signal = QtCore.pyqtSignal(bool, str)
    VISUAL_JOG_AXES = {
        'X+': (1.0, 0.0, 0.0),
        'X-': (-1.0, 0.0, 0.0),
        'Y+': (0.0, 1.0, 0.0),
        'Y-': (0.0, -1.0, 0.0),
        'Z+': (0.0, 0.0, 1.0),
        'Z-': (0.0, 0.0, -1.0),
    }
    COLOR_PRESETS = {
        'red': ([[0, 70, 50], [10, 255, 255]], [[170, 70, 50], [180, 255, 255]], '红色'),
        'orange': ([[10, 80, 60], [25, 255, 255]], None, '橙色'),
        'yellow': ([[25, 60, 80], [35, 255, 255]], None, '黄色'),
        'green': ([[28, 35, 35], [95, 255, 255]], None, '绿色'),
        'cyan': ([[85, 40, 40], [100, 255, 255]], None, '青色'),
        'blue': ([[100, 50, 40], [130, 255, 255]], None, '蓝色'),
        'purple': ([[130, 40, 40], [160, 255, 255]], None, '紫色'),
        'pink': ([[160, 40, 40], [170, 255, 255]], None, '粉色'),
        'black': ([[0, 0, 0], [180, 255, 60]], None, '黑色'),
        'white': ([[0, 0, 180], [180, 55, 255]], None, '白色'),
    }
    COLOR_KEYWORDS = {
        '红': 'red', '红色': 'red', 'red': 'red',
        '橙': 'orange', '橙色': 'orange', 'orange': 'orange',
        '黄': 'yellow', '黄色': 'yellow', 'yellow': 'yellow',
        '绿': 'green', '绿色': 'green', 'green': 'green',
        '青': 'cyan', '青色': 'cyan', 'cyan': 'cyan',
        '蓝': 'blue', '蓝色': 'blue', 'blue': 'blue',
        '紫': 'purple', '紫色': 'purple', 'purple': 'purple',
        '粉': 'pink', '粉色': 'pink', 'pink': 'pink',
        '黑': 'black', '黑色': 'black', 'black': 'black',
        '白': 'white', '白色': 'white', 'white': 'white',
    }
    DRAW_COLORS_RGB = {
        'red': (255, 70, 70),
        'orange': (255, 150, 60),
        'yellow': (255, 230, 80),
        'green': (80, 255, 120),
        'cyan': (70, 230, 255),
        'blue': (90, 150, 255),
        'purple': (190, 110, 255),
        'pink': (255, 120, 190),
        'black': (40, 40, 40),
        'white': (245, 245, 245),
    }
    SHAPE_KEYWORDS = {
        '圆': ('circle', '圆形'), '圆形': ('circle', '圆形'), '圆柱': ('circle', '圆形'), 'circle': ('circle', '圆形'),
        '方': ('square', '方形'), '方形': ('square', '方形'), '正方形': ('square', '方形'), '方块': ('square', '方形'), 'square': ('square', '方形'),
        '矩形': ('rectangle', '矩形'), '长方形': ('rectangle', '矩形'), 'rectangle': ('rectangle', '矩形'),
        '三角': ('triangle', '三角形'), '三角形': ('triangle', '三角形'), 'triangle': ('triangle', '三角形'),
    }
    YOLO_CLASS_KEYWORDS = [
        ('sports ball', 'sports ball'), ('ball', 'sports ball'), ('球', 'sports ball'),
        ('cell phone', 'cell phone'), ('手机', 'cell phone'),
        ('bottle', 'bottle'), ('瓶子', 'bottle'), ('瓶', 'bottle'),
        ('cup', 'cup'), ('杯子', 'cup'), ('杯', 'cup'),
        ('keyboard', 'keyboard'), ('键盘', 'keyboard'),
        ('mouse', 'mouse'), ('鼠标', 'mouse'),
        ('book', 'book'), ('书本', 'book'), ('书', 'book'),
        ('apple', 'apple'), ('苹果', 'apple'),
        ('banana', 'banana'), ('香蕉', 'banana'),
        ('bowl', 'bowl'), ('碗', 'bowl'),
        ('remote', 'remote'), ('遥控器', 'remote'),
        ('scissors', 'scissors'), ('剪刀', 'scissors'),
        ('toothbrush', 'toothbrush'), ('牙刷', 'toothbrush'),
        ('spoon', 'spoon'), ('勺子', 'spoon'), ('勺', 'spoon'),
        ('fork', 'fork'), ('叉子', 'fork'), ('叉', 'fork'),
        ('knife', 'knife'), ('小刀', 'knife'), ('刀', 'knife'),
    ]

    def __init__(self, topic='/perception/object', color_topic=None, depth_topic=None):
        super().__init__()
        self._alive = True
        self._subscriber = None
        self._detector_status_subscriber = None
        self._mask_subscriber = None
        self.bridge = CvBridge()
        self.topic = topic
        self.last_object = None
        self._current_object_stamp = None
        self._current_object_detected = False
        self._latest_mask = None
        self._latest_mask_stamp = None
        self._mask_status = 'mask waiting'
        self.pregrasp_pose = None
        self._planned_pregrasp_pose = None
        self._planned_pregrasp_executable = False
        self._planned_pregrasp_time = 0.0
        self._planned_target_base_xyz = None
        self._locked_grasp_target_base_xyz = None
        self._locked_grasp_target_time = 0.0
        self._pending_plan_pose = None
        self._pending_plan_token = None
        self._last_object_receive_time = None
        self._planning_active = False
        self._pregrasp_worker_token = None
        self._grasp_active = False
        self._plan_token = 0
        self._plan_timeout_sec = float(rospy.get_param('/gui/pregrasp_plan_timeout_sec', 12.0))
        self._planned_pregrasp_max_age_sec = float(rospy.get_param('/gui/pregrasp_execute_max_age_sec', 30.0))
        self._grasp_flow_lock_max_age_sec = float(rospy.get_param('/gui/grasp_flow_lock_max_age_sec', 60.0))
        self._pregrasp_status_hold_sec = float(rospy.get_param('/gui/pregrasp_status_hold_sec', 8.0))
        self._status_hold_until = 0.0
        self._pregrasp_mode = str(rospy.get_param('/grasp/pregrasp_offset_mode', 'camera_ray'))
        self._use_grasp6d_plan = bool(rospy.get_param('/grasp/use_grasp6d_plan', False))
        self._base_frame = str(rospy.get_param('/handeye/base_frame', 'base_link'))
        self._camera_frame = str(rospy.get_param('/handeye/camera_frame', rospy.get_param('/camera/frame_id', 'camera_link')))
        self._localization_warn_error_m = float(rospy.get_param('/perception/localization_warn_error_m', 0.08))
        self._max_object_age_sec = float(rospy.get_param('/perception/max_object_age_sec', 1.5))
        self._localization_ok = True
        self._localization_error_m = None
        self._required_stable_detections = max(1, int(rospy.get_param('/gui/pregrasp_required_stable_detections', 3)))
        self._object_stability_radius_m = float(rospy.get_param('/gui/pregrasp_stability_radius_m', 0.08))
        self._object_stability_pixel_radius_px = float(rospy.get_param('/gui/pregrasp_stability_pixel_radius_px', 45.0))
        self._object_stability_depth_radius_m = float(rospy.get_param('/gui/pregrasp_stability_depth_radius_m', 0.15))
        self._object_stable_count = 0
        self._last_object_base_xyz = None
        self._last_target_signature = None
        self._tf_buffer = None
        self._tf_listener = None
        if tf2_ros is not None:
            try:
                self._tf_buffer = tf2_ros.Buffer()
                self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)
            except Exception as exc:
                rospy.logwarn('PerceptionWidget TF listener unavailable: %s', exc)

        layout = QtWidgets.QHBoxLayout(self)
        layout.setContentsMargins(14, 14, 14, 14)
        layout.setSpacing(14)
        controls = QtWidgets.QVBoxLayout()
        controls.setSpacing(0)
        frame, body = panel('目标识别与抓取位姿')
        controls.addWidget(frame)

        perception_cfg = rospy.get_param('/perception', {})
        self.model_profiles = normalize_model_profiles(perception_cfg)
        model_row = QtWidgets.QHBoxLayout()
        model_row.setSpacing(10)
        model_row.addWidget(QtWidgets.QLabel('检测模型'))
        self.model_combo = QtWidgets.QComboBox()
        current_choice = str(perception_cfg.get('yolo_model_choice', 'original'))
        self._populate_model_choices(self.model_profiles, current_choice)
        confirm_model_btn = QtWidgets.QPushButton('确定模型')
        confirm_model_btn.setObjectName('PrimaryButton')
        confirm_model_btn.clicked.connect(self.confirm_model_selection)
        model_row.addWidget(self.model_combo, 1)
        model_row.addWidget(confirm_model_btn)
        body.addLayout(model_row)
        self.model_status_chip = metric_chip('等待模型状态', accent=True)
        body.addWidget(self.model_status_chip)
        self.mask_status_chip = metric_chip('mask waiting', accent=True)
        body.addWidget(self.mask_status_chip)

        command_row = QtWidgets.QGridLayout()
        command_row.setHorizontalSpacing(10)
        command_row.setVerticalSpacing(8)
        self.description_edit = QtWidgets.QLineEdit(rospy.get_param('/perception/target_description', '绿色目标'))
        self.description_edit.setPlaceholderText('例如：红色圆形物体、蓝色方块、黑色矩形')
        apply_btn = QtWidgets.QPushButton('开始识别')
        apply_btn.setObjectName('PrimaryButton')
        apply_btn.clicked.connect(self.apply_params)
        command_row.addWidget(QtWidgets.QLabel('目标描述'), 0, 0)
        command_row.addWidget(self.description_edit, 0, 1)
        command_row.addWidget(apply_btn, 0, 2)
        body.addLayout(command_row)

        self.interpret_chip = metric_chip('等待识别指令', accent=True)
        body.addWidget(self.interpret_chip)

        advanced = QtWidgets.QGroupBox('高级视觉参数')
        advanced.setCheckable(True)
        advanced.setChecked(False)
        advanced_body = QtWidgets.QVBoxLayout(advanced)
        advanced_body.setContentsMargins(10, 12, 10, 10)
        body.addWidget(advanced)

        param_grid = QtWidgets.QGridLayout()
        param_grid.setHorizontalSpacing(10)
        param_grid.setVerticalSpacing(8)
        self.enabled = QtWidgets.QCheckBox('启用检测')
        self.enabled.setChecked(bool(rospy.get_param('/perception/enabled', True)))
        self.label_edit = QtWidgets.QLineEdit(rospy.get_param('/perception/object_label', 'target'))
        self.yolo_class_edit = QtWidgets.QLineEdit(rospy.get_param('/perception/yolo_target_class', ''))
        self.yolo_class_edit.setPlaceholderText('例如 bottle、cup、sports ball；留空表示检测全部 YOLO 类别')
        initial_selected = select_yolo_model(
            perception_cfg,
            current_choice if current_choice in self.model_profiles else 'original',
            perception_cfg.get('yolo_target_class', ''),
        )
        self._sync_model_class_editor(initial_selected)
        self.lower_edit = QtWidgets.QLineEdit(self._list_text(rospy.get_param('/perception/hsv_lower', [35, 40, 40])))
        self.upper_edit = QtWidgets.QLineEdit(self._list_text(rospy.get_param('/perception/hsv_upper', [85, 255, 255])))
        self.min_area = QtWidgets.QSpinBox()
        self.min_area.setRange(1, 1000000)
        self.min_area.setValue(int(rospy.get_param('/perception/min_area', 300)))
        self.min_area.setSuffix(' px²')
        self.min_area.setToolTip('小于这个像素面积的色块会被当作噪声忽略。画面杂点多时调大，目标小或距离远时调小。')
        self.pregrasp = QtWidgets.QDoubleSpinBox()
        self.pregrasp.setRange(0.0, 0.5)
        self.pregrasp.setDecimals(3)
        self.pregrasp.setSingleStep(0.005)
        self.pregrasp.setValue(float(rospy.get_param('/grasp/pregrasp_distance', 0.08)))
        self.pregrasp.setSuffix(' m')
        self.pregrasp.setToolTip('目标识别后先到达目标上方/前方的安全距离，再继续抓取。')

        rows = [
            ('内部标签', self.label_edit),
            ('YOLO 类别', self.yolo_class_edit),
            ('HSV 下限', self.lower_edit),
            ('HSV 上限', self.upper_edit),
            ('忽略小目标', self.min_area),
            ('预抓取距离', self.pregrasp),
        ]
        param_grid.addWidget(self.enabled, 0, 0, 1, 2)
        for index, (label, widget) in enumerate(rows, start=1):
            param_grid.addWidget(QtWidgets.QLabel(label), index, 0)
            param_grid.addWidget(widget, index, 1)
        advanced_body.addLayout(param_grid)
        advanced.toggled.connect(lambda checked: self._set_layout_visible(advanced_body, checked))
        self._set_layout_visible(advanced_body, False)

        alignment = QtWidgets.QGroupBox('视觉对准控制')
        alignment_body = QtWidgets.QVBoxLayout(alignment)
        alignment_body.setContentsMargins(10, 12, 10, 10)
        alignment_body.setSpacing(8)
        body.addWidget(alignment)

        alignment_options = QtWidgets.QHBoxLayout()
        alignment_options.setSpacing(8)
        alignment_options.addWidget(QtWidgets.QLabel('步长'))
        self.visual_step = QtWidgets.QComboBox()
        self.visual_step.addItem('1 mm', 0.001)
        self.visual_step.addItem('5 mm', 0.005)
        self.visual_step.addItem('10 mm', 0.010)
        self.visual_step.setCurrentIndex(1)
        alignment_options.addWidget(self.visual_step)
        self.visual_execute = QtWidgets.QCheckBox('执行微动')
        self.visual_execute.setChecked(True)
        self.visual_execute.setToolTip('勾选后按钮会真正执行机械臂点动；取消勾选只做 MoveIt 规划。')
        alignment_options.addWidget(self.visual_execute)
        alignment_options.addStretch(1)
        alignment_body.addLayout(alignment_options)

        alignment_grid = QtWidgets.QGridLayout()
        alignment_grid.setSpacing(8)
        for index, axis in enumerate(('X+', 'X-', 'Y+', 'Y-', 'Z+', 'Z-')):
            button = QtWidgets.QPushButton(axis)
            button.setObjectName('AxisButton')
            button.setCursor(QtCore.Qt.PointingHandCursor)
            button.clicked.connect(lambda _, axis=axis: self.visual_jog(axis))
            alignment_grid.addWidget(button, index // 2, index % 2)
        alignment_body.addLayout(alignment_grid)

        metrics = QtWidgets.QGridLayout()
        metrics.setSpacing(8)
        self.detected_chip = metric_chip('未检测', accent=True)
        self.pixel_chip = metric_chip('像素 --')
        self.depth_chip = metric_chip('深度 --')
        self.conf_chip = metric_chip('置信度 --')
        self.camera_chip = metric_chip('相机坐标 --')
        self.base_chip = metric_chip('基座坐标 --')
        self.pregrasp_chip = metric_chip('预抓取 --')
        chips = [
            self.detected_chip, self.pixel_chip, self.depth_chip,
            self.conf_chip, self.camera_chip, self.base_chip,
            self.pregrasp_chip,
        ]
        for index, chip in enumerate(chips):
            metrics.addWidget(chip, index // 2, index % 2)
        body.addLayout(metrics)

        action_mode = perception_grasp_action_mode(self._use_grasp6d_plan)
        self.grasp_mode_note = QtWidgets.QLabel(action_mode['note'])
        self.grasp_mode_note.setObjectName('StateBanner')
        self.grasp_mode_note.setWordWrap(True)
        body.addWidget(self.grasp_mode_note)

        actions = QtWidgets.QHBoxLayout()
        self.plan_pregrasp_btn = QtWidgets.QPushButton('规划预抓取')
        self.execute_pregrasp_btn = QtWidgets.QPushButton('执行已规划预抓取')
        self.start_grasp_btn = QtWidgets.QPushButton('执行抓取流程')
        self.plan_pregrasp_btn.setObjectName('PrimaryButton')
        self.execute_pregrasp_btn.setObjectName('DangerButton')
        self.start_grasp_btn.setObjectName('DangerButton')
        self.execute_pregrasp_btn.setEnabled(False)
        self.execute_pregrasp_btn.setToolTip('先规划预抓取；规划成功后执行这一次已规划的轨迹。')
        self.start_grasp_btn.setToolTip('执行完整流程：预抓取、接近目标、柔顺闭合、抬升。')
        self.plan_pregrasp_btn.clicked.connect(lambda: self.plan_pregrasp(False))
        self.execute_pregrasp_btn.clicked.connect(lambda: self.plan_pregrasp(True))
        self.start_grasp_btn.clicked.connect(self.start_grasp_flow)
        actions.addWidget(self.plan_pregrasp_btn)
        actions.addWidget(self.execute_pregrasp_btn)
        actions.addWidget(self.start_grasp_btn)
        body.addLayout(actions)
        if not action_mode['show_legacy_pregrasp']:
            self.plan_pregrasp_btn.hide()
            self.execute_pregrasp_btn.hide()
            self.start_grasp_btn.hide()

        self.status = QtWidgets.QLabel('等待目标识别数据')
        self.status.setObjectName('StateBanner')
        self.status.setWordWrap(True)
        body.addWidget(self.status)
        controls.addStretch(1)
        layout.addLayout(controls, 4)
        if color_topic and depth_topic:
            self.camera_preview = CameraWidget(color_topic, depth_topic, compact=True, default_mode='split')
            self.camera_preview.color_frame_updated.connect(self._on_camera_color_frame)
            layout.addWidget(self.camera_preview, 3)

        self.object_signal.connect(self.update_object)
        self.mask_signal.connect(self.update_mask)
        self.detector_status_signal.connect(self._update_detector_status)
        self.plan_result_signal.connect(self._finish_pregrasp_worker)
        self.grasp_result_signal.connect(self._finish_grasp_worker)
        self._subscriber = rospy.Subscriber(topic, ObjectPose, self._emit_if_alive, queue_size=1)
        self._detector_status_subscriber = rospy.Subscriber(
            '/perception/detector_status',
            String,
            self._emit_detector_status_if_alive,
            queue_size=1,
        )
        self._mask_subscriber = rospy.Subscriber(
            '/perception/object_mask',
            Image,
            self._emit_mask_if_alive,
            queue_size=1,
        )
        self.destroyed.connect(lambda *_: self._shutdown_ros())

    def _emit_if_alive(self, msg):
        if not self.__dict__.get('_alive', False):
            return
        try:
            self.object_signal.emit(msg)
        except RuntimeError:
            self._shutdown_ros()

    def _emit_detector_status_if_alive(self, msg):
        if not self.__dict__.get('_alive', False):
            return
        try:
            self.detector_status_signal.emit(str(getattr(msg, 'data', msg)))
        except RuntimeError:
            self._shutdown_ros()

    def _emit_mask_if_alive(self, msg):
        if not self.__dict__.get('_alive', False):
            return
        try:
            self.mask_signal.emit(msg)
        except RuntimeError:
            self._shutdown_ros()

    def _on_camera_color_frame(self, _rgb):
        if not self.__dict__.get('_alive', False):
            return
        self._refresh_detection_overlay()

    def update_mask(self, msg):
        if not self.__dict__.get('_alive', False):
            return
        try:
            mask = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
            binary = np.where(np.asarray(mask) > 0, 255, 0).astype(np.uint8)
            if binary.size == 0 or not np.any(binary):
                self._latest_mask = None
                self._latest_mask_stamp = None
                self._set_mask_status('mask empty')
                self._refresh_detection_overlay()
                return
            declared_shape = (int(getattr(msg, 'height', 0)), int(getattr(msg, 'width', 0)))
            camera_preview = self.__dict__.get('camera_preview', None)
            camera_rgb = getattr(camera_preview, '_last_color_rgb', None) if camera_preview is not None else None
            camera_shape = tuple(camera_rgb.shape[:2]) if camera_rgb is not None else None
            if (
                binary.ndim != 2
                or (all(declared_shape) and tuple(binary.shape) != declared_shape)
                or (camera_shape is not None and tuple(binary.shape) != camera_shape)
            ):
                self._latest_mask = None
                self._latest_mask_stamp = None
                self._set_mask_status('mask size mismatch')
                self._refresh_detection_overlay()
                return
            self._latest_mask = binary
            self._latest_mask_stamp = self._stamp_key(getattr(msg, 'header', None))
            self._refresh_detection_overlay()
        except Exception as exc:
            self._latest_mask = None
            self._latest_mask_stamp = None
            self._set_mask_status('mask error')
            self._refresh_detection_overlay()
            rospy.logwarn_throttle(2.0, 'PerceptionWidget mask convert failed: %s', exc)

    @staticmethod
    def _stamp_key(header):
        stamp = getattr(header, 'stamp', None)
        if stamp is None:
            return None
        try:
            return int(stamp.to_nsec())
        except Exception:
            return (int(getattr(stamp, 'secs', 0)), int(getattr(stamp, 'nsecs', 0)))

    def _refresh_detection_overlay(self):
        camera_preview = self.__dict__.get('camera_preview', None)
        msg = self.__dict__.get('last_object', None)
        if camera_preview is None:
            return
        if msg is None or not getattr(msg, 'detected', False):
            camera_preview.set_detection_overlay(None)
            if self.__dict__.get('_latest_mask', None) is not None:
                self._set_mask_status('mask stale')
            return
        bbox = self._object_bbox(msg)
        if bbox is None:
            camera_preview.set_detection_overlay(None)
            return
        label = msg.label or self.label_edit.text()
        camera_preview.set_detection_overlay(
            bbox,
            label,
            self._overlay_color_rgb(label or self.description_edit.text()),
            self._matching_mask_contour(msg),
        )

    def _matching_mask_contour(self, msg):
        mask = self.__dict__.get('_latest_mask', None)
        mask_stamp = self.__dict__.get('_latest_mask_stamp', None)
        current_stamp = self.__dict__.get('_current_object_stamp', None)
        current_detected = bool(self.__dict__.get('_current_object_detected', False))
        if mask is None:
            if self.__dict__.get('_mask_status', '') not in ('mask empty', 'mask size mismatch', 'mask error'):
                self._set_mask_status('mask stale')
            return None
        camera_preview = self.__dict__.get('camera_preview', None)
        camera_rgb = getattr(camera_preview, '_last_color_rgb', None) if camera_preview is not None else None
        if camera_rgb is not None and tuple(mask.shape) != tuple(camera_rgb.shape[:2]):
            self._latest_mask = None
            self._latest_mask_stamp = None
            self._set_mask_status('mask size mismatch')
            return None
        if not current_detected or mask_stamp is None or mask_stamp != current_stamp:
            self._set_mask_status('mask stale')
            return None
        contours, _ = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            self._set_mask_status('mask empty')
            return None
        self._set_mask_status('mask ready')
        return max(contours, key=cv2.contourArea).reshape(-1, 2)

    def _set_mask_status(self, text):
        self._mask_status = str(text)
        chip = self.__dict__.get('mask_status_chip', None)
        if chip is not None:
            chip.setText(self._mask_status)

    def _clear_mask_state(self):
        self._current_object_stamp = None
        self._current_object_detected = False
        self._latest_mask = None
        self._latest_mask_stamp = None
        self._set_mask_status('mask waiting')

    def _invalidate_current_mask(self):
        self._latest_mask = None
        self._latest_mask_stamp = None
        self._set_mask_status('mask stale')

    def _emit_plan_result_if_alive(self, token, execute, success, message):
        if not self.__dict__.get('_alive', False):
            return
        try:
            self.plan_result_signal.emit(int(token), bool(execute), bool(success), str(message))
        except RuntimeError:
            self._shutdown_ros()

    def _shutdown_ros(self):
        self._alive = False
        self._planning_active = False
        self._plan_token += 1
        for attribute in ('_subscriber', '_detector_status_subscriber', '_mask_subscriber'):
            subscriber = self.__dict__.get(attribute, None)
            flag = attribute + '_unregistered'
            if subscriber is not None and not self.__dict__.get(flag, False):
                try:
                    subscriber.unregister()
                except Exception:
                    pass
                self.__dict__[flag] = True

    def closeEvent(self, event):
        self._shutdown_ros()
        super().closeEvent(event)

    def _populate_model_choices(self, profiles, current_choice):
        for choice, profile in profiles.items():
            self.model_combo.addItem(profile['display_name'], choice)
        current_index = self.model_combo.findData(str(current_choice))
        self.model_combo.setCurrentIndex(current_index if current_index >= 0 else 0)

    def _sync_model_class_editor(self, selected):
        self.yolo_class_edit.setText(selected['target_class'])
        self.yolo_class_edit.setEnabled(selected['target_class_mode'] != 'fixed')

    def confirm_model_selection(self):
        try:
            description = self.description_edit.text().strip() or '目标物体'
            parsed = self._parse_description(description)
            perception_cfg = rospy.get_param('/perception', {})
            choice = str(self.model_combo.currentData() or 'original')
            selected = select_yolo_model(
                perception_cfg,
                choice,
                parsed.get('yolo_target_class', ''),
            )
            resolve_yolo_model_path(selected['model_path'])
            perception_cfg.update({
                'yolo_model_choice': selected['choice'],
                'yolo_model': selected['model_path'],
                'yolo_target_class': selected['target_class'],
                'yolo_reload_generation': int(perception_cfg.get('yolo_reload_generation', 0)) + 1,
            })
            rospy.set_param('/perception', perception_cfg)
            self._sync_model_class_editor(selected)
            self._invalidate_actionable_target()
            self.model_status_chip.setText('%s正在加载' % selected['display_name'])
            self._set_perception_status('模型选择已提交，视觉节点正在刷新')
        except Exception as exc:
            self._set_perception_status('模型切换失败：%s' % exc)

    def _update_detector_status(self, text):
        raw_text = str(text or '')
        parts = raw_text.split(':', 2)
        state = parts[0] if parts else ''
        choice = parts[1] if len(parts) > 1 else ''
        detail = parts[2] if len(parts) > 2 else ''
        if state not in ('loading', 'ready', 'error') or not choice:
            self.model_status_chip.setText('检测模型状态未知')
            self._set_perception_status('无法解析检测模型状态：%s' % (raw_text or '空消息'))
            return
        profile = self.model_profiles.get(choice, {})
        display_name = str(profile.get('display_name', choice or '检测模型'))
        if state == 'loading':
            self._invalidate_actionable_target()
            self.model_status_chip.setText('%s正在加载' % display_name)
        elif state == 'ready':
            self.model_status_chip.setText('%s已就绪' % display_name)
        elif state == 'error':
            self._invalidate_actionable_target()
            self.model_status_chip.setText('%s加载失败' % display_name)
            self._set_perception_status('模型加载失败：%s' % detail)

    def _invalidate_actionable_target(self):
        self.last_object = None
        self.pregrasp_pose = None
        self._last_object_receive_time = None
        self._planning_active = False
        self._plan_token = int(self.__dict__.get('_plan_token', 0)) + 1
        self._pending_plan_pose = None
        self._pending_plan_token = None
        self._clear_planned_pregrasp()
        self._clear_locked_grasp_target()
        self._reset_target_stability()
        self._localization_error_m = None
        self._clear_mask_state()
        camera_preview = self.__dict__.get('camera_preview', None)
        if camera_preview is not None:
            camera_preview.set_detection_overlay(None)

    def apply_params(self):
        try:
            description = self.description_edit.text().strip() or '目标物体'
            parsed = self._parse_description(description)
            lower = self._parse_hsv(self.lower_edit.text())
            upper = self._parse_hsv(self.upper_edit.text())
            hsv_ranges = parsed['hsv_ranges']
            if hsv_ranges:
                lower, upper = hsv_ranges[0]
                self.lower_edit.setText(self._list_text(lower))
                self.upper_edit.setText(self._list_text(upper))
            perception_cfg = rospy.get_param('/perception', {})
            active_choice = str(perception_cfg.get('yolo_model_choice', 'original'))
            selected = select_yolo_model(
                perception_cfg,
                active_choice,
                parsed.get('yolo_target_class', ''),
            )
            yolo_target_class = selected['target_class']
            self._sync_model_class_editor(selected)
            label = description
            self.label_edit.setText(label)
            perception_cfg.update({
                'enabled': bool(self.enabled.isChecked()),
                'object_label': label,
                'target_description': description,
                'yolo_target_class': yolo_target_class,
                'hsv_lower': lower,
                'hsv_upper': upper,
                'hsv_ranges': hsv_ranges,
                'shape': parsed['shape'],
                'min_area': int(self.min_area.value()),
            })
            rospy.set_param('/perception', perception_cfg)
            rospy.set_param('/grasp/pregrasp_distance', float(self.pregrasp.value()))
            self._clear_locked_grasp_target()
            self.interpret_chip.setText(parsed['summary'])
            self._set_perception_status('识别指令已更新，视觉节点会自动刷新')
        except Exception as exc:
            self._set_perception_status('参数格式错误：%s' % exc)

    def update_object(self, msg):
        if not self.__dict__.get('_alive', False):
            return
        self._current_object_stamp = self._stamp_key(getattr(msg, 'header', None))
        self._current_object_detected = bool(getattr(msg, 'detected', False))
        self._last_object_receive_time = time.monotonic()
        if (
            not getattr(msg, 'detected', False)
            and self.__dict__.get('_grasp_active', False)
            and self._has_recent_locked_grasp_target()
        ):
            self._invalidate_current_mask()
            self._refresh_detection_overlay()
            self.detected_chip.setText('目标已锁定 %s' % (msg.label or self.label_edit.text()))
            self._set_perception_status('目标已锁定，运动中临时丢帧不打断抓取流程')
            return
        self.last_object = msg
        if not msg.detected:
            self._invalidate_current_mask()
            self.pregrasp_pose = None
            self._reset_target_stability()
            self._localization_error_m = None
            self.detected_chip.setText('未检测到 %s' % (msg.label or self.label_edit.text()))
            if self._has_recent_locked_grasp_target():
                self._set_perception_status('当前画面暂时丢失目标；已保留锁定目标，可继续执行抓取流程')
            elif self.__dict__.get('_planned_pregrasp_pose', None) is not None:
                self._set_perception_status('当前画面暂时丢失目标；已保留已规划预抓取轨迹，可在有效期内执行')
            else:
                self._set_perception_status('未识别到目标，调整 HSV 阈值或移动目标到视野内')
            self._refresh_detection_overlay()
            return

        cam = msg.pose_camera.pose.position
        base = msg.pose_base.pose.position
        camera_pose_base = self._lookup_camera_pose_base()
        self.pregrasp_pose = make_pregrasp_pose(
            msg.pose_base,
            float(self.pregrasp.value()),
            camera_pose=camera_pose_base,
            mode=self._pregrasp_mode,
        )
        pre = self.pregrasp_pose.pose.position
        self._update_localization_health(msg, camera_pose_base)
        self._update_target_stability(msg)
        planned_invalidated = self._invalidate_planned_pregrasp_if_target_moved(msg)

        self.detected_chip.setText('已检测 %s' % msg.label)
        self.pixel_chip.setText('像素 u=%d v=%d' % (msg.u, msg.v))
        self.depth_chip.setText('深度 %.3f m' % msg.depth_m)
        self.conf_chip.setText('置信度 %.3f' % msg.confidence)
        self.camera_chip.setText('相机 xyz %.3f %.3f %.3f' % (cam.x, cam.y, cam.z))
        self.base_chip.setText('基座 xyz %.3f %.3f %.3f' % (base.x, base.y, base.z))
        self.pregrasp_chip.setText('预抓取 %s %.3f %.3f %.3f' % (self._pregrasp_mode_text(camera_pose_base), pre.x, pre.y, pre.z))
        if planned_invalidated:
            self._set_perception_status('目标位置变化，已取消旧的预抓取规划，请重新规划')
        elif self._localization_ok:
            if self._pregrasp_target_is_stable():
                self._set_perception_status('目标识别稳定，已更新目标坐标和预抓取位姿')
            else:
                self._set_perception_status(
                    '目标识别成功，等待坐标稳定 %s' % self._target_stability_progress_text()
                )
        else:
            self._set_perception_status(
                '目标定位不可信：相机坐标距离与基座坐标距离偏差 %.3f m，请停稳后重新识别或检查手眼标定'
                % float(self._localization_error_m or 0.0)
            )
        if self.__dict__.get('camera_preview', None) is not None:
            self._refresh_detection_overlay()

    def plan_pregrasp(self, execute):
        pose = self._pregrasp_pose_for_request(execute)
        rospy.loginfo('GUI pregrasp button clicked execute=%s', bool(execute))
        if pose is None:
            if execute:
                self.status.setText('请先点击规划预抓取，规划成功后再执行预抓取')
                rospy.logwarn('GUI pregrasp execute blocked: no successful planned pregrasp pose')
            else:
                self.status.setText('没有可用预抓取位姿')
                rospy.logwarn('GUI pregrasp plan blocked: no pregrasp pose')
            return
        if execute and not self.__dict__.get('_planned_pregrasp_executable', False):
            self.status.setText('当前只有“仅位置预览”或没有可执行姿态轨迹，已禁止执行预抓取；请重新规划到姿态可达的目标')
            rospy.logwarn('GUI pregrasp execute blocked: no executable planned pose')
            return
        if execute and self._planned_pregrasp_is_stale():
            self._clear_planned_pregrasp()
            self.status.setText('已规划预抓取轨迹过期，已取消执行；请重新识别并规划')
            rospy.logwarn('GUI pregrasp execute blocked: planned pose is stale')
            return
        if execute and not getattr(self, '_localization_ok', True):
            self.status.setText('定位不可信，已禁止直接执行预抓取；请重新识别或检查手眼标定，可先只做规划验证')
            rospy.logwarn('GUI pregrasp execute blocked: localization is not trusted')
            return
        if not execute and self._pregrasp_pose_is_stale():
            self.status.setText('目标坐标已过期，已禁止规划预抓取；请等待视觉刷新或重新识别')
            rospy.logwarn('GUI pregrasp plan blocked: live pose is stale')
            return
        if not execute and not self._pregrasp_target_is_stable():
            self.status.setText(
                '目标识别还不稳定，已禁止规划预抓取；当前稳定度 %s，请保持目标在视野内或稍微调整视角'
                % self._target_stability_progress_text()
            )
            rospy.logwarn(
                'GUI pregrasp plan blocked: target not stable count=%s required=%s',
                self.__dict__.get('_object_stable_count', 0),
                self.__dict__.get('_required_stable_detections', 3),
            )
            return
        if (
            self.__dict__.get('_pregrasp_worker_token', None) is not None
            or getattr(self, '_planning_active', False)
        ):
            self.status.setText('已有预抓取规划正在后台执行，请等待结果')
            rospy.logwarn('GUI pregrasp request ignored: previous request still active')
            return
        self._planning_active = True
        self._plan_token += 1
        token = self._plan_token
        self._pregrasp_worker_token = token
        if execute:
            self._pending_plan_pose = None
            self._pending_plan_token = None
        else:
            self._pending_plan_pose = deepcopy(pose)
            self._pending_plan_token = token
        rospy.loginfo(
            'GUI pregrasp worker dispatch execute=%s token=%s %s',
            bool(execute),
            token,
            self._pose_xyz_text(pose),
        )
        self._set_pregrasp_buttons_enabled(False)
        if execute:
            self.status.setText('后台执行已规划预抓取中，预抓取操作按钮已锁定；超时后继续锁定，直到后台请求结束')
        else:
            self.status.setText('后台规划中，预抓取操作按钮已锁定；超时后继续锁定，直到后台请求结束')
        self._start_pregrasp_worker(deepcopy(pose), execute, token)
        try:
            timeout_sec = float(self._plan_timeout_sec)
        except Exception:
            timeout_sec = 12.0
        QtCore.QTimer.singleShot(int(max(1.0, timeout_sec) * 1000), lambda token=token: self._timeout_pregrasp_worker(token))

    def _start_pregrasp_worker(self, pose, execute, token):
        thread = threading.Thread(
            target=self._run_pregrasp_request,
            args=(pose, execute, token),
            daemon=True,
        )
        thread.start()

    def _run_pregrasp_request(self, pose, execute, token):
        success = False
        try:
            rospy.loginfo(
                'GUI pregrasp worker started execute=%s token=%s %s',
                bool(execute),
                token,
                self._pose_xyz_text(pose),
            )
            rospy.wait_for_service('/supervisor/move_to_pose', timeout=1.0)
            rospy.loginfo('GUI pregrasp worker service ready execute=%s token=%s', bool(execute), token)
            srv = rospy.ServiceProxy('/supervisor/move_to_pose', SetTargetPose)
            res = srv(pose, execute)
            success = bool(res.success)
            message = self._pregrasp_status_text(execute, success, res.message)
            rospy.loginfo(
                'GUI pregrasp worker service result execute=%s token=%s success=%s message=%s',
                bool(execute),
                token,
                success,
                res.message,
            )
        except Exception as exc:
            message = '预抓取请求失败：%s' % exc
            rospy.logwarn(
                'GUI pregrasp worker service failed execute=%s token=%s error=%s',
                bool(execute),
                token,
                exc,
            )
        self._emit_plan_result_if_alive(token, execute, success, message)

    def _finish_pregrasp_worker(self, token, execute, success, message):
        if not self.__dict__.get('_alive', False):
            return
        worker_token = self.__dict__.get('_pregrasp_worker_token', None)
        if worker_token is None and self.__dict__.get('_planning_active', False):
            worker_token = self._plan_token
        if token != worker_token:
            return
        self._pregrasp_worker_token = None
        if token != self._plan_token or not self._planning_active:
            self._set_pregrasp_buttons_enabled(True)
            return
        self._planning_active = False
        if execute:
            if success:
                self._lock_grasp_target_from_current_plan()
                self._clear_planned_pregrasp()
        elif success:
            if self._is_position_only_plan_message(message):
                self._clear_planned_pregrasp()
                message = (
                    '规划仅位置预览成功：%s；未生成可执行姿态轨迹，已禁止执行预抓取，'
                    '请重新对准目标或检查手眼标定/末端姿态'
                ) % message
            else:
                self._planned_pregrasp_pose = deepcopy(self.__dict__.get('_pending_plan_pose', None))
                self._planned_pregrasp_executable = self._planned_pregrasp_pose is not None
                self._planned_pregrasp_time = time.monotonic() if self._planned_pregrasp_executable else 0.0
                self._planned_target_base_xyz = self.__dict__.get('_last_object_base_xyz', None)
        else:
            self._clear_planned_pregrasp()
        self._pending_plan_pose = None
        self._pending_plan_token = None
        self._set_pregrasp_buttons_enabled(True)
        if not execute and success and self.__dict__.get('_planned_pregrasp_executable', False):
            message = '%s；现在可以点击“执行已规划预抓取”' % message
        if execute and success:
            message = '%s；已到达预抓取点，可点击“执行抓取流程”继续接近并闭合夹爪' % message
        self._set_status(message, hold_sec=float(self.__dict__.get('_pregrasp_status_hold_sec', 8.0)))

    def _timeout_pregrasp_worker(self, token):
        if not self.__dict__.get('_alive', False):
            return
        worker_token = self.__dict__.get('_pregrasp_worker_token', None)
        if worker_token is None and self.__dict__.get('_planning_active', False):
            worker_token = self._plan_token
        if token != worker_token:
            return
        if token != self._plan_token or not self._planning_active:
            return
        self._planning_active = False
        self._plan_token += 1
        rospy.logwarn('GUI pregrasp worker timeout token=%s', token)
        self._set_status(
            '预抓取请求等待超时，后台请求仍未结束；已禁止新的规划或执行，请检查 MoveIt、关节反馈和目标位姿是否可达',
            hold_sec=float(self.__dict__.get('_pregrasp_status_hold_sec', 8.0)),
        )

    def start_grasp_flow(self):
        rospy.loginfo('GUI grasp flow button clicked')
        if getattr(self, '_grasp_active', False):
            self.status.setText('抓取流程正在后台执行，请等待结果')
            rospy.logwarn('GUI grasp flow ignored: previous request still active')
            return
        last_object = self.__dict__.get('last_object', None)
        has_live_object = last_object is not None and getattr(last_object, 'detected', False)
        has_locked_target = self._has_recent_locked_grasp_target()
        if not has_live_object and not has_locked_target:
            self.status.setText('没有可用目标，无法执行抓取流程')
            rospy.logwarn('GUI grasp flow blocked: no detected object')
            return
        if has_live_object and not has_locked_target and not self._pregrasp_target_is_stable():
            self.status.setText(
                '目标识别还不稳定，已禁止执行抓取流程；当前稳定度 %s'
                % self._target_stability_progress_text()
            )
            rospy.logwarn('GUI grasp flow blocked: target not stable')
            return
        if has_live_object and not has_locked_target and not getattr(self, '_localization_ok', True):
            self.status.setText('定位不可信，已禁止执行抓取流程；请重新识别或检查手眼标定')
            rospy.logwarn('GUI grasp flow blocked: localization is not trusted')
            return
        self._grasp_active = True
        start_grasp_btn = self.__dict__.get('start_grasp_btn', None)
        if start_grasp_btn is not None:
            start_grasp_btn.setEnabled(False)
        self._set_pregrasp_buttons_enabled(False)
        if has_locked_target:
            self.status.setText('后台执行抓取流程中：使用锁定目标，接近目标、柔顺闭合、抬升')
        else:
            self.status.setText('后台执行抓取流程中：预抓取、接近目标、柔顺闭合、抬升')
        self._start_grasp_flow_worker()

    def _start_grasp_flow_worker(self):
        thread = threading.Thread(target=self._run_grasp_flow_request, daemon=True)
        thread.start()

    def _run_grasp_flow_request(self):
        success = False
        try:
            rospy.wait_for_service('/grasp/start', timeout=1.0)
            srv = rospy.ServiceProxy('/grasp/start', StartGrasp)
            res = srv(True)
            success = bool(res.success)
            message = ('抓取流程成功：' if success else '抓取流程失败：') + str(res.message)
            rospy.loginfo('GUI grasp flow result success=%s message=%s', success, res.message)
        except Exception as exc:
            message = '抓取流程请求失败：%s' % exc
            rospy.logwarn('GUI grasp flow failed: %s', exc)
        self._emit_grasp_result_if_alive(success, message)

    def _emit_grasp_result_if_alive(self, success, message):
        if not self.__dict__.get('_alive', False):
            return
        try:
            self.grasp_result_signal.emit(bool(success), str(message))
        except RuntimeError:
            self._shutdown_ros()

    def _finish_grasp_worker(self, success, message):
        if not self.__dict__.get('_alive', False):
            return
        self._grasp_active = False
        start_grasp_btn = self.__dict__.get('start_grasp_btn', None)
        if start_grasp_btn is not None:
            start_grasp_btn.setEnabled(True)
        self._set_pregrasp_buttons_enabled(True)
        self._set_status(message, hold_sec=float(self.__dict__.get('_pregrasp_status_hold_sec', 8.0)))

    def _set_pregrasp_buttons_enabled(self, enabled):
        if hasattr(self, 'plan_pregrasp_btn'):
            self.plan_pregrasp_btn.setEnabled(bool(enabled))
        start_grasp_btn = self.__dict__.get('start_grasp_btn', None)
        if start_grasp_btn is not None:
            start_grasp_btn.setEnabled(bool(enabled) and not bool(getattr(self, '_grasp_active', False)))
        if hasattr(self, 'execute_pregrasp_btn'):
            can_execute = (
                self.__dict__.get('_planned_pregrasp_pose', None) is not None
                and self.__dict__.get('_planned_pregrasp_executable', False)
                and not self._planned_pregrasp_is_stale()
            )
            self.execute_pregrasp_btn.setEnabled(bool(enabled) and can_execute)

    def _pregrasp_pose_for_request(self, execute):
        if execute:
            return self.__dict__.get('_planned_pregrasp_pose', None)
        return self.__dict__.get('pregrasp_pose', None)

    @staticmethod
    def _pose_xyz_text(pose):
        try:
            p = pose.pose.position if hasattr(pose, 'pose') else pose.position
            return 'xyz=(%.3f, %.3f, %.3f)' % (float(p.x), float(p.y), float(p.z))
        except Exception:
            return 'xyz=(unavailable)'

    def _clear_planned_pregrasp(self):
        self._planned_pregrasp_pose = None
        self._planned_pregrasp_executable = False
        self._planned_pregrasp_time = 0.0
        self._planned_target_base_xyz = None
        if hasattr(self, 'execute_pregrasp_btn'):
            self.execute_pregrasp_btn.setEnabled(False)

    def _set_status(self, text, hold_sec=0.0):
        if hasattr(self, 'status'):
            self.status.setText(text)
        if hold_sec > 0.0:
            self._status_hold_until = time.monotonic() + hold_sec

    def _set_perception_status(self, text):
        if (
            self.__dict__.get('_pregrasp_worker_token', None) is not None
            or self.__dict__.get('_planning_active', False)
        ):
            return
        if float(self.__dict__.get('_status_hold_until', 0.0) or 0.0) > time.monotonic():
            return
        self._set_status(text)

    def _lookup_camera_pose_base(self):
        if self.__dict__.get('_tf_buffer', None) is None:
            return None
        try:
            transform = self._tf_buffer.lookup_transform(
                self.__dict__.get('_base_frame', 'base_link'),
                self.__dict__.get('_camera_frame', 'camera_link'),
                rospy.Time(0),
                rospy.Duration(0.05),
            )
        except Exception as exc:
            rospy.logwarn_throttle(2.0, 'GUI camera TF lookup failed: %s', exc)
            return None
        pose = PoseStamped()
        pose.header = transform.header
        pose.pose.position.x = transform.transform.translation.x
        pose.pose.position.y = transform.transform.translation.y
        pose.pose.position.z = transform.transform.translation.z
        pose.pose.orientation = transform.transform.rotation
        return pose

    def _update_localization_health(self, msg, camera_pose_base):
        if camera_pose_base is None:
            self._localization_ok = True
            self._localization_error_m = None
            return
        error_m = self._localization_error_meters(msg, camera_pose_base)
        self._localization_error_m = error_m
        self._localization_ok = error_m is None or error_m <= float(self.__dict__.get('_localization_warn_error_m', 0.08))

    def _reset_target_stability(self):
        self._object_stable_count = 0
        self._last_object_base_xyz = None
        self._last_target_signature = None

    def _update_target_stability(self, msg):
        xyz = self._object_base_xyz(msg)
        if xyz is None or self._pregrasp_pose_is_stale():
            self._reset_target_stability()
            return
        signature = self._target_signature(msg)
        previous = self.__dict__.get('_last_object_base_xyz', None)
        previous_signature = self.__dict__.get('_last_target_signature', None)
        if previous_signature is not None and signature is not None:
            stable = self._target_signature_matches(previous_signature, signature)
        elif previous is not None:
            stable = self._xyz_distance(previous, xyz) <= float(self.__dict__.get('_object_stability_radius_m', 0.08))
        else:
            stable = False
        if stable:
            self._object_stable_count = int(self.__dict__.get('_object_stable_count', 0)) + 1
        else:
            self._object_stable_count = 1
        self._last_object_base_xyz = xyz
        self._last_target_signature = signature

    def _pregrasp_target_is_stable(self):
        if '_required_stable_detections' not in self.__dict__:
            return True
        required = max(1, int(self.__dict__.get('_required_stable_detections', 3)))
        return int(self.__dict__.get('_object_stable_count', 0)) >= required

    def _target_stability_progress_text(self):
        return '%d/%d，像素容差 %.0f px，深度容差 %.2f m' % (
            int(self.__dict__.get('_object_stable_count', 0)),
            int(self.__dict__.get('_required_stable_detections', 3)),
            float(self.__dict__.get('_object_stability_pixel_radius_px', 45.0)),
            float(self.__dict__.get('_object_stability_depth_radius_m', 0.15)),
        )

    def _target_signature(self, msg):
        try:
            width = int(getattr(msg, 'bbox_width', 0))
            height = int(getattr(msg, 'bbox_height', 0))
            if width > 0 and height > 0:
                u = float(getattr(msg, 'bbox_x', 0)) + width * 0.5
                v = float(getattr(msg, 'bbox_y', 0)) + height * 0.5
            else:
                u = float(getattr(msg, 'u', 0))
                v = float(getattr(msg, 'v', 0))
            depth = float(getattr(msg, 'depth_m', 0.0))
            if depth <= 0.0 or math.isnan(depth) or math.isinf(depth):
                depth = None
            return {
                'label': str(getattr(msg, 'label', '') or ''),
                'u': u,
                'v': v,
                'depth': depth,
            }
        except Exception:
            return None

    def _target_signature_matches(self, previous, current):
        prev_label = str(previous.get('label', '') or '')
        cur_label = str(current.get('label', '') or '')
        if prev_label and cur_label and prev_label != cur_label:
            return False
        pixel_radius = float(self.__dict__.get('_object_stability_pixel_radius_px', 45.0))
        du = float(previous.get('u', 0.0)) - float(current.get('u', 0.0))
        dv = float(previous.get('v', 0.0)) - float(current.get('v', 0.0))
        if math.sqrt(du * du + dv * dv) > pixel_radius:
            return False
        prev_depth = previous.get('depth', None)
        cur_depth = current.get('depth', None)
        if prev_depth is not None and cur_depth is not None:
            depth_radius = float(self.__dict__.get('_object_stability_depth_radius_m', 0.15))
            if abs(float(prev_depth) - float(cur_depth)) > depth_radius:
                return False
        return True

    def _invalidate_planned_pregrasp_if_target_moved(self, msg):
        planned = self.__dict__.get('_planned_target_base_xyz', None)
        if planned is None or self.__dict__.get('_planned_pregrasp_pose', None) is None:
            return False
        current = self._object_base_xyz(msg)
        if current is None:
            return False
        threshold = max(0.02, float(self.__dict__.get('_object_stability_radius_m', 0.03)) * 1.5)
        if self._xyz_distance(planned, current) <= threshold:
            return False
        self._clear_planned_pregrasp()
        self._clear_locked_grasp_target()
        rospy.logwarn(
            'GUI pregrasp planned pose invalidated: target moved %.3f m',
            self._xyz_distance(planned, current),
        )
        return True

    def _lock_grasp_target_from_current_plan(self):
        xyz = self.__dict__.get('_planned_target_base_xyz', None)
        if xyz is None:
            last_object = self.__dict__.get('last_object', None)
            if last_object is not None and getattr(last_object, 'detected', False):
                xyz = self._object_base_xyz(last_object)
        if xyz is None:
            return False
        self._locked_grasp_target_base_xyz = tuple(float(value) for value in xyz)
        self._locked_grasp_target_time = time.monotonic()
        return True

    def _clear_locked_grasp_target(self):
        self._locked_grasp_target_base_xyz = None
        self._locked_grasp_target_time = 0.0

    def _has_recent_locked_grasp_target(self):
        if self.__dict__.get('_locked_grasp_target_base_xyz', None) is None:
            return False
        try:
            max_age = max(0.0, float(self.__dict__.get('_grasp_flow_lock_max_age_sec', 60.0)))
            locked_time = float(self.__dict__.get('_locked_grasp_target_time', 0.0) or 0.0)
        except Exception:
            return False
        if max_age <= 0.0:
            return True
        if locked_time <= 0.0:
            return False
        if time.monotonic() - locked_time <= max_age:
            return True
        self._clear_locked_grasp_target()
        return False

    @staticmethod
    def _object_base_xyz(msg):
        try:
            base = msg.pose_base.pose.position
            return (float(base.x), float(base.y), float(base.z))
        except Exception:
            return None

    @staticmethod
    def _xyz_distance(first, second):
        try:
            return math.sqrt(
                (float(first[0]) - float(second[0])) ** 2
                + (float(first[1]) - float(second[1])) ** 2
                + (float(first[2]) - float(second[2])) ** 2
            )
        except Exception:
            return float('inf')

    def _pregrasp_pose_is_stale(self):
        pose = self.__dict__.get('pregrasp_pose', None)
        max_age = max(0.0, float(self.__dict__.get('_max_object_age_sec', 1.5)))
        if max_age <= 0.0:
            return False
        receive_time = self.__dict__.get('_last_object_receive_time', None)
        if receive_time is not None:
            try:
                return time.monotonic() - float(receive_time) > max_age
            except Exception:
                pass
        stamp = getattr(getattr(pose, 'header', None), 'stamp', None)
        if stamp is None:
            return False
        try:
            stamp_sec = float(stamp.to_sec())
        except Exception:
            return False
        if stamp_sec <= 0.0:
            return False
        try:
            return (rospy.Time.now() - stamp).to_sec() > max_age
        except Exception:
            return False

    def _planned_pregrasp_is_stale(self):
        if self.__dict__.get('_planned_pregrasp_pose', None) is None:
            return False
        try:
            max_age = max(0.0, float(self.__dict__.get('_planned_pregrasp_max_age_sec', 30.0)))
            created = float(self.__dict__.get('_planned_pregrasp_time', 0.0) or 0.0)
        except Exception:
            return False
        if max_age <= 0.0 or created <= 0.0:
            return False
        return time.monotonic() - created > max_age

    def _pregrasp_mode_text(self, camera_pose_base):
        if str(self.__dict__.get('_pregrasp_mode', '')).lower() in ('camera_ray', 'line_of_sight') and camera_pose_base is not None:
            return '视线回退 xyz'
        return '基座Z xyz'

    @staticmethod
    def _localization_error_meters(msg, camera_pose_base):
        try:
            cam = msg.pose_camera.pose.position
            base = msg.pose_base.pose.position
            camera = camera_pose_base.pose.position
            camera_space_range = math.sqrt(float(cam.x) ** 2 + float(cam.y) ** 2 + float(cam.z) ** 2)
            base_space_range = math.sqrt(
                (float(base.x) - float(camera.x)) ** 2
                + (float(base.y) - float(camera.y)) ** 2
                + (float(base.z) - float(camera.z)) ** 2
            )
            return abs(camera_space_range - base_space_range)
        except Exception:
            return None

    @staticmethod
    def _pregrasp_status_text(execute, success, message):
        if execute:
            prefix = '执行成功：' if success else '执行失败：'
        else:
            prefix = '规划成功：' if success else '规划失败：'
        text = str(message)
        if not success and PerceptionWidget._is_unreachable_pregrasp_message(text):
            text = '识别成功但目标不可达/姿态不可达：' + text
        return prefix + text

    @staticmethod
    def _is_unreachable_pregrasp_message(message):
        text = str(message or '').lower()
        markers = (
            'target unreachable',
            'pose orientation invalid',
            'position-only',
            'candidate orientation',
            'unable to solve',
            'ik',
        )
        return any(marker in text for marker in markers)

    @staticmethod
    def _is_position_only_plan_message(message):
        return 'position-only' in str(message or '').lower()

    def visual_jog(self, axis):
        step = float(self.visual_step.currentData())
        execute = bool(self.visual_execute.isChecked())
        try:
            request = self._visual_jog_values(axis, step, execute)
            rospy.wait_for_service('/supervisor/cartesian_jog', timeout=1.0)
            srv = rospy.ServiceProxy('/supervisor/cartesian_jog', CartesianJog)
            res = srv(*request)
            mode = '执行微动' if execute else '规划微动'
            self.status.setText('%s：%s；等待识别刷新' % (mode, res.message))
        except Exception as exc:
            self.status.setText('视觉对准失败：%s' % exc)

    def _parse_hsv(self, text):
        values = [int(v.strip()) for v in text.replace(';', ',').split(',') if v.strip()]
        if len(values) != 3:
            raise ValueError('HSV 需要 3 个整数，例如 35,40,40')
        return [max(0, min(255, v)) for v in values]

    def _list_text(self, values):
        return ','.join(str(int(v)) for v in values[:3])

    def _object_bbox(self, msg):
        width = int(getattr(msg, 'bbox_width', 0))
        height = int(getattr(msg, 'bbox_height', 0))
        if width <= 0 or height <= 0:
            return None
        return (
            int(getattr(msg, 'bbox_x', 0)),
            int(getattr(msg, 'bbox_y', 0)),
            width,
            height,
        )

    def _overlay_color_rgb(self, description):
        text = str(description or '').lower()
        for keyword, key in self.COLOR_KEYWORDS.items():
            if keyword.lower() in text:
                return self.DRAW_COLORS_RGB.get(key, (80, 255, 120))
        return (80, 255, 120)

    @staticmethod
    def _visual_jog_values(axis, step_m, execute):
        if axis not in PerceptionWidget.VISUAL_JOG_AXES:
            raise ValueError('未知点动方向：%s' % axis)
        step = abs(float(step_m))
        sx, sy, sz = PerceptionWidget.VISUAL_JOG_AXES[axis]
        return (sx * step, sy * step, sz * step, 0.0, 0.0, 0.0, bool(execute))

    def _parse_description(self, description):
        text = description.lower()
        color_key = None
        color_name = '使用高级 HSV'
        for keyword, key in self.COLOR_KEYWORDS.items():
            if keyword.lower() in text:
                color_key = key
                break

        hsv_ranges = []
        if color_key:
            first, second, color_name = self.COLOR_PRESETS[color_key]
            hsv_ranges.append((first[0], first[1]))
            if second:
                hsv_ranges.append((second[0], second[1]))

        shape = 'any'
        shape_name = '任意形状'
        for keyword, value in self.SHAPE_KEYWORDS.items():
            if keyword.lower() in text:
                shape, shape_name = value
                break

        yolo_target_class = ''
        for keyword, class_name in self.YOLO_CLASS_KEYWORDS:
            if keyword.lower() in text:
                yolo_target_class = class_name
                break

        class_name = yolo_target_class if yolo_target_class else '全部模型类别'
        summary = '颜色：%s；形状：%s；YOLO类别：%s' % (color_name, shape_name, class_name)
        return {
            'hsv_ranges': hsv_ranges,
            'shape': shape,
            'yolo_target_class': yolo_target_class,
            'summary': summary,
        }

    def _set_layout_visible(self, layout, visible):
        for index in range(layout.count()):
            item = layout.itemAt(index)
            widget = item.widget()
            child_layout = item.layout()
            if widget is not None:
                widget.setVisible(visible)
            if child_layout is not None:
                self._set_layout_visible(child_layout, visible)
