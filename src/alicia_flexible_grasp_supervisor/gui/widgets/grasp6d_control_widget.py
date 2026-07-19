from copy import deepcopy
from dataclasses import dataclass
import math
import threading
import time

from PyQt5 import QtCore, QtWidgets
import rospy
from geometry_msgs.msg import PoseArray
from std_msgs.msg import String

from alicia_flexible_grasp.vision.remote_grasp6d_client import RemoteGrasp6DClient
from alicia_flexible_grasp.grasp.rich_plan_integrity import (
    plan_id_matches_content,
    required_open_width_is_valid,
    stamp_nanoseconds as _stamp_nanoseconds,
    stamp_seconds as _stamp_seconds,
    strict_plan_id_equal,
    validate_finite_pose,
    validate_plan_header_binding,
    validate_rich_geometry,
)
from alicia_flexible_grasp_supervisor.msg import Grasp6DPlan, GraspState
from alicia_flexible_grasp_supervisor.srv import StartGrasp, StopGrasp, TriggerZero
from gui.theme import metric_chip, panel


@dataclass
class Grasp6DPlanState:
    fresh: bool
    age_sec: float
    text: str


def _ros_now_seconds():
    return _stamp_seconds(rospy.Time.now())


def _finite_pose(pose):
    try:
        validate_finite_pose(pose)
    except (TypeError, ValueError, AttributeError):
        return False
    return True


def _finite_geometry(geometry):
    try:
        validate_rich_geometry(geometry)
    except (TypeError, ValueError, AttributeError):
        return False
    return True


def validate_enriched_plan(plan, now_sec=None, validity_sec=2.0):
    now = _ros_now_seconds() if now_sec is None else float(now_sec)
    validity = max(0.0, float(validity_sec))
    diagnostic = str(getattr(plan, 'diagnostic', '') or '')
    if plan is None or not bool(getattr(plan, 'valid', False)):
        return Grasp6DPlanState(False, float('inf'), diagnostic or '富计划无效')
    plan_id = str(getattr(plan, 'plan_id', '') or '').strip()
    if not plan_id:
        return Grasp6DPlanState(False, float('inf'), 'PLAN_ID_MISSING: 富计划标识为空')
    if not str(getattr(plan, 'model_choice', '') or '').strip():
        return Grasp6DPlanState(False, float('inf'), 'MODEL_MISSING: 富计划模型为空')
    poses = list(getattr(plan, 'poses', ()) or ())
    if len(poses) != 4 or not all(_finite_pose(pose) for pose in poses):
        return Grasp6DPlanState(False, float('inf'), 'PLAN_MALFORMED: 必须包含四个有限姿态')
    try:
        required_width = float(plan.required_open_width_m)
    except Exception:
        required_width = float('nan')
    if not required_open_width_is_valid(required_width):
        return Grasp6DPlanState(False, float('inf'), 'GRIPPER_TOO_NARROW: 富计划开口宽度无效')
    if not _finite_geometry(getattr(plan, 'object_geometry', None)):
        return Grasp6DPlanState(False, float('inf'), 'OBB_INVALID: 富计划物体几何无效')
    try:
        validate_plan_header_binding(plan)
    except (TypeError, ValueError, AttributeError) as exc:
        return Grasp6DPlanState(
            False,
            float('inf'),
            'PLAN_SNAPSHOT_MISMATCH: %s' % exc,
        )
    if not plan_id_matches_content(plan):
        return Grasp6DPlanState(
            False,
            float('inf'),
            'PLAN_ID_MISMATCH: 富计划内容摘要不匹配',
        )
    stamp_sec = _stamp_seconds(getattr(getattr(plan, 'header', None), 'stamp', None))
    if not math.isfinite(stamp_sec) or stamp_sec <= 0.0:
        return Grasp6DPlanState(False, float('inf'), 'PLAN_STALE: 源时间戳为空')
    age = now - stamp_sec
    if age < 0.0:
        return Grasp6DPlanState(False, age, 'PLAN_FUTURE: 富计划来自未来时间')
    if age > validity:
        return Grasp6DPlanState(False, age, 'PLAN_STALE: 富计划已过期 %.1fs' % age)
    return Grasp6DPlanState(
        True,
        age,
        '富计划 %s %.1fs 内有效' % (plan_id, age),
    )


class Grasp6DReadinessTracker:
    def __init__(self, validity_sec=2.0):
        self.validity_sec = max(0.0, float(validity_sec))
        self.legacy_pose_count = 0
        self.enriched_seen = False
        self.plan_id = ''
        self._plan = None
        self._watermark_stamp_ns = 0
        self._watermark_plan_id = ''
        self._watermark_tombstoned = False
        self._invalid_text = '等待 6D 富计划'
        self._lock = threading.RLock()

    def clear(self, reason='等待 6D 富计划'):
        with self._lock:
            if self._plan is not None and self._watermark_stamp_ns <= 0:
                self._watermark_stamp_ns = _stamp_nanoseconds(
                    self._plan.header.stamp
                )
                self._watermark_plan_id = str(self._plan.plan_id or '')
            if self._watermark_stamp_ns > 0:
                self._watermark_tombstoned = True
            self.plan_id = ''
            self._plan = None
            self._invalid_text = str(reason or '等待 6D 富计划')

    def update_legacy(self, message):
        with self._lock:
            self.legacy_pose_count = len(getattr(message, 'poses', ()) or ())

    def update_enriched(self, message, now_sec=None):
        state = validate_enriched_plan(
            message,
            now_sec=now_sec,
            validity_sec=self.validity_sec,
        )
        with self._lock:
            self.enriched_seen = True
            incoming_ns = _stamp_nanoseconds(
                getattr(getattr(message, 'header', None), 'stamp', None)
            )
            incoming_id = str(getattr(message, 'plan_id', '') or '')
            if not state.fresh:
                if incoming_ns > self._watermark_stamp_ns:
                    self._watermark_stamp_ns = incoming_ns
                    self._watermark_plan_id = incoming_id
                if self._watermark_stamp_ns > 0:
                    self._watermark_tombstoned = True
                self.plan_id = ''
                self._plan = None
                self._invalid_text = state.text
                return state
            replayed_source = (
                incoming_ns < self._watermark_stamp_ns
                or (
                    incoming_ns == self._watermark_stamp_ns
                    and (
                        self._watermark_tombstoned
                        or not strict_plan_id_equal(
                            incoming_id, self._watermark_plan_id
                        )
                    )
                )
            )
            if replayed_source:
                replayed = Grasp6DPlanState(
                    False,
                    state.age_sec,
                    'PLAN_REPLAYED: 收到旧的、冲突的或已撤权富计划源时间戳',
                )
                if self._watermark_stamp_ns > 0:
                    self._watermark_tombstoned = True
                self.plan_id = ''
                self._plan = None
                self._invalid_text = replayed.text
                return replayed
            if incoming_ns > self._watermark_stamp_ns:
                self._watermark_stamp_ns = incoming_ns
                self._watermark_plan_id = incoming_id
                self._watermark_tombstoned = False
            copied = deepcopy(message)
            self._plan = copied
            self.plan_id = str(copied.plan_id)
            self._invalid_text = ''
            return state

    def state(self, now_sec=None):
        with self._lock:
            if self._plan is None:
                return Grasp6DPlanState(False, float('inf'), self._invalid_text)
            state = validate_enriched_plan(
                self._plan,
                now_sec=now_sec,
                validity_sec=self.validity_sec,
            )
            if not state.fresh:
                if self._watermark_stamp_ns > 0:
                    self._watermark_tombstoned = True
                self.plan_id = ''
                self._plan = None
                self._invalid_text = state.text
            return state

    def matches_current(self, plan_id, now_sec=None):
        state = self.state(now_sec=now_sec)
        with self._lock:
            return state.fresh and strict_plan_id_equal(self.plan_id, plan_id)


def local_plan_execution_notice(use_remote_grasp6d):
    if bool(use_remote_grasp6d):
        return ''
    return '本地旧版候选仅供显示，执行需要富计划'


def is_local_legacy_status(text):
    normalized = str(text or '').strip().lower()
    return (
        normalized.startswith('6d grasp ')
        and not normalized.startswith('remote 6d grasp ')
    )


@dataclass
class Grasp6DButtonLabels:
    check_remote: str
    request_plan: str
    stop_inference: str
    execute_grasp: str
    stop: str


@dataclass
class Grasp6DGuiState:
    remote_status: str
    preview_state: Grasp6DPlanState
    plan_state: Grasp6DPlanState
    grasp_state: str
    grasp_message: str

    def summary(self):
        preview_prefix = '稳定' if self.preview_state.fresh else '未稳定'
        execution_prefix = '可执行' if self.plan_state.fresh else '不可执行'
        return (
            '远程推理：%s\n'
            'Preview：%s，%s\n'
            'Execution：%s，%s\n'
            '抓取状态：%s | %s'
            % (
                self.remote_status or '等待状态',
                preview_prefix,
                self.preview_state.text,
                execution_prefix,
                self.plan_state.text,
                self.grasp_state or 'UNKNOWN',
                self.grasp_message or '',
            )
        )


def grasp6d_button_labels():
    return Grasp6DButtonLabels(
        check_remote='检查远程推理端',
        request_plan='生成 6D 候选',
        stop_inference='停止生成候选',
        execute_grasp='执行 6D 抓取流程',
        stop='停止抓取',
    )


def format_grasp6d_plan_state(last_plan_time_sec, now_sec=None, max_age_sec=2.0):
    if last_plan_time_sec is None:
        return Grasp6DPlanState(False, float('inf'), '等待 6D 抓取候选')
    now_sec = time.monotonic() if now_sec is None else float(now_sec)
    max_age_sec = max(0.0, float(max_age_sec))
    age = max(0.0, now_sec - float(last_plan_time_sec))
    if age <= max_age_sec:
        return Grasp6DPlanState(True, age, '6D 抓取候选 %.1fs 内更新' % age)
    return Grasp6DPlanState(False, age, '6D 抓取候选已过期 %.1fs' % age)


class Grasp6DControlWidget(QtWidgets.QWidget):
    status_signal = QtCore.pyqtSignal(str)
    plan_signal = QtCore.pyqtSignal(object)
    preview_plan_signal = QtCore.pyqtSignal(object)
    legacy_plan_signal = QtCore.pyqtSignal(object)
    grasp_signal = QtCore.pyqtSignal(object)
    check_signal = QtCore.pyqtSignal(bool, str)
    stream_signal = QtCore.pyqtSignal(bool, bool, str)
    command_signal = QtCore.pyqtSignal(bool, str)

    def __init__(
        self,
        status_topic='/grasp_6d/status',
        plan_topic='/grasp_6d/plan',
        grasp_state_topic='/grasp/state',
        compact=False,
        enriched_plan_topic='/grasp_6d/plan_enriched',
        preview_enriched_plan_topic='/grasp_6d/preview_plan_enriched',
    ):
        super().__init__()
        self._alive = True
        self._compact = bool(compact)
        self._plan_validity_sec = float(
            rospy.get_param('/grasp_6d/plan_validity_sec', 2.0)
        )
        self._readiness = Grasp6DReadinessTracker(self._plan_validity_sec)
        self._preview_readiness = Grasp6DReadinessTracker(
            self._plan_validity_sec
        )
        self._last_plan_pose_count = 0
        self._use_remote_grasp6d = bool(
            rospy.get_param('/grasp_6d/use_remote_grasp6d', True)
        )
        self._legacy_only_status = False
        self._remote_status = '等待远程 6D 状态'
        self._grasp_state = 'IDLE'
        self._grasp_message = ''
        self._checking = False
        self._requesting_plan = False
        self._streaming_enabled = bool(
            rospy.get_param('/grasp_6d/remote/auto_request', False)
        )
        self._command_active = False
        self._server_url = str(rospy.get_param('/grasp_6d/remote/server_url', 'http://172.23.132.97:8000'))
        enriched_plan_topic = str(
            rospy.get_param(
                '/grasp/grasp6d_enriched_plan_topic',
                enriched_plan_topic,
            )
        )
        labels = grasp6d_button_labels()

        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        frame, body = panel('远程 6D 抓取' if not compact else '6D 抓取状态')
        layout.addWidget(frame)

        self.summary = QtWidgets.QLabel('等待 6D 抓取状态')
        self.summary.setObjectName('StateBanner')
        self.summary.setWordWrap(True)
        self.summary.setMinimumHeight(76 if not compact else 58)
        body.addWidget(self.summary)

        self.endpoint_chip = None
        self.remote_chip = None
        self.preview_chip = None
        self.plan_chip = None
        self.pose_chip = None
        self.grasp_chip = None
        if not self._compact:
            metrics = QtWidgets.QGridLayout()
            metrics.setSpacing(8)
            self.endpoint_chip = metric_chip('端点 %s' % self._server_url)
            self.remote_chip = metric_chip('远程等待', accent=True)
            self.preview_chip = metric_chip('Preview 等待')
            self.plan_chip = metric_chip('Execution 等待')
            self.pose_chip = metric_chip('路径 --')
            self.grasp_chip = metric_chip('抓取 IDLE')
            for index, chip in enumerate((self.endpoint_chip, self.remote_chip, self.preview_chip, self.plan_chip, self.pose_chip, self.grasp_chip)):
                metrics.addWidget(chip, index // 2, index % 2)
            body.addLayout(metrics)

        actions = QtWidgets.QHBoxLayout()
        actions.setSpacing(10)
        self.check_btn = QtWidgets.QPushButton(labels.check_remote)
        self.check_btn.setObjectName('PrimaryButton')
        self.request_plan_btn = QtWidgets.QPushButton(labels.request_plan)
        self.request_plan_btn.setObjectName('PrimaryButton')
        self.execute_btn = QtWidgets.QPushButton(labels.execute_grasp)
        self.execute_btn.setObjectName('DangerButton')
        self.stop_btn = QtWidgets.QPushButton(labels.stop)
        self.stop_btn.setObjectName('DangerButton')
        self.check_btn.clicked.connect(self.check_remote)
        self.request_plan_btn.clicked.connect(self.request_plan)
        self.execute_btn.clicked.connect(self.execute_grasp)
        self.stop_btn.clicked.connect(self.stop_grasp)
        actions.addWidget(self.check_btn, 2)
        actions.addWidget(self.request_plan_btn, 2)
        actions.addWidget(self.execute_btn, 2)
        actions.addWidget(self.stop_btn, 1)
        body.addLayout(actions)
        if not self._compact:
            body.addStretch(1)

        self.status_signal.connect(self._update_remote_status)
        self.plan_signal.connect(self._update_plan)
        self.preview_plan_signal.connect(self._update_preview_plan)
        self.legacy_plan_signal.connect(self._update_legacy_plan)
        self.grasp_signal.connect(self._update_grasp)
        self.check_signal.connect(self._finish_remote_check)
        self.stream_signal.connect(self._finish_stream_request)
        self.command_signal.connect(self._finish_command)
        self._subscribers = [
            rospy.Subscriber(status_topic, String, self._emit_status_if_alive, queue_size=1),
            rospy.Subscriber(
                enriched_plan_topic,
                Grasp6DPlan,
                self._emit_plan_if_alive,
                queue_size=1,
            ),
            rospy.Subscriber(
                preview_enriched_plan_topic,
                Grasp6DPlan,
                self._emit_preview_plan_if_alive,
                queue_size=1,
            ),
            rospy.Subscriber(
                plan_topic,
                PoseArray,
                self._emit_legacy_plan_if_alive,
                queue_size=1,
            ),
            rospy.Subscriber(grasp_state_topic, GraspState, self._emit_grasp_if_alive, queue_size=1),
        ]
        self._timer = QtCore.QTimer(self)
        self._timer.timeout.connect(self._refresh_view)
        self._timer.start(500)
        self.destroyed.connect(lambda *_: self._shutdown_ros())
        self._refresh_view()

    def _emit_status_if_alive(self, msg):
        if self.__dict__.get('_alive', False):
            self.status_signal.emit(str(getattr(msg, 'data', '')))

    def _emit_plan_if_alive(self, msg):
        if self.__dict__.get('_alive', False):
            self.plan_signal.emit(msg)

    def _emit_preview_plan_if_alive(self, msg):
        if self.__dict__.get('_alive', False):
            self.preview_plan_signal.emit(msg)

    def _emit_legacy_plan_if_alive(self, msg):
        if self.__dict__.get('_alive', False):
            self.legacy_plan_signal.emit(msg)

    def _emit_grasp_if_alive(self, msg):
        if self.__dict__.get('_alive', False):
            self.grasp_signal.emit(msg)

    def _update_remote_status(self, text):
        self._remote_status = text or '等待远程 6D 状态'
        if (
            is_local_legacy_status(self._remote_status)
            and not self._readiness.enriched_seen
        ):
            self._legacy_only_status = True
            self._use_remote_grasp6d = False
        self._refresh_view()

    def _update_plan(self, msg):
        self._readiness.update_enriched(msg)
        self._refresh_view()

    def _update_preview_plan(self, msg):
        self._preview_readiness.update_enriched(msg)
        self._refresh_view()

    def _update_legacy_plan(self, msg):
        self._readiness.update_legacy(msg)
        self._last_plan_pose_count = self._readiness.legacy_pose_count
        self._refresh_view()

    def _update_grasp(self, msg):
        self._grasp_state = str(getattr(msg, 'state', 'UNKNOWN'))
        self._grasp_message = str(getattr(msg, 'message', ''))
        self._refresh_view()

    def check_remote(self):
        if self._checking:
            return
        self._checking = True
        self.check_btn.setEnabled(False)
        self.remote_chip.setText('远程检查中')
        thread = threading.Thread(target=self._run_remote_check, daemon=True)
        thread.start()

    def _run_remote_check(self):
        try:
            health = RemoteGrasp6DClient(self._server_url, timeout_sec=2.0).health()
            ok = bool(health.get('ok', False))
            if ok:
                message = '远程推理端在线：%s' % health.get('backend', 'unknown')
            else:
                missing = health.get('missing') or health.get('error') or 'unknown'
                message = '远程推理端未就绪：%s' % missing
        except Exception as exc:
            ok = False
            message = '远程推理端连接失败：%s' % exc
        self._emit_check_result_if_alive(ok, message)

    def request_plan(self):
        if self._requesting_plan:
            return
        enable = not self._streaming_enabled
        self._requesting_plan = True
        self.request_plan_btn.setEnabled(False)
        self._remote_status = (
            '正在启动持续 6D 候选生成...'
            if enable
            else '正在停止持续 6D 候选生成...'
        )
        self._refresh_view()
        thread = threading.Thread(
            target=self._run_set_streaming,
            args=(enable,),
            daemon=True,
        )
        thread.start()

    def _run_set_streaming(self, enabled):
        enabled = bool(enabled)
        try:
            rospy.wait_for_service('/grasp_6d/request_plan', timeout=1.5)
            res = rospy.ServiceProxy('/grasp_6d/request_plan', TriggerZero)(enabled)
            self._emit_stream_result_if_alive(
                enabled,
                bool(res.success),
                str(res.message),
            )
        except Exception as exc:
            action = '启动' if enabled else '停止'
            self._emit_stream_result_if_alive(
                enabled,
                False,
                '%s持续 6D 候选失败：%s' % (action, exc),
            )

    def _run_request_plan(self):
        """Compatibility worker: a legacy direct call starts streaming."""
        self._run_set_streaming(True)

    def execute_grasp(self):
        if self._command_active:
            return
        plan_state = self._readiness.state()
        if not plan_state.fresh:
            self._set_command_status(False, '没有新鲜的 6D 抓取候选，无法执行；请确认 WSL2 推理端和相机画面')
            return
        self._execution_plan_id = str(self._readiness.plan_id)
        self._command_active = True
        self.execute_btn.setEnabled(False)
        self.check_btn.setEnabled(False)
        self.summary.setText('正在请求 /grasp/start，执行 6D 抓取流程...')
        thread = threading.Thread(target=self._run_start_grasp, daemon=True)
        thread.start()

    def _run_start_grasp(self):
        plan_id = str(getattr(self, '_execution_plan_id', '') or '')
        if not self._readiness.matches_current(plan_id):
            self._emit_command_result_if_alive(False, 'PLAN_REPLACED: 富计划在服务调用前失效')
            return
        try:
            rospy.wait_for_service('/grasp/start', timeout=1.5)
            if not self._readiness.matches_current(plan_id):
                self._emit_command_result_if_alive(False, 'PLAN_REPLACED: 富计划在等待服务时失效')
                return
            res = rospy.ServiceProxy('/grasp/start', StartGrasp)(
                execute=True,
                plan_id=plan_id,
            )
            self._emit_command_result_if_alive(bool(res.success), str(res.message))
        except Exception as exc:
            self._emit_command_result_if_alive(False, str(exc))

    def stop_grasp(self):
        thread = threading.Thread(target=self._run_stop_grasp, daemon=True)
        thread.start()

    def _run_stop_grasp(self):
        try:
            rospy.wait_for_service('/grasp/stop', timeout=1.0)
            res = rospy.ServiceProxy('/grasp/stop', StopGrasp)(True)
            self._emit_command_result_if_alive(bool(res.success), '停止请求：%s' % res.message)
        except Exception as exc:
            self._emit_command_result_if_alive(False, '停止请求失败：%s' % exc)

    def _emit_check_result_if_alive(self, ok, message):
        if self.__dict__.get('_alive', False):
            self.check_signal.emit(bool(ok), str(message))

    def _emit_command_result_if_alive(self, ok, message):
        if self.__dict__.get('_alive', False):
            self.command_signal.emit(bool(ok), str(message))

    def _emit_stream_result_if_alive(self, enabled, ok, message):
        if self.__dict__.get('_alive', False):
            self.stream_signal.emit(bool(enabled), bool(ok), str(message))

    def _finish_remote_check(self, ok, message):
        self._checking = False
        self.check_btn.setEnabled(True)
        self._remote_status = message
        self._refresh_view()

    def _finish_stream_request(self, enabled, ok, message):
        self._requesting_plan = False
        if ok:
            self._streaming_enabled = bool(enabled)
        self._remote_status = str(message)
        self._refresh_view()

    def _finish_command(self, ok, message):
        self._command_active = False
        self.check_btn.setEnabled(True)
        self.request_plan_btn.setEnabled(True)
        self.execute_btn.setEnabled(True)
        self._set_command_status(ok, message)

    def _set_command_status(self, ok, message):
        prefix = '操作成功：' if ok else '操作失败：'
        self._grasp_message = prefix + str(message)
        self._refresh_view()

    def _refresh_view(self):
        self._plan_validity_sec = float(
            rospy.get_param(
                '/grasp_6d/plan_validity_sec',
                self._plan_validity_sec,
            )
        )
        self._readiness.validity_sec = max(0.0, self._plan_validity_sec)
        self._preview_readiness.validity_sec = max(
            0.0, self._plan_validity_sec
        )
        configured_remote = bool(
            rospy.get_param(
                '/grasp_6d/use_remote_grasp6d',
                self._use_remote_grasp6d,
            )
        )
        self._use_remote_grasp6d = configured_remote and not self._legacy_only_status
        plan_state = self._readiness.state()
        preview_state = self._preview_readiness.state()
        local_notice = local_plan_execution_notice(self._use_remote_grasp6d)
        if local_notice and not plan_state.fresh:
            plan_state = Grasp6DPlanState(False, plan_state.age_sec, local_notice)
        state = Grasp6DGuiState(
            remote_status=self._remote_status,
            preview_state=preview_state,
            plan_state=plan_state,
            grasp_state=self._grasp_state,
            grasp_message=self._grasp_message,
        )
        self.summary.setText(state.summary())
        if self.remote_chip is not None:
            self.remote_chip.setText('远程 %s' % _short_text(self._remote_status, 26))
        if self.preview_chip is not None:
            self.preview_chip.setText(
                ('Preview 稳定 ' if preview_state.fresh else 'Preview 未稳定 ')
                + preview_state.text
            )
        if self.plan_chip is not None:
            self.plan_chip.setText(
                ('Execution 可执行 ' if plan_state.fresh else 'Execution 不可用 ')
                + plan_state.text
            )
        if self.pose_chip is not None:
            rich_count = 4 if plan_state.fresh else 0
            self.pose_chip.setText(
                '富路径 %d / 4 | 可视化 %d'
                % (rich_count, int(self._last_plan_pose_count))
            )
        if self.grasp_chip is not None:
            self.grasp_chip.setText('抓取 %s' % _short_text(self._grasp_state, 18))
        self.request_plan_btn.setEnabled(not self._requesting_plan)
        labels = grasp6d_button_labels()
        self.request_plan_btn.setText(
            labels.stop_inference
            if self._streaming_enabled
            else labels.request_plan
        )
        self.execute_btn.setEnabled((not self._command_active) and plan_state.fresh)

    def _shutdown_ros(self):
        self._alive = False
        for subscriber in self.__dict__.get('_subscribers', []):
            try:
                subscriber.unregister()
            except Exception:
                pass
        self._subscribers = []

    def closeEvent(self, event):
        self._shutdown_ros()
        super().closeEvent(event)


def _short_text(text, limit):
    text = str(text or '')
    limit = max(4, int(limit))
    if len(text) <= limit:
        return text
    return text[:limit - 1] + '…'
