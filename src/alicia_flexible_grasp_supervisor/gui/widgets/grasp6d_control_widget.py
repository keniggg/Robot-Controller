from dataclasses import dataclass
import threading
import time

from PyQt5 import QtCore, QtWidgets
import rospy
from geometry_msgs.msg import PoseArray
from std_msgs.msg import String

from alicia_flexible_grasp.vision.remote_grasp6d_client import RemoteGrasp6DClient
from alicia_flexible_grasp_supervisor.msg import GraspState
from alicia_flexible_grasp_supervisor.srv import StartGrasp, StopGrasp, TriggerZero
from gui.theme import metric_chip, panel


@dataclass
class Grasp6DPlanState:
    fresh: bool
    age_sec: float
    text: str


@dataclass
class Grasp6DButtonLabels:
    check_remote: str
    request_plan: str
    execute_grasp: str
    stop: str


@dataclass
class Grasp6DGuiState:
    remote_status: str
    plan_state: Grasp6DPlanState
    grasp_state: str
    grasp_message: str

    def summary(self):
        plan_prefix = '可执行' if self.plan_state.fresh else '不可执行'
        return (
            '远程推理：%s\n'
            '候选计划：%s，%s\n'
            '抓取状态：%s | %s'
            % (
                self.remote_status or '等待状态',
                plan_prefix,
                self.plan_state.text,
                self.grasp_state or 'UNKNOWN',
                self.grasp_message or '',
            )
        )


def grasp6d_button_labels():
    return Grasp6DButtonLabels(
        check_remote='检查远程推理端',
        request_plan='生成 6D 候选',
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
    grasp_signal = QtCore.pyqtSignal(object)
    check_signal = QtCore.pyqtSignal(bool, str)
    command_signal = QtCore.pyqtSignal(bool, str)

    def __init__(
        self,
        status_topic='/grasp_6d/status',
        plan_topic='/grasp_6d/plan',
        grasp_state_topic='/grasp/state',
        compact=False,
    ):
        super().__init__()
        self._alive = True
        self._compact = bool(compact)
        self._last_plan_time = None
        self._last_plan_pose_count = 0
        self._remote_status = '等待远程 6D 状态'
        self._grasp_state = 'IDLE'
        self._grasp_message = ''
        self._checking = False
        self._requesting_plan = False
        self._command_active = False
        self._plan_max_age_sec = float(rospy.get_param('/grasp/grasp6d_plan_max_age_sec', 2.0))
        self._server_url = str(rospy.get_param('/grasp_6d/remote/server_url', 'http://172.23.132.97:8000'))
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
        self.plan_chip = None
        self.pose_chip = None
        self.grasp_chip = None
        if not self._compact:
            metrics = QtWidgets.QGridLayout()
            metrics.setSpacing(8)
            self.endpoint_chip = metric_chip('端点 %s' % self._server_url)
            self.remote_chip = metric_chip('远程等待', accent=True)
            self.plan_chip = metric_chip('候选等待')
            self.pose_chip = metric_chip('路径 --')
            self.grasp_chip = metric_chip('抓取 IDLE')
            for index, chip in enumerate((self.endpoint_chip, self.remote_chip, self.plan_chip, self.pose_chip, self.grasp_chip)):
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
        self.grasp_signal.connect(self._update_grasp)
        self.check_signal.connect(self._finish_remote_check)
        self.command_signal.connect(self._finish_command)
        self._subscribers = [
            rospy.Subscriber(status_topic, String, self._emit_status_if_alive, queue_size=1),
            rospy.Subscriber(plan_topic, PoseArray, self._emit_plan_if_alive, queue_size=1),
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

    def _emit_grasp_if_alive(self, msg):
        if self.__dict__.get('_alive', False):
            self.grasp_signal.emit(msg)

    def _update_remote_status(self, text):
        self._remote_status = text or '等待远程 6D 状态'
        self._refresh_view()

    def _update_plan(self, msg):
        self._last_plan_pose_count = len(getattr(msg, 'poses', []) or [])
        self._last_plan_time = time.monotonic() if self._last_plan_pose_count >= 4 else None
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
        self._requesting_plan = True
        self._last_plan_time = None
        self._last_plan_pose_count = 0
        self.request_plan_btn.setEnabled(False)
        self.execute_btn.setEnabled(False)
        self._remote_status = '正在请求远程 6D 候选...'
        self._refresh_view()
        thread = threading.Thread(target=self._run_request_plan, daemon=True)
        thread.start()

    def _run_request_plan(self):
        try:
            rospy.wait_for_service('/grasp_6d/request_plan', timeout=1.5)
            res = rospy.ServiceProxy('/grasp_6d/request_plan', TriggerZero)(True)
            self._emit_check_result_if_alive(bool(res.success), str(res.message))
        except Exception as exc:
            self._emit_check_result_if_alive(False, '请求 6D 候选失败：%s' % exc)

    def execute_grasp(self):
        if self._command_active:
            return
        plan_state = format_grasp6d_plan_state(self._last_plan_time, max_age_sec=self._plan_max_age_sec)
        if not plan_state.fresh:
            self._set_command_status(False, '没有新鲜的 6D 抓取候选，无法执行；请确认 WSL2 推理端和相机画面')
            return
        self._command_active = True
        self.execute_btn.setEnabled(False)
        self.check_btn.setEnabled(False)
        self.summary.setText('正在请求 /grasp/start，执行 6D 抓取流程...')
        thread = threading.Thread(target=self._run_start_grasp, daemon=True)
        thread.start()

    def _run_start_grasp(self):
        try:
            rospy.wait_for_service('/grasp/start', timeout=1.5)
            res = rospy.ServiceProxy('/grasp/start', StartGrasp)(True)
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

    def _finish_remote_check(self, ok, message):
        self._checking = False
        self._requesting_plan = False
        self.check_btn.setEnabled(True)
        self.request_plan_btn.setEnabled(True)
        self._remote_status = message
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
        self._plan_max_age_sec = float(rospy.get_param('/grasp/grasp6d_plan_max_age_sec', self._plan_max_age_sec))
        plan_state = format_grasp6d_plan_state(self._last_plan_time, max_age_sec=self._plan_max_age_sec)
        state = Grasp6DGuiState(
            remote_status=self._remote_status,
            plan_state=plan_state,
            grasp_state=self._grasp_state,
            grasp_message=self._grasp_message,
        )
        self.summary.setText(state.summary())
        if self.remote_chip is not None:
            self.remote_chip.setText('远程 %s' % _short_text(self._remote_status, 26))
        if self.plan_chip is not None:
            self.plan_chip.setText(('候选可执行 ' if plan_state.fresh else '候选不可用 ') + plan_state.text)
        if self.pose_chip is not None:
            self.pose_chip.setText('路径 %d / 4' % int(self._last_plan_pose_count))
        if self.grasp_chip is not None:
            self.grasp_chip.setText('抓取 %s' % _short_text(self._grasp_state, 18))
        self.request_plan_btn.setEnabled(not self._requesting_plan)
        self.execute_btn.setEnabled((not self._command_active) and (not self._requesting_plan) and plan_state.fresh)

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
