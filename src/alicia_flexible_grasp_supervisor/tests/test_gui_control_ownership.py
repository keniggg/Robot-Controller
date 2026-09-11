from types import SimpleNamespace as NS
import pathlib
import sys

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / 'src'))

import pytest
from gui.widgets.joint_control_widget import JointControlWidget, DirectCommandGesture
import gui.widgets.joint_control_widget as widget_module
from test_grasp_task_sequence import grasp_task_node
import test_motion_gateway_controller_start as gateway_tests


class Slider:
    def __init__(self, value=0):
        self.current = value
        self.blocked = False
        self.emitted = []
        self.enabled = True
    def minimum(self): return -3140
    def maximum(self): return 3140
    def value(self): return self.current
    def blockSignals(self, blocked):
        old, self.blocked = self.blocked, blocked
        return old
    def setValue(self, value):
        self.current = value
        if not self.blocked: self.emitted.append(value)
    def setEnabled(self, enabled): self.enabled = enabled


def widget(checked):
    w = JointControlWidget.__new__(JointControlWidget)
    w.names = ['Joint1', 'Joint2']
    w.sliders = [Slider(100), Slider(200)]
    w.realtime_direct = NS(isChecked=lambda: checked)
    w.show_feedback_angles = False
    w.current_state = None
    w.labels = [NS(setText=lambda text: None) for _ in w.names]
    w._syncing_sliders = False
    return w


def test_automatic_mode_follows_named_real_feedback_without_publishing_commands():
    w = widget(False)
    w.update_current_state(NS(name=['Joint2', 'Joint1'], position=[-.25, .32]))
    assert [s.value() for s in w.sliders] == [320, -250]
    assert all(not s.emitted and not s.blocked for s in w.sliders)
    w.update_current_state(NS(name=['Joint1', 'Joint2'], position=[float('nan'), -.28]))
    assert [s.value() for s in w.sliders] == [320, -280]


def test_manual_mode_feedback_does_not_replace_slider_target():
    w = widget(True)
    w.update_current_state(NS(name=w.names, position=[.8, .9]))
    assert [s.value() for s in w.sliders] == [100, 200]
    assert w.current_state.position == [.8, .9]


def test_unchecking_publishes_ownership_cancels_pending_slider_and_updates_display(monkeypatch):
    w = widget(False)
    w.current_state = NS(name=w.names, position=[.31, .42])
    w.plan_btn = Slider(); w.exec_btn = Slider()
    w._direct_gesture = DirectCommandGesture()
    w._pending_direct_publish = True
    events = []
    w.direct_recovery = NS(cancel=lambda: events.append('cancel_slider'))
    w.control_mode_pub = NS(publish=lambda msg: events.append(('mode', msg.data)))
    w.status = NS(setText=lambda text: None)
    w.switch_trajectory_controllers = lambda enabled: pytest.fail('mode switch must not stop controllers')
    monkeypatch.setattr(widget_module.rospy, 'set_param', lambda k,v: events.append((k,v)))
    w.update_control_mode(False)
    assert ('mode', False) in events and 'cancel_slider' in events
    assert not w._pending_direct_publish
    assert [s.value() for s in w.sliders] == [310, 420]
    assert all(not s.enabled and not s.emitted for s in w.sliders)


def test_manual_mode_blocks_start_even_when_actuation_confirmed(monkeypatch):
    node = grasp_task_node.GraspTaskNode.__new__(grasp_task_node.GraspTaskNode)
    node.latest_actuation_status = 'CONFIRMED:MEASURED_DIRECTIONAL_RESPONSE'
    monkeypatch.setattr(grasp_task_node.rospy, 'get_param', lambda k,d=None: True if k == '/gui/joint_direct_mode' else d)
    assert node._automatic_actuation_gate({}) == (False, 'MANUAL_CONTROL_ACTIVE: joint sliders own motion')
    assert not node._actuation_bootstrap_allowed(dict(use_grasp6d_plan=True, auto_confirm_actuation_from_observation=True))
    result = node._validate_bound_plan_locked(None, {})
    assert not result.ok and result.code == 'MANUAL_CONTROL_ACTIVE'


@pytest.mark.parametrize('method', ['handle_observation_actuation', 'handle_pose_strict_execute', 'handle_pose_strict_plan_execute', 'handle_pose', 'handle_pose_linear', 'handle_joints', 'handle_jog', 'handle_gripper'])
def test_manual_mode_rejects_every_automatic_motion_entry(method):
    gateway = gateway_tests.MotionGatewayControllerStartTest().make_gateway()
    gateway._gui_direct_mode = True
    response = getattr(gateway, method)(NS(execute=True))
    assert not response.success and response.message.startswith('MANUAL_CONTROL_ACTIVE')
    assert gateway.planner.calls == [] and gateway.controller_checks == 0


def test_manual_takeover_cancels_existing_moveit_trajectory_without_disabling_controller():
    gateway = gateway_tests.MotionGatewayControllerStartTest().make_gateway()
    events = []
    gateway.planner.ready = True
    gateway.planner.manipulator = NS(stop=lambda: events.append('cancel_moveit_trajectory'))
    gateway._gui_direct_mode = False
    gateway._gui_mode_cb(NS(data=True))
    gateway._gui_mode_cb(NS(data=True))
    gateway._gui_mode_cb(NS(data=False))
    assert events == ['cancel_moveit_trajectory']
    assert not gateway.positive_enable_requests and gateway.controller_checks == 0
