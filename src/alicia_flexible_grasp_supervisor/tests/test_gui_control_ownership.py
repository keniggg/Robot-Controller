from types import SimpleNamespace as NS
import pathlib
import sys

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / 'src'))

import pytest
from gui.widgets.joint_control_widget import JointControlWidget, DirectCommandGesture, DirectControlRecovery
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
    def isSliderDown(self): return False


def widget(checked):
    w = JointControlWidget.__new__(JointControlWidget)
    w.names = ['Joint1', 'Joint2']
    w.sliders = [Slider(100), Slider(200)]
    w.realtime_direct = NS(isChecked=lambda: checked)
    w.show_feedback_angles = False
    w.current_state = None
    w.labels = [NS(setText=lambda text: None) for _ in w.names]
    w._syncing_sliders = False
    w._pending_direct_edits = {}
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


def direct_widget(monkeypatch):
    w = widget(True)
    w._alive = True
    w._pending_direct_publish = False
    w._active_direct_slider_index = None
    w._direct_gesture = DirectCommandGesture()
    w.direct_recovery = DirectControlRecovery()
    w.direct_recovery.update_feedback_ready(True)
    w.direct_recovery.update_actuation_status('CONFIRMED:MEASURED_DIRECTIONAL_RESPONSE')
    sent = []
    w.pub = NS(publish=lambda msg: sent.append(msg), get_num_connections=lambda: 1)
    w.enable_pub = NS(publish=lambda msg: sent.append(('enable', msg.data)))
    w.status = NS(setText=lambda text: None)
    w.command_conn_chip = NS(setText=lambda text: None)
    monkeypatch.setattr(widget_module.rospy.Time, 'now', lambda: widget_module.rospy.Time(123))
    return w, sent


def test_two_sliders_in_one_flush_publish_both_one_hot_intents(monkeypatch):
    w, sent = direct_widget(monkeypatch)
    w.sliders[0].current = 800
    w._queue_direct_edit(0)
    w.sliders[1].current = -600
    w._queue_direct_edit(1)
    w.sliders[0].current = 900
    w._queue_direct_edit(0)  # Only coalesce J1's older sample.
    w.flush_direct_publish()
    assert len(sent) == 2
    assert [list(msg.effort) for msg in sent] == [[1., 0.], [0., 1.]]
    assert sent[0].position[0] == .9 and sent[1].position[1] == -.6
    assert all(msg.header.frame_id == 'gui_direct' for msg in sent)
    assert not w._pending_direct_edits


def test_switching_slider_does_not_erase_prior_target_display(monkeypatch):
    w, _ = direct_widget(monkeypatch)
    w.current_state = NS(name=w.names, position=[.01, .02])
    w.sliders[0].current = 800
    w._begin_direct_slider_gesture(0)
    w._end_direct_slider_gesture()
    w._begin_direct_slider_gesture(1)
    assert [s.value() for s in w.sliders] == [800, 200]


def test_second_axis_waits_for_first_bounded_recovery_then_is_published(monkeypatch):
    w, sent = direct_widget(monkeypatch)
    now = [10.0]
    monkeypatch.setattr(widget_module.time, 'monotonic', lambda: now[0])
    w.direct_recovery.update_actuation_status('UNCONFIRMED:ENCODER_RESPONSE_TIMEOUT')
    w._queue_direct_edit(0)
    w._queue_direct_edit(1)
    w.flush_direct_publish()
    assert sent == [('enable', False)]
    assert w.direct_recovery.pending_edit == (True, 0)
    assert list(w._pending_direct_edits) == [1]
    now[0] = 10.1
    w.direct_recovery.update_actuation_status('PENDING:POSITIVE_ENABLE_REQUESTED')
    w.direct_recovery.note_joint_feedback([0., 0.], 10.1)
    w.flush_direct_publish()
    assert sent[-1].header.frame_id == 'gui_direct_sync'
    w.direct_recovery.update_actuation_status('PENDING:COMMAND_SYNCHRONIZED')
    w.flush_direct_publish()
    assert sent[-1].position == [.025, 0.]
    assert sent[-1].effort == [1., 0.]
    w.direct_recovery.update_actuation_status('CONFIRMED:MEASURED_DIRECTIONAL_RESPONSE')
    w.flush_direct_publish()
    assert [msg.effort for msg in sent[-2:]] == [[1., 0.], [0., 1.]]
    assert sent[-2].position[0] == .1 and sent[-1].position[1] == .2
    assert not w._pending_direct_edits


def test_recovery_timeout_discards_all_queued_axes_without_disabling(monkeypatch):
    w, sent = direct_widget(monkeypatch)
    now = [10.0]
    monkeypatch.setattr(widget_module.time, 'monotonic', lambda: now[0])
    w.direct_recovery.update_actuation_status('UNCONFIRMED:ENCODER_RESPONSE_TIMEOUT')
    w._queue_direct_edit(0)
    w._queue_direct_edit(1)
    w.flush_direct_publish()
    now[0] = 12.1
    w.flush_direct_publish()
    assert not w._pending_direct_edits
    assert not w.direct_recovery.pending_edit[0]
    assert sent == [('enable', False)]


def test_expired_unsent_slider_is_not_replayed(monkeypatch):
    w, sent = direct_widget(monkeypatch)
    now = [10.0]
    monkeypatch.setattr(widget_module.time, 'monotonic', lambda: now[0])
    w._queue_direct_edit(0)
    now[0] = 12.1
    w.flush_direct_publish()
    assert not sent and not w._pending_direct_edits


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


@pytest.mark.parametrize('initial, requested', [(True, False), (False, True)])
def test_remote_mode_change_refreshes_gui_latch_once_without_echo_loop(monkeypatch, initial, requested):
    w = widget(initial)
    checked = [initial]
    blocked = [False]
    checkbox_events = []
    def block_signals(value):
        previous, blocked[0] = blocked[0], value
        return previous
    def set_checked(value):
        assert blocked[0], 'remote checkbox synchronization must not emit toggled'
        checked[0] = value
        checkbox_events.append(value)
    w.realtime_direct = NS(isChecked=lambda: checked[0],
                           blockSignals=block_signals, setChecked=set_checked)
    w.plan_btn = Slider()
    w.exec_btn = Slider()
    w._direct_gesture = DirectCommandGesture()
    w._pending_direct_publish = True
    w._pending_direct_edits = {0: 'unsent old slider target'}
    events = []
    w.direct_recovery = NS(cancel=lambda: events.append('cancel_slider'))
    w.status = NS(setText=lambda text: None)
    w.switch_trajectory_controllers = lambda *_a: pytest.fail('no controller switch')
    w.pub = NS(publish=lambda *_a: pytest.fail('no joint target on ownership change'))
    w.enable_pub = NS(publish=lambda *_a: pytest.fail('no enable/disable on ownership change'))
    latched = [initial]
    def publish(msg):
        latched[0] = msg.data
        events.append(('mode', msg.data))
        # Model the GUI receiving its own newly latched publication. The
        # checkbox already matches, so even synchronous echo must be a no-op.
        w._apply_remote_control_mode(msg.data)
    w.control_mode_pub = NS(publish=publish)
    monkeypatch.setattr(widget_module.rospy, 'set_param',
                        lambda key, value: events.append((key, value)))

    w._apply_remote_control_mode(requested)
    assert latched[0] is requested  # What a newly connected driver receives.
    assert events.count(('mode', requested)) == 1
    assert events.count(('/gui/joint_direct_mode', requested)) == 1
    assert checkbox_events == [requested] and not blocked[0]
    if not requested:
        assert not w._pending_direct_publish and not w._pending_direct_edits
    before = list(events)
    w._apply_remote_control_mode(requested)
    w._apply_remote_control_mode(requested)
    assert events == before and checkbox_events == [requested]


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


def reset_aware_widget(monkeypatch):
    w, sent = direct_widget(monkeypatch)
    w._display_feedback_ready = True
    w._display_actuation_status = 'CONFIRMED:MEASURED_DIRECTIONAL_RESPONSE'
    w._display_sync_required = False
    w._display_sync_manual = False
    w._display_sync_after_sec = 0.0
    w.messages = []
    w.status = NS(setText=w.messages.append)
    return w, sent


def feedback(w, positions, stamp=122.5, names=None):
    return NS(name=list(w.names if names is None else names), position=positions,
              header=NS(stamp=widget_module.rospy.Time.from_sec(stamp)))


def test_power_recovery_replaces_old_absolute_slider_baseline_without_motion(monkeypatch):
    w, sent = reset_aware_widget(monkeypatch)
    w.names = ['Joint%d' % i for i in range(1, 7)]
    # Recorded September 26 startup angles retained by the old GUI.
    w.sliders = [Slider(value) for value in [-5, 1302, -15, -3, -416, -6]]
    w.labels = [NS(setText=lambda text: None) for _ in w.names]
    w._pending_direct_edits[1] = ([0., 1.1, 0., 0., -.416, 0.], 10.)
    w._apply_feedback_ready(False, 122.1)
    w._apply_feedback_ready(True, 122.2)
    assert not w._pending_direct_edits
    measured = [-.003, -.0107, -.015, -.005, -.0107, -.006]
    w.update_current_state(feedback(w, measured))
    assert [s.value() for s in w.sliders] == [-3, -11, -15, -5, -11, -6]
    assert not w._display_sync_required and not sent
    assert all(not slider.emitted for slider in w.sliders)
    # A fresh leftward edit is now below the measured J2 angle, not the old
    # +1.302 rad target, and only its explicitly edited axis is admitted.
    w.sliders[1].current = -40
    w._queue_direct_edit(1)
    w.flush_direct_publish()
    assert len(sent) == 1 and sent[0].position[1] < measured[1]
    assert sent[0].effort == [0., 1., 0., 0., 0., 0.]
    # Ordinary confirmed manual feedback must still leave the target intact.
    w.update_current_state(feedback(w, [-.003, -.025, -.015, -.005, -.011, -.006], 122.6))
    assert w.sliders[1].value() == -40


def test_recovery_during_drag_requires_explicit_sync_and_never_replays_release(monkeypatch):
    w, sent = reset_aware_widget(monkeypatch)
    down = [True]
    w.sliders[1].isSliderDown = lambda: down[0]
    w.sender = lambda: w.sliders[1]
    w._begin_direct_slider_gesture(1)
    w._queue_direct_edit(1)
    w._apply_feedback_ready(False, 122.1)
    w._apply_feedback_ready(True, 122.2)
    w.update_current_state(feedback(w, [-.02, -.03]))
    assert w._display_sync_manual
    assert [s.value() for s in w.sliders] == [100, 200]
    down[0] = False
    w.handle_slider_released()
    w.update_current_state(feedback(w, [-.02, -.03], 122.6))
    w.flush_direct_publish()
    assert w._display_sync_required and not sent
    assert [s.value() for s in w.sliders] == [100, 200]
    assert not w.direct_recovery.pending_edit[0] and not w._pending_direct_edits
    w.sync_current_state()
    assert [s.value() for s in w.sliders] == [-20, -30]
    assert not w._display_sync_required and not sent


@pytest.mark.parametrize('stamp,positions,names', [
    (122.1, [.01, .02], None),  # Before the recovered-ready event.
    (121.9, [.01, .02], None),  # Stale.
    (123.1, [.01, .02], None),  # Future timestamp.
    (0., [.01, .02], None),
    (122.5, [.01], ['Joint1']),
    (122.5, [.01, float('nan')], None),
])
def test_display_reset_rejects_stale_or_incomplete_feedback(monkeypatch, stamp, positions, names):
    w, sent = reset_aware_widget(monkeypatch)
    w._apply_feedback_ready(False, 122.1)
    w._apply_feedback_ready(True, 122.2)
    w.update_current_state(feedback(w, positions, stamp, names))
    w.sync_current_state()
    assert w._display_sync_required
    assert [s.value() for s in w.sliders] == [100, 200] and not sent


def test_positive_enable_reset_discards_old_handshake_without_wait_cycle(monkeypatch):
    w, sent = reset_aware_widget(monkeypatch)
    w.direct_recovery.update_actuation_status('UNCONFIRMED:ENCODER_RESPONSE_TIMEOUT')
    w.publish_direct(target=[.1, 1.3], edited_index=1)
    assert sent == [('enable', False)] and w.direct_recovery.pending_edit[0]
    w._apply_actuation_status('PENDING:POSITIVE_ENABLE_REQUESTED', 122.2)
    assert not w.direct_recovery.pending_edit[0]
    # No active drag: receiving a new valid sample permits display-only sync,
    # without waiting for CONFIRMED (which would itself require a new goal).
    w.update_current_state(feedback(w, [-.01, -.02]))
    assert not w._display_sync_required
    assert [s.value() for s in w.sliders] == [-10, -20]
    w.flush_direct_publish()
    assert sent == [('enable', False)]
    # Repeated latched status must not erase normal new explicit input.
    w._queue_direct_edit(0)
    w._apply_actuation_status('PENDING:POSITIVE_ENABLE_REQUESTED', 122.7)
    assert not w._display_sync_required and 0 in w._pending_direct_edits


def test_edit_before_recovery_sample_cannot_publish_stale_absolute_target(monkeypatch):
    w, sent = reset_aware_widget(monkeypatch)
    w._apply_feedback_ready(False, 122.1)
    w._apply_feedback_ready(True, 122.2)
    w.publish_direct(target=[.1, 1.3], edited_index=1)
    w._queue_direct_edit(1)
    w.flush_direct_publish()
    assert not sent and not w._pending_direct_edits
    w.update_current_state(feedback(w, [-.01, -.02]))
    assert w._display_sync_required and w._display_sync_manual
    w.sync_current_state()
    assert not w._display_sync_required and not sent


@pytest.mark.parametrize('status', [
    'DISABLED:NOT_REQUESTED',
    'UNCONFIRMED:ENCODER_FEEDBACK_NOT_READY',
    'UNCONFIRMED:SERIAL_RECONNECTED_TORQUE_UNCHANGED',
    'UNCONFIRMED:DISCONTINUOUS_FEEDBACK_RECOVERY',
])
def test_driver_feedback_epoch_change_resynchronizes_without_ready_false(monkeypatch, status):
    w, sent = reset_aware_widget(monkeypatch)
    w._apply_actuation_status(status, 122.2)
    assert w._display_sync_required
    w.update_current_state(feedback(w, [-.01, -.02]))
    assert not w._display_sync_required
    assert [s.value() for s in w.sliders] == [-10, -20] and not sent


def test_gui_startup_waits_for_ready_and_new_complete_sample(monkeypatch):
    w, sent = reset_aware_widget(monkeypatch)
    w._display_feedback_ready = None
    w._display_sync_required = True
    w.update_current_state(feedback(w, [.8, .9], 122.1))
    assert [s.value() for s in w.sliders] == [100, 200]
    w._apply_feedback_ready(True, 122.2)
    assert w._display_sync_required
    w.update_current_state(feedback(w, [-.01, -.02]))
    assert not w._display_sync_required
    assert [s.value() for s in w.sliders] == [-10, -20] and not sent
