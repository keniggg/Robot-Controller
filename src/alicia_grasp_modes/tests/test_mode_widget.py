"""Real offscreen Qt widget with inert ROS transport; never opens a ROS node."""
import json
import os
from pathlib import Path
import sys
import time
from types import SimpleNamespace

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'alicia_flexible_grasp_supervisor'))

import pytest
from PyQt5 import QtWidgets
from alicia_grasp_modes import mode_widget


@pytest.fixture(scope='module')
def app():
    instance = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    yield instance


@pytest.fixture
def widget(app, monkeypatch):
    removed = []
    calls = []
    monkeypatch.setattr(mode_widget.rospy, 'Subscriber',
        lambda *a, **k: SimpleNamespace(unregister=lambda: removed.append(True)))
    monkeypatch.setattr(mode_widget.rospy, 'wait_for_service', lambda *a, **k: None)
    def proxy(name, service):
        def send(*args):
            calls.append((name, args))
            return SimpleNamespace(success=True, message='SWITCHING')
        return send
    monkeypatch.setattr(mode_widget.rospy, 'ServiceProxy', proxy)
    instance = mode_widget.GraspModeWidget()
    yield instance, calls, removed
    instance.close()
    app.processEvents()


def status(widget, **changes):
    value = dict(mode='unknown', strategy='two_stage', generation=7,
                 state='ready', active=False, start_reserved=False, pending='')
    value.update(changes)
    widget._update_status(json.dumps(value))


def test_no_connection_never_offers_optimistic_selection(widget):
    view, calls, _ = widget
    assert not view.apply_button.isEnabled()
    view.apply_selection()
    assert not calls


def test_atomic_request_contains_both_choices_and_never_starts_motion(widget, app):
    view, calls, _ = widget
    status(view)
    view.strategy.setCurrentIndex(view.strategy.findData('direct'))
    view.apply_selection()
    deadline = time.monotonic() + 2.
    while view._requesting and time.monotonic() < deadline:
        app.processEvents()
    assert not view._requesting
    assert calls == [('/grasp_mode/select', ('unknown', 'direct'))]


def test_execution_and_service_reservation_lock_both_selectors(widget):
    view, _, _ = widget
    status(view, start_reserved=True)
    assert not view.target.isEnabled()
    assert not view.strategy.isEnabled()
    assert not view.apply_button.isEnabled()
    status(view, active=True)
    assert not view.apply_button.isEnabled()
    status(view)
    assert view.apply_button.isEnabled()


def test_status_updates_other_panels_but_keeps_unapplied_user_choices(widget):
    view, _, _ = widget
    status(view, strategy='direct')
    assert view.strategy.currentData() == 'direct'
    view.target.setCurrentIndex(view.target.findData('carton'))
    status(view, strategy='two_stage')
    assert view.target.currentData() == 'carton'
    assert '未知小物体' in view.status.text()
    assert '两阶段' in view.status.text()


def test_stale_heartbeat_and_teardown_do_not_issue_robot_commands(widget):
    view, calls, removed = widget
    status(view)
    view._last_receipt = time.monotonic() - 5.
    view._refresh_controls()
    assert not view.apply_button.isEnabled()
    assert '中断' in view.status.text()
    view._shutdown_ros()
    view._shutdown_ros()
    assert removed == [True]
    assert calls == []
