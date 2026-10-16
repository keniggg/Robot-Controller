"""The mode CLI selects policy only; tests use inert ROS functions."""
from pathlib import Path
import runpy
import sys
from unittest import mock

import pytest
import rospy

SCRIPT = Path(__file__).resolve().parents[1] / 'scripts/switch_grasp_mode.py'


def test_carton_observe_return_rejected_before_ros_init(monkeypatch):
    init = mock.Mock()
    monkeypatch.setattr(rospy, 'init_node', init)
    monkeypatch.setattr(sys, 'argv', [str(SCRIPT), 'carton', '--strategy', 'observe_return'])
    with pytest.raises(SystemExit) as exc:
        runpy.run_path(str(SCRIPT), run_name='__main__')
    assert exc.value.code == 2
    init.assert_not_called()


def test_unknown_observe_return_cli_only_selects_requested_strategy(monkeypatch):
    monkeypatch.setattr(sys, 'argv', [str(SCRIPT), 'unknown', '--strategy', 'observe_return'])
    monkeypatch.setattr(rospy, 'init_node', mock.Mock())
    wait = mock.Mock()
    monkeypatch.setattr(rospy, 'wait_for_service', wait)
    service = mock.Mock(return_value=mock.Mock(success=True, message='SWITCHING'))
    proxy = mock.Mock(return_value=service)
    monkeypatch.setattr(rospy, 'ServiceProxy', proxy)
    with pytest.raises(SystemExit) as exc:
        runpy.run_path(str(SCRIPT), run_name='__main__')
    assert exc.value.code == 0
    wait.assert_called_once_with('/grasp_mode/select', timeout=10.)
    assert proxy.call_count == 1 and proxy.call_args.args[0] == '/grasp_mode/select'
    service.assert_called_once_with('unknown', 'observe_return')
