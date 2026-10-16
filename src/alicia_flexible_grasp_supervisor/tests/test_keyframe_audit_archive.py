"""Exercise the passive archive without a ROS master or control interfaces."""
import ast
import hashlib
import json
from pathlib import Path

import pytest


SCRIPT = Path(__file__).resolve().parents[3] / 'tools/record_grasp_keyframes_readonly.py'
TREE = ast.parse(SCRIPT.read_text())
FUNCTION = next(n for n in TREE.body if isinstance(n, ast.FunctionDef)
                and n.name == 'archive_gate_audit')
SCOPE = dict(hashlib=hashlib, json=json, Path=Path)
exec(compile(ast.Module(body=[FUNCTION], type_ignores=[]), str(SCRIPT), 'exec'), SCOPE)


def test_audit_payload_is_bound_and_idempotent(tmp_path):
    source = tmp_path / 'latest.json'
    payload = b'{"source_stamp_ns":1789263207109177929}'
    source.write_bytes(payload)
    reference = json.dumps(dict(report_path=str(source),
                                report_sha256=hashlib.sha256(payload).hexdigest()))
    output = SCOPE['archive_gate_audit'](reference, tmp_path)
    assert output.read_bytes() == payload
    assert SCOPE['archive_gate_audit'](reference, tmp_path) == output
    source.write_bytes(b'{"replaced":true}')
    with pytest.raises(ValueError, match='hash mismatch'):
        SCOPE['archive_gate_audit'](reference, tmp_path)
    assert output.read_bytes() == payload


def test_recorder_has_no_ros_control_client():
    forbidden = {'Publisher', 'ServiceProxy', 'SimpleActionClient', 'set_param'}
    calls = {n.func.attr for n in ast.walk(TREE)
             if isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute)}
    assert not calls.intersection(forbidden)


def test_recorder_covers_topic_handoff_and_action_execution():
    literals = {n.value for n in ast.walk(TREE)
                if isinstance(n, ast.Constant) and isinstance(n.value, str)}
    assert {
        '/alicia_controller/command',
        '/alicia_controller/follow_joint_trajectory/goal',
        '/alicia_controller/follow_joint_trajectory/result',
        '/alicia_d/control_reference', '/alicia_d/control_reference_epoch',
        '/alicia_d/sdk_command', '/alicia_d/accepted_joint_states',
        '/alicia_d/sdk_diagnostics', '/alicia_d/device_info',
    } <= literals
