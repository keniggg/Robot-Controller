"""Audit writer storage boundaries, using exact function bodies without ROS."""
import ast
import hashlib
import json
import os
from pathlib import Path
import sys
import tempfile
import threading
from types import SimpleNamespace

import pytest


ROOT = Path(__file__).resolve().parents[1]
SUPPORT = ROOT.parents[1] / 'tools/tests'
if str(SUPPORT) not in sys.path:
    sys.path.insert(0, str(SUPPORT))
from storage_os_adapter import storage_os


class AuditError(RuntimeError):
    def __init__(self, code, reason):
        self.code, self.reason = code, reason
        super().__init__(code + ': ' + reason)


@pytest.fixture(params=('planning', 'execution'))
def writer(request):
    planning = request.param == 'planning'
    source = ROOT / 'scripts' / ('remote_grasp6d_node.py' if planning else 'grasp_task_node.py')
    names = ({'_audit_storage_check', 'validate_mandatory_planning_audit',
              'validate_distinct_audit_paths', '_write_gate_audit_report'} if planning else
             {'_audit_storage_check', 'normalize_audit_output_path', 'write_mujoco_execution_audit'})
    tree = ast.parse(source.read_bytes())
    functions = [node for node in ast.walk(tree)
                 if isinstance(node, ast.FunctionDef) and node.name in names]
    assert {node.name for node in functions} == names
    namespace = dict(os=os, json=json, hashlib=hashlib, tempfile=tempfile,
        threading=threading, CandidateContractError=AuditError,
        StreamResultCancelled=type('StreamResultCancelled', (RuntimeError,), {}),
        PLANNING_AUDIT_CONFIG_INVALID='PLANNING_AUDIT_CONFIG_INVALID',
        PLANNING_AUDIT_WRITE_FAILED='PLANNING_AUDIT_WRITE_FAILED',
        AUDIT_PATH_CONFLICT='AUDIT_PATH_CONFLICT',
        MUJOCO_AUDIT_DEFAULT_PATH='unused_mujoco_audit.json')
    exec(compile(ast.Module(body=functions, type_ignores=[]), str(source), 'exec'), namespace)

    def write(path, report=None):
        report = {'rows': [], 'retained': 'original evidence'} if report is None else report
        if planning:
            node = SimpleNamespace(gate_audit_output_path=str(path), gate_audit_enabled=True,
                mujoco_audit_output_path=str(path)+'.execution', _stream_condition=threading.RLock())
            return namespace['_write_gate_audit_report'](node, report)
        return namespace['write_mujoco_execution_audit'](str(path), report)
    return write


@pytest.fixture
def disks(storage_os):
    """Use the production selector/validator; only OS mount identity is synthetic."""
    result = {}
    for name in ('ssd', 'sanstand'):
        selected = storage_os.select(name)
        result[name] = SimpleNamespace(base=selected.base, backend=name,
            identity=selected.validate(selected.base), state=storage_os.states[name])
    return result


@pytest.mark.parametrize('backend', ('ssd', 'sanstand'))
def test_runtime_audit_persists_on_selected_backend(writer, disks, monkeypatch, backend):
    disk = disks[backend]
    monkeypatch.setenv('ALICIA_STORAGE_IDENTITY', json.dumps(disk.identity))
    monkeypatch.setenv('ALICIA_STORAGE_BACKEND', backend)
    path = disk.base / 'new_session' / 'audit.json'
    reference = writer(path)
    assert json.loads(path.read_bytes())['retained'] == 'original evidence'
    assert reference.get('sha256', reference.get('report_sha256')) == hashlib.sha256(path.read_bytes()).hexdigest()
    assert not list(disks['sanstand' if backend == 'ssd' else 'ssd'].base.iterdir())


def test_missing_runtime_mount_rejects_before_directory_creation(writer, disks, monkeypatch):
    disk = disks['ssd']
    monkeypatch.setenv('ALICIA_STORAGE_IDENTITY', json.dumps(disk.identity))
    disk.state['mounted'] = False
    path = disk.base / 'not_created' / 'audit.json'
    with pytest.raises(Exception, match='STORAGE_UNAVAILABLE'):
        writer(path)
    assert not path.parent.exists()


@pytest.mark.parametrize('identity', ('', 'null', '[]', '{broken'))
def test_malformed_runtime_identity_is_not_an_offline_fallback(writer, disks, monkeypatch, identity):
    monkeypatch.setenv('ALICIA_STORAGE_IDENTITY', identity)
    path = disks['ssd'].base / 'not_created' / 'audit.json'
    with pytest.raises(Exception):
        writer(path)
    assert not path.parent.exists()


def test_cross_backend_runtime_path_is_rejected(writer, disks, monkeypatch):
    monkeypatch.setenv('ALICIA_STORAGE_IDENTITY', json.dumps(disks['ssd'].identity))
    monkeypatch.setenv('ALICIA_STORAGE_BACKEND', 'ssd')
    path = disks['sanstand'].base / 'not_created' / 'audit.json'
    with pytest.raises(Exception, match='STORAGE_UNAVAILABLE'):
        writer(path)
    assert not path.parent.exists()


def test_mount_change_after_mkdir_creates_no_audit_file(writer, disks, monkeypatch):
    disk = disks['ssd']
    monkeypatch.setenv('ALICIA_STORAGE_IDENTITY', json.dumps(disk.identity))
    path = disk.base / 'new_session' / 'audit.json'
    original = os.makedirs

    def mkdir_then_remount(name, *args, **kwargs):
        original(name, *args, **kwargs)
        if Path(name) == path.parent:
            disk.state['mount_id'] += 1

    monkeypatch.setattr(os, 'makedirs', mkdir_then_remount)
    with pytest.raises(Exception, match='identity changed'):
        writer(path)
    assert not list(path.parent.iterdir())


@pytest.mark.parametrize('inherited', (False, True))
def test_mount_change_before_commit_preserves_previous_audit(writer, disks, monkeypatch, inherited):
    disk = disks['ssd']
    if inherited:
        monkeypatch.setenv('ALICIA_STORAGE_IDENTITY', json.dumps(disk.identity))
    path = disk.base / 'audit.json'
    path.write_bytes(b'previous audit bytes')
    original = os.fsync

    def sync_then_remount(fd):
        original(fd)
        disk.state['mount_id'] += 1

    monkeypatch.setattr(os, 'fsync', sync_then_remount)
    with pytest.raises(Exception, match='identity changed'):
        writer(path)
    assert path.read_bytes() == b'previous audit bytes'
    # An identity change must not authorize cleanup at a replaced mount path.
    assert any('tmp-' in p.name for p in disk.base.iterdir())


def test_unbound_managed_path_still_validates_disk(writer, disks):
    disk = disks['ssd']; disk.state['mounted'] = False
    path = disk.base / 'not_created' / 'audit.json'
    with pytest.raises(Exception, match='STORAGE_UNAVAILABLE'):
        writer(path)
    assert not path.parent.exists()


def test_unbound_offline_path_retains_original_write_behavior(writer, disks, storage_os):
    path = storage_os.sandbox / 'offline' / 'audit.json'
    for disk in disks.values():
        disk.state['mounted'] = False
    writer(path)
    assert json.loads(path.read_bytes())['retained'] == 'original evidence'


@pytest.mark.parametrize('direction', ('managed_to_offline', 'offline_to_managed'))
def test_managed_symlink_cannot_become_unchecked_offline_path(writer, disks, storage_os, direction):
    offline = storage_os.sandbox / 'offline'; offline.mkdir()
    if direction == 'managed_to_offline':
        link, target = disks['ssd'].base/'alias', offline
    else:
        link, target = offline/'alias', disks['sanstand'].base
    link.symlink_to(target, target_is_directory=True)
    with pytest.raises(Exception, match='STORAGE_UNAVAILABLE'):
        writer(link/'audit.json')
    assert not (target/'audit.json').exists()


def test_invalid_configuration_cannot_make_managed_path_offline(writer, disks, storage_os):
    path = disks['ssd'].base / 'not_created' / 'audit.json'
    storage_os.config.write_text('{invalid')
    with pytest.raises(Exception):
        writer(path)
    assert not path.parent.exists()
