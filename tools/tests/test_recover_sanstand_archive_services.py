"""Late-mount recovery uses fake systemctl only; no real service operations."""
import importlib.util
import json
from pathlib import Path
import subprocess
from types import SimpleNamespace

import pytest

MODULE = Path(__file__).resolve().parents[1] / 'recover_sanstand_archive_services.py'
spec = importlib.util.spec_from_file_location('recover_sanstand_services', MODULE)
recover = importlib.util.module_from_spec(spec)
spec.loader.exec_module(recover)


@pytest.fixture(autouse=True)
def isolated_control(tmp_path, monkeypatch):
    monkeypatch.setattr(recover, 'PAUSE_STATUS', tmp_path / 'storage_control.json')
    recover.PAUSE_STATUS.write_text(json.dumps({'schema': 1, 'state': 'resumed', 'archive_policy': 'github_verified'}))


def runner(states, calls, loaded=True):
    def invoke(arguments):
        calls.append(arguments)
        if arguments[0] == 'show':
            return SimpleNamespace(stdout='LoadState=%s\nActiveState=%s\n' % (
                'loaded' if loaded else 'not-found', states[arguments[1]]))
        return SimpleNamespace(stdout='')
    return invoke


def test_absent_mount_makes_no_service_calls_and_creates_no_paths(tmp_path):
    calls = []
    result = recover.resume_services(tmp_path / 'unmounted', tmp_path / 'missing/status.json',
                                    runner({}, calls), mount_check=lambda: False)
    assert result['mounted'] is False and calls == []
    assert list(tmp_path.iterdir()) == [recover.PAUSE_STATUS]


def test_late_mount_starts_inactive_and_failed_but_preserves_running_workers(tmp_path):
    states = dict(zip(recover.SERVICES, ['active', 'inactive', 'failed', 'activating']))
    calls = []
    result = recover.resume_services(tmp_path, tmp_path / 'none.json', runner(states, calls), lambda: True)
    assert result['started'] == list(recover.SERVICES[1:3])
    assert ['reset-failed', recover.SERVICES[2]] in calls
    assert not any(call[0] in ('restart', 'stop', 'kill') for call in calls)
    assert [call[-1] for call in calls if call[0] == 'start'] == list(recover.SERVICES[1:3])


def test_all_live_services_are_unchanged(tmp_path):
    calls = []
    states = dict.fromkeys(recover.SERVICES, 'active')
    result = recover.resume_services(tmp_path, tmp_path / 'none.json', runner(states, calls), lambda: True)
    assert result['started'] == [] and len(calls) == 4
    assert all(call[0] == 'show' for call in calls)


def test_completed_one_time_finisher_is_not_restarted(tmp_path):
    receipt = tmp_path / 'status.json'
    receipt.write_text(json.dumps({'schema': 1, 'phase': 'complete', 'complete': True,
                                  'scope': 'frozen-experiments-through-20260927'}))
    calls = []
    result = recover.resume_services(tmp_path, receipt, runner(dict.fromkeys(recover.SERVICES, 'inactive'), calls), lambda: True)
    assert result['started'] == list(recover.SERVICES[:3])
    assert not any(recover.FINISH_SERVICE in call for call in calls)


def test_missing_unit_is_never_created_or_enabled(tmp_path):
    calls = []
    result = recover.resume_services(tmp_path, tmp_path / 'none', runner(dict.fromkeys(recover.SERVICES, 'inactive'), calls, False), lambda: True)
    assert result['started'] == [] and all(call[0] == 'show' for call in calls)


def test_mount_disappearing_before_start_leaves_service_unchanged(tmp_path):
    checks = iter([True, True, False])
    calls = []
    result = recover.resume_services(tmp_path, tmp_path / 'none', runner(dict.fromkeys(recover.SERVICES, 'inactive'), calls), lambda: next(checks))
    assert result['mounted'] is False and result['started'] == []
    assert len(calls) == 1 and calls[0][0] == 'show'


def test_service_start_failure_is_reported_and_other_storage_services_continue(tmp_path):
    calls = []
    base = runner(dict.fromkeys(recover.SERVICES, 'inactive'), calls)
    def invoke(arguments):
        result = base(arguments)
        if arguments[0] == 'start' and arguments[-1] == recover.SERVICES[0]:
            raise subprocess.CalledProcessError(1, arguments)
        return result
    result = recover.resume_services(tmp_path, tmp_path / 'none', invoke, lambda: True)
    assert len(result['failed']) == 1 and result['started'] == list(recover.SERVICES[1:])
    assert all('spool-cleanup' not in ' '.join(call) for call in calls)


@pytest.mark.parametrize('contents', [json.dumps({'schema': 1, 'state': 'paused'}), '{broken',
                                    json.dumps({'schema': 2, 'state': 'resumed'}), 'null'])
def test_paused_or_invalid_control_never_starts_services(tmp_path, contents):
    recover.PAUSE_STATUS.write_text(contents)
    calls = []
    result = recover.resume_services(tmp_path, tmp_path / 'none', runner({}, calls), lambda: True)
    assert result['paused'] is True and result['started'] == [] and calls == []


def test_resume_control_permits_recovery(tmp_path):
    recover.PAUSE_STATUS.write_text(json.dumps({'schema': 1, 'state': 'resumed', 'archive_policy': 'github_verified'}))
    calls = []
    result = recover.resume_services(tmp_path, tmp_path / 'none', runner(dict.fromkeys(recover.SERVICES, 'inactive'), calls), lambda: True)
    assert result['started'] == list(recover.SERVICES)


def test_pause_after_show_prevents_start(tmp_path):
    calls = []
    base = runner(dict.fromkeys(recover.SERVICES, 'inactive'), calls)
    def invoke(arguments):
        result = base(arguments)
        recover.PAUSE_STATUS.write_text(json.dumps({'schema': 1, 'state': 'paused'}))
        return result
    result = recover.resume_services(tmp_path, tmp_path / 'none', invoke, lambda: True)
    assert result['paused'] is True and result['started'] == [] and len(calls) == 1


def test_state_write_requires_mount_and_does_not_create_fallback(tmp_path):
    path = tmp_path / 'unmounted/maintenance/storage_control.json'
    with pytest.raises(RuntimeError, match='EXPERIMENT_STORAGE_UNAVAILABLE'):
        recover.set_control('paused', 'user pause', mount=tmp_path, path=path, mount_check=lambda: False)
    assert not path.parent.exists()


def test_state_write_round_trip_is_persistent(tmp_path):
    path = tmp_path / 'maintenance/storage_control.json'
    for state in ('paused', 'resumed'):
        result = recover.set_control(state, 'explicit user instruction', mount=tmp_path, path=path, mount_check=lambda: True)
        assert json.loads(path.read_text()) == result
        assert recover.control_status(path)['paused'] == (state == 'paused')
    assert list(path.parent.iterdir()) == [path]


def test_execcondition_rejects_pause_and_absent_mount(tmp_path, monkeypatch):
    monkeypatch.setattr('sys.argv', ['recover', '--check-enabled', '--unit', 'alicia-active-log-mirror.service'])
    monkeypatch.setattr(Path, 'is_mount', lambda _path: True)
    recover.PAUSE_STATUS.write_text(json.dumps({'schema': 1, 'state': 'paused'}))
    assert recover.main() == 1
    recover.PAUSE_STATUS.write_text(json.dumps({'schema': 1, 'state': 'resumed', 'archive_policy': 'github_verified'}))
    assert recover.main() == 0
    monkeypatch.setattr(Path, 'is_mount', lambda _path: False)
    assert recover.main() == 1


def test_missing_control_fails_closed(tmp_path):
    recover.PAUSE_STATUS.unlink()
    calls = []
    result = recover.resume_services(tmp_path, tmp_path / 'none', runner({}, calls), lambda: True)
    assert result['paused'] is True and result['reason'] == 'storage_control_missing'
    assert calls == []


def test_local_only_recovers_mirror_and_never_starts_archives(tmp_path):
    recover.PAUSE_STATUS.write_text(json.dumps({'schema': 1, 'state': 'resumed', 'archive_policy': 'local_only'}))
    calls = []
    result = recover.resume_services(tmp_path, tmp_path / 'none', runner(dict.fromkeys(recover.SERVICES, 'inactive'), calls), lambda: True)
    assert result['started'] == ['alicia-active-log-mirror.service']
    assert all('archive' not in ' '.join(call) for call in calls)


def test_legacy_resumed_marker_does_not_authorize_new_uploads(tmp_path):
    recover.PAUSE_STATUS.write_text(json.dumps({'schema': 1, 'state': 'resumed'}))
    state = recover.control_status()
    assert state['archive_policy'] == 'local_only'
    assert recover.service_allowed('alicia-active-log-mirror.service', state)
    assert not recover.service_allowed(recover.FINISH_SERVICE, state)


def test_pause_resume_preserves_local_only_policy(tmp_path):
    for state in ('paused', 'resumed'):
        result = recover.set_control(state, 'explicit user storage instruction', mount=tmp_path,
                                     mount_check=lambda: True, archive_policy='local_only' if state == 'paused' else None)
        assert result['archive_policy'] == 'local_only'


@pytest.mark.parametrize('unit,allowed', [(u, False) for u in recover.SERVICES if 'mirror' not in u] +
                         [('alicia-active-log-mirror.service', True), ('alicia-log-spool-cleanup.service', True), ('unknown.service', False)])
def test_local_only_execcondition_separates_storage_from_uploads(tmp_path, monkeypatch, unit, allowed):
    recover.PAUSE_STATUS.write_text(json.dumps({'schema': 1, 'state': 'resumed', 'archive_policy': 'local_only'}))
    monkeypatch.setattr('sys.argv', ['recover', '--check-enabled', '--unit', unit])
    monkeypatch.setattr(Path, 'is_mount', lambda _path: True)
    assert recover.main() == (0 if allowed else 1)
