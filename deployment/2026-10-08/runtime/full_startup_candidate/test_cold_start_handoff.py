"""Pure identity/policy tests; no service or ROS operations."""
import importlib.util
from pathlib import Path
import pytest
spec=importlib.util.spec_from_file_location('cold_handoff',Path(__file__).with_name('cold_start_handoff.py'))
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)


@pytest.mark.parametrize('argv', [
    ['/usr/bin/python3','/opt/ros/noetic/bin/roslaunch','something.launch'],
    ['/bin/bash','/tmp/other.sh'], ['/bin/bash',m.ROOT_ENTRY,'extra']])
def test_live_roslaunch_or_unrelated_shell_cannot_stop_any_owner(argv):
    with pytest.raises(RuntimeError):m.validate_cold_parent(dict(MainPID='42',ActiveState='active',Type='simple'),42,argv)


def test_only_the_new_root_entry_parent_is_accepted():
    root=dict(MainPID='42',ActiveState='active',Type='simple')
    m.validate_cold_parent(root,42,['/bin/bash',m.ROOT_ENTRY])
    with pytest.raises(RuntimeError):m.validate_cold_parent(root,43,['/bin/bash',m.ROOT_ENTRY])


def test_changed_owner_command_cannot_be_stopped_by_name_alone():
    p=dict(LoadState='loaded',ActiveState='active',ExecStart='{ argv[]=/bin/other ; ignore_errors=no ; }')
    with pytest.raises(RuntimeError):m.validate_owner('reviewed.service',p,'/bin/reviewed')
    p['ExecStart']='{ argv[]=/bin/reviewed ; ignore_errors=no ; }'
    assert m.validate_owner('reviewed.service',p,'/bin/reviewed')
    p['ActiveState']='inactive'
    assert m.validate_owner('reviewed.service',p,'/bin/reviewed')
    p['LoadState']='not-found'
    assert not m.validate_owner('reviewed.service',p,'/bin/reviewed')


def test_default_is_review_only(monkeypatch,capsys):
    def forbidden():raise AssertionError('default performed handoff')
    monkeypatch.setattr(m,'prepare',forbidden)
    m.main([])
    assert 'review_only' in capsys.readouterr().out


@pytest.fixture
def handoff(monkeypatch, tmp_path):
    import json
    owners = {'alicia-task-postmove-20260929.service': '/bin/task',
              'alicia-accepted-tf-20260929.service': '/bin/tf'}
    file = tmp_path/'owners.json'; file.write_text(json.dumps(owners))
    monkeypatch.setattr(m, 'OWNERS', file)
    monkeypatch.setattr(m, 'cold_parent', lambda: {'ConsistsOf': ' '.join(owners)})
    rows = {unit: dict(LoadState='loaded', ActiveState='active', Job='',
        ExecStart='{ argv[]='+command+' ; ignore_errors=no ; }') for unit,command in owners.items()}
    monkeypatch.setattr(m, 'properties', lambda unit: rows[unit])
    calls = []
    def run(command, **kwargs):
        calls.append(command)
        for unit in command[3:]: rows[unit].update(ActiveState='inactive', Job='')
    monkeypatch.setattr(m.subprocess, 'run', run)
    return owners, rows, calls


def test_all_owners_are_validated_before_any_stop(handoff):
    owners, rows, calls = handoff
    rows[next(reversed(owners))]['ExecStart'] = '{ argv[]=/bin/unexpected ; ignore_errors=no ; }'
    with pytest.raises(RuntimeError, match='changed'): m.prepare()
    assert calls == []


def test_inactive_owner_with_pending_start_is_retired(handoff):
    owners, rows, calls = handoff
    rows[next(iter(owners))].update(ActiveState='inactive', Job='42 /org/freedesktop/systemd1/job/42')
    m.prepare()
    assert calls == [['systemctl', '--user', 'stop', *owners]]
    assert m.ROOT_UNIT not in calls[0]


def test_unexpected_root_child_prevents_all_service_operations(handoff, monkeypatch):
    owners, rows, calls = handoff
    monkeypatch.setattr(m, 'cold_parent', lambda: {'ConsistsOf': ' '.join(owners)+' unrelated.service'})
    with pytest.raises(RuntimeError, match='unlisted'): m.prepare()
    assert calls == []


def test_stop_failure_does_not_report_handoff_success(handoff, monkeypatch, capsys):
    def fail(*a, **kw): raise m.subprocess.TimeoutExpired('stop', 40.)
    monkeypatch.setattr(m.subprocess, 'run', fail)
    with pytest.raises(m.subprocess.TimeoutExpired): m.prepare()
    assert 'temporary_owners_retired' not in capsys.readouterr().out


def test_still_queued_owner_rejected(handoff, monkeypatch):
    owners, rows, calls = handoff
    monkeypatch.setattr(m.subprocess, 'run', lambda *a, **kw: None)
    with pytest.raises(RuntimeError, match='still active or queued'): m.prepare()


@pytest.mark.parametrize('service_type', ['notify', 'oneshot', 'forking'])
def test_root_startup_type_cannot_wait_for_child_after_itself(service_type):
    with pytest.raises(RuntimeError):
        m.validate_cold_parent(dict(MainPID='42', ActiveState='activating',
            Type=service_type), 42, ['/bin/bash', m.ROOT_ENTRY])


IDLE_CACHE_UNIT = 'alicia-idle-detector-cache-20260929.service'
IDLE_CACHE_COMMAND = ('/bin/bash /home/zhuyupei/alicia_wa_full/.worktrees/protocol-v3-upgrade/'
    '.runtime/model_pair_reload/start_consumers.sh '
    '/home/zhuyupei/alicia_wa_full/.worktrees/protocol-v3-upgrade/'
    '.runtime/model_pair_reload/carton_only.launch '
    '/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/'
    'idle_detector_repair_20260929T142135Z/ros')
COMPOSITE_PERCEPTION_UNIT = 'alicia-model-pair-perception-reload-resume-20260929T045300Z.service'


@pytest.fixture
def registry_handoff(monkeypatch):
    import json
    owners = json.loads(Path(__file__).with_name('legacy_owner_commands.json').read_text())
    root_children = set(owners) | {IDLE_CACHE_UNIT, COMPOSITE_PERCEPTION_UNIT}
    monkeypatch.setattr(m, 'cold_parent', lambda: {'ConsistsOf': ' '.join(sorted(root_children))})
    rows = {unit: dict(LoadState='loaded', ActiveState='active', Job='',
        ExecStart='{ argv[]='+command+' ; ignore_errors=no ; }') for unit,command in owners.items()}
    rows[IDLE_CACHE_UNIT] = dict(LoadState='loaded', ActiveState='active', Job='',
        ExecStart='{ argv[]='+IDLE_CACHE_COMMAND+' ; ignore_errors=no ; }')
    monkeypatch.setattr(m, 'properties', lambda unit: rows[unit])
    calls = []
    def run(command, **kwargs):
        calls.append(command)
        for unit in command[3:]: rows[unit].update(ActiveState='inactive', Job='')
    monkeypatch.setattr(m.subprocess, 'run', run)
    return owners, rows, calls


@pytest.mark.parametrize('state,job', [('active', ''),
    ('inactive', '42 /org/freedesktop/systemd1/job/42')])
def test_new_carton_owner_and_old_composite_are_both_retired(registry_handoff, state, job):
    owners, rows, calls = registry_handoff
    rows[IDLE_CACHE_UNIT].update(ActiveState=state, Job=job)
    m.prepare()
    assert len(calls) == 1
    assert calls[0][:3] == ['systemctl', '--user', 'stop']
    assert IDLE_CACHE_UNIT in calls[0][3:]
    assert COMPOSITE_PERCEPTION_UNIT in calls[0][3:]
    assert m.ROOT_UNIT not in calls[0]


def test_changed_carton_owner_command_prevents_all_handoff(registry_handoff):
    owners, rows, calls = registry_handoff
    rows[IDLE_CACHE_UNIT]['ExecStart'] = '{ argv[]=/bin/unexpected ; ignore_errors=no ; }'
    with pytest.raises(RuntimeError, match='changed'): m.prepare()
    assert calls == []
