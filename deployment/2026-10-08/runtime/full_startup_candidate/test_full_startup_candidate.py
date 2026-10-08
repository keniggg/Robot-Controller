"""Pure file/fake-transport checks. Never starts ROS, a server, or systemd."""
import configparser
import hashlib
import importlib.util
import io
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest

HERE = Path(__file__).resolve().parent
spec = importlib.util.spec_from_file_location('staged_full_startup', HERE / 'candidate_simulation_startup.py')
m = importlib.util.module_from_spec(spec)
spec.loader.exec_module(m)


@pytest.fixture
def storage(tmp_path, monkeypatch):
    mount = tmp_path / 'mounted_disk'
    mount.mkdir()
    monkeypatch.setattr(m, 'MOUNT', mount)
    monkeypatch.setattr(m, 'STORAGE_ROOT', mount / 'experiments/ros_log')
    monkeypatch.setattr(m.os.path, 'ismount', lambda p: Path(p) == mount)
    return mount


def healthy(manifest):
    result = {'ok': False, 'digital_twin': {'ok': True}}
    for dotted, value in manifest['health_bindings'].items():
        parts = dotted.split('.')
        target = result
        for key in parts[:-1]:
            target = target.setdefault(key, {})
        target[parts[-1]] = value
    return result


def test_default_is_only_review(monkeypatch, capsys):
    def forbidden(*a, **kw):
        raise AssertionError('default path touched runtime')
    for name in ('run_server', 'wait_ready', 'reserve_output', 'require_storage'):
        monkeypatch.setattr(m, name, forbidden)
    monkeypatch.setattr(m.urllib.request, 'urlopen', forbidden)
    monkeypatch.setattr(m.os, 'execv', forbidden)
    assert m.main([]) == 0
    assert json.loads(capsys.readouterr().out)['status'] == 'staged_review_only'


def test_each_launch_reserves_unique_parent_and_nonexistent_service_output(storage):
    first, output1 = m.reserve_output(m.frozen_manifest())
    (first / 'preserve.raw').write_bytes(b'original experiment')
    second, output2 = m.reserve_output(m.frozen_manifest())
    assert first != second
    assert output1.parent == first and output2.parent == second
    assert not output1.exists() and not output2.exists()
    assert (first / 'preserve.raw').read_bytes() == b'original experiment'
    assert json.loads((second / 'startup_request.json').read_text())['output'] == str(output2)


def test_missing_mount_rejects_without_creating_storage(storage, monkeypatch):
    monkeypatch.setattr(m.os.path, 'ismount', lambda p: False)
    with pytest.raises(RuntimeError, match='not mounted'):
        m.reserve_output(m.frozen_manifest())
    assert not m.STORAGE_ROOT.exists()


def test_symlink_escape_rejected(storage, tmp_path, monkeypatch):
    outside = tmp_path / 'outside'
    outside.mkdir()
    link = storage / 'escape'
    link.symlink_to(outside, target_is_directory=True)
    monkeypatch.setattr(m, 'STORAGE_ROOT', link / 'ros_log')
    with pytest.raises(RuntimeError, match='escapes'):
        m.require_storage()
    assert not (outside / 'ros_log').exists()


def test_frozen_entry_change_is_rejected_before_output(tmp_path, monkeypatch):
    changed = tmp_path / 'changed.sh'
    changed.write_text('# changed\n')
    monkeypatch.setattr(m, 'START', changed)
    with pytest.raises(ValueError, match='start script changed'):
        m.frozen_manifest()


def test_manifest_physics_binding_change_is_rejected(tmp_path, monkeypatch):
    changed = json.loads(m.MANIFEST.read_text())
    changed['health_bindings']['digital_twin.max_snapshot_age_sec'] = 999.0
    path = tmp_path / 'changed_manifest.json'
    path.write_text(json.dumps(changed))
    monkeypatch.setattr(m, 'MANIFEST', path)
    with pytest.raises(ValueError, match='manifest changed'):
        m.frozen_manifest()


def test_serve_executes_only_frozen_wrapper_with_new_external_output(storage, monkeypatch):
    captured = []
    class Executed(Exception):
        pass
    def fake_exec(path, argv):
        captured.append((path, argv))
        raise Executed()
    monkeypatch.setattr(m.os, 'dup2', lambda *a: None)
    monkeypatch.setattr(m.os, 'execv', fake_exec)
    with pytest.raises(Executed):
        m.main(['--serve'])
    path, argv = captured[0]
    assert path == '/bin/bash'
    assert argv[:4] == ['/bin/bash', str(m.START), '--serve', '--output']
    assert storage in Path(argv[4]).parents and not Path(argv[4]).exists()
    assert (Path(argv[4]).parent / 'bootstrap.log').exists()


def test_real_manifest_and_dedicated_backend_health_match():
    manifest = m.frozen_manifest()
    m.validate_health(healthy(manifest), manifest)


@pytest.mark.parametrize('key', ['candidate_model_sha256', 'candidate_handeye_sha256', 'candidate_pair_sha256', 'model_xml_sha256'])
def test_wrong_loaded_pair_rejected(key):
    manifest = m.frozen_manifest()
    payload = healthy(manifest)
    payload['digital_twin'][key] = '0' * 64
    with pytest.raises(ValueError, match='mismatch'):
        m.validate_health(payload, manifest)


def test_not_ready_backend_rejected():
    manifest = m.frozen_manifest()
    payload = healthy(manifest)
    payload['digital_twin']['ok'] = False
    with pytest.raises(ValueError, match='not ready'):
        m.validate_health(payload, manifest)


def test_wait_ready_uses_only_fake_loopback_transport(storage, monkeypatch):
    manifest = m.frozen_manifest()
    calls = []
    def fake_open(url, timeout):
        calls.append((url, timeout))
        return io.BytesIO(json.dumps(healthy(manifest)).encode())
    monkeypatch.setattr(m.urllib.request, 'urlopen', fake_open)
    m.wait_ready()
    assert calls == [(m.URL + '/health', 1.0)]


def test_wait_ready_connection_failure_is_bounded(storage, monkeypatch):
    now = [0.]
    monkeypatch.setattr(m.time, 'monotonic', lambda: now[0])
    monkeypatch.setattr(m.time, 'sleep', lambda dt: now.__setitem__(0, now[0] + dt))
    def unavailable(*a, **kw):
        raise OSError('not listening')
    monkeypatch.setattr(m.urllib.request, 'urlopen', unavailable)
    with pytest.raises(TimeoutError):
        m.wait_ready(timeout=.4)
    assert now[0] == pytest.approx(.4)


def test_staged_launch_preserves_existing_configuration_and_single_owners():
    manifest = json.loads((HERE / 'staging_manifest.json').read_text())
    launch_path = next(Path(p) for p in manifest['source_entry_sha256'] if p.endswith('.launch'))
    original = ET.parse(manifest.get('baseline_files', {}).get(str(launch_path), str(launch_path))).getroot()
    staged = ET.parse(HERE / 'resume_modes.staged.launch').getroot()
    # The simulator URL moves to the overlay. The user's temporary close/hold
    # policy is the only later change to this staged baseline; keep all other
    # driver/enable/GUI/camera/limits and policy XML byte-equivalent.
    original.remove(next(e for e in original if e.get('name') == '/mujoco_digital_twin/server_url'))
    lift = next(e for e in original if e.get('name') == '/grasp/unknown_lift_after_close_enabled')
    assert lift.get('value') == 'true'
    lift.set('value', 'false')
    last = staged[-1]
    assert last.tag == 'include' and last.get('file') == str(HERE / 'candidate_pair_and_simulation.launch')
    staged.remove(last)
    for tree in (original, staged):
        for element in tree.iter():
            if element.text is not None and not element.text.strip():
                element.text = None
            if element.tail is not None and not element.tail.strip():
                element.tail = None
    assert ET.tostring(original) == ET.tostring(staged)
    overlay = ET.parse(HERE / 'candidate_pair_and_simulation.launch').getroot()
    assert not overlay.findall('.//node')
    assert overlay[-1].attrib == {'name': '/mujoco_digital_twin/server_url', 'value': m.URL}
    assert 'candidate_parameters.launch' in overlay[0].get('file')
    assert 'camera_only.launch' not in (HERE / 'resume_modes.staged.launch').read_text()
    assert 'unknown_only.launch' not in (HERE / 'resume_modes.staged.launch').read_text()


def test_active_entries_remain_byte_identical_to_recorded_sources():
    manifest = json.loads((HERE / 'staging_manifest.json').read_text())
    expected_entries = manifest.get('deployed_entry_sha256') or manifest['source_entry_sha256']
    for path, expected in expected_entries.items():
        assert hashlib.sha256(Path(path).read_bytes()).hexdigest() == expected
    for source, baseline in manifest.get('baseline_files', {}).items():
        assert hashlib.sha256(Path(baseline).read_bytes()).hexdigest() == manifest['source_entry_sha256'][source]


def test_reviewed_unit_dependencies_and_entry_simulation_start():
    unit = configparser.ConfigParser(interpolation=None, strict=False)
    unit.read(HERE / 'alicia-candidate-simulation.service')
    assert 'PartOf' not in unit['Unit']
    assert unit['Unit']['BindsTo'] == 'alicia-ros-live-20260928.service'
    assert unit['Unit']['After'] == 'alicia-ros-live-20260928.service'
    assert unit['Service']['Restart'] == 'no'
    assert unit['Service']['ExecStart'].endswith('candidate_simulation_startup.py --serve')
    assert 'Install' not in unit
    script = (HERE / 'start_ros.staged.sh').read_text()
    assert 'systemctl --user start alicia-candidate-simulation.service' in script
    assert script.index('--wait-ready') < script.index('exec roslaunch')
    assert ' stop ' not in script and ' restart ' not in script
    assert 'export GRASP6D_URL=http://172.23.132.97:8000' in script


@pytest.fixture(autouse=True)
def isolated_service_identity(monkeypatch):
    # All tests remain offline; never inspect the user's actual unit or sockets.
    monkeypatch.setattr(m, 'service_snapshot', lambda: dict(
        pid=1234, invocation_id='a'*32, control_group='/user/test/'+m.UNIT))
    monkeypatch.setattr(m, 'require_listener_owned_by', lambda _: None)


@pytest.mark.parametrize('change', [
    {'ActiveState':'failed'}, {'SubState':'exited'}, {'MainPID':'0'},
    {'InvocationID':''}, {'InvocationID':'g'*32}, {'ControlGroup':'/user/old.service'}])
def test_wrong_or_failed_service_identity_is_rejected(change):
    properties=dict(ActiveState='active',SubState='running',MainPID='1234',
                    InvocationID='a'*32,ControlGroup='/user/test/'+m.UNIT)
    properties.update(change)
    with pytest.raises((RuntimeError,ValueError)):
        m.validate_service_properties(properties)


def test_failed_new_unit_cannot_accept_old_healthy_server(storage,monkeypatch):
    called=[]
    def failed():raise RuntimeError('requested candidate simulation unit is not running')
    monkeypatch.setattr(m,'service_snapshot',failed)
    monkeypatch.setattr(m.urllib.request,'urlopen',lambda *a,**kw:called.append(True))
    with pytest.raises(RuntimeError,match='not running'):m.wait_ready()
    assert called==[]


def test_wrong_listener_rejected_even_when_old_health_matches(storage,monkeypatch):
    called=[]
    def other(_):raise RuntimeError('port 8002 listener belongs to another service')
    monkeypatch.setattr(m,'require_listener_owned_by',other)
    monkeypatch.setattr(m.urllib.request,'urlopen',lambda *a,**kw:called.append(True))
    with pytest.raises(RuntimeError,match='another service'):m.wait_ready()
    assert called==[]


def test_invocation_change_during_health_read_is_rejected(storage,monkeypatch):
    identity=[dict(pid=1234,invocation_id='a'*32,control_group='/user/test/'+m.UNIT)]
    monkeypatch.setattr(m,'service_snapshot',lambda:dict(identity[0]))
    def read(*a,**kw):
        identity[0]['invocation_id']='b'*32
        return io.BytesIO(json.dumps(healthy(m.frozen_manifest())).encode())
    monkeypatch.setattr(m.urllib.request,'urlopen',read)
    with pytest.raises(RuntimeError,match='changed during health'):m.wait_ready()
