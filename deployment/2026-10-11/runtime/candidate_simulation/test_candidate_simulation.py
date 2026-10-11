"""Pure offline contracts: never bind a socket or contact ROS/WSL."""
import copy
import importlib.util
import io
import json
from pathlib import Path
import sys
import types
import xml.etree.ElementTree as ET
import pytest

HERE = Path(__file__).resolve().parent
spec = importlib.util.spec_from_file_location('candidate_simulation_under_test', HERE/'run_candidate.py')
C = importlib.util.module_from_spec(spec)
spec.loader.exec_module(C)


def test_default_review_never_loads_engine_or_serves(monkeypatch, capsys):
    def fail(*a, **kw):
        raise AssertionError('default review attempted runtime entry')
    monkeypatch.setattr(C, 'load_engine', fail)
    monkeypatch.setattr(C, 'serve', fail)
    monkeypatch.setattr(C, 'storage_guard', fail)
    assert C.main([]) == 0
    assert json.loads(capsys.readouterr().out)['status'] == 'prepared_not_started'


def test_real_static_pins_and_exact_meshdir_only():
    pins = C.verify_pins()
    src = Path(pins['source_candidate_xml']).read_text()
    dst = Path(pins['model_xml']).read_text()
    assert dst == C.service_xml(src, pins['meshdir'])
    a, b = ET.fromstring(src), ET.fromstring(dst)
    b.find('compiler').set('meshdir', a.find('compiler').get('meshdir'))
    assert ET.tostring(a) == ET.tostring(b)
    assert not pins['physics_overrides']


@pytest.mark.parametrize('key', ['candidate_model_sha256', 'candidate_handeye_sha256',
                                  'candidate_pair_sha256', 'source_sha256'])
def test_wrong_pair_pin_refused(monkeypatch, tmp_path, key):
    p = json.loads((HERE/'pins.json').read_text())
    p[key] = '0'*64
    (tmp_path/'pins.json').write_text(json.dumps(p))
    monkeypatch.setattr(C, 'HERE', tmp_path)
    with pytest.raises(ValueError, match='frozen candidate'):
        C.verify_pins()


def test_changed_pinned_content_refused(monkeypatch, tmp_path):
    p = json.loads((HERE/'pins.json').read_text())
    changed = tmp_path/'changed.txt'
    changed.write_text('changed')
    p['files'][str(changed)] = '0'*64
    (tmp_path/'pins.json').write_text(json.dumps(p))
    monkeypatch.setattr(C, 'HERE', tmp_path)
    with pytest.raises(ValueError, match='pinned file changed'):
        C.verify_pins()


@pytest.mark.parametrize('xml,mesh', [('<mujoco/>','/a'),
    ('<mujoco><compiler meshdir="x"/><compiler meshdir="x"/></mujoco>','/a'),
    ('<mujoco><compiler meshdir="x"/></mujoco>','relative')])
def test_meshdir_rewrite_rejects_ambiguous_inputs(xml, mesh):
    with pytest.raises(ValueError):
        C.meshdir_only(xml, mesh)


def test_absent_external_mount_refuses_without_creating_path(monkeypatch, tmp_path):
    monkeypatch.setattr(C, 'MOUNT', tmp_path)
    monkeypatch.setattr(C, 'ARCHIVE', tmp_path/'ros_log')
    with pytest.raises(RuntimeError, match='no fallback'):
        C.storage_guard(tmp_path/'ros_log'/'new')
    assert not (tmp_path/'ros_log').exists()


def test_system_disk_output_refused():
    with pytest.raises(RuntimeError):
        C.storage_guard('/tmp/candidate_simulation_must_not_exist')


def test_compilation_requires_explicit_external_output():
    with pytest.raises(SystemExit) as exc:
        C.main(['--verify-only'])
    assert exc.value.code == 2


def test_listen_requires_explicit_external_output():
    with pytest.raises(SystemExit) as exc:
        C.main(['--serve'])
    assert exc.value.code == 2


def test_reload_manifest_contract_without_network():
    spec = importlib.util.spec_from_file_location('reload_contract', HERE.parent/'model_pair_reload/reload_pair.py')
    R = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(R)
    m = json.loads((HERE/'simulation_manifest.json').read_text())
    assert R.simulation_manifest(m) == m
    assert m['server_url'] == 'http://127.0.0.1:8002'
    assert m['health_bindings']['digital_twin.max_snapshot_age_sec'] == 120


def fake_engine():
    class Handler:
        def do_POST(self):
            self.native_calls += 1
            payload = self._read_payload()
            self._send_json(200, {'original_payload': payload})
        def _send_json(self, status, payload):
            self.sent.append((status, payload))
        def do_GET(self):
            self._send_json(200, {'digital_twin': {'ok': True}})
    return types.SimpleNamespace(MujocoDigitalTwinHTTPHandler=Handler)


def request_handler(monkeypatch, tmp_path, path, raw=b'{}'):
    monkeypatch.setattr(C, 'storage_guard', lambda p: Path(p))
    h = object.__new__(C.handler_class(fake_engine()))
    h.server = types.SimpleNamespace(archive=tmp_path)
    h.command, h.path = 'POST', path
    h.headers = {'Content-Length': str(len(raw))}
    h.rfile, h.sent, h.native_calls = io.BytesIO(raw), [], 0
    return h


@pytest.mark.parametrize('endpoint', ['/sync_joint_state', '/predict', '/hardware', '/simulate_grasp?x=1'])
def test_only_exact_simulation_route_reaches_engine(monkeypatch, tmp_path, endpoint):
    h = request_handler(monkeypatch, tmp_path, endpoint)
    h.do_POST()
    assert h.native_calls == 0
    assert h.sent[0][0] == 404
    assert (h.record_directory/'response.json').exists()


def test_simulation_archive_contains_exact_payload_and_response(monkeypatch, tmp_path):
    raw = b'{"plan_id":"offline", "snapshot_stamp_sec": 123.45}'
    h = request_handler(monkeypatch, tmp_path, '/simulate_grasp', raw)
    h.do_POST()
    assert h.native_calls == 1
    assert (h.record_directory/'request.raw.json').read_bytes() == raw
    assert json.loads((h.record_directory/'response.json').read_text()) == h.sent[0][1]
    assert h.sent[0][1]['original_payload']['snapshot_stamp_sec'] == 123.45


def test_storage_failure_never_calls_simulation(monkeypatch, tmp_path):
    h = request_handler(monkeypatch, tmp_path, '/simulate_grasp')
    monkeypatch.setattr(C, 'storage_guard', lambda p: (_ for _ in ()).throw(RuntimeError('disk absent')))
    h.do_POST()
    assert h.native_calls == 0 and h.sent[0][0] == 503


def test_runtime_defaults_and_original_health_contract_are_exact():
    pins = C.verify_pins()
    old = json.loads(Path(pins['old_contract_source']).read_text())['readonly_health']['digital_twin']
    for key, value in pins['simulation_contract'].items():
        assert value == old[key]
    source = (HERE/'run_candidate.py').read_text()
    assert 'max_snapshot_age_sec=120.0' in source
    assert 'ros_sync_joint_states=False' in source
    assert '--host' not in source and '--port' not in source


def test_shell_does_not_add_serve_or_ros_environment():
    text = (HERE/'start_candidate.sh').read_text()
    commands = '\n'.join(x for x in text.splitlines() if not x.lstrip().startswith('#'))
    assert '--serve' not in commands
    assert 'roslaunch' not in commands and 'setup.bash' not in commands
    assert '/usr/bin/python3' in commands and 'PYTHONNOUSERSITE=1' in commands


def test_only_known_nonsemantic_header_is_removed():
    pins = C.verify_pins()
    source = Path(pins['source_candidate_xml']).read_text()
    runtime = C.service_xml(source, pins['meshdir'])
    assert runtime.startswith('<mujoco ')
    with pytest.raises(ValueError, match='exact legacy review comment'):
        C.service_xml('<!-- unrelated -->\n'+source, pins['meshdir'])
    # Exercise the original options/actuator injectors, never a replacement.
    sys.path.insert(0, str(C.ROOT/'tools'))
    import mujoco_digital_twin_server as engine
    with pytest.raises(ET.ParseError):
        ET.fromstring(engine._inject_options(source))
    ready = engine._inject_actuators(engine._inject_options(runtime))
    root = ET.fromstring(ready)
    assert root.find('option').attrib == {'integrator': 'implicitfast', 'cone': 'elliptic'}
    assert len(root.find('actuator')) == 8
