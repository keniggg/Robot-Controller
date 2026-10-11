#!/usr/bin/env python3
"""Review by default; --verify-only compiles offline; --serve alone opens HTTP.

No ROS imports/synchronization, hardware endpoint, model fitting or physics
overrides. This wraps the pinned existing simulation implementation.
"""
import argparse
import contextlib
import hashlib
import importlib
import json
import os
from pathlib import Path
import re
import sys
import time
import traceback
import uuid
import xml.etree.ElementTree as ET

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
MOUNT = Path('/media/zhuyupei/SanStand')
ARCHIVE = MOUNT / '实验记录/Robot-Controller/ros_log'
HOST, PORT = '127.0.0.1', 8002
URDF_SHA = '7812d6b69e854ed9f37b4e3cbd813ecd53dab3504065b2d04d287abc6fd852e0'
X_SHA = '637701accca5eb5042075c213c5e00e9b189d3cd7788d916c8c06bf89f7b1a0c'
PAIR_SHA = '841ca1491fb2e05bced1a1f962749a496a9c4b9944d2833eefa2c3f1b02c3789'
SERVER_SHA = '5342482a2ccdff1e555bb914f6a049727892aec7a769f5c907faaff59621fb59'


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def meshdir_only(source, absolute_meshdir):
    """One textual replacement retains every other byte, including physics."""
    tree = ET.fromstring(source)
    if len(tree.findall('compiler')) != 1:
        raise ValueError('one compiler element required')
    old = tree.find('compiler').get('meshdir')
    if not old or not Path(absolute_meshdir).is_absolute():
        raise ValueError('existing meshdir and absolute replacement required')
    pattern = r'(<compiler\b[^>]*\bmeshdir=")' + re.escape(old) + r'(")'
    result, count = re.subn(pattern, lambda m: m[1] + str(absolute_meshdir) + m[2], source)
    if count != 1:
        raise ValueError('ambiguous compiler meshdir')
    return result


def service_xml(source, absolute_meshdir):
    # The unchanged legacy injector inserts <option> after the first '>'.
    # Remove only this pinned, non-semantic pre-root review note, never arbitrary
    # XML content. The original artifact/comment remains preserved and hashed.
    note = '<!-- REVIEW ONLY: retains legacy TCP/frame mismatch; not deployable equivalence -->\n'
    if not source.startswith(note):
        raise ValueError('expected exact legacy review comment')
    return meshdir_only(source[len(note):], absolute_meshdir)


def verify_pins():
    pins = json.loads((HERE / 'pins.json').read_text())
    expected = {'candidate_model_sha256': URDF_SHA,
                'candidate_handeye_sha256': X_SHA,
                'candidate_pair_sha256': PAIR_SHA,
                'source_sha256': SERVER_SHA}
    for key, value in expected.items():
        if pins.get(key) != value:
            raise ValueError('frozen candidate identity mismatch: ' + key)
    for filename, digest in pins['files'].items():
        if sha(filename) != digest:
            raise ValueError('pinned file changed: ' + filename)
    source = Path(pins['source_candidate_xml']).read_text()
    actual = Path(pins['model_xml']).read_text()
    if actual != service_xml(source, pins['meshdir']):
        raise ValueError('runtime MJCF changed beyond meshdir/exact review comment')
    if str(HERE / 'candidate_absolute_meshdir.xml') != pins['model_xml']:
        raise ValueError('unexpected runtime model location')
    return pins


def storage_guard(path):
    path = Path(path).resolve()
    if not MOUNT.is_mount() or path == ARCHIVE.resolve() or ARCHIVE.resolve() not in path.parents:
        raise RuntimeError('SanStand mounted ros_log subdirectory required; no fallback')
    # Refuse a nested mount or symlink that resolves outside the selected disk.
    ancestor = path
    while not ancestor.exists():
        ancestor = ancestor.parent
    if ancestor.stat().st_dev != MOUNT.stat().st_dev:
        raise RuntimeError('output is not on SanStand filesystem')
    return path


def write_json(path, value):
    path = storage_guard(path)
    path.write_text(json.dumps(value, ensure_ascii=False, allow_nan=False, indent=2) + '\n')


def load_engine(pins):
    os.environ['MUJOCO_GL'] = 'disable'
    sys.dont_write_bytecode = True
    runtime = ROOT / '.runtime/mujoco-3.2.3'
    sys.path[:0] = [str(runtime), str(ROOT / 'tools')]
    engine = importlib.import_module('mujoco_digital_twin_server')
    mj = importlib.import_module('mujoco')
    if Path(engine.__file__).resolve() != ROOT / 'tools/mujoco_digital_twin_server.py':
        raise RuntimeError('unexpected simulation source module')
    if runtime.resolve() not in Path(mj.__file__).resolve().parents or mj.__version__ != '3.2.3':
        raise RuntimeError('pinned local MuJoCo 3.2.3 required')
    return engine


def build_backend(engine, pins, runtime_model):
    class CandidateBackend(engine.MujocoDigitalTwinBackend):
        def __init__(self):
            # These are the only original constructor arguments overridden.
            super().__init__(model_xml=runtime_model, max_snapshot_age_sec=120.0,
                             ros_sync_joint_states=False)
            self.compiled_identity = None

        def health(self):
            try:
                current = verify_pins()
                if sha(self.model_xml) != current['model_xml_sha256']:
                    raise RuntimeError('actual loaded XML changed')
                result = super().health()
                if self.compiled_identity != current['model_xml_sha256']:
                    raise RuntimeError('actual model compilation and gripper contract not complete')
                for key, expected in current['simulation_contract'].items():
                    if result.get(key) != expected:
                        raise RuntimeError('original simulation contract changed: ' + key)
                result.update({k: current[k] for k in (
                    'model_xml_sha256', 'source_sha256', 'candidate_model_sha256',
                    'candidate_handeye_sha256', 'candidate_pair_sha256')})
                result.update(wrapper_sha256=sha(__file__),
                              compiled_model_xml_sha256=self.compiled_identity,
                              compiled_gripper_contract_passed=True,
                              initial_unknown_ik_position_tolerance_m=0.0002,
                              initial_unknown_ik_orientation_tolerance_rad=0.005,
                              network_scope='loopback_simulation_only',
                              candidate_pair_role='provenance; X is not applied to qpos or physics')
                return result
            except Exception as exc:
                return dict(ok=False, backend='mujoco', reason=str(exc))

        def simulate_grasp(self, payload):
            if not self.health().get('ok'):
                raise RuntimeError('candidate simulation pins/compiled contract changed')
            return super().simulate_grasp(payload)

    backend = CandidateBackend()
    backend._import_mujoco()
    if backend._mujoco.__version__ != '3.2.3':
        raise RuntimeError('MuJoCo version changed')
    # Compile through the REAL production injectors/cache/metadata and original
    # collision-mesh gripper contract. This fixture only compiles; no mj_step,
    # source timestamps, simulated grasp verdict or measured object is invented.
    fixture = dict(object_model=dict(type='box', size_xyz_m=[0.020, 0.014, 0.030],
                                    mass_kg=0.020, friction=[1.0, 0.1, 0.1]))
    model, _, _ = backend._model_for_payload(fixture)
    verify_pins()  # Files must still match after the compiler consumed them.
    backend.compiled_identity = pins['model_xml_sha256']
    health = backend.health()
    if health.get('ok') is not True:
        raise RuntimeError('compiled candidate backend unhealthy: ' + str(health))
    return backend, dict(compiled=True, original_gripper_contract_passed=True,
                         nq=model.nq, nv=model.nv, nu=model.nu,
                         fixture_scope='compilation only; no grasp/lift verdict', health=health)


def handler_class(engine):
    class CandidateHandler(engine.MujocoDigitalTwinHTTPHandler):
        def begin_record(self):
            directory = storage_guard(self.server.archive / 'requests' /
                                      ('%d_%s' % (time.time_ns(), uuid.uuid4().hex)))
            directory.mkdir(parents=True)
            self.record_directory = directory
            self.request_started = time.monotonic()
            write_json(directory / 'request_metadata.json',
                       dict(method=self.command, path=self.path, received_ns=time.time_ns()))

        def do_GET(self):
            try:
                self.begin_record()
                super().do_GET()
            except Exception as exc:
                engine.MujocoDigitalTwinHTTPHandler._send_json(self, 503, dict(ok=False, error=str(exc)))

        def do_POST(self):
            try:
                self.begin_record()
                if self.path != '/simulate_grasp':
                    self._send_json(404, dict(ok=False, error='simulation-only endpoint'))
                    return
                super().do_POST()
            except Exception as exc:
                engine.MujocoDigitalTwinHTTPHandler._send_json(self, 503, dict(ok=False, error=str(exc)))

        def _read_payload(self):
            data = self.rfile.read(int(self.headers.get('Content-Length', '0')))
            storage_guard(self.record_directory / 'request.raw.json').write_bytes(data)
            return json.loads(data.decode('utf-8'))

        def _send_json(self, status, payload):
            write_json(self.record_directory / 'response.json', payload)
            write_json(self.record_directory / 'timing.json',
                       dict(http_status=status, elapsed_sec=time.monotonic() - self.request_started))
            super()._send_json(status, payload)

    return CandidateHandler


def serve(engine, backend, output):
    class UnavailableInference:
        name = 'not_configured'
        def health(self):
            return dict(ok=False, backend=self.name, loaded=False,
                        reason='Dedicated real MuJoCo; existing WSL GraspNet remains on port 8000.')
    # Intentionally not engine.make_server: its handler exposes joint-cache
    # synchronization and inference. Neither endpoint belongs to this process.
    server = engine.ThreadingHTTPServer((HOST, PORT), handler_class(engine))
    server.archive = storage_guard(output)
    server.sim_backend = backend
    server.grasp_backend = UnavailableInference()
    server.max_snapshot_age_sec = 120.0
    print('READY_CANDIDATE_SIMULATION', json.dumps(backend.health()), flush=True)
    try:
        server.serve_forever()
    finally:
        server.server_close()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument('--serve', action='store_true')
    mode.add_argument('--verify-only', action='store_true')
    parser.add_argument('--output', type=Path)
    args = parser.parse_args(argv)
    pins = verify_pins()
    if not args.serve and not args.verify_only:
        print(json.dumps(dict(status='prepared_not_started', execution_ready=False,
                              server_url='http://127.0.0.1:8002',
                              candidate_pair_sha256=PAIR_SHA,
                              reason='Existing user authorization; prospective/model-pair readiness is separate.'), indent=2))
        return 0
    if args.output is None:
        parser.error('--serve/--verify-only require a NEW SanStand --output directory')
    output = storage_guard(args.output)
    output.mkdir(parents=True, exist_ok=False)
    with (output / 'service.log').open('x', buffering=1) as logfile:
        saved_fds = [os.dup(1), os.dup(2)]
        try:
            # Capture native diagnostics as well as Python output on SanStand.
            os.dup2(logfile.fileno(), 1)
            os.dup2(logfile.fileno(), 2)
            with contextlib.redirect_stdout(logfile), contextlib.redirect_stderr(logfile):
                try:
                    # The unmodified compiler changes cwd to this XML parent.
                    runtime_model = output / 'loaded_candidate.xml'
                    storage_guard(runtime_model).write_bytes(Path(pins['model_xml']).read_bytes())
                    os.chdir(str(output))
                    engine = load_engine(pins)
                    backend, report = build_backend(engine, pins, runtime_model)
                    write_json(output / 'startup_verification.json', report)
                    if args.verify_only:
                        return 0
                    serve(engine, backend, output)
                except Exception:
                    traceback.print_exc()
                    return 1
        finally:
            logfile.flush()
            for fd, saved in zip((1, 2), saved_fds):
                os.dup2(saved, fd)
                os.close(saved)

    return 0


if __name__ == '__main__':
    raise SystemExit(main())
