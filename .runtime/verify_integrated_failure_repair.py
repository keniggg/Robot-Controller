#!/usr/bin/env python3
"""Frozen declared offline scope, with a private kernel sandbox per test file.

Case collection, every execution phase, actual project imports and external
fixture identities are retained. A pilot is never a complete-scope release.
"""
import argparse
from collections import Counter
from concurrent.futures import ThreadPoolExecutor, as_completed
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import time

W = Path(__file__).resolve().parents[1]
E = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
BASE = E/'ros_log/full_latest_gen41_restart_20261018T011222Z'
RUN = BASE/'gen58_resume_r1/observation_pose_fix/held_lattice_fix_r1/seed_wait_fix_r1/contact_feedback_fix_r1'
R = RUN/'actual_gen63_baseline_validation_20261023T041149Z/high_accuracy_grasp_r1/arrival_margin_repair_r1'
RUN_STORAGE = None
RUN_STORAGE_BINDING = None
CORE = ['src/alicia_grasp_modes', 'src/alicia_flexible_grasp_supervisor']
DERIVED_PARTS = {'__pycache__', '.git', '.pytest_cache', '.mypy_cache', '.ruff_cache',
                 'htmlcov', '.tox', '.venv'}
DERIVED_NAMES = {'.coverage', 'coverage.xml'}
DEFAULT_PROCESS_TIMEOUT_SEC = 300.0
# Offline process budget only: this 14-case file took 295.44 s in r21;
# r22's matching 13 completed cases were 13.33% slower. No motion deadline changes.
PROCESS_TIMEOUT_OVERRIDES_SEC = {
    'src/alicia_grasp_modes/tests/test_gen41_pending_handoff_chain.py': 450.0,
}
EXCLUDED = {
    'src/alicia_flexible_grasp_supervisor/tests/test_object_pose.py': 'manual ROS diagnostic, no pytest cases; must not run script',
    'src/alicia_flexible_grasp_supervisor/tests/test_tactile_read.py': 'manual ROS diagnostic, no pytest cases; must not run script',
    'src/arm-mujoco/test/test_paths.py': 'return-code script, not an asserting pytest test; separate receipt required',
    'tools/diagnostics/test_camera_io_trace.py': 'import executes native fixture; separate isolated validation required',
    'tools/vscode_usage_monitor/test_core.py': 'unrelated editor usage monitor, outside grasp runtime scope',
    'tools/vscode_usage_monitor/test_restore_after_host_exit.py': 'unrelated editor usage monitor, outside grasp runtime scope',
    'src/easy_handeye/tests/test_infrastructure_validation.py': 'excluded_optional_development_environment: checks pytest-cov/pytest-mock availability, marker/config files and empty package imports; contains no calibration/grasp computation. Audited system Python lacks both optional plugins; original two failures are retained in astra_integrated_runner_pilot_r2.',
    'src/Alicia-D-SDK-use/tests/test_robocore_v250_compatibility.py': 'Separate Python >=3.11 Synria RoboCore >=2.5 SDK compatibility suite; dependencies absent from ROS Python3.8 and not imported by the configured ROS nodes. Original 32 failures retained in integrated_full_regression_r1; no SDK compatibility pass claimed.',
    'tools/tests/test_finish_sanstand_archive.py': 'Legacy session-specific archive finisher outside this workspace and current startup dependencies; user-disabled archival workflow. Original source-audit collection refusal retained in integrated_full_regression_r1; not imported or started for this release.',
}
HISTORICAL_SOURCES = {
    '.runtime/gen37_baseline_source/src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/robot/observation_path_guard.py': 'f3af52527035ad8211e785e3aeb14ecd596570bf236a989662547fd9ce73de9a',
    '.runtime/gen37_baseline_source/src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/robot/contact_sequence_precheck.py': 'dba703313e5bde18ef74b300f01d15f69132bc0ccbae2a3855eaf51e7e976f88',
    '.runtime/gen38_baseline_source/src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/vision/multiview_surface.py': '369a2947114c6c3953c439a7937cf26971e337c015e65abf96825720bc5c1ba8',
    '.runtime/gen4_root_cause/visual_snapshot_tap.py': 'f090991b50ee6116e9d912532f0a85933cfe04921e46c3921949664d3cb59519',
}
HTTP_FIXTURE_FILES = {
    'src/alicia_flexible_grasp_supervisor/tests/test_mujoco_digital_twin_server_protocol.py',
    'src/alicia_flexible_grasp_supervisor/tests/test_graspnet_baseline_server_protocol.py',
    'tools/tests/test_wsl_grasp_stack.py',
}
MUJOCO_RUNTIME_FILES = {
    'src/alicia_flexible_grasp_supervisor/tests/test_mujoco_arm_velocity_reference.py',
    'src/alicia_flexible_grasp_supervisor/tests/test_mujoco_closure_dynamics.py',
    'src/alicia_flexible_grasp_supervisor/tests/test_mujoco_digital_twin_server_protocol.py',
}
# Synthetic fixtures use private tmpfs in every worker. They are not deployment
# records on the host's read-only mount. Storage-specific tests explicitly
# model their OS mount boundary; every fixture is archived/read back below.
HISTORICAL_TEST = '.runtime/gen6_runner_prewarm_20261012/tests/test_gen6_initial_template_late_start.py'
PLUGIN = '''import json, os, tarfile, hashlib
from pathlib import Path
items=[]
reports=[]
collection=[]
deselected=[]
def pytest_collection_finish(session):
    items.extend(item.nodeid for item in session.items)
    Path(os.environ['CASE_INVENTORY']).write_text(json.dumps(items))
def pytest_collectreport(report):
    collection.append(dict(nodeid=report.nodeid,outcome=report.outcome,
                           reason=str(report.longrepr) if report.outcome!='passed' else None))
def pytest_deselected(items):
    deselected.extend(item.nodeid for item in items)
def pytest_runtest_logreport(report):
    reports.append(dict(nodeid=report.nodeid,when=report.when,outcome=report.outcome,duration=report.duration,
                        skip_reason=str(report.longrepr) if report.outcome=='skipped' else None,
                        wasxfail=getattr(report,'wasxfail',None)))
def pytest_sessionfinish(session,exitstatus):
    Path(os.environ['CASE_REPORT']).write_text(json.dumps(dict(pid=os.getpid(),items=items,reports=reports,
        collection=collection,deselected=deselected,testscollected=session.testscollected,exitstatus=int(exitstatus))))
    output=Path(os.environ['INTEGRATED_WORKER_DIR'])
    archive=output/'synthetic_fixtures.tar.gz'
    members={}
    def digest(stream):
        h=hashlib.sha256()
        for part in iter(lambda:stream.read(1024*1024),b''): h.update(part)
        return h.hexdigest()
    with tarfile.open(archive,'w:gz',dereference=False) as saved:
        for root in (Path('/tmp'),Path('/var/tmp')):
            for path in sorted(root.rglob('*')):
                name=str(path).lstrip('/')
                if path.is_file() and not path.is_symlink():
                    with path.open('rb') as source: members[name]=digest(source)
                saved.add(path,arcname=name,recursive=False)
    with tarfile.open(archive,'r:gz') as saved:
        for name,wanted in members.items():
            with saved.extractfile(name) as recovered: assert digest(recovered)==wanted,name
    with archive.open('rb') as source: archive_sha=digest(source)
    (output/'synthetic_fixture_archive.json').write_text(json.dumps(dict(
        path=str(archive),sha256=archive_sha,regular_file_sha256=members,readback_verified=True,
        storage='private tmpfs synthetic fixtures; result/log/archive files on the selected experiment disk')))
'''


def sha(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for chunk in iter(lambda:stream.read(1024*1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def write(path, data):
    raw = (json.dumps(data, ensure_ascii=False, indent=2, allow_nan=False)+'\n').encode()
    with Path(path).open('xb') as stream:
        stream.write(raw)
        stream.flush()
        os.fsync(stream.fileno())
    assert Path(path).read_bytes() == raw


def regular_files(folder):
    for path in folder.rglob('*'):
        if (path.is_file() and not DERIVED_PARTS.intersection(path.parts)
                and path.name not in DERIVED_NAMES and not path.name.startswith('.coverage.')
                and path.suffix not in ('.pyc', '.pyo')):
            yield path


def scope():
    discovered = {str(p.relative_to(W)) for root in ('src', 'tools')
                  for p in regular_files(W/root) if p.name.startswith('test_') and p.suffix == '.py'}
    roots = CORE+['tools/tests', 'src/real-arm/alicia_d_calibration/tests',
                 'src/Alicia-D-SDK-use/tests', 'src/easy_handeye/tests']
    included = {str(p.relative_to(W)) for root in roots for p in regular_files(W/root)
                if p.name.startswith('test_') and p.suffix == '.py'} - set(EXCLUDED)
    included.add(HISTORICAL_TEST)
    unexpected = discovered-included-set(EXCLUDED)
    if unexpected:
        raise RuntimeError('unreviewed test files: '+repr(sorted(unexpected)))
    return sorted(included), [dict(path=p, reason=why) for p, why in EXCLUDED.items() if p in discovered]


def source_manifest(tests):
    old = json.loads((BASE/'verification/gen63_release_manifest.json').read_bytes())['source_sha256']
    paths = {W/p for p in old} | {W/p for p in tests}
    # Include model meshes, message/service definitions and pytest configuration,
    # not just an extension allowlist. Generated Python ROS bindings are inputs too.
    for folder in ('src', 'tools', 'devel/lib/python3/dist-packages',
                   '.runtime/realsense-2.56.5', '.runtime/mujoco-3.2.3',
                   '.runtime/candidate_simulation', '.runtime/full_startup_candidate'):
        paths.update(regular_files(W/folder))
    paths.update(p for p in (W/'.runtime').iterdir()
                 if p.is_file() and p.suffix in ('.py', '.sh', '.launch'))
    contract = json.loads((RUN/'full_stack_restart_r3/contract.json').read_bytes())
    paths.update(Path(p) for p in contract['current_sha256'] if str(p).startswith(str(W)+'/'))
    paths.update((Path(__file__).resolve(), W/'.runtime/integrated_offline_guard.py',
                  W/'.runtime/integrated_import_audit.py', W/'.runtime/experiment_storage.json'))
    for name,digest in HISTORICAL_SOURCES.items():
        assert sha(W/name)==digest, 'historical comparator changed: '+name
        paths.add(W/name)
    return {str(p.relative_to(W)):sha(p) for p in sorted(paths)}


def _read_json(path, default, errors):
    try:
        return json.loads(path.read_bytes())
    except (OSError, ValueError) as exc:
        errors.append(dict(path=str(path), read_error=str(exc)))
        return default


def _read_lines(path, errors, required=False):
    if not path.exists() and not required:
        return []
    try:
        return [json.loads(line) for line in path.read_text().splitlines() if line]
    except (OSError, ValueError) as exc:
        errors.append(dict(path=str(path), read_error=str(exc)))
        return []


def run_file(index, test, output):
    if RUN_STORAGE is not None:
        RUN_STORAGE.validate(output, subtree=RUN_STORAGE.ros_log_root, expected=RUN_STORAGE_BINDING)
    process_timeout_sec = PROCESS_TIMEOUT_OVERRIDES_SEC.get(test, DEFAULT_PROCESS_TIMEOUT_SEC)
    d = output/('file_%04d'%index)
    d.mkdir()
    for name in ('bootstrap', 'tmp', 'ros', 'ros_home', 'cache', 'pycache', 'empty'):
        (d/name).mkdir()
    (d/'bootstrap/sitecustomize.py').write_bytes((W/'.runtime/integrated_offline_guard.py').read_bytes())
    (d/'bootstrap/handoff_import_audit.py').write_bytes((W/'.runtime/integrated_import_audit.py').read_bytes())
    (d/'bootstrap/integrated_cases.py').write_text(PLUGIN)
    bootstrap = {str(p.resolve()):sha(p) for p in (d/'bootstrap').iterdir()}
    write(d/'bootstrap_manifest.json', bootstrap)
    temporary_root = '/tmp'
    pytest_root = '/tmp/pytest-fixtures'
    env = dict(os.environ, PYTHONDONTWRITEBYTECODE='1', PYTHONNOUSERSITE='1',
        PYTHONPYCACHEPREFIX=str(d/'pycache'), PYTEST_DISABLE_PLUGIN_AUTOLOAD='1',
        OPENBLAS_NUM_THREADS='1', OMP_NUM_THREADS='1', MKL_NUM_THREADS='1', NUMEXPR_NUM_THREADS='1',
        LD_LIBRARY_PATH='/opt/ros/noetic/lib:/opt/ros/noetic/lib/x86_64-linux-gnu',
        ROS_MASTER_URI='http://127.0.0.1:1', ROS_LOG_DIR=str(d/'ros'), ROS_HOME=str(d/'ros_home'),
        TMPDIR=temporary_root, TMP=temporary_root, TEMP=temporary_root, XDG_CACHE_HOME=str(d/'cache'),
        OFFLINE_BOUNDARY_LOG=str(d/'boundary.jsonl'), OFFLINE_SANDBOX_LOG=str(d/'sandbox.jsonl'),
        OFFLINE_FIXTURE_NETWORK_LOG=str(d/'fixture_network.jsonl'),
        OFFLINE_ALLOW_OWN_LOOPBACK='1' if test in HTTP_FIXTURE_FILES else '0',
        INTEGRATED_SANDBOX='1', INTEGRATED_HOST_NET_NS=os.readlink('/proc/self/ns/net'),
        INTEGRATED_WORKSPACE_ROOT=str(W), INTEGRATED_PROJECT_ROOT=str(W.parents[1]),
        INTEGRATED_EXPERIMENT_ROOT=str(E), INTEGRATED_WORKER_DIR=str(d),
        INTEGRATED_EXPERIMENT_ROOTS=json.dumps([str(E), str(RUN_STORAGE.base)] if RUN_STORAGE else [str(E)]),
        INTEGRATED_GENERATED_ROOTS=json.dumps(['/tmp','/var/tmp']),
        INTEGRATED_SOURCE_MANIFEST=str(output/'manifest.json'),
        INTEGRATED_BOOTSTRAP_MANIFEST=str(d/'bootstrap_manifest.json'),
        CASE_REPORT=str(d/'cases.json'), CASE_INVENTORY=str(d/'collected_before_execution.json'),
        ROS_PACKAGE_PATH=str(W/'src')+':/opt/ros/noetic/share')
    for key in ('PYTEST_ADDOPTS', 'PYTEST_PLUGINS', 'PYTHONHOME', 'LD_PRELOAD', 'DISPLAY',
                'WAYLAND_DISPLAY', 'DBUS_SESSION_BUS_ADDRESS', 'ROS_IP', 'ROS_HOSTNAME', 'ROS_NAMESPACE',
                'HTTP_PROXY', 'HTTPS_PROXY', 'ALL_PROXY', 'http_proxy', 'https_proxy', 'all_proxy'):
        env.pop(key, None)
    env['PYTHONPATH'] = ':'.join(str(p) for p in (d/'bootstrap',
        W/'src/alicia_flexible_grasp_supervisor/tests', W/'src/alicia_grasp_modes/tests',
        W/'src/alicia_grasp_modes/src', W/'src/alicia_flexible_grasp_supervisor/src', W/'tools',
        W/'devel/lib/python3/dist-packages', Path('/opt/ros/noetic/lib/python3/dist-packages')))
    if test == 'src/real-arm/alicia_d_calibration/tests/test_charuco_runtime_model.py':
        env['PYTHONPATH'] += ':'+str(W/'.runtime/realsense-2.56.5')
        env['LD_LIBRARY_PATH'] = str(W/'.runtime/realsense-2.56.5/lib')+':'+env['LD_LIBRARY_PATH']
    if test in MUJOCO_RUNTIME_FILES:
        env['PYTHONPATH'] += ':'+str(W/'.runtime/mujoco-3.2.3')
        env['MUJOCO_SMOKE'] = '1'
        env['MUJOCO_GL'] = 'disable'
    command = ['/usr/bin/bwrap',
        '--die-with-parent', '--new-session', '--unshare-all', '--ro-bind', '/', '/',
        '--dev', '/dev', '--proc', '/proc', '--bind', str(d), str(d),
        '--tmpfs', '/tmp', '--tmpfs', '/var/tmp',
        '--ro-bind', str(d/'empty'), '/run', '--ro-bind', str(d/'empty'), '/sys',
        '--chdir', str(W), '/usr/bin/time', '-v', '-o', str(d/'resources.txt'),
        sys.executable, '-B', '-m', 'pytest', '-q',
        '-o', 'addopts=', '--rootdir='+str(W), '-p', 'no:cacheprovider',
        '-p', 'handoff_import_audit', '-p', 'integrated_cases',
        '--basetemp='+pytest_root, '--junitxml='+str(d/'pytest.xml'), '--tb=short', test]
    write(d/'command.json', dict(command=command, test_file=test,
        process_timeout_sec=process_timeout_sec,
        host_net_namespace=env['INTEGRATED_HOST_NET_NS'], python_cache_prefix=env['PYTHONPYCACHEPREFIX'],
        own_loopback_fixture_enabled=test in HTTP_FIXTURE_FILES,
        storage_binding=RUN_STORAGE_BINDING,
        synthetic_fixture_storage='private tmpfs; exact gzip archive and readback on selected experiment disk',
        isolation='bwrap: private network/pid/ipc/dev; root read-only; only selected-disk worker directory and private tmpfs synthetic fixtures writable'))
    started = time.monotonic()
    errors = []
    with (d/'pytest.log').open('w') as log:
        try:
            child = subprocess.Popen(command, cwd=W, env=env, stdout=log,
                                     stderr=subprocess.STDOUT, start_new_session=True)
            try:
                code = child.wait(timeout=process_timeout_sec)
            except subprocess.TimeoutExpired:
                os.killpg(child.pid, signal.SIGKILL)
                child.wait()
                code = 124
        except OSError as exc:
            code = 127
            errors.append(dict(process_error=str(exc)))
    cases = _read_json(d/'cases.json', dict(items=[], reports=[], collection=[]), errors)
    fixture_archive = _read_json(d/'synthetic_fixture_archive.json', {}, errors)
    if not fixture_archive.get('readback_verified') or sha(d/'synthetic_fixtures.tar.gz') != fixture_archive.get('sha256'):
        errors.append(dict(error='synthetic fixture archive not verified'))
    inventory = _read_json(d/'collected_before_execution.json', [], errors)
    boundary = _read_lines(d/'boundary.jsonl', errors)
    sandbox = _read_lines(d/'sandbox.jsonl', errors, required=True)
    imports = [_read_json(p, {}, errors) for p in sorted(d.glob('imports.*.json'))]
    source_events = [r for p in sorted(d.glob('source_events.*.jsonl')) for r in _read_lines(p, errors)]
    external_inputs = [r for p in sorted(d.glob('input_events.*.jsonl')) for r in _read_lines(p, errors)]
    return dict(path=test, output=str(d), exitcode=code, wall_sec=time.monotonic()-started,
        process_timeout_sec=process_timeout_sec,
        inventory=inventory, cases=cases, boundary=boundary, sandbox=sandbox, imports=imports,
        source_events=source_events, external_inputs=external_inputs, bootstrap=bootstrap, errors=errors,
        synthetic_fixture_archive=fixture_archive)


def valid_case(row):
    return row['teardown'] == 'passed' and (
        (row['setup'] == 'passed' and row['call'] in ('passed', 'skipped'))
        or (row['setup'] == 'skipped' and row['call'] is None))


def reconcile(rows, source):
    expected, results, boundaries, bad, collection_skips = [], [], [], [], []
    canonical = {str((W/p).resolve()):value for p, value in source.items()}
    external = {}
    for row in sorted(rows, key=lambda r:r['path']):
        inventory, cases = row['inventory'], row['cases']
        boundaries.extend(row['boundary'])
        problems = list(row['errors'])
        if (row['exitcode'] or cases.get('exitstatus') != row['exitcode']
                or inventory != cases.get('items') or not inventory
                or cases.get('testscollected') != len(inventory)):
            problems.append(dict(error='process/collection identity incomplete', exitcode=row['exitcode']))
        if cases.get('deselected'):
            problems.append(dict(error='cases deselected', cases=cases['deselected']))
        expected.extend(inventory)
        if any(not nodeid.startswith(row['path']+'::') for nodeid in inventory):
            problems.append(dict(error='collection nodeid belongs to a different file'))
        for collection in cases.get('collection', []):
            if collection.get('outcome') != 'passed':
                problems.append(dict(collection=collection))
                if collection.get('outcome') == 'skipped':
                    collection_skips.append(dict(path=row['path'], **collection))
        seen = set()
        by_case = {nodeid:dict(nodeid=nodeid, setup=None, call=None, teardown=None) for nodeid in inventory}
        for report in cases.get('reports', []):
            nodeid, phase, outcome = report.get('nodeid'), report.get('when'), report.get('outcome')
            key = (nodeid, phase)
            if nodeid not in by_case or phase not in ('setup', 'call', 'teardown') or key in seen:
                problems.append(dict(error='unexpected or duplicate execution report', report=report))
                continue
            seen.add(key)
            if outcome not in ('passed', 'failed', 'skipped'):
                problems.append(dict(error='unknown execution outcome', report=report))
            by_case[nodeid][phase] = outcome
            if outcome == 'skipped':
                if not report.get('skip_reason'):
                    problems.append(dict(error='missing skip reason', report=report))
                by_case[nodeid]['skip_reason'] = report.get('skip_reason')
            if report.get('wasxfail'):
                by_case[nodeid]['wasxfail'] = report['wasxfail']
        results.extend(by_case[nodeid] for nodeid in inventory)
        if (not row['sandbox'] or any(not r.get('private_namespace_verified')
                or r.get('net_namespace') == r.get('host_net_namespace') for r in row['sandbox'])):
            problems.append(dict(error='private namespace proof missing'))
        if not any(r.get('pid') == cases.get('pid') and r.get('complete') is True for r in row['imports']):
            problems.append(dict(error='main test interpreter provenance missing'))
        for imported in row['imports']:
            problems.extend(dict(import_violation=v) for v in imported.get('violations', []))
        events = row['source_events']
        if not events:
            problems.append(dict(error='actual project import/exec inventory missing'))
        if not any(event.get('path') == str((W/row['path']).resolve()) for event in events):
            problems.append(dict(error='collected test file absent from actual execution/import provenance'))
        for event in events:
            category, path = event.get('category'), event.get('path', '')
            if category == 'generated_fixture':
                if not path.startswith((row['output']+'/', '/tmp/', '/var/tmp/')):
                    problems.append(dict(error='generated source outside worker', source=event))
                continue
            wanted = row['bootstrap'].get(path) if category == 'runner_bootstrap' else canonical.get(path)
            if wanted is None or event.get('sha256') != wanted:
                problems.append(dict(error='actual source differs from frozen manifest', source=event))
        for identity in row['external_inputs']:
            path = identity['path']
            if path in external and identity != external[path]:
                problems.append(dict(error='external fixture identity changed between reads', path=path))
            external[path] = identity
        if problems:
            bad.append(dict(path=row['path'], output=row['output'], problems=problems))
    duplicates = [nodeid for nodeid, count in Counter(expected).items() if count != 1]
    for path, identity in external.items():
        try:
            st = Path(path).stat()
            current = dict(path=path, size=st.st_size, mtime_ns=st.st_mtime_ns, device=st.st_dev, inode=st.st_ino)
        except OSError as exc:
            current = dict(error=str(exc))
        if current != identity:
            bad.append(dict(error='external fixture changed after execution', before=identity, after=current))
    return expected, results, boundaries, bad, duplicates, collection_skips, list(external.values())


MAX_JOBS = 8  # Explicit user limit; this is not a measured hardware limit.


def parse_jobs(value):
    try:
        jobs = int(value)
    except (TypeError, ValueError):
        raise argparse.ArgumentTypeError('jobs must be an integer from 1 to 8')
    if not 1 <= jobs <= MAX_JOBS:
        raise argparse.ArgumentTypeError('jobs must be an integer from 1 to 8')
    return jobs


def allocate_jobs(requested, available_cpus, file_count):
    """Bound the user request by actual affinity and independent file count.

    Online CPU/memory/IO pressure is measured before launching a run; an
    explicit lower request records that decision instead of silently using 4.
    """
    if requested is not None and (type(requested) is not int or not 1 <= requested <= MAX_JOBS):
        raise ValueError('requested jobs outside the user limit')
    if any(type(v) is not int or v <= 0 for v in (available_cpus, file_count)):
        raise ValueError('positive CPU and test-file counts required')
    return min(MAX_JOBS if requested is None else requested, available_cpus, file_count)


def selected_output(backend, output_base, name):
    """Select one mounted experiment disk without changing historical inputs."""
    if not name or Path(name).name != name or name in ('.', '..'):
        raise ValueError('run name must be a single new directory name')
    spec = importlib.util.spec_from_file_location('_integrated_storage', W/'tools/experiment_storage.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    selected = module.select(backend, path=output_base)
    base = Path(output_base).absolute() if output_base else selected.ros_log_root/'integrated_regression'
    output = base/name
    binding = selected.validate(output, subtree=selected.ros_log_root)
    if shutil.disk_usage(selected.mount).free <= 2*1024**3:
        raise RuntimeError('selected experiment disk lacks regression reserve; no fallback')
    return selected, binding, output


def main(argv=None):
    global RUN_STORAGE, RUN_STORAGE_BINDING
    parser = argparse.ArgumentParser()
    parser.add_argument('--name', required=True)
    parser.add_argument('--jobs', type=parse_jobs, default=None,
                        help='worker limit, 1..8; default min(8, affinity CPUs, test files)')
    parser.add_argument('--pilot-file', action='append', default=[], help='run only these declared files; never release evidence')
    parser.add_argument('--storage-backend', choices=('ssd', 'sanstand'))
    parser.add_argument('--output-base', type=Path, help='parent under selected ros_log; defaults to its integrated_regression directory')
    args = parser.parse_args(argv)
    RUN_STORAGE, RUN_STORAGE_BINDING, output = selected_output(args.storage_backend, args.output_base, args.name)
    assert shutil.which('bwrap') == '/usr/bin/bwrap', 'required kernel sandbox unavailable; no fallback'
    output.parent.mkdir(parents=True, exist_ok=True)
    RUN_STORAGE.validate(output, subtree=RUN_STORAGE.ros_log_root, expected=RUN_STORAGE_BINDING)
    output.mkdir()
    declared, excluded = scope()
    tests = sorted(args.pilot_file) if args.pilot_file else declared
    assert tests and len(tests) == len(set(tests)) and set(tests) <= set(declared)
    affinity = sorted(os.sched_getaffinity(0))
    jobs = allocate_jobs(args.jobs, len(affinity), len(tests))
    complete_scope = not args.pilot_file
    source = source_manifest(declared)
    history = {}
    for p in sorted(R.glob('*/worker_*/cases.json'), key=lambda p:p.stat().st_mtime_ns):
        per_file = {}
        for report in json.loads(p.read_bytes()).get('reports', []):
            key = report['nodeid'].split('::')[0]
            per_file[key] = per_file.get(key, 0.)+report['duration']
        history.update(per_file)
    history_paths = set(R.glob('integrated_full_regression_r*/result.json')) | set(output.parent.glob('integrated_full_regression_r*/result.json'))
    for p in sorted(history_paths, key=lambda p:p.stat().st_mtime_ns):
        for row in json.loads(p.read_bytes()).get('files',[]):
            history[row['path']]=row['wall_sec']
    ordered = sorted(tests, key=lambda p:history.get(p, 1.), reverse=True)
    canonical_source = {str((W/path).resolve()):digest for path, digest in source.items()}
    write(output/'manifest.json', dict(source_sha256=source, canonical_source_sha256=canonical_source,
        storage_binding=RUN_STORAGE_BINDING, historical_input_root=str(E),
        test_files=tests, declared_test_files=declared,
        excluded_files=excluded, complete_scope_requested=complete_scope,
        jobs=jobs, requested_jobs=args.jobs, user_max_jobs=MAX_JOBS,
        default_process_timeout_sec=DEFAULT_PROCESS_TIMEOUT_SEC,
        process_timeout_sec_by_file={path: PROCESS_TIMEOUT_OVERRIDES_SEC.get(
            path, DEFAULT_PROCESS_TIMEOUT_SEC) for path in tests},
        worker_allocation='min(requested or user limit, affinity CPUs, independent test files)',
        affinity=affinity, load=os.getloadavg(),
        meminfo=Path('/proc/meminfo').read_text(), estimated_seconds=history, ordered_files=ordered,
        frozen_tree_roots=['src', 'tools', 'devel/lib/python3/dist-packages'],
        derived_exclusions=sorted(DERIVED_PARTS|DERIVED_NAMES)))
    started, rows = time.monotonic(), []
    print(json.dumps(dict(started=True, files=len(tests), workers=jobs, pilot=not complete_scope,
                          output=str(output))), flush=True)
    with ThreadPoolExecutor(max_workers=jobs) as pool:
        futures = [pool.submit(run_file, i, path, output) for i, path in enumerate(ordered)]
        for future in as_completed(futures):
            row = future.result()
            rows.append(row)
            if row['exitcode'] or len(rows)%40 == 0:
                print(json.dumps(dict(done=len(rows), total=len(tests), file=row['path'],
                    exitcode=row['exitcode'], wall_sec=time.monotonic()-started)), flush=True)
    expected, results, boundaries, bad, duplicates, collection_skips, external = reconcile(rows, source)
    try:
        after_scope, after_excluded = scope()
        after = source_manifest(after_scope)
        changed = sorted(p for p in set(source)|set(after) if source.get(p) != after.get(p))
        if after_scope != declared or after_excluded != excluded:
            bad.append(dict(error='declared test scope changed during run'))
    except (OSError, ValueError, RuntimeError) as exc:
        changed = ['source_or_scope_verification_error: '+str(exc)]
    skipped = [r['nodeid'] for r in results if r['setup'] == 'skipped' or r['call'] == 'skipped']
    passed = bool(results) and not (bad or boundaries or duplicates or changed) and all(valid_case(r) for r in results)
    result = dict(schema=1, scope_name=('current offline grasp runtime and supporting tools' if complete_scope
        else 'pilot only: selected offline files; not complete declared scope'), passed=passed,
        full_regression_passed=passed and complete_scope and not skipped,
        offline_scope_complete=passed and complete_scope, source_unchanged=not changed,
        source_sha256=source, changed_sources=changed, test_files=tests, excluded_files=excluded,
        storage_binding=RUN_STORAGE_BINDING, historical_input_root=str(E),
        test_roots=['src', 'tools'], explicit_test_files=[HISTORICAL_TEST],
        complete_scope_requested=complete_scope, declared_test_files=declared,
        expected_case_ids=expected, case_results=results, collection_skips=collection_skips,
        external_boundary_attempts=boundaries, external_inputs=external, skipped_cases=skipped,
        bad_files=bad, duplicate_cases=duplicates, wall_sec=time.monotonic()-started,
        summary=dict(tests=len(expected), passed=sum(r['call'] == 'passed' for r in results),
            skipped=len(skipped), failed=sum(not valid_case(r) for r in results),
            collection_skipped_files=len(collection_skips)),
        files=[{k:r[k] for k in ('path', 'output', 'exitcode', 'wall_sec', 'process_timeout_sec')}
               for r in sorted(rows, key=lambda r:r['path'])])
    RUN_STORAGE.validate(output, subtree=RUN_STORAGE.ros_log_root, expected=RUN_STORAGE_BINDING)
    write(output/'result.json', result)
    print(json.dumps({k:result[k] for k in ('passed', 'full_regression_passed', 'summary', 'wall_sec',
        'bad_files', 'changed_sources')}, ensure_ascii=False), flush=True)
    return 0 if passed else 1


if __name__ == '__main__':
    sys.exit(main())
