#!/usr/bin/env python3
"""Prepare, start once, or inspect the installed full stack in manual mode.

Use a new session on the selected experiment disk. `plan` freezes the release, user
instruction and actual roslaunch expansion; `start` dispatches the existing
simulation and full-stack entries; `accept` only observes their new instances.
New manual restarts do not run a cold scan or wait for remote health before
dispatch. The running-instance acceptance still requires the actual dependency.
No generation/PID/selection from a previous session is treated as current.
No operation starts a grasp, switches mode, queries temperature or disables a
joint. The existing reviewed driver retains its positive startup enable.
"""
import argparse
import base64
from collections import deque
import hashlib
import importlib.util
import json
import math
import os
from pathlib import Path
import re
import shutil
import stat
import subprocess
import sys
import threading
import time
import xmlrpc.client

sys.dont_write_bytecode = True
W = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(W/'tools'))
from experiment_storage import select as select_storage

# Historical evidence references keep their original paths. New output uses
# the selected backend and is never redirected through these constants.
M = Path('/media/zhuyupei/SanStand')
E = M/'实验记录/Robot-Controller'
ENTRY = W/'.runtime/start_full_latest_gen36_20261016.sh'
LAUNCH = W/'.runtime/full_latest_gen36_single_observation_20261016.launch'
REMOTE = W/'.runtime/verify_active_remote_readiness_20261020.py'
SIM = W/'.runtime/full_startup_candidate/candidate_simulation_startup.py'
DESKTOP = W/'.runtime/gen58_cold_install_guard_20261021.py'
BACKENDS = E/'ros_log/full_latest_gen41_restart_20261018T011222Z/gen56_runtime/remote_startup_binding_r1/startup_binding.json'
CONTROL = E/'ros_log/full_latest_frozen_no_skin_20261012T021050Z/continuous_authorization_20261012/control.json'
DRIVER_BUILD = E/'ros_log/pending_actuation_drift_20261007T104410.504268Z/driver_build.json'
CLIENT = W/'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/vision/remote_grasp6d_client.py'
HAND_EYE = W/'.runtime/candidate_models/expanded_metric_20260929/handeye_candidate_config.yaml'
CURRENT_TEST_ROOTS = ('src/alicia_grasp_modes/tests', 'src/alicia_flexible_grasp_supervisor/tests', 'tools/tests')
HISTORICAL_PREWARM_TEST = '.runtime/gen6_runner_prewarm_20261012/tests/test_gen6_initial_template_late_start.py'
LIVE_STARTUP_POLICY_KEYS = (
    '/gui/joint_direct_mode', '/alicia_d_driver_node/software_temperature_monitoring_enabled',
    '/alicia_d_driver_node/temperature_poll_rate_hz', '/alicia_d_driver_node/self_check_poll_rate_hz',
    '/alicia_d_driver_node/auto_torque_on_startup', '/camera/simulate', '/camera/fallback_to_simulation',
    '/grasp/unknown_observe_return_max_observations', '/grasp/unknown_lift_after_close_enabled',
    '/robot/strict_execution_controller_sync_timeout_sec', '/robot/strict_execution_controller_sync_duration_sec')


def require(ok, why):
    if not ok:
        raise RuntimeError(why)


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def ref(path):
    path = Path(path).resolve(strict=True)
    return dict(path=str(path), sha256=sha(path))


def read_ref(value):
    raw = Path(value['path']).read_bytes()
    require(hashlib.sha256(raw).hexdigest() == value['sha256'], 'reference changed: '+value['path'])
    return json.loads(raw)


def load(path):
    # Execute verified helper source; do not let an old pyc replace the helper.
    path = Path(path)
    module = type(sys)('_cold_'+path.stem)
    module.__file__ = str(path)
    module.__name__ = '_cold_'+path.stem
    exec(compile(path.read_bytes(), str(path), 'exec'), module.__dict__)
    return module


def storage(path, *, manual_return=False, backend=None, expected=None):
    require(type(manual_return) is bool, 'storage purpose must be explicit')
    path = Path(path).absolute()
    selected = select_storage(backend, path=path)
    identity = selected.validate(path, subtree=selected.ros_log_root, expected=expected)
    require(selected.ros_log_root in path.parents, 'new session must be below selected ros_log')
    archive = json.loads((selected.archive_root/'archive_config.json').read_bytes())
    policy = json.loads((selected.maintenance_root/'storage_control.json').read_bytes())
    require(archive['upload_enabled'] is False and archive['delete_enabled'] is False
            and policy['archive_policy'] == 'local_only', 'storage policy changed')
    free = shutil.disk_usage(selected.mount).free
    # Ending an admitted task does not start another recording. Keep every
    # mount/device/policy check, but do not make manual restoration depend on
    # the reserve needed to admit a new recording.
    if not manual_return:
        require(free >= archive['min_free_bytes'], 'selected disk recording minimum not available')
    return dict(free_bytes=free, min_free_bytes=archive['min_free_bytes'], archive_policy='local_only', storage=identity)


def write(path, value):
    path = Path(path); storage(path)
    raw = (json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False)+'\n').encode()
    with path.open('xb') as stream:
        stream.write(raw); stream.flush(); os.fsync(stream.fileno())
    require(path.read_bytes() == raw, 'evidence readback differs: '+str(path))
    return ref(path)


def release_evidence(release):
    """Preserve full-baseline versus current targeted evidence as distinct roles."""
    require(all(release.get(k) is True for k in ('passed', 'reviewed', 'installed')), 'release is not installed/reviewed')
    require(release.get('source_count') == len(release['source_sha256']), 'release source count differs')
    if release.get('kind') == 'current_offline_release':
        return current_release_evidence(release)
    if release.get('kind') == 'inherited_full_plus_targeted':
        return inherited_release_evidence(release)
    require(release.get('kind') == 'targeted_compute_release'
            and release.get('full_regression_passed') is False, 'this entry expects a truthful targeted release')
    full = read_ref(release['baseline_full_regression'])
    focus = read_ref(release['actual_targeted_regression'])
    require(full.get('passed') is True and full.get('full_regression_passed') is True
            and full.get('scope_complete') is True and full.get('test_bodies_executed') is True,
            'historical full regression incomplete')
    summary = full['summary']
    require(summary['tests'] > 0 and all(summary[k] == 0 for k in ('failures', 'errors', 'skipped')),
            'historical full regression failed')
    cases = focus['case_results']
    identities = [tuple(row['identity']) for row in cases]
    require(focus.get('passed') is True and focus.get('full_regression_passed') is False
            and focus.get('exact_case_inventory') is True and focus.get('duplicate_cases') == 0
            and focus.get('omitted_cases') == 0 and all(row['passed'] is True for row in cases)
            and len(set(identities)) == len(cases) == focus['total_distinct_case_count'] == release['targeted_distinct_cases'],
            'targeted case evidence differs/incomplete')
    changed = dict(focus['production_sha256'], **focus['current_test_sha256'])
    require(set(focus['production_sha256']) == set(release['changed_production_sources'])
            and set(focus['current_test_sha256']) == set(release['changed_test_sources']), 'targeted source scope differs')
    expected = dict(full['source_sha256'], **changed)
    require(expected == release['source_sha256'], 'release is not exact historical baseline plus tested delta')
    return dict(baseline_full=release['baseline_full_regression'], baseline_summary=summary,
                baseline_scope=release['baseline_full_scope'], targeted=release['actual_targeted_regression'],
                targeted_distinct_cases=len(cases), current_full_regression_passed=False)


def offline_regression_evidence(run, *, complete, inherited_explicit_files=()):
    """Read actual recorded scope/phases; historical inputs are never rewritten."""
    require(run.get('schema') == 1 and run.get('passed') is True
            and run.get('source_unchanged') is True
            and all(run.get(k) == [] for k in ('changed_sources', 'external_boundary_attempts',
                'bad_files', 'duplicate_cases', 'collection_skips')), 'offline regression failed or incomplete')
    require(run.get('complete_scope_requested') is complete
            and run.get('offline_scope_complete') is complete, 'offline regression scope claim differs')
    sources = run.get('source_sha256')
    require(type(sources) is dict and bool(sources) and all(type(p) is str
            and not Path(p).is_absolute() and '..' not in Path(p).parts
            and type(h) is str and re.fullmatch(r'[0-9a-f]{64}', h)
            for p, h in sources.items()), 'offline source manifest invalid')
    files, declared, expected, cases = (run.get(k) for k in
        ('test_files', 'declared_test_files', 'expected_case_ids', 'case_results'))
    for label, rows in (('executed files', files), ('declared files', declared), ('case inventory', expected)):
        require(type(rows) is list and rows and all(type(x) is str for x in rows)
                and len(rows) == len(set(rows)), 'offline '+label+' invalid')
    require(set(files) <= set(declared) <= set(sources)
            and (not complete or set(files) == set(declared)), 'offline file scope differs')
    require(all('::' in c for c in expected)
            and {c.split('::', 1)[0] for c in expected} == set(files), 'offline collection/file scope differs')
    require(type(cases) is list and all(type(c) is dict and type(c.get('nodeid')) is str for c in cases),
            'offline phase inventory invalid')
    ids = [c['nodeid'] for c in cases]
    require(len(ids) == len(set(ids)) and set(ids) == set(expected), 'offline cases omitted, duplicated or foreign')
    skipped = []
    for case in cases:
        setup, call, teardown = (case.get(k) for k in ('setup', 'call', 'teardown'))
        require(teardown == 'passed' and ((setup == 'passed' and call in ('passed', 'skipped'))
                or (setup == 'skipped' and call is None)), 'offline case phase failed: '+case['nodeid'])
        if setup == 'skipped' or call == 'skipped':
            require(type(case.get('skip_reason')) is str and bool(case['skip_reason']), 'offline skip reason missing')
            skipped.append(dict(nodeid=case['nodeid'], reason=case['skip_reason']))
    require(run.get('skipped_cases') == [r['nodeid'] for r in skipped], 'offline skip ledger differs')
    summary = dict(tests=len(cases), passed=len(cases)-len(skipped), skipped=len(skipped),
                   failed=0, collection_skipped_files=0)
    require(run.get('summary') == summary
            and run.get('full_regression_passed') is (complete and not skipped), 'offline summary/full claim differs')
    exclusions, roots, explicit = (run.get(k) for k in ('excluded_files', 'test_roots', 'explicit_test_files'))
    require(type(exclusions) is list and all(type(r) is dict and type(r.get('path')) is str
            and type(r.get('reason')) is str and r['reason'] and r['path'] not in declared for r in exclusions)
            and len({r['path'] for r in exclusions}) == len(exclusions), 'offline exclusions invalid')
    require(type(roots) is list and roots and all(type(p) is str and not Path(p).is_absolute()
            and '..' not in Path(p).parts for p in roots) and len(roots) == len(set(roots))
            and all(any(Path(r) == Path(p) or Path(p) in Path(r).parents for p in roots)
                    for r in CURRENT_TEST_ROOTS), 'offline test roots omitted')
    require(type(explicit) is list and all(type(p) is str and p in sources for p in explicit)
            and len(explicit) == len(set(explicit)), 'offline explicit files invalid')
    require(type(inherited_explicit_files) in (list, tuple)
            and all(type(p) is str and p in sources for p in inherited_explicit_files)
            and len(inherited_explicit_files) == len(set(inherited_explicit_files))
            and (not complete or not inherited_explicit_files), 'offline inherited explicit declaration invalid')
    root_files = {p for p in sources if Path(p).name.startswith('test_') and Path(p).suffix == '.py'
                  and any(Path(root) in Path(p).parents for root in roots)}
    require(set(files)-root_files <= set(explicit), 'executed extra file lacks its actual explicit declaration')
    discovered = root_files | set(explicit) | set(inherited_explicit_files)
    require(discovered == set(declared) | {r['path'] for r in exclusions}, 'offline declared scope omitted or invented')
    if HISTORICAL_PREWARM_TEST in sources:
        require(HISTORICAL_PREWARM_TEST in declared, 'historical prewarm coverage omitted')
    require(type(run.get('scope_name')) is str and bool(run['scope_name']), 'offline scope name missing')
    return dict(summary=summary, test_file_count=len(files), skipped_cases=skipped,
                test_roots=roots, excluded_files=exclusions, scope=run['scope_name'],
                declared_test_file_count=len(declared), explicit_test_files=list(explicit),
                inherited_explicit_test_files=list(inherited_explicit_files))


def inherited_release_evidence(release):
    """Admit a reviewed finite delta without claiming a new complete regression."""
    require(release.get('schema') == 1 and release.get('full_regression_passed') is False,
            'inherited release cannot claim a current full regression')
    baseline = read_ref(release['baseline_full_regression'])
    target = read_ref(release['actual_targeted_regression'])
    previous = offline_regression_evidence(baseline, complete=True)
    # A targeted run records only its selected extra files (plus the runner's
    # historical prewarm declaration), while retaining the complete declared
    # scope. The already validated full baseline supplies the other explicit
    # declarations; no target row or historical case is rewritten or rerun here.
    require(type(target.get('explicit_test_files')) is list
            and all(type(p) is str for p in target['explicit_test_files'])
            and set(target['explicit_test_files']) <= set(baseline['explicit_test_files']),
            'target introduces an unreviewed explicit test declaration')
    current = offline_regression_evidence(target, complete=False,
        inherited_explicit_files=baseline['explicit_test_files'])
    require(not current['skipped_cases'], 'targeted delta has unexecuted cases')
    sources = release['source_sha256']; before = baseline['source_sha256']
    require(target['source_sha256'] == sources and set(before) <= set(sources),
            'targeted source inventory differs or deletes historical sources')
    delta = {p:dict(before_sha256=before.get(p), after_sha256=sources[p])
             for p in sorted(sources) if before.get(p) != sources[p]}
    require(delta and release.get('source_delta') == delta, 'release source delta differs from actual manifests')
    files = target['test_files']
    require(release.get('targeted_test_files') == files
            and all(p in files for p in delta if Path(p).name.startswith('test_') and Path(p).suffix == '.py'),
            'changed test file omitted from targeted execution')
    require(target['test_roots'] == baseline['test_roots']
            and target['excluded_files'] == baseline['excluded_files'], 'inherited scope/exclusion policy changed')
    review = read_ref(release['evidence_review'])
    require(all(review.get(k) is True for k in ('passed', 'actual_import_inventory_verified',
                'retained_baseline_cases_verified', 'original_skips_preserved'))
            and all(review.get(k) == release[k] for k in ('baseline_full_regression',
                'actual_targeted_regression', 'source_delta', 'targeted_test_files')),
            'independent case/import/source review differs or incomplete')
    source_check(sources)
    return dict(baseline_full=release['baseline_full_regression'], baseline=previous,
                targeted=release['actual_targeted_regression'], targeted_summary=current['summary'],
                targeted_test_files=files, source_delta=delta, evidence_review=release['evidence_review'],
                targeted_declaration=dict(declared_test_file_count=current['declared_test_file_count'],
                    explicit_test_files=current['explicit_test_files'],
                    inherited_explicit_test_files=current['inherited_explicit_test_files']),
                baseline_cases_inherited=True, current_full_regression_passed=False,
                test_bodies_executed_in_current_run=current['summary']['tests'])


def current_release_evidence(release):
    """Verify current case/phase evidence without relabelling skipped bodies."""
    require(release.get('schema') == 1, 'unsupported current release schema')
    run = read_ref(release['actual_regression'])
    require(run.get('passed') is True and run.get('source_unchanged') is True
            and run.get('external_boundary_attempts') == [], 'current regression failed or crossed offline boundary')
    require(run.get('source_sha256') == release['source_sha256'], 'current regression source differs')
    source_check(release['source_sha256'])
    files = run['test_files']; expected = run['expected_case_ids']; cases = run['case_results']
    require(type(files) is list and files and len(files) == len(set(files))
            and all(type(p) is str and p in release['source_sha256'] for p in files), 'current test file inventory invalid')
    require(type(expected) is list and expected and all(type(c) is str and '::' in c for c in expected)
            and len(expected) == len(set(expected)), 'current collection inventory invalid')
    require({c.split('::', 1)[0] for c in expected} == set(files), 'current collection/file scope differs')
    require(type(cases) is list and all(type(c) is dict and type(c.get('nodeid')) is str for c in cases),
            'current case result inventory invalid')
    identities = [c['nodeid'] for c in cases]
    require(len(identities) == len(set(identities)) and set(identities) == set(expected),
            'current cases duplicated, omitted or unexpected')
    skipped = []
    for case in cases:
        setup, call, teardown = (case.get(k) for k in ('setup', 'call', 'teardown'))
        require(teardown == 'passed' and ((setup == 'passed' and call in ('passed', 'skipped'))
                or (setup == 'skipped' and call is None)), 'current case phase failed/incomplete: ' + case['nodeid'])
        if setup == 'skipped' or call == 'skipped':
            require(type(case.get('skip_reason')) is str and bool(case['skip_reason']), 'skip reason missing')
            skipped.append(dict(nodeid=case['nodeid'], reason=case['skip_reason']))
    exclusions = run.get('excluded_files', [])
    require(type(exclusions) is list and all(type(row) is dict and type(row.get('path')) is str
            and type(row.get('reason')) is str and row['reason'] and row['path'] not in files for row in exclusions),
            'test exclusions must be explicit and outside executed scope')
    require(len({row['path'] for row in exclusions}) == len(exclusions), 'duplicate test exclusions')
    roots, explicit = run['test_roots'], run['explicit_test_files']
    require(type(roots) is list and roots and len(roots) == len(set(roots))
            and all(type(p) is str and not Path(p).is_absolute() and '..' not in Path(p).parts
                    and (W/p).is_dir() for p in roots), 'current discovery roots invalid')
    require(all(any(Path(required) == Path(root) or Path(root) in Path(required).parents for root in roots)
                for required in CURRENT_TEST_ROOTS), 'current core/tools test roots omitted')
    require(type(explicit) is list and len(explicit) == len(set(explicit))
            and all(type(p) is str and not Path(p).is_absolute() and '..' not in Path(p).parts
                    and (W/p).is_file() for p in explicit), 'explicit current test files invalid')
    if (W/HISTORICAL_PREWARM_TEST).exists():
        require(HISTORICAL_PREWARM_TEST in files, 'historical prewarm coverage omitted')
    discovered = {str(p.relative_to(W)) for root in roots for p in (W/root).rglob('test_*.py')
                  if '__pycache__' not in p.parts} | set(explicit)
    require(discovered == set(files) | {row['path'] for row in exclusions}, 'current discovered test files omitted or invented')
    require(type(run.get('scope_name')) is str and bool(run['scope_name']), 'current regression scope must be named')
    complete = not skipped
    require(type(release.get('full_regression_passed')) is bool
            and release['full_regression_passed'] is complete
            and run.get('full_regression_passed') is complete, 'current full-regression claim differs from executed bodies')
    return dict(current=release['actual_regression'], scope=run['scope_name'],
        current_summary=dict(tests=len(cases), passed=len(cases)-len(skipped), skipped=len(skipped), failures=0, errors=0),
        exact_case_inventory=True, test_file_count=len(files), test_roots=roots, excluded_files=exclusions,
        skipped_cases=skipped, test_bodies_executed=complete, current_full_regression_passed=complete)


def source_check(mapping):
    for name, digest in mapping.items():
        path = Path(name)
        require(not path.is_absolute() and '..' not in path.parts, 'invalid relative source')
        require(sha(W/path) == digest, 'installed source changed: '+name)


def driver_build_evidence(release):
    # New releases select their own immutable receipt. Historical releases keep
    # their original build; never overwrite that evidence to admit a new binary.
    explicit = 'driver_build' in release
    reference = release['driver_build'] if explicit else ref(DRIVER_BUILD)
    require(type(reference) is dict and set(reference) == {'path', 'sha256'}
            and type(reference['path']) is str and Path(reference['path']).is_absolute()
            and type(reference['sha256']) is str
            and re.fullmatch(r'[0-9a-f]{64}', reference['sha256']) is not None,
            'invalid driver build reference')
    driver = read_ref(reference)
    require(type(driver) is dict and type(driver.get('returncode')) is int
            and driver['returncode'] == 0, 'reviewed driver build failed')
    sources = driver.get('source_sha256')
    require(type(sources) is dict and bool(sources), 'driver build sources missing')
    source_check(sources)
    if explicit:
        require(all(release['source_sha256'].get(name) == digest for name, digest in sources.items()),
                'driver build sources differ from release')
    require(sha(W/'devel/lib/alicia_d_driver/alicia_d_driver_node') == driver.get('binary_sha256'),
            'driver binary differs from reviewed build')
    return dict(reference), driver


def configuration(session=None):
    import roslaunch
    session = Path(session or os.environ['ALICIA_SESSION_LOG_DIR']).absolute()
    previous = os.environ.get('ALICIA_SESSION_LOG_DIR')
    os.environ['ALICIA_SESSION_LOG_DIR'] = str(session)
    try:
        cfg = roslaunch.config.load_config_default([str(LAUNCH)], 11311, verbose=False, assign_machines=False)
    finally:
        if previous is None:
            os.environ.pop('ALICIA_SESSION_LOG_DIR', None)
        else:
            os.environ['ALICIA_SESSION_LOG_DIR'] = previous
    nodes = {}
    for n in cfg.nodes+cfg.nodes_core:
        name = n.namespace.rstrip('/')+'/'+n.name
        require(name not in nodes and n.machine_name is None, 'unexpected node construction: '+name)
        nodes[name] = dict(package=n.package, executable=n.type, args=n.args,
                           launch_prefix=n.launch_prefix, respawn=n.respawn, required=n.required)
    params = {k:v.value for k,v in cfg.params.items()}
    audit_paths = {'/grasp_6d/remote/gate_audit_output_path':'grasp6d_gate_audit_latest.json',
                   '/mujoco_digital_twin/audit_output_path':'grasp6d_mujoco_audit_latest.json'}
    require(all(params.get(key) == str(session/'planning_audits'/name) for key, name in audit_paths.items()),
            'planning/simulation audit paths differ from frozen storage session')
    policy = {'/gui/joint_direct_mode': True,
              '/alicia_d_driver_node/software_temperature_monitoring_enabled': False,
              '/alicia_d_driver_node/temperature_poll_rate_hz': 0.0,
              '/alicia_d_driver_node/self_check_poll_rate_hz': 0.0,
              '/alicia_d_driver_node/auto_torque_on_startup': True,
              '/camera/simulate': False, '/camera/fallback_to_simulation': False}
    require(all(params.get(k) == v and (type(v) is not bool or type(params[k]) is bool)
                for k,v in policy.items()), 'manual/temperature/real-camera startup policy differs')
    required = {'/supervisor_camera', '/alicia_supervisor_gui', '/alicia_d_driver_node',
                '/grasp_task_node', '/remote_grasp6d_node', '/unknown_tabletop_perception', '/motion_gateway'}
    require(required <= set(nodes) and not any('tactile' in n for n in nodes), 'full launch scope differs')
    loader = nodes['/controller_loader_once']
    require(loader['executable'] == 'controller_loader_once.py' and loader['args'].split() ==
            ['alicia_controller', 'hand_controller'], 'unexpected controller loader')
    check = load(REMOTE)
    remote = {k:params[k] for k in check.PARAMETERS}; check.validate_parameters(remote)
    require(re.findall(r'^export GRASP6D_URL=(\S+)$', ENTRY.read_text(), re.M) == [remote[check.URL]],
            'entry URL differs from expanded remote configuration')
    return dict(nodes=nodes, persistent_nodes=sorted(set(nodes)-{'/controller_loader_once'}),
                parameters=params, remote_parameters=remote,
                launch_source_sha256={str(Path(p).resolve()):sha(p) for p in cfg.roslaunch_files},
                launch=str(LAUNCH), entry=str(ENTRY), parameters_uploaded=False)


def process(pid):
    fields = (Path('/proc')/str(pid)/'stat').read_text().rsplit(')', 1)[1].split()
    require(fields[0] != 'Z', 'zombie process')
    return dict(pid=int(pid), start_ticks=fields[19], parent_pid=int(fields[1]))


def in_group(pid, group):
    return any(line.rsplit(':', 1)[-1] == group or line.rsplit(':', 1)[-1].startswith(group+'/')
               for line in (Path('/proc')/str(pid)/'cgroup').read_text().splitlines())


def properties(unit):
    raw = subprocess.check_output(['systemctl', '--user', 'show', unit,
        '--property=ActiveState,SubState,MainPID,InvocationID,ControlGroup'], text=True, timeout=4.)
    result = dict(line.split('=', 1) for line in raw.splitlines() if '=' in line)
    require(result['ActiveState'] == 'active' and result['SubState'] == 'running'
            and int(result['MainPID']) > 1 and len(result['InvocationID']) == 32, 'service not running: '+unit)
    return result


def device_inventory():
    alias = Path('/dev/alicia_arm'); device = alias.resolve(strict=True)
    require(alias.is_symlink() and stat.S_ISCHR(device.stat().st_mode), 'arm device alias invalid')
    raw = subprocess.check_output(['udevadm', 'info', '--query=property', '--name='+str(device)], text=True, timeout=4.)
    props = dict(line.split('=', 1) for line in raw.splitlines() if '=' in line)
    require(props.get('ID_SERIAL_SHORT') == '5B3D052693', 'reviewed arm USB identity differs')
    devices = {str(device):device.stat().st_rdev}
    for path in Path('/dev').glob('video*'):
        if stat.S_ISCHR(path.stat().st_mode):
            devices[str(path)] = path.stat().st_rdev
    usb = []
    for path in Path('/sys/bus/usb/devices').iterdir():
        try:
            product = (path/'product').read_text().strip()
            if 'RealSense' not in product:
                continue
            serial = (path/'serial').read_text().strip()
            bus, number = int((path/'busnum').read_text()), int((path/'devnum').read_text())
            node = Path('/dev/bus/usb')/('%03d'%bus)/('%03d'%number)
            devices[str(node)] = node.stat().st_rdev
            usb.append(dict(product=product, serial=serial, device=str(node),
                            vendor=(path/'idVendor').read_text().strip(),
                            product_id=(path/'idProduct').read_text().strip()))
        except (FileNotFoundError, NotADirectoryError):
            continue
    require(len(usb) == 1 and (usb[0]['vendor'], usb[0]['product_id']) == ('8086', '0b5b'),
            'one real D405 USB device required')
    return dict(serial_alias=str(alias), serial_device=str(device), serial_id=props['ID_SERIAL_SHORT'],
                camera_usb=usb, devices=devices)


def cold_scan(config, allow_simulation=None):
    """Inspect present owners; never equate a changed parent PID with exit."""
    devices = device_inventory(); rdevs = set(devices['devices'].values())
    desktop = load(DESKTOP)
    names = {'__name:='+prefix+n.lstrip('/') for n in config['nodes'] for prefix in ('', '/')}
    entries = {r['executable'] for r in config['nodes'].values()} | {
        'roslaunch', 'roscore', 'rosmaster', 'candidate_simulation_startup.py', 'run_recorded_grasp.py'}
    conflicts, unreadable = [], []
    for proc in Path('/proc').iterdir():
        if not proc.name.isdigit() or int(proc.name) == os.getpid():
            continue
        try:
            uid = proc.stat().st_uid
            argv = (proc/'cmdline').read_bytes().rstrip(b'\0').decode(errors='replace').split('\0')
            if not argv or not argv[0]:
                continue
            permitted = allow_simulation and in_group(int(proc.name), allow_simulation['ControlGroup'])
            if not permitted and (names.intersection(argv) or any(Path(a).name in entries for a in argv)):
                conflicts.append(dict(pid=int(proc.name), reason='runtime process'))
            try:
                maps = (proc/'maps').read_text()
                if 'pyrealsense2' in maps or 'librealsense' in maps:
                    conflicts.append(dict(pid=int(proc.name), reason='RealSense SDK still mapped'))
                fds = list((proc/'fd').iterdir())
            except PermissionError:
                helper = desktop.protected_system_session_helper(proc, argv) if uid == os.getuid() else False
                require(uid != os.getuid() or helper, 'unreadable same-user device owner: '+proc.name)
                unreadable.append(dict(pid=int(proc.name), uid=uid, known_desktop_helper=bool(helper)))
                continue
            for fd in fds:
                try:
                    actual = fd.stat()
                    if stat.S_ISCHR(actual.st_mode) and actual.st_rdev in rdevs:
                        conflicts.append(dict(pid=int(proc.name), fd=fd.name, reason='device held'))
                except (FileNotFoundError, ProcessLookupError):
                    pass
                except PermissionError:
                    require(uid != os.getuid(), 'unreadable same-user descriptor: '+str(fd))
        except (FileNotFoundError, ProcessLookupError):
            continue
    listeners = []
    for net in ('tcp', 'tcp6'):
        for line in (Path('/proc/net')/net).read_text().splitlines()[1:]:
            row = line.split(); port = int(row[1].rsplit(':', 1)[1], 16)
            if row[3] == '0A' and port in (11311, 8002) and not (port == 8002 and allow_simulation):
                listeners.append(port)
    query = subprocess.run(['fuser', *devices['devices']], capture_output=True, text=True, timeout=5.)
    require(query.returncode == 1 and not query.stdout.strip(), 'serial/camera owner exists or fuser failed')
    require(not conflicts and not listeners, 'stack/devices not cold: '+repr(dict(conflicts=conflicts, listeners=listeners)))
    return dict(recorded_ns=time.time_ns(), boot_id=Path('/proc/sys/kernel/random/boot_id').read_text().strip(),
                devices=devices, conflicts=conflicts, listeners=listeners,
                uninspectable_system_processes=unreadable,
                owner_scope='all visible device FDs and SDK maps; identified protected desktop/other-user tables remain unreadable')


def prestart_health(config, contract, output):
    output.mkdir()
    check = load(REMOTE)
    binding = dict(parameters=config['remote_parameters'], allowed_backends=contract['allowed_backends'],
                   _client_contract=check.client_contract(contract['client']))
    url = check.valid_url(config['remote_parameters'][check.URL])+'/health'
    probe = check.fetch(url, {k:os.environ[k] for k in check.PROXY_KEYS if k in os.environ})
    raw = base64.b64decode(probe.pop('body_base64'), validate=True) if 'body_base64' in probe else None
    result = dict(passed=False, probe=probe, current_node_dependency_ready=False, prediction_requests=0)
    if raw is not None:
        path = output/'health_response.body'; path.write_bytes(raw)
        require(path.read_bytes() == raw, 'health response readback differs')
        probe.update(body_path=str(path), body_sha256=sha(path))
    try:
        require('error_type' not in probe and probe.get('http_status') == 200 and probe.get('response_url') == url
                and probe.get('body_complete') is True and raw is not None, 'REMOTE_DEPENDENCY_NOT_READY')
        result['capabilities'] = check.validate_health(check.strict_json(raw), binding)
        result['passed'] = True
    except Exception as exc:
        result['error'] = repr(exc)
    write(output/'result.json', result)
    require(result['passed'], 'REMOTE_DEPENDENCY_NOT_READY: '+str(result.get('error')))
    return ref(output/'result.json')


def validate_contract(contract, *, manual_return=False):
    require(contract['schema'] == 1 and contract['kind'] in
            ('manual_full_stack_cold_start', 'manual_full_stack_restart')
            and contract['workspace_root'] == str(W), 'wrong startup contract')
    require(contract.get('manual_only') is True and contract.get('automatic_grasp_authorized') is False,
            'manual-only startup contract required')
    require(type(contract.get('storage')) is dict, 'frozen storage identity required')
    if contract['kind'] == 'manual_full_stack_restart':
        require(contract.get('cold_check_executed') is False
                and contract.get('remote_health_check_stage') == 'post_start_acceptance'
                and type(contract.get('boot_id')) is str and bool(contract['boot_id']),
                'manual restart workflow binding invalid')
    out = Path(contract['output_root'])
    storage(out, manual_return=manual_return, backend=contract['storage']['backend'], expected=contract['storage'])
    for path, digest in contract['current_sha256'].items():
        require(sha(path) == digest, 'current startup input changed: '+path)
    instruction = read_ref(contract['instruction']); control = read_ref(contract['control'])
    require(instruction.get('source') == 'actual_user_message' and instruction.get('full_restart_authorized') is True
            and instruction.get('new_scene_confirmation_claimed') is False and bool(instruction.get('text')),
            'actual restart instruction required')
    # An explicit manual restart does not resume the independent grasp grant.
    # Keep its exact frozen snapshot so a later pause/resume still invalidates
    # this startup contract; automatic attempts enforce active authority.
    require(control.get('state') in ('active', 'paused') and control == contract['control_snapshot'],
            'current control changed')
    release = read_ref(contract['release']); source_check(release['source_sha256'])
    require(release_evidence(release) == contract['verification'], 'release evidence roles changed')
    driver_ref, driver = driver_build_evidence(release)
    require(driver_ref == contract.get('driver_build'), 'driver build selection changed')
    require(driver['binary_sha256'] == contract['driver_binary_sha256'], 'driver build binary binding changed')
    config = read_ref(contract['configuration'])
    require(configuration(session=out) == config, 'expanded launch/source configuration changed')
    require(not Path(contract['pycache_prefix']).exists(), 'fresh unused Python cache prefix was populated')
    require(sha(W/'devel/lib/alicia_d_driver/alicia_d_driver_node') == contract['driver_binary_sha256'], 'driver binary changed')
    return out, config, release


def plan(args):
    out = args.session.absolute()
    selected = select_storage(getattr(args, 'storage_backend', None), path=out)
    storage(out, backend=selected.backend); out.mkdir(exist_ok=False)
    os.environ.update(selected.environment(out/'configuration_ros', session=out))
    release_ref = dict(path=str(args.release.resolve()), sha256=args.release_sha256)
    instruction_ref = dict(path=str(args.request.resolve()), sha256=args.request_sha256)
    release = read_ref(release_ref); verification = release_evidence(release)
    source_check(release['source_sha256'])
    config = configuration(); config_ref = write(out/'startup_configuration.json', config)
    driver_ref, driver = driver_build_evidence(release)
    backend_path = getattr(args, 'backend_binding', None) or BACKENDS
    control_path = getattr(args, 'control', None) or CONTROL
    backend = json.loads(backend_path.read_bytes())
    paths = [Path(__file__).resolve(), W/'tools/grasp_attempt_boundary.py', ENTRY, LAUNCH, REMOTE, SIM, DESKTOP, Path(driver_ref['path']), backend_path, CLIENT,
             HAND_EYE, control_path, selected.archive_root/'archive_config.json', selected.maintenance_root/'storage_control.json',
             W/'tools/experiment_storage.py', selected.config_path, W/'.runtime/candidate_simulation/run_candidate.py']
    # Relay files, the camera's local SDK wrapper, and launch dependencies are current construction inputs.
    for row in config['nodes'].values():
        entry = W/'devel/lib'/row['package']/row['executable']
        if entry.is_file():
            paths.append(entry)
    paths.append(W/'tools/with_local_realsense_sdk.sh')
    paths.extend(Path(p) for p in config['launch_source_sha256'])
    contract = dict(schema=1, kind='manual_full_stack_restart', workspace_root=str(W), output_root=str(out),
        release=release_ref, instruction=instruction_ref, control=ref(control_path), control_snapshot=read_ref(ref(control_path)),
        storage=selected.validate(out),
        configuration=config_ref, verification=verification, client=ref(CLIENT), allowed_backends=backend['allowed_backends'],
        driver_build=driver_ref, driver_binary_sha256=driver['binary_sha256'],
        current_sha256={str(p.resolve()):sha(p) for p in sorted(set(paths))},
        pycache_prefix=str(out/'unused_python_cache'), manual_only=True, automatic_grasp_authorized=False,
        physical_grasp_verified=False, temperature_queried=False, torque_disable_commands_sent=False,
        boot_id=Path('/proc/sys/kernel/random/boot_id').read_text().strip(), cold_check_executed=False,
        remote_health_check_stage='post_start_acceptance')
    validate_contract(contract)
    contract_ref = write(out/'contract.json', contract)
    write(out/'plan.json', dict(contract=contract_ref, cold_check_executed=False,
         remote_health_check_stage='post_start_acceptance', prepared_ns=time.time_ns(), starts_services=False))
    print(json.dumps(dict(prepared=True, contract=contract_ref, verification=verification), ensure_ascii=False))


def start(contract, contract_ref):
    out, config, release = validate_contract(contract)
    selected = select_storage(contract['storage']['backend'], path=out)
    os.environ.update(selected.environment(out/'startup_ros', session=out))
    legacy = contract['kind'] == 'manual_full_stack_cold_start'
    require(Path('/proc/sys/kernel/random/boot_id').read_text().strip()
            == contract['cold_boot_id' if legacy else 'boot_id'], 'host boot changed')
    legacy_evidence = {}
    if legacy:
        cold = cold_scan(config)
        require(cold['devices'] == contract['cold_devices'], 'device identity changed since plan')
        legacy_evidence = dict(cold=cold, dispatch_remote_health=prestart_health(config, contract, out/'dispatch_remote_health'))
    validate_contract(contract)
    write(out/'start_claim.json', dict(pid=os.getpid(), claimed_ns=time.time_ns(), contract=contract_ref))
    suffix = hashlib.sha256(str(out).encode()).hexdigest()[:12]
    result = dict(contract=contract_ref, started_ns=time.time_ns(), release_label=release['generation'],
                  full_restart=True, manual_only=True, automatic_grasp_started=False, components={},
                  cold_check_executed=legacy, remote_health_check_stage='post_start_acceptance' if not legacy else 'prestart_and_post_start',
                  startup_positive_enable_configured=True, **legacy_evidence,
                  temperature_queried=False, torque_disable_commands_sent=False)
    try:
        for component in ('simulation', 'core'):
            if component == 'core':
                sim = load(SIM); sim.UNIT = result['components']['simulation']['unit']
                sim.wait_ready(timeout=40.)
                result['simulation_ready'] = sim.service_snapshot()
                if legacy:
                    write(out/'before_core_owner_check.json', cold_scan(config, properties(sim.UNIT)))
                validate_contract(contract)
            unit = 'alicia-manual-full-'+component+'-'+suffix+'.service'
            entry = ['/usr/bin/python3', '-B', str(SIM), '--serve'] if component == 'simulation' else ['/bin/bash', str(ENTRY)]
            command = ['systemd-run', '--user', '--collect', '--unit='+unit, '--property=Type=exec',
                '--property=Restart=no', '--property=WorkingDirectory='+str(W),
                '--property=StandardOutput=append:'+str(out/(component+'.log')),
                '--property=StandardError=append:'+str(out/(component+'.log')),
                *['--setenv='+key+'='+value for key, value in
                  selected.environment(out/(component+'_ros'), session=out).items()],
                '--setenv=PYTHONDONTWRITEBYTECODE=1', '--setenv=PYTHONNOUSERSITE=1',
                '--setenv=PYTHONPYCACHEPREFIX='+contract['pycache_prefix'], *entry]
            write(out/(component+'_launch_intent.json'), dict(unit=unit, command=command, contract=contract_ref))
            run = subprocess.run(command, capture_output=True, text=True, timeout=30.)
            row = dict(unit=unit, command=command, returncode=run.returncode, stdout=run.stdout, stderr=run.stderr)
            result['components'][component] = row; write(out/(component+'_dispatch.json'), row)
            require(run.returncode == 0, 'service dispatch failed: '+component)
        result['dispatched'] = True
    except BaseException as exc:
        result.update(dispatched=False, error=repr(exc), no_automatic_retry=True)
        raise
    finally:
        result['finished_ns'] = time.time_ns(); write(out/'startup.json', result)
    print(json.dumps(dict(dispatched=True, startup=str(out/'startup.json'), components=result['components']), ensure_ascii=False))


class Monitor:
    def __init__(self, config):
        import rospy
        from actionlib_msgs.msg import GoalStatusArray
        from sensor_msgs.msg import JointState, Image
        from std_msgs.msg import Bool, Header, String
        from alicia_flexible_grasp_supervisor.msg import GraspState
        self.rospy = rospy; self.lock = threading.Lock(); self.values = {}
        self.joints = deque(maxlen=80); self.images = {'color':deque(maxlen=30), 'depth':deque(maxlen=30)}
        rows = [('joint', '/alicia_d/accepted_joint_states', JointState), ('epoch', '/alicia_d/control_reference_epoch', Header),
                ('task', '/grasp/state', GraspState), ('mode', '/grasp_mode/status', String),
                ('manual', '/gui/joint_direct_mode', Bool), ('arm', '/alicia_controller/follow_joint_trajectory/status', GoalStatusArray),
                ('hand', '/hand_controller/follow_joint_trajectory/status', GoalStatusArray),
                ('motion_enabled', '/alicia_d/motion_enabled', Bool), ('actuation', '/alicia_d/actuation_status', String),
                ('color', config['parameters']['/camera/color_topic'], Image), ('depth', config['parameters']['/camera/depth_topic'], Image)]
        self.subscriptions = [rospy.Subscriber(topic, kind, lambda msg,k=key:self.receive(k,msg), queue_size=40) for key,topic,kind in rows]

    def receive(self, key, msg):
        with self.lock:
            self.values[key] = (time.monotonic(), msg)
            if key == 'joint': self.joints.append(msg)
            if key in self.images: self.images[key].append(msg)

    def sample(self, started_ns):
        with self.lock:
            data = dict(self.values); joints = list(self.joints)[-20:]
            images = {k:list(v) for k,v in self.images.items()}
        require(len(data) == 11 and len(joints) == 20, 'startup telemetry pending')
        values = {k:v[1] for k,v in data.items()}; now = self.rospy.Time.now().to_nsec()
        owners = dict(joint='/alicia_d_driver_node', epoch='/alicia_d_driver_node', manual='/alicia_supervisor_gui',
            task='/grasp_task_node', mode='/grasp_mode_router', arm='/bessica_d_hw_interface', hand='/bessica_d_hw_interface',
            motion_enabled='/alicia_d_driver_node', actuation='/alicia_d_driver_node', color='/supervisor_camera', depth='/supervisor_camera')
        require(all(m._connection_header.get('callerid') == owners[k] for k,m in values.items()), 'telemetry owner differs')
        epoch = values['epoch'].stamp.to_nsec(); names = list(joints[-1].name)
        stamps = [m.header.stamp.to_nsec() for m in joints]
        require(epoch >= started_ns and epoch <= now and {'Joint%d'%i for i in range(1,7)} <= set(names)
                and len(names) == len(set(names)), 'new driver epoch/joint layout invalid')
        require(all(m._connection_header.get('callerid') == owners['joint']
                    and m.header.frame_id == 'sdk_measured' and list(m.name) == names and len(m.position) == len(names)
                    and all(math.isfinite(v) for v in m.position) and epoch <= m.header.stamp.to_nsec() <= now for m in joints)
                and all(b > a for a,b in zip(stamps, stamps[1:])) and 0 <= now-stamps[-1] < 200_000_000,
                'real SDK window stale/unordered/invalid')
        spans = [max(m.position[i] for m in joints)-min(m.position[i] for m in joints) for i in range(len(names))]
        require(all(v <= .004 for v in spans), 'SDK window not stationary')
        # /grasp/state publishes transitions and latches its latest state. The
        # current publisher instance is checked by node_snapshot before/after
        # this observer; time since a transition is not a heartbeat deadline.
        require(values['task']._connection_header.get('latching') == '1', 'task state must be the current latched state')
        require(values['manual']._connection_header.get('latching') == '1', 'GUI acknowledgement must be latched')
        require(values['manual'].data is True and not values['task'].active, 'manual idle required')
        load(W/'tools/grasp_attempt_boundary.py').validate_idle_router(values['mode'].data)
        for k in ('arm','hand'):
            msg = values[k]
            require(0 <= now-msg.header.stamp.to_nsec() < 2_000_000_000 and
                    not any(g.status in (0,1,6,7) for g in msg.status_list), 'trajectory active/stale')
        paired = {}
        for kind,messages in images.items():
            for m in messages:
                stamp = m.header.stamp.to_nsec()
                if 0 <= now-stamp < 500_000_000 and m._connection_header.get('callerid') == '/supervisor_camera':
                    require(m.width > 0 and m.height > 0 and m.step > 0 and len(m.data) == m.step*m.height
                            and m.encoding == ('bgr8' if kind == 'color' else '16UC1'), 'RGB-D layout invalid')
                    paired.setdefault(stamp, {})[kind] = dict(stamp_ns=stamp, width=m.width, height=m.height, encoding=m.encoding)
        complete = [v for stamp,v in sorted(paired.items()) if set(v) == {'color','depth'}]
        require(complete, 'fresh synchronous RGB-D pending')
        require(type(values['motion_enabled'].data) is bool and bool(values['actuation'].data), 'execution status missing')
        return dict(epoch_ns=epoch, feedback_stamps_ns=stamps, names=names, positions=list(joints[-1].position), spans=spans,
            manual_mode=True, gui_owning_manual_ack=True, task_active=False, actions_active=False, camera=complete[-1],
            motion_enabled=values['motion_enabled'].data, actuation_status=values['actuation'].data,
            actuation_confirmation_not_required_for_manual_startup=True, scene_clearance_granted=False,
            task_state_contract='event_driven_latched_current_publisher',
            task_state_received_age_sec=time.monotonic()-data['task'][0])

    def close(self):
        for sub in self.subscriptions: sub.unregister()


def node_snapshot(master, core, config, caller):
    system = master.getSystemState()
    observed = set()
    for group in system:
        for _,names in group: observed.update(names)
    observed.discard(caller)
    expected = set(config['persistent_nodes'])
    extra = observed-expected
    require(expected <= observed and len(extra) <= 1
            and all(re.fullmatch('/move_group_commander_wrappers_[0-9]+', name) for name in extra),
            'configured graph differs: '+repr(dict(missing=sorted(expected-observed), extra=sorted(extra))))
    result = {}
    transport = load(REMOTE).RpcTransport
    for name in sorted(expected):
        uri = master.lookupNode(name)
        reply = xmlrpc.client.ServerProxy(uri, transport=transport()).getPid(caller)
        require(reply[0] == 1, 'node PID unavailable: '+name)
        row = process(reply[2]); proc = Path('/proc')/str(row['pid'])
        require(row['parent_pid'] == int(core['MainPID']) and in_group(row['pid'], core['ControlGroup']), 'node outside new core: '+name)
        command = (proc/'cmdline').read_bytes(); argv = command.rstrip(b'\0').decode().split('\0')
        require(any(Path(a).name == config['nodes'][name]['executable'] for a in argv), 'node executable differs: '+name)
        result[name] = dict(row, cmdline_sha256=hashlib.sha256(command).hexdigest(), uri=uri)
    require(len({v['pid'] for v in result.values()}) == len(result), 'duplicate node PID')
    aliases = {}
    for name in sorted(extra):
        uri = master.lookupNode(name)
        reply = xmlrpc.client.ServerProxy(uri, transport=transport()).getPid(caller)
        require(reply[0] == 1, 'gateway alias PID unavailable')
        row = process(reply[2])
        gateway = result['/motion_gateway']
        require(all(row[k] == gateway[k] for k in ('pid','start_ticks','parent_pid')),
                'helper name is not the actual motion_gateway process')
        aliases[name] = dict(row, uri=uri)
    publishers = dict(system[0])
    owners = {'/alicia_d/accepted_joint_states':'/alicia_d_driver_node',
              '/grasp/state':'/grasp_task_node',
              '/alicia_d/control_reference_epoch':'/alicia_d_driver_node',
              '/alicia_d/motion_enabled':'/alicia_d_driver_node',
              '/alicia_d/actuation_status':'/alicia_d_driver_node',
              config['parameters']['/camera/color_topic']:'/supervisor_camera',
              config['parameters']['/camera/depth_topic']:'/supervisor_camera'}
    require(all(publishers.get(topic) == [owner] for topic,owner in owners.items()), 'feedback/camera publisher not unique')
    return result, aliases


def gui_windows(pid):
    env = dict(os.environ, DISPLAY=':0', XAUTHORITY='/run/user/1000/gdm/Xauthority')
    raw = subprocess.check_output(['xprop', '-root', '_NET_CLIENT_LIST'], env=env, text=True, timeout=3.)
    found = []
    for window in re.findall(r'0x[0-9a-fA-F]+', raw):
        details = subprocess.check_output(['xprop','-id',window,'_NET_WM_PID'], env=env, text=True, timeout=3.)
        match = re.search(r'=\s*(\d+)\s*$', details)
        if match and int(match.group(1)) == pid: found.append(window)
    require(found, 'GUI has no mapped X11 window')
    return found


def live_startup_policy(config, get_parameter):
    values = {key:get_parameter(key) for key in LIVE_STARTUP_POLICY_KEYS}
    for key, value in values.items():
        require(value == config['parameters'][key]
                and (type(config['parameters'][key]) is not bool or type(value) is bool),
                'live startup policy differs: '+key)
    return values


def accept(contract, contract_ref, destination=None, verifier_revision=None, expected_runtime=None):
    out, config, release = validate_contract(contract)
    selected = select_storage(contract['storage']['backend'], path=out)
    destination = Path(destination).resolve() if destination is not None else out/'acceptance'
    require(destination.parent == out and destination != out, 'acceptance output must be a new child of original startup session')
    destination.mkdir(exist_ok=False)
    os.environ.update(selected.environment(destination/'observer_ros', session=out))
    startup_ref = ref(out/'startup.json'); startup = read_ref(startup_ref)
    require(startup['contract'] == contract_ref and startup['dispatched'] is True, 'startup incomplete/wrong contract')
    result = dict(passed=False, full_restart=True, verification=contract['verification'], manual_only=True,
        automatic_grasp_started=False, motion_commands_sent=False, temperature_queried=False, torque_disable_commands_sent=False,
        physical_grasp_verified=False, actuation_confirmation_not_implied_by_restart=True, contract=contract_ref,
        started_ns=time.time_ns(), source_manifest=contract['release'], verifier_revision=verifier_revision)
    monitor = None
    try:
        import rosgraph
        import rospy
        import yaml
        from controller_manager_msgs.srv import ListControllers
        rospy.init_node('manual_full_start_acceptance', anonymous=True, disable_rosout=True)
        master = rosgraph.Master(rospy.get_name())
        core = properties(startup['components']['core']['unit'])
        core_proc = Path('/proc')/core['MainPID']
        argv = (core_proc/'cmdline').read_bytes().rstrip(b'\0').decode().split('\0')
        require(argv[-2:] == ['--skip-log-check', str(LAUNCH.relative_to(W))]
                and core_proc.joinpath('cwd').resolve() == W, 'core entry/cwd differs')
        master_pid = master.getPid()
        require(process(master_pid)['parent_pid'] == int(core['MainPID']) and in_group(master_pid,core['ControlGroup']), 'master outside new core')
        monitor = Monitor(config)
        deadline = time.monotonic()+35.; last = None
        while time.monotonic() < deadline:
            try:
                nodes, aliases = node_snapshot(master,core,config,rospy.get_name())
                measured = monitor.sample(startup['started_ns']); break
            except Exception as exc:
                last = repr(exc); time.sleep(.1)
        else: raise RuntimeError('full startup pending: '+str(last))
        if expected_runtime is not None:
            require(nodes == expected_runtime['nodes'] and aliases == expected_runtime['auxiliary_nodes']
                    and core == expected_runtime['core_service'], 'startup instance changed since prior acceptance')
            result['preserved_startup_instances'] = True
        result.update(nodes=nodes, auxiliary_nodes=aliases, core_service=core,
                      core_unit=startup['components']['core']['unit'], master_identity=process(master_pid), state=measured,
                      gui_windows=gui_windows(nodes['/alicia_supervisor_gui']['pid']))
        # Existing launch parameters were frozen before dispatch, including remote URL.
        result['startup_policy_parameters'] = live_startup_policy(config, rospy.get_param)
        for name,row in nodes.items():
            proc = Path('/proc')/str(row['pid'])
            env = dict(item.decode().split('=',1) for item in (proc/'environ').read_bytes().split(b'\0') if b'=' in item)
            require(env.get('PYTHONPYCACHEPREFIX') == contract['pycache_prefix']
                    and env.get('PYTHONDONTWRITEBYTECODE') == '1' and env.get('PYTHONNOUSERSITE') == '1'
                    and env.get('ROS_LOG_DIR') == str(out/'core_ros'), 'source/cache/external-log environment differs: '+name)
        require(sha(Path('/proc')/str(nodes['/alicia_d_driver_node']['pid'])/'exe') == contract['driver_binary_sha256'], 'running driver binary differs')
        controllers = rospy.ServiceProxy('/controller_manager/list_controllers', ListControllers)().controller
        result['controllers'] = {c.name:c.state for c in controllers}
        require(all(result['controllers'].get(n) == 'running' for n in ('alicia_controller','hand_controller')), 'controller not running')
        log = (out/'core.log').read_bytes()[:2*1024*1024].decode(errors='replace')
        begun = re.findall(r'process\[controller_loader_once-([0-9]+)\]: started with pid \[([0-9]+)\]',log)
        ended = re.findall(r'\[controller_loader_once-([0-9]+)\] process has finished cleanly',log)
        require(len(begun) == 1 and ended == [begun[0][0]], 'one-shot controller loader not sealed')
        result['controller_loader_once'] = dict(pid=int(begun[0][1]), finished_cleanly=True)
        sim = load(SIM); sim.UNIT = startup['components']['simulation']['unit']; sim.wait_ready(timeout=40.)
        result['simulation'] = sim.service_snapshot()
        require(hashlib.sha256(rospy.get_param('/robot_description').encode()).hexdigest() == sim.IDENTITY['candidate_model_sha256'], 'paired URDF differs')
        require(rospy.get_param('/handeye') == yaml.safe_load(HAND_EYE.read_text())['handeye'], 'paired hand-eye differs')
        selection = rospy.get_param('/grasp_mode/selection')
        require(selection['mode'] == 'unknown' and selection['strategy'] == 'observe_return', 'startup selection differs')
        remote = nodes['/remote_grasp6d_node']; identity = {k:remote[k] for k in ('pid','parent_pid','start_ticks','cmdline_sha256')}
        binding = dict(schema=1, node_name='/remote_grasp6d_node', master_uri='http://localhost:11311',
            parameters=config['remote_parameters'], allowed_backends=contract['allowed_backends'],
            client_contract_source=contract['client'], node_identity=identity,
            evidence=[contract_ref, startup_ref, contract['configuration'], contract['release'], contract['instruction']]+
                ([verifier_revision] if verifier_revision is not None else [])+
                [dict(path=p,sha256=h) for p,h in contract['current_sha256'].items()],
            construction_evidence_review_required=True, construction_provenance='actual frozen full-stack launch and new process')
        binding_ref = write(destination/'remote_startup_binding.json',binding)
        result['remote_startup_binding'] = binding_ref
        check = load(REMOTE); validated = check.load_binding(Path(binding_ref['path']),binding_ref['sha256'])
        remote_result = check.evaluate(validated,destination)
        result['remote_dependency'] = write(destination/'remote_readiness.json',remote_result)
        require(remote_result['passed'] is True and remote_result['dependency_ready'] is True, 'REMOTE_DEPENDENCY_NOT_READY: '+str(remote_result.get('error')))
        validate_contract(contract)
        require(node_snapshot(master,core,config,rospy.get_name()) == (nodes, aliases) and properties(startup['components']['core']['unit']) == core,
                'runtime instance changed during acceptance')
        require(rospy.get_param('/grasp_mode/selection') == selection and master.getPid() == master_pid, 'reference/master changed')
        final = monitor.sample(startup['started_ns'])
        require(final['epoch_ns'] == measured['epoch_ns'], 'driver reference changed')
        require(time.time_ns()-remote_result['finished_ns'] < 30_000_000_000, 'remote health expired during acceptance')
        result.update(passed=True,state=final,selection=selection,all_configured_nodes_started=True,
                      source_cache_process_binding=True,memory_attestation=False)
    except BaseException as exc:
        result['error'] = repr(exc); raise
    finally:
        if monitor: monitor.close()
        result['finished_ns'] = time.time_ns(); write(destination/'result.json',result)
    print(json.dumps(dict(passed=True,result=ref(destination/'result.json'),state=result['state']),ensure_ascii=False))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest='operation', required=True)
    p = sub.add_parser('plan')
    p.add_argument('--storage-backend', choices=('ssd', 'sanstand'))
    p.add_argument('--backend-binding', type=Path, help='reviewed remote backend declaration; historical default preserved')
    p.add_argument('--control', type=Path, help='current explicit control receipt; historical default preserved')
    for name in ('session','release','request'): p.add_argument('--'+name,type=Path,required=True)
    for name in ('release-sha256','request-sha256'): p.add_argument('--'+name,required=True)
    for name in ('start','accept'):
        p = sub.add_parser(name); p.add_argument('--contract',type=Path,required=True); p.add_argument('--sha256',required=True)
    args = parser.parse_args()
    if args.operation == 'plan':
        try:
            return plan(args)
        except BaseException as exc:
            out = args.session.resolve()
            if out.is_dir() and not (out/'contract.json').exists() and not (out/'failure.json').exists():
                write(out/'failure.json', dict(passed=False,stage='plan',error=repr(exc),recorded_ns=time.time_ns(),
                     services_started=False,motion_commands_sent=False,temperature_queried=False))
            raise
    reference = dict(path=str(args.contract.resolve(strict=True)),sha256=args.sha256)
    contract = read_ref(reference)
    return start(contract,reference) if args.operation == 'start' else accept(contract,reference)


if __name__ == '__main__':
    main()
