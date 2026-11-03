"""GEN55 pinned release and idle reload checks. No ROS on import."""
import hashlib, importlib.util, json, math, os, shutil, time
from pathlib import Path
import xmlrpc.client
from collections import Counter
W = Path(__file__).resolve().parents[1]
M = Path('/media/zhuyupei/SanStand')
E = M/'实验记录/Robot-Controller'
N = E/'ros_log/full_latest_gen41_restart_20261018T011222Z'
BASE_SHA = 'ec0d1c1d2ebdb88fc9c77174790190dfff71024e58fea6958b356cb9ebf5d3f8'
PREVIOUS = E/'ros_log/full_validated_gen54_restart_20261020T012700Z/gen54_current_runtime_identity.json'
PREVIOUS_SHA = '7e256ee6f5cb64756d40e4c2b07468a791d10704c81d4154144b141a11f23bd3'
BASELINE = W/'.runtime/gen54_baseline_source/runner_manifest.json'
BASELINE_SHA = 'e0ea9529ff6afdff8a6794578f1e64d996cdb124ee1adf3e6732e38e44984748'
HISTORICAL_XML = N/'verification/gen53_full_parallel_r1/pytest.xml'
HISTORICAL_XML_SHA = '3ce8c6f4e6de0be68745042e113f2e727f7b8f3cdc862e4728dca0523b8ebb78'
CONTROL = E/'ros_log/full_latest_frozen_no_skin_20261012T021050Z/continuous_authorization_20261012/control.json'
CONTROL_EXPECTED_SHA = 'c9d29dac7b6caadcef967e497fe0bf628a2b6fcec642d48ca13713722020a0f0'
COMPUTE = ('/grasp_task_node', '/remote_grasp6d_node', '/unknown_tabletop_perception')
TARGETS = COMPUTE + ('/motion_gateway',)
PRESERVE = ('/alicia_d_driver_node', '/bessica_d_hw_interface', '/move_group',
    '/supervisor_camera', '/compliant_gripper_controller', '/grasp_mode_router',
    '/alicia_supervisor_gui')
HELPERS = tuple(W/'.runtime'/name for name in (
    'gen55_idle_reload_common_20261019.py',
    'read_gen55_deployment_preflight_20261019.py',
    'load_gen55_idle_compute_gateway_20261019.py',
    'gen55_observation_measurement_consumers_20261019.launch',
    'start_gen55_observation_measurement_consumers_20261019.sh'))
GATEWAY_SERVICES = ('/supervisor/check_contact_sequence', '/supervisor/check_pose_strict',
    '/supervisor/check_observation_pose_strict')
PARAMETERS = ('/grasp/unknown_observe_return_max_observations',
    '/grasp/unknown_lift_after_close_enabled',
    '/unknown_perception/support_max_local_shift_m',
    '/unknown_perception/support_local_shift_warning_m',
    '/grasp_6d/remote/multiview/maximum_support_offset_delta_m',
    '/unknown_perception/detect_hz',
    '/alicia_d_driver_node/software_temperature_monitoring_enabled',
    '/alicia_d_driver_node/temperature_poll_rate_hz')

def sha(p):
    return hashlib.sha256(Path(p).read_bytes()).hexdigest()

def read(p):
    return json.loads(Path(p).read_text())

def require(condition, message):
    if not condition:
        raise RuntimeError(message)

def inside(path, root):
    try:
        Path(path).resolve().relative_to(Path(root).resolve())
        return True
    except ValueError:
        return False

def storage(output):
    require(M.is_mount() and not os.statvfs(M).f_flag & os.ST_RDONLY, 'writable SanStand required')
    require(inside(output, N) and Path(output).resolve() != N.resolve(), 'output must stay in current external session')
    require(Path(output).parent.stat().st_dev == M.stat().st_dev, 'output parent is not on SanStand')
    cfg, policy = read(E/'grasp_records/archive_config.json'), read(E/'maintenance/storage_control.json')
    require(cfg['upload_enabled'] is False and cfg['delete_enabled'] is False and policy['archive_policy'] == 'local_only', 'storage policy changed')
    require(shutil.disk_usage(E).free >= cfg['min_free_bytes'], 'external free space below policy')

def write(path, value):
    path = Path(path)
    storage(path.parent)
    temporary = path.with_name(path.name+'.tmp')
    with temporary.open('x') as stream:
        json.dump(value, stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.write('\n'); stream.flush(); os.fsync(stream.fileno())
    temporary.replace(path)

def validate_sources(release):
    for name, digest in release['source_sha256'].items():
        relative = Path(name)
        require(not relative.is_absolute() and '..' not in relative.parts and sha(W/relative) == digest, 'source changed: '+name)
    for name, digest in release['evidence_sha256'].items():
        require(sha(name) == digest, 'release evidence changed: '+name)

class Transport(xmlrpc.client.Transport):
    def make_connection(self, host):
        connection = super().make_connection(host)
        connection.timeout = 3.
        return connection

def process(pid):
    fields = (Path('/proc')/str(pid)/'stat').read_text().rsplit(')', 1)[1].split()
    require(fields[0] != 'Z', 'zombie process')
    return dict(pid=int(pid), start_ticks=fields[19], parent_pid=int(fields[1]))

def exited(row):
    try:
        return process(row['pid'])['start_ticks'] != row['start_ticks']
    except (FileNotFoundError, RuntimeError):
        return True

def feedback(rospy, minimum=20):
    # Read-only SDK sampling; this deliberately never reads temperature values.
    import threading
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Header
    lock, samples = threading.Lock(), []
    def receive(msg):
        with lock:
            samples.append(msg)
    sub = rospy.Subscriber('/alicia_d/accepted_joint_states', JointState, receive, queue_size=40)
    try:
        end = time.monotonic()+8.
        while time.monotonic() < end:
            with lock:
                if len(samples) >= minimum:
                    break
            time.sleep(.025)
        with lock:
            history = list(samples)
        require(len(history) >= minimum, 'insufficient SDK measured samples')
        joint = history[-1]
        require(all(q.header.frame_id == 'sdk_measured' for q in history), 'non-SDK joint feedback')
        require(0 <= (rospy.Time.now()-joint.header.stamp).to_sec() < .2, 'SDK feedback stale')
        require(len(joint.name) == len(joint.position) > 0 and len(set(joint.name)) == len(joint.name), 'invalid SDK joint layout')
        require(all(math.isfinite(v) for q in history for v in q.position), 'nonfinite SDK feedback')
        require(all(q.name == joint.name and len(q.position) == len(joint.position) for q in history), 'SDK layout changed during collection')
        return dict(names=list(joint.name), positions=list(joint.position), frame_id=joint.header.frame_id,
            stamp_ns=joint.header.stamp.to_nsec(), epoch_ns=rospy.wait_for_message('/alicia_d/control_reference_epoch', Header, timeout=5.).stamp.to_nsec(), sample_count=len(history))
    finally:
        sub.unregister()

def candidate_context(manifest_path, manifest_sha, runner_path, runner_sha):
    """Validate the reviewed runner and its complete frozen source/case context."""
    runner_path = Path(runner_path).resolve(strict=True)
    require(runner_path == W/'.runtime/verify_gen55_parallel_offline_20261019.py'
        and sha(runner_path) == runner_sha, 'GEN55 reviewed runner differs')
    manifest_path = Path(manifest_path).resolve(strict=True)
    require(inside(manifest_path, N/'verification') and sha(manifest_path) == manifest_sha,
        'GEN55 candidate manifest differs')
    spec = importlib.util.spec_from_file_location('gen55_bound_offline_runner', runner_path)
    runner = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(runner)
    manifest = runner.bind_manifest(manifest_path, manifest_sha)
    context = runner.current_context(runner.load_serial())
    require(manifest['full_scope_ready'] is True, 'complete collected scope required')
    require(len(context['case_identities']) == len(set(map(tuple, context['case_identities']))),
        'duplicate frozen case identities')
    return manifest, context, runner


def validate_result(path, manifest_path, manifest_sha, runner_path, runner_sha, mode):
    """Bind exact cases and every completed worker artifact, without rerunning tests."""
    manifest, context, runner = candidate_context(manifest_path, manifest_sha, runner_path, runner_sha)
    path = Path(path).resolve(strict=True)
    require(inside(path, N/'verification'), 'result outside current external session')
    result = read(path)
    require(result['mode'] == mode and all(result.get(k) is True for k in
        ('passed', 'scope_complete', 'source_unchanged', 'test_bodies_executed', 'combined_xml_matches_worker_results')),
        'offline result is not complete/passing')
    require(result['candidate_manifest_sha256'] == manifest_sha and
        Path(result['candidate_manifest_path']).resolve(strict=True) == Path(manifest_path).resolve(strict=True)
        and result['runner_sha256'] == runner_sha and result['source_sha256'] == context['sources'],
        'result source or runner binding differs')
    require(not result['interrupted'] and result['launcher_error'] is None and
        not result['offline_child_cleanup_required'] and not result['residual_offline_process_groups'] and
        result['worker_returncodes'] == [0]*result['worker_count'], 'offline coordinator did not finish cleanly')
    require(not any(result[k] for k in ('external_boundary_attempts', 'foreign_or_inconsistent_imports',
        'duplicate_case_identities', 'missing_case_identities', 'unexpected_case_identities', 'missing_worker_results')),
        'incomplete case/provenance/boundary evidence')
    scope = context['scope'] if mode == 'full' else manifest['focus_scope']
    expected = [c for c in context['case_identities'] if c[0] in scope]
    xml = path.parent/'pytest.xml'
    summary, cases, _ = runner.inventory(xml)
    require(sha(xml) == result['xml_sha256'] and summary == result['summary'] and
        summary['tests'] == len(expected) and all(summary[k] == 0 for k in ('failures','errors','skipped')) and
        Counter(map(tuple,cases)) == Counter(map(tuple,expected)), 'exact executed case inventory differs')
    require(result['test_scope_files'] == len(scope) and result['source_count'] == len(context['sources']),
        'source/file count differs')
    plan_path = path.parent/'plan.json'; plan = read(plan_path)
    require(sha(plan_path) == result['plan_sha256'] and plan['context']['sources'] == context['sources'] and
        Counter(map(tuple,plan['expected_case_identities'])) == Counter(map(tuple,expected)) and
        plan['test_scope'] == scope and plan['mode'] == mode and plan['runner_sha256'] == runner_sha,
        'executed plan differs')
    evidence = dict(context['compatibility_dependencies'])
    evidence.update({str(p):sha(p) for p in (path, xml, plan_path, Path(manifest_path), Path(runner_path))})
    serial = runner.load_serial()
    workspace = Path(manifest['workspace_root']).resolve(strict=True)
    required = (set(context['required_changed_sources']) if mode == 'full' else
        {p for test in scope for p in context['required_imports_by_test_file'].get(test, [])})
    if not required:
        required = set(context['required_changed_sources'])
    require(set(plan['required_changed_sources']) == required, 'required module set differs')
    globally_loaded = {p: False for p in required}
    child_cases = []; seen = set()
    for filename, digest in result['worker_result_sha256'].items():
        p = Path(filename).resolve(strict=True); row = read(p); index = row['worker_index']
        require(type(index) is int and 0 <= index < result['worker_count'] and index not in seen and
            p == path.parent/('worker_%d'%index)/'result.json' and sha(p) == digest,
            'worker identity changed')
        seen.add(index); shard = plan['shards'][index]
        require(all(row.get(k) is True for k in ('passed','scope_complete','source_unchanged','direct_child_reaped','test_bodies_executed'))
            and row['returncode'] == 0 and row['interrupted'] is False and not row['direct_child_cleanup_signals'] and
            not row['external_boundary_attempts'] and not row['foreign_or_inconsistent_imports'] and
            row['plan_sha256'] == result['plan_sha256'] and row['test_scope'] == shard['scope'] and row['mode'] == mode,
            'worker did not finish reviewed scope cleanly')
        expected_required = sorted({p for test in shard['scope']
            for p in context['required_imports_by_test_file'].get(test, [])})
        require(shard['required_changed_sources'] == row['required_changed_sources'] == expected_required,
            'worker required module scope differs')
        boundary = p.parent/'boundary_attempts.jsonl'
        require(not boundary.exists() or not boundary.read_text().splitlines(), 'boundary evidence appended')
        for filename,key in (('pytest.xml','xml_sha256'),('pytest.log','log_sha256'),('module_provenance.json','module_provenance_sha256')):
            artifact = p.parent/filename
            require(sha(artifact) == row[key], 'worker artifact changed')
            evidence[str(artifact)] = row[key]
        provenance = read(p.parent/'module_provenance.json')
        require(isinstance(provenance, dict) and provenance, 'empty module provenance')
        for label, imported in provenance.items():
            module_path = Path(imported['path']); digest = imported['sha256']
            if serial.inside(module_path, workspace/'src'):
                relative = str(module_path.relative_to(workspace))
                valid = context['sources'].get(relative) == digest and sha(module_path) == digest
            elif serial.inside(module_path, W/'devel'):
                valid = context['generated_imports'].get(str(module_path)) == digest and sha(module_path) == digest
            else:
                valid = (label == serial.ORACLE_LABEL and module_path == serial.ORACLE
                    and digest == serial.ORACLE_SHA and serial.ORACLE_TEST in plan['test_scope']
                    and context['sources'].get(serial.ORACLE_TEST) == serial.ORACLE_TEST_SHA
                    and sha(module_path) == digest)
            require(valid, 'foreign or changed import: '+str(module_path))
        loaded = {name: any(imported['path'] == str(workspace/name)
            and imported['sha256'] == context['sources'][name] for imported in provenance.values())
            for name in context['required_changed_sources']}
        require(row['loaded_changed_sources'] == loaded and
            all(loaded[name] for name in expected_required), 'worker module-load evidence differs')
        for name in globally_loaded:
            globally_loaded[name] = globally_loaded[name] or loaded[name]
        expected_bootstrap = {'bootstrap/sitecustomize.py': context['guard_sha256'],
                              'bootstrap/gen42_import_audit.py': context['audit_sha256']}
        require(row['generated_bootstrap_sha256'] == expected_bootstrap,
            'worker isolation bootstrap inventory differs')
        for filename,digest in row['generated_bootstrap_sha256'].items():
            rel = Path(filename)
            require(not rel.is_absolute() and '..' not in rel.parts and sha(p.parent/rel) == digest, 'bootstrap changed')
            evidence[str(p.parent/rel)] = digest
        child_summary, current, _ = runner.inventory(p.parent/'pytest.xml')
        require(child_summary == row['summary'] and current == row['case_identities'] and
            Counter(map(tuple,current)) == Counter(map(tuple,shard['case_identities'])), 'worker exact cases differ')
        child_cases.extend(current); evidence[str(p)] = sha(p)
    require(len(seen) == result['worker_count'] and Counter(map(tuple,child_cases)) == Counter(map(tuple,cases)),
        'combined XML differs from worker union')
    require(result['loaded_changed_sources_global'] == globally_loaded and all(globally_loaded.values()),
        'global required module-load evidence differs')
    if mode == 'full':
        require(result['full_regression_scope'] and result['full_regression_passed'] and
            all(result['loaded_changed_sources_global'].values()), 'full verification required')
    return manifest, context, result, evidence


def validate_release(path):
    release = read(path)
    require(all(release.get(k) is True for k in ('passed','reviewed','installed')) and
        release['generation'] == 'GEN55' and release['baseline_release_sha256'] == BASE_SHA,
        'GEN55 release not sealed/installed')
    manifest, context, full, _ = validate_result(release['actual_regression'],
        release['candidate_manifest'], release['candidate_manifest_sha256'],
        release['runner'], release['runner_sha256'], 'full')
    require(Path(manifest['workspace_root']).resolve(strict=True) == W and
        sha(release['actual_regression']) == release['actual_regression_sha256'] and
        release['source_sha256'] == context['sources'] == full['source_sha256'] and
        release['source_count'] == len(context['sources']) and release['test_summary'] == full['summary'] and
        release['test_scope_files'] == full['test_scope_files'], 'installed release scope differs')
    validate_sources(release)
    return release


def validate_previous(path):
    require(Path(path).resolve(strict=True) == PREVIOUS.resolve(strict=True) and sha(path) == PREVIOUS_SHA,
        'previous GEN54 deployment identity changed')
    previous = read(path)
    require(previous['deployment_passed'] is True and previous['whole_patch_deployed'] is True and
        previous['source_manifest_sha256'] == BASE_SHA and
        sha(previous['source_manifest']) == BASE_SHA, 'previous GEN54 deployment not accepted')
    require(len(previous['after']) == 4 and len(previous['preserved_after']) == 7, 'previous scope differs')
    expected = dict(previous['after'], **previous['preserved_after'])
    require(set(expected) == set(TARGETS+PRESERVE), 'previous eleven-node scope differs')
    return previous, expected


def validate_control(path):
    require(Path(path).resolve(strict=True) == CONTROL.resolve(strict=True), 'unexpected control path')
    value = read(path)
    require(value['state'] == 'active' and type(value['revision']) is int and value['revision'] == 58 and
        sha(path) == CONTROL_EXPECTED_SHA, 'GEN55 resume control changed')
    return value
