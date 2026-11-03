"""GEN57 pinned release and idle reload checks. No ROS on import."""
import hashlib, importlib.util, json, math, os, shutil, time
from pathlib import Path
import xmlrpc.client
from collections import Counter
from gen57_offline_result_verifier_r2_20261020 import candidate_context, validate_result
W = Path(__file__).resolve().parents[1]
M = Path('/media/zhuyupei/SanStand')
E = M/'实验记录/Robot-Controller'
N = E/'ros_log/full_latest_gen41_restart_20261018T011222Z'
BASE_SHA = '784af71745650269b1d114a5f9cbba32eaa792f8cf2029825eb5a56d52a8f255'
PREVIOUS = N/'gen56_runtime/gen56_runtime_deployment.json'
PREVIOUS_SHA = 'd02a3dde8f8b7d52b93ed57ce958a64526a68d11a912dd518435ffcd74995f85'
BASELINE = W/'.runtime/gen57_baseline_source/runner_manifest.json'
BASELINE_SHA = 'f570485f6f460648fc1bdc62c4804ed9ed69082e43b4b982273fcb3a57a1da3e'
HISTORICAL_XML = N/'verification/gen56_full_parallel_r2/pytest.xml'
HISTORICAL_XML_SHA = 'd58c113a4c3d5bc75df9ffc004cad0fe54057965043bfe4508726fa569fa5116'
CONTROL = E/'ros_log/full_latest_frozen_no_skin_20261012T021050Z/continuous_authorization_20261012/control.json'
CONTROL_EXPECTED_SHA = 'b2177bee366a8b5ef90ebd9e9fc4373a6bb8305599ab8cb68235c49e36cea662'
CONTROL_SNAPSHOT = N/'gen57_resume_r2/control.active66.json'
RESUME_REFERENCE = N/'gen57_resume_r2/reference_r1/result.json'
RESUME_REFERENCE_SHA = '0236ad9e5183def17a4198c5306c61de50ba8158a85c542d5dba40588c77c1e2'
RESUME_COLLECTOR = W/'.runtime/capture_gen57_resume_control66_reference_r1_20261020.py'
RESUME_COLLECTOR_SHA = 'a6b099c01fc7d3246b4a695d42d14300a87c21ec832c9d13701e5569f800116b'
COMPUTE = ('/grasp_task_node', '/remote_grasp6d_node', '/unknown_tabletop_perception')
TARGETS = COMPUTE + ('/motion_gateway',)
PRESERVE = ('/alicia_d_driver_node', '/bessica_d_hw_interface', '/move_group',
    '/supervisor_camera', '/compliant_gripper_controller', '/grasp_mode_router',
    '/alicia_supervisor_gui')
HELPERS = tuple(W/'.runtime'/name for name in (
    'gen57_runtime_common_20261020.py',
    'gen57_offline_result_verifier_r2_20261020.py',
    'read_gen57_deployment_preflight_20261020.py',
    'load_gen57_idle_compute_gateway_20261020.py',
    'gen57_observation_measurement_consumers_20261020.launch',
    'start_gen57_observation_measurement_consumers_20261020.sh'))
GATEWAY_SERVICES = ('/supervisor/check_contact_sequence', '/supervisor/check_pose_strict',
    '/supervisor/check_observation_pose_strict')
PARAMETERS = ('/grasp/unknown_observe_return_max_observations',
    '/grasp/unknown_lift_after_close_enabled',
    '/unknown_perception/support_max_local_shift_m',
    '/unknown_perception/support_local_shift_warning_m',
    '/grasp_6d/remote/multiview/maximum_support_offset_delta_m',
    '/unknown_perception/detect_hz',
    '/alicia_d_driver_node/software_temperature_monitoring_enabled',
    '/alicia_d_driver_node/temperature_poll_rate_hz',
    '/grasp_6d/remote/server_url',
    '/grasp_6d/remote/require_candidate_depth',
    '/grasp_6d/remote/timeout_sec')

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





def validate_release(path):
    release = read(path)
    require(all(release.get(k) is True for k in ('passed','reviewed','installed')) and
        release['generation'] == 'GEN57' and release['baseline_release_sha256'] == BASE_SHA,
        'GEN57 release not sealed/installed')
    manifest, context, full, _ = validate_result(release['actual_regression'],
        release['candidate_manifest'], release['candidate_manifest_sha256'],
        release['runner'], release['runner_sha256'], 'full')
    require(Path(manifest['workspace_root']).resolve(strict=True) in
        (W, W/'.runtime/gen57_candidate_r1') and
        sha(release['actual_regression']) == release['actual_regression_sha256'] and
        release['source_sha256'] == context['sources'] == full['source_sha256'] and
        release['source_count'] == len(context['sources']) and release['test_summary'] == full['summary'] and
        release['test_scope_files'] == full['test_scope_files'], 'installed release scope differs')
    validate_sources(release)
    return release


def validate_previous(path):
    require(Path(path).resolve(strict=True) == PREVIOUS.resolve(strict=True) and sha(path) == PREVIOUS_SHA,
        'previous GEN56 deployment identity changed')
    previous = read(path)
    require(previous['deployment_passed'] is True and previous['whole_patch_deployed'] is True and
        previous['source_manifest_sha256'] == BASE_SHA and
        sha(previous['source_manifest']) == BASE_SHA, 'previous GEN56 deployment not accepted')
    require(len(previous['after']) == 4 and len(previous['preserved_after']) == 7, 'previous scope differs')
    expected = dict(previous['after'], **previous['preserved_after'])
    require(set(expected) == set(TARGETS+PRESERVE), 'previous eleven-node scope differs')
    return previous, expected


def validate_control(path):
    require(Path(path).resolve(strict=True) == CONTROL.resolve(strict=True), 'unexpected control path')
    value = read(path)
    require(value['state'] == 'active' and type(value['revision']) is int and value['revision'] == 66 and
        sha(path) == CONTROL_EXPECTED_SHA, 'GEN57 resume control changed')
    return value


def validate_resume_reference():
    """Accept only the fixed, reviewed idle SDK reference; grant no motion."""
    require(sha(RESUME_REFERENCE) == RESUME_REFERENCE_SHA and
        sha(RESUME_COLLECTOR) == RESUME_COLLECTOR_SHA and
        sha(CONTROL_SNAPSHOT) == CONTROL_EXPECTED_SHA, 'resumed reference evidence changed')
    value = read(RESUME_REFERENCE)
    previous, expected = validate_previous(PREVIOUS)
    base = N/'verification/gen56_release_manifest.json'
    require(sha(base) == BASE_SHA, 'resumed reference baseline release changed')
    control = validate_control(CONTROL)
    require(value['kind'] == 'GEN57_resumed_reference_readonly_acceptance' and
        value['passed'] is True and value['helper_path'] == str(RESUME_COLLECTOR) and
        value['helper_sha256'] == RESUME_COLLECTOR_SHA and
        value['release_path'] == str(base) and value['deployment_path'] == str(PREVIOUS) and
        value['control_path'] == str(CONTROL) and
        value['input_sha256'] == {str(base): BASE_SHA, str(PREVIOUS): PREVIOUS_SHA,
            str(CONTROL): CONTROL_EXPECTED_SHA} and
        value['control_snapshot'] == control == read(CONTROL_SNAPSHOT),
        'resumed reference input binding differs')
    require(value['source_sha256'] == previous['source_sha256'] == read(base)['source_sha256'] and
        value['source_count'] == len(value['source_sha256']) == 1085 and
        value['node_identities_before'] == value['node_identities_after'] ==
        value['node_identities'] == expected, 'resumed reference source/process baseline differs')
    epoch = value['epoch_ns']
    require(type(epoch) is int and epoch > 0 and
        value['epoch_before_ns'] == value['epoch_after_ns'] == epoch and
        value['historical_epoch_ns'] == previous['epoch_ns'],
        'resumed reference epoch was not stable')
    selection = value['selection']
    require(value['selection_before'] == value['selection_after'] == selection and
        selection['mode'] == 'unknown' and selection['strategy'] == 'observe_return' and
        type(selection['generation']) is int and
        selection['generation'] >= previous['selection']['generation'] and
        int(selection['stamp_ns']) >= int(previous['selection']['stamp_ns']),
        'resumed reference selection differs')
    require(all(value[k] is True for k in ('manual_mode', 'feedback_ready')) and
        all(value[k] is False for k in ('task_active', 'router_active', 'actions_active',
            'scene_authorized', 'motion_authorized', 'motion_commands_sent',
            'temperature_queried', 'joint_disable_commands_sent')) and
        not value['missing_topics'] and set(value['action_statuses']) == {'arm', 'hand'} and
        not any(s in (0, 1, 6, 7) for row in value['action_statuses'].values() for s in row) and
        type(value['motion_enabled']) is bool and
        isinstance(value['actuation_status'], str) and bool(value['actuation_status']),
        'resumed reference is not a complete idle observation')
    samples = value['samples']
    require(type(value['sample_count']) is int and len(samples) == value['sample_count'] >= 20 and
        type(value['unique_sample_count']) is int and
        len({s['stamp_ns'] for s in samples}) == value['unique_sample_count'] >= 20,
        'resumed reference needs twenty unique SDK samples')
    names = samples[-1]['names']
    require(names and len(set(names)) == len(names) and
        {'Joint%d' % i for i in range(1, 7)} <= set(names),
        'resumed reference joint layout invalid')
    require(type(value['started_ns']) is int and type(value['recorded_ns']) is int and
        value['started_ns'] <= value['recorded_ns'] and
        all(s['frame_id'] == 'sdk_measured' and s['names'] == names and
            len(s['positions']) == len(names) and
            all(type(q) in (int, float) and math.isfinite(q) for q in s['positions']) and
            type(s['stamp_ns']) is int and type(s['received_ns']) is int and
            epoch <= s['stamp_ns'] <= s['received_ns'] and
            value['started_ns'] <= s['received_ns'] <= value['recorded_ns'] and
            s['received_ns'] - s['stamp_ns'] < 200_000_000 for s in samples),
        'resumed reference has invalid or stale SDK samples')
    age = value['recorded_ns'] - samples[-1]['stamp_ns']
    spans = {name: max(s['positions'][i] for s in samples) -
        min(s['positions'][i] for s in samples) for i, name in enumerate(names)}
    require(value['latest_feedback_age_ns'] == age and 0 <= age < 200_000_000 and
        value['sample_spans'] == spans and all(span <= .004 for span in spans.values()),
        'resumed reference SDK samples were stale or unstable')
    return value
