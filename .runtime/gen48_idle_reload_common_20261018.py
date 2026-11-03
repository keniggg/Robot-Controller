"""Shared pinned inputs for GEN48 idle reload helpers; no ROS is imported here."""
import hashlib
import json
import math
import os
from pathlib import Path
import shutil
import time
import xmlrpc.client

W = Path(__file__).resolve().parents[1]
M = Path('/media/zhuyupei/SanStand')
E = M/'实验记录/Robot-Controller'
N = E/'ros_log/full_latest_gen41_restart_20261018T011222Z'
BASE_SHA = '69e63cc71c4c63f1bbeaa2693dd88829363f06c878117f5f6129f38eaaf43e5a'
PREVIOUS = N/'gen47_runtime/gen47_runtime_deployment.json'
PREVIOUS_SHA = '6e41bb2528b44adbbe6409fcae6c1a8f61099ffc0df78b454b37d9ac16b2a637'
CONTROL = E/'ros_log/full_latest_frozen_no_skin_20261012T021050Z/continuous_authorization_20261012/control.json'
COMPUTE = ('/grasp_task_node', '/remote_grasp6d_node', '/unknown_tabletop_perception')
TARGETS = COMPUTE + ('/motion_gateway',)
PRESERVE = ('/alicia_d_driver_node', '/bessica_d_hw_interface', '/move_group',
    '/supervisor_camera', '/compliant_gripper_controller', '/grasp_mode_router',
    '/alicia_supervisor_gui')
HELPERS = tuple(W/'.runtime'/name for name in (
    'gen48_idle_reload_common_20261018.py',
    'read_gen48_deployment_preflight_20261018.py',
    'load_gen48_idle_compute_gateway_20261018.py',
    'gen48_observation_measurement_consumers_20261018.launch',
    'start_gen48_observation_measurement_consumers_20261018.sh'))
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


def validate_release(path):
    release = read(path)
    require(all(release.get(k) is True for k in ('passed', 'reviewed', 'installed')), 'GEN48 release not sealed/installed')
    require(release['baseline_release_sha256'] == BASE_SHA, 'GEN48 baseline differs from deployed GEN47')
    summary = release['test_summary']
    require(summary['tests'] > 0 and all(summary[k] == 0 for k in ('failures', 'errors', 'skipped')), 'incomplete release regression')
    require(release['source_count'] == len(release['source_sha256']) and release['source_count'] > 1052, 'source manifest incomplete')
    require(release.get('generation') == 'GEN48', 'unexpected release generation')
    full = read(release['actual_regression'])
    require(sha(release['actual_regression']) == release['actual_regression_sha256'], 'full regression bytes changed')
    require(full.get('passed') is True and full.get('full_regression_scope') is True and
        full.get('full_regression_passed') is True and full.get('scope_complete') is True and
        full.get('source_unchanged') is True and full.get('combined_xml_matches_worker_results') is True, 'GEN48 complete parallel regression is required')
    require(full['source_sha256'] == release['source_sha256'] and
        full['summary'] == summary and full['test_scope_files'] == release['test_scope_files'] and
        full['source_count'] == release['source_count'],
        'release/full regression source or scope differs')
    require(full['summary']['tests'] > 8646 and full['test_scope_files'] == 306 + release['new_test_files'],
        'GEN48 complete visibility-family test scope is missing')
    require(release.get('historical_test_files') == 306 and release.get('historical_tests') == 8646
        and release.get('supplemental_existing_test_files') == 0
        and release.get('new_test_files') == 1,
        'GEN47 historical scope or supplemental accounting differs')
    require(summary['tests'] == 8646 + release['supplemental_existing_test_cases']
        + release['new_test_cases'], 'GEN48 exact testcase accounting differs')
    validate_sources(release)
    return release


def validate_sources(release):
    for name, digest in release['source_sha256'].items():
        relative = Path(name)
        require(not relative.is_absolute() and '..' not in relative.parts and sha(W/relative) == digest, 'source changed: '+name)
    for name, digest in release['evidence_sha256'].items():
        require(sha(name) == digest, 'release evidence changed: '+name)


def validate_previous(path):
    require(Path(path).resolve() == PREVIOUS.resolve() and sha(path) == PREVIOUS_SHA, 'previous GEN47 deployment identity changed')
    previous = read(path)
    require(previous['deployment_passed'] is True and previous['whole_patch_deployed'] is True and previous['source_manifest_sha256'] == BASE_SHA, 'previous GEN47 deployment not accepted')
    require(len(previous['after']) == 4 and len(previous['preserved_after']) == 7, 'previous deployment scope differs')
    expected = dict(previous['after'], **previous['preserved_after'])
    require(set(expected) == set(TARGETS+PRESERVE), '11-node previous deployment scope differs')
    return previous, expected


def validate_control(path):
    require(Path(path).resolve() == CONTROL.resolve(), 'unexpected control file')
    value = read(path)
    require(value['state'] == 'active' and type(value['revision']) is int and value['revision'] == 52, 'expected unchanged active control revision 52')
    return value


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
