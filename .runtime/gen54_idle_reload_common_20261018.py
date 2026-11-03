"""Shared pinned inputs for GEN54 idle reload helpers; no ROS is imported here."""
import hashlib
import json
import math
import os
from pathlib import Path
import shutil
import time
import xmlrpc.client
import xml.etree.ElementTree as ET
from collections import Counter

W = Path(__file__).resolve().parents[1]
M = Path('/media/zhuyupei/SanStand')
E = M/'实验记录/Robot-Controller'
N = E/'ros_log/full_latest_gen41_restart_20261018T011222Z'
BASE_SHA = 'e9f07af55bfac1ae9fd7562d93f04748b6b2e68139f7bab3e0f645cc46ef255f'
PREVIOUS = N/'gen53_runtime/gen53_runtime_deployment.json'
PREVIOUS_SHA = 'd3e8c419d724e7eb941403c2bf6d47e0bc5f4cb77dbb549f93158d6c0d16202f'
BASELINE = W/'.runtime/gen54_baseline_source/runner_manifest.json'
BASELINE_SHA = 'e0ea9529ff6afdff8a6794578f1e64d996cdb124ee1adf3e6732e38e44984748'
HISTORICAL_XML = N/'verification/gen53_full_parallel_r1/pytest.xml'
HISTORICAL_XML_SHA = '3ce8c6f4e6de0be68745042e113f2e727f7b8f3cdc862e4728dca0523b8ebb78'
CONTROL = E/'ros_log/full_latest_frozen_no_skin_20261012T021050Z/continuous_authorization_20261012/control.json'
CONTROL_EXPECTED_SHA = '135af0f40ab4dbfc30b4d75be7c44c963263c2498484da271f056d8104966833'
COMPUTE = ('/grasp_task_node', '/remote_grasp6d_node', '/unknown_tabletop_perception')
TARGETS = COMPUTE + ('/motion_gateway',)
PRESERVE = ('/alicia_d_driver_node', '/bessica_d_hw_interface', '/move_group',
    '/supervisor_camera', '/compliant_gripper_controller', '/grasp_mode_router',
    '/alicia_supervisor_gui')
HELPERS = tuple(W/'.runtime'/name for name in (
    'gen54_idle_reload_common_20261018.py',
    'read_gen54_deployment_preflight_20261018.py',
    'load_gen54_idle_compute_gateway_20261018.py',
    'gen54_observation_measurement_consumers_20261018.launch',
    'start_gen54_observation_measurement_consumers_20261018.sh'))
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
SUPPLEMENTAL_TESTS = frozenset((
    'src/alicia_flexible_grasp_supervisor/tests/test_gripper_geometry.py',
    'src/alicia_flexible_grasp_supervisor/tests/test_tabletop_geometry_candidates.py',
    'src/alicia_grasp_modes/tests/test_contact_tilt_replay.py',
    'src/alicia_grasp_modes/tests/test_contact_placement_clearance.py',
))
# Exact three reviewed old-test updates; all other original test bytes stay fixed.
REVIEWED_OLD_TEST_CHANGES = [{'path': 'src/alicia_flexible_grasp_supervisor/tests/test_gripper_geometry.py', 'baseline_sha256': '0b2b44da8962476a81a4cdfcd9655b5e94cd20160eb6a334cd0766d9ddd3d45f', 'candidate_sha256': '4fe13a1336b611b4417ea2944ad1020d052b7586d994ed829f562415116dd06b', 'review_evidence': {'path': '/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_gen41_restart_20261018T011222Z/verification/gen54_collision_geometry_r1/near_limit_versioned_expectation_review.json', 'sha256': '431c590a5d7304a29a2a9c80316a7122529ac6c7381d41297c275b28e19b8a2a'}}, {'path': 'src/alicia_flexible_grasp_supervisor/tests/test_remote_grasp6d_streaming.py', 'baseline_sha256': '6957b2d808da254dface00bd025791362a44c98e93765980de25a74bb1e68638', 'candidate_sha256': 'b6b4c8004caadc7826969568bd81a5964d65b87299d5836898cd72ea4f1d9c77', 'review_evidence': {'path': '/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_gen41_restart_20261018T011222Z/verification/gen54_old_expectation_review_r1/recorded_expectation_review.json', 'sha256': 'f1884d032e508c43e268d3a2ba2929b9127199a2643ac1bf7661b5066b52a1de'}}, {'path': 'src/alicia_flexible_grasp_supervisor/tests/test_contact_height_window_proposals.py', 'baseline_sha256': '331388555feb1323fdb8013e4745113826662d4013796d0462e2c0393be87a11', 'candidate_sha256': 'd8ed14b42ccb3a1a8681e7791490097143f7a14d97ca7bb392fbf3f4e4b52201', 'review_evidence': {'path': '/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_gen41_restart_20261018T011222Z/verification/gen54_geometry_pipeline_r1/height_cardinality_only_old_test_review_r1.json', 'sha256': '55df9ba96aed22b77be4db213859b41c59cdea3d90b0ffc853a85471109cbf1b'}}]
SUPPLEMENTAL_CAD_INPUTS = {'src/arm-mujoco/synriard/urdf/Alicia_D_v5_6/Alicia_D_v5_6_gripper_50mm.urdf': '7e08a01f1a6cf3b45ed78a26d3a9f5939e2bb6bc33f6bc01032db097882edaeb', 'src/arm-mujoco/synriard/meshes/Alicia_D_v5_6/gripper_50mm/Link6.STL': 'cff46c5522d1a869b0397965adb885a7f4c62c5b137b409d8df605aab3cc3d5d', 'src/arm-mujoco/synriard/meshes/Alicia_D_v5_6/gripper_50mm/Link7.STL': 'd546b7ab908c74281a41261412089a2f83f7519c2444739afe8ffff46450d19d', 'src/arm-mujoco/synriard/meshes/Alicia_D_v5_6/gripper_50mm/Link8.STL': '5e5b548734269b731c1d60502c742e3d70d5a97a1e8316db523e2232b4c5043b', 'src/real-arm/alicia_d_descriptions/urdf/alicia_duo_with_gripper.urdf': '890592a24b8ad818d995e69beb670ea4254bf84f589615bfac3c8562f62449b7', 'src/real-arm/alicia_d_descriptions/meshes/Link7.STL': 'ff006151147d25c431d868fc6a0f46b96275690d30526d13c741ffe51ac6fcea', 'src/real-arm/alicia_d_descriptions/meshes/Link8.STL': 'fd49b1a4c9ecf39854d0de1ee3645239aae2427cca6ce7a24b195bf041c6098e', 'src/real-arm/alicia_d_moveit/config/alicia_d_descriptions.srdf': 'ac14d9a4de083d30444b3c8e762bc166a61382ec334ac455a93ac792ae6a42c2'}
SUPPLEMENTAL_ORIGINAL_COLLECTION = N/'verification/gen54_old_geometry_collect_r1/result.json'
SUPPLEMENTAL_ORIGINAL_COLLECTION_SHA = '8658aaa84ba970a2b93fda982a4527740e2aedef722df7739563b50365fee29e'



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
    require(all(release.get(k) is True for k in ('passed', 'reviewed', 'installed')), 'GEN54 release not sealed/installed')
    require(release['baseline_release_sha256'] == BASE_SHA, 'GEN54 baseline differs from deployed GEN53')
    summary = release['test_summary']
    require(summary['tests'] > 0 and all(summary[k] == 0 for k in ('failures', 'errors', 'skipped')), 'incomplete release regression')
    require(type(release['source_count']) is int and release['source_count'] == len(release['source_sha256']), 'source manifest incomplete')
    require(release.get('generation') == 'GEN54', 'unexpected release generation')
    full = read(release['actual_regression'])
    require(sha(release['actual_regression']) == release['actual_regression_sha256'], 'full regression bytes changed')
    require(full.get('passed') is True and full.get('full_regression_scope') is True and
        full.get('full_regression_passed') is True and full.get('scope_complete') is True and
        full.get('source_unchanged') is True and full.get('combined_xml_matches_worker_results') is True, 'GEN54 complete parallel regression is required')
    require(full['source_sha256'] == release['source_sha256'] and
        full['summary'] == summary and full['test_scope_files'] == release['test_scope_files'] and
        full['source_count'] == release['source_count'],
        'release/full regression source or scope differs')
    validate_candidate_scope(release, full)
    validate_sources(release)
    return release



def case_inventory(path):
    tree = ET.parse(path).getroot()
    summary = {key: sum(int(suite.get(key, 0)) for suite in tree.iter('testsuite'))
        for key in ('tests', 'failures', 'errors', 'skipped')}
    cases = [(case.get('file', ''), case.get('classname', ''), case.get('name', ''))
        for case in tree.iter('testcase')]
    require(summary['tests'] == len(cases) == len(set(cases)), 'incomplete/duplicate XML case identities')
    for key, tag in (('failures', 'failure'), ('errors', 'error'), ('skipped', 'skipped')):
        require(summary[key] == sum(len(case.findall(tag)) for case in tree.iter('testcase')),
            'XML child outcome differs: '+key)
    return summary, cases


def validate_candidate_scope(release, full):
    # Counts and identities come from the final installed-W manifest, not a
    # guessed GEN54 scope or an earlier isolated focus manifest.
    manifest_path = Path(release['candidate_manifest']).resolve(strict=True)
    require(inside(manifest_path, N/'verification') and
        sha(manifest_path) == release['candidate_manifest_sha256'] == full['candidate_manifest_sha256'] and
        Path(full['candidate_manifest_path']).resolve(strict=True) == manifest_path,
        'sealed release/full candidate manifest differs')
    manifest = read(manifest_path)
    require(manifest['kind'] == 'GEN54_frozen_offline_candidate' and
        Path(manifest['workspace_root']).resolve(strict=True) == W.resolve(strict=True) and
        manifest.get('variant', 'candidate') == 'candidate' and manifest.get('full_scope_ready') is True,
        'GEN54 full manifest is not the installed workspace')
    require(sha(BASELINE) == BASELINE_SHA and
        Path(manifest['baseline_manifest']['path']).resolve(strict=True) == BASELINE.resolve(strict=True) and
        manifest['baseline_manifest']['sha256'] == BASELINE_SHA,
        'GEN54 historical baseline manifest differs')
    baseline = read(BASELINE)
    require(baseline['baseline_generation'] == 'GEN53' and
        baseline['baseline_release_sha256'] == BASE_SHA and baseline['source_count'] == 1069 and
        len(baseline['source_sha256']) == 1069 and baseline['test_scope_files'] == 322 and
        baseline['test_summary'] == dict(tests=9010, failures=0, errors=0, skipped=0),
        'GEN53 historical source/scope differs')
    require(sha(HISTORICAL_XML) == HISTORICAL_XML_SHA, 'GEN53 historical XML changed')
    old_summary, old_cases = case_inventory(HISTORICAL_XML)
    require(old_summary == baseline['test_summary'] and
        dict(Counter(case[0] for case in old_cases)) == baseline['test_case_counts_by_file'],
        'GEN53 historical exact inventory differs')
    require(isinstance(REVIEWED_OLD_TEST_CHANGES, list) and REVIEWED_OLD_TEST_CHANGES,
        'GEN54 reviewed old-test exception inventory is not yet bound')
    approved_rows = manifest.get('approved_old_test_changes', [])
    require(approved_rows == REVIEWED_OLD_TEST_CHANGES,
        'GEN54 old-test exceptions differ from the exact reviewed rows')
    approved = {row['path']: row for row in approved_rows}
    require(len(approved) == len(approved_rows), 'duplicate reviewed old-test paths')
    for relative, row in approved.items():
        require(relative in baseline['source_sha256'] and '/tests/test_' in relative and
            row['baseline_sha256'] == baseline['source_sha256'][relative] and
            row['candidate_sha256'] == manifest['source_sha256'][relative],
            'reviewed old-test source binding differs: '+relative)
        evidence = Path(row['review_evidence']['path']).resolve(strict=True)
        require(inside(evidence, N/'verification') and sha(evidence) == row['review_evidence']['sha256'] and
            release['evidence_sha256'].get(str(evidence)) == row['review_evidence']['sha256'],
            'reviewed old-test evidence differs: '+relative)
    require(sha(SUPPLEMENTAL_ORIGINAL_COLLECTION) == SUPPLEMENTAL_ORIGINAL_COLLECTION_SHA,
        'original supplemental geometry inventory changed')
    original_supplement = read(SUPPLEMENTAL_ORIGINAL_COLLECTION)
    original_supplement_cases = [tuple(case) for case in original_supplement['collected_case_identities']]
    require(original_supplement['mode'] == 'collect' and original_supplement['passed'] is True and
        original_supplement['test_bodies_executed'] is False and
        len(original_supplement_cases) == len(set(original_supplement_cases)) == 206,
        'original 206 supplemental case identities are incomplete')
    for relative, digest in SUPPLEMENTAL_CAD_INPUTS.items():
        path = W/relative
        require(path.resolve(strict=True) == path and relative not in manifest['source_sha256'] and
            sha(path) == digest and release['evidence_sha256'].get(str(path)) == digest,
            'readonly CAD dependency differs: '+relative)
    new = manifest['new_tests']
    supplemental = manifest.get('supplemental_existing_tests', [])
    require(len(supplemental) == len(SUPPLEMENTAL_TESTS) and
        {row['path'] for row in supplemental} == SUPPLEMENTAL_TESTS and
        all(row['sha256'] == (approved[row['path']]['candidate_sha256']
            if row['path'] in approved else baseline['source_sha256'][row['path']])
            for row in supplemental), 'GEN54 mandatory existing geometry tests differ')
    require(sorted(tuple(case) for row in supplemental for case in row['case_identities']) ==
        sorted(original_supplement_cases), 'GEN54 changed an original supplemental testcase identity')
    expected_cases = list(old_cases)
    scope = set(baseline['test_scope'])
    require(len(scope) == 322 and new, 'GEN54 collected additions required')
    for row in supplemental+new:
        path = row['path']; cases = [tuple(case) for case in row['case_identities']]
        require(path not in scope and type(row['expected_count']) is int and
            row['expected_count'] > 0 and len(cases) == row['expected_count'] and
            all(len(case) == 3 and case[0] == path for case in cases),
            'GEN54 collected file/case inventory differs: '+path)
        scope.add(path); expected_cases.extend(cases)
    require(len(expected_cases) == len(set(expected_cases)), 'duplicate GEN54 expected case identities')
    xml_path = Path(release['actual_regression']).resolve(strict=True).parent/'pytest.xml'
    require(sha(xml_path) == full['xml_sha256'], 'GEN54 complete XML changed')
    actual_summary, actual_cases = case_inventory(xml_path)
    require(actual_summary == release['test_summary'] and
        sorted(actual_cases) == sorted(expected_cases) and
        dict(Counter(case[0] for case in actual_cases)) == full['test_case_counts_by_file'] and
        full['test_scope_files'] == release['test_scope_files'] == len(scope),
        'GEN54 full exact case inventory differs')
    require(release.get('historical_test_files') == 322 and release.get('historical_tests') == 9010 and
        release.get('supplemental_existing_test_files') == len(supplemental) and
        release.get('new_test_files') == len(new) and
        release.get('supplemental_existing_test_cases') == sum(row['expected_count'] for row in supplemental) and
        release.get('new_test_cases') == sum(row['expected_count'] for row in new),
        'GEN54 release collected-scope accounting differs')
    require(manifest['source_sha256'] == release['source_sha256'] == full['source_sha256'],
        'GEN54 final manifest/source map differs')


def validate_sources(release):
    for name, digest in release['source_sha256'].items():
        relative = Path(name)
        require(not relative.is_absolute() and '..' not in relative.parts and sha(W/relative) == digest, 'source changed: '+name)
    for name, digest in release['evidence_sha256'].items():
        require(sha(name) == digest, 'release evidence changed: '+name)


def validate_previous(path):
    require(Path(path).resolve() == PREVIOUS.resolve() and sha(path) == PREVIOUS_SHA, 'previous GEN53 deployment identity changed')
    previous = read(path)
    require(previous['deployment_passed'] is True and previous['whole_patch_deployed'] is True and previous['source_manifest_sha256'] == BASE_SHA, 'previous GEN53 deployment not accepted')
    require(len(previous['after']) == 4 and len(previous['preserved_after']) == 7, 'previous deployment scope differs')
    expected = dict(previous['after'], **previous['preserved_after'])
    require(set(expected) == set(TARGETS+PRESERVE), '11-node previous deployment scope differs')
    return previous, expected


def validate_control(path):
    require(Path(path).resolve() == CONTROL.resolve(), 'unexpected control file')
    value = read(path)
    require(value['state'] == 'active' and type(value['revision']) is int and value['revision'] == 56, 'expected unchanged active control revision 56')
    require(sha(path) == CONTROL_EXPECTED_SHA, 'control differs from accepted GEN54 resume receipt')
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
