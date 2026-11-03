"""Accept the GEN42 idle reload against a frozen current preflight.

Only ROS reads and atomic metadata writes are used. This helper never starts a
grasp, changes control, reloads a node, sends motion, or queries temperature.
GEN41 deployment history and the sealed release remain immutable. Control
fields in the new progress file describe deployment time only.
"""
import argparse
import fcntl
import hashlib
import json
import math
import os
from pathlib import Path
import time
import xmlrpc.client

from continuous_grasp_authorization_20261012 import atomic_json
from alicia_grasp_modes.selection import parse_selection
from verify_gen42_offline_tests_20261017 import W, E, M, BASE, BASE_SHA, sha


COMPUTE = frozenset(('/grasp_task_node', '/remote_grasp6d_node', '/unknown_tabletop_perception'))
TARGETS = COMPUTE
PRESERVED = frozenset(('/alicia_d_driver_node', '/bessica_d_hw_interface', '/move_group',
    '/supervisor_camera', '/compliant_gripper_controller', '/grasp_mode_router',
    '/alicia_supervisor_gui', '/motion_gateway'))
ALL_NODES = TARGETS | PRESERVED
RESTART = E/'ros_log/full_latest_gen41_restart_20261018T011222Z'
RUNTIME = RESTART/'gen42_runtime'
PREVIOUS = RESTART/'cold_restart_acceptance_r2/result.json'
PREVIOUS_SHA = 'ccada9df3f667986370bdbae6f4affe7dc241bbd84e5d169df2f9700b513c0a2'
CONTROL = E/'ros_log/full_latest_frozen_no_skin_20261012T021050Z/continuous_authorization_20261012/control.json'


def require(condition, reason):
    if not condition:
        raise ValueError(reason)


def read(path):
    return json.loads(Path(path).read_text())


def write_verified(path, value):
    atomic_json(path, value)
    require(read(path) == value, 'metadata read-back differs: '+str(path))


def merge(left, right):
    require(isinstance(left, dict) and isinstance(right, dict) and not set(left) & set(right),
        'node identity maps overlap or are not mappings')
    return dict(left, **right)


def identities_valid(rows, names):
    require(isinstance(rows, dict) and set(rows) == set(names), 'node identity set differs')
    for name, row in rows.items():
        require(isinstance(row, dict) and set(row) == {'pid', 'parent_pid', 'start_ticks'},
            'process identity fields differ: '+name)
        require(type(row['pid']) is int and row['pid'] > 0 and
            type(row['parent_pid']) is int and row['parent_pid'] > 0 and
            isinstance(row['start_ticks'], str) and row['start_ticks'].isdigit() and
            int(row['start_ticks']) > 0, 'invalid process identity: '+name)


def deployment_chain(previous, preflight, compute):
    """Pure receipt consistency; current ROS identities are checked separately."""
    baseline = merge(previous['after'], previous['preserved_after'])
    identities_valid(baseline, ALL_NODES)
    identities_valid(preflight['node_identities'], ALL_NODES)
    require(preflight['node_identities'] == baseline,
        'current preflight processes differ from GEN41 runtime deployment')
    identities_valid(compute['before'], COMPUTE)
    identities_valid(compute['after'], COMPUTE)
    identities_valid(compute['preserved_before'], ALL_NODES-COMPUTE)
    require(merge(compute['before'], compute['preserved_before']) == preflight['node_identities'],
        'compute before does not equal frozen current preflight')
    require(compute['preserved_before'] == compute['preserved_after'],
        'compute reload changed a preserved process')
    identities_valid(compute['preserved_after'], PRESERVED)
    after = compute['after']
    identities_valid(after, TARGETS)
    preserve = {name:baseline[name] for name in PRESERVED}
    expected = merge(after, preserve)
    require(all(after[name] != baseline[name] for name in TARGETS),
        'one of three compute consumers was not replaced')
    require(compute['preserved_after'] == preserve,
        'one of eight preserved processes differs from the current baseline')
    require(len(expected) == 11 and len(after) == 3 and len(preserve) == 8,
        'three-compute / eight-preserved conservation differs')
    return baseline, after, preserve, expected


def joint_map(names, positions):
    require(isinstance(names, list) and isinstance(positions, list) and len(names) == len(positions)
        and bool(names) and len(set(names)) == len(names), 'joint feedback dimensions differ')
    require(all(isinstance(name, str) and name for name in names) and all(
        type(value) in (int, float) and math.isfinite(value) for value in positions),
        'joint feedback is not finite named data')
    return dict(zip(names, positions))


class Transport(xmlrpc.client.Transport):
    def make_connection(self, host):
        connection = super().make_connection(host)
        connection.timeout = 3.
        return connection


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ('release', 'compute-deployment', 'previous', 'session', 'preflight'):
        parser.add_argument('--'+name, required=True, type=Path)
    args = parser.parse_args()
    N = args.session.resolve(strict=True)
    require(M.is_mount() and N.stat().st_dev == M.stat().st_dev, 'SanStand session unavailable')
    N.relative_to((E/'ros_log').resolve(strict=True))
    require(N == RUNTIME.resolve(strict=True), 'new GEN42 runtime child session required')
    paths = [path.resolve(strict=True) for path in (
        args.release, args.compute_deployment, args.previous, args.preflight)]
    release_path, compute_path, previous_path, preflight_path = paths
    require(previous_path == PREVIOUS.resolve(strict=True) and sha(previous_path) == PREVIOUS_SHA,
        'previous deployment is not the accepted GEN41 cold restart r2')
    require(N in compute_path.parents, 'compute reload receipt is not in the new runtime session')
    require(N != previous_path.parent, 'new deployment must not overwrite the GEN41 session')
    output_names = ('gen42_runtime_deployment.json', 'current_progress.json',
        'runtime_deployment_status.json', 'previous_current_resumed_grasp_session.json',
        'deployment_finalization.json', 'deployment_finalization_claim.json')
    require(not any((N/name).exists() for name in output_names), 'new deployment snapshot already exists')
    pinned = [(path, path.read_bytes()) for path in paths]
    release, compute, previous, preflight = [json.loads(raw) for _, raw in pinned]
    input_sha = {str(path):hashlib.sha256(raw).hexdigest() for path, raw in pinned}
    require(all(release.get(key) is True for key in ('passed', 'reviewed', 'installed')),
        'release has not passed installation review')
    require(release['baseline_release_sha256'] == BASE_SHA and sha(BASE) == BASE_SHA and
        Path(release['baseline_release']).resolve(strict=True) == BASE.resolve(strict=True),
        'GEN41 baseline release differs')
    require(previous['source_manifest_sha256'] == BASE_SHA and
        Path(previous['source_manifest']).resolve(strict=True) == BASE.resolve(strict=True),
        'previous runtime is not the GEN41 deployment')
    require(previous['deployment_passed'] is True and previous['whole_patch_deployed'] is True,
        'previous deployment was not accepted')
    require(previous['full_restart'] is True and previous['all_processes_replaced'] is True
        and previous['preserved_after'] == {}, 'previous receipt is not the complete cold restart')
    require(compute['deployment_passed'] is True, 'compute reload receipt did not pass')
    require(preflight['passed'] is True and preflight['manual_mode'] is True and
        preflight['task_active'] is False and preflight['actions_active'] is False,
        'preflight is not manual idle')
    require(type(preflight['epoch_ns']) is int and preflight['epoch_ns'] > 0 and
        type(preflight['recorded_ns']) is int and 0 < preflight['recorded_ns'] <= time.time_ns() and
        type(preflight['sample_count']) is int and preflight['sample_count'] >= 20,
        'preflight feedback provenance invalid')
    require(Path(preflight['source_manifest']).resolve(strict=True) == release_path and
        preflight['source_manifest_sha256'] == input_sha[str(release_path)] and
        Path(preflight['previous_deployment']).resolve(strict=True) == previous_path and
        preflight['previous_deployment_sha256'] == input_sha[str(previous_path)],
        'preflight release or deployment binding differs')
    source = release['source_sha256']
    require(release['source_count'] == len(source) and compute['source_sha256'] == source,
        'new source manifest differs from the compute reload')
    baseline, after, preserve, expected = deployment_chain(previous, preflight, compute)
    require(Path(compute['deployment_preflight']).resolve(strict=True) == preflight_path,
        'compute reload is not bound to the current GEN42 preflight')
    for service in ('/grasp_mode/reset_remote', '/grasp_mode/reset_task'):
        reply = json.loads(compute['reset_ack'][service])
        require(reply.get('ready') is True and parse_selection(reply) ==
            parse_selection(preflight['selection']), 'compute reset ACK differs: '+service)
    control_path = Path(preflight['control_path']).resolve(strict=True)
    control_path.relative_to((E/'ros_log').resolve(strict=True))
    require(control_path == CONTROL.resolve(strict=True), 'unexpected authoritative control path')
    control = read(control_path)
    require(control == preflight['control_snapshot'], 'authoritative control differs from frozen preflight')
    require(control['state'] == 'paused' and type(control['revision']) is int
        and control['revision'] == 47, 'paused control revision 47 changed')
    input_sha[str(control_path)] = sha(control_path)
    require(preflight['control_sha256'] == input_sha[str(control_path)],
        'control bytes differ from current preflight')
    preflight_inputs = {str(Path(path).resolve(strict=True)):digest
        for path, digest in preflight['input_sha256'].items()}
    require(len(preflight_inputs) == len(preflight['input_sha256']) == 3 and preflight_inputs == {
        str(path):input_sha[str(path)] for path in (release_path, previous_path, control_path)},
        'preflight original release/deployment/control bytes differ')
    joint_map(preflight['names'], preflight['positions'])
    pointer_path = W/'.runtime/current_resumed_grasp_session.json'
    pointer_bytes = pointer_path.read_bytes()
    pointer_before = json.loads(pointer_bytes)
    pointer_sha = hashlib.sha256(pointer_bytes).hexdigest()
    finalizer_sha = sha(__file__)
    require(Path(pointer_before['directory']).resolve(strict=True) == RESTART.resolve(strict=True),
        'current runtime pointer is not the accepted cold GEN41 session')

    def frozen_inputs():
        require(all(sha(path) == digest for path, digest in input_sha.items()), 'frozen input changed')
        require(read(control_path) == preflight['control_snapshot'], 'authoritative control changed')
        require(all(not Path(name).is_absolute() and '..' not in Path(name).parts and
            sha(W/name) == digest for name, digest in source.items()), 'installed source changed')
        require(all(sha(path) == digest for path, digest in release['evidence_sha256'].items()),
            'release evidence changed')
        require(sha(__file__) == finalizer_sha, 'finalizer source changed')

    frozen_inputs()
    os.environ['ROS_LOG_DIR'] = str(N/'deployment_observer_ros')
    import rosgraph
    import rospy
    from actionlib_msgs.msg import GoalStatusArray
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Header, String
    from alicia_flexible_grasp_supervisor.msg import GraspState
    rospy.init_node('gen42_idle_deployment_readonly', anonymous=True, disable_rosout=True)
    master = rosgraph.Master(rospy.get_name())

    def identities():
        for name, row in expected.items():
            code, _, pid = xmlrpc.client.ServerProxy(master.lookupNode(name), transport=Transport()).getPid(rospy.get_name())
            require(code == 1 and pid == row['pid'], 'live PID differs: '+name)
            fields = (Path('/proc')/str(pid)/'stat').read_text().rsplit(')', 1)[1].split()
            require(fields[0] != 'Z' and fields[19] == row['start_ticks'] and
                int(fields[1]) == row['parent_pid'], 'live process incarnation differs: '+name)

    def idle():
        require(rospy.get_param('/gui/joint_direct_mode') is True, 'manual ownership changed')
        require(not rospy.wait_for_message('/grasp/state', GraspState, timeout=5.).active, 'grasp task active')
        mode = json.loads(rospy.wait_for_message('/grasp_mode/status', String, timeout=5.).data)
        require(not any(mode.get(key) for key in ('active', 'start_reserved', 'pending')), 'grasp mode active')
        for topic in ('/alicia_controller/follow_joint_trajectory/status', '/hand_controller/follow_joint_trajectory/status'):
            message = rospy.wait_for_message(topic, GoalStatusArray, timeout=5.)
            require(not any(goal.status in (0, 1, 6, 7) for goal in message.status_list), 'action active: '+topic)

    identities()
    idle()
    epoch = rospy.wait_for_message('/alicia_d/control_reference_epoch', Header, timeout=5.).stamp.to_nsec()
    require(epoch == preflight['epoch_ns'], 'current epoch differs from current preflight')
    require(rospy.get_param('/grasp_mode/selection') == preflight['selection'], 'current selection differs')
    policy = {'/grasp/unknown_observe_return_max_observations':1,
        '/grasp/unknown_lift_after_close_enabled':False,
        '/alicia_d_driver_node/software_temperature_monitoring_enabled':False,
        '/alicia_d_driver_node/temperature_poll_rate_hz':0.0}
    require(all(rospy.get_param(key) == value for key, value in policy.items()), 'runtime policy differs')
    frozen_inputs()
    identities()
    idle()
    final_epoch = rospy.wait_for_message('/alicia_d/control_reference_epoch', Header, timeout=5.).stamp.to_nsec()
    require(final_epoch == preflight['epoch_ns'] and rospy.get_param('/grasp_mode/selection') ==
        preflight['selection'], 'epoch or selection changed during final acceptance')
    q = rospy.wait_for_message('/alicia_d/accepted_joint_states', JointState, timeout=5.)
    require(q.header.frame_id == 'sdk_measured' and q.header.stamp.to_nsec() >= epoch
        and 0 <= (rospy.Time.now()-q.header.stamp).to_sec() < .2,
        'fresh accepted measured feedback unavailable')
    positions = joint_map(list(q.name), list(q.position))
    reference = joint_map(preflight['names'], preflight['positions'])
    require(set(positions) == set(reference) and all(abs(positions[name]-value) <= .004
        for name, value in reference.items()), 'accepted posture differs from current preflight feedback')
    require(all(rospy.get_param(key) == value for key, value in policy.items()),
        'runtime policy changed during acceptance')
    frozen_inputs()
    # Source/evidence hashing may take longer than a feedback period. Keep the
    # final saved SDK sample fresh after those reads; this grants no motion.
    idle()
    require(rospy.get_param('/grasp_mode/selection') == preflight['selection'] and
        rospy.wait_for_message('/alicia_d/control_reference_epoch', Header, timeout=5.).stamp.to_nsec()
        == preflight['epoch_ns'], 'selection or epoch changed before publication')
    q = rospy.wait_for_message('/alicia_d/accepted_joint_states', JointState, timeout=5.)
    require(q.header.frame_id == 'sdk_measured' and q.header.stamp.to_nsec() >= epoch
        and 0 <= (rospy.Time.now()-q.header.stamp).to_sec() < .2,
        'final accepted measured feedback unavailable')
    positions = joint_map(list(q.name), list(q.position))
    require(set(positions) == set(reference) and all(abs(positions[name]-value) <= .004
        for name, value in reference.items()), 'final accepted posture differs from current preflight')
    require(sha(pointer_path) == pointer_sha, 'current runtime pointer changed during acceptance')
    deployment_path = N/'gen42_runtime_deployment.json'
    status_path = N/'runtime_deployment_status.json'
    feedback = dict(names=list(q.name), positions=list(q.position), epoch_ns=epoch,
        stamp_ns=q.header.stamp.to_nsec(), frame_id=q.header.frame_id)
    deployment = dict(deployment_passed=True, whole_patch_deployed=True,
        deployment_kind='idle_three_compute_reload_eight_preserved_including_gateway',
        source_manifest=str(release_path), source_manifest_sha256=sha(release_path), source_sha256=source,
        test_summary=release['test_summary'], before={name:baseline[name] for name in TARGETS},
        after=after, preserved_before=preserve, preserved_after=preserve, unit=compute['unit'],
        previous_deployment=str(previous_path), preflight=str(preflight_path),
        compute_deployment=str(compute_path), replaced_count=3, preserved_count=8,
        evidence_sha256=input_sha, manual_mode=True, task_active=False, actions_active=False,
        selection=preflight['selection'], epoch_ns=epoch, feedback=feedback,
        control_path=str(control_path), control_state=control['state'], control_revision=control['revision'],
        control_snapshot=control, control_unchanged=True, recorded_ns=time.time_ns(),
        motion_commands_sent=False, temperature_queried=False, torque_disable_commands_sent=False,
        lifecycle_commands_sent=False, physical_grasp_verified=False,
        new_selection_required_before_automatic_grasp=True, scene_permission=False,
        feedback_reference='frozen current GEN42 preflight; no gateway reload')
    progress = dict(runtime_session=str(N), directory=str(N),
        status='gen42_deployed_manual_idle_pending_fresh_grasp_scene',
        snapshot_semantics='immutable_deployment_time_snapshot; authoritative dynamic control file unchanged',
        deployment_passed=True, whole_patch_deployed=True, gen42_deployed=True,
        source_manifest=str(release_path), source_manifest_sha256=sha(release_path),
        latest_deployment=str(deployment_path), deployment_status=str(status_path),
        current_test_result=release['actual_regression'], source_count=release['source_count'],
        test_scope_files=release['test_scope_files'], test_summary=release['test_summary'],
        deployed_node_identities=expected, selection=preflight['selection'], epoch_ns=epoch,
        control_path=str(control_path), control_state=control['state'], control_revision=control['revision'],
        control_snapshot=control, manual_mode=True, task_active=False, actions_active=False,
        automatic_grasp_started=False, new_motion_authorized_by_this_finalization=False,
        pending_work='Deployment acceptance only; subsequent grasp uses existing authoritative control and fresh scene, robot and path validation.',
        physical_grasp_verified=False, updated_ns=time.time_ns())
    # Brief shared metadata lock only: never hold control ownership while
    # waiting for ROS or hashing the full source manifest. If an operator is
    # updating control, fail immediately rather than delaying that update.
    with (control_path.parent/'control.lock').open('rb') as lock:
        fcntl.flock(lock, fcntl.LOCK_SH | fcntl.LOCK_NB)
        require(read(control_path) == preflight['control_snapshot'] and
            sha(control_path) == input_sha[str(control_path)], 'control changed before snapshot publication')
        require(sha(pointer_path) == pointer_sha, 'current pointer changed before publication')
        with (N/'deployment_finalization_claim.json').open('x') as claim:
            json.dump(dict(claimed_ns=time.time_ns(), finalizer_sha256=finalizer_sha,
                pointer_before_sha256=pointer_sha, input_sha256=input_sha), claim, indent=2)
            claim.flush()
            os.fsync(claim.fileno())
        write_verified(deployment_path, deployment)
        write_verified(N/'current_progress.json', progress)
        write_verified(status_path, dict(source_manifest=str(release_path), source_manifest_sha256=sha(release_path),
            runtime_deployment_pending=False, runtime_deployment_passed=True,
            deployment=str(deployment_path), deployment_sha256=sha(deployment_path),
            sealed_release_unchanged=True, recorded_ns=time.time_ns()))
        write_verified(N/'previous_current_resumed_grasp_session.json', pointer_before)
        require(read(control_path) == preflight['control_snapshot'] and all(
            sha(path) == digest for path, digest in input_sha.items()),
            'frozen input bytes changed during snapshot publication')
        write_verified(N/'deployment_finalization.json', dict(passed=True, recorded_ns=time.time_ns(),
            evidence_sha256={str(path):sha(path) for path in (deployment_path, status_path,
                N/'current_progress.json', N/'previous_current_resumed_grasp_session.json',
                N/'deployment_finalization_claim.json', Path(__file__))},
            authoritative_control_unchanged=True,
            automatic_grasp_started=False, motion_commands_sent=False, lifecycle_commands_sent=False,
            temperature_queried=False, physical_grasp_verified=False))
        # Publish the current pointer only after all acceptance evidence and
        # the terminal marker exist. A failed earlier write keeps GEN41 current.
        require(sha(pointer_path) == pointer_sha, 'current pointer changed before final publication')
        write_verified(pointer_path, dict(directory=str(N), runtime_session=str(N),
            deployment_snapshot=str(N/'current_progress.json'), deployment_status=str(status_path),
            deployment_finalization=str(N/'deployment_finalization.json'),
            control_path=str(control_path), control_state=control['state'], control_revision=control['revision'],
            control_fields_are_deployment_snapshot=True, updated_ns=time.time_ns()))
    print(json.dumps(dict(passed=True, deployment=str(deployment_path)), ensure_ascii=False))


if __name__ == '__main__':
    main()
