"""Finalize reviewed GEN49 idle reload metadata; never change control or move.

All ROS operations are reads, completed before the short nonblocking shared
control lock. The current deployment pointer is published last. GEN48 history
and the sealed release are retained without modification.
"""
import argparse
import fcntl
import math
import os
import time
import xmlrpc.client

from continuous_grasp_authorization_20261012 import atomic_json
from alicia_grasp_modes.selection import parse_selection
from gen49_idle_reload_common_20261018 import *


RUNTIME = N/'gen49_runtime'
RELEASE = N/'verification/gen49_release_manifest.json'
POINTER = W/'.runtime/current_resumed_grasp_session.json'
HELPER_SHA = {'/home/zhuyupei/alicia_wa_full/.worktrees/protocol-v3-upgrade/.runtime/gen49_idle_reload_common_20261018.py': '6c5881aea77e6a11f0316012b2062936e21c858031e9d0a47df8e08fd2d4f9f5', '/home/zhuyupei/alicia_wa_full/.worktrees/protocol-v3-upgrade/.runtime/read_gen49_deployment_preflight_20261018.py': 'bc87b2d321b0d7e3569708b7d80abb4d7b2b74865e80750a0bfa0dbf920289e0', '/home/zhuyupei/alicia_wa_full/.worktrees/protocol-v3-upgrade/.runtime/load_gen49_idle_compute_gateway_20261018.py': '12e373fb58275c704153a56ea62802c10561ce467b3784933a97b0bb3177295e', '/home/zhuyupei/alicia_wa_full/.worktrees/protocol-v3-upgrade/.runtime/gen49_observation_measurement_consumers_20261018.launch': '8cb7b5093bfcf31dbcc05a1d1fbe6ad246aaa5748831e34c70b81b7146ff7329', '/home/zhuyupei/alicia_wa_full/.worktrees/protocol-v3-upgrade/.runtime/start_gen49_observation_measurement_consumers_20261018.sh': '7633c569e485e8730f1a3ba964f174001f84ddf0c3796aa26613f4bd03a77c5e'}


def write_verified(path, value):
    atomic_json(path, value)
    require(read(path) == value, 'metadata read-back differs: '+str(path))


def identities_valid(rows, names):
    require(isinstance(rows, dict) and set(rows) == set(names), 'process identity set differs')
    for name, row in rows.items():
        require(set(row) == {'pid', 'parent_pid', 'start_ticks'}, 'process identity fields differ: '+name)
        require(type(row['pid']) is int and row['pid'] > 1 and
            type(row['parent_pid']) is int and row['parent_pid'] > 1 and
            isinstance(row['start_ticks'], str) and row['start_ticks'].isdigit() and
            int(row['start_ticks']) > 0, 'invalid process identity: '+name)


def joint_map(names, positions):
    require(isinstance(names, list) and isinstance(positions, list) and names and
        len(names) == len(positions) == len(set(names)), 'joint layout differs')
    require(all(isinstance(n, str) and n for n in names) and all(
        type(q) in (int, float) and math.isfinite(q) for q in positions), 'invalid joint feedback')
    return dict(zip(names, positions))


def deployment_chain(previous, preflight, compute):
    require(not set(previous['after']) & set(previous['preserved_after']), 'old process maps overlap')
    baseline = dict(previous['after'], **previous['preserved_after'])
    identities_valid(baseline, TARGETS+PRESERVE)
    require(preflight['node_identities'] == baseline, 'preflight differs from accepted GEN48')
    identities_valid(compute['before'], TARGETS)
    identities_valid(compute['after'], TARGETS)
    identities_valid(compute['preserved_before'], PRESERVE)
    identities_valid(compute['preserved_after'], PRESERVE)
    require(dict(compute['before'], **compute['preserved_before']) == baseline,
        'reload before does not equal current preflight')
    preserve = {name: baseline[name] for name in PRESERVE}
    require(compute['preserved_before'] == compute['preserved_after'] == preserve,
        'one of seven retained processes changed')
    after = compute['after']
    require(all((after[n]['pid'], after[n]['start_ticks']) !=
        (baseline[n]['pid'], baseline[n]['start_ticks']) for n in TARGETS),
        'one of four consumers was not replaced')
    require(compute['replaced_count'] == len(after) == 4 and
        compute['preserved_count'] == len(preserve) == 7, 'four-plus-seven scope differs')
    parent = preflight['gateway_original_parent']
    require(compute['gateway_respawn_parent'] == parent and
        compute['gateway_after_respawn'] == after['/motion_gateway'] and
        after['/motion_gateway']['parent_pid'] == parent['pid'], 'gateway parent binding differs')
    require(compute['gateway_started_by_compute_launch'] is False and
        compute['gateway_original_parent_preserved'] is True, 'duplicate gateway launch policy differs')
    require(len({after[n]['parent_pid'] for n in COMPUTE}) == 1,
        'new three compute consumers have different launch parents')
    require(compute['gateway_service_owners'] == {name: ['/motion_gateway'] for name in GATEWAY_SERVICES},
        'gateway service ownership differs')
    return baseline, after, preserve, dict(after, **preserve), parent


def validate_ack(compute, selection):
    for service in ('/grasp_mode/reset_remote', '/grasp_mode/reset_task'):
        row = compute['reset_ack'][service]
        ack = json.loads(row['message'])
        require(row['success'] is True and row['ready'] is True and ack.get('ready') is True and
            row['raw_selection'] == ack and row['selection'] == parse_selection(ack) and
            parse_selection(ack) == parse_selection(selection), 'reset ACK differs: '+service)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ('release', 'compute-deployment', 'previous', 'session', 'preflight'):
        parser.add_argument('--'+name, required=True, type=Path)
    args = parser.parse_args()
    session = args.session.resolve(strict=True)
    storage(session)
    require(session == RUNTIME.resolve(strict=True), 'new GEN49 runtime session required')
    paths = [p.resolve(strict=True) for p in (args.release, args.compute_deployment,
        args.previous, args.preflight)]
    release_path, compute_path, previous_path, preflight_path = paths
    require(release_path == RELEASE.resolve(strict=True), 'unexpected GEN49 sealed release')
    require(session in compute_path.parents, 'reload receipt must stay in GEN49 session')
    names = ('gen49_runtime_deployment.json', 'current_progress.json',
        'runtime_deployment_status.json', 'previous_current_resumed_grasp_session.json',
        'deployment_finalization.json', 'deployment_finalization_claim.json')
    require(not any((session/name).exists() for name in names), 'finalization already attempted')
    pinned = {str(p): p.read_bytes() for p in paths}
    release, compute, previous, preflight = [json.loads(pinned[str(p)]) for p in paths]
    validate_release(release_path)
    checked_previous, original = validate_previous(previous_path)
    require(checked_previous == previous and release['source_count'] ==
        len(release['source_sha256']) and release['source_count'] > 1053, 'release/previous differs')
    require(preflight['passed'] is True and preflight['manual_mode'] is True and
        preflight['task_active'] is False and preflight['actions_active'] is False and
        preflight['sample_count'] >= 20, 'preflight is not accepted manual idle')
    require(Path(preflight['source_manifest']).resolve(strict=True) == release_path and
        preflight['source_manifest_sha256'] == sha(release_path) and
        Path(preflight['previous_deployment']).resolve(strict=True) == previous_path and
        preflight['previous_deployment_sha256'] == PREVIOUS_SHA, 'preflight input binding differs')
    require(compute['deployment_passed'] is True and compute['whole_patch_deployed'] is True and
        compute['manual_mode'] is True and compute['task_active'] is False and
        compute['actions_active'] is False, 'reload was not accepted manual idle')
    require(compute['source_sha256'] == release['source_sha256'] and
        compute['source_manifest_sha256'] == sha(release_path) and
        Path(compute['source_manifest']).resolve(strict=True) == release_path and
        Path(compute['deployment_preflight']).resolve(strict=True) == preflight_path and
        compute['deployment_preflight_sha256'] == sha(preflight_path) and
        compute['previous_deployment_sha256'] == PREVIOUS_SHA and
        Path(compute['previous_deployment']).resolve(strict=True) == previous_path,
        'reload source/preflight/previous binding differs')
    require(preflight['helper_sha256'] == compute['helper_sha256'] == HELPER_SHA and
        all(sha(p) == h for p, h in HELPER_SHA.items()), 'operational helper source differs')
    baseline, after, preserve, expected, gateway_parent = deployment_chain(previous, preflight, compute)
    require(baseline == original, 'prior eleven-process identities differ')
    validate_ack(compute, preflight['selection'])
    control_path = Path(preflight['control_path']).resolve(strict=True)
    control = validate_control(control_path)
    require(control == preflight['control_snapshot'] == compute['control_snapshot'] and
        preflight['control_sha256'] == compute['control_sha256'] == sha(control_path),
        'frozen control snapshot differs')
    for p in (control_path, *HELPERS): pinned[str(p)] = p.read_bytes()
    input_sha = {p: hashlib.sha256(raw).hexdigest() for p, raw in pinned.items()}
    require(preflight['input_sha256'] == {p: input_sha[p] for p in
        map(str, (release_path, previous_path, control_path)+HELPERS)}, 'preflight input bytes differ')
    require(compute['input_sha256'] == {p: input_sha[p] for p in
        map(str, (release_path, preflight_path, control_path, previous_path)+HELPERS)},
        'reload input bytes differ')
    pointer_bytes = POINTER.read_bytes()
    pointer_before = json.loads(pointer_bytes)
    pointer_sha = hashlib.sha256(pointer_bytes).hexdigest()
    require(Path(pointer_before['directory']).resolve(strict=True) == PREVIOUS.parent.resolve(strict=True),
        'current pointer is not accepted GEN48 runtime')
    finalizer_sha = sha(__file__)
    reference = joint_map(preflight['names'], preflight['positions'])

    def frozen_inputs():
        require(all(Path(p).read_bytes() == raw for p, raw in pinned.items()), 'pinned input/helper changed')
        require(read(control_path) == control and sha(__file__) == finalizer_sha, 'control/finalizer changed')
        validate_sources(release)
        require(sha(POINTER) == pointer_sha, 'current deployment pointer changed')

    frozen_inputs()
    os.environ['ROS_LOG_DIR'] = str(session/'deployment_observer_ros')
    import rosgraph
    import rospy
    from actionlib_msgs.msg import GoalStatusArray
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Header, String
    from alicia_flexible_grasp_supervisor.msg import GraspState
    rospy.init_node('gen49_idle_finalization_readonly', anonymous=True, disable_rosout=True)
    master = rosgraph.Master(rospy.get_name())

    def identities():
        for name, row in expected.items():
            endpoint = xmlrpc.client.ServerProxy(master.lookupNode(name), transport=Transport())
            code, _, pid = endpoint.getPid(rospy.get_name())
            require(code == 1 and process(pid) == row, 'live process differs: '+name)
        require(process(gateway_parent['pid']) == gateway_parent, 'original gateway parent changed')
        providers = {name: list(nodes) for name, nodes in master.getSystemState()[2]}
        require(all(providers.get(name) == ['/motion_gateway'] for name in GATEWAY_SERVICES),
            'current gateway service owner differs')
        require(providers.get('/grasp_mode/reset_remote') == ['/remote_grasp6d_node'] and
            providers.get('/grasp_mode/reset_task') == ['/grasp_task_node'], 'reset service owner differs')

    def idle():
        require(rospy.get_param('/gui/joint_direct_mode') is True, 'manual ownership changed')
        require(not rospy.wait_for_message('/grasp/state', GraspState, timeout=5.).active, 'task active')
        mode = json.loads(rospy.wait_for_message('/grasp_mode/status', String, timeout=5.).data)
        require(not any(mode.get(k) for k in ('active', 'start_reserved', 'pending')), 'router busy')
        for topic in ('/alicia_controller/follow_joint_trajectory/status', '/hand_controller/follow_joint_trajectory/status'):
            msg = rospy.wait_for_message(topic, GoalStatusArray, timeout=5.)
            require(not any(row.status in (0, 1, 6, 7) for row in msg.status_list), 'action active')
        require(rospy.get_param('/grasp_mode/selection') == preflight['selection'], 'selection changed')
        require(rospy.wait_for_message('/alicia_d/control_reference_epoch', Header, timeout=5.).stamp.to_nsec()
            == preflight['epoch_ns'], 'control epoch changed')
        require({p: rospy.get_param(p) for p in PARAMETERS} == preflight['runtime_parameters'],
            'runtime policy/parameters changed')

    identities(); idle(); frozen_inputs()
    sdk = feedback(rospy)
    require(sdk['epoch_ns'] == preflight['epoch_ns'] and sdk['stamp_ns'] >= sdk['epoch_ns'],
        'SDK epoch differs')
    actual = joint_map(sdk['names'], sdk['positions'])
    require(set(actual) == set(reference) and all(abs(actual[n]-q) <= .004
        for n, q in reference.items()), 'posture changed since preflight')
    identities(); idle(); frozen_inputs()
    # Complete expensive source hashing before the final fresh sample; never
    # hold the shared control lock while collecting SDK data or waiting on ROS.
    sdk = feedback(rospy)
    require(sdk['epoch_ns'] == preflight['epoch_ns'] and sdk['stamp_ns'] >= sdk['epoch_ns'], 'final SDK epoch differs')
    actual = joint_map(sdk['names'], sdk['positions'])
    require(set(actual) == set(reference) and all(abs(actual[n]-q) <= .004
        for n, q in reference.items()), 'final posture differs from preflight')
    collection = sdk
    identities(); idle()
    final_joint = rospy.wait_for_message('/alicia_d/accepted_joint_states', JointState, timeout=5.)
    require(final_joint.header.frame_id == 'sdk_measured' and
        final_joint.header.stamp.to_nsec() >= preflight['epoch_ns'] and
        0 <= (rospy.Time.now()-final_joint.header.stamp).to_sec() < .2, 'final measured sample is stale')
    actual = joint_map(list(final_joint.name), list(final_joint.position))
    require(set(actual) == set(reference) and all(abs(actual[n]-q) <= .004
        for n, q in reference.items()), 'final measured posture changed')
    sdk = dict(names=list(final_joint.name), positions=list(final_joint.position),
        frame_id=final_joint.header.frame_id, stamp_ns=final_joint.header.stamp.to_nsec(),
        epoch_ns=preflight['epoch_ns'], sample_count=1)
    require(sha(POINTER) == pointer_sha and read(control_path) == control, 'pointer/control changed before publication')
    deployment_path = session/'gen49_runtime_deployment.json'
    status_path = session/'runtime_deployment_status.json'
    deployment = dict(deployment_passed=True, whole_patch_deployed=True,
        deployment_kind='idle_four_geometry_consumers_gateway_original_parent_seven_preserved',
        source_manifest=str(release_path), source_manifest_sha256=sha(release_path),
        source_sha256=release['source_sha256'], test_summary=release['test_summary'],
        before={n: baseline[n] for n in TARGETS}, after=after,
        preserved_before=preserve, preserved_after=preserve, replaced_count=4, preserved_count=7,
        unit=compute['unit'], previous_deployment=str(previous_path), previous_deployment_sha256=PREVIOUS_SHA,
        preflight=str(preflight_path), compute_deployment=str(compute_path),
        evidence_sha256=input_sha, finalizer_sha256=finalizer_sha,
        gateway_original_parent=gateway_parent, gateway_reloaded=True, duplicate_gateway_started=False,
        gateway_service_owners=compute['gateway_service_owners'], reset_ack=compute['reset_ack'],
        manual_mode=True, task_active=False, actions_active=False, selection=preflight['selection'],
        epoch_ns=preflight['epoch_ns'], feedback=sdk, feedback_collection=collection,
        runtime_parameters=preflight['runtime_parameters'],
        control_path=str(control_path), control_state=control['state'], control_revision=control['revision'],
        control_snapshot=control, control_unchanged=True, recorded_ns=time.time_ns(),
        lifecycle_commands_sent=False, motion_commands_sent=False, temperature_queried=False,
        torque_disable_commands_sent=False, physical_grasp_verified=False, scene_permission=False,
        new_selection_required_before_automatic_grasp=True)
    progress = dict(runtime_session=str(session), directory=str(session),
        status='gen49_deployed_manual_idle_pending_fresh_grasp_scene', running_release='GEN49',
        snapshot_semantics='immutable_deployment_time_snapshot; authoritative control file remains unchanged',
        deployment_passed=True, whole_patch_deployed=True, gen49_deployed=True,
        source_manifest=str(release_path), source_manifest_sha256=sha(release_path),
        latest_deployment=str(deployment_path), deployment_status=str(status_path),
        current_test_result=release['actual_regression'], source_count=release['source_count'],
        test_scope_files=release['test_scope_files'], test_summary=release['test_summary'],
        deployed_node_identities=expected, selection=preflight['selection'], epoch_ns=preflight['epoch_ns'],
        control_path=str(control_path), control_state=control['state'], control_revision=control['revision'],
        control_snapshot=control, manual_mode=True, task_active=False, actions_active=False,
        automatic_grasp_started=False, new_motion_authorized_by_this_finalization=False,
        pending_work='Fresh target/robot/opening/path acceptance and existing control authorization are still required for a later grasp.',
        physical_grasp_verified=False, updated_ns=time.time_ns())
    # No ROS, sleeps or full-source hashing occur inside this nonblocking lock.
    with (control_path.parent/'control.lock').open('rb') as lock:
        fcntl.flock(lock, fcntl.LOCK_SH | fcntl.LOCK_NB)
        require(read(control_path) == control and sha(control_path) == input_sha[str(control_path)] and
            sha(POINTER) == pointer_sha, 'control/pointer changed before snapshot publication')
        with (session/'deployment_finalization_claim.json').open('x') as claim:
            json.dump(dict(claimed_ns=time.time_ns(), finalizer_sha256=finalizer_sha,
                pointer_before_sha256=pointer_sha, input_sha256=input_sha), claim, indent=2)
            claim.flush(); os.fsync(claim.fileno())
        backup = session/'previous_current_resumed_grasp_session.json'
        with backup.open('xb') as stream:
            stream.write(pointer_bytes); stream.flush(); os.fsync(stream.fileno())
        require(sha(backup) == pointer_sha, 'old current pointer backup differs')
        write_verified(deployment_path, deployment)
        write_verified(session/'current_progress.json', progress)
        write_verified(status_path, dict(source_manifest=str(release_path), source_manifest_sha256=sha(release_path),
            runtime_deployment_pending=False, runtime_deployment_passed=True,
            deployment=str(deployment_path), deployment_sha256=sha(deployment_path),
            sealed_release_unchanged=True, recorded_ns=time.time_ns()))
        require(all(Path(p).read_bytes() == raw for p, raw in pinned.items()) and
            sha(__file__) == finalizer_sha and sha(POINTER) == pointer_sha,
            'frozen input/helper/pointer changed during snapshot publication')
        write_verified(session/'deployment_finalization.json', dict(passed=True, recorded_ns=time.time_ns(),
            evidence_sha256={str(p): sha(p) for p in (deployment_path, status_path,
                session/'current_progress.json', backup, session/'deployment_finalization_claim.json', Path(__file__))},
            authoritative_control_unchanged=True, automatic_grasp_started=False,
            lifecycle_commands_sent=False, motion_commands_sent=False, temperature_queried=False,
            physical_grasp_verified=False))
        require(sha(POINTER) == pointer_sha and read(control_path) == control,
            'control/current pointer changed before final publication')
        write_verified(POINTER, dict(directory=str(session), runtime_session=str(session),
            deployment_snapshot=str(session/'current_progress.json'), deployment_status=str(status_path),
            deployment_finalization=str(session/'deployment_finalization.json'), control_path=str(control_path),
            control_state=control['state'], control_revision=control['revision'],
            control_fields_are_deployment_snapshot=True, updated_ns=time.time_ns()))
    print(json.dumps(dict(passed=True, deployment=str(deployment_path)), ensure_ascii=False))


if __name__ == '__main__':
    main()
