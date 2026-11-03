"""Strict bindings for one GEN60 gateway-reload-session attempt. No ROS work on import."""
import argparse
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import shutil
import sys
import time
import xmlrpc.client

W = Path(__file__).resolve().parents[1]
M = Path('/media/zhuyupei/SanStand')
E = M / '实验记录/Robot-Controller'
N = E / 'ros_log/full_latest_gen41_restart_20261018T011222Z'
O = N / 'gen58_resume_r1/observation_pose_fix'
E60 = O / 'held_lattice_fix_r1'
S = E60 / 'actual_attempt_r1'
RUNTIME = W / '.runtime/gen60_gateway_reload_common_20261022.py'
A = E / 'ros_log/full_latest_frozen_no_skin_20261012T021050Z/continuous_authorization_20261012/authorization.json'
AUTH_SHA = '132e83f926a249f8d1294ddeb74e8b38224744ea369a7acf8a6c29802f0c9172'
CONTROL = A.parent / 'control.json'
CONTROL_SHA = '8beaf2eeabac1d86f217959524083c09b927969f0be4f22646d6c306e6865754'
REQUEST = N / 'gen57_resume_r3/user_resume_request.json'
REQUEST_SHA = '89128addf34ef53109378f3d08f50d927131c206f7e43a62ad341d6aa6396023'
CONTINUE_REQUEST = N / 'gen58_resume_r1/user_continue_request.json'
CONTINUE_REQUEST_SHA = '55dd4a17729108de8964bb72692d7ee0a326c14715cf5b29e354cc659d533281'
CAPTURE = W / '.runtime/capture_continuation_scene_20261012.py'
CAPTURE_SHA = '4ed0d920eb6283c0fd973e18cd35469577ff9fa2ef65b4c14211ea26fda37149'
CHECKER = W / '.runtime/verify_active_remote_readiness_20261020.py'
CHECKER_SHA = 'eaa51eaf5b636b74d9957798bbf3b0407a0d25d5c6b11097c02b22a5e97948e5'
VERIFIER = W / '.runtime/verify_gen58_full_cold_restart_20261021.py'
GUI = '/alicia_supervisor_gui'
DRIVER = '/alicia_d_driver_node'
ACTION_OWNER = '/bessica_d_hw_interface'
ACTION_STATUS_TOPICS = (
    '/alicia_controller/follow_joint_trajectory/status',
    '/hand_controller/follow_joint_trajectory/status',
)
PUBLISHERS = {
    '/supervisor/camera/color/image_raw': '/supervisor_camera',
    '/supervisor/camera/depth/image_raw': '/supervisor_camera',
    '/alicia_d/accepted_joint_states': DRIVER,
    '/alicia_d/control_reference_epoch': DRIVER,
    '/alicia_d/feedback_ready': DRIVER,
    '/alicia_d/motion_enabled': DRIVER,
    '/alicia_d/actuation_status': DRIVER,
    '/gui/joint_direct_mode': GUI,
    '/grasp/state': '/grasp_task_node',
    '/grasp_mode/status': '/grasp_mode_router',
}


def require(value, reason):
    if not value:
        raise RuntimeError(reason)


def selection_identity(value):
    """Compare the complete selection across exact XML-RPC stamp representations.

    Preserve the raw captured object for the original runner. The production
    parser permits decimal-text nanoseconds because XML-RPC integers are int32;
    no missing-field defaults, ignored extra keys or numeric coercion are used.
    """
    from alicia_grasp_modes.selection import parse_selection
    require(type(value) is dict and set(value) == {'mode','strategy','generation','stamp_ns'},
            'selection must contain exactly mode/strategy/generation/stamp_ns')
    return parse_selection(value)


def same_selection(*values):
    require(len(values) >= 2, 'selection comparison requires at least two complete records')
    identities = [selection_identity(value) for value in values]
    return all(identity == identities[0] for identity in identities[1:])


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def read(path):
    return json.loads(Path(path).read_bytes())


def ref(path):
    return dict(path=str(Path(path).resolve(strict=True)), sha256=sha(path))


def storage(path):
    path = Path(path).resolve()
    require(M.is_mount() and not os.statvfs(M).f_flag & os.ST_RDONLY,
            'writable SanStand required; no system-disk fallback')
    require(S in path.parents and path.parent.exists() and
            path.parent.stat().st_dev == M.stat().st_dev, 'output outside current gateway attempt session')
    return path


def new_json(path, value):
    path = storage(path)
    raw = (json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False) + '\n').encode()
    with path.open('xb') as stream:
        stream.write(raw)
        stream.flush()
        os.fsync(stream.fileno())
    require(path.read_bytes() == raw, 'written evidence read-back differs')


def parser(description):
    result = argparse.ArgumentParser(description=description)
    for name in ('release', 'deployment'):
        result.add_argument('--' + name, type=Path, required=True)
        result.add_argument('--' + name + '-sha256', required=True)
    result.add_argument('--helper-sha256', required=True)
    result.add_argument('--common-sha256', required=True)
    return result


def checked_module(path, expected, name):
    require(sha(path) == expected, 'helper changed: ' + str(path))
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def validate_acceptance_values(release, deployment, previous, post, before):
    """Check the mixed-generation value contract without I/O or live calls.

    The runtime validator separately closes the source/case inventory against
    its pinned manifest and full regression. No previous source/test count is
    reused here as a substitute for that closure.
    """
    require(release['generation'] == 'GEN60' and all(release[k] is True for k in
            ('passed','reviewed','installed','full_regression_passed')) and
            release['source_count'] == len(release['source_sha256']) > 0, 'GEN60 release incomplete')
    summary = release['test_summary']
    require(set(summary) == {'tests','failures','errors','skipped'} and
            type(summary['tests']) is int and summary['tests'] > 0 and
            all(type(summary[k]) is int and summary[k] == 0 for k in ('failures','errors','skipped')),
            'full GEN60 regression differs')
    require(deployment['generation'] == 'GEN60' and
            all(deployment[k] is True for k in ('deployment_passed','whole_patch_deployed',
                'other_sixteen_processes_preserved','epoch_preserved','gateway_original_parent_preserved')) and
            deployment['full_restart'] is False and deployment['all_processes_replaced'] is False and
            deployment['deployment_kind'] == 'original_parent_gateway_respawn_sixteen_preserved',
            'GEN60 gateway replacement not accepted')
    require(deployment['source_sha256'] == release['source_sha256'] and
            deployment['test_summary'] == summary, 'deployment source differs')
    nodes = deployment['full_runtime_nodes']; gateway, remote = '/motion_gateway', '/remote_grasp6d_node'
    require(len(nodes) == 17 and nodes == deployment['all_registered_nodes'] ==
            deployment['final_all_registered_nodes'] and set(nodes) == set(before) and
            before == previous['full_runtime_nodes'] == post['full_runtime_nodes_before'] ==
            post['full_runtime_nodes_after'], '17 process inventory differs')
    preserved = {name:row for name,row in before.items() if name != gateway}
    require(deployment['before'] == before and deployment['preserved_before'] ==
            deployment['preserved_after'] == preserved and all(nodes[n] == row for n,row in preserved.items()) and
            deployment['replaced_before'] == {gateway:before[gateway]} and
            deployment['replaced_after'] == {gateway:nodes[gateway]} and
            nodes[gateway] != before[gateway] and
            nodes[gateway]['parent_pid'] == before[gateway]['parent_pid'] and
            deployment['new_gateway_identity'] == nodes[gateway] and
            deployment['replaced_count'] == 1 and deployment['preserved_count'] == 16,
            'replacement/preservation partition differs')
    require(nodes[remote] == before[remote] and deployment['remote_unit'] == previous['remote_unit'] and
            deployment['remote_service'] == previous['remote_service'], 'retained GEN59 remote instance changed')
    composite = deployment['composite_runtime']
    require(composite['gateway_generation'] == 'GEN60' and composite['remote_generation'] == 'GEN59' and
            composite['remote_reloaded'] is False and composite['remote_preload_attestation_retained'] is True and
            composite['changed_source_consumers'] == [gateway] and
            deployment['gateway_source_load_proof']['same_process_module_attestation'] is False,
            'mixed-generation source provenance overstated')
    require(deployment['automatic_attempt_authorized'] is False and
            deployment['fresh_scene_and_dependency_gate_required_before_automatic_attempt'] is True,
            'reload acceptance must not grant future attempt authority')
    require(deployment['epoch_ns'] == previous['epoch_ns'] == post['epoch_ns'] and
            same_selection(deployment['selection'],post['selection']) and
            deployment['manual_mode'] is True and deployment['task_active'] is False and
            deployment['actions_active'] is False and deployment['actuation_status'].startswith('CONFIRMED:') and
            deployment['gui_owning_manual_ack'] == dict(callerid=GUI,manual=True), 'accepted manual feedback differs')
    for name in ('startup_configuration','startup_configuration_sha256',
                 'runtime_import_attestation','runtime_import_attestation_sha256'):
        require(deployment[name] == previous[name], 'retained GEN59 construction/import reference changed: '+name)
    for name in ('startup_binding','startup_binding_sha256'):
        require(deployment['remote_dependency'][name] == previous['remote_dependency'][name],
                'retained GEN59 remote startup binding changed: '+name)
    return nodes


def load_context(args, helper):
    """Bind the new gateway and the sixteen retained owners, including GEN59 remote."""
    require(sha(helper) == args.helper_sha256 and sha(__file__) == args.common_sha256,
            'reviewed attempt helper/common changed')
    release_path, deployment_path = args.release.resolve(strict=True), args.deployment.resolve(strict=True)
    require(release_path == N/'verification/gen60_release_manifest.json' and
            deployment_path == E60/'runtime_reload_r1/acceptance/result.json', 'unexpected GEN60 release/deployment')
    pins = {release_path:args.release_sha256, deployment_path:args.deployment_sha256,
            Path(helper).resolve():args.helper_sha256, Path(__file__).resolve():args.common_sha256,
            A:AUTH_SHA, CONTROL:CONTROL_SHA, REQUEST:REQUEST_SHA,
            CONTINUE_REQUEST:CONTINUE_REQUEST_SHA, CAPTURE:CAPTURE_SHA, CHECKER:CHECKER_SHA}
    require(all(sha(p) == h for p,h in pins.items()), 'attempt evidence digest differs')
    release, deployment = read(release_path), read(deployment_path)
    require(deployment['source_manifest'] == str(release_path) and
            deployment['source_manifest_sha256'] == args.release_sha256, 'deployment release reference differs')
    pins.update({Path(p):h for p,h in deployment['input_sha256'].items()})
    pins.update({Path(p):h for p,h in release['evidence_sha256'].items()})
    require(RUNTIME in pins and all(sha(p) == h for p,h in pins.items()), 'runtime helper/evidence changed')
    runtime = checked_module(RUNTIME,pins[RUNTIME],'gen60_attempt_runtime_checks')
    runtime_context = runtime.accepted_context(deployment_path,args.deployment_sha256)
    # accepted_context already closes the exact installed release against its
    # full regression and rechecks the retained remote's real GEN59 imports.
    require(runtime_context['acceptance'] == deployment, 'runtime acceptance differs')
    previous = runtime_context['previous']; remote = runtime.REMOTE
    nodes = validate_acceptance_values(release,deployment,previous,runtime_context['post'],
                                      runtime_context['before_nodes'])
    require(runtime_context['nodes'] == nodes, 'accepted runtime map differs')
    control = read(CONTROL)
    require(control['state'] == 'active' and control['revision'] == 67 and
            deployment['control_snapshot'] == control and deployment['control_revision'] == 67 and
            deployment['control_unchanged'] is True, 'continuing authorization changed')
    construction_path = Path(deployment['startup_configuration'])
    remote_release_path = Path(previous['source_manifest'])
    require(remote_release_path == N/'verification/gen59_release_manifest.json' and
            sha(remote_release_path) == previous['source_manifest_sha256'], 'retained GEN59 release changed')
    remote_release = read(remote_release_path)
    require(sha(construction_path) == deployment['startup_configuration_sha256'], 'retained GEN59 construction changed')
    construction = read(construction_path)
    require(construction['generation'] == remote_release['generation'] == 'GEN59' and
            construction['release_sha256'] == previous['source_manifest_sha256'] and
            construction['source_sha256'] == remote_release['source_sha256'], 'retained GEN59 launch source differs')
    attestation_path = Path(deployment['runtime_import_attestation'])
    require(sha(attestation_path) == deployment['runtime_import_attestation_sha256'], 'runtime source attestation changed')
    dependency = deployment['remote_dependency']; binding_path = Path(dependency['startup_binding'])
    require(binding_path == O/'runtime_reload_r1/remote_startup_binding.json' and
            sha(binding_path) == dependency['startup_binding_sha256'], 'retained remote startup binding differs')
    checker = runtime_context['checker']; binding = checker.load_binding(binding_path,dependency['startup_binding_sha256'])
    require(binding['parameters'] == construction['parameters'] == runtime_context['config']['remote_parameters'] and
            all(binding['node_identity'][k] == nodes[remote][k] for k in ('pid','parent_pid','start_ticks')),
            'retained remote instance/configuration differs')
    pins.update({Path(p):h for p,h in runtime_context['pins'].items()})
    pins.update({p:sha(p) for p in (construction_path,attestation_path,binding_path,remote_release_path)})
    context = dict(release=release,deployment=deployment,nodes=nodes,pins=pins,
        startup=runtime_context['startup'],config=runtime_context['config'],
        verifier=runtime_context['v'],checker=checker,binding=binding,binding_path=binding_path,
        runtime=runtime,runtime_context=runtime_context,release_path=release_path,deployment_path=deployment_path)
    require(context['startup']['boot_id'] == Path('/proc/sys/kernel/random/boot_id').read_text().strip(), 'boot changed')
    recheck(context)
    return context


def recheck(context):
    require(all(sha(p) == h for p, h in context['pins'].items()), 'pinned runtime input changed')
    for name, digest in context['release']['source_sha256'].items():
        relative = Path(name)
        require(not relative.is_absolute() and '..' not in relative.parts and
                sha(W / relative) == digest, 'deployed release source changed: ' + name)
    config, policy = read(E / 'grasp_records/archive_config.json'), read(E / 'maintenance/storage_control.json')
    require(config['upload_enabled'] is False and config['delete_enabled'] is False and
            policy['archive_policy'] == 'local_only', 'storage policy changed')
    free = shutil.disk_usage(E).free
    require(free >= config['min_free_bytes'], 'SanStand recording space insufficient')
    return dict(free_bytes=free, min_free_bytes=config['min_free_bytes'])


class Transport(xmlrpc.client.Transport):
    def make_connection(self, host):
        connection = super().make_connection(host)
        connection.timeout = 3.
        return connection


def require_action_status_publishers(publishers, nodes):
    """Both idle heartbeats must have one publisher owned by the accepted hardware process."""
    require(ACTION_OWNER in nodes, 'accepted action-status hardware owner missing')
    for topic in ACTION_STATUS_TOPICS:
        require(publishers.get(topic) == [ACTION_OWNER], 'action-status publisher missing/foreign/duplicate: '+topic)


def live_identity(context, rospy, master, graph_wait=3.):
    dep, runtime = context['deployment'], context['runtime']
    require(context['verifier'].properties(dep['remote_unit']) == dep['remote_service'],
            'retained GEN59 remote unit changed')
    graph = runtime.graph(context['runtime_context'],rospy,master,
        gateway=context['nodes'][runtime.GATEWAY],auxiliaries=dep['auxiliary_nodes'],graph_wait=graph_wait)
    require(graph['node_identities'] == context['nodes'], 'GEN60 mixed-generation instance graph changed')
    require_action_status_publishers(graph['publishers'],graph['node_identities'])
    return dict(nodes=graph['node_identities'],configured_nodes=graph['configured_nodes'],
        publishers=graph['publishers'],core_service=graph['core_service'],
        master_process=graph['master_process'],gui_windows=graph['gui_windows'],
        auxiliary_nodes=graph['auxiliary_nodes'],remote_service=dep['remote_service'])


def input_receipts(context):
    return {str(path): digest for path, digest in context['pins'].items()}
