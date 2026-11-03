"""Strict bindings for one accepted GEN62 three-compute-owner attempt. No ROS work on import."""
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
E61 = E60 / 'seed_wait_fix_r1'
E62 = E61 / 'contact_feedback_fix_r1'
S = E62 / 'actual_attempt_r1'
RUNTIME = W / '.runtime/gen62_compute_runtime_r2_20261022.py'
A = E / 'ros_log/full_latest_frozen_no_skin_20261012T021050Z/continuous_authorization_20261012/authorization.json'
AUTH_SHA = '132e83f926a249f8d1294ddeb74e8b38224744ea369a7acf8a6c29802f0c9172'
CONTROL = A.parent / 'control.json'
CONTROL_SHA = '8fe6865ef453dad1808faf32f01da1b5bac0493755a4e934fdb63bc76114d995'
REQUEST = E61 / 'user_resume_r1/user_resume_request.json'
REQUEST_SHA = '87ed176de63a7612d41d9b1a314421a2033b867ad76f8a05cc84a7c1421b3f15'
CONTINUE_REQUEST = REQUEST
CONTINUE_REQUEST_SHA = REQUEST_SHA
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


def validate_acceptance_values(release, deployment, bundle, transition):
    """Bind three replaced computation owners and fourteen retained instances."""
    require(release['generation'] == 'GEN62' and all(release[k] is True for k in
        ('passed','reviewed','installed','full_regression_passed')), 'GEN62 release incomplete')
    require(bundle['deployment_ready'] is True and
        release['source_count'] == len(release['source_sha256']) == len(bundle['source_sha256']) and
        release['source_sha256'] == bundle['source_sha256'] == deployment['source_sha256'], 'source closure differs')
    summary = release['test_summary']
    require(set(summary) == {'tests','failures','errors','skipped'} and
        type(summary['tests']) is int and summary['tests'] == 10213 and
        all(type(summary[k]) is int and summary[k] == 0 for k in ('failures','errors','skipped')),
        'complete frozen regression required')
    require(deployment['kind'] == 'GEN62_three_compute_acceptance' and deployment['generation'] == 'GEN62'
        and deployment['deployment_passed'] is True and deployment['full_restart'] is False
        and deployment['replaced_count'] == 3 and deployment['preserved_count'] == 14,
        'three-owner deployment not accepted')
    names = {'/unknown_tabletop_perception','/remote_grasp6d_node','/grasp_task_node'}
    nodes, original = deployment['full_runtime_nodes'], bundle['nodes']
    require(len(nodes) == len(original) == 17 and set(nodes) == set(original)
        and set(deployment['targets']) == names
        and deployment['targets'] == transition['targets'] == {n:nodes[n] for n in names}
        and all(nodes[n] != original[n] for n in names), 'replacement inventory differs')
    preserved = {n:row for n,row in original.items() if n not in names}
    require(deployment['preserved'] == bundle['preserved'] == preserved
        and all(nodes[n] == row for n,row in preserved.items())
        and deployment['before']['node_identities'] == deployment['after']['node_identities'] == nodes,
        'retained fourteen or acceptance graph differs')
    require(transition['passed'] is True and transition['kind'] == 'GEN62_three_compute_reload'
        and set(transition['units']) == names
        and deployment['after']['services'] == transition['after']['services']
        and deployment['auxiliary_nodes'] == deployment['after']['auxiliary_nodes'] == transition['after']['auxiliary_nodes'],
        'compute unit or gateway auxiliary ownership differs')
    require(deployment['epoch_ns'] == bundle['epoch_ns'] and
        same_selection(deployment['selection'],bundle['selection'])
        and deployment['manual_mode'] is True and deployment['task_active'] is False
        and deployment['actions_active'] is False and deployment['actuation_status'].startswith('CONFIRMED:')
        and deployment['gui_owning_manual_ack'] is True, 'accepted manual feedback differs')
    require(deployment['automatic_attempt_authorized'] is False
        and deployment['fresh_scene_and_dependency_gate_required_before_automatic_attempt'] is True,
        'acceptance must not grant scene or future attempt authority')
    proof = deployment['source_process_proof']
    require(set(proof) == names and all(proof[n]['identity'] == nodes[n]
        and proof[n]['source_cache_process_proof'] is True and proof[n]['memory_attestation'] is False for n in names),
        'source/process provenance differs or is overstated')
    return nodes


def load_context(args, helper):
    """Use accepted GEN62 compute units plus retained GEN60 gateway/hardware."""
    require(sha(helper) == args.helper_sha256 and sha(__file__) == args.common_sha256,
            'reviewed attempt helper/common changed')
    release_path, deployment_path = args.release.resolve(strict=True), args.deployment.resolve(strict=True)
    require(release_path == N/'verification/gen62_release_manifest.json' and
        deployment_path == E62/'runtime_reload_r2/acceptance/result.json', 'unexpected GEN62 release/acceptance')
    pins = {release_path:args.release_sha256,deployment_path:args.deployment_sha256,
        Path(helper).resolve():args.helper_sha256,Path(__file__).resolve():args.common_sha256,
        A:AUTH_SHA,CONTROL:CONTROL_SHA,REQUEST:REQUEST_SHA,CONTINUE_REQUEST:CONTINUE_REQUEST_SHA,
        CAPTURE:CAPTURE_SHA,CHECKER:CHECKER_SHA}
    require(all(sha(p) == h for p,h in pins.items()), 'attempt evidence differs')
    release,deployment = read(release_path),read(deployment_path)
    require(release['acceptance'] == ref(deployment_path), 'release acceptance binding differs')
    pins.update({Path(p):h for p,h in release['evidence_sha256'].items()})
    require(RUNTIME in pins and all(sha(p) == h for p,h in pins.items()), 'runtime helper/evidence differs')
    bundle_path, transition_path = Path(deployment['bundle']),Path(deployment['reload'])
    require(sha(bundle_path) == deployment['bundle_sha256'] and
        sha(transition_path) == deployment['reload_sha256'], 'deployment bundle/transition differs')
    bundle,transition = read(bundle_path),read(transition_path)
    nodes = validate_acceptance_values(release,deployment,bundle,transition)
    require(transition['bundle'] == str(bundle_path) and transition['bundle_sha256'] == deployment['bundle_sha256'],
        'transition bundle differs')
    from types import SimpleNamespace
    runtime = checked_module(RUNTIME,pins[RUNTIME],'gen62_attempt_runtime_checks')
    runtime_base,runtime_context,checked_bundle = runtime.context(SimpleNamespace(
        bundle=bundle_path,bundle_sha256=deployment['bundle_sha256']))
    require(checked_bundle == bundle and runtime_context['nodes'] == bundle['nodes'],
        'retained prior instance binding differs')
    control = read(CONTROL)
    require(control['state'] == 'active' and control['revision'] == 69
        and runtime_base.recheck(runtime_context) == control, 'continuing authorization changed')
    construction_path = Path(transition['construction'])
    require(sha(construction_path) == transition['construction_sha256'], 'construction changed')
    construction = read(construction_path)
    require(construction['source_sha256'] == release['source_sha256']
        and construction['bundle'] == str(bundle_path) and construction['bundle_sha256'] == deployment['bundle_sha256'],
        'construction source differs')
    binding_path = Path(transition['remote_binding']['path'])
    require(sha(binding_path) == transition['remote_binding']['sha256'], 'new remote binding changed')
    checker = runtime_context['checker']
    binding = checker.load_binding(binding_path,transition['remote_binding']['sha256'])
    require(binding['parameters'] == construction['remote_parameters'] == runtime_context['config']['remote_parameters']
        and all(binding['node_identity'][k] == nodes['/remote_grasp6d_node'][k] for k in ('pid','parent_pid','start_ticks')),
        'new remote instance/configuration differs')
    for name,item in deployment['source_process_proof'].items():
        receipt_path = Path(item['receipt']);require(sha(receipt_path) == item['sha256'], 'source receipt changed')
        receipt = read(receipt_path)
        require(receipt['node'] == name and receipt['source_sha256'] == release['source_sha256']
            and receipt['construction_sha256'] == transition['construction_sha256'], 'source receipt binding differs')
        pins[receipt_path] = item['sha256']
    pins.update({Path(p):h for p,h in runtime_context['pins'].items()})
    for path in (bundle_path,transition_path,construction_path,binding_path):pins[path] = sha(path)
    context = dict(release=release,deployment=deployment,nodes=nodes,pins=pins,
        startup=runtime_context['startup'],config=runtime_context['config'],verifier=runtime_context['v'],
        checker=checker,binding=binding,binding_path=binding_path,runtime=runtime,runtime_base=runtime_base,
        runtime_context=runtime_context,bundle=bundle,transition=transition,
        release_path=release_path,deployment_path=deployment_path)
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
    dep,runtime = context['deployment'],context['runtime']
    transition = context['transition']
    graph = runtime.graph(context['runtime_base'],context['runtime_context'],rospy,master,
        dep['targets'],transition['units'],dep['auxiliary_nodes'],graph_wait=graph_wait)
    require(graph['node_identities'] == context['nodes'] and graph['services'] == dep['after']['services'],
        'GEN62 compute or retained instance ownership changed')
    require_action_status_publishers(graph['publishers'],graph['node_identities'])
    return dict(nodes=graph['node_identities'],configured_nodes=graph['configured_nodes'],
        publishers=graph['publishers'],core_service=graph['core_service'],
        master_process=context['runtime_context']['previous']['master_process'],gui_windows=graph['gui_windows'],
        auxiliary_nodes=graph['auxiliary_nodes'],remote_service=graph['services']['/remote_grasp6d_node'])


def input_receipts(context):
    return {str(path): digest for path, digest in context['pins'].items()}
