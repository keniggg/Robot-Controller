"""Strict bindings for one GEN59 remote-reload-session attempt. No ROS work on import."""
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
S = O / 'actual_attempt_r1'
RUNTIME = W / '.runtime/gen59_remote_reload_common_20261022.py'
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
            path.parent.stat().st_dev == M.stat().st_dev, 'output outside current remote attempt session')
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


def load_context(args, helper):
    """Bind GEN59's one replaced process and 16 preserved current instances."""
    from types import SimpleNamespace
    require(sha(helper) == args.helper_sha256 and sha(__file__) == args.common_sha256,
            'reviewed attempt helper/common changed')
    release_path, deployment_path = args.release.resolve(strict=True), args.deployment.resolve(strict=True)
    require(release_path == N/'verification/gen59_release_manifest.json' and
            deployment_path == O/'runtime_reload_r1/acceptance/result.json', 'unexpected GEN59 release/deployment')
    pins = {release_path:args.release_sha256, deployment_path:args.deployment_sha256,
            Path(helper).resolve():args.helper_sha256, Path(__file__).resolve():args.common_sha256,
            A:AUTH_SHA, CONTROL:CONTROL_SHA, REQUEST:REQUEST_SHA,
            CONTINUE_REQUEST:CONTINUE_REQUEST_SHA, CAPTURE:CAPTURE_SHA, CHECKER:CHECKER_SHA}
    require(all(sha(p) == h for p,h in pins.items()), 'attempt evidence digest differs')
    release, deployment = read(release_path), read(deployment_path)
    require(release['generation'] == 'GEN59' and all(release[k] is True for k in
            ('passed','reviewed','installed','full_regression_passed')) and
            release['source_count'] == len(release['source_sha256']) == 1094, 'GEN59 release incomplete')
    summary = release['test_summary']
    require(summary == dict(tests=9993,failures=0,errors=0,skipped=0), 'full GEN59 regression differs')
    require(all(deployment[k] is True for k in ('deployment_passed','whole_patch_deployed',
            'remote_reloaded','other_sixteen_processes_preserved','epoch_preserved')) and
            deployment['full_restart'] is False and deployment['all_processes_replaced'] is False and
            deployment['deployment_kind'] == 'GEN59_remote_only_sixteen_processes_preserved',
            'GEN59 remote replacement not accepted')
    require(deployment['source_manifest'] == str(release_path) and
            deployment['source_manifest_sha256'] == args.release_sha256 and
            deployment['source_sha256'] == release['source_sha256'] and
            deployment['test_summary'] == summary, 'deployment source differs')
    pins.update({Path(p):h for p,h in deployment['input_sha256'].items()})
    pins.update({Path(p):h for p,h in release['evidence_sha256'].items()})
    require(RUNTIME in pins and all(sha(p) == h for p,h in pins.items()), 'runtime helper/evidence changed')
    runtime = checked_module(RUNTIME,pins[RUNTIME],'gen59_attempt_runtime_checks')
    runtime_args = SimpleNamespace(previous_deployment=runtime.PREVIOUS,
        previous_deployment_sha256=runtime.PREVIOUS_SHA,
        helper_sha256=args.helper_sha256,common_sha256=pins[RUNTIME])
    runtime_context = runtime.context(runtime_args,helper)
    runtime.source_release(runtime_context,release_path,args.release_sha256)
    nodes = deployment['full_runtime_nodes']; before = runtime_context['nodes']
    preserved = runtime_context['preserved']; remote = runtime.REMOTE
    require(len(nodes) == 17 and nodes == deployment['all_registered_nodes'] ==
            deployment['final_all_registered_nodes'] and set(nodes) == set(before), '17 process inventory differs')
    require(deployment['before'] == before and deployment['preserved_before'] ==
            deployment['preserved_after'] == preserved and all(nodes[n] == row for n,row in preserved.items()) and
            deployment['replaced_before'] == {remote:before[remote]} and
            deployment['replaced_after'] == {remote:nodes[remote]} and
            nodes[remote] != before[remote], 'replacement/preservation partition differs')
    require(deployment['epoch_ns'] == runtime_context['previous']['epoch_ns'] and
            deployment['manual_mode'] is True and deployment['task_active'] is False and
            deployment['actions_active'] is False and deployment['actuation_status'].startswith('CONFIRMED:') and
            deployment['gui_owning_manual_ack'] == dict(callerid=GUI,manual=True), 'accepted manual feedback differs')
    control = read(CONTROL)
    require(control['state'] == 'active' and control['revision'] == 67 and
            deployment['control_snapshot'] == control and deployment['control_revision'] == 67 and
            deployment['control_unchanged'] is True, 'continuing authorization changed')
    construction_path = Path(deployment['startup_configuration'])
    require(sha(construction_path) == deployment['startup_configuration_sha256'], 'GEN59 construction changed')
    construction = read(construction_path)
    require(construction['generation'] == 'GEN59' and construction['release_sha256'] == args.release_sha256 and
            construction['source_sha256'] == release['source_sha256'], 'GEN59 launch source differs')
    attestation_path = Path(deployment['runtime_import_attestation'])
    require(sha(attestation_path) == deployment['runtime_import_attestation_sha256'], 'runtime source attestation changed')
    runtime.verify_attestation(runtime_context,read(attestation_path),nodes[remote],construction_path,release)
    dependency = deployment['remote_dependency']; binding_path = Path(dependency['startup_binding'])
    require(binding_path == O/'runtime_reload_r1/remote_startup_binding.json' and
            sha(binding_path) == dependency['startup_binding_sha256'], 'new remote startup binding differs')
    checker = runtime_context['checker']; binding = checker.load_binding(binding_path,dependency['startup_binding_sha256'])
    require(binding['parameters'] == construction['parameters'] == runtime_context['config']['remote_parameters'] and
            all(binding['node_identity'][k] == nodes[remote][k] for k in ('pid','parent_pid','start_ticks')),
            'new remote instance/configuration differs')
    pins.update({Path(p):h for p,h in runtime_context['pins'].items()})
    pins.update({p:sha(p) for p in (construction_path,attestation_path,binding_path)})
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


def live_identity(context, rospy, master, graph_wait=3.):
    dep, runtime = context['deployment'], context['runtime']
    require(context['verifier'].properties(dep['remote_unit']) == dep['remote_service'],
            'accepted GEN59 remote unit changed')
    graph = runtime.graph(context['runtime_context'],rospy,master,
        context['nodes'][runtime.REMOTE],dep['remote_unit'],dep['auxiliary_nodes'],graph_wait=graph_wait)
    require(graph['node_identities'] == context['nodes'], 'GEN59 instance graph changed')
    return dict(nodes=graph['node_identities'],configured_nodes=graph['configured_nodes'],
        publishers=graph['publishers'],core_service=graph['core_service'],
        master_process=graph['master_process'],gui_windows=graph['gui_windows'],
        auxiliary_nodes=graph['auxiliary_nodes'],remote_service=dep['remote_service'])


def input_receipts(context):
    return {str(path): digest for path, digest in context['pins'].items()}
