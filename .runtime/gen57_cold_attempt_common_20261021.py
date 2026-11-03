"""Strict bindings for one GEN57 cold-session attempt. No ROS work on import."""
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
S = N / 'gen57_resume_r3/full_restart_r1'
A = E / 'ros_log/full_latest_frozen_no_skin_20261012T021050Z/continuous_authorization_20261012/authorization.json'
AUTH_SHA = '132e83f926a249f8d1294ddeb74e8b38224744ea369a7acf8a6c29802f0c9172'
CONTROL = A.parent / 'control.json'
CONTROL_SHA = '8beaf2eeabac1d86f217959524083c09b927969f0be4f22646d6c306e6865754'
REQUEST = N / 'gen57_resume_r3/user_resume_request.json'
REQUEST_SHA = '89128addf34ef53109378f3d08f50d927131c206f7e43a62ad341d6aa6396023'
CAPTURE = W / '.runtime/capture_continuation_scene_20261012.py'
CAPTURE_SHA = '4ed0d920eb6283c0fd973e18cd35469577ff9fa2ef65b4c14211ea26fda37149'
CHECKER = W / '.runtime/verify_active_remote_readiness_20261020.py'
CHECKER_SHA = 'eaa51eaf5b636b74d9957798bbf3b0407a0d25d5c6b11097c02b22a5e97948e5'
VERIFIER = W / '.runtime/verify_gen57_full_cold_restart_20261020.py'
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
            path.parent.stat().st_dev == M.stat().st_dev, 'output outside current cold session')
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
    require(sha(helper) == args.helper_sha256 and sha(__file__) == args.common_sha256,
            'reviewed attempt helper/common changed')
    require(M.is_mount(), 'SanStand missing')
    release_path = args.release.resolve(strict=True)
    deployment_path = args.deployment.resolve(strict=True)
    require(release_path == N / 'verification/gen57_release_manifest.json' and
            deployment_path == S / 'acceptance/result.json', 'unexpected release/deployment path')
    pins = {release_path: args.release_sha256, deployment_path: args.deployment_sha256,
            Path(helper).resolve(): args.helper_sha256, Path(__file__).resolve(): args.common_sha256,
            A: AUTH_SHA, CONTROL: CONTROL_SHA, REQUEST: REQUEST_SHA,
            CAPTURE: CAPTURE_SHA, CHECKER: CHECKER_SHA}
    require(all(sha(p) == h for p, h in pins.items()), 'explicit input digest differs')
    release, deployment = read(release_path), read(deployment_path)
    require(release['generation'] == 'GEN57' and
            all(release.get(k) is True for k in ('passed', 'reviewed', 'installed')),
            'final GEN57 release is not accepted')
    require(release['source_count'] == len(release['source_sha256']) == 1088,
            'GEN57 source scope differs')
    summary = release['test_summary']
    require(summary['tests'] > 0 and all(summary[k] == 0 for k in ('failures', 'errors', 'skipped')),
            'full release regression is incomplete or failed')
    require(all(deployment.get(k) is True for k in
                ('deployment_passed', 'whole_patch_deployed', 'full_restart', 'all_processes_replaced')),
            'full cold deployment not accepted')
    require(deployment['deployment_kind'] == 'GEN57_full_cold_restart_with_current_remote_dependency' and
            deployment['dedicated_compute_reloader_compatible'] is False and
            deployment['preserved_after'] == {} and deployment['preserved_before'] == {},
            'cold deployment must not claim preserved old instances')
    require(Path(deployment['source_manifest']) == release_path and
            deployment['source_manifest_sha256'] == args.release_sha256 and
            deployment['source_sha256'] == release['source_sha256'] and
            deployment['test_summary'] == summary, 'deployment and release disagree')
    startup_path, config_path = S / 'startup.json', S / 'startup_configuration.json'
    startup, config = read(startup_path), read(config_path)
    require(startup['source_manifest_sha256'] == args.release_sha256 and startup['generation'] == 'GEN57' and
            startup['full_restart'] is True and startup['startup_configuration'] == str(config_path) and
            startup['startup_configuration_sha256'] == sha(config_path), 'cold construction binding differs')
    require(startup['control_sha256'] == CONTROL_SHA and
            startup['actual_user_instruction_sha256'] == REQUEST_SHA and
            startup['boot_id'] == Path('/proc/sys/kernel/random/boot_id').read_text().strip(),
            'cold startup control/request/boot differs')
    control, request = read(CONTROL), read(REQUEST)
    require(control['state'] == 'active' and type(control['revision']) is int and control['revision'] == 67 and
            control['authorization_id'] == read(A)['authorization_id'] and
            deployment['control_snapshot'] == startup['control'] == control and
            deployment['control_revision'] == 67 and deployment['control_unchanged'] is True,
            'actual active67 control differs')
    require(request['source'] == 'actual_user_message' and
            request['text'] == '继续处理，并且要重启启动各节点和上位机', 'actual resume request differs')
    nodes = deployment['all_registered_nodes']
    require(len(nodes) == 17 and nodes == deployment['final_all_registered_nodes'] and
            set(nodes) == set(config['persistent_nodes']) and len(deployment['after']) == 11 and
            all(nodes[n] == row for n, row in deployment['after'].items()), 'complete 17-instance binding differs')
    require(deployment['epoch_ns'] > 0 and deployment['manual_mode'] is True and
            deployment['task_active'] is False and deployment['actions_active'] is False and
            deployment['gui_owning_manual_ack']['callerid'] == GUI and
            deployment['gui_owning_manual_ack']['manual'] is True, 'cold idle/GUI reference invalid')
    pins.update({startup_path: sha(startup_path), config_path: sha(config_path)})
    pins.update({Path(p): h for p, h in startup['operational_source_sha256'].items()})
    pins.update({Path(p): h for p, h in release['evidence_sha256'].items()})
    verifier = checked_module(VERIFIER, pins[VERIFIER], 'gen57_attempt_cold_verifier')
    dependency = deployment['remote_dependency']
    binding_path = Path(dependency['startup_binding']).resolve(strict=True)
    require(binding_path == S / 'acceptance/remote_startup_binding.json' and
            sha(binding_path) == dependency['startup_binding_sha256'], 'cold remote binding differs')
    pins[binding_path] = dependency['startup_binding_sha256']
    checker = checked_module(CHECKER, CHECKER_SHA, 'gen57_attempt_remote_checker')
    binding = checker.load_binding(binding_path, dependency['startup_binding_sha256'])
    require(binding['parameters'] == config['remote_parameters'] and
            all(binding['node_identity'][k] == nodes['/remote_grasp6d_node'][k]
                for k in ('pid', 'parent_pid', 'start_ticks')), 'remote construction/current instance mismatch')
    context = dict(release=release, deployment=deployment, startup=startup, config=config,
                   nodes=nodes, pins=pins, verifier=verifier, checker=checker, binding=binding,
                   binding_path=binding_path, release_path=release_path, deployment_path=deployment_path)
    recheck(context)
    return context


def recheck(context):
    require(all(sha(p) == h for p, h in context['pins'].items()), 'pinned cold input changed')
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
    expected, deployment, v = context['nodes'], context['deployment'], context['verifier']
    core = v.properties(deployment['unit'])
    require(core == deployment['core_service'], 'cold core service identity changed')
    require(v.process(master.getPid()) == deployment['master_process'], 'cold master identity changed')
    deadline = time.monotonic() + graph_wait
    while True:
        system = master.getSystemState()
        registered = {name for group in system for _, names in group for name in names}
        actual = registered - {rospy.get_name()}
        if actual == set(expected):
            break
        if time.monotonic() >= deadline:
            raise RuntimeError('current graph/observer cleanup not ready: ' + repr(dict(
                missing=sorted(set(expected)-actual), unexpected=sorted(actual-set(expected)))))
        time.sleep(.05)
    rows = {}
    for name, row in expected.items():
        response = xmlrpc.client.ServerProxy(master.lookupNode(name), transport=Transport()).getPid(rospy.get_name())
        require(response[0] == 1 and response[2] == row['pid'], 'ROS node owner changed: ' + name)
        rows[name] = v.process(response[2])
        require(rows[name] == row, 'node instance changed: ' + name)
    configured = v.configured_nodes(rows, context['config'], core)
    publishers = dict(system[0])
    ownership = {}
    for topic, owner in PUBLISHERS.items():
        names = publishers.get(topic, [])
        require((owner in names if topic == '/gui/joint_direct_mode' else names == [owner]),
                'publisher ownership differs: ' + topic + ' ' + repr(names))
        ownership[topic] = list(names)
    windows = v.gui_windows(rows[GUI]['pid'])
    return dict(nodes=rows, configured_nodes=configured, publishers=ownership,
                core_service=core, master_process=deployment['master_process'], gui_windows=windows)


def input_receipts(context):
    return {str(path): digest for path, digest in context['pins'].items()}
