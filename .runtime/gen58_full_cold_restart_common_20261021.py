"""Read-only GEN58 cold-start binding helpers. No ROS work on import.

Configuration comes from roslaunch's real XML/YAML expansion. It is frozen
before dispatch and later bound to the newly created remote process. No health
response alone is a proof of prediction, scene clearance or motion permission.
"""
import base64
import hashlib
import importlib.util
import json
import math
import os
from pathlib import Path
import re
import sys
import time

sys.dont_write_bytecode = True
W = Path(__file__).resolve().parents[1]
M = Path('/media/zhuyupei/SanStand')
E = M/'实验记录/Robot-Controller'
N = E/'ros_log/full_latest_gen41_restart_20261018T011222Z'
CONTROL = E/'ros_log/full_latest_frozen_no_skin_20261012T021050Z/continuous_authorization_20261012/control.json'
CONTROL_SHA = '8beaf2eeabac1d86f217959524083c09b927969f0be4f22646d6c306e6865754'
REQUEST = N/'gen57_resume_r3/user_resume_request.json'
REQUEST_SHA = '89128addf34ef53109378f3d08f50d927131c206f7e43a62ad341d6aa6396023'
CONTINUE_REQUEST = N/'gen58_resume_r1/user_continue_request.json'
CONTINUE_REQUEST_SHA = '55dd4a17729108de8964bb72692d7ee0a326c14715cf5b29e354cc659d533281'
SESSION = N/'gen58_resume_r1/full_restart_r1'
RELEASE = N/'verification/gen58_release_manifest.json'
BASE_RELEASE = N/'verification/gen57_release_manifest.json'
BASE_RELEASE_SHA = '257e407b60d7153368f6122c531844e0d16dea2756b201fc9a0279f4c1f8e192'
PREVIOUS_DEPLOYMENT = N/'gen57_resume_r3/full_restart_r1/acceptance/result.json'
PREVIOUS_DEPLOYMENT_SHA = 'cb5a5fca99da4556dda93cfe037d83e9197e58c549cc7b9841351f23ba48dc25'
LAUNCH = W/'.runtime/full_latest_gen36_single_observation_20261016.launch'
ENTRY = W/'.runtime/start_full_latest_gen36_20261016.sh'
CHECKER = W/'.runtime/verify_active_remote_readiness_20261020.py'
CHECKER_SHA = 'eaa51eaf5b636b74d9957798bbf3b0407a0d25d5c6b11097c02b22a5e97948e5'
CLIENT = W/'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/vision/remote_grasp6d_client.py'
OLD_BINDING = N/'gen56_runtime/remote_startup_binding_r1/startup_binding.json'
OLD_BINDING_SHA = 'cd9f737f65a7772bf3e81dadabd19ad745c6716341ec9c2b74deb0819dda5ae8'
REMOTE_KEYS = ('/grasp_6d/remote/server_url', '/grasp_6d/remote/require_candidate_depth',
               '/grasp_6d/remote/timeout_sec')
CORE_NODES = ('/grasp_task_node', '/remote_grasp6d_node', '/unknown_tabletop_perception',
    '/alicia_d_driver_node', '/bessica_d_hw_interface', '/move_group', '/motion_gateway',
    '/supervisor_camera', '/compliant_gripper_controller', '/grasp_mode_router',
    '/alicia_supervisor_gui')


def require(value, reason):
    if not value:
        raise RuntimeError(reason)


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def read(path):
    return json.loads(Path(path).read_bytes())


def new_json(path, value):
    path = Path(path)
    require(M.is_mount() and E/'ros_log' in path.resolve().parents and
        path.parent.stat().st_dev == M.stat().st_dev and
        not os.statvfs(M).f_flag & os.ST_RDONLY, 'writable SanStand ros_log required')
    raw = (json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False)+'\n').encode()
    with path.open('xb') as stream:
        stream.write(raw); stream.flush(); os.fsync(stream.fileno())
    require(path.read_bytes() == raw, 'evidence read-back differs: '+str(path))


def control_and_request():
    require(sha(CONTROL) == CONTROL_SHA and sha(REQUEST) == REQUEST_SHA
        and sha(CONTINUE_REQUEST) == CONTINUE_REQUEST_SHA,
        'actual resume authorization changed')
    control, request = read(CONTROL), read(REQUEST)
    require(control.get('state') == 'active' and type(control.get('revision')) is int and
        control['revision'] == 67 and request.get('source') == 'actual_user_message' and
        request.get('text') == '继续处理，并且要重启启动各节点和上位机',
        'GEN58 full restart does not match actual request')
    continuation = read(CONTINUE_REQUEST)
    require(continuation.get('source') == 'actual_user_message'
        and continuation.get('text') == '继续处理'
        and continuation.get('existing_control_path') == str(CONTROL)
        and continuation.get('existing_control_sha256') == CONTROL_SHA
        and continuation.get('existing_control') == control
        and continuation.get('new_scene_confirmation_claimed') is False,
        'GEN58 continuation differs or claims new scene clearance')
    return control, request


def release_scope(path, release):
    """Bind this revision to the fully tested, reviewed three-file source delta."""
    require(Path(path).resolve(strict=True) == RELEASE and sha(BASE_RELEASE) == BASE_RELEASE_SHA,
        'GEN58 release/baseline location or identity differs')
    baseline = read(BASE_RELEASE)['source_sha256']
    source = release['source_sha256']
    changed = {name for name in baseline.keys() & source.keys() if baseline[name] != source[name]}
    require(release.get('generation') == 'GEN58' and len(baseline) == 1088
        and release.get('source_count') == len(source) == 1089
        and set(baseline) <= set(source)
        and changed == {
            'src/alicia_grasp_modes/src/alicia_grasp_modes/rgb_instance.py',
            'src/alicia_grasp_modes/tests/test_observation_arrival_probable_foreground.py'}
        and set(source)-set(baseline) == {'src/alicia_grasp_modes/tests/test_missing_depth_rgb_prior.py'}
        and release.get('test_summary') == dict(tests=9893,failures=0,errors=0,skipped=0),
        'GEN58 tested source delta/count/scope differs')
    require(source['src/alicia_grasp_modes/src/alicia_grasp_modes/rgb_instance.py'] ==
        '3d400dd2e17f3b1ac6a2078e85c9a4b9a855d4c7402b0bc4aca291312b467b0a',
        'reviewed missing-depth production bytes differ')


def stopped_reference(path, expected_sha256):
    """A stopped-stack receipt is a new-instance baseline, not motion authority."""
    path = Path(path).resolve(strict=True)
    require(N/'gen58_resume_r1' in path.parents and sha(path) == expected_sha256,
        'GEN58 transition receipt location or digest differs')
    value = read(path)
    before = value['before']
    require(value.get('passed') is True and value.get('stop_requested') is True
        and value.get('stop_performed') is True and value['cold_after']['passed'] is True
        and value['cold_after']['after']['boot_id'] == Path('/proc/sys/kernel/random/boot_id').read_text().strip(),
        'transition did not confirm the owned stack stopped on this boot')
    require(value['release'] == dict(path=str(RELEASE),sha256=sha(RELEASE))
        and before['source_sha256'] == read(RELEASE)['source_sha256']
        and before['control_sha256'] == CONTROL_SHA and before['control_snapshot'] == read(CONTROL)
        and before['sample_count'] >= 20 and before['manual_mode'] is True
        and before['gui_owning_manual_ack'] is True and before['task_active'] is False
        and before['actions_active'] is False
        and all(sha(p) == h for p,h in value['input_sha256'].items()),
        'transition source/control/idle receipt differs')
    require(type(before['epoch_ns']) is int and before['epoch_ns'] > 0
        and isinstance(before['node_identities'],dict) and len(before['node_identities']) == 17
        and set(CORE_NODES) <= set(before['node_identities']),
        'stop receipt lacks actual pre-stop epoch and complete 17-node identities')
    for row in before['node_identities'].values():
        require(type(row['pid']) is int and row['pid'] > 1
            and type(row['parent_pid']) is int and row['parent_pid'] > 1
            and str(row['start_ticks']).isdigit(), 'invalid pre-stop process identity')
    return value


def checker_module():
    require(sha(CHECKER) == CHECKER_SHA, 'remote checker changed')
    spec = importlib.util.spec_from_file_location('gen57_cold_remote_checker', CHECKER)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def configuration():
    """Expand launch locally; never upload params or initialize a ROS node."""
    import roslaunch
    cfg = roslaunch.config.load_config_default([str(LAUNCH)], 11311,
        verbose=False, assign_machines=False)
    nodes = {}
    for node in cfg.nodes + cfg.nodes_core:
        name = node.namespace.rstrip('/')+'/'+node.name
        require(name not in nodes, 'duplicate launch node: '+name)
        require(node.machine_name is None, 'unexpected nonlocal launch node: '+name)
        nodes[name] = dict(package=node.package, executable=node.type,
            args=node.args, respawn=node.respawn, required=node.required,
            source_file=node.filename, launch_prefix=node.launch_prefix)
    require(set(CORE_NODES) <= set(nodes), 'full launch omits a principal node')
    loader = nodes.get('/controller_loader_once', {})
    require(loader.get('package') == 'alicia_d_driver' and
        loader.get('executable') == 'controller_loader_once.py' and
        loader.get('args', '').split() == ['alicia_controller', 'hand_controller'] and
        loader.get('respawn') is False, 'unexpected one-shot controller loader')
    persistent = sorted(set(nodes)-{'/controller_loader_once'})
    all_params = {k:v.value for k,v in cfg.params.items()}
    remote = {k:all_params[k] for k in REMOTE_KEYS}
    check = checker_module(); check.validate_parameters(remote)
    require(all_params['/gui/joint_direct_mode'] is True and
        all_params['/alicia_d_driver_node/software_temperature_monitoring_enabled'] is False and
        all_params['/alicia_d_driver_node/temperature_poll_rate_hz'] == 0.0 and
        all_params['/alicia_d_driver_node/self_check_poll_rate_hz'] == 0.0 and
        all_params['/alicia_d_driver_node/auto_torque_on_startup'] is True,
        'manual/positive-enable/temperature startup policy changed')
    environment_url = re.findall(r'^export GRASP6D_URL=(\S+)$', ENTRY.read_text(), re.M)
    require(environment_url == [remote[REMOTE_KEYS[0]]],
        'entry environment URL differs from actual expanded launch URL')
    require(sha(OLD_BINDING) == OLD_BINDING_SHA, 'reviewed backend policy changed')
    backend_policy = read(OLD_BINDING)
    contract = dict(path=str(CLIENT), sha256=sha(CLIENT))
    check.client_contract(contract)
    files = {str(Path(p).resolve()):sha(p) for p in sorted(set(cfg.roslaunch_files))}
    files[str(ENTRY)] = sha(ENTRY)
    return dict(schema=1, launch=str(LAUNCH), launch_sha256=sha(LAUNCH),
        nodes=nodes, persistent_nodes=persistent, one_shot_nodes=['/controller_loader_once'],
        remote_parameters=remote, client_contract_source=contract,
        allowed_backends=backend_policy['allowed_backends'],
        backend_policy_reference=dict(path=str(OLD_BINDING),sha256=OLD_BINDING_SHA),
        launch_source_sha256=files,
        all_resolved_parameters_sha256=hashlib.sha256(json.dumps(all_params,
            sort_keys=True,ensure_ascii=False,allow_nan=False).encode()).hexdigest(),
        full_configuration_from_roslaunch=True, parameters_uploaded=False)


def standalone_health(config, output):
    """Prestart HTTP diagnostic, explicitly without current-node readiness."""
    output = Path(output); output.mkdir(exist_ok=False)
    check = checker_module()
    binding = dict(parameters=config['remote_parameters'], allowed_backends=config['allowed_backends'],
        _client_contract=check.client_contract(config['client_contract_source']))
    proxy = {k:os.environ[k] for k in check.PROXY_KEYS if k in os.environ}
    url = check.valid_url(binding['parameters'][check.URL])+'/health'
    result = dict(passed=False, current_node_dependency_ready=False,
        scope='prestart configured URL reachability/protocol only; no live node exists yet',
        started_ns=time.time_ns(), method='GET', prediction_requests=0,
        motion_commands_sent=False, temperature_queried=False)
    try:
        probe = check.fetch(url, proxy); result['probe'] = probe
        encoded = probe.pop('body_base64', None)
        raw = base64.b64decode(encoded, validate=True) if encoded is not None else None
        if raw is not None:
            p = output/'health_response.body'
            with p.open('xb') as stream: stream.write(raw)
            require(p.read_bytes() == raw, 'health body read-back differs')
            probe.update(body_path=str(p),body_sha256=sha(p),body_bytes=len(raw))
        require('error_type' not in probe and probe.get('http_status') == 200 and
            probe.get('response_url') == url and probe.get('body_complete') is True and raw is not None,
            'REMOTE_DEPENDENCY_NOT_READY: '+str(probe))
        result['capabilities'] = check.validate_health(check.strict_json(raw), binding)
        result['passed'] = True
    except Exception as exc:
        result.update(error=repr(exc), failure_code=getattr(exc,'code','REMOTE_DEPENDENCY_NOT_READY'))
    result['finished_ns'] = time.time_ns()
    new_json(output/'result.json', result)
    return result


def verify_feedback_samples(samples, epoch, now_ns):
    require(len(samples) >= 20 and len({s.header.stamp.to_nsec() for s in samples}) >= 20,
        'twenty unique SDK samples required')
    joint = samples[-1]
    names = list(joint.name)
    require(len(names) == len(set(names)) and
        {'Joint%d' % i for i in range(1,7)} <= set(names), 'SDK joint layout invalid')
    require(all(s.header.frame_id == 'sdk_measured' and list(s.name) == names and
        len(s.position) == len(names) and all(math.isfinite(v) for v in s.position) and
        epoch <= s.header.stamp.to_nsec() <= now_ns for s in samples),
        'SDK sample source/layout/epoch differs')
    require(0 <= now_ns-joint.header.stamp.to_nsec() < 200_000_000, 'latest SDK sample stale')
    stamps = [s.header.stamp.to_nsec() for s in samples]
    require(all(b > a for a,b in zip(stamps,stamps[1:])), 'SDK measurement stamps not increasing')
    spans = {n:max(s.position[i] for s in samples)-min(s.position[i] for s in samples)
        for i,n in enumerate(names)}
    require(all(v <= .004 for v in spans.values()), 'SDK sample window is not stationary')
    return dict(sample_count=len(samples), unique_sample_count=len(set(stamps)),
        position_spans=spans, stable_span_limit=.004, first_stamp_ns=stamps[0],last_stamp_ns=stamps[-1])


def bind_and_probe_remote(session, output, startup, config, identity, selection, release_path):
    check = checker_module()
    proc = Path('/proc')/str(identity['pid'])
    command = (proc/'cmdline').read_bytes()
    exact = dict(identity,cmdline_sha256=hashlib.sha256(command).hexdigest())
    evidence = {str(session/'startup.json'):sha(session/'startup.json'),
        str(session/'startup_configuration.json'):sha(session/'startup_configuration.json'),
        str(release_path):sha(release_path), str(REQUEST):REQUEST_SHA,
        str(CONTINUE_REQUEST):CONTINUE_REQUEST_SHA,
        startup['stop_receipt']:startup['stop_receipt_sha256'],
        str(Path(__file__).resolve()):sha(__file__), **startup['operational_source_sha256']}
    release = read(release_path)
    for name in ('src/alicia_flexible_grasp_supervisor/scripts/remote_grasp6d_node.py',
                 'src/alicia_grasp_modes/scripts/mode_aware_remote_grasp6d.py'):
        require(sha(W/name) == release['source_sha256'][name], 'new remote source differs from release')
        evidence[str(W/name)] = sha(W/name)
    binding = dict(schema=1,node_name='/remote_grasp6d_node',master_uri='http://localhost:11311',
        parameters=config['remote_parameters'],allowed_backends=config['allowed_backends'],
        client_contract_source=config['client_contract_source'],node_identity=exact,
        evidence=[dict(path=p,sha256=h) for p,h in sorted(evidence.items())],
        construction_evidence_review_required=True,ready_health_not_implied_by_startup_binding=True,
        construction_provenance=dict(kind='fresh_full_cold_restart_expanded_launch',
            startup_configuration=str(session/'startup_configuration.json'),
            startup_configuration_sha256=sha(session/'startup_configuration.json'),
            core_unit=startup['core']['unit'], actual_launch=str(LAUNCH),
            live_parameters_do_not_define_expected_values=True), created_ns=time.time_ns())
    binding_path = output/'remote_startup_binding.json'
    new_json(binding_path,binding)
    validated = check.load_binding(binding_path,sha(binding_path))
    result = check.evaluate(validated,output)
    result.update(startup_binding=str(binding_path),startup_binding_sha256=sha(binding_path),
        helper_sha256=CHECKER_SHA,automatic_grasp_started=False)
    new_json(output/'remote_readiness.json',result)
    require(result['passed'] is True and result['dependency_ready'] is True,
        'REMOTE_DEPENDENCY_NOT_READY: '+str(result.get('error')))
    require(result['before']['selection'] == result['after']['selection'] == selection and
        result['before']['node_identity'] == result['after']['node_identity'] == exact,
        'remote instance/selection changed during full restart acceptance')
    require(check.load_binding(binding_path,sha(binding_path)) == validated,
        'remote construction evidence changed during probe')
    return dict(path=str(output/'remote_readiness.json'),sha256=sha(output/'remote_readiness.json'),
        startup_binding=str(binding_path),startup_binding_sha256=sha(binding_path),
        finished_ns=result['finished_ns'],capabilities=result['capabilities'])
