"""GEN59 remote-only transition primitives; importing this module does no ROS work.

The prior GEN34 bounded cleanup and GEN58 instance/feedback checks remain the
basis. A stopped-target receipt separates source installation from node launch.
No function selects automatic mode, moves the arm, or reads temperature values.
"""
import argparse
import ast
import hashlib
import importlib.util
import json
import math
import os
from pathlib import Path
import re
import subprocess
import sys
import threading
import time
import xmlrpc.client

sys.dont_write_bytecode = True
W = Path(__file__).resolve().parents[1]
M = Path('/media/zhuyupei/SanStand')
E = M/'实验记录/Robot-Controller'
N = E/'ros_log/full_latest_gen41_restart_20261018T011222Z'
S = N/'gen58_resume_r1/full_restart_r1'
O = N/'gen58_resume_r1/observation_pose_fix'
PREVIOUS = S/'acceptance/result.json'
PREVIOUS_SHA = '3cfda4096fb1590cafaba2f8df525b1579120c950f1a9fbdb9ecada67888eaf2'
BASE = N/'verification/gen58_release_manifest.json'
BASE_SHA = '03bf9e669329f804921982564f730572e7758ef81c9e9d624a24cfb3700b333e'
RELEASE = N/'verification/gen59_release_manifest.json'
REMOTE = '/remote_grasp6d_node'
ENTRY = 'src/alicia_grasp_modes/scripts/mode_aware_remote_grasp6d.py'
PRODUCTION = {ENTRY, 'src/alicia_grasp_modes/src/alicia_grasp_modes/observe_return_visual.py',
              'src/alicia_grasp_modes/src/alicia_grasp_modes/reachable_observation_views.py'}
LEGACY = W/'.runtime/reload_gen34_remote_20261016.py'
OLD_COMMON = W/'.runtime/gen58_cold_attempt_common_20261021.py'
VERIFIER = W/'.runtime/verify_gen58_full_cold_restart_20261021.py'
CHECKER = W/'.runtime/verify_active_remote_readiness_20261020.py'
STATIC_PINS = {
    str(LEGACY): 'c601f2f0d87d133f7a7744e25664cc13f4ee7bb6edc9c2a764b35f4a10da6065',
    str(OLD_COMMON): 'c051892ed0e88957bbe2d36127d9b934072909159bb3e54e051e7a8100599787',
    str(VERIFIER): '7d80620665bdd26b6feb3794a84db3bb0dc79c8ff75238a5c430952eadb9d06c',
    str(CHECKER): 'eaa51eaf5b636b74d9957798bbf3b0407a0d25d5c6b11097c02b22a5e97948e5',
    str(W/'.runtime/continuous_grasp_authorization_20261012.py'): '916f5b03585ce26402c50515d88592b27354650c85aab3c9ea8c2538e76f8454',
    str(PREVIOUS): PREVIOUS_SHA, str(BASE): BASE_SHA,
    str(S/'startup.json'): 'e8c1a6543fb5347882c922ae29559b3b9b786c8ee6f04fdf8343f21e876fd5ba',
    str(S/'startup_configuration.json'): 'da307074579d7fb8af1f45c6e12d3957f95329e57206e75cc4cdd616d210b176',
}


def require(ok, reason):
    if not ok:
        raise RuntimeError(reason)


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def read(path):
    return json.loads(Path(path).read_bytes())


def sha_value(value):
    return hashlib.sha256(json.dumps(value,sort_keys=True,separators=(',',':'),allow_nan=False).encode()).hexdigest()


def module(path, digest, name):
    require(sha(path) == digest, 'pinned helper changed: '+str(path))
    spec = importlib.util.spec_from_file_location(name, path)
    result = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(result)
    return result


def storage(path):
    path = Path(path).resolve()
    require(M.is_mount() and not os.statvfs(M).f_flag & os.ST_RDONLY, 'writable SanStand required')
    require(O in path.parents, 'output outside GEN59 observation repair')
    ancestor = path.parent
    while not ancestor.exists():
        ancestor = ancestor.parent
    require(ancestor.stat().st_dev == M.stat().st_dev, 'output is not on SanStand')
    cfg = read(E/'grasp_records/archive_config.json')
    require(cfg['upload_enabled'] is False and cfg['delete_enabled'] is False
            and read(E/'maintenance/storage_control.json')['archive_policy'] == 'local_only', 'storage policy changed')
    stat = os.statvfs(E)
    require(stat.f_bavail*stat.f_frsize >= cfg['min_free_bytes'], 'external recording space insufficient')


def save(path, value):
    storage(path)
    raw = (json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False)+'\n').encode()
    with Path(path).open('xb') as stream:
        stream.write(raw); stream.flush(); os.fsync(stream.fileno())
    require(Path(path).read_bytes() == raw, 'receipt readback differs')


def parser(description):
    result = argparse.ArgumentParser(description=description)
    for key in ('previous-deployment', 'output'):
        result.add_argument('--'+key, type=Path, required=True)
    for key in ('previous-deployment-sha256', 'helper-sha256', 'common-sha256'):
        result.add_argument('--'+key, required=True)
    return result


def context(args, helper):
    require(args.previous_deployment.resolve(strict=True) == PREVIOUS
            and args.previous_deployment_sha256 == PREVIOUS_SHA, 'unexpected previous acceptance')
    pins = dict(STATIC_PINS, **{str(Path(helper).resolve()): args.helper_sha256,
                              str(Path(__file__).resolve()): args.common_sha256})
    require(all(sha(p) == h for p,h in pins.items()), 'input hash changed')
    old = module(OLD_COMMON, STATIC_PINS[str(OLD_COMMON)], 'gen59_old_attempt')
    # Import only after all transitive operational helpers have been pinned.
    startup, previous = read(S/'startup.json'), read(PREVIOUS)
    require(startup['boot_id']==Path('/proc/sys/kernel/random/boot_id').read_text().strip(), 'boot changed since accepted stack')
    pins.update(startup['operational_source_sha256'])
    require(all(sha(p) == h for p,h in pins.items()), 'historical operational helper changed')
    v = module(VERIFIER, STATIC_PINS[str(VERIFIER)], 'gen59_old_verifier')
    legacy = module(LEGACY, STATIC_PINS[str(LEGACY)], 'gen59_old_remote_reload')
    checker = module(CHECKER, STATIC_PINS[str(CHECKER)], 'gen59_current_dependency')
    config = read(S/'startup_configuration.json')
    require(previous['deployment_passed'] and previous['whole_patch_deployed'] and previous['full_restart']
            and previous['source_manifest_sha256'] == BASE_SHA and previous['source_sha256'] == read(BASE)['source_sha256'],
            'previous deployed GEN58 release differs')
    nodes = previous['all_registered_nodes']
    require(len(nodes) == 17 and nodes == previous['final_all_registered_nodes']
            and set(nodes) == set(config['persistent_nodes']), 'previous complete node inventory differs')
    declared = config['nodes'][REMOTE]
    require(declared['respawn'] is False and declared['required'] is False and declared['args'] == ''
            and declared['launch_prefix'] is None and declared['executable'] == Path(ENTRY).name,
            'remote-only replacement incompatible with original launch')
    for path,digest in ((old.A,old.AUTH_SHA),(old.CONTROL,old.CONTROL_SHA),(old.REQUEST,old.REQUEST_SHA),
                        (old.CONTINUE_REQUEST,old.CONTINUE_REQUEST_SHA)):
        pins[str(path)] = digest
    ctx = dict(old=old,v=v,legacy=legacy,checker=checker,pins=pins,previous=previous,startup=startup,
               config=config,nodes=nodes,preserved={n:r for n,r in nodes.items() if n != REMOTE})
    recheck(ctx)
    return ctx


def recheck(ctx, sources=None):
    require(all(sha(p) == h for p,h in ctx['pins'].items()), 'pinned transition input changed')
    old = ctx['old']; ctl = read(old.CONTROL)
    require(ctl['state'] == 'active' and type(ctl['revision']) is int and ctl['revision'] == 67
            and ctl['authorization_id'] == read(old.A)['authorization_id'], 'active67 authorization changed')
    continuation = read(old.CONTINUE_REQUEST)
    require(continuation['source'] == 'actual_user_message' and continuation['text'] == '继续处理'
            and continuation['existing_control'] == ctl and continuation['new_scene_confirmation_claimed'] is False,
            'continuation binding changed')
    if sources is not None:
        ctx['v'].source_check(sources)
    return ctl


def identity(ctx, master, caller, name):
    reply = xmlrpc.client.ServerProxy(master.lookupNode(name), transport=ctx['old'].Transport()).getPid(caller)
    require(reply[0] == 1, 'node PID unavailable: '+name)
    return ctx['v'].process(reply[2])


def graph(ctx, rospy, master, remote=None, remote_unit=None, auxiliaries=None, graph_wait=3.):
    """All 16 preserved owners; any optional lazy alias must be the same gateway."""
    require(type(graph_wait) in (int,float) and math.isfinite(graph_wait) and 0.<=graph_wait<=3.,
            'graph poll wait must be finite and within original three-second limit')
    v, dep = ctx['v'], ctx['previous']; core = v.properties(dep['unit'])
    require(core == dep['core_service'] and v.process(master.getPid()) == dep['master_process'], 'core/master changed')
    sim = v.properties(dep['simulation_unit']); oldsim = dep['simulation_service']
    require(int(sim['MainPID']) == oldsim['pid'] and sim['InvocationID'] == oldsim['invocation_id']
            and sim['ControlGroup'] == oldsim['control_group'], 'simulation unit changed')
    expected = dict(ctx['preserved'])
    if remote is not None:
        expected[REMOTE] = remote
    deadline = time.monotonic()+graph_wait
    while True:
        system = master.getSystemState()
        names = {n for group in system for _,owners in group for n in owners}-{rospy.get_name()}
        extra = names-set(expected)
        valid_extra = len(extra) <= 1 and all(re.fullmatch('/move_group_commander_wrappers_[0-9]+',n) for n in extra)
        if set(expected) <= names and valid_extra:
            break
        require(time.monotonic() < deadline, 'graph missing/unexpected owners: '+repr(sorted(names ^ set(expected))))
        time.sleep(.05)
    aliases = {n:identity(ctx,master,rospy.get_name(),n) for n in sorted(extra)}
    require(all(r == expected['/motion_gateway'] for r in aliases.values()), 'lazy alias not owned by accepted gateway')
    if auxiliaries is not None:
        require(aliases == auxiliaries, 'gateway alias set/identity changed')
    rows = {n:identity(ctx,master,rospy.get_name(),n) for n in sorted(expected)}
    require(rows == expected, 'node process changed')
    # Preserve the original configured_nodes executable, parent, cgroup and
    # duplicate-process tests, with only remote's dedicated unit substituted.
    preserved_config = dict(ctx['config'], persistent_nodes=sorted(ctx['preserved']))
    configured = v.configured_nodes({n:rows[n] for n in ctx['preserved']}, preserved_config, core)
    if remote is not None and remote_unit is None:
        require(remote == ctx['nodes'][REMOTE] and remote['parent_pid'] == int(core['MainPID'])
                and v.in_group(remote['pid'],core['ControlGroup']), 'old remote ownership differs')
    if remote is not None and remote_unit is not None:
        unit = v.properties(remote_unit)
        require(int(unit['MainPID']) == remote['pid'] and v.in_group(remote['pid'],unit['ControlGroup']), 'new remote unit ownership differs')
    remote_processes = ctx['legacy'].remote_processes()
    require(remote_processes == ([] if remote is None else [remote]), 'duplicate/unregistered remote process')
    pub = dict(system[0]); ownership = {}
    for topic,owner in ctx['old'].PUBLISHERS.items():
        actual = pub.get(topic,[])
        require((owner in actual if topic == '/gui/joint_direct_mode' else actual == [owner]), 'publisher ownership changed: '+topic)
        ownership[topic] = actual
    return dict(node_identities=rows,auxiliary_nodes=aliases,core_service=core,master_process=dep['master_process'],
                simulation_service=sim,configured_nodes=configured,publishers=ownership,
                gui_windows=v.gui_windows(rows['/alicia_supervisor_gui']['pid']))


def sample_idle(ctx, rospy, master, expected_selection=None, expected_epoch=None):
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Header, String, Bool
    from actionlib_msgs.msg import GoalStatusArray
    from alicia_flexible_grasp_supervisor.msg import GraspState
    from alicia_grasp_modes.selection import parse_selection
    from controller_manager_msgs.srv import ListControllers
    lock, values, history = threading.Lock(), {}, []
    topics = [('joint','/alicia_d/accepted_joint_states',JointState),('epoch','/alicia_d/control_reference_epoch',Header),
        ('task','/grasp/state',GraspState),('mode','/grasp_mode/status',String),('gui','/gui/joint_direct_mode',Bool),
        ('actuation','/alicia_d/actuation_status',String),('motion','/alicia_d/motion_enabled',Bool),
        ('feedback','/alicia_d/feedback_ready',Bool),('arm','/alicia_controller/follow_joint_trajectory/status',GoalStatusArray),
        ('hand','/hand_controller/follow_joint_trajectory/status',GoalStatusArray)]
    owners = {'joint':'/alicia_d_driver_node','epoch':'/alicia_d_driver_node','task':'/grasp_task_node',
              'mode':'/grasp_mode_router','gui':'/alicia_supervisor_gui','actuation':'/alicia_d_driver_node',
              'motion':'/alicia_d_driver_node','feedback':'/alicia_d_driver_node',
              'arm':'/bessica_d_hw_interface','hand':'/bessica_d_hw_interface'}
    def receive(key,msg):
        received=rospy.Time.now().to_nsec()
        with lock:
            values[key] = (msg,received)
            if key == 'joint':
                history.append(dict(stamp_ns=msg.header.stamp.to_nsec(),received_ns=received,
                    frame_id=msg.header.frame_id,names=list(msg.name),positions=list(msg.position),
                    callerid=msg._connection_header.get('callerid')))
                history[:]=history[-80:]
    subs = [rospy.Subscriber(t,k,lambda m,key=n:receive(key,m),queue_size=40) for n,t,k in topics]
    try:
        end = time.monotonic()+8.
        while time.monotonic() < end:
            with lock:
                ready = len(values) == len(topics) and len(history) >= 20
            if ready:
                break
            time.sleep(.025)
        services=dict(master.getSystemState()[2])
        require(services.get('/controller_manager/list_controllers')==['/bessica_d_hw_interface'], 'controller service ownership changed')
        rospy.wait_for_service('/controller_manager/list_controllers',timeout=5.)
        controllers={item.name:item.state for item in rospy.ServiceProxy('/controller_manager/list_controllers',ListControllers)().controller}
        require(all(controllers.get(name)=='running' for name in ('alicia_controller','hand_controller')), 'required controllers not running')
        for key,topic,kind in topics:
            if key not in ('joint','arm','hand'):
                receive(key,rospy.wait_for_message(topic,kind,timeout=3.))
        with lock:
            states = {k:v[0] for k,v in values.items()}; times = {k:v[1] for k,v in values.items()}
            samples=list(history[-20:])
        require(len(states) == len(topics) and len(samples) == 20, 'missing state or 20 unique SDK samples')
        now = rospy.Time.now().to_nsec(); q=states['joint']; epoch=states['epoch'].stamp.to_nsec()
        require(not rospy.is_shutdown() and all(0<=now-times[k]<1_000_000_000 for k in states), 'stale current state receipt')
        require(all(states[k]._connection_header.get('callerid')==owners[k] for k in states if k not in ('arm','hand')), 'state publisher owner changed')
        names=samples[-1]['names']; stamps=[s['stamp_ns'] for s in samples]
        require(len(set(names))==len(names) and {'Joint%d'%i for i in range(1,7)}<=set(names), 'invalid joint layout')
        require(len(set(stamps))==20 and all(b>a for a,b in zip(stamps,stamps[1:]))
                and 0<=now-stamps[-1]<200_000_000 and epoch>0, 'SDK arrivals not unique, ordered or fresh')
        require(all(s['frame_id']=='sdk_measured' and s['callerid']=='/alicia_d_driver_node' and s['names']==names
                    and len(s['positions'])==len(names) and all(math.isfinite(x) for x in s['positions'])
                    and epoch<=s['stamp_ns']<=s['received_ns'] and s['received_ns']-s['stamp_ns']<200_000_000
                    for s in samples), 'invalid SDK source, layout or per-message age')
        spans={n:max(s['positions'][i] for s in samples)-min(s['positions'][i] for s in samples) for i,n in enumerate(names)}
        require(all(x<=.004 for x in spans.values()), 'joint movement during idle sampling')
        reference=ctx.setdefault('idle_reference',dict(names=names,positions=samples[-1]['positions']))
        require(reference['names']==names and len(reference['positions'])==len(names), 'transition reference layout changed')
        maximum=max(abs(s['positions'][i]-reference['positions'][i]) for s in samples for i in range(len(names)))
        require(maximum<=.004, 'pose moved from transition reference')
        require(not states['task'].active and states['gui'].data is True and master.getParam('/gui/joint_direct_mode') is True, 'not GUI-owned manual idle')
        router=json.loads(states['mode'].data)
        require(not any(router.get(k) for k in ('active','start_reserved','pending')), 'router active')
        require(all(0<=now-states[k].header.stamp.to_nsec()<=2_000_000_000 and not any(g.status in (0,1,6,7) for g in states[k].status_list)
                    for k in ('arm','hand')), 'active or stale action status')
        require(states['feedback'].data is True and states['motion'].data is True
                and states['actuation'].data.startswith('CONFIRMED:')
                and master.getParam('/grasp/require_actuation_confirmation') is True, 'measured actuation/feedback not ready')
        require(master.getParam('/alicia_d_driver_node/software_temperature_monitoring_enabled') is False
                and master.getParam('/alicia_d_driver_node/temperature_poll_rate_hz') == 0.
                and master.getParam('/grasp/unknown_lift_after_close_enabled') is False
                and master.getParam('/grasp/unknown_observe_return_max_observations') == 1, 'temperature/lift/observation policy changed')
        selection=parse_selection(master.getParam('/grasp_mode/selection'))
        require(selection['generation']==2 and selection['mode']=='unknown' and selection['strategy']=='observe_return', 'selection differs')
        require(expected_selection is None or selection==expected_selection, 'selection changed during transition')
        require(epoch == (ctx['previous']['epoch_ns'] if expected_epoch is None else expected_epoch), 'control reference epoch changed; fresh reference required')
        ctl=recheck(ctx)
        return dict(recorded_ns=time.time_ns(),epoch_ns=epoch,selection=selection,sample_count=20,names=names,
            positions=samples[-1]['positions'],sample_position_spans=spans,sample_stable_limit=.004,
            maximum_joint_delta_rad=maximum,
            sample_stamps_ns=stamps,samples=samples,state_received_ns=times,
            latch_receipt_time_is_not_source_generation_time=True,manual_mode=True,gui_owning_manual_ack=True,
            task_active=False,task_state=states['task'].state,actions_active=False,router=router,
            controllers=controllers,
            actuation_status=states['actuation'].data,motion_enabled=True,feedback_ready=True,
            control_snapshot=ctl,control_sha256=sha(ctx['old'].CONTROL))
    finally:
        for sub in subs:
            sub.unregister()


def fingerprints(ctx, master):
    result={}
    for key in ('/grasp','/grasp_6d')+ctx['legacy'].CACHED_KEYS:
        try:
            value=master.getParam(key)
        except Exception as exc:
            import rosgraph
            if not isinstance(exc,rosgraph.MasterError):
                raise
            result[key]=dict(present=False); continue
        raw=json.dumps(value,sort_keys=True,separators=(',',':'),allow_nan=False).encode()
        comparison=json.loads(raw)
        if key=='/camera' and isinstance(comparison,dict) and isinstance(comparison.get('runtime_profile'),dict):
            comparison['runtime_profile'].pop('stationary_temporal',None)
        compared=json.dumps(comparison,sort_keys=True,separators=(',',':'),allow_nan=False).encode()
        result[key]=dict(present=True,value_sha256=hashlib.sha256(raw).hexdigest(),comparison_sha256=hashlib.sha256(compared).hexdigest())
    return result


def same_parameters(before, after):
    comparable=lambda rows:{k:(v['present'],v.get('comparison_sha256')) for k,v in rows.items()}
    require(comparable(before)==comparable(after), 'parameter configuration changed')


def source_release(ctx, path, expected):
    path=Path(path).resolve(strict=True)
    require(path==RELEASE and sha(path)==expected, 'GEN59 release identity differs')
    release=read(path)
    require(release['generation']=='GEN59' and all(release[k] is True for k in
            ('passed','reviewed','installed','full_regression_passed','installed_bytes_exact_tested_candidate')), 'GEN59 release not accepted')
    require(release['source_count']==len(release['source_sha256'])==1094 and release['test_scope_files']==350, 'source/test scope differs')
    require(release['baseline_release_sha256']==BASE_SHA and release['previous_deployment']==str(PREVIOUS)
            and release['previous_deployment_sha256']==PREVIOUS_SHA, 'release predecessor differs')
    summary=release['test_summary']
    require(summary['tests']>9893 and all(summary[k]==0 for k in ('failures','errors','skipped')), 'full regression incomplete')
    require(sha(release['actual_regression'])==release['actual_regression_sha256'], 'offline regression changed')
    full=read(release['actual_regression'])
    require(full['passed'] and full['full_regression_passed'] and full['scope_complete']
            and full['source_sha256']==release['source_sha256'] and full['summary']==summary, 'full tested source differs')
    ctx['pins'][str(path)]=expected
    ctx['pins'].update(release['evidence_sha256'])
    recheck(ctx,release['source_sha256'])
    return release


def verify_stopped(ctx, receipt):
    require(receipt['passed'] and receipt['kind']=='GEN59_remote_stopped_install_readiness'
            and all(receipt[k] is True for k in ('remote_stopped','old_remote_exited','remote_registration_absent','remote_processes_absent')),
            'remote stop receipt incomplete')
    require(receipt['node_identities']==ctx['nodes'] and receipt['preserved_node_identities']==ctx['preserved']
            and receipt['old_remote_identity']==ctx['nodes'][REMOTE] and receipt['deployment_sha256']==PREVIOUS_SHA
            and receipt['release_sha256']==BASE_SHA, 'stopped predecessor identity differs')
    require(ctx['legacy'].exited(receipt['old_remote_identity']) and not ctx['legacy'].remote_processes(), 'remote is no longer absent')
    require(sha(receipt['helper_path'])==receipt['helper_sha256'] and sha(receipt['common_path'])==receipt['common_sha256'], 'stopped helper changed')
    require(all(sha(p)==h for p,h in receipt['input_sha256'].items()), 'stopped receipt dependency changed')


def verify_attestation(ctx, value, remote, construction_path, release):
    require(value['kind']=='GEN59_same_process_source_preload' and value['planning_executed'] is False
            and value['physical_grasp_verified'] is False, 'unexpected import attestation')
    command=(Path('/proc')/str(remote['pid'])/'cmdline').read_bytes()
    require(value['node_identity']==dict(remote,cmdline_sha256=hashlib.sha256(command).hexdigest()), 'attestation is not this remote instance')
    require(value['construction']==str(construction_path) and value['construction_sha256']==sha(construction_path)
            and value['release']==str(RELEASE) and value['release_sha256']==sha(RELEASE), 'attestation construction differs')
    require(set(value['loaded_sources'])==PRODUCTION, 'three changed modules not attested')
    for name,row in value['loaded_sources'].items():
        require(row['path']==str(W/name) and row['sha256']==release['source_sha256'][name]
                and row['compiled_from_verified_source_bytes'] is True and sha(W/name)==row['sha256'], 'loaded module hash differs')
    argv=command.split(b'\0')
    require(b'.runtime/start_gen59_remote_20261022.py' in argv and ENTRY.encode() in argv
            and b'__name:=remote_grasp6d_node' in argv, 'new remote entrypoint differs')
    env=dict(item.split(b'=',1) for item in (Path('/proc')/str(remote['pid'])/'environ').read_bytes().split(b'\0') if b'=' in item)
    require(env.get(b'PYTHONNOUSERSITE')==b'1' and env.get(b'PYTHONDONTWRITEBYTECODE')==b'1'
            and env.get(b'OPENBLAS_NUM_THREADS')==b'1' and env.get(b'OMP_NUM_THREADS')==b'1', 'new remote environment differs')
    require(Path(os.fsdecode(env[b'ROS_LOG_DIR'])).resolve()==Path(read(construction_path)['output'])/'remote_ros', 'remote log destination differs')

