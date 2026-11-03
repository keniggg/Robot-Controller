"""GEN60 gateway-only transition primitives. Importing performs no ROS work."""
import argparse
import ast
from copy import deepcopy
import hashlib
import importlib.util
import json
import math
import os
from pathlib import Path
import re
import struct
import sys
import threading
import time
import xmlrpc.client
import xml.etree.ElementTree as ET
from types import SimpleNamespace
import gen60_gateway_release_contract_20261022 as contract
sys.dont_write_bytecode=True
W,M,E,N,O=contract.W,contract.M,contract.E,contract.N,contract.O
PREVIOUS=contract.PREVIOUS
PREVIOUS_SHA=contract.PREVIOUS_SHA
BASE=contract.BASE
BASE_SHA=contract.BASE_SHA
RELEASE=N/'verification/gen60_release_manifest.json'
POST=O.parent/'actual_attempt_r1/post_attempt_idle_r1/result.json'
POST_SHA='19cfb9b00ffd6b29c032801fa73f1d6e4466138f1ffadaf802bf7ba74e45da4c'
GATEWAY='/motion_gateway'
REMOTE='/remote_grasp6d_node'
EPOCH=1792639534222874592
SELECTION=dict(mode='unknown',strategy='observe_return',generation=3,stamp_ns=1792644272392787456)
RUNTIME59=W/'.runtime/gen59_remote_reload_common_20261022.py'
RUNTIME59_SHA='c4fe6271af1197b113c2b5889cbf0fbd47f7fb31f0b221865d456ef9a9043e65'
ENTRY='src/alicia_flexible_grasp_supervisor/scripts/motion_gateway_node.py'
RELAY='devel/lib/alicia_flexible_grasp_supervisor/motion_gateway_node.py'
PACKAGE_RELAY='devel/lib/python3/dist-packages/alicia_flexible_grasp/__init__.py'
HELPERS=tuple(W/'.runtime'/name for name in ('gen60_gateway_release_contract_20261022.py','gen60_gateway_reload_common_20261022.py','prepare_gen60_gateway_reload_20261022.py','install_reload_gen60_gateway_20261022.py','verify_gen60_gateway_reload_20261022.py'))
require,sha,read,storage,save=contract.require,contract.sha,contract.read,contract.storage,contract.save


def module(path,digest,name):
    require(sha(path)==digest,'pinned helper changed: '+str(path))
    spec=importlib.util.spec_from_file_location(name,path);value=importlib.util.module_from_spec(spec);spec.loader.exec_module(value);return value


def helper_map():
    return {str(p):sha(p) for p in HELPERS}


def selection_identity(value):
    from alicia_grasp_modes.selection import parse_selection
    require(type(value) is dict and set(value)=={'mode','strategy','generation','stamp_ns'},'complete selection required')
    return parse_selection(value)


def context(args=None, helper=None):
    if args is not None:
        require(helper is not None and sha(helper)==args.helper_sha256
                and sha(__file__)==args.common_sha256,'reviewed invoking helper/common changed')
    runtime=module(RUNTIME59,RUNTIME59_SHA,'gen60_reviewed_runtime59')
    historical_args=SimpleNamespace(previous_deployment=runtime.PREVIOUS,previous_deployment_sha256=runtime.PREVIOUS_SHA,
        helper_sha256=sha(__file__),common_sha256=RUNTIME59_SHA)
    ctx=runtime.context(historical_args,__file__)
    for p,h in ((PREVIOUS,PREVIOUS_SHA),(BASE,BASE_SHA),(POST,POST_SHA)):
        require(sha(p)==h,'accepted GEN59 binding changed');ctx['pins'][str(p)]=h
    previous,post=read(PREVIOUS),read(POST)
    require(previous['deployment_passed'] and previous['whole_patch_deployed'] and previous['generation']=='GEN59'
        and previous['source_manifest_sha256']==BASE_SHA,'GEN59 deployment not accepted')
    nodes=previous['full_runtime_nodes']
    require(len(nodes)==17 and nodes==previous['final_all_registered_nodes']==post['full_runtime_nodes_before']==post['full_runtime_nodes_after'],'accepted17 identity mismatch')
    require(post['passed'] and post['epoch_ns']==previous['epoch_ns']==EPOCH and post['selection']==SELECTION,'post-attempt reference changed')
    require(previous['control_snapshot']['revision']==67,'authorization revision changed')
    for p,h in previous['input_sha256'].items():
        require(sha(p)==h,'previous deployment input changed: '+p);ctx['pins'][p]=h
    ctx.update(runtime59=runtime,previous=previous,post=post,nodes=nodes,preserved={n:r for n,r in nodes.items() if n!=GATEWAY})
    if args is not None:
        ctx['pins'][str(Path(helper).resolve())]=args.helper_sha256
    ctx['pins'][str(Path(__file__).resolve())]=sha(__file__)
    recheck(ctx)
    return ctx


def recheck(ctx,sources=None):
    value=ctx['runtime59'].recheck(ctx,sources)
    require(value==ctx['previous']['control_snapshot'],'active67 control snapshot changed')
    return value


def identity(ctx,master,caller,name):
    return ctx['runtime59'].identity(ctx,master,caller,name)


def graph(ctx, rospy, master, gateway=None, auxiliaries=None, graph_wait=3.):
    """Authoritative17 map, gateway-only substitution; retained GEN59 remote unit."""
    require(type(graph_wait) in (int,float) and math.isfinite(graph_wait) and 0.<=graph_wait<=3.,
            'graph poll wait must be finite and within original three-second limit')
    v, dep = ctx['v'], ctx['previous']; core = v.properties(dep['unit'])
    require(core == dep['core_service'] and v.process(master.getPid()) == dep['master_process'], 'core/master changed')
    sim = v.properties(dep['simulation_unit']); oldsim = dep['simulation_service']
    require(sim == oldsim and sim['ActiveState']=='active' and sim['SubState']=='running',
            'simulation unit changed')
    expected = dict(ctx['nodes'])
    if gateway is not None:
        expected[GATEWAY] = gateway
    remote=expected[REMOTE];remote_unit=dep['remote_unit']
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
    core_nodes={n:r for n,r in rows.items() if n!=REMOTE}
    preserved_config = dict(ctx['config'], persistent_nodes=sorted(core_nodes))
    configured = v.configured_nodes(core_nodes, preserved_config, core)
    if remote is not None and remote_unit is None:
        require(remote == ctx['nodes'][REMOTE] and remote['parent_pid'] == int(core['MainPID'])
                and v.in_group(remote['pid'],core['ControlGroup']), 'old remote ownership differs')
    if remote is not None and remote_unit is not None:
        unit = v.properties(remote_unit)
        require(unit==dep['remote_service'] and int(unit['MainPID']) == remote['pid']
                and v.in_group(remote['pid'],unit['ControlGroup']), 'retained remote unit ownership differs')
    remote_processes = ctx['legacy'].remote_processes()
    require(remote_processes == ([] if remote is None else [remote]), 'duplicate/unregistered remote process')
    pub = dict(system[0]); ownership = {}
    expected_publishers=dict(ctx['old'].PUBLISHERS,**{
        '/alicia_controller/follow_joint_trajectory/status':'/bessica_d_hw_interface',
        '/hand_controller/follow_joint_trajectory/status':'/bessica_d_hw_interface'})
    for topic,owner in expected_publishers.items():
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
        publishers=dict(master.getSystemState()[0])
        require(all(publishers.get(topic)==['/bessica_d_hw_interface'] for topic in (
            '/alicia_controller/follow_joint_trajectory/status','/hand_controller/follow_joint_trajectory/status')),
            'action status publisher ownership changed')
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
        require(all(states[k]._connection_header.get('callerid')==owners[k] for k in states), 'state publisher owner changed')
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
        require(not states['task'].active and states['gui'].data is True and states['gui']._connection_header.get('latching')=='1' and master.getParam('/gui/joint_direct_mode') is True, 'not GUI-owned manual idle')
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
        selection=selection_identity(master.getParam('/grasp_mode/selection'))
        require(selection['generation']==3 and selection['mode']=='unknown' and selection['strategy']=='observe_return', 'selection differs')
        require(selection==SELECTION and (expected_selection is None or selection==expected_selection), 'selection changed during transition')
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


DESIGN=O/'runtime_design_r1/result.json'
DESIGN_SHA='ca129bce1a1ff5e3ce2b0f1e4a7efdd76c4be21e632df03c6428aeb676637da1'
DEPLOYMENT=O/'runtime_reload_r1/acceptance/result.json'
LOAD_PATHS=(ENTRY,RELAY,PACKAGE_RELAY,'devel/_setup_util.py',
    'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/__init__.py',
    'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/robot/__init__.py',
    'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/robot/contact_sequence_precheck.py',
    'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/robot/contact_pregrasp_gateway.py',
    'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/robot/observation_compensation_gateway.py',
    contract.SELECTOR,'.runtime/start_full_latest_gen36_20261016.sh')
RELAY_SHA='93c72ff075180d1a4fe1112e6ffcb08758b301bf1d0c8824c6e761530cf00ab8'


def sha_value(value):
    return hashlib.sha256(json.dumps(value,sort_keys=True,separators=(',',':'),allow_nan=False).encode()).hexdigest()


def parser(description, evidence=False):
    p=argparse.ArgumentParser(description=description)
    for key in ('helper-review','output'):
        p.add_argument('--'+key,type=Path,required=True)
    for key in ('helper-review-sha256','helper-sha256','common-sha256'):
        p.add_argument('--'+key,required=True)
    if evidence:
        for key in ('manifest','full','review'):
            p.add_argument('--'+key,type=Path,required=True)
            p.add_argument('--'+key+'-sha256',required=True)
    return p


def bind(ctx,path,expected):
    path=Path(path).resolve(strict=True)
    require(sha(path)==expected,'input changed: '+str(path))
    require(str(path) not in ctx['pins'] or ctx['pins'][str(path)]==expected,'conflicting input pin')
    ctx['pins'][str(path)]=expected
    return read(path)


def reviewed_helpers(ctx,args):
    review=bind(ctx,args.helper_review,args.helper_review_sha256)
    require(review['passed'] is True and review['offline_contracts_passed'] is True
            and review['helper_sha256']==helper_map(),'helper review scope/bytes differ')
    ctx['pins'].update(review['helper_sha256'])
    for p,h in review['dependency_sha256'].items():
        require(sha(p)==h,'helper review evidence changed');ctx['pins'][p]=h
    recheck(ctx)
    return review


def frozen_evidence(ctx,args,installed=False):
    for key in ('manifest','full','review'):
        bind(ctx,getattr(args,key),getattr(args,key+'_sha256'))
    checked=contract.validate(args.manifest,args.full,args.review,installed=installed)
    ctx['pins'].update(checked['evidence_sha256'])
    recheck(ctx)
    return checked


def verify_retained_remote(ctx):
    """Actual GEN59 preload and construction; never relabel the remote GEN60."""
    dep=ctx['previous']; base=read(BASE)
    construction_path=Path(dep['startup_configuration'])
    construction=bind(ctx,construction_path,dep['startup_configuration_sha256'])
    require(construction['generation']=='GEN59' and construction['release_sha256']==BASE_SHA
            and construction['source_sha256']==base['source_sha256'],'retained remote construction differs')
    attestation_path=Path(dep['runtime_import_attestation'])
    attestation=bind(ctx,attestation_path,dep['runtime_import_attestation_sha256'])
    ctx['runtime59'].verify_attestation(ctx,attestation,ctx['nodes'][REMOTE],construction_path,base)
    dependency=dep['remote_dependency'];binding_path=Path(dependency['startup_binding'])
    bind(ctx,binding_path,dependency['startup_binding_sha256'])
    binding=ctx['checker'].load_binding(binding_path,dependency['startup_binding_sha256'])
    require(binding['parameters']==construction['parameters']==ctx['config']['remote_parameters']
            and all(binding['node_identity'][k]==ctx['nodes'][REMOTE][k] for k in ('pid','parent_pid','start_ticks')),
            'retained remote instance/configuration differs')
    return dict(construction_path=construction_path,attestation_path=attestation_path,
        binding_path=binding_path,binding=binding)


def fresh_remote_health(ctx,out):
    retained=verify_retained_remote(ctx)
    out=Path(out);storage(out);require(not out.exists(),'health output exists');out.mkdir()
    health=ctx['checker'].evaluate(retained['binding'],out)
    save(out/'result.json',health)
    require(health['passed'] and health['dependency_ready'],'current retained remote dependency not ready')
    return dict(startup_binding=str(retained['binding_path']),startup_binding_sha256=sha(retained['binding_path']),
        readiness=str(out/'result.json'),readiness_sha256=sha(out/'result.json'),passed=True,
        retained_remote_generation='GEN59',health_not_a_prediction_or_geometry_proof=True)


CAMERA_CONTRACT_PATH=W/'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/robot/observation_view_gateway.py'
CAMERA_PRODUCER_PATH=W/'src/alicia_flexible_grasp_supervisor/scripts/camera_node.py'
CAMERA_MANAGER_PATH=W/'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/vision/realsense_manager.py'
CAMERA_ERROR_PATH=W/'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/robot/observation_path_guard.py'
CAMERA_PINS={str(CAMERA_CONTRACT_PATH):'7babdcd76b3e261e207326e9018673390a73d04399ad7fc9dcf9fb95623591c5',
    str(CAMERA_PRODUCER_PATH):'ecca101511eb2430baa703d87d6b516f61728bddc29fcdee5d186a8700fc2409',
    str(CAMERA_MANAGER_PATH):'ecea6f4226f3007bc76c8d09edb3f4d54c82e2c97c839afc6efaca21e9ae5f70',
    str(CAMERA_ERROR_PATH):'228f28203cc319ba17b93b443831b6f2efa505896bf8e04fe1e5c401403246b2',
    str(W/'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/vision/regional_stationary_depth.py'):
        'ebbe85faec63d345da6a88034653f794ccebea685af23d1d8fbed714ecd6fe0d'}
_CAMERA_FUNCTIONS=None


def camera_functions():
    """Reuse exact reviewed pure definitions; never import their ROS modules."""
    global _CAMERA_FUNCTIONS
    require(all(sha(p)==h for p,h in CAMERA_PINS.items()),'camera configuration/diagnostic producer source changed')
    if _CAMERA_FUNCTIONS is None:
        import copy
        namespace={'deepcopy':deepcopy,'copy':copy}
        for path,name,kind in ((CAMERA_ERROR_PATH,'ObservationPathError',ast.ClassDef),
                (CAMERA_CONTRACT_PATH,'_camera_configuration_contract',ast.FunctionDef),
                (CAMERA_MANAGER_PATH,'_parameter_pair_history',ast.FunctionDef)):
            definitions=[n for n in ast.parse(path.read_bytes()).body if isinstance(n,kind) and n.name==name]
            require(len(definitions)==1,'unique pinned pure camera definition missing')
            exec(compile(ast.Module(body=definitions,type_ignores=[]),str(path),'exec'),namespace)
        _CAMERA_FUNCTIONS=namespace
    return _CAMERA_FUNCTIONS


def strict_json(value):
    def pairs(items):
        result={}
        for key,item in items:
            require(key not in result,'duplicate diagnostic JSON key');result[key]=item
        return result
    def nonfinite(value):raise ValueError('nonfinite diagnostic JSON '+value)
    require(type(value) is str,'diagnostic payload must be JSON text')
    return json.loads(value,object_pairs_hook=pairs,parse_constant=nonfinite)


def known_mapping(value,known,required=()):
    require(type(value) is dict and set(required)<=set(value)<=set(known),'unknown/missing diagnostic structure')
    return value


def finite_number(value,minimum=0.,nullable=False):
    require((nullable and value is None) or (type(value) in (int,float) and math.isfinite(value) and value>=minimum),
            'invalid diagnostic finite number')


def raw_frame_schema(raw,camera):
    known_mapping(raw,('color','depth'))
    metadata=('frame_counter','frame_timestamp','sensor_timestamp','backend_timestamp','time_of_arrival','actual_fps')
    for stream,row in raw.items():
        known_mapping(row,('present','frame_number','device_timestamp_ms','timestamp_domain','stream_fps','metadata_raw','unavailable'))
        require(row.get('present') is None or type(row['present']) is bool,'raw frame presence type differs')
        if 'frame_number' in row:require(type(row['frame_number']) is int and row['frame_number']>=0,'invalid SDK frame count')
        if 'device_timestamp_ms' in row:finite_number(row['device_timestamp_ms'])
        if 'timestamp_domain' in row:require(type(row['timestamp_domain']) is str,'raw timestamp domain differs')
        if 'stream_fps' in row:require(type(row['stream_fps']) in (int,float) and row['stream_fps']==camera['fps'],'raw stream configuration differs')
        for key in ('metadata_raw','unavailable'):
            values=known_mapping(row.get(key,{}),metadata if key=='metadata_raw' else (*metadata,'frame','frame_number','device_timestamp_ms','timestamp_domain','stream_fps','profile_signature'))
            require(all(type(v) is int for v in values.values()) if key=='metadata_raw' else all(type(v) is str for v in values.values()),'raw metadata value type differs')


def diagnostic_evidence(value,camera,functions):
    if type(value) is dict and value.get('available') is False:
        require(value==dict(available=False,reason='complete_read_diagnostic_unavailable'),'unavailable diagnostic schema differs')
        return
    fields=('schema','result','failed_stage','total_sec','host_wait','pair_timing','stages_sec',
        'sdk_color_alignment_applied','stationary_pair_filter_host_monotonic','regional_stationary_evidence',
        'available','raw_frames','stationary_pair_history')
    known_mapping(value,fields,('schema','result','failed_stage','total_sec','host_wait','pair_timing','stages_sec','available','raw_frames'))
    require(type(value['schema']) is int and value['schema']==2 and value['result']=='complete'
        and value['failed_stage'] is None and value['available'] is True,'successful read diagnostic schema differs')
    finite_number(value['total_sec'])
    host=known_mapping(value['host_wait'],('enter','exit','elapsed_sec','thread_cpu_elapsed_sec'),('enter','exit'))
    for side in ('enter','exit'):
        for item in known_mapping(host[side],('monotonic_sec','wall_time_sec','thread_cpu_sec')).values():finite_number(item,nullable=True)
    for key in ('elapsed_sec','thread_cpu_elapsed_sec'):
        if key in host:finite_number(host[key],nullable=True)
    pair=known_mapping(value['pair_timing'],('accepted','reason','policy','stream_fps','timestamp_domains','skew_ms','max_skew_ms'),
        ('accepted','reason','policy','stream_fps','timestamp_domains','skew_ms','max_skew_ms'))
    require(pair['accepted'] is True and pair['reason']=='within_nominal_pair_tolerance'
        and pair['policy']=='half_slowest_stream_period','successful pair policy differs')
    require(pair['stream_fps']=={'color':camera['fps'],'depth':camera['fps']},'pair stream configuration differs')
    known_mapping(pair['timestamp_domains'],('color','depth'),('color','depth'))
    require(all(v in ('hardware_clock','global_time') for v in pair['timestamp_domains'].values()),'pair timestamp domain invalid')
    finite_number(pair['skew_ms']);finite_number(pair['max_skew_ms'])
    require(pair['max_skew_ms']==500./camera['fps'],'pair tolerance configuration differs')
    stages={'wait_for_frames','raw_frame_diagnostics','pair_timing','align','extract_frames','color_array',
        'stationary_evidence','regional_stationary_depth','stationary_temporal_init','stationary_temporal_process',
        'color_projection','depth_array_and_clip','pair_rectification','simulate','stationary_hole_check','visual_stationary_evidence'}
    applied=camera.get('runtime_profile',{}).get('depth_filters',{}).get('applied',[])
    stages.update('depth_filter_%d_%s'%(i,item['name']) for i,item in enumerate(applied))
    for item in known_mapping(value['stages_sec'],stages).values():finite_number(item)
    if 'sdk_color_alignment_applied' in value:require(type(value['sdk_color_alignment_applied']) is bool,'alignment diagnostic type differs')
    if 'stationary_pair_filter_host_monotonic' in value:finite_number(value['stationary_pair_filter_host_monotonic'])
    raw_frame_schema(value['raw_frames'],camera)
    base_contract=functions['_camera_configuration_contract'](camera)
    if 'regional_stationary_evidence' in value:
        regional=value['regional_stationary_evidence']
        known_mapping(regional,('enabled','active','fully_active','whole_frame_stationarity_qualified','owner_schema','reason',
            'alpha','delta_depth_units','persistence','coordinate_domain','filtered_pixel_count','filtered_fraction','changed_pixel_count',
            'current_zero_pixels_preserved','regions','nested_center_shadow','global_history_reset_reason','error_kind'),
            ('enabled','owner_schema','alpha','delta_depth_units','persistence','coordinate_domain','regions','nested_center_shadow'))
        temporal=camera['runtime_profile']['stationary_temporal']
        require(regional['enabled'] is True and temporal.get('regional_enabled') is True
            and regional['coordinate_domain']=='sdk_color_aligned_before_pair_rectification'
            and all(regional[k]==temporal[k] for k in ('owner_schema','alpha','delta_depth_units','persistence')),
            'regional diagnostic configuration differs')
        for key in ('active','fully_active','whole_frame_stationarity_qualified','current_zero_pixels_preserved'):
            if key in regional:require(type(regional[key]) is bool,'regional diagnostic boolean invalid')
        if 'current_zero_pixels_preserved' in regional:
            require(regional['current_zero_pixels_preserved'] is True,'regional zero-depth preservation invariant differs')
        for key in ('filtered_pixel_count','changed_pixel_count'):
            if key in regional:require(type(regional[key]) is int and regional[key]>=0,'regional diagnostic count invalid')
        if 'filtered_fraction' in regional:finite_number(regional['filtered_fraction']);require(regional['filtered_fraction']<=1.,'regional fraction invalid')
        require(type(regional['regions']) is list,'regional rows type differs')
        visual_keys=('qualified','reason','window_sec','max_motion_px','max_points','tracked_points','coordinate_units',
            'frame_number','device_timestamp_ms','timestamp_domain','stream_fps','continuous_frames','covered_quadrants',
            'host_quiet_span_sec','quiet_span_sec','motion_median_px','motion_p90_px')
        for row in regional['regions']:
            known_mapping(row,('name','bounds_xyxy','owned_pixels','qualified','warming','history_reset','filter_applied','visual'),
                ('name','bounds_xyxy','owned_pixels'))
            if row.get('visual') is not None:known_mapping(row['visual'],visual_keys)
        injected=deepcopy(camera);injected['runtime_profile']['stationary_temporal']['regions']=deepcopy(regional['regions'])
        require(functions['_camera_configuration_contract'](injected)==base_contract,'regional diagnostic fixed schema or unknown row differs')
        shadow=known_mapping(regional['nested_center_shadow'],('diagnostic_only','pixels_changed','used_for_depth_permission','motion_authorized',
            'temporal_filter_created','bounds_xyxy','coordinate_domain','reason','qualified','visual'),
            ('diagnostic_only','pixels_changed','used_for_depth_permission','motion_authorized','temporal_filter_created','bounds_xyxy','coordinate_domain','reason','qualified','visual'))
        require(shadow['diagnostic_only'] is True and all(shadow[k] is False for k in
            ('pixels_changed','used_for_depth_permission','motion_authorized','temporal_filter_created','qualified'))
            and shadow['coordinate_domain']=='sdk_color_aligned_before_pair_rectification'
            and shadow['reason']=='diagnostic_disabled' and shadow['visual'] is None,'unreviewed shadow diagnostic configuration')
        intrinsic=camera['runtime_profile']['color_intrinsics'];w,h=intrinsic['width'],intrinsic['height']
        x0,y0,x1,y1=int(.2*w),int(.1*h),int(.8*w),int(.9*h)
        nw,nh=(x1-x0)//2,(y1-y0)//2;left,top=x0+(x1-x0-nw)//2,y0+(y1-y0-nh)//2
        require(shadow['bounds_xyxy'] in (None,[left,top,left+nw,top+nh]),'shadow diagnostic ROI differs')
    if 'stationary_pair_history' in value:
        history=known_mapping(value['stationary_pair_history'],('state','reason','diagnostic_only','authorizes_pair_or_motion',
            'history_epoch','last_complete_raw_frames','last_complete_filter_host_monotonic','rejected_raw_frames',
            'latest_rejected_raw_frames','first_rejection_host_monotonic','rejected_packet_count','next_accepted_raw_frames',
            'failed_stage','existing_gate_evaluation'),('diagnostic_only','authorizes_pair_or_motion'))
        require(history['diagnostic_only'] is True and history['authorizes_pair_or_motion'] is False,
                'pair-history authorization semantics differ')
        for key in ('state','reason'):
            if key in history:require(type(history[key]) is str,'pair-history text field invalid')
        if 'failed_stage' in history:
            require(history['failed_stage'] is None or type(history['failed_stage']) is str,'pair-history failed stage invalid')
        for key in ('history_epoch','rejected_packet_count'):
            if key in history:require(type(history[key]) is int and history[key]>=0,'pair-history count invalid')
        for key in ('last_complete_filter_host_monotonic','first_rejection_host_monotonic'):
            if key in history:finite_number(history[key],nullable=True)
        # The producer's XML-RPC converter removes nulls. Validate the raw tree
        # first so an unknown null cannot disappear before configuration review.
        for key in ('last_complete_raw_frames','rejected_raw_frames','latest_rejected_raw_frames','next_accepted_raw_frames'):
            if history.get(key) is not None:raw_frame_schema(history[key],camera)
        if 'existing_gate_evaluation' in history:
            evaluation=known_mapping(history['existing_gate_evaluation'],('encoder_active','alignment_proven',
                'filtered_pixel_count','regional_global_reset_reason','regions'))
            for key in ('encoder_active','alignment_proven'):
                if key in evaluation:require(type(evaluation[key]) is bool,'pair-history gate boolean invalid')
            if 'filtered_pixel_count' in evaluation:
                require(type(evaluation['filtered_pixel_count']) is int and evaluation['filtered_pixel_count']>=0,
                        'pair-history gate count invalid')
            if 'regional_global_reset_reason' in evaluation:
                require(evaluation['regional_global_reset_reason'] is None or type(evaluation['regional_global_reset_reason']) is str,
                        'pair-history reset reason invalid')
            require(type(evaluation.get('regions',[])) is list,'pair-history region list differs')
            for row in evaluation.get('regions',[]):
                known_mapping(row,('name','filter_applied','qualified','warming','history_reset'))
                for key,item in row.items():
                    require(item is None or type(item) is (str if key=='name' else bool),'pair-history region leaf invalid')
        injected=deepcopy(camera)
        injected['runtime_profile']['stationary_temporal']['pair_history']=functions['_parameter_pair_history'](history)
        require(functions['_camera_configuration_contract'](injected)==base_contract,
                'pair-history diagnostic fixed schema/unknown configuration differs')


def diagnostic_configuration(envelope,camera,functions):
    require(type(envelope) is dict and type(envelope.get('schema')) is int and envelope['schema']==1
        and envelope.get('encoding')=='json','camera diagnostic envelope differs')
    payload=strict_json(envelope['payload_json'])
    require(type(payload) is dict and type(payload.get('schema')) is int and payload['schema']==1
        and type(payload.get('capacity')) is int and payload['capacity']==32
        and payload.get('diagnostic_only') is True and payload.get('motion_authorized') is False
        and payload.get('scope')=='successful read candidates; match exact source stamp to a received RGB-D pair; publication is not guaranteed',
        'camera diagnostic producer schema differs')
    require(type(payload.get('geometry_generation')) is int and payload['geometry_generation']>=0
        and payload.get('frame_id')==camera['frame_id']
        and payload.get('geometry_activation_stamp_ns')==camera['runtime_profile']['geometry_activation_stamp_ns'],
        'camera diagnostic geometry identity differs')
    require(type(payload.get('copy_errors')) is int and payload['copy_errors']>=0,'camera diagnostic copy counter invalid')
    rows=payload.get('entries');require(type(rows) is list and len(rows)<=32,'diagnostic history capacity differs')
    previous_stamp=-1;previous_monotonic=-1.
    rowkeys={'source_stamp_ns','read_completed_monotonic','geometry_generation','frame_id','geometry_activation_stamp_ns',
        'image_header_clock','exposure_clock_calibrated','publication_outcome_unknown','evidence'}
    for row in rows:
        require(type(row) is dict and set(row)==rowkeys,'unknown/missing successful-read row fields')
        require(type(row['source_stamp_ns']) is int and row['source_stamp_ns']>max(0,previous_stamp),'diagnostic source order invalid')
        finite_number(row['read_completed_monotonic']);require(row['read_completed_monotonic']>previous_monotonic,'diagnostic monotonic order invalid')
        require(all(type(row[k]) is type(payload[k]) and row[k]==payload[k] for k in ('geometry_generation','frame_id','geometry_activation_stamp_ns'))
            and row['image_header_clock']=='host_read_completion' and row['exposure_clock_calibrated'] is False
            and row['publication_outcome_unknown'] is True,'successful-read geometry/clock semantics differ')
        diagnostic_evidence(row['evidence'],camera,functions)
        previous_stamp,previous_monotonic=row['source_stamp_ns'],row['read_completed_monotonic']
    result=deepcopy(envelope)
    # Unknown wrapper/payload keys remain in the configuration contract. Unknown
    # entry/evidence structure fails validation above, never vanishes silently.
    projected=deepcopy(payload);projected.pop('entries');projected.pop('copy_errors')
    result['payload_json']=projected
    return result


def parameter_configuration(values):
    projected=deepcopy(values);functions=camera_functions()
    if 'camera' in values:
        require(type(values['camera']) is dict,'camera configuration is not a mapping')
        projected['camera']=functions['_camera_configuration_contract'](values['camera'])
    if 'camera_read_diagnostics' in values:
        require('camera' in values,'diagnostic exists without camera configuration')
        projected['camera_read_diagnostics']=diagnostic_configuration(values['camera_read_diagnostics'],values['camera'],functions)
    return projected


def parameters(master,output=None,label=None):
    """Retain exact raw tree; compare only the strictly validated configuration."""
    values=master.getParam('/')
    if output is not None:
        require(type(label) is str and re.fullmatch('[a-z0-9_]+',label),'parameter evidence label invalid')
        raw_path=Path(output)/('parameters_'+label+'_raw.json')
        save(raw_path,dict(recorded_ns=time.time_ns(),parameter_values=values))
    require(type(values) is dict,'parameter root must be a mapping')
    rgb=values.get('rgb_acquisition',{})
    require(type(rgb) is dict,'RGB namespace invalid')
    ledger=rgb.get('ledger')
    if ledger is not None:
        require(type(ledger) is dict and ledger.get('phase') not in ('reserved','executing'),
                'RGB acquisition ledger busy or malformed')
    projection=parameter_configuration(values)
    result=dict(all_parameters_sha256=sha_value(values),parameter_values=values,
        configuration_sha256=sha_value(projection),configuration_values=projection,
        camera_contract_source_sha256=CAMERA_PINS,rgb_ledger=deepcopy(ledger),rgb_ledger_idle=True)
    if output is not None:result['raw_evidence']=dict(path=str(raw_path),sha256=sha(raw_path))
    return result


def parameter_differences(before,after,path=''):
    if type(before) is dict and type(after) is dict:
        rows=[]
        for key in sorted(set(before)|set(after)):
            if key not in before or key not in after:
                rows.append(dict(path=path+'/'+key,present_before=key in before,present_after=key in after))
            else:rows.extend(parameter_differences(before[key],after[key],path+'/'+key))
        return rows
    if type(before) is not type(after) or before!=after:
        return [dict(path=path,before_sha256=sha_value(before),after_sha256=sha_value(after))]
    return []


def same_parameters(before,after,output=None,label=None):
    differences=parameter_differences(before['configuration_values'],after['configuration_values'])
    raw_differences=parameter_differences(before['parameter_values'],after['parameter_values'])
    integrity=all(row['all_parameters_sha256']==sha_value(row['parameter_values'])
        and row['configuration_sha256']==sha_value(row['configuration_values'])
        and row['rgb_ledger']==row['parameter_values'].get('rgb_acquisition',{}).get('ledger')
        and row['camera_contract_source_sha256']==CAMERA_PINS for row in (before,after))
    passed=integrity and not differences and before['rgb_ledger']==after['rgb_ledger']
    if output is not None:
        require(type(label) is str and re.fullmatch('[a-z0-9_]+',label),'parameter comparison label invalid')
        save(Path(output)/('parameter_comparison_'+label+'.json'),dict(passed=passed,snapshot_integrity=integrity,
            raw_changed_paths=raw_differences,configuration_changed_paths=differences,
            before=before,after=after,comparison_scope='all configuration including unknown fields; known validated camera telemetry excluded'))
    require(passed,'parameter snapshot invalid, configuration or RGB ledger changed: '+repr([r['path'] for r in differences]))


def service_owners(ctx,master):
    bind(ctx,DESIGN,DESIGN_SHA)
    names=read(DESIGN)['constructor_review']['registered_service_names']
    require(len(names)==len(set(names))==31,'reviewed gateway service inventory differs')
    providers=dict(master.getSystemState()[2])
    require(all(providers.get(n)==[GATEWAY] for n in names),'gateway service missing/duplicate owner')
    # A provider name is only meaningful with an independently resolved PID.
    require(identity(ctx,master,'/gen60_gateway_owner_readonly',GATEWAY)==ctx['expected_gateway'],
            'gateway identity changed during service ownership check')
    return {n:providers[n] for n in names}


def cache_status(path,source_bytes,source_stat,magic):
    """Conservative cache validity classification without importing its code."""
    path=Path(path)
    if not path.exists():
        return dict(path=str(path),present=False,usable=False,reason='absent')
    require(path.is_file() and not path.is_symlink(),'target bytecode is not a regular file')
    raw=path.read_bytes();st=path.stat()
    row=dict(path=str(path),present=True,sha256=hashlib.sha256(raw).hexdigest(),
        device=st.st_dev,inode=st.st_ino,size=st.st_size,mtime_ns=st.st_mtime_ns)
    if len(raw)<16 or raw[:4]!=magic:
        return dict(row,usable=False,reason='invalid_header_or_magic')
    flags=struct.unpack('<I',raw[4:8])[0]
    if flags not in (0,1,3):
        return dict(row,usable=False,reason='invalid_flags',flags=flags)
    if flags==0:
        mtime,size=struct.unpack('<II',raw[8:16])
        valid=mtime==(int(source_stat.st_mtime)&0xffffffff) and size==(len(source_bytes)&0xffffffff)
        return dict(row,flags=flags,source_mtime=mtime,source_size=size,usable=valid,
                    reason='matching_timestamp_size' if valid else 'timestamp_or_size_mismatch')
    # Unchecked hash caches can run regardless of the current source. Checked
    # hash cache validation is conservative: quarantine it too before respawn.
    return dict(row,flags=flags,usable=True,reason='hash_cache_requires_exact_target_quarantine')


def target_caches():
    source=W/contract.SELECTOR
    folder=source.parent/'__pycache__'
    paths=list(folder.glob(source.stem+'.*.pyc')) if folder.exists() else []
    legacy=source.with_suffix('.pyc')
    if legacy.exists():paths.append(legacy)
    return [cache_status(p,source.read_bytes(),source.stat(),importlib.util.MAGIC_NUMBER) for p in sorted(paths)]


def check_launch_argument(argv,cwd,launch):
    cwd=Path(cwd);launch=Path(launch)
    launch=launch if launch.is_absolute() else W/launch
    arguments=[(Path(a) if Path(a).is_absolute() else cwd/a).resolve() for a in argv if a.endswith('.launch')]
    require(cwd==W and any(Path(a).name=='roslaunch' for a in argv) and arguments==[launch.resolve()],
            'original parent launch entry/cwd differs')
    return str(launch.resolve())


def default_gateway_cwd(env):
    value=Path(os.fsdecode(env[b'ROS_HOME'])) if b'ROS_HOME' in env else Path(os.fsdecode(env[b'HOME']))/'.ros'
    require(value.is_absolute(),'gateway ROS_HOME must be absolute')
    return value.resolve()


def same_gateway_load_contract(before,after):
    require(before['processes']['parent']==after['processes']['parent'],'original parent load configuration changed')
    for key in ('cwd','executable','pythonpath','environment_sha256'):
        require(before['processes']['gateway'][key]==after['processes']['gateway'][key],
                'gateway startup configuration changed: '+key)


def source_load_proof(ctx,gateway,source_map,parent_expected=None,allow_usable_cache=False):
    """New-process/source/path/cache evidence, not process-memory attestation."""
    require(sys.version_info[:2]==(3,8),'gateway load proof requires the production Python3.8 interpreter')
    v=ctx['v'];require(v.process(gateway['pid'])==gateway,'gateway process changed')
    parent=v.process(gateway['parent_pid'])
    require(parent['pid']==int(ctx['previous']['core_service']['MainPID']), 'gateway parent is not accepted core')
    require(parent_expected is None or parent==parent_expected,'original roslaunch parent changed')
    require(v.in_group(parent['pid'],ctx['previous']['core_service']['ControlGroup'])
            and v.in_group(gateway['pid'],ctx['previous']['core_service']['ControlGroup']), 'gateway/parent cgroup changed')
    declared=ctx['config']['nodes'][GATEWAY]
    require(declared['respawn'] is True and declared['required'] is False and declared['args']==''
            and declared['launch_prefix'] is None and declared['executable']=='motion_gateway_node.py',
            'original-parent respawn declaration changed')
    launch_nodes=[n for n in ET.parse(declared['source_file']).getroot().iter('node') if n.get('name')=='motion_gateway']
    require(len(launch_nodes)==1 and launch_nodes[0].get('cwd') is None,
            'reviewed gateway default ROS_HOME working-directory declaration changed')
    source_pins={str(W/p):sha(W/p) for p in LOAD_PATHS}
    require(source_pins[str(W/RELAY)]==RELAY_SHA,'reviewed devel entry relay changed')
    for relative in LOAD_PATHS:
        if relative in source_map:require(source_pins[str(W/relative)]==source_map[relative],'gateway source load chain changed')
    require(sha(DESIGN)==DESIGN_SHA,'load design changed')
    design=read(DESIGN)
    for p,h in design['input_sha256'].items():
        if p in source_pins and p!=str(W/contract.SELECTOR):require(source_pins[p]==h,'reviewed relay/source chain changed')
    require(str(W/ENTRY) in (W/RELAY).read_text() and
            str(W/'src/alicia_flexible_grasp_supervisor/src') in (W/PACKAGE_RELAY).read_text(), 'catkin relay source path differs')
    rows={}
    for label,process in (('parent',parent),('gateway',gateway)):
        proc=Path('/proc')/str(process['pid'])
        argv=[os.fsdecode(x) for x in (proc/'cmdline').read_bytes().split(b'\0') if x]
        env=dict(item.split(b'=',1) for item in (proc/'environ').read_bytes().split(b'\0') if b'=' in item)
        require(all(env.get(k)==b'1' for k in (b'PYTHONNOUSERSITE',b'PYTHONDONTWRITEBYTECODE',b'OPENBLAS_NUM_THREADS',b'OMP_NUM_THREADS')),
                'parent/gateway environment differs')
        require(b'PYTHONPYCACHEPREFIX' not in env and b'PYTHONHOME' not in env,'unexpected Python path/cache override')
        paths=[Path(os.fsdecode(p)).resolve() for p in env.get(b'PYTHONPATH',b'').split(b':') if p]
        require(paths and paths[0]==W/'devel/lib/python3/dist-packages'
                and Path('/opt/ros/noetic/lib/python3/dist-packages') in paths,'Python overlay precedence differs')
        cwd=Path(os.readlink(proc/'cwd')).resolve()
        if label=='gateway':
            require(cwd==default_gateway_cwd(env), 'gateway default ROS_HOME working directory differs')
            require(str(W/RELAY) in argv and '__name:=motion_gateway' in argv
                    and Path(os.readlink(proc/'exe')).resolve()==Path(sys.executable).resolve(), 'gateway interpreter/entry differs')
        else:
            check_launch_argument(argv,cwd,ctx['config']['launch'])
        rows[label]=dict(identity=process,argv=argv,executable=os.readlink(proc/'exe'),
            cwd=str(cwd),pythonpath=[str(p) for p in paths],environment_sha256=hashlib.sha256((proc/'environ').read_bytes()).hexdigest())
    caches=target_caches()
    require(allow_usable_cache or not any(r['usable'] for r in caches),'usable target bytecode must be quarantined before respawn')
    require(v.process(gateway['pid'])==gateway and v.process(parent['pid'])==parent,'load proof process raced')
    return dict(parent_identity=parent,processes=rows,load_source_sha256=source_pins,target_bytecode=caches,
        usable_target_cache_absent=not any(r['usable'] for r in caches),
        eager_import_before_service_registration=True,same_process_module_attestation=False,
        scope='process identity, original eager entry/path, source hashes and explicit target-cache validity; no memory introspection')


def source_release(ctx,path,expected):
    path=Path(path).resolve(strict=True)
    require(path==RELEASE,'unexpected GEN60 release path')
    release=bind(ctx,path,expected)
    require(release['generation']=='GEN60' and all(release[k] is True for k in
        ('passed','reviewed','installed','full_regression_passed','installed_bytes_exact_tested_candidate')),'GEN60 release incomplete')
    require(release['baseline_release_sha256']==BASE_SHA and release['previous_deployment']==str(PREVIOUS)
        and release['previous_deployment_sha256']==PREVIOUS_SHA,'GEN60 predecessor differs')
    require(release['source_count']==len(release['source_sha256'])==1097 and release['test_scope_files']==352
        and release['test_summary']==dict(tests=10044,failures=0,errors=0,skipped=0),'GEN60 exact scope differs')
    checked=contract.validate(Path(release['candidate_manifest']),Path(release['actual_regression']),Path(release['review']),installed=True)
    require(checked['source_sha256']==release['source_sha256'] and checked['evidence_sha256']==release['evidence_sha256'],
            'GEN60 tested/install evidence differs')
    ctx['pins'].update(release['evidence_sha256']);recheck(ctx,release['source_sha256'])
    return release


def validate_partition(ctx,dep):
    before=ctx['nodes'];nodes=dep['full_runtime_nodes']
    require(dep['generation']=='GEN60' and dep['deployment_kind']=='original_parent_gateway_respawn_sixteen_preserved'
            and dep['deployment_passed'] is True and dep['whole_patch_deployed'] is True
            and dep['full_restart'] is False and dep['all_processes_replaced'] is False,'wrong GEN60 deployment kind')
    require(len(nodes)==17 and set(nodes)==set(before) and nodes==dep['all_registered_nodes']==dep['final_all_registered_nodes'],
            'GEN60 complete17 inventory differs')
    require(dep['before']==before and dep['preserved_before']==dep['preserved_after']==ctx['preserved']
            and all(nodes[k]==v for k,v in ctx['preserved'].items())
            and dep['replaced_before']=={GATEWAY:before[GATEWAY]} and dep['replaced_after']=={GATEWAY:nodes[GATEWAY]}
            and nodes[GATEWAY]!=before[GATEWAY] and nodes[GATEWAY]['parent_pid']==before[GATEWAY]['parent_pid'],
            'GEN60 gateway-only partition differs')
    require(dep['replaced_count']==1 and dep['preserved_count']==16 and dep['gateway_original_parent_preserved'] is True
            and dep['epoch_ns']==EPOCH and selection_identity(dep['selection'])==SELECTION,'GEN60 reference differs')
    require(dep['composite_runtime']['gateway_generation']=='GEN60' and dep['composite_runtime']['remote_generation']=='GEN59'
            and dep['gateway_source_load_proof']['same_process_module_attestation'] is False,'composite provenance overstated')
    for k in ('startup_configuration','startup_configuration_sha256','runtime_import_attestation','runtime_import_attestation_sha256','remote_unit','remote_service'):
        require(dep[k]==ctx['previous'][k],'retained remote receipt changed: '+k)
    return nodes


def accepted_context(path,expected):
    ctx=context();path=Path(path).resolve(strict=True)
    require(path==DEPLOYMENT,'unexpected GEN60 acceptance path')
    dep=bind(ctx,path,expected);nodes=validate_partition(ctx,dep)
    release=source_release(ctx,dep['source_manifest'],dep['source_manifest_sha256'])
    require(dep['source_sha256']==release['source_sha256'] and dep['test_summary']==release['test_summary'], 'accepted source differs')
    for p,h in dep['input_sha256'].items():
        require(sha(p)==h,'accepted input changed');ctx['pins'][p]=h
    ctx.update(before_nodes=ctx['nodes'],nodes=nodes,acceptance=dep,expected_gateway=nodes[GATEWAY],release=release)
    verify_retained_remote(ctx);recheck(ctx,release['source_sha256'])
    return ctx
