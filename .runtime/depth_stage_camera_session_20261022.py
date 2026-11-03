"""Camera-only diagnostic transition, with manual/idle and source identity checks."""
import argparse
from collections import deque
import copy
import hashlib
import http.client
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import threading
import time
import xmlrpc.client

W = Path('/home/zhuyupei/alicia_wa_full/.worktrees/protocol-v3-upgrade')
E = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
F = E/'ros_log/full_latest_gen41_restart_20261018T011222Z/gen58_resume_r1/observation_pose_fix/held_lattice_fix_r1/seed_wait_fix_r1/contact_feedback_fix_r1'
OUT = F/'depth_stage_online_session'
ENTRY = W/'.runtime/depth_stage_camera_entry_20261022.py'
VISION = 'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/vision'


def require(value, reason):
    if not value:
        raise RuntimeError(reason)


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def read_ref(reference):
    raw = Path(reference['path']).read_bytes()
    require(hashlib.sha256(raw).hexdigest() == reference['sha256'],
            'evidence source changed: '+reference['path'])
    return json.loads(raw)


def process_identity(row):
    return {key:row[key] for key in ('pid','start_ticks','parent_pid')}


def restart_binding(contract_ref, acceptance_ref):
    contract = read_ref(contract_ref)
    accepted = read_ref(acceptance_ref)
    require(contract['kind'] == 'manual_full_stack_cold_start'
            and contract['workspace_root'] == str(W), 'wrong current full-stack contract')
    require(accepted.get('passed') is True and accepted.get('manual_only') is True
            and accepted['contract'] == contract_ref, 'current full-stack acceptance did not pass')
    configuration = read_ref(contract['configuration'])
    startup_path = Path(contract['output_root'])/'startup.json'
    startup_ref = dict(path=str(startup_path),sha256=digest(startup_path))
    startup = read_ref(startup_ref)
    require(startup.get('dispatched') is True and startup['contract'] == contract_ref,
            'full-stack startup is not bound to this contract')
    if 'verifier_revision' in accepted:
        read_ref(accepted['verifier_revision'])
        require(accepted.get('preserved_startup_instances') is True,
                'revised acceptance did not preserve startup instances')
    camera = configuration['nodes']['/supervisor_camera']
    require(camera['respawn'] is False and camera['required'] is False,
            'camera launch would respawn or stop the full stack')
    require(configuration['parameters']['/gui/joint_direct_mode'] is True
            and accepted['selection']['mode'] == 'unknown', 'manual unknown-mode startup required')
    devices = startup['cold']['devices']
    camera_devices = {p:rdev for p,rdev in devices['devices'].items()
                      if p != devices['serial_device']}
    require(camera_devices and devices['camera_usb'], 'camera device identities absent')
    return dict(contract=contract_ref,acceptance=acceptance_ref,startup=startup_ref,
        configuration=contract['configuration'],accepted=accepted,
        startup_started_ns=startup['started_ns'],
        core_unit=startup['components']['core']['unit'],camera_devices=camera_devices)


def save(path, value):
    require(os.path.ismount('/media/zhuyupei/SanStand'), 'SanStand unavailable')
    path = Path(path)
    require(E.resolve() in path.resolve().parents, 'external evidence path required')
    with path.open('x') as stream:
        os.chmod(path, 0o600)
        json.dump(value, stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.write('\n')
        stream.flush()
        os.fsync(stream.fileno())
    return digest(path)


def identity(pid):
    p = Path('/proc')/str(pid)
    fields = (p/'stat').read_text().rsplit(')', 1)[1].split()
    require(fields[0] != 'Z', 'process is zombie')
    return dict(pid=int(pid), start_ticks=fields[19], parent_pid=int(fields[1]))


def alive(row):
    try:
        current = identity(row['pid'])
        # Reparenting does not release SDK ownership. Keep parent_pid as
        # provenance, but only PID and start time identify the live process.
        return (current['pid'] == row['pid']
                and current['start_ticks'] == row['start_ticks'])
    except (OSError, RuntimeError):
        return False


class Transport(xmlrpc.client.Transport):
    def make_connection(self, host):
        return http.client.HTTPConnection(host, timeout=2.)


class Monitor:
    def __init__(self):
        import rospy
        import rosgraph
        from sensor_msgs.msg import JointState, Image
        from std_msgs.msg import Bool, String, Header
        from actionlib_msgs.msg import GoalStatusArray
        from alicia_flexible_grasp_supervisor.msg import GraspState
        self.rospy = rospy
        rospy.init_node('finite_depth_stage_session', anonymous=True, disable_rosout=True)
        self.master = rosgraph.Master(rospy.get_name())
        self.lock, self.values, self.joints = threading.Lock(), {}, deque(maxlen=80)
        self.sdk_commands = deque(maxlen=80)
        self.subs = []
        self.images = deque(maxlen=1200)
        for kind, topic in [('color','/supervisor/camera/color/image_raw'),
                            ('depth','/supervisor/camera/depth/image_raw')]:
            self.subs.append(rospy.Subscriber(topic, Image,
                lambda m,k=kind:self.receive_image(k,m), queue_size=3, buff_size=4*1024*1024))
        for key, topic, kind in [
            ('joint','/alicia_d/accepted_joint_states',JointState),
            ('sdk','/alicia_d/sdk_command',JointState),
            ('manual','/gui/joint_direct_mode',Bool), ('task','/grasp/state',GraspState),
            ('epoch','/alicia_d/control_reference_epoch',Header),
            ('mode','/grasp_mode/status',String),('actuation','/alicia_d/actuation_status',String),
            ('arm','/alicia_controller/follow_joint_trajectory/status',GoalStatusArray),
            ('hand','/hand_controller/follow_joint_trajectory/status',GoalStatusArray)]:
            self.subs.append(rospy.Subscriber(topic,kind,lambda m,k=key:self.receive(k,m),queue_size=80))

    def receive_image(self, kind, msg):
        row = dict(kind=kind,stamp_ns=msg.header.stamp.to_nsec(),width=msg.width,
            height=msg.height,encoding=msg.encoding,step=msg.step,is_bigendian=msg.is_bigendian,
            pixel_sha256=hashlib.sha256(msg.data).hexdigest(),
            caller=msg._connection_header.get('callerid'),received_ns=time.time_ns())
        with self.lock:
            self.images.append(row)

    def receive(self, key, msg):
        with self.lock:
            self.values[key] = (time.monotonic(),msg)
            if key == 'joint' and (not self.joints or msg.header.stamp != self.joints[-1].header.stamp):
                self.joints.append(msg)
            if key == 'sdk' and (not self.sdk_commands or msg.header.stamp != self.sdk_commands[-1].header.stamp):
                self.sdk_commands.append(msg)

    def sample(self):
        deadline = time.monotonic()+4.
        while time.monotonic()<deadline:
            with self.lock:
                if set(self.values) >= {'joint','manual','task','epoch','mode','actuation','arm','hand'} and len(self.joints)>=20:
                    break
            time.sleep(.025)
        with self.lock:
            values=dict(self.values); joints=list(self.joints)[-20:]
            sdk=list(self.sdk_commands)[-20:]
        require(set(values) >= {'joint','manual','task','epoch','mode','actuation','arm','hand'}
                and len(joints)==20, 'missing idle evidence')
        now = self.rospy.Time.now().to_nsec()
        stamps=[m.header.stamp.to_nsec() for m in joints]
        owners=dict(joint='/alicia_d_driver_node',sdk='/alicia_d_driver_node',epoch='/alicia_d_driver_node',
            actuation='/alicia_d_driver_node',manual='/alicia_supervisor_gui',task='/grasp_task_node',
            mode='/grasp_mode_router',arm='/bessica_d_hw_interface',hand='/bessica_d_hw_interface')
        require(all(m._connection_header.get('callerid')==owners[k] for k,(_,m) in values.items()),'state owner differs')
        require(all(b>a for a,b in zip(stamps,stamps[1:])) and 0<=now-stamps[-1]<250_000_000,'SDK feedback stale/unordered')
        names=list(joints[-1].name)
        epoch=values['epoch'][1].stamp.to_nsec()
        require(epoch>0 and epoch<=now and len(set(names))==len(names)
            and {'Joint%d'%i for i in range(1,7)} <= set(names)
            and all(m.header.frame_id=='sdk_measured' and list(m.name)==names
                and len(m.position)==len(names) and all(math.isfinite(v) for v in m.position)
                and epoch<=m.header.stamp.to_nsec()<=now for m in joints),'SDK layout/epoch differs')
        spans=[max(m.position[i] for m in joints)-min(m.position[i] for m in joints) for i in range(len(names))]
        require(names and all(0<=v<=.004 for v in spans),'joints moving')
        require(values['manual'][1].data is True and self.master.getParam('/gui/joint_direct_mode') is True,'manual mode required')
        require(not values['task'][1].active
                and values['task'][1]._connection_header.get('latching')=='1','task active or not current latched state')
        router=json.loads(values['mode'][1].data)
        require(not any(router.get(k) for k in ('active','start_reserved','pending')),'router active')
        for key in ('arm','hand'):
            msg=values[key][1]
            require(0<=now-msg.header.stamp.to_nsec()<2_000_000_000 and not any(g.status in (0,1,6,7) for g in msg.status_list),'action active/stale')
        return dict(recorded_ns=time.time_ns(),names=names,positions=list(joints[-1].position),
            spans=spans,stamps_ns=stamps,epoch_ns=epoch,
            manual=True,task_active=False,actions_active=False,router=router,
            task_state_contract='event_driven_latched_current_publisher',
            sdk_command_evidence=[dict(stamp_ns=m.header.stamp.to_nsec(),frame_id=m.header.frame_id,
                names=list(m.name),positions=list(m.position)) for m in sdk],
            actuation_status=values['actuation'][1].data,
            actuation_confirmation_not_required_for_camera_only_diagnostic=True,
            selection=self.master.getParam('/grasp_mode/selection'))

    def node(self, name):
        uri=self.master.lookupNode(name)
        response=xmlrpc.client.ServerProxy(uri,transport=Transport()).getPid(self.rospy.get_name())
        require(response[0]==1,'node getPid failed')
        return identity(response[2])

    def close(self):
        for sub in self.subs:
            sub.unregister()


def preserved(plan, monitor):
    for name,row in plan['preserved'].items():
        require(alive(row) and monitor.node(name)==row,'preserved process changed: '+name)
    if 'core_process' in plan:
        require(alive(plan['core_process']), 'full-stack core instance changed')
        require(identity(monitor.master.getPid())==plan['master_process'], 'ROS master instance changed')


def require_device_owner(plan, expected_pid=None):
    devices=plan['camera_devices']
    for path,rdev in devices.items():
        require(os.stat(path).st_rdev==rdev, 'camera device identity changed: '+path)
    query=subprocess.run(['fuser',*devices],capture_output=True,text=True,timeout=5.)
    require(query.returncode in (0,1), 'camera device-owner query failed')
    owners={int(value) for value in query.stdout.split()}
    expected=set() if expected_pid is None else {expected_pid}
    require(owners==expected, 'camera device ownership differs: '+repr(sorted(owners)))
    return sorted(owners)


def trial_configuration(plan, phase):
    expected=copy.deepcopy(plan['configuration_contract'])
    if phase!='restored':
        trial=next(row for row in plan['diagnostic_trials'] if row['label']==phase)
        if trial['preset'] is not None:
            preset=expected['runtime']['mode_depth_preset']
            preset['configured_unknown_preset']=trial['preset']
            preset['preset']=trial['preset']
    return expected


def configuration_contract(cfg):
    runtime=cfg.get('runtime_profile', {})
    return dict(configuration={k:v for k,v in cfg.items() if k!='runtime_profile'},
        runtime={k:runtime.get(k) for k in ('sdk_version','device','color_intrinsics',
            'native_depth_intrinsics','depth_to_color_extrinsics','alignment',
            'geometry_intrinsics','geometry_intrinsics_source','depth_scale_m_per_unit',
            'depth_filters','rectification','mode_depth_preset')})


def unit_pid(unit):
    result=subprocess.run(['systemctl','--user','show',unit,'--property=MainPID','--value'],
        capture_output=True,text=True,timeout=4.)
    if result.returncode or not result.stdout.strip().isdigit(): return None
    pid=int(result.stdout.strip())
    if not pid:return None
    try:return identity(pid)
    except (OSError,RuntimeError):return None


def owned_unit_camera(phase):
    intent=OUT/phase/'launch_intent.json'
    if not intent.exists():return None
    unit=json.loads(intent.read_text())['unit']
    row=unit_pid(unit)
    if row is None:return None
    argv=(Path('/proc')/str(row['pid'])/'cmdline').read_bytes().rstrip(b'\0').decode().split('\0')
    require(str(ENTRY) in argv and str(OUT/'plan.json') in argv and digest(OUT/'plan.json') in argv,
            'unit process is not this camera session')
    require(('--production' in argv)==(phase=='restored'),'unit camera phase mismatch')
    if phase!='restored':
        require('--trial' in argv and argv[argv.index('--trial')+1]==phase,
                'unit camera diagnostic trial differs')
    return row


def stop_camera(row):
    require(alive(row),'camera identity changed before stop')
    os.kill(row['pid'],signal.SIGINT)
    deadline=time.monotonic()+12.
    while alive(row) and time.monotonic()<deadline:time.sleep(.05)
    require(not alive(row),'camera has not exited; no second SDK owner started')


def stop_probe_unit(phase='probe'):
    """Cancel this exact unit, including a pending start, before restoration."""
    intent=OUT/phase/'launch_intent.json'
    if not intent.exists():return
    unit=json.loads(intent.read_text())['unit']
    def properties():
        result=subprocess.run(['systemctl','--user','show',unit,
            '--property=LoadState,ActiveState,SubState,MainPID,Job,ExecStart'],
            capture_output=True,text=True,timeout=4.)
        data=dict(line.split('=',1) for line in result.stdout.splitlines() if '=' in line)
        require(data.get('LoadState') in ('loaded','not-found'),'cannot determine probe unit state')
        return data
    state=properties()
    row=owned_unit_camera(phase)
    if state['LoadState']=='loaded':
        require(str(ENTRY) in state.get('ExecStart','') and str(OUT/'plan.json') in state['ExecStart'],
                'probe service source differs')
        result=subprocess.run(['systemctl','--user','stop',unit],capture_output=True,text=True,timeout=20.)
        require(result.returncode==0,'probe service did not stop')
    final=properties()
    require(final.get('MainPID','0')=='0' and not final.get('Job') and
        (final['LoadState']=='not-found' or final.get('ActiveState') in ('inactive','failed')),
        'probe unit still active or has a pending job')
    require(row is None or not alive(row),'probe process still alive after unit stop')
    save(OUT/phase/'unit_stopped.json',dict(unit=unit,before=state,after=final,process=row))


def require_camera_absent(monitor,known_uris):
    import rosgraph
    try:uri=monitor.master.lookupNode('/supervisor_camera')
    except rosgraph.MasterError:return
    require(uri in known_uris,'an unexpected camera registration appeared')
    # A crashed owned node may leave a stale master registration. Only its
    # previously verified URI is accepted; a master connection failure is not.
    try:
        response=xmlrpc.client.ServerProxy(uri,transport=Transport()).getPid(monitor.rospy.get_name())
    except (OSError,xmlrpc.client.ProtocolError):return
    require(False,'camera registration still responds: '+str(response[0]))


def launch(plan, production, trial=None):
    phase='restored' if production else (trial or 'probe')
    out=OUT/phase
    out.mkdir(exist_ok=True);(out/'ros').mkdir(exist_ok=True)
    unit='alicia-depth-stage-'+phase+'-'+str(time.time_ns())
    args=['systemd-run','--user','--collect','--unit='+unit,
        '--property=Type=exec','--property=Restart=no',
        '--property=KillSignal=SIGINT','--property=TimeoutStopSec=12','--property=SendSIGKILL=no',
        '--property=StandardOutput=append:'+str(out/'stdout.log'),
        '--property=StandardError=append:'+str(out/'stderr.log'),
        plan['interpreter'],'-B',str(ENTRY),'--plan',str(OUT/'plan.json'),
        '--sha256',digest(OUT/'plan.json')]
    if production:args.append('--production')
    else:args.extend(['--trial',phase])
    save(out/'launch_intent.json',dict(unit=unit,phase=phase,args=args,recorded_ns=time.time_ns()))
    result=subprocess.run(args,capture_output=True,text=True,timeout=10.)
    save(out/'launch.json',dict(unit=unit,returncode=result.returncode,stdout=result.stdout,stderr=result.stderr))
    require(result.returncode==0,'camera service did not launch')
    return unit


def validate_control(plan,monitor):
    preserved(plan,monitor)
    require({key:monitor.master.getParam('/perception/'+key) for key in
        ('depth_min_m','depth_max_m')}==plan['perception_depth_range'],
        'camera perception range dependency changed')
    now=monitor.sample();before=plan['before']
    require(now['selection']==before['selection'] and now['epoch_ns']==before['epoch_ns'],
            'control reference changed')
    require(now['names']==before['names'] and
        max(abs(a-b) for a,b in zip(now['positions'],before['positions']))<=.004,'pose changed')
    return now


def validate_restored_control(plan,monitor):
    preserved(plan,monitor)
    now=monitor.sample();before=plan['before']
    require(now['epoch_ns']>=plan['restart_evidence']['startup_started_ns'],
            'restored feedback predates the preserved driver startup')
    now['diagnostic_reference_preserved']=(now['selection']==before['selection']
        and now['epoch_ns']==before['epoch_ns'] and now['names']==before['names']
        and max(abs(a-b) for a,b in zip(now['positions'],before['positions']))<=.004)
    return now


def wait_ready(plan,monitor,phase,known_uris=None):
    deadline=time.monotonic()+28.
    last_error=None
    while time.monotonic()<deadline:
        try:
            row=owned_unit_camera(phase)
            require(row is not None and monitor.node('/supervisor_camera')==row,'camera/unit registration pending')
            startup=json.loads((OUT/phase/'source_start.json').read_text())
            require(startup['pid']==row['pid'] and startup['plan_sha256']==digest(OUT/'plan.json'),
                    'camera source receipt differs')
            if known_uris is not None:
                known_uris.add(monitor.master.lookupNode('/supervisor_camera'))
            cfg=monitor.master.getParam('/camera')
            require(configuration_contract(cfg)==trial_configuration(plan,phase),'effective camera configuration differs')
            activation=int(cfg['runtime_profile']['geometry_activation_stamp_ns'])
            now=monitor.rospy.Time.now().to_nsec()
            with monitor.lock:images=list(monitor.images)
            pairs={}
            for item in images:
                if (item['caller']=='/supervisor_camera' and item['stamp_ns']>=activation
                        and 0<=now-item['stamp_ns']<1_000_000_000):
                    pairs.setdefault(item['stamp_ns'],{})[item['kind']]=item
            complete=[pair for pair in pairs.values() if set(pair)=={'color','depth'}]
            require(complete,'fresh synchronized RGB-D pending')
            fresh=complete[-1]
            require(all(r['width']==cfg['width'] and r['height']==cfg['height']
                and r['encoding']==('bgr8' if k=='color' else '16UC1')
                for k,r in fresh.items()),'camera image layout differs')
            for topic in (cfg['color_topic'],cfg['depth_topic']):
                publishers=dict(monitor.master.getSystemState()[0]).get(topic,[])
                require(publishers==['/supervisor_camera'],'camera publisher is not unique')
            state=(validate_restored_control(plan,monitor) if phase=='restored'
                   else validate_control(plan,monitor))
            device_owners=require_device_owner(plan,row['pid'])
            result=dict(passed=True,phase=phase,new_camera=row,images=fresh,state=state,
                        camera_configuration=cfg,node_uri=monitor.master.lookupNode('/supervisor_camera'),
                        device_owners=device_owners,
                        production_source_restored=phase=='restored')
            save(OUT/phase/'registered.json',result)
            return result
        except Exception as exc:last_error=repr(exc)
        time.sleep(.1)
    raise RuntimeError('camera readiness timed out: '+str(last_error))


def current_camera(monitor):
    try:return monitor.node('/supervisor_camera')
    except Exception:return None


def prepare(monitor, binding, preset_aba=False):
    OUT.mkdir()
    accepted=binding['accepted']
    nodes={name:process_identity(row) for name,row in accepted['nodes'].items()}
    old=nodes.pop('/supervisor_camera')
    require(monitor.node('/supervisor_camera')==old,'camera identity changed')
    p=Path('/proc')/str(old['pid'])
    argv=(p/'cmdline').read_bytes().rstrip(b'\0').decode().split('\0')
    require(argv[1]==str(W/'devel/lib/alicia_flexible_grasp_supervisor/camera_node.py'),'unexpected camera entry')
    env=dict(item.decode().split('=',1) for item in (p/'environ').read_bytes().split(b'\0') if b'=' in item)
    candidate=W/'.runtime/depth_stage_probe_candidate'/VISION
    sources=[ENTRY,Path(__file__),candidate/'realsense_manager.py',candidate/'depth_stage_probe.py',
        Path(argv[1]),W/'src/alicia_flexible_grasp_supervisor/scripts/camera_node.py']
    sources+=list((W/VISION).glob('*.py'))
    cfg=monitor.master.getParam('/camera')
    sources.append(Path(cfg['mode_depth_preset']['baseline_path']))
    sdk_libraries=sorted({line.split()[-1] for line in (p/'maps').read_text().splitlines()
                          if ('pyrealsense2' in line or 'librealsense' in line)
                          and line.split()[-1].startswith('/')})
    require(sdk_libraries,'original camera SDK library source unavailable')
    sources.extend(Path(path) for path in sdk_libraries)
    # ROS-only subscribers may join or leave independently. Record them, but
    # never treat their presence as a second SDK owner or require their exit.
    readers={}
    subscribers=monitor.master.getSystemState()[1]
    for topic,names in subscribers:
        if topic in (cfg['color_topic'],cfg['depth_topic']):
            for name in names:
                if name not in nodes and name not in ('/supervisor_camera',monitor.rospy.get_name()):
                    readers[name]=monitor.node(name)
    before=monitor.sample()
    require(before['epoch_ns']>=binding['startup_started_ns'], 'current epoch predates this driver startup')
    require(before['selection']['mode']=='unknown', 'diagnostic requires current unknown task mode')
    trials=([dict(label='A1',preset='high_accuracy'),dict(label='B',preset='saved_baseline'),
             dict(label='A2',preset='high_accuracy')] if preset_aba else [dict(label='probe',preset=None)])
    core_process=identity(int(accepted['core_service']['MainPID']))
    require(core_process['pid']==unit_pid(binding['core_unit'])['pid'], 'current full-stack service changed')
    plan=dict(schema=2,kind='same_pair_online_camera_only_probe',output=str(OUT),old_camera=old,preserved=nodes,
        old_camera_uri=monitor.master.lookupNode('/supervisor_camera'),
        environment=env,cwd=os.readlink(str(p/'cwd')),interpreter=argv[0],entry=argv[1],ros_args=argv[2:],
        sources={str(s):digest(s) for s in sources},manager=str(candidate/'realsense_manager.py'),
        probe_module=str(candidate/'depth_stage_probe.py'),camera_configuration=cfg,
        configuration_contract=configuration_contract(cfg),before=before,
        perception_depth_range={key:monitor.master.getParam('/perception/'+key)
            for key in ('depth_min_m','depth_max_m')},sdk_libraries=sdk_libraries,
        restart_evidence={k:v for k,v in binding.items() if k!='accepted'},
        core_process=core_process,master_process=identity(monitor.master.getPid()),
        camera_devices=binding['camera_devices'],readonly_subscribers=readers,
        readonly_subscribers_may_coexist=True,diagnostic_trials=trials,
        probe_limits=dict(frame_limit=10,seconds=30.,warmup_sec=5.),
        required_stationary_provider='original camera_node accepted_joint_states and sdk_command subscriptions',
        motion_commands_sent=False,mode_changes_sent=False,preset_changed=preset_aba,
        camera_only=True,full_stack_restart=False,automatic_production_restore=True,
        physical_scene_stationarity='User-aligned scene; robot pose/epoch are monitored. Image equality is measured, not assumed.')
    preserved(plan,monitor)
    require_device_owner(plan,old['pid'])
    save(OUT/'plan.json',plan)
    print(json.dumps(dict(prepared=True,path=str(OUT/'plan.json'),sha256=digest(OUT/'plan.json'),
                         preserved_count=len(nodes))),flush=True)


def run_session(plan,monitor):
    trials=plan['diagnostic_trials']
    for phase in [row['label'] for row in trials]+['restored']:
        args=[plan['interpreter'],'-B',str(ENTRY),'--plan',str(OUT/'plan.json'),
              '--sha256',digest(OUT/'plan.json'),'--preflight-only']
        args += ['--production'] if phase=='restored' else ['--trial',phase]
        check=subprocess.run(args,capture_output=True,text=True,timeout=20.)
        save(OUT/(phase+'_preflight.json'),dict(returncode=check.returncode,stdout=check.stdout,stderr=check.stderr))
        require(check.returncode==0,'camera preflight failed; original camera kept running')
    before=validate_control(plan,monitor)
    require(configuration_contract(monitor.master.getParam('/camera'))==plan['configuration_contract'],
            'camera configuration changed before stop')
    require(current_camera(monitor)==plan['old_camera'],'camera owner changed before stop')
    require_device_owner(plan,plan['old_camera']['pid'])
    save(OUT/'before_stop.json',dict(camera=plan['old_camera'],state=before))
    stopped=False;error=None;restoration=None;active_phase=None
    known_uris={plan['old_camera_uri']};results=[]
    try:
        stopped=True
        stop_camera(plan['old_camera'])
        for trial in trials:
            phase=trial['label'];active_phase=phase
            require_camera_absent(monitor,known_uris)
            require_device_owner(plan)
            validate_control(plan,monitor)
            launch(plan,False,phase)
            ready=wait_ready(plan,monitor,phase,known_uris)
            known_uris.add(ready['node_uri'])
            print(json.dumps(dict(probe_ready=True,trial=phase,pid=ready['new_camera']['pid'])),flush=True)
            deadline=time.monotonic()+35.
            summary=OUT/phase/'capture/summary.json'
            completion=OUT/phase/'probe_completion.json'
            while (not summary.exists() or not completion.exists()) and time.monotonic()<deadline:
                validate_control(plan,monitor)
                require(owned_unit_camera(phase)==ready['new_camera'],'probe camera exited')
                time.sleep(.15)
            require(summary.exists() and completion.exists(),'finite probe did not finish')
            result=json.loads(summary.read_text());done=json.loads(completion.read_text())
            counts=result['counts']
            require(result['error'] is None and done.get('error') is None and
                all(counts[key]==10 for key in ('selected','completed','written')) and
                counts['queue_dropped']==0 and counts['interrupted_pairs']==0,
                'finite probe did not save all 10 complete pairs')
            pairs=[]
            for entry in result['files']:
                path=summary.parent/entry['name']
                require(path.stat().st_size==entry['bytes'] and digest(path)==entry['sha256'],
                        'saved capture file changed: '+entry['name'])
                if path.suffix=='.json':
                    pair=json.loads(path.read_text())
                    require(pair.get('complete') is True,'capture pair is incomplete: '+entry['name'])
                    pairs.append(pair)
            require(len(pairs)==10,'saved pair metadata count differs')
            times=sorted(pair['selected_monotonic'] for pair in pairs)
            require(all(b>a for a,b in zip(times,times[1:])), 'selected pair times are not strictly increasing')
            diagnostics=monitor.master.getParam('/camera_read_diagnostics')
            with monitor.lock:images=list(monitor.images)
            observed=dict(images=images,read_diagnostics=diagnostics,state=validate_control(plan,monitor),
                summary_sha256=digest(summary),completion_sha256=digest(completion),
                selected_intervals_sec=[b-a for a,b in zip(times,times[1:])],
                counts=counts,unselected_backpressure_reads=done['unselected_backpressure_reads'])
            save(OUT/phase/'observed_publications.json',observed)
            results.append(dict(trial=phase,passed=True,counts=counts,
                summary_sha256=digest(summary),selected_intervals_sec=observed['selected_intervals_sec']))
            stop_probe_unit(phase)
            require_camera_absent(monitor,known_uris)
            require_device_owner(plan)
            active_phase=None
    except BaseException as exc:
        error=repr(exc)
        save(OUT/'probe_failure.json',dict(error=error,trial=active_phase,recorded_ns=time.time_ns()))
    finally:
        if stopped:
            try:
                if active_phase is not None:
                    row=owned_unit_camera(active_phase)
                    if row is not None and current_camera(monitor)==row:
                        known_uris.add(monitor.master.lookupNode('/supervisor_camera'))
                    stop_probe_unit(active_phase)
                deadline=time.monotonic()+12.
                while alive(plan['old_camera']) and time.monotonic()<deadline:time.sleep(.05)
                require(not alive(plan['old_camera']) and
                    all(owned_unit_camera(row['label']) is None for row in trials),'existing SDK owner still alive')
                require_camera_absent(monitor,known_uris)
                require_device_owner(plan)
                # Restore production even when the diagnostic pose/epoch check
                # failed. Never grant motion or auto-mode permission here.
                preserved(plan,monitor)
                launch(plan,True)
                restoration=wait_ready(plan,monitor,'restored')
                print(json.dumps(dict(production_restored=True,pid=restoration['new_camera']['pid'])),flush=True)
            except BaseException as exc:
                save(OUT/'restoration_failure.json',dict(error=repr(exc),recorded_ns=time.time_ns(),
                    active_trial=active_phase,original_camera=plan['old_camera'],
                    restored_unit_intent=str(OUT/'restored/launch_intent.json'),
                    root_takeover_required=True,full_stack_restart_requested=False))
                raise
    save(OUT/'result.json',dict(passed=error is None,probe_error=error,trials=results,
        production_restored=restoration is not None,restoration=restoration,preserved=plan['preserved'],
        motion_commands_sent=False,mode_changes_sent=False,temperature_queried=False,
        presets_tested=trials,production_configuration_restored=restoration is not None,
        control_epoch_preserved=(restoration['state']['diagnostic_reference_preserved']
            if restoration else False),scene_permission=False,full_stack_restart=False))
    require(error is None,'probe failed; production camera was restored: '+str(error))


def main():
    global OUT
    parser=argparse.ArgumentParser()
    parser.add_argument('operation',choices=['prepare','run','inspect'])
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--restart-contract',type=Path)
    parser.add_argument('--restart-sha256')
    parser.add_argument('--acceptance',type=Path)
    parser.add_argument('--acceptance-sha256')
    parser.add_argument('--preset-aba',action='store_true')
    parser.add_argument('--plan-sha256')
    args=parser.parse_args()
    require(os.path.ismount('/media/zhuyupei/SanStand'),'SanStand unavailable')
    OUT=args.output.resolve()
    require(E/'ros_log' in OUT.parents,'external diagnostic output required')
    binding=None
    if args.operation=='prepare':
        require(all((args.restart_contract,args.restart_sha256,args.acceptance,args.acceptance_sha256)),
                'current restart contract and passing acceptance references required')
        binding=restart_binding(dict(path=str(args.restart_contract.resolve()),sha256=args.restart_sha256),
            dict(path=str(args.acceptance.resolve()),sha256=args.acceptance_sha256))
    else:
        require(args.plan_sha256 and digest(OUT/'plan.json')==args.plan_sha256,'diagnostic plan digest differs')
    observer=OUT.parent/(OUT.name+'_controller_'+args.operation+'_'+str(time.time_ns()))
    observer.mkdir();os.environ['ROS_LOG_DIR']=str(observer/'ros')
    monitor=Monitor()
    try:
        if args.operation=='prepare':prepare(monitor,binding,args.preset_aba);return
        plan=json.loads((OUT/'plan.json').read_text())
        for path,sha in plan['sources'].items():require(digest(path)==sha,'source changed: '+path)
        if args.operation=='inspect':
            print(json.dumps(dict(state=validate_control(plan,monitor),camera=current_camera(monitor)),ensure_ascii=False))
        else:run_session(plan,monitor)
    finally:monitor.close()


if __name__=='__main__':
    def interrupted(_signal,_frame):
        raise KeyboardInterrupt('camera diagnostic controller interrupted')
    signal.signal(signal.SIGTERM,interrupted)
    main()
