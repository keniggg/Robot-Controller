"""Finish the retired, never-restarted gen5 trial after first-status timing failure.

Only the camera setting and visual services are touched. Current user robot
commands, core owners and the already retired selection must remain unchanged.
"""
import importlib.util,json,os,signal,subprocess,threading,time
from collections import OrderedDict
from datetime import datetime,timezone
from pathlib import Path

W=Path(__file__).resolve().parents[1]
R=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/saved_baseline_trial_20261005T033848.688318Z')

def load(name,path):
    spec=importlib.util.spec_from_file_location(name,path)
    m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m);return m

def main():
    import rospy
    from sensor_msgs.msg import Image,JointState
    from std_msgs.msg import Bool,Header
    from alicia_flexible_grasp_supervisor.msg import GraspState
    h=load('process_helper',W/'.runtime/gen4_root_cause/reload_camera_visual_gate.py')
    e=load('visual_entry',W/'.runtime/run_matched_shadow_current_20261005.py')
    loader=load('camera_helper',W/e.LOADER)
    Runtime=e.make_runtime_class(loader,h,None)
    old=json.loads((R/'result.json').read_text())
    assert old['error']=="RuntimeError('original stationary temporal policy changed')"
    assert not old['trial_loaded'] and not old['visual_nodes_restored']
    starts=[x for x in old['events'] if x['event']=='start_requested']
    assert all(x['role']=='camera' for x in starts), 'a consumer already saw this generation'
    selection=old['maintenance_retirement']['new_selection']
    assert old['maintenance_retirement']['reset_acknowledgements_complete'] and selection['generation']==5
    parent=Path(next(p for p in old['input_files'] if p.endswith('/consumer_recovery.json'))).parent
    context=json.loads((parent/'result.json').read_text())['context']
    assert Path('/media/zhuyupei/SanStand').is_mount()
    out=R/('continuation_'+datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ'));out.mkdir();(out/'ros').mkdir()
    os.environ['ROS_LOG_DIR']=str(out/'ros')
    report=dict(trial_loaded=False,visual_nodes_restored=False,selection=selection,events=[],
        prior_failed_trial=str(R/'result.json'),prior_sha256=e.sha(R/'result.json'),
        motion_commands_sent=False,torque_commands_sent=False,temperature_queries_sent=False)
    def save():(out/'result.json').write_text(json.dumps(report,indent=2)+'\n')
    rospy.init_node('continue_unacquired_visual_trial',anonymous=True,disable_signals=True)
    latest={};pairs=OrderedDict();lock=threading.RLock()
    def put(k,v):
        with lock:latest[k]=v
    def image(m,k):
        with lock:
            pairs.setdefault(m.header.stamp.to_nsec(),{})[k]=dict(width=m.width,height=m.height,
                encoding=m.encoding,step=m.step,bytes=len(m.data),frame_id=m.header.frame_id,
                caller=(m._connection_header or {}).get('callerid'))
            while len(pairs)>30:pairs.popitem(last=False)
    cfg=rospy.get_param('/camera')
    prior_rejection=R/'continuation_20261005T034456Z/result.json'
    resume=cfg['mode_depth_preset']['unknown_preset']=='saved_baseline'
    if resume:
        rejected=json.loads(prior_rejection.read_text())
        assert rejected.get('preset_written')=='saved_baseline' and rejected.get('error','').startswith('CalledProcessError(1,')
        assert not rejected['trial_loaded'] and all(v['role']=='camera' for v in rejected['events'])
        report['resume_rejected_environment']=dict(path=str(prior_rejection),sha256=e.sha(prior_rejection))
    else:assert cfg['mode_depth_preset']['unknown_preset']=='high_accuracy'
    subs=[rospy.Subscriber('/alicia_d/motion_enabled',Bool,lambda m:put('enabled',m.data)),
        rospy.Subscriber('/grasp/state',GraspState,lambda m:put('active',m.active)),
        rospy.Subscriber('/alicia_d/control_reference_epoch',Header,lambda m:put('epoch',m.stamp.to_nsec())),
        rospy.Subscriber('/alicia_d/sdk_command',JointState,lambda m:put('sdk',(dict(zip(m.name,m.position)),time.monotonic())))]
    for k,key in [('rgb','color_topic'),('depth','depth_topic')]:
        subs.append(rospy.Subscriber(cfg[key],Image,lambda m,s=k:image(m,s),queue_size=3,buff_size=2**24))
    def guard():
        assert Path('/media/zhuyupei/SanStand').is_mount()
        assert rospy.get_param('/grasp_mode/selection')==selection
        assert rospy.get_param('/handeye')==context['/handeye']
        assert rospy.get_param('/rgb_acquisition/ledger',None)==context['/rgb_acquisition/ledger']
        for name,identity in old['preserved'].items():
            assert h.alive(identity) and h.node_pid(rospy,name)==identity['pid'],name
        assert latest.get('enabled') is True and latest.get('active') is False
        assert latest.get('epoch')==old['epoch_ns']
        assert latest['sdk'][0]==old['sdk_hold'] and time.monotonic()-latest['sdk'][1]<1.
    def registered(name):
        try:return h.node_pid(rospy,name)
        except (h.CameraRegistrationPending,ConnectionRefusedError):return None
    def ensure_consumers_absent():
        for role in ('unknown','remote'):
            assert not h.alive(old['before'][role]) and registered(e.NODES[role]) is None
    h.wait(lambda:all(k in latest for k in ('enabled','active','epoch','sdk')),5,'current idle feedback')
    guard();ensure_consumers_absent()
    event=starts[-1];cmd=event['command']
    if resume:
        camera=rejected['before_camera'];assert not h.alive(camera) and registered('/supervisor_camera') is None
    else:
        props=Runtime.unit_properties(event['unit']);camera=h.identity(int(props['MainPID']))
        assert camera['argv']==cmd[cmd.index('/usr/bin/python3'):] and registered('/supervisor_camera')==camera['pid']
    manifest=json.loads((W/'.runtime/saved_baseline_trial_validation_20261005.json').read_text())
    for name,digest in manifest['files'].items():
        if name not in ('.runtime/load_saved_baseline_trial_20261005.py','.runtime/test_load_saved_baseline_trial_20261005.py'):
            assert e.sha(W/name)==digest,name
    env={v[len('--setenv='):].split('=',1)[0]:v[len('--setenv='):].split('=',1)[1]
         for v in cmd if v.startswith('--setenv=')}
    report.update(before_camera=camera,baseline_before=cfg,core_preserved=old['preserved']);save()
    def start(role):
        guard()
        assert registered(e.NODES[role]) is None
        unit='alicia-baseline-final-'+role+'-'+datetime.now(timezone.utc).strftime('%Y%m%dt%H%M%S%f')
        command=e.build_command(role,old['normal_entries'][role],env,out,unit,
            tuple(key for key in env if key not in ('ROS_LOG_DIR','PYTHONUNBUFFERED')))
        report['events'].append(dict(event='start_requested',role=role,unit=unit,command=command));save()
        subprocess.run(command,check=True,timeout=12)
        def ready():
            pid=int(Runtime.unit_properties(unit).get('MainPID','0'))
            if not pid:return False
            identity=h.identity(pid)
            assert identity['argv']==command[command.index('/usr/bin/python3'):]
            actual=registered(e.NODES[role])
            if actual is None:return False
            assert actual==pid
            return identity
        identity=h.wait(ready,25,role+' registration',guard)
        report.setdefault('owners',{})[role]=identity;save();return identity
    try:
        guard();ensure_consumers_absent()
        if not resume:
            os.kill(camera['pid'],signal.SIGINT)
            h.wait(lambda:not h.alive(camera),15,'old camera exit',guard)
        ensure_consumers_absent();rospy.set_param('/camera/mode_depth_preset/unknown_preset','saved_baseline')
        assert rospy.get_param('/camera/mode_depth_preset/unknown_preset')=='saved_baseline'
        report['preset_written']='saved_baseline';save();new_camera=start('camera')
        cutoff=rospy.Time.now().to_nsec();began=time.monotonic();seen=set();active=[]
        def warm():
            c=rospy.get_param('/camera');profile=c.get('runtime_profile',{})
            policy=profile.get('mode_depth_preset',{})
            if policy.get('configured_unknown_preset')!='saved_baseline':return False
            assert policy.get('mode')=='unknown' and policy.get('preset')=='saved_baseline'
            assert policy['baseline_sha256']==cfg['runtime_profile']['mode_depth_preset']['baseline_sha256']
            assert all(c[k]==cfg[k] for k in ('width','height','fx','fy','cx','cy','depth_scale','frame_id'))
            assert profile['rectification']==cfg['runtime_profile']['rectification']
            status=profile.get('stationary_temporal')
            if status is None:return False
            assert status.get('enabled') is True and status.get('alpha')==.4 and status.get('delta_depth_units')==100 and status.get('persistence')==0
            now=rospy.Time.now().to_nsec()
            with lock:seen.update(ns for ns,p in pairs.items() if e.complete_pair_is_valid(ns,p,c,cutoff,now))
            if status.get('active') is True:active.append(now)
            if time.monotonic()-began<3 or len(seen)<10 or not active:return False
            report.update(camera_after=c,fresh_pairs=len(seen),stationary_active_diagnostic=dict(observed_ns=active[0],per_frame_binding=False))
            return True
        h.wait(warm,20,'first actual temporal status and fresh pairs',guard)
        start('unknown');start('remote');guard()
        report.update(trial_loaded=True,visual_nodes_restored=True);save()
        print(json.dumps(dict(output=str(out),trial_loaded=True)),flush=True)
    except BaseException as ex:
        report['error']=repr(ex);save();raise
    finally:
        for sub in subs:sub.unregister()

if __name__=='__main__':main()
