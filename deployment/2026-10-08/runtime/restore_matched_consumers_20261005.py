"""Resume only quiesced visual consumers after completed camera restoration."""
from collections import OrderedDict
from datetime import datetime, timezone
import importlib.util
import json
import os
from pathlib import Path
import subprocess
import threading
import time
from types import SimpleNamespace

W=Path(__file__).resolve().parents[1]
OUT=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/matched_shadow_current_20261005T024652.714830Z')

def load(name,path):
    spec=importlib.util.spec_from_file_location(name,path)
    m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m);return m

def main():
    import rospy
    from sensor_msgs.msg import Image,JointState
    from std_msgs.msg import Bool,Header
    from alicia_flexible_grasp_supervisor.msg import GraspState
    h=load('identity_helper',W/'.runtime/gen4_root_cause/reload_camera_visual_gate.py')
    loader=load('pair_helper',W/'.runtime/load_pair_rectification_20261004.py')
    entry=load('entry_helpers',W/'.runtime/run_matched_shadow_current_20261005.py')
    d=json.loads((OUT/'result.json').read_text())
    summary=json.loads((OUT/'stages/summary.json').read_text())
    assert summary['restoration']['succeeded'] and summary['original_read_restored'] and summary['original_sync_restored']
    assert d['maintenance_retirement']['reset_acknowledgements_complete']
    selection=d['maintenance_retirement']['new_selection']
    event=[x for x in d['events'] if x['event']=='start_requested' and x['role']=='camera'][0]
    command=event['command']; camera_pid=38860
    camera=h.identity(camera_pid)
    assert camera['argv']==command[command.index('/usr/bin/python3'):]
    manifest=json.loads((OUT/'matched_shadow_reviewed_20261005.json').read_text())
    # The lifecycle entry's race fix is not deployed to camera/consumers.
    for name,expected in manifest['files'].items():
        if name not in (entry.SELF,entry.TEST,'.runtime/analyze_matched_shadow_capture_20261005.py',
                        '.runtime/test_analyze_matched_shadow_capture_20261005.py'):
            assert entry.sha(W/name)==expected,name
    assert Path('/media/zhuyupei/SanStand').is_mount()
    os.environ['ROS_LOG_DIR']=str(OUT/'ros')
    rospy.init_node('restore_matched_visual_consumers',anonymous=True,disable_signals=True)
    latest={};pairs=OrderedDict();lock=threading.RLock()
    result=dict(started_utc=datetime.now(timezone.utc).isoformat(),motion_commands_sent=False,
        torque_commands_sent=False,temperature_queries_sent=False,selection=selection,
        original_lifecycle_failed=True,visual_nodes_restored=False,consumers={})
    def save():(OUT/'consumer_recovery.json').write_text(json.dumps(result,indent=2)+'\n')
    def put(k,v):
        with lock:latest[k]=v
    def image(msg,kind):
        with lock:
            pairs.setdefault(msg.header.stamp.to_nsec(),{})[kind]=dict(width=msg.width,height=msg.height,
                encoding=msg.encoding,step=msg.step,bytes=len(msg.data),frame_id=msg.header.frame_id,
                caller=(msg._connection_header or {}).get('callerid'))
            while len(pairs)>30:pairs.popitem(last=False)
    cfg=rospy.get_param('/camera')
    subscribers=[rospy.Subscriber('/alicia_d/motion_enabled',Bool,lambda m:put('enabled',m.data)),
        rospy.Subscriber('/grasp/state',GraspState,lambda m:put('active',m.active)),
        rospy.Subscriber('/alicia_d/control_reference_epoch',Header,lambda m:put('epoch',m.stamp.to_nsec())),
        rospy.Subscriber('/alicia_d/sdk_command',JointState,lambda m:put('sdk',(dict(zip(m.name,m.position)),time.monotonic())))]
    for kind,key in [('rgb','color_topic'),('depth','depth_topic')]:
        subscribers.append(rospy.Subscriber(cfg[key],Image,lambda m,k=kind:image(m,k),queue_size=3,buff_size=2**24))
    def guard():
        assert Path('/media/zhuyupei/SanStand').is_mount()
        assert rospy.get_param('/grasp_mode/selection')==selection
        assert rospy.get_param('/handeye')==d['context']['/handeye']
        assert rospy.get_param('/rgb_acquisition/ledger',None)==d['context']['/rgb_acquisition/ledger']
        for name,identity in d['preserved'].items():
            assert h.alive(identity) and h.node_pid(rospy,name)==identity['pid'],name
        assert h.alive(camera) and h.node_pid(rospy,'/supervisor_camera')==camera_pid
        assert latest.get('enabled') is True and latest.get('active') is False
        assert latest.get('epoch')==d['epoch_ns']
        assert time.monotonic()-latest['sdk'][1]<1.
        assert latest['sdk'][0]==d['sdk_hold'],(latest['sdk'][0],d['sdk_hold'])
    h.wait(lambda:all(k in latest for k in ('enabled','active','epoch','sdk')),5,'passive current state')
    guard();loader.unchanged_camera_settings(d['camera_before'],cfg)
    assert cfg['runtime_profile']['mode_depth_preset']['preset']=='high_accuracy'
    assert cfg['runtime_profile']['geometry_activation_stamp_ns']!=d['activation_before']
    cutoff=rospy.Time.now().to_nsec();started=time.monotonic();seen=set()
    def warm():
        now=rospy.Time.now().to_nsec()
        with lock:seen.update(ns for ns,p in pairs.items() if entry.complete_pair_is_valid(ns,p,cfg,cutoff,now))
        return time.monotonic()-started>=3 and len(seen)>=10
    h.wait(warm,15,'restored fresh pairs',guard)
    result['fresh_pairs']=len(seen);result['camera_after']=cfg;save()
    allowed=tuple(dict.fromkeys(h.ENV_KEYS+loader.EXTRA_ENV+('ROS_HOME',)))
    for role in ('unknown','remote'):
        old=d['before'][role]
        assert not h.alive(old)
        try:pid=h.node_pid(rospy,entry.NODES[role])
        except (h.CameraRegistrationPending,ConnectionRefusedError):pid=None
        assert pid is None,('unexpected consumer',role,pid)
        parent=d['owner_parent_bindings'][role]['parent']
        # Both original visual consumers inherit the same reviewed roslaunch
        # environment. The camera command recorded those environment values.
        env={s[len('--setenv='):].split('=',1)[0]:s[len('--setenv='):].split('=',1)[1]
             for s in command if s.startswith('--setenv=')}
        for key in ('ALICIA_EXPERIMENT_STORAGE_BASE','ALICIA_SESSION_LOG_DIR','GRASP_RECORD_ROOT'):
            assert str(entry.EXT) in env[key]
        unit='alicia-restored-'+role+'-'+datetime.now(timezone.utc).strftime('%Y%m%dt%H%M%S%f')
        cmd=entry.build_command(role,old,env,OUT,unit,allowed)
        result['consumers'][role]=dict(unit=unit,command=cmd);save();guard()
        subprocess.run(cmd,check=True,timeout=12)
        def ready():
            props=entry.make_runtime_class(loader,h,None).unit_properties(unit)
            value=int(props.get('MainPID','0'))
            if not value:return False
            identity=h.identity(value)
            assert identity['argv']==cmd[cmd.index('/usr/bin/python3'):]
            try:registered=h.node_pid(rospy,entry.NODES[role])
            except (h.CameraRegistrationPending,ConnectionRefusedError):return False
            assert registered==value
            return identity
        result['consumers'][role]['identity']=h.wait(ready,25,role+' restored',guard);save()
    guard();result['visual_nodes_restored']=True;result['preserved_nodes_unchanged']=True;save()
    print(json.dumps(dict(report=str(OUT/'consumer_recovery.json'),restored=True)))
    for sub in subscribers:sub.unregister()

if __name__=='__main__':main()
