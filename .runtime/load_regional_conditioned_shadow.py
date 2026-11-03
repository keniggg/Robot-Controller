"""Camera-only bounded conditional shadow trial; never starts a robot task or changes presets."""
from datetime import datetime, timezone
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import signal
import subprocess

W=Path(__file__).resolve().parents[1]
M=Path('/media/zhuyupei/SanStand')
EXT=M/'实验记录/Robot-Controller'
PREVIOUS=EXT/'ros_log/regional_shadow_20261005T050101Z/result.json'

def load(name,path):
    spec=importlib.util.spec_from_file_location(name,W/path)
    mod=importlib.util.module_from_spec(spec);spec.loader.exec_module(mod);return mod

def digest(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()

def completed_json(path):
    try:return json.loads(Path(path).read_text())
    except (FileNotFoundError,json.JSONDecodeError):return False

def main():
    import argparse
    parser=argparse.ArgumentParser();parser.add_argument('--execute',action='store_true')
    if not parser.parse_args().execute:raise RuntimeError('--execute required')
    assert M.is_mount(),'SanStand unavailable; no fallback'
    import rospy
    h=load('shadow_owner','.runtime/gen4_root_cause/reload_camera_visual_gate.py')
    ab=load('shadow_evidence','.runtime/camera_preset_ab_observer.py')
    manifest=json.loads((W/'.runtime/regional_conditioned_validation_20261005.json').read_text())
    assert manifest['tests_passed'] and manifest['reviewed']
    def verify():
        for name,sha in manifest['files'].items():assert digest(W/name)==sha,name
    verify()
    stamp=datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')
    out=EXT/'ros_log'/('regional_conditioned_shadow_'+stamp);out.mkdir();(out/'ros').mkdir()
    os.environ['ROS_LOG_DIR']=str(out/'ros')
    rospy.init_node('regional_conditioned_shadow_loader',anonymous=True,disable_signals=True)
    ev=ab.RosEvidence(rospy)
    report=dict(out=str(out),loaded=False,completed=False,robot_commands_sent=False,
        preset_changed=False,regional_production_enabled=False,validation_manifest=manifest)
    try:
        old=json.loads(PREVIOUS.read_text())['new_camera']
        assert h.alive(old) and h.node_pid(rospy,h.NODE)==old['pid']
        core={n:h.identity(h.node_pid(rospy,n)) for n in (
            '/alicia_d_driver_node','/unknown_tabletop_perception','/remote_grasp6d_node')}
        cfg=rospy.get_param('/camera')
        assert cfg['mode_depth_preset']['unknown_preset']=='saved_baseline'
        assert cfg['mode_depth_preset'].get('regional_stationary_enabled',False) is False
        assert cfg['fps']==30 and cfg['publish_fps']==20
        h.wait(lambda:ev.guard.check(ev.now_ns(),freeze=True),5,'current unchanged feedback')
        def guard():
            assert M.is_mount()
            ev.check_context()
            assert ev.guard.check(ev.now_ns(),freeze=True)
            assert h.stable_camera_configuration(rospy.get_param('/camera'))==h.stable_camera_configuration(cfg)
            for name,owner in core.items():assert h.alive(owner) and h.node_pid(rospy,name)==owner['pid'],name
        guard();verify()
        old_argv=old['argv']
        assert Path(old_argv[1]).resolve()==(W/'.runtime/regional_stationary_shadow.py').resolve()
        assert old_argv[2]=='--camera-script' and old_argv[4]=='--output' and old_argv[6]=='--'
        entry,tail=h.camera_entry_and_ros_tail([old_argv[0],old_argv[3]]+old_argv[7:])
        tail=[a for a in tail if not a.startswith('__log:=')]+['__log:='+str(out/'camera.ros.log')]
        env=h.camera_environment(old['pid'])
        unit='alicia-regional-conditioned-'+stamp.lower()
        command=['systemd-run','--user','--collect','--unit='+unit,'--property=Type=exec',
            '--property=Restart=no','--property=WorkingDirectory='+old['cwd'],
            '--property=StandardOutput=append:'+str(out/'camera.stdout.log'),
            '--property=StandardError=append:'+str(out/'camera.stderr.log')]
        command+=['--setenv='+k+'='+v for k,v in env.items()]
        command+=['--setenv=ROS_LOG_DIR='+str(out/'ros'),'--setenv=PYTHONUNBUFFERED=1',
            old['argv'][0],str(W/'.runtime/regional_conditioned_shadow.py'),
            '--camera-script',entry,'--output',str(out/'capture'),'--']+tail
        report.update(old_camera=old,core_preserved=core,camera_parameters_before=cfg,
            initial_feedback=ev.guard.snapshot(),command=command,unit=unit)
        (out/'result.json').write_text(json.dumps(report,indent=2)+'\n')
        guard();os.kill(old['pid'],signal.SIGINT)
        h.wait(lambda:not h.alive(old),12,'old camera exit',guard)
        start=subprocess.run(command,check=True,capture_output=True,text=True,timeout=10)
        (out/'systemd_start.txt').write_text(start.stdout+start.stderr)
        def new_owner():
            pid=int(subprocess.check_output(['systemctl','--user','show',unit,'--property=MainPID','--value'],text=True,timeout=3).strip() or 0)
            if not pid:return False
            try:
                if h.node_pid(rospy,h.NODE)!=pid:return False
            except (h.CameraRegistrationPending,ConnectionRefusedError):return False
            owner=h.identity(pid)
            assert owner['argv']==command[command.index(old['argv'][0]):]
            return owner
        report['new_camera']=h.wait(new_owner,20,'new camera registration',guard)
        report['loaded']=True
        summary=out/'capture/summary.json'
        report['capture_summary']=h.wait(lambda:completed_json(summary),45,'bounded shadow capture',guard)
        guard();verify()
        report['completed']=report['capture_summary']['capture_complete']
        report['camera_parameters_after']=rospy.get_param('/camera')
        report['final_feedback']=ev.guard.snapshot()
        report['upstream_stall_resolution_verified']=False
    except BaseException as exc:
        report['error']=repr(exc)
        raise
    finally:
        ev.close()
        (out/'result.json').write_text(json.dumps(report,indent=2)+'\n')
        print(json.dumps({k:report[k] for k in ('out','loaded','completed','error') if k in report}),flush=True)

if __name__=='__main__':main()
