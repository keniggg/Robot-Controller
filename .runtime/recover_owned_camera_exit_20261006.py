"""Recover one positively identified camera after its recorded SIGINT timeout."""
import hashlib,importlib.util,json,os,signal,subprocess
from pathlib import Path
W=Path(__file__).resolve().parents[1]
FAIL=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/regional_production_20261006T122301.075575Z/result.json')
failed_bytes=FAIL.read_bytes()
failed_sha=hashlib.sha256(failed_bytes).hexdigest()
failed=json.loads(failed_bytes)
assert failed['camera_loaded'] is False and failed['error']=="RuntimeError('timeout: owned camera exit')"
assert failed['old_camera_sigint_pid']==failed['old_camera']['pid']==144048
spec=importlib.util.spec_from_file_location('owned_camera_recovery_loader',W/'.runtime/load_regional_production_20261006.py')
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
original_factory=m.runtime_class

def factory(rospy,h,ab,alignment):
    original_node_pid=h.node_pid
    previous=FAIL.parent.parent/'regional_production_20261006T112923.272799Z/result.json'
    previous_bytes=previous.read_bytes();previous_report=json.loads(previous_bytes)
    assert previous_report['new_camera']==failed['old_camera']
    def owned_node_pid(ros,node):
        try:
            return original_node_pid(ros,node)
        except (ConnectionRefusedError,h.CameraRegistrationPending):
            # rospy may close its slave socket before the SDK stop hangs.
            # Do not reinterpret arbitrary registration failures as ownership.
            if node!=m.CAMERA or not h.alive(failed['old_camera']):
                raise
            assert h.identity(failed['old_camera']['pid'])==failed['old_camera']
            main_pid=int(subprocess.check_output(['systemctl','--user','show',previous_report['systemd_user_unit'],'--property=MainPID','--value'],text=True,timeout=3))
            assert main_pid==failed['old_camera']['pid']
            owners=[]
            for path in Path('/proc').glob('[0-9]*/cmdline'):
                try:argv=path.read_bytes().decode().strip('\0').split('\0')
                except (FileNotFoundError,PermissionError,ProcessLookupError):continue
                if '__name:=supervisor_camera' in argv:owners.append(int(path.parent.name))
            assert owners==[main_pid],'camera process ownership is ambiguous'
            return main_pid
    h.node_pid=owned_node_pid
    Base=original_factory(rospy,h,ab,alignment)
    class Recovery(Base):
        def stop_camera(self):
            self.guard(force=True)
            assert self.old==failed['old_camera'],'camera identity changed after recorded timeout'
            assert h.alive(self.old) and h.node_pid(rospy,m.CAMERA)==self.old['pid']
            assert h.stable_camera_configuration(self.before)==h.stable_camera_configuration(failed['camera_parameters_before'])
            assert self.core==failed['core_preserved'],'a core owner changed'
            self.report['owned_camera_exit_recovery']=dict(prior_failure=str(FAIL),prior_failure_sha256=failed_sha,prior_sigint_timeout_seconds=12,signal='SIGKILL',pid=self.old['pid'],start_ticks=self.old['start_ticks'],scope='only the unchanged camera process that failed its recorded orderly shutdown',script_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
            self.report['owned_camera_exit_recovery']['prior_systemd_camera_result']=str(previous)
            self.report['owned_camera_exit_recovery']['prior_systemd_camera_result_sha256']=hashlib.sha256(previous_bytes).hexdigest()
            self.save('verified_owned_camera_exit_recovery')
            self.guard(force=True)
            assert hashlib.sha256(FAIL.read_bytes()).hexdigest()==failed_sha,'prior failure changed'
            assert h.alive(self.old) and h.node_pid(rospy,m.CAMERA)==self.old['pid']
            os.kill(self.old['pid'],signal.SIGKILL)
            h.wait(lambda:not h.alive(self.old),5.,'owned hung camera exit',self.guard)
            self.save('old_camera_exited')
    return Recovery
m.runtime_class=factory
m.main()
