#!/usr/bin/env python3
"""Prepare an explicit, reversible unknown-mode saved-baseline visual trial.

Default invocation only describes the manifest. --execute changes one camera
parameter and restarts only the three visual owners. It retires the old target
generation first; it never claims alignment, plans a motion, or starts a grasp.
"""
import argparse
from copy import deepcopy
from datetime import datetime, timezone
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import subprocess
import threading
import time
from collections import OrderedDict

WORK=Path(__file__).resolve().parents[1]
EXT=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
DEFAULT_RECORD=EXT/'ros_log/matched_shadow_current_20261005T024652.714830Z'
RUNNER='.runtime/run_matched_shadow_current_20261005.py'
SELF='.runtime/load_saved_baseline_trial_20261005.py'
TEST='.runtime/test_load_saved_baseline_trial_20261005.py'
PARAM='/camera/mode_depth_preset/unknown_preset'
PRESET='saved_baseline'


def load(name,path):
    spec=importlib.util.spec_from_file_location(name,path)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module);return module


def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def verify_manifest(manifest,runner,root=WORK):
    required=set(runner.REQUIRED)|{SELF,TEST,
        'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/vision/depth_preset.py',
        'src/alicia_flexible_grasp_supervisor/tests/test_mode_depth_preset.py',
        'src/alicia_flexible_grasp_supervisor/tests/test_stationary_depth.py',
        '.runtime/d405_carton_depth_baseline.json'}
    if (manifest.get('schema')!=1 or manifest.get('scope')!='saved_baseline_unknown_trial'
            or manifest.get('reviewed') is not True or manifest.get('tests_passed') is not True
            or not manifest.get('validation_results') or not required<=manifest.get('files',{}).keys()):
        raise RuntimeError('new reviewed saved-baseline trial manifest required')
    for name,digest in manifest['files'].items():
        rel=Path(name);path=(root/rel).resolve()
        if rel.is_absolute() or '..' in rel.parts or root.resolve() not in path.parents or str(rel)!=name:
            raise RuntimeError('noncanonical source manifest path')
        if sha(path)!=digest:raise RuntimeError('reviewed source changed: '+name)


def completed_wrapper_command(argv,report,recovery,summary,runner,record):
    """Unwrap exactly the completed, restored capture named by these receipts."""
    if (recovery.get('visual_nodes_restored') is not True
            or recovery.get('preserved_nodes_unchanged') is not True
            or summary.get('restoration',{}).get('succeeded') is not True
            or summary.get('original_read_restored') is not True
            or summary.get('original_sync_restored') is not True):
        raise RuntimeError('completed wrapper restoration receipts are missing')
    events=[event for event in report.get('events',[]) if event.get('event')=='start_requested'
            and event.get('role')=='camera']
    if len(events)!=1:raise RuntimeError('camera wrapper launch receipt is ambiguous')
    command=events[0]['command'];expected=command[command.index('/usr/bin/python3'):]
    if argv!=expected or len(argv)<14 or argv[1]!=str(WORK/runner.WRAPPER):
        raise RuntimeError('current camera is not the recorded completed wrapper')
    if argv.count('--')!=1:raise RuntimeError('wrapper delimiter is ambiguous')
    split=argv.index('--');options=argv[2:split]
    camera_entry=str(WORK/runner.RELAY['camera'])
    if options!=['--enable-preset-ab','--external-consumers-quiesced',
        '--matched-stationary-shadow','--camera-script',camera_entry,
        '--output',str(record/'stages'),'--seconds','75','--interval','.5']:
        raise RuntimeError('wrapper options differ from the finite matched experiment')
    normal=runner.direct_argv('camera',[argv[0],camera_entry]+argv[split+1:])
    return normal,events[0]['unit']


def stationary_policy_ready(profile):
    """Only an unpublished asynchronous diagnostic is pending, never invalid policy."""
    temporal=profile.get('stationary_temporal')
    if temporal is None:return False
    if (not isinstance(temporal,dict) or temporal.get('enabled') is not True
            or type(temporal.get('alpha')) not in (int,float) or temporal['alpha']!=.4
            or type(temporal.get('delta_depth_units')) not in (int,float) or temporal['delta_depth_units']!=100
            or type(temporal.get('persistence')) not in (int,float) or temporal['persistence']!=0):
        raise RuntimeError('original stationary temporal policy changed')
    return True


def trial_transaction(io):
    """One generation retirement; recover old camera setting before consumers."""
    disturbed=False;failure=None
    def consumers():
        io.verify()
        for role in ('unknown','remote'):
            if io.current.get(role) is None:io.start(role)
        io.final_validate()
    try:
        io.verify();disturbed=True;io.stop('unknown');io.retire_selection()
        io.stop('remote');io.stop('camera');io.configure(PRESET)
        io.start('camera');io.validate_camera();io.restored_warmup()
        io.report['saved_baseline_camera_warmup_complete']=True
        consumers();io.report['trial_loaded']=True
    except BaseException as exc:
        failure=exc;io.report.update(trial_error=repr(exc),trial_loaded=False,visual_nodes_restored=False)
        if disturbed and io.retirement_completed:
            try:
                for role in ('unknown','remote'):io.quiesce_consumer(role)
                io.clear_camera_validation();io.settle_camera_launch();io.stop('camera')
                io.configure(io.old_preset);io.start('camera');io.validate_camera()
                io.restored_warmup();io.report['old_preset_recovered']=True
                consumers()
            except BaseException as recovery:io.report['camera_recovery_error']=repr(recovery)
    finally:
        io.record('saved_baseline_trial_finished')
    if failure:raise failure


def make_runtime(runner,loader,helper):
    Base=runner.make_runtime_class(loader,helper,None)
    class Trial(Base):
        def __init__(self,manifest,manifest_path,record,out):
            import rospy
            from sensor_msgs.msg import Image,JointState
            from std_msgs.msg import Header,Bool,String
            from alicia_flexible_grasp_supervisor.msg import GraspState
            self.ros=rospy;self.helper=helper;self.manifest=manifest;self.out=out
            self.manifest_path=manifest_path;self.manifest_sha=sha(manifest_path)
            self.input_paths=[record/'result.json',record/'consumer_recovery.json',record/'stages/summary.json']
            self.input_hashes={str(path):sha(path) for path in self.input_paths}
            report,recovery,summary=[json.loads(path.read_text()) for path in self.input_paths]
            if manifest.get('input_files')!=self.input_hashes:raise RuntimeError('reviewed recovery receipts changed')
            self.report=dict(events=[],trial_loaded=False,visual_nodes_restored=False,
                input_files=deepcopy(self.input_hashes),motion_commands_sent=False,torque_commands_sent=False,
                temperature_queries_sent=False,new_user_alignment_confirmed=False,new_motion_authority_created=False,
                parameter_writes_sent=False,selection_commands_sent=False,old_preset_recovered=False)
            self.retire_selection_for_maintenance=True;self.retirement_completed=False
            self.selection_transition=None;self.retirement_successor_seen=None;self.retirement_files={}
            self.before={};self.current={};self.last_known={};self.registrations={};self.units={};self.unit_commands={}
            self.environments={};self.parents={};self.core={};self.pairs=OrderedDict()
            self.lock=threading.RLock();self.latest={};self.frozen={};self.faults=[]
            self.camera_started_ns=None;self.verified_camera=None;self.mode_status=None;self.detector_status=None
            self.allowed_env=tuple(dict.fromkeys(helper.ENV_KEYS+loader.EXTRA_ENV+('ROS_HOME',)))
            rospy.init_node('saved_baseline_visual_trial',anonymous=True,disable_signals=True)
            self.subscribers=[rospy.Subscriber('/alicia_d/sdk_command',JointState,self.sdk_cb,queue_size=100),
                rospy.Subscriber('/alicia_d/control_reference_epoch',Header,self.epoch_cb,queue_size=100),
                rospy.Subscriber('/grasp/state',GraspState,self.task_cb,queue_size=100),
                rospy.Subscriber('/grasp_mode/status',String,self.mode_cb,queue_size=8),
                rospy.Subscriber('/perception/unknown/detector_status',String,self.detector_cb,queue_size=8),
                rospy.Subscriber('/alicia_d/motion_enabled',Bool,lambda msg:self.latch('enabled',bool(msg.data)),queue_size=10)]
            helper.wait(lambda:all(key in self.latest for key in ('sdk','epoch','task','enabled')),5.,'current passive state')
            with self.lock:
                if self.latest['task'] or not self.latest['enabled']:raise RuntimeError('inactive task and existing enabled state required')
                self.frozen=deepcopy(self.latest)
            for name in runner.PRESERVED:
                owner=helper.identity(helper.node_pid(rospy,name))
                if owner!=report['preserved'][name]:raise RuntimeError('preserved owner differs from current recovery: '+name)
                self.core[name]=owner;parent=helper.identity(owner['ppid']);self.parents[parent['pid']]=parent
            for role,name in runner.NODES.items():
                owner=helper.identity(helper.node_pid(rospy,name));normal=deepcopy(owner)
                if role=='camera':
                    normal['argv'],unit=completed_wrapper_command(owner['argv'],report,recovery,summary,runner,record)
                else:
                    if owner!=recovery['consumers'][role]['identity']:raise RuntimeError('recovered consumer owner changed: '+role)
                    runner.direct_argv(role,owner['argv']);unit=recovery['consumers'][role]['unit']
                runner.validate_owner_cwd(role,owner,manifest)
                props=self.unit_properties(unit);parent=helper.identity(owner['ppid'])
                runner.validate_parent(role,owner,parent,unit=props)
                self.parents[parent['pid']]=parent
                self.before[role]=normal;self.current[role]=self.last_known[role]=owner
                if self.registered(role)!=owner['pid']:raise RuntimeError('current visual registration changed')
                self.environments[role]=loader.read_environment(owner['pid'],self.allowed_env)
                for key in ('ALICIA_EXPERIMENT_STORAGE_BASE','ALICIA_SESSION_LOG_DIR','GRASP_RECORD_ROOT'):
                    value=self.environments[role].get(key)
                    if role!='camera' and (not value or not (Path(value).resolve()==EXT or EXT in Path(value).resolve().parents)):
                        raise RuntimeError('consumer storage is not on SanStand')
            self.configuration=rospy.get_param('/camera');self.new_configuration=deepcopy(self.configuration)
            from alicia_grasp_modes.rgb_observation_source import camera_geometry_snapshot
            loader.unchanged_camera_settings(recovery['camera_after'],self.configuration)
            if camera_geometry_snapshot(self.configuration)!=camera_geometry_snapshot(recovery['camera_after']):
                raise RuntimeError('recovered camera geometry changed')
            self.old_preset=self.configuration.get('mode_depth_preset',{}).get('unknown_preset')
            if self.old_preset!='high_accuracy':raise RuntimeError('trial requires restored high_accuracy baseline')
            diagnostic=self.configuration.get('runtime_profile',{}).get('preset_ab_diagnostic',{})
            if diagnostic.get('active') is not False or diagnostic.get('restoration_succeeded') is not True:
                raise RuntimeError('wrapper still active or restoration unproved')
            self.context={key:rospy.get_param(key,None) for key in ('/grasp_mode/selection','/handeye','/rgb_acquisition/ledger')}
            if (runner.maintenance_selection(self.context['/grasp_mode/selection'])!=runner.maintenance_selection(recovery['selection'])
                    or self.context['/rgb_acquisition/ledger']):raise RuntimeError('recovery selection or empty ledger changed')
            self.robot_description_sha=hashlib.sha256(rospy.get_param('/robot_description','').encode()).hexdigest()
            helper.wait(lambda:runner.mode_selection_idle(self.mode_status,self.context['/grasp_mode/selection']),5.,'inactive unreserved recovery selection',self.guard)
            for kind,key in (('rgb','color_topic'),('depth','depth_topic')):
                self.subscribers.append(rospy.Subscriber(self.configuration[key],Image,
                    lambda msg,stream=kind:self.image_cb(msg,stream),queue_size=3,buff_size=2**24))
            self.report.update(before=deepcopy(self.current),normal_entries=deepcopy(self.before),
                preserved=deepcopy(self.core),camera_before=deepcopy(self.configuration),
                activation_before=self.configuration['runtime_profile']['geometry_activation_stamp_ns'],
                sdk_hold=deepcopy(self.frozen['sdk']),epoch_ns=self.frozen['epoch'],old_preset=self.old_preset)
            self.verify();self.record('saved_baseline_trial_bound')

        def verify(self):
            verify_manifest(self.manifest,runner)
            if sha(self.manifest_path)!=self.manifest_sha or any(sha(path)!=digest for path,digest in self.input_hashes.items()):
                raise RuntimeError('trial source/recovery receipt changed')
            self.guard()

        def guard(self):
            Base.guard(self)
            if self.ros.get_param(PARAM)!=self.configuration['mode_depth_preset']['unknown_preset']:
                raise RuntimeError('trial preset parameter changed externally')

        def configure(self,value):
            if value not in (self.old_preset,PRESET):raise RuntimeError('unapproved trial preset')
            self.verify()
            if any(self.current.get(role) is not None or self.registered(role) is not None for role in runner.MUTABLE):
                raise RuntimeError('all visual owners must exit before preset configuration')
            self.clear_camera_validation();self.report['parameter_writes_sent']=True
            self.record('preset_change_requested',parameter=PARAM,value=value)
            try:self.ros.set_param(PARAM,value)
            finally:
                # Reconcile an ambiguous XML-RPC response only with readback of
                # the two explicitly allowed values, so recovery can restore.
                actual=self.ros.get_param(PARAM)
                if actual not in (self.old_preset,PRESET):raise RuntimeError('unrecognized preset after parameter write')
                self.configuration['mode_depth_preset']['unknown_preset']=actual
                self.new_configuration=deepcopy(self.configuration)
            if actual!=value:raise RuntimeError('trial preset readback differs')

        def quiesce_consumer(self,role):
            """Also settle a unit started despite a lost systemd-run response."""
            if role not in ('unknown','remote'):raise RuntimeError('consumer role required')
            self.verify()
            if role not in self.units:
                self.stop(role)
                if self.registered(role) is not None:raise RuntimeError('consumer replacement appeared')
                return
            unit=self.units[role];pid=int(self.unit_properties(unit).get('MainPID','0'))
            registered=self.registered(role)
            if registered is not None and registered!=pid:raise RuntimeError('foreign consumer registered')
            if pid:
                owner=helper.identity(pid);command=self.unit_commands[role]
                if (owner['argv']!=command[command.index('/usr/bin/python3'):]
                        or owner['cwd']!=self.before[role]['cwd']
                        or owner['exe_sha256']!=self.before[role]['exe_sha256']):
                    raise RuntimeError('ambiguous consumer unit identity')
                self.current[role]=owner
                if registered is not None and self.registered(role)!=pid:
                    raise RuntimeError('owned consumer registration changed')
            subprocess.run(['systemctl','--user','stop',unit],check=True,timeout=15)
            if pid:helper.wait(lambda:not helper.alive(owner),8.,'owned consumer exit',self.guard)
            if int(self.unit_properties(unit).get('MainPID','0')):raise RuntimeError('owned consumer still alive')
            self.current[role]=None
            if self.registered(role) is not None:raise RuntimeError('consumer registration remains live')

        def validate_camera(self,require_temporal=False):
            Base.validate_camera(self)
            profile=self.report['camera_after']['runtime_profile']
            preset=profile.get('mode_depth_preset',{})
            if preset.get('mode')!='unknown' or preset.get('preset')!=self.configuration['mode_depth_preset']['unknown_preset']:
                raise RuntimeError('camera did not apply expected unknown-mode preset')
            if preset.get('configured_unknown_preset')!=self.configuration['mode_depth_preset']['unknown_preset']:
                raise RuntimeError('camera did not load reviewed explicit preset policy')
            if preset.get('baseline_sha256')!=self.report['camera_before']['runtime_profile']['mode_depth_preset']['baseline_sha256']:
                raise RuntimeError('camera saved baseline changed')
            ready=stationary_policy_ready(profile)
            self.report['stationary_temporal_initial_diagnostic_pending']=not ready
            if require_temporal and not ready:
                raise RuntimeError('stationary temporal diagnostic disappeared after warmup')

        def restored_warmup(self):
            cutoff=self.ros.Time.now().to_nsec();began=time.monotonic();observed=set();active=[]
            def warm():
                camera=self.ros.get_param('/camera');now=self.ros.Time.now().to_nsec()
                if camera['runtime_profile']['mode_depth_preset']['preset']!=self.configuration['mode_depth_preset']['unknown_preset']:
                    raise RuntimeError('trial preset changed during warmup')
                temporal_ready=stationary_policy_ready(camera['runtime_profile'])
                temporal=camera['runtime_profile'].get('stationary_temporal')
                if temporal_ready and temporal.get('active') is True and not active:
                    active.append(dict(observed_ros_stamp_ns=now,owner_pid=self.current['camera']['pid'],
                        geometry_activation_stamp_ns=camera['runtime_profile']['geometry_activation_stamp_ns'],
                        diagnostic_only=True,per_frame_binding=False,status=deepcopy(temporal)))
                with self.lock:observed.update(ns for ns,pair in self.pairs.items() if runner.complete_pair_is_valid(ns,pair,camera,cutoff,now))
                return time.monotonic()-began>=3. and len(observed)>=10 and temporal_ready and bool(active)
            helper.wait(warm,12.,'fresh normal-camera trial warmup',self.guard);self.validate_camera(require_temporal=True)
            self.report['post_restore_warmup']=dict(duration_sec=time.monotonic()-began,distinct_complete_pairs=len(observed),
                after_stamp_ns=cutoff,stationary_active_diagnostic=active[0])
    return Trial


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--execute',action='store_true')
    parser.add_argument('--manifest',type=Path);parser.add_argument('--record',type=Path,default=DEFAULT_RECORD)
    args=parser.parse_args(argv)
    if not args.execute:
        print(json.dumps(dict(prepared_only=True,scope='saved_baseline_unknown_trial',record=str(args.record),
            parameter=PARAM,value=PRESET,unknown_mode_preserved=True,new_motion_authority_created=False,
            persistent_config_files_changed=False,device_actions_performed=False)));return
    if args.manifest is None:parser.error('--execute requires a reviewed --manifest')
    if not EXT.parents[1].is_mount() or EXT/'ros_log' not in args.record.resolve().parents:
        raise RuntimeError('SanStand recovery evidence unavailable; no fallback')
    runner=load('saved_baseline_current_owner_helpers',WORK/RUNNER)
    manifest=json.loads(args.manifest.read_text());verify_manifest(manifest,runner)
    loader=load('saved_baseline_camera_validation',WORK/runner.LOADER)
    helper=load('saved_baseline_process_identity',WORK/runner.HELPER)
    out=EXT/'ros_log'/('saved_baseline_trial_'+datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S.%fZ'))
    out.mkdir();(out/'ros').mkdir();os.environ['ROS_LOG_DIR']=str(out/'ros');runtime=None
    try:
        runtime=make_runtime(runner,loader,helper)(manifest,args.manifest,args.record,out)
        trial_transaction(runtime)
    except BaseException as exc:
        report=runtime.report if runtime else dict(trial_loaded=False,visual_nodes_restored=False)
        report['error']=repr(exc);(out/'result.json').write_text(json.dumps(report,indent=2)+'\n');raise
    finally:
        if runtime:runtime.close()


if __name__=='__main__':main()
