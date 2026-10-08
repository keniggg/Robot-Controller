#!/usr/bin/env python3
"""Prepared current-owner visual experiment; no actions without --execute.

Uses a new reviewed manifest and current ROS/PID identities. Never imports the
old deployment's Runtime constructor, PID baseline, or experiment receipt.
Only the camera and two perception consumers may be restarted. An explicit
maintenance option retires the old selection while its remote reset owner is
still alive. That creates no user confirmation or motion authority. No direct
parameter, joint, torque, task-start, or temperature command is issued.
"""
import argparse
from collections import OrderedDict
from copy import deepcopy
from datetime import datetime, timezone
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import signal
import subprocess
import threading
import time
import xml.etree.ElementTree as ET
import xmlrpc.client

WORK = Path(__file__).resolve().parents[1]
PROJECT = WORK.parents[1]
MOUNT = Path('/media/zhuyupei/SanStand')
EXT = MOUNT/'实验记录/Robot-Controller'
STARTUP_RUN = EXT/'ros_log/full_ros_resume_20261005T012259Z'
RETIREMENT_RECORD = EXT/'grasp_records/20261005T020640.087979Z_9385a36be68d4465a8d86f609e7179d2'
WRAPPER = '.runtime/camera_preset_ab_observer.py'
WRAPPER_SHA256 = '1e0a645fa8b6dcd53b328840b6bfd333602e45bcedbca5ba457639db4cc46b8d'
PLAN_SHA256 = 'acbe697d8051226b241760fa2869b85d277e4e01a0037fd772ff8bd07ef1f107'
LOADER = '.runtime/load_pair_rectification_20261004.py'
HELPER = '.runtime/gen4_root_cause/reload_camera_visual_gate.py'
STALE = '.runtime/visual_registration_20261004.py'
SELF = '.runtime/run_matched_shadow_current_20261005.py'
TEST = '.runtime/test_run_matched_shadow_current_20261005.py'
MUTABLE = ('camera', 'unknown', 'remote')
PRESERVED = ('/alicia_d_driver_node', '/bessica_d_hw_interface', '/grasp_task_node', '/motion_gateway', '/grasp_mode_router')
NODES = dict(camera='/supervisor_camera', unknown='/unknown_tabletop_perception', remote='/remote_grasp6d_node')
MAIN = dict(camera='src/alicia_flexible_grasp_supervisor/scripts/camera_node.py',
            unknown='src/alicia_grasp_modes/scripts/unknown_tabletop_perception.py',
            remote='src/alicia_grasp_modes/scripts/mode_aware_remote_grasp6d.py')
RELAY = {r:'devel/lib/'+('alicia_grasp_modes/' if r!='camera' else 'alicia_flexible_grasp_supervisor/')+Path(p).name for r,p in MAIN.items()}
LAUNCH_ROOT = PROJECT/'.ros_log/ros_live_20260922_session/resume_modes.launch'
LAUNCH_DEFINITIONS = dict(camera='src/alicia_flexible_grasp_supervisor/launch/sensors.launch',
                          unknown='src/alicia_grasp_modes/launch/perception_modes.launch',
                          remote='src/alicia_grasp_modes/launch/perception_modes.launch')
REQUIRED = {SELF,TEST,WRAPPER,'.runtime/test_camera_preset_ab_observer.py',
            '.runtime/camera_preset_ab_plan_20261004.json','.runtime/camera_stage_observer.py',
            LOADER,HELPER,STALE,*MAIN.values(),*RELAY.values(),*LAUNCH_DEFINITIONS.values(),
            'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/vision/stationary_depth.py',
            'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/vision/visual_stationary.py',
            'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/vision/realsense_manager.py',
            'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/vision/rgbd_rectification.py',
            'src/alicia_grasp_modes/src/alicia_grasp_modes/rgb_observation_source.py',
            'src/alicia_grasp_modes/scripts/mode_router.py',
            'src/alicia_grasp_modes/src/alicia_grasp_modes/policy.py',
            'src/alicia_grasp_modes/src/alicia_grasp_modes/selection.py',
            'src/alicia_grasp_modes/srv/SetGraspMode.srv'}


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def load(name, path):
    spec=importlib.util.spec_from_file_location(name,path)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    return module


def verify_manifest(manifest, root=WORK, required=None):
    """The root supplies new, actually tested/reviewed files; never old hashes."""
    if (manifest.get('schema') != 1 or manifest.get('scope') != 'matched_shadow_current_owner'
            or manifest.get('reviewed') is not True or manifest.get('tests_passed') is not True
            or not manifest.get('validation_results')):
        raise RuntimeError('current reviewed validation manifest required')
    files=manifest.get('files',{})
    if not isinstance(files,dict) or not (REQUIRED if required is None else set(required)) <= files.keys():
        raise RuntimeError('current manifest scope incomplete')
    if required is None and (files.get(WRAPPER)!=WRAPPER_SHA256
            or files.get('.runtime/camera_preset_ab_plan_20261004.json')!=PLAN_SHA256):
        raise RuntimeError('matched shadow differs from the independently reviewed freeze')
    for name,expected in files.items():
        rel=Path(name);path=(root/rel).resolve()
        if rel.is_absolute() or '..' in rel.parts or str(rel)!=name or root.resolve() not in path.parents:
            raise RuntimeError('noncanonical manifest path')
        if sha(path)!=expected:
            raise RuntimeError('reviewed source changed: '+name)
    return files


def direct_argv(role, argv, root=WORK):
    """Normal current entry only; do not silently revive any previous wrapper."""
    if role not in MUTABLE or len(argv)<3 or argv[0]!='/usr/bin/python3':
        raise RuntimeError('unexpected current visual entry')
    entry=Path(argv[1])
    if not entry.is_absolute() or entry.resolve() not in {(root/MAIN[role]).resolve(),(root/RELAY[role]).resolve()}:
        raise RuntimeError('current visual owner must be a normal reviewed entry: '+role)
    seen=set()
    for arg in argv[2:]:
        if not isinstance(arg,str) or arg.startswith('-') or ':=' not in arg:
            raise RuntimeError('unexpected non-ROS argument')
        key,value=arg.split(':=',1)
        if not key or not value or key in seen:
            raise RuntimeError('invalid ROS remap')
        seen.add(key)
    if '__name:='+NODES[role][1:] not in argv[2:]:
        raise RuntimeError('node name mismatch')
    return list(argv)


def validate_nonrespawning_definition(role, path):
    declarations=[n for n in ET.parse(path).getroot().iter('node') if n.get('name')==NODES[role][1:]]
    if len(declarations)!=1 or declarations[0].get('respawn','false').lower()!='false':
        raise RuntimeError('visual owner may automatically respawn: '+role)


def validate_parent(role, owner, parent, *, launch_root=LAUNCH_ROOT, unit=None):
    if owner['ppid']!=parent['pid']:
        raise RuntimeError('parent identity mismatch')
    argv=parent['argv']
    if '/opt/ros/noetic/bin/roslaunch' in argv:
        if str(launch_root) not in argv:
            raise RuntimeError('unreviewed launch parent')
        return 'current_roslaunch_nonrespawning_definition'
    if '--user' in argv and Path(parent['exe']).name=='systemd':
        if not isinstance(unit,dict) or unit.get('MainPID')!=str(owner['pid']) or unit.get('Restart')!='no':
            raise RuntimeError('user service owner or no-restart policy unproved')
        return 'current_user_service_restart_no'
    raise RuntimeError('unsupported visual parent: '+role)


def validate_owner_cwd(role, owner, manifest):
    """Bind the reviewed real cwd, including roslaunch's usual ROS_HOME cwd."""
    actual=owner.get('cwd');expected=manifest.get('owner_cwd',{}).get(role)
    if not isinstance(actual,str) or not isinstance(expected,str):
        raise RuntimeError('reviewed current owner cwd missing: '+role)
    path=Path(actual)
    if not path.is_absolute() or str(path.resolve())!=actual or actual!=expected:
        raise RuntimeError('current owner cwd differs from reviewed fact: '+role)
    return actual


def registration_uri(rospy, name):
    """Read the master endpoint without treating arbitrary lookup errors as absent."""
    code, detail, uri = rospy.get_master().lookupNode(name)
    if code != 1:
        if detail == 'unknown node [' + name + ']':
            return None
        raise RuntimeError('ROS lookup failed: ' + detail)
    if not isinstance(uri, str) or not uri.startswith('http://'):
        raise RuntimeError('ROS node registration URI is invalid')
    return uri


def registered_pid_at_uri(rospy, uri, helper):
    """Query exactly the captured URI; a second hidden lookup can race a restart."""
    code, detail, pid = xmlrpc.client.ServerProxy(uri, transport=helper.Transport()).getPid(rospy.get_name())
    if code != 1 or type(pid) is not int or pid <= 0:
        raise RuntimeError('ROS PID failed: ' + str(detail))
    return pid


def complete_pair_is_valid(stamp, pair, camera, cutoff, now):
    if set(pair)!={'rgb','depth'} or not cutoff<stamp<=now or now-stamp>800000000:
        return False
    if stamp<=int(camera['runtime_profile']['geometry_activation_stamp_ns']):
        return False
    for kind,bpp,encodings in (('rgb',3,('bgr8','rgb8')),('depth',2,('16UC1',))):
        row=pair[kind]
        if (row['caller']!=NODES['camera'] or row['frame_id']!=camera['frame_id']
                or row['encoding'] not in encodings
                or (row['width'],row['height'])!=(camera['width'],camera['height'])
                or row['step']<row['width']*bpp or row['bytes']!=row['step']*row['height']):
            return False
    return True


def maintenance_selection(value):
    if not isinstance(value,dict):raise RuntimeError('maintenance selection missing')
    stamp=value.get('stamp_ns')
    if isinstance(stamp,str) and stamp.isdecimal():stamp=int(stamp)
    generation=value.get('generation')
    if (value.get('mode')!='unknown' or value.get('strategy')!='observe_return'
            or type(generation) is not int or generation<1 or type(stamp) is not int or stamp<=0):
        raise RuntimeError('maintenance requires exact unknown observe_return selection')
    return dict(mode='unknown',strategy='observe_return',generation=generation,stamp_ns=stamp)


def retirement_successor(old,value):
    old,current=maintenance_selection(old),maintenance_selection(value)
    if (current['generation']!=old['generation']+1 or current['stamp_ns']<=old['stamp_ns']):
        raise RuntimeError('maintenance selection is not the single expected successor')
    return current


def retirement_transition_guard(old,seen,actual):
    if maintenance_selection(actual)==maintenance_selection(old):
        if seen is not None:raise RuntimeError('maintenance selection rolled back')
        return None
    successor=retirement_successor(old,actual)
    if seen is not None and seen!=successor:raise RuntimeError('maintenance successor changed')
    return successor


def mode_reset_complete(status,selection):
    if not isinstance(status,dict):return False
    try:
        same=maintenance_selection(status)==maintenance_selection(selection)
    except RuntimeError:return False
    return bool(same and status.get('state')=='waiting_for_target'
                and status.get('active') is False and status.get('start_reserved') is False)


def mode_selection_idle(status,selection):
    if not isinstance(status,dict) or status.get('state') in ('switching','switch_queued'):
        return False
    return mode_reset_complete(dict(status,state='waiting_for_target'),selection)


def validate_retirement_record(selection,command,dispatch,manifest,closure):
    """Closed prior attempt only; this does not attest alignment or grant motion."""
    selection=maintenance_selection(selection)
    if (command.get('runner_finished') is not True
            or type(command.get('evidence',{}).get('runner_returncode')) is not int
            or manifest.get('state')!='complete' or manifest.get('recorder_closed') is not True
            or closure.get('bag_closed') is not True
            or maintenance_selection(dispatch.get('selection'))!=selection):
        raise RuntimeError('maintenance requires the closed runner for this selection')
    endings=[v for v in dispatch.get('events',[]) if v.get('phase')=='metric_runner_finished']
    if (len(endings)!=1 or endings[0].get('returncode')!=command['evidence']['runner_returncode']):
        raise RuntimeError('maintenance runner completion receipt inconsistent')
    return selection


def build_command(role, old, environment, out, unit, allowed, *, experiment=False, root=WORK):
    argv=[a for a in direct_argv(role,old['argv'],root) if not a.startswith('__log:=')]
    argv.append('__log:='+str(out/(role+'.ros.log')))
    if experiment:
        if role!='camera':raise RuntimeError('only camera can run the diagnostic')
        argv=argv[:1]+[str(root/WRAPPER),'--enable-preset-ab','--external-consumers-quiesced',
            '--matched-stationary-shadow','--camera-script',argv[1],'--output',str(out/'stages'),
            '--seconds','75','--interval','.5','--']+argv[2:]
    command=['systemd-run','--user','--collect','--unit='+unit,'--property=Type=exec',
        '--property=Restart=no','--property=WorkingDirectory='+old['cwd'],
        '--property=StandardOutput=append:'+str(out/(role+'.stdout.log')),
        '--property=StandardError=append:'+str(out/(role+'.stderr.log'))]
    command+=['--setenv='+key+'='+environment[key] for key in allowed if key in environment]
    return command+['--setenv=ROS_LOG_DIR='+str(out/'ros'),'--setenv=PYTHONUNBUFFERED=1']+argv


def run_transaction(io):
    """Testable lifecycle; no consumer restart until fresh camera proof exists."""
    disturbed=False;ready=False;failure=None
    try:
        io.verify()
        disturbed=True;io.stop('unknown')
        if getattr(io,'retire_selection_for_maintenance',False):io.retire_selection()
        io.stop('remote')
        io.stop('camera');io.start('camera',experiment=True);io.validate_camera()
        io.await_experiment();io.restored_warmup();ready=True
        if io.report.get('experiment_valid') is not True:
            # Restore availability first, then return failure to the caller.
            # A readable summary is not a successful matched comparison.
            io.report['capture_invalid']=True
            failure=RuntimeError('capture protocol not validated; consult preserved sensor summary')
    except BaseException as exc:
        failure=exc;io.report['experiment_error']=repr(exc)
        if disturbed and (not getattr(io,'retire_selection_for_maintenance',False)
                          or getattr(io,'retirement_completed',False)):
            try:
                # A first consumer stop may have failed. Prove both are gone
                # before any camera replacement, including recovery.
                for role in MUTABLE[1:]:io.stop(role)
                io.clear_camera_validation();io.settle_camera_launch();io.stop('camera')
                io.start('camera',experiment=False);io.validate_camera();io.restored_warmup();ready=True
                io.report['normal_camera_recovered_after_error']=True
            except BaseException as recovery:io.report['camera_recovery_error']=repr(recovery)
    finally:
        if disturbed and (not getattr(io,'retire_selection_for_maintenance',False)
                          or getattr(io,'retirement_completed',False)):
            try:
                io.verify()
                if not ready:raise RuntimeError('fresh camera not proved; consumers remain quiesced')
                for role in MUTABLE[1:]:
                    if io.current.get(role) is None:io.start(role)
                io.final_validate()
            except BaseException as recovery:
                io.report['consumer_recovery_error']=repr(recovery);failure=failure or recovery
        if getattr(io,'retire_selection_for_maintenance',False) and not getattr(io,'retirement_completed',False):
            io.report['maintenance_retirement_incomplete_consumers_not_restarted']=True
        io.record('matched_shadow_finished')
    if failure:raise failure


def make_runtime_class(loader, helper, stale):
    class CurrentRuntime:
        # These methods are identity/topic helpers only. Do not inherit the
        # old constructor, guard, parameter writer, rollback or gateway action.
        latch=loader.Runtime.latch
        sdk_cb=loader.Runtime.sdk_cb
        epoch_cb=loader.Runtime.epoch_cb
        task_cb=loader.Runtime.task_cb
        image_cb=loader.Runtime.image_cb

        def __init__(self, manifest, manifest_path, startup_run, out, *, retire_selection_for_maintenance=False,
                     retirement_record=RETIREMENT_RECORD):
            import rospy
            from sensor_msgs.msg import Image, JointState
            from std_msgs.msg import Header, Bool, String
            from alicia_flexible_grasp_supervisor.msg import GraspState
            self.ros=rospy;self.helper=helper;self.out=out;self.manifest=manifest
            self.retire_selection_for_maintenance=bool(retire_selection_for_maintenance)
            self.retirement_completed=False;self.selection_transition=None;self.retirement_files={}
            self.retirement_successor_seen=None
            self.manifest_path=manifest_path;self.manifest_sha=sha(manifest_path)
            self.startup_path=startup_run/'startup.json';self.startup_sha=sha(self.startup_path)
            startup=json.loads(self.startup_path.read_text())
            if startup.get('worktree')!=str(WORK) or not startup.get('startup_runtime_verified'):
                raise RuntimeError('current startup receipt mismatch')
            self.launch_hash=sha(LAUNCH_ROOT)
            if manifest.get('launch_root_sha256')!=self.launch_hash or manifest.get('startup_sha256')!=self.startup_sha:
                raise RuntimeError('reviewed current startup/launch chain mismatch')
            self.report=dict(experiment_complete=False,experiment_valid=False,visual_nodes_restored=False,
                started_at_utc=datetime.now(timezone.utc).isoformat(),startup_run=str(startup_run),
                startup_sha256=self.startup_sha,manifest_sha256=self.manifest_sha,
                motion_commands_sent=False,torque_commands_sent=False,selection_commands_sent=False,
                temperature_queries_sent=False,parameter_writes_sent=False,events=[],
                maintenance_retirement_requested=self.retire_selection_for_maintenance,
                new_user_alignment_confirmed=False,new_motion_authority_created=False)
            self.allowed_env=tuple(dict.fromkeys(helper.ENV_KEYS+loader.EXTRA_ENV+('ROS_HOME',)))
            self.before={};self.current={};self.last_known={};self.registrations={};self.units={};self.unit_commands={}
            self.environments={};self.parents={};self.core={};self.pairs=OrderedDict()
            self.lock=threading.RLock();self.latest={};self.frozen={};self.faults=[]
            self.camera_started_ns=None;self.verified_camera=None;self.subscribers=[]
            self.mode_status=None;self.detector_status=None
            rospy.init_node('matched_shadow_current',anonymous=True,disable_signals=True)
            self.subscribers=[rospy.Subscriber('/alicia_d/sdk_command',JointState,self.sdk_cb,queue_size=100),
                rospy.Subscriber('/alicia_d/control_reference_epoch',Header,self.epoch_cb,queue_size=100),
                rospy.Subscriber('/grasp/state',GraspState,self.task_cb,queue_size=100),
                rospy.Subscriber('/grasp_mode/status',String,self.mode_cb,queue_size=8),
                rospy.Subscriber('/perception/unknown/detector_status',String,self.detector_cb,queue_size=8),
                rospy.Subscriber('/alicia_d/motion_enabled',Bool,lambda m:self.latch('enabled',bool(m.data)),queue_size=10)]
            helper.wait(lambda:all(k in self.latest for k in ('sdk','epoch','task','enabled')),5.,'current passive state')
            with self.lock:
                if self.latest['task'] or not self.latest['enabled']:raise RuntimeError('requires inactive task and existing enabled state')
                self.frozen=deepcopy(self.latest)
            for name in PRESERVED:
                self.core[name]=helper.identity(helper.node_pid(rospy,name))
                parent=helper.identity(self.core[name]['ppid']);self.parents[parent['pid']]=parent
            for role,name in NODES.items():
                old=helper.identity(helper.node_pid(rospy,name));direct_argv(role,old['argv']);loader.validate_relay(role)
                validate_owner_cwd(role,old,manifest)
                validate_nonrespawning_definition(role,WORK/LAUNCH_DEFINITIONS[role])
                parent=helper.identity(old['ppid']);unit=None
                if Path(parent['exe']).name=='systemd' and '--user' in parent['argv']:
                    names={part for line in (Path('/proc')/str(old['pid'])/'cgroup').read_text().splitlines()
                        for part in line.rsplit(':',1)[-1].split('/') if part.endswith('.service')}
                    names={n for n in names if n.startswith('alicia-')}
                    if len(names)!=1:raise RuntimeError('current user service cgroup ambiguous')
                    unit=self.unit_properties(next(iter(names)));unit['name']=next(iter(names))
                parent_kind=validate_parent(role,old,parent,unit=unit)
                self.parents[parent['pid']]=parent
                self.before[role]=self.current[role]=self.last_known[role]=old
                if self.registered(role) != old['pid']:
                    raise RuntimeError('initial visual registration changed: '+role)
                self.environments[role]=loader.read_environment(old['pid'],self.allowed_env)
                self.report.setdefault('owner_parent_bindings',{})[role]=dict(parent=parent,kind=parent_kind,unit=unit)
                for key in ('ALICIA_EXPERIMENT_STORAGE_BASE','ALICIA_SESSION_LOG_DIR','GRASP_RECORD_ROOT'):
                    value=self.environments[role].get(key)
                    if role!='camera' and (not value or not (Path(value).resolve()==EXT or EXT in Path(value).resolve().parents)):
                        raise RuntimeError('consumer external-storage environment mismatch: '+key)
            self.configuration=rospy.get_param('/camera');self.new_configuration=deepcopy(self.configuration)
            from alicia_flexible_grasp.vision.rgbd_rectification import rectification_contract
            if (rectification_contract(self.configuration) is None or self.configuration.get('pair_rectification',{}).get('enabled') is not True
                    or self.configuration.get('mode_depth_preset',{}).get('unknown_preset')!='high_accuracy'):
                raise RuntimeError('requires current paired high_accuracy camera')
            self.context={key:rospy.get_param(key,None) for key in ('/grasp_mode/selection','/handeye','/rgb_acquisition/ledger')}
            self.robot_description_sha=hashlib.sha256(rospy.get_param('/robot_description','').encode()).hexdigest()
            if not isinstance(self.context['/grasp_mode/selection'],dict) or self.context['/grasp_mode/selection'].get('mode')!='unknown':
                raise RuntimeError('current unknown selection required')
            ledger=self.context['/rgb_acquisition/ledger']
            if self.retire_selection_for_maintenance:
                if ledger:raise RuntimeError('maintenance retirement cannot modify or replace RGB ledger')
                record=retirement_record.resolve()
                if EXT/'grasp_records' not in record.parents:raise RuntimeError('retirement record must be on SanStand')
                docs={}
                for name in ('command_result.json','dispatch.json','manifest.json','recorder_outcome.json'):
                    path=record/name;self.retirement_files[str(path)]=sha(path);docs[name]=json.loads(path.read_text())
                validate_retirement_record(self.context['/grasp_mode/selection'],docs['command_result.json'],
                    docs['dispatch.json'],docs['manifest.json'],docs['recorder_outcome.json'])
                self.report['maintenance_runner_closed_evidence']=dict(files=deepcopy(self.retirement_files),
                    scope='Closed prior runner only, no alignment confirmation or motion authority.')
                helper.wait(lambda:mode_selection_idle(self.mode_status,self.context['/grasp_mode/selection']),
                    5.,'inactive unreserved old selection',self.guard)
            else:
                helper.wait(lambda:self.detector_status is not None,5.,'current detector loss state')
                self.require_no_lost_selection_restart()
            self.report['ledger_status']=loader.terminal_ledger(ledger,rospy.Time.now().to_nsec()) if ledger else 'absent_no_acquisition_authority'
            for stream,key in (('rgb','color_topic'),('depth','depth_topic')):
                self.subscribers.append(rospy.Subscriber(self.configuration[key],Image,
                    lambda m,kind=stream:self.image_cb(m,kind),queue_size=3,buff_size=2**24))
            self.report.update(before=deepcopy(self.before),preserved=deepcopy(self.core),camera_before=self.configuration,
                context=deepcopy(self.context),sdk_hold=deepcopy(self.frozen['sdk']),epoch_ns=self.frozen['epoch'],
                enable_state=self.frozen['enabled'],activation_before=self.configuration['runtime_profile']['geometry_activation_stamp_ns'])
            self.verify();self.record('current_owner_bound')

        def mode_cb(self,message):
            try:
                value=json.loads(message.data)
                if not isinstance(value,dict):return
                with self.lock:
                    self.mode_status=value
                    if value.get('active') is True or value.get('start_reserved') is True:
                        self.faults.append('mode_task_active_or_reserved')
            except (TypeError,ValueError):return

        def detector_cb(self,message):
            try:
                value=json.loads(message.data)
                if not isinstance(value,dict):return
                with self.lock:self.detector_status=value
            except (TypeError,ValueError):return

        def require_no_lost_selection_restart(self):
            status=self.detector_status
            selection=self.context['/grasp_mode/selection']
            if (isinstance(status,dict) and status.get('target_lost') is True
                    and status.get('generation')==selection.get('generation')):
                raise RuntimeError('lost tracker cannot restart in same selection; explicit maintenance retirement required')

        def retire_selection(self):
            """Retire old generation while original remote reset service exists."""
            from alicia_grasp_modes.srv import SetGraspMode
            self.verify()
            if not self.retire_selection_for_maintenance or self.retirement_completed or self.selection_transition is not None:
                raise RuntimeError('maintenance retirement must be explicit and attempted once')
            if self.current.get('unknown') is not None or self.registered('unknown') is not None:
                raise RuntimeError('old unknown producer must have fully exited before retirement')
            remote=self.current.get('remote')
            if not remote or not helper.alive(remote) or self.registered('remote')!=remote['pid']:
                raise RuntimeError('original remote reset owner must remain alive')
            old=deepcopy(self.context['/grasp_mode/selection'])
            if not mode_selection_idle(self.mode_status,old):raise RuntimeError('old selection is active or reserved')
            self.report['maintenance_retirement']=dict(previous_selection=deepcopy(old),
                reason='retire_before_visual_maintenance',new_user_alignment_confirmed=False,
                target_identity_preserved=False,old_tracker_restored=False,motion_authorized=False)
            self.record('maintenance_retirement_requested');self.verify()
            self.selection_transition=deepcopy(old)
            self.report['selection_commands_sent']=True
            response=self.ros.ServiceProxy('/grasp_mode/select',SetGraspMode)('unknown','observe_return')
            self.report['maintenance_retirement']['service_response']=dict(success=response.success,message=response.message)
            if response.success is not True:raise RuntimeError('maintenance retirement rejected: '+str(response.message))
            def changed():
                value=self.ros.get_param('/grasp_mode/selection')
                if maintenance_selection(value)==maintenance_selection(old):return False
                retirement_successor(old,value)
                return deepcopy(value)
            new=helper.wait(changed,8.,'maintenance new selection',self.guard)
            # The changed parameter alone does not prove reset_remote/reset_task.
            self.context['/grasp_mode/selection']=new;self.selection_transition=None
            helper.wait(lambda:mode_reset_complete(self.mode_status,new),10.,
                'maintenance remote/task reset acknowledgements',self.guard)
            self.verify();self.retirement_completed=True
            self.report['maintenance_retirement'].update(new_selection=deepcopy(new),
                reset_acknowledgements_complete=True)
            self.record('maintenance_selection_retired_before_remote_exit')

        @staticmethod
        def unit_properties(unit):
            result=subprocess.run(['systemctl','--user','show',unit,'--property=MainPID,Restart,LoadState'],
                check=True,text=True,stdout=subprocess.PIPE,timeout=3)
            return dict(line.split('=',1) for line in result.stdout.splitlines() if '=' in line)

        def registered(self,role):
            # A process may already exist in its systemd unit while the master
            # still advertises the previous, dead process's URI. Keep the last
            # *confirmed registration* separate from that pending new owner.
            name=NODES[role];uri=registration_uri(self.ros,name)
            if uri is None:return None
            try:
                pid=registered_pid_at_uri(self.ros,uri,helper)
            except ConnectionRefusedError:
                previous=self.registrations.get(role)
                expected=self.current.get(role)
                if (previous is None or previous['uri']!=uri
                        or helper.alive(previous['identity'])):
                    raise
                if (expected is not None and not helper.alive(expected)
                        and (expected['pid'],expected['start_ticks']) != (
                            previous['identity']['pid'],previous['identity']['start_ticks'])):
                    raise
                if registration_uri(self.ros,name)!=uri:
                    raise RuntimeError('registration changed during refused endpoint lookup')
                return None
            if registration_uri(self.ros,name)!=uri:
                raise RuntimeError('registration changed during PID lookup')
            expected=self.current.get(role)
            if expected is not None and pid==expected['pid'] and helper.alive(expected):
                self.registrations[role]=dict(uri=uri,identity=deepcopy(expected))
                self.last_known[role]=deepcopy(expected)
            # A foreign live PID remains visible to the caller's owner check.
            return pid

        def guard(self):
            if not MOUNT.is_mount() or self.ros.is_shutdown():raise RuntimeError('storage/ROS unavailable')
            with self.lock:
                if self.faults:raise RuntimeError('latched preserved state changed: '+repr(self.faults))
                if time.monotonic()-self.latest.get('sdk_receipt',0)>1.:raise RuntimeError('SDK hold source unavailable')
            for name,old in self.core.items():
                if not helper.alive(old) or helper.node_pid(self.ros,name)!=old['pid']:raise RuntimeError('preserved owner changed: '+name)
            if any(not helper.alive(old) for old in self.parents.values()):raise RuntimeError('original parent chain changed')
            for k,v in self.context.items():
                actual=self.ros.get_param(k,None)
                if k=='/grasp_mode/selection' and self.selection_transition is not None:
                    self.retirement_successor_seen=retirement_transition_guard(
                        self.selection_transition,self.retirement_successor_seen,actual)
                elif actual!=v:raise RuntimeError('frozen current context changed')
            if not self.retire_selection_for_maintenance:self.require_no_lost_selection_restart()
            if hashlib.sha256(self.ros.get_param('/robot_description','').encode()).hexdigest()!=self.robot_description_sha:
                raise RuntimeError('robot model changed')
            if self.verified_camera is not None:
                from alicia_grasp_modes.rgb_observation_source import camera_geometry_snapshot
                camera=self.ros.get_param('/camera');loader.unchanged_camera_settings(self.configuration,camera)
                if camera_geometry_snapshot(camera)!=self.verified_camera:raise RuntimeError('new camera geometry changed after validation')

        def verify(self):
            verify_manifest(self.manifest)
            if sha(self.manifest_path)!=self.manifest_sha or sha(self.startup_path)!=self.startup_sha or sha(LAUNCH_ROOT)!=self.launch_hash:
                raise RuntimeError('reviewed manifest/startup chain changed')
            if any(sha(path)!=digest for path,digest in self.retirement_files.items()):
                raise RuntimeError('maintenance closed runner evidence changed')
            self.guard()

        def record(self,event,**fields):
            self.report['events'].append(dict(event=event,monotonic=time.monotonic(),**fields))
            (self.out/'result.json').write_text(json.dumps(self.report,indent=2,allow_nan=False)+'\n')

        def stop(self,role):
            if role not in MUTABLE:raise RuntimeError('nonvisual lifecycle forbidden')
            if (role in ('remote','camera') and self.retire_selection_for_maintenance
                    and not self.retirement_completed):
                raise RuntimeError('maintenance retirement must complete before remote/camera exit')
            self.verify();old=self.current.get(role)
            if old is None:return
            if not helper.alive(old):
                if self.registered(role) is not None:raise RuntimeError('replacement owner appeared')
                self.current[role]=None;return
            if self.registered(role)!=old['pid']:raise RuntimeError('registered owner changed before stop')
            self.record('stop_requested',role=role,identity=old);self.verify()
            if not helper.alive(old) or self.registered(role)!=old['pid']:raise RuntimeError('owner changed immediately before signal')
            os.kill(old['pid'],signal.SIGINT)
            helper.wait(lambda:not helper.alive(old),15.,role+' exit',self.guard)
            self.current[role]=None

        def start(self,role,experiment=False):
            self.verify()
            if self.retire_selection_for_maintenance and not self.retirement_completed:
                raise RuntimeError('maintenance retirement incomplete; no old tracker restart')
            if role not in MUTABLE or self.current.get(role) is not None or self.registered(role) is not None:
                raise RuntimeError('duplicate or nonvisual start rejected')
            if role=='camera' and any(self.current.get(r) is not None or self.registered(r) is not None for r in MUTABLE[1:]):
                raise RuntimeError('both consumers must remain quiesced before a camera start')
            if role!='camera' and (self.verified_camera is None or not self.report.get('post_restore_warmup')):
                raise RuntimeError('consumer start requires restored camera and fresh warmup')
            unit='alicia-matched-'+role+'-'+datetime.now(timezone.utc).strftime('%Y%m%dt%H%M%S%f')
            command=build_command(role,self.before[role],self.environments[role],self.out,unit,self.allowed_env,experiment=experiment)
            self.units[role]=unit;self.unit_commands[role]=command
            if role=='camera':self.camera_started_ns=self.ros.Time.now().to_nsec()
            self.record('start_requested',role=role,unit=unit,command=command);self.verify()
            subprocess.run(command,check=True,timeout=12)
            def ready():
                pid=int(self.unit_properties(unit).get('MainPID','0'))
                if not pid:return False
                identity=helper.identity(pid)
                expected=command[command.index('/usr/bin/python3'):]
                if (identity['argv']!=expected or identity['cwd']!=self.before[role]['cwd']
                        or identity['exe_sha256']!=self.before[role]['exe_sha256']):raise RuntimeError('replacement argv/exe/cwd mismatch')
                self.current[role]=identity
                registered=self.registered(role)
                if registered is not None and registered!=pid:
                    raise RuntimeError('foreign owner registered during visual start')
                return identity if registered==pid else False
            helper.wait(ready,25.,role+' current registration',self.guard)

        def clear_camera_validation(self):
            self.verified_camera=None
            self.report.pop('post_restore_warmup',None)

        def validate_camera(self):
            loader.Runtime.validate_camera(self,True)
            activation=self.report['camera_after']['runtime_profile']['geometry_activation_stamp_ns']
            if activation==self.report['activation_before']:raise RuntimeError('new camera activation boundary absent')
            self.report['activation_after']=activation

        def restored_warmup(self):
            cutoff=self.ros.Time.now().to_nsec();began=time.monotonic();observed=set()
            def warm():
                camera=self.ros.get_param('/camera')
                if camera['runtime_profile']['mode_depth_preset']['preset']!='high_accuracy':raise RuntimeError('high_accuracy not restored')
                now=self.ros.Time.now().to_nsec()
                with self.lock:observed.update(ns for ns,pair in self.pairs.items() if complete_pair_is_valid(ns,pair,camera,cutoff,now))
                return time.monotonic()-began>=3. and len(observed)>=10
            helper.wait(warm,12.,'post-restoration fresh warmup',self.guard)
            self.validate_camera()
            self.report['post_restore_warmup']=dict(duration_sec=time.monotonic()-began,distinct_complete_pairs=len(observed),after_stamp_ns=cutoff)

        def await_experiment(self):
            path=self.out/'stages/summary.json'
            def completed():
                if not path.exists():return False
                try:return json.loads(path.read_text())
                except json.JSONDecodeError:return False
            summary=helper.wait(completed,85.,'finite matched shadow summary',self.guard)
            self.report['sensor_summary']=summary
            if not (summary.get('restoration',{}).get('succeeded') and summary.get('original_read_restored') and summary.get('original_sync_restored')):
                raise RuntimeError('sensor restoration not proved')
            if not summary.get('matched_stationary_shadow',{}).get('enabled'):raise RuntimeError('requested shadow missing')
            for item in summary.get('files',[]):
                name=Path(item['name']);file=path.parent/name
                if name.name!=str(name) or file.stat().st_size!=item['bytes'] or sha(file)!=item['sha256']:
                    raise RuntimeError('capture file evidence mismatch')
            size=sum(p.stat().st_size for p in path.parent.iterdir() if p.is_file())
            if size>159000000:raise RuntimeError('capture byte limit exceeded')
            self.report.update(experiment_complete=True,experiment_valid=bool(summary.get('experiment_valid')),actual_capture_bytes=size)

        def settle_camera_launch(self):
            current=self.current.get('camera')
            if current is not None and helper.alive(current):return
            self.verify()
            registered=self.registered('camera')
            if 'camera' in self.units:
                unit=self.units['camera'];props=self.unit_properties(unit);pid=int(props.get('MainPID','0'))
                if pid:
                    identity=helper.identity(pid);command=self.unit_commands['camera']
                    if (identity['argv']!=command[command.index('/usr/bin/python3'):]
                            or identity['cwd']!=self.before['camera']['cwd']
                            or identity['exe_sha256']!=self.before['camera']['exe_sha256']):raise RuntimeError('ambiguous camera unit identity')
                if registered is not None and registered!=pid:raise RuntimeError('unowned camera registered')
                if pid and registered==pid:
                    # An ambiguous systemd reply may have left current=None.
                    # Bind this proven unit's live URI before stopping it, so
                    # its own stale registration can be identified afterwards.
                    self.current['camera']=identity
                    if self.registered('camera')!=pid:raise RuntimeError('owned camera registration changed before unit stop')
                subprocess.run(['systemctl','--user','stop',unit],check=True,timeout=15)
                if pid:helper.wait(lambda:not helper.alive(identity),8.,'owned ambiguous camera exit',self.guard)
                props=self.unit_properties(unit)
                if int(props.get('MainPID','0')):raise RuntimeError('owned camera still running')
                if self.registered('camera') is not None:raise RuntimeError('camera registration remains live after owned unit exit')
            elif registered is not None:raise RuntimeError('unowned camera registered')
            self.current['camera']=None

        def final_validate(self):
            self.verify()
            for role in MUTABLE:
                now=self.current[role]
                if not now or not helper.alive(now) or self.registered(role)!=now['pid']:raise RuntimeError('visual restoration incomplete')
            self.report.update(visual_nodes_restored=True,after=deepcopy(self.current),preserved_nodes_unchanged=True)
            self.record('visual_restoration_verified')

        def close(self):
            for sub in self.subscribers:sub.unregister()
    return CurrentRuntime


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execute',action='store_true')
    parser.add_argument('--manifest',type=Path)
    parser.add_argument('--startup-run',type=Path,default=STARTUP_RUN)
    parser.add_argument('--retire-selection-for-maintenance',action='store_true')
    parser.add_argument('--retirement-record',type=Path,default=RETIREMENT_RECORD)
    args=parser.parse_args(argv)
    if not args.execute:
        print(json.dumps(dict(prepared_only=True,required_manifest_scope='matched_shadow_current_owner',
            required_files=sorted(REQUIRED),startup_run=str(args.startup_run),maximum_capture_seconds=75,
            required_owner_cwd='Manifest owner_cwd maps camera/unknown/remote to reviewed current absolute cwd; preserve it and any ROS_HOME unchanged.',
            maintenance_retirement_requested=args.retire_selection_for_maintenance,
            retirement_record=str(args.retirement_record),new_user_alignment_confirmed=False,
            new_motion_authority_created=False,
            time_scope='75 seconds bounds capture only; loading, bounded warmup and recovery add time.',
            lifecycle_roles=list(MUTABLE),preserved_nodes=list(PRESERVED),device_actions_performed=False)))
        return
    if args.manifest is None:parser.error('--execute requires the new reviewed --manifest')
    if not MOUNT.is_mount() or EXT/'ros_log' not in args.startup_run.resolve().parents:
        raise RuntimeError('SanStand current startup receipt unavailable; no fallback')
    manifest=json.loads(args.manifest.read_text());verify_manifest(manifest)
    loader=load('current_visual_pure_helpers',WORK/LOADER)
    helper=load('current_visual_identity_helpers',WORK/HELPER)
    stale=load('current_visual_dead_registration',WORK/STALE)
    out=EXT/'ros_log'/('matched_shadow_current_'+datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S.%fZ'))
    out.mkdir();(out/'ros').mkdir();os.environ['ROS_LOG_DIR']=str(out/'ros')
    runtime=None
    try:
        runtime=make_runtime_class(loader,helper,stale)(manifest,args.manifest,args.startup_run,out,
            retire_selection_for_maintenance=args.retire_selection_for_maintenance,
            retirement_record=args.retirement_record)
        run_transaction(runtime)
    except BaseException as exc:
        report=runtime.report if runtime is not None else dict(experiment_valid=False,visual_nodes_restored=False)
        report['error']=repr(exc);(out/'result.json').write_text(json.dumps(report,indent=2)+'\n')
        raise
    finally:
        if runtime is not None:runtime.close()
        for path in (Path(__file__),WORK/WRAPPER,WORK/'.runtime/camera_preset_ab_plan_20261004.json',args.manifest):
            (out/path.name).write_bytes(path.read_bytes())
    print(json.dumps(dict(output=str(out),experiment_valid=runtime.report['experiment_valid'],visual_nodes_restored=runtime.report['visual_nodes_restored'])))


if __name__=='__main__':main()
