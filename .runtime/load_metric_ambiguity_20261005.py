"""Replace only unknown perception, retiring the failed selection before restart.

Uses the actual user alignment receipt. This is a new acquisition after a code
repair, never restoration of a lost tracker, cached geometry or old motion plan.
No arm, gripper, enable/disable or temperature commands are sent.
"""
import argparse
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

WORK = Path(__file__).resolve().parents[1]
MOUNT = Path('/media/zhuyupei/SanStand')
RUN = MOUNT/'实验记录/Robot-Controller/ros_log/full_ros_resume_20261005T012259Z'
RECORD = MOUNT/'实验记录/Robot-Controller/grasp_records/20261005T013130.144638Z_f95c4f0498e54f8a922171b982c2df69'
MANIFEST = WORK/'.runtime/metric_ambiguity_validation_20261005.json'
PRESERVE = ('/alicia_d_driver_node', '/bessica_d_hw_interface', '/grasp_task_node',
            '/supervisor_camera', '/remote_grasp6d_node', '/motion_gateway', '/grasp_mode_router')


def module(name, relative):
    spec = importlib.util.spec_from_file_location(name, WORK/relative)
    value = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(value)
    return value


def reload_transaction(io):
    io.verify()
    io.stop_producer()
    # A new producer must never observe the retired generation, even briefly.
    io.select_new_generation()
    io.start_producer()
    io.verify_final()


def validated_epoch(message, expected):
    value=message.stamp.to_nsec()
    if (message.frame_id!='sdk_reference_epoch' or type(value) is not int
            or value<=0 or value!=int(expected)):
        raise ValueError('control_epoch_changed')
    return value


class Runtime:
    def __init__(self):
        import rospy
        from sensor_msgs.msg import JointState
        from std_msgs.msg import Header, String
        from alicia_flexible_grasp_supervisor.msg import GraspState
        self.ros = rospy
        self.loader = module('pair_loader', '.runtime/load_pair_rectification_20261004.py')
        self.helper = module('visual_process', '.runtime/gen4_root_cause/reload_camera_visual_gate.py')
        self.guardlib = module('alignment_binding', 'tools/diagnostics/prepare_rgb_acquisition_retry.py')
        assert MOUNT.is_mount() and RUN.is_dir(), 'SanStand unavailable; no fallback'
        self.manifest = json.loads(MANIFEST.read_text())
        self.manifest_sha = self.loader.sha(MANIFEST)
        self.loader.verify_manifest(self.manifest)
        prior_path = RUN/'new_target_selection.json'
        prior = json.loads(prior_path.read_text())
        self.selection = self.guardlib.parse_selection(prior['new_selection'])
        assert self.selection['generation'] == 2
        receipt = prior['alignment_receipt']
        assert receipt['confirmation_id'] == 'call_miGeToTAr9eAhD7cnx0dgXKW:0'
        assert receipt['confirmation_source']['text'] == '已调好并对准新目标，可以继续'
        assert self.loader.sha(prior['alignment_receipt_file']) == prior['alignment_receipt_sha256']
        assert self.loader.sha(receipt['event_file']) == receipt['event_file_sha256']
        self.context = json.loads(Path(receipt['event_file']).read_text())['source_context']
        self.monitor = self.guardlib.AlignmentPoseMonitor(receipt)
        closure = json.loads((RECORD/'recorder_outcome.json').read_text())
        motion = json.loads((RECORD/'first_ambiguity_analysis/no_motion_recording_receipt.json').read_text())
        assert closure['bag_closed'] is True and motion['dispatch_exited'] is True
        assert motion['trajectory_goal_messages'] == motion['runner_grasp_start_call_occurrences'] == 0
        self.lock = threading.RLock()
        self.latest, self.fault = {}, None
        self.report = dict(preview_only=False, loaded=False, alignment_receipt=receipt,
            alignment_receipt_file=prior['alignment_receipt_file'],
            alignment_receipt_sha256=prior['alignment_receipt_sha256'],
            previous_selection=self.selection, new_selection=None,
            selection_kind='fresh_acquisition_after_code_repair',
            no_motion_recording_scope=motion['scope'], prior_attempt=str(RECORD),
            no_motion_receipt_sha256=self.loader.sha(RECORD/'first_ambiguity_analysis/no_motion_recording_receipt.json'),
            recorder_closure_sha256=self.loader.sha(RECORD/'recorder_outcome.json'),
            previous_transition_sha256=self.loader.sha(prior_path),
            manifest_sha256=self.manifest_sha, motion_commands_sent=False,
            torque_commands_sent=False, temperature_queries_sent=False, events=[])
        capture = MOUNT/'实验记录/Robot-Controller/ros_log/rgb_observation_bootstrap_20261005T015643Z/result.json'
        self.report['current_scene_capture'] = dict(path=str(capture),sha256=self.loader.sha(capture),
            image_review='Original centered metal target remains visible; current measurement must reacquire independently.')
        self.out = RUN/('metric_ambiguity_load_'+datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ'))
        self.out.mkdir(); (self.out/'ros').mkdir()
        rospy.init_node('load_metric_ambiguity_perception_only', anonymous=True, disable_signals=True)
        def accepted(msg):
            with self.lock:
                try:
                    values = dict(zip(msg.name, msg.position))
                    assert msg.header.frame_id == 'sdk_measured'
                    q = [values[n] for n in self.guardlib.JOINT_NAMES]
                    self.monitor.observe(msg.header.stamp.to_nsec(), q)
                    self.latest['accepted'] = msg.header.stamp.to_nsec()
                except Exception as exc:
                    self.fault = repr(exc)
        def state(msg):
            with self.lock:
                self.latest['task'] = bool(msg.active)
                if msg.active: self.fault = 'task_became_active'
        def epoch(msg):
            with self.lock:
                try:
                    self.latest['epoch'] = validated_epoch(msg,receipt['epoch_ns'])
                except Exception as exc:
                    self.fault=repr(exc)
        def mode(msg):
            try:
                value=json.loads(msg.data)
            except (TypeError,ValueError):return
            with self.lock:self.latest['mode']=value
        self.subs = [rospy.Subscriber('/alicia_d/accepted_joint_states', JointState, accepted, queue_size=100),
            rospy.Subscriber('/alicia_d/control_reference_epoch', Header, epoch, queue_size=4),
            rospy.Subscriber('/grasp/state', GraspState, state, queue_size=8),
            rospy.Subscriber('/grasp_mode/status', String, mode, queue_size=8)]
        self.helper.wait(lambda: all(k in self.latest for k in ('accepted','epoch','task')), 5., 'current feedback')
        self.before = {n:self.helper.identity(self.helper.node_pid(rospy,n)) for n in PRESERVE}
        self.old = self.helper.identity(self.helper.node_pid(rospy, '/unknown_tabletop_perception'))
        self.loader.direct_argv('unknown', self.old['argv'])
        self.loader.validate_relay('unknown')
        self.env_keys = self.helper.ENV_KEYS + self.loader.EXTRA_ENV
        self.env = self.loader.read_environment(self.old['pid'], self.env_keys)
        self.report.update(preserved_processes=self.before, old_producer=self.old)
        self.save('prepared')

    def save(self, event, **fields):
        self.report['events'].append(dict(event=event, time=time.time(), **fields))
        (self.out/'result.json').write_text(json.dumps(self.report,ensure_ascii=False,indent=2)+'\n')

    def guard(self):
        assert MOUNT.is_mount(), 'SanStand unavailable'
        with self.lock:
            assert not self.fault, self.fault
            assert self.latest['task'] is False
            self.monitor.require_stationary()
            self.guardlib.require_fresh_source(self.latest['accepted'], self.ros.Time.now().to_nsec())
        assert self.guardlib.parse_selection(self.ros.get_param('/grasp_mode/selection')) == self.selection
        assert not self.ros.get_param('/rgb_acquisition/ledger', {}), 'unexpected observation episode'
        current = self.guardlib.retry_source_context(self.context['selection'], self.latest['epoch'],
            self.ros.get_param('/camera'), self.ros.get_param('/handeye'), self.ros.get_param('/robot_description'))
        assert self.guardlib.digest(current) == self.guardlib.digest(self.context), 'alignment geometry changed'

    def verify(self):
        self.guard()
        assert self.loader.sha(MANIFEST) == self.manifest_sha
        self.loader.verify_manifest(self.manifest)
        for n, old in self.before.items():
            assert self.helper.alive(old) and self.helper.node_pid(self.ros,n) == old['pid'], n

    def stop_producer(self):
        self.verify()
        assert self.helper.alive(self.old)
        assert self.helper.node_pid(self.ros,'/unknown_tabletop_perception') == self.old['pid']
        os.kill(self.old['pid'], signal.SIGINT)
        self.helper.wait(lambda:not self.helper.alive(self.old), 15., 'old perception exit', self.guard)
        self.save('old_producer_exited')

    def select_new_generation(self):
        from alicia_grasp_modes.srv import SetGraspMode
        self.verify()
        assert not self.helper.alive(self.old)
        old = deepcopy(self.selection)
        response = self.ros.ServiceProxy('/grasp_mode/select', SetGraspMode)('unknown','observe_return')
        self.report['response'] = dict(success=response.success,message=response.message)
        assert response.success, response.message
        def changed():
            value = self.guardlib.parse_selection(self.ros.get_param('/grasp_mode/selection'))
            if value == old: return False
            assert (value['generation'] == old['generation']+1 and value['stamp_ns'] > old['stamp_ns']
                    and value['mode']=='unknown' and value['strategy']=='observe_return')
            return value
        self.selection = self.helper.wait(changed, 8., 'new selection')
        def resets_done():
            with self.lock:value=deepcopy(self.latest.get('mode',{}))
            return (all(value.get(k)==v for k,v in self.selection.items())
                    and value.get('state')=='waiting_for_target'
                    and value.get('active') is False and value.get('start_reserved') is False)
        self.helper.wait(resets_done,10.,'remote/task reset acknowledgements',self.guard)
        self.report['new_selection'] = self.selection
        self.save('failed_generation_retired_before_producer_restart')

    def start_producer(self):
        self.verify()
        unit = 'alicia-metric-ambiguity-perception-'+datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ').lower()
        command = self.loader.service_argv('unknown', self.old, self.env, self.out, unit, self.env_keys)
        subprocess.run(command,check=True,timeout=15,capture_output=True,text=True)
        self.report['unit'] = unit
        def replaced():
            try:
                pid = self.helper.node_pid(self.ros,'/unknown_tabletop_perception')
            except (self.helper.CameraRegistrationPending, ConnectionRefusedError):
                return False
            return self.helper.identity(pid) if pid != self.old['pid'] else False
        self.report['new_producer'] = self.helper.wait(replaced,20.,'new perception registration',self.guard)
        self.save('new_producer_registered')

    def verify_final(self):
        self.verify()
        self.report['loaded'] = True
        self.save('loaded_preserved_hardware_processes')
        (RUN/'metric_ambiguity_new_selection.json').write_text(json.dumps(self.report,ensure_ascii=False,indent=2)+'\n')


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--execute',action='store_true')
    if not parser.parse_args().execute: raise SystemExit('Explicit --execute required')
    io=Runtime()
    try:
        reload_transaction(io)
    except Exception as exc:
        io.report['failure']=repr(exc);io.save('failed')
        raise
    finally:
        print(json.dumps(dict(result=str(io.out/'result.json'),loaded=io.report['loaded'],
            new_selection=io.report['new_selection'],failure=io.report.get('failure')),ensure_ascii=False),flush=True)
        for sub in io.subs:sub.unregister()


if __name__=='__main__':main()
