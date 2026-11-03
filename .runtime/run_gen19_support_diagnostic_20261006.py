#!/usr/bin/env python3
"""Gen18 -> gen19 perception-only diagnostic after a verified camera reload.

Keeps the actual original user confirmation. Selects fresh perception only;
never starts the grasp runner, requests planning or submits robot commands.
"""
import argparse
import copy
from datetime import datetime, timezone
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import subprocess
import sys
import time

W=Path(__file__).resolve().parents[1]
MOUNT=Path('/media/zhuyupei/SanStand')
EXT=MOUNT/'实验记录/Robot-Controller'
INDEX=W/'.runtime/current_alignment_supplement_view_20261006.json'
GRASP_INDEX=W/'.runtime/current_grasp_supplement_view_20261006.json'
PRIOR_RECORD=EXT/'grasp_records/20261006T110552.716537Z_6050e2d687f8496fb41dd6d13f7ef9b1'
CONFIRMATION_ID='call_bxnlc8gAuMDZZirhSJaHItcz:0'
CONFIRMATION_TEXT='已调整并对准，可以继续验证'


def require(condition,reason):
    if not condition:raise ValueError(reason)

def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def read(path):return json.loads(Path(path).read_text())
def write_new(path,value):
    with Path(path).open('x') as f:json.dump(value,f,ensure_ascii=False,indent=2,allow_nan=False);f.write('\n')
def load(name,path):
    spec=importlib.util.spec_from_file_location(name,path);m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m);return m

def external_path(path):
    p=Path(path).resolve();require(EXT.resolve() in p.parents,'artifact_not_on_SanStand');return p


def validate_activation_only(before,after):
    b,a=copy.deepcopy(before),copy.deepcopy(after)
    old=b['camera']['runtime_profile_geometry'].pop('geometry_activation_stamp_ns')
    new=a['camera']['runtime_profile_geometry'].pop('geometry_activation_stamp_ns')
    def valid_stamp(value):
        return ((type(value) is int and value>0) or
                (type(value) is str and value.isascii() and value.isdecimal()
                 and str(int(value))==value and int(value)>0))
    require(valid_stamp(old) and valid_stamp(new) and int(new)>int(old),'new_camera_activation_required')
    require(b==a,'context_changed_beyond_camera_activation')
    return old,new


def validate_selection_ack(previous,selected):
    require(previous['generation']==18 and selected['generation']==19
        and selected['mode']=='unknown' and selected['strategy']=='observe_return'
        and selected['stamp_ns']>previous['stamp_ns'],'unexpected_selection_acknowledgement')


def validate_capture_summary(summary):
    require(summary.get('archive_closed') is True and summary.get('complete_without_writer_error') is True,'capture_not_sealed')
    require(summary.get('byte_budget_passed') is True and summary.get('directory_budget_passed') is True,'capture_budget_failed')
    require(summary.get('cooperative_total_deadline_passed') is True,'capture_total_time_budget_failed')


def bind_final_source_status(report,unchanged):
    report['source_unchanged']=bool(unchanged)
    if not unchanged:
        report['complete']=False
        report.setdefault('error','diagnostic_source_changed_during_capture')


def validate_prior_values(receipt,origin,selection_report,manifest,command,runner):
    require(receipt.get('confirmation_id')==CONFIRMATION_ID,'original_confirmation_id_changed')
    require(receipt.get('confirmation_source',{}).get('text')==CONFIRMATION_TEXT,'original_confirmation_text_changed')
    require(receipt.get('confirmation_source')==origin.get('confirmation_source') and
            receipt.get('confirmed_ns')==origin.get('confirmed_ns'),'original_confirmation_provenance_mismatch')
    require(receipt.get('kind')=='explicit_user_target_alignment' and receipt.get('target_aligned') is True,'alignment_receipt_invalid')
    require(selection_report.get('response',{}).get('success') is True and not selection_report.get('failure'),'prior_selection_failed')
    require(selection_report.get('alignment_receipt')==receipt,'selection_parent_receipt_mismatch')
    previous=selection_report['previous_selection'];current=selection_report['new_selection']
    require(previous==receipt['selection'] and previous['generation']==17,'prior_17_receipt_edge_required')
    require(current['generation']==18 and current['mode']=='unknown' and current['strategy']=='observe_return' and current['stamp_ns']>previous['stamp_ns'],'only_generation18_to19_supported')
    require(manifest.get('state')=='complete' and manifest.get('recorder_closed') is True,'prior_record_not_sealed')
    require(command.get('runner_finished') is True and command.get('evidence',{}).get('runner_returncode')==3,'prior_runner_not_terminal')
    require(manifest.get('result_evidence',{}).get('runner_returncode')==3 and
            manifest.get('result_evidence',{}).get('observations')==[],'prior_motion_evidence_not_empty')
    require('NO_FRESH_MODE_PREVIEW' in manifest.get('failure_reason','') and 'generation=18' in manifest['failure_reason'],'prior_failure_not_expected')
    lines=runner.splitlines()
    allowed=('CURRENT_CONFIRMED_UNKNOWN_RUN ','CONFIRMED_DISPATCH ','MODE_ATTEMPT ',
             'UNKNOWN_PLANNING_TERMINAL ','CANDIDATE_COMPUTATION_START ',
             'NO_FRESH_MODE_PREVIEW ','CANDIDATE_COMPUTATION_STOP ')
    require(all(line.startswith(allowed) for line in lines if line.strip()),'unexpected_prior_runner_event')
    require(any(line.startswith('CANDIDATE_COMPUTATION_STOP success=True') for line in lines),'prior_candidate_worker_not_stopped')
    phases=[]
    for line in lines:
        if line.startswith('CONFIRMED_DISPATCH '):
            event=json.loads(line.split(' ',1)[1]);phases.append(event['phase'])
            require(event.get('motion_authorized') is not True,'prior_motion_authority_detected')
    require(phases==['metric_entry','metric_runner_start_requested','metric_runner_finished'],'prior_dispatch_phases_changed')
    return current


def artifacts(deployment_path):
    deployment_path=external_path(deployment_path);d=read(deployment_path)
    require(d.get('camera_loaded') is True and d.get('static_validation_passed') is True and not d.get('error'),'camera_deployment_not_passed')
    require(d.get('events',[])[-1]['event']=='finished','camera_deployment_not_finished')
    require(all(d.get(k) is False for k in ('robot_commands_sent','torque_commands_sent','selection_changed',
            'alignment_confirmation_reissued','new_motion_authority_created')),'camera_deployment_changed_authority')
    vp=external_path(d['manifest_path']);require(sha(vp)==d['manifest_sha256'],'deployment_manifest_hash_changed')
    v=read(vp);require(v.get('tests_passed') is True and v.get('reviewed') is True and v.get('source_unchanged') is True,'source_manifest_not_validated')
    for relative,expected in v['files'].items():
        p=(W/relative).resolve();require(W in p.parents and sha(p)==expected,'validated_source_changed:'+relative)
    index=read(INDEX);rp=external_path(index['receipt']);r=read(rp)
    ep=external_path(r['event_file']);require(sha(ep)==r['event_file_sha256'],'parent_event_hash_changed');origin=read(ep)
    sp=rp.parent/'new_target_selection.json';prior=read(sp)
    require(prior.get('alignment_receipt_file')==str(rp) and prior.get('alignment_receipt_sha256')==sha(rp),'prior_selection_receipt_binding_changed')
    manifest=read(PRIOR_RECORD/'manifest.json');command=read(PRIOR_RECORD/'command_result.json');runner=(PRIOR_RECORD/'runner.log').read_text()
    byname={x['path']:x for x in manifest['files']}
    for name in ('command_result.json','runner.log'):
        require(sha(PRIOR_RECORD/name)==byname[name]['sha256'],'sealed_prior_artifact_changed:'+name)
    selected=validate_prior_values(r,origin,prior,manifest,command,runner)
    parse_selection=load('gen19_pure_selection',W/'src/alicia_grasp_modes/src/alicia_grasp_modes/selection.py').parse_selection
    require(parse_selection(read(GRASP_INDEX)['selection'])==selected,'latest_grasp_index_not_generation18')
    require(parse_selection(d['current_selection'])==selected,'deployment_selection_not_generation18')
    require(d['alignment_receipt_path']==str(rp) and d['alignment_receipt_sha256']==sha(rp),'deployment_parent_receipt_changed')
    require(origin['source_context']['selection']==r['selection'],'parent_event_selection_mismatch')
    return dict(deployment_path=deployment_path,deployment=d,parent_path=rp,parent=r,origin=origin,
                prior_selection_path=sp,selection=selected,index_sha256=sha(INDEX),manifest_path=PRIOR_RECORD/'manifest.json')


def main(argv=None):
    p=argparse.ArgumentParser();p.add_argument('--deployment',required=True,type=Path)
    p.add_argument('--output',type=Path,default=EXT/'ros_log/gen19_support_measurement_20261006')
    args=p.parse_args(argv)
    require(MOUNT.is_mount(),'SanStand_unavailable_no_fallback')
    output=external_path(args.output);require(not output.exists(),'output_already_exists')
    a=artifacts(args.deployment)
    output.mkdir(parents=True,exist_ok=False)
    os.environ['ROS_LOG_DIR']=str(output/'ros')
    # ROS imports and all effects occur only in main; pure contract tests import safely.
    import numpy as np
    import rospy
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Header
    from alicia_flexible_grasp_supervisor.msg import GraspState
    from alicia_grasp_modes.srv import SetGraspMode
    guard=load('gen19_alignment_contract',W/'tools/diagnostics/prepare_rgb_acquisition_retry.py')
    require(guard.digest(a['origin']['source_context'])==a['parent']['source_context_sha256'],'parent_source_context_hash_changed')
    rospy.init_node('gen19_support_diagnostic_only',anonymous=True,disable_signals=True)
    report=dict(scope='gen18_to_gen19_perception_only',preview_only=True,started_wall_ns=time.time_ns(),
        new_user_confirmation_received=False,confirmation_id=a['parent']['confirmation_id'],
        original_confirmed_ns=a['parent']['confirmed_ns'],planning_requested=False,
        runner_started=False,motion_commands_sent=False,torque_commands_sent=False,
        deployment=str(a['deployment_path']),deployment_sha256=sha(a['deployment_path']),
        prior_record=str(PRIOR_RECORD),prior_manifest_sha256=sha(a['manifest_path']),
        source_sha256={str(x.relative_to(W)):sha(x) for x in (Path(__file__).resolve(),W/'.runtime/record_support_gap_15s.py')})
    child=None;log=None;subscriptions=[]
    monitor=guard.AlignmentPoseMonitor(a['parent']);epoch_changed=[]
    def measured_callback(msg):
        try:
            require(msg.header.frame_id=='sdk_measured','unmeasured_feedback')
            q=dict(zip(msg.name,msg.position));monitor.observe(msg.header.stamp.to_nsec(),[q[n] for n in guard.JOINT_NAMES])
        except Exception:monitor.invalidated=True
    def epoch_callback(msg):
        if msg.stamp.to_nsec()!=int(a['parent']['epoch_ns']):epoch_changed.append(msg.stamp.to_nsec())
    def live():
        require(sha(INDEX)==a['index_sha256'],'alignment_index_changed')
        require(not rospy.get_param('/gui/joint_direct_mode',True),'manual_control_active')
        require(not rospy.wait_for_message('/grasp/state',GraspState,timeout=3).active,'grasp_task_active')
        require(not rospy.get_param('/rgb_acquisition/ledger',{}),'observation_authority_present')
        require(rospy.get_param('/grasp/unknown_lift_after_close_enabled') is False,'close_hold_setting_changed')
        selected=guard.parse_selection(rospy.get_param('/grasp_mode/selection'))
        require(selected==a['selection'],'current_selection_changed')
        epoch=rospy.wait_for_message('/alicia_d/control_reference_epoch',Header,timeout=3).stamp.to_nsec()
        require(epoch==int(a['parent']['epoch_ns']) and not epoch_changed,'control_epoch_changed')
        msg=rospy.wait_for_message('/alicia_d/accepted_joint_states',JointState,timeout=3)
        require(msg.header.frame_id=='sdk_measured','unmeasured_feedback')
        q=dict(zip(msg.name,msg.position));positions=guard._q([q[n] for n in guard.JOINT_NAMES])
        require(np.max(np.abs(positions-guard._q(a['parent']['bound_feedback']['positions'])))<=guard.POSE_TOLERANCE_RAD,'pose_changed')
        guard.require_fresh_source(msg.header.stamp.to_nsec(),rospy.Time.now().to_nsec())
        require(msg.header.stamp.to_nsec()>=int(a['parent']['bound_feedback']['stamp_ns']),'feedback_before_parent')
        monitor.require_stationary()
        handeye=rospy.get_param('/handeye');model=rospy.get_param('/robot_description')
        before=guard.retry_source_context(selected,epoch,a['deployment']['camera_parameters_before'],handeye,model)
        expected=copy.deepcopy(a['origin']['source_context']);expected['selection']=selected
        require(before==expected,'pre_reload_context_changed')
        after=guard.retry_source_context(selected,epoch,rospy.get_param('/camera'),handeye,model)
        expected_after=guard.retry_source_context(selected,epoch,a['deployment']['camera_parameters_after'],handeye,model)
        require(after==expected_after,'context_changed_after_deployment')
        activations=validate_activation_only(before,after)
        require(not rospy.get_param('/gui/joint_direct_mode',True),'manual_control_acquired')
        return after,dict(stamp_ns=str(msg.header.stamp.to_nsec()),frame_id='sdk_measured',positions=positions.tolist()),activations
    try:
        subscriptions=[rospy.Subscriber('/alicia_d/accepted_joint_states',JointState,measured_callback,queue_size=10),
                       rospy.Subscriber('/alicia_d/control_reference_epoch',Header,epoch_callback,queue_size=10)]
        live();rospy.wait_for_service('/grasp_mode/select',timeout=4)
        capture=output/'raw_support';log=(output/'capture.log').open('x')
        child=subprocess.Popen([sys.executable,str(W/'.runtime/record_support_gap_15s.py'),'--output',str(capture),'--seconds','15'],
            cwd=W,env=dict(os.environ,ROS_LOG_DIR=str(output/'ros')),stdout=log,stderr=subprocess.STDOUT)
        until=time.monotonic()+7
        while not (capture/'ready.json').is_file():
            require(child.poll() is None,'recorder_exited_before_ready')
            require(time.monotonic()<until,'recorder_ready_timeout');time.sleep(.02)
        ready=read(capture/'ready.json');require(ready['initial_generation']==18,'recorder_wrong_initial_generation')
        context,feedback,(old_activation,new_activation)=live()
        require(time.monotonic()+4<ready['deadline_monotonic'],'insufficient_recording_window_after_revalidation')
        require(child.poll() is None,'recorder_exited_before_selection')
        derivation=dict(kind='unchanged_alignment_after_camera_thread_and_parameter_repair',new_user_confirmation_received=False,
            parent_receipt=str(a['parent_path']),parent_receipt_sha256=sha(a['parent_path']),
            prior_selection_record=str(a['prior_selection_path']),prior_selection_sha256=sha(a['prior_selection_path']),
            prior_sealed_manifest=str(a['manifest_path']),prior_sealed_manifest_sha256=sha(a['manifest_path']),
            deployment=str(a['deployment_path']),deployment_sha256=sha(a['deployment_path']),
            old_activation_stamp_ns=old_activation,new_activation_stamp_ns=new_activation,
            purpose='perception_only_gen19_measurement_no_new_motion_authority')
        event=dict(confirmation_source=a['parent']['confirmation_source'],confirmed_ns=a['parent']['confirmed_ns'],
            confirmation_time_semantics='original actual user reply, later measured revalidation',
            target_aligned=True,source_context=context,bound_feedback=feedback,derivation=derivation,
            robot_commands_sent=False,torque_commands_sent=False)
        ep=output/'alignment_revalidation_event.json';write_new(ep,event)
        receipt=copy.deepcopy(a['parent']);receipt.update(selection=a['selection'],bound_feedback=feedback,
            source_context_sha256=guard.digest(context),event_file=str(ep),event_file_sha256=sha(ep),
            derivation=derivation,recorded_at_utc=datetime.now(timezone.utc).isoformat())
        rp=output/'alignment_receipt.json';write_new(rp,receipt)
        selection_report=dict(preview_only=True,new_user_confirmation_received=False,alignment_receipt=receipt,
            alignment_receipt_file=str(rp),alignment_receipt_sha256=sha(rp),previous_selection=a['selection'],
            new_selection=None,selection_kind='diagnostic_perception_only_support_measurement',
            planning_requested=False,runner_started=False,motion_commands_sent=False,torque_commands_sent=False)
        report['selection_report']=selection_report
        report.update(capture_ready=ready,alignment_receipt=str(rp),selection_started_wall_ns=time.time_ns())
        # Sole ROS mutation: mode reset stops inference internally and starts
        # fresh perception. No request_plan/start-streaming or runner call exists.
        immediate_context,_,_=live()
        require(immediate_context==context,'context_changed_while_saving_derivative')
        require(time.monotonic()+4<ready['deadline_monotonic'],'insufficient_recording_window_before_selection')
        require(child.poll() is None,'recorder_exited_before_selection')
        response=rospy.ServiceProxy('/grasp_mode/select',SetGraspMode)('unknown','observe_return')
        selection_report['response']=dict(success=response.success,message=response.message)
        require(response.success,'fresh_selection_refused:'+response.message)
        until=time.monotonic()+4
        while time.monotonic()<until:
            selected=guard.parse_selection(rospy.get_param('/grasp_mode/selection'))
            if selected!=a['selection']:
                validate_selection_ack(a['selection'],selected)
                selection_report['new_selection']=selected;break
            time.sleep(.02)
        require(selection_report['new_selection'] is not None,'fresh_selection_ack_missing')
        write_new(output/'new_target_selection.json',selection_report)
        report['new_selection']=selection_report['new_selection']
        # Keep existing authority/index records untouched. This diagnostic's
        # derivative receipt and selection are explicitly returned for review.
        child.wait(timeout=22);report['capture_returncode']=child.returncode
        require(child.returncode==0,'diagnostic_capture_failed')
        summary=read(capture/'summary.json');report['capture_summary']=summary
        validate_capture_summary(summary)
        monitor.require_stationary();require(not epoch_changed,'pose_or_epoch_changed_during_capture')
        require(not rospy.get_param('/gui/joint_direct_mode',True),'manual_control_changed_during_capture')
        require(not rospy.wait_for_message('/grasp/state',GraspState,timeout=3).active,'unexpected_task_during_capture')
        report['complete']=True
    except Exception as exc:
        report['error']=repr(exc);raise
    finally:
        if child is not None and child.poll() is None:
            child.terminate();report['diagnostic_child_terminated']=True
            try:child.wait(timeout=3)
            except subprocess.TimeoutExpired:report['diagnostic_child_exit_unconfirmed']=True
        if log is not None:log.close()
        for s in subscriptions:s.unregister()
        report['finished_wall_ns']=time.time_ns()
        previously_failed=bool(report.get('error'))
        try:unchanged=all(sha(W/p)==h for p,h in report['source_sha256'].items())
        except OSError as exc:
            unchanged=False;report['source_verification_error']=repr(exc)
        bind_final_source_status(report,unchanged)
        write_new(output/'run.json',report)
        print(json.dumps({k:v for k,v in report.items() if k not in ('capture_summary','capture_ready')},ensure_ascii=False),flush=True)
        if not unchanged and not previously_failed:raise RuntimeError(report['error'])

if __name__=='__main__':main()
