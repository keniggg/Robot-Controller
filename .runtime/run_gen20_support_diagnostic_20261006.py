#!/usr/bin/env python3
"""One gen19 -> gen20 perception-only window after camera-only reload.

No execution on import. Preserves the original user reply and old indexes.
Completeness refers to recording, never support quality or grasp authority.
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
FROZEN=W/'.runtime/run_gen19_support_diagnostic_20261006.py'
FROZEN_SHA='bb128a5443ac5c31f39402d25b6d952e76ea55e351f2e6b0dececdbf4bfe5a9e'
if hashlib.sha256(FROZEN.read_bytes()).hexdigest()!=FROZEN_SHA:
    raise ValueError('frozen_gen19_tool_changed')
_spec=importlib.util.spec_from_file_location('frozen_gen19_diagnostic_contract',FROZEN)
_base=importlib.util.module_from_spec(_spec);_spec.loader.exec_module(_base)
for _name in ('MOUNT','EXT','INDEX','GRASP_INDEX','CONFIRMATION_ID','CONFIRMATION_TEXT',
    'require','sha','read','write_new','load','external_path','validate_activation_only',
    'validate_capture_summary','bind_final_source_status'):
    globals()[_name]=getattr(_base,_name)
ORIGINAL_CONFIRMED_NS='1791267609469281435'
parse_selection=load('gen20_pure_selection',W/'src/alicia_grasp_modes/src/alicia_grasp_modes/selection.py').parse_selection
NO_EFFECTS=('planning_requested','runner_started','motion_commands_sent','torque_commands_sent','new_user_confirmation_received')


def validate_selection_ack(previous,selected):
    previous=parse_selection(previous);selected=parse_selection(selected)
    require(previous['generation']==19 and selected['generation']==20
        and selected['mode']=='unknown' and selected['strategy']=='observe_return'
        and selected['stamp_ns']>previous['stamp_ns'],'only_generation19_to20_supported')


def validate_prior_values(run,receipt,origin,selection_report,summary):
    require(run.get('scope')=='gen18_to_gen19_perception_only','prior_not_gen19_diagnostic')
    require(run.get('complete') is True and run.get('source_unchanged') is True
        and run.get('capture_returncode')==0 and not run.get('error'),'prior_diagnostic_not_complete')
    for value in (run,selection_report):
        require(value.get('preview_only') is True and all(value.get(k) is False for k in NO_EFFECTS),
            'prior_diagnostic_effect_scope_changed')
    require(run.get('source_sha256',{}).get(str(FROZEN.relative_to(W)))==FROZEN_SHA,'prior_tool_not_frozen_gen19')
    require(run.get('capture_summary')==summary,'prior_capture_summary_mismatch')
    validate_capture_summary(summary)
    require(receipt.get('kind')=='explicit_user_target_alignment' and receipt.get('target_aligned') is True,'invalid_alignment_receipt')
    require(receipt.get('confirmation_id')==CONFIRMATION_ID and run.get('confirmation_id')==CONFIRMATION_ID,'original_confirmation_id_changed')
    require(receipt.get('confirmed_ns')==ORIGINAL_CONFIRMED_NS and origin.get('confirmed_ns')==ORIGINAL_CONFIRMED_NS
        and run.get('original_confirmed_ns')==ORIGINAL_CONFIRMED_NS,'original_confirmation_time_changed')
    require(receipt.get('confirmation_source',{}).get('kind')=='user_message'
        and receipt['confirmation_source'].get('text')==CONFIRMATION_TEXT
        and receipt['confirmation_source']==origin.get('confirmation_source'),'original_confirmation_provenance_changed')
    require(selection_report.get('response',{}).get('success') is True and not selection_report.get('failure'), 'prior_selection_failed')
    require(selection_report.get('alignment_receipt')==receipt and run.get('selection_report')==selection_report,'prior_receipt_or_selection_report_mismatch')
    previous=parse_selection(selection_report['previous_selection']);selected=parse_selection(selection_report['new_selection'])
    require(previous==parse_selection(receipt['selection']) and previous['generation']==18
        and selected['generation']==19 and selected['mode']=='unknown' and selected['strategy']=='observe_return'
        and selected['stamp_ns']>previous['stamp_ns'],'prior_selection_edge_not18_to19')
    require(parse_selection(run['new_selection'])==selected,'prior_run_selection_mismatch')
    require(origin.get('target_aligned') is True and origin.get('robot_commands_sent') is False
        and origin.get('torque_commands_sent') is False,'prior_event_effect_scope_changed')
    require(origin['source_context']['selection']==previous and int(origin['source_context']['epoch_ns'])==int(receipt['epoch_ns']),
        'prior_context_selection_or_epoch_mismatch')
    require(origin.get('bound_feedback')==receipt.get('bound_feedback') and origin.get('derivation')==receipt.get('derivation'),
        'prior_event_receipt_binding_changed')
    return selected


def artifacts(deployment_path,prior_diagnostic):
    deployment_path=external_path(deployment_path);d=read(deployment_path)
    require(d.get('camera_loaded') is True and d.get('static_validation_passed') is True and not d.get('error'),'camera_deployment_not_passed')
    require(d.get('events') and d['events'][-1]['event']=='finished','camera_deployment_not_finished')
    require(all(d.get(k) is False for k in ('robot_commands_sent','torque_commands_sent','selection_changed',
        'alignment_confirmation_reissued','new_motion_authority_created')),'camera_deployment_changed_authority')
    vp=external_path(d['manifest_path']);require(sha(vp)==d['manifest_sha256'],'deployment_manifest_hash_changed')
    v=read(vp);require(v.get('tests_passed') is True and v.get('reviewed') is True and v.get('source_unchanged') is True,'source_manifest_not_validated')
    source_hashes=dict(v['files'])
    for relative,expected in source_hashes.items():
        p=(W/relative).resolve();require(W in p.parents and sha(p)==expected,'validated_source_changed:'+relative)
    prior=external_path(prior_diagnostic)
    runp=prior/'run.json';rp=prior/'alignment_receipt.json';sp=prior/'new_target_selection.json';summaryp=prior/'raw_support/summary.json'
    run,r,selection_report,summary=map(read,(runp,rp,sp,summaryp))
    require(run.get('alignment_receipt')==str(rp),'prior_run_receipt_path_mismatch')
    require(selection_report.get('alignment_receipt_file')==str(rp) and selection_report.get('alignment_receipt_sha256')==sha(rp),'prior_selection_receipt_binding_changed')
    ep=external_path(r['event_file']);require(ep.parent==prior and sha(ep)==r['event_file_sha256'],'prior_event_path_or_hash_changed');origin=read(ep)
    selected=validate_prior_values(run,r,origin,selection_report,summary)
    # Gen19 deliberately did not replace the older global authority indexes.
    # Verify its exact parent edge instead of renaming that receipt as gen19.
    legacy=external_path(r['derivation']['parent_receipt']);legacy_r=read(legacy)
    require(sha(legacy)==r['derivation']['parent_receipt_sha256'],'legacy_receipt_hash_changed')
    require(all(legacy_r[k]==r[k] for k in ('confirmation_id','confirmation_source','confirmed_ns','epoch_ns')),'legacy_confirmation_or_epoch_changed')
    index=read(INDEX)
    require(external_path(index['receipt'])==legacy and index.get('confirmation_id')==CONFIRMATION_ID,'global_alignment_index_changed')
    require(parse_selection(read(GRASP_INDEX)['selection'])==parse_selection(r['selection']),'global_grasp_index_changed')
    require(parse_selection(d['current_selection'])==selected,'deployment_selection_not_generation19')
    # The reload is bound to the immediately preceding diagnostic receipt.
    # Its legacy index edge was checked above; it is not the reload's parent.
    require(external_path(d['alignment_receipt_path'])==rp and d['alignment_receipt_sha256']==sha(rp),'deployment_direct_gen19_parent_changed')
    prior_deployment=external_path(run['deployment']);require(sha(prior_deployment)==run['deployment_sha256'],'prior_deployment_changed')
    prior_manifest=external_path(run['prior_record'])/'manifest.json'
    require(sha(prior_manifest)==run['prior_manifest_sha256'],'prior_sealed_manifest_changed')
    require(read(prior_manifest).get('state')=='complete' and read(prior_manifest).get('recorder_closed') is True,'prior_grasp_not_sealed')
    source_paths=(Path(__file__).resolve(),FROZEN,W/'.runtime/record_support_gap_15s.py',
        W/'tools/diagnostics/prepare_rgb_acquisition_retry.py',W/'src/alicia_grasp_modes/src/alicia_grasp_modes/selection.py')
    source_hashes.update({str(p.relative_to(W)):sha(p) for p in source_paths})
    paths=(deployment_path,vp,runp,rp,sp,summaryp,ep,legacy,prior_deployment,prior_manifest)
    return dict(deployment_path=deployment_path,deployment=d,parent_path=rp,parent=r,origin=origin,
        prior_selection_path=sp,selection=selected,index_sha256=sha(INDEX),grasp_index_sha256=sha(GRASP_INDEX),
        prior_dir=prior,prior_run_path=runp,summary_path=summaryp,source_hashes=source_hashes,
        artifact_hashes={str(p):sha(p) for p in paths})


def main(argv=None):
    p=argparse.ArgumentParser();p.add_argument('--deployment',required=True,type=Path)
    p.add_argument('--prior-diagnostic',required=True,type=Path)
    p.add_argument('--output',type=Path,default=EXT/'ros_log/gen20_support_measurement_20261006')
    args=p.parse_args(argv)
    require(MOUNT.is_mount(),'SanStand_unavailable_no_fallback')
    output=external_path(args.output);require(not output.exists(),'output_already_exists')
    a=artifacts(args.deployment,args.prior_diagnostic)
    output.mkdir(parents=True,exist_ok=False)
    os.environ['ROS_LOG_DIR']=str(output/'ros')
    # ROS imports and all effects occur only in main; pure contract tests import safely.
    import numpy as np
    import rospy
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Header
    from alicia_flexible_grasp_supervisor.msg import GraspState
    from alicia_grasp_modes.srv import SetGraspMode
    guard=load('gen20_alignment_contract',W/'tools/diagnostics/prepare_rgb_acquisition_retry.py')
    require(guard.digest(a['origin']['source_context'])==a['parent']['source_context_sha256'],'parent_source_context_hash_changed')
    rospy.init_node('gen20_support_diagnostic_only',anonymous=True,disable_signals=True)
    report=dict(scope='gen19_to_gen20_perception_only',preview_only=True,started_wall_ns=time.time_ns(),
        new_user_confirmation_received=False,confirmation_id=a['parent']['confirmation_id'],
        original_confirmed_ns=a['parent']['confirmed_ns'],planning_requested=False,
        runner_started=False,motion_commands_sent=False,torque_commands_sent=False,
        deployment=str(a['deployment_path']),deployment_sha256=sha(a['deployment_path']),
        prior_diagnostic=str(a['prior_dir']),prior_run_sha256=sha(a['prior_run_path']),
        diagnostic_completeness_is_not_measurement_quality=True,
        source_sha256=a['source_hashes'])
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
        require(sha(GRASP_INDEX)==a['grasp_index_sha256'],'grasp_index_changed')
        require(all(sha(Path(p))==h for p,h in a['artifact_hashes'].items()),'immutable_prior_or_deployment_artifact_changed')
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
        ready=read(capture/'ready.json');require(ready['initial_generation']==19,'recorder_wrong_initial_generation')
        context,feedback,(old_activation,new_activation)=live()
        require(time.monotonic()+4<ready['deadline_monotonic'],'insufficient_recording_window_after_revalidation')
        require(child.poll() is None,'recorder_exited_before_selection')
        derivation=dict(kind='unchanged_alignment_after_source_diagnostic_camera_reload',new_user_confirmation_received=False,
            parent_receipt=str(a['parent_path']),parent_receipt_sha256=sha(a['parent_path']),
            prior_selection_record=str(a['prior_selection_path']),prior_selection_sha256=sha(a['prior_selection_path']),
            prior_diagnostic_run=str(a['prior_run_path']),prior_diagnostic_run_sha256=sha(a['prior_run_path']),
            prior_diagnostic_summary=str(a['summary_path']),prior_diagnostic_summary_sha256=sha(a['summary_path']),
            deployment=str(a['deployment_path']),deployment_sha256=sha(a['deployment_path']),
            old_activation_stamp_ns=old_activation,new_activation_stamp_ns=new_activation,
            purpose='perception_only_gen20_measurement_no_new_motion_authority')
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
        require(guard.parse_selection(rospy.get_param('/grasp_mode/selection'))==report['new_selection'],'selection_changed_during_capture')
        require(sha(INDEX)==a['index_sha256'] and sha(GRASP_INDEX)==a['grasp_index_sha256'],'existing_index_changed_during_capture')
        require(all(sha(Path(p))==h for p,h in a['artifact_hashes'].items()),'immutable_artifact_changed_during_capture')
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
