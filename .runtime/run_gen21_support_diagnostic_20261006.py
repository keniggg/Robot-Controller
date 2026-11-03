#!/usr/bin/env python3
"""One gen20 -> gen21 perception-only window after verified camera-only reload.

No execution on import. Original user confirmation and indexes are preserved;
recording completeness is neither support-quality acceptance nor motion authority.
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
FROZEN=W/'.runtime/run_gen20_support_diagnostic_20261006.py'
FROZEN_SHA='181259e2c73ee7c0ad86c2b902a63fdd5483f434016b3c195d5ad7910cea5414'
if hashlib.sha256(FROZEN.read_bytes()).hexdigest()!=FROZEN_SHA:
    raise ValueError('frozen_gen20_tool_changed')
_spec=importlib.util.spec_from_file_location('frozen_gen20_diagnostic_contract',FROZEN)
_base=importlib.util.module_from_spec(_spec);_spec.loader.exec_module(_base)
for _name in ('MOUNT','EXT','INDEX','GRASP_INDEX','CONFIRMATION_ID','CONFIRMATION_TEXT',
    'require','sha','read','write_new','load','external_path','validate_activation_only',
    'validate_capture_summary','bind_final_source_status','ORIGINAL_CONFIRMED_NS','parse_selection'):
    globals()[_name]=getattr(_base,_name)
CHAIN_PATH=W/'.runtime/perception_diagnostic_chain_20261006.py'
_chain=load('gen21_diagnostic_chain',CHAIN_PATH)
NO_EFFECTS=_chain.NO_EFFECTS


def context_digest(value):
    return hashlib.sha256(json.dumps(value,sort_keys=True,separators=(',',':'),allow_nan=False).encode()).hexdigest()


def validate_selection_ack(previous,selected):
    previous=parse_selection(previous);selected=parse_selection(selected)
    require(previous['generation']==20 and selected['generation']==21
        and selected['mode']=='unknown' and selected['strategy']=='observe_return'
        and selected['stamp_ns']>previous['stamp_ns'],'only_generation20_to21_supported')


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
    rp=prior/'alignment_receipt.json'
    index=read(INDEX);indexed=external_path(index['receipt'])
    require(index.get('confirmation_id')==CONFIRMATION_ID,'global_alignment_confirmation_changed')
    grasp_index=read(GRASP_INDEX)
    require(external_path(grasp_index['alignment_receipt'])==indexed,'global_grasp_receipt_changed')
    original_source=dict(kind='user_message',message_id='["request_user_input_async", "call_bxnlc8gAuMDZZirhSJaHItcz", 0]',text=CONFIRMATION_TEXT)
    chain=_chain.validate_diagnostic_chain(rp,indexed,grasp_index['selection'],
        read_json=read,digest_file=sha,resolve_path=external_path,parse_selection=parse_selection,
        digest_context=context_digest,validate_activation_only=validate_activation_only,
        validate_capture_summary=validate_capture_summary,
        frozen_scripts={19:(str(_base.FROZEN.relative_to(W)),_base.FROZEN_SHA),
                        20:(str(FROZEN.relative_to(W)),FROZEN_SHA)},
        confirmation_id=CONFIRMATION_ID,confirmation_source=original_source,
        confirmed_ns=ORIGINAL_CONFIRMED_NS)
    nodes=chain['nodes'];current=nodes[-1]
    require([n['selected']['generation'] for n in nodes]==[18,19,20],'expected_18_19_20_chain_required')
    r,origin,selected=current['receipt'],current['event'],current['selected']
    require(parse_selection(d['current_selection'])==selected,'deployment_selection_not_generation20')
    require(external_path(d['alignment_receipt_path'])==rp and d['alignment_receipt_sha256']==sha(rp),'deployment_direct_gen20_parent_changed')
    # Verify the legacy indexed gen18 record as sealed, motion-free terminal
    # evidence. Do not pretend its older selection17 receipt is a gen20 parent.
    root,first=nodes[0],nodes[1]
    prior_record=external_path(first['run']['prior_record'])
    mp=prior_record/'manifest.json';manifest=read(mp)
    require(sha(mp)==first['run']['prior_manifest_sha256'],'prior_sealed_manifest_changed')
    derivation=first['receipt']['derivation']
    require(external_path(derivation['prior_sealed_manifest'])==mp
        and derivation['prior_sealed_manifest_sha256']==sha(mp),'prior_derivative_sealed_manifest_changed')
    commandp=prior_record/'command_result.json';runnerp=prior_record/'runner.log'
    byname={row['path']:row for row in manifest['files']}
    for path in (commandp,runnerp):
        require(sha(path)==byname[path.name]['sha256'],'sealed_prior_artifact_changed:'+path.name)
    legacy_selected=_base._base.validate_prior_values(root['receipt'],root['event'],root['selection_report'],
        manifest,read(commandp),runnerp.read_text())
    require(parse_selection(legacy_selected)==root['selected'],'legacy_selection_changed')
    source_paths=(Path(__file__).resolve(),CHAIN_PATH,FROZEN,_base.FROZEN,W/'.runtime/record_support_gap_15s.py',
        W/'tools/diagnostics/prepare_rgb_acquisition_retry.py',W/'src/alicia_grasp_modes/src/alicia_grasp_modes/selection.py')
    source_hashes.update({str(p.relative_to(W)):sha(p) for p in source_paths})
    hashes=chain['artifact_hashes']
    hashes.update({str(p):sha(p) for p in (deployment_path,vp,mp,commandp,runnerp)})
    return dict(deployment_path=deployment_path,deployment=d,parent_path=rp,parent=r,origin=origin,
        prior_selection_path=current['selection_path'],selection=selected,
        index_sha256=sha(INDEX),grasp_index_sha256=sha(GRASP_INDEX),
        prior_dir=prior,prior_run_path=current['run_path'],summary_path=current['summary_path'],
        source_hashes=source_hashes,artifact_hashes=hashes)


def main(argv=None):
    p=argparse.ArgumentParser();p.add_argument('--deployment',required=True,type=Path)
    p.add_argument('--prior-diagnostic',required=True,type=Path)
    p.add_argument('--output',type=Path,default=EXT/'ros_log/gen21_support_measurement_20261006')
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
    guard=load('gen21_alignment_contract',W/'tools/diagnostics/prepare_rgb_acquisition_retry.py')
    require(guard.digest(a['origin']['source_context'])==a['parent']['source_context_sha256'],'parent_source_context_hash_changed')
    rospy.init_node('gen21_support_diagnostic_only',anonymous=True,disable_signals=True)
    report=dict(scope='gen20_to_gen21_perception_only',preview_only=True,started_wall_ns=time.time_ns(),
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
        ready=read(capture/'ready.json');require(ready['initial_generation']==20,'recorder_wrong_initial_generation')
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
            purpose='perception_only_gen21_measurement_no_new_motion_authority')
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
