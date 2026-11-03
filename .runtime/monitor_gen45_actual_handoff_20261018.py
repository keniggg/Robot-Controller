"""Snapshot one recorded grasp's text handoffs. Standard library, no ROS I/O.

Use after the root starts a NEW record; this script cannot start, stop, reset,
retry, or authorize anything. Planning retirement is not a Task terminal.
"""
import argparse
from datetime import datetime
import hashlib
import json
from pathlib import Path
import re
import time

MOUNT=Path('/media/zhuyupei/SanStand')
E=MOUNT/'实验记录/Robot-Controller'
N=E/'ros_log/full_latest_gen41_restart_20261018T011222Z'
V=N/'verification'


def inside(p,root):
    try:Path(p).resolve().relative_to(root.resolve());return True
    except ValueError:return False


def digest(data):return hashlib.sha256(data).hexdigest()


def read_rows(path,limit):
    with path.open('rb')as f:
        size=path.stat().st_size;start=max(0,size-limit);f.seek(start)
        if start:f.readline();start=f.tell()
        data=f.read()
    rows=[];offset=start
    for raw in data.splitlines(keepends=True):
        if not raw.endswith(b'\n'):break  # Do not parse a currently partial write.
        text=raw.decode(errors='replace').rstrip('\r\n')
        # Do not inspect or retain temperature readings from general logs.
        if 'temperature' not in text.lower():
            rows.append((text,dict(path=str(path),offset=offset,bytes=len(raw),sha256=digest(raw))))
        offset+=len(raw)
    return rows,dict(path=str(path),captured_start_offset=start,captured_bytes=len(data),
                     captured_sha256=digest(data),file_size_at_read=size)


def numeric_time(line):
    match=re.search(r'\[(\d{10}\.\d+)\]',line)
    if match:return float(match.group(1))
    match=re.search(r' (\d{4}-\d\d-\d\d \d\d:\d\d:\d\d,\d{3}): ',line)
    if match:
        # Python ROS logging and this observer use the same system local clock.
        dt=datetime.strptime(match.group(1),'%Y-%m-%d %H:%M:%S,%f')
        return time.mktime(dt.timetuple())+dt.microsecond/1e6
    return None


def json_after(line,marker):
    if marker not in line:return None
    try:return json.loads(line.split(marker,1)[1])
    except (json.JSONDecodeError,TypeError):return None


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--record',required=True,type=Path)
    ap.add_argument('--consumers',required=True,type=Path)
    ap.add_argument('--gateway-log',required=True,type=Path)
    ap.add_argument('--output',required=True,type=Path)
    args=ap.parse_args()
    assert MOUNT.is_mount() and inside(args.record,E/'grasp_records')
    assert inside(args.consumers,N) and inside(args.gateway_log,N) and inside(args.output,V)
    assert args.output.resolve()!=V.resolve()
    args.output.mkdir(exist_ok=True)
    runner,runner_ref=read_rows(args.record/'runner.log',2*1024*1024)
    consumers,consumer_ref=read_rows(args.consumers,4*1024*1024)
    gateway,gateway_ref=read_rows(args.gateway_log,8*1024*1024)
    attempts=[];selection=None;start_ns=None
    for text,ref in runner:
        match=re.search(r'^MODE_ATTEMPT selection=(\{.*\}) minimum_stamp_ns=(\d+)',text)
        if match:
            value=json.loads(match.group(1));attempts.append(dict(selection=value,
                minimum_stamp_ns=int(match.group(2)),evidence=ref))
            selection=value;start_ns=value.get('stamp_ns',int(match.group(2)))
    assert selection and type(start_ns)is int and start_ns>0,'new-record mode attempt unavailable'
    start=start_ns*1e-9
    events=[];task_phases=[];candidate_rejections=[];visual_retirements=[]
    known_plans=set();execution_plan=None;initial_plan=None;task_failure=None
    def event(kind,stamp,ref,**values):
        row=dict(kind=kind,stamp_sec=stamp,evidence=ref,**values);events.append(row);return row
    for text,ref in consumers:
        stamp=numeric_time(text)
        if stamp is None or stamp<start:continue
        match=re.search(r'\[Grasp\] (\S+) (.*)',text)
        if match:
            state,message=match.groups();row=event('task_phase',stamp,ref,state=state,message=message)
            task_phases.append(row)
            rebound=re.search(r'near-field 6D plan rebound: ([a-z0-9]+)',message)
            if rebound:execution_plan=rebound.group(1);known_plans.add(execution_plan)
            if state=='FAILED' and task_failure is None:
                task_failure=row
                if ': {'in message:
                    try:row['inner_report']=json.loads(message[message.index('{'):])
                    except json.JSONDecodeError:pass
        aperture=json_after(text,'Measured contact aperture: ')
        if aperture is not None:
            diagnostic=aperture.get('rigid_residual_projection_diagnostic',{})
            event('measured_stage_gate',stamp,ref,plan_id=aperture.get('plan_id'),
                pose_index=aperture.get('stage_reference_pose_index'),ok=aperture.get('ok'),code=aperture.get('code'),
                accepted_stamp_ns=aperture.get('accepted_stamp_ns'),epoch_ns=aperture.get('epoch_ns'),
                model_sha256=aperture.get('model_sha256'),opening_m=aperture.get('measured_opening_width_m'),
                required_next_path_contract=aperture.get('required_next_path_contract'),
                current_support_clearance_m=aperture.get('actual_current_minimum_support_clearance_m'),
                required_open_width_m=aperture.get('required_open_width_m'),
                projection_diagnostic_only=diagnostic.get('diagnostic_only'),
                projection_diagnostic_code=diagnostic.get('code'),
                projection_diagnostic_required_width_m=diagnostic.get('required_open_width_m'))
        delivery=json_after(text,'remote 6D contact precheck delivery: ')
        if delivery is not None:
            audit=delivery.get('precheck',{});record=dict(plan_id=delivery.get('plan_id'),
                phase_id=delivery.get('phase_id'),operation=delivery.get('operation'),
                outcome=delivery.get('consumer_outcome'),reason=delivery.get('consumer_error'),
                failure_code=audit.get('failure_code'),failed_stage=audit.get('failed_stage'),
                plan_wire_sha256=delivery.get('plan_wire_sha256'),stamp_sec=stamp,evidence=ref)
            replay=audit.get('contact_failure_replay',{}).get('reference')
            if replay:record['immutable_replay_reference']=replay
            candidate_rejections.append(record)
        if 'Unknown target first loss' in text or 'previous visual evidence retired' in text:
            visual_retirements.append(dict(stamp_sec=stamp,text=text,evidence=ref,
                association='perception/planning retirement; not a Task failure by itself'))
    planning_terminals=[];runner_result=None
    for text,ref in runner:
        if text.startswith('FRESH_MODE_PREVIEW '):
            match=re.search(r'plan_id=([a-z0-9]+)',text)
            if match and initial_plan is None:initial_plan=match.group(1);known_plans.add(initial_plan)
        if text.startswith('OBSERVE_RETURN_PLANNING_TERMINAL '):
            planning_terminals.append(dict(text=text,evidence=ref))
        if text.startswith('GRASP_RESULT '):
            match=re.match(r'GRASP_RESULT success=(True|False) message=(.*)',text)
            if match:runner_result=dict(success=match.group(1)=='True',message=match.group(2),evidence=ref)
    last_curve=None
    for text,ref in gateway:
        stamp=numeric_time(text)
        if stamp is None or stamp<start:continue
        for marker,kind in [('Pregrasp feedforward proposal: ','endpoint_proposal'),
            ('Pregrasp feedforward measured result: ','endpoint_measured_result'),
            ('Pregrasp feedforward stopped: ','endpoint_stopped'),
            ('Contact linear compensation planned: ','contact_linear_path_planned')]:
            data=json_after(text,marker)
            if data is None:continue
            plan=data.get('plan_id')
            if known_plans and plan not in known_plans:continue
            event(kind,stamp,ref,plan_id=plan,pose_index=data.get('fixed_goal_pose_index'),
                code=data.get('code'),sdk_stamp_ns=data.get('sdk_stamp_ns'),
                accepted_stamp_ns=data.get('accepted_stamp_ns'),epoch_ns=data.get('epoch_ns'),
                steps_completed=data.get('steps_completed'),no_motion_submitted=data.get('no_motion_submitted'),
                preference_outcome=data.get('precision_preference_outcome'),
                required_next_path_contract=data.get('required_next_path_contract'),
                future_path_checked=data.get('future_path_checked'),execution_authorized=data.get('execution_authorized'),
                position_error_m=data.get('position_error_m'),orientation_error_rad=data.get('orientation_error_rad'),
                proposed_delta_counts=data.get('proposed_delta_counts'),last_step_response=data.get('last_step_response'),
                hardware_response_verified=data.get('hardware_response_verified'),
                trajectory_sha256=data.get('compensated_trajectory_sha256'))
        audit=json_after(text,'Frozen observation continuous path audit: ')
        if audit is not None and (not known_plans or audit.get('plan_id')in known_plans):
            last_curve=event('bound_continuous_path_audit',stamp,ref,plan_id=audit.get('plan_id'),
                trajectory_sha256=audit.get('trajectory_sha256'),scene_sha256=audit.get('scene_sha256'),
                model_sha256=audit.get('model_sha256'),minimum_clearance_m=audit.get('certified_minimum_clearance_m'),
                interval_checks=audit.get('interval_checks'),segments=audit.get('segments'),
                pose_index=audit.get('compensation_pose_index'))
        submitted=re.search(r': submitted trajectory duration=([0-9.]+)s controller_reference_time_scale=([0-9.]+)',text)
        if submitted:
            bound=last_curve if last_curve is not None and 0<=stamp-last_curve['stamp_sec']<5. else None
            event('actual_submitted_trajectory',stamp,ref,duration_sec=float(submitted.group(1)),
                controller_reference_time_scale=float(submitted.group(2)),
                plan_id=bound.get('plan_id')if bound else None,
                trajectory_sha256=bound.get('trajectory_sha256')if bound else None,
                association='nearest preceding same-log bound audit within5sec'if bound else'unbound; do not infer from task label')
        elif 'strict trajectory retimed duration='in text:
            retimed=re.search(r'strict trajectory retimed duration=([0-9.]+)s',text)
            if retimed:event('initial_retime_only',stamp,ref,duration_sec=float(retimed.group(1)),
                actual_submission_claimed=False,plan_id=None)
    events.sort(key=lambda x:(x['stamp_sec'],x['evidence']['path'],x['evidence']['offset']))
    relevant=[x for x in events if x.get('plan_id')==execution_plan]
    zero=[x for x in relevant if x['kind']=='endpoint_measured_result'and x.get('pose_index')==0 and x.get('no_motion_submitted')is True]
    # The gateway's early no-op response is logged by Task, rather than by the
    # gateway's measured-result branch. Capture that actual reply as well.
    for text,ref in consumers:
        data=json_after(text,'Measured fixed endpoint compensation completed: ')
        if data is None or data.get('plan_id')!=execution_plan:continue
        stamp=numeric_time(text)
        if stamp is None or stamp<start:continue
        item=event('task_received_endpoint_reply',stamp,ref,plan_id=data.get('plan_id'),
            pose_index=data.get('fixed_goal_pose_index'),code=data.get('code'),
            no_motion_submitted=data.get('no_motion_submitted'),steps_completed=data.get('steps_completed'),
            preference_outcome=data.get('precision_preference_outcome'),
            required_next_path_contract=data.get('required_next_path_contract'),
            sdk_stamp_ns=data.get('sdk_stamp_ns'),accepted_stamp_ns=data.get('accepted_stamp_ns'),
            epoch_ns=data.get('epoch_ns'),position_error_m=data.get('position_error_m'),
            orientation_error_rad=data.get('orientation_error_rad'))
        if data.get('fixed_goal_pose_index')==0 and data.get('no_motion_submitted')is True:zero.append(item)
    events.sort(key=lambda x:x['stamp_sec'])
    phase_names=[x['state']for x in task_phases]
    observed=dict(zero_step_pose0_reply=bool(zero),
        stage0_current_CAD_passed=any(x['kind']=='measured_stage_gate'and x.get('plan_id')==execution_plan and x.get('pose_index')==0 and x.get('ok')is True for x in events),
        held_approach_measured_converged=any(x['kind']in('endpoint_measured_result','task_received_endpoint_reply')and x.get('plan_id')==execution_plan and x.get('pose_index')==1 and x.get('code')=='PREGRASP_MEASURED_CONVERGED'for x in events),
        contact_descent_plan=any(x['kind']=='contact_linear_path_planned'and x.get('plan_id')==execution_plan and x.get('pose_index')==2 for x in events),
        stage2_measured_passed=any(x['kind']=='measured_stage_gate'and x.get('plan_id')==execution_plan and x.get('pose_index')==2 and x.get('ok')is True for x in events),
        closing_task_phase_seen=any('CLOSE'in name or 'CLOSING'in name for name in phase_names))
    manifest_path=args.record/'manifest.json';manifest=None
    if manifest_path.exists():
        manifest=json.loads(manifest_path.read_text())
    result=dict(kind='read_only_actual_grasp_handoff_snapshot',captured_ns=time.time_ns(),record=str(args.record.resolve()),
        selection=selection,mode_generation=selection.get('generation'),recorded_mode_attempts=attempts,
        initial_observation_plan=initial_plan,accepted_contact_plan=execution_plan,
        observed_handoffs=observed,latest_task_phase=task_phases[-1]if task_phases else None,
        first_Task_failure=task_failure,runner_result=runner_result,
        planning_terminal_diagnostics=planning_terminals,visual_retirement_diagnostics=visual_retirements,
        candidate_local_rejections=candidate_rejections,events=events,
        record_closed=manifest.get('recorder_closed')if manifest else False,
        record_result=manifest.get('result')if manifest else None,
        source_ranges=[runner_ref,consumer_ref,gateway_ref],
        source_log_timezone=list(time.tzname),helper_sha256=digest(Path(__file__).read_bytes()),
        physical_grasp_success_claimed=False,ROS_or_hardware_access=False,motion_or_reset_commands_sent=False,
        temperature_read=False,source_or_record_modified=False,
        interpretation='Only original Task FAILED/GRASP_RESULT or sealed record outcome is terminal. Candidate rejection, nested rigid diagnostic and visual retirement alone are not. No prior consumed episode is reused.')
    output=args.output/('snapshot_'+str(result['captured_ns'])+'.json')
    with output.open('x')as f:json.dump(result,f,ensure_ascii=False,indent=2,allow_nan=False);f.write('\n')
    assert json.loads(output.read_text())==result
    print(json.dumps(dict(snapshot=str(output),sha256=digest(output.read_bytes()),selection=selection,
        accepted_plan=execution_plan,observed=observed,
        first_failure=task_failure['message'][:300]if task_failure else None,
        runner_success=runner_result['success']if runner_result else None),ensure_ascii=False))


if __name__=='__main__':main()
