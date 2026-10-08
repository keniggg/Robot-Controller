#!/usr/bin/env python3
"""Closed-bag timing audit only. No ROS node, service or publisher calls."""
import argparse
import json
import math
import pathlib
import re


def plain(value):
    if hasattr(value, 'to_nsec'):
        return value.to_nsec()
    if hasattr(value, '__slots__'):
        return {key: plain(getattr(value, key)) for key in value.__slots__}
    if isinstance(value, (list, tuple)):
        return [plain(item) for item in value]
    return value


def payload(record):
    if ': {' not in record['text']:
        return None, None
    label, raw = record['text'].split(': ', 1)
    try:
        return label, json.loads(raw)
    except ValueError:
        return None, None


def summarize(logs, states, bag_start_ns):
    """No wall duration is inferred from the commanded trajectory duration."""
    episodes, settles, endpoints = [], [], []
    for record in sorted(logs, key=lambda item: item['stamp_ns']):
        stamp, text = record['stamp_ns'], record['text']
        label, data = payload(record)
        if record['node'] == '/motion_gateway' and label == 'Pregrasp feedforward proposal':
            episodes.append(dict(plan_id=data['plan_id'], pose_index=data['fixed_goal_pose_index'],
                epoch_ns=data.get('epoch_ns'), proposal_stamp_ns=stamp,
                proposed_delta_counts=data.get('proposed_delta_counts'),
                configured_timing=data.get('feedforward_timing'),
                before_fk_position_error_m=data.get('position_error_m'),
                proposal=data, coverage='proposal_only'))
        if record['node'] == '/motion_gateway' and 'bounded exact joint probe' in text and text.startswith('submitted trajectory duration='):
            pending = [e for e in episodes if 'submit_stamp_ns' not in e and 'measured_result_stamp_ns' not in e]
            if pending:
                episode = pending[-1]
                episode.update(submit_stamp_ns=stamp,
                    submitted_duration_sec_rounded=float(re.search(r'duration=([0-9.]+)s',text).group(1)),
                    coverage='submitted_without_measured_result')
        if record['node'] == '/motion_gateway' and label in ('Pregrasp feedforward measured result', 'Pregrasp feedforward stopped'):
            matches = [e for e in episodes if e['plan_id'] == data.get('plan_id')
                       and e['pose_index'] == data.get('fixed_goal_pose_index') and 'measured_result_stamp_ns' not in e]
            if matches:
                episode = matches[-1]
                episode.update(measured_result_stamp_ns=stamp, measured_result=data,
                    proposal_to_measured_result_wall_sec=(stamp-episode['proposal_stamp_ns'])/1e9,
                    actual_response_counts=data.get('last_step_response',{}).get('measured_delta_counts'),
                    post_fk_position_error_m=data.get('position_error_m'),
                    post_fk_orientation_error_rad=data.get('orientation_error_rad'),
                    result_code=data.get('code'),
                    coverage='measured_result' if label.endswith('measured result') else 'stopped')
                if 'submit_stamp_ns' in episode:
                    episode['submit_to_measured_result_wall_sec']=(stamp-episode['submit_stamp_ns'])/1e9
        if 'Grasp motion settled after ' in text or 'Measured endpoint contract stable after ' in text:
            duration = re.search(r'after ([0-9.]+)s',text)
            settles.append(dict(stamp_ns=stamp, reported_wall_sec=float(duration.group(1)) if duration else None, text=text))
        if label in ('Pregrasp Cartesian compensation', 'Measured fixed endpoint compensation completed', 'Measured contact before closure'):
            endpoints.append(dict(stamp_ns=stamp,label=label,evidence=data))
    current_states=[r for r in states if r['value']['header']['stamp'] >= bag_start_ns]
    boundaries={}
    for row in current_states:
        value=row['value'];boundaries.setdefault(value['message'],value['header']['stamp'])
    pairs={'pregrasp_total':('moving 6D pregrasp','planning bounded 6D approach'),
           'bounded_approach_total':('moving bounded 6D approach','planning linear 6D grasp pose'),
           'linear_grasp_total':('moving linear 6D grasp pose','fixed gripper close'),
           'pregrasp_start_to_close':('moving 6D pregrasp','fixed gripper close')}
    stages={name:dict(start_label=a,end_label=b,complete=a in boundaries and b in boundaries,
                     elapsed_wall_sec=(boundaries[b]-boundaries[a])/1e9 if a in boundaries and b in boundaries else None)
            for name,(a,b) in pairs.items()}
    return dict(feedforward_episodes=episodes,
                feedforward_phase_coverage={str(i):[e['coverage'] for e in episodes if e['pose_index']==i] or ['not_observed'] for i in (0,1)},
                settles=settles, endpoint_model_evidence=endpoints, stage_wall_times=stages,
                excluded_prebag_latched_states=len(states)-len(current_states),
                semantics={'submitted_duration_sec':'commanded trajectory time, not actual wall time',
                           'submit_to_measured_result_wall_sec':'includes action execution, feedback/settle acquisition and result validation',
                           'proposal_to_measured_result_wall_sec':'also includes planning/proof before submission',
                           'actual_response_counts':'encoder counts reported by the recorded result; unavailable on interrupted/missing result',
                           'fk_error':'model-derived error, not independent physical metrology'})


def action_records(path):
    import rosbag
    goals, results = [], {}
    with rosbag.Bag(str(path)) as bag:
        for topic,message,receipt in bag.read_messages(topics=['/alicia_controller/follow_joint_trajectory/goal','/alicia_controller/follow_joint_trajectory/result']):
            if topic.endswith('/goal'):
                goals.append(dict(goal_id=message.goal_id.id,goal_receipt_ns=receipt.to_nsec(),
                    source_stamp_ns=message.header.stamp.to_nsec(),
                    exact_commanded_duration_sec=message.goal.trajectory.points[-1].time_from_start.to_sec(),
                    waypoints=len(message.goal.trajectory.points)))
            else:
                results[message.status.goal_id.id]=dict(result_receipt_ns=receipt.to_nsec(),
                    action_status=int(message.status.status),error_code=int(message.result.error_code))
    for goal in goals:
        result=results.get(goal['goal_id'])
        if result:
            goal.update(result)
            goal['goal_receipt_to_result_receipt_wall_sec']=(result['result_receipt_ns']-goal['goal_receipt_ns'])/1e9
        goal['result_observed']=bool(result)
    return goals


def main():
    import rosbag
    parser=argparse.ArgumentParser(description=__doc__)
    group=parser.add_mutually_exclusive_group()
    group.add_argument('--bag',help='explicit closed scene bag')
    group.add_argument('--latest',nargs='?',const='.runtime/latest_candidate_real_grasp.json',help='read current record pointer; default when neither option is supplied')
    parser.add_argument('--output',help='default: session/timing_analysis')
    parser.add_argument('--keyframes',help='optional closed formal keyframes.bag, supplies full action goal/result wall times')
    parser.add_argument('--compare',help='optional prior report.json for separate old/new recorded episode values')
    args=parser.parse_args()
    pointer=None
    if args.bag:
        source=pathlib.Path(args.bag).resolve()
    else:
        pointer=pathlib.Path(args.latest or '.runtime/latest_candidate_real_grasp.json').resolve()
        source=pathlib.Path(json.loads(pointer.read_text())['directory']).resolve()/'scene_diagnostic.bag'
    session=source.parent
    archive=pathlib.Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
    output=pathlib.Path(args.output).resolve() if args.output else session/'timing_analysis'
    if not archive.parent.parent.is_mount() or archive not in output.parents:
        raise ValueError('output must be on the mounted SanStand archive; no fallback')
    closure_path=session/'scene_recorder_result.json'
    closure=json.loads(closure_path.read_text()) if closure_path.exists() else {}
    if closure.get('closed') is not True:
        raise ValueError('scene recorder has not confirmed closed; wait for closure instead of analyzing a live bag')
    states,logs=[],[]
    with rosbag.Bag(str(source)) as bag:
        metadata=dict(start=bag.get_start_time(),end=bag.get_end_time(),file_size_bytes=source.stat().st_size)
        for topic,message,receipt in bag.read_messages(topics=['/grasp/state','/rosout_agg']):
            if topic=='/grasp/state':
                states.append(dict(receipt_ns=receipt.to_nsec(),value=plain(message)))
            elif message.name in ('/grasp_task_node','/motion_gateway') and re.search(r'pregrasp|feedforward|compensation|retimed|submitted trajectory|execution result|settled|Measured rich|Measured contact|Lift continuity|Continuous.*speed audit|Frozen observation continuous path audit',message.msg,re.I):
                logs.append(dict(stamp_ns=message.header.stamp.to_nsec(),receipt_ns=receipt.to_nsec(),node=message.name,text=message.msg))
    analysis=summarize(logs,states,int(metadata['start']*1e9))
    actions=action_records(pathlib.Path(args.keyframes).resolve()) if args.keyframes else []
    for episode in analysis['feedforward_episodes']:
        if 'submit_stamp_ns' not in episode:continue
        matching=[a for a in actions if 0 <= a['goal_receipt_ns']-episode['submit_stamp_ns'] <= 1_000_000_000
                  and abs(a['exact_commanded_duration_sec']-episode['submitted_duration_sec_rounded']) <= .001]
        if len(matching)==1:episode['action_evidence']=matching[0]
        else:episode['action_evidence_status']='unavailable_or_ambiguous; no exact action wall duration asserted'
    physical_path=session/'physical_result_user_confirmed.json'
    result=dict(schema='closed_grasp_timing_audit_v2',source=str(source),pointer=str(pointer) if pointer else None,
        bag=metadata,closure=closure,scope='offline recorded timing and model-derived endpoint error only',
        physical_confirmation=json.loads(physical_path.read_text()) if physical_path.exists() else None,
        physical_confirmation_status='recorded_this_session' if physical_path.exists() else 'not_recorded; do not infer success',
        state_source_timestamps=states,analysis=analysis,action_records=actions,
        keyframes_source=args.keyframes,motion_authorized=False)
    if args.compare:
        old_path=pathlib.Path(args.compare).resolve();old=json.loads(old_path.read_text())
        old_analysis=old.get('analysis')
        if old_analysis is None:
            old_logs=json.loads((old_path.parent/'selected_rosout.json').read_text())
            old_analysis=summarize(old_logs,old['state_source_timestamps'],int(old['bag']['start']*1e9))
        result['comparison']=dict(prior_report=str(old_path),prior_episodes=old_analysis['feedforward_episodes'],
            current_episodes=analysis['feedforward_episodes'],different_trials_not_controlled_identical_motion=True,
            note='Compare commanded and wall columns separately; unavailable/missing phases remain missing, no T/4 inference.')
    output.mkdir(parents=True,exist_ok=True)
    (output/'report.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
    (output/'selected_rosout.json').write_text(json.dumps(logs,ensure_ascii=False,indent=2)+'\n')
    compact=[{k:v for k,v in e.items() if k not in ('proposal','measured_result')} for e in analysis['feedforward_episodes']]
    print(json.dumps(dict(output=str(output),feedforward_episodes=compact,phase_coverage=analysis['feedforward_phase_coverage'],settles=analysis['settles'],physical_confirmation_status=result['physical_confirmation_status']),ensure_ascii=False,indent=2))


if __name__=='__main__':
    main()
