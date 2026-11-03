#!/usr/bin/env python3
"""Read finalized camera metadata only; never starts ROS or opens devices."""
import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path
import sys

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src/alicia_flexible_grasp_supervisor/src'))
from alicia_flexible_grasp.vision.pair_history_continuity import interleaved_pair_reason, successor_pair_reason


def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    p=argparse.ArgumentParser();p.add_argument('--run',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True);args=p.parse_args()
    summary=json.loads((args.run/'read_diagnostics.summary.json').read_text())
    assert summary['original_read_restored'] and not summary['errors'] and summary['queued_unwritten']==0
    rows=[json.loads(line) for line in (args.run/'read_diagnostics.jsonl').read_text().splitlines()]
    c=summary['counts'];assert c['written']==c['enqueued']==c['observed']==len(rows)
    assert not any(c.get(k,0) for k in ('queue_dropped','copy_errors','serialization_errors','write_errors'))
    ext=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller').resolve()
    assert Path('/media/zhuyupei/SanStand').is_mount() and ext in args.output.resolve().parents
    assert not args.output.exists()
    events=[];violations=[];states=Counter();reasons=Counter();filtered_after_resume=0
    for index,row in enumerate(rows):
        d=row['existing_read_diagnostics'];decision=d.get('stationary_pair_history')
        # runtime_profile retains the last decision. Only per-read decisions
        # count as events; do not count their later telemetry copies again.
        if not isinstance(decision,dict):continue
        state=decision['state'];states[state]+=1;reasons[decision['reason']]+=1
        previous=decision.get('last_complete_raw_frames') or {}
        current=(decision.get('next_accepted_raw_frames') if state=='resumed_for_existing_gates'
                 else d.get('raw_frames')) or {}
        event=dict(row=index,state=state,reason=decision['reason'],result=d['result'],
            rejected_packet_count=decision.get('rejected_packet_count'),
            captured_wall_time_ns=row['captured_wall_time_ns'],per_stream={})
        for stream in ('color','depth'):
            before,after=previous.get(stream,{}),current.get(stream,{})
            if all(key in before and key in after for key in ('frame_number','device_timestamp_ms')):
                event['per_stream'][stream]=dict(previous_number=before['frame_number'],
                    current_number=after['frame_number'],frame_increment=after['frame_number']-before['frame_number'],
                    device_interval_ms=after['device_timestamp_ms']-before['device_timestamp_ms'],
                    profile_unchanged=before.get('profile_signature')==after.get('profile_signature'),
                    domain_unchanged=before.get('timestamp_domain')==after.get('timestamp_domain'))
        if state=='quarantined':
            forbidden=[stage for stage in d['stages_sec'] if stage=='align' or stage.startswith('depth_filter_')
                       or stage in ('regional_stationary_depth','stationary_temporal_process','pair_rectification')]
            event.update(processing_stages_entered=forbidden,pair_accepted=d.get('pair_timing',{}).get('accepted'))
            if forbidden or d['result']!='rejected_pair' or d['pair_timing']['accepted'] is not False:
                violations.append(dict(row=index,reason='rejected_pair_reached_processing'))
            actual_previous=d.get('previous_completed_raw_frames',{}).get('raw_frames')
            event['bound_to_actual_last_complete']=actual_previous==previous
            if actual_previous!=previous:violations.append(dict(row=index,reason='last_complete_binding_mismatch'))
            reason=interleaved_pair_reason(previous,decision.get('rejected_raw_frames'),
                decision.get('last_complete_filter_host_monotonic'),decision.get('first_rejection_host_monotonic'))
            event['first_rejection_original_source_check']=reason
            if reason is not None:violations.append(dict(row=index,reason='quarantined_source_contract_failed:'+reason))
        if state=='resumed_for_existing_gates':
            host=d.get('stationary_pair_filter_host_monotonic')
            original=decision.get('last_complete_filter_host_monotonic')
            check=successor_pair_reason(previous,current,original,host)
            gate=decision.get('existing_gate_evaluation',{})
            event.update(actual_filter_host_span_sec=None if host is None or original is None else host-original,
                successor_check_at_actual_filter_host=check,existing_gate_evaluation=gate,
                current_raw_matches_decision=current==d.get('raw_frames'))
            filtered=gate.get('filtered_pixel_count',0)
            filtered_after_resume+=int(filtered>0)
            if (current!=d.get('raw_frames') or set(event['per_stream'])!={'color','depth'}
                    or any(v['frame_increment']!=1 for v in event['per_stream'].values())):
                violations.append(dict(row=index,reason='next_accepted_source_not_current_immediate_successor'))
            if filtered and (check is not None or gate.get('encoder_active') is not True
                             or gate.get('alignment_proven') is not True):
                violations.append(dict(row=index,reason='history_output_without_current_existing_gates'))
        events.append(event)
    report=dict(schema_version=1,run=str(args.run),read_count=len(rows),
        source_sha256=sha(Path(__file__)),input_sha256=sha(args.run/'read_diagnostics.jsonl'),
        metadata_summary_sha256=sha(args.run/'read_diagnostics.summary.json'),
        state_counts=dict(states),reason_counts=dict(reasons),
        actual_resume_events_with_filtered_pixels=filtered_after_resume,violations=violations,events=events,
        scope='Original completed 30-second metadata; no pixel or support-quality/grasp proof; no inference from earlier gen17',
        motion_or_ros_commands_sent=False)
    args.output.write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    print(json.dumps({key:report[key] for key in ('read_count','state_counts','reason_counts',
        'actual_resume_events_with_filtered_pixels','violations')},ensure_ascii=False))


if __name__=='__main__':main()
