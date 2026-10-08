#!/usr/bin/env python3
"""Read sealed metadata only. No ROS, pixel decoding, SDK or device access."""
import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path

import numpy as np

STREAMS=('color','depth')


def digest(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()


def stats(values):
    a=np.asarray(values,dtype=float)
    return dict(n=len(a),percentiles=dict(zip(('min','p50','p95','max'),
        np.percentile(a,[0,50,95,100]).tolist()))) if len(a) else dict(n=0)


def stream_delta(previous,current):
    a,b=previous,current
    fields=('frame_timestamp','actual_fps','backend_timestamp','time_of_arrival')
    return dict(previous_frame_number=a['frame_number'],frame_number=b['frame_number'],
        count_step=b['frame_number']-a['frame_number'],
        device_dt_ms=b['device_timestamp_ms']-a['device_timestamp_ms'],
        previous_device_timestamp_ms=a['device_timestamp_ms'],device_timestamp_ms=b['device_timestamp_ms'],
        domain_unchanged=a['timestamp_domain']==b['timestamp_domain'],
        profile_unchanged=a['profile_signature']==b['profile_signature'],
        metadata_raw_previous={f:a['metadata_raw'].get(f) for f in fields},
        metadata_raw_current={f:b['metadata_raw'].get(f) for f in fields},
        metadata_raw_deltas={f:b['metadata_raw'][f]-a['metadata_raw'][f]
            for f in fields if f!='actual_fps' and f in a['metadata_raw'] and f in b['metadata_raw']})


def analyze(folder):
    path=folder/'read_diagnostics.jsonl'
    sealed=json.loads((folder/'read_diagnostics.summary.json').read_text())
    assert sealed['original_read_restored'] and not sealed['errors'] and sealed['queued_unwritten']==0
    rows=[json.loads(line) for line in path.open()]
    assert len(rows)==sealed['counts']['written']==sealed['counts']['enqueued']==sealed['counts']['observed']
    complete=[];resets=[];rejects=[];history=[];segments=[];segment=[];prior=None
    previous_seen={};unique_streams={s:[] for s in STREAMS};unique_deltas={s:[] for s in STREAMS}
    center_reasons=Counter();shadow_reasons=Counter();classified=Counter();pending_rejects=[]
    for i,row in enumerate(rows):
        d=row['existing_read_diagnostics'];raw=d.get('raw_frames',{});ev=d.get('regional_stationary_evidence',{})
        decision=d.get('stationary_pair_history')
        if decision:history.append(dict(row=i,state=decision['state'],reason=decision['reason']))
        for s in STREAMS:
            current=raw[s];p=previous_seen.get(s)
            if p is None or current['frame_number']!=p['frame_number'] or current['device_timestamp_ms']!=p['device_timestamp_ms']:
                unique_streams[s].append(dict(row=i,source=current))
                if p is not None:unique_deltas[s].append(dict(row=i,**stream_delta(p,current)))
                previous_seen[s]=current
        if d['result']!='complete':
            item=dict(row=i,result=d['result'],pair_timing=d['pair_timing'],raw_frames=raw,
                previous_completed_raw_frames=d.get('previous_completed_raw_frames'),history=decision,
                total_ms=d['total_sec']*1000,wait_ms=d['host_wait']['elapsed_sec']*1000)
            rejects.append(item);pending_rejects.append(item)
            if decision is None or decision['state']!='quarantined':
                if segment:segments.append(segment);segment=[]
            continue
        complete.append((i,row))
        center=ev['regions'][0];center_reasons[center['visual']['reason']]+=1
        shadow_reasons[ev['nested_center_shadow']['reason']]+=1
        for item in pending_rejects:
            item['next_complete_row']=i
            item['next_complete_history']=decision
            item['next_complete_center_qualified']=center['qualified']
            old=item['previous_completed_raw_frames']['raw_frames']
            item['next_complete_source_delta']={s:stream_delta(old[s],raw[s]) for s in STREAMS}
        pending_rejects=[]
        if ev.get('global_history_reset_reason') and segment:
            segments.append(segment);segment=[]
        segment.append(dict(row=i,host=d['stationary_pair_filter_host_monotonic'],
            color_device_ms=raw['color']['device_timestamp_ms'],qualified=center['qualified']))
        if prior is not None:
            j,pd,pr=prior
            assert d['previous_completed_raw_frames']['raw_frames']==pd['raw_frames']
            if ev.get('global_history_reset_reason')=='sdk_source_discontinuity':
                delta={s:stream_delta(pd['raw_frames'][s],raw[s]) for s in STREAMS}
                host_ms=(d['stationary_pair_filter_host_monotonic']-pd['stationary_pair_filter_host_monotonic'])*1000
                cause=[]
                if delta['color']['count_step']!=1:cause.append('color_frame_counter_step')
                if not 1000*.25/raw['color']['stream_fps']<=delta['color']['device_dt_ms']<=1000*1.75/raw['color']['stream_fps']:
                    cause.append('color_device_interval')
                if not 0<host_ms<=100:cause.append('filter_host_interval')
                classified['+'.join(cause)]+=1
                resets.append(dict(row=i,previous_complete_row=j,intervening_read_count=i-j-1,
                    source_delta=delta,exact_previous_raw_frames=pd['raw_frames'],exact_current_raw_frames=raw,
                    gate_reasons=cause,filter_host_interval_ms=host_ms,
                    previous_processing_ms=(pd['total_sec']-pd['host_wait']['elapsed_sec'])*1000,
                    previous_stages_sec=pd['stages_sec'],current_stages_sec=d['stages_sec'],
                    current_wait_ms=d['host_wait']['elapsed_sec']*1000,
                    current_wait_cpu_ms=d['host_wait']['thread_cpu_elapsed_sec']*1000,
                    between_read_gap_ms=(row['read_started_monotonic']-pr['read_completed_monotonic'])*1000,
                    between_wait_wall_ms=(d['host_wait']['enter']['monotonic_sec']-pd['host_wait']['exit']['monotonic_sec'])*1000,
                    between_wait_thread_cpu_ms=(d['host_wait']['enter']['thread_cpu_sec']-pd['host_wait']['exit']['thread_cpu_sec'])*1000))
        prior=i,d,row
    if segment:segments.append(segment)
    stable=[]
    for group in segments:
        stable.append(dict(first_row=group[0]['row'],last_row=group[-1]['row'],complete_frames=len(group),
            host_span_sec=group[-1]['host']-group[0]['host'],
            device_span_sec=(group[-1]['color_device_ms']-group[0]['color_device_ms'])/1000,
            center_qualified_frames=sum(r['qualified'] for r in group)))
    groups={}
    for name,predicate in (
        ('color_counter_plus_one_slow',lambda r:r['source_delta']['color']['count_step']==1),
        ('both_counters_plus_two',lambda r:all(r['source_delta'][s]['count_step']==2 for s in STREAMS)),
        ('color_plus_two_depth_plus_one',lambda r:r['source_delta']['color']['count_step']==2 and r['source_delta']['depth']['count_step']==1)):
        group=[r for r in resets if predicate(r)]
        groups[name]=dict(rows=[r['row'] for r in group],count=len(group),
            streams_count_steps={s:dict(Counter(r['source_delta'][s]['count_step'] for r in group)) for s in STREAMS},
            previous_processing_ms=stats([r['previous_processing_ms']for r in group]),
            previous_processing_over_nominal_33ms=sum(r['previous_processing_ms']>1000/30 for r in group),
            current_wait_ms=stats([r['current_wait_ms']for r in group]),
            current_wait_cpu_ms=stats([r['current_wait_cpu_ms']for r in group]),
            between_wait_wall_ms=stats([r['between_wait_wall_ms']for r in group]),
            between_wait_thread_cpu_ms=stats([r['between_wait_thread_cpu_ms']for r in group]),
            device_dt_ms={s:stats([r['source_delta'][s]['device_dt_ms']for r in group])for s in STREAMS},
            raw_frame_timestamp_delta={s:stats([r['source_delta'][s]['metadata_raw_deltas']['frame_timestamp']for r in group])for s in STREAMS},
            raw_actual_fps={s:stats([r['source_delta'][s]['metadata_raw_current']['actual_fps']for r in group])for s in STREAMS})
    summary=dict(rows=len(rows),results=dict(Counter(r['existing_read_diagnostics']['result']for r in rows)),
        global_resets=dict(Counter(str(r['existing_read_diagnostics'].get('regional_stationary_evidence',{}).get('global_history_reset_reason')) for i,r in complete)),
        reset_gate_reasons=dict(classified),reset_groups=groups,
        history_decisions=dict(Counter(r['state']+'/'+r['reason']for r in history)),
        center_reasons=dict(center_reasons),shadow_reasons=dict(shadow_reasons),
        continuity_segments=stable,longest_segment=max(stable,key=lambda r:r['complete_frames']),
        complete_processing_ms=stats([(r['existing_read_diagnostics']['total_sec']-r['existing_read_diagnostics']['host_wait']['elapsed_sec'])*1000 for i,r in complete]),
        stages_ms={k:stats([r['existing_read_diagnostics']['stages_sec'].get(k,0)*1000 for i,r in complete])
            for k in ('raw_frame_diagnostics','align','depth_filter_0_spatial','regional_stationary_depth','pair_rectification')},
        unique_stream_statistics={s:dict(frames=len(unique_streams[s]),
            steps=dict(Counter(r['count_step']for r in unique_deltas[s])),
            device_dt_ms=stats([r['device_dt_ms']for r in unique_deltas[s]]),
            long_plus_one_rows=[r['row']for r in unique_deltas[s] if r['count_step']==1 and r['device_dt_ms']>1000*1.75/30])for s in STREAMS})
    return dict(schema=1,read_only=True,input=str(path),input_sha256=digest(path),
        analysis_source_sha256=digest(__file__),summary=summary,reset_events=resets,
        rejected_pairs=rejects,history_decisions=history,unique_source_intervals=unique_deltas,
        limitations=[
            'Only exact saved source metadata; no nearest-frame replacement or raw image decoding.',
            'get_frame_number is the SDK counter. Native frame_counter metadata is unsupported here; cannot prove where a missing frame was lost.',
            'SDK global_time timestamps are not independently calibrated exposure timestamps. Native frame_timestamp deltas are retained as raw SDK integers.',
            'Actual exposure, gain and automatic-exposure mode were not recorded. A doubled source interval cannot establish exposure, USB or firmware causality.',
            'Previous processing duration excludes SDK wait; between-wait CPU includes application/observer work. Correlation is not proof of device-side causality.',
            'Tap diagnostic workload was enabled in this capture; these measurements are not an uninstrumented performance baseline.'])


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('folder',type=Path)
    parser.add_argument('--output',type=Path);args=parser.parse_args()
    report=analyze(args.folder)
    if args.output:
        mount=Path('/media/zhuyupei/SanStand')
        assert mount.is_mount() and mount in args.output.resolve().parents
        args.output.parent.mkdir(parents=True,exist_ok=True)
        with args.output.open('x')as f:json.dump(report,f,ensure_ascii=False,indent=2,allow_nan=False)
    print(json.dumps(report['summary'],ensure_ascii=False,indent=2,allow_nan=False))
