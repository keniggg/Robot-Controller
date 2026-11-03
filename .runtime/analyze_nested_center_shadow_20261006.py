"""Metadata-only same-read parent/nested comparison; no live ROS or pixels."""
import argparse
from collections import Counter
import hashlib
import json
import math
from pathlib import Path
import statistics

EXT=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')

def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def read(p):return json.loads(p.read_text())
def values(rows,side,key):
    return [r[side]['visual'][key] for r in rows if isinstance(r[side].get('visual'),dict)
            and type(r[side]['visual'].get(key)) in (int,float) and math.isfinite(r[side]['visual'][key])]
def distribution(v):
    if not v:return dict(count=0)
    v=sorted(v)
    return dict(count=len(v),min=v[0],p50=statistics.median(v),p90=v[round((len(v)-1)*.9)],
                p95=v[round((len(v)-1)*.95)],max=v[-1])

def source_contiguous(a,b):
    if not a or not b:return False
    for s in ('color','depth'):
        x,y=a['raw_frames'][s],b['raw_frames'][s]
        if y['frame_number']!=x['frame_number']+1:return False
        if y['timestamp_domain']!=x['timestamp_domain'] or y['stream_fps']!=x['stream_fps']:return False
        dt=(y['device_timestamp_ms']-x['device_timestamp_ms'])/1000.
        if not .25/y['stream_fps']<=dt<=1.75/y['stream_fps']:return False
    gap=b['filter_host_monotonic']-a['filter_host_monotonic']
    return 0<gap<=max(.1,2.5/b['raw_frames']['color']['stream_fps'])

def runs(rows,flag):
    result=[];active=[];previous=None
    def finish():
        if active:
            a,b=active[0],active[-1]
            result.append(dict(frames=len(active),first_line=a['line'],last_line=b['line'],
                first_color_frame=a['raw_frames']['color']['frame_number'],
                last_color_frame=b['raw_frames']['color']['frame_number'],
                host_span_sec=b['filter_host_monotonic']-a['filter_host_monotonic'],
                device_span_sec=(b['raw_frames']['color']['device_timestamp_ms']-a['raw_frames']['color']['device_timestamp_ms'])/1000.))
    for row in rows:
        if row is None or not flag(row):
            finish();active=[];previous=row;continue
        if active and not source_contiguous(previous,row):finish();active=[]
        active.append(row);previous=row
    finish()
    return dict(count=len(result),longest=max(result,key=lambda r:r['host_span_sec']) if result else None,
                all_runs=result,semantics='Intervals between observed qualifying source samples; rejected/missing reads and either SDK-source discontinuity break the run, with no inferred inter-sample authority.')

def main():
    p=argparse.ArgumentParser();p.add_argument('--deployment',required=True,type=Path)
    p.add_argument('--output',required=True,type=Path);args=p.parse_args()
    path=args.deployment.resolve();out=args.output.resolve()
    assert EXT in path.parents and EXT in out.parents and not out.exists()
    result=read(path/'result.json');summary=read(path/'read_diagnostics.summary.json')
    assert result['camera_loaded'] and result['static_validation_passed'] and not result.get('error')
    assert result['events'][-1]['event']=='finished'
    assert not any(summary.get('counts',{}).get(k,0) for k in ('copy_errors','serialization_errors','write_errors','queue_dropped'))
    counts=Counter();reasons={s:Counter() for s in ('parent','shadow')};pairs=[];timeline=[];rejects=[]
    for line_no,line in enumerate((path/'read_diagnostics.jsonl').open(),1):
        raw=json.loads(line);d=raw['existing_read_diagnostics'];counts['all_reads']+=1
        counts[d['result']]+=1
        e=d.get('regional_stationary_evidence',{});child=e.get('nested_center_shadow')
        parent=next((r for r in e.get('regions',[])if r['name']=='center'),None)
        if d['result']!='complete' or not child or not parent:
            timeline.append(None)
            rejects.append(dict(line=line_no,result=d['result'],pair_timing=d.get('pair_timing'),
                pair_history=d.get('stationary_pair_history'),raw_frames=d.get('raw_frames')))
            continue
        assert child['diagnostic_only'] and child['pixels_changed'] is False
        assert child['used_for_depth_permission'] is False and child['temporal_filter_created'] is False
        source=d['raw_frames']['color']
        for r in (parent,child):
            v=r['visual']
            assert v['frame_number']==source['frame_number']
            assert v['device_timestamp_ms']==source['device_timestamp_ms']
            assert v['window_sec']==.35 and v['max_motion_px']==.25
        pq,sq=parent['qualified']is True,child['qualified']is True
        counts['parent_%s_shadow_%s'%(pq,sq)]+=1
        row=dict(line=line_no,parent=parent,shadow=child,raw_frames=d['raw_frames'],
            captured_wall_time_ns=raw['captured_wall_time_ns'],
            read_started_monotonic=raw['read_started_monotonic'],read_completed_monotonic=raw['read_completed_monotonic'],
            filter_host_monotonic=d['stationary_pair_filter_host_monotonic'],
            global_history_reset_reason=e.get('global_history_reset_reason'),
            read_total_sec=d['total_sec'],regional_stage_sec=d['stages_sec'].get('regional_stationary_depth'))
        for side in reasons:reasons[side][row[side]['visual']['reason']]+=1
        pairs.append(row);timeline.append(row)
    n=len(pairs)
    qualified={s:sum(r[s]['qualified']is True for r in pairs) for s in reasons}
    support=sum(not r['parent']['qualified']and r['shadow']['qualified']for r in pairs)
    report=dict(schema=1,scope='Read-only original 30-second loader metadata; no image processing, ROS calls or production changes.',
        deployment=str(path),input_hashes={n:sha(path/n) for n in ('result.json','read_diagnostics.summary.json','read_diagnostics.jsonl')},
        observer_summary=summary,counts=dict(counts),paired_complete_reads=n,
        qualified={s:dict(frames=qualified[s],fraction=qualified[s]/n,reasons=dict(reasons[s]))for s in reasons},
        parent_unqualified_child_qualified_frames=support,
        fallback_fraction_of_parent_unqualified=support/(n-qualified['parent']) if n!=qualified['parent']else None,
        distributions={s:{k:distribution(values(pairs,s,k))for k in ('motion_p90_px','motion_median_px','quiet_span_sec','continuous_frames','tracked_points','covered_quadrants')}for s in reasons},
        qualifying_runs={s:runs(timeline,lambda r,s=s:r[s]['qualified']is True)for s in reasons},
        fallback_runs=runs(timeline,lambda r:not r['parent']['qualified']and r['shadow']['qualified']),
        regional_stage_sec=distribution([r['regional_stage_sec']for r in pairs]),
        global_resets=dict(Counter(str(r['global_history_reset_reason'])for r in pairs)),
        evidence_supports_implementing_this_nested_fallback=support>0,
        interpretation=('No nested-frame qualification or parent rejection recovery was observed; the proposed nested fallback is not supported by this window.' if not support else
            'Some parent-unqualified reads have qualified nested evidence; this alone does not prove finite pixel ownership or support-plane quality improvement.'),
        limitations=['A parent visual-motion rejection is not established to be false merely because a smaller crop passes; here the nested crop did not pass at all.',
            'Metadata contains source-bound optical-flow magnitudes, not per-feature vectors or enough image evidence to distinguish real scene motion, illumination and estimator noise.',
            'Shadow used the original .25px/.35s and texture/coverage/source gates; it never filtered depth, so no grasp or support-fit improvement can be claimed.',
            'Parent 384x384 input is internally resized to320x320, while nested192x192 is not resized. This follows original generic VisualStationaryWindow; whether resampling/noise or spatial content explains the difference remains unproven.',
            'Full temporal-filter and parent visual results remained production values; there was no concurrent same-input no-shadow process for a causal runtime comparison.'],
        read_timeline=pairs,rejected_or_missing_reads=rejects)
    out.parent.mkdir(parents=True,exist_ok=True)
    with out.open('x')as f:json.dump(report,f,ensure_ascii=False,indent=2,allow_nan=False);f.write('\n')
    print(json.dumps({k:report[k]for k in ('counts','paired_complete_reads','qualified','parent_unqualified_child_qualified_frames','distributions','global_resets','interpretation')},indent=2))
    print(json.dumps(dict(output=str(out),sha256=sha(out),longest_runs={s:report['qualifying_runs'][s]['longest']for s in reasons})))

if __name__=='__main__':main()
