"""Freeze the three completed receiver observations; not a control operation."""
import collections, hashlib, json
from pathlib import Path
from summarize_rgbd_delivery_load_20261006 import numeric
EXT=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
BASE=EXT/'ros_log/gen16_perception_load_20261006'
PATHS=[EXT/'ros_log/grasp_supplement_view_20261006T092959Z/transport_observer_v1',BASE/'transport_observer_v2',BASE/'transport_observer_v3']

def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()
def window(path):
    manifest=json.loads((path/'manifest.json').read_text())
    events=[json.loads(s) for s in (path/'events.jsonl').read_text().splitlines()]
    statuses=[r for r in events if r['kind']=='unknown_status']
    frames={r.get('frame_timing',{}).get('source_stamp_ns'):r.get('frame_timing',{})for r in statuses}
    report=dict(path=str(path),started_wall_ns=manifest['started_wall_ns'],
        finished_wall_ns=manifest['started_wall_ns']+round(manifest['elapsed_sec']*1e9),
        elapsed_sec=manifest['elapsed_sec'],artifact_sha256={k:sha(path/k)for k in ('manifest.json','events.jsonl','passive.jsonl')},
        status_counts=[dict(generation=k[0],state=k[1],detail=k[2],count=v)for k,v in collections.Counter((r.get('generation'),r.get('state'),r.get('detail'))for r in statuses).items()],streams={})
    for stream in ('color','depth'):
        raw=[r for r in events if r['kind']=='image_header' and r['stream']==stream]
        by={r['source_stamp_ns']:r for r in raw}
        u=[f['raw_arrivals'][stream]for f in frames.values() if stream in f.get('raw_arrivals',{})]
        matched=[r for r in u if r['source_stamp_ns']in by]
        report['streams'][stream]=dict(independent_age_sec=numeric([r['source_age_sec']for r in raw]),
            independent_receive_gap_sec=numeric([b['receive_monotonic']-a['receive_monotonic']for a,b in zip(raw,raw[1:])]),
            unknown_matching_age_sec=numeric([r['source_age_at_arrival_sec']for r in matched]),
            unknown_minus_independent_receive_sec=numeric([(r['arrival_ros_ns']-by[r['source_stamp_ns']]['receive_wall_ns'])*1e-9 for r in matched]))
    return report,events,statuses

def main():
    assert Path('/media/zhuyupei/SanStand').is_mount()
    windows=[]
    for p in PATHS:
        w,e,s=window(p);windows.append(w)
    gen16=[r for r in s if r.get('generation')==16]
    loss=next(r for r in gen16 if r.get('detail')=='source_qualification_rejected: source_exceeds_existing_frame_limit')
    good=[r for r in gen16 if r.get('state')=='ready' and r['receive_wall_ns']<loss['receive_wall_ns']][-1]
    pending=[r for r in gen16 if good['receive_wall_ns']<r['receive_wall_ns']<loss['receive_wall_ns']]
    failure=dict(last_good_source_ns=good['source_stamp_ns'],rejected_source_ns=loss['source_stamp_ns'],
        source_delta_sec=(loss['source_stamp_ns']-good['source_stamp_ns'])*1e-9,
        observer_received_failure_status_wall_ns=loss['receive_wall_ns'],
        rejected_frame_observer_arrivals=[r for r in e if r['kind']=='image_header' and r['source_stamp_ns']==loss['source_stamp_ns']],
        preceding_pending=[dict(source_stamp_ns=r['source_stamp_ns'],detail=r['detail'],frame_timing=r.get('frame_timing'))for r in pending],
        log_first_loss_wall_sec=1791280699.666391,
        conclusion='This event followed three consecutive support-pending processed frames, then the new source exceeded the last measured source + existing 0.8-second limit. It did not reproduce the earlier RGB ingress backlog.',
        unavailable_evidence='The header observer omitted full support diagnostics and RGB-D pixels. Production log contains the first-loss source/anchor/reason and throttled inlier_ratio_low strings, not each pending ring ratio/residual/coverage. No exact geometric diagnosis can be reconstructed from this observation alone.')
    load=json.loads((BASE/'tracked_load_interval.json').read_text())
    overlap=[dict(window=w['path'],overlap_sec=max(0,min(w['finished_wall_ns'],load['end_wall_ns'])-max(w['started_wall_ns'],load['start_wall_ns']))*1e-9)for w in windows]
    first_result=BASE/'offline_candidates_v1/result.json'
    result=dict(schema=1,windows=windows,gen16_failure=failure,tracked_named_load=load,tracked_named_load_observer_overlap=overlap,
        tracked_named_load_actual_state='The recorded load started after generation16 had already lost identity and after observer_v3 ended. The directory name is not evidence of tracked overlap.',
        first_offline_load=dict(path=str(first_result),recorded_wall_interval=None,result_file_mtime_ns=first_result.stat().st_mtime_ns,
            limitation='No start/end wall interval was retained. File mtime occurs after observer_v2 and is not used as a substitute for execution timing; overlap is unproven.'),
        historical_gen15=dict(reference=str(EXT/'ros_log/grasp_supplement_view_20261006T092959Z/ingress_stale_diagnostic/report.json'),
            finding='At source1791279029636935234, depth arrived169.263ms old, RGB744.673ms old (575.415ms unequal ingress); the later source was rejected before TF/segmentation. That event remains intermittently unlocated between publisher connection and unknown receive/deserialization/scheduling.',
            not_proven_resolved=True),
        diagnostic_rpc_limitation='getBusInfo identified color/depth TCP connections. getBusStats overflowed its XMLRPC int32 cumulative byte counters for long-running nodes; v3 recorded the first error then avoided repeating that RPC. ss native counters remained available.',
        limitations=['Extra full RGB-D subscriptions increase transport load; none of these healthy windows proves a historical intermittent stall is fixed.',
            'Sampled socket queues and process schedstat do not prove a blocking cause without a coincident delayed source.',
            'Only status-sampled raw callback timing is available from unknown; the independent observer records all its own sources.',
            'No verified simultaneous tracked refinement plus candidate-compute load interval was obtained.',
            'No production source, ROS configuration, hardware state, enable state or quality deadline was changed by this diagnostic.'],
        source_sha256=sha(Path(__file__)))
    output=BASE/'comparison.json'
    if output.exists():raise RuntimeError('preserve prior comparison')
    output.write_text(json.dumps(result,ensure_ascii=False,indent=2,allow_nan=False)+'\n')
    print(json.dumps({'output':str(output),'sha256':sha(output),'source_delta_sec':failure['source_delta_sec'],'overlap':overlap},ensure_ascii=False))
if __name__=='__main__':main()
