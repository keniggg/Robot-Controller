"""Offline exact-source metadata association. No image decoding or ROS calls."""
from collections import Counter
import hashlib
import json
from pathlib import Path
import zipfile

EXT=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log')
RAW=EXT/'gen20_support_measurement_20261006/raw_support'
META=EXT/'gen20_support_metadata_20261006_v2'

def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()
def read(path):return json.loads(path.read_text())
def center(entry):
    return next(r for r in entry['evidence']['regional_stationary_evidence']['regions'] if r['name']=='center')

def compact(entry):
    e=entry['evidence'];r=center(entry);v=r['visual'];raw=e['raw_frames']
    return dict(source_stamp_ns=entry['source_stamp_ns'],geometry_generation=entry['geometry_generation'],
        geometry_activation_stamp_ns=entry['geometry_activation_stamp_ns'],
        read_completed_monotonic=entry['read_completed_monotonic'],
        global_history_reset_reason=e['regional_stationary_evidence'].get('global_history_reset_reason'),
        filtered_fraction=e['regional_stationary_evidence'].get('filtered_fraction'),
        center={k:r.get(k) for k in ('qualified','warming','history_reset','filter_applied')},
        visual={k:v.get(k) for k in ('reason','frame_number','device_timestamp_ms','motion_p90_px',
            'motion_median_px','quiet_span_sec','host_quiet_span_sec','continuous_frames','tracked_points','covered_quadrants')},
        raw_frames={s:{k:raw[s].get(k) for k in ('frame_number','device_timestamp_ms','stream_fps','timestamp_domain')} for s in ('color','depth')},
        pair_timing=e.get('pair_timing'),pair_history=e.get('stationary_pair_history'),
        read_total_sec=e.get('total_sec'),stages_sec=e.get('stages_sec'),
        stationary_pair_filter_host_monotonic=e.get('stationary_pair_filter_host_monotonic'))

def main():
    ms,rs=read(META/'summary.json'),read(RAW/'summary.json')
    assert ms['sealed'] and not ms['error'] and not ms['metadata_errors']
    assert rs['archive_closed'] and rs['complete_without_writer_error'] and not rs['error']
    assert sha(META/'metadata.jsonl')==ms['jsonl_sha256']
    entries={};appearances={};statuses=[];duplicates=0
    for line_no,line in enumerate((META/'metadata.jsonl').open(),1):
        row=json.loads(line)
        if row['kind']=='detector_status':
            s=json.loads(row['original_status_data'])
            if s.get('generation')==20:statuses.append((s,line_no,row))
        elif row['kind']=='camera_read_diagnostics_parameter':
            payload=json.loads(row['original_parameter_value']['payload_json'])
            for e in payload['entries']:
                ns=e['source_stamp_ns'];assert type(ns) is int
                if ns in entries:
                    assert entries[ns]==e,'same source has conflicting evidence'
                    duplicates+=1
                entries[ns]=e;appearances.setdefault(ns,[]).append(line_no)
    lost=next(s for s,_,_ in statuses if s.get('first_loss'))['first_loss']
    stop=lost['source_stamp_ns'];start=lost['anchor_source_stamp_ns'];lastgood=lost['last_good_source_stamp_ns']
    beforeloss=[]
    with zipfile.ZipFile(RAW/'diagnostic.zip') as z:
        names=set(z.namelist());raw_statuses={}
        for name in sorted(n for n in names if n.startswith('status/') and n.endswith('.json')):
            r=json.loads(z.read(name));s=json.loads(r['original_status_data'])
            if s.get('generation')==20 and type(s.get('source_stamp_ns')) is int:
                raw_statuses.setdefault(s['source_stamp_ns'],[]).append((name,r))
        for s,line_no,original in statuses:
            ns=s.get('source_stamp_ns')
            if type(ns) is not int or not start<=ns<=stop:continue
            assert s['color_stamp_ns']==s['depth_stamp_ns']==s['tf_lookup_stamp_ns']==ns
            exact=entries.get(ns);q=s.get('source_qualification') or {};timing=s.get('frame_timing') or {}
            matching=[n for n,r in raw_statuses.get(ns,[]) if r['original_status_data']==original['original_status_data']]
            row=dict(source_stamp_ns=ns,state=s['state'],target_lost=s['target_lost'],
                detail=s.get('detail'),source_qualification=q,first_loss=s.get('first_loss'),
                exact_camera_evidence_found=exact is not None,observer_status_line=line_no,
                observer_parameter_lines=appearances.get(ns,[]),identical_raw_status_members=matching,
                exact_original_pair_saved=('pairs/%d.json'%ns in names),
                frame_timing={k:timing.get(k) for k in ('ingress_source_age_sec','raw_arrival_difference_sec',
                    'input_lock_wait_sec','processing_wall_sec','processing_thread_cpu_sec','stage_wall_sec')},
                camera=None if exact is None else compact(exact))
            beforeloss.append(row)
    continuous=[compact(entries[n]) for n in sorted(entries) if start<=n<=stop]
    resets=[r for r in continuous if r['visual']['reason']!='warming_up' and (
        r['visual']['reason']!='stationary' or r['global_history_reset_reason'] is not None)]
    stats={}
    for stream in ('color','depth'):
        deltas=[b['raw_frames'][stream]['frame_number']-a['raw_frames'][stream]['frame_number'] for a,b in zip(continuous,continuous[1:])]
        intervals=[b['raw_frames'][stream]['device_timestamp_ms']-a['raw_frames'][stream]['device_timestamp_ms'] for a,b in zip(continuous,continuous[1:])]
        stats[stream]=dict(frame_number_deltas=dict(Counter(deltas)),device_interval_ms_min=min(intervals),device_interval_ms_max=max(intervals))
    intervals=[b['stationary_pair_filter_host_monotonic']-a['stationary_pair_filter_host_monotonic'] for a,b in zip(continuous,continuous[1:])]
    report=dict(schema=1,scope='Offline exact source metadata association only; no image decoding or device/ROS operations.',
        inputs=dict(metadata_jsonl=str(META/'metadata.jsonl'),metadata_sha256=ms['jsonl_sha256'],
            metadata_summary_sha256=sha(META/'summary.json'),raw_summary_sha256=sha(RAW/'summary.json'),
            diagnostic_zip=str(RAW/'diagnostic.zip')),
        first_loss=lost,source_delta_from_last_good_sec=(stop-lastgood)/1e9,
        observed_delta_from_last_good_sec=(lost['observed_stamp_ns']-lastgood)/1e9,
        unique_read_sources=len(entries),identical_repeated_sources=duplicates,
        all_status_rows_exact_camera_match=all(r['exact_camera_evidence_found'] for r in beforeloss),
        all_status_rows_match_original_zip=all(r['identical_raw_status_members'] for r in beforeloss),
        sdk_continuity=stats,filter_host_interval_sec_min=min(intervals),filter_host_interval_sec_max=max(intervals),
        center_reasons=dict(Counter(r['visual']['reason'] for r in continuous)),
        global_reset_reasons=dict(Counter(str(r['global_history_reset_reason']) for r in continuous)),
        source_qualified_status_timeline=beforeloss,all_successful_read_timeline=continuous,
        center_reset_events=resets,
        limitations=['A camera entry is a successful read candidate, not proof of publication. Exact matching received detector RGB/depth/tf stamps and recorder original source establishes the association here.',
            'Full 30Hz successful read evidence is inspected, including reads not chosen by the 20Hz publisher; no nearest-source substitution is used.',
            'Visual motion metrics are optical-flow evidence, not proof of robot physical motion. Encoder/scene/light/flow error causes require separate evidence.',
            'Ready frames can still pass measured support while center filtering is warming. Temporal filtering state correlates with these failures but is not a mathematical explanation of every plane rejection.',
            'No depth pixels were read in this report; support-plane inlier ratios must come from a separate exact-frame replay.'])
    output=EXT/'gen20_support_measurement_20261006/exact_source_filter_evidence.json'
    with output.open('x') as f:json.dump(report,f,ensure_ascii=False,indent=2,allow_nan=False);f.write('\n')
    print(json.dumps(dict(output=str(output),sha256=sha(output),statuses=len(beforeloss),
        exact_camera_match=report['all_status_rows_exact_camera_match'],zip_match=report['all_status_rows_match_original_zip'],
        first_loss=lost,sdk_continuity=stats,filter_host_interval_sec_max=max(intervals),
        center_reasons=report['center_reasons'],reset_events=[dict(source_stamp_ns=r['source_stamp_ns'],
            reason=r['visual']['reason'],p90=r['visual']['motion_p90_px'],median=r['visual']['motion_median_px']) for r in resets]),ensure_ascii=False,indent=2))

if __name__=='__main__':main()
