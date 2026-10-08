#!/usr/bin/env python3
"""Offline summaries of original fresh-probe statuses; no ROS imports or effects."""
import argparse
from collections import Counter, defaultdict
import hashlib
import json
from pathlib import Path
import zipfile
import zlib

def digest(raw): return hashlib.sha256(raw).hexdigest()
def load(path): return json.loads(Path(path).read_text())
def limits(rows,key):
    vals=[r[key] for r in rows if isinstance(r.get(key),(float,int))]
    return dict(count=len(vals),minimum=min(vals),maximum=max(vals)) if vals else None

def main():
    p=argparse.ArgumentParser();p.add_argument('--input',required=True,type=Path);p.add_argument('--output',required=True,type=Path);a=p.parse_args()
    require=lambda condition:None if condition else (_ for _ in ()).throw(ValueError('input/output contract'))
    external=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller').resolve()
    require(external in a.input.resolve().parents and external in a.output.resolve().parents and not a.output.exists())
    report=load(a.input/'summary.json');metadata_summary=load(a.input/'metadata/summary.json')
    raw=(a.input/'metadata/metadata.jsonl').read_bytes();require(digest(raw)==metadata_summary['jsonl_sha256'])
    rows=[json.loads(line) for line in raw.splitlines()]
    generation=report['before']['context']['/grasp_mode/selection']['generation']+1
    statuses=[];history=defaultdict(dict)
    for row in rows:
        if row['kind']=='detector_status':
            value=json.loads(row['original_status_data'])
            if value.get('generation')==generation:statuses.append((value,row['receipt']))
        elif row.get('annotation',{}).get('schema_valid'):
            value=json.loads(row['original_parameter_value']['payload_json'])
            for entry in value['entries']:
                history[entry['source_stamp_ns']][digest(json.dumps(entry,sort_keys=True).encode())]=entry
    pairs={};pair_hashes={};raw_statuses=[]
    with zipfile.ZipFile(a.input/'raw_support/diagnostic.zip') as z:
        for name in z.namelist():
            if name.startswith('pairs/') and name.endswith('.json'):
                value=json.loads(z.read(name));stamp=value['source_stamp_ns'];pairs[stamp]=value
                pair_hashes[stamp]={stream:digest(zlib.decompress(z.read('pairs/%d_%s.rosmsg.zlib'%(stamp,stream))))==r['serialized_sha256'] for stream,r in value['streams'].items()}
            elif name.startswith('status/') and name.endswith('.json'):
                value=json.loads(z.read(name));parsed=json.loads(value['original_status_data'])
                if parsed.get('generation')==generation:raw_statuses.append(parsed)
    ready=[s for s,_ in statuses if s.get('state')=='ready']
    matches=[]
    for status in ready:
        stamp=status['source_stamp_ns'];entries=history.get(stamp,{})
        center=[]
        for entry in entries.values():
            center.extend(r for r in entry['evidence'].get('regional_stationary_evidence',{}).get('regions',[]) if r['name']=='center')
        matches.append(dict(source_stamp_ns=stamp,exact_rgbd=status.get('exact_rgbd'),
            equal_rgbd_tf_source=all(status.get(k)==stamp for k in ('color_stamp_ns','depth_stamp_ns','tf_lookup_stamp_ns')),
            raw_pair_saved=stamp in pairs,raw_original_serialized_hashes=pair_hashes.get(stamp),
            exact_read_history_variants=len(entries),center=[{k:r.get(k) for k in ('qualified','warming','filter_applied')} for r in center]))
    metrics=[r.get('geometry_metrics',{}) for r in ready]
    with (a.input/'current_feedback_and_state.jsonl').open() as handle:
        own=[json.loads(l) for l in handle]
    task=[r for r in own if r['kind']=='task'];feedback=[r for r in own if r['kind']=='feedback']
    result=dict(schema=1,diagnostic_only=True,input=str(a.input.resolve()),selected_generation=generation,
        source_tool_at_capture=report['sources'],analysis_script_sha256=digest(Path(__file__).read_bytes()),
        input_summary_sha256=digest((a.input/'summary.json').read_bytes()),input_metadata_jsonl_sha256=digest(raw),
        probe_capture_closed=report['capture_closed'],probe_error=report.get('error'),service_calls=report['service_calls'],
        interpretation='The single select succeeded. The wrapper incorrectly applied a 1-second heartbeat-age rule to latched, change-only inactive GraspState; its timeout interrupted only its recorders. This is not a perception rejection.',
        latched_task_messages=len(task),task_messages_all_inactive=all(r['active'] is False for r in task),
        metadata_generation_statuses=len(statuses),raw_archive_generation_statuses=len(raw_statuses),
        states_and_details=dict(Counter(s.get('state','')+':'+s.get('detail','') for s,_ in statuses)),
        first_loss_reports=[s.get('first_loss') for s,_ in statuses if s.get('first_loss')],
        target_lost_reports=sum(bool(s.get('target_lost')) for s,_ in statuses),
        ready_count=len(ready),ready_source_span_sec=(ready[-1]['source_stamp_ns']-ready[0]['source_stamp_ns'])/1e9 if ready else None,
        ready_exact_source_associations=matches,ready_raw_pair_count=sum(r['raw_pair_saved'] for r in matches),
        all_saved_original_serialized_hashes_valid=all(all(row.values()) for row in pair_hashes.values()),
        original_support_metric_ranges={key:limits(metrics,key) for key in
            ('support_current_audit_inlier_ratio','support_current_audit_residual_m','support_current_audit_quadrants',
             'support_reference_max_distance_m','support_reference_angle_deg','median_height_m','valid_points')},
        accepted_feedback_messages=len(feedback),feedback_positions_distinct=len({tuple(r['positions']) for r in feedback}),
        last_status_source_stamp_ns=ready[-1]['source_stamp_ns'] if ready else None,
        quality_window_complete=False,physical_stationarity_proven=False,cross_view_or_grasp_verified=False,
        limitations=['Only approximately seven seconds of metadata and five seconds of raw recording survived the wrapper error; no full 15-second capture claim.',
         'Read-history entries are successful read candidates, associated here only by exact source with published status; they do not alone guarantee publication.',
         'Recorded ready status is perception output, not independent object identity or grasp success. No plan, grasp or additional selection was requested.',
         'No changed support, visual, source-continuity or freshness thresholds; no old selection support reference was reused.'])
    a.output.mkdir();(a.output/'report.json').write_text(json.dumps(result,ensure_ascii=False,sort_keys=True,indent=2)+'\n')
    print(json.dumps({k:result[k] for k in ('selected_generation','states_and_details','ready_count','ready_raw_pair_count','original_support_metric_ranges')},ensure_ascii=False))
    print('report_sha256='+digest((a.output/'report.json').read_bytes()))
if __name__=='__main__':main()
