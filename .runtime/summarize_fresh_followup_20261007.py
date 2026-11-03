#!/usr/bin/env python3
"""Summarize sealed follow-up evidence only; no ROS, device or motion calls."""
import argparse
from collections import Counter, defaultdict
import hashlib
import json
from pathlib import Path

def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def read(path):return json.loads(Path(path).read_text())
def bounds(values):
    values=[v for v in values if v is not None]
    return dict(count=len(values),minimum=min(values),maximum=max(values)) if values else None

def main():
    p=argparse.ArgumentParser();p.add_argument('--capture',required=True,type=Path);a=p.parse_args();c=a.capture.resolve()
    external=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller').resolve()
    if external not in c.parents:raise ValueError('external capture required')
    meta=read(c/'metadata/summary.json');rawp=c/'metadata/metadata.jsonl'
    if sha(rawp)!=meta['jsonl_sha256']:raise ValueError('metadata digest mismatch')
    rows=[json.loads(l) for l in rawp.read_text().splitlines()];statuses=[];history=defaultdict(dict)
    for row in rows:
        if row['kind']=='detector_status':statuses.append(json.loads(row['original_status_data']))
        elif row.get('annotation',{}).get('schema_valid'):
            for e in json.loads(row['original_parameter_value']['payload_json'])['entries']:
                history[e['source_stamp_ns']][json.dumps(e,sort_keys=True)]=e
    expected=meta['expected_generation'];current=[s for s in statuses if s.get('generation')==expected]
    matched=[]
    for s in current:
        source=s.get('source_stamp_ns');es=history.get(source,{})
        center=[r for e in es.values() for r in e['evidence'].get('regional_stationary_evidence',{}).get('regions',[]) if r['name']=='center']
        matched.append(dict(source_stamp_ns=source,history_variants=len(es),exact_rgbd=s.get('exact_rgbd'),
            rgb_depth_tf_same_source=source is not None and all(s.get(k)==source for k in ('color_stamp_ns','depth_stamp_ns','tf_lookup_stamp_ns')),
            center_qualified=[r['qualified'] for r in center],center_filter_applied=[r['filter_applied'] for r in center]))
    support_path=c/'rgbd/original_support_analysis_v1/report.json';support=read(support_path)
    first_rings=[r['analysis']['rings'][0] for r in support['rows']]
    accepted_rings=[next(q for q in r['analysis']['rings'] if q['outer_base_px']==r['analysis']['selected_outer_base_px']) for r in support['rows']]
    geometries=[s.get('geometry_metrics',{}) for s in current]
    result=dict(schema=1,diagnostic_only=True,capture=str(c),metadata_summary_sha256=sha(c/'metadata/summary.json'),
        metadata_jsonl_sha256=sha(rawp),support_report_path=str(support_path),support_report_sha256=sha(support_path),
        analysis_script_sha256=sha(__file__),expected_generation=expected,capture_duration_sec=meta['capture_duration_sec'],
        completed_requested_metadata_window=meta['sealed'] and meta['reason']=='duration_limit' and not meta['error'],
        status_count=len(current),states_and_details=dict(Counter(s['state']+':'+s['detail'] for s in current)),
        first_loss_count=sum(bool(s.get('first_loss')) for s in current),target_lost_count=sum(bool(s.get('target_lost')) for s in current),
        anchors=sorted({s.get('anchor_source_stamp_ns') for s in current if s.get('anchor_source_stamp_ns')}),
        source_span_sec=(current[-1]['source_stamp_ns']-current[0]['source_stamp_ns'])/1e9,
        exact_source_associations=matched,exact_history_unique_count=sum(r['history_variants']==1 for r in matched),
        current_support_status_ranges={key:bounds(g.get(key) for g in geometries) for key in
            ('support_current_audit_inlier_ratio','support_current_audit_residual_m','support_current_audit_quadrants',
             'support_reference_max_distance_m','support_reference_angle_deg','valid_points','median_height_m')},
        independent_saved_rgbd_pairs=support['saved_pair_count'],independent_original_ring_pass_count=support['quality_pass_count'],
        independent_accepted_ring_inlier_ratio=bounds(r['fit']['ratio'] for r in accepted_rings),
        independent_accepted_ring_residual_m=bounds(r['fit']['residual_m'] for r in accepted_rings),
        independent_accepted_ring_quadrants=bounds(r['quadrants'] for r in accepted_rings),
        independent_selected_outer_base_px=dict(Counter(r['outer_base_px'] for r in accepted_rings)),
        independent_rejected_first_ring_ratios=[r['fit']['original_locals']['ratio'] for r in first_rings if not r['fit']['original_fit_qualified']],
        independent_raw_pair_source_history_unique_count=sum(r['exact_read_history_unique'] for r in support['rows']),
        independent_same_stamp_tf_base_plane_count=sum('plane_base' in r['analysis'] for r in support['rows']),
        independent_first_ring_owner_counts=dict(Counter(k for r in first_rings for k,v in r['actual_region_ownership_valid_points'].items() if v)),
        static_perception_and_current_support_observed_pass=bool(current and all(s['state']=='ready' and not s.get('first_loss') and not s.get('target_lost') for s in current)
            and support['quality_pass_count']==support['saved_pair_count']==10),
        quality_thresholds_changed=False,motion_commands_sent=False,selection_service_called_in_followup=False,
        cross_view_fusion_validated=False,grasp_validated=False,
        limitations=['Static-scene qualification is bounded to these actual recorded windows, not a guarantee of all viewpoints, objects or future frames.',
            'The cadence-sampled 10 RGB-D pairs are independent current sources, not relabelled to match the 151 detector-status sources.',
            'Exact read-history matches identify successful read candidates associated with published sources; no exposure-time calibration is inferred.',
            'No further selection, plan or physical action; the earlier short probe failure remains recorded and was a wrapper heartbeat/latched-state error.',
            'No baseline/counterfactual here isolates which individual filter or environment condition caused the improvement.'])
    path=c/'static_perception_interpretation.json'
    with path.open('x') as f:json.dump(result,f,ensure_ascii=False,sort_keys=True,indent=2);f.write('\n')
    print(json.dumps({k:result[k] for k in ('status_count','states_and_details','exact_history_unique_count','current_support_status_ranges','independent_accepted_ring_inlier_ratio','independent_accepted_ring_residual_m','independent_same_stamp_tf_base_plane_count','static_perception_and_current_support_observed_pass')},ensure_ascii=False))
    print('interpretation_sha256='+sha(path))
if __name__=='__main__':main()
