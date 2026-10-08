#!/usr/bin/env python3
"""Read sealed metadata and audit actual existing regional gates; no ROS calls."""
import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--run', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    a = p.parse_args()
    summary = json.loads((a.run / 'read_diagnostics.summary.json').read_text())
    assert summary['original_read_restored'] and not summary['errors']
    assert not summary['queued_unwritten']
    rows = [json.loads(x) for x in (a.run / 'read_diagnostics.jsonl').read_text().splitlines()]
    assert summary['counts']['written'] == summary['counts']['observed'] == len(rows)
    assert Path('/media/zhuyupei/SanStand').is_mount()
    ext = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller').resolve()
    assert ext in a.output.resolve().parents and not a.output.exists()
    complete = 0
    center_reasons = Counter()
    filtered_center = 0
    visual_resets = []
    source_resets = []
    violations = []
    previous = None
    for i, row in enumerate(rows):
        d = row['existing_read_diagnostics']
        if d['result'] != 'complete':
            continue
        complete += 1
        e = d.get('regional_stationary_evidence', {})
        if not e:
            continue
        c = next((x for x in e.get('regions', []) if x['name'] == 'center'), None)
        if c is not None:
            v = c.get('visual', {})
            center_reasons[v.get('reason')] += 1
            filtered_center += bool(c['filter_applied'])
            if c['history_reset'] and v.get('reason') == 'visual_motion':
                visual_resets.append(dict(row=i,frame_number=v.get('frame_number'),motion_p90_px=v.get('motion_p90_px')))
        if e.get('global_history_reset_reason') == 'sdk_source_discontinuity':
            item = dict(row=i, global_history_reset_reason=e['global_history_reset_reason'], streams={})
            if previous is not None:
                pd = previous['existing_read_diagnostics']
                item['filter_host_interval_sec'] = d['stationary_pair_filter_host_monotonic'] - pd['stationary_pair_filter_host_monotonic']
                for k in ('color','depth'):
                    b, n = pd['raw_frames'][k], d['raw_frames'][k]
                    item['streams'][k] = dict(previous_number=b['frame_number'],number=n['frame_number'],
                        number_increment=n['frame_number']-b['frame_number'],
                        device_interval_ms=n['device_timestamp_ms']-b['device_timestamp_ms'],
                        actual_fps_metadata_raw=n.get('metadata_raw',{}).get('actual_fps'))
            source_resets.append(item)
        for region in e.get('regions', []):
            v = region.get('visual', {})
            if region.get('filter_applied'):
                if not (region.get('qualified') is True and v.get('qualified') is True
                    and v.get('reason') == 'stationary' and v.get('quiet_span_sec', -1) >= .35
                    and v.get('host_quiet_span_sec', -1) >= .35
                    and v.get('motion_p90_px', float('inf')) <= .25):
                    violations.append(dict(row=i, region=region['name'], reason='original_visual_gate_not_met'))
        if e.get('filtered_pixel_count', 0) and e.get('current_zero_pixels_preserved') is not True:
            violations.append(dict(row=i,reason='current_depth_holes_not_attested_preserved'))
        previous = row
    episodes = []
    for i,row in enumerate(rows):
        d=row['existing_read_diagnostics']
        ph=d.get('stationary_pair_history',{})
        if ph.get('state') not in ('resumed_for_existing_gates','discarded'):
            continue
        item=dict(row=i,state=ph['state'],reason=ph['reason'])
        if ph.get('first_rejection_host_monotonic') is not None and ph.get('last_complete_filter_host_monotonic') is not None:
            item['original_checkpoint_to_first_rejection_host_sec']=ph['first_rejection_host_monotonic']-ph['last_complete_filter_host_monotonic']
        for j in range(i,len(rows)):
            nd=rows[j]['existing_read_diagnostics']
            if nd['result']!='complete':continue
            c=next((x for x in nd.get('regional_stationary_evidence',{}).get('regions',[]) if x['name']=='center'),None)
            if c is not None and c['filter_applied']:
                item['first_later_center_filtered']=dict(row=j,frame_number=c['visual'].get('frame_number'),
                    quiet_span_sec=c['visual'].get('quiet_span_sec'),host_quiet_span_sec=c['visual'].get('host_quiet_span_sec'),
                    continuous_frames=c['visual'].get('continuous_frames'))
                break
        episodes.append(item)
    report=dict(schema=1,input_sha256=sha(a.run/'read_diagnostics.jsonl'),script_sha256=sha(Path(__file__)),
        complete_reads=complete,center_reason_counts=dict(center_reasons),center_filtered_reads=filtered_center,
        source_discontinuity_resets=source_resets,center_visual_motion_resets=visual_resets,
        pair_history_followups=episodes,violations=violations,
        conclusion='Both real resumes retain only warming history: a prior complete source-discontinuous pair already reset mature history. Original visual/source/host gates remain active; this window does not prove improved support-plane quality.',
        evidence_limit='Metadata only; SDK actual_fps reported in raw units, no inferred exposure or root hardware cause; no motion or ROS calls')
    a.output.write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    print(json.dumps({k:report[k] for k in ('complete_reads','center_reason_counts','center_filtered_reads','violations')},ensure_ascii=False))

if __name__ == '__main__':
    main()
