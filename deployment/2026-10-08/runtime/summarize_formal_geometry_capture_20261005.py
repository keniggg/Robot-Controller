#!/usr/bin/env python3
"""Archive source-bounded conclusions from the closed gen5 read-only capture."""
from collections import Counter
import json
from pathlib import Path
import re
import shutil

import numpy as np
from replay_matched_shadow_geometry_20261005 import support

EXT=support.EXT
CAP=EXT/'ros_log/rgb_observation_bootstrap_20261005T034713Z'
TRIAL=EXT/'ros_log/saved_baseline_trial_20261005T033848.688318Z/continuation_20261005T034614Z'
OLD=EXT/'ros_log/matched_shadow_current_20261005T024652.714830Z/stages/sample_11.json'


def main():
    if not support.MOUNT.is_mount():raise RuntimeError('SanStand unavailable; no fallback')
    rows=json.loads((CAP/'result.json').read_text())['rows']
    replay=json.loads((CAP/'offline_geometry_20261005T0348Z/report.json').read_text())
    trial=json.loads((TRIAL/'result.json').read_text())
    old=json.loads(OLD.read_text())
    q=np.array([r['q_source'] for r in rows]); transforms=np.array([r['T_base_camera'] for r in rows])
    first=next(r['detector']['first_loss'] for r in rows if (r.get('detector') or {}).get('first_loss'))
    events=[]
    for line in (TRIAL/'camera.stderr.log').read_text().splitlines():
        match=re.search(r'\[(179117\d+\.\d+)\]',line)
        if match and 1791172030<float(match[1])<1791172044 and '{' in line:
            data=json.loads(line[line.index('{'):])
            events.append(dict(log_ros_sec=float(match[1]),result=data.get('result'),
                pair_timing=data.get('pair_timing'),host_wait=data.get('host_wait'),
                stationary_rgb_evidence=data.get('stationary_rgb_evidence'),
                stages_sec=data.get('stages_sec')))
    result=dict(schema=1,scope='closed read-only capture; no motion or planning',
        geometry_authority=False,new_user_alignment_confirmation=False,
        capture=str(CAP),sample_count=len(rows),source_window_sec=(rows[-1]['source_stamp_ns']-rows[0]['source_stamp_ns'])*1e-9,
        capture_budget=dict(duration_max_sec=30,max_frames=12,ended_by='maximum_frames'),
        selection=rows[0]['selection'],epochs=sorted({r['epoch_ns'] for r in rows}),
        measured_q_rad=q[0].tolist(),maximum_q_spans_rad=np.ptp(q,axis=0).tolist(),
        joint_delta_from_old_ab_sample11_rad=(q[0]-np.array(old['source_evidence']['feedback']['q_at_anchor'][:6])).tolist(),
        saved_requested_source_tf_maximum_matrix_difference=float(np.max(abs(transforms-transforms[0]))),
        strict_returned_tf_stamp_saved=False,
        fresh_samples=sum(r.get('fresh_for_existing_frame_limit') is True for r in rows),
        context_unchanged_samples=sum(r.get('source_context_unchanged') is True for r in rows),
        replay_summary=replay['summary'],
        initial_support_passes=sum(r.get('replay',{}).get('stop_stage')!='support_segmentation' for r in replay['rows']),
        first_loss=first, exact_first_loss_rgbd_saved=any(r['source_stamp_ns']==first['source_stamp_ns'] for r in rows),
        last_good_source_saved=any(r['source_stamp_ns']==first['last_good_source_stamp_ns'] for r in rows),
        next_saved_good_single_frame_source_stamp_ns=rows[-1]['source_stamp_ns'],
        next_saved_good_since_last_good_sec=(rows[-1]['source_stamp_ns']-first['last_good_source_stamp_ns'])*1e-9,
        lost_generation_must_not_revive=True,
        visual_inspection=dict(observer='assistant',new_user_confirmation=False,
            images=[str(CAP/(str(rows[0]['source_stamp_ns'])+'_rgb.png')),
                    str(CAP/(str(rows[-1]['source_stamp_ns'])+'_rgb.png')),
                    str(CAP/'offline_geometry_20261005T0348Z'/(str(rows[0]['source_stamp_ns'])+'_refined.png'))],
            finding='Silver upright object remains at the same image location. First-frame original refinement mask covers the body/base, not the long lower-right shadow. No coordinates were fed into selection.'),
        temporal_evidence=dict(per_frame_active_saved=False,
            earlier_loader_active_diagnostic=trial['stationary_active_diagnostic'],camera_logged_events=events,
            conclusion='A 33.0369ms-skew pair was rejected about 214ms before the first-loss source. Per-frame visual/temporal state is absent, so this is a temporal association, not a proven reset cause.'),
        limitations=[
            'Initial-selection replay and the online locked-reference segmentation are different production paths. The saved last-good frame fails initial-ring support while its locked online source was accepted.',
            'First-loss RGB-D source is not among the 12 retained frames. Its online reason is preserved, but that exact fit cannot be replayed.',
            'Single-frame pre-snapshot geometry is not a completed multi-frame remote snapshot, physical dimension measurement or executable grasp proof.',
            'Detector state was sampled at capture selection and can refer to an earlier image; see per-row source deltas in the geometry report.',
            'No firmware/temperature data, robot commands or planning requests were read or sent.'],
        sources={str(p):support.sha(p) for p in (CAP/'result.json',CAP/'offline_geometry_20261005T0348Z/report.json',
            TRIAL/'result.json',OLD,Path(__file__))})
    out=CAP/'formal_geometry_findings.json'
    if out.exists():raise RuntimeError('findings already exist')
    out.write_text(json.dumps(result,ensure_ascii=False,indent=2,allow_nan=False)+'\n')
    shutil.copy2(__file__,CAP/Path(__file__).name)
    print(json.dumps(dict(path=str(out),sha256=support.sha(out))))


if __name__=='__main__':main()
