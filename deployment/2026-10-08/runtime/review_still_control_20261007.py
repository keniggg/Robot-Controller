#!/usr/bin/env python3
"""Offline source/reset ancestry and two-motion review of the new still control."""
from collections import Counter
import hashlib,importlib.util,json
from pathlib import Path
import cv2,numpy as np

W=Path(__file__).resolve().parents[1]
EXT=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log')
ROOT=EXT/'regional_production_20261007T015523.918802Z'
OLD=EXT/'regional_production_20261007T011805.778547Z'
ANALYZER=W/'.runtime/analyze_visual_context_models_20261007.py'
spec=importlib.util.spec_from_file_location('still_motion_context_models',ANALYZER)
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)


def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()
def stat(values):return dict(count=len(values),minimum=float(min(values)),median=float(np.median(values)),p95=float(np.percentile(values,95)),maximum=float(max(values)))


def dataset(root):
    rows=[json.loads(x)for x in (root/'read_diagnostics.jsonl').read_text().splitlines()]
    good=[x for x in rows if x['existing_read_diagnostics']['result']=='complete']
    result=dict(read_results=dict(Counter(x['existing_read_diagnostics']['result']for x in rows)),
        global_reset=dict(Counter(str(x['existing_read_diagnostics']['regional_stationary_evidence']['global_history_reset_reason'])for x in good)))
    for region in ('center','nested_center'):
        values=[(x['existing_read_diagnostics']['regional_stationary_evidence']['regions'][0]
                 if region=='center'else x['existing_read_diagnostics']['regional_stationary_evidence']['nested_center_shadow'])['visual'] for x in good]
        result[region]=dict(reasons=dict(Counter(v['reason']for v in values)),
            motion_p90_px=stat([v['motion_p90_px']for v in values if 'motion_p90_px'in v]))
    result['successful_read_host_intervals_ms']=stat([1000*(b['read_completed_monotonic']-a['read_completed_monotonic'])for a,b in zip(good,good[1:])])
    result['source_intervals']={}
    for stream in ('color','depth'):
        raw=[x['existing_read_diagnostics']['raw_frames'][stream]for x in good]
        delta=[dict(frame_number=b['frame_number'],counter_delta=b['frame_number']-a['frame_number'],device_interval_ms=b['device_timestamp_ms']-a['device_timestamp_ms'],
                    actual_fps_raw=b['metadata_raw'].get('actual_fps'))for a,b in zip(raw,raw[1:])]
        abnormal=[x for x in delta if x['counter_delta']!=1 or not 1000*.25/30<=x['device_interval_ms']<=1000*1.75/30]
        result['source_intervals'][stream]=dict(interval_ms=stat([x['device_interval_ms']for x in delta]),
            counter_deltas=dict(Counter(x['counter_delta']for x in delta)),noncontinuous_count=len(abnormal),
            consecutive_counter_long_interval_count=sum(x['counter_delta']==1 for x in abnormal),noncontinuous_events=abnormal)
    result['pair_history_events']=dict(Counter(x['existing_read_diagnostics']['stationary_pair_history']['state']+':'+x['existing_read_diagnostics']['stationary_pair_history']['reason']
        for x in rows if 'stationary_pair_history'in x['existing_read_diagnostics']))
    warming=Counter();current='unidentified';seed_starts=[];motions=[];last_rejection=None
    for i,x in enumerate(rows):
        d=x['existing_read_diagnostics']
        if d['result']!='complete':
            last_rejection=dict(row=i,**{k:d.get('stationary_pair_history',{}).get(k)for k in ('state','reason')});continue
        ev=d['regional_stationary_evidence'];v=ev['regions'][0]['visual'];reason=ev['global_history_reset_reason']
        if reason=='image_geometry_changed':current='initial_geometry'
        elif reason=='sdk_source_discontinuity':current='sdk_source_discontinuity'
        elif v['reason']=='visual_motion':
            current='visual_motion';motions.append(dict(read_index=i,source=v,host=x['read_completed_monotonic']))
        elif v['reason']=='warming_up' and v['tracked_points']==0:
            current=('preceding_discarded_pair_reset' if last_rejection and last_rejection['row']==i-1 and last_rejection['state']=='discarded' else 'other_unproven_reset')
            seed_starts.append(dict(read_index=i,frame_number=v['frame_number'],attribution=current,preceding_rejection=last_rejection))
        if v['reason']=='warming_up':warming[current]+=1
    result['warming_by_last_observed_reset_ancestry']=dict(warming)
    result['seed_starts_without_regional_global_reason']=seed_starts
    result['motion_events']=motions
    return rows,good,result


def main():
    cv2.setNumThreads(1)
    paths=[ROOT/'read_diagnostics.jsonl',ROOT/'visual_snapshots/summary.json',ROOT/'exact_replay_v1/report.json',
           OLD/'read_diagnostics.jsonl',ANALYZER,Path(__file__).resolve(),m.REPLAYER,m.m.VISUAL_PATH]
    inputs={str(p):sha(p)for p in paths}
    summary=json.loads(paths[1].read_text());assert summary['sealed']and not summary['errors']
    rows,good,analysis=dataset(ROOT)
    _,_,prior=dataset(OLD)
    exact=json.loads(paths[2].read_text())
    differences=[];diagnostics=[];figures=[]
    for row in exact['rows']:
        if not row['exact_return_equal']:
            differences.append(dict(file=row['file'],fields={k:dict(original=row['original_return'].get(k),replayed=row['replayed_return'].get(k))
                for k in set(row['original_return'])|set(row['replayed_return'])if row['original_return'].get(k)!=row['replayed_return'].get(k)}))
        if row['category']!='visual_motion':continue
        path=ROOT/'visual_snapshots'/row['file'];inputs[str(path)]=sha(path)
        assert inputs[str(path)]==row['file_sha256']
        with np.load(path,allow_pickle=False)as a:arrays={k:a[k].copy()for k in a.files}
        meta=json.loads(arrays['metadata_json'].tobytes());review=m.review(arrays,meta);review['file']=row['file']
        valid=[x for x in row['points']if x['valid']]
        vectors=np.asarray([x['motion_full_input_xy']for x in valid]);median=np.median(vectors,axis=0)
        review['photometric_diagnostic']=dict(valid_points=len(valid),patch_mean_delta=stat([x['patch_mean_delta']for x in valid]),
            patch_zero_mean_rms=stat([x['patch_zero_mean_rms']for x in valid]),
            median_flow_vector_full_px=median.tolist(),p90_after_median_translation=float(np.percentile(np.linalg.norm(vectors-median,axis=1),90)),
            interpretation='Patch photometric residuals and fitted motion are diagnostics; neither establishes noise or physical stillness.')
        diagnostics.append(review);figures.append((arrays,meta,review))
    assert len(diagnostics)==2
    motion_index=next(i for i,x in enumerate(good)if x['existing_read_diagnostics']['regional_stationary_evidence']['regions'][0]['visual']['reason']=='visual_motion')
    recovered=next(i for i in range(motion_index+1,len(good))if good[i]['existing_read_diagnostics']['regional_stationary_evidence']['regions'][0]['visual']['qualified'])
    v0=good[motion_index]['existing_read_diagnostics']['regional_stationary_evidence']['regions'][0]['visual']
    vr=good[recovered]['existing_read_diagnostics']['regional_stationary_evidence']['regions'][0]['visual']
    result=dict(schema=1,diagnostic_only=True,motion_authorized=False,production_modified=False,input_sha256=inputs,
        user_condition_context='Parent supplied explicit user confirmation of30seconds without hand intrusion or lighting changes; this is a stated experimental condition, not a calibrated physical-motion sensor.',
        new_control=analysis,previous_different_scene=prior,
        motion_recovery=dict(frame_number=v0['frame_number'],device_timestamp_ms=v0['device_timestamp_ms'],
            following_warming_frames=recovered-motion_index-1,recovered_frame_number=vr['frame_number'],
            recovered_quiet_span_sec=vr['quiet_span_sec'],source_delta_sec=(vr['device_timestamp_ms']-v0['device_timestamp_ms'])/1000.,
            no_intervening_regional_global_reset=all(good[i]['existing_read_diagnostics']['regional_stationary_evidence']['global_history_reset_reason']is None for i in range(motion_index+1,recovered+1))),
        exact_replay=dict(rows=len(exact['rows']),strictly_equal=sum(x['exact_return_equal']for x in exact['rows']),
            all_exact=exact['all_original_returns_exactly_reproduced'],all_reason_and_qualified_unchanged=all(x['original_return']['reason']==x['replayed_return']['reason']and x['original_return']['qualified']==x['replayed_return']['qualified']for x in exact['rows']),differences=differences),
        isolated_motion_diagnostics=diagnostics,
        conclusions=[
            'The one parent/child visual-motion event shares exact sourceframe603; it explains only10 subsequent warming frames before stationary permission returns atframe614 after0.36675seconds.',
            'Of430 warming reads,399 follow52 regional SDK source-discontinuity resets,16 follow2 immediately preceding pair-discard resets,5 follow initial geometry,10 follow the one motion event. These are observed state/reset ancestry, not a claim about hardware root cause.',
            'The remaining unstable-window problem in this control is dominated by source/reset continuity rather than repeated visual-motion decisions.',
            'The old scene has65/99 parent/child motion readings; this new control has1/1. Different pose/lighting/timing and user condition prevent attributing the reduction to one software change.',
            'Original gate values remain unchanged; no quality0.12 experiment is repeated or promoted. A rare visual rejection alone does not justify bypassing freshness, source or motion gates.'
        ],limitations=[
            'User-confirmed stillness does not prove zero mechanical micromotion; observed residuals do not identify physical motion, noise or illumination as the unique cause.',
            'Exact dictionary equality is deliberately false for four P90 floating-tail differences; original results are retained and no tolerance is used to grant authority.',
            'Affine/homography and expanded-context replay are diagnostic only and do not remove movement from a stationary decision.',
            'No RGB-D support-plane fit, target identity continuity, grasp plan or actual grasp is validated by this camera-only control.'
        ])
    assert all(sha(Path(p))==v for p,v in inputs.items())
    assert Path('/media/zhuyupei/SanStand').is_mount()
    output=ROOT/'still_control_review_v1';output.mkdir(exist_ok=False)
    for arrays,meta,review in figures:m.visualize(output,review['file'].replace('.npz',''),arrays,meta,review)
    with (output/'report.json').open('x')as f:json.dump(result,f,indent=2,ensure_ascii=False,allow_nan=False);f.write('\n')
    print(json.dumps(dict(report=str(output/'report.json'),sha256=sha(output/'report.json'),warming=analysis['warming_by_last_observed_reset_ancestry'],exact_rows=result['exact_replay']['strictly_equal'])))


if __name__=='__main__':main()
