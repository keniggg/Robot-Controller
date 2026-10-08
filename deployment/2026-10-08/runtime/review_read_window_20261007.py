#!/usr/bin/env python3
"""Offline, bounded metadata review of one sealed diagnostic window; no ROS."""
from collections import Counter
import hashlib
import json
from pathlib import Path

import numpy as np

ROOT = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/regional_production_20261007T011805.778547Z')


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def distribution(values):
    return None if not values else dict(count=len(values), minimum=float(min(values)),
        median=float(np.median(values)), p95=float(np.percentile(values, 95)), maximum=float(max(values)))


def main():
    paths = [ROOT/'read_diagnostics.jsonl', ROOT/'visual_snapshots/summary.json',
             ROOT/'post_window_diagnostic_history.json', ROOT/'exact_replay_v1/report.json']
    before = {str(p): digest(p) for p in paths}
    rows = [json.loads(line) for line in paths[0].read_text().splitlines()]
    summary = json.loads(paths[1].read_text())
    post_record = json.loads(paths[2].read_text())
    post = json.loads(post_record['envelope']['payload_json'])
    replay = json.loads(paths[3].read_text())
    assert summary['sealed'] and not summary['errors']
    assert summary['close_reason'] == 'window_expired'
    assert all(summary[k] is True for k in ('original_process_restored', 'original_observe_restored', 'original_reset_restored'))
    assert summary['inflight_nested_shadows'] == summary['nested_shadow_retained_owners'] == 0
    good = [r for r in rows if r['existing_read_diagnostics']['result'] == 'complete']
    rejected = [r['existing_read_diagnostics'] for r in rows if r['existing_read_diagnostics']['result'] != 'complete']
    report = dict(schema=1, diagnostic_only=True, ros_calls=False, hardware_actions=False,
        source_files_sha256=before, script_sha256=digest(Path(__file__).resolve()),
        read_results=dict(Counter(r['existing_read_diagnostics']['result'] for r in rows)),
        completion_interval_ms=distribution([1000*(b['read_completed_monotonic']-a['read_completed_monotonic']) for a,b in zip(good,good[1:])]),
        complete_read_duration_ms=distribution([1000*r['existing_read_diagnostics']['total_sec'] for r in good]),
        regional_stage_including_tap_diagnostic_ms=distribution([1000*r['existing_read_diagnostics']['stages_sec']['regional_stationary_depth'] for r in good]),
        global_history_resets=dict(Counter(str(r['existing_read_diagnostics']['regional_stationary_evidence']['global_history_reset_reason']) for r in good)))
    for name in ('center', 'nested_center'):
        data = []
        for r in good:
            ev = r['existing_read_diagnostics']['regional_stationary_evidence']
            region = ev['regions'][0] if name == 'center' else ev['nested_center_shadow']
            assert name != 'center' or region['name'] == 'center'
            if region.get('visual'):
                data.append(region)
        report[name] = dict(observed=len(data), qualified=sum(x.get('qualified') is True for x in data),
            reasons=dict(Counter(x['visual']['reason'] for x in data)),
            motion_p90_px=distribution([x['visual']['motion_p90_px'] for x in data if 'motion_p90_px' in x['visual']]),
            motion_median_px=distribution([x['visual']['motion_median_px'] for x in data if 'motion_median_px' in x['visual']]))
    report['parent_child_qualification_joint'] = dict(Counter(
        'parent_%s_child_%s' % (r['existing_read_diagnostics']['regional_stationary_evidence']['regions'][0]['qualified'],
                               r['existing_read_diagnostics']['regional_stationary_evidence']['nested_center_shadow']['qualified']) for r in good))
    report['rejected_pair'] = dict(count=len(rejected), reasons=dict(Counter(x['pair_timing']['reason'] for x in rejected)),
        skew_ms=distribution([x['pair_timing']['skew_ms'] for x in rejected]), unchanged_limit_ms=16.666666666666668)
    history = [r['existing_read_diagnostics']['stationary_pair_history'] for r in rows if 'stationary_pair_history' in r['existing_read_diagnostics']]
    report['pair_history_events'] = dict(Counter(x['state']+':'+x['reason'] for x in history))
    report['source_intervals'] = {}
    for stream in ('color', 'depth'):
        frames = [r['existing_read_diagnostics']['raw_frames'][stream] for r in good]
        changes = [(b['frame_number']-a['frame_number'], b['device_timestamp_ms']-a['device_timestamp_ms'], b)
                   for a,b in zip(frames,frames[1:])]
        noncontinuous = [(dn,dt,b) for dn,dt,b in changes if dn != 1 or not 1000*.25/30 <= dt <= 1000*1.75/30]
        report['source_intervals'][stream] = dict(frame_number_deltas=dict(Counter(dn for dn,dt,b in changes)),
            device_interval_ms=distribution([dt for dn,dt,b in changes]),
            noncontinuous_count=len(noncontinuous),
            consecutive_counter_but_long_interval_count=sum(dn == 1 for dn,dt,b in noncontinuous),
            noncontinuous_events=[dict(frame_number=b['frame_number'], counter_delta=dn, device_interval_ms=dt,
                actual_fps_raw=b['metadata_raw'].get('actual_fps')) for dn,dt,b in noncontinuous])
    entries = post['entries']
    assert post_record['read_only'] is True and post['diagnostic_only'] is True and post['motion_authorized'] is False
    assert post['copy_errors'] == 0 and len(entries) == 32
    assert len({x['source_stamp_ns'] for x in entries}) == len(entries)
    assert all(x['read_completed_monotonic'] > summary['finished_monotonic'] for x in entries)
    assert all(x['evidence']['regional_stationary_evidence']['nested_center_shadow']['reason'] == 'diagnostic_disabled' for x in entries)
    assert all(x['evidence']['regional_stationary_evidence']['nested_center_shadow']['qualified'] is False
               and x['evidence']['regional_stationary_evidence']['nested_center_shadow']['visual'] is None for x in entries)
    report['actual_post_window_default_off'] = dict(verified=True, count=len(entries), copy_errors=post['copy_errors'],
        first_source_stamp_ns=entries[0]['source_stamp_ns'], last_source_stamp_ns=entries[-1]['source_stamp_ns'],
        minimum_seconds_after_tap_sealed=min(x['read_completed_monotonic']-summary['finished_monotonic'] for x in entries),
        latest_completed_seconds_after_tap_sealed=max(x['read_completed_monotonic']-summary['finished_monotonic'] for x in entries),
        source_stamp_clock='host_read_completion; not calibrated physical exposure clock',
        publication_outcome_unknown=all(x['publication_outcome_unknown'] is True for x in entries))
    report['sealed_tap'] = {k:summary[k] for k in ('categories', 'counts', 'close_reason', 'first_read_monotonic',
        'finished_monotonic', 'original_process_restored', 'original_observe_restored', 'original_reset_restored',
        'inflight_nested_shadows', 'nested_shadow_retained_owners')}
    exact_rows = replay['rows']
    report['replay_crosscheck'] = dict(rows=len(exact_rows), strictly_equal=sum(x['exact_return_equal'] for x in exact_rows),
        all_original_returns_exactly_reproduced=replay['all_original_returns_exactly_reproduced'],
        mismatches=[dict(file=x['file'], fields={k:dict(original=x['original_return'].get(k), replayed=x['replayed_return'].get(k))
            for k in set(x['original_return'])|set(x['replayed_return']) if x['original_return'].get(k)!=x['replayed_return'].get(k)})
            for x in exact_rows if not x['exact_return_equal']],
        gaussian_motion_frames_recovered={str(s):sum(z['p90_motion_gate_passed'] for x in exact_rows if x['category']=='visual_motion'
            for z in x['gaussian_diagnostic']['actual_pair'] if z['sigma']==s) for s in (.6,.8,1.)})
    report['limitations'] = [
        'Pose, lighting and acquisition time differ from the previous scene; visual qualification changes are not a patch-effect comparison.',
        'The sampled 30 second read window itself contains no post-window disabled report; default-off verification uses the separately saved later 32-source history.',
        'Tap image copies and sixth visual observer add diagnostic work; measured regional stage timings are not default-off production timings.',
        'SDK source discontinuities and image motion qualifications are distinct causes of history resets; no attribution to USB, exposure, physical motion or hardware noise is established.',
        'Only 31/861 reads qualify the child while the parent is unqualified; this does not establish a reliable support-plane fallback.',
        'Low directional resultant and subtraction of median translation do not exclude rotation, scale, perspective or depth-dependent real motion.',
        'Gaussian diagnostics recovered none of the 16 selected motion examples; neither synthetic controls nor this camera-only capture validate support-plane quality, target identity or grasp success.',
        'Strict report equality remains false for two sub-threshold P90 values differing by 2.7755575615628914e-17 pixels; no stored original report or acceptance policy was changed.'
    ]
    assert before == {str(p):digest(p) for p in paths}
    assert Path('/media/zhuyupei/SanStand').is_mount()
    output = ROOT/'read_window_review.json'
    with output.open('x') as f:
        json.dump(report,f,ensure_ascii=False,indent=2,allow_nan=False)
        f.write('\n')
    print(json.dumps(dict(report=str(output), sha256=digest(output), complete_reads=len(good),
        post_window_disabled=len(entries), replay_strictly_equal=report['replay_crosscheck']['strictly_equal'])))


if __name__ == '__main__':
    main()
