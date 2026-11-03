#!/usr/bin/env python3
"""Compare closed read-observer windows; never attach to a live device/node."""
from collections import Counter
import hashlib
import json
from pathlib import Path

import numpy as np

EXT = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log')
OLD = EXT / 'regional_production_20261006T105318.899385Z'
NEW = EXT / 'regional_production_20261006T112923.272799Z'


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def percentiles(values):
    if not values:
        return None
    return dict(zip(('min', 'p50', 'p95', 'p99', 'max'),
                    map(float, np.percentile(values, [0, 50, 95, 99, 100]))))


def analyze(directory):
    summary = json.loads((directory / 'read_diagnostics.summary.json').read_text())
    assert summary['original_read_restored'] and not summary['queued_unwritten']
    assert not summary['errors'] and not summary['counts']['queue_dropped']
    deployment = json.loads((directory / 'result.json').read_text())
    assert deployment['camera_loaded'] and deployment['static_validation_passed']
    assert deployment['events'][-1]['event'] == 'finished'
    path = directory / 'read_diagnostics.jsonl'
    rows = [json.loads(line) for line in path.read_text().splitlines()]
    reads = [row['existing_read_diagnostics'] for row in rows]
    complete = [row for row in reads if row['result'] == 'complete']
    region_counts = {}
    motion = {}
    pair_events = []
    source_gaps = []
    host_gaps = []
    intervals = {stream: [] for stream in ('color', 'depth')}
    host_intervals = []
    global_resets = Counter()
    previous = None
    qualified_runs = {name: [] for name in ('center', 'top_left', 'top_right', 'bottom_left', 'bottom_right')}
    run = {name: [] for name in qualified_runs}
    for i, row in enumerate(reads):
        evidence = row.get('regional_stationary_evidence')
        if evidence is not None:
            global_resets[str(evidence.get('global_history_reset_reason'))] += 1
        event = row.get('stationary_pair_history')
        if event:
            pair_events.append(dict(read_index=i, state=event['state'], reason=event['reason']))
        for region in (evidence or {}).get('regions', []):
            name = region['name']
            counts = region_counts.setdefault(name, Counter())
            counts['total'] += 1
            for key in ('qualified', 'warming', 'history_reset', 'filter_applied'):
                counts[key] += bool(region[key])
            counts['reason:'+region['visual']['reason']] += 1
            if region['visual']['reason'] == 'visual_motion':
                motion.setdefault(name, []).append(region['visual'])
            if region['qualified']:
                run[name].append((row['raw_frames']['color']['device_timestamp_ms'],
                                  row['stationary_pair_filter_host_monotonic']))
            elif run[name]:
                qualified_runs[name].append(run[name]);run[name] = []
        if row['result'] != 'complete':
            continue
        if previous is not None:
            old = previous[1]
            for stream in intervals:
                before, current = old['raw_frames'][stream], row['raw_frames'][stream]
                dt = current['device_timestamp_ms']-before['device_timestamp_ms']
                increment = current['frame_number']-before['frame_number']
                intervals[stream].append(dt)
                if increment != 1 or not 250/current['stream_fps'] <= dt <= 1750/current['stream_fps']:
                    source_gaps.append(dict(read_index=i, previous_complete_index=previous[0],
                        stream=stream, frame_number_increment=increment, device_interval_ms=dt))
            dt_host = row['stationary_pair_filter_host_monotonic']-old['stationary_pair_filter_host_monotonic']
            host_intervals.append(dt_host*1000)
            if not 0 < dt_host <= max(.1, 2.5/row['raw_frames']['color']['stream_fps']):
                host_gaps.append(dict(read_index=i, previous_complete_index=previous[0], interval_ms=dt_host*1000))
        previous = (i, row)
    for name, value in run.items():
        if value:
            qualified_runs[name].append(value)
    regions = {}
    for name, counts in region_counts.items():
        regions[name] = dict(counts, qualified_fraction=counts['qualified']/counts['total'],
            visual_motion_p90_px=percentiles([x['motion_p90_px'] for x in motion.get(name, [])]),
            visual_motion_median_px=percentiles([x['motion_median_px'] for x in motion.get(name, [])]),
            qualified_run_count=len(qualified_runs[name]),
            qualified_run_source_spans_sec=percentiles([(r[-1][0]-r[0][0])/1000 for r in qualified_runs[name]]),
            qualified_run_filter_host_spans_sec=percentiles([r[-1][1]-r[0][1] for r in qualified_runs[name]]))
    stage_keys = ('wait_for_frames', 'align', 'depth_filter_0_spatial', 'stationary_evidence',
                  'regional_stationary_depth', 'depth_array_and_clip', 'pair_rectification')
    stages = {key: percentiles([r['stages_sec'][key]*1000 for r in reads if key in r['stages_sec']])
              for key in stage_keys}
    nonwait = [sum(value for key, value in r['stages_sec'].items() if key != 'wait_for_frames')*1000
               for r in complete]
    return dict(directory=str(directory), log_sha256=sha(path), observer_summary=summary,
        result_counts=dict(Counter(row['result'] for row in reads)), regions=regions,
        global_reset_reasons=dict(global_resets), fresh_pair_history_events=pair_events,
        source_discontinuities=source_gaps, source_discontinuous_reads=len({x['read_index'] for x in source_gaps}),
        filter_host_discontinuities=host_gaps,
        consecutive_complete_source_interval_ms={key: percentiles(value) for key,value in intervals.items()},
        consecutive_complete_filter_host_interval_ms=percentiles(host_intervals),
        stage_ms=stages, complete_nonwait_stage_total_ms=percentiles(nonwait),
        nonwait_over_30fps_period_count=sum(v > 1000/30 for v in nonwait))


def main():
    assert Path('/media/zhuyupei/SanStand').is_mount()
    result = dict(schema=1, diagnostic_only=True, hardware_commands_sent=False,
        source_sha256=sha(Path(__file__)), old=analyze(OLD), new=analyze(NEW),
        conclusions=[
            'All six new pre-filter mismatched pairs were quarantined then resumed only after the original continuity checks. This does not prove sustained support quality.',
            'The new center region qualified in 342/892 complete reads (38.34%), compared with 728/887 (82.07%) previously; center visual-motion rejections increased from 11 to 83.',
            'Real device/source discontinuities remain: seven read transitions in each window. Host-filter intervals exceeding the existing bound decreased from four to one.',
            'Regional processing wall-clock median is 11.11 ms old versus 11.51 ms new, while P99 is 15.15 ms old versus 14.78 ms new. This independent window comparison does not establish a causal performance benefit.',
            'Windows were sequential and did not preserve identical exposure/color frames. Changes cannot be uniquely attributed to OpenCV thread count, illumination, SDK, USB, or scheduling.',
            'No exact support-fit result is joined here; neither this static window nor a camera load pass demonstrates supplementary observation or grasp success.',
            'Invalid-pair diagnostics are counted from fresh read records only, not persistent runtime-profile events. Qualified runs describe accepted complete frames and do not claim all intervening SDK packets were accepted.'
        ])
    output = NEW / 'independent_window_comparison.json'
    assert not output.exists()
    output.write_text(json.dumps(result, indent=2, ensure_ascii=False)+'\n')
    print(json.dumps(dict(path=str(output), sha256=sha(output),
        center_old=result['old']['regions']['center']['qualified_fraction'],
        center_new=result['new']['regions']['center']['qualified_fraction'])))


if __name__ == '__main__':
    main()
