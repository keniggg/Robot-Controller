#!/usr/bin/env python3
"""Analyze sealed read-only JSON history; no ROS, SDK, images or device access."""
import argparse
from collections import Counter, defaultdict
import hashlib
import json
from pathlib import Path

import numpy as np


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def stats(values):
    a = np.asarray(values, dtype=float)
    if not len(a):
        return {'n': 0}
    return dict(n=len(a), mean=float(np.mean(a)), sum=float(np.sum(a)),
                percentiles=dict(zip(('min', 'p50', 'p95', 'max'),
                                     np.percentile(a, [0, 50, 95, 100]).tolist())))


def stream_delta(a, b):
    return dict(previous_frame_number=a['frame_number'], frame_number=b['frame_number'],
                count_step=b['frame_number']-a['frame_number'],
                device_dt_ms=b['device_timestamp_ms']-a['device_timestamp_ms'],
                native_frame_timestamp_delta=b['metadata_raw']['frame_timestamp']-a['metadata_raw']['frame_timestamp'],
                previous_raw=a, current_raw=b)


def analyze(folder, tap_report):
    path = folder/'history.jsonl'
    receipt = json.loads((folder/'summary.json').read_text())
    assert receipt['read_only'] and not receipt['robot_commands_sent'] and not receipt['parameter_writes']
    assert sha(path) == receipt['sha256'] and path.stat().st_size == receipt['bytes']
    rows = [json.loads(line) for line in path.open()]
    assert len(rows) == receipt['queries']
    by_source, query_rows, previous_sources, owners = {}, [], None, set()
    duplicate_entries = 0
    for index, row in enumerate(rows):
        envelope = row['envelope']
        assert envelope['schema'] == 1 and envelope['encoding'] == 'json'
        payload = json.loads(envelope['payload_json'])
        assert payload['schema'] == 1 and payload['capacity'] == 32 and payload['copy_errors'] == 0
        assert payload['diagnostic_only'] and not payload['motion_authorized']
        entries = payload['entries']
        assert len(entries) == 32
        sources = [e['source_stamp_ns'] for e in entries]
        assert sources == sorted(set(sources))
        overlap = None if previous_sources is None else sorted(previous_sources & set(sources))
        # Overlapping finite rings prove no successful-read entry was skipped
        # between these parameter snapshots. Rejected reads are never in rings.
        assert overlap is None or overlap
        query_rows.append(dict(index=index, received_ros_ns=row['received_ros_ns'],
                               first_source_ns=sources[0], last_source_ns=sources[-1],
                               previous_query_overlap_source_ns=overlap,
                               query_duration_sec=row['query_duration_sec']))
        previous_sources = set(sources)
        for entry in entries:
            owners.add((entry['geometry_generation'], entry['frame_id'], entry['geometry_activation_stamp_ns']))
            assert entry['geometry_generation'] == payload['geometry_generation']
            assert entry['geometry_activation_stamp_ns'] == payload['geometry_activation_stamp_ns']
            assert entry['publication_outcome_unknown'] and not entry['exposure_clock_calibrated']
            assert entry['image_header_clock'] == 'host_read_completion'
            assert entry['evidence']['result'] == 'complete'
            assert entry['evidence']['regional_stationary_evidence']['nested_center_shadow']['reason'] == 'diagnostic_disabled'
            source = entry['source_stamp_ns']
            if source in by_source:
                assert by_source[source] == entry
                duplicate_entries += 1
            by_source[source] = entry
    assert len(owners) == 1
    entries = sorted(by_source.values(), key=lambda e: e['source_stamp_ns'])
    center_reasons, global_resets, stages, unavailable, supported = Counter(), Counter(), defaultdict(list), {}, {}
    resets, all_deltas, histories, intervals, current_interval = [], [], [], [], []
    for i, entry in enumerate(entries):
        evidence = entry['evidence']
        regional = evidence['regional_stationary_evidence']
        center = regional['regions'][0]
        assert center['name'] == 'center'
        center_reasons[center['visual']['reason']] += 1
        reason = regional['global_history_reset_reason']
        global_resets[str(reason)] += 1
        for k, v in evidence['stages_sec'].items():
            stages[k].append(v*1000)
        decision = evidence.get('stationary_pair_history')
        if decision:
            histories.append(dict(source_stamp_ns=entry['source_stamp_ns'], decision=decision))
        if reason and current_interval:
            intervals.append(current_interval)
            current_interval = []
        current_interval.append(entry)
        if i == 0:
            continue
        previous = entries[i-1]
        pe = previous['evidence']
        raw, old = evidence['raw_frames'], pe['raw_frames']
        for stream in ('color', 'depth'):
            assert raw[stream]['timestamp_domain'] == old[stream]['timestamp_domain']
            assert raw[stream]['stream_fps'] == old[stream]['stream_fps'] == 30
        delta = {s: stream_delta(old[s], raw[s]) for s in ('color', 'depth')}
        event = dict(index=i, source_stamp_ns=entry['source_stamp_ns'],
                     previous_source_stamp_ns=previous['source_stamp_ns'], reason=reason,
                     source_delta=delta,
                     previous_processing_ms=(pe['total_sec']-pe['host_wait']['elapsed_sec'])*1000,
                     current_wait_ms=evidence['host_wait']['elapsed_sec']*1000,
                     current_wait_thread_cpu_ms=evidence['host_wait']['thread_cpu_elapsed_sec']*1000,
                     filter_host_interval_ms=(evidence['stationary_pair_filter_host_monotonic']-pe['stationary_pair_filter_host_monotonic'])*1000,
                     between_wait_wall_ms=(evidence['host_wait']['enter']['monotonic_sec']-pe['host_wait']['exit']['monotonic_sec'])*1000,
                     between_wait_thread_cpu_ms=(evidence['host_wait']['enter']['thread_cpu_sec']-pe['host_wait']['exit']['thread_cpu_sec'])*1000,
                     previous_stages_sec=pe['stages_sec'], current_stages_sec=evidence['stages_sec'])
        all_deltas.append(event)
        if reason:
            resets.append(event)
    if current_interval:
        intervals.append(current_interval)
    for stream in ('color', 'depth'):
        unavailable[stream] = dict(Counter(k+'/'+str(v) for e in entries for k, v in e['evidence']['raw_frames'][stream]['unavailable'].items()))
        supported[stream] = dict(Counter(k for e in entries for k in e['evidence']['raw_frames'][stream]['metadata_raw']))
    groups = {}
    for name, subset in (
            ('color_counter_plus_one_slow', [r for r in resets if r['source_delta']['color']['count_step'] == 1]),
            ('both_counters_plus_two', [r for r in resets if all(r['source_delta'][s]['count_step'] == 2 for s in ('color', 'depth'))])):
        groups[name] = dict(count=len(subset), sources=[r['source_stamp_ns'] for r in subset],
                            previous_processing_over_33ms=sum(r['previous_processing_ms']>1000/30 for r in subset),
                            **{k:stats([r[k] for r in subset]) for k in (
                                'previous_processing_ms', 'current_wait_ms', 'current_wait_thread_cpu_ms',
                                'filter_host_interval_ms', 'between_wait_thread_cpu_ms', 'between_wait_wall_ms')})
    stable = [dict(first_source_ns=x[0]['source_stamp_ns'], last_source_ns=x[-1]['source_stamp_ns'],
                   complete_frames=len(x),
                   host_span_sec=x[-1]['read_completed_monotonic']-x[0]['read_completed_monotonic'],
                   center_qualified_frames=sum(e['evidence']['regional_stationary_evidence']['regions'][0]['qualified'] for e in x)) for x in intervals]
    processing = [(e['evidence']['total_sec']-e['evidence']['host_wait']['elapsed_sec'])*1000 for e in entries]
    span = entries[-1]['read_completed_monotonic']-entries[0]['read_completed_monotonic']
    summary = dict(query_count=len(rows), original_entry_count=len(rows)*32, unique_complete_sources=len(entries),
                   duplicate_identical_entries=duplicate_entries, complete_ring_overlap_verified=True,
                   first_source_ns=entries[0]['source_stamp_ns'], last_source_ns=entries[-1]['source_stamp_ns'],
                   successful_read_span_sec=span, owners=[list(x) for x in owners],
                   center_reasons=dict(center_reasons), global_resets=dict(global_resets),
                   center_qualified_fraction=center_reasons['stationary']/len(entries),
                   reset_count_steps=dict(Counter('%s/color+%s/depth+%s'%(r['reason'],r['source_delta']['color']['count_step'],r['source_delta']['depth']['count_step']) for r in resets)),
                   reset_groups=groups, complete_processing_ms=stats(processing),
                   processing_over_33ms=sum(x>1000/30 for x in processing),
                   stages_ms={k:stats(v) for k,v in stages.items()},
                   between_successful_reads_wait_boundary_thread_cpu_ms=stats([r['between_wait_thread_cpu_ms'] for r in all_deltas]),
                   between_successful_reads_wait_boundary_wall_ms=stats([r['between_wait_wall_ms'] for r in all_deltas]),
                   approximate_reader_one_core_cpu_fraction=sum(r['between_wait_thread_cpu_ms'] for r in all_deltas)/1000/span,
                   unavailable_metadata=unavailable, available_metadata=supported,
                   successful_history_decision_counts=dict(Counter(x['decision']['state']+'/'+x['decision']['reason'] for x in histories)),
                   stable_intervals=stable, longest_interval=max(stable,key=lambda x:x['complete_frames']))
    previous = json.loads(tap_report.read_text())
    return dict(schema=1, read_only=True, input=str(path), input_sha256=sha(path),
                analysis_source_sha256=sha(__file__), summary=summary,
                comparison_tap_input=dict(report=str(tap_report),sha256=sha(tap_report),summary=previous['summary']),
                query_rows=query_rows, reset_events=resets, successful_history_decisions=histories,
                all_source_intervals=all_deltas,
                metadata_proposal=dict(
                    scope='Optional three additional metadata fields on already returned SDK frames only; no device options, control or temperatures.',
                    source='realsense_manager.py:_raw_frame_diagnostics',
                    fields={'actual_exposure':'API unit microseconds; retain exact raw integer',
                            'gain_level':'Relative integer gain level, not dB',
                            'auto_exposure':'Raw mode indicator; zero means disabled; missing is not zero'},
                    installed_distribution_version='pyrealsense2 2.55.1.6486',
                    documentation='https://raw.githubusercontent.com/IntelRealSense/librealsense/v2.55.1/include/librealsense2/h/rs_frame.h',
                    support_not_yet_observed=True),
                limitations=[
                    'Rings contain successful read candidates only, not evidence of eventual ROS image publication.',
                    'Overlaps cover all successful reads within this interval, but rejected/failed SDK reads are not retained. Reject counts and previous_complete anchors cannot be reconstructed from this ring alone.',
                    'Same device/geometry activation, sequential windows, not randomized concurrent A/B. Improvements cannot all be attributed to tap removal.',
                    'No per-stage thread CPU measurement. Stage durations are wall time. Between-successful-read wait boundaries include camera bookkeeping and any intervening rejected SDK read processing.',
                    'Native FRAME_COUNTER and SENSOR_TIMESTAMP are unsupported on both streams for all saved frames. SDK frame numbers and ACTUAL_FPS alone cannot locate loss at exposure, firmware, USB/backend or SDK queue.',
                    'No recorded exposure/gain/auto-exposure values; adding optional metadata may produce only unsupported markers. No cause is inferred from unrecorded settings.',
                    'No freshness, frame pairing, source continuity or visual/depth quality threshold was modified.'])


if __name__ == '__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('folder',type=Path)
    parser.add_argument('--tap-report',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    result=analyze(args.folder,args.tap_report)
    mount=Path('/media/zhuyupei/SanStand')
    assert mount.is_mount() and mount in args.output.resolve().parents
    args.output.parent.mkdir(parents=True,exist_ok=True)
    with args.output.open('x') as stream:
        json.dump(result,stream,ensure_ascii=False,indent=2,allow_nan=False)
    print(json.dumps(result['summary'],ensure_ascii=False,indent=2,allow_nan=False))
