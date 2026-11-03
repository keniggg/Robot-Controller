#!/usr/bin/env python3
"""Analyze sealed scalar timing + existing read records; no ROS/SDK imports.

Bindings are exact call ids and containment in the actual read invocation.
There is no nearest-frame matching or interpolation of per-stage CPU time.
"""
import argparse
from bisect import bisect_left
from collections import Counter, defaultdict
import hashlib
import json
import math
from pathlib import Path
import statistics


STAGES = {'read', 'remember', 'submit', 'publish', 'temporal_status'}
EXT = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def require(condition, message):
    if not condition:
        raise ValueError(message)


def finite(value):
    return type(value) in (int, float) and math.isfinite(value)


def stats(values):
    a = sorted(float(v) for v in values if finite(v))
    if not a:
        return {'n': 0}
    def percentile(q):
        pos = (len(a)-1)*q
        lo = int(pos)
        return a[lo]+(a[min(lo+1, len(a)-1)]-a[lo])*(pos-lo)
    return dict(n=len(a), mean=statistics.mean(a), total=sum(a), minimum=a[0],
                p50=percentile(.5), p95=percentile(.95), p99=percentile(.99), maximum=a[-1])


def ns(seconds):
    require(finite(seconds), 'missing recorded monotonic clock')
    return round(seconds*1e9)


def pair_events(events):
    calls = defaultdict(dict)
    for row in events:
        call_id = row.get('call_id')
        require(type(call_id) is int and call_id > 0, 'invalid call id')
        require(row.get('event') in ('start', 'end'), 'invalid event type')
        require(row['event'] not in calls[call_id], 'duplicate event for call id')
        require(row.get('stage') in STAGES, 'unknown stage')
        calls[call_id][row['event']] = row
    spans = []
    for call_id, pair in calls.items():
        require(set(pair) == {'start', 'end'}, 'unpaired call event')
        a, b = pair['start'], pair['end']
        for key in ('stage', 'thread_id', 'owner_id', 'source_stamp_ns', 'read_call_id', 'read_owner_id'):
            require(a.get(key) == b.get(key), 'call identity changed: '+key)
        for key in ('monotonic_ns', 'thread_cpu_ns'):
            require(type(a.get(key)) is int and type(b.get(key)) is int and b[key] >= a[key],
                    'invalid or reversed call clock')
        wall, cpu = (b['monotonic_ns']-a['monotonic_ns'])/1e6, (b['thread_cpu_ns']-a['thread_cpu_ns'])/1e6
        spans.append(dict(call_id=call_id, stage=a['stage'], start_ns=a['monotonic_ns'],
            end_ns=b['monotonic_ns'], thread_id=a['thread_id'], owner_id=a['owner_id'],
            source_stamp_ns=a.get('source_stamp_ns'), read_call_id=a.get('read_call_id'),
            read_owner_id=a.get('read_owner_id'), wall_ms=wall, thread_cpu_ms=cpu,
            wall_minus_thread_cpu_ms=wall-cpu, entry_hook_ms=b.get('entry_hook_ns', 0)/1e6,
            entry_hook_cpu_ms=b.get('entry_hook_cpu_ns', 0)/1e6,
            exception_type=b.get('exception_type'), crosses_window_end=b.get('crosses_window_end')))
    return sorted(spans, key=lambda s: (s['start_ns'], s['call_id']))


def overlap_ms(lo, hi, spans):
    intervals = sorted((max(lo, s['start_ns']), min(hi, s['end_ns'])) for s in spans
                       if s['end_ns'] > lo and s['start_ns'] < hi)
    total = 0
    right = lo
    for a, b in intervals:
        total += max(0, b-max(right, a))
        right = max(right, b)
    return total/1e6


def source_delta(a, b):
    result = {}
    for stream in ('color', 'depth'):
        old, new = a.get('raw_frames', {}).get(stream, {}), b.get('raw_frames', {}).get(stream, {})
        if not (type(old.get('frame_number')) is int and type(new.get('frame_number')) is int
                and finite(old.get('device_timestamp_ms')) and finite(new.get('device_timestamp_ms'))):
            result[stream] = {'available': False}
            continue
        result[stream] = dict(available=True, previous_frame_number=old['frame_number'],
            frame_number=new['frame_number'], counter_delta=new['frame_number']-old['frame_number'],
            device_delta_ms=new['device_timestamp_ms']-old['device_timestamp_ms'],
            same_domain_fps=(old.get('timestamp_domain') == new.get('timestamp_domain')
                             and old.get('stream_fps') == new.get('stream_fps')))
        old_metadata, new_metadata = old.get('metadata_raw', {}), new.get('metadata_raw', {})
        names = ('frame_timestamp', 'backend_timestamp', 'time_of_arrival', 'actual_fps')
        result[stream]['previous_metadata_raw'] = {k: old_metadata.get(k) for k in names}
        result[stream]['current_metadata_raw'] = {k: new_metadata.get(k) for k in names}
        result[stream]['metadata_raw_deltas'] = {
            k: new_metadata[k]-old_metadata[k] for k in names
            if type(old_metadata.get(k)) is int and type(new_metadata.get(k)) is int}
        native = result[stream]['metadata_raw_deltas'].get('frame_timestamp')
        if native is not None:
            result[stream]['native_delta_positive_no_wrap_adjustment'] = native > 0
            if native > 0:
                # rs_frame.h v2.56.5 declares FRAME_TIMESTAMP in microseconds.
                # Do not unwrap or normalize backend/arrival clocks here.
                result[stream]['native_frame_delta_ms'] = native/1000
                result[stream]['global_minus_native_delta_ms'] = result[stream]['device_delta_ms']-native/1000
    return result


def summarize_reads(records):
    fields = ('read_wall_ms', 'read_cpu_ms', 'manager_processing_wall_ms', 'read_minus_wait_cpu_ms',
        'read_observer_tail_ms', 'regional_wall_ms', 'spatial_wall_ms', 'remember_wall_ms',
        'remember_cpu_ms', 'read_non_cpu_ms', 'processing_publish_overlap_ms',
        'processing_temporal_overlap_ms', 'previous_inter_read_ms')
    return dict(count=len(records), complete=sum(r['result'] == 'complete' for r in records),
                **{key: stats([r.get(key) for r in records]) for key in fields})


def analyze(events, rows):
    spans = pair_events(events)
    stages = {stage: [s for s in spans if s['stage'] == stage] for stage in STAGES}
    read_by_id = {s['call_id']: s for s in stages['read']}
    remember_by_read = {}
    source_reads = {}
    unmatched_remember = []
    for s in stages['remember']:
        read = read_by_id.get(s['read_call_id'])
        if read is None:
            unmatched_remember.append(s)
            continue
        require(s['thread_id'] == read['thread_id'] and s['owner_id'] == read['owner_id']
                and s['start_ns'] >= read['end_ns'], 'remember is not downstream of same owner/thread read')
        require(s['source_stamp_ns'] is not None and read['call_id'] not in remember_by_read,
                'duplicate or absent successful source binding')
        require(s['source_stamp_ns'] not in source_reads, 'duplicate source stamp across reads')
        remember_by_read[read['call_id']] = s
        source_reads[s['source_stamp_ns']] = read['call_id']

    ordered_reads = stages['read']
    starts = [r['start_ns'] for r in ordered_reads]
    bindings, outside_rows = {}, []
    for index, row in enumerate(rows):
        lo, hi = ns(row['read_started_monotonic']), ns(row['read_completed_monotonic'])
        at = bisect_left(starts, lo)
        options = [r for r in ordered_reads[max(0, at-1):at+1] if r['start_ns'] <= lo <= hi <= r['end_ns']]
        require(len(options) <= 1, 'ambiguous nested read metadata')
        if not options:
            outside_rows.append(index)
            continue
        read = options[0]
        require(read['call_id'] not in bindings, 'multiple SDK reads in one node call')
        bindings[read['call_id']] = (index, row)

    records = []
    previous = None
    previous_complete = None
    reset_events = []
    for read in ordered_reads:
        bound = bindings.get(read['call_id'])
        if bound is None:
            continue
        index, row = bound
        d = row['existing_read_diagnostics']
        remember = remember_by_read.get(read['call_id'])
        wait = d.get('host_wait', {})
        wait_ms = wait.get('elapsed_sec')
        wait_cpu = wait.get('thread_cpu_elapsed_sec')
        stage = d.get('stages_sec', {})
        exit_ns = ns(wait['exit']['monotonic_sec']) if wait.get('exit') else read['start_ns']
        processing_end = ns(row['read_completed_monotonic'])
        regional = d.get('regional_stationary_evidence', {})
        record = dict(read_call_id=read['call_id'], metadata_row=index, result=d.get('result'),
            source_stamp_ns=remember['source_stamp_ns'] if remember else None,
            read_start_ns=read['start_ns'], read_end_ns=read['end_ns'],
            read_wall_ms=read['wall_ms'], read_cpu_ms=read['thread_cpu_ms'],
            read_non_cpu_ms=read['wall_minus_thread_cpu_ms'],
            manager_processing_wall_ms=(d['total_sec']-wait_ms)*1000 if finite(wait_ms) and finite(d.get('total_sec')) else None,
            read_minus_wait_cpu_ms=read['thread_cpu_ms']-wait_cpu*1000 if finite(wait_cpu) else None,
            read_observer_tail_ms=(read['end_ns']-processing_end)/1e6,
            regional_wall_ms=stage.get('regional_stationary_depth', 0)*1000,
            spatial_wall_ms=stage.get('depth_filter_0_spatial', 0)*1000,
            remember_wall_ms=remember['wall_ms'] if remember else None,
            remember_cpu_ms=remember['thread_cpu_ms'] if remember else None,
            previous_inter_read_ms=(read['start_ns']-previous['read']['end_ns'])/1e6 if previous else None,
            sdk_reset_reason=regional.get('global_history_reset_reason'),
            center_reason=(regional.get('regions') or [{}])[0].get('visual', {}).get('reason'),
            processing_publish_overlap_ms=overlap_ms(exit_ns, processing_end, stages['publish']),
            processing_temporal_overlap_ms=overlap_ms(exit_ns, processing_end, stages['temporal_status']),
            whole_read_publish_overlap_ms=overlap_ms(read['start_ns'], read['end_ns'], stages['publish']),
            whole_read_temporal_overlap_ms=overlap_ms(read['start_ns'], read['end_ns'], stages['temporal_status']))
        records.append(record)
        if record['sdk_reset_reason']:
            event = dict(current=record, previous_packet=previous['record'] if previous else None,
                previous_complete=previous_complete['record'] if previous_complete else None,
                previous_packet_source_delta=source_delta(previous['diagnostic'], d) if previous else None,
                previous_complete_source_delta=source_delta(previous_complete['diagnostic'], d) if previous_complete else None)
            if previous_complete:
                old_host = previous_complete['diagnostic'].get('stationary_pair_filter_host_monotonic')
                new_host = d.get('stationary_pair_filter_host_monotonic')
                event['filter_host_interval_since_previous_complete_ms'] = (
                    (new_host-old_host)*1000 if finite(old_host) and finite(new_host) else None)
            if previous:
                boundary_lo = ns(previous['diagnostic']['host_wait']['exit']['monotonic_sec'])
                boundary_hi = ns(wait['enter']['monotonic_sec'])
                event['preceding_wait_exit_to_next_enter'] = dict(wall_ms=(boundary_hi-boundary_lo)/1e6,
                    publish_overlap_ms=overlap_ms(boundary_lo, boundary_hi, stages['publish']),
                    temporal_overlap_ms=overlap_ms(boundary_lo, boundary_hi, stages['temporal_status']))
            reset_events.append(event)
        previous = dict(read=read, diagnostic=d, record=record)
        if d.get('result') == 'complete':
            previous_complete = previous

    stage_summary = {stage: dict(count=len(values), threads=dict(Counter(str(s['thread_id']) for s in values)),
        exceptions=dict(Counter(str(s['exception_type']) for s in values)),
        **{key: stats([s[key] for s in values]) for key in (
            'wall_ms', 'thread_cpu_ms', 'wall_minus_thread_cpu_ms', 'entry_hook_ms', 'entry_hook_cpu_ms')})
        for stage, values in stages.items()}
    complete = [r for r in records if r['result'] == 'complete']
    groups = {'all': summarize_reads(records), 'complete': summarize_reads(complete)}
    for label, key in (('publish', 'processing_publish_overlap_ms'), ('temporal', 'processing_temporal_overlap_ms')):
        for present in (True, False):
            group = [r for r in complete if (r[key] > 0) is present]
            groups[label+('_overlap' if present else '_no_overlap')] = summarize_reads(group)
    previous_ids = {e['previous_packet']['read_call_id'] for e in reset_events if e['previous_packet']}
    groups['immediately_precedes_reset'] = summarize_reads([r for r in records if r['read_call_id'] in previous_ids])
    groups['does_not_immediately_precede_reset'] = summarize_reads([r for r in records if r['read_call_id'] not in previous_ids])
    source_unmatched = {stage: [s['source_stamp_ns'] for s in stages[stage] if s['source_stamp_ns'] not in source_reads]
                        for stage in ('submit', 'publish')}
    first_start = min(s['start_ns'] for s in spans)
    startup = {stage: {label: {
        key: stats([s[key] for s in values if (s['start_ns']-first_start < 3e9) is early])
        for key in ('wall_ms', 'thread_cpu_ms', 'wall_minus_thread_cpu_ms')}
        for label, early in (('first_3_seconds', True), ('after_3_seconds', False))}
        for stage, values in stages.items()}
    return dict(schema=1, diagnostic_only=True, stages=stage_summary, groups=groups,
        startup_descriptive_split=startup,
        bindings=dict(read_event_calls=len(ordered_reads), metadata_rows=len(rows), exact_contained_rows=len(bindings),
            unmatched_metadata_rows=outside_rows, unmatched_read_call_ids=sorted(set(read_by_id)-set(bindings)),
            matched_successful_sources=len(source_reads), unmatched_remember=unmatched_remember,
            unmatched_publication_sources=source_unmatched),
        reason_counts=dict(Counter(str(r['sdk_reset_reason']) for r in records)),
        center_reason_counts=dict(Counter(str(r['center_reason']) for r in complete)),
        reset_events=reset_events, reads=records,
        timer_calls=stages['temporal_status'],
        limitations=[
            'Stage spans include reported entry-hook overhead; existing observer copy work lies inside the read span.',
            'read_observer_tail_ms is observer copy/queue plus original _timed_stage trailing bookkeeping, not isolated copy CPU.',
            'SDK stage diagnostics are wall times. Read-minus-wait CPU includes the metadata observer and wrapper, not per-filter CPU.',
            'Wall-minus-thread-CPU includes scheduling, SDK waits and Python contention; it does not identify GIL ownership.',
            'Overlapping work is a measured association, not a randomized causal comparison. Image content, reseeding and filter state can differ.',
            'Remember links existing host-read completion source stamps, not calibrated exposure time or final subscriber delivery.',
            'The three-second startup split is post-hoc descriptive, not a preplanned causal A/B or a new control threshold.',
            'Records outside the actual timing window are retained as unmatched; no nearest-source substitute is made.'])


def analyze_folder(folder):
    paths = {k: folder/v for k, v in (
        ('events', 'node_timings/events.jsonl'), ('timing_summary', 'node_timings/summary.json'),
        ('reads', 'read_diagnostics.jsonl'), ('read_summary', 'read_diagnostics.summary.json'),
        ('loaded', 'loaded_observer_modules.json'))}
    frozen = {str(p): sha(p) for p in paths.values()}
    timing = json.loads(paths['timing_summary'].read_text())
    summary = json.loads(paths['read_summary'].read_text())
    require(timing.get('complete') is True and timing.get('sealed') is True and timing.get('window_complete') is True,
            'node timing window is not complete/sealed')
    require(timing.get('class_hooks_restored') is True and timing.get('observed_bound_callbacks_restored') is True,
            'node timing hooks not restored')
    require(timing.get('events_sha256') == sha(paths['events']), 'timing event hash differs')
    require(summary.get('original_read_restored') is True and not summary.get('errors')
            and summary.get('queued_unwritten') == 0, 'read observer not sealed')
    require(timing.get('source_implementation') == json.loads(paths['loaded'].read_text()), 'source manifest mismatch')
    events = [json.loads(line) for line in paths['events'].read_text().splitlines()]
    rows = [json.loads(line) for line in paths['reads'].read_text().splitlines()]
    require(len(events) == timing['counts']['written'] == timing['counts']['started']*2, 'event counts differ')
    require(len(rows) == summary['counts']['observed'] == summary['counts']['written'] == summary['counts']['enqueued'],
            'existing read counts differ')
    report = analyze(events, rows)
    require(frozen == {str(p): sha(p) for p in paths.values()}, 'input changed during offline analysis')
    report.update(input_folder=str(folder), input_sha256=frozen, analysis_source_sha256=sha(__file__))
    return report


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('folder', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    require(Path('/media/zhuyupei/SanStand').is_mount() and EXT.resolve() in args.output.resolve().parents,
            'mounted external output required')
    report = analyze_folder(args.folder)
    encoded = json.dumps(report, ensure_ascii=False, separators=(',', ':'), allow_nan=False)+'\n'
    require(len(encoded.encode()) < 4*1024*1024, 'bounded analysis output exceeded')
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open('x') as stream:
        stream.write(encoded)
    print(json.dumps(dict(output=str(args.output), sha256=sha(args.output), bindings=report['bindings'],
                         stages=report['stages'], groups=report['groups'], reason_counts=report['reason_counts']),
                     ensure_ascii=False, indent=2, allow_nan=False))
