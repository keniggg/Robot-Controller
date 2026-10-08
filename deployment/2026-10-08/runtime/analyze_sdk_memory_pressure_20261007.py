#!/usr/bin/env python3
"""Offline analysis of sealed passive SDK/proc evidence; no ROS or SDK imports."""
import argparse
from collections import Counter, defaultdict
import hashlib
import json
import math
from pathlib import Path

FILES = {'resources': 'resources.jsonl', 'rpc': 'camera_rpc.jsonl', 'sdk': 'sdk_entries.jsonl'}
EXTERNAL = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log')
MOUNT = Path('/media/zhuyupei/SanStand')


def require(condition, reason):
    if not condition:
        raise ValueError(reason)


def canonical(value):
    return (json.dumps(value, ensure_ascii=False, separators=(',', ':'), allow_nan=False)+'\n').encode()


def sha(raw):
    return hashlib.sha256(raw).hexdigest()


def finite(value):
    return type(value) in (int, float) and math.isfinite(value)


def stats(values):
    values = sorted(v for v in values if finite(v))
    if not values:
        return {'count': 0}
    def pct(q):
        index = (len(values)-1)*q
        lo = int(index)
        return values[lo]+(values[min(lo+1, len(values)-1)]-values[lo])*(index-lo)
    return {'count': len(values), 'min': values[0], 'p50': pct(.5), 'p95': pct(.95),
            'max': values[-1], 'sum': sum(values), 'mean': sum(values)/len(values)}


def entry_key(entry):
    return (entry['geometry_generation'], entry.get('geometry_activation_stamp_ns'), entry['source_stamp_ns'])


def load_sealed(folder):
    folder = Path(folder)
    require(not list(folder.glob('*.tmp')), 'partial temporary files present')
    sums_raw = (folder/'SHA256SUMS').read_bytes()
    sums = {}
    for line in sums_raw.decode().splitlines():
        digest, name = line.split('  ', 1)
        require(Path(name).name == name and name not in sums, 'unsafe/duplicate checksum name')
        require(len(digest) == 64, 'malformed checksum')
        sums[name] = digest
    required = {'ready.json', 'outcome.json', *FILES.values()}
    require(set(sums) == required, 'unexpected checksum inventory')
    raw = {name: (folder/name).read_bytes() for name in required}
    for name, value in raw.items():
        require(sha(value) == sums[name], 'checksum mismatch: '+name)
    ready, outcome = (json.loads(raw[name]) for name in ('ready.json', 'outcome.json'))
    require(outcome.get('capture_closed') is True, 'capture not sealed')
    require(ready.get('schema') == outcome.get('schema') == 1, 'schema mismatch')
    require(ready['source_sha256'] == outcome['source_sha256'], 'observer source mismatch')
    for name in required-{'outcome.json'}:
        require(outcome['files'][name] == {'sha256': sums[name], 'bytes': len(raw[name])},
                'outcome file binding mismatch: '+name)
    rows = {kind: [json.loads(line) for line in raw[name].splitlines()] for kind, name in FILES.items()}
    for kind, values in rows.items():
        require(len(values) == outcome['record_state']['counts'][kind], 'row count mismatch: '+kind)
    return ready, outcome, rows, {'SHA256SUMS': sha(sums_raw), **sums}


def parse_psi(raw):
    if not isinstance(raw, str):
        return {}
    parsed = {}
    try:
        for line in raw.splitlines():
            fields = line.split()
            values = dict(item.split('=', 1) for item in fields[1:])
            parsed[fields[0]] = int(values['total'])
    except (ValueError, KeyError, IndexError):
        return {}
    return parsed


def deltas(before, after):
    result, invalid = {}, []
    for name in sorted(set(before) | set(after)):
        a, b = before.get(name), after.get(name)
        if type(a) is not int or type(b) is not int or b < a:
            invalid.append(name)
        else:
            result[name] = b-a
    return {'deltas': result, 'unavailable_or_decreased': invalid}


def counters(vmstat):
    return {k: v for k, v in vmstat.items() if not k.startswith('nr_') and type(v) is int}


def disk_counters(raw):
    result = {}
    if not isinstance(raw, str):
        return result
    for line in raw.splitlines():
        fields = line.split()
        if len(fields) >= 14:
            try:
                result[fields[2]] = {key: int(fields[index]) for key, index in (
                    ('read_operations', 3), ('read_sectors_512B', 5), ('read_ms', 6),
                    ('write_operations', 7), ('write_sectors_512B', 9), ('write_ms', 10),
                    ('busy_ms', 12), ('weighted_io_ms', 13))}
            except ValueError:
                continue
    return result


def sched(raw):
    try:
        values = list(map(int, raw.split()))
        return dict(zip(('runtime_ns', 'runqueue_wait_ns', 'timeslices'), values)) if len(values) == 3 else {}
    except (AttributeError, ValueError):
        return {}


def process_bracket(rows, bound, sched_enabled):
    result = {}
    for pid, identity in bound.items():
        samples = [r.get('processes', {}).get(pid, {}) for r in rows]
        expected = identity.get('start_ticks')
        valid = expected is not None and all(s.get('stat', {}).get('start_ticks') == expected
                                             and not any('identity_error' in k for k in s) for s in samples)
        process = {'comm': identity.get('comm'), 'start_ticks': expected, 'identity_valid': valid}
        result[pid] = process
        if not valid:
            process['reason'] = 'unavailable_or_changed_process_identity'
            continue
        keys = ('minflt', 'majflt', 'utime_ticks', 'stime_ticks')
        process['stat_changes'] = deltas({k: samples[0]['stat'].get(k) for k in keys},
                                         {k: samples[-1]['stat'].get(k) for k in keys})
        process['sampled_states'] = dict(Counter(s['stat'].get('state', 'unknown') for s in samples))
        process['status_samples'] = [dict(monotonic=s.get('status_sampled_monotonic'), values=s['status'])
                                      for s in samples if 'status' in s]
        process['tasks'] = {}
        for tid, task_id in identity.get('tasks', {}).items():
            ts = [s.get('tasks', {}).get(tid, {}) for s in samples]
            valid_task = all(t.get('stat', {}).get('start_ticks') == task_id['start_ticks']
                             and not any('identity_error' in k for k in t) for t in ts)
            task = {'comm': task_id.get('comm'), 'start_ticks': task_id['start_ticks'], 'identity_valid': valid_task}
            process['tasks'][tid] = task
            if not valid_task:
                task['reason'] = 'unavailable_or_changed_task_identity'
                continue
            task['sampled_states'] = dict(Counter(t['stat'].get('state', 'unknown') for t in ts))
            task['sampled_wchan'] = dict(Counter(str(t.get('wchan_raw', 'unavailable')) for t in ts))
            task['schedstat_delta'] = deltas(sched(ts[0].get('schedstat_raw')), sched(ts[-1].get('schedstat_raw')))
            task['runqueue_wait_counter_interpretable'] = bool(sched_enabled and
                'runqueue_wait_ns' in task['schedstat_delta']['deltas'])
            task['state_observations'] = [dict(start_monotonic=r['started_monotonic'],
                finish_monotonic=r['finished_monotonic'], state=t['stat'].get('state'),
                wchan=t.get('wchan_raw')) for r,t in zip(rows,ts)]
    return result


def resource_association(rows, start, end, ready, same_boot):
    result = {'available': False, 'event_start_monotonic': start, 'event_end_monotonic': end,
              'causality_established': False}
    if not same_boot:
        return dict(result, reason='boot_identity_mismatch')
    before = [i for i, r in enumerate(rows) if r['finished_monotonic'] <= start]
    after = [i for i, r in enumerate(rows) if r['started_monotonic'] >= end]
    if not before or not after:
        return dict(result, reason='not_bracketed_by_complete_resource_rows')
    lo, hi = before[-1], after[0]
    selected = rows[lo:hi+1]
    gaps = [b['started_monotonic']-a['started_monotonic'] for a, b in zip(selected, selected[1:])]
    max_gap = max(gaps, default=0)
    result.update(resource_row_indices=[lo, hi], maximum_start_gap_sec=max_gap,
                  resource_row_count=len(selected), sample_start_monotonic=selected[0]['started_monotonic'],
                  sample_end_monotonic=selected[-1]['finished_monotonic'],
                  preceding_margin_sec=start-selected[0]['finished_monotonic'],
                  following_margin_sec=selected[-1]['started_monotonic']-end,
                  fully_inside_wait_row_indices=[lo+i for i, r in enumerate(selected)
                    if r['started_monotonic'] >= start and r['finished_monotonic'] <= end])
    # This is an analysis quality flag; it never changes a camera/control gate.
    if max_gap > 2*ready['resource_period_sec']:
        return dict(result, reason='resource_sampling_gap_no_counter_bridge')
    a, b = selected[0], selected[-1]
    span = b['started_monotonic']-a['started_monotonic']
    psi = {}
    for kind in ('cpu', 'memory', 'io'):
        change = deltas(parse_psi(a.get('pressure', {}).get(kind)), parse_psi(b.get('pressure', {}).get(kind)))
        psi[kind] = dict(change, bracket_fraction_percent={k: v/1e6/span*100
                              for k, v in change['deltas'].items()} if span > 0 else {})
    da, db = disk_counters(a.get('diskstats')), disk_counters(b.get('diskstats'))
    result.update(available=True, reason='adjacent_resource_rows_bracket_wait_not_atomic',
                  counter_span_sec=span, psi=psi,
                  vmstat=deltas(counters(a.get('vmstat', {})), counters(b.get('vmstat', {}))),
                  meminfo_before=a.get('meminfo'), meminfo_after=b.get('meminfo'),
                  vmstat_gauges_before={k:v for k,v in a.get('vmstat', {}).items() if k.startswith('nr_')},
                  vmstat_gauges_after={k:v for k,v in b.get('vmstat', {}).items() if k.startswith('nr_')},
                  diskstats={name: deltas(da.get(name, {}), db.get(name, {})) for name in sorted(set(da)|set(db))},
                  processes=process_bracket(selected, ready['bound_processes'], ready.get('sched_schedstats') == '1'),
                  resource_quality_gaps=[{'index': lo+i, 'gaps': r['quality_gaps']} for i,r in enumerate(selected)
                                         if r.get('quality_gaps')])
    return result


def source_delta(previous, current):
    result = {}
    for stream in ('color', 'depth'):
        a, b = previous.get(stream, {}), current.get(stream, {})
        item = {'previous_raw': a, 'current_raw': b, 'delta': {}}
        for key in ('frame_number', 'device_timestamp_ms'):
            if finite(a.get(key)) and finite(b.get(key)):
                item['delta'][key] = b[key]-a[key]
        for key in ('frame_timestamp', 'backend_timestamp', 'time_of_arrival'):
            va, vb = a.get('metadata_raw', {}).get(key), b.get('metadata_raw', {}).get(key)
            if finite(va) and finite(vb):
                item['delta'][key] = vb-va
        item['same_timestamp_domain'] = a.get('timestamp_domain') == b.get('timestamp_domain')
        result[stream] = item
    return result


def analyze(folder, camera_pid=None, long_wait_sec=.1):
    require(finite(long_wait_sec) and long_wait_sec > 0, 'invalid diagnostic wait threshold')
    ready, outcome, rows, hashes = load_sealed(folder)
    same_boot = isinstance(ready.get('boot_id'), str) and ready['boot_id'] == outcome.get('boot_id_at_end')
    start, end = outcome['capture_start_monotonic'], outcome['capture_end_monotonic']
    require(finite(start) and finite(end) and end > start, 'invalid capture window')
    if camera_pid is not None:
        require(str(camera_pid) in ready['bound_processes'], 'camera PID not in original bound processes')
    resources = rows['resources']
    for i, row in enumerate(resources):
        require(finite(row.get('started_monotonic')) and finite(row.get('finished_monotonic'))
                and row['finished_monotonic'] >= row['started_monotonic'], 'invalid resource clocks')
        if i:
            require(row['started_monotonic'] >= resources[i-1]['finished_monotonic'], 'overlapping/reversed resource rows')
    entries = {}
    for row in rows['sdk']:
        entry = row['original_entry']; key = entry_key(entry)
        require(type(key[2]) is int and key[2] > 0 and finite(entry.get('read_completed_monotonic')), 'bad SDK source')
        require(sha(canonical(entry)) == row['entry_sha256'], 'SDK entry digest mismatch')
        require(key not in entries, 'duplicate original SDK entry')
        entries[key] = row
    rpc_coverage = []; previous = None
    for i, row in enumerate(rows['rpc']):
        if not row.get('schema_valid'):
            rpc_coverage.append({'index': i, 'valid': False, 'error': row.get('error')}); previous = None; continue
        refs = row['entry_references']; keys = [tuple(ref['key']) for ref in refs]
        require(len(keys) == len(set(keys)), 'duplicate RPC entry reference')
        for ref,key in zip(refs,keys):
            require(key in entries and entries[key]['entry_sha256'] == ref['entry_sha256'], 'unbound RPC entry reference')
        if keys:
            require([key[2] for key in keys] == sorted(key[2] for key in keys), 'nonmonotonic RPC sources')
        owner = (row['payload_without_entries'].get('geometry_generation'),
                 row['payload_without_entries'].get('geometry_activation_stamp_ns'))
        require(all(key[:2] == owner for key in keys), 'RPC owner/reference mismatch')
        same_owner = previous is not None and previous[0] == owner
        overlap = len(set(keys) & previous[1]) if same_owner else None
        copy_errors = row['payload_without_entries'].get('copy_errors')
        copy_history_complete = (type(copy_errors) is int and copy_errors == 0 and
                                 previous is not None and previous[2] == 0)
        rpc_coverage.append({'index': i, 'valid': True, 'owner': list(owner), 'entry_count': len(keys),
                             'producer_copy_errors': copy_errors,
                             'previous_same_owner_overlap_count': overlap,
                             'no_successful_history_gap_proven': bool(same_owner and overlap and copy_history_complete),
                             'elapsed_sec': row.get('elapsed_sec')})
        previous = (owner, set(keys), copy_errors if type(copy_errors) is int else None)
    ordered = sorted((r['original_entry'] for r in entries.values()), key=lambda e:e['read_completed_monotonic'])
    selected = [e for e in ordered if start <= e['read_completed_monotonic'] < end]
    stages = defaultdict(list); waits = []; wholly_window_waits = []; events = []; resets = Counter(); owners = set(); previous = None
    for entry in selected:
        owner = (entry['geometry_generation'], entry.get('geometry_activation_stamp_ns'), entry.get('frame_id'))
        owners.add(owner); ev = entry['evidence']
        require(ev.get('result') == 'complete', 'SDK entry is not successful read evidence')
        for k,v in ev.get('stages_sec', {}).items():
            if finite(v): stages[k].append(v)
        resets[str(ev.get('regional_stationary_evidence', {}).get('global_history_reset_reason'))] += 1
        wait = ev.get('host_wait', {}); duration = wait.get('elapsed_sec')
        if finite(duration): waits.append(duration)
        ws = wait.get('enter', {}).get('monotonic_sec'); we = wait.get('exit', {}).get('monotonic_sec')
        valid_clock = (finite(duration) and finite(ws) and finite(we) and we >= ws
                       and abs(we-ws-duration) < 1e-6)
        if valid_clock and start <= ws <= we < end:
            wholly_window_waits.append(duration)
        if finite(duration) and duration > long_wait_sec:
            event = {'source_stamp_ns': entry['source_stamp_ns'], 'owner': list(owner), 'wait': wait,
                     'read_completed_monotonic': entry['read_completed_monotonic'], 'stages_sec': ev.get('stages_sec'),
                     'raw_frames': ev.get('raw_frames'), 'regional_global_reset_reason':
                     ev.get('regional_stationary_evidence', {}).get('global_history_reset_reason'),
                     'wait_clock_valid': valid_clock, 'wholly_within_capture_window': bool(valid_clock and start <= ws <= we < end)}
            if previous is not None and previous[0] == owner:
                event['previous_successful_source_ns'] = previous[1]['source_stamp_ns']
                event['stream_changes_from_previous_visible_success'] = source_delta(previous[1]['evidence'].get('raw_frames', {}), ev.get('raw_frames', {}))
            event['resources'] = (resource_association(resources, ws, we, ready, same_boot)
                                   if event['wholly_within_capture_window'] else
                                   {'available': False, 'reason': 'wait_outside_capture_or_invalid_clock'})
            events.append(event)
        previous = (owner,entry)
    resource_gaps = [b['started_monotonic']-a['started_monotonic'] for a,b in zip(resources,resources[1:])]
    return {'schema': 1, 'offline_only': True, 'input': str(Path(folder).resolve()), 'input_sha256': hashes,
            'analysis_source_sha256': sha(Path(__file__).read_bytes()), 'camera_pid_label': camera_pid,
            'integrity': {'sealed_hashes_valid': True, 'same_boot': same_boot,
                          'observer_integrity_passed': outcome.get('observer_integrity_passed'),
                          'bound_processes': ready['bound_processes'], 'observer_errors': outcome.get('errors'),
                          'rpc_errors': outcome.get('rpc_errors'), 'proc_quality_gaps': outcome.get('proc_quality_gaps')},
            'window': {'start_monotonic': start, 'end_monotonic': end, 'duration_sec': end-start,
                       'unique_entries_total': len(entries), 'completed_inside': len(selected),
                       'excluded_outside': len(entries)-len(selected), 'owners': [list(x) for x in sorted(owners,key=str)],
                       'resources': len(resources), 'resource_start_gaps_sec': stats(resource_gaps),
                       'resource_gaps_over_two_periods': sum(x>2*ready['resource_period_sec'] for x in resource_gaps)},
            'summary': {'long_wait_diagnostic_threshold_sec': long_wait_sec, 'long_wait_count': len(events),
                        'wait_seconds': stats(waits),
                        'wait_seconds_scope': 'Reads completed inside capture; their waits may have begun before capture.',
                        'wait_seconds_wholly_inside_capture': stats(wholly_window_waits),
                        'long_wait_count_wholly_inside_capture': sum(e['wholly_within_capture_window'] for e in events),
                        'stages_seconds': {k:stats(v) for k,v in stages.items()},
                        'global_history_reset_reasons': dict(resets)},
            'rpc_history_coverage': rpc_coverage, 'long_wait_events': events,
            'monitor_self_usage': outcome.get('monitor_self_usage'),
            'limitations': ['Association is not causation; resource reads and deltas bracket rather than exactly integrate the wait.',
                'Boot mismatch blocks resource association; per-process and per-TID identity mismatch blocks their deltas.',
                'Resource gaps over two configured periods are exposed and never bridged for event counter changes.',
                'schedstat runqueue zeros when disabled, wchan 0, and absent fields cannot demonstrate no waiting.',
                'No SDK read-thread TID is inferred from a generic Python thread name; sampled states only identify their bound TIDs.',
                'Only successful read history is available, not every rejected/raw SDK return or guaranteed ROS publication.',
                'ACTUAL_FPS is SDK derived and API frame counters can fall back to software counts; neither independently establishes sensor production rate.',
                'Global memory PSI, allocstall, balloon and I/O counters cannot alone locate a fault in camera/USB/VM/host or establish out-of-memory.'],
            'problem_resolved': False, 'physical_validation_passed': False}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('input', type=Path)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--camera-pid', type=int)
    args = parser.parse_args()
    report = analyze(args.input, args.camera_pid)
    out = args.output.resolve()
    require(MOUNT.is_mount() and EXTERNAL in out.parents, 'output must remain on mounted SanStand ros_log')
    require(not out.exists() and out.parent.is_dir(), 'new output in existing directory required')
    with out.open('xb') as stream:
        stream.write(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False).encode()+b'\n')
    print(json.dumps({'path': str(out), 'sha256': sha(out.read_bytes()), 'summary': report['summary']}, ensure_ascii=False))


if __name__ == '__main__':
    main()
