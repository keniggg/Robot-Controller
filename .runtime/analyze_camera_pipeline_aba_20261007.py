#!/usr/bin/env python3
"""Offline analysis of SEALED inner A/B/A metadata. Never import ROS or SDK."""
import argparse
from collections import Counter
import hashlib
import json
import math
from pathlib import Path
import statistics

EXT = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
PHASES = ('A1', 'B', 'A2')


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def finite(value):
    return type(value) in (int, float) and math.isfinite(value)


def distribution(values):
    a = sorted(float(v) for v in values if finite(v))
    if not a:
        return dict(count=0)
    def percentile(q):
        pos = (len(a)-1)*q
        lo = int(pos)
        return a[lo]+(a[min(lo+1, len(a)-1)]-a[lo])*(pos-lo)
    return dict(count=len(a), minimum=a[0], maximum=a[-1], mean=statistics.mean(a),
                p50=percentile(.5), p90=percentile(.9), p95=percentile(.95), p99=percentile(.99))


def source(row, stream):
    return row.get('diagnostic', {}).get('raw_frames', {}).get(stream, {})


def source_key(raw):
    fields = ('frame_number', 'device_timestamp_ms', 'timestamp_domain', 'stream_fps', 'profile_sha256')
    if (raw.get('present') is not True or type(raw.get('frame_number')) is not int
            or not finite(raw.get('device_timestamp_ms')) or not finite(raw.get('stream_fps'))
            or raw['stream_fps'] <= 0 or not isinstance(raw.get('timestamp_domain'), str)):
        return None
    return tuple(raw.get(k) for k in fields)


def host_exit(row):
    return row.get('diagnostic', {}).get('host_wait', {}).get('exit', {}).get('monotonic_sec')


def event(row, stream):
    return dict(requested_phase=row.get('requested_phase'), phase=row.get('phase'),
        elapsed_start_sec=row.get('elapsed_start_sec'), elapsed_end_sec=row.get('elapsed_end_sec'),
        raw=source(row, stream), host_wait=row.get('diagnostic', {}).get('host_wait'),
        read_thread_cpu_sec=row.get('read_thread_cpu_sec'),
        pair_timing=row.get('diagnostic', {}).get('pair_timing'),
        stages_sec=row.get('diagnostic', {}).get('stages_sec'))


def stream_analysis(rows, phase, stream, accepted_only=False):
    seen = set()
    previous = None
    previous_key = None
    deltas = []
    repeats = invalid = cross_boundary = 0
    counts = Counter()
    anomalies = []
    selected = 0
    previous_packet = None
    for row in rows:
        prior_packet = previous_packet
        previous_packet = row
        if accepted_only and row.get('diagnostic', {}).get('pair_timing', {}).get('accepted') is not True:
            continue
        raw = source(row, stream)
        key = source_key(raw)
        in_phase = row.get('phase') == phase
        if in_phase:
            selected += 1
        if key is None:
            invalid += int(in_phase)
            continue
        if key in seen:
            repeats += int(in_phase)
            continue
        seen.add(key)
        if in_phase:
            counts['unique_sources'] += 1
            if previous is not None:
                if previous.get('phase') != phase:
                    cross_boundary += 1
                else:
                    old = source(previous, stream)
                    same_domain = key[2:] == previous_key[2:]
                    dn = raw['frame_number']-old['frame_number']
                    dt = raw['device_timestamp_ms']-old['device_timestamp_ms']
                    host_dt = (host_exit(row)-host_exit(previous)
                               if finite(host_exit(row)) and finite(host_exit(previous)) else None)
                    timing = dict(counter_delta=dn, device_dt_ms=dt,
                                  host_exit_dt_ms=host_dt*1000 if host_dt is not None else None,
                                  same_domain_fps_profile=same_domain)
                    deltas.append(timing)
                    reason = []
                    if not same_domain:
                        reason.append('domain_fps_or_profile_changed')
                    if dn != 1:
                        reason.append('counter_not_plus_one')
                    if same_domain and not 250./raw['stream_fps'] <= dt <= 1750./raw['stream_fps']:
                        reason.append('device_interval_outside_original_source_gate')
                    if host_dt is None or not 0 < host_dt <= max(.1, 2.5/raw['stream_fps']):
                        reason.append('host_interval_outside_original_source_gate')
                    counts['counter_delta_'+str(dn)] += 1
                    if reason:
                        for cause in reason:
                            counts[cause] += 1
                        prior_diagnostic = (prior_packet or {}).get('diagnostic', {})
                        wait = prior_diagnostic.get('host_wait', {}).get('elapsed_sec')
                        total = prior_diagnostic.get('total_sec')
                        processing = total-wait if finite(total) and finite(wait) else None
                        anomalies.append(dict(reasons=reason, **timing,
                            preceding_read_processing_ms=processing*1000 if processing is not None else None,
                            previous_unique_source_read=event(previous, stream),
                            immediately_preceding_packet=event(prior_packet, stream) if prior_packet else None,
                            current_read=event(row, stream)))
        previous, previous_key = row, key
    return dict(selected_returned_packets=selected, exact_repeats_removed=repeats,
        unusable_source_packets=invalid, cross_boundary_first_source_intervals_excluded=cross_boundary,
        counts=dict(counts), internal_interval_count=len(deltas),
        device_dt_ms=distribution([d['device_dt_ms'] for d in deltas]),
        host_exit_dt_ms=distribution([d['host_exit_dt_ms'] for d in deltas]),
        anomalies=anomalies,
        source_gate_note='Original color gate numerical interval rules applied descriptively per stream; no new filtering authority.')


def extra_metadata(rows, phase, stream):
    seen = set()
    values = {name:[] for name in ('actual_exposure', 'gain_level', 'auto_exposure')}
    unavailable = {name:Counter() for name in values}
    original_supported = Counter()
    original_unavailable = Counter()
    for row in rows:
        if row.get('phase') != phase:
            continue
        raw = source(row, stream)
        key = source_key(raw)
        if key is None or key in seen:
            continue
        seen.add(key)
        extra = raw.get('diagnostic_extra_metadata', {})
        for name in values:
            value = extra.get('values', {}).get(name)
            if type(value) is int:
                values[name].append(value)
            else:
                unavailable[name][extra.get('unavailable', {}).get(name, 'not_recorded')] += 1
        for name in raw.get('metadata_raw', {}):
            original_supported[name] += 1
        for name, reason in raw.get('unavailable', {}).items():
            original_unavailable[name+':'+str(reason)] += 1
    return dict(unique_source_count=len(seen), fields={name:dict(
        observed=distribution(values[name]), exact_value_counts=dict(Counter(map(str, values[name]))),
        unavailable=dict(unavailable[name])) for name in values},
        original_metadata_supported=dict(original_supported), original_metadata_unavailable=dict(original_unavailable))


def analyze(rows, summary):
    reads = [r for r in rows if r.get('kind') == 'read']
    result = dict(schema=1, kind='sealed_camera_pipeline_aba_analysis',
        acquisition=summary['acquisition'], limitations=[
            'Counters and actual_fps are SDK outputs, not guaranteed native hardware counters.',
            'Host wait time includes SDK waiting and host scheduling; not an exposure-only measurement.',
            'Per-stage times are wall times, not isolated CPU measurements.',
            'B has no image extraction/processing/publication or visual qualification.',
            'Boundary reads and cross-stage source intervals excluded from within-stage comparisons.',
            'One A/B/A run cannot identify USB, firmware, host scheduling or exposure as sole upstream cause.'],
        boundary_reads=[r for r in reads if r.get('phase') == 'boundary'], phases={})
    for phase in PHASES:
        selected = [r for r in reads if r.get('phase') == phase]
        stages = {}
        regional_counts = Counter()
        center = Counter()
        reasons = Counter()
        history = Counter()
        processing, wait, cpu = [], [], []
        for row in selected:
            d = row.get('diagnostic', {})
            ws = d.get('host_wait', {}).get('elapsed_sec')
            total = d.get('total_sec')
            wait.append(ws)
            cpu.append(row.get('read_thread_cpu_sec'))
            if finite(ws) and finite(total):
                processing.append(total-ws)
            reasons[str(d.get('pair_timing', {}).get('reason'))] += 1
            if isinstance(d.get('pair_history'), dict):
                history[str(d['pair_history'].get('state'))+':'+str(d['pair_history'].get('reason'))] += 1
            for name, seconds in d.get('stages_sec', {}).items():
                stages.setdefault(name, []).append(seconds*1000 if finite(seconds) else None)
            regional = d.get('regional', {})
            if regional:
                regional_counts['records'] += 1
                regional_counts['global_reset_'+str(regional.get('global_history_reset_reason'))] += 1
                for region in regional.get('regions', []):
                    if region.get('name') == 'center':
                        center['records'] += 1
                        center['qualified'] += int(region.get('qualified') is True)
                        center['filter_applied'] += int(region.get('filter_applied') is True)
                        center['history_reset'] += int(region.get('history_reset') is True)
                        center['reason_'+str(region.get('visual', {}).get('reason'))] += 1
        accepted = sum(r.get('diagnostic', {}).get('pair_timing', {}).get('accepted') is True for r in selected)
        result['phases'][phase] = dict(reads=len(selected), pair_accepted=accepted,
            pair_rejected_or_unknown=len(selected)-accepted, pair_reasons=dict(reasons),
            result_counts=dict(Counter(r.get('diagnostic', {}).get('result') for r in selected)),
            read_thread_cpu_ms=distribution([v*1000 if finite(v) else None for v in cpu]),
            read_thread_cpu_total_sec=sum(v for v in cpu if finite(v)),
            cpu_accounting_note='Sum of read call CPU only; excludes inter-read guards/JSON and writer thread.',
            wait_ms=distribution([v*1000 if finite(v) else None for v in wait]),
            processing_wall_ms=distribution([v*1000 for v in processing]),
            processing_gt_30fps_budget=sum(v > 1./30 for v in processing),
            stages_wall_ms={name:distribution(v) for name,v in stages.items()},
            regional=dict(regional_counts), center=dict(center),
            pair_history_decisions=dict(history),
            raw_streams={s:stream_analysis(reads, phase, s) for s in ('color','depth')},
            accepted_pair_streams={s:stream_analysis(reads, phase, s, True) for s in ('color','depth')},
            metadata={s:extra_metadata(reads, phase, s) for s in ('color','depth')})
    return result


def sealed_input(directory):
    directory = Path(directory)
    summary_path, metadata = directory/'summary.json', directory/'metadata.jsonl'
    summary = json.loads(summary_path.read_text())
    if not (summary.get('complete') is True and summary.get('pipeline_closed') is True
            and summary.get('writer', {}).get('drained') is True
            and summary.get('writer', {}).get('worker_alive') is False):
        raise ValueError('sealed_complete_input_required')
    before = sha(metadata)
    if before != summary['writer']['sha256']:
        raise ValueError('sealed_metadata_hash_mismatch')
    rows = [json.loads(line) for line in metadata.read_text().splitlines()]
    if len(rows) != summary['writer']['rows_written'] or before != sha(metadata):
        raise ValueError('metadata_changed_or_row_count_mismatch')
    return rows, summary, dict(summary_sha256=sha(summary_path), metadata_sha256=before)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--probe', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    args = p.parse_args()
    rows, summary, hashes = sealed_input(args.probe)
    result = analyze(rows, summary)
    result.update(input_path=str(args.probe.resolve()), input_sha256=hashes,
                  analysis_source_sha256=sha(__file__))
    out = args.output.resolve()
    if EXT not in out.parents or not EXT.parent.parent.is_mount():
        raise ValueError('SanStand_output_required')
    out.mkdir(parents=True, exist_ok=False)
    (out/'report.json').write_text(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False)+'\n')
    print(json.dumps(dict(output=str(out), reads=sum(p['reads'] for p in result['phases'].values()),
                         report_sha256=sha(out/'report.json'))))


if __name__ == '__main__':
    main()
