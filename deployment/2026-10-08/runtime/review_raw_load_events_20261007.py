"""Bounded offline event/sidecar review; no ROS initialization or pixel decoding."""
import argparse
from collections import Counter
import hashlib
import importlib.util
import json
from pathlib import Path
import sys

HERE = Path(__file__).resolve().parent
EXT = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
spec = importlib.util.spec_from_file_location('passive_analysis', HERE/'analyze_passive_load_round_20261007.py')
A = importlib.util.module_from_spec(spec)
spec.loader.exec_module(A)


def sha(path):
    h = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            h.update(block)
    return h.hexdigest()


def within(path, root):
    try:
        path.relative_to(root)
        return True
    except ValueError:
        return False


def bracket_system(samples, start, end):
    before = [r for r in samples if r['monotonic'] <= start]
    after = [r for r in samples if r['monotonic'] >= end]
    if not before or not after:
        return {'available': False, 'reason': 'event_not_bracketed_by_saved_samples'}
    left, right = before[-1], after[0]
    out = A.system_summary(samples, left['monotonic'] - 1e-6, right['monotonic'] + 1e-6)
    out['brackets_event_not_exact_event_integration'] = True
    out['preceding_margin_sec'] = start-left['monotonic']
    out['following_margin_sec'] = right['monotonic']-end
    out['samples_inside_wait'] = [r['monotonic'] for r in samples if start <= r['monotonic'] <= end]
    return out


def native_delta(before, after):
    result = {}
    for stream in ('color', 'depth'):
        x = before.get('raw_frames', {}).get(stream, {})
        y = after.get('raw_frames', {}).get(stream, {})
        mx, my = x.get('metadata_raw', {}), y.get('metadata_raw', {})
        row = {'previous_raw': x, 'current_raw': y, 'deltas': {}, 'not_recorded_fields': []}
        for key in ('frame_number', 'device_timestamp_ms'):
            row['deltas'][key] = y[key]-x[key] if A.finite(x.get(key)) and A.finite(y.get(key)) else None
        for key in ('frame_timestamp', 'backend_timestamp', 'time_of_arrival'):
            row['deltas'][key] = my[key]-mx[key] if A.finite(mx.get(key)) and A.finite(my.get(key)) else None
        for key in ('actual_exposure', 'gain_level', 'auto_exposure'):
            if key not in my:
                row['not_recorded_fields'].append(key)
        result[stream] = row
    return result


def run(capture):
    original = Path(capture)
    capture = original.resolve()
    if not within(capture, EXT) or original.is_symlink():
        raise ValueError('external_capture_required')
    declarations = {}
    for line in (capture/'SHA256SUMS').read_text().splitlines():
        digest, relative = line.split('  ', 1)
        declarations[relative] = digest
    checked = {}
    for name in ('observations.json', 'B_raw_recording/outcome.json', 'B_raw_recording/ready.json', 'B_raw_recording/raw_rgbd.bag'):
        path = capture/name
        if path.is_symlink() or not path.is_file() or not within(path.resolve(),capture):
            raise ValueError('invalid_sealed_path:'+name)
        actual = sha(path)
        if actual != declarations.get(name):
            raise ValueError('sealed_hash_mismatch:'+name)
        checked[name] = {'sha256': actual, 'bytes': path.stat().st_size, 'recomputed': True}
    d = json.loads((capture/'observations.json').read_text())
    outcome = json.loads((capture/'B_raw_recording/outcome.json').read_text())
    summary = json.loads((capture/'summary.json').read_text())
    assert outcome == d['recorder_outcome'], 'embedded_outcome_differs'
    assert summary['observations_sha256'] == checked['observations.json']['sha256']
    assert all(summary['artifact_hashes'][k] == v['sha256'] for k,v in checked.items())
    phases = [p for p in d['phases'] if p['name'] != 'B_owned_process_lifetime']
    sdk = sorted(d['sdk_reads'], key=lambda r: r['monotonic'])
    samples = sorted(d['samples'], key=lambda r:r['monotonic'])
    pairs = outcome['pairs']
    pair_sources = {p['source_ns'] for p in pairs}
    arrivals = outcome['receive']['arrivals']
    stream_sources = {name:{r['source_ns'] for r in rows} for name,rows in arrivals.items()}
    only = {name:sorted(stream_sources[name]-stream_sources[other]) for name,other in [('color','depth'),('depth','color')]}
    sdk_by_source = {}
    for row in sdk:
        sdk_by_source.setdefault(row['source_ns'], []).append(row)
    sdk_matches = Counter(len(sdk_by_source.get(ns, [])) for ns in pair_sources)
    header_mismatches = [p['source_ns'] for p in pairs if any(v['source_ns'] != p['source_ns'] for v in p['images'].values())]
    arrival_missing = [p['source_ns'] for p in pairs if any(p['source_ns'] not in stream_sources[n] for n in ('color','depth'))]

    # ROS bag INDEX only: seeks over compressed chunks and reads timestamps/counts.
    # No ROS master, nodes, read_messages, decompression, or Image deserialization.
    sys.path.insert(0, '/opt/ros/noetic/lib/python3/dist-packages')
    import rosbag
    bag_review = {}
    with rosbag.Bag(str(capture/'B_raw_recording/raw_rgbd.bag'), 'r') as bag:
        for connection in bag._get_connections():
            times = [e.time.to_nsec() for indexes in bag._get_indexes([connection]) for e in indexes]
            bag_review[connection.topic] = {'connection_id':connection.id, 'datatype':connection.datatype,
                'message_count':len(times), 'unique_index_stamps':len(set(times)),
                'index_stamps_match_all_pair_sources': sorted(times) == sorted(pair_sources),
                'first_source_ns':min(times) if times else None, 'last_source_ns':max(times) if times else None}

    events = []
    for index, row in enumerate(sdk):
        wait = row.get('host_wait') or {}
        if not A.finite(wait.get('elapsed_sec')) or wait['elapsed_sec'] <= .1 or index == 0:
            continue
        start, end = wait['enter']['monotonic_sec'], wait['exit']['monotonic_sec']
        phase = next((p for p in phases if p['start'] <= start <= end <= row['monotonic'] < p['end']), None)
        previous = sdk[index-1]
        event = {'source_ns':row['source_ns'], 'previous_source_ns':previous['source_ns'],
            'phase':phase['name'] if phase else 'cross_phase_or_outside',
            'wholly_within_phase':phase is not None,
            'wait_enter_offset_sec':start-phase['start'] if phase else None,
            'wait':wait, 'read_completed_monotonic':row['monotonic'],
            'read_gap_sec':row['monotonic']-previous['monotonic'],
            'source_gap_sec':(row['source_ns']-previous['source_ns'])/1e9,
            'previous_completed_to_wait_enter_sec':start-previous['monotonic'],
            'wait_exit_to_read_completed_sec':row['monotonic']-end,
            'native':native_delta(previous,row), 'stages_sec':row.get('stages_sec'),
            'system_bracket':bracket_system(samples,start,end)}
        if phase and phase['name'].startswith('A2'):
            life = next(p for p in d['phases'] if p['name']=='B_owned_process_lifetime')
            event['since_owned_child_exit_sec'] = start-life['end']
            event['since_bag_closed_and_fsynced_report_sec'] = start-outcome['completed_monotonic']
        events.append(event)

    bins = []
    for name in ('B_raw_rgbd_recording', 'A2_without_extra_raw_recording'):
        phase = next(p for p in phases if p['name']==name)
        offset = 0
        while phase['start']+offset < phase['end']:
            lo, hi = phase['start']+offset, min(phase['start']+offset+2,phase['end'])
            rows = [r for r in sdk if lo <= r['monotonic'] < hi]
            waits = [r['host_wait'] for r in rows if lo <= r['host_wait']['enter']['monotonic_sec'] <= r['host_wait']['exit']['monotonic_sec'] < hi]
            received = [p for p in pairs if lo <= max(x['receipt_monotonic'] for x in p['images'].values()) < hi]
            written = [p for p in pairs if lo <= p['write_completed_monotonic'] < hi]
            bins.append({'phase':name,'offset_start_sec':offset,'duration_sec':hi-lo,
                'sdk_reads_completed':len(rows),'sdk_wait_ms_fully_inside':A.stats([w['elapsed_sec']*1000 for w in waits]),
                'sdk_wait_cpu_ms_fully_inside':A.stats([w['thread_cpu_elapsed_sec']*1000 for w in waits]),
                'sdk_source_intervals_ms':A.interval_stats(sdk,lo,hi),
                'pair_receipts':len(received),'pair_write_completions':len(written),
                'color_arrivals':sum(lo <= r['receipt_monotonic'] < hi for r in arrivals['color']),
                'depth_arrivals':sum(lo <= r['receipt_monotonic'] < hi for r in arrivals['depth']),
                'system':A.system_summary(samples,lo,hi)})
            offset += 2
    pairs_by_write = sorted(pairs,key=lambda p:p['write_completed_monotonic'])
    completion_gaps = []
    for before,after in zip(pairs_by_write,pairs_by_write[1:]):
        lo,hi=before['write_completed_monotonic'],after['write_completed_monotonic']
        if hi-lo < .2:
            continue
        rows = [r for r in sdk if lo <= r['monotonic'] <= hi]
        completion_gaps.append({'start_monotonic':lo,'end_monotonic':hi,'seconds':hi-lo,
            'before_source_ns':before['source_ns'],'after_source_ns':after['source_ns'],
            'sdk_reads_completed_inside':len(rows),
            'sdk_wait_ms_fully_inside':A.stats([r['host_wait']['elapsed_sec']*1000 for r in rows if lo<=r['host_wait']['enter']['monotonic_sec']<=r['host_wait']['exit']['monotonic_sec']<=hi]),
            'system_bracket':bracket_system(samples,lo,hi)})
    report={'schema':1,'offline_only':True,'ros_or_hardware_calls':False,'pixels_decoded':False,
        'input':str(capture),'source_sha256':sha(Path(__file__)),
        'base_analyzer_sha256':sha(HERE/'analyze_passive_load_round_20261007.py'),
        'verified_original_sha256':checked,'summary_sha256':sha(capture/'summary.json'),
        'embedded_outcome_matches_standalone':True,'manifest_matches_summary':True,
        'bag_index_only':bag_review,'sidecar_sources':{
            'pairs':len(pairs),'unique_pair_sources':len(pair_sources),
            'received_counts':{k:len(v) for k,v in arrivals.items()},
            'unique_received_sources':{k:len(v) for k,v in stream_sources.items()},
            'intersection_count':len(stream_sources['color'] & stream_sources['depth']),
            'pairs_equal_full_received_intersection':pair_sources == stream_sources['color'] & stream_sources['depth'],
            'single_stream_sources':only,'sdk_history_match_multiplicity':dict(sdk_matches),
            'pair_header_source_mismatches':header_mismatches,'pair_source_missing_from_arrivals':arrival_missing,
            'original_reported_receive_counts':outcome['receive']['counts'],
            'unmatched_counter_issue':'Original ExactPairs.stop is called before drain and again in finally. Second call sees cleared pending and overwrites unmatched end counts with zero. Historical sidecar is preserved; distinct arrival sets independently recover 29 color-only and 29 depth-only sources.',
            'original_capture_valid_scope':'Successfully closed exact pairs; does not assert absence of unpaired arrivals or complete publisher stream.'},
        'actual_recording_rate':{'capture_seconds':outcome['capture_end_monotonic']-outcome['capture_start_monotonic'],
            'bag_bytes_verified':checked['B_raw_recording/raw_rgbd.bag']['bytes'],
            'compressed_bag_MiB_per_capture_second':outcome['bag_bytes']/2**20/(outcome['capture_end_monotonic']-outcome['capture_start_monotonic']),
            'raw_pixel_bytes_from_headers':sum(v['data_bytes'] for p in pairs for v in p['images'].values()),
            'rates_are_file_capture_averages_not_device_block_throughput':True},
        'long_sdk_wait_events_over_100ms':events,'two_second_bins':bins,'write_completion_gaps_over_200ms':completion_gaps,
        'limits':[
            'Only original sealed files are read; bag SHA is recomputed and its metadata index timestamps/counts checked. Compressed image payloads are not decoded; original Image header equality relies on the source-bound sidecar/write assertion.',
            'Events require wait enter, exit and read completion fully within a phase. Cross-phase intervals remain separate in base report. Two-second bin wait maxima exclude waits crossing bin boundaries; event list preserves these.',
            'System deltas are the saved samples bracketing events, not exact event integration; metadata RPC/proc reads follow the sample monotonic timestamp. PSI is system-wide, not camera-thread causality.',
            'Raw recording adds subscription, original-message retention, LZ4 compression and external writes. Its actual 15.94 MiB/s is higher than historical about5.5 MiB/s; this is not a pure disk contrast.',
            'B close includes bag close and fsync; cooldown is recorded separately. A2 still can carry unrelated or delayed background effects. A long A2 gap prevents a claim that active recorder write is necessary for the observed interruption.',
            'Arm is powered off; absence of feedback is not interpreted as a stall. Exposure/gain/auto-exposure fields absent from this archive cannot be inferred or queried.',
            'Low wait-thread CPU shows blocking/descheduling, not whether the cause is SDK, USB, kernel, guest memory reclaim, hypervisor or host memory. Native timing gap and global PSI association do not uniquely identify cause.' ]}
    target=capture/'analysis'/'sidecar_and_event_review.json'
    if target.exists():
        raise FileExistsError(target)
    payload=A.encoded(report)
    target.write_bytes(payload)
    digest=hashlib.sha256(payload).hexdigest()
    target.with_suffix('.json.sha256').write_text(digest+'  '+target.name+'\n')
    print(json.dumps({'report':str(target),'sha256':digest,'long_waits':len(events),'sidecar_pairs':len(pairs),'bag_index_topics':bag_review},ensure_ascii=False))


if __name__=='__main__':
    parser=argparse.ArgumentParser()
    parser.add_argument('--capture',required=True)
    run(parser.parse_args().capture)
