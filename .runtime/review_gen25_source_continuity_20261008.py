"""Offline arithmetic review of sealed gen25 metadata; no ROS/SDK imports."""
import argparse
import collections
import hashlib
import json
from pathlib import Path
import statistics
import zipfile


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def stats(values):
    values = list(values)
    return dict(count=len(values), min=min(values), median=statistics.median(values),
                p95=statistics.quantiles(values, n=100, method='inclusive')[94],
                max=max(values), mean=statistics.mean(values),
                above_nominal_33_333ms=sum(v > 1000 / 30 for v in values))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--selection', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    raw = args.selection / 'raw_support'
    scope_path = args.selection / 'analysis/gen25_raw_status_summary.json'
    prior = json.loads(scope_path.read_text())
    summary = json.loads((raw / 'summary.json').read_text())
    assert summary['archive_closed'] and summary['complete_without_writer_error']
    archive = raw / 'diagnostic.zip'
    assert digest(archive) == prior['input']['zip_sha256']
    entries, adjacency, metadata_hashes = {}, set(), {}
    with zipfile.ZipFile(archive) as z:
        names = sorted((n for n in z.namelist() if n.startswith('metadata/')
                        and n.endswith('.json')), key=lambda n: int(n[9:-5]))
        for name in names:
            data = z.read(name)
            sha = hashlib.sha256(data).hexdigest()
            assert sha == prior['input']['verified_entry_sha256'][name]
            metadata_hashes[name] = sha
            payload = json.loads(json.loads(data)['parameters']
                                 ['/camera_read_diagnostics']['payload_json'])
            assert type(payload['copy_errors']) is int and payload['copy_errors'] == 0
            chunk = payload['entries']
            adjacency.update((a['source_stamp_ns'], b['source_stamp_ns'])
                             for a, b in zip(chunk, chunk[1:]))
            for entry in chunk:
                source = entry['source_stamp_ns']
                assert source not in entries or entries[source] == entry
                entries[source] = entry
    ordered = sorted(entries.values(), key=lambda e: e['read_completed_monotonic'])
    window = prior['window']
    rows = []
    for previous, current in zip(ordered, ordered[1:]):
        source = current['source_stamp_ns']
        if not window['selection_stamp_ns'] <= source <= window['last_current_status_receipt_ns']:
            continue
        pe, ce = previous['evidence'], current['evidence']
        pc, cc = pe['raw_frames']['color'], ce['raw_frames']['color']
        pd, cd = pe['raw_frames']['depth'], ce['raw_frames']['depth']
        fps = cc['stream_fps']
        gd = cc['device_timestamp_ms'] - pc['device_timestamp_ms']
        nd = (cc['metadata_raw']['frame_timestamp'] - pc['metadata_raw']['frame_timestamp']) / 1000.
        hd = 1000 * (ce['stationary_pair_filter_host_monotonic'] - pe['stationary_pair_filter_host_monotonic'])
        gv, nv = (.25 / fps * 1000 <= v <= 1.75 / fps * 1000 for v in (gd, nd))
        predicates = dict(frame_number_not_plus_one=cc['frame_number'] != pc['frame_number'] + 1,
                          global_dt_outside_original_bounds=not gv,
                          domain_or_fps_changed=(cc['timestamp_domain'], fps) != (pc['timestamp_domain'], pc['stream_fps']),
                          host_dt_outside_original_bounds=not 0 < hd <= max(.1, 2.5 / fps) * 1000)
        evidence = ce['regional_stationary_evidence']
        rows.append(dict(source_stamp_ns=source, previous_source_stamp_ns=previous['source_stamp_ns'],
            adjacency_proven_in_original_snapshot=(previous['source_stamp_ns'], source) in adjacency,
            global_history_reset_reason=evidence['global_history_reset_reason'],
            center_visual_reason=evidence['regions'][0]['visual']['reason'],
            frame_number_delta=cc['frame_number'] - pc['frame_number'],
            depth_frame_number_delta=cd['frame_number'] - pd['frame_number'],
            global_dt_ms=gd, native_dt_ms=nd,
            depth_native_dt_ms=(cd['metadata_raw']['frame_timestamp'] - pd['metadata_raw']['frame_timestamp']) / 1000.,
            host_dt_ms=hd, global_dt_valid=gv, native_dt_valid=nv,
            triggered_predicates=[k for k, v in predicates.items() if v],
            pair_history=ce.get('stationary_pair_history'),
            previous_read_total_ms=1000 * pe['total_sec'],
            previous_after_wait_to_complete_ms=1000 * (previous['read_completed_monotonic'] - pe['host_wait']['exit']['monotonic_sec']),
            previous_complete_to_next_wait_enter_ms=1000 * (ce['host_wait']['enter']['monotonic_sec'] - previous['read_completed_monotonic']),
            current_sdk_wait_ms=1000 * ce['host_wait']['elapsed_sec']))
    resets = [r for r in rows if r['global_history_reset_reason'] == 'sdk_source_discontinuity']
    assert len(rows) == 287 and len(resets) == 98
    assert all(r['adjacency_proven_in_original_snapshot'] for r in resets)
    assert all(r['triggered_predicates'] and not r['native_dt_valid'] for r in resets)
    assert all(r['native_dt_valid'] == r['global_dt_valid'] for r in rows)
    group = collections.Counter(tuple(r['triggered_predicates']) for r in resets)
    report = dict(schema=1, offline_only=True, production_changed=False, device_or_ros_calls=False,
        script_sha256=digest(__file__), archive_path=str(archive), archive_sha256=digest(archive),
        original_summary_sha256=digest(raw / 'summary.json'), scope_path=str(scope_path), scope_sha256=digest(scope_path),
        metadata_sha256=metadata_hashes, window=window,
        result=dict(visible_reads=len(rows), reported_resets=len(resets),
            reset_predicate_counts=[dict(predicates=list(k), count=v) for k, v in group.items()],
            reset_native_delta_rounded_ms=dict(collections.Counter(str(round(r['native_dt_ms'], 2)) for r in resets)),
            reset_global_native_classification_disagreements=0,
            window_global_native_classification_disagreements=0,
            max_global_native_delta_difference_ms=max(abs(r['global_dt_ms'] - r['native_dt_ms']) for r in rows),
            unproven_snapshot_adjacency_sources=[r['source_stamp_ns'] for r in rows if not r['adjacency_proven_in_original_snapshot']],
            reset_previous_timing_ms={k: stats(r[k] for r in resets) for k in (
                'previous_read_total_ms', 'previous_after_wait_to_complete_ms',
                'previous_complete_to_next_wait_enter_ms', 'current_sdk_wait_ms')}),
        original_window_stage_stats=prior['sdk']['stages_seconds'],
        interpretation=[
            'Manager explicitly supplies raw color source; no color/depth key alternation is present in this path.',
            'All 98 resets have native source gaps too; replacing global time with native time would not change these decisions.',
            '96/98 preceding post-wait processing spans exceed a nominal 30 FPS period; outside-read gaps are at most 3.387 ms.',
            'This establishes read-internal budget overrun in the observed window, not the unique physical/SDK cause of each dropped source.',
            'All 287 center regions remain warming. Do not relax continuity/quality gates or regard raw depth as qualified temporal output.',
            'One history gap is not bridged for causal claims; one non-global reset is already discarded by pair-history color_not_immediate_successor.'
        ], rows=rows)
    report['source_files_sha256'] = {str(p): digest(p) for p in (
        Path('src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/vision/regional_stationary_depth.py'),
        Path('src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/vision/realsense_manager.py'))}
    args.output.mkdir(parents=False, exist_ok=False)
    out = args.output / 'report.json'
    out.write_text(json.dumps(report, indent=2, ensure_ascii=False) + '\n')
    print(json.dumps(dict(path=str(out), sha256=digest(out), result=report['result']), ensure_ascii=False))


if __name__ == '__main__':
    main()
