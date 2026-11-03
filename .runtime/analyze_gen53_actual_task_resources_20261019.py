#!/usr/bin/env python3
"""Read sealed or explicitly partial GEN53 proc/log prefixes; no live ROS/device access."""
import argparse
import collections
import hashlib
import json
import os
from pathlib import Path
import re
import time

M = Path('/media/zhuyupei/SanStand')
E = M / '实验记录/Robot-Controller'
N = E / 'ros_log/full_latest_gen41_restart_20261018T011222Z'
V = N / 'verification'
S = V / 'gen53_actual_task_resources_r1'
R = E / 'grasp_records/20261019T014854.594021Z_26025c111cc64bdf85bf143a55e2f053'
L = N / 'gen53_runtime/compute_gateway_deployment_r1/consumers.log'
DEPLOYMENT_SHA = 'd3e8c419d724e7eb941403c2bf6d47e0bc5f4cb77dbb549f93158d6c0d16202f'


def sha(data):
    return hashlib.sha256(data).hexdigest()


def prefix(path):
    data = path.read_bytes()
    data = data[:data.rfind(b'\n') + 1]
    return data, dict(path=str(path), prefix_bytes=len(data), prefix_sha256=sha(data),
                      prefix_only=True)


def ns_from_log(line):
    match = re.search(rb'\[(\d{10})\.(\d+)\]', line)
    return None if match is None else int(match[1]) * 10**9 + int(match[2].ljust(9, b'0')[:9])


def read_evidence():
    events, metrics, refs = [], [], []
    raw, reference = prefix(L)
    refs.append(reference)
    offset = 0
    for number, line in enumerate(raw.splitlines(keepends=True), 1):
        stamp = ns_from_log(line)
        marker = None
        for token, kind in (
            (b'[Grasp] MOVE_PREGRASP capturing initial observation pose', 'first_recorded_task_stage'),
            (b'[Grasp] MOVE_PREGRASP moving 6D pregrasp', 'observation_motion_stage'),
            (b'Observation endpoint compensation completed:', 'observation_endpoint_completed'),
            (b'6D planning phase signal: near_field', 'near_field_started'),
            (b'Task failed:', 'task_terminal_failure'),
            (b'[Grasp] FAILED', 'task_terminal_failure'),
            (b'[Grasp] DONE', 'task_terminal_done'),
            (b'Task completed', 'task_terminal_done')):
            if token in line:
                marker = kind
                break
        if marker and stamp:
            events.append(dict(kind=marker, stamp_ns=stamp, path=str(L), line_number=number,
                offset=offset, size=len(line), sha256=sha(line), raw_line=line.decode()))
        if b'remote 6D pipeline metrics: ' in line:
            value = json.loads(line.split(b'remote 6D pipeline metrics: ', 1)[1])
            metrics.append(dict(recorded_ns=stamp, request_id=value.get('request_id'),
                generation=value.get('generation'), target_epoch=value.get('target_epoch'),
                event=value.get('event'), status=value.get('status'),
                snapshot_evidence=value.get('snapshot_evidence'),
                request_preparation_profile=value.get('request_preparation_profile'),
                timings={key: value.get(key) for key in ('ros_prepare_ms', 'end_to_end_ms',
                    'result_age_ms', 'encode_ms', 'transport_ms', 'decode_ms',
                    'wsl_total_ms', 'wsl_inference_ms', 'wsl_postprocess_ms', 'wsl_preprocess_ms')},
                path=str(L), line_number=number, offset=offset, size=len(line), sha256=sha(line)))
        offset += len(line)
    runner, reference = prefix(R / 'runner.log')
    refs.append(reference)
    offset = 0
    selection = None
    for number, line in enumerate(runner.splitlines(keepends=True), 1):
        if line.startswith(b'USER_REQUESTED_GRASP '):
            value = json.loads(line.split(b' ', 1)[1])
            selection = value['selection']
            events.append(dict(kind='selection_commit',
                stamp_ns=value['runner_startup_timing']['selection_commit_ros_ns'],
                selection=selection, path=str(R / 'runner.log'), line_number=number,
                offset=offset, size=len(line), sha256=sha(line)))
        offset += len(line)
    return sorted(events, key=lambda row: row['stamp_ns']), metrics, refs, selection


def summarize(label, lower, upper, rows, hz):
    pairs = [(a, b) for a, b in zip(rows, rows[1:])
             if lower <= a['recorded_ns'] < b['recorded_ns'] <= upper and
             abs((b['recorded_ns'] - a['recorded_ns']) -
                 (b['monotonic_ns'] - a['monotonic_ns'])) <= 20_000_000]
    if not pairs:
        return dict(label=label, nominal_start_ns=lower, nominal_end_ns=upper,
                    complete_intervals=0, measured_duration_sec=0.)
    wall = sum((b['monotonic_ns'] - a['monotonic_ns']) / 1e9 for a, b in pairs)
    processes, host, disk = {}, collections.Counter(), collections.Counter()
    errors, peak_cores = [], 0.
    for a, b in pairs:
        old = {(p['pid'], p['start_ticks']): p for p in a['processes']}
        interval_cpu = 0.
        for p in b['processes']:
            identity = (p['pid'], p['start_ticks'])
            previous = old.get(identity)
            entry = processes.setdefault('%d:%d' % identity, dict(pid=p['pid'],
                start_ticks=p['start_ticks'], name=p['name'], attribution=p['attribution'],
                cpu_ticks=0, rss_peak_bytes=0, matched_intervals=0, io=collections.Counter(),
                affinity_cpus=p['affinity_cpus'], thread_peak=0))
            entry['rss_peak_bytes'] = max(entry['rss_peak_bytes'], p['rss_bytes'])
            entry['thread_peak'] = max(entry['thread_peak'], p['threads'])
            if previous is None:
                continue
            delta = p['cpu_ticks'] - previous['cpu_ticks']
            assert delta >= 0
            entry['cpu_ticks'] += delta
            entry['matched_intervals'] += 1
            interval_cpu += delta / hz
            for key, value in p['io'].items():
                if key in previous['io']:
                    entry['io'][key] += value - previous['io'][key]
        peak_cores = max(peak_cores, interval_cpu / ((b['monotonic_ns'] - a['monotonic_ns']) / 1e9))
        delta = [y - x for x, y in zip(a['cpu_ticks'][:8], b['cpu_ticks'][:8])]
        host.update(total=sum(delta), busy=sum(delta) - delta[3] - delta[4], iowait=delta[4])
        old_disk, new_disk = a['external_filesystem_device'], b['external_filesystem_device']
        if (old_disk['major'], old_disk['minor']) == (new_disk['major'], new_disk['minor']):
            aa, bb = old_disk['counters'], new_disk['counters']
            if aa and bb:
                for key in ('read_bytes', 'write_bytes', 'busy_ms', 'weighted_busy_ms'):
                    disk[key] += bb[key] - aa[key]
        errors.extend(a['identity_errors'])
        errors.extend(b['identity_errors'])
    cpu = sum(row['cpu_ticks'] for row in processes.values()) / hz
    for row in processes.values():
        row['cpu_sec'] = row['cpu_ticks'] / hz
        row['cpu_percent_one_core_100'] = 100. * row['cpu_sec'] / wall
        row['io'] = dict(row['io'])
    return dict(label=label, nominal_start_ns=lower, nominal_end_ns=upper,
        first_complete_interval_start_ns=pairs[0][0]['recorded_ns'],
        last_complete_interval_end_ns=pairs[-1][1]['recorded_ns'],
        complete_intervals=len(pairs), measured_duration_sec=wall,
        tracked_equivalent_cores=cpu / wall, tracked_peak_interval_equivalent_cores=peak_cores,
        host_busy_percent=100. * host['busy'] / host['total'],
        host_iowait_percent=100. * host['iowait'] / host['total'],
        external_device_delta=dict(disk),
        available_memory_min_kib=min(row['memory_kib']['MemAvailable'] for pair in pairs for row in pair),
        identity_error_count=len(errors), processes=processes)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    output = args.output.resolve()
    assert M.is_mount() and E / 'ros_log' in output.parents and output.parent.exists()
    assert output.parent.stat().st_dev == M.stat().st_dev and not output.exists()
    assert not os.statvfs(M).f_flag & os.ST_RDONLY
    meta = json.loads((S / 'metadata.json').read_text())
    assert meta['deployment_sha256'] == DEPLOYMENT_SHA
    raw, sample_ref = prefix(S / 'samples.jsonl')
    rows = [json.loads(line) for line in raw.splitlines()]
    assert len(rows) >= 2
    for row in rows:
        keys = [(p['pid'], p['start_ticks']) for p in row['processes']]
        assert len(keys) == len(set(keys)), 'duplicate process identity'
    events, metrics, refs, selection = read_evidence()
    assert selection and selection['generation'] == 2
    anchors = {}
    for event in events:
        if event['stamp_ns'] >= selection['stamp_ns']:
            anchors.setdefault(event['kind'], event['stamp_ns'])
    start, end = rows[0]['recorded_ns'], rows[-1]['recorded_ns']
    terminal = min((event['stamp_ns'] for event in events
                    if event['kind'].startswith('task_terminal_')), default=None)
    order = [('selection_commit', 'preselection_idle_and_launcher'),
             ('first_recorded_task_stage', 'selected_target_preview'),
             ('observation_motion_stage', 'initial_task_and_observation_preflight'),
             ('observation_endpoint_completed', 'observation_motion_and_arrival_wait'),
             ('near_field_started', 'observation_station_requalification')]
    windows, boundaries = [], []
    previous = start
    for key, label in order:
        if key in anchors and previous < anchors[key] < end:
            windows.append(summarize(label, previous, anchors[key], rows, meta['clock_ticks_per_second']))
            previous = anchors[key]
            boundaries.append(previous)
    if previous < end:
        until = min(terminal, end) if terminal is not None else end
        if until > previous:
            windows.append(summarize('near_field_until_terminal' if terminal else 'near_field_partial',
                                     previous, until, rows, meta['clock_ticks_per_second']))
        if terminal and terminal < end:
            boundaries.append(terminal)
            windows.append(summarize('post_terminal_recorder_tail_not_task_load', terminal, end,
                                     rows, meta['clock_ticks_per_second']))
    aggregate = summarize('selected_task_pipeline_only_excludes_idle_prefix_and_terminal_tail',
        anchors['selection_commit'], min(terminal, end) if terminal else end,
        rows, meta['clock_ticks_per_second'])
    manifest = json.loads((R / 'manifest.json').read_text())
    ready = json.loads((R / 'recorder_ready.json').read_text())
    identities = {(p['pid'], p['start_ticks']): p['attribution'] for row in rows for p in row['processes']}
    assert (manifest['owner']['pid'], int(manifest['owner']['start_ticks'])) in identities
    assert any(pid == ready['pid'] and item['role'] == 'recording' for (pid, ticks), item in identities.items())
    sealed = (S / 'result.json').is_file()
    final = json.loads((S / 'result.json').read_text()) if sealed else None
    if sealed:
        assert final['samples_sha256'] == sample_ref['prefix_sha256']
    output.mkdir()
    result = dict(kind='GEN53_actual_task_proc_phase_analysis', created_ns=time.time_ns(),
        sample_count=len(rows), sample_start_ns=start, sample_end_ns=end,
        sampler_sealed=sealed, observed_terminal=terminal is not None,
        complete_recording_closed=manifest.get('recorder_closed', False),
        record_id=manifest['record_id'], selection=selection,
        actual_task_started_event_not_yet_extracted=True,
        first_task_stage_is_not_claimed_as_exact_task_started=True,
        deployment=str(meta['deployment']), deployment_sha256=DEPLOYMENT_SHA,
        helper_sha256=sha(Path(__file__).read_bytes()), sampler_metadata_sha256=sha((S / 'metadata.json').read_bytes()),
        evidence_prefixes=[sample_ref] + refs, anchors=anchors,
        observed_process_identities=[dict(pid=pid, start_ticks=ticks, **value) for (pid, ticks), value in identities.items()],
        every_sample_unique_pid_start=True, identity_error_count=sum(len(row['identity_errors']) for row in rows),
        wrapper_pid=manifest['owner']['pid'], recorder_pid=ready['pid'],
        task_pipeline_aggregate=aggregate, phase_windows=windows,
        boundary_straddling_intervals=[dict(start_ns=a['recorded_ns'], end_ns=b['recorded_ns'], boundary_ns=boundary)
            for a, b in zip(rows, rows[1:]) for boundary in boundaries
            if a['recorded_ns'] < boundary < b['recorded_ns']],
        clock_inconsistent_intervals=[dict(start_ns=a['recorded_ns'], end_ns=b['recorded_ns'])
            for a, b in zip(rows, rows[1:]) if abs((b['recorded_ns'] - a['recorded_ns']) -
                (b['monotonic_ns'] - a['monotonic_ns'])) > 20_000_000],
        observer_result=final,
        limits=['No ROS/device connections, temperature reads or changes to sampler/control/production.',
            'The selected-target pipeline includes initial preview computation before task state becomes active.',
            'The first task state log is an exact stage marker, not a fabricated task_started timestamp.',
            'Only whole intervals inside each stage and with consistent clocks contribute to means.',
            'CPU100 means one core. Per-process/thread/descendant counters are not summed twice.',
            'Missing transient processes can underestimate tracked totals. Host utilization includes unrelated work.',
            'External disk counters include all writers; logical wchar and storage write_bytes differ.',
            'GUI CPU is not rendered FPS/input latency. Scheduler waiting is not a ROS callback queue.',
            'Request thread CPU excludes worker CPU. Wall-minus-thread CPU does not uniquely identify I/O/receipt waits.',
            'This analysis does not certify physical grasp success or speedup.'],
        ROS_initialized=False, hardware_contacted=False, temperature_read=False, motion_commands_sent=False)
    for name, value in [('phase_event_evidence.json', events), ('request_profiles.json', metrics), ('analysis.json', result)]:
        with (output / name).open('x') as stream:
            json.dump(value, stream, ensure_ascii=False, indent=2, allow_nan=False)
            stream.write('\n')
    print(json.dumps(dict(output=str(output), analysis_sha256=sha((output / 'analysis.json').read_bytes()),
        sampler_sealed=sealed, observed_terminal=terminal is not None,
        sample_count=len(rows), unique_processes=len(identities),
        actual_pipeline_duration_sec=aggregate['measured_duration_sec'],
        equivalent_cores=aggregate.get('tracked_equivalent_cores'),
        host_busy_percent=aggregate.get('host_busy_percent'), host_iowait_percent=aggregate.get('host_iowait_percent')), ensure_ascii=False))


if __name__ == '__main__':
    main()
