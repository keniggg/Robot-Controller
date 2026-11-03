"""Bounded file-only resource analysis; no ROS/device/network access."""
import argparse
import hashlib
import json
import os
from pathlib import Path

M = Path('/media/zhuyupei/SanStand')
S = M / '实验记录/Robot-Controller/ros_log/full_latest_gen41_restart_20261018T011222Z/gen58_resume_r1'
R = S / 'actual_task_resources_r1'
RECORD = M / '实验记录/Robot-Controller/grasp_records/20261022T032906.414053Z_6a8a761e427d48ff9e4c14741837f7a3'


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def load(path):
    return json.loads(Path(path).read_bytes())


def summarize(rows, start_ns, end_ns, hz, accepted):
    # Only adjacent intervals wholly inside the requested window contribute.
    selected = [r for r in rows if start_ns <= r['recorded_ns'] <= end_ns]
    if len(selected) < 2:
        return {'sample_count': len(selected), 'interval_count': 0,
                'limitation': 'No complete sample interval; CPU/IO rate not estimated.'}
    elapsed = (selected[-1]['monotonic_ns'] - selected[0]['monotonic_ns']) / 1e9
    accum = {}
    host_delta = [0] * 8
    peak_total_cpu = 0.
    for before, after in zip(selected, selected[1:]):
        dt = (after['monotonic_ns'] - before['monotonic_ns']) / 1e9
        prior = {(p['pid'], p['start_ticks']): p for p in before['processes']}
        interval_cpu = 0.
        for p in after['processes']:
            key = (p['pid'], p['start_ticks'])
            old = prior.get(key)
            if old is None:
                continue
            row = accum.setdefault(key, dict(pid=key[0], start_ticks=key[1], name=p['name'],
                attribution=p['attribution'], accepted_node=key in accepted, cpu_sec=0.,
                matched_seconds=0., interval_count=0, peak_interval_cores=0., io_delta={}))
            cpu = (p['cpu_ticks'] - old['cpu_ticks']) / hz
            assert cpu >= 0 and dt > 0
            row['cpu_sec'] += cpu
            row['matched_seconds'] += dt
            row['interval_count'] += 1
            row['peak_interval_cores'] = max(row['peak_interval_cores'], cpu / dt)
            interval_cpu += cpu
            for k, value in p['io'].items():
                if k in old['io']:
                    row['io_delta'][k] = row['io_delta'].get(k, 0) + value - old['io'][k]
        peak_total_cpu = max(peak_total_cpu, interval_cpu / dt)
        host_delta = [v + b - a for v, a, b in zip(host_delta, before['cpu_ticks'][:8], after['cpu_ticks'][:8])]
    for key, row in accum.items():
        series = [p for r in selected for p in r['processes'] if (p['pid'], p['start_ticks']) == key]
        row['average_cores_over_observed_window'] = row['cpu_sec'] / elapsed
        row['peak_rss_bytes'] = max(p['rss_bytes'] for p in series)
        row['min_rss_bytes'] = min(p['rss_bytes'] for p in series)
        row['max_threads'] = max(p['threads'] for p in series)
        row['max_vm_swap_kib'] = max(p.get('status', {}).get('VmSwap', 0) for p in series)
    total = sum(host_delta)
    host_busy = sum(host_delta) - host_delta[3] - host_delta[4]
    io0 = selected[0]['external_filesystem_device']['counters']
    io1 = selected[-1]['external_filesystem_device']['counters']
    return dict(sample_count=len(selected), interval_count=len(selected)-1,
        requested_window_ns=[start_ns, end_ns], measured_window_ns=[selected[0]['recorded_ns'], selected[-1]['recorded_ns']],
        measured_elapsed_sec=elapsed, omitted_boundary_sec=[(selected[0]['recorded_ns']-start_ns)/1e9, (end_ns-selected[-1]['recorded_ns'])/1e9],
        total_matched_cpu_sec=sum(p['cpu_sec'] for p in accum.values()),
        accepted_node_cpu_sec=sum(p['cpu_sec'] for p in accum.values() if p['accepted_node']),
        recording_tree_cpu_sec=sum(p['cpu_sec'] for p in accum.values() if not p['accepted_node']),
        average_tracked_cores=sum(p['cpu_sec'] for p in accum.values())/elapsed,
        peak_interval_tracked_cores=peak_total_cpu,
        host=dict(busy_percent=100*host_busy/total, iowait_percent=100*host_delta[4]/total,
            average_busy_core_equivalent=host_busy/hz/elapsed,
            min_loadavg_1m=min(float(r['loadavg'].split()[0]) for r in selected),
            max_loadavg_1m=max(float(r['loadavg'].split()[0]) for r in selected),
            min_mem_available_kib=min(r['memory_kib']['MemAvailable'] for r in selected),
            min_swap_free_kib=min(r['memory_kib']['SwapFree'] for r in selected),
            max_swap_free_kib=max(r['memory_kib']['SwapFree'] for r in selected),
            max_dirty_kib=max(r['memory_kib']['Dirty'] for r in selected),
            peak_sum_tracked_rss_bytes=max(sum(p['rss_bytes'] for p in r['processes']) for r in selected),
            external_device_delta={k:io1[k]-v for k,v in io0.items() if isinstance(v,int)}),
        processes=sorted(accum.values(),key=lambda p:-p['cpu_sec']))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--start-ns', type=int, default=1792639765600000000)
    parser.add_argument('--events', type=Path)
    parser.add_argument('--output-name', default='bounded_task_analysis_r2.json')
    args = parser.parse_args()
    meta, result = load(R/'metadata.json'), load(R/'result.json')
    assert sha(R/'samples.jsonl') == result['samples_sha256']
    assert sha(R/'metadata.json') == result['metadata_sha256']
    assert sha(meta['deployment']) == meta['deployment_sha256']
    assert sha(meta['helper_path']) == meta['helper_sha256']
    assert sha(meta['primitives_path']) == meta['primitives_sha256']
    rows = [json.loads(s) for s in (R/'samples.jsonl').read_text().splitlines()]
    assert len(rows) == result['sample_count'] == 217
    accepted = {(p['pid'], p['start_ticks']) for p in meta['target_process_identities'].values()}
    assert len(accepted) == 11 and all(accepted <= {(p['pid'], p['start_ticks']) for p in r['processes']} for r in rows)
    terminal = load(RECORD/'recorder_outcome.json')['task_result']['evidence']['message']['stamp_ns']
    timeline = load(args.events) if args.events else None
    if timeline is not None:
        assert timeline['task_active_start_header_ns'] == args.start_ns
        assert timeline['task_active_end_header_ns'] == terminal
    auditpath = S/'actual_attempt_r1_geometry/terminal_gate_audit.json'
    audit = load(auditpath)
    contact = audit['snapshot_stamp_ns']
    windows = {'pre_task': (rows[0]['recorded_ns'], args.start_ns),
        'actual_task': (args.start_ns, terminal),
        'task_before_contact_snapshot': (args.start_ns, contact),
        'contact_snapshot_to_terminal': (contact, terminal),
        'post_terminal_tail': (terminal, rows[-1]['recorded_ns'])}
    if timeline is not None:
        windows.update(task_to_arrival=(args.start_ns, timeline['arrival_ns']),
            arrival_to_qualified=(timeline['arrival_ns'], timeline['qualified_request_header_ns']),
            qualified_to_contact_snapshot=(timeline['qualified_request_header_ns'], contact))
    phases = {k:summarize(rows,*bounds,meta['clock_ticks_per_second'],accepted) for k,bounds in windows.items()}
    errors = [dict(sample_index=i, recorded_ns=r['recorded_ns'], **e) for i,r in enumerate(rows) for e in r['identity_errors']]
    inputs = [R/'metadata.json',R/'result.json',R/'samples.jsonl',Path(meta['deployment']),Path(meta['helper_path']),Path(meta['primitives_path']),RECORD/'recorder_outcome.json',auditpath]
    if args.events:
        inputs.append(args.events)
    scopes = audit['pipeline_metrics']['request_preparation_profile']['scopes']
    base = next(s for s in scopes if s['scope']=='base_prepare')
    geometry = next(s for s in base['components'] if s['name']=='geometry_registration')
    predict = next(s for s in base['components'] if s['name']=='predict')
    report = dict(kind='GEN58_actual_task_bounded_resource_analysis',analysis_helper_sha256=sha(__file__),
        inputs_sha256={str(p):sha(p) for p in inputs},sample_count=len(rows),
        boundaries=dict(task_start_ns=args.start_ns,task_start_precision='exact exported event' if args.events else 'parent supplied approximate 0.1 second; inner sample interval unaffected',
            task_terminal_ns=terminal,terminal_source='recorder_outcome.task_result.evidence.message.stamp_ns',
            contact_snapshot_ns=contact,contact_source='terminal_gate_audit.snapshot_stamp_ns'),
        exported_timeline=timeline,
        method='Adjacent same PID/start counter differences wholly within each phase; no interpolation or attribution of boundary-crossing intervals.',
        accepted_node_identities_all_217_samples_present=True,identity_errors=errors,
        identity_error_assessment='Only recording wrapper PID 140929/start 9579234 disappeared during sample 65 at 1792639809.041023257, 4.333 seconds after terminal. A /proc exit/read race is consistent with FileNotFoundError; accepted runtime nodes did not change. No dropped final wrapper counters invented.',
        phases=phases,
        final_request_profile=dict(request_id=audit['request_id'], snapshot_stamp_ns=contact,
            geometry_registration_wall_sec=geometry['wall_sec'],geometry_registration_thread_cpu_sec=geometry['thread_cpu_sec'],
            predict_wall_sec=predict['wall_sec'],predict_local_thread_cpu_sec=predict['thread_cpu_sec'],
            pipeline_elapsed_sec=audit['pipeline_metrics']['end_to_end_ms']/1000,
            geometry_fraction_of_pipeline=geometry['wall_sec']/(audit['pipeline_metrics']['end_to_end_ms']/1000),
            interpretation='Geometry registration is the largest measured component of this final request. This is not the timing of the whole task; wall minus thread CPU does not establish I/O wait.'),
        observer=dict(total_elapsed_sec=result['elapsed_sec'],total_cpu_sec=result['observer_cpu_sec'],maximum_collection_sec=result['maximum_collection_sec']),
        limitations=meta['limitations']+['No new tests, ROS, motion, temperature or network calls were performed.','No speedup claimed. Host usage includes unrelated processes; process RSS may double-count shared pages.','Proc samples do not measure camera arrival delay, callback queue depth, GUI frame latency, remote host utilization, or motion receipt latency.','Intervals shorter than sampling cadence cannot be resolved; interval peaks are sampled averages, not instantaneous peaks.'],
        speedup_claimed=False,physical_success_claimed=False)
    assert M.is_mount() and R.stat().st_dev==M.stat().st_dev
    assert args.output_name in ('bounded_task_analysis_r1.json','bounded_task_analysis_r2.json')
    target=R/args.output_name
    raw=(json.dumps(report,ensure_ascii=False,indent=2,allow_nan=False)+'\n').encode()
    with target.open('xb') as f:
        f.write(raw);f.flush();os.fsync(f.fileno())
    assert target.read_bytes()==raw
    print(json.dumps({'path':str(target),'sha256':sha(target),'actual_task':{k:v for k,v in phases['actual_task'].items() if k!='processes'}},ensure_ascii=False))


if __name__=='__main__':
    main()
