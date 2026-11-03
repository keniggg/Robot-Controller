"""Summarize already captured GEN50 /proc data; no live process/device access."""
from pathlib import Path
import collections
import hashlib
import json
import os
import time

W = Path(__file__).resolve().parents[1]
E = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
N = E/'ros_log/full_latest_gen41_restart_20261018T011222Z'
OUT = N/'verification/gen50_actual_task_resource_r1'
TASK_END_NS = 1792326000213945000
NEARFIELD_NS = 1792325911819409000
RECORD = E/'grasp_records/20261018T121732.849491Z_2ed8c857049d4bfebf3cfcc9f496ec7b'


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def read(path):
    return json.loads(Path(path).read_text())


def write(path, value):
    with Path(path).open('x') as stream:
        json.dump(value, stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.write('\n')
        stream.flush()
        os.fsync(stream.fileno())
    assert read(path) == value


def disks(text):
    result = {}
    for line in text.splitlines():
        cols = line.split()
        if len(cols) >= 14:
            result[(int(cols[0]), int(cols[1]))] = dict(name=cols[2],
                read_bytes=int(cols[5])*512, write_bytes=int(cols[9])*512,
                busy_ms=int(cols[12]), weighted_busy_ms=int(cols[13]))
    return result


def main():
    metadata = read(OUT/'metadata.json')
    raw_result = read(OUT/'result.json')
    rows = [json.loads(line) for line in (OUT/'samples.jsonl').read_text().splitlines()]
    assert sha(OUT/'samples.jsonl') == raw_result['samples_sha256']
    targets = metadata['target_process_identities']
    identity_failures = []
    for row in rows:
        observed = {p['name']: p for p in row['processes']}
        for name, expected in targets.items():
            p = observed.get(name)
            if (p is None or p['pid'] != expected['pid']
                    or p['start_ticks'] != int(expected['start_ticks'])
                    or p['ppid'] != expected['parent_pid']):
                identity_failures.append(dict(recorded_ns=row['recorded_ns'], name=name))
        identity_failures.extend(dict(recorded_ns=row['recorded_ns'], **error)
            for error in row['identity_errors'])
    assert not identity_failures, identity_failures
    assert len(targets) == 11
    # Freeze only the two necessary source lines, preserving their raw bytes,
    # offsets and hashes. The complete live consumer log is not re-hashed.
    consumer = N/'gen50_runtime/compute_gateway_deployment_r1/consumers.log'
    wanted = {b'[1792325911.819409]': 'nearfield_wait_started',
              b'[1792326000.213945]': 'task_failed'}
    evidence = []
    offset = 0
    with consumer.open('rb') as stream:
        for number, line in enumerate(stream, 1):
            for needle, role in wanted.items():
                if needle in line:
                    evidence.append(dict(role=role, path=str(consumer), line_number=number,
                        offset=offset, size=len(line), raw_line=line.decode(),
                        sha256=hashlib.sha256(line).hexdigest()))
            offset += len(line)
    assert len(evidence) == 2
    phase = dict(nearfield_wait_started_ns=NEARFIELD_NS, task_failed_ns=TASK_END_NS,
        failure='NEAR_FIELD_DIRECT_TIMEOUT', exact_source_lines=evidence,
        result_summary=str(RECORD/'result_summary.json'),
        result_summary_sha256=sha(RECORD/'result_summary.json'),
        no_CONTACT_descent_or_close_claim='Root/C handoff report, not inferred from /proc')
    write(OUT/'task_phase_evidence.json', phase)
    intervals = list(zip(rows, rows[1:]))
    hz = metadata['clock_ticks_per_second']
    dev = RECORD.stat().st_dev
    device = os.major(dev), os.minor(dev)

    def summarize(label, lo, hi):
        # Only fully contained measured intervals are counted. A terminal
        # straddling interval is excluded instead of mixing post-task idle.
        selected = [(a, b) for a, b in intervals
            if lo <= a['recorded_ns'] < b['recorded_ns'] <= hi]
        duration = sum((b['monotonic_ns']-a['monotonic_ns'])/1e9 for a,b in selected)
        assert duration > 0
        process_data = {}
        host_total = host_busy = host_iowait = 0
        storage = collections.Counter()
        for a, b in selected:
            aa = {(p['pid'], p['start_ticks']): p for p in a['processes']}
            for p in b['processes']:
                previous = aa.get((p['pid'], p['start_ticks']))
                if previous is None:
                    continue
                entry = process_data.setdefault(p['name'], dict(cpu_ticks=0, rss_min_bytes=p['rss_bytes'],
                    rss_peak_bytes=0, thread_min=p['threads'], thread_peak=0, io_delta=collections.Counter()))
                entry['cpu_ticks'] += p['cpu_ticks']-previous['cpu_ticks']
                entry['rss_min_bytes'] = min(entry['rss_min_bytes'], p['rss_bytes'], previous['rss_bytes'])
                entry['rss_peak_bytes'] = max(entry['rss_peak_bytes'], p['rss_bytes'], previous['rss_bytes'])
                entry['thread_min'] = min(entry['thread_min'], p['threads'], previous['threads'])
                entry['thread_peak'] = max(entry['thread_peak'], p['threads'], previous['threads'])
                for key, value in p['io'].items():
                    if key in previous['io']:
                        entry['io_delta'][key] += value-previous['io'][key]
            delta = [y-x for x,y in zip(a['cpu_ticks'][:8],b['cpu_ticks'][:8])]
            host_total += sum(delta)
            host_busy += sum(delta)-delta[3]-delta[4]
            host_iowait += delta[4]
            da, db = disks(a['diskstats']), disks(b['diskstats'])
            if device in da and device in db:
                for key in ('read_bytes','write_bytes','busy_ms','weighted_busy_ms'):
                    storage[key] += db[device][key]-da[device][key]
        for entry in process_data.values():
            entry['cpu_percent_one_core_100'] = 100*entry['cpu_ticks']/hz/duration
            entry['cpu_sec'] = entry['cpu_ticks']/hz
            entry['io_delta'] = dict(entry['io_delta'])
        root_cpu = sum(process_data[name]['cpu_percent_one_core_100'] for name in targets)
        sampled_rows = [r for r in rows if lo <= r['recorded_ns'] <= hi]
        return dict(label=label, nominal_start_ns=lo, nominal_end_ns=hi,
            first_complete_interval_start_ns=selected[0][0]['recorded_ns'],
            last_complete_interval_end_ns=selected[-1][1]['recorded_ns'],
            complete_intervals=len(selected), measured_duration_sec=duration,
            deployed11_total_cpu_percent_one_core_100=root_cpu,
            deployed11_equivalent_cores=root_cpu/100,
            host_cpu_busy_percent=100*host_busy/host_total,
            host_iowait_percent=100*host_iowait/host_total,
            available_memory_min_kib=min(r['memory_kib']['MemAvailable'] for r in sampled_rows),
            filesystem_device=dict(major=device[0], minor=device[1],
                name=disks(selected[0][0]['diskstats'])[device]['name'], **storage),
            processes=process_data)

    start, end = rows[0]['recorded_ns'], rows[-1]['recorded_ns']
    assert NEARFIELD_NS < start < TASK_END_NS < end
    task = summarize('actual_nearfield_task_only', start, TASK_END_NS)
    post = summarize('post_terminal_only_not_task_load', TASK_END_NS, end)
    windows=[]
    for label, lo, hi in [('task_0_to_30s',start,min(start+30_000_000_000,TASK_END_NS)),
        ('task_30s_to_terminal',start+30_000_000_000,TASK_END_NS),
        ('post_terminal_0_to_30s',TASK_END_NS,min(end,TASK_END_NS+30_000_000_000)),
        ('post_terminal_30_to_60s',TASK_END_NS+30_000_000_000,min(end,TASK_END_NS+60_000_000_000)),
        ('post_terminal_60s_to_sample_end',TASK_END_NS+60_000_000_000,end)]:
        if hi-lo > 1_100_000_000:
            windows.append(summarize(label,lo,hi))
    write(OUT/'windows.json',windows)
    result = dict(kind='GEN50_actual_task_resource_analysis', passed=True, created_ns=time.time_ns(),
        deployment=metadata['deployment'], deployment_sha256=metadata['deployment_sha256'],
        record=metadata['record'], sample_count=len(rows),
        all11_pid_start_parent_matched_every_sample=True, identity_errors=[],
        sample_start_ns=start,sample_end_ns=end, task_end_ns=TASK_END_NS,
        sampling_began_sec_after_nearfield_wait=(start-NEARFIELD_NS)/1e9,
        task=task,post_terminal=post,
        observer_cpu_sec=raw_result['observer_cpu_sec'],
        observer_average_cpu_percent_one_core_100=100*raw_result['observer_cpu_sec']/raw_result['elapsed_sec'],
        maximum_collection_sec=raw_result['maximum_collection_sec'],
        excluded_terminal_straddling_intervals=[dict(start_ns=a['recorded_ns'],end_ns=b['recorded_ns'])
            for a,b in intervals if a['recorded_ns']<TASK_END_NS<b['recorded_ns']],
        evidence_sha256={str(p):sha(p) for p in [OUT/'metadata.json',OUT/'samples.jsonl',OUT/'result.json',
            OUT/'task_phase_evidence.json',OUT/'windows.json',Path(__file__)]},
        limitations=[
            'The observation motion, settle, and first 31.39 seconds of nearfield wait predate this sample window.',
            'Post-terminal samples are explicitly separate and do not dilute actual-task means.',
            'CPU 100 percent is one logical core; available affinity contains 12 CPUs. This is unrelated to offline regression speedup.',
            'Only the pinned deployed 11 processes plus verified descendants were sampled. The independent recorder, wrapper, mirror and other host processes may be outside scope.',
            'RSS includes shared pages and must not be summed as unique physical RAM. rchar/wchar are logical IO, read_bytes/write_bytes are kernel-attributed storage IO.',
            'Filesystem device counters include all writers on SanStand, including unrelated tasks, and cannot attribute all disk IO to the recorder.',
            'Only fully contained intervals contribute to means; the interval crossing Task failure is excluded. Proc samples do not identify callback queues, exact algorithm cost, or the cause of timeout.'
        ], ROS_initialized=False,hardware_contacted=False,temperature_read=False,
        configuration_or_nodes_changed=False,tests_or_motion_success_claimed=False)
    write(OUT/'analysis.json',result)
    print(json.dumps(dict(analysis_sha256=sha(OUT/'analysis.json'), task_sec=task['measured_duration_sec'],
        task_cpu_percent=task['deployed11_total_cpu_percent_one_core_100'],post_cpu_percent=post['deployed11_total_cpu_percent_one_core_100'],
        task_host_busy_percent=task['host_cpu_busy_percent'],task_host_iowait_percent=task['host_iowait_percent'],
        task_storage=task['filesystem_device'],top_task=[dict(name=k,cpu=v['cpu_percent_one_core_100'],rss_peak_mib=v['rss_peak_bytes']/2**20)
            for k,v in sorted(task['processes'].items(),key=lambda x:x[1]['cpu_percent_one_core_100'],reverse=True)[:6]]),ensure_ascii=False))


if __name__=='__main__':
    main()
