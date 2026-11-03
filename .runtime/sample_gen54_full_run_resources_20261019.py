#!/usr/bin/env python3
"""Read /proc for one 40-second actual full-run window; no ROS or runtime writes."""
from collections import Counter
import hashlib
import json
import os
from pathlib import Path
import resource
import statistics
import time

W = Path('/home/zhuyupei/alicia_wa_full/.worktrees/protocol-v3-upgrade')
V = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_gen41_restart_20261018T011222Z/verification')
RUN = V / 'gen54_full_parallel_r1'
OUT = V / 'gen54_full_runtime_resource_sample_r1'
RUNNER = W / '.runtime/verify_gen54_parallel_offline_20261018.py'
HZ = os.sysconf('SC_CLK_TCK')
PAGE = os.sysconf('SC_PAGE_SIZE')


def sha(p):
    return hashlib.sha256(Path(p).read_bytes()).hexdigest()


def processes():
    result = {}
    for p in Path('/proc').iterdir():
        if not p.name.isdigit(): continue
        try:
            f = (p / 'stat').read_text().rsplit(')', 1)[1].split()
            result[int(p.name)] = dict(pid=int(p.name), state=f[0], ppid=int(f[1]),
                ticks=int(f[11])+int(f[12]), start_ticks=int(f[19]), rss_bytes=int(f[21])*PAGE,
                threads=int(f[17]))
        except (OSError, ValueError, IndexError): pass
    return result


def pressure():
    out = {}
    for name in ('cpu', 'memory', 'io'):
        p = Path('/proc/pressure') / name
        if p.exists():
            out[name] = {line.split()[0]: {k: float(v) for k, v in
                (item.split('=') for item in line.split()[1:])} for line in p.read_text().splitlines()}
    return out


def capture(roots):
    started = time.monotonic()
    table = processes()
    owner = {pid: index for index, row in roots.items() for pid in [row['pid']]
             if pid in table and table[pid]['start_ticks'] == row['start_ticks']}
    for _ in range(len(table)):
        added = {pid: owner[row['ppid']] for pid, row in table.items()
                 if pid not in owner and row['ppid'] in owner}
        if not added: break
        owner.update(added)
    selected = {}
    for pid, index in owner.items():
        row = dict(table[pid], worker_index=index,
                   role='worker_coordinator' if pid == roots[index]['pid'] else 'pytest_subtree')
        try:
            row['io'] = {k: int(v) for k, v in (x.split(':', 1) for x in (Path('/proc') / str(pid) / 'io').read_text().splitlines())}
        except OSError: row['io'] = None
        try:
            s = (Path('/proc') / str(pid) / 'schedstat').read_text().split()
            row['leader_schedstat_ns'] = dict(runtime=int(s[0]), runqueue_wait=int(s[1]))
        except OSError: row['leader_schedstat_ns'] = None
        selected[str(pid)] = row
    stat = Path('/proc/stat').read_text().splitlines()
    total = [int(x) for x in stat[0].split()[1:9]]  # exclude guest double counts
    misc = {x.split()[0]: int(x.split()[1]) for x in stat if x.startswith(('procs_running ', 'procs_blocked '))}
    mem = {k: int(v.split()[0])*1024 for k, v in (x.split(':', 1) for x in Path('/proc/meminfo').read_text().splitlines())
           if k in ('MemTotal', 'MemAvailable', 'SwapTotal', 'SwapFree', 'Dirty', 'Writeback')}
    return dict(wall_time=time.time(), monotonic=time.monotonic(), cpu_ticks=total,
        memory_bytes=mem, load_average=list(os.getloadavg()), process_queue=misc,
        pressure=pressure(), processes=selected, capture_wall_sec=time.monotonic()-started)


def stats(values):
    if not values: return None
    ordered = sorted(values)
    return dict(mean=statistics.mean(values), min=min(values), max=max(values),
                p50=statistics.median(values), p95=ordered[min(len(ordered)-1, int(.95*len(ordered)))])


def main():
    assert Path('/media/zhuyupei/SanStand').is_mount() and not OUT.exists()
    pinned = {str(RUN / 'plan.json'): sha(RUN / 'plan.json'), str(RUNNER): sha(RUNNER)}
    plan = json.loads((RUN / 'plan.json').read_text())
    assert plan['mode'] == 'full' and plan['worker_count'] == 7 and plan['expected_case_count'] == 9255
    roots = {}
    for pid, row in processes().items():
        try: args = [x.decode() for x in (Path('/proc') / str(pid) / 'cmdline').read_bytes().split(b'\0') if x]
        except (OSError, UnicodeError): continue
        if '--worker-plan' not in args or '--worker-index' not in args: continue
        if args[args.index('--worker-plan')+1] != str(RUN / 'plan.json'): continue
        index = int(args[args.index('--worker-index')+1])
        assert index not in roots and 0 <= index < 7
        roots[index] = dict(pid=pid, parent_pid=row['ppid'], start_ticks=row['start_ticks'])
    assert roots, 'full-run workers already ended; do not sample idle as task load'
    OUT.mkdir()
    samples = []
    own = resource.getrusage(resource.RUSAGE_SELF)
    begin = time.monotonic()
    with (OUT / 'samples.jsonl').open('x') as f:
        for i in range(41):
            if i:
                time.sleep(max(0., begin+i-time.monotonic()))
            row = capture(roots); samples.append(row)
            f.write(json.dumps(row, sort_keys=True)+'\n'); f.flush()
    duration = samples[-1]['monotonic']-samples[0]['monotonic']
    host_active, io_wait, tree_cores, active_workers = [], [], [], []
    worker_cpu = Counter(); worker_io = {i: Counter() for i in range(7)}
    disappeared, born, changed_identity = [], [], []
    leader_wait = Counter()
    for a, b in zip(samples, samples[1:]):
        dt = b['monotonic']-a['monotonic']
        delta = [y-x for x,y in zip(a['cpu_ticks'], b['cpu_ticks'])]; total = sum(delta)
        host_active.append(100.*(total-delta[3]-delta[4])/total)
        io_wait.append(100.*delta[4]/total)
        cpu = 0.
        for pid, old in a['processes'].items():
            new = b['processes'].get(pid)
            if new is None:
                disappeared.append(dict(pid=int(pid), last_seen_wall=a['wall_time'], worker_index=old['worker_index'])); continue
            if new['start_ticks'] != old['start_ticks']:
                changed_identity.append(int(pid)); continue
            used = (new['ticks']-old['ticks'])/HZ
            assert used >= 0.
            worker_cpu[(new['worker_index'], new['role'])] += used
            cpu += used
            if new['io'] is not None and old['io'] is not None:
                for key in ('read_bytes', 'write_bytes', 'rchar', 'wchar'):
                    worker_io[new['worker_index']][key] += max(0, new['io'][key]-old['io'][key])
            if new['leader_schedstat_ns'] is not None and old['leader_schedstat_ns'] is not None:
                leader_wait[new['worker_index']] += max(0, new['leader_schedstat_ns']['runqueue_wait']-old['leader_schedstat_ns']['runqueue_wait'])/1e9
        born.extend(dict(pid=int(pid), first_seen_wall=b['wall_time'], worker_index=row['worker_index'])
                    for pid,row in b['processes'].items() if pid not in a['processes'])
        tree_cores.append(cpu/dt)
        active_workers.append(len({r['worker_index'] for r in b['processes'].values() if r['role']=='pytest_subtree' and r['state'] != 'Z'}))
    rows = []
    for i in range(7):
        rss = [sum(r['rss_bytes'] for r in s['processes'].values() if r['worker_index']==i) for s in samples]
        pytest_cpu = worker_cpu[(i, 'pytest_subtree')]
        coord_cpu = worker_cpu[(i, 'worker_coordinator')]
        rows.append(dict(worker_index=i, frozen_worker_identity=roots.get(i),
            observed_cpu_sec=pytest_cpu+coord_cpu, pytest_subtree_cpu_sec=pytest_cpu,
            coordinator_cpu_sec=coord_cpu, average_cores=(pytest_cpu+coord_cpu)/duration,
            RSS_sum_peak_bytes=max(rss), io_delta_bytes=dict(worker_io[i]),
            leader_threads_runqueue_wait_sec=leader_wait[i],
            sample_count_with_pytest=sum(any(r['worker_index']==i and r['role']=='pytest_subtree' for r in s['processes'].values()) for s in samples)))
    final_workers = []
    for i in range(7):
        p = RUN / ('worker_%d' % i) / 'result.json'
        if p.exists():
            r = json.loads(p.read_text())
            final_workers.append(dict(worker_index=i, result_path=str(p), result_sha256=sha(p),
                passed=r.get('passed'), elapsed_sec=r.get('elapsed_sec'),
                summary=r.get('summary'), resource_usage={k:v for k,v in r.items() if any(x in k for x in ('rusage','cpu_user','cpu_system','rss','resource'))}))
    after = resource.getrusage(resource.RUSAGE_SELF)
    result = dict(kind='GEN54_full_actual_40_second_resource_window',
        scope='one_actual_mid_run_window_not_full_regression_average',
        wall_start=samples[0]['wall_time'], wall_end=samples[-1]['wall_time'], duration_sec=duration,
        sample_count=len(samples), sample_period_sec=1., expected_workers=7, roots_found_at_start=len(roots),
        CPU_count=os.cpu_count(), affinity_cpus=sorted(os.sched_getaffinity(0)),
        host_active_percent=stats(host_active), host_iowait_percent=stats(io_wait),
        worker_tree_cores=stats(tree_cores), active_pytest_workers=stats(active_workers), workers=rows,
        memory_available_min_bytes=min(s['memory_bytes']['MemAvailable'] for s in samples),
        memory_available_max_bytes=max(s['memory_bytes']['MemAvailable'] for s in samples),
        swap_used_max_bytes=max(s['memory_bytes']['SwapTotal']-s['memory_bytes']['SwapFree'] for s in samples),
        worker_RSS_sum_peak_bytes=max(sum(r['rss_bytes'] for r in s['processes'].values()) for s in samples),
        process_queue_running=stats([s['process_queue']['procs_running'] for s in samples]),
        process_queue_blocked=stats([s['process_queue']['procs_blocked'] for s in samples]),
        disappeared_process_intervals=disappeared, newly_seen_processes=born,
        PID_start_identity_mismatches=changed_identity,
        CPU_IO_delta_scope='identity-matched consecutive samples; process exit tails or newly born intervals may be omitted and are listed',
        schedstat_scope='sum of observed process leader schedstat waits, not all threads or application queue latency',
        RSS_scope='sum of process resident sets, may double-count shared pages',
        PSI_first=samples[0]['pressure'], PSI_last=samples[-1]['pressure'],
        completed_worker_results_available_at_end=final_workers,
        observer_CPU_sec=(after.ru_utime+after.ru_stime)-(own.ru_utime+own.ru_stime),
        observer_capture_wall_sec_sum=sum(s['capture_wall_sec'] for s in samples),
        samples_sha256=sha(OUT / 'samples.jsonl'), input_sha256=pinned,
        helper_sha256=sha(__file__), frozen_inputs_unchanged=all(sha(p)==h for p,h in pinned.items()),
        production_or_frozen_scripts_modified=False, ROS_or_hardware_accessed=False,
        temperature_queried_or_values_diagnosed=False)
    with (OUT / 'result.json').open('x') as f:
        json.dump(result,f,ensure_ascii=False,indent=2); f.write('\n')
    print(json.dumps(dict(result=str(OUT / 'result.json'), result_sha256=sha(OUT / 'result.json'),
        duration_sec=duration, host_active_percent=result['host_active_percent'],
        host_iowait_percent=result['host_iowait_percent'], worker_cores=result['worker_tree_cores'],
        available_GiB=result['memory_available_min_bytes']/2**30, exited=len(disappeared),
        observer_CPU_sec=result['observer_CPU_sec']), ensure_ascii=False))


if __name__ == '__main__':
    main()
