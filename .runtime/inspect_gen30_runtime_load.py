#!/usr/bin/env python3
"""Read-only host proc sampling. No ROS, process attach, signals or hardware reads."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import time

ENV_KEYS = {'OPENBLAS_NUM_THREADS', 'OMP_NUM_THREADS', 'MKL_NUM_THREADS',
            'OPENCV_FOR_THREADS_NUM', 'NUMEXPR_NUM_THREADS', 'PYTHONNOUSERSITE',
            'PYTHONDONTWRITEBYTECODE', 'PYTHONPATH', 'LD_LIBRARY_PATH'}

def read(path):
    try:
        return Path(path).read_text()
    except (OSError, UnicodeError):
        return None

def status(path):
    allowed = {'Name', 'State', 'Threads', 'Cpus_allowed_list', 'Mems_allowed_list',
               'VmRSS', 'VmSwap', 'voluntary_ctxt_switches', 'nonvoluntary_ctxt_switches'}
    return {k: v.strip() for line in (read(path) or '').splitlines()
            for k, v in [line.split(':', 1)] if k in allowed}

def process(pid, initial=False):
    root = Path('/proc') / str(pid)
    out = {'pid': pid, 'status': status(root / 'status'), 'threads': {}}
    if initial:
        env = (read(root / 'environ') or '').split('\0')
        out['environment_allowlist'] = {s.split('=', 1)[0]: s.split('=', 1)[1]
                                        for s in env if '=' in s and s.split('=', 1)[0] in ENV_KEYS}
        out['cgroup'] = read(root / 'cgroup')
        out['libraries'] = sorted({line.split()[-1] for line in (read(root / 'maps') or '').splitlines()
                                   if any(x in line.lower() for x in ['blas', 'lapack', 'mkl', 'libgomp',
                                                                      'libomp', 'libtbb', 'opencv', 'numpy', 'scipy'])})
    try:
        tasks = list((root / 'task').iterdir())
    except OSError:
        tasks = []
    for task in tasks:
        stat = read(task / 'stat')
        sched = read(task / 'schedstat')
        if not stat or not sched:
            continue
        fields = stat.rsplit(')', 1)[1].split()  # start with original stat field 3
        out['threads'][task.name] = {
            'name': (read(task / 'comm') or '').strip(),
            'state': fields[0], 'minor_faults': int(fields[7]), 'major_faults': int(fields[9]),
            'utime_ticks': int(fields[11]), 'stime_ticks': int(fields[12]),
            'priority': int(fields[15]), 'nice': int(fields[16]),
            'processor': int(fields[36]), 'policy': int(fields[38]),
            'runtime_ns': int(sched.split()[0]), 'runqueue_wait_ns': int(sched.split()[1]),
            'timeslices': int(sched.split()[2]), 'wchan': read(task / 'wchan'),
            'status': status(task / 'status'),
        }
    return out

def sample(pids, initial=False):
    return {'wall_stamp_ns': time.time_ns(), 'monotonic_ns': time.monotonic_ns(),
            'loadavg': read('/proc/loadavg'),
            'psi': {k: read('/proc/pressure/' + k) for k in ('cpu', 'memory', 'io')},
            'cpu_totals': (read('/proc/stat') or '').splitlines()[0],
            'memory': {line.split(':')[0]: line.split(':')[1].strip()
                       for line in (read('/proc/meminfo') or '').splitlines()
                       if line.split(':')[0] in {'MemTotal', 'MemAvailable', 'SwapTotal', 'SwapFree', 'Dirty', 'Writeback'}},
            'processes': [process(p, initial) for p in pids]}

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--out', required=True)
    parser.add_argument('--seconds', type=int, default=12)
    parser.add_argument('--pids', type=int, nargs='+', default=[226839, 226840, 226841])
    args = parser.parse_args()
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=False)
    samples = [sample(args.pids, True)]
    for _ in range(args.seconds):
        time.sleep(1)
        samples.append(sample(args.pids))
    first, last = samples[0], samples[-1]
    elapsed = (last['monotonic_ns'] - first['monotonic_ns']) / 1e9
    summaries = []
    for a, b in zip(first['processes'], last['processes']):
        rows = []
        for tid, finish in b['threads'].items():
            start = a['threads'].get(tid)
            if start is None:
                continue
            row = {'tid': int(tid), 'name': finish['name'],
                   'runtime_sec': (finish['runtime_ns'] - start['runtime_ns']) / 1e9,
                   'runqueue_wait_sec': (finish['runqueue_wait_ns'] - start['runqueue_wait_ns']) / 1e9,
                   'minor_faults': finish['minor_faults'] - start['minor_faults'],
                   'major_faults': finish['major_faults'] - start['major_faults'],
                   'cpu_seconds_per_wall_second': (finish['runtime_ns'] - start['runtime_ns']) / 1e9 / elapsed,
                   'cpu_affinity': finish['status'].get('Cpus_allowed_list'),
                   'policy': finish['policy'], 'nice': finish['nice']}
            rows.append(row)
        summaries.append({'pid': a['pid'], 'threads_first': len(a['threads']), 'threads_last': len(b['threads']),
                          'runtime_sum_sec': sum(r['runtime_sec'] for r in rows),
                          'runqueue_wait_sum_sec': sum(r['runqueue_wait_sec'] for r in rows),
                          'major_faults_sum': sum(r['major_faults'] for r in rows),
                          'minor_faults_sum': sum(r['minor_faults'] for r in rows),
                          'threads': sorted(rows, key=lambda r: -r['runtime_sec'])})
    limits = {p: read(p) for p in ['/sys/fs/cgroup/cpu/cpu.cfs_quota_us',
                                  '/sys/fs/cgroup/cpu/cpu.cfs_period_us',
                                  '/sys/fs/cgroup/cpu/cpu.stat',
                                  '/sys/fs/cgroup/cpuset/cpuset.cpus']}
    result = {'scope': 'Post-failure passive host sampling, not failure-window causality; no ROS or hardware access.',
              'elapsed_sec': elapsed, 'processes': summaries, 'cpu_cgroup_limits': limits,
              'first_loadavg': first['loadavg'], 'last_loadavg': last['loadavg'],
              'first_psi': first['psi'], 'last_psi': last['psi'],
              'script_sha256': hashlib.sha256(Path(__file__).read_bytes()).hexdigest()}
    for name, value in [('samples.json', samples), ('summary.json', result)]:
        (out / name).write_text(json.dumps(value, indent=2) + '\n')
    print(json.dumps({'output': str(out), 'elapsed_sec': elapsed,
                      'processes': [{k: v for k, v in p.items() if k != 'threads'} for p in summaries]}))

if __name__ == '__main__':
    main()
