"""Bounded, passive /proc observations; no ROS imports or device access."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import time

M = Path('/media/zhuyupei/SanStand')
E = M / '实验记录/Robot-Controller'
V = E / 'ros_log/full_latest_gen41_restart_20261018T011222Z/verification'
HZ = os.sysconf('SC_CLK_TCK')
PAGE = os.sysconf('SC_PAGE_SIZE')
EXTRA_EXECUTABLES = {'move_group', 'rosmaster', 'rosout', 'record', 'roslaunch'}
EXTRA_SCRIPTS = {'run_candidate.py', 'mirror_active_ros_logs.py', 'record_grasp_keyframes_readonly.py'}


def read(path):
    try:
        return Path(path).read_text().strip()
    except (FileNotFoundError, PermissionError, ProcessLookupError):
        return None


def numeric_lines(path):
    text = read(path)
    result = {}
    if text is not None:
        for line in text.splitlines():
            key, _, value = line.partition(':')
            try:
                result[key] = int(value.split()[0])
            except (ValueError, IndexError):
                pass
    return result


def identity(pid):
    """Inspect argv for identity only; never retain arguments or environment."""
    p = Path('/proc') / str(pid)
    argv = [v.decode(errors='replace') for v in (p / 'cmdline').read_bytes().split(b'\0') if v]
    if not argv:
        return None
    name = next((v.partition(':=')[2] for v in argv
                 if v.startswith('__name:=') and re.fullmatch(r'[A-Za-z0-9_/]+', v.partition(':=')[2])), None)
    executable = Path(argv[0]).name
    script = next((Path(v).name for v in argv[1:]
                   if v.startswith('/home/zhuyupei/alicia_wa_full/') and v.endswith('.py')), None)
    if name:
        return name
    if script in EXTRA_SCRIPTS:
        return script
    if executable in EXTRA_EXECUTABLES and argv[0].startswith(('/opt/ros/', '/usr/bin/')):
        return executable
    return None


def stat(path):
    raw = Path(path).read_text().rsplit(')', 1)[1].split()
    return dict(state=raw[0], ppid=int(raw[1]), start_ticks=int(raw[19]),
                cpu_ticks=int(raw[11]) + int(raw[12]), rss_bytes=int(raw[21]) * PAGE,
                threads=int(raw[17]), processor=int(raw[36]), priority=int(raw[15]),
                nice=int(raw[16]), rt_priority=int(raw[37]), policy=int(raw[38]),
                block_io_delay_ticks=int(raw[39]) if len(raw) > 39 else None)


def cgroup_limits(pid):
    text = read(Path('/proc') / str(pid) / 'cgroup') or ''
    rows = {}
    for line in text.splitlines():
        _, controllers, relative = line.split(':', 2)
        if '..' in Path(relative).parts:
            continue
        for controller in controllers.split(','):
            if controller in ('cpu', 'cpuacct'):
                directory = Path('/sys/fs/cgroup/cpu') / relative.lstrip('/')
                keys = ('cpu.cfs_quota_us', 'cpu.cfs_period_us', 'cpu.stat')
            elif controller == 'cpuset':
                directory = Path('/sys/fs/cgroup/cpuset') / relative.lstrip('/')
                keys = ('cpuset.cpus', 'cpuset.effective_cpus')
            elif controller == 'memory':
                directory = Path('/sys/fs/cgroup/memory') / relative.lstrip('/')
                keys = ('memory.limit_in_bytes', 'memory.usage_in_bytes')
            elif not controller:
                directory = Path('/sys/fs/cgroup') / relative.lstrip('/')
                keys = ('cpu.max', 'cpu.stat', 'cpuset.cpus.effective', 'memory.max', 'memory.current')
            else:
                continue
            rows.update({str(directory / key): value for key in keys
                         if (value := read(directory / key)) is not None})
    return rows


def process(pid, name, threads=False, limits=False):
    p = Path('/proc') / str(pid)
    row = dict(pid=pid, name=name, **stat(p / 'stat'))
    row['affinity_cpus'] = sorted(os.sched_getaffinity(pid))
    row['io'] = numeric_lines(p / 'io')
    status = numeric_lines(p / 'status')
    row['status'] = {k: status[k] for k in ('VmRSS', 'VmHWM', 'VmSwap', 'Threads',
                    'voluntary_ctxt_switches', 'nonvoluntary_ctxt_switches') if k in status}
    row['schedstat'] = read(p / 'schedstat')
    row['wchan'] = read(p / 'wchan')
    if limits:
        row['cgroup_limits'] = cgroup_limits(pid)
    if threads:
        rows = []
        for t in (p / 'task').iterdir():
            try:
                item = stat(t / 'stat')
                rows.append(dict(tid=int(t.name), state=item['state'],
                    cpu_ticks=item['cpu_ticks'], start_ticks=item['start_ticks'],
                    schedstat=read(t / 'schedstat'), wchan=read(t / 'wchan')))
            except (FileNotFoundError, PermissionError, ProcessLookupError, ValueError, IndexError):
                pass
        row['thread_samples'] = rows
    return row


def snapshot(index):
    began = time.monotonic()
    rows = []
    for p in Path('/proc').iterdir():
        if not p.name.isdigit() or int(p.name) == os.getpid():
            continue
        try:
            name = identity(int(p.name))
            if name is not None:
                rows.append(process(int(p.name), name, threads=index % 5 == 0,
                                    limits=index % 30 == 0))
        except (FileNotFoundError, PermissionError, ProcessLookupError, ValueError, IndexError):
            pass
    cpu = [int(v) for v in Path('/proc/stat').read_text().splitlines()[0].split()[1:]]
    memory = numeric_lines('/proc/meminfo')
    return dict(recorded_ns=time.time_ns(), monotonic_ns=time.monotonic_ns(),
        cpu_ticks=cpu, loadavg=read('/proc/loadavg'),
        memory_kib={k: memory[k] for k in ('MemTotal', 'MemAvailable', 'SwapTotal', 'SwapFree',
                    'Dirty', 'Writeback') if k in memory},
        pressure={kind: read('/proc/pressure/' + kind) for kind in ('cpu', 'memory', 'io')},
        processes=sorted(rows, key=lambda r: r['pid']), collection_sec=time.monotonic() - began)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--context', required=True,
                        help='Explicit observation window; no inferred task or idle state.')
    parser.add_argument('--interval-sec', type=float, default=1.)
    parser.add_argument('--duration-sec', type=float, default=900.)
    args = parser.parse_args()
    assert .5 <= args.interval_sec <= 10. and 1. <= args.duration_sec <= 1800.
    assert 1 <= len(args.context) <= 200
    output = args.output.resolve()
    assert M.is_mount() and not os.statvfs(M).f_flag & os.ST_RDONLY
    relative = output.relative_to(V.resolve())
    assert relative.parts, 'Use a child of the external verification directory.'
    assert not output.exists(), 'Use a fresh observation directory.'
    output.mkdir(parents=True)
    assert output.stat().st_dev == M.stat().st_dev
    helper_sha = hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
    meta = dict(kind='passive_runtime_proc_sampling', context=args.context,
        created_ns=time.time_ns(), helper_path=str(Path(__file__).resolve()), helper_sha256=helper_sha,
        clock_ticks_per_second=HZ, page_size=PAGE, os_cpu_count=os.cpu_count(),
        observer_affinity_cpus=sorted(os.sched_getaffinity(0)),
        cpu_online=read('/sys/devices/system/cpu/online'),
        interval_sec=args.interval_sec, duration_limit_sec=args.duration_sec,
        thread_sampling_every=5, cgroup_sampling_every=30, ROS_initialized=False,
        hardware_contacted=False, environment_read=False, complete_argv_stored=False,
        temperature_read=False, control_or_configuration_changed=False,
        limitations='Proc states show resource use and wait channels, not ROS queue occupancy, callback entry, task phase or root cause. Correlate with sealed source-stamped task logs; idle samples cannot establish task bottlenecks.')
    (output / 'metadata.json').write_text(json.dumps(meta, ensure_ascii=False, indent=2) + '\n')
    began, cpu_began = time.monotonic(), time.process_time()
    count = 0
    maximum_collection = 0.
    previous = None
    try:
        with (output / 'samples.jsonl').open('x', buffering=1) as stream:
            while True:
                row = snapshot(count)
                maximum_collection = max(maximum_collection, row['collection_sec'])
                if previous is not None:
                    dt = (row['monotonic_ns'] - previous['monotonic_ns']) / 1e9
                    old = {(r['pid'], r['start_ticks']): r for r in previous['processes']}
                    for item in row['processes']:
                        prior = old.get((item['pid'], item['start_ticks']))
                        if prior is not None:
                            item['interval_cpu_percent'] = 100. * (item['cpu_ticks'] - prior['cpu_ticks']) / HZ / dt
                            item['interval_io_delta'] = {k: v - prior['io'][k] for k, v in item['io'].items() if k in prior['io']}
                stream.write(json.dumps(row, ensure_ascii=False, separators=(',', ':')) + '\n')
                count += 1
                previous = row
                elapsed = time.monotonic() - began
                if elapsed >= args.duration_sec or (output / 'STOP').exists():
                    break
                time.sleep(max(0., min(args.interval_sec, args.duration_sec - elapsed)))
    except KeyboardInterrupt:
        pass
    result = dict(passed=True, kind=meta['kind'], context=args.context,
        observation_only=True, sample_count=count, elapsed_sec=time.monotonic() - began,
        observer_cpu_sec=time.process_time() - cpu_began, maximum_collection_sec=maximum_collection,
        helper_sha256=helper_sha, metadata_sha256=hashlib.sha256((output / 'metadata.json').read_bytes()).hexdigest(),
        samples_sha256=hashlib.sha256((output / 'samples.jsonl').read_bytes()).hexdigest(),
        tests_or_motion_success_claimed=False)
    (output / 'result.json').write_text(json.dumps(result, ensure_ascii=False, indent=2) + '\n')
    print(json.dumps(dict(output=str(output), **result), ensure_ascii=False))


if __name__ == '__main__':
    main()
