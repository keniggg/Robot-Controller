"""Read-only process/CPU/IO samples bound to one GEN45 external full run."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import time

W = Path(__file__).resolve().parents[1]
M = Path('/media/zhuyupei/SanStand')
E = M/'实验记录/Robot-Controller'
V = E/'ros_log/full_latest_gen41_restart_20261018T011222Z/verification'
RUN = V/'gen45_full_parallel_r1'
OUT = V/'gen45_full_parallel_independent_monitor_r1'
RUNNER = W/'.runtime/verify_gen45_parallel_offline_20261018.py'
HZ = os.sysconf('SC_CLK_TCK')
PAGE = os.sysconf('SC_PAGE_SIZE')


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def locations(run, output):
    run, output = Path(run).resolve(), Path(output).resolve()
    assert M.is_mount() and not os.statvfs(M).f_flag & os.ST_RDONLY
    assert run.parent == output.parent == V.resolve()
    assert re.fullmatch(r'gen45_full_parallel_r[0-9]+', run.name)
    assert re.fullmatch(r'gen45_full_parallel_independent_monitor_r[0-9]+', output.name)
    assert run.is_dir() and output.parent.stat().st_dev == M.stat().st_dev
    cfg = json.loads((E/'grasp_records/archive_config.json').read_bytes())
    policy = json.loads((E/'maintenance/storage_control.json').read_bytes())
    assert cfg['upload_enabled'] is False and cfg['delete_enabled'] is False
    assert policy['archive_policy'] == 'local_only'
    return run, output


def process(pid):
    p = Path('/proc')/str(pid)
    raw = (p/'stat').read_text().rsplit(')', 1)[1].split()
    arguments = [a.decode(errors='replace') for a in (p/'cmdline').read_bytes().split(b'\0') if a]
    io = {}
    try:
        io = {k: int(v) for k, v in (line.split(':', 1) for line in (p/'io').read_text().splitlines())}
    except (PermissionError, FileNotFoundError, ProcessLookupError):
        pass
    return dict(pid=pid, ppid=int(raw[1]), pgid=int(raw[2]), sid=int(raw[3]), state=raw[0],
        start_ticks=int(raw[19]), cpu_sec=(int(raw[11])+int(raw[12]))/HZ,
        rss_bytes=int(raw[21])*PAGE, threads=int(raw[17]),
        block_io_delay_sec=int(raw[39])/HZ if len(raw)>39 else None,
        arguments=arguments, command=' '.join(arguments), io=io)


def sample(run=RUN):
    run = Path(run).resolve()
    def belongs(argument):
        value = argument.split('=',1)[1] if argument.startswith('--') and '=' in argument else argument
        path = Path(value)
        if not path.is_absolute():
            return False
        try:
            path.resolve().relative_to(run)
            return True
        except ValueError:
            return False
    rows = []
    for p in Path('/proc').iterdir():
        if not p.name.isdigit() or int(p.name) == os.getpid():
            continue
        try:
            row = process(int(p.name))
            argv = row['arguments']
            if not any(belongs(argument) for argument in argv):
                continue
            # Exact argv membership excludes this monitor/finalizer and their
            # shell wrappers when --run contains the same output path.
            if any(argv[i:i+2] == ['-m', 'pytest'] for i in range(len(argv)-1)):
                kind = 'pytest'
            elif any(Path(argument).name == RUNNER.name for argument in argv):
                kind = 'worker' if '--worker-plan' in argv else 'coordinator'
            else:
                continue
            row['kind'] = kind
            rows.append(row)
        except (FileNotFoundError, ProcessLookupError, PermissionError, IndexError, ValueError):
            pass
    cpu = list(map(int, Path('/proc/stat').read_text().splitlines()[0].split()[1:]))
    mem = {k:int(v.split()[0])*1024 for k,v in
        (line.split(':',1) for line in Path('/proc/meminfo').read_text().splitlines())}
    pressure = Path('/proc/pressure/io')
    return dict(recorded_ns=time.time_ns(), monotonic=time.monotonic(),
        cpu_ticks=cpu, processes=sorted(rows, key=lambda r:r['pid']),
        mem_available_bytes=mem['MemAvailable'],
        io_pressure_text=pressure.read_text() if pressure.exists() else None,
        affinity_cpus=sorted(os.sched_getaffinity(0)), os_cpu_count=os.cpu_count())


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run', type=Path, default=RUN)
    parser.add_argument('--output', type=Path, default=OUT)
    parser.add_argument('--interval-sec', type=float, default=5.)
    args = parser.parse_args()
    run, output = locations(args.run, args.output)
    assert 0. < args.interval_sec <= 10.
    plan_sha = sha(run/'plan.json')
    plan = json.loads((run/'plan.json').read_bytes())
    assert plan['mode'] == 'full' and plan['context']['compatibility']['variant'] == 'candidate'
    assert Path(plan['output']).resolve() == run and Path(plan['runner_path']).resolve() == RUNNER
    assert sha(RUNNER) == plan['runner_sha256']
    output.mkdir(exist_ok=True)
    before = sample(run); time.sleep(args.interval_sec); after = sample(run)
    assert sha(run/'plan.json') == plan_sha
    dt = after['monotonic']-before['monotonic']
    old = {(r['pid'], r['start_ticks']):r for r in before['processes']}
    for row in after['processes']:
        previous = old.get((row['pid'], row['start_ticks']))
        if previous:
            row['sample_cpu_percent'] = 100.*(row['cpu_sec']-previous['cpu_sec'])/dt
            if row['block_io_delay_sec'] is not None and previous['block_io_delay_sec'] is not None:
                row['sample_block_io_delay_sec'] = row['block_io_delay_sec']-previous['block_io_delay_sec']
            row['sample_io_delta'] = {k:row['io'][k]-previous['io'][k]
                for k in row['io'] if k in previous['io']}
    delta = [b-a for a,b in zip(before['cpu_ticks'], after['cpu_ticks'])]
    total = sum(delta[:8])  # Linux user/nice already include guest columns.
    value = dict(kind='GEN45_independent_read_only_actual_worker_sample', run=str(run),
        plan_sha256=plan_sha, planned_worker_count=plan['worker_count'],
        before=before, after=after, interval_sec=dt,
        system_iowait_percent=100.*delta[4]/total if total else None,
        system_busy_percent=100.*(total-delta[3]-delta[4])/total if total else None,
        pytest_cpu_percent_sum=sum(r.get('sample_cpu_percent',0.) for r in after['processes'] if r['kind']=='pytest'),
        pytest_rss_bytes_sum=sum(r['rss_bytes'] for r in after['processes'] if r['kind']=='pytest'),
        pytest_process_count=sum(r['kind']=='pytest' for r in after['processes']),
        pytest_pgids=sorted({r['pgid'] for point in (before, after)
            for r in point['processes'] if r['kind']=='pytest'}),
        helper_sha256=sha(__file__), physical_or_ROS_access=False,
        environment_read=False, temperature_read=False,
        limitations='Two bounded process/system snapshots; not whole-run per-test IO attribution or a full peak-memory guarantee.')
    target = output/('sample_'+str(after['recorded_ns'])+'.json')
    with target.open('x') as stream:
        json.dump(value,stream,ensure_ascii=False,indent=2,allow_nan=False)
        stream.write('\n');stream.flush();os.fsync(stream.fileno())
    assert json.loads(target.read_bytes()) == value
    print(json.dumps(dict(output=str(target),sha256=sha(target),
        pytest_count=value['pytest_process_count'],pgids=value['pytest_pgids'],
        cpu_percent_sum=value['pytest_cpu_percent_sum'],rss_bytes_sum=value['pytest_rss_bytes_sum'],
        iowait_percent=value['system_iowait_percent']),ensure_ascii=False))


if __name__ == '__main__':
    main()
