"""Bounded read-only process and system samples for the exact GEN50 full run."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import time

W = Path(__file__).resolve().parents[1]
V = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_gen41_restart_20261018T011222Z/verification')
RUN = V/'gen50_full_parallel_r1'
OUT = V/'gen50_full_parallel_independent_monitor_r1'
MANIFEST_SHA = 'cee46e1b6b928b02e9658198d86b671ee1e3cf056684ea794a2531c11f4e4c5d'
HZ = os.sysconf('SC_CLK_TCK')
PAGE = os.sysconf('SC_PAGE_SIZE')


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def read(path):
    return json.loads(Path(path).read_bytes())


def process(pid):
    base = Path('/proc')/str(pid)
    fields = (base/'stat').read_text().rsplit(')', 1)[1].split()
    args = (base/'cmdline').read_bytes().replace(b'\0', b' ').decode(errors='replace').strip()
    io = {}
    try:
        io = {key:int(value) for key,value in
              (line.split(':', 1) for line in (base/'io').read_text().splitlines())}
    except (PermissionError, FileNotFoundError, ProcessLookupError):
        pass
    status = dict(line.split(':', 1) for line in (base/'status').read_text().splitlines())
    return dict(pid=pid, ppid=int(fields[1]), pgid=int(fields[2]), sid=int(fields[3]),
        state=fields[0], start_ticks=int(fields[19]),
        cpu_sec=(int(fields[11])+int(fields[12]))/HZ,
        rss_bytes=int(fields[21])*PAGE, threads=int(fields[17]),
        peak_rss_kib=int(status.get('VmHWM', '0').split()[0]),
        block_io_delay_sec=int(fields[39])/HZ if len(fields)>39 else None,
        command_sha256=hashlib.sha256(args.encode()).hexdigest(),
        exact_output_prefix_in_command=str(RUN) in args,
        kind=('pytest' if ' -m pytest ' in ' '+args+' ' else
              'worker' if ' --worker-plan ' in ' '+args+' ' else 'coordinator'), io=io)


def descendants(coordinator_pid):
    # Traverse only this explicitly assigned regression subtree. No other
    # process command lines or environments are read.
    pending = [(coordinator_pid, 0)]
    seen = set()
    rows = []
    while pending:
        pid, depth = pending.pop()
        if pid in seen or depth > 3:
            continue
        seen.add(pid)
        try:
            row = process(pid)
            if not row['exact_output_prefix_in_command']:
                continue
            rows.append(row)
            child_file = Path('/proc')/str(pid)/'task'/str(pid)/'children'
            pending.extend((int(child), depth+1) for child in child_file.read_text().split())
        except (FileNotFoundError, ProcessLookupError, PermissionError, IndexError, ValueError):
            pass
    return sorted(rows, key=lambda row: row['pid'])


def sample(coordinator_pid):
    cpu = list(map(int, Path('/proc/stat').read_text().splitlines()[0].split()[1:]))
    mem = {key:int(value.split()[0])*1024 for key,value in
           (line.split(':', 1) for line in Path('/proc/meminfo').read_text().splitlines())}
    pressure = Path('/proc/pressure/io')
    return dict(recorded_ns=time.time_ns(), monotonic=time.monotonic(),
        cpu_ticks=cpu, processes=descendants(coordinator_pid),
        mem_available_bytes=mem['MemAvailable'],
        io_pressure_text=pressure.read_text() if pressure.exists() else None,
        affinity_cpus=sorted(os.sched_getaffinity(0)), os_cpu_count=os.cpu_count())


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--coordinator-pid', type=int, required=True)
    parser.add_argument('--interval-sec', type=float, default=4.)
    args = parser.parse_args()
    assert 0. < args.interval_sec <= 10.
    assert Path('/media/zhuyupei/SanStand').is_mount()
    plan = read(RUN/'plan.json')
    assert plan['worker_count'] == 7 and plan['expected_case_count'] == 8818
    assert plan['candidate_manifest_sha256'] == MANIFEST_SHA
    assert sha(plan['candidate_manifest_path']) == MANIFEST_SHA
    sources = plan['context']['sources']
    assert len(sources) == 1058 and all(sha(W/path) == value for path,value in sources.items())
    before = sample(args.coordinator_pid)
    time.sleep(args.interval_sec)
    after = sample(args.coordinator_pid)
    dt = after['monotonic']-before['monotonic']
    old = {(row['pid'], row['start_ticks']):row for row in before['processes']}
    for row in after['processes']:
        previous = old.get((row['pid'], row['start_ticks']))
        if previous:
            row['sample_cpu_percent'] = 100.*(row['cpu_sec']-previous['cpu_sec'])/dt
            row['sample_io_delta'] = {key:row['io'][key]-previous['io'][key]
                                     for key in row['io'] if key in previous['io']}
            row['sample_block_io_delay_sec'] = row['block_io_delay_sec']-previous['block_io_delay_sec']
    delta = [new-old for old,new in zip(before['cpu_ticks'], after['cpu_ticks'])]
    total = sum(delta[:8])  # guest is included in user/nice already.
    pytest = [row for row in after['processes'] if row['kind'] == 'pytest']
    value = dict(kind='independent_read_only_exact_seven_worker_sample',
        run=str(RUN), coordinator_pid=args.coordinator_pid, before=before, after=after,
        interval_sec=dt, source1058_sha256_verified_before=True,
        pytest_process_count=len(pytest), pytest_pgids=sorted({row['pgid'] for row in pytest}),
        pytest_cpu_percent_sum=sum(row.get('sample_cpu_percent', 0.) for row in pytest),
        pytest_rss_bytes_sum=sum(row['rss_bytes'] for row in pytest),
        system_iowait_percent=100.*delta[4]/total if total else None,
        system_busy_percent=100.*(total-delta[3]-delta[4])/total if total else None,
        helper_sha256=sha(__file__), environment_read=False,
        physical_or_ROS_access=False, temperature_read=False,
        limits='Bounded sampled windows. System-wide iowait is not whole-run or per-test IO attribution.')
    OUT.mkdir(exist_ok=True)
    output = OUT/('sample_'+str(after['recorded_ns'])+'.json')
    with output.open('x') as handle:
        json.dump(value, handle, ensure_ascii=False, indent=2);handle.write('\n')
    assert read(output) == value
    print(json.dumps(dict(output=str(output), sha256=sha(output),
        pytest_count=len(pytest), pgids=value['pytest_pgids'],
        cpu_percent_sum=value['pytest_cpu_percent_sum'],
        rss_bytes_sum=value['pytest_rss_bytes_sum'],
        iowait_percent=value['system_iowait_percent']), ensure_ascii=False))


if __name__ == '__main__':
    main()
