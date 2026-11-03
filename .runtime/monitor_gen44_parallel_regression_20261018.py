"""Read-only process/CPU/IO samples for one exact external regression run."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import time

RUN = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_gen41_restart_20261018T011222Z/verification/gen44_full_parallel_r2')
OUT = RUN.parent/'gen44_full_parallel_independent_monitor_r1'
HZ = os.sysconf('SC_CLK_TCK')
PAGE = os.sysconf('SC_PAGE_SIZE')


def process(pid):
    p = Path('/proc')/str(pid)
    raw = (p/'stat').read_text().rsplit(')', 1)[1].split()
    args = (p/'cmdline').read_bytes().replace(b'\0', b' ').decode(errors='replace').strip()
    io = {}
    try:
        io = {k: int(v) for k, v in (line.split(':', 1) for line in (p/'io').read_text().splitlines())}
    except (PermissionError, FileNotFoundError, ProcessLookupError):
        pass
    return dict(pid=pid, ppid=int(raw[1]), pgid=int(raw[2]), sid=int(raw[3]), state=raw[0],
        start_ticks=int(raw[19]), cpu_sec=(int(raw[11])+int(raw[12]))/HZ,
        rss_bytes=int(raw[21])*PAGE, threads=int(raw[17]),
        block_io_delay_sec=int(raw[39])/HZ if len(raw)>39 else None,
        command=args, io=io)


def sample():
    rows = []
    for p in Path('/proc').iterdir():
        if not p.name.isdigit() or int(p.name) == os.getpid():
            continue
        try:
            # The accepted run's exact absolute output prefix appears in its
            # coordinator, worker and pytest arguments; no environment is read.
            row = process(int(p.name))
            if str(RUN) in row['command']:
                row['kind'] = ('pytest' if ' -m pytest ' in ' '+row['command']+' '
                    else 'worker' if ' --worker-plan ' in ' '+row['command']+' '
                    else 'coordinator')
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
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--interval-sec', type=float, default=5.)
    args=parser.parse_args()
    assert 0.<args.interval_sec<=10. and RUN.is_dir()
    assert Path('/media/zhuyupei/SanStand').is_mount()
    OUT.mkdir(exist_ok=True)
    before=sample();time.sleep(args.interval_sec);after=sample()
    dt=after['monotonic']-before['monotonic']
    old={(r['pid'],r['start_ticks']):r for r in before['processes']}
    for r in after['processes']:
        previous=old.get((r['pid'],r['start_ticks']))
        if previous:
            r['sample_cpu_percent']=100.*(r['cpu_sec']-previous['cpu_sec'])/dt
            r['sample_block_io_delay_sec']=r['block_io_delay_sec']-previous['block_io_delay_sec']
            r['sample_io_delta']={k:r['io'][k]-previous['io'][k] for k in r['io'] if k in previous['io']}
    delta=[b-a for a,b in zip(before['cpu_ticks'],after['cpu_ticks'])]
    # Linux user/nice already include guest; exclude guest columns in denominator.
    total=sum(delta[:8])
    value=dict(kind='independent_read_only_actual_eight_worker_sample', run=str(RUN),
        before=before, after=after, interval_sec=dt,
        system_iowait_percent=100.*delta[4]/total if total else None,
        system_busy_percent=100.*(total-delta[3]-delta[4])/total if total else None,
        pytest_cpu_percent_sum=sum(r.get('sample_cpu_percent',0.) for r in after['processes'] if r['kind']=='pytest'),
        pytest_rss_bytes_sum=sum(r['rss_bytes'] for r in after['processes'] if r['kind']=='pytest'),
        pytest_process_count=sum(r['kind']=='pytest' for r in after['processes']),
        pytest_pgids=sorted({r['pgid'] for r in after['processes'] if r['kind']=='pytest'}),
        helper_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        physical_or_ROS_access=False, environment_read=False, temperature_read=False,
        limitations='Bounded process/system samples only; system-wide iowait is not whole-run per-test IO attribution.')
    output=OUT/('sample_'+str(after['recorded_ns'])+'.json')
    with output.open('x') as f:json.dump(value,f,ensure_ascii=False,indent=2);f.write('\n')
    assert json.loads(output.read_text())==value
    print(json.dumps(dict(output=str(output),sha256=hashlib.sha256(output.read_bytes()).hexdigest(),
        pytest_count=value['pytest_process_count'], pgids=value['pytest_pgids'],
        cpu_percent_sum=value['pytest_cpu_percent_sum'],rss_bytes_sum=value['pytest_rss_bytes_sum'],
        iowait_percent=value['system_iowait_percent']),ensure_ascii=False))


if __name__=='__main__':main()
