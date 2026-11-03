"""Bounded proc-only observer; no ROS, image subscription, or process control."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import time

from observe_sdk_memory_pressure_20261007 import (
    ProcSampler, bind_processes, encode, parse_stat, prepare_output, read_text,
    write_atomic,
)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--unknown', required=True, help='PID:start_ticks')
    p.add_argument('--aggregate', action='append', default=[], help='PID:start_ticks')
    p.add_argument('--output', required=True)
    p.add_argument('--seconds', type=float, default=60.)
    args = p.parse_args()
    assert 0 < args.seconds <= 60.
    bound = bind_processes([args.unknown])
    for value in args.aggregate:
        pid, ticks = value.split(':'); assert int(pid) > 0 and int(ticks) > 0
        base = Path('/proc')/pid
        stat = parse_stat(read_text(base/'stat'))
        assert stat['start_ticks'] == int(ticks)
        if pid not in bound:
            bound[pid] = dict(start_ticks=int(ticks), comm=read_text(base/'comm', 256),
                              explicit_watch=True, tasks={})
    out = prepare_output(args.output)
    ready = dict(pid=os.getpid(), observer_only=True, hardware_commands_sent=False,
                 ros_connections_created=False, period_sec=.1, seconds=args.seconds,
                 bound_processes=bound, sched_schedstats=read_text('/proc/sys/kernel/sched_schedstats'),
                 source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
    write_atomic(out/'ready.json', [encode(ready)])
    sampler, rows, used = ProcSampler(bound), [], 0
    start, cpu = time.monotonic(), time.process_time()
    reason = 'duration_elapsed'
    while time.monotonic()-start < args.seconds and len(rows) < 610:
        before = time.monotonic()
        raw = encode(sampler.sample())
        if used+len(raw) > 24*1024**2:
            reason = 'byte_budget'; break
        rows.append(raw); used += len(raw)
        time.sleep(max(0., min(.1-(time.monotonic()-before), args.seconds-(time.monotonic()-start))))
    evidence = write_atomic(out/'resources.jsonl', rows)
    result = dict(stop_reason=reason, rows=len(rows), bytes=used,
                  elapsed_sec=time.monotonic()-start, observer_cpu_sec=time.process_time()-cpu,
                  resources=evidence, changes_made=False)
    write_atomic(out/'result.json', [encode(result)])
    print(json.dumps(dict(output=str(out), **result)))


if __name__ == '__main__':
    main()
