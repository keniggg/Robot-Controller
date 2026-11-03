#!/usr/bin/env python3
"""Bounded, passive /proc samples for explicitly identified processes.

No ROS imports, process signals, device reads, or implicit PID discovery. This
is resource evidence, not a task/robot readiness check. A task window must be
provided by the actual caller; collection time alone never implies task load.
"""
import argparse
import hashlib
import importlib.util
import json
import math
import os
from pathlib import Path
import signal
import sys
import time

sys.dont_write_bytecode = True
STORAGE = None
STORAGE_BINDING = None
HZ = os.sysconf('SC_CLK_TCK')
PAGE = os.sysconf('SC_PAGE_SIZE')


def require(ok, reason):
    if not ok:
        raise ValueError(reason)


def read_json(path):
    raw = Path(path).read_bytes()
    return json.loads(raw), hashlib.sha256(raw).hexdigest()


def save_new(path, value):
    with Path(path).open('x') as stream:
        json.dump(value, stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.write('\n')


def storage(output, expected_device=None):
    require(STORAGE is not None, 'experiment storage selection is missing')
    STORAGE.validate(output, subtree=STORAGE.ros_log_root, expected=STORAGE_BINDING)
    device = STORAGE.mount.stat().st_dev
    require(expected_device is None or device == expected_device, 'selected storage device changed')
    require(STORAGE.ros_log_root in output.parents, 'output must be below selected ros_log')
    parent = output if output.exists() else output.parent
    require(parent.is_dir() and parent.stat().st_dev == device, 'output parent is not on selected disk')
    return device


def proc_stat(pid):
    # comm may contain spaces and closing parentheses; fields after its last
    # closing parenthesis begin at stat field 3 (state).
    raw = (Path('/proc') / str(pid) / 'stat').read_text()
    fields = raw[raw.rfind(')') + 2:].split()
    return dict(pid=int(pid), state=fields[0], parent_pid=int(fields[1]),
        cpu_ticks=int(fields[11]) + int(fields[12]), threads=int(fields[17]),
        start_ticks=int(fields[19]), rss_bytes=int(fields[21]) * PAGE,
        block_io_delay_ticks=int(fields[39]) if len(fields) > 39 else None)


def identity_matches(row, expected):
    return (row['pid'] == expected['pid'] and row['start_ticks'] == expected['start_ticks']
        and row['state'] != 'Z' and ('parent_pid' not in expected
            or row['parent_pid'] == expected['parent_pid']))


def normalize_targets(value):
    require(isinstance(value, dict) and value, 'targets must be a nonempty label/identity mapping')
    result = {}
    for label, record in value.items():
        require(isinstance(record, dict), 'target identity must be an object')
        identity = record.get('identity', record)
        require(isinstance(label, str) and label and isinstance(identity, dict), 'invalid target')
        result[label] = {key: int(identity[key]) for key in ('pid', 'start_ticks')}
        if 'parent_pid' in identity:
            result[label]['parent_pid'] = int(identity['parent_pid'])
        require(all(number > 0 for number in result[label].values()), 'invalid process identity')
    require(len({(row['pid'], row['start_ticks']) for row in result.values()}) == len(result),
            'duplicate process identities would double count resources')
    return result


def sample_process(expected):
    try:
        before = proc_stat(expected['pid'])
        if not identity_matches(before, expected):
            return dict(valid=False, reason='identity_missing_or_changed', observed=before)
        io, io_error = {}, None
        try:
            for line in (Path('/proc') / str(expected['pid']) / 'io').read_text().splitlines():
                key, value = line.split(':', 1)
                io[key] = int(value)
        except (OSError, ValueError) as exc:
            io_error = type(exc).__name__
        after = proc_stat(expected['pid'])
        if not identity_matches(after, expected):
            return dict(valid=False, reason='identity_changed_during_read', observed=after)
        return dict(after, valid=True, io=io, io_error=io_error,
                    observed_monotonic_ns=time.monotonic_ns())
    except (OSError, ValueError, IndexError) as exc:
        return dict(valid=False, reason=type(exc).__name__)


def host_sample():
    cpu, counters = None, {}
    for line in Path('/proc/stat').read_text().splitlines():
        fields = line.split()
        if fields and fields[0] == 'cpu':
            cpu = [int(value) for value in fields[1:9]]  # exclude guest double counting
        elif fields and fields[0] in ('procs_running', 'procs_blocked'):
            counters[fields[0]] = int(fields[1])
    mem = {}
    for line in Path('/proc/meminfo').read_text().splitlines():
        key, value = line.split(':', 1)
        if key in ('MemTotal', 'MemAvailable', 'SwapTotal', 'SwapFree', 'Dirty', 'Writeback'):
            mem[key + '_bytes'] = int(value.split()[0]) * 1024
    return dict(cpu_ticks=cpu, loadavg=Path('/proc/loadavg').read_text().strip(),
                memory=mem, **counters)


def annotate_rates(row, previous):
    if previous is None:
        return
    dt = (row['monotonic_ns'] - previous['monotonic_ns']) / 1e9
    row['interval_sec'] = dt
    old_cpu, cpu = previous['host']['cpu_ticks'], row['host']['cpu_ticks']
    delta = [new - old for new, old in zip(cpu, old_cpu)]
    total = sum(delta)
    if total > 0:
        row['host']['busy_fraction_excluding_iowait'] = (total - delta[3] - delta[4]) / total
        row['host']['iowait_fraction'] = delta[4] / total
    for label, current in row['processes'].items():
        old = previous['processes'].get(label, {})
        if not current.get('valid') or not old.get('valid'):
            continue
        seconds = (current['observed_monotonic_ns'] - old['observed_monotonic_ns']) / 1e9
        if seconds <= 0 or (current['pid'], current['start_ticks']) != (old['pid'], old['start_ticks']):
            continue
        current['cpu_core_equivalents'] = (current['cpu_ticks'] - old['cpu_ticks']) / HZ / seconds
        current['io_per_sec'] = {key: (value - old['io'][key]) / seconds
            for key, value in current['io'].items() if key in old.get('io', {})}


def task_scope(path, wall_ns):
    if path is None or not path.exists():
        return dict(scope='unclassified_requires_task_timeline')
    try:
        window, digest = read_json(path)
        start = window['start_ns']
        end = window.get('end_ns')
        require(type(start) is int and start > 0, 'invalid task start')
        require(end is None or type(end) is int and end >= start, 'invalid task end')
        scope = 'before_task' if wall_ns < start else ('after_task' if end is not None and wall_ns >= end else 'task_window')
        return dict(scope=scope, window=window, window_sha256=digest,
                    authority='caller_supplied_boundary_not_ROS_observed')
    except (OSError, ValueError, KeyError, TypeError) as exc:
        return dict(scope='unclassified_invalid_task_window', error=type(exc).__name__)


def main(argv=None):
    global STORAGE, STORAGE_BINDING
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--targets', required=True, type=Path,
        help='JSON with targets={label:{pid,start_ticks,parent_pid?}} and optional evidence')
    parser.add_argument('--output', required=True, type=Path, help='fresh directory under selected disk ros_log')
    parser.add_argument('--storage-backend', choices=('ssd', 'sanstand'))
    parser.add_argument('--duration-sec', type=float, default=900.)
    parser.add_argument('--interval-sec', type=float, default=1.)
    parser.add_argument('--stop-file', type=Path)
    parser.add_argument('--task-window-file', type=Path)
    parser.add_argument('--extra-targets-file', type=Path,
        help='optional caller-written targets JSON for explicit recorder/runner identities started later')
    args = parser.parse_args(argv)
    require(math.isfinite(args.duration_sec) and 1 <= args.duration_sec <= 3600, 'duration must be 1..3600 seconds')
    require(math.isfinite(args.interval_sec) and .5 <= args.interval_sec <= 10, 'interval must be .5..10 seconds')
    output = args.output.absolute()
    spec = importlib.util.spec_from_file_location('_resource_storage', Path(__file__).resolve().parents[1]/'experiment_storage.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    STORAGE = module.select(args.storage_backend, path=output)
    STORAGE_BINDING = STORAGE.validate(output, subtree=STORAGE.ros_log_root)
    device = storage(output)
    binding, binding_sha = read_json(args.targets)
    targets = normalize_targets(binding['targets'])
    for label, expected in targets.items():
        require(identity_matches(proc_stat(expected['pid']), expected), 'initial identity differs: ' + label)
    output.mkdir()  # no overwrite/resume of prior samples
    stop_file = args.stop_file.resolve() if args.stop_file else output / 'STOP'
    task_file = args.task_window_file.resolve() if args.task_window_file else output / 'task_window.json'
    extra_file = args.extra_targets_file.resolve() if args.extra_targets_file else output / 'extra_targets.json'
    stopped = []
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda number, _frame: stopped.append(number))
    began = time.monotonic_ns()
    deadline = began + int(args.duration_sec * 1e9)
    metadata = dict(schema=1, started_wall_ns=time.time_ns(), started_monotonic_ns=began,
        storage_binding=STORAGE_BINDING,
        sampler_identity=proc_stat(os.getpid()), sampler_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        target_binding=dict(path=str(args.targets.resolve()), sha256=binding_sha, value=binding),
        cpu_count=os.cpu_count(), sampler_cpu_affinity=sorted(os.sched_getaffinity(0)),
        clock_ticks_per_sec=HZ, page_bytes=PAGE, interval_sec=args.interval_sec,
        duration_limit_sec=args.duration_sec, stop_file=str(stop_file), task_window_file=str(task_file),
        extra_targets_file=str(extra_file),
        scope='explicit processes only; remote-server and unlisted recorder CPU are not measured',
        io_semantics='rchar/wchar are logical bytes; read_bytes/write_bytes are process storage accounting, not causal disk diagnosis',
        rss_semantics='per-process RSS can include shared pages; summed RSS is not unique host memory',
        authority='read-only resource observer; no ROS or device commands')
    save_new(output / 'manifest.json', metadata)
    previous, count, invalid, max_sampling_sec = None, 0, set(), 0.
    next_tick = began
    reason = 'duration_limit'
    with (output / 'samples.jsonl').open('x', buffering=1) as stream:
        while time.monotonic_ns() < deadline:
            storage(output, device)
            if stopped or stop_file.exists():
                reason = 'signal' if stopped else 'stop_file'
                break
            sample_start = time.monotonic_ns()
            selected, extra_binding = dict(targets), None
            if extra_file.exists():
                try:
                    extra, digest = read_json(extra_file)
                    additions = normalize_targets(extra['targets'])
                    require(not set(additions).intersection(targets), 'extra label overlaps base target')
                    selected = normalize_targets(dict(targets, **additions))
                    extra_binding = dict(path=str(extra_file), sha256=digest, targets=additions)
                except (OSError, ValueError, KeyError, TypeError) as exc:
                    selected = dict(targets)
                    extra_binding = dict(error=type(exc).__name__, detail=str(exc))
            row = dict(index=count, wall_ns=time.time_ns(), monotonic_ns=sample_start,
                schedule_lateness_sec=max(0., (sample_start - next_tick) / 1e9),
                host=host_sample(), processes={label: sample_process(expected) for label, expected in selected.items()},
                extra_target_binding=extra_binding)
            row['task_scope'] = task_scope(task_file, row['wall_ns'])
            annotate_rates(row, previous)
            row['sampling_sec'] = (time.monotonic_ns() - sample_start) / 1e9
            max_sampling_sec = max(max_sampling_sec, row['sampling_sec'])
            invalid.update(label for label, value in row['processes'].items() if not value['valid'])
            storage(output, device)
            stream.write(json.dumps(row, ensure_ascii=False, allow_nan=False) + '\n')
            previous, count = row, count + 1
            next_tick += int(args.interval_sec * 1e9)
            # Avoid catch-up bursts if the scheduler or filesystem delayed us.
            if next_tick < time.monotonic_ns():
                next_tick = time.monotonic_ns() + int(args.interval_sec * 1e9)
            while not stopped and time.monotonic_ns() < min(next_tick, deadline):
                time.sleep(min(.1, max(0., (min(next_tick, deadline) - time.monotonic_ns()) / 1e9)))
        stream.flush()
    storage(output, device)
    save_new(output / 'completion.json', dict(schema=1, stop_reason=reason,
        completed_wall_ns=time.time_ns(), elapsed_sec=(time.monotonic_ns() - began) / 1e9,
        samples=count, invalid_process_labels=sorted(invalid), maximum_sampling_sec=max_sampling_sec,
        task_load_summary_computed=False, task_window_file=str(task_file),
        note='Align exact wall timestamps with actual task timeline; preparation/idle samples are not task load.'))
    return 0


if __name__ == '__main__':
    try:
        sys.exit(main())
    except (OSError, ValueError, RuntimeError, KeyError, IndexError) as exc:
        print('resource sampler stopped: ' + str(exc), file=sys.stderr)
        sys.exit(2)
