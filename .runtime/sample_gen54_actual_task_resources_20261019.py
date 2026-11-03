#!/usr/bin/env python3
"""Passive whole-task /proc sampling; does not import ROS or touch devices.

Use a current accepted deployment and start before the recorded task. A record
wrapper is discovered only when its exact workspace script and explicit external
--root/--config argument agree. PID reuse does not inherit an old attribution.
"""
import argparse
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import signal
import sys
import time

sys.dont_write_bytecode = True
W = Path(__file__).resolve().parents[1]
M = Path('/media/zhuyupei/SanStand')
E = M / '实验记录/Robot-Controller'
BASE = W / '.runtime/sample_gen50_actual_task_resources_20261018.py'
BASE_SHA = '9cac8cb02c1b9cacf80e3d4bf7833b7d6b6e0a89bd8dd0d71ea50dcdbc6ee367'
ROLES = {
    '/supervisor_camera': 'camera',
    '/unknown_tabletop_perception': 'perception',
    '/remote_grasp6d_node': 'registration_and_candidate_planning',
    '/move_group': 'path_planning',
    '/grasp_task_node': 'task_state_and_execution',
    '/motion_gateway': 'motion_handoff_and_feedback',
    '/alicia_d_driver_node': 'driver_feedback',
    '/bessica_d_hw_interface': 'controller_feedback',
    '/compliant_gripper_controller': 'gripper_feedback',
    '/grasp_mode_router': 'mode_state',
    '/alicia_supervisor_gui': 'gui',
}


def require(condition, text):
    if not condition:
        raise RuntimeError(text)


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def save(path, value):
    with path.open('x') as stream:
        json.dump(value, stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.write('\n')


def storage_check(output, device=None):
    require(M.is_mount(), 'SanStand is not mounted; no local fallback')
    require(not os.statvfs(M).f_flag & os.ST_RDONLY, 'SanStand is read-only')
    current = M.stat().st_dev
    require(device is None or device == current, 'SanStand device changed')
    parent = output if output.exists() else output.parent
    require(parent.exists() and parent.stat().st_dev == current,
            'output parent is not on SanStand')
    require((E / 'ros_log').resolve() in output.parents,
            'output must be a fresh child under external ros_log')
    return current


def primitives():
    require(sha(BASE) == BASE_SHA, 'reviewed /proc primitives changed')
    spec = importlib.util.spec_from_file_location('gen54_proc_primitives', BASE)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def deployment_targets(value):
    require(value.get('deployment_passed') is True and
            value.get('whole_patch_deployed') is True, 'deployment not accepted')
    after, preserved = value.get('after', {}), value.get('preserved_after', {})
    require(not set(after) & set(preserved), 'overlapping deployment groups')
    targets = dict(after, **preserved)
    require(set(targets) == set(ROLES), 'expected exact eleven critical nodes')
    require((len(after), len(preserved)) in ((4, 7), (11, 0)),
            'unsupported deployment layout (expected compute 4+7 or cold 11+0)')
    require(len({int(row['pid']) for row in targets.values()}) == 11,
            'critical deployment PID aliases are unexpected')
    return {name: dict(pid=int(row['pid']), start_ticks=int(row['start_ticks']),
                      parent_pid=int(row['parent_pid']))
            for name, row in targets.items()}


def inventory(base):
    rows = {}
    for entry in Path('/proc').iterdir():
        if not entry.name.isdigit() or int(entry.name) == os.getpid():
            continue
        try:
            row = base.stat(entry / 'stat')
            if row['state'] != 'Z':
                rows[int(entry.name)] = row
        except (OSError, ValueError, IndexError):
            continue
    return rows


def script_and_args(pid, allowed_basenames=None):
    """Read only argv needed for process identity; no argv/environment retained."""
    proc = Path('/proc') / str(pid)
    argv = [part.decode(errors='replace') for part in
            (proc / 'cmdline').read_bytes().split(b'\0') if part]
    if not argv:
        return None, ()
    executable = Path(argv[0]).name
    if executable.startswith('python'):
        candidates = argv[1:]
        while candidates and candidates[0] in ('-u', '-B', '-I', '-E', '-s', '-S', '-O', '-OO'):
            candidates = candidates[1:]
        if not candidates or candidates[0].startswith('-'):
            return None, ()
        script, arguments = candidates[0], candidates[1:]
    else:
        script, arguments = argv[0], argv[1:]
    path = Path(script)
    if allowed_basenames is not None and path.name not in allowed_basenames:
        return None, ()
    if not path.is_absolute():
        path = (proc / 'cwd').resolve(strict=True) / path
    return path.resolve(strict=True), arguments


def option_values(arguments, name):
    # A wrapper's trailing command is not part of the wrapper identity options.
    arguments = list(arguments)
    if '--' in arguments:
        arguments = arguments[:arguments.index('--')]
    found = []
    for index, argument in enumerate(arguments):
        if argument == name and index + 1 < len(arguments):
            found.append(arguments[index + 1])
        elif argument.startswith(name + '='):
            found.append(argument.partition('=')[2])
    return found


def record_root_identity(pid, expected_scripts, record_root):
    script, arguments = script_and_args(pid, {path.name for path in expected_scripts})
    if script not in expected_scripts:
        return None
    explicit = []
    for flag, expected in (('--root', record_root),
                           ('--config', record_root / 'archive_config.json')):
        values = option_values(arguments, flag)
        if values:
            # Ambiguous repeated flags or relative path spellings are rejected.
            if len(values) != 1 or not Path(values[0]).is_absolute():
                return None
            if Path(values[0]).resolve(strict=True) != expected:
                return None
            explicit.append(flag)
    if not explicit:
        return None
    return dict(script=str(script), verified_external_options=explicit,
                record_root=str(record_root))


def disk_counters(text):
    result = {}
    for line in (text or '').splitlines():
        fields = line.split()
        if len(fields) >= 14:
            result[(int(fields[0]), int(fields[1]))] = dict(
                name=fields[2], read_bytes=int(fields[5]) * 512,
                write_bytes=int(fields[9]) * 512, busy_ms=int(fields[12]),
                weighted_busy_ms=int(fields[13]))
    return result


def snapshot(base, index, targets, owned, scripts, record_root, device):
    began = time.monotonic()
    inv = inventory(base)
    errors, discoveries, valid_deployment_names = [], [], set()
    for name, expected in targets.items():
        pid = expected['pid']
        actual = inv.get(pid)
        if actual is None or actual['start_ticks'] != expected['start_ticks'] or actual['ppid'] != expected['parent_pid']:
            errors.append(dict(name=name, pid=pid, reason='deployment_identity_missing_or_changed'))
            continue
        key = (pid, actual['start_ticks'])
        valid_deployment_names.add(name)
        owned.setdefault(key, dict(name=name, role=ROLES[name], owner=name,
                                   lineage='accepted_deployment', parent_at_discovery=actual['ppid']))
    for pid, actual in inv.items():
        key = (pid, actual['start_ticks'])
        if key in owned:
            continue
        try:
            identity = record_root_identity(pid, scripts, record_root)
            # Verify identity again after reading cmdline to exclude a PID race.
            check = base.stat(Path('/proc') / str(pid) / 'stat')
            if identity and check['start_ticks'] == actual['start_ticks'] and check['ppid'] == actual['ppid']:
                owned[key] = dict(name='record_wrapper', role='recording_wrapper',
                    owner='record_wrapper:%d:%d' % key, lineage='exact_script_and_external_root',
                    parent_at_discovery=actual['ppid'], **identity)
                discoveries.append(dict(pid=pid, start_ticks=actual['start_ticks'], **owned[key]))
        except (OSError, ValueError, IndexError):
            continue
    children = {}
    for pid, actual in inv.items():
        children.setdefault(actual['ppid'], []).append(pid)
    queue = [key for key in owned if key[0] in inv and inv[key[0]]['start_ticks'] == key[1]
             and (owned[key]['owner'] not in ROLES or owned[key]['owner'] in valid_deployment_names)]
    visited = set()
    while queue:
        key = queue.pop(0)
        if key in visited:
            continue
        visited.add(key)
        parent = owned[key]
        for pid in children.get(key[0], ()):
            actual = inv[pid]
            child = (pid, actual['start_ticks'])
            if child not in owned:
                # A /proc enumeration is not atomic. Confirm both sides of a
                # new lineage edge before retaining it across later reparenting.
                try:
                    parent_before = base.stat(Path('/proc') / str(key[0]) / 'stat')
                    child_now = base.stat(Path('/proc') / str(pid) / 'stat')
                    parent_after = base.stat(Path('/proc') / str(key[0]) / 'stat')
                    if (parent_before['start_ticks'] != key[1] or
                            parent_after['start_ticks'] != key[1] or
                            child_now['start_ticks'] != child[1] or child_now['ppid'] != key[0]):
                        continue
                except (OSError, ValueError, IndexError):
                    continue
                owned[child] = dict(name=parent['name'] + '/descendant', role=parent['role'],
                    owner=parent['owner'], lineage='observed_parent_child',
                    parent_identity=dict(pid=key[0], start_ticks=key[1]),
                    parent_at_discovery=key[0])
                try:
                    script, unused_args = script_and_args(pid)
                    if script == W / 'tools/record_grasp_keyframes_readonly.py':
                        owned[child].update(name='record_grasp_keyframes_readonly.py', role='recording')
                except (OSError, ValueError, IndexError):
                    pass
                discoveries.append(dict(pid=pid, start_ticks=child[1], **owned[child]))
            queue.append(child)
    processes = []
    for key in sorted(visited):
        try:
            item = base.process(key[0], owned[key]['name'], threads=index % 5 == 0,
                                limits=index % 30 == 0)
            after = base.stat(Path('/proc') / str(key[0]) / 'stat')
            if (item['start_ticks'] != key[1] or item['ppid'] != inv[key[0]]['ppid'] or
                    after['start_ticks'] != key[1] or after['ppid'] != item['ppid'] or
                    after['state'] == 'Z'):
                raise ValueError('identity_changed_during_read')
            item['attribution'] = owned[key]
            processes.append(item)
        except (OSError, ValueError, IndexError) as exc:
            errors.append(dict(pid=key[0], start_ticks=key[1], reason=type(exc).__name__))
    require(len({(row['pid'], row['start_ticks']) for row in processes}) == len(processes),
            'duplicate process accounting')
    memory = base.numeric_lines('/proc/meminfo')
    disks = base.read('/proc/diskstats')
    counters = disk_counters(disks)
    external = counters.get((os.major(device), os.minor(device)))
    return dict(recorded_ns=time.time_ns(), monotonic_ns=time.monotonic_ns(),
        cpu_ticks=[int(v) for v in Path('/proc/stat').read_text().splitlines()[0].split()[1:]],
        loadavg=base.read('/proc/loadavg'), diskstats=disks,
        external_filesystem_device=dict(major=os.major(device), minor=os.minor(device), counters=external),
        memory_kib={k: memory[k] for k in ('MemTotal', 'MemAvailable', 'SwapTotal', 'SwapFree', 'Dirty', 'Writeback') if k in memory},
        pressure={kind: base.read('/proc/pressure/' + kind) for kind in ('cpu', 'memory', 'io')},
        processes=processes, identity_errors=errors, discoveries=discoveries,
        collection_sec=time.monotonic() - began)


def interval(base, previous, current):
    seconds = (current['monotonic_ns'] - previous['monotonic_ns']) / 1e9
    prior = {(row['pid'], row['start_ticks']): row for row in previous['processes']}
    cpu_sum = 0.
    matched = 0
    for row in current['processes']:
        old = prior.get((row['pid'], row['start_ticks']))
        if old is None:
            continue
        ticks = row['cpu_ticks'] - old['cpu_ticks']
        require(ticks >= 0 and seconds > 0, 'nonmonotonic process sample')
        row['interval_cpu_percent_one_core_100'] = 100. * ticks / base.HZ / seconds
        row['interval_io_delta'] = {key: value - old['io'][key]
                                    for key, value in row['io'].items() if key in old['io']}
        row['interval_context_switch_delta'] = {key: row['status'][key] - old['status'][key]
            for key in ('voluntary_ctxt_switches', 'nonvoluntary_ctxt_switches')
            if key in row['status'] and key in old['status']}
        cpu_sum += row['interval_cpu_percent_one_core_100']
        matched += 1
    delta = [b - a for a, b in zip(previous['cpu_ticks'][:8], current['cpu_ticks'][:8])]
    total = sum(delta)
    current['interval_summary'] = dict(seconds=seconds, matched_unique_processes=matched,
        tracked_cpu_percent_one_core_100=cpu_sum, tracked_equivalent_cores=cpu_sum / 100.,
        host_busy_percent=100. * (total - delta[3] - delta[4]) / total if total > 0 else None,
        host_iowait_percent=100. * delta[4] / total if total > 0 else None,
        note='new or exited-between-samples processes do not contribute a matched interval')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--deployment', required=True, type=Path)
    parser.add_argument('--deployment-sha256', required=True)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--context', required=True)
    parser.add_argument('--record-root', type=Path, default=E / 'grasp_records')
    parser.add_argument('--record-parent-cmd', action='append', default=None,
                        help='Exact script basename under this workspace tools/ (default run_recorded_grasp.py)')
    parser.add_argument('--interval-sec', type=float, default=1.)
    parser.add_argument('--duration-sec', type=float, default=1800.)
    args = parser.parse_args()
    require(1. <= args.interval_sec <= 10. and 1. <= args.duration_sec <= 1800., 'invalid bounded sampling interval/duration')
    require(0 < len(args.context) <= 200, 'context is required and must be concise')
    deployment = args.deployment.resolve(strict=True)
    require(sha(deployment) == args.deployment_sha256, 'deployment digest changed')
    targets = deployment_targets(json.loads(deployment.read_text()))
    output = args.output.resolve()
    device = storage_check(output)
    require(not output.exists(), 'use a fresh output directory')
    record_root = args.record_root.resolve(strict=True)
    require(record_root == (E / 'grasp_records').resolve(strict=True), 'unexpected record root')
    scripts = set()
    for name in args.record_parent_cmd or ['run_recorded_grasp.py']:
        require(Path(name).name == name and name.endswith('.py'), 'record parent must be a script basename')
        scripts.add((W / 'tools' / name).resolve(strict=True))
    base = primitives()
    for name, expected in targets.items():
        actual = base.stat(Path('/proc') / str(expected['pid']) / 'stat')
        require(actual['start_ticks'] == expected['start_ticks'] and actual['ppid'] == expected['parent_pid'] and actual['state'] != 'Z',
                'accepted deployment process is no longer current: ' + name)
    output.mkdir()
    storage_check(output, device)
    own_sha = sha(__file__)
    metadata = dict(kind='GEN54_passive_whole_task_proc_sampling', context=args.context,
        created_ns=time.time_ns(), helper_path=str(Path(__file__).resolve()), helper_sha256=own_sha,
        primitives_path=str(BASE), primitives_sha256=BASE_SHA, workspace=str(W),
        deployment=str(deployment), deployment_sha256=args.deployment_sha256,
        target_process_identities=targets, process_roles=ROLES,
        record_root=str(record_root), record_parent_scripts=sorted(str(p) for p in scripts),
        clock_ticks_per_second=base.HZ, page_size=base.PAGE, os_cpu_count=os.cpu_count(),
        observer_affinity_cpus=sorted(os.sched_getaffinity(0)), cpu_online=base.read('/sys/devices/system/cpu/online'),
        interval_sec=args.interval_sec, duration_limit_sec=args.duration_sec,
        thread_sampling_every=5, cgroup_sampling_every=30,
        ROS_initialized=False, hardware_contacted=False, temperature_read=False,
        environment_read=False, complete_argv_stored=False, argv_read_for_identity_only=True,
        control_or_configuration_changed=False, task_or_motion_success_claimed=False,
        limitations=[
            'CPU 100 percent means one logical core; 12 CPUs is capacity, not measured task use.',
            'Host counters include unrelated work; process totals count each PID/start once, including verified descendants.',
            'Only exact accepted deployment identities are attributed; a restarted node needs a new accepted deployment/sample.',
            'Record wrapper needs an explicit --root or --config agreeing with SanStand. Default-root-only invocation is deliberately not inferred.',
            'Processes born and exited between samples may be missed; a discovered unchanged PID/start retains lineage if reparented.',
            'Process main-thread wchan/context switches do not describe every worker; thread samples occur every fifth sample.',
            'schedstat is scheduler run/wait data, not application queue occupancy. Disabled counters may be zero.',
            'Proc data cannot measure ROS callback queue, frame latency, GUI frame time, algorithm substage or receipt delays.',
            'Correlate source-stamped task events/request profiles; wall minus thread CPU is not automatically I/O wait.',
            'RSS shares pages; rchar/wchar are logical I/O. Device stats include all SanStand writers.',
            'No speedup or physical outcome is established by this sampler; separate idle/startup/active/terminal windows.'])
    save(output / 'metadata.json', metadata)
    stop = {'signal': None}
    for number in (signal.SIGINT, signal.SIGTERM):
        signal.signal(number, lambda signum, frame: stop.update(signal=signum))
    began, cpu_began = time.monotonic(), time.process_time()
    count, maximum_collection, previous = 0, 0., None
    owned, error_count, discovered_count = {}, 0, 0
    reason = 'duration_limit'
    try:
        with (output / 'samples.jsonl').open('x', buffering=1) as stream:
            while True:
                storage_check(output, device)
                row = snapshot(base, count, targets, owned, scripts, record_root, device)
                if previous is not None:
                    interval(base, previous, row)
                stream.write(json.dumps(row, ensure_ascii=False, separators=(',', ':'), allow_nan=False) + '\n')
                maximum_collection = max(maximum_collection, row['collection_sec'])
                error_count += len(row['identity_errors'])
                discovered_count += len(row['discoveries'])
                count += 1
                previous = row
                if stop['signal'] or (output / 'STOP').exists():
                    reason = 'signal' if stop['signal'] else 'STOP_file'
                    break
                elapsed = time.monotonic() - began
                if elapsed >= args.duration_sec:
                    break
                time.sleep(max(0., min(args.interval_sec - row['collection_sec'], args.duration_sec - elapsed)))
    except BaseException:
        # Never redirect missing/unmounted external evidence to the system disk.
        raise
    storage_check(output, device)
    result = dict(kind=metadata['kind'], observation_only=True, context=args.context,
        sample_count=count, elapsed_sec=time.monotonic() - began,
        observer_cpu_sec=time.process_time() - cpu_began, maximum_collection_sec=maximum_collection,
        stop_reason=reason, stop_signal=stop['signal'], identity_error_count=error_count,
        discovered_process_count=discovered_count, tracked_identity_count=len(owned),
        deployment_unchanged=sha(deployment) == args.deployment_sha256,
        helper_sha256=own_sha, metadata_sha256=sha(output / 'metadata.json'),
        samples_sha256=sha(output / 'samples.jsonl'),
        tests_or_motion_success_claimed=False)
    save(output / 'result.json', result)
    print(json.dumps(dict(output=str(output), result_sha256=sha(output / 'result.json'), **result), ensure_ascii=False))


if __name__ == '__main__':
    main()
