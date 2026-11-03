#!/usr/bin/env python3
"""Offline GEN55 resource analysis after root seals and reviews a new record.

This helper reads a sealed /proc sample, whitelisted recorded bag topics and
bounded log prefixes. It never samples /proc, initializes ROS, contacts a node,
changes a parameter or controls hardware. No historical task counts or outcome
are substituted for the new record. The original analysis mathematics and bag
reader are reused by pinned file hash; their historical main() is never called.
"""
import argparse
from collections import Counter
import datetime
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import re
import sys
import time

sys.dont_write_bytecode = True
W = Path(__file__).resolve().parents[1]
M = Path('/media/zhuyupei/SanStand')
E = M / '实验记录/Robot-Controller'
N = E / 'ros_log/full_latest_gen41_restart_20261018T011222Z'
V = N / 'verification'
BASE = W / '.runtime/analyze_gen54_gen53_task_resources_20261019.py'
BASE_SHA = 'b50e3e28b9f7b8fc5a718f6b3748ac562253c720ba1f8271b240c13c536504cd'
CONSUMERS = N / 'gen55_runtime/compute_gateway_deployment_r1/consumers.log'
GATEWAY = E / 'ros_log/full_validated_gen54_restart_20261020T012700Z/core_ros/12936474-cc26-11f1-8561-6b34442c9236/motion_gateway-7.log'
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


def require(value, message):
    if not value:
        raise RuntimeError(message)


def sha(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            h.update(block)
    return h.hexdigest()


def read(path):
    return json.loads(Path(path).read_bytes())


def write(output, name, value):
    with (output / name).open('x') as stream:
        json.dump(value, stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.write('\n')


def checked_file(path, references, expected=None):
    path = Path(path).resolve(strict=True)
    h = sha(path)
    require(expected is None or h == expected, 'source digest changed: ' + str(path))
    references[str(path)] = h
    return path


def storage(output):
    require(M.is_mount(), 'SanStand is not mounted; no local fallback')
    require(not os.statvfs(M).f_flag & os.ST_RDONLY, 'SanStand is read-only')
    require(V.resolve() == output.parent and output.name.startswith('gen55_actual_resources_analysis_'),
            'output must be a fresh GEN55 analysis directory in external verification')
    require(output.parent.stat().st_dev == M.stat().st_dev, 'output device is not SanStand')
    require(not output.exists(), 'refusing to overwrite an earlier analysis')
    return M.stat().st_dev


def reviewed_base(record, references):
    checked_file(BASE, references, BASE_SHA)
    spec = importlib.util.spec_from_file_location('gen55_pinned_offline_resource_math', BASE)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    # The pinned module imports rosbag only for local file decoding. No ROS node
    # or client is initialized and none of its historical task main() is called.
    module.R = record
    return module


def extract_log(path, output, low, high, local=False):
    raw = path.read_bytes()
    raw = raw[:raw.rfind(b'\n') + 1]
    rows, lines, offset = [], [], 0
    for number, line in enumerate(raw.splitlines(keepends=True), 1):
        match = re.search(rb'\[(\d{10})\.(\d+)\]', line)
        stamp = int(match[1]) * 10**9 + int(match[2].ljust(9, b'0')[:9]) if match else None
        if local:
            match = re.search(rb'(\d{4}-\d\d-\d\d \d\d:\d\d:\d\d,\d+)', line)
            if match:
                dt = datetime.datetime.strptime(match[1].decode(), '%Y-%m-%d %H:%M:%S,%f')
                # This exact accepted logger path uses America/Los_Angeles.
                # This run spans October 18/19 UTC and October 18/19 PDT.
                # A different local date requires a fresh offset review.
                require((dt.year, dt.month) == (2026, 10) and dt.day in (18, 19),
                        'gateway logger date changed; review its UTC offset')
                dt = dt.replace(tzinfo=datetime.timezone(datetime.timedelta(hours=-7)))
                stamp = int(dt.timestamp() * 10**9)
        if stamp is not None and low <= stamp <= high:
            rows.append(dict(stamp_ns=stamp, text=line.decode().rstrip(),
                evidence=dict(path=str(path), original_line=number, offset=offset,
                              bytes=len(line), sha256=hashlib.sha256(line).hexdigest())))
            lines.append(line)
        offset += len(line)
    name = 'gateway_task_excerpt.log' if local else 'consumers_task_excerpt.log'
    with (output / name).open('xb') as stream:
        stream.write(b''.join(lines))
    return rows, dict(path=str(path), prefix_bytes=len(raw),
        prefix_sha256=hashlib.sha256(raw).hexdigest(), captured_ns=time.time_ns(),
        live_log_prefix_not_future_file_hash=True, immutable_excerpt=str(output / name),
        excerpt_sha256=sha(output / name), local_timestamp_offset_hours=-7 if local else None)


def resource(base, label, low, high, samples, hz, **evidence):
    require(isinstance(low, int) and isinstance(high, int) and low <= high,
            'invalid phase boundary: ' + label)
    result = base.resource_window(label, low, high, samples, hz)
    result.update(evidence)
    result['no_complete_sample_interval'] = result['intervals'] == 0
    result['coverage_fraction'] = (result['measured_duration_sec'] / result['nominal_duration_sec']
                                   if result['nominal_duration_sec'] else None)
    result['critical_roles_not_observed'] = sorted(set(ROLES) - {p['name'] for p in result['processes']})
    # A newly seen process has RSS evidence but no CPU delta. Preserve unknown
    # instead of reporting zero CPU for that process.
    for row in result['processes']:
        row['cpu_delta_measured'] = row['intervals'] > 0
        if not row['intervals']:
            for key in ('cpu_sec', 'equivalent_cores', 'cpu_percent_one_core_100'):
                row[key] = None
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--record', type=Path, required=True)
    parser.add_argument('--samples', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--release', type=Path, required=True)
    parser.add_argument('--release-sha256', required=True)
    args = parser.parse_args()
    output = args.output.resolve()
    device = storage(output)
    record = args.record.resolve(strict=True)
    sample_dir = args.samples.resolve(strict=True)
    require(record.parent == (E / 'grasp_records').resolve(), 'record is outside SanStand grasp_records')
    require(sample_dir.parent == V.resolve() and sample_dir.name.startswith('gen55_actual_task_resources_'),
            'a sealed GEN55 resource sample is required')
    refs = {}
    checked_file(__file__, refs)
    checked_file(sample_dir / 'result.json', refs)
    sealed = read(sample_dir / 'result.json')
    checked_file(sample_dir / 'metadata.json', refs, sealed['metadata_sha256'])
    checked_file(sample_dir / 'samples.jsonl', refs, sealed['samples_sha256'])
    meta = read(sample_dir / 'metadata.json')
    require(sealed['deployment_unchanged'] is True and sealed['identity_error_count'] == 0,
            'sample deployment or process identity is inconsistent')
    require(meta['process_roles'] == ROLES and set(meta['target_process_identities']) == set(ROLES),
            'expected all eleven critical process roles')
    for key in ('helper_path', 'primitives_path', 'deployment'):
        checked_file(meta[key], refs, meta[key.replace('_path', '') + '_sha256'])
    require(sealed['helper_sha256'] == meta['helper_sha256'], 'sampler helper provenance mismatch')
    deployment = read(meta['deployment'])
    require(deployment.get('deployment_passed') is True and deployment.get('whole_patch_deployed') is True,
            'deployment is not accepted')
    require(Path(meta['deployment']).resolve() == N / 'gen55_runtime/gen55_runtime_deployment.json',
            'sampler was not bound to this GEN55 deployment')
    deployed = dict(deployment.get('after', {}), **deployment.get('preserved_after', {}))
    require(set(deployed) == set(ROLES), 'deployment role set changed')
    for name, identity in meta['target_process_identities'].items():
        require(all(int(deployed[name][key]) == int(identity[key])
                    for key in ('pid', 'start_ticks', 'parent_pid')), 'deployment PID identity mismatch: ' + name)
    samples = [json.loads(line) for line in (sample_dir / 'samples.jsonl').read_text().splitlines()]
    require(len(samples) == sealed['sample_count'] and len(samples) >= 2, 'incomplete sealed sample')
    for sample in samples:
        identities = [(p['pid'], p['start_ticks']) for p in sample['processes']]
        require(len(identities) == len(set(identities)) and not sample['identity_errors'],
                'sample contains duplicate or inconsistent identities')
    require(all(a['recorded_ns'] < b['recorded_ns'] and a['monotonic_ns'] < b['monotonic_ns']
                for a, b in zip(samples, samples[1:])), 'sample clocks are not strictly monotonic')

    checked_file(record / 'manifest.json', refs)
    manifest = read(record / 'manifest.json')
    require(manifest['state'] == 'complete' and manifest['recorder_closed'] is True,
            'record is not sealed and closed')
    require(manifest['record_id'] == record.name, 'record identity does not match its directory')
    required = {'keyframes.bag', 'runner.log', 'command_result.json', 'recorder_outcome.json'}
    seen = set()
    for item in manifest['files']:
        if item['path'] in required:
            path = record / item['path']
            require(path.stat().st_size == item['size_bytes'], 'recorded file size changed: ' + str(path))
            checked_file(path, refs, item['sha256'])
            seen.add(item['path'])
    require(seen == required, 'record lacks required closure and timing evidence')
    checked_file(record / 'authorization.json', refs)
    authorization = read(record / 'authorization.json')
    release_path = checked_file(args.release, refs, args.release_sha256)
    require(release_path == V / 'gen55_release_manifest.json', 'expected this GEN55 release')
    release = read(release_path)
    source_paths = [
        'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/robot/held_pregrasp_selection.py',
        'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/robot/contact_sequence_precheck.py',
        'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/robot/finger_cad_separation.py',
        'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/robot/moveit_planner.py',
        'src/alicia_flexible_grasp_supervisor/scripts/motion_gateway_node.py',
        'src/alicia_flexible_grasp_supervisor/scripts/remote_grasp6d_node.py',
        'src/alicia_flexible_grasp_supervisor/scripts/grasp_task_node.py',
        'src/alicia_grasp_modes/scripts/unknown_tabletop_perception.py',
    ]
    for path in source_paths:
        checked_file(W / path, refs, release['source_sha256'][path])
    base = reviewed_base(record, refs)
    bag = base.read_bag()
    tasks = bag['task_events']
    require(tasks and {x['schema_version'] for x in tasks} == {1}, 'missing or unsupported task-event schema')
    require([x['event_seq'] for x in tasks] == list(range(1, len(tasks) + 1)),
            'task-event sequence is missing or repeated')
    require(len({x['task_run_id'] for x in tasks}) == 1, 'multiple task identities need separate review')
    require(len({(x['selection']['generation'], x['selection']['stamp_ns']) for x in tasks}) == 1,
            'task selection identity changed')
    require(all(a['stamp_ns'] <= b['stamp_ns'] for a, b in zip(tasks, tasks[1:])),
            'task-event source times are not ordered')
    terminal = next((x for x in tasks if x['state'] in ('FAILED', 'SUCCESS')), None)
    released = next((x for x in tasks if not x['active']), None)
    require(terminal is not None or released is not None, 'no terminal or release event; do not infer a task outcome')
    end_event = terminal or released
    start = tasks[0]['stamp_ns']
    end = end_event['stamp_ns']
    selection = authorization['runner_startup_timing']['selection_commit_ros_ns']
    require(isinstance(selection, int) and selection <= start <= end, 'invalid selection/task boundary')
    if terminal and released:
        require(released['stamp_ns'] >= terminal['stamp_ns'], 'release precedes terminal event')
    if terminal and terminal['state'] == 'FAILED':
        require(manifest['result'] != 'success', 'manifest success conflicts with FAILED task; review required')
    output.mkdir()
    require(M.is_mount() and M.stat().st_dev == device and output.stat().st_dev == device,
            'SanStand changed during analysis')
    logs, cref = extract_log(CONSUMERS, output, min(selection, samples[0]['recorded_ns']),
                             max(tasks[-1]['stamp_ns'] + 10**9, samples[-1]['recorded_ns']))
    gateway, gref = extract_log(GATEWAY, output, min(selection, samples[0]['recorded_ns']),
                              max(tasks[-1]['stamp_ns'] + 10**9, samples[-1]['recorded_ns']), True)
    metrics, deliveries, retries, events = [], [], [], []
    for row in logs:
        for marker, target in [('remote 6D pipeline metrics: ', metrics),
                               ('remote 6D contact precheck delivery: ', deliveries),
                               ('remote 6D contact height retry: ', retries)]:
            if marker in row['text']:
                target.append(dict(stamp_ns=row['stamp_ns'], evidence=row['evidence'],
                                   value=json.loads(row['text'].split(marker, 1)[1])))
                break
        else:
            if 'contact_geometry_evidence' not in row['text']:
                events.append(row)
    deliveries = [x for x in deliveries if start <= x['value']['request_wall_ns'] <= end]
    require(len({x['value']['diagnostic_call_id'] for x in deliveries}) == len(deliveries),
            'duplicate strict delivery identity')
    for wrapped in deliveries:
        d = wrapped['value']
        require(not d.get('task_run_id') or d['task_run_id'] == tasks[0]['task_run_id'],
                'strict diagnostic belongs to a different task')

    windows, latency, anchors = [], [], dict(selection_commit=selection, first_task=start, task_end=end,
        sample_start=samples[0]['recorded_ns'], sample_end=samples[-1]['recorded_ns'])
    def window(label, low, high, **evidence):
        windows.append(resource(base, label, low, high, samples, meta['clock_ticks_per_second'], **evidence))
        latency.append(base.latency_window(label, low, high, bag))
    if samples[0]['recorded_ns'] <= selection:
        window('idle_prefix_not_task', samples[0]['recorded_ns'], selection, included_in_task_average=False)
    window('selected_preview', selection, start)
    for left, right in zip(tasks, tasks[1:]):
        if left['stamp_ns'] >= end:
            continue
        window('task_event_%d_%s' % (left['event_seq'], left['state']), left['stamp_ns'],
               min(right['stamp_ns'], end), event_seq=left['event_seq'], state=left['state'],
               message=left['message'], boundary_source='recorded task_event source stamp')
    if end <= samples[-1]['recorded_ns']:
        window('terminal_idle_tail_not_task', end, samples[-1]['recorded_ns'], included_in_task_average=False)
    window('selected_pipeline_excluding_idle', selection, end, aggregate_overlaps_event_windows=True)
    window('active_task', start, end, aggregate_overlaps_event_windows=True)
    near = next((x for x in tasks if x['message'] == 'waiting for near-field 6D preview'), None)
    if near:
        following = next((x for x in tasks if x['event_seq'] > near['event_seq']), end_event)
        near_end = min(following['stamp_ns'], end)
        anchors.update(near_field_start=near['stamp_ns'], near_field_end=near_end)
        window('near_field_wait_for_contact_plan', near['stamp_ns'], near_end,
               aggregate_overlaps_event_windows=True)
        first = next((x['value']['request_wall_ns'] for x in deliveries
                      if near['stamp_ns'] <= x['value']['request_wall_ns'] <= near_end), None)
        if first is not None:
            anchors['first_strict_request'] = first
            window('near_field_before_first_strict', near['stamp_ns'], first,
                   aggregate_overlaps_event_windows=True)
            window('near_field_strict_and_candidate_processing', first, near_end,
                   aggregate_overlaps_event_windows=True)
    # Only a uniquely recorded observation action can be paired by the reviewed
    # legacy bag schema (which has no result goal_id). Multiple actions remain
    # raw evidence; their exact task-event windows above are always retained.
    observation_high = near['stamp_ns'] if near else end
    goals = [x for x in bag['actions'] if x['kind'] == 'controller_goal' and start <= x['stamp_ns'] <= observation_high]
    results = [x for x in bag['actions'] if x['kind'] == 'controller_result' and start <= x['stamp_ns'] <= observation_high]
    action_pairing = dict(goal_count=len(goals), result_count=len(results), paired=False,
                         limitation='legacy timing schema lacks result goal_id; never pair multiple actions by position')
    if len(goals) == len(results) == 1 and goals[0]['stamp_ns'] <= results[0]['stamp_ns']:
        action_pairing['paired'] = True
        window('single_controller_action_before_near_field' if near else 'single_controller_action_during_task',
               goals[0]['stamp_ns'], results[0]['stamp_ns'],
               aggregate_overlaps_event_windows=True, unique_action_pair_in_observation_window=True)
        qualified = [x for x in tasks if x['message'] ==
                     'qualified observation station; checking range before contact planning']
        if near and len(qualified) == 1:
            qualifying = qualified[0]['stamp_ns']
            settling = [x for x in events if 'Grasp motion settled' in x['text'] and
                        results[0]['stamp_ns'] <= x['stamp_ns'] <= qualifying]
            if len(settling) == 1 and qualifying <= near['stamp_ns']:
                anchors.update(controller_goal=goals[0]['stamp_ns'], controller_result=results[0]['stamp_ns'],
                               observation_settled=settling[0]['stamp_ns'], station_qualified=qualifying)
                for label, low_key, high_key in [
                        ('observation_preflight', 'first_task', 'controller_goal'),
                        ('observation_trajectory_execution', 'controller_goal', 'controller_result'),
                        ('arrival_feedback_settle', 'controller_result', 'observation_settled'),
                        ('station_requalification', 'observation_settled', 'station_qualified'),
                        ('near_field_entry', 'station_qualified', 'near_field_start')]:
                    window(label, anchors[low_key], anchors[high_key],
                           aggregate_overlaps_event_windows=True,
                           boundary_source='unique recorded controller action, settled log and task events')

    service_rows, service_windows, gaps = [], [], []
    for index, wrapped in enumerate(deliveries):
        d = wrapped['value']; response = d.get('response'); p = d.get('precheck') or {}
        finish = response['received_wall_ns'] if response else d['recorded_wall_ns']
        mono_finish = response['received_monotonic_ns'] if response else d['recorded_monotonic_ns']
        elapsed = (mono_finish - d['request_monotonic_ns']) / 1e9
        require(elapsed >= 0 and finish >= d['request_wall_ns'], 'negative strict-call duration')
        held = p.get('held_pregrasp_selection') or {}
        feedback = p.get('following_sample_acquisition')
        source = p.get('source_sample') or {}
        service_rows.append(dict(index=index, diagnostic_call_id=d['diagnostic_call_id'],
            plan_id=d['plan_id'], plan_wire_sha256=d.get('plan_wire_sha256'), operation=d['operation'],
            request=d['request'], request_wall_ns=d['request_wall_ns'], response_wall_ns=finish,
            completed_rpc=response is not None, elapsed_sec=elapsed,
            strict_response_success=response.get('success') if response else None,
            message=response.get('message') if response else d.get('consumer_error'),
            failure_code=response.get('failure_code') if response else None,
            source_to_request_sec=(d['request_wall_ns'] - d['source_stamp_ns']) / 1e9,
            held_pregrasp_selection_sec=held.get('elapsed_sec'),
            held_optimizer_iterations=[t.get('iterations') for t in held.get('trials', [])],
            feedback_acquisition_sec=sum(x['elapsed_sec'] for x in feedback) if feedback is not None else None,
            feedback_attempts=[x['attempts'] for x in feedback] if feedback is not None else None,
            feedback_waited=[x['waited'] for x in feedback] if feedback is not None else None,
            accepted_feedback_age_at_acquisition_sec=(source['stamp_sec'] * 1e9 - source['accepted_stamp_ns']) / 1e9
                if 'stamp_sec' in source and 'accepted_stamp_ns' in source else None,
            sdk_feedback_age_at_acquisition_sec=(source['stamp_sec'] * 1e9 - source['sdk_stamp_ns']) / 1e9
                if 'stamp_sec' in source and 'sdk_stamp_ns' in source else None,
            pregrasp_full_path_geometry_sec={k:v['geometry']['elapsed_sec']
                for k,v in held.get('full_path', {}).items() if isinstance(v, dict) and 'geometry' in v},
            response_to_diagnostic_sec=(d['recorded_monotonic_ns'] - mono_finish) / 1e9,
            evidence=wrapped['evidence']))
        service_windows.append(resource(base, 'strict_call_%d' % index, d['request_wall_ns'], finish,
                                       samples, meta['clock_ticks_per_second']))
        if index:
            prior = service_rows[index - 1]['response_wall_ns']
            if prior <= d['request_wall_ns']:
                gaps.append(resource(base, 'between_strict_%d_%d' % (index - 1, index), prior,
                                     d['request_wall_ns'], samples, meta['clock_ticks_per_second']))
            else:
                gaps.append(dict(after=index - 1, before=index, overlap_ns=prior - d['request_wall_ns'],
                                 serial_gap_measured=False))
    completed = [x for x in service_rows if x['completed_rpc']]
    request_keys = {(x['request'].get('generation'), x['request'].get('request_id'),
                     x['request'].get('target_epoch')) for x in service_rows}
    related_metrics = [x for x in metrics if (x['value'].get('generation'), x['value'].get('request_id'),
                                              x['value'].get('target_epoch')) in request_keys]
    request_timings = [dict(request_identity={k:x['value'].get(k) for k in
            ('generation', 'request_id', 'target_epoch', 'snapshot_stamp_sec')},
        status=x['value'].get('status'), diagnostic_stamp_ns=x['stamp_ns'], evidence=x['evidence'],
        end_to_end_ms=x['value'].get('end_to_end_ms'), transport_ms=x['value'].get('transport_ms'),
        remote_total_ms=x['value'].get('wsl_total_ms'),
        preparation_scopes=(x['value'].get('request_preparation_profile') or {}).get('scopes', []),
        nested_scopes_are_not_additive=True) for x in related_metrics]
    summary = dict(completed_strict_rpc_count=len(completed),
        local_attempt_without_service_response_count=len(service_rows) - len(completed),
        completed_strict_rpc_elapsed_sec=base.stats([x['elapsed_sec'] for x in completed]),
        measured_held_selection_sec=base.stats([x['held_pregrasp_selection_sec'] for x in completed]),
        measured_feedback_acquisition_sec=base.stats([x['feedback_acquisition_sec'] for x in completed]),
        related_request_keys=[list(x) for x in sorted(request_keys, key=repr)],
        related_metric_count=len(related_metrics),
        final_cumulative_counters={k:metrics[-1]['value'].get(k) for k in
            ('submitted', 'started', 'completed', 'busy', 'pending_replacements', 'replaced',
             'stale', 'accepted', 'expired', 'failed')} if metrics else {},
        cumulative_counters_are_not_task_deltas=True,
        metric_status_counts_by_phase={x['label']:dict(Counter(m['value']['status'] for m in metrics
            if x['start_ns'] <= m['stamp_ns'] < x['end_ns'])) for x in windows},
        no_gateway_application_lock_wait_or_ROS_callback_queue_timer=True)
    for path, expected in refs.items():
        require(sha(path) == expected, 'an immutable input changed during analysis: ' + path)
    require(M.is_mount() and M.stat().st_dev == device, 'SanStand changed before final write')
    result = dict(kind='GEN55_actual_task_phase_resources_and_latency', schema=1,
        record_id=manifest['record_id'], task_run_id=tasks[0]['task_run_id'],
        task_result=manifest['result'], recorded_terminal_state=terminal['state'] if terminal else None,
        physical_grasp_success=None, physical_success_not_inferred_from_software_or_record_exit=True,
        sample_count=len(samples), sampler_elapsed_sec=sealed['elapsed_sec'],
        observer_CPU_sec_full_sampling_run=sealed['observer_cpu_sec'],
        observer_overhead_nonzero_and_not_attributed_per_phase=True,
        identity_error_count=sealed['identity_error_count'],
        unique_process_identities=len({(p['pid'], p['start_ticks']) for s in samples for p in s['processes']}),
        CPU_count=meta['os_cpu_count'], total_memory_bytes=samples[0]['memory_kib']['MemTotal'] * 1024,
        anchors_ns=anchors, resource_windows=windows, latency_windows=latency,
        action_pairing=action_pairing, queue_and_service_summary=summary,
        related_request_timings=request_timings,
        strict_services=service_rows, strict_resource_windows=service_windows, between_strict_calls=gaps,
        clock_inconsistent_intervals=sum(abs((b['recorded_ns'] - a['recorded_ns']) -
            (b['monotonic_ns'] - a['monotonic_ns'])) > 20_000_000 for a,b in zip(samples, samples[1:])),
        boundary_straddling_intervals=[dict(start_ns=a['recorded_ns'], end_ns=b['recorded_ns'], boundary_ns=z)
            for a,b in zip(samples, samples[1:]) for z in sorted({x['stamp_ns'] for x in tasks} | set(anchors.values()))
            if a['recorded_ns'] < z < b['recorded_ns']],
        input_sha256=refs, live_log_prefix_references=[cref, gref], helper_sha256=sha(__file__),
        limitations=[
            'No whole-sampling-run average is used as task load; idle prefixes and terminal tails are separate.',
            'Every resource delta uses complete interior intervals. Short phases may have no measured CPU interval.',
            'Task-event windows are sequential. Named aggregate, action and strict-call windows overlap and must not be added.',
            'Main-thread wait channels and matched-thread schedstat do not measure application lock or callback queues.',
            'RPC elapsed, nested optimizer time and nested feedback acquisition are distinct evidence, not additive phases.',
            'Unprofiled RPC residual time cannot be labeled lock, settling, transport or I/O wait.',
            'Recorder feedback ages are not control consumer latency; precheck acquisition ages describe that consumer separately.',
            'Image headers mark host read completion, not calibrated exposure. GUI CPU does not establish GUI frame time.',
            'Latest-slot ages and replacement counters are not an unmeasured FIFO depth.',
            'Host counters include unrelated activity and observer overhead; disk counters include all SanStand writers; RSS shares pages.',
            'Current source hashes must match the accepted release; runtime ownership is inherited from the verified deployment sample.',
            'No offline speedup, old candidate result or software success proves new physical grasp success.',
        ], ROS_initialized=False, hardware_contacted=False, temperature_queried=False,
        new_live_sampling=False, production_or_runtime_changed=False, recorded_ns=time.time_ns())
    write(output, 'bag_timing_evidence.json', bag)
    write(output, 'log_event_evidence.json', dict(events=events, gateway=gateway, source_references=[cref, gref]))
    write(output, 'request_profiles.json', metrics)
    write(output, 'contact_deliveries.json', deliveries)
    write(output, 'height_retry_evidence.json', retries)
    result['generated_evidence_sha256'] = {name:sha(output / name) for name in
        ('bag_timing_evidence.json', 'log_event_evidence.json', 'request_profiles.json',
         'contact_deliveries.json', 'height_retry_evidence.json')}
    write(output, 'analysis.json', result)
    print(json.dumps(dict(analysis=str(output / 'analysis.json'), sha256=sha(output / 'analysis.json'),
                         record_id=manifest['record_id'], sample_count=len(samples),
                         completed_strict_rpc_count=len(completed)), ensure_ascii=False))


if __name__ == '__main__':
    main()
