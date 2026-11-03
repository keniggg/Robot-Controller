"""Analyze sealed GEN54 task samples, whitelisted bag timing and bounded log excerpts."""
from pathlib import Path
from collections import Counter
import datetime
import hashlib
import importlib.util
import json
import re
import time

W = Path('/home/zhuyupei/alicia_wa_full/.worktrees/protocol-v3-upgrade')
E = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
N = E/'ros_log/full_latest_gen41_restart_20261018T011222Z'
V = N/'verification'
S = V/'gen54_actual_task_resources_r1'
G = E/'grasp_records/20261019T031844.566032Z_249db38c06c54219b4cc46bfc70fab51'
O = V/'gen54_actual_resources_analysis_r1'
BASE = W/'.runtime/analyze_gen54_gen53_task_resources_20261019.py'
CONSUMERS = N/'gen54_runtime/compute_gateway_deployment_r1/consumers.log'
GATEWAY = E/'ros_log/full_validated_gen52_restart_20261019T013414Z/core_ros/40a36d9c-cb5d-11f1-a61a-77e20f59d0e2/motion_gateway-7.log'


def read(p):
    return json.loads(Path(p).read_bytes())


def digest(data):
    return hashlib.sha256(data).hexdigest()


def sha(p):
    h = hashlib.sha256()
    with Path(p).open('rb') as f:
        for data in iter(lambda: f.read(1024*1024), b''): h.update(data)
    return h.hexdigest()


def save(name, value):
    with (O/name).open('x') as f:
        json.dump(value, f, ensure_ascii=False, indent=2, allow_nan=False); f.write('\n')


def extract_log(path, low, high, local=False):
    raw = path.read_bytes(); raw = raw[:raw.rfind(b'\n')+1]
    rows = []; selected = []; offset = 0
    for number, line in enumerate(raw.splitlines(keepends=True), 1):
        match = re.search(rb'\[(\d{10})\.(\d+)\]', line)
        stamp = int(match[1])*10**9+int(match[2].ljust(9, b'0')[:9]) if match else None
        if local:
            match = re.search(rb'(\d{4}-\d\d-\d\d \d\d:\d\d:\d\d,\d+)', line)
            if match:
                dt = datetime.datetime.strptime(match[1].decode(), '%Y-%m-%d %H:%M:%S,%f')
                dt = dt.replace(tzinfo=datetime.timezone(datetime.timedelta(hours=-7)))
                stamp = int(dt.timestamp()*10**9)
        if stamp and low <= stamp <= high:
            rows.append(dict(stamp_ns=stamp, original_line=number, offset=offset,
                bytes=len(line), sha256=digest(line), text=line.decode().rstrip()))
            selected.append(line)
        offset += len(line)
    name = 'gateway_task_excerpt.log' if local else 'consumers_task_excerpt.log'
    with (O/name).open('xb') as f: f.write(b''.join(selected))
    return rows, dict(path=str(path), prefix_bytes=len(raw), prefix_sha256=digest(raw),
        captured_ns=time.time_ns(), live_log_prefix_not_future_file_hash=True,
        immutable_excerpt=str(O/name), excerpt_sha256=sha(O/name),
        local_timestamp_offset_hours=-7 if local else None)


def main():
    assert Path('/media/zhuyupei/SanStand').is_mount()
    assert O.exists() and not (O/'analysis.json').exists()
    expected = {'result.json':'61c1903effd166a2aac1351e4bd79e5cc778caed16086938d1374c8e21f649b5',
        'samples.jsonl':'f75c8c40a0aeea1068b98bc3e6c673a3c62b0e9159d36ea246db77fc8ecd60a3',
        'metadata.json':'9584d38e87cc23012208990dcacd0a5302e2628b43b0d47789a066fececae731'}
    refs = {str(S/p): h for p, h in expected.items()}
    assert all(sha(p) == h for p, h in refs.items())
    meta = read(S/'metadata.json'); sealed = read(S/'result.json')
    assert sealed['sample_count'] == 187 and sealed['identity_error_count'] == 0 and sealed['deployment_unchanged']
    samples = [json.loads(line) for line in (S/'samples.jsonl').read_text().splitlines()]
    assert len(samples) == 187
    for s in samples:
        ids = [(p['pid'], p['start_ticks']) for p in s['processes']]
        assert len(ids) == len(set(ids)) and not s['identity_errors']
    for key in ('helper_path', 'primitives_path', 'deployment'):
        path = meta[key]; h = meta[key.replace('_path', '')+'_sha256']
        assert sha(path) == h; refs[path] = h
    manifest = read(G/'manifest.json')
    assert manifest['state'] == 'complete' and manifest['result'] == 'failure' and manifest['recorder_closed']
    for item in manifest['files']:
        if item['path'] in ('keyframes.bag', 'runner.log', 'command_result.json', 'recorder_outcome.json'):
            p = G/item['path']; assert p.stat().st_size == item['size_bytes'] and sha(p) == item['sha256']
            refs[str(p)] = item['sha256']
    refs[str(G/'manifest.json')] = sha(G/'manifest.json')
    refs[str(BASE)] = sha(BASE)
    assert sha(O/'bag_timing_evidence.json') == 'da1c56e82c1ec886b2fdecb708c4d59e50863f93c08a7503e7fd31339db21f01'
    refs[str(O/'bag_timing_evidence.json')] = sha(O/'bag_timing_evidence.json')
    spec = importlib.util.spec_from_file_location('gen54_existing_offline_resource_math', BASE)
    base = importlib.util.module_from_spec(spec); spec.loader.exec_module(base)
    bag = read(O/'bag_timing_evidence.json'); tasks = bag['task_events']
    assert [e['event_seq'] for e in tasks] == list(range(1,12))
    assert {e['task_run_id'] for e in tasks} == {'30d7bac61f08402c96cf336d929e66fd'}
    assert all(e['selection']['generation'] == 3 for e in tasks)
    logs, cref = extract_log(CONSUMERS, samples[0]['recorded_ns'], tasks[-1]['stamp_ns']+10**9)
    gateway, gref = extract_log(GATEWAY, samples[0]['recorded_ns'], tasks[-1]['stamp_ns']+10**9, True)
    metrics = []; deliveries = []; events = []; height_retries = []
    for row in logs:
        for marker, target in [('remote 6D pipeline metrics: ', metrics),
                ('remote 6D contact precheck delivery: ', deliveries),
                ('remote 6D contact height retry: ', height_retries)]:
            if marker in row['text']:
                d = json.loads(row['text'].split(marker, 1)[1])
                target.append(dict(stamp_ns=row['stamp_ns'], value=d,
                    evidence={k:v for k,v in row.items() if k != 'text'}))
                break
        else:
            if 'contact_geometry_evidence' not in row['text']:
                events.append(row)
    assert len(deliveries) == 9 and sum(d['value']['response'] is not None for d in deliveries) == 8
    auth = read(G/'authorization.json'); refs[str(G/'authorization.json')] = sha(G/'authorization.json')
    a = dict(sample_start=samples[0]['recorded_ns'], sample_end=samples[-1]['recorded_ns'],
        selection_commit=auth['runner_startup_timing']['selection_commit_ros_ns'],
        first_task=tasks[0]['stamp_ns'],
        controller_goal=next(e['stamp_ns'] for e in bag['actions'] if e['kind']=='controller_goal'),
        controller_result=next(e['stamp_ns'] for e in bag['actions'] if e['kind']=='controller_result'),
        observation_settled=next(e['stamp_ns'] for e in events if 'Grasp motion settled' in e['text']),
        station_qualified=tasks[7]['stamp_ns'], near_field_wait=tasks[8]['stamp_ns'],
        first_strict_request=deliveries[0]['value']['request_wall_ns'],
        task_failure=tasks[-2]['stamp_ns'], released=tasks[-1]['stamp_ns'])
    definitions=[('idle_prefix_not_task','sample_start','selection_commit'),
        ('selected_preview','selection_commit','first_task'),
        ('observation_preflight','first_task','controller_goal'),
        ('observation_trajectory_execution','controller_goal','controller_result'),
        ('arrival_feedback_settle','controller_result','observation_settled'),
        ('station_requalification','observation_settled','station_qualified'),
        ('near_field_entry','station_qualified','near_field_wait'),
        ('near_field_before_first_strict','near_field_wait','first_strict_request'),
        ('near_field_strict_and_candidate_processing','first_strict_request','task_failure'),
        ('terminal_idle_tail_not_task','task_failure','sample_end')]
    aggregates=[('selected_pipeline_excluding_idle','selection_commit','task_failure'),
        ('active_task','first_task','task_failure'), ('near_field','near_field_wait','task_failure')]
    windows = [base.resource_window(label,a[lo],a[hi],samples,meta['clock_ticks_per_second']) for label,lo,hi in definitions+aggregates]
    latency = [base.latency_window(label,a[lo],a[hi],bag) for label,lo,hi in definitions+aggregates]
    service_rows = []; service_resources = []; gap_rows = []
    for i, wrapped in enumerate(deliveries):
        d = wrapped['value']; response = d.get('response'); p = d.get('precheck') or {}
        held = p.get('held_pregrasp_selection') or {}; following = p.get('following_sample_acquisition') or []
        end = response['received_wall_ns'] if response else d['recorded_wall_ns']
        mono_end = response['received_monotonic_ns'] if response else d['recorded_monotonic_ns']
        source = p.get('source_sample') or {}
        service_rows.append(dict(index=i, plan_id=d['plan_id'], operation=d['operation'],
            request=d['request'], request_wall_ns=d['request_wall_ns'], response_wall_ns=end,
            completed_rpc=response is not None, elapsed_sec=(mono_end-d['request_monotonic_ns'])/1e9,
            failure=response['message'] if response else d['consumer_error'],
            source_to_request_sec=(d['request_wall_ns']-d['source_stamp_ns'])/1e9,
            held_pregrasp_selection_sec=held.get('elapsed_sec'),
            held_optimizer_iterations=[t['iterations'] for t in held.get('trials',[])],
            held_optimizer_normalized_fallbacks=[t['normalized_fallback'] for t in held.get('trials',[])],
            feedback_acquisition_sec=sum(x['elapsed_sec'] for x in following),
            feedback_attempts=[x['attempts'] for x in following], feedback_waited=[x['waited'] for x in following],
            accepted_feedback_age_at_acquisition_sec=(source.get('stamp_sec',0)*1e9-source.get('accepted_stamp_ns',0))/1e9 if source else None,
            sdk_feedback_age_at_acquisition_sec=(source.get('stamp_sec',0)*1e9-source.get('sdk_stamp_ns',0))/1e9 if source else None,
            pregrasp_full_path_geometry_sec={k:v['geometry']['elapsed_sec'] for k,v in held.get('full_path',{}).items() if isinstance(v,dict) and 'geometry' in v},
            pregrasp_full_path_collision_state_checks={k:v['collision_state_checks'] for k,v in held.get('full_path',{}).items() if isinstance(v,dict) and 'collision_state_checks' in v},
            response_to_diagnostic_sec=(d['recorded_monotonic_ns']-mono_end)/1e9,
            evidence=wrapped['evidence']))
        service_resources.append(base.resource_window('contact_rpc_%d'%i,d['request_wall_ns'],end,samples,meta['clock_ticks_per_second']))
        if i:
            previous = service_rows[i-1]['response_wall_ns']; assert d['request_wall_ns'] > previous
            gap_rows.append(dict(after_rpc=i-1, before_rpc=i, start_ns=previous,
                end_ns=d['request_wall_ns'], elapsed_sec=(d['request_wall_ns']-previous)/1e9,
                resources=base.resource_window('between_contact_%d_%d'%(i-1,i),previous,d['request_wall_ns'],samples,meta['clock_ticks_per_second'])))
    last = next(x['value'] for x in reversed(metrics) if x['value']['request_id']==17)
    completed = service_rows[:8]
    strict_total = sum(r['elapsed_sec'] for r in completed)
    queue=dict(final_cumulative={k:last.get(k) for k in ['submitted','started','completed','busy','pending_replacements','replaced','stale','accepted','expired','failed']},
        request17_end_to_end_sec=last['end_to_end_ms']/1000,
        request17_prepare_wall_sec=max(x['elapsed_wall_sec'] for x in last['request_preparation_profile']['scopes']),
        request17_prepare_thread_cpu_sec=max(x['elapsed_thread_cpu_sec'] for x in last['request_preparation_profile']['scopes']),
        request17_remote_total_sec=last['wsl_total_ms']/1000,
        request17_remote_transport_roundtrip_sec=last['transport_ms']/1000,
        request17_start_estimate_ns=int((last['snapshot_stamp_sec']+(last['result_age_ms']-last['end_to_end_ms'])/1000)*1e9),
        serial_contact_rpc_count=8, deadline_precondition_refusal_count=1,
        contact_rpc_elapsed_sec=base.stats([r['elapsed_sec'] for r in completed]),
        strict_total_feedback_acquisition_sec=sum(r['feedback_acquisition_sec'] for r in completed),
        strict_total_held_pregrasp_selection_sec=sum(r['held_pregrasp_selection_sec'] for r in completed),
        inter_contact_gap_sec=base.stats([r['elapsed_sec'] for r in gap_rows]),
        contact_rpc_fraction_of_request17=strict_total/(last['end_to_end_ms']/1000),
        metric_status_counts_by_phase={label:dict(Counter(row['value']['status'] for row in metrics if a[lo]<=row['stamp_ns']<=a[hi])) for label,lo,hi in definitions},
        pending_replacement_residence_sec=base.stats([x['value']['end_to_end_ms']/1000 for x in metrics if x['value']['status']=='PENDING_REPLACED']),
        no_ROS_callback_queue_depth_or_gateway_lock_acquisition_timer=True)
    thread_samples = []
    for name in ['/motion_gateway','/remote_grasp6d_node','/unknown_tabletop_perception','/grasp_task_node']:
        row=next(p for window in windows if window['label']=='near_field' for p in window['processes'] if p['name']==name)
        thread_samples.append(dict(name=name, thread_schedstat=row['thread_schedstat'],
            main_thread_wchan_counts=row['main_thread_wait_channels'], main_thread_sched_wait_ns=row['main_thread_sched_wait_ns']))
    sources = ['src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/robot/held_pregrasp_selection.py',
        'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/robot/contact_sequence_precheck.py',
        'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/robot/moveit_planner.py',
        'src/alicia_flexible_grasp_supervisor/scripts/motion_gateway_node.py',
        'src/alicia_flexible_grasp_supervisor/scripts/remote_grasp6d_node.py',
        'src/alicia_grasp_modes/scripts/unknown_tabletop_perception.py']
    release = read(V/'gen54_release_manifest.json')
    for p in sources:
        assert sha(W/p) == release['source_sha256'][p]; refs[str(W/p)] = sha(W/p)
    refs[str(V/'gen54_release_manifest.json')] = sha(V/'gen54_release_manifest.json')
    observations=[
        'Eight completed strict contact RPCs are predominantly compute-active: motion_gateway plus move_group use close to one core over interior 1 Hz intervals; feedback acquisition takes only 7-12 ms and never waits/retries in these calls.',
        'The deliberate planner RLock serializes planning, but this recording has no lock-acquisition duration. CPU evidence and immediate feedback exclude treating all RPC elapsed as fixed settling or receipt wait.',
        'One observation trajectory and its approximately one-second arrival settle occur before near-field entry. They must not be charged once per contact candidate.',
        'Held endpoint selection runs two optimizer starts per completed candidate and records about 18 seconds in total. constraints(x) recomputes FK for the same x[:6] against two different target poses; objective/constraints also repeat after(x) FK.',
        'Near-field input slot wait is short relative to frame processing. Latest-pending ages and replacement counters are bounded-latest-slot observations, not an unmeasured FIFO depth.',
        'Current perception CPU is dominated by tracking then segmentation, unlike the earlier GEN53 RGB-refinement case. Current stage evidence must drive optimization.',
        'The host retains CPU and memory headroom during this task. Camera/control callback freshness, GUI frame time and exact application-lock contention remain separately bounded or unmeasured.']
    recommendations=[dict(priority=1,proposal='Reuse strictly identical q-to-FK transforms within one held endpoint solve; evaluate both target residuals from one transform.',
        evidence='held_pregrasp_selection.py objective/constraints/residual; per-RPC held optimizer elapsed and iterations',
        constraints='Keep every callback deadline check and fresh request snapshot. Keep both original starts/order, solver options, final quantization, true FK and all full-path checks. No cross-candidate success reuse.',measured_speedup=False),
        dict(priority=2,proposal='Profile and reuse immutable geometry work before first strict RPC and between height trials.',evidence='Near-field first-RPC wait 24.759 s; remote process approximately one core before it. Only preparation scopes are individually timed; remainder is not yet attributed.',constraints='Use exact source/model/config/target/height keys and re-run all candidate-specific strict validators. Do not call the shared mutable MoveIt planner concurrently.',measured_speedup=False),
        dict(priority=3,proposal='Consider bounded independent optimizer starts only after exact local reuse is measured.',evidence='Two independent SLSQP starts under frozen FK/targets with spare host capacity.',constraints='Separate immutable inputs and deterministic selection/order; original deadline/cancellation and final owner remain shared. No expanded motion authority.',measured_speedup=False),
        dict(priority=4,proposal='Measure tracking subcomponents and gateway lock wait before widening concurrency or changing queue policy.',evidence='Tracking dominates near-field perception CPU; no gateway lock acquisition or ROS callback FIFO depth measurement.',constraints='Keep camera/control resources and source freshness. Do not raise detection frequency simply to fill cores.',measured_speedup=False)]
    result=dict(kind='GEN54_actual_task_phase_resources_and_latency',record_id=manifest['record_id'],
        task_run_id=tasks[0]['task_run_id'],task_result='failure',physical_grasp_success=False,
        sample_count=187,observer_CPU_sec_full_sampling_run=sealed['observer_cpu_sec'],
        observer_overhead_nonzero_and_not_attributed_per_phase=True,
        sampler_elapsed_sec=sealed['elapsed_sec'],identity_error_count=0,unique_process_identities=len({(p['pid'],p['start_ticks']) for s in samples for p in s['processes']}),
        CPU_count=meta['os_cpu_count'],total_memory_bytes=samples[0]['memory_kib']['MemTotal']*1024,
        anchors_ns=a,resource_windows=windows,latency_windows=latency,queue_and_service_summary=queue,
        strict_services=service_rows,strict_resource_windows=service_resources,between_strict_calls=gap_rows,
        near_field_thread_wait_visibility=thread_samples,
        clock_inconsistent_intervals=sum(abs((b['recorded_ns']-a['recorded_ns'])-(b['monotonic_ns']-a['monotonic_ns']))>20_000_000 for a,b in zip(samples,samples[1:])),
        boundary_straddling_intervals=[dict(start_ns=x['recorded_ns'],end_ns=y['recorded_ns'],boundary_ns=z) for x,y in zip(samples,samples[1:]) for z in sorted(set(a.values())) if x['recorded_ns']<z<y['recorded_ns']],
        input_sha256=refs,live_log_prefix_references=[cref,gref],helper_sha256=sha(__file__),
        observations=observations,proposed_efficiency_improvements=recommendations,
        limitations=['All phase resource deltas use complete interior 1 Hz intervals; short stages can have zero intervals. No fractional boundaries invented.',
          'Host counters include unrelated activity and sampling overhead. Process RSS shares pages; external disk counters include all SanStand writers.',
          'Five-second matched-thread schedstat is scheduler runqueue time, not application-lock or callback queue time. Sleeping main thread does not prove sleeping service threads.',
          'Recorder-side feedback ages do not measure control consumer latency. Precheck sample ages independently measure feedback at that consumer; frame timings measure perception ingress/processing.',
          'Frame header is host read-completion, not calibrated exposure. Recorded timing frames are not a complete camera frame-rate measurement.',
          'Nested preparation scopes overlap. Held pregrasp full-path timing is nested and must not be added again to its enclosing RPC.',
          'Frozen scene age at strict call is not transport queue age. Failures do not establish hardware contact or future following behavior.',
          'No production code, ROS parameters, runtime ownership or hardware changed during this analysis.'],
        ROS_initialized=False,hardware_contacted=False,temperature_queried=False,new_live_sampling=False,recorded_ns=time.time_ns())
    save('log_event_evidence.json',dict(events=events,gateway=gateway,source_references=[cref,gref]))
    save('request_profiles.json',metrics);save('contact_deliveries.json',deliveries);save('height_retry_evidence.json',height_retries)
    save('analysis.json',result)
    print(json.dumps(dict(analysis=str(O/'analysis.json'),sha256=sha(O/'analysis.json'),strict_elapsed_sec=queue['contact_rpc_elapsed_sec'],held_total_sec=queue['strict_total_held_pregrasp_selection_sec'],feedback_total_sec=queue['strict_total_feedback_acquisition_sec']),ensure_ascii=False))


if __name__ == '__main__':
    main()
