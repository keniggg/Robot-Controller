#!/usr/bin/env python3
"""Offline-only analysis of closed GEN59 sampler and saved task evidence."""
from pathlib import Path
from collections import defaultdict, Counter
import gzip
import hashlib
import json
import math
import statistics
import time

W = Path('/home/zhuyupei/alicia_wa_full/.worktrees/protocol-v3-upgrade')
N = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_gen41_restart_20261018T011222Z')
S = N / 'gen58_resume_r1/observation_pose_fix/actual_attempt_r1'
OUT = S / 'resource_review_r1'
R = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/grasp_records/20261022T044426.800138Z_c2cd1e6e1a8d4226ab4af1eaaa6646ff')

def sha(p):
    return hashlib.sha256(Path(p).read_bytes()).hexdigest()

def read(p):
    return json.loads(Path(p).read_text())

def write(p, obj):
    with p.open('x') as f:
        json.dump(obj, f, indent=2, ensure_ascii=False, allow_nan=False); f.write('\n')

def percentile(values, fraction):
    return sorted(values)[math.ceil(fraction * len(values)) - 1] if values else None

def main():
    assert Path('/media/zhuyupei/SanStand').is_mount()
    OUT.mkdir(exist_ok=False)
    paths = [S / 'resource_samples_r1' / n for n in ('metadata.json', 'samples.jsonl', 'STOP', 'result.json')]
    paths += [S / 'text_terminal_review_r1/result.json', S / 'text_terminal_review_r1/events.jsonl',
              S / 'terminal_process_receipt_r1/result.json', S / 'post_attempt_idle_r1/result.json',
              R / 'result_summary.json', R / 'manifest.json',
              R / 'keyframes_audits/27ac190088e2c5fb1bbacbd1ead32f26f62f7322218682bdc26ca7edc554f822.json']
    pins = {str(p): sha(p) for p in paths}
    meta, closed = read(paths[0]), read(paths[3])
    assert pins[str(paths[0])] == closed['metadata_sha256']
    assert pins[str(paths[1])] == closed['samples_sha256'] == '9c00a3624ac3501292216f542c37576269947f90447090f7d3d1a2a36f83baca'
    assert pins[str(S / 'terminal_process_receipt_r1/result.json')] == 'bcbbbd224ab7b74bb10c88f8f43854c14bf9b968c1f0851dacff3452e51a98e6'
    closure = read(S / 'terminal_process_receipt_r1/result.json')
    assert closure['residual_attempt_replay_sampler_scripts'] == []
    unit = dict(line.split('=', 1) for line in closure['units']['alicia-gen59-actual-resource-r1.service'].splitlines())
    assert unit['MainPID'] == '0' and unit['ExecMainStatus'] == '0' and unit['SubState'] == 'exited'
    for p, h in closure['input_sha256'].items():
        assert sha(p) == h
        pins[p] = h
    assert closed['stop_reason'] == 'STOP_file' and closed['deployment_unchanged'] and closed['identity_error_count'] == 0
    assert sha(meta['helper_path']) == meta['helper_sha256'] and sha(meta['primitives_path']) == meta['primitives_sha256']
    pins.update({meta['helper_path']: meta['helper_sha256'], meta['primitives_path']: meta['primitives_sha256']})
    samples = [json.loads(line) for line in paths[1].read_text().splitlines()]
    assert len(samples) == closed['sample_count'] == 756
    assert all(not s['identity_errors'] for s in samples)
    assert all(b['recorded_ns'] > a['recorded_ns'] and b['monotonic_ns'] > a['monotonic_ns'] for a, b in zip(samples, samples[1:]))
    events = [json.loads(line) for line in (S / 'text_terminal_review_r1/events.jsonl').read_text().splitlines()]
    audit = read(paths[-1]); terminal = read(R / 'result_summary.json')
    assert terminal['state'] == 'complete' and terminal['result'] == 'failure'
    active_start = 1792644293514122000
    active_end = 1792644421730645179
    motion_start = 1792644294418618000
    motion_end = 1792644330504366000
    near_start = audit['observe_return']['reached_view']['start_stamp_ns']
    assert near_start == 1792644332413553237
    assert any(e['logged_stamp_sec'] == '1792644293.514122' and 'capturing initial observation pose' in e['message_prefix'] for e in events)
    assert any(o.get('evidence', {}).get('message', {}).get('stamp_ns') == active_end for o in terminal['result_evidence']['observations'])
    windows = [('pre_task_not_task_load', samples[0]['recorded_ns'], active_start),
               ('task_active_observed', active_start, active_end),
               ('observation_motion_stage_including_setup', motion_start, motion_end),
               ('arrival_qualification', motion_end, near_start),
               ('near_field', near_start, active_end),
               ('post_task_tail', active_end, samples[-1]['recorded_ns'])]
    hz = meta['clock_ticks_per_second']
    process_ids = {name: (p['pid'], p['start_ticks']) for name, p in meta['target_process_identities'].items()}
    for s in samples:
        s['_proc'] = {(p['pid'], p['start_ticks']): p for p in s['processes']}
        assert len(s['_proc']) == len(s['processes'])
        assert all(key in s['_proc'] for key in process_ids.values())
    def window_report(label, start, end):
        inside = [s for s in samples if start <= s['recorded_ns'] <= end]
        pairs = [(a, b) for a, b in zip(samples, samples[1:]) if start <= a['recorded_ns'] and b['recorded_ns'] <= end]
        covered = sum((b['monotonic_ns'] - a['monotonic_ns']) / 1e9 for a, b in pairs)
        rows = defaultdict(lambda: dict(cpu_sec=0., covered_sec=0., io=Counter(), interval_cpu=[], rss=[], threads=[], states=Counter(), main_thread_wchan=Counter(), main_thread_sched_wait_sec=0., main_thread_sched_run_sec=0.))
        busy = idle = iowait = total = 0
        disk = Counter()
        for a, b in pairs:
            dt = (b['monotonic_ns'] - a['monotonic_ns']) / 1e9
            ticks = [y-x for x, y in zip(a['cpu_ticks'][:8], b['cpu_ticks'][:8])]
            assert min(ticks) >= 0
            total += sum(ticks); idle += ticks[3]; iowait += ticks[4]; busy += sum(ticks)-ticks[3]-ticks[4]
            da, db = a['external_filesystem_device'], b['external_filesystem_device']
            assert (da['major'],da['minor']) == (db['major'],db['minor']) == (8,18)
            for k in ('read_bytes','write_bytes','busy_ms','weighted_busy_ms'):
                disk[k] += db['counters'][k] - da['counters'][k]
            for key in a['_proc'].keys() & b['_proc'].keys():
                p, q = a['_proc'][key], b['_proc'][key]
                assert p['attribution'] == q['attribution']
                r = rows[key]; r['name'] = q['name']; r['attribution'] = q['attribution']
                cpu = (q['cpu_ticks'] - p['cpu_ticks']) / hz
                assert cpu >= 0
                r['cpu_sec'] += cpu; r['covered_sec'] += dt; r['interval_cpu'].append(cpu/dt*100)
                for k in ('rchar','wchar','read_bytes','write_bytes','cancelled_write_bytes','syscr','syscw'):
                    if isinstance(p.get('io'),dict) and isinstance(q.get('io'),dict):
                        delta = q['io'].get(k,0)-p['io'].get(k,0)
                        assert delta >= 0
                        r['io'][k] += delta
                x, y = (p.get('schedstat') or '').split(), (q.get('schedstat') or '').split()
                if len(x) >= 2 and len(y) >= 2:
                    r['main_thread_sched_run_sec'] += (int(y[0])-int(x[0]))/1e9
                    r['main_thread_sched_wait_sec'] += (int(y[1])-int(x[1]))/1e9
        for s in inside:
            for key, p in s['_proc'].items():
                r = rows[key];r['name']=p['name'];r['attribution']=p['attribution']
                r['rss'].append(p['rss_bytes']);r['threads'].append(p['threads']);r['states'][p['state']]+=1;r['main_thread_wchan'][p.get('wchan')]+=1
        output = []
        for key, r in rows.items():
            output.append(dict(pid=key[0],start_ticks=key[1],name=r['name'],attribution=r['attribution'],cpu_sec=r['cpu_sec'],
                measured_pair_covered_sec=r['covered_sec'],mean_cpu_percent_one_core_100=100*r['cpu_sec']/r['covered_sec'] if r['covered_sec'] else None,
                peak_interval_cpu_percent_one_core_100=max(r['interval_cpu']) if r['interval_cpu'] else None,
                p95_interval_cpu_percent_one_core_100=percentile(r['interval_cpu'],.95),rss_peak_bytes=max(r['rss']) if r['rss'] else None,
                threads_peak=max(r['threads']) if r['threads'] else None,io_delta=dict(r['io']),states=dict(r['states']),
                main_thread_wchan_samples=dict(r['main_thread_wchan']),main_thread_sched_wait_sec=r['main_thread_sched_wait_sec'],main_thread_sched_run_sec=r['main_thread_sched_run_sec']))
        output.sort(key=lambda r:r['cpu_sec'],reverse=True)
        accepted_cpu=sum(r['cpu_sec'] for r in output if r['attribution']['lineage']=='accepted_deployment')
        all_cpu=sum(r['cpu_sec'] for r in output)
        return dict(label=label,start_ns=start,end_ns=end,exact_window_sec=(end-start)/1e9,sample_count=len(inside),pair_count=len(pairs),
            counter_pair_coverage_sec=covered,first_counter_ns=pairs[0][0]['recorded_ns'] if pairs else None,last_counter_ns=pairs[-1][1]['recorded_ns'] if pairs else None,
            accepted_process_cpu_sec=accepted_cpu,all_tracked_cpu_sec=all_cpu,all_tracked_mean_cpu_percent_one_core_100=100*all_cpu/covered if covered else None,
            accepted_mean_cpu_percent_one_core_100=100*accepted_cpu/covered if covered else None,
            tracked_rss_sum_peak_bytes=max(sum(p['rss_bytes'] for p in s['processes']) for s in inside) if inside else None,
            mem_available_min_kib=min(s['memory_kib']['MemAvailable'] for s in inside) if inside else None,
            swap_free_min_kib=min(s['memory_kib']['SwapFree'] for s in inside) if inside else None,
            host_cpu_busy_percent=100*busy/total if total else None,host_cpu_iowait_percent=100*iowait/total if total else None,
            host_cpu_busy_core_equivalent=busy/hz/covered if covered else None,
            host_loadavg_1m_min=min(float(s['loadavg'].split()[0]) for s in inside) if inside else None,
            host_loadavg_1m_max=max(float(s['loadavg'].split()[0]) for s in inside) if inside else None,
            host_runnable_tasks_peak=max(int(s['loadavg'].split()[3].split('/')[0]) for s in inside) if inside else None,
            external_disk_delta=dict(disk),processes=output)
    reports={label:window_report(label,a,b) for label,a,b in windows}
    deliveries=[]
    for e in events:
        if 'contact precheck delivery' not in e['message_prefix']:
            continue
        p=e['payload']; a,b=p['request_wall_ns'],p['recorded_wall_ns']
        assert near_start <= a < b <= active_end
        deliveries.append(dict(plan_id=p['plan_id'],start_ns=a,end_ns=b,wall_sec=(b-a)/1e9,
                               consumer_outcome=p['consumer_outcome'],consumer_error=p['consumer_error']))
    deliveries.sort(key=lambda r:r['start_ns'])
    assert len(deliveries)==17 and all(a['end_ns']<=b['start_ns'] for a,b in zip(deliveries,deliveries[1:]))
    call_wall=sum(r['wall_sec'] for r in deliveries)
    terminal_screens=[]
    for i in (32,34,36):
        row=audit['lineage'][i];ref=row['contact_sequence_precheck']['contact_failure_replay']['reference']
        path=Path(ref['store_root'])/ref['relative_path'];b=path.read_bytes();raw=gzip.decompress(b)
        assert hashlib.sha256(b).hexdigest()==ref['gzip_sha256'] and hashlib.sha256(raw).hexdigest()==ref['object_sha256']
        obj=json.loads(raw);sample=obj['first_failed_sample'];pins[str(path)]=ref['gzip_sha256']
        assert sample['validation_kind']=='terminal_endpoint_screen' and sample['continuous_validation_executed'] is False
        terminal_screens.append(dict(track_id=row['tracking']['track_id'],plan_id=row['contact_sequence_precheck']['plan_id'],
            receipt_path=str(path),object_sha256=ref['object_sha256'],sample={k:sample[k] for k in ('validation_kind','checked_point_count','continuous_validation_executed','endpoint_screen_only','other_ik_branches_rejected','first_collision_along_curve_claimed','trajectory_point_index','sample_selection_policy')}))
    source=W/'.runtime/gen59_observation_pose_candidate_r3/src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/robot/contact_sequence_precheck.py'
    release=read(N/'verification/gen59_release_manifest.json')
    assert sha(source)==release['source_sha256']['src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/robot/contact_sequence_precheck.py']
    pins[str(source)]=sha(source)
    result=dict(kind='GEN59_frozen_actual_resource_analysis',passed=True,recorded_ns=time.time_ns(),helper_sha256=sha(__file__),input_sha256=pins,
        sampling_closure=dict(stop_reason=closed['stop_reason'],sample_count=756,elapsed_sec=closed['elapsed_sec'],observer_cpu_sec=closed['observer_cpu_sec'],
                              maximum_collection_sec=closed['maximum_collection_sec'],identity_error_count=0,target_process_count=17,tracked_identity_count=21,
                              terminal_process_receipt=str(S/'terminal_process_receipt_r1/result.json'),main_pid_zero_and_exit0_confirmed_by_parent_receipt=True),
        interval_quality=dict(first_sample_ns=samples[0]['recorded_ns'],last_sample_ns=samples[-1]['recorded_ns'],
            min_interval_sec=min((b['monotonic_ns']-a['monotonic_ns'])/1e9 for a,b in zip(samples,samples[1:])),
            max_interval_sec=max((b['monotonic_ns']-a['monotonic_ns'])/1e9 for a,b in zip(samples,samples[1:])),
            wall_minus_monotonic_duration_sec=((samples[-1]['recorded_ns']-samples[0]['recorded_ns'])-(samples[-1]['monotonic_ns']-samples[0]['monotonic_ns']))/1e9),
        task_boundary_method='Start is earliest saved explicit active Grasp state log at1792644293.514122; exact header start unavailable. Another state log is36.057ms later. End is exact inactive GraspState header1792644421730645179. Counter statistics use only adjacent samples entirely inside each interval; boundary partial intervals are excluded, never interpolated.',
        phases=reports,contact_precheck_deliveries=deliveries,contact_precheck_wall_sum_sec=call_wall,
        contact_precheck_wall_fraction_of_near_field=call_wall/((active_end-near_start)/1e9),
        saved_prepare_stage_wall_sec={k:v/1000 for k,v in audit['remote_diagnostics']['ros_prepare_stages_ms'].items()},
        observation_submitted_trajectory_duration_sec=33.048,terminal_collision_terminology=terminal_screens,
        limitations=['CPU100% is one logical core;12CPUs is capacity, not utilization. Host counters include unrelated work.',
            'Observed task start is a saved activity log timestamp, not an invented exact message header.',
            'RSS sums can double-count shared pages; peaks are sampled. CPU/IO totals omit partial boundary intervals and short-lived unseen processes.',
            'rchar/wchar are logical I/O; disk read/write bytes are storage counters; SanStand disk totals include other writers.',
            'Main-thread schedstat/wchan do not measure whole-process waiting, ROS callback queues, camera frame latency, GUI frame time, or remote-server resource usage.',
            '17service delivery wall durations include compute, transport and waits; they are not17CPU durations or17top-level candidates.8candidate checks,9height-window retries,16storedprecheck planIDs plus1deadline refusal delivery.',
            'Geometry-registration andtabletop generation stage walls are saved measurements, not a complete CPU critical-path attribution. WallminusCPU is not automatically I/Owait.',
            'Three terminal screens reject one proposed branch at its endpoint; continuous validation was notexecuted for that screen andotherIK branches were notrejected.',
            'No speedup, whole-path safety, or physical grasp success is claimed.'],no_ROS_calls=True,no_network_calls=True,no_motion=True,no_tests_run=True)
    write(OUT/'result.json',result)
    print(json.dumps(dict(path=str(OUT/'result.json'),sha256=sha(OUT/'result.json'),task_summary={k:v for k,v in reports['task_active_observed'].items() if k!='processes'},near_field_summary={k:v for k,v in reports['near_field'].items() if k!='processes'}),ensure_ascii=False))

if __name__=='__main__':
    main()
