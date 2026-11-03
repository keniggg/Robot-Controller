#!/usr/bin/env python3
"""GEN54 read-only analysis of sealed GEN53 resources and whitelisted bag topics.

No ROS initialization, network/device access, production edits, or new live samples.
"""
import argparse
import collections
import hashlib
import json
import math
import os
from pathlib import Path
import re
import statistics
import time

import rosbag

M = Path('/media/zhuyupei/SanStand')
E = M / '实验记录/Robot-Controller'
W = Path('/home/zhuyupei/alicia_wa_full/.worktrees/protocol-v3-upgrade')
N = E / 'ros_log/full_latest_gen41_restart_20261018T011222Z'
V = N / 'verification'
S = V / 'gen53_actual_task_resources_r1'
R = E / 'grasp_records/20261019T014854.594021Z_26025c111cc64bdf85bf143a55e2f053'
L = N / 'gen53_runtime/compute_gateway_deployment_r1/consumers.log'
EXPECTED = {
    S / 'result.json': 'f81c03da5805a9f060671826202593355a2556ed8b640213b56c43418a10e48d',
    S / 'samples.jsonl': '78bc02582f16f10bee6bd2ec771fdf6709638142952c964fdecb4c64a0535826',
    S / 'metadata.json': 'f31c3a586fc515f0dc1688b302d6c3140bc02d590ca9a179d9ffac1db734b81b',
}
TOPICS = [
    '/grasp/task_event', '/grasp/state', '/grasp/near_field_phase',
    '/perception/unknown/detector_status',
    '/alicia_controller/follow_joint_trajectory/goal',
    '/alicia_controller/follow_joint_trajectory/result',
    '/alicia_d/accepted_joint_states', '/alicia_d/sdk_command',
    '/alicia_controller/state',
]


def sha(data):
    return hashlib.sha256(data).hexdigest()


def file_sha(path):
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def stats(values):
    values = sorted(float(v) for v in values if v is not None and math.isfinite(v))
    if not values:
        return dict(count=0)
    def quantile(p):
        i = (len(values)-1) * p
        lo = int(i)
        return values[lo] + (values[min(lo+1, len(values)-1)]-values[lo])*(i-lo)
    return dict(count=len(values), min=values[0], p50=quantile(.5), p95=quantile(.95),
                max=values[-1], mean=statistics.mean(values), total=sum(values))


def ns(line):
    match = re.search(rb'\[(\d{10})\.(\d+)\]', line)
    return int(match[1])*10**9+int(match[2].ljust(9,b'0')[:9]) if match else None


def psi(value):
    return {line.split()[0]: int(re.search(r'total=(\d+)', line)[1])
            for line in (value or '').splitlines() if 'total=' in line}


def sched(value):
    return [int(v) for v in (value or '0 0 0').split()]


def resource_window(label, low, high, samples, hz):
    pairs = [(a,b) for a,b in zip(samples,samples[1:])
             if low <= a['recorded_ns'] < b['recorded_ns'] <= high
             and abs((b['recorded_ns']-a['recorded_ns'])-
                     (b['monotonic_ns']-a['monotonic_ns'])) <= 20_000_000]
    result = dict(label=label, start_ns=low, end_ns=high,
                  nominal_duration_sec=(high-low)/1e9, intervals=len(pairs))
    if not pairs:
        result.update(measured_duration_sec=0., processes=[])
        return result
    duration = sum((b['monotonic_ns']-a['monotonic_ns'])/1e9 for a,b in pairs)
    processes = {}
    host, disk, pressures = collections.Counter(),collections.Counter(),collections.Counter()
    instantaneous = []
    for a,b in pairs:
        dt = (b['monotonic_ns']-a['monotonic_ns'])/1e9
        old = {(p['pid'],p['start_ticks']):p for p in a['processes']}
        interval_cpu = 0.
        for p in b['processes']:
            identity = (p['pid'],p['start_ticks'])
            q = old.get(identity)
            row = processes.setdefault(identity, dict(pid=p['pid'],start_ticks=p['start_ticks'],
                name=p['name'],attribution=p['attribution'],cpu_ticks=0,rss_peak_bytes=0,
                io=collections.Counter(),intervals=0,thread_peak=0,
                main_thread_sched_runtime_ns=0,main_thread_sched_wait_ns=0,
                main_thread_voluntary_context_switches=0,main_thread_involuntary_context_switches=0,
                main_thread_wait_channels=collections.Counter(),affinity_cpus=p['affinity_cpus']))
            row['rss_peak_bytes']=max(row['rss_peak_bytes'],p['rss_bytes'])
            row['thread_peak']=max(row['thread_peak'],p['threads'])
            row['main_thread_wait_channels'][p.get('wchan') or 'unavailable']+=1
            if q is None:
                continue
            dc=p['cpu_ticks']-q['cpu_ticks']; assert dc>=0
            row['cpu_ticks']+=dc; row['intervals']+=1; interval_cpu+=dc/hz
            for key,value in p['io'].items():
                if key in q['io']:
                    row['io'][key]+=value-q['io'][key]
            aa,bb=sched(q['schedstat']),sched(p['schedstat'])
            row['main_thread_sched_runtime_ns']+=bb[0]-aa[0]
            row['main_thread_sched_wait_ns']+=bb[1]-aa[1]
            for field,out in [('voluntary_ctxt_switches','main_thread_voluntary_context_switches'),
                              ('nonvoluntary_ctxt_switches','main_thread_involuntary_context_switches')]:
                row[out]+=p['status'].get(field,0)-q['status'].get(field,0)
        instantaneous.append(interval_cpu/dt)
        delta=[y-x for x,y in zip(a['cpu_ticks'][:8],b['cpu_ticks'][:8])]
        host.update(total=sum(delta),busy=sum(delta)-delta[3]-delta[4],iowait=delta[4])
        da,db=a['external_filesystem_device'],b['external_filesystem_device']
        assert (da['major'],da['minor'])==(db['major'],db['minor'])
        for key in ['read_bytes','write_bytes','busy_ms','weighted_busy_ms']:
            disk[key]+=db['counters'][key]-da['counters'][key]
        for kind in ['cpu','memory','io']:
            pa,pb=psi(a['pressure'][kind]),psi(b['pressure'][kind])
            for key in set(pa)&set(pb):pressures[kind+'_'+key]+=pb[key]-pa[key]
    for identity,row in processes.items():
        row['cpu_sec']=row['cpu_ticks']/hz
        row['equivalent_cores']=row['cpu_sec']/duration
        row['cpu_percent_one_core_100']=100*row['equivalent_cores']
        row['io']=dict(row['io'])
        row['main_thread_wait_channels']=dict(row['main_thread_wait_channels'])
        row['thread_schedstat']=dict(matched_thread_intervals=0,runtime_ns=0,runqueue_wait_ns=0,
            thread_cpu_ticks=0,covered_process_interval_sec=0.)
        sampled=[(s,p) for s in samples if low<=s['recorded_ns']<=high
                 for p in s['processes'] if (p['pid'],p['start_ticks'])==identity and 'thread_samples' in p]
        for (sa,a),(sb,b) in zip(sampled,sampled[1:]):
            if (sb['monotonic_ns']-sa['monotonic_ns'])/1e9>7.:continue
            old={(t['tid'],t['start_ticks']):t for t in a['thread_samples']}
            matched=False
            for t in b['thread_samples']:
                previous=old.get((t['tid'],t['start_ticks']))
                if previous is None:continue
                aa,bb=sched(previous['schedstat']),sched(t['schedstat'])
                sc=row['thread_schedstat'];sc['matched_thread_intervals']+=1
                sc['runtime_ns']+=bb[0]-aa[0];sc['runqueue_wait_ns']+=bb[1]-aa[1]
                sc['thread_cpu_ticks']+=t['cpu_ticks']-previous['cpu_ticks'];matched=True
            if matched:row['thread_schedstat']['covered_process_interval_sec']+=(sb['monotonic_ns']-sa['monotonic_ns'])/1e9
    selected=[s for a,b in pairs for s in (a,b)]
    result.update(measured_duration_sec=duration,
        first_interval_start_ns=pairs[0][0]['recorded_ns'],last_interval_end_ns=pairs[-1][1]['recorded_ns'],
        tracked_equivalent_cores=sum(r['cpu_sec'] for r in processes.values())/duration,
        tracked_interval_core_stats=stats(instantaneous),
        host_busy_percent=100*host['busy']/host['total'],host_iowait_percent=100*host['iowait']/host['total'],
        external_disk=dict(disk),external_disk_busy_wall_percent=disk['busy_ms']/10/duration,
        psi_stall_microseconds=dict(pressures),
        psi_stall_wall_percent={k:v/1e4/duration for k,v in pressures.items()},
        minimum_available_memory_bytes=min(s['memory_kib']['MemAvailable'] for s in selected)*1024,
        max_swap_used_bytes=max(s['memory_kib']['SwapTotal']-s['memory_kib']['SwapFree'] for s in selected)*1024,
        maximum_dirty_bytes=max(s['memory_kib']['Dirty'] for s in selected)*1024,
        processes=sorted(processes.values(),key=lambda r:-r['cpu_sec']))
    return result


def read_log():
    raw=L.read_bytes();raw=raw[:raw.rfind(b'\n')+1]
    reference=dict(path=str(L),prefix_bytes=len(raw),prefix_sha256=sha(raw),live_prefix_only=True)
    metrics,deliveries,events=[],[],[]
    offset=0
    markers=[b'remote 6D pipeline metrics: ',b'remote 6D contact precheck delivery: ']
    for number,line in enumerate(raw.splitlines(keepends=True),1):
        stamp=ns(line)
        evidence=dict(path=str(L),line=number,offset=offset,bytes=len(line),sha256=sha(line))
        if stamp and 1792374535*10**9<=stamp<=1792374677*10**9:
            if markers[0] in line:
                metrics.append(dict(recorded_ns=stamp,evidence=evidence,value=json.loads(line.split(markers[0],1)[1])))
            elif markers[1] in line:
                x=json.loads(line.split(markers[1],1)[1]);x.update(evidence=evidence,logged_ns=stamp);deliveries.append(x)
            elif any(k in line for k in [b'[Grasp]',b'Grasp motion settled',b'Observation endpoint compensation completed',
                    b'6D planning phase signal:',b'direct contact snapshot waiting',b'OBSERVE_RETURN_SESSION_HANDOFF',
                    b'measurement request:',b'measurement acknowledged:',b'observation first FK strict',
                    b'Accepted rich 6D execution plan',b'remote 6D final candidate rejected']):
                events.append(dict(stamp_ns=stamp,evidence=evidence,text=line.decode().strip()))
        offset+=len(line)
    return reference,metrics,deliveries,events


def read_bag():
    task,states,phases,detectors,feedback,actions=[],[],[],[],[],[]
    with rosbag.Bag(str(R/'keyframes.bag'),'r') as bag:
        for topic,msg,t in bag.read_messages(topics=TOPICS):
            bn=t.to_nsec()
            if topic=='/grasp/task_event':
                x=json.loads(msg.data);x['recorder_receipt_ns']=bn;task.append(x)
            elif topic=='/grasp/state':
                states.append(dict(stamp_ns=msg.header.stamp.to_nsec(),recorder_receipt_ns=bn,
                    state=msg.state,active=msg.active,success=msg.success,message=msg.message))
            elif topic=='/grasp/near_field_phase':
                phases.append(dict(stamp_ns=msg.header.stamp.to_nsec(),recorder_receipt_ns=bn,
                    active=msg.active,phase_id=msg.phase_id,deadline_ns=msg.deadline.to_nsec()))
            elif topic.endswith('/goal'):
                actions.append(dict(kind='controller_goal',stamp_ns=msg.header.stamp.to_nsec(),
                    recorder_receipt_ns=bn,goal_id_stamp_ns=msg.goal_id.stamp.to_nsec(),
                    planned_duration_sec=msg.goal.trajectory.points[-1].time_from_start.to_sec(),
                    point_count=len(msg.goal.trajectory.points)))
            elif topic.endswith('/result'):
                actions.append(dict(kind='controller_result',stamp_ns=msg.header.stamp.to_nsec(),
                    recorder_receipt_ns=bn,status=msg.status.status,error_code=msg.result.error_code))
            elif topic=='/perception/unknown/detector_status':
                x=json.loads(msg.data);f=x.get('frame_timing') or {}
                scalar={k:v for k,v in f.items() if isinstance(v,(float,int)) and not isinstance(v,bool)}
                timings={k:f.get(k,{}) for k in ['stage_wall_sec','stage_thread_cpu_sec','decode_components',
                                               'rgb_components','segmentation_components']}
                appearance=f.get('observation_appearance_identity') or []
                appearance_identities=[sha(json.dumps(a,sort_keys=True,separators=(',',':')).encode()) for a in appearance]
                detector=dict(recorder_receipt_ns=bn,source_stamp_ns=x.get('source_stamp_ns'),
                    generation=x.get('generation'),selection_stamp_ns=x.get('selection_stamp_ns'),
                    state=x.get('state'),detail=x.get('detail'),timing=scalar,
                    timing_components=timings,appearance_evidence_count=len(appearance),
                    appearance_exact_duplicate_count=len(appearance_identities)-len(set(appearance_identities)))
                detectors.append(detector)
            else:
                source=msg.header.stamp.to_nsec()
                feedback.append(dict(topic=topic,source_ns=source,recorder_receipt_ns=bn,
                    recorder_source_age_sec=(bn-source)/1e9))
    return dict(task_events=task,states=states,near_field_phases=phases,actions=actions,
                detectors=detectors,feedback=feedback)


def latency_window(label,low,high,bag):
    detectors=[r for r in bag['detectors'] if low<=r['timing'].get('processing_start_ros_ns',0)
        and r['timing'].get('status_build_ros_ns',r['recorder_receipt_ns'])<=high]
    scalars=collections.defaultdict(list);stages=collections.defaultdict(lambda:collections.defaultdict(list))
    components=collections.defaultdict(lambda:collections.defaultdict(list))
    for r in detectors:
        f=r['timing']
        for k,v in f.items():
            if k.endswith('_sec') and not any(s in k for s in ['monotonic','start_thread','max_frame']):scalars[k].append(v)
        if 'status_build_ros_ns' in f:scalars['status_build_to_recorder_receipt_sec'].append((r['recorder_receipt_ns']-f['status_build_ros_ns'])/1e9)
        for name,v in r['timing_components']['stage_wall_sec'].items():stages[name]['wall_sec'].append(v)
        for name,v in r['timing_components']['stage_thread_cpu_sec'].items():stages[name]['thread_cpu_sec'].append(v)
        for group in ['rgb_components','segmentation_components','decode_components']:
            for name,rows in r['timing_components'][group].items():
                for c in rows if isinstance(rows,list) else [rows]:
                    for key in ['wall_sec','thread_cpu_sec']:
                        if key in c:components[group+'.'+name][key].append(c[key])
    feedback={}
    for topic in sorted(set(r['topic'] for r in bag['feedback'])):
        rows=[r for r in bag['feedback'] if r['topic']==topic and low<=r['recorder_receipt_ns']<=high and r['source_ns']>=low]
        unique=sorted(set(r['source_ns'] for r in rows))
        feedback[topic]=dict(count=len(rows),recorder_source_age_sec=stats([r['recorder_source_age_sec'] for r in rows]),
            unique_source_count=len(unique),source_interarrival_sec=stats([(b-a)/1e9 for a,b in zip(unique,unique[1:])]))
    return dict(label=label,complete_processing_frame_count=len(detectors),
        detector_states=dict(collections.Counter(r['state'] for r in detectors)),
        perception_scalar_seconds={k:stats(v) for k,v in scalars.items()},
        perception_stages={k:{t:stats(v) for t,v in d.items()} for k,d in stages.items()},
        nested_components_not_additive_to_parent={k:{t:stats(v) for t,v in d.items()} for k,d in components.items()},
        duplicate_appearance_evidence_frames=sum(r['appearance_exact_duplicate_count']>0 for r in detectors),
        duplicate_appearance_evidence_records=sum(r['appearance_exact_duplicate_count'] for r in detectors),
        feedback_recorder_observation=feedback,
        caveat='Recorder receipt ages are recorder-side latency, not the motion/control consumer callback. Appearance duplicate records alone do not prove duplicate computation.')


def main():
    began=time.monotonic();cpu_began=time.process_time()
    ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('--output',type=Path,required=True);args=ap.parse_args()
    out=args.output.resolve();assert out.parent==V.resolve() and not out.exists()
    assert M.is_mount() and out.parent.stat().st_dev==M.stat().st_dev and not os.statvfs(M).f_flag&os.ST_RDONLY
    references=[];identities={}
    for path,expected in EXPECTED.items():
        assert file_sha(path)==expected,(str(path),'hash mismatch')
        references.append(dict(path=str(path),sha256=expected,bytes=path.stat().st_size));identities[path]=(path.stat().st_size,path.stat().st_mtime_ns)
    meta=json.loads((S/'metadata.json').read_text());sealed=json.loads((S/'result.json').read_text())
    samples=[json.loads(l) for l in (S/'samples.jsonl').read_text().splitlines()]
    assert len(samples)==250 and sealed['sample_count']==250 and sealed['identity_error_count']==0
    assert sealed['deployment_unchanged'] is True
    for s in samples:
        ids=[(p['pid'],p['start_ticks']) for p in s['processes']];assert len(ids)==len(set(ids)) and not s['identity_errors']
    manifest=json.loads((R/'manifest.json').read_text());assert manifest['recorder_closed'] and manifest['result']=='failure'
    bag_path=R/'keyframes.bag';bref=next(x for x in manifest['files'] if x['path']=='keyframes.bag')
    assert bag_path.stat().st_size==bref['size_bytes'] and file_sha(bag_path)==bref['sha256']
    references.append(dict(path=str(bag_path),sha256=bref['sha256'],bytes=bref['size_bytes'],manifest_verified=True))
    identities[bag_path]=(bag_path.stat().st_size,bag_path.stat().st_mtime_ns)
    for path in [R/'manifest.json',R/'runner.log',V/'gen53_actual_handoff_monitor_r1/final_review_r1.json']:
        references.append(dict(path=str(path),sha256=file_sha(path),bytes=path.stat().st_size))
    reference,metrics,deliveries,events=read_log();references.append(reference)
    bag=read_bag()
    task=bag['task_events'];assert len(task)==11 and {x['task_run_id'] for x in task}=={'55b2a7dfa1c54da5a5284d3182915a48'}
    assert [r['event_seq'] for r in task]==list(range(1,12))
    assert all(x['selection']['generation']==2 for x in task)
    runner=json.loads((R/'runner.log').read_text().splitlines()[0].split(' ',1)[1])
    anchor=dict(sample_start=samples[0]['recorded_ns'],sample_end=samples[-1]['recorded_ns'],
        selection_commit=runner['runner_startup_timing']['selection_commit_ros_ns'],
        first_active_task_event=task[0]['stamp_ns'],
        controller_goal=next(x['stamp_ns'] for x in bag['actions'] if x['kind']=='controller_goal'),
        controller_result=next(x['stamp_ns'] for x in bag['actions'] if x['kind']=='controller_result'),
        observation_settled=next(x['stamp_ns'] for x in events if 'Grasp motion settled' in x['text']),
        near_field_signal=next(x['stamp_ns'] for x in bag['near_field_phases'] if x['active']),
        near_field_wait=next(x['stamp_ns'] for x in task if x['message']=='waiting for near-field 6D preview'),
        first_strict_request=deliveries[0]['request_wall_ns'],
        first_strict_completed=deliveries[0]['response']['received_wall_ns'],
        task_failure=next(x['stamp_ns'] for x in task if x['state']=='FAILED'),
        execution_slot_released=task[-1]['stamp_ns'])
    definitions=[('preselection_idle_not_task','sample_start','selection_commit'),
        ('selected_target_preview','selection_commit','first_active_task_event'),
        ('task_observation_preflight_until_controller_goal','first_active_task_event','controller_goal'),
        ('observation_trajectory_execution','controller_goal','controller_result'),
        ('arrival_settle_feedback_wait','controller_result','observation_settled'),
        ('station_requalification_and_near_field_entry','observation_settled','near_field_wait'),
        ('near_field_before_first_strict_request','near_field_wait','first_strict_request'),
        ('near_field_strict_sequence_and_candidate_processing','first_strict_request','task_failure'),
        ('terminal_tail_not_task','task_failure','sample_end')]
    aggregates=[('selected_pipeline_excluding_idle_and_tail','selection_commit','task_failure'),
        ('active_task_first_event_to_failure','first_active_task_event','task_failure'),
        ('near_field_wait_to_failure','near_field_wait','task_failure')]
    windows=[resource_window(label,anchor[a],anchor[b],samples,meta['clock_ticks_per_second']) for label,a,b in definitions+aggregates]
    latency=[latency_window(label,anchor[a],anchor[b],bag) for label,a,b in definitions+aggregates]
    services=[]
    for d in deliveries:
        p=d['precheck'];response=d['response']
        waits=p.get('following_sample_acquisition') or []
        held=p.get('held_pregrasp_selection') or {}
        services.append(dict(plan_id=d['plan_id'],request=d['request'],failure_code=response['failure_code'],
            request_wall_ns=d['request_wall_ns'],response_wall_ns=response['received_wall_ns'],
            rpc_elapsed_sec=(response['received_monotonic_ns']-d['request_monotonic_ns'])/1e9,
            response_to_diagnostic_sec=(d['recorded_monotonic_ns']-response['received_monotonic_ns'])/1e9,
            source_to_request_age_sec=(d['request_wall_ns']-d['source_stamp_ns'])/1e9,
            source_to_response_age_sec=(response['received_wall_ns']-d['source_stamp_ns'])/1e9,
            feedback_acquisition_elapsed_sec=sum(x.get('elapsed_sec',0.) for x in waits),
            feedback_acquisition_retry_count=sum(x.get('attempts',0)-1 for x in waits),
            held_pregrasp_selection_elapsed_sec=held.get('elapsed_sec'),
            accepted_feedback_age_at_measurement_sec=(p.get('source_sample',{}).get('stamp_sec',0.)*1e9-p.get('source_sample',{}).get('accepted_stamp_ns',0))/1e9,
            sdk_command_age_at_measurement_sec=(p.get('source_sample',{}).get('stamp_sec',0.)*1e9-p.get('source_sample',{}).get('sdk_stamp_ns',0))/1e9,
            evidence=d['evidence']))
    assert len(services)==8 and sum(s['failure_code']=='CONTACT_SEQUENCE_PRECHECK_FAILED' for s in services)==7
    assert all(a['response_wall_ns']<b['request_wall_ns'] for a,b in zip(services,services[1:]))
    latest=metrics[-1]['value'];assert latest['request_id']==20
    queue=dict(metrics_event_status_counts=dict(collections.Counter((r['value']['event']+':'+r['value']['status']) for r in metrics)),
        final_cumulative={k:latest.get(k) for k in ['submitted','started','completed','busy','pending_replacements','replaced','stale','accepted','expired','failed']},
        pending_replacement_residence_sec=stats([r['value']['end_to_end_ms']/1000 for r in metrics if r['value']['status']=='PENDING_REPLACED']),
        strict_calls_serial=True,strict_call_count=8,completed_collision_count=7,
        strict_rpc_elapsed_sec=stats([s['rpc_elapsed_sec'] for s in services]),
        completed_collision_rpc_elapsed_sec=stats([s['rpc_elapsed_sec'] for s in services[:7]]),
        strict_total_feedback_acquisition_sec=sum(s['feedback_acquisition_elapsed_sec'] for s in services),
        strict_total_held_pregrasp_selection_sec=sum(s['held_pregrasp_selection_elapsed_sec'] or 0. for s in services),
        request20_end_to_end_sec=latest['end_to_end_ms']/1000,
        request20_prepare_wall_sec=max(x['elapsed_wall_sec'] for x in latest['request_preparation_profile']['scopes']),
        request20_remote_total_sec=latest['wsl_total_ms']/1000,
        request20_prepare_thread_cpu_sec=max(x['elapsed_thread_cpu_sec'] for x in latest['request_preparation_profile']['scopes']),
        request20_post_prepare_wall_unclassified_sec=latest['end_to_end_ms']/1000-max(x['elapsed_wall_sec'] for x in latest['request_preparation_profile']['scopes']),
        request20_strict_rpc_wall_sec=sum(s['rpc_elapsed_sec'] for s in services),
        no_fifo_depth_or_gui_frame_time_measurement=True)
    source_paths=[W/'tools/record_grasp_keyframes_readonly.py',W/'src/alicia_grasp_modes/scripts/unknown_tabletop_perception.py',
                  W/'src/alicia_grasp_modes/scripts/mode_aware_remote_grasp6d.py',
                  W/'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/vision/request_preparation_profile.py']
    for path in source_paths:references.append(dict(path=str(path),sha256=file_sha(path),bytes=path.stat().st_size,role='current_source_for_semantics_only_not_claimed_deployment_hash'))
    for path,identity in identities.items():assert identity==(path.stat().st_size,path.stat().st_mtime_ns),'sealed input identity changed'
    result=dict(kind='GEN54_sealed_GEN53_actual_phase_resource_and_latency_analysis',created_ns=time.time_ns(),
        record_id=manifest['record_id'],physical_grasp_success=False,task_run_id=task[0]['task_run_id'],
        source_sampler=sealed,sample_count=len(samples),unique_process_identities=len({(p['pid'],p['start_ticks']) for s in samples for p in s['processes']}),
        cpu_capacity=meta['os_cpu_count'],total_memory_bytes=samples[0]['memory_kib']['MemTotal']*1024,
        input_references=references,helper_path=str(Path(__file__).resolve()),helper_sha256=file_sha(Path(__file__)),
        anchors_ns=anchor,resource_windows=windows,latency_windows=latency,queue_and_service_summary=queue,
        boundary_straddling_intervals=[dict(start_ns=a['recorded_ns'],end_ns=b['recorded_ns'],boundary_ns=c)
            for a,b in zip(samples,samples[1:]) for c in sorted(set(anchor.values())) if a['recorded_ns']<c<b['recorded_ns']],
        clock_inconsistent_intervals=sum(abs((b['recorded_ns']-a['recorded_ns'])-(b['monotonic_ns']-a['monotonic_ns']))>20_000_000 for a,b in zip(samples,samples[1:])),
        analysis_elapsed_sec=time.monotonic()-began,analysis_cpu_sec=time.process_time()-cpu_began,
        allowed_bag_topics=TOPICS,ROS_initialized=False,hardware_contacted=False,temperature_read=False,
        commands_or_parameter_changes_sent=False,production_source_modified=False,new_live_sampling=False,
        limitations=[
          'CPU100 is one logical core. Host totals include unrelated work and observer overhead; tracked PIDs are each counted once.',
          'The 22.098982947 observer CPU seconds belong to the full 250.0726-second sampling run and are not attributable per phase.',
          'Only complete approximately 1-second resource intervals within a phase contribute; short phases may have zero such intervals.',
          'Five-second thread schedstat deltas describe scheduler runqueue waiting, not application or ROS callback queues; transient threads can be missed.',
          'RSS includes shared pages. Device counters include all SanStand writers, and logical I/O differs from storage bytes.',
          'First active task event is source-stamped; it is not claimed as the untimestamped GRASP_START_CALL service invocation.',
          'Perception frame timings directly measure consumer ingress/processing. Feedback bag timestamps only measure recorder receipt, not control callback latency.',
          'Camera header is host read completion, not a calibrated exposure timestamp. Keyframe recording is sampled and cannot establish full camera FPS.',
          'Preparation profile base/mode scopes overlap; nested components and process CPU must not be added to parent scopes.',
          'Strict RPC elapsed includes gateway work, waits, transport and scheduling; it is not pure CPU time. Worker CPU is absent from request thread CPU.',
          'Identical appearance evidence records are a review lead, not proof of duplicate expensive computation or cache safety.',
          'GUI CPU is not FPS or input response latency. No speedup or physical contact success is established by this analysis.'])
    outputs=dict(analysis=result,phase_event_evidence=dict(log_events=events,task_events=task,states=bag['states'],near_field_phases=bag['near_field_phases'],actions=bag['actions']),
        request_profiles=metrics,strict_service_timing=services,perception_frame_timing=bag['detectors'])
    out.mkdir()
    for name,value in outputs.items():
        with (out/(name+'.json')).open('x') as stream:json.dump(value,stream,ensure_ascii=False,indent=2,allow_nan=False);stream.write('\n')
    receipt=dict(output=str(out),analysis_sha256=file_sha(out/'analysis.json'),helper_sha256=result['helper_sha256'],
                 verified_input_count=len(references),sample_count=len(samples),elapsed_sec=time.monotonic()-began,cpu_sec=time.process_time()-cpu_began)
    (out/'run_receipt.json').write_text(json.dumps(receipt,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps(receipt,ensure_ascii=False))


if __name__=='__main__':
    main()
