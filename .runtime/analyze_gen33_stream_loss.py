#!/usr/bin/env python3
"""Sealed gen33 evidence only. No ROS node, RPC, subscription, or hardware."""
import bisect
from collections import Counter
import hashlib
import json
from pathlib import Path
import re

import numpy as np
import rosbag

RECORD = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/grasp_records/20261008T123552.595043Z_704863368cdf4c2abd14255ee3daab9e')
SESSION = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/supplement_grasp_20261008T103117Z')
JOINTS = ['Joint%d' % i for i in range(1, 7)]


def sha(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as source:
        for block in iter(lambda: source.read(4 * 1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def stats(values):
    values = [float(v) for v in values if v is not None and np.isfinite(v)]
    return (dict(count=len(values), min=min(values), median=float(np.median(values)),
        p95=float(np.percentile(values, 95)), max=max(values)) if values else dict(count=0))


def main():
    manifest = json.loads((RECORD/'manifest.json').read_text())
    assert manifest['state'] == 'complete' and manifest['recorder_closed'] is True
    integrity = []
    for entry in manifest['files']:
        path = RECORD/entry['path']
        actual = sha(path)
        integrity.append(dict(entry, actual_sha256=actual,
            passed=actual == entry['sha256'] and path.stat().st_size == entry['size_bytes']))
    assert all(item['passed'] for item in integrity)
    topics = ['/perception/unknown/detector_status', '/grasp_record/camera_runtime_metadata',
        '/alicia_d/accepted_joint_states', '/alicia_d/control_reference_epoch',
        '/supervisor/camera/color/image_raw', '/supervisor/camera/depth/image_raw',
        '/grasp/state', '/grasp_6d/plan_enriched', '/grasp_6d/preview_plan_enriched',
        '/alicia_controller/follow_joint_trajectory/status']
    statuses, metadata, joints, images, epochs, plans, states, actions = [], [], [], [], [], [], [], []
    with rosbag.Bag(str(RECORD/'keyframes.bag')) as bag:
        for topic, msg, stamp in bag.read_messages(topics=topics):
            receipt = stamp.to_nsec()
            if topic.endswith('detector_status'):
                row = json.loads(msg.data)
                if row.get('generation') == 33:
                    row['bag_recorded_ns'] = receipt
                    statuses.append(row)
            elif topic.endswith('camera_runtime_metadata'):
                row = json.loads(msg.data)
                metadata.append(dict(bag_recorded_ns=receipt, payload=row))
            elif topic.endswith('accepted_joint_states'):
                values = dict(zip(msg.name, msg.position))
                if msg.header.frame_id == 'sdk_measured' and all(n in values for n in JOINTS):
                    joints.append(dict(source_stamp_ns=msg.header.stamp.to_nsec(), bag_recorded_ns=receipt,
                        q=[values[n] for n in JOINTS]))
            elif topic.endswith('control_reference_epoch'):
                epochs.append(dict(stamp_ns=msg.stamp.to_nsec(), frame_id=msg.frame_id, bag_recorded_ns=receipt))
            elif topic.endswith('image_raw'):
                images.append(dict(stream='color' if '/color/' in topic else 'depth',
                    source_stamp_ns=msg.header.stamp.to_nsec(), sequence=msg.header.seq,
                    bag_recorded_ns=receipt))
            elif topic.endswith('plan_enriched'):
                plans.append(dict(topic=topic, valid=bool(msg.valid), plan_id=msg.plan_id,
                    source_stamp_ns=msg.header.stamp.to_nsec(), bag_recorded_ns=receipt))
            elif topic == '/grasp/state':
                states.append(dict(bag_recorded_ns=receipt,
                    values={k: str(getattr(msg, k)) for k in msg.__slots__}))
            else:
                actions.append(dict(bag_recorded_ns=receipt,
                    statuses=[int(s.status) for s in msg.status_list]))
    first = next(row for row in statuses if row.get('first_loss'))
    loss = first['first_loss']; observed = loss['observed_stamp_ns']; last_good = loss['last_good_source_stamp_ns']
    assert observed == 1791462985330554723
    pre = [r for r in statuses if not r.get('first_loss')]
    ready = [r for r in pre if r.get('state') == 'ready']
    last = next(r for r in reversed(ready) if r['source_stamp_ns'] == last_good)
    joint_unique = {j['source_stamp_ns']: j for j in joints}
    ordered = sorted(joint_unique); selection = loss['selection_stamp_ns']
    def bracket(ns):
        lo, hi = bisect.bisect_right(ordered, ns)-1, bisect.bisect_left(ordered, ns)
        if lo < 0 or hi >= len(ordered): return dict(source_stamp_ns=ns, available=False)
        a,b=joint_unique[ordered[lo]],joint_unique[ordered[hi]]
        span=(b['source_stamp_ns']-a['source_stamp_ns'])*1e-6
        return dict(source_stamp_ns=ns, available=True, before=a, after=b,
            span_ms=span, within_original_200ms=span<=200.,
            maximum_bracket_joint_delta_rad=max(abs(x-y) for x,y in zip(a['q'],b['q'])),
            scope='Recorded accepted-source timestamp bracket, not proof samples reached producer before its bounded wait ended.')
    brackets=[bracket(row['source_stamp_ns']) for row in ready]
    surrounding=[]
    for row in statuses:
        ns=row.get('source_stamp_ns')
        if ((ns is not None and last_good-1_200_000_000 <= ns <= observed+2_000_000_000)
                or row is first):
            surrounding.append({k:row.get(k) for k in ('source_stamp_ns','bag_recorded_ns','state','detail',
                'first_loss','frame_timing','rgbd_ingress_timing','source_qualification','geometry_metrics')})
    camera=[]
    for row in metadata:
        payload=row['payload']; interval=payload.get('parameter_read_interval',{})
        ns=(interval.get('finished') or {}).get('ros_time_ns') or row['bag_recorded_ns']
        if last_good-2_000_000_000 <= ns <= observed+3_000_000_000:
            params=payload.get('parameters') or {}; runtime=params.get('runtime_profile',{})
            camera.append(dict(bag_recorded_ns=row['bag_recorded_ns'], parameter_read_interval=interval,
                image_time_association=payload.get('image_time_association'), fps=params.get('fps'),
                publish_fps=params.get('publish_fps'), runtime={k:runtime.get(k) for k in (
                    'geometry_activation_stamp_ns','sdk_version','publication','stationary_temporal','mode_depth_preset')}))
    image_window=[row for row in images if last_good-1_000_000_000 <= row['source_stamp_ns'] <= observed+3_000_000_000]
    image_gaps={}
    for stream in ('color','depth'):
        rows=sorted([r for r in images if r['stream']==stream],key=lambda r:r['source_stamp_ns'])
        before=max((r for r in rows if r['source_stamp_ns']<=last_good),key=lambda r:r['source_stamp_ns'])
        after=min((r for r in rows if r['source_stamp_ns']>last_good),key=lambda r:r['source_stamp_ns'])
        image_gaps[stream]=dict(before=before, after=after,
            source_gap_ms=(after['source_stamp_ns']-before['source_stamp_ns'])*1e-6,
            recorder_gap_ms=(after['bag_recorded_ns']-before['bag_recorded_ns'])*1e-6,
            missing_source_sequences=after['sequence']-before['sequence']-1)
    ingress=first['rgbd_ingress_timing']['latest_raw_arrivals']
    q=np.asarray([j['q'] for j in joints if selection <= j['source_stamp_ns'] <= observed])
    all_fps=sorted(set((row['payload'].get('parameters') or {}).get('fps') for row in metadata
        if (row['payload'].get('parameters') or {}).get('fps') is not None))
    metrics=[r.get('geometry_metrics',{}) for r in ready]
    summary=dict(pre_first_loss_status_count=len(pre), ready_count=len(ready),
        details=dict(Counter(r.get('detail') for r in pre)),
        ready_source_span_sec=(ready[-1]['source_stamp_ns']-ready[0]['source_stamp_ns'])*1e-9,
        recorded_camera_fps_values=all_fps,
        first_loss_watchdog_has_source_pair=loss['source_pair_available'],
        source_age_at_loss_ms=(observed-last_good)*1e-6,
        subscriber_raw_gap_at_loss_ms={s:dict(first_loss_observed_ns=observed,
            last_raw_arrival_ros_ns=v['arrival_ros_ns'],gap_ms=(observed-v['arrival_ros_ns'])*1e-6)
                                      for s,v in ingress.items()},
        support_current_audit_available_counts=dict(Counter(str(m.get('support_current_audit_available')) for m in metrics)),
        support_ratio=stats(m.get('support_current_audit_inlier_ratio') for m in metrics),
        support_residual_m=stats(m.get('support_current_audit_residual_m') for m in metrics),
        support_pending_count=sum(r.get('source_qualification',{}).get('kind')=='support_measurement_gap'
            for r in pre if isinstance(r.get('source_qualification'),dict)),
        ready_bracket_span_ms=stats(r.get('span_ms') for r in brackets),
        ready_brackets_exceed_200ms=sum(r.get('within_original_200ms') is False for r in brackets),
        ready_processing_wall_ms=stats(r.get('frame_timing',{}).get('processing_wall_sec',0)*1000 for r in ready),
        ready_processing_thread_cpu_ms=stats(r.get('frame_timing',{}).get('processing_thread_cpu_sec',0)*1000 for r in ready),
        accepted_feedback_maximum_joint_span_rad=np.ptp(q,axis=0).tolist(),
        image_gap_across_last_good=image_gaps)
    outcome=json.loads((RECORD/'recorder_outcome.json').read_text())
    result=json.loads((RECORD/'command_result.json').read_text())
    camera_log=SESSION/'verification/full_load_fps15/trial15/leg1_15fps/camera.stderr.log'
    log_bytes=camera_log.read_bytes(); log_events=[]
    for line_number,line in enumerate(log_bytes.decode().splitlines(),1):
        match=re.search(r'\[(\d+\.\d+)\]:',line)
        if match and observed*1e-9-2. <= float(match.group(1)) <= observed*1e-9+2.:
            payload=None
            if '{' in line:
                try: payload=json.loads(line[line.index('{'):])
                except ValueError: pass
            log_events.append(dict(line_number=line_number,ros_log_time_sec=float(match.group(1)),
                raw_line=line,raw_line_sha256=hashlib.sha256(line.encode()).hexdigest(),payload=payload))
    backend=next(e['payload'] for e in log_events if e['payload']
        and e['payload'].get('result')=='complete'
        and e['payload'].get('host_wait',{}).get('elapsed_sec',0)>1.)
    wait=backend['host_wait']
    assert wait['enter']['wall_time_sec'] < observed*1e-9 < wait['exit']['wall_time_sec']
    summary['camera_backend_wait_spans_first_loss']=True
    summary['camera_backend_wait_wall_ms']=wait['elapsed_sec']*1000
    summary['camera_backend_wait_thread_cpu_ms']=wait['thread_cpu_elapsed_sec']*1000
    summary['camera_region_filter_after_wait_wall_ms']=backend['stages_sec']['regional_stationary_depth']*1000
    summary['camera_backend_raw_streams']={stream:dict(
        before=backend['previous_completed_raw_frames']['raw_frames'][stream],
        after=backend['raw_frames'][stream],
        sdk_timestamp_delta_ms=backend['raw_frames'][stream]['device_timestamp_ms']-
            backend['previous_completed_raw_frames']['raw_frames'][stream]['device_timestamp_ms'])
        for stream in ('color','depth')}
    previous=json.loads((SESSION/'gen32_failure_analysis.json').read_text())
    report=dict(schema_version=1,generation=33,record_dir=str(RECORD),session_dir=str(SESSION),
        analysis_scope='Sealed bag/files only. No ROS calls, parameters, actions, production changes, or temperature values.',
        first_loss=loss,first_loss_status_recorded_ns=first['bag_recorded_ns'],
        first_loss_ingress=first['rgbd_ingress_timing'],summary=summary,
        last_ready_frame_timing=last['frame_timing'],last_ready_support=last['geometry_metrics'],
        last_ready_feedback_bracket=bracket(last_good),surrounding_status_rows=surrounding,
        accepted_feedback_near_loss=[j for j in joints if last_good-500_000_000 <= j['source_stamp_ns'] <= observed+2_000_000_000],
        ready_feedback_brackets=brackets,published_image_metadata=image_window,camera_metadata_near_loss=camera,
        execution_evidence=dict(record_result=result['result'],failure_reason=result['failure_reason'],
            recorder_active_seen=outcome['active_seen'],valid_plan_count=sum(r['valid'] for r in plans),
            plans=plans,grasp_states=states,
            controller_active_status_count=sum(any(s in (0,1,6,7) for s in r['statuses']) for r in actions),
            control_reference_epochs=epochs,physical_grasp_success=False),
        record_integrity=dict(manifest_sha256=sha(RECORD/'manifest.json'),all_manifest_files_match=True,files=integrity),
        camera_log_corroboration=dict(path=str(camera_log),captured_prefix_size=len(log_bytes),
            captured_prefix_sha256=hashlib.sha256(log_bytes).hexdigest(),events=log_events,
            source_log_may_remain_open=True),
        prior_gen32_comparison=dict(path=str(SESSION/'gen32_failure_analysis.json'),
            sha256=sha(SESSION/'gen32_failure_analysis.json'),summary=previous['summary'],
            first_loss_reason=previous['first_loss']['reason'],causal_fps_comparison=False),
        script_sha256=sha(__file__),conclusions=[
            'This 15 FPS full-load attempt failed, never published a valid plan, and recorder saw no active execution. No physical grasp success is claimed.',
            '149 ready frames span 29.804 seconds; 148 tracked frames have independent current support with ratios at least 0.72278. No support-measurement hold preceded this loss. This interval is better sustained support than gen32, but different runs do not establish FPS causality.',
            'Every recorded ready-source accepted-feedback bracket is at most 110.621 ms and six arm joint values remain identical. The previous gen32 223.400 ms bracket failure is not reproduced before this gen33 first loss.',
            'The first-loss watchdog has no source pair. Both raw subscriber arrivals stop on source 1791462984514408349; color and depth are already 812.879 and 810.063 ms old at the decision. This is not a color-only late-callback pattern.',
            'Independent recorded images have consecutive source sequences 1657 and 1658 separated by 1328.951 ms. The next pair reaches perception only 3.229 ms (color) and 10.935 ms (depth) after its source stamp, corroborating a whole-pair input gap rather than inference from logging alone.',
            'Camera backend evidence places a 1298.909 ms wait_for_frames interval across the exact first-loss time, consuming only 0.087131 ms of that thread CPU. Both SDK stream counters advance once across a 1355.839 ms SDK timestamp interval. Regional filtering after the wait takes 13.668 ms. The observed boundary is the camera SDK wait/return, before per-frame region filtering and publication; device/USB/host scheduling root cause is not identified.',
            'The last ready perception frame takes 164.720 ms wall / 121.001 ms thread CPU and is published at 182.149 ms source age. It neither overruns the 0.8 s source-age gate nor accounts for the subsequent whole-pair input gap.',
            'Lower camera load does not solve all prior failure modes: this attempt retains good support and feedback brackets but still exceeds the unchanged exact-stream watchdog. No threshold or loss latch was altered.'],limitations=[
            'Bag write/record timestamps are independent recorder observations, not perception raw callback times.',
            'Camera runtime metadata is periodically copied parameter content without exact image association; it cannot reconstruct every missing SDK read.',
            'Recorded feedback can prove source bracket availability in the recording; it does not prove callback/cache timing in perception.',
            'Different generation/time windows do not isolate FPS as the cause of support-quality differences.',
            'No contemporaneous scheduler/GIL/USB trace is present; input-gap cause must not be assigned from wall/CPU difference alone.'])
    output=SESSION/'gen33_failure_analysis.json'
    output.write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps(dict(output=str(output),summary=summary,execution_evidence={k:v for k,v in report['execution_evidence'].items() if k not in ('plans','grasp_states')}),ensure_ascii=False,indent=2))


if __name__ == '__main__': main()
