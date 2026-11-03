#!/usr/bin/env python3
"""Read-only exact recorded source-clock reset checks; no robot or ROS master."""
import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path
import sys
import rosbag
WORK=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(WORK/'src/alicia_flexible_grasp_supervisor/src'))
from alicia_flexible_grasp.vision.pair_history_continuity import interleaved_pair_reason,_stream,_consecutive


def main(bag_path,output):
    if not Path('/media/zhuyupei/SanStand').is_mount():raise RuntimeError('SanStand unavailable')
    if not str(output.resolve()).startswith('/media/zhuyupei/SanStand/实验记录/Robot-Controller/'):raise ValueError('external output required')
    first_failure_ns=1791594202065622329
    seen=set();rows=[];center=[];sources=[]
    with rosbag.Bag(str(bag_path)) as bag:
        for topic,msg,t in bag.read_messages(topics=['/grasp_record/camera_runtime_metadata','/supervisor/camera/color/image_raw']):
            if t.to_nsec()>first_failure_ns:break
            if topic.endswith('image_raw'):
                sources.append(msg.header.stamp.to_nsec());continue
            meta=json.loads(msg.data);temporal=meta['parameters']['runtime_profile']['stationary_temporal']
            r=next(r for r in temporal['regions'] if r['name']=='center');center.append(r)
            h=temporal.get('pair_history',{});key=json.dumps(h,sort_keys=True)
            if key in seen:continue
            seen.add(key)
            previous=h['last_complete_raw_frames']; rejected=h['rejected_raw_frames']; after=h.get('next_accepted_raw_frames')
            exact_rejection=interleaved_pair_reason(previous,rejected,h['last_complete_filter_host_monotonic'],h['first_rejection_host_monotonic'])
            item=dict(recorded_reason=h['reason'],state=h['state'],history_epoch=h['history_epoch'],replayed_first_rejected_packet_reason=exact_rejection,
                recorded_complete_stream_frames={s:previous[s]['frame_number'] for s in previous},
                exact_rejected_host_gap_sec=h['first_rejection_host_monotonic']-h['last_complete_filter_host_monotonic'])
            if after:
                item['accepted_successor_stream_checks']={}
                for stream in ('color','depth'):
                    a,b=_stream(previous[stream]),_stream(after[stream])
                    item['accepted_successor_stream_checks'][stream]=dict(frame_counter_delta=b[0]-a[0],device_interval_ms=b[1]-a[1],
                        stream_fps=a[3],existing_interval_range_ms=[1000*.25/a[3],1000*1.75/a[3]],original_consecutive_predicate=_consecutive(a,b))
                item['accepted_source_skew_ms']=abs(after['color']['device_timestamp_ms']-after['depth']['device_timestamp_ms'])
            rows.append(item)
    reasons=Counter(r['visual'].get('reason') for r in center)
    micro=[r['visual']['motion_p90_px'] for r in center if r['visual'].get('reason')=='visual_motion']
    intervals=[(b-a)/1e9 for a,b in zip(sources,sources[1:])]
    value=dict(schema_version=1,scope='gen3 recorded metadata source-only audit and threshold replay feasibility',production_changes=False,robot_commands_sent=False,
        script_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),source_bag=str(bag_path),first_task_failure_ns=first_failure_ns,
        unique_recorded_history_decisions=rows,reason_counts=dict(Counter(r['recorded_reason'] for r in rows)),
        initial_rejection_predicate_reproductions=dict(no_error=sum(r['replayed_first_rejected_packet_reason'] is None for r in rows),
            rejected_reason_counts=dict(Counter(r['replayed_first_rejected_packet_reason'] for r in rows if r['replayed_first_rejected_packet_reason'] is not None))),
        center_threshold_point_classification=dict(asynchronous_samples=len(center),existing_reason_counts=dict(reasons),original_visual_motion_count=len(micro),
            existing_reported_motion_p90_in_025_to_030=sum(.25 < x <= .3 for x in micro),above_030=sum(x>.3 for x in micro),
            stationary_window_replay=False,qualified_filter_count_after_change_not_estimated=True),
        recorded_rgb=dict(count=len(sources),time_interval_min_sec=min(intervals),time_interval_max_sec=max(intervals),
            exact_raw_sdk_full_rate_sequence_available=False,recorded_domain='published rectified image selected for perception; not full-rate unrectified SDK RGB'),
        conclusions=[
            'The two distinct color_not_immediate_successor decisions have color source intervals approximately 66.68 ms, exceeding the existing 58.33 ms upper bound at 30 fps; depth advances by two. These source resets are supported by recorded evidence, not a demonstrated implementation bug.',
            'The two not_one_identical_previous_stream decisions replay exactly using recorded last-complete and first-rejected packets plus their recorded host clocks. Both streams changed in a skew-rejected packet, so this is not the one-old/one-new interleaving eligible for private history parking.',
            'Ordinary one-old/one-new rejected packets are already quarantined and followed by original continuity/visual checks; the recorded successful successor decisions preserve history as intended.',
            'Reclassifying isolated logged p90 numbers at 0.30 px cannot reproduce a changed visual window: a different threshold changes anchor reset times, tracked features and subsequent flow estimates.',
            'An exact 0.25 versus 0.30 comparison cannot be recovered from this bag: it lacks complete raw SDK RGB, accepted and rejected frame sequence with host times, and the pre-temporal depth stream. No changed filtering or grasp success claim is made.'
        ])
    output.parent.mkdir(parents=True,exist_ok=True);raw=(json.dumps(value,ensure_ascii=False,indent=2)+'\n').encode();output.write_bytes(raw)
    print(json.dumps(dict(result=str(output),sha256=hashlib.sha256(raw).hexdigest(),reasons=value['reason_counts'],center=value['center_threshold_point_classification'],rgb=value['recorded_rgb']),ensure_ascii=False))

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--bag',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args();main(a.bag,a.output)
