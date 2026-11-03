"""Summarize sealed gen13 source-matched support gaps without changing runtime."""
from pathlib import Path
import collections,json,hashlib
import rosbag
E=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
R=E/'grasp_records/20261006T081830.262906Z_86c9846b27134c9f9e22b0ae99a37e1f'
O=E/'ros_log/grasp_supplement_view_20261006T081830Z/pending_diagnostic_v1'
assert Path('/media/zhuyupei/SanStand').is_mount()
manifest=json.loads((R/'manifest.json').read_text());assert manifest['state']=='complete' and manifest['recorder_closed']
statuses=[];objects={};goals=[];counts=collections.Counter();rings=collections.Counter()
with rosbag.Bag(str(R/'keyframes.bag')) as bag:
    topics=bag.get_type_and_topic_info().topics
    goal_topics=[t for t in topics if t.endswith('/goal') and ('trajectory' in t or 'gripper' in t)]
    for topic,m,t in bag.read_messages(topics=['/perception/unknown/detector_status','/perception/object']+goal_topics):
        if topic=='/perception/unknown/detector_status':
            s=json.loads(m.data);s['bag_stamp_ns']=t.to_nsec();statuses.append(s)
        elif topic=='/perception/object':objects[m.header.stamp.to_nsec()]=dict(detected=m.detected,bag_stamp_ns=t.to_nsec())
        else:goals.append(dict(topic=topic,stamp_ns=t.to_nsec()))
active=[s for s in statuses if s['bag_stamp_ns']<=1791274803353483200]
transitions=[];prior=None
for s in active:
    counts[s.get('detail')]+=1
    metrics=s.get('geometry_metrics') or {}
    r=metrics.get('support_current_audit_ring_outer_radius_px')
    if r is not None:rings[str(r)]+=1
    q=s.get('source_qualification') or {}
    if q.get('kind')!='support_measurement_gap':continue
    if s.get('detail')=='tracked' and q.get('state')!='recovered_unique_measured_frame':continue
    transitions.append(dict(source_stamp_ns=s['source_stamp_ns'],bag_stamp_ns=s['bag_stamp_ns'],detail=s['detail'],object=objects.get(s['source_stamp_ns']),target_locked=s.get('target_locked'),target_lost=s.get('target_lost'),anchor_source_stamp_ns=s.get('anchor_source_stamp_ns'),qualification=q))
recoveries=[x for x in transitions if x['qualification'].get('state')=='recovered_unique_measured_frame']
summary=dict(scope='sealed record, runtime interval until runner exit; no new robot commands',record=str(R),manifest_sha256=hashlib.sha256((R/'manifest.json').read_bytes()).hexdigest(),status_count=len(active),details=dict(counts),qualified_ring_radius_counts=dict(rings),support_gap_rows=len(transitions)-len(recoveries),recovery_rows=len(recoveries),recovered_reference_max_mm=max([x['qualification']['support_audit']['support_reference_max_distance_m']*1000 for x in recoveries] or [0]),recovered_rgb_iou_min=min([x['qualification']['rgb_identity']['candidates'][0]['mask_iou'] for x in recoveries] or [0]),goal_messages=goals,transitions=transitions)
O.mkdir(exist_ok=False);(O/'report.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2)+'\n')
print(json.dumps({k:v for k,v in summary.items() if k!='transitions'},ensure_ascii=False))
