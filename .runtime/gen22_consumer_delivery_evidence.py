"""Read sealed non-temperature perception topics; no ROS master or hardware."""
from pathlib import Path
import json,hashlib
import rosbag,rospy
EXT=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
assert Path('/media/zhuyupei/SanStand').is_mount()
NEW=EXT/'ros_log/full_latest_restart_20261010T002347Z'
OUT=NEW/'gen22_analysis/consumer_geometry'
station=NEW/'gen22_watch/station_compute_22_1791638725420783774.jsonl'
boundary=NEW/'gen22_watch/motion_boundary_22_1791638728327403179.jsonl'
s=[json.loads(l) for l in station.open()];b=[json.loads(l) for l in boundary.open()]
rows=[]
for r in b:
    if r['kind']!='geometry' or not r.get('reason','').startswith('TARGET_PROCESSING_PENDING: ') or not 1791638795e9<r['received_ros_ns']<1791638833e9:continue
    contract=json.loads(r['reason'].split(': ',1)[1])['station_compute_wait']
    latest=max((v for v in s if v['kind']=='detector' and v.get('source_stamp_ns') and v['received_ros_ns']<r['received_ros_ns']),key=lambda v:v['source_stamp_ns'])
    rows.append(dict(invalidation_ros_ns=r['received_ros_ns'],pinned_source_ns=contract['source_ns'],pinned_age_ms=(r['received_ros_ns']-contract['source_ns'])/1e6,latest_recorded_ready_source_ns=latest['source_stamp_ns'],latest_recorded_ready_age_ms=(r['received_ros_ns']-latest['source_stamp_ns'])/1e6,latest_state=latest['state'],recorded_topics={}))
bag=EXT/'grasp_records/20261010T132545.078718Z_69bac5676950480b80e6cae5cdc4f0ce/keyframes.bag'
sources={r['latest_recorded_ready_source_ns']:r for r in rows}
topics=['/perception/object','/perception/object_mask','/perception/unknown/detector_status']
with rosbag.Bag(str(bag),'r') as stream:
    for topic,msg,t in stream.read_messages(topics=topics,start_time=rospy.Time(1791638796),end_time=rospy.Time(1791638821)):
        if topic.endswith('detector_status'):
            status=json.loads(msg.data);ns=status.get('source_stamp_ns')
            extra=dict(state=status.get('state'),has_registration=status.get('station_compute_wait') is not None,has_recovery_parent=status.get('station_compute_recovery') is not None)
        else:
            ns=msg.header.stamp.to_nsec();extra=dict(detected=bool(msg.detected)) if topic.endswith('/object') else dict(height=msg.height,width=msg.width)
        if ns in sources:sources[ns]['recorded_topics'][topic]=dict(received_ns=t.to_nsec(),**extra)
result=dict(rows=rows,limitations=['Independent watch/bag receive times are not remote callback ingress times.','Six-order deterministic regression proves an admission-order defect; the recording alone cannot quantify remote lock/RPC delay.'],record=str(bag),watch_sha256={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in [station,boundary]})
out=OUT/'recorded_delivery_evidence.json';out.write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
print(json.dumps(rows,ensure_ascii=False))
