from pathlib import Path
import json
from collections import defaultdict
import numpy as np
import rospy,rosbag
R=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/grasp_records/20260930T102353.778847Z_286be61d7a0f4cc8996ebf32219f012b')
O=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/resumed_observe_return_grasp_20260930T040740Z')
loss=1790763935539345502
rows=[]; logs=[]; streams=defaultdict(list)
topics=['/perception/unknown/detector_status','/rosout_agg','/alicia_d/accepted_joint_states','/alicia_d/sdk_command','/alicia_controller/state']
with rosbag.Bag(str(R/'keyframes.bag')) as b:
    counts={k:v.message_count for k,v in b.get_type_and_topic_info().topics.items() if k in topics or k.endswith('/goal')}
    for topic,m,t in b.read_messages(topics=topics,start_time=rospy.Time.from_sec((loss-3_000_000_000)/1e9),end_time=rospy.Time.from_sec((loss+3_000_000_000)/1e9)):
        if topic.endswith('detector_status'):
            d=json.loads(m.data)
            if d.get('generation')==22:rows.append({'bag_ns':t.to_nsec(),'status':d})
        elif topic=='/rosout_agg':
            if m.level>=4 and m.name in ['/supervisor_camera','/unknown_tabletop_perception','/alicia_d_driver_node']:
                logs.append({'bag_ns':t.to_nsec(),'node':m.name,'message':m.msg})
        else:streams[topic].append([m.header.stamp.to_nsec(),t.to_nsec()])
def stats(data):
 return {k:float(v) for k,v in [('median',np.median(data)),('p99',np.quantile(data,.99)),('maximum',max(data))]} if len(data) else {}
summary=[]
for r in rows:
 s=r['status'];t=s.get('frame_timing',{});raw=t.get('raw_arrivals',{})
 summary.append({'bag_relative_ms':(r['bag_ns']-loss)/1e6,'source':s.get('source_stamp_ns'),'detail':s.get('detail'),'failed_stage':t.get('failed_stage'),'ingress_age_ms':1000*t.get('ingress_source_age_sec',0),'start_age_ms':1000*t.get('processing_start_source_age_sec',0),'wait_to_process_ms':1000*t.get('ingress_to_processing_sec',0),'wall_ms':1000*t.get('processing_wall_sec',0),'cpu_ms':1000*t.get('processing_thread_cpu_sec',0),'raw_age_ms':{k:1000*v['source_age_at_arrival_sec'] for k,v in raw.items() if v},'raw_difference_ms':1000*(t.get('raw_arrival_difference_sec') or 0),'after_latest_raw_ms':1000*(t.get('after_latest_raw_arrival_sec') or 0),'input_lock_ms':1000*(t.get('input_lock_wait_sec') or 0),'stage_wall_sec':t.get('stage_wall_sec')})
r={'record':str(R),'loss_ns':loss,'counts':counts,'rows':rows,'logs':logs,'streams':dict(streams),'summary':summary,'stream_gap_ms':{k:{name:stats(np.diff(np.array(v)[:,i])/1e6) for i,name in enumerate(['source','bag'])} for k,v in streams.items()}}
(O/'gen22_first_loss_ingress_evidence.json').write_text(json.dumps(r,ensure_ascii=False,indent=2)+'\n')
print(json.dumps({'counts':counts,'stream_gap_ms':r['stream_gap_ms'],'frames':summary},ensure_ascii=False,indent=2))
