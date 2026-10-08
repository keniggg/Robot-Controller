"""Bounded recorded move-to-settle analysis; no robot calls."""
import json,pathlib
import numpy as np
import rospy,rosbag
out=pathlib.Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/resumed_observe_return_grasp_20260930T040740Z')
record=out.parents[1]/'grasp_records/20260930T110135.933325Z_eaa2ff40c32d47419db07d18583cc2b9'
topics=['/alicia_d/sdk_command','/alicia_d/accepted_joint_states','/joint_states','/alicia_controller/follow_joint_trajectory/goal','/alicia_controller/follow_joint_trajectory/result','/perception/unknown/detector_status']
streams={t:[] for t in topics};vision=[];motion={}
with rosbag.Bag(str(record/'keyframes.bag')) as b:
    for topic,m,t in b.read_messages(topics=topics):
        if topic.endswith('detector_status'):
            d=json.loads(m.data)
            if d.get('generation')==24:
                ft=d.get('frame_timing',{});vision.append(dict(bag_sec=t.to_sec(),target_lost=d.get('target_lost'),age_sec=ft.get('status_source_age_sec'),source_ns=ft.get('source_stamp_ns')))
        elif topic.endswith('/goal'):
            points=m.goal.trajectory.points
            motion.update(goal_sec=t.to_sec(),trajectory_duration_sec=points[-1].time_from_start.to_sec(),point_count=len(points))
        elif topic.endswith('/result'):
            motion.update(result_sec=m.header.stamp.to_sec(),action_status=m.status.status,error_code=m.result.error_code)
        elif getattr(m,'position',None):streams[topic].append((m.header.stamp.to_sec(),list(m.position[:6])))
summary={}
end=motion['result_sec']
for topic,items in streams.items():
    if not items:continue
    a=np.array([x[0] for x in items]);q=np.array([x[1] for x in items]);windows={}
    for label,left,right in [('motion',motion['goal_sec'],end),('after_result_1s',end,end+1.),('after_result_4s',end,end+4.),('after_settled',1790766167.872475,end+4.)]:
        idx=(a>=left)&(a<=right);selected=q[idx];ts=a[idx]
        windows[label]=dict(count=int(idx.sum()),joint_ptp_deg=np.degrees(np.ptp(selected,axis=0)).tolist() if len(selected) else [],max_source_gap_ms=float(np.max(np.diff(ts)))*1000 if len(ts)>1 else None,first=selected[0].tolist() if len(selected) else [],last=selected[-1].tolist() if len(selected) else [])
    summary[topic]=windows
active=[v for v in vision if motion['goal_sec']<=v['bag_sec']<=end+1 and v['age_sec'] is not None]
report=dict(record=str(record),motion=motion,streams=summary,vision_motion=dict(count=len(active),target_lost_count=sum(bool(v['target_lost']) for v in active),max_source_age_ms=max(v['age_sec'] for v in active)*1000 if active else None),scope='One successful observation movement followed by geometry bookkeeping failure; no closure. This does not establish that wrist oscillation is absent in all poses.')
(out/'gen24_move_to_settle_analysis.json').write_text(json.dumps(report,indent=2)+'\n')
print('MOTION',motion)
print('VISION',report['vision_motion'])
for topic,windows in summary.items():print(topic,{k:{f:v[f] for f in ['count','joint_ptp_deg','max_source_gap_ms']} for k,v in windows.items() if k!='motion'})
