"""Read a closed historical bag only; save the exact lift inputs for replay."""
from pathlib import Path
import hashlib,json
import rosbag,yaml
import rospy

archive=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/grasp_records/20260928T084603.735476Z_0270fad562cf46a9ae14b8c203da5bc1')
bag=archive/'keyframes.bag'
root=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/grasp_diagnostic_20260928T084245Z')
cut=1790585355.953595
latest={}; goals=[]; windows={}
topics=['/grasp_6d/plan_enriched','/alicia_d/sdk_command','/alicia_d/accepted_joint_states','/alicia_d/control_reference_epoch','/alicia_controller/follow_joint_trajectory/goal']
with rosbag.Bag(str(bag)) as b:
 for topic,m,t in b.read_messages(topics=topics):
  ts=t.to_sec()
  if ts > cut+.001: break
  latest[topic]=(m,ts)
  if 'follow_joint_trajectory/goal' in topic: goals.append((m,ts))
  elif ts >= cut-.8:
   windows.setdefault(topic,[]).append(dict(receipt=ts,message=yaml.safe_load(str(m))))
last=goals[-1]
data=dict(schema_version=1,scope='exact_historical_lift_inputs_no_new_motion',
 source_bag=str(bag),source_goal_id=last[0].goal_id.id,goal_receipt=last[1],
 trajectory=yaml.safe_load(str(last[0].goal.trajectory)),
 plan=yaml.safe_load(str(latest[topics[0]][0])),
 sdk=yaml.safe_load(str(latest[topics[1]][0])),
 accepted=yaml.safe_load(str(latest[topics[2]][0])),
 epoch=yaml.safe_load(str(latest[topics[3]][0])),
 final_stationary_window=windows,
 robot_description=(root/'robot_description.urdf').read_text(),
 parameters=json.loads((root/'execution_parameters.json').read_text()))
out=Path('src/alicia_flexible_grasp_supervisor/tests/fixtures/lift_following_20260928.json')
out.write_text(json.dumps(data,indent=2,ensure_ascii=False)+'\n')
print(str(out),out.stat().st_size)
print('epoch',data['epoch'],'opening',data['accepted']['name'],data['accepted']['position'])
print('sdk',data['sdk']['position'])
print('parameter keys',list(data['parameters']))
