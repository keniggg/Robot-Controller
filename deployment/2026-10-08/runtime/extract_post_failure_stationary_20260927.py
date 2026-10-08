#!/usr/bin/env python3
"""Read real post-failure source timestamps and retain the validated window."""
import json
from pathlib import Path
import rosbag
from rospy import Time
import numpy as np
from alicia_flexible_grasp.robot.stationary_following import SerialUrdfFk,ARM_NAMES,SDK_QUANTUM_RAD as Q,stationary_following_error

HERE=Path(__file__).resolve().parents[1]
FIX=HERE/'src/alicia_flexible_grasp_supervisor/tests/fixtures/contact_no_response_20260927.json'
d=json.loads(FIX.read_text())
xml=json.loads((FIX.parent/d['robot_description_fixture']).read_text())['robot_description']
fk=SerialUrdfFk(xml,ARM_NAMES)
topics=['/alicia_d/sdk_command','/alicia_d/accepted_joint_states']
rows={t:[] for t in topics}
with rosbag.Bag(d['source']['bag_path']) as bag:
    for topic,msg,stamp in bag.read_messages(topics=topics,start_time=Time.from_sec(1790491095.85),end_time=Time.from_sec(1790491097.9)):
        rows[topic].append(msg)
sample=stationary_following_error(fk,rows[topics[0]],rows[topics[1]],now_sec=1790491097.9,epoch_ns=d['epoch_ns'])
assert sample['stationary_window_start_ns']>1790491095840000000
window={}
for key,topic in [('sdk','/alicia_d/sdk_command'),('accepted','/alicia_d/accepted_joint_states')]:
    messages=rows[topic]
    i=max(i for i,m in enumerate(messages) if m.header.stamp.to_nsec()<=sample['stationary_window_start_ns'])
    selected=messages[i:]
    window[key+'_rows']=[{'stamp_ns':m.header.stamp.to_nsec(),'frame_id':m.header.frame_id,
                        'positions_rad':list(m.position)} for m in selected]
    span=np.ptp(np.array([m.position[:6] for m in selected]),axis=0)
    window[key+'_span_rad']=span.tolist()
window['sdk_unchanged']=max(window['sdk_span_rad'])==0.
window['accepted_span_within_one_count']=max(window['accepted_span_rad'])<=Q+1e-12
sample['window_proof']=window
d['post_failure_stationary']=sample
FIX.write_text(json.dumps(d,indent=2)+'\n')
print(json.dumps(sample,indent=2))
