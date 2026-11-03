#!/usr/bin/env python3
"""Read recorded RGBD samples only; no ROS node or hardware access."""
import json
from pathlib import Path
import cv2
import numpy as np
import rosbag
from rospy import Time
from hover_camera_intrinsics_20260927 import provenance

BASE=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
RECORD=BASE/'grasp_records/20260927T071553.378661Z_f557b777f3ee4ad48b4e05890fdaac34'
OUT=BASE/'ros_log/unknown_grasp_20260923/hover_speed_failure_20260927'
if not Path('/media/zhuyupei/SanStand').is_mount():raise SystemExit('SanStand unavailable')
OUT.mkdir(parents=True,exist_ok=True)
topics=['/supervisor/camera/color/image_raw','/supervisor/camera/depth/image_raw','/perception/object_mask','/tf']
result={'camera_intrinsics_evidence':provenance()}
with rosbag.Bag(str(RECORD/'keyframes.bag')) as bag:
    for label,target in [('frozen',1790493363.5590162),('preclose',1790493696.5917058),('closed',1790493699.5)]:
        selected={};transforms=[]
        for topic,msg,t in bag.read_messages(topics=topics,start_time=Time.from_sec(target-.5),end_time=Time.from_sec(target+.5)):
            if topic=='/tf':
                for tr in msg.transforms:
                    transforms.append({'parent':tr.header.frame_id,'child':tr.child_frame_id,'stamp_ns':tr.header.stamp.to_nsec(),
                       'xyz':[tr.transform.translation.x,tr.transform.translation.y,tr.transform.translation.z],
                       'xyzw':[tr.transform.rotation.x,tr.transform.rotation.y,tr.transform.rotation.z,tr.transform.rotation.w]})
                continue
            if topic not in selected or abs(msg.header.stamp.to_sec()-target)<abs(selected[topic].header.stamp.to_sec()-target):selected[topic]=msg
        result[label]={'requested_sec':target,'images':{},'transforms':transforms}
        for topic,msg in selected.items():
            dtype=np.uint16 if msg.encoding in ('16UC1','mono16') else np.uint8
            array=np.frombuffer(msg.data,dtype=dtype).reshape(msg.height,msg.step//np.dtype(dtype).itemsize)
            if msg.encoding in ('rgb8','bgr8'):array=array[:,:msg.width*3].reshape(msg.height,msg.width,3)
            else:array=array[:,:msg.width]
            kind='depth' if 'depth' in topic else ('mask' if 'mask' in topic else 'color')
            if kind=='depth':np.save(str(OUT/(label+'_depth.npy')),array)
            elif kind=='mask':cv2.imwrite(str(OUT/(label+'_mask.png')),array)
            else:cv2.imwrite(str(OUT/(label+'_color.jpg')),cv2.cvtColor(array,cv2.COLOR_RGB2BGR) if msg.encoding=='rgb8' else array)
            result[label]['images'][kind]={'stamp_ns':msg.header.stamp.to_nsec(),'frame_id':msg.header.frame_id,
                'encoding':msg.encoding,'width':msg.width,'height':msg.height,'nonzero_pixels':int(np.count_nonzero(array))}
    static=[]
    for _,msg,t in bag.read_messages(topics=['/tf_static']):
        for tr in msg.transforms:static.append({'parent':tr.header.frame_id,'child':tr.child_frame_id,'stamp_ns':tr.header.stamp.to_nsec(),
          'xyz':[tr.transform.translation.x,tr.transform.translation.y,tr.transform.translation.z],
          'xyzw':[tr.transform.rotation.x,tr.transform.rotation.y,tr.transform.rotation.z,tr.transform.rotation.w]})
    result['static']=static
(OUT/'rgbd_frame_metadata.json').write_text(json.dumps(result,indent=2))
print(json.dumps({k:{a:b for a,b in v.items() if a!='transforms'} for k,v in result.items() if k not in ('static','camera_intrinsics_evidence')},indent=2))
print('static',json.dumps(static,indent=2))
