#!/usr/bin/env python3
"""Export sealed two-station RGB-D/TF with explicitly bounded reference roles."""
from pathlib import Path
import json, hashlib
import cv2, numpy as np, rosbag, rospy, tf2_py
from cv_bridge import CvBridge
from tf.transformations import quaternion_matrix
from alicia_grasp_modes.rgb_observation_handoff import _appearance, _mask

EXT=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
NEW=EXT/'ros_log/full_latest_ros_20261009T003407Z'
REC=EXT/'grasp_records/20261009T104542.972841Z_73cf790625184b9e83c9c15be18db8f9'
ROOT=NEW/'gen23_handoff_analysis';OUT=ROOT/'source_images'
summary=json.loads((ROOT/'arrival_and_terminal.json').read_text())
statuses=json.loads((ROOT/'detector_status.json').read_text())
manifest=json.loads((REC/'manifest.json').read_text())
assert manifest['state']=='complete' and manifest['recorder_closed']
anchor_status=next(r['message'] for r in statuses if r['message'].get('detail')=='acquired')
anchor_ns=anchor_status['anchor_source_stamp_ns']
roles={anchor_ns:['anchor']};references=[];windows=[];online={}
for view in summary['views']:
    index=view['view_index'];request=view['arrival_request']
    accepted=next(r for r in view['protocol_acks'] if r['message']['state']=='executing'
        and r['message']['participant']=='unknown' and r['message']['accepted'])
    last=max((r for r in statuses if r['message'].get('state')=='ready'
        and r['received_ns']<=accepted['received_ns']),key=lambda r:r['message']['source_stamp_ns'])
    last_ns=last['message']['source_stamp_ns']
    roles.setdefault(last_ns,[]).append('view%d_last_pre_motion'%index)
    if index==2:
        assert last_ns==summary['first_loss']['message']['first_loss']['last_good_source_stamp_ns']
    references.append(dict(view_index=index,source_stamp_ns=last_ns,status=last,
        executing_unknown_ack=accepted,
        provenance=('last READY before executing unknown ACK; final first_loss.last_good_source independently agrees'
            if index==2 else 'last READY before executing unknown ACK; source captured before executing request; callback receipt may follow request')))
    windows.append((index,request['arrival_stamp_ns'],request['capture_deadline_ns']))
    for r in view['statuses']:
        if r['measurement_state'] in ('requalifying','qualified'):
            online[r['source_stamp_ns']]=(index,r['reason'])
images={};masks={};static=[];dynamic=[];camera=[]
topics=['/tf','/tf_static','/grasp_record/camera_runtime_metadata',
    '/supervisor/camera/color/image_raw','/supervisor/camera/depth/image_raw',
    '/perception/object_mask','/experiment/observation_rgbd/color','/experiment/observation_rgbd/depth']
with rosbag.Bag(str(REC/'keyframes.bag')) as bag:
    for topic,msg,received in bag.read_messages(topics=topics):
        if topic=='/tf':dynamic.extend(msg.transforms)
        elif topic=='/tf_static':static.extend(msg.transforms)
        elif topic.endswith('camera_runtime_metadata'):
            data=json.loads(msg.data)
            if data.get('parameters'):camera.append(data['parameters'])
        else:
            ns=msg.header.stamp.to_nsec()
            if ns not in roles and not any(start<ns<end for _,start,end in windows):continue
            if topic.endswith('object_mask'):masks[ns]=msg
            else:
                key='rgb' if '/color/' in topic or topic.endswith('/color') else 'depth'
                images.setdefault(ns,{})[key]=(msg,topic,received.to_nsec())
cam=camera[0]
assert all(all(c[k]==cam[k] for k in ('fx','fy','cx','cy','width','height','depth_scale')) for c in camera)
buf=tf2_py.BufferCore(rospy.Duration(600.))
for t in static:buf.set_transform_static(t,'sealed_gen23')
for t in sorted(dynamic,key=lambda t:t.header.stamp.to_nsec()):buf.set_transform(t,'sealed_gen23')
bridge=CvBridge();OUT.mkdir(exist_ok=True);rows=[]
for ns,pair in sorted(images.items()):
    if set(pair)!={'rgb','depth'}:continue
    rgb=bridge.imgmsg_to_cv2(pair['rgb'][0],'bgr8')
    raw=bridge.imgmsg_to_cv2(pair['depth'][0],'passthrough')
    transform=buf.lookup_transform_core('base_link','camera_link',rospy.Time(ns//10**9,ns%10**9)).transform
    q=transform.rotation;T=quaternion_matrix([q.x,q.y,q.z,q.w]);T[:3,3]=[transform.translation.x,transform.translation.y,transform.translation.z]
    tags=roles.get(ns,[])+['view%d_arrival'%i for i,start,end in windows if start<ns<end]
    prefix=tags[0]+'_'+str(ns)
    payload=dict(rgb=rgb,depth_raw=raw,T_base_camera=T)
    row=dict(roles=tags,source_stamp_ns=ns,rgb_png=prefix+'_rgb.png',npz=prefix+'.npz',
        online_evaluated=ns in online,online_view_index=online.get(ns,(None,None))[0],online_reason=online.get(ns,(None,None))[1],
        rgb_received_ns=pair['rgb'][2],depth_received_ns=pair['depth'][2],rgb_topic=pair['rgb'][1],depth_topic=pair['depth'][1],
        rgb_frame_id=pair['rgb'][0].header.frame_id,depth_frame_id=pair['depth'][0].header.frame_id,
        rgb_sha256=hashlib.sha256(rgb.tobytes()).hexdigest(),raw_depth_sha256=hashlib.sha256(raw.tobytes()).hexdigest())
    if ns in masks:
        mask=bridge.imgmsg_to_cv2(masks[ns],'mono8');payload['refined_mask']=mask
        hist,contour=_appearance(rgb,_mask(mask,mask.shape))
        payload.update(histogram=hist,contour=contour)
        yy,xx=np.nonzero(mask)
        row.update(mask_pixels=len(xx),mask_bbox_xyxy=[int(xx.min()),int(yy.min()),int(xx.max()+1),int(yy.max()+1)],
            mask_sha256=hashlib.sha256(mask.tobytes()).hexdigest(),mask_png=prefix+'_mask.png')
        cv2.imwrite(str(OUT/row['mask_png']),mask)
        overlay=rgb.copy();cv2.drawContours(overlay,cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)[-2],-1,(0,255,0),1)
        cv2.imwrite(str(OUT/(prefix+'_overlay.png')),overlay)
    cv2.imwrite(str(OUT/row['rgb_png']),rgb);np.savez_compressed(str(OUT/row['npz']),**payload);rows.append(row)
report=dict(record=str(REC),camera=cam,anchor_source_stamp_ns=anchor_ns,anchor_status=anchor_status,
    frozen_references=references,windows=[dict(view_index=i,start_ns=start,deadline_ns=end) for i,start,end in windows],
    online_sources_missing_rgbd=sorted(set(online)-{r['source_stamp_ns'] for r in rows}),files=rows,
    no_segmentation=True,no_ros_calls=True,limitations=['5Hz saved exact RGB-D is not every online evaluation source.',
        'Reference histogram/contour recomputed only from original same-source published mask and RGB; no synthetic pixels.',
        'First station last-reference provenance is bounded by unknown executing ACK; second also independently confirmed by terminal last_good source.'])
(OUT/'manifest.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
print(json.dumps(dict(path=str(OUT/'manifest.json'),references=[dict(view_index=r['view_index'],source_stamp_ns=r['source_stamp_ns']) for r in references],
    arrivals={str(i):[dict(source_stamp_ns=r['source_stamp_ns'],online_evaluated=r['online_evaluated'],rgb_png=str(OUT/r['rgb_png']),npz=str(OUT/r['npz'])) for r in rows if 'view%d_arrival'%i in r['roles']] for i,_,_ in windows}),ensure_ascii=False,indent=2))
