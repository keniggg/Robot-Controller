#!/usr/bin/env python3
"""Offline original initial acquisition at recorded gen7 observation endpoint."""
import hashlib
import json
from pathlib import Path
from dataclasses import asdict
import cv2
import numpy as np
import rosbag
import rospy
import tf2_py
from cv_bridge import CvBridge
from tf.transformations import quaternion_matrix
from alicia_grasp_modes import tabletop
from alicia_grasp_modes import rgb_instance
from alicia_grasp_modes.acquisition import InitialAcquisition
from alicia_flexible_grasp.vision.rgbd_rectification import rectification_rgb_valid_mask

SESSION=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_ros_20261009T003407Z')
CAPTURE=SESSION/'gen7_current_arrival_capture'

def main():
    cv2.setNumThreads(1)
    frames={};cameras=[];static=[];dynamic=[];joints=[]
    with rosbag.Bag(str(CAPTURE/'arrival.bag')) as bag:
        for topic,msg,_ in bag.read_messages():
            if topic=='/tf':dynamic.extend(msg.transforms)
            elif topic=='/tf_static':static.extend(msg.transforms)
            elif topic.endswith('camera_parameters'):cameras.append(json.loads(msg.data)['parameters'])
            elif topic.endswith('accepted_joint_states'):
                if msg.header.frame_id=='sdk_measured':joints.append(msg)
            else:frames.setdefault(msg.header.stamp.to_nsec(),{})['rgb' if '/color/' in topic else 'depth']=msg
    camera=cameras[0];k=[camera[n] for n in ('fx','fy','cx','cy')]
    assert all(all(row[n]==camera[n] for n in ('fx','fy','cx','cy','width','height','depth_scale')) for row in cameras)
    buffer=tf2_py.BufferCore(rospy.Duration(600.))
    for t in static:buffer.set_transform_static(t,'sealed_gen7_initial')
    for t in sorted(dynamic,key=lambda t:t.header.stamp.to_nsec()):buffer.set_transform(t,'sealed_gen7_initial')
    bridge=CvBridge();config=tabletop.Config();acquisition=InitialAcquisition(config);rows=[]
    # Skip capture's first source because its TF may precede the first dynamic sample.
    selected=sorted(ns for ns,pair in frames.items() if set(pair)=={'rgb','depth'})[1:4]
    for ns in selected:
        row=dict(source_stamp_ns=ns)
        try:
            pair=frames[ns];rgb=bridge.imgmsg_to_cv2(pair['rgb'],'bgr8')
            depth=np.asarray(bridge.imgmsg_to_cv2(pair['depth'],'passthrough'),dtype=np.float32)*camera['depth_scale']
            transform=buffer.lookup_transform_core('base_link','camera_link',rospy.Time(ns//10**9,ns%10**9))
            tr=transform.transform;q=tr.rotation;T=quaternion_matrix([q.x,q.y,q.z,q.w]);T[:3,3]=[tr.translation.x,tr.translation.y,tr.translation.z]
            result=tabletop.segment_with_support_fallback(depth,k,T,config)
            raw,reason=tabletop.TargetTracker(config).choose(result,depth.shape)
            row.update(candidate_count=len(result.candidates),initial_choice_reason=reason,support_metrics=result.metrics)
            assert raw is not None,reason
            row.update(center_uv=raw.center_uv.tolist(),center_distance_px=float(np.linalg.norm(raw.center_uv-np.array([depth.shape[1]/2,depth.shape[0]/2]))),center_limit_px=min(depth.shape)*config.center_radius_fraction,raw_bbox=list(raw.bbox),raw_quality=float(raw.quality))
            assert raw.quality>=.5,'insufficient_geometry_quality'
            valid=rectification_rgb_valid_mask(camera,depth.shape)
            kwargs={} if valid is None else dict(rgb_valid=valid)
            silhouette=rgb_instance.refine_candidate(rgb,depth,k,T,raw,config,**kwargs)
            confirmed,detail=acquisition.observe(raw,silhouette,ns)
            row.update(passed=True,refined_bbox=list(silhouette.bbox),refined_metrics=silhouette.metrics,initial_confirmation=confirmed,detail=detail)
        except Exception as exc:
            row.update(passed=False,error=type(exc).__name__+': '+str(exc));acquisition.reset()
        rows.append(row)
    output=dict(scope='offline three current-endpoint frames through original initial segmentation/center choice/default RGB16-to32/three-frame acquisition; no existing anchor or arrival retry',capture=str(CAPTURE/'arrival.bag'),config=asdict(config),frames=rows,passed_frames=sum(row['passed'] for row in rows),three_frame_confirmation=bool(rows and rows[-1].get('initial_confirmation')),accepted_joint_peak_to_peak_rad=np.ptp(np.array([msg.position[:6] for msg in joints]),axis=0).tolist(),ros_actions=False,execution_authority=False,limitations=['Uses original pure Config defaults and minimum quality .5; camera intrinsics/depth scale/exact source TF are recorded.','Does not repair cross-view registration or prove live scheduling freshness; user subsequently moved so this old endpoint is diagnostic only.'],source_sha256={str(path):hashlib.sha256(path.read_bytes()).hexdigest() for path in (Path(__file__),Path(tabletop.__file__),Path(rgb_instance.__file__))})
    path=CAPTURE/'initial_acquisition_replay.json';path.write_text(json.dumps(output,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps(dict(path=str(path),passed_frames=output['passed_frames'],three_frame_confirmation=output['three_frame_confirmation'],rows=[{k:r[k] for k in ('source_stamp_ns','passed','center_distance_px','center_limit_px','error','initial_confirmation') if k in r} for r in rows]),ensure_ascii=False))

if __name__=='__main__':main()
