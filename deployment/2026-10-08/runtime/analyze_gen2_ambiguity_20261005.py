#!/usr/bin/env python3
"""Offline sealed-bag ambiguity diagnosis. No ROS master, subscriptions or devices."""
from dataclasses import asdict
import hashlib
import json
from pathlib import Path
import sys

import cv2
import numpy as np
import rosbag
import rospy
import tf2_ros
from cv_bridge import CvBridge
from tf.transformations import quaternion_matrix

WORK = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(WORK/'src/alicia_grasp_modes/src'),
               str(WORK/'src/alicia_flexible_grasp_supervisor/src')]
from alicia_grasp_modes import tabletop
from alicia_grasp_modes.rgb_instance import refine_candidate
from alicia_grasp_modes.rgb_observation_handoff import _appearance
from alicia_flexible_grasp.vision.rgbd_rectification import rectification_rgb_valid_mask

ROOT = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/grasp_records/'
            '20261005T013130.144638Z_f95c4f0498e54f8a922171b982c2df69')


def sha(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for chunk in iter(lambda: stream.read(8*1024*1024), b''):
            h.update(chunk)
    return h.hexdigest()


def stamp(ns):
    return rospy.Time(ns//10**9, ns%10**9)


def matrix(value):
    t,q=value.transform.translation,value.transform.rotation
    result=quaternion_matrix([q.x,q.y,q.z,q.w]);result[:3,3]=[t.x,t.y,t.z]
    return result


def to_candidate(position, extent, footprint):
    return tabletop.Candidate(None,None,None,None,np.array(position),
        np.array(extent),0.,0.,{},np.array(footprint))


def describe(c):
    return dict(bbox=list(c.bbox),center_uv=c.center_uv.tolist(),
        position_base=c.position_base.tolist(),extent_m=c.extent_m.tolist(),
        quality=c.quality,mask_pixels=int(np.count_nonzero(c.mask)),metrics=c.metrics)


def main():
    assert Path('/media/zhuyupei/SanStand').is_mount()
    manifest=json.loads((ROOT/'manifest.json').read_text())
    assert manifest['state']=='complete' and manifest['recorder_closed']
    bag_path=ROOT/'keyframes.bag'
    expected=next(r['sha256'] for r in manifest['files'] if r['path']=='keyframes.bag')
    assert sha(bag_path)==expected
    output=ROOT/'first_ambiguity_analysis';output.mkdir(exist_ok=False)
    cv2.setNumThreads(1)
    statuses=[];metadata=[]
    with rosbag.Bag(str(bag_path)) as bag:
        for topic,msg,arrival in bag.read_messages(topics=[
                '/perception/unknown/detector_status','/grasp_record/camera_runtime_metadata']):
            value=json.loads(msg.data)
            if topic.endswith('detector_status') and value.get('generation')==2:
                statuses.append(value)
            elif topic.endswith('camera_runtime_metadata'):
                metadata.append(dict(arrival_ns=arrival.to_nsec(),value=value))
    first=next(s for s in statuses if s.get('detail')=='ambiguous_locked_instances')
    loss=first['first_loss'];loss_ns=loss['source_stamp_ns'];good_ns=loss['last_good_source_stamp_ns']
    selected=[s for s in statuses if loss_ns-10**9<=s.get('source_stamp_ns',0)<=loss_ns+500000000]
    by_stamp={s['source_stamp_ns']:s for s in selected}
    assert good_ns in by_stamp
    images={};accepted=[]
    tf=tf2_ros.Buffer(cache_time=rospy.Duration(300),debug=False)
    topics=['/tf','/tf_static','/alicia_d/accepted_joint_states',
            '/supervisor/camera/color/image_raw','/supervisor/camera/depth/image_raw',
            '/perception/object_mask','/perception/object']
    bridge=CvBridge()
    with rosbag.Bag(str(bag_path)) as bag:
        for topic,msg,arrival in bag.read_messages(topics=topics):
            if topic in ('/tf','/tf_static'):
                for transform in msg.transforms:
                    if topic=='/tf_static':tf.set_transform_static(transform,'sealed_bag')
                    elif loss_ns-2*10**9<=transform.header.stamp.to_nsec()<=loss_ns+2*10**9:
                        tf.set_transform(transform,'sealed_bag')
                continue
            ns=msg.header.stamp.to_nsec()
            if topic=='/alicia_d/accepted_joint_states':
                if loss_ns-10**9<=ns<=loss_ns+500000000:
                    accepted.append(dict(stamp_ns=ns,names=list(msg.name),positions=list(msg.position)))
                continue
            if ns not in by_stamp:continue
            item=images.setdefault(ns,{})
            if topic.endswith('color/image_raw'):item['rgb']=bridge.imgmsg_to_cv2(msg,'bgr8')
            elif topic.endswith('depth/image_raw'):item['depth']=bridge.imgmsg_to_cv2(msg,'passthrough')
            elif topic.endswith('object_mask'):item['mask']=bridge.imgmsg_to_cv2(msg,'passthrough')
            else:item['object']=dict(detected=msg.detected)
    camera=next(m['value']['parameters'] for m in reversed(metadata)
                if m['arrival_ns']<=good_ns and m['value'].get('status')=='available')
    profile=camera['runtime_profile'];intr=profile['geometry_intrinsics']
    K=np.array([intr[k] for k in ('fx','fy','cx','cy')])
    scale=profile['depth_scale_m_per_unit'];cfg=tabletop.Config()
    anchor=to_candidate(first['anchor_position_base'],first['anchor_extent_m'],
                        first['reference_support_footprint_base'])
    last=to_candidate(loss['candidate_evidence']['last_position_base'],
                      first['anchor_extent_m'],first['reference_support_footprint_base'])
    plane=np.array(first['reference_support_plane_base'])
    prev=images[good_ns];ref_mask=prev['mask']>0;ref=_appearance(prev['rgb'],ref_mask.astype(np.uint8))
    result=dict(schema=1,scope=__doc__,source_bag=str(bag_path),source_bag_sha256=expected,
        manifest_sha256=sha(ROOT/'manifest.json'),script_sha256=sha(__file__),
        first_loss=loss,first_loss_status=first,source_frames=[],
        accepted_feedback=accepted,config=asdict(cfg),camera_metadata=profile,
        limitations=[
            'Config uses unchanged source defaults; no exact /unknown_perception snapshot exists in this bag. Exact recorded candidate/segmentation metrics are independently compared below.',
            'Initial anchor predates this bag; its immutable metrics/plane/footprint are taken from exact online first-loss status, not a fabricated initial RGB frame.',
            'Neighboring-frame segmentation is descriptive: anchor and last-position state are held from first-loss evidence, not a full historical tracker-state replay.',
            'RGB color/shape and containment diagnostics are evidence only, not new target authority or permission to revive the lost generation.',
            'No ROS master, devices or robot commands are accessed.'],
        source_implementation_sha256={str(Path(m.__file__).resolve()):sha(m.__file__)
            for m in (tabletop,sys.modules[refine_candidate.__module__])})
    for ns,status in sorted(by_stamp.items()):
        row=dict(source_stamp_ns=ns,status=status)
        result['source_frames'].append(row)
        if ns not in images or not all(k in images[ns] for k in ('rgb','depth','mask','object')):
            row['missing_exact_pair']=True;continue
        image=images[ns];T=matrix(tf.lookup_transform('base_link','camera_link',stamp(ns)))
        rgb,raw=image['rgb'],image['depth'];depth=raw.astype(np.float32)*scale
        assert rgb.shape[:2]==raw.shape==(intr['height'],intr['width'])
        row['exact_tf']=T.tolist();row['recorded_object']=image['object']
        valid_rgb=rectification_rgb_valid_mask(camera,depth.shape)
        segment=tabletop.segment_with_support_fallback(depth,K,T,cfg,anchor,last,plane)
        row['segmentation_metrics']=segment.metrics;row['candidates']=[]
        arrays=dict(rgb=rgb,depth_raw=raw,published_mask=image['mask'],T_base_camera=T,K=K)
        for i,c in enumerate(segment.candidates):
            masks=c.mask>0;item=describe(c)
            item.update(index=i,anchor_distance_m=float(np.linalg.norm(c.position_base-anchor.position_base)),
                last_distance_m=float(np.linalg.norm(c.position_base-last.position_base)),
                extent_ratios_xy=(c.extent_m[:2]/anchor.extent_m[:2]).tolist(),
                previous_rgb_mask_overlap=int(np.count_nonzero(masks&ref_mask)),
                previous_rgb_mask_iou=float(np.count_nonzero(masks&ref_mask)/np.count_nonzero(masks|ref_mask)))
            arrays[f'candidate_{i}_mask']=c.mask
            try:
                refined=refine_candidate(rgb,depth,K,T,c,cfg,rgb_valid=valid_rgb)
            except ValueError as error:item['refinement_error']=str(error)
            else:
                item['refined']=describe(refined);arrays[f'candidate_{i}_refined']=refined.mask
                hist,contour=_appearance(rgb,refined.mask.astype(np.uint8))
                item['refined']['previous_color_distance']=float(np.sqrt(max(0.,1-np.sum(np.sqrt(hist*ref[0])))))
                item['refined']['previous_contour_distance']=float(cv2.matchShapes(ref[1],contour,cv2.CONTOURS_MATCH_I1,0.))
            row['candidates'].append(item)
        match=tabletop._matching_candidates(segment.candidates,anchor,last,cfg)
        row['eligible_count']=len(match)
        if ns==loss_ns:
            recorded=loss['candidate_evidence']['candidates'];assert len(recorded)==len(segment.candidates)
            differences=[]
            for got,want in zip(row['candidates'],recorded):
                differences.append({key:float(np.max(np.abs(np.array(got[key])-np.array(want[key]))))
                    for key in ('position_base','extent_m','quality','anchor_distance_m','last_distance_m','extent_ratios_xy')})
            row['online_candidate_absolute_differences']=differences
            assert max(v for d in differences for v in d.values())<1e-9
        prefix=str(ns);np.savez_compressed(output/(prefix+'.npz'),**arrays)
        cv2.imwrite(str(output/(prefix+'_rgb.png')),rgb)
        # Diagnostic plot preserves the unmodified raw RGB separately above.
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        fig,axes=plt.subplots(1,2,figsize=(12,4.7));axes[0].imshow(cv2.cvtColor(rgb,cv2.COLOR_BGR2RGB))
        axes[0].set_title('Exact source RGB');axes[1].imshow(cv2.cvtColor(rgb,cv2.COLOR_BGR2RGB))
        colors=['lime','cyan','yellow','magenta']
        for i,c in enumerate(segment.candidates):
            axes[1].contour(c.mask>0,levels=[.5],colors=[colors[i%len(colors)]],linewidths=1)
            axes[1].text(c.center_uv[0],c.center_uv[1],str(i),color=colors[i%len(colors)])
        axes[1].set_title('Native measured components');[ax.set_axis_off() for ax in axes]
        fig.tight_layout();fig.savefig(output/(prefix+'_components.png'),dpi=140);plt.close(fig)
    result['files']=[dict(path=p.name,sha256=sha(p),bytes=p.stat().st_size)
        for p in output.iterdir() if p.is_file()]
    (output/'report.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps(dict(report=str(output/'report.json'),sha256=sha(output/'report.json'),
                         source_count=len(result['source_frames']))))


if __name__=='__main__':main()
