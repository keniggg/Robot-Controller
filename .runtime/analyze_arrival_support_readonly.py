#!/usr/bin/env python3
"""Offline independent current-ring fits against the original immutable reference."""
import hashlib
import json
from pathlib import Path
from dataclasses import asdict
from types import SimpleNamespace as NS

import numpy as np
import rosbag
import rospy
import tf2_py
from cv_bridge import CvBridge
from tf.transformations import quaternion_matrix
from alicia_grasp_modes import tabletop as table

ROOT=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
RECORD=ROOT/'grasp_records/20261009T005610.945830Z_db47451a69e647de884e9af9aaeb38a2'
OUTPUT=ROOT/'ros_log/full_latest_ros_20261009T003407Z/arrival_static_capture'
FIRST_LOSS=1791507424405864715


def sha(path):
    h=hashlib.sha256()
    with path.open('rb') as f:
        for block in iter(lambda:f.read(4194304),b''):h.update(block)
    return h.hexdigest()


def matrix(message):
    t=message.transform
    m=quaternion_matrix([t.rotation.x,t.rotation.y,t.rotation.z,t.rotation.w])
    m[:3,3]=[t.translation.x,t.translation.y,t.translation.z]
    return m


def read(path, original=False):
    depths={};feedback=[];dynamic=[];static=[];cameras=[];anchor=None
    topics=['/supervisor/camera/depth/image_raw','/alicia_d/accepted_joint_states','/tf','/tf_static',
            '/arrival_diagnostic/camera_parameters','/grasp_record/camera_runtime_metadata']
    if original:topics.append('/perception/unknown/detector_status')
    with rosbag.Bag(str(path)) as bag:
        for topic,msg,stamp in bag.read_messages(topics=topics):
            if topic.endswith('/depth/image_raw'):
                if not original or msg.header.stamp.to_nsec()==FIRST_LOSS:
                    depths[msg.header.stamp.to_nsec()]=msg
            elif topic=='/tf':dynamic.extend(msg.transforms)
            elif topic=='/tf_static':static.extend(msg.transforms)
            elif topic=='/alicia_d/accepted_joint_states':
                feedback.append(dict(stamp_ns=msg.header.stamp.to_nsec(),names=list(msg.name),q=list(msg.position)))
            elif topic in ('/arrival_diagnostic/camera_parameters','/grasp_record/camera_runtime_metadata'):
                row=json.loads(msg.data)
                if isinstance(row.get('parameters'),dict):cameras.append(row['parameters'])
            elif topic.endswith('detector_status'):
                row=json.loads(msg.data)
                if row.get('detail')=='acquired' and row.get('generation')==3 and anchor is None:anchor=row
    buffer=tf2_py.BufferCore(rospy.Duration(600.))
    for t in static:buffer.set_transform_static(t,'sealed_diagnostic')
    for t in sorted(dynamic,key=lambda t:t.header.stamp.to_nsec()):buffer.set_transform(t,'sealed_diagnostic')
    return depths,feedback,cameras,buffer,anchor


def main():
    old=read(RECORD/'keyframes.bag',True)
    current=read(OUTPUT/'arrival.bag')
    reference=old[4];assert reference is not None
    plane=np.asarray(reference['reference_support_plane_base'])
    anchor=NS(position_base=np.asarray(reference['anchor_position_base']),
              support_footprint_base=np.asarray(reference['reference_support_footprint_base']))
    cfg=table.Config();assert cfg.support_max_local_shift_m==.004
    old_camera=old[2][0];camera=current[2][0]
    keys=('fx','fy','cx','cy','width','height','depth_scale','align_depth_to_color')
    assert all(row[k]==camera[k] for row in current[2] for k in keys)
    assert all(old_camera[k]==camera[k] for k in keys)
    assert old_camera['runtime_profile']['rectification']==camera['runtime_profile']['rectification']
    assert old_camera['runtime_profile']['geometry_activation_stamp_ns']==camera['runtime_profile']['geometry_activation_stamp_ns']
    bridge=CvBridge(); intrinsics=[camera[k] for k in ('fx','fy','cx','cy')]

    def fit(ns,data):
        transform=matrix(data[3].lookup_transform_core('base_link','camera_link',rospy.Time(ns//10**9,ns%10**9)))
        depth=np.asarray(bridge.imgmsg_to_cv2(data[0][ns],'passthrough'),dtype=np.float32)*float(camera['depth_scale'])
        uv=table.project_base_to_pixel(anchor.position_base,transform,intrinsics)
        v,u=np.indices(depth.shape);scale=min(depth.shape[1]/640.,depth.shape[0]/480.)
        valid=np.isfinite(depth)&(depth>=cfg.depth_min_m)&(depth<=cfg.depth_max_m)
        row=dict(source_stamp_ns=ns,T_base_camera_link=transform.tolist(),target_projected_uv=uv.tolist())
        try:
            p,ratio,residual,count,ring,points,outer=table._fit_support_ring_sequence(
                depth,intrinsics,valid,np.hypot(u-uv[0],v-uv[1]),scale,cfg)
            supporting=np.abs(points@p[:3]+p[3])<=cfg.support_threshold_m
            quadrants=table._support_quadrants(supporting,u[ring],v[ring],uv)
            cur=table._normalized_plane(np.linalg.solve(transform.T,p))
            if cur[:3]@plane[:3]<0:cur=-cur
            ref=table._normalized_plane(plane);delta=cur-ref
            anchor_shift=abs(float(delta[:3]@anchor.position_base+delta[3]))
            footprint_shift=float(np.max(np.abs(anchor.support_footprint_base@delta[:3]+delta[3])))
            row.update(current_plane_base=cur.tolist(),inlier_ratio=ratio,residual_m=residual,
                sample_count=int(count),quadrants=int(quadrants),ring_outer_px=outer,
                normal_change_deg=float(np.rad2deg(np.arccos(np.clip(cur[:3]@ref[:3],-1.,1.)))),
                anchor_distance_m=anchor_shift,footprint_distance_m=footprint_shift,
                maximum_local_distance_m=max(anchor_shift,footprint_shift),
                original_fit_quality_passed=bool(residual<=cfg.support_threshold_m and quadrants>=3))
            try:
                row['original_audit']=table._audit_current_support(depth,intrinsics,transform,cfg,anchor,plane)
                row['original_gate_passed']=bool(row['original_audit']['support_current_audit_available'])
            except ValueError as exc:
                row.update(original_gate_passed=False,original_gate_error=str(exc))
        except ValueError as exc:
            row.update(original_gate_passed=False,fit_error=str(exc))
        before=max((f for f in data[1] if f['stamp_ns']<=ns),key=lambda f:f['stamp_ns'])
        after=min((f for f in data[1] if f['stamp_ns']>=ns),key=lambda f:f['stamp_ns'])
        row['feedback_bracket']=dict(before=before,after=after,span_sec=(after['stamp_ns']-before['stamp_ns'])*1e-9)
        return row

    stamps=sorted(current[0]); chosen=[stamps[i] for i in np.linspace(2,len(stamps)-3,10,dtype=int)]
    rows=[fit(ns,current) for ns in chosen]
    first_loss=fit(FIRST_LOSS,old)
    q=np.asarray([[dict(zip(f['names'],f['q']))['Joint%d'%i] for i in range(1,7)] for f in current[1]])
    distances=[r['maximum_local_distance_m'] for r in rows if 'maximum_local_distance_m' in r]
    report=dict(scope='offline diagnostic; lost target identity and execution authority remain retired',
        execution_authority=False,live_actions_called=False,reference_acquired_source_ns=reference['source_stamp_ns'],
        reference_plane_base=plane.tolist(),reference_anchor_base=anchor.position_base.tolist(),
        reference_footprint_base=anchor.support_footprint_base.tolist(),config=asdict(cfg),
        config_scope='unchanged pure Config defaults; exact recorded 4mm rejection independently replayed below',
        original_record=str(RECORD),original_bag_sha256=sha(RECORD/'keyframes.bag'),
        capture_bag_sha256=sha(OUTPUT/'arrival.bag'),source_code_sha256=sha(Path(table.__file__)),
        script_sha256=sha(Path(__file__)),camera_intrinsics_scale_rectification_activation_equal=True,
        recorded_first_loss_replay=first_loss,stationary_samples=rows,
        stationary_feedback_count=len(q),stationary_arm_joint_min_rad=q.min(axis=0).tolist(),
        stationary_arm_joint_max_rad=q.max(axis=0).tolist(),stationary_arm_joint_range_rad=np.ptp(q,axis=0).tolist(),
        summary=dict(samples=len(rows),qualified_current_fit_count=sum(r.get('original_fit_quality_passed',False) for r in rows),
            original_gate_pass_count=sum(r['original_gate_passed'] for r in rows),
            maximum_local_distance_min_m=min(distances) if distances else None,
            maximum_local_distance_max_m=max(distances) if distances else None,
            maximum_local_distance_median_m=float(np.median(distances)) if distances else None),
        limitations=['Diagnostic uses the captured arrival pose only; subsequent user movement cannot inherit it.',
            'Persistence or disappearance of plane discrepancy alone cannot distinguish calibration error, depth bias, fitting bias or physical support shape.',
            'No new target generation, new acquisition, closed-loop action or lost-latch recovery is performed.'])
    output=OUTPUT/'support_plane_analysis.json'
    with output.open('x') as handle:json.dump(report,handle,ensure_ascii=False,indent=2);handle.write('\n')
    print(json.dumps(dict(output=str(output),summary=report['summary'],
        recorded_first_loss=first_loss.get('maximum_local_distance_m'),
        feedback_range_rad=report['stationary_arm_joint_range_rad'],sha256=sha(output)),ensure_ascii=False))


if __name__=='__main__':main()
