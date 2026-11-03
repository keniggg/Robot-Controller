#!/usr/bin/env python3
"""Reconstructed gen14 raw source for offline checks; no historical authority."""
import argparse
import hashlib
import json
from pathlib import Path
from types import SimpleNamespace as NS

import numpy as np
import rosbag
import rospy
import tf2_py
import yaml
from cv_bridge import CvBridge
from tf.transformations import quaternion_matrix

from alicia_grasp_modes.silhouette_observation_view_source import measured_view_source, array_fingerprint
from alicia_flexible_grasp.robot.observation_view_limits import (
    DEFAULT_OBSERVATION_VIEW_PARAMETERS, effective_observation_view_limits)
from alicia_flexible_grasp.robot.observation_view_guard import validate_observation_support_projection
from alicia_flexible_grasp.robot.observation_path_guard import SerialUrdfFk

ROOT = Path(__file__).resolve().parents[1]
ARCHIVE = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
RECORD = ARCHIVE/'grasp_records/20261009T050350.036425Z_ce203c7b9d4c4ba29e4303eb51b69d62'
AUDIT = RECORD/'keyframes_audits/b8addcd0918bfbffd80a832646845834de2aa73078746db6eb867911e16c8031.json'
STAMP = 1791522311602023839


def sha(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as f:
        for block in iter(lambda:f.read(4*1024*1024),b''):h.update(block)
    return h.hexdigest()


def matrix(message):
    t,q=message.transform.translation,message.transform.rotation
    values=np.array([q.x,q.y,q.z,q.w]); values/=np.linalg.norm(values)
    result=quaternion_matrix(values); result[:3,3]=[t.x,t.y,t.z]
    return result


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--output',type=Path,required=True)
    output=parser.parse_args().output.resolve()
    assert Path('/media/zhuyupei/SanStand').is_mount() and ARCHIVE in output.parents and not output.exists()
    audit=json.loads(AUDIT.read_text());assert audit['request_id']==21 and audit['snapshot_stamp_ns']==STAMP
    manifest=json.loads((RECORD/'manifest.json').read_text())
    assert manifest['state']=='complete' and manifest['recorder_closed'] is True
    files={row['path']:row for row in manifest['files']}
    for name in ('keyframes.bag',str(AUDIT.relative_to(RECORD))):
        assert sha(RECORD/name)==files[name]['sha256']
    image_topics=dict(color='/supervisor/camera/color/image_raw',depth='/supervisor/camera/depth/image_raw',
        mask='/perception/object_mask',object='/perception/object')
    reverse={v:k for k,v in image_topics.items()}
    exact,metadata,statuses,feedback={},{},[],[]
    camera_rows=[]; static=[]; dynamic=[]
    with rosbag.Bag(str(RECORD/'keyframes.bag')) as bag:
        for topic,msg,received in bag.read_messages(topics=list(reverse)+[
                '/grasp_record/camera_runtime_metadata','/tf','/tf_static',
                '/perception/unknown/detector_status','/alicia_d/accepted_joint_states']):
            if topic in reverse:
                if msg.header.stamp.to_nsec()==STAMP:
                    key=reverse[topic];assert key not in exact;exact[key]=msg
            elif topic.endswith('camera_runtime_metadata'):
                row=json.loads(msg.data)
                if row.get('status')=='available' and row.get('parameters'):camera_rows.append(row)
            elif topic=='/tf_static':static.extend(msg.transforms)
            elif topic=='/tf':dynamic.extend(msg.transforms)
            elif topic.endswith('detector_status'):
                row=json.loads(msg.data)
                if row.get('source_stamp_ns')==STAMP:statuses.append(row)
            elif msg.header.frame_id=='sdk_measured':
                feedback.append(dict(stamp_ns=msg.header.stamp.to_nsec(),names=list(msg.name),q=list(msg.position)))
    assert set(exact)==set(image_topics) and camera_rows
    assert any(s['generation']==14 and s['state']=='ready' and s['detail']=='tracked' for s in statuses)
    keys=('width','height','fx','fy','cx','cy','depth_scale','frame_id')
    camera={k:camera_rows[0]['parameters'][k] for k in keys}
    assert all({k:r['parameters'][k] for k in keys}==camera for r in camera_rows)
    original_runtime=camera_rows[0]['parameters']['runtime_profile']
    for key in ('device','sdk_version','color_intrinsics','native_depth_intrinsics',
                'depth_to_color_extrinsics','alignment','geometry_intrinsics',
                'depth_scale_m_per_unit','rectification','mode_depth_preset','geometry_activation_stamp_ns'):
        assert all(r['parameters']['runtime_profile'][key]==original_runtime[key] for r in camera_rows),key
    bridge=CvBridge();raw=np.asarray(bridge.imgmsg_to_cv2(exact['depth'],'passthrough'))
    color=np.asarray(bridge.imgmsg_to_cv2(exact['color'],'bgr8'));mask=np.asarray(bridge.imgmsg_to_cv2(exact['mask'],'mono8'))
    buffer=tf2_py.BufferCore(rospy.Duration(300.))
    for t in static:buffer.set_transform_static(t,'sealed_recording')
    for t in sorted(dynamic,key=lambda t:t.header.stamp.to_nsec()):buffer.set_transform(t,'sealed_recording')
    stamp=rospy.Time(STAMP//10**9,STAMP%10**9)
    mount=matrix(buffer.lookup_transform_core('tool0','camera_link',stamp))
    base_camera=matrix(buffer.lookup_transform_core('base_link','camera_link',stamp))
    base_tool=matrix(buffer.lookup_transform_core('base_link','tool0',stamp))
    optical_to_camera=np.eye(4);optical_to_camera[:3,:3]=[[0,0,1],[-1,0,0],[0,-1,0]]
    optical=base_camera@optical_to_camera
    before=max((r for r in feedback if r['stamp_ns']<=STAMP),key=lambda r:r['stamp_ns'])
    after=min((r for r in feedback if r['stamp_ns']>=STAMP),key=lambda r:r['stamp_ns'])
    assert before['names']==after['names'] and after['stamp_ns']-before['stamp_ns']<=200_000_000
    assert before['q']==after['q'] and 'right_finger' in before['names']
    qmap=dict(zip(before['names'],before['q']));q=[qmap['Joint%d'%i] for i in range(1,7)]
    model=ARCHIVE/'ros_log/user_grasp_20261008T090751Z/source_snapshot_review/gen28_fk_ik_100ms.model.urdf'
    assert sha(model)==audit['observation_failure_diagnostics']['joint_space_fk_search']['audit']['model_sha256']
    fk=SerialUrdfFk(model.read_text(),['Joint%d'%i for i in range(1,7)])
    error=float(np.max(np.abs(fk(q)-base_tool))); assert error<1e-10
    geo=audit['replay_geometry'];assert geo['available'] and geo['source_mode']=='instance_mask'
    points=np.asarray(geo['object_points_base_m'],dtype=float)
    assert len(points)==geo['object_points_count'] and array_fingerprint(points)==geo['object_points_sha256']
    geometry=NS(ok=True,support_normal_base=np.array(geo['support_normal_base']),support_offset_m=geo['support_offset_m'],
        center_base=np.array(geo['obb_center_base_m']),axes_base=np.array(geo['R_base_obb']),size_xyz_m=np.array(geo['obb_size_xyz_m']))
    identity=NS(epoch=audit['target_epoch'],track_id=audit['observe_return']['target_track_id'])
    snapshot=NS(sample_stamp_ns=tuple(audit['pipeline_funnel']['snapshot_evidence']['source_stamp_ns']),
        stamp_ns=STAMP,stamp_sec=audit['snapshot_stamp_sec'],target_identity=identity,target_epoch=identity.epoch,
        frame_id='camera_link',color_bgr=color)
    ticket=NS(payload=(snapshot,None),snapshot_stamp_sec=snapshot.stamp_sec,target_epoch=identity.epoch,
        request_id=audit['request_id'],generation=audit['generation'])
    sample=NS(stamp_ns=STAMP,target_identity=identity,target_epoch=identity.epoch,frame_id='camera_link',
        object_msg=exact['object'],color_bgr=color,depth_raw=raw,object_mask=mask)
    prepared=NS(snapshot=snapshot,ticket=ticket,geometry=geometry,pose_estimator=NS(_T_base_optical=optical))
    source=measured_view_source(sample,prepared,NS(**camera),mount)
    original_source_audit=audit['observation_failure_diagnostics']['source']['captured_audit']
    corroborated={key:source['audit'].get(key)==original_source_audit.get(key) for key in ('raw_depth_sha256','raw_mask_sha256','support_points_sha256','tool_camera_sha256','base_optical_sha256','conservative_target_corners_sha256')}
    assert all(corroborated[key] for key in ('raw_depth_sha256','raw_mask_sha256',
        'tool_camera_sha256','conservative_target_corners_sha256')),corroborated
    assert source['audit']['support_point_count']==original_source_audit['support_point_count']
    origin_delta=float(np.max(np.abs(base_camera[:3,3]-np.asarray(
        audit['observe_return']['measured_missing_sides'][0]['initial_camera_origin_base_m']))))
    assert origin_delta<1e-9,origin_delta
    config=ROOT/'src/alicia_flexible_grasp_supervisor/config/grasp_params.yaml'
    configured=yaml.safe_load(config.read_text())['grasp_6d']['remote']
    requested=original_source_audit['view_limits']['requested_visibility_limits_snapshot']
    effective=effective_observation_view_limits(requested)
    source['view_limits']=dict(requested_visibility_limits_snapshot=requested,effective_visibility_limits=effective)
    source['audit']['view_limits']=source['view_limits']
    try:
        projection=validate_observation_support_projection(base_camera,source['points_base'],source['center_base'],
            [camera[k] for k in ('fx','fy','cx','cy')],camera['width'],camera['height'],
            target_corners_base=source['corners_base'],**effective)
    except ValueError as exc:projection=dict(getattr(exc,'audit',{}),ok=False,reason=str(exc))
    document={k:v for k,v in source.items() if k not in ('points_base','corners_base')}
    document.update(schema=1,purpose='reconstructed_recorded_input_not_original_authority',
        execution_authorized=False,original_snapshot_transform_available=False,
        geometry={k:v for k,v in geo.items() if k!='object_points_base_m'},
        grasp_mode_selection=audit['grasp_mode_selection'],measured_missing_sides=audit['observe_return']['measured_missing_sides'],
        measured_joint_seed=dict(names=before['names'],q=before['q'],before=before,after=after,
            maximum_bracket_sec=(after['stamp_ns']-before['stamp_ns'])*1e-9),
        source_provenance=dict(audit_path=str(AUDIT),audit_sha256=sha(AUDIT),
            bag_path=str(RECORD/'keyframes.bag'),bag_manifest_sha256=files['keyframes.bag']['sha256'],
            bag_manifest_sha256_rehashed_this_run=True,manifest_sha256=sha(RECORD/'manifest.json'),
            exact_image_topics=image_topics,raw_color_sha256=array_fingerprint(color),
            source_status_count=len(statuses),camera_metadata_equal_projection_count=len(camera_rows),
            camera_metadata_scope='recorded parameter contents; no exact exposure association',
            config_path=str(config),config_sha256=sha(config),model_path=str(model),model_sha256=sha(model),
            source_code_path=str(Path(__file__)),source_code_sha256=sha(__file__)),
        calibration_evidence={k:original_runtime[k] for k in ('device','sdk_version','geometry_activation_stamp_ns',
            'rectification','mode_depth_preset','depth_scale_m_per_unit')},
        replay_checks=dict(original_source_audit_hash_matches=corroborated,
            actual_camera_origin_to_recorded_gap_max_abs_difference_m=origin_delta,
            original_support_point_count_matches=True,
            snapshot_transform_sha256_matches=None,original_reference_bit_identity_claimed=False,
            exact_time_bag_tf_reconstruction=True,fk_to_recorded_tool_max_abs_difference=error,
            object_points_original_sha256_matches=True,reference_endpoint_support_projection=projection),
        limitations=['Raw RGB-D, target corners and camera mount hashes match; exact-time TF reconstruction has different floating-point optical/support hashes, matching original support point count and measured gap camera origin to 1e-9. This does not restore authority.',
            'Visibility limits are read from the original failed request captured source audit.',
            'Archived URDF bytes hash matches original request recorded model hash; original model joint limits are retained.',
            'Support uses only the exact final raw depth outside its original mask and original audit plane; no plane points or missing depth are synthesized.',
            'Complete strict trajectory, following/stop, all-path visibility and fresh arrival gates remain mandatory. No live task or hardware service was called.'])
    output.mkdir(parents=True,exist_ok=False)
    np.savez_compressed(output/'source.npz',points_base=source['points_base'],corners_base=source['corners_base'],
        tool_from_camera=mount,center_base=source['center_base'],base_from_tool=source['base_from_tool'],
        base_from_camera=base_camera,base_from_optical=optical,raw_depth=raw,raw_mask=mask,color_bgr=color,
        object_points_base=points,current_joint_positions=q)
    document['source_npz_sha256']=sha(output/'source.npz')
    (output/'source.json').write_text(json.dumps(document,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps(dict(output=str(output),source_sha256=sha(output/'source.json'),
        points=len(source['points_base']),fk_error=error,reference_projection=projection),ensure_ascii=False))


if __name__=='__main__':main()
