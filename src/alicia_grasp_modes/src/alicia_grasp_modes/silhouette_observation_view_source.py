"""Measured support for one exact unknown-observation RGB-D request."""
import hashlib
import itertools

import numpy as np

from .silhouette_observation_order import rigid_transform, transform_fingerprint


def array_fingerprint(value):
    return hashlib.sha256(np.ascontiguousarray(value).tobytes()).hexdigest()


def camera_parameters(intrinsics):
    values = dict(width=int(intrinsics.width),height=int(intrinsics.height),
        fx=float(intrinsics.fx),fy=float(intrinsics.fy),cx=float(intrinsics.cx),
        cy=float(intrinsics.cy),depth_scale=float(intrinsics.depth_scale))
    if (not 1 <= values['width'] <= 16384 or not 1 <= values['height'] <= 16384
            or not all(np.isfinite(v) for v in values.values())
            or min(values['fx'],values['fy'],values['depth_scale']) <= 0.
            or not 0. <= values['cx'] < values['width']
            or not 0. <= values['cy'] < values['height']):
        raise ValueError('invalid same-source camera parameters')
    return values


def measured_view_source(sample, prepared, intrinsics, tool_from_camera):
    """Use only measured pixels within 2 mm of the frozen support plane.

    Never use median depth, a voted mask, synthetic plane points, or later RGB-D.
    Full input checks are performed once before long path searches. Consumers
    must recheck request authority and current camera/handeye parameters.
    """
    snapshot,ticket=prepared.snapshot,prepared.ticket
    ns=sample.stamp_ns
    stamps=tuple(snapshot.sample_stamp_ns)
    if (type(ns) is not int or ns <= 0 or not stamps or stamps[-1] != ns
            or ns != snapshot.stamp_ns or list(stamps) != sorted(set(stamps))
            or ticket.payload[0] is not snapshot
            or ticket.snapshot_stamp_sec != snapshot.stamp_sec
            or sample.target_identity != snapshot.target_identity
            or sample.target_epoch != snapshot.target_epoch
            or sample.target_epoch != ticket.target_epoch
            or sample.target_identity.epoch != sample.target_epoch
            or sample.frame_id != snapshot.frame_id or sample.frame_id != 'camera_link'
            or sample.object_msg.header.stamp.to_nsec() != ns
            or not sample.object_msg.detected
            or not np.array_equal(sample.color_bgr,snapshot.color_bgr)):
        raise ValueError('raw support sample is not the exact final snapshot source')
    parameters=camera_parameters(intrinsics)
    raw=np.asarray(sample.depth_raw);mask=np.asarray(sample.object_mask)
    if (raw.dtype != np.uint16 or raw.shape != (parameters['height'],parameters['width'])
            or mask.shape != raw.shape or mask.dtype != np.uint8 or not np.any(mask)
            or np.asarray(sample.color_bgr).shape != raw.shape+(3,)):
        raise ValueError('original RGB-D shape or mask mismatch')
    optical=rigid_transform(prepared.pose_estimator._T_base_optical)
    tool=rigid_transform(tool_from_camera)
    geometry=prepared.geometry
    normal=np.asarray(geometry.support_normal_base,dtype=float)
    offset=float(geometry.support_offset_m)
    center=np.asarray(geometry.center_base,dtype=float)
    axes=np.asarray(geometry.axes_base,dtype=float);size=np.asarray(geometry.size_xyz_m,dtype=float)
    if (not geometry.ok or normal.shape != (3,) or abs(np.linalg.norm(normal)-1.)>1e-6
            or center.shape != (3,) or axes.shape != (3,3) or size.shape != (3,)
            or not np.all(np.isfinite(np.r_[normal,offset,center,size,axes.ravel()]))
            or np.any(size<=0.) or not np.allclose(axes.T@axes,np.eye(3),atol=1e-6,rtol=0.)):
        raise ValueError('invalid frozen support plane or conservative target OBB')
    ys,xs=np.nonzero((raw>0)&(mask==0))
    z=raw[ys,xs].astype(float)*parameters['depth_scale']
    optical_points=np.column_stack(((xs-parameters['cx'])*z/parameters['fx'],
        (ys-parameters['cy'])*z/parameters['fy'],z))
    points=optical_points@optical[:3,:3].T+optical[:3,3]
    measured=points[np.abs(points@normal+offset)<=.002]
    if not len(measured):raise ValueError('no measured support points within frozen plane tolerance')
    corners=center+(np.asarray(list(itertools.product((-.5,.5),repeat=3)))*size)@axes.T
    measured.setflags(write=False);corners.setflags(write=False)
    binding=dict(request_id=ticket.request_id,generation=ticket.generation,
        target_epoch=ticket.target_epoch,target_track_id=snapshot.target_identity.track_id,
        snapshot_stamp_ns=ns)
    audit=dict(binding,source_kind='exact_last_raw_rgbd_sample',source_stamp_ns=ns,
        raw_depth_sha256=array_fingerprint(raw),raw_depth_dtype=str(raw.dtype),
        raw_depth_shape=list(raw.shape),raw_mask_sha256=array_fingerprint(mask),
        support_points_sha256=array_fingerprint(measured),support_point_count=len(measured),
        support_plane_base=[*normal.tolist(),offset],support_distance_tolerance_m=.002,
        base_optical_sha256=transform_fingerprint(optical),tool_camera_sha256=transform_fingerprint(tool),
        camera_parameters=parameters,conservative_target_corners_sha256=array_fingerprint(corners),
        projection_prediction_only=True,synthetic_support_points=False,
        certifies_new_view_perception=False)
    optical_to_camera=np.eye(4);optical_to_camera[:3,:3]=[[0,0,1],[-1,0,0],[0,-1,0]]
    reference=rigid_transform(optical@optical_to_camera.T@np.linalg.inv(tool))
    audit['frozen_tool_reference_sha256']=transform_fingerprint(reference)
    return dict(binding=binding,audit=audit,points_base=measured,corners_base=corners,
        base_from_tool=tuple(tuple(float(v) for v in row) for row in reference),
        center_base=tuple(float(v) for v in center),camera=parameters,
        tool_from_camera=tuple(tuple(float(v) for v in row) for row in tool))
