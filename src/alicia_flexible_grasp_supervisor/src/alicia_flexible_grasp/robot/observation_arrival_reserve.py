"""Conditional endpoint visibility over the controller's goal-error box.

The fixed world points are transformed through the inverse URDF chain.
Their exact first derivatives and a global second-derivative remainder bound
enclose every state in that box. A sampled corner or optimizer success never
certifies visibility. This does not certify calibration, hardware error, RGB
identity, continuous tracking, or the independently required measured arrival.
"""
from copy import deepcopy
from collections.abc import Mapping
import math

import numpy as np

from .observation_path_guard import SerialUrdfFk, ObservationPathError, _axis_rotation
from .stationary_following import SDK_QUANTUM_RAD


def observation_goal_error_bounds(constraints, names):
    """Read the unchanged positive JTC goal tolerances, plus half a wire count."""
    if not isinstance(constraints, Mapping):
        raise ObservationPathError('OBSERVATION_GOAL_TOLERANCES_REQUIRED')
    result = []
    for name in names:
        row = constraints.get(name)
        value = row.get('goal') if isinstance(row, Mapping) else None
        if (isinstance(value, bool) or not isinstance(value, (int, float))
                or not math.isfinite(value) or not 0. < value <= .5):
            raise ObservationPathError('OBSERVATION_GOAL_TOLERANCE_INVALID: '+name)
        result.append(float(value)+SDK_QUANTUM_RAD/2)
    return np.array(result)


def arrival_reserve_from_source(fk,errors,source):
    points=np.asarray(source['corners_base'],dtype=float)
    arrival=source.get('arrival_projection')
    if arrival is not None:
        points=np.concatenate((points,np.asarray(arrival['prism_base'],dtype=float)))
    return PreparedObservationArrivalReserve(fk,errors,source['tool_from_camera'],points,
        source['camera'],source['view_limits']['effective_visibility_limits'])


class PreparedObservationArrivalReserve:
    def __init__(self, fk, errors, mount, points, camera, limits):
        if not isinstance(fk, SerialUrdfFk):
            raise ObservationPathError('OBSERVATION_ARRIVAL_BOUND_URDF_REQUIRED')
        errors, mount, points = (np.array(v,dtype=float,copy=True) for v in (errors,mount,points))
        if (errors.shape != (len(fk.names),) or not np.all(np.isfinite(errors))
                or np.any(errors <= 0.) or np.any(errors > .501)
                or mount.shape != (4,4) or not np.all(np.isfinite(mount))
                or not np.allclose(mount[3],[0,0,0,1],rtol=0,atol=1.e-9)
                or not np.allclose(mount[:3,:3].T@mount[:3,:3],np.eye(3),rtol=0,atol=1.e-9)
                or abs(np.linalg.det(mount[:3,:3])-1) > 1.e-9
                or points.ndim != 2 or points.shape[1:] != (3,)
                or not 8 <= len(points) <= 2056 or not np.all(np.isfinite(points))):
            raise ObservationPathError('OBSERVATION_ARRIVAL_BOUND_INPUT_INVALID')
        k=np.array([camera[n] for n in ('fx','fy','cx','cy')],dtype=float)
        w,h=camera['width'],camera['height']
        margin,near,far=[float(limits[n]) for n in ('roi_margin_px','minimum_depth_m','maximum_depth_m')]
        if (not np.all(np.isfinite(k)) or min(k[:2]) <= 0
                or type(w) is not int or type(h) is not int or min(w,h) <= 0
                or not all(math.isfinite(v) for v in (margin,near,far))
                or margin < 24. or margin*2 >= min(w,h) or near < .07 or far > .5 or not near < far):
            raise ObservationPathError('OBSERVATION_ARRIVAL_BOUND_CAMERA_INVALID')
        self.fk,self.errors,self.mount,self.points=deepcopy(fk),errors,mount,points
        # Frozen request mappings may be MappingProxyType. These validated
        # camera/limit fields are scalar; copy their values without unfreezing
        # or retaining the caller's mapping object.
        self.camera,self.limits=dict(camera),dict(limits)
        # For the inverse chain, fixed base transforms compose exactly on each
        # world point before the first moving axis. Thereafter triangle radii
        # bound every posture. The reversed-chain Hessian satisfies
        # |d_i d_j p_camera| <= min(radius_i,radius_j).
        radii=np.zeros((len(points),len(fk.names)));local=points.copy();radius=None
        for origin,index,_ in fk.chain:
            if radius is None:
                local=(local-origin[:3,3])@origin[:3,:3]
            else:
                radius=radius+np.linalg.norm(origin[:3,3])
            if index is not None:
                if radius is None:radius=np.linalg.norm(local,axis=1)
                radii[:,index]=radius
        self.remainder=.5*np.einsum('i,pij,j->p',errors,
            np.minimum(radii[:,:,None],radii[:,None,:]),errors)+1.e-12
        for v in (errors,mount,points,self.remainder):v.setflags(write=False)

    def geometry(self,q):
        camera=self.fk(q)@self.mount
        pose=np.eye(4);origins=np.zeros((len(q),3));axes=np.zeros_like(origins)
        for origin,index,axis in self.fk.chain:
            pose=pose@origin
            if index is not None:
                origins[index]=pose[:3,3];axes[index]=pose[:3,:3]@axis
                pose[:3,:3]=pose[:3,:3]@_axis_rotation(axis,q[index])
        local=(self.points-camera[:3,3])@camera[:3,:3]
        jacobian=-np.cross(axes[None,:,:],self.points[:,None,:]-origins[None,:,:])@camera[:3,:3]
        return local,jacobian

    def _lower(self,geometry,normals,offsets):
        local,jacobian=geometry
        normals=np.asarray(normals,dtype=float);offsets=np.asarray(offsets,dtype=float)
        gradients=np.einsum('pij,kj->pki',jacobian,normals)
        return (local@normals.T+offsets-np.abs(gradients)@self.errors
                -self.remainder[:,None]*np.linalg.norm(normals,axis=1))

    def lower_bounds(self,q,*,extra_margin_px=0.,geometry=None):
        if not math.isfinite(extra_margin_px) or extra_margin_px < 0:
            raise ObservationPathError('OBSERVATION_ARRIVAL_EXTRA_MARGIN_INVALID')
        k=self.camera;w,h=k['width'],k['height'];m=self.limits['roi_margin_px']+extra_margin_px
        normals=np.array([[1,0,0],[-1,0,0],[k['cx']-m,-k['fx'],0],
            [w-1-m-k['cx'],k['fx'],0],[k['cy']-m,0,-k['fy']],
            [h-1-m-k['cy'],0,k['fy']]],dtype=float)
        offsets=np.array([-self.limits['minimum_depth_m'],self.limits['maximum_depth_m'],0,0,0,0])
        norms=np.linalg.norm(normals,axis=1);normals/=norms[:,None];offsets/=norms
        return np.min(self._lower(self.geometry(q) if geometry is None else geometry,normals,offsets),axis=0)

    def audit(self,q,*,rgb_valid=None):
        geometry=self.geometry(q);lower=self.lower_bounds(q,geometry=geometry)
        local=geometry[0];k=self.camera
        uv=np.array([k['cx'],k['cy']])-np.array([k['fx'],k['fy']])*local[:,1:]/local[:,:1]
        edge=float(np.min(np.c_[uv[:,0],k['width']-1-uv[:,0],uv[:,1],k['height']-1-uv[:,1]]))
        report=dict(kind='conditional_arrival_joint_error_box_visibility',ok=bool(np.all(lower>=0.)),
            joint_names=list(self.fk.names),goal_joint_positions_rad=np.asarray(q).tolist(),
            goal_error_bounds_rad=self.errors.tolist(),model_sha256=self.fk.model_sha256,
            nominal_minimum_image_edge_distance_px=edge,required_image_margin_px=self.limits['roi_margin_px'],
            halfspace_order=['near','far','left','right','top','bottom'],
            halfspace_clearance_lower_bounds_m=lower.tolist(),
            maximum_second_order_remainder_m=float(np.max(self.remainder)),
            frozen_point_count=len(self.points),execution_authority=False,
            actual_arrival_measurement_required=True,certifies_hardware_tracking_or_calibration=False)
        if rgb_valid is not None and report['ok']:
            valid=np.asarray(rgb_valid)
            if valid.dtype != np.bool_ or valid.shape != (k['height'],k['width']):
                raise ObservationPathError('OBSERVATION_ARRIVAL_RGB_DOMAIN_INVALID')
            # Bound each projected extreme by monotone image halfspaces, not
            # independent xyz intervals which discard perspective correlation.
            low=np.zeros(4);high=np.array([k['width']-1,k['width']-1,k['height']-1,k['height']-1],dtype=float)
            for _ in range(18):
                mid=(low+high)/2
                normals=[[k['cx']-mid[0],-k['fx'],0],[mid[1]-k['cx'],k['fx'],0],
                         [k['cy']-mid[2],0,-k['fy']],[mid[3]-k['cy'],0,k['fy']]]
                passed=np.min(self._lower(geometry,normals,np.zeros(4)),axis=0)>=0
                move_low=passed^np.array([False,True,False,True])
                low=np.where(move_low,mid,low);high=np.where(move_low,high,mid)
            pad=max(8,int(round(16*min(k['width']/640.,k['height']/480.))))
            x0,y0=np.floor([low[0],low[2]]).astype(int)-pad
            x1,y1=np.ceil([high[1],high[3]]).astype(int)+1+pad
            observed=bool(x0>=0 and y0>=0 and x1<=k['width'] and y1<=k['height']
                          and np.all(valid[y0:y1,x0:x1]))
            report.update(required_observed_box_xyxy=[int(x0),int(y0),int(x1),int(y1)],
                          rectification_domain_observed=observed,ok=observed)
        return report
