"""Offline camera profile and pure SDK math; never opens a device or ROS node."""
import argparse
import hashlib
import json
from pathlib import Path
import numpy as np

OUT = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/unknown_grasp_20260923/hover_speed_failure_20260927')
RUNTIME = OUT/'runtime_parameters_readonly.json'

def profile():
    cfg=json.loads(RUNTIME.read_text())['/camera']
    p=cfg['runtime_profile'];intr=p['geometry_intrinsics']
    if p['source']!='realsense_active_profile' or cfg['intrinsics_source']!='sdk_active_color_profile':
        raise ValueError('missing active SDK geometry evidence')
    for k in ('fx','fy','cx','cy'):
        if cfg[k]!=intr[k]:raise ValueError('namespace/profile intrinsics disagree')
    if cfg['depth_scale']!=p['depth_scale_m_per_unit']:raise ValueError('depth scales disagree')
    return cfg

def provenance():
    c=profile()
    return {'source_path':str(RUNTIME),'source_sha256':hashlib.sha256(RUNTIME.read_bytes()).hexdigest(),
            'capture_timestamp_in_file':None,'not_per_frame_CameraInfo':True,
            'intrinsics_source':c['intrinsics_source'],'runtime_profile':c['runtime_profile'],
            'crosscheck':'intrinsics_audit_crosscheck.json: exact frozen-audit visibility projection',
            'stale_camera_yaml_rejected':True}

def arguments():
    p=argparse.ArgumentParser();p.add_argument('--model',choices=('pinhole','sdk_distortion'),default='pinhole')
    return p.parse_args()

def suffix(model):return '' if model=='pinhole' else '_sdk_distortion'

class Projection:
    def __init__(self,model='pinhole'):
        self.cfg=profile();self.model=model
        c=self.cfg;self.K=np.array([[c['fx'],0,c['cx']],[0,c['fy'],c['cy']],[0,0,1.]])
        self.provenance=provenance()
        if model=='sdk_distortion':
            import pyrealsense2 as rs
            self.rs=rs;p=c['runtime_profile']['geometry_intrinsics'];i=rs.intrinsics()
            i.width=p['width'];i.height=p['height'];i.fx=p['fx'];i.fy=p['fy'];i.ppx=p['cx'];i.ppy=p['cy']
            i.model=getattr(rs.distortion,p['model']);i.coeffs=p['coeffs'];self.intr=i
            self.provenance['sdk_math_module']=rs.__file__
    def rays(self,u,v):
        u=np.asarray(u);v=np.asarray(v)
        if self.model=='pinhole':return np.column_stack(((u-self.K[0,2])/self.K[0,0],(v-self.K[1,2])/self.K[1,1],np.ones(u.size)))
        return np.array([self.rs.rs2_deproject_pixel_to_point(self.intr,[float(x),float(y)],1.) for x,y in zip(u.flat,v.flat)])
    def project(self,points):
        points=np.atleast_2d(points);guess=points@self.K.T;guess=guess[:,:2]/guess[:,2:]
        if self.model=='pinhole':return guess
        # Inverse Brown is a deprojection model; numerically invert the official
        # SDK ray function rather than calling an unsupported SDK projection.
        result=[]
        for p,g in zip(points,guess):
            target=p[:2]/p[2];uv=g.copy()
            f=lambda at: self.rays([at[0]],[at[1]])[0,:2]-target
            for _ in range(30):
                residual=f(uv)
                if np.max(abs(residual))<2e-7:break
                h=.05
                jac=np.column_stack(((f(uv+[h,0])-f(uv-[h,0]))/(2*h),(f(uv+[0,h])-f(uv-[0,h]))/(2*h)))
                uv-=np.linalg.solve(jac,residual)
            if not np.all(np.isfinite(uv)) or not np.all(np.isfinite(f(uv))) or np.max(abs(f(uv)))>2e-7 or np.max(abs(uv))>1e6:raise ValueError('SDK inverse ray projection did not converge')
            result.append(uv)
        return np.array(result)
