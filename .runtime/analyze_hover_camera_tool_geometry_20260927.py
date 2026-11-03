#!/usr/bin/env python3
"""Offline camera/tool-only geometry; no base FK in final table clearance."""
import json
from pathlib import Path
import cv2
import numpy as np
from tf.transformations import quaternion_matrix
from hover_camera_intrinsics_20260927 import Projection, arguments, suffix
from alicia_flexible_grasp.grasp.gripper_geometry import (
 GripperGeometry,_stage_boxes,_box_corners,_signed_polygon_margin,
 ANALYTICAL_FINGER_PAIR_CENTER_TOOL_XYZ_M,ANALYTICAL_FINGER_CONTACT_PATCH_TOOL_XZ_M)

OUT=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/unknown_grasp_20260923/hover_speed_failure_20260927')
if not Path('/media/zhuyupei/SanStand').is_mount():raise SystemExit('SanStand unavailable')
args=arguments();projection=Projection(args.model);tag=suffix(args.model)
frames=json.loads((OUT/'rgbd_frame_metadata.json').read_text())
planes=json.loads((OUT/('rgbd_table_plane_consistency'+tag+'.json')).read_text())
tf=next(r for r in frames['static'] if r['parent']=='tool0' and r['child']=='camera_link')
T_tool_camera=quaternion_matrix(tf['xyzw']);T_tool_camera[:3,3]=tf['xyz']
rot=np.eye(4);rot[:3,:3]=[[0,0,1],[-1,0,0],[0,-1,0]]
T_tool_optical=T_tool_camera@rot;T_optical_tool=np.linalg.inv(T_tool_optical)
gripper=GripperGeometry(.05,.002,np.array([.0434,.0286,.06]),np.array([.1175,.155,.0774]),.003)
K=np.array(planes['K']);report={'scope':'camera_depth_plus_static_handeye_and_CAD_no_base_FK','does_not_calibrate_real_handeye_or_CAD':True,'intrinsics_source':projection.provenance,'ray_model':args.model,'depth_scale_m_per_unit':projection.cfg['depth_scale'],'frames':{}}
for label in ('frozen','preclose','closed'):
    n=np.array(planes['frames'][label]['observed_plane_camera']['normal']);d=planes['frames'][label]['observed_plane_camera']['offset_m']
    clearances={}
    for name,c,size in _stage_boxes(np.eye(4),gripper,.04975,'y','z'):
        tool=_box_corners(c,np.eye(3),size);cam=tool@T_optical_tool[:3,:3].T+T_optical_tool[:3,3]
        clearances[name]=float(np.min(cam@n+d))
    p=ANALYTICAL_FINGER_CONTACT_PATCH_TOOL_XZ_M
    tool=np.column_stack((p[:,0],np.zeros(len(p)),p[:,1]))+ANALYTICAL_FINGER_PAIR_CENTER_TOOL_XYZ_M
    cam=tool@T_optical_tool[:3,:3].T+T_optical_tool[:3,3]
    row={'CAD_clearance_above_observed_table_m':clearances,
         'contact_patch_support_height_minmax_m':[float(np.min(cam@n+d)),float(np.max(cam@n+d))]}
    if label!='frozen':
        color=cv2.imread(str(OUT/(label+'_color.jpg')));hsv=cv2.cvtColor(color,cv2.COLOR_BGR2HSV)
        depth=np.load(OUT/(label+'_depth.npy'))*projection.cfg['depth_scale'];vv,uu=np.indices(depth.shape)
        selection=(uu>=348)&(uu<=414)&(vv>=435)&(vv<=479)&(hsv[:,:,1]<80)&(hsv[:,:,2]>55)&(depth>.08)&(depth<.20)
        u,v,z=uu[selection],vv[selection],depth[selection]
        points=projection.rays(u,v)*z[:,None];heights=points@n+d
        obj=(heights>.002)&(heights<.030)
        points,u,v,heights=points[obj],u[obj],v[obj],heights[obj]
        toolpoints=points@T_tool_optical[:3,:3].T+T_tool_optical[:3,3]
        local=toolpoints-ANALYTICAL_FINGER_PAIR_CENTER_TOOL_XYZ_M
        margins=np.array([_signed_polygon_margin(p[[0,2]],ANALYTICAL_FINGER_CONTACT_PATCH_TOOL_XZ_M) for p in local])
        row['visible_object_region']={'kind':'manual_image_ROI_partial_motor_not_full_contact_evidence','roi_xyxy':[348,435,414,479],
            'valid_foreground_points':len(points),'height_quantiles_m':np.quantile(heights,[0,.05,.5,.95,1]).tolist(),
            'pixel_centroid':[float(np.mean(u)),float(np.mean(v))],'optical_centroid_m':np.mean(points,axis=0).tolist(),
            'tool_centroid_m':np.mean(toolpoints,axis=0).tolist(),'tool_xyz_min_m':toolpoints.min(axis=0).tolist(),
            'tool_xyz_max_m':toolpoints.max(axis=0).tolist(),'contact_polygon_signed_margin_quantiles_m':np.quantile(margins,[0,.05,.5,.95,1]).tolist(),
            'count_inside_contact_xz_polygon':int(np.sum(margins>=0)),
            'count_inside_open_jaw_y_slab':int(np.sum(abs(toolpoints[:,1])<=.04975/2))}
        image=color.copy();image[v,u]=[0,255,255]
        for name,c,size in _stage_boxes(np.eye(4),gripper,.04975,'y','z'):
            if name=='palm':continue
            tool=_box_corners(c,np.eye(3),size);cam=tool@T_optical_tool[:3,:3].T+T_optical_tool[:3,3]
            try: px=projection.project(cam)
            except ValueError:
                row.setdefault('overlay_projection_warnings',[]).append(name+': inverse SDK ray has no converged projection for out-of-image CAD corners; omitted overlay only')
                continue
            poly=cv2.convexHull(np.rint(px).astype(np.int32));cv2.polylines(image,[poly],True,(255,0,255),2)
        cv2.imwrite(str(OUT/(label+'_observed_motor_and_CAD_projection'+tag+'.jpg')),image)
    report['frames'][label]=row
(OUT/('camera_only_contact_geometry'+tag+'.json')).write_text(json.dumps(report,indent=2))
print(json.dumps(report,indent=2))
