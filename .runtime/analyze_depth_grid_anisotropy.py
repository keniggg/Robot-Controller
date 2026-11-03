#!/usr/bin/env python3
"""Offline X/Y grid metric audit from stored depth-supported ChArUco corners.

No ROS dependencies or parameter writes. Fits no camera or board calibration.
The dimensionless ID layout labels axes; all millimetres come from stored depth.
"""
import argparse
import hashlib
import importlib.util
import json
import math
import os
from pathlib import Path
import numpy as np

ROOT=Path(__file__).resolve().parents[1];MOUNT=Path('/media/zhuyupei/SanStand')
spec=importlib.util.spec_from_file_location('anisotropy_geometry',ROOT/'.runtime/analyze_fixed_handeye_translation.py')
G=importlib.util.module_from_spec(spec);spec.loader.exec_module(G)


def summary(values):
    values=np.asarray(values,dtype=float)
    if not len(values):return None
    return dict(count=len(values),minimum=float(np.min(values)),maximum=float(np.max(values)),
                mean=float(np.mean(values)),median=float(np.median(values)),p05=float(np.percentile(values,5)),p95=float(np.percentile(values,95)))


def metrics(points,unit):
    p=np.asarray(points,dtype=float);u=np.asarray(unit,dtype=float)
    if p.shape!=u.shape or p.ndim!=2 or p.shape[1]!=3 or len(p)<12:raise ValueError('at least 12 matching grid points')
    design=np.column_stack([u[:,:2],np.ones(len(u))])
    coef,_residual,rank,singular=np.linalg.lstsq(design,p,rcond=1e-10)
    if rank<3:raise ValueError('grid does not span both board axes')
    predicted=design@coef;error=predicted-p
    lengths=np.linalg.norm(coef[:2],axis=1)*1000
    angle=math.degrees(math.acos(float(np.clip(coef[0]@coef[1]/np.prod(lengths/1000),-1,1))))
    adjacent={0:[],1:[]};regression={0:[],1:[]}
    for index in (0,1):
        other=1-index
        for i in range(len(p)):
            for j in range(i):
                delta=np.abs(u[i]-u[j])
                if abs(delta[index]-1)<1e-6 and delta[other]<1e-6 and delta[2]<1e-6:
                    adjacent[index].append(float(np.linalg.norm(p[i]-p[j])*1000))
        # Separate each complete/partial physical grid row or column. The
        # points are depth-supported and share the same fitted depth plane.
        for group in sorted(set(np.round(u[:,other],6))):
            keep=np.isclose(u[:,other],group,atol=1e-6)
            if np.count_nonzero(keep)<3 or np.ptp(u[keep,index])<2:continue
            local=np.column_stack([u[keep,index],np.ones(sum(keep))])
            beta=np.linalg.lstsq(local,p[keep],rcond=1e-10)[0]
            residual=local@beta-p[keep]
            regression[index].append(dict(fixed_other_coordinate=float(group),count=int(sum(keep)),
                span_squares=float(np.ptp(u[keep,index])),square_mm=float(np.linalg.norm(beta[0])*1000),
                direction_optical=(beta[0]/np.linalg.norm(beta[0])).tolist(),
                residual_rms_mm=float(np.sqrt(np.mean(np.sum(residual**2,axis=1)))*1000)))
    horizontal=summary(adjacent[0]);vertical=summary(adjacent[1])
    if horizontal is None or vertical is None:raise ValueError('no horizontal or vertical adjacent corner pairs')
    # Spatial quadrant leaveout gauges grid/model sensitivity; it does not
    # resample an independent depth plane or remove its common uncertainty.
    center=np.median(u[:,:2],axis=0);labels=(u[:,0]>=center[0]).astype(int)+2*(u[:,1]>=center[1]).astype(int)
    leaveout=[]
    for label in range(4):
        keep=labels!=label
        if sum(keep)<12:continue
        b=np.linalg.lstsq(design[keep],p[keep],rcond=1e-10)[0];l=np.linalg.norm(b[:2],axis=1)*1000
        leaveout.append(dict(removed_quadrant=label,remaining=int(sum(keep)),x_square_mm=float(l[0]),y_square_mm=float(l[1]),x_over_y=float(l[0]/l[1])))
    return dict(horizontal_adjacent_mm=horizontal,vertical_adjacent_mm=vertical,
        adjacent_median_x_over_y=horizontal['median']/vertical['median'],
        joint_3d_affine_regression=dict(x_square_mm=float(lengths[0]),y_square_mm=float(lengths[1]),x_over_y=float(lengths[0]/lengths[1]),
            x_delta_from_15mm=float(lengths[0]-15),y_delta_from_15mm=float(lengths[1]-15),axis_angle_deg=angle,
            square_vectors_optical_mm=(coef[:2]*1000).tolist(),residual_rms_mm=float(np.sqrt(np.mean(np.sum(error**2,axis=1)))*1000),
            residual_axis_xyz_rms_mm=(np.sqrt(np.mean(error**2,axis=0))*1000).tolist(),design_singular_values=singular.tolist()),
        horizontal_row_regressions=regression[0],vertical_column_regressions=regression[1],
        horizontal_row_square_mm=summary([row['square_mm'] for row in regression[0]]),
        vertical_column_square_mm=summary([row['square_mm'] for row in regression[1]]),
        quadrant_leaveout=leaveout)


def normal_sensitivity(points,unit,normal):
    points=np.asarray(points);normal=np.asarray(normal);normal/=np.linalg.norm(normal)
    center=points.mean(axis=0);rays=points/points[:,2,None]
    result=[]
    for degree in (.1,.5,1.):
        for axis in (0,1):
            for sign in (-1,1):
                rotation=np.zeros(3);rotation[axis]=math.radians(sign*degree)
                n=G.exp_so3(rotation)@normal;offset=-float(n@center)
                distance=-offset/(rays@n);newpoints=rays*distance[:,None]
                design=np.column_stack([unit[:,:2],np.ones(len(unit))]);coef=np.linalg.lstsq(design,newpoints,rcond=1e-10)[0]
                length=np.linalg.norm(coef[:2],axis=1)*1000
                result.append(dict(optical_rotation_axis='xy'[axis],normal_rotation_deg=sign*degree,
                    x_square_mm=float(length[0]),y_square_mm=float(length[1]),x_over_y=float(length[0]/length[1])))
    return result


def analyze(report):
    samples=[]
    for source in report['samples']:
        p=np.asarray(source['depth_supported_corners_optical_m']);u=np.asarray(source['unit_board_xyz'])
        row=dict(geometry=source['geometry'],source_stamp_ns=source['source_stamp_ns'],charuco_count=len(p),
            independent_depth_plane_training_rms_mm=source['plane']['inlier_rms_mm'],
            independent_depth_plane_holdout_rms_mm=source['plane']['validation_rms_mm'],
            depth_minus_pnp_normal_deg=source['depth_plane_vs_archived_pnp_normal_deg'],
            **metrics(p,u),normal_perturbation_sensitivity=normal_sensitivity(p,u,source['plane']['normal_optical']))
        if source.get('offline_stationary_phase'):row['offline_stationary_phase']=source['offline_stationary_phase']
        samples.append(row)
    return dict(schema=1,kind='depth_supported_dimensionless_id_grid_anisotropy',session=report['session'],samples=samples,
        scope='No calibration/board model fit is deployed; no ROS I/O.',
        limits=['Depth-supported 3D shares camera rays, depth scale, plane tilt and registration assumptions.',
                'Rows/columns and neighbor lengths share one fitted depth plane; they are not independent absolute measurements.',
                'Quadrant leaveout is model/coverage sensitivity, not an external confidence interval.',
                'Normal perturbations keep the 3D centroid fixed and vary only plane orientation; they demonstrate sensitivity, not a fitted correction.',
                'Uniform printed scale cannot change PnP rotation; anisotropic printing could, but depth-inferred anisotropy is confounded with plane tilt and camera geometry.',
                'No old/new session absolute board coordinates are subtracted.'])


def main():
    ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('report',type=Path);ap.add_argument('--output',required=True,type=Path);args=ap.parse_args()
    if not os.path.ismount(MOUNT) or MOUNT not in args.output.resolve().parents:raise RuntimeError('Mounted SanStand required')
    report=json.loads(args.report.read_text());result=analyze(report)
    result['source']=dict(path=str(args.report),sha256=hashlib.sha256(args.report.read_bytes()).hexdigest())
    args.output.mkdir(parents=True,exist_ok=True)
    (args.output/'depth_grid_anisotropy.json').write_text(json.dumps(result,ensure_ascii=False,indent=2,allow_nan=False)+'\n')
    (args.output/'analysis_source.py').write_bytes(Path(__file__).read_bytes())
    print(json.dumps(dict(output=str(args.output),samples=[dict(geometry=r['geometry'],horizontal=r['horizontal_adjacent_mm']['median'],vertical=r['vertical_adjacent_mm']['median'],adjacent_ratio=r['adjacent_median_x_over_y'],affine=r['joint_3d_affine_regression'],row_span=[r['horizontal_row_square_mm']['minimum'],r['horizontal_row_square_mm']['maximum']],column_span=[r['vertical_column_square_mm']['minimum'],r['vertical_column_square_mm']['maximum']]) for r in result['samples']]),ensure_ascii=False))

if __name__=='__main__':main()
