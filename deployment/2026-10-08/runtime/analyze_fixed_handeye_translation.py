#!/usr/bin/env python3
"""Offline identifiability audit. Reads closed multiframe data; no ROS imports.

Fits a diagnostic fixed tool-frame translation delta with hand-eye rotation held
fixed. Separate per-session board nuisance positions prevent cross-board mixing.
Nothing emitted by this program is a calibration, target or motion command.
"""
import argparse
from collections import defaultdict
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
MOUNT = Path('/media/zhuyupei/SanStand')
ARCHIVE = MOUNT / '实验记录/Robot-Controller'
PRIOR = ARCHIVE / 'ros_log/grasp_repair_resume_20260928T104059Z'
CONFIG = [(5, 'j5_multiframe_105805', 'bounded_joint_response_20260928T105805Z_23293'),
          (2, 'j2_multiframe_105915', 'bounded_joint_response_20260928T105915Z_24761'),
          (3, 'j3_multiframe_110035', 'bounded_joint3_response_20260928T110035Z_26293')]
CENTER = np.array([.09, .0675, 0.])


def skew(v):
    x,y,z = v
    return np.array([[0.,-z,y],[z,0.,-x],[-y,x,0.]])


def exp_so3(vector):
    vector = np.asarray(vector, dtype=float)
    theta = np.linalg.norm(vector)
    if theta < 1e-12:
        return np.eye(3) + skew(vector)
    k = skew(vector/theta)
    return np.eye(3) + math.sin(theta)*k + (1.-math.cos(theta))*(k@k)


def log_so3(rotation):
    angle = math.acos(float(np.clip((np.trace(rotation)-1.)/2., -1., 1.)))
    vector = np.array([rotation[2,1]-rotation[1,2], rotation[0,2]-rotation[2,0], rotation[1,0]-rotation[0,1]])
    if angle < 1e-8:
        return vector / 2.
    if math.pi-angle < 1e-5:
        vals, vecs = np.linalg.eigh((rotation+np.eye(3))/2.)
        axis = vecs[:,np.argmax(vals)]
        if axis@vector < 0.: axis = -axis
        return axis*angle
    return vector * (angle/(2.*math.sin(angle)))


def inv(t):
    answer = np.eye(4)
    answer[:3,:3] = t[:3,:3].T
    answer[:3,3] = -answer[:3,:3]@t[:3,3]
    return answer


def average_transform(transforms):
    ts = np.asarray(transforms)
    u,_s,v = np.linalg.svd(np.mean(ts[:,:3,:3],axis=0))
    rotation = u@np.diag([1.,1.,np.linalg.det(u@v)])@v
    answer = np.eye(4)
    answer[:3,:3] = rotation
    answer[:3,3] = np.mean(ts[:,:3,3],axis=0)
    return answer


def summarize(rows, point, resample=None, bin_sec=.5):
    start = min(r['source_stamp_ns'] for r in rows)
    blocks = defaultdict(list)
    for row in rows:
        index = int((row['source_stamp_ns']-start)//int(bin_sec*1e9))
        blocks[index].append(row)
    groups = list(blocks.values())
    if resample is not None:
        groups = [groups[i] for i in resample.integers(0,len(groups),size=len(groups))]
    # Equal time-block weights limit repeated frames from masquerading as
    # independent evidence. Each block retains its within-block frame mean.
    rs, ps, cameras, qs, board_rotations = [], [], [], [], []
    for group in groups:
        tools = np.asarray([r['T_base_tool_fk'] for r in group])
        boards = np.asarray([r['T_base_board_fk'] for r in group])
        rs.append(np.mean(tools[:,:3,:3],axis=0))
        ps.append(np.mean(boards[:,:3,3]+boards[:,:3,:3]@point,axis=0))
        cameras.append(average_transform([r['T_camera_board'] for r in group]))
        qs.append(np.mean([r['accepted_rad'] for r in group],axis=0))
        board_rotations.append(average_transform(boards))
    return dict(R_tool=np.mean(rs,axis=0), position=np.mean(ps,axis=0),
                camera=average_transform(cameras), joints=np.mean(qs,axis=0),
                board_pose=average_transform(board_rotations),
                frame_count=len(rows), block_count=len(blocks),
                span_sec=(max(r['source_stamp_ns'] for r in rows)-start)*1e-9)


def contrast(first, second, label, joint):
    return dict(label=label, joint=joint, matrix=second['R_tool']-first['R_tool'],
                delta=second['position']-first['position'],
                baseline=first, outcome=second,
                relative_board_rotation_deg=math.degrees(np.linalg.norm(log_so3(first['board_pose'][:3,:3].T@second['board_pose'][:3,:3]))))


def fit(contrasts):
    matrix = np.vstack([c['matrix'] for c in contrasts])
    delta = np.concatenate([c['delta'] for c in contrasts])
    correction, _r, rank, singular = np.linalg.lstsq(matrix,-delta,rcond=1e-10)
    _u,_s,vt = np.linalg.svd(matrix,full_matrices=False)
    return dict(correction=correction, rank=int(rank), singular=singular,
                condition=float(singular[0]/singular[-1]) if singular[-1]>1e-12 else None,
                weakest_direction=vt[-1], residual=matrix@correction+delta,
                matrix=matrix)


def screw(transform, minimum_angle=1e-5):
    vector = log_so3(transform[:3,:3])
    theta = np.linalg.norm(vector)
    if theta < minimum_angle:
        raise ValueError('Rotation too small for a bounded screw-axis estimate')
    axis = vector/theta
    axial = float(axis@transform[:3,3])
    matrix = np.vstack([np.eye(3)-transform[:3,:3],axis])
    target = np.r_[transform[:3,3]-axis*axial,0.]
    point = np.linalg.lstsq(matrix,target,rcond=1e-10)[0]
    return dict(axis=axis, point=point, angle_rad=theta, axial_translation_m=axial,
                translation_lever_gain=float(1./(2.*math.sin(theta/2.))),
                closure_residual_m=float(np.linalg.norm(matrix@point-target)))


def camera_world_motion(first, second):
    # C = camera_T_board; board-frame rigid camera displacement avoids X.
    return inv(second['camera'])@first['camera']


def pair_parallel_distance(first_motion, second_motion):
    a, b = screw(first_motion), screw(second_motion)
    aligned = b['axis'] if a['axis']@b['axis'] >= 0. else -b['axis']
    axis = (a['axis']+aligned)/np.linalg.norm(a['axis']+aligned)
    points=[]
    for motion in (first_motion,second_motion):
        angle = float(log_so3(motion[:3,:3])@axis)
        rotation = exp_so3(axis*angle)
        target = motion[:3,3] - axis*(axis@motion[:3,3])
        points.append(np.linalg.lstsq(np.vstack([np.eye(3)-rotation,axis]),np.r_[target,0.],rcond=1e-10)[0])
    return dict(distance_m=float(np.linalg.norm(points[1]-points[0])),
                measured_axis_angle_deg=math.degrees(math.acos(float(np.clip(abs(a['axis']@b['axis']),0.,1.)))),
                common_axis=axis, points=points,
                caveat='Parallel-constrained fit; raw axis angle is reported separately. Metric depends on RGB intrinsics and unmeasured printed board scale, but not hand-eye X.')


def serial_fk_and_axes(xml, values):
    robot = ET.fromstring(xml)
    bychild={j.find('child').get('link'):j for j in robot.findall('joint')}
    chain=[]; child='tool0'
    while child != 'base_link':
        joint=bychild[child];chain.append(joint);child=joint.find('parent').get('link')
    t=np.eye(4); axes={}
    for j in reversed(chain):
        origin=j.find('origin')
        xyz=np.fromstring(origin.get('xyz','0 0 0'),sep=' ')
        rpy=np.fromstring(origin.get('rpy','0 0 0'),sep=' ')
        o=np.eye(4);o[:3,3]=xyz
        o[:3,:3]=exp_so3([0,0,rpy[2]])@exp_so3([0,rpy[1],0])@exp_so3([rpy[0],0,0])
        t=t@o
        if j.get('type')=='revolute':
            name=j.get('name');index=int(name.replace('Joint',''))-1
            a=np.fromstring(j.find('axis').get('xyz'),sep=' ');a/=np.linalg.norm(a)
            axes[index+1]=dict(point=t[:3,3].copy(),direction=t[:3,:3]@a)
            t[:3,:3]=t[:3,:3]@exp_so3(a*values[index])
    return t,axes


def as_json(x):
    if isinstance(x,np.ndarray):return x.tolist()
    if isinstance(x,np.generic):return x.item()
    raise TypeError(type(x).__name__)


def percentile(values):
    a=np.asarray(values)
    return dict(median=np.median(a,axis=0),p025=np.percentile(a,2.5,axis=0),p975=np.percentile(a,97.5,axis=0))


def load():
    sessions=[]
    for joint,multi,record in CONFIG:
        samples_path=PRIOR/multi/'samples.json'
        rows=json.loads(samples_path.read_text())
        base=ARCHIVE/'ros_log'/record
        geometry=json.loads((base/'baseline_geometry.json').read_text())
        sessions.append(dict(joint=joint, rows=rows, geometry=geometry, directory=str(base),
                             source=str(samples_path),sha256=hashlib.sha256(samples_path.read_bytes()).hexdigest()))
    return sessions


def build(sessions,point,resample=None,population='quality_tf'):
    contrasts=[]; returns=[]; windows={}
    for session in sessions:
        bywindow=defaultdict(list)
        for r in session['rows']:
            if population=='exact_rgb' and not r['exact_fresh_rgb_quality_tf']:continue
            bywindow[r['window']].append(r)
        ww={key:summarize(rows,point,resample) for key,rows in bywindow.items()}
        windows[session['joint']]=ww
        if not all(k in ww for k in ('baseline','outbound_stationary')):continue
        contrasts.append(contrast(ww['baseline'],ww['outbound_stationary'],'outbound',session['joint']))
        if 'return_stationary' in ww:
            returns.append(contrast(ww['baseline'],ww['return_stationary'],'return',session['joint']))
    return contrasts,returns,windows


def evaluate(contrasts,correction):
    return [dict(joint=c['joint'],label=c['label'],original_delta_mm=c['delta']*1000,
                 corrected_delta_mm=(c['delta']+c['matrix']@correction)*1000,
                 original_norm_mm=float(np.linalg.norm(c['delta'])*1000),
                 corrected_norm_mm=float(np.linalg.norm(c['delta']+c['matrix']@correction)*1000),
                 board_rotation_difference_deg=c['relative_board_rotation_deg']) for c in contrasts]


def run(sessions,nboot,seed):
    results={}
    for name,point in [('board_origin',np.zeros(3)),('board_center',CENTER)]:
        contrasts,returns,windows=build(sessions,point)
        solution=fit(contrasts)
        rng=np.random.default_rng(seed)
        samples=[]; held=[]; distances=[]
        for _ in range(nboot):
            cb,rb,wb=build(sessions,point,rng)
            f=fit(cb);samples.append(f['correction']*1000)
            held.append([np.linalg.norm(c['delta']+c['matrix']@f['correction'])*1000 for c in cb])
            byjoint={c['joint']:c for c in cb}
            if 2 in byjoint and 3 in byjoint:
                a,b=byjoint[2],byjoint[3]
                value=pair_parallel_distance(camera_world_motion(a['baseline'],a['outcome']),camera_world_motion(b['baseline'],b['outcome']))
                distances.append([value['distance_m']*1000,value['measured_axis_angle_deg']])
        leave=[]
        for c in contrasts:
            rest=[x for x in contrasts if x['joint']!=c['joint']]
            f=fit(rest)
            leave.append(dict(held_out_joint=c['joint'],fit_rank=f['rank'],fit_singular_values=f['singular'],
                              condition=f['condition'],correction_mm=f['correction']*1000,
                              heldout=evaluate([c],f['correction'])[0]))
        exact,er,_ew=build(sessions,point,population='exact_rgb')
        results[name]=dict(point_board_m=point,correction_tool_mm=solution['correction']*1000,
            rank=solution['rank'],singular_values=solution['singular'],condition=solution['condition'],
            weakest_tool_direction=solution['weakest_direction'],training=evaluate(contrasts,solution['correction']),
            return_holdout=evaluate(returns,solution['correction']),leave_one_joint_out=leave,
            exact_rgb_subset_same_fit=evaluate(exact,solution['correction']),
            window_counts={joint:{key:{k:w[k] for k in ('frame_count','block_count','span_sec')} for key,w in ww.items()} for joint,ww in windows.items()},
            conditional_block_bootstrap=dict(replicates=nboot,bin_sec=.5,seed=seed,
                correction_mm=percentile(samples),training_residual_mm=percentile(held),
                parallel_j2_j3_axis_distance_mm_and_axis_angle_deg=percentile(distances),
                limitation='Conditional empirical time-block spread only; short windows and fixed systematic PnP, board scale, intrinsics and robot-model errors are not covered. Not an external accuracy confidence interval.'))
    cs,_,ws=build(sessions,CENTER)
    screws={}
    for c in cs:
        motion=camera_world_motion(c['baseline'],c['outcome'])
        screws[c['joint']]=dict(camera_motion_board=motion,**screw(motion))
    byjoint={c['joint']:c for c in cs}
    a,b=byjoint[2],byjoint[3]
    distance=pair_parallel_distance(camera_world_motion(a['baseline'],a['outcome']),camera_world_motion(b['baseline'],b['outcome']))
    xml=(Path(sessions[0]['directory'])/'robot_description.urdf').read_text()
    q=a['baseline']['joints'];f0,axes=serial_fk_and_axes(xml,q)
    v=axes[3]['point']-axes[2]['point'];u=axes[2]['direction']
    nominal=float(np.linalg.norm(v-u*(u@v)))
    axisangles={str(j):math.degrees(math.acos(float(np.clip(abs(axes[2]['direction']@axes[j]['direction']),0.,1.)))) for j in (3,4,5,6)}
    proposed=[]
    for degree in (2.,4.,6.,8.):
        changed=q.copy();changed[1]+=math.radians(degree);changed[2]-=math.radians(degree)
        after,_=serial_fk_and_axes(xml,changed)
        rel=inv(f0)@after
        proposed.append(dict(counterrotation_deg=degree,delta_tool_base_m=after[:3,3]-f0[:3,3],
                             translation_norm_mm=float(np.linalg.norm(rel[:3,3])*1000),
                             orientation_delta_deg=math.degrees(np.linalg.norm(log_so3(rel[:3,:3]))),
                             theoretical_2L_sin_half_mm=2.*nominal*math.sin(math.radians(degree)/2.)*1000,
                             execution_authorized=False,live_path_not_checked=True))
    return dict(schema=1,generated_utc=datetime.now(timezone.utc).isoformat(),
        scope='Offline diagnostic fit only; no ROS interface, TF update, motion plan or command.',
        method='For each session/window, d = mean(p_out)-mean(p_base); M = mean(R_tool_out)-mean(R_tool_base); solve M delta_t = -d. Separate board offsets cancel within each session.',
        source_sessions=[{k:s[k] for k in ('joint','directory','source','sha256')} for s in sessions],
        fits=results,camera_only_screws=screws,parallel_joint23_camera_fit=distance,
        urdf=dict(sha256=hashlib.sha256(xml.encode()).hexdigest(),joint23_axis_separation_mm=nominal*1000,joint2_to_joint_axis_angles_deg=axisangles),
        mathematical_counterrotation_examples=proposed,
        limits=['A full-rank nearly parallel design can still be practically unidentifiable.',
                'Any fitted translation is diagnostic only and cannot establish that hand-eye is the unique cause.',
                'Screw-axis distances from RGB are independent of a constant hand-eye, but not of printed board scale or camera geometric bias.',
                'No old/new board absolute positions are mixed. Separate within-session contrasts are used.',
                'J3 has no return; return holdout cannot validate its actual reversibility.',
                'Existing small-angle data cannot validate the 10–11 mm error over the much larger grasp trajectory.'])


def old_grasp_holdout(report):
    """Reexpress existing camera-derived planes; never refit or mix board poses."""
    source = ARCHIVE/'ros_log/grasp_diagnostic_20260928T084245Z/geometry_forensics/geometry_report.json'
    old = json.loads(source.read_text())
    xml = (ROOT/'src/real-arm/alicia_d_descriptions/urdf/alicia_duo_with_gripper.urdf').read_text()
    if hashlib.sha256(xml.encode()).hexdigest() != old['urdf_sha256']:
        raise ValueError('Old grasp URDF differs from diagnostic source')
    rotations = {f['label']:serial_fk_and_axes(xml,f['q_rad'])[0][:3,:3] for f in old['frames']}
    rows=[]
    for point,fit_result in report['fits'].items():
        delta=np.asarray(fit_result['correction_tool_mm'])/1000.
        for patch in old['common_patch_comparisons']:
            stage=patch['stage']; initial=patch['fits']['initial']; later=patch['fits'][stage]
            na=np.asarray(initial['n']);nb=np.asarray(later['n'])
            # Translating a point cloud by v changes plane offset d to d-n.v,
            # hence z at fixed x,y increases by n.v/n_z.
            adjustment=(nb@(rotations[stage]@delta)/nb[2]-na@(rotations['initial']@delta)/na[2])*1000.
            rows.append(dict(fit_point=point,stage=stage,center_xy_base_m=patch['center_xy_base_m'],
                radius_m=patch['radius_m'],old_later_minus_initial_z_mm=patch['later_minus_initial_z_mm'],
                predicted_translation_adjustment_mm=float(adjustment),
                diagnostic_corrected_difference_mm=float(patch['later_minus_initial_z_mm']+adjustment)))
    return dict(kind='historical_grasp_holdout_fixed_translation_only',source=str(source),
        source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),rows=rows,
        limitation='Diagnostic reexpression of each already-fitted visible red plane under the hypothetical fixed translation; no plane refit, no new image projection, no actual calibration or grasp correction. Initial and close common-patch comparisons only within the old grasp; no board absolute coordinates mixed.')


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--output',type=Path,required=True)
    ap.add_argument('--bootstrap',type=int,default=1000)
    ap.add_argument('--seed',type=int,default=2809)
    args=ap.parse_args()
    if not os.path.ismount(MOUNT) or MOUNT not in args.output.resolve().parents:
        raise RuntimeError('Mounted SanStand output required; no system-disk fallback')
    if not 100 <= args.bootstrap <= 10000: raise ValueError('Bootstrap count must be 100..10000')
    report=run(load(),args.bootstrap,args.seed)
    args.output.mkdir(parents=True,exist_ok=True)
    path=args.output/'fixed_translation_identifiability.json'
    path.write_text(json.dumps(report,ensure_ascii=False,indent=2,default=as_json,allow_nan=False)+'\n')
    (args.output/'analysis_source.py').write_bytes(Path(__file__).read_bytes())
    (args.output/'old_grasp_translation_holdout.json').write_text(json.dumps(old_grasp_holdout(report),ensure_ascii=False,indent=2,allow_nan=False)+'\n')
    print(json.dumps(dict(report=str(path),fits={key:{k:value[k] for k in ('rank','condition','correction_tool_mm','training','return_holdout')} for key,value in report['fits'].items()},parallel_axis=report['parallel_joint23_camera_fit'],urdf=report['urdf']),ensure_ascii=False,default=as_json))


if __name__=='__main__':main()
