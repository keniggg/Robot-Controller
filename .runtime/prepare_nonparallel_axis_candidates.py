#!/usr/bin/env python3
"""Fresh read-only J4/J6 candidate audit. No command publisher or execute option.

Collects one current stationary RGB/quality/TF/SDK reference, closes recording,
then evaluates pure geometry offline. Every result expires as an execution
credential immediately: an executor must independently reacquire and replan.
Public bounded diagnostic and geometry helpers are imported, never modified.
"""
import importlib.util
import json
import math
import os
from pathlib import Path
import signal
from types import SimpleNamespace
from datetime import datetime,timezone
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
MOUNT=Path('/media/zhuyupei/SanStand')
ARCHIVE=MOUNT/'实验记录/Robot-Controller/ros_log'

def module(name,path):
    spec=importlib.util.spec_from_file_location(name,path);m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m);return m

BASE=module('nonparallel_readonly_base',ROOT/'.runtime/bounded_joint_response_diagnostic.py')
MATH=module('nonparallel_geometry_math',ROOT/'.runtime/analyze_fixed_handeye_translation.py')


def candidate_targets(reference,joint,step_deg=2.):
    if joint not in (4,6) or step_deg!=2.:raise ValueError('only J4/J6 exact 2-degree bound')
    reference=np.asarray(reference,dtype=float)
    if reference.shape!=(7,) or not np.isfinite(reference).all():raise ValueError('finite seven-value SDK reference required')
    count=int(math.floor(math.radians(step_deg)/BASE.QUANTUM))
    targets=[]
    for sign in (1,-1):
        target=reference.copy();target[joint-1]+=sign*count*BASE.QUANTUM
        difference=np.array(BASE.wire_words(target))-np.array(BASE.wire_words(reference))
        expected=np.zeros(6,dtype=int);expected[joint-1]=sign*count
        if not np.array_equal(difference,expected) or target[6]!=reference[6]:raise ValueError('held command word/gripper changed')
        targets.append((sign,target))
    return targets


def geometry_sample(fk,gripper,q,opening,geometry):
    tool=fk(q);x=geometry['X'];y=geometry['Y'];model=geometry['model']
    normal=y[:3,2].copy()
    if normal[2]<0:normal=-normal
    offset=-float(normal@y[:3,3])
    corners=np.array([[xx,yy,0.,1.] for xx in (0.,.18) for yy in (0.,.135)])
    clearance=min(float(np.min(BASE._box_corners(c,tool[:3,:3],size)@normal+offset))
                  for _,c,size in BASE._stage_boxes(tool,gripper,float(opening),'y','z'))
    camera=(np.linalg.inv(tool@x)@y@corners.T).T[:,:3]
    forward=float(np.min(camera[:,0]))
    optical=np.column_stack([-camera[:,1],-camera[:,2],camera[:,0]])
    if forward<=0.:raise ValueError('board behind camera')
    uv=np.asarray([model.sdk.rs2_project_point_to_pixel(model.intrinsics,p.tolist()) for p in optical])
    ci=model.evidence['color_intrinsics']
    edge=float(min(uv[:,0].min(),ci['width']-1-uv[:,0].max(),uv[:,1].min(),ci['height']-1-uv[:,1].max()))
    return dict(plane_m=clearance,edge_px=edge,forward_m=forward,
                pass_geometry=bool(clearance>=.04 and edge>=20 and forward>=.12))


def evaluate(fk,gripper,measured,reference,geometry,lower,upper,xml):
    measured=np.asarray(measured);reference=np.asarray(reference)
    tool0,axes=MATH.serial_fk_and_axes(xml,measured[:6]);result=[]
    openings=sorted(set((float(measured[6]),float(reference[6]))))
    def sample(q):
        if np.any(q<lower) or np.any(q>upper):raise ValueError('joint limits exceeded')
        rows=[geometry_sample(fk,gripper,q,g,geometry) for g in openings]
        return dict(q_rad=q.tolist(),plane_m=min(r['plane_m'] for r in rows),edge_px=min(r['edge_px'] for r in rows),
                    forward_m=min(r['forward_m'] for r in rows),pass_geometry=all(r['pass_geometry'] for r in rows))
    def summarize(rows):
        return dict(samples=len(rows),minimum_plane_clearance_m=min(r['plane_m'] for r in rows),
                    minimum_full_board_edge_px=min(r['edge_px'] for r in rows),minimum_forward_m=min(r['forward_m'] for r in rows),
                    all_samples_pass=all(r['pass_geometry'] for r in rows),
                    worst_board_sample=min(rows,key=lambda row:row['edge_px']))
    for joint in (4,6):
        index=joint-1;absolute=[measured[index]-math.radians(4.8),measured[index]+math.radians(4.8)]
        reserved=[]
        for bridge in (0.,.5,1.):
            held=measured[:6]*(1-bridge)+reference[:6]*bridge
            for angle in np.linspace(*absolute,65):
                q=held.copy();q[index]=angle;reserved.append(sample(q))
        reserve=summarize(reserved)
        angle=math.degrees(math.acos(float(np.clip(abs(axes[2]['direction']@axes[joint]['direction']),0,1))))
        designs={}
        for axis_group in ([2,3,5],[2,joint],[2,3,5,joint]):
            matrices=[]
            for j in axis_group:
                q=measured[:6].copy();q[j-1]+=22*BASE.QUANTUM
                matrices.append(fk(q)[:3,:3]-tool0[:3,:3])
            ss=np.linalg.svd(np.vstack(matrices),compute_uv=False)
            designs['+'.join('J%d'%j for j in axis_group)]=dict(singular_values=ss.tolist(),rank=int(np.sum(ss>1e-10)),condition=float(ss[0]/ss[-1]) if ss[-1]>1e-12 else None)
        for sign,target in candidate_targets(reference,joint):
            nominal=[]
            # Initial physical pose -> outbound SDK, outbound SDK -> original SDK.
            # Both include actual/SDK bridge rather than pretending reference==actual.
            for first,last in ((measured[:6],target[:6]),(target[:6],reference[:6])):
                for alpha in np.linspace(0,1,33):nominal.append(sample(first*(1-alpha)+last*alpha))
            after=fk(target[:6]);relative=np.linalg.inv(tool0)@after
            pure=measured[:6].copy();pure[index]+=sign*22*BASE.QUANTUM;puretool=fk(pure)
            result.append(dict(joint=joint,direction=sign,requested_sdk_step_deg=math.degrees(22*BASE.QUANTUM),
                fixed_absolute_response_rad=absolute,target_inside_fixed_absolute_interval=bool(absolute[0]<=target[index]<=absolute[1]),
                sdk_target_rad=target.tolist(),return_sdk_rad=reference.tolist(),nominal=summarize(nominal),fixed_reservation=reserve,
                joint2_axis_angle_deg=angle,local_translation_observability=designs,
                measured_to_sdk_target_tool_displacement_mm=float(np.linalg.norm(after[:3,3]-tool0[:3,3])*1000),
                measured_to_sdk_target_rotation_deg=math.degrees(np.linalg.norm(MATH.log_so3(relative[:3,:3]))),
                theoretical_single_axis_tool_displacement_mm=float(np.linalg.norm(puretool[:3,3]-tool0[:3,3])*1000)))
    return result


def main():
    if not os.path.ismount(MOUNT) or not ARCHIVE.is_dir():raise RuntimeError('mounted SanStand required')
    directory=ARCHIVE/('nonparallel_axis_readonly_'+datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')+'_'+str(os.getpid()))
    directory.mkdir();(directory/'ros').mkdir();os.environ['ROS_LOG_DIR']=str(directory/'ros')
    (directory/'source.py').write_bytes(Path(__file__).read_bytes())
    BASE.rospy.init_node('nonparallel_axis_readonly',anonymous=True,disable_signals=True)
    signal.signal(signal.SIGALRM,lambda *_:(_ for _ in ()).throw(RuntimeError('readonly audit deadline exhausted')))
    signal.setitimer(signal.ITIMER_REAL,50.)
    # Literal False and no CLI execution flag; the base creates no publisher.
    args=SimpleNamespace(execute=False,bootstrap_pending=False,bootstrap_sync_bound_deg=4.8,joint=4,step_deg=2.)
    experiment=None;terminal=dict(execute=False,commands_authorized=0,status='failed')
    try:
        experiment=BASE.Diagnostic(args,directory)
        if experiment.publisher is not None:raise RuntimeError('read-only process unexpectedly has a publisher')
        reference,stationary=experiment.await_initial();geometry=experiment.snapshot('current_readonly')
        with experiment.lock:
            experiment.guard();measured=BASE.positions(experiment.accepted[-1])
            if np.max(np.abs(measured[:6]-geometry['q'][:6]))>math.radians(.15):raise RuntimeError('pose moved during readonly snapshot')
            epoch=experiment.frozen_epoch;status=experiment.state['status']
        experiment.close()
        candidates=evaluate(experiment.fk,experiment.gripper,measured,reference,geometry,experiment.lo,experiment.hi,
                            (directory/'robot_description.urdf').read_text())
        report=dict(kind='fresh_snapshot_offline_nonparallel_axis_candidates',execute=False,commands_authorized=0,
            frozen_epoch_ns=epoch,status_at_capture=status,source_stamp_ns=geometry['source_stamp_ns'],stationary=stationary,
            measured_rad=measured.tolist(),held_sdk_rad=reference.tolist(),candidates=candidates,
            limits=['Fresh at acquisition only; no saved result is an execution authorization.',
                'Sampled geometry is not a continuous motion or whole-arm collision proof. No MoveIt service was called in this offline calculation.',
                'Fixed +/-4.8-degree edited-axis bounds stay centered on the initial measured pose for return.',
                'Three held-axis bridge states are sampled; independent held-axis tracking errors are not certified.',
                'Execution must start from fresh CONFIRMED state with no bootstrap/sync/enable, exactly outbound then original SDK, no trim.'])
        BASE.write_json(directory/'candidates.json',report)
        terminal.update(status='prepared_no_motion',source_stamp_ns=geometry['source_stamp_ns'],candidate_count=len(candidates))
        print(json.dumps(dict(directory=str(directory),candidates=[dict(joint=c['joint'],direction=c['direction'],nominal=c['nominal'],reserve=c['fixed_reservation'],axis_angle=c['joint2_axis_angle_deg'],observability=c['local_translation_observability']) for c in candidates]),ensure_ascii=False))
    except BaseException as error:
        terminal['reason']=str(error);print(json.dumps(dict(directory=str(directory),terminal=terminal),ensure_ascii=False))
    finally:
        if experiment is not None and not experiment.closed:experiment.close()
        signal.setitimer(signal.ITIMER_REAL,0.)
        BASE.write_json(directory/'terminal.json',terminal)
    if terminal['status']!='prepared_no_motion':raise SystemExit(2)

if __name__=='__main__':main()
