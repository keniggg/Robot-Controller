#!/usr/bin/env python3
"""Finite fresh board-centering observation; default read-only, no calibration edits.

At most one one-hot command each to J2, J3, J1, from one fresh solve; no return,
retry, sync, enable, disable, or old-plan input. All existing scene/source limits
remain: 40 mm CAD-plane, 20 px full-board edge, 120 mm board depth, +/-3 degree
command, INITIAL +/-4.8 degree edited response, held .4 degree, reverse .5 degree.
"""
import argparse
from collections import OrderedDict
from datetime import datetime,timezone
import hashlib,json,math,os,signal,sys,time
from pathlib import Path
import numpy as np
HERE=Path(__file__).resolve().parent;sys.path.insert(0,str(HERE))
import bounded_wrist_response_diagnostic as wrist
import bounded_motor_centering as motor
from bounded_async_bag_writer import BoundedAsyncBagWriter
from bootstrap_mixed_geometry import certify_joint_box_geometry,projection_corner_box_batch
from coupled_joint23_geometry import words

BOARD_CENTER=np.array([.09,.0675,0.])
COMMAND_ORDER=(1,2,0)
ORIGINAL_URDF_SHA256='890592a24b8ad818d995e69beb670ea4254bf84f589615bfac3c8562f62449b7'


def require_model_identity(fk_sha, archived_sha, current_xml):
    current_sha=hashlib.sha256(current_xml.encode('utf-8')).hexdigest()
    if not fk_sha==archived_sha==current_sha==ORIGINAL_URDF_SHA256:
        raise RuntimeError('original runtime FK/archived/current URDF identity differs')
    return current_sha


def require_frozen_observation_model(frozen_x, frozen_camera, geometry):
    x=np.asarray(geometry['X'],dtype=float)
    fingerprint=geometry['model'].fingerprint
    if x.shape!=(4,4) or not np.isfinite(x).all() or not isinstance(fingerprint,str) or not fingerprint:
        raise RuntimeError('invalid hand-eye or camera model identity')
    if frozen_x is not None and (not np.allclose(x,frozen_x,rtol=0,atol=1e-10) or fingerprint!=frozen_camera):
        raise RuntimeError('hand-eye or camera model changed since first fresh snapshot')
    return x.copy(),fingerprint


def solve_board_center(fk,geometry):
    """One finite J1/J2/J3 minimum-norm solve, using only the current board."""
    from scipy.optimize import least_squares
    q=np.asarray(geometry['q'])[:6];X,Y,model=geometry['X'],geometry['Y'],geometry['model']
    world=Y[:3,:3]@BOARD_CENTER+Y[:3,3]
    def project(delta):
        C=np.linalg.inv(fk(q+np.r_[delta,np.zeros(3)])@X);p=C[:3,:3]@world+C[:3,3]
        if p[0]<=.12:raise RuntimeError('board center depth below 120 mm')
        return np.asarray(model.sdk.rs2_project_point_to_pixel(model.intrinsics,[-p[1],-p[2],p[0]]))
    initial=project(np.zeros(3));radius=float(np.linalg.norm(initial-[320.,240.]))
    if radius<=10.:
        return np.zeros(3,dtype=int),dict(initial_pixel=initial.tolist(),predicted_pixel=initial.tolist(),already_centered=True,solve_count=0)
    def residual(d):return np.r_[(project(d)-[320.,240.])/10.,d/math.radians(5.)*.12]
    def jac(d):
        h=1e-4
        return np.column_stack([(residual(d+e*h)-residual(d-e*h))/(2*h) for e in np.eye(3)])
    fit=least_squares(residual,np.zeros(3),jac=jac,bounds=(-motor.LIMIT,motor.LIMIT),max_nfev=160,ftol=1e-9,xtol=1e-9,gtol=1e-9)
    if not fit.success:raise RuntimeError('single fresh board-centering solve did not converge')
    increments=motor.quantized_deltas(fit.x);final=project(increments*motor.QUANTUM)
    if not np.any(increments) or np.linalg.norm(final-[320.,240.])>=radius-2.:
        raise RuntimeError('quantized proposal fails two-pixel board-centering improvement')
    return increments,dict(initial_pixel=initial.tolist(),continuous_delta_deg=np.degrees(fit.x).tolist(),
        quantized_delta_deg=np.degrees(increments*motor.QUANTUM).tolist(),predicted_pixel=final.tolist(),
        predicted_radius_px=float(np.linalg.norm(final-[320.,240.])),solve_count=1,jacobian_step_rad=1e-4,
        optimizer_cost=float(fit.cost),command_order=[i+1 for i in COMMAND_ORDER],automatic_retry=False)


def board_contract(measured,reference,epoch,increments):
    """Reuse the exact reviewed contract, with one declared J2/J3/J1 ordering."""
    contract=motor.CenteringContract(measured,reference,epoch,increments)
    contract.indices=[i for i in COMMAND_ORDER if increments[i]!=0]
    contract.labels=['center_J%d'%(i+1) for i in contract.indices]
    contract.targets=[];target=contract.reference.copy()
    for i in contract.indices:
        target=target.copy();target[i]+=int(increments[i])*motor.QUANTUM
        if np.any(abs(target[:6]-contract.origin[:6])>motor.ABSOLUTE):
            raise RuntimeError('reordered nominal target escapes unchanged initial absolute bounds')
        contract.targets.append(target)
    return contract


def prove_board_path(fk,gripper,contract,index,measured,target,reference,geometry,lo,hi,
                     corner_function,stage_function,guard=lambda:None,valid_state=None):
    """Entire independent directional physical box and separate nominal box."""
    measured,reference,target=map(wrist.vector,(measured,reference,target))
    lower,upper,directions=contract.rectangle(measured,target,reference,index)
    endpoints=np.vstack((measured[:6],reference[:6],target[:6]));bl,bu=endpoints.min(0),endpoints.max(0)
    if np.any(lower<lo) or np.any(upper>hi) or np.any(bl<lo) or np.any(bu>hi):
        raise RuntimeError('physical or nominal box exceeds unchanged joint limits')
    proofs=[];counts=[];visited=set();maximum_distance=0.
    openings=sorted(set(contract.gripper_bounds))
    def check(q,opening):
        guard();key=tuple(q)+(opening,)
        if key not in visited:
            if valid_state is not None:valid_state(q,opening)
            visited.add(key)
    for a,b in ((lower,upper),(bl,bu)):
        count=0
        for center,half in motor.joint_cells(a,b):
            guard();count+=1
            for opening in openings:
                proof=certify_joint_box_geometry(fk,gripper,np.r_[center,opening],geometry,lo,hi,half)
                proofs.append(proof)
                local=np.concatenate([corner_function(c,np.eye(3),size) for _,c,size in stage_function(np.eye(4),gripper,opening,'y','z')])
                T=fk(center);world=local@T[:3,:3].T+T[:3,3]
                radius=np.array([np.dot(fk.corner_motion_radii(p),half) for p in local])
                maximum_distance=max(maximum_distance,float(np.max(np.linalg.norm(world-geometry['Y'][:3,3],axis=1)+radius)))
                check(center,opening)
        counts.append(count)
    for first,last in ((measured[:6],target[:6]),(measured[:6],reference[:6]),(reference[:6],target[:6])):
        for fraction in np.linspace(0.,1.,17):
            for opening in openings:check(first*(1-fraction)+last*fraction,opening)
    batch=(np.asarray([p['camera_coordinate_taylor_bound']['corner_lower_xyz_m'] for p in proofs]),
           np.asarray([p['camera_coordinate_taylor_bound']['corner_upper_xyz_m'] for p in proofs]))
    result=dict(lower_rad=lower.tolist(),upper_rad=upper.tolist(),directions=directions.tolist(),
        physical_response_cells=counts[0],nominal_sdk_bridge_cells=counts[1],continuous_cell_proofs=len(proofs),
        moveit_samples=len(visited) if valid_state is not None else 0,offline_geometric_sample_count=len(visited),
        sdk_bridge_does_not_expand_feedback_rectangle=True,nominal_sdk_bridge_independent_box=True,
        nominal_sdk_bridge_lower_rad=bl.tolist(),nominal_sdk_bridge_upper_rad=bu.tolist(),
        maximum_cad_distance_to_board_origin_m=maximum_distance,
        minimum_plane_clearance_lower_bound_m=min(p['minimum_plane_clearance_lower_bound_m'] for p in proofs),
        minimum_full_board_edge_lower_bound_px=min(p['minimum_full_board_edge_lower_bound_px'] for p in proofs),
        minimum_board_depth_lower_bound_m=min(p['minimum_board_depth_lower_bound_m'] for p in proofs),
        frozen_gripper_opening_interval_m=list(contract.gripper_bounds),physical_clearance_requirement_m=.04,
        full_board_edge_requirement_px=20.,board_depth_requirement_m=.12,
        hardware_tracking_or_stopping_certified=False,whole_arm_continuous_collision_certified=False)
    result['whole_rectangle_scene_proof']=wrist.verify_fresh_scene(result,geometry['Y'],geometry['Y'],geometry['model'],projection_corner_box_batch,batch)
    return result,batch


def make_diagnostic_class(base):
    parent=wrist.make_diagnostic_class(base)
    reviewed_motor=motor.make_diagnostic_class(base)
    class Diagnostic(parent):
        # These exact reviewed methods do not contain super() or motor semantics.
        write_bag=reviewed_motor.write_bag
        close=reviewed_motor.close
        def __init__(self,args,directory):
            self.depth=OrderedDict();self.receipts=OrderedDict();self.receive_active=0;self.writer_summary=None
            self.frozen_observation_x=None;self.frozen_observation_camera=None
            def archive_failure(reason):
                with self.pair_changed:
                    self.fault='board-centering archive: '+str(reason);self.pair_changed.notify_all()
            def write_raw(metadata,payload,pytype):
                ns=metadata['stamp_ns'];stamp=base.rospy.Time(ns//10**9,ns%10**9)
                self.bag.write(metadata['topic'],(metadata['datatype'],payload,metadata['md5sum'],pytype),stamp,raw=True)
            self.archive_writer=BoundedAsyncBagWriter(write_raw,archive_failure)
            super().__init__(args,directory);self.deadline=time.monotonic()+180.
            self.archived_urdf_sha256=hashlib.sha256((directory/'robot_description.urdf').read_bytes()).hexdigest()
            self.check_model_identity()
        def check_model_identity(self):
            return require_model_identity(self.fk.model_sha256,self.archived_urdf_sha256,base.rospy.get_param('/robot_description'))
        def guard(self):
            self.archive_writer.require_healthy();super().guard();self.archive_writer.require_healthy()
        def receive(self,topic,msg):
            received=base.rospy.Time.now().to_nsec()
            with self.pair_changed:
                if self.closed:return
                self.receive_active+=1
            try:
                with self.lock:
                    stamps=[]
                    if topic in (motor.DEPTH,base.RGB):stamps=[msg.header.stamp.to_nsec()]
                    elif topic=='/charuco/quality':stamps=[json.loads(msg.data)['source_stamp_ns']]
                    elif topic=='/tf':stamps=[t.header.stamp.to_nsec() for t in msg.transforms if t.header.frame_id=='camera_link']
                    for ns in stamps:self.cache(self.receipts,(topic,ns),received,400)
                    if topic==motor.DEPTH:self.cache(self.depth,msg.header.stamp.to_nsec(),msg,12)
                return super().receive(topic,msg)
            except BaseException as error:
                self.archive_writer.latch_failure('board-centering callback: '+str(error))
            finally:
                with self.pair_changed:self.receive_active-=1;self.pair_changed.notify_all()
        def snapshot(self,label):
            self.check_model_identity()
            deadline=min(self.deadline,time.monotonic()+4.)
            while True:
                g=super().snapshot(label);ns=g['source_stamp_ns']
                with self.lock:
                    depth=self.depth.get(ns);rgb=self.rgb.get(ns)
                    receipt={t:self.receipts.get((t,ns)) for t in (motor.DEPTH,base.RGB,'/charuco/quality','/tf')}
                if depth is not None and rgb is not None:break
                if time.monotonic()>=deadline:raise RuntimeError('fresh exact board RGB/Z16 tuple unavailable')
                with self.pair_changed:self.pair_changed.wait(timeout=.02)
            if any(r is None or not 0<=r-ns<=350_000_000 for r in receipt.values()):
                raise RuntimeError('board RGB/Z16/quality/TF receipt is missing, delayed or future')
            if depth.encoding not in ('16UC1','mono16') or (depth.width,depth.height)!=(rgb.width,rgb.height) or depth.header.frame_id!=rgb.header.frame_id:
                raise RuntimeError('exact aligned Z16/RGB dimensions or frame invalid')
            self.frozen_observation_x,self.frozen_observation_camera=require_frozen_observation_model(
                self.frozen_observation_x,self.frozen_observation_camera,g)
            camera=np.linalg.inv(self.fk(g['q'][:6])@g['X']);world=g['Y'][:3,:3]@BOARD_CENTER+g['Y'][:3,3];p=camera[:3,:3]@world+camera[:3,3]
            pixel=np.asarray(g['model'].sdk.rs2_project_point_to_pixel(g['model'].intrinsics,[-p[1],-p[2],p[0]]))
            g['board_observation']=dict(point_board_m=BOARD_CENTER.tolist(),observed_pixel=pixel.tolist(),radius_px=float(np.linalg.norm(pixel-[320.,240.])),source_stamp_ns=ns,receipts_ns=receipt)
            self.write_bag('/bounded_board/snapshot_depth',depth,base.rospy.Time.now())
            base.write_json(self.out/(label+'_board_observation.json'),g['board_observation'])
            with self.lock:
                self.guard();wrist.validate_source_binding(base.rospy.Time.now().to_nsec(),self.frozen_epoch,
                    self.accepted[-1].header.stamp.to_nsec(),self.reference[-1].header.stamp.to_nsec(),ns)
            return g
        def path(self,measured,target,geometry,reference=None):
            reference=base.positions(self.reference[-1]) if reference is None else wrist.vector(reference)
            def valid(q,opening):
                request=base.GetStateValidityRequest(group_name='alicia');request.robot_state=base.RobotState()
                request.robot_state.joint_state=base.JointState(name=base.NAMES,position=list(q)+[opening])
                if not self.validity(request).valid:raise RuntimeError('MoveIt rejects complete response cells or nominal SDK bridge')
            result,batch=prove_board_path(self.fk,self.gripper,self.contract,self.args.joint-1,measured,target,reference,geometry,
                self.lo,self.hi,base._box_corners,base._stage_boxes,self.guard,valid)
            self.path_sequence+=1;archive=self.out/('continuous_cells_%03d.npz'%self.path_sequence)
            np.savez_compressed(archive,camera_lower=batch[0],camera_upper=batch[1]);result.update(cell_enclosure_archive=str(archive),cell_enclosure_sha256=hashlib.sha256(archive.read_bytes()).hexdigest())
            self.geometry_batches[str(archive)]=batch
            return result
        def run(self):
            self.await_initial()
            self.stable_hold_recording(3.)
            geometry=self.snapshot('baseline')
            with self.lock:
                self.guard();measured=base.positions(self.accepted[-1]);reference=base.positions(self.reference[-1])
                if np.max(abs(measured[:6]-geometry['q'][:6]))>math.radians(.15):raise RuntimeError('fresh board image differs from settled start')
                geometry['q']=measured.copy()
            increments,proposal=solve_board_center(self.fk,geometry)
            with self.lock:
                self.guard()
                if np.max(abs(base.positions(self.accepted[-1])[:6]-measured[:6]))>math.radians(.15):raise RuntimeError('measured pose changed during one-time solve')
                wrist.validate_source_binding(base.rospy.Time.now().to_nsec(),self.frozen_epoch,self.accepted[-1].header.stamp.to_nsec(),self.reference[-1].header.stamp.to_nsec(),geometry['source_stamp_ns'])
                self.contract=board_contract(measured,reference,self.frozen_epoch,increments)
            plan=dict(schema=1,execute=self.args.execute,maximum_commands=len(self.contract.indices),initial_measured_rad=measured.tolist(),starting_sdk_rad=reference.tolist(),frozen_epoch_ns=self.frozen_epoch,
                source_stamp_ns=geometry['source_stamp_ns'],board_observation=geometry['board_observation'],proposal=proposal,baseline_recording_seconds=3.,
                robot_description_sha256=self.archived_urdf_sha256,frozen_T_tool_camera=self.frozen_observation_x.tolist(),camera_model_fingerprint=self.frozen_observation_camera,
                commands=[dict(label=l,joint=i+1,target_rad=t.tolist(),words=words(t)) for l,i,t in zip(self.contract.labels,self.contract.indices,self.contract.targets)],
                production_target_identity_verified=False,clears_grasp_fault=False,bootstrap_pending=False,automatic_return=False,automatic_retry=False,
                source='fresh board geometry only; no motor evidence or archived action target',archive_policy='local_only',upload_enabled=False,delete_enabled=False)
            base.write_json(self.out/'plan.json',plan)
            if not self.args.execute:
                if self.contract.indices:
                    self.args.joint=self.contract.indices[0]+1;plan['first_step_geometry']=self.path(measured,self.contract.targets[0],geometry,reference);base.write_json(self.out/'plan.json',plan)
                self.event('prepared_no_motion',plan=str(self.out/'plan.json'));return
            observed=geometry
            for i,label,target in zip(self.contract.indices,self.contract.labels,self.contract.targets):
                self.args.joint=i+1;observed=self.execute_step(label,target)
            self.event('finite_board_observation_complete',commands_published=self.publish_count,actual_latest_board_observation=observed['board_observation'],automatic_retry=False,automatic_return=False)
    return Diagnostic


def parse_args(argv=None):
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--execute',action='store_true');a=p.parse_args(argv)
    a.bootstrap_pending=False;a.bootstrap_sync_bound_deg=4.8;a.joint=2
    return a


def main():
    args=parse_args()
    from scipy.optimize import least_squares as _solver_import_check
    import pyrealsense2 as _sdk_import_check
    if not os.path.ismount(wrist.MOUNT) or not wrist.ARCHIVE.is_dir():raise RuntimeError('SanStand unavailable; no fallback')
    out=wrist.ARCHIVE/('bounded_board_centering_'+datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')+'_'+str(os.getpid()));out.mkdir();(out/'ros').mkdir();os.environ['ROS_LOG_DIR']=str(out/'ros')
    manifest={}
    for name in ('bounded_board_centering.py','bounded_motor_centering.py','bounded_wrist_response_diagnostic.py','bounded_joint_response_diagnostic.py','bounded_async_bag_writer.py','bootstrap_mixed_geometry.py','coupled_joint23_geometry.py'):
        data=(HERE/name).read_bytes();(out/name).write_bytes(data);manifest[name]=hashlib.sha256(data).hexdigest()
    (out/'source_sha256.json').write_text(json.dumps(manifest,indent=2)+'\n')
    base=motor.load_module('bounded_board_base',HERE/'bounded_joint_response_diagnostic.py');base.rospy.init_node('bounded_board_centering',anonymous=True,disable_signals=True)
    terminal=dict(status='failed',execute=args.execute,commands_published=0);experiment=None
    def alarm(*_):raise RuntimeError('180-second finite board observation deadline exhausted')
    signal.signal(signal.SIGALRM,alarm);signal.setitimer(signal.ITIMER_REAL,180.)
    try:
        experiment=make_diagnostic_class(base)(args,out);experiment.run();terminal['status']='completed' if args.execute else 'prepared_no_motion'
    except BaseException as error:
        terminal['reason']=str(error)
        if experiment is not None:experiment.event('no_further_commands',reason=str(error),recovery_attempted=False,stop_or_disable_sent=False)
        raise
    finally:
        signal.setitimer(signal.ITIMER_REAL,0)
        if experiment is not None:
            terminal['commands_published']=experiment.publish_count
            try:experiment.close()
            except BaseException as error:terminal.update(status='failed',reason='archive close failed: '+str(error))
            terminal['archive_writer']=experiment.writer_summary or experiment.archive_writer.summary()
            if not terminal['archive_writer']['archive_complete']:terminal.update(status='failed',reason='archive incomplete')
        base.write_json(out/'terminal.json',terminal);print('SESSION '+str(out),flush=True)
    if terminal['status']=='failed':raise RuntimeError(terminal.get('reason','archive failed'))
if __name__=='__main__':main()
