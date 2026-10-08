#!/usr/bin/env python3
"""Explicit fresh one-shot return to one closed experiment's ORIGINAL SDK.

Default is read-only. --execute permits at most J2 then J3, each one-hot with
written SDK and independent stationary evidence. No retry/recovery/enable/disable.
Parent records supply only identity, old absolute origin and exact goal; every
path, camera scene, response box and MoveIt check is recomputed in this process.
"""
import argparse
from datetime import datetime,timezone
import hashlib
import importlib.util
import itertools
import json
import math
import os
from pathlib import Path
import signal
import sys
import time
import numpy as np
HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(HERE))
import bounded_joint23_counterrotation as cr

PARENT_FILES=('plan.json','terminal.json','events.jsonl','baseline_geometry.json',
              'positive_post_hold_geometry.json','source_sha256.json','robot_description.urdf')

def load_parent(directory, expected_digest=None):
    directory=Path(directory).resolve()
    if directory.parent!=cr.ARCHIVE.resolve() or not directory.name.startswith('bounded_joint23_counterrotation_'):
        raise RuntimeError('closed counterrotation archive parent required')
    raw={name:(directory/name).read_bytes() for name in PARENT_FILES}
    hashes={name:hashlib.sha256(data).hexdigest() for name,data in raw.items()}
    digest=hashlib.sha256(json.dumps(hashes,sort_keys=True,separators=(',',':')).encode()).hexdigest()
    if expected_digest is not None and expected_digest!=digest:
        raise RuntimeError('parent digest changed; no return')
    plan=json.loads(raw['plan.json']);terminal=json.loads(raw['terminal.json'])
    events=[json.loads(line) for line in raw['events.jsonl'].decode().splitlines() if line]
    commands=[e for e in events if e.get('event')=='command']
    confirmations=[e for e in events if e.get('event')=='one_hot_sdk_confirmed_and_stationary']
    if (plan.get('schema')!=2 or plan.get('maximum_commands')!=4
            or terminal.get('status')!='failed' or terminal.get('commands_published')!=2
            or [e.get('edited_joint') for e in commands]!=[2,3]
            or [e.get('one_hot_part') for e in commands]!=[1,2]
            or [e.get('edited_joint') for e in confirmations]!=[2,3]
            or any(e.get('label')!='positive' for e in commands+confirmations)
            or not any(e.get('event')=='post_hold_observation' and e.get('label')=='positive' for e in events)):
        raise RuntimeError('parent is not exactly confirmed positive J2/J3 with zero return commands')
    origin=cr.vector(plan['initial_measured_rad']);goal=cr.vector(plan['starting_sdk_rad'])
    prior=cr.vector(commands[-1]['target_rad']);actual=cr.vector(confirmations[-1]['measured_rad'])
    delta=np.array(cr.words(prior))-cr.words(goal)
    count=plan.get('step_words')
    if (type(count) is not int or count != 22
            or delta.tolist()!=[0,count,-count,0,0,0]
            or confirmations[-1]['actual_sdk_words']!=cr.words(prior)
            or not np.array_equal(goal[list(cr.HELD)+[6]],prior[list(cr.HELD)+[6]])):
        raise RuntimeError('parent SDK pair/held-channel identity invalid')
    first=goal.copy();first[1]=prior[1]
    if (cr.words(cr.vector(commands[0]['target_rad']))!=cr.words(first)
            or confirmations[0]['actual_sdk_words']!=cr.words(first)):
        raise RuntimeError('parent first one-hot SDK predecessor invalid')
    epoch=plan.get('frozen_epoch_ns')
    if type(epoch) is not int or epoch<=0:
        raise RuntimeError('parent frozen epoch invalid')
    return dict(directory=str(directory),digest=digest,hashes=hashes,origin=origin,
        original_sdk=goal,expected_current_sdk=prior,expected_actual=actual,epoch=epoch,
        baseline=json.loads(raw['baseline_geometry.json']),
        post_hold=json.loads(raw['positive_post_hold_geometry.json']))

class ReturnContract(cr.CounterrotationContract):
    def __init__(self,parent,measured,reference,epoch):
        if epoch!=parent['epoch']:
            raise RuntimeError('parent epoch changed; no explicit return')
        if cr.words(reference)!=cr.words(parent['expected_current_sdk']) or reference[6]!=parent['original_sdk'][6]:
            raise RuntimeError('current SDK differs from parent confirmed endpoint')
        if np.max(abs(cr.vector(measured)[:6]-parent['expected_actual'][:6]))>math.radians(.15):
            raise RuntimeError('actual endpoint moved since parent confirmation')
        # Original origin remains authoritative, never recentered to current q.
        super().__init__(parent['origin'],parent['original_sdk'],epoch,2.,positive_only=True)
        self.original_sdk=parent['original_sdk'].copy()
        self.reference=cr.vector(reference)
        self.targets=(self.original_sdk.copy(),)
        self.labels=('explicit_original_sdk_return',)
        self.maximum_commands=2
        self.expected_words=cr.words(self.reference)
        self.assert_feedback(measured,epoch)


def partition_cell(lower,upper,prove,inspect,guard,budget,depth=0):
    """Only refine failed certificate cells; exact failing samples reject early."""
    guard();budget['checks']+=1
    if budget['checks']>budget['maximum_checks']:
        raise RuntimeError('bounded return cell proof budget exhausted')
    midpoint=.5*(lower+upper);radius=max(float(.5*np.sum(upper-lower)),1e-12)
    try:
        prove(midpoint,radius,.5*(upper-lower))
        yield midpoint,radius
        return
    except RuntimeError as error:
        if not str(error).startswith(('continuous mixed initial sync violates','continuous explicit joint box violates')):
            raise
        sample=inspect(midpoint,radius,.5*(upper-lower))
        budget['rejected_cells']+=1
        if len(budget['diagnostics'])<12:
            budget['diagnostics'].append(dict(depth=depth,reason=str(error),**sample))
        if (sample['exact_center_edge_px']<20. or sample['exact_center_plane_clearance_m']<.04
                or sample['exact_center_depth_m']<.12):
            raise RuntimeError('exact return cell center violates unchanged 40mm/20px/120mm; no subdivision')
        if depth>=4:
            raise RuntimeError('return geometry unresolved at bounded four-level subdivision')
    axis=int(np.argmax(upper-lower));middle=midpoint[axis]
    left_upper=upper.copy();left_upper[axis]=middle
    right_lower=lower.copy();right_lower[axis]=middle
    yield from partition_cell(lower,left_upper,prove,inspect,guard,budget,depth+1)
    yield from partition_cell(right_lower,upper,prove,inspect,guard,budget,depth+1)


def make_diagnostic_class(base):
    Parent=cr.make_diagnostic_class(base)
    class Diagnostic(Parent):
        def path_cell_geometry(self,midpoint,radius,geometry,opening):
            key=(tuple(midpoint),float(radius),float(opening))
            if key not in self.return_cell_cache:
                raise RuntimeError('explicit joint box certificate was not prepared for this leaf')
            return self.return_cell_cache[key]

        def path_cell_partition(self,axes,geometry,openings):
            from bootstrap_mixed_geometry import board_camera_box_taylor_bound,certify_joint_box_geometry
            model=geometry['model'];ci=model.evidence['color_intrinsics']
            board=np.array([[u,v,0.,1.] for u in np.linspace(0.,.18,11) for v in np.linspace(0.,.135,9)])
            normal=geometry['Y'][:3,2].copy()
            if normal[2]<0:normal=-normal
            offset=-normal@geometry['Y'][:3,3]
            def prove(midpoint,radius,half):
                # Always prove this exact cell: equal center/L1 radius does not
                # imply equal axis widths. Publish the cache only after ALL
                # gripper openings pass; caller consumes it immediately on yield.
                fresh={}
                for opening in openings:
                    key=(tuple(midpoint),float(radius),float(opening))
                    fresh[key]=certify_joint_box_geometry(self.fk,self.gripper,
                        np.r_[midpoint,opening],geometry,self.lo,self.hi,half)
                self.return_cell_cache.update(fresh)
            def inspect(midpoint,radius,half):
                tool=self.fk(midpoint);camera=(np.linalg.inv(tool@geometry['X'])@geometry['Y']@board.T).T[:,:3]
                uv=np.array([model.sdk.rs2_project_point_to_pixel(model.intrinsics,[-p[1],-p[2],p[0]]) for p in camera])
                edge=float(min(uv[:,0].min(),ci['width']-1-uv[:,0].max(),uv[:,1].min(),ci['height']-1-uv[:,1].max()))
                plane=min(float(np.min(base._box_corners(c,tool[:3,:3],size)@normal+offset))
                    for opening in openings for _,c,size in base._stage_boxes(tool,self.gripper,opening,'y','z'))
                t=board_camera_box_taylor_bound(self.fk,midpoint,geometry['X'],geometry['Y'],half)
                width=np.asarray(t['corner_upper_xyz_m'])-np.asarray(t['corner_lower_xyz_m'])
                return dict(center_rad=midpoint.tolist(),exact_center_edge_px=edge,
                    exact_center_plane_clearance_m=plane,exact_center_depth_m=float(camera[:,0].min()),
                    corner_coordinate_width_m=width.tolist())
            for indices in itertools.product(*(range(len(axis)-1) for axis in axes)):
                lower=np.array([axis[j] for axis,j in zip(axes,indices)])
                upper=np.array([axis[j+1] for axis,j in zip(axes,indices)])
                yield from partition_cell(lower,upper,prove,inspect,self.guard,self.return_partition)

        def path(self,*args,**kwargs):
            self.return_cell_cache={}
            self.return_partition=dict(checks=0,maximum_checks=6000,rejected_cells=0,diagnostics=[])
            try:
                result=super().path(*args,**kwargs)
                result['bounded_failed_cell_subdivision']=self.return_partition
                result['cell_geometry_domain']='explicit_six_axis_half_widths_physical_and_nominal_bridge'
                return result
            finally:
                base.write_json(self.out/'return_cell_partition_audit.json',self.return_partition)

        def run(self):
            reference,stationary=self.await_initial()
            geometry=self.snapshot('fresh_return_baseline')
            parent=self.args.parent_identity
            if self.fk.model_sha256!=parent['hashes']['robot_description.urdf']:
                raise RuntimeError('URDF changed since parent experiment')
            if (geometry['model'].fingerprint!=parent['baseline']['quality']['camera_model_fingerprint']
                    or not np.allclose(geometry['X'],parent['baseline']['T_tool_camera'],atol=1e-10,rtol=0)):
                raise RuntimeError('camera model or handeye changed since parent experiment')
            previous=np.asarray(parent['post_hold']['T_base_board'])
            if (np.linalg.norm(geometry['Y'][:3,3]-previous[:3,3])>.003
                    or cr.rotation_angle(geometry['Y'][:3,:3].T@previous[:3,:3])>math.radians(.5)):
                raise RuntimeError('board scene changed since parent endpoint')
            with self.lock:
                self.guard();measured=base.positions(self.accepted[-1])
                self.contract=ReturnContract(parent,measured,reference,self.frozen_epoch)
                self.expected_words=cr.words(reference)
            goal=self.contract.original_sdk
            record=dict(schema=1,kind='explicit_fresh_one_shot_original_sdk_return',execute=self.args.execute,
                parent_directory=parent['directory'],parent_digest=parent['digest'],parent_file_hashes=parent['hashes'],
                initial_absolute_origin_rad=self.contract.origin.tolist(),fresh_starting_actual_rad=measured.tolist(),
                fresh_starting_sdk_rad=reference.tolist(),original_sdk_goal_rad=goal.tolist(),
                original_sdk_goal_words=cr.words(goal),frozen_epoch_ns=self.frozen_epoch,maximum_commands=2,
                one_hot_joint_order=[2,3],stationary=stationary,old_path_or_geometry_reused=False,
                no_retry_no_automatic_recovery=True,archive_policy='local_only',upload_enabled=False,delete_enabled=False)
            base.write_json(self.out/'return_identity.json',record)
            if not self.args.execute:
                details=self.path(measured,goal,geometry,reference)
                base.write_json(self.out/'fresh_return_prepare.json',details)
                self.event('prepared_no_motion',**record,**details)
                return
            self.execute_step(self.contract.labels[0],goal)
            self.event('completed_original_sdk_return',maximum_commands=2,commands_published=self.publish_count,
                original_words=cr.words(goal),final_words=cr.words(base.positions(self.reference[-1])),
                parent_digest=parent['digest'])
    return Diagnostic


def parse_args(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--parent',type=Path,required=True)
    parser.add_argument('--parent-digest')
    parser.add_argument('--execute',action='store_true')
    args=parser.parse_args(argv)
    if args.execute and not args.parent_digest:parser.error('--execute requires reviewed --parent-digest')
    args.bootstrap_pending=False;args.bootstrap_sync_bound_deg=4.8;args.joint=2
    return args


def main():
    args=parse_args()
    if not os.path.ismount(cr.MOUNT) or not cr.ARCHIVE.is_dir():raise RuntimeError('SanStand unavailable; no fallback')
    args.parent_identity=load_parent(args.parent,args.parent_digest)
    directory=cr.ARCHIVE/('bounded_joint23_fresh_return_'+datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')+'_'+str(os.getpid()))
    directory.mkdir();(directory/'ros').mkdir();os.environ['ROS_LOG_DIR']=str(directory/'ros')
    manifest={}
    for name in ('bounded_joint23_fresh_return.py','bounded_joint23_counterrotation.py','bounded_joint_response_diagnostic.py','bootstrap_mixed_geometry.py','coupled_joint23_geometry.py'):
        data=(HERE/name).read_bytes();(directory/name).write_bytes(data);manifest[name]=hashlib.sha256(data).hexdigest()
    (directory/'source_sha256.json').write_text(json.dumps(manifest,indent=2)+'\n')
    spec=importlib.util.spec_from_file_location('fresh_return_base',HERE/'bounded_joint_response_diagnostic.py')
    base=importlib.util.module_from_spec(spec);spec.loader.exec_module(base)
    base.rospy.init_node('bounded_joint23_fresh_return',anonymous=True,disable_signals=True)
    terminal=dict(status='failed',execute=args.execute,maximum_commands=2 if args.execute else 0,parent_digest=args.parent_identity['digest'])
    experiment=None
    def alarm(*_):raise RuntimeError('110-second fresh return deadline exhausted; no further command')
    signal.signal(signal.SIGALRM,alarm);signal.setitimer(signal.ITIMER_REAL,110.)
    try:
        experiment=make_diagnostic_class(base)(args,directory);experiment.run()
        terminal['status']='completed' if args.execute else 'prepared_no_motion'
    except BaseException as error:
        terminal['reason']=str(error)
        if experiment is not None:experiment.event('no_further_commands',reason=str(error),recovery_attempted=False,stop_or_disable_sent=False)
        raise
    finally:
        signal.setitimer(signal.ITIMER_REAL,0)
        if experiment is not None:
            terminal['commands_published']=experiment.publish_count;terminal['paired_rgb_count']=len(experiment.saved_rgb);experiment.close()
        base.write_json(directory/'terminal.json',terminal);print('SESSION '+str(directory),flush=True)
if __name__=='__main__':main()
