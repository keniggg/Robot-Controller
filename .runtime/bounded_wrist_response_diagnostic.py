#!/usr/bin/env python3
"""Fresh J4/J6 two-command wrist diagnostic; default has no command publisher.

Only a 22-word (1.9336 degree) outbound increment and original SDK return are
permitted. Physical bounds stay fixed about INITIAL actual feedback: edited
joint +/-4.8 degrees, all unedited joints +/-0.4 degrees, frozen gripper range.
Held SDK offsets are modeled separately and never expand actual feedback bounds.
No bootstrap, sync, enable, trim, stop, disable, old plan input or recovery.
"""
import argparse
from datetime import datetime, timezone
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

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
sys.path.insert(0, str(HERE))
from coupled_joint23_geometry import words, rotation_angle

LABELS = ('outbound', 'return_original_sdk')
ABSOLUTE_RAD = math.radians(4.8)
HELD_RAD = math.radians(.4)
REVERSE_RAD = math.radians(.5)
GRID_STEP_RAD = math.radians(.6)
QUANTUM = 2*math.pi/4096
MOUNT = Path('/media/zhuyupei/SanStand')
ARCHIVE = MOUNT/'实验记录/Robot-Controller/ros_log'


def vector(value):
    value = np.asarray(value, dtype=float)
    if (value.shape != (7,) or not np.isfinite(value).all()
            or np.any(abs(value[:6]) > math.pi) or not 0 <= value[6] <= .05):
        raise RuntimeError('finite seven-joint vector within encoded limits required')
    return value.copy()


def validate_source_binding(now_ns, epoch_ns, accepted_ns, sdk_ns, board_ns=None):
    if type(epoch_ns) is not int or epoch_ns <= 0:
        raise RuntimeError('fresh positive control epoch required')
    for name,stamp,age_limit in (('accepted',accepted_ns,300_000_000),
                                ('SDK',sdk_ns,300_000_000),
                                ('board',board_ns,350_000_000)):
        if stamp is None and name == 'board':
            continue
        if type(stamp) is not int or stamp < epoch_ns or not 0 <= now_ns-stamp <= age_limit:
            raise RuntimeError(name+' source is stale, future or before frozen epoch')


def verify_fresh_scene(proof, previous, fresh, model, projection, corner_batch):
    """Re-prove latest scene from cached continuous cell enclosures, not samples.

    A board-point change in world coordinates has the same norm in EVERY camera
    pose. Expanding the union camera box by its maximum therefore bounds the
    full new board. Plane rotation is debited by a whole-cell CAD-to-origin
    distance bound, not just the midpoint distance.
    """
    previous,fresh=np.asarray(previous),np.asarray(fresh)
    delta_t=fresh[:3,3]-previous[:3,3]
    angle=rotation_angle(previous[:3,:3].T@fresh[:3,:3])
    if np.linalg.norm(delta_t)>.003 or angle>math.radians(.5):
        raise RuntimeError('fresh board left the bounded scene neighborhood')
    corners=np.array([[x,y,0.,1.] for x in (0.,.18) for y in (0.,.135)])
    shifts=np.linalg.norm(((fresh-previous)@corners.T).T[:,:3],axis=1)
    shift=float(np.max(shifts))
    old_n=previous[:3,2].copy();new_n=fresh[:3,2].copy()
    if old_n[2]<0:old_n=-old_n
    if new_n[2]<0:new_n=-new_n
    clearance=proof['minimum_plane_clearance_lower_bound_m']-float(np.linalg.norm(delta_t))
    clearance-=float(np.linalg.norm(new_n-old_n))*proof['maximum_cad_distance_to_board_origin_m']
    lower=np.asarray(corner_batch[0])-shifts[None,:,None]
    upper=np.asarray(corner_batch[1])+shifts[None,:,None]
    if np.min(lower[:,:,0])<.12:
        raise RuntimeError('fresh-scene continuous board depth below 120 mm')
    rectangle=projection(model,lower,upper)
    ci=model.evidence['color_intrinsics']
    edge=float(min(rectangle[0,0],ci['width']-1-rectangle[0,1],rectangle[1,0],ci['height']-1-rectangle[1,1]))
    if clearance<.04 or edge<20.:
        raise RuntimeError('fresh-scene continuous proof violates 40 mm plane / 20 px board margin')
    return dict(minimum_plane_clearance_lower_bound_m=float(clearance),
        minimum_full_board_edge_lower_bound_px=edge,minimum_board_depth_lower_bound_m=float(np.min(lower[:,:,0])),
        observed_board_point_displacement_bound_m=shift,
        observed_board_translation_m=float(np.linalg.norm(delta_t)),observed_board_rotation_deg=math.degrees(angle))


def segment_rectangle(measured, target, reference, origin, joint):
    """Entire INITIAL actual interval; nominal SDK bias is proved separately."""
    measured,target,reference,origin=map(vector,(measured,target,reference,origin))
    if joint not in (4,6): raise RuntimeError('wrist joint must be J4 or J6')
    j=joint-1
    lower,upper=origin[:6]-HELD_RAD,origin[:6]+HELD_RAD
    lower[j],upper[j]=origin[j]-ABSOLUTE_RAD,origin[j]+ABSOLUTE_RAD
    if np.any(measured[:6]<lower) or np.any(measured[:6]>upper):
        raise RuntimeError('measured pose escapes fixed initial actual envelope')
    # Target/SDK coordinates are nominal. Only the edited target must lie in
    # its physical interval; unchanged held-word biases may exceed 0.4 degrees.
    # All nominal endpoints must still lie within initial +/-4.8 degrees.
    for q in (target,reference):
        if np.any(abs(q[:6]-origin[:6])>ABSOLUTE_RAD):
            raise RuntimeError('nominal SDK endpoint escapes initial absolute bounds')
    difference=target[j]-measured[j]
    if abs(difference)<.003:
        raise RuntimeError('edited wrist axis lacks intended measured displacement')
    directions=np.zeros(6);directions[j]=1. if difference>0 else -1.
    return lower,upper,directions


def rectangle_grid(lower, upper, joint):
    """Full 6-D cell coverage: edited width <=0.6 deg, others <=0.4 deg."""
    lower,upper=np.asarray(lower),np.asarray(upper)
    if (lower.shape!=(6,) or upper.shape!=(6,) or not np.isfinite([lower,upper]).all()
            or np.any(lower>upper) or joint not in (4,6)):
        raise RuntimeError('invalid wrist rectangle')
    axes=[]
    for index in range(6):
        spacing=GRID_STEP_RAD if index==joint-1 else math.radians(.4)
        count=max(1,int(math.ceil((upper[index]-lower[index])/spacing-1e-12)))
        axes.append(np.linspace(lower[index],upper[index],count+1))
    return axes


def rectangle_cells(axes):
    for indices in itertools.product(*(range(len(axis)-1) for axis in axes)):
        lower = np.array([axis[index] for axis,index in zip(axes,indices)])
        upper = np.array([axis[index+1] for axis,index in zip(axes,indices)])
        yield .5*(lower+upper), float(.5*np.sum(upper-lower))


class WristContract:
    """Exactly one quantized edited axis, then exact original SDK return."""
    def __init__(self, measured, reference, epoch, joint, direction=1):
        self.origin,self.reference=vector(measured),vector(reference)
        if type(epoch) is not int or epoch<=0 or joint not in (4,6) or direction not in (-1,1):
            raise RuntimeError('positive epoch, J4/J6 and unit direction required')
        self.epoch,self.joint,self.index=epoch,joint,joint-1
        self.held=tuple(i for i in range(6) if i!=self.index)
        self.gripper_bounds=(min(self.origin[6],self.reference[6]),max(self.origin[6],self.reference[6]))
        self.step_words=22
        outbound=self.reference.copy();outbound[self.index]+=direction*self.step_words*QUANTUM
        delta=np.asarray(words(outbound))-np.asarray(words(self.reference))
        expected=np.zeros(6,dtype=int);expected[self.index]=direction*self.step_words
        if not np.array_equal(delta,expected): raise RuntimeError('wrist increment changed unexpected wire words')
        self.targets=(outbound,self.reference.copy())
        self.count=0;self.failed=None;self.active=None;self.expected_words=words(self.reference)
        self.target_seen=False;self.latest_sdk_stamp=0
        self.assert_feedback(self.origin,epoch)
        for target in self.targets:
            if np.any(abs(target[:6]-self.origin[:6])>ABSOLUTE_RAD):
                raise RuntimeError('nominal SDK target escapes initial absolute bounds')

    def fail(self,message):
        if self.failed is None:self.failed=message
        raise RuntimeError(self.failed)

    def live(self,epoch):
        if self.failed is not None:raise RuntimeError(self.failed)
        if epoch!=self.epoch:self.fail('driver epoch changed; no further command')

    def assert_feedback(self,current,epoch):
        self.live(epoch)
        try:q=vector(current)
        except RuntimeError as error:self.fail(str(error))
        if not self.gripper_bounds[0]<=q[6]<=self.gripper_bounds[1]:
            self.fail('measured gripper escaped frozen geometry opening interval')
        if abs(q[self.index]-self.origin[self.index])>ABSOLUTE_RAD:
            self.fail('wrist feedback escaped INITIAL absolute +/-4.8 degree interval')
        if np.any(abs(q[list(self.held)]-self.origin[list(self.held)])>HELD_RAD):
            self.fail('unedited axis moved beyond INITIAL +/-0.4 degree bound')
        if self.active is not None:
            if np.any(q[:6]<self.active['lower']) or np.any(q[:6]>self.active['upper']):
                self.fail('feedback escaped proved fixed initial envelope')
            signed=self.active['directions'][self.index]*(q[self.index]-self.active['start'][self.index])
            if signed < -REVERSE_RAD:self.fail('wrist reversed more than 0.5 degrees')
        return q

    def begin(self,label,target,measured,current_reference,epoch,stamp_ns):
        self.live(epoch)
        if self.count>=2 or label!=LABELS[self.count]:
            self.fail('two-command sequence exhausted or out of order')
        if self.active is not None and not self.active.get('settled'):
            self.fail('previous command lacks independent stationary completion')
        target,reference=vector(target),vector(current_reference)
        predecessor=self.reference if self.count==0 else self.targets[0]
        if words(reference)!=words(predecessor) or reference[6]!=self.reference[6]:
            self.fail('current SDK differs from required sequence predecessor')
        if not np.array_equal(target,self.targets[self.count]):
            self.fail('command differs from quantized outbound or original SDK return')
        if any(target[j]!=self.reference[j] for j in self.held) or target[6]!=self.reference[6]:
            self.fail('wrist command would edit held joint or gripper')
        if abs(words(target)[self.index]-words(reference)[self.index])!=22:
            self.fail('wrist command must change exactly 22 encoder words')
        current=self.assert_feedback(measured,epoch)
        lo,hi,directions=segment_rectangle(current,target,reference,self.origin,self.joint)
        self.active=dict(label=label,start=current,target=target,lower=lo,upper=hi,
            directions=directions,prior_words=words(reference),target_words=words(target),
            command_stamp_ns=int(stamp_ns),settled=False)
        self.count+=1;self.target_seen=False;self.expected_words=None
        return self.active

    def observe_sdk(self,current,stamp_ns,epoch):
        self.live(epoch)
        try:q=vector(current)
        except RuntimeError as error:self.fail(str(error))
        if q[6]!=self.reference[6]:self.fail('gripper SDK reference changed')
        w=words(q)
        if int(stamp_ns)<self.latest_sdk_stamp:self.fail('SDK source stamp regressed')
        if self.active is None:
            if w!=words(self.reference):self.fail('SDK changed before outbound')
        else:
            a=self.active
            if self.target_seen:
                if w!=a['target_words']:self.fail('SDK target changed after arrival')
            elif (any(w[j]!=a['prior_words'][j] for j in self.held)
                    or not min(a['prior_words'][self.index],a['target_words'][self.index])
                    <=w[self.index]<=max(a['prior_words'][self.index],a['target_words'][self.index])):
                self.fail('SDK reference outside one-axis word segment')
            if w==a['target_words']:
                if int(stamp_ns)<a['command_stamp_ns']:self.fail('target SDK predates command')
                self.target_seen=True;self.expected_words=w
        self.latest_sdk_stamp=max(self.latest_sdk_stamp,int(stamp_ns))

    def complete(self,current,epoch,independent_stationary):
        q=self.assert_feedback(current,epoch)
        if self.active is None or not self.target_seen or independent_stationary is not True:
            self.fail('completion lacks target SDK and independent stationary feedback')
        delta=q[self.index]-self.active['start'][self.index]
        if self.active['directions'][self.index]*delta<.003:
            self.fail('wrist lacks directional actual response')
        self.active['settled']=True


def actual_motion_metrics(fk, before, after):
    before, after = vector(before), vector(after)
    first, last = fk(before[:6]), fk(after[:6])
    residual = math.degrees(rotation_angle(first[:3, :3].T @ last[:3, :3]))
    return dict(actual_delta_deg=np.degrees(after[:6]-before[:6]).tolist(),
        actual_fk_rotation_residual_deg=residual,
        actual_fk_rotation_within_one_encoder_word=bool(residual <= math.degrees(QUANTUM)+1e-10),
        interpretation='Measured FK rotation/translation diagnostic; no automatic trim or calibration update.')


def make_diagnostic_class(base):
    class Diagnostic(base.Diagnostic):
        def __init__(self, args, directory):
            self.contract = None
            self.saved_rgb = set()
            self.publish_count = 0
            self.geometry_batches = {}
            self.path_sequence = 0
            super().__init__(args, directory)
            self.deadline = time.monotonic()+85.

        def receive(self, topic, msg):
            super().receive(topic, msg)
            pairs = []
            with self.lock:
                if self.closed:
                    return
                try:
                    if self.contract is not None:
                        if topic == '/alicia_d/accepted_joint_states':
                            self.contract.assert_feedback(base.positions(msg), self.epoch)
                        elif topic == '/alicia_d/control_reference':
                            self.contract.observe_sdk(base.positions(msg), msg.header.stamp.to_nsec(), self.epoch)
                            self.expected_words = self.contract.expected_words
                        elif topic == '/alicia_d/control_reference_epoch':
                            self.contract.live(self.epoch)
                    if topic in (base.RGB, '/charuco/quality', '/tf'):
                        for stamp, quality in self.quality.items():
                            key = (stamp, quality.get('board_frame'))
                            if (quality.get('pose_ok') is True and stamp in self.rgb
                                    and key in self.board and stamp not in self.saved_rgb):
                                self.saved_rgb.add(stamp)
                                pairs.append((stamp, self.rgb[stamp], quality))
                except Exception as error:
                    self.fault = 'wrist callback: '+str(error)
                    self.pair_changed.notify_all()
            # State cache is never locked during external bag writes. All pose_ok
            # exact RGB tuples are retained, including diagnostic-only older rows;
            # execution snapshots still enforce the unchanged 350 ms source age.
            for stamp, rgb, quality in pairs:
                now = base.rospy.Time.now()
                try:
                    self.write_bag('/bounded_wrist/paired_rgb', rgb, now)
                    self.write_bag('/bounded_wrist/pair_provenance', base.String(data=json.dumps(dict(
                        source_stamp_ns=stamp, recorded_stamp_ns=now.to_nsec(),
                        camera_model_fingerprint=quality.get('camera_model_fingerprint'),
                        board_frame=quality.get('board_frame'), exact_rgb_board_quality=True))), now)
                except Exception as error:
                    with self.pair_changed:
                        self.fault = 'wrist RGB archive: '+str(error)
                        self.pair_changed.notify_all()

        def guard(self):
            super().guard()
            with self.lock:
                if self.reference and self.accepted:
                    validate_source_binding(base.rospy.Time.now().to_nsec(),
                        self.frozen_epoch if self.frozen_epoch is not None else self.epoch,
                        self.accepted[-1].header.stamp.to_nsec(),self.reference[-1].header.stamp.to_nsec())
                if self.contract is not None:
                    self.contract.assert_feedback(base.positions(self.accepted[-1]), self.epoch)
                    if self.fault:
                        self.contract.fail(self.fault)

        def bootstrap_once(self):
            raise RuntimeError('wrist has no bootstrap/sync/enable path')

        def path(self, measured, target, geometry, reference=None):
            from bootstrap_mixed_geometry import certify_local_cell_geometry, projection_corner_box_batch
            reference = base.positions(self.reference[-1]) if reference is None else vector(reference)
            lower, upper, directions = segment_rectangle(measured, target, reference, self.contract.origin, self.args.joint)
            if np.any(lower < self.lo) or np.any(upper > self.hi):
                raise RuntimeError('response rectangle exceeds current joint limits')
            axes = rectangle_grid(lower, upper, self.args.joint)
            proofs, visited = [], set()
            openings = sorted(set(self.contract.gripper_bounds))
            local_by_opening = {}
            for opening in openings:
                local=np.concatenate([base._box_corners(c,np.eye(3),size) for _,c,size in
                    base._stage_boxes(np.eye(4),self.gripper,opening,'y','z')])
                radii=np.array([max(self.fk.corner_motion_radii(p)) for p in local])
                local_by_opening[opening]=(local,radii)
            maximum_cad_distance=0.
            def state_valid(q, opening):
                self.guard()
                q = np.asarray(q, dtype=float)
                key = tuple(q.tolist())+(opening,)
                if key in visited:
                    return
                visited.add(key)
                request = base.GetStateValidityRequest(group_name='alicia')
                request.robot_state = base.RobotState()
                request.robot_state.joint_state = base.JointState(name=base.NAMES,
                    position=list(q)+[opening])
                if not self.validity(request).valid:
                    raise RuntimeError('current MoveIt scene rejects full response cell grid or SDK bridge')
            def prove_cell(midpoint, radius):
                nonlocal maximum_cad_distance
                self.guard()
                for opening in openings:
                    proof = certify_local_cell_geometry(self.fk, self.gripper,
                        np.r_[midpoint, opening], geometry, self.lo, self.hi,
                        math.degrees(radius))
                    proofs.append(proof)
                    local,radii=local_by_opening[opening];tool=self.fk(midpoint)
                    world=local@tool[:3,:3].T+tool[:3,3]
                    maximum_cad_distance=max(maximum_cad_distance,float(np.max(
                        np.linalg.norm(world-geometry['Y'][:3,3],axis=1)+radius*radii)))
                    state_valid(midpoint, opening)
            # Held intervals are bisected: their centers are +/-0.2 degrees and
            # half-widths 0.2 degrees for a full +/-0.4 interval. No double count.
            response_cells=0
            for midpoint, radius in rectangle_cells(axes):
                prove_cell(midpoint,radius)
                response_cells+=1
            # Nominal SDK bridges are separately proved. Existing command bias
            # can put a held SDK endpoint outside the physical reverse-response
            # interval; that NEVER expands the accepted-feedback permission.
            # Their endpoints remain inside the ORIGINAL absolute nominal bounds.
            # Actual unedited axes still have their original +/-0.4 degree guard.
            # SDK axes can update independently too. Prove their complete
            # command-space box, not merely current->SDK->target diagonals.
            endpoints=np.vstack((measured[:6],reference[:6],target[:6]))
            bridge_lower,bridge_upper=endpoints.min(axis=0),endpoints.max(axis=0)
            bridge_axes=rectangle_grid(bridge_lower,bridge_upper,self.args.joint)
            bridge_cells=0
            for midpoint,radius in rectangle_cells(bridge_axes):
                prove_cell(midpoint,max(radius,1e-12))
                bridge_cells+=1
            for first, last in ((measured[:6], target[:6]),
                    (measured[:6], reference[:6]), (reference[:6], target[:6])):
                for fraction in np.linspace(0., 1., 17):
                    for opening in openings:
                        state_valid(first*(1-fraction)+last*fraction, opening)
            result = dict(lower_rad=lower.tolist(), upper_rad=upper.tolist(),
                directions=directions.tolist(), grid_axis_counts=[len(a) for a in axes],
                moveit_samples=len(visited), continuous_cell_proofs=len(proofs),
                physical_response_cells=response_cells, nominal_sdk_bridge_cells=bridge_cells,
                sdk_bridge_does_not_expand_feedback_rectangle=True,
                nominal_sdk_bridge_independent_box=True,
                nominal_sdk_bridge_lower_rad=bridge_lower.tolist(),nominal_sdk_bridge_upper_rad=bridge_upper.tolist(),
                full_independent_rectangle=True, sdk_bridge_included=True,
                frozen_gripper_opening_interval_m=list(self.contract.gripper_bounds),
                maximum_cad_distance_to_board_origin_m=maximum_cad_distance,
                minimum_plane_clearance_lower_bound_m=min(p['minimum_plane_clearance_lower_bound_m'] for p in proofs),
                minimum_full_board_edge_lower_bound_px=min(p['minimum_full_board_edge_lower_bound_px'] for p in proofs),
                minimum_board_depth_lower_bound_m=min(p['minimum_board_depth_lower_bound_m'] for p in proofs),
                hardware_tracking_or_stopping_certified=False, whole_arm_continuous_collision_certified=False)
            # Preserve cell AND corner identities until each positive-depth
            # normalization. A cross-cell XYZ union is unnecessarily looser.
            batch=(np.asarray([p['camera_coordinate_taylor_bound']['corner_lower_xyz_m'] for p in proofs]),
                   np.asarray([p['camera_coordinate_taylor_bound']['corner_upper_xyz_m'] for p in proofs]))
            self.path_sequence += 1
            archive=self.out/('continuous_cells_%03d.npz'%self.path_sequence)
            np.savez_compressed(str(archive),camera_lower=batch[0],camera_upper=batch[1])
            result['cell_enclosure_archive']=str(archive)
            result['cell_enclosure_sha256']=hashlib.sha256(archive.read_bytes()).hexdigest()
            self.geometry_batches[str(archive)]=batch
            result['whole_rectangle_scene_proof']=verify_fresh_scene(result,geometry['Y'],geometry['Y'],geometry['model'],projection_corner_box_batch,batch)
            return result

        def execute_step(self, label, target):
            if not self.args.execute or self.publisher is None:
                raise RuntimeError('preparation process cannot publish')
            self.guard()
            self.stationary(max(self.started_ns, self.epoch))
            geometry = self.snapshot(label)
            with self.lock:
                measured = base.positions(self.accepted[-1])
                reference = base.positions(self.reference[-1])
            details = self.path(measured, target, geometry, reference)
            self.driver_connection()
            from bootstrap_mixed_geometry import projection_corner_box_batch
            for attempt in range(3):
                fresh = self.snapshot(label+'_precommand_%d'%attempt)
                if (np.linalg.norm(fresh['Y'][:3, 3]-geometry['Y'][:3, 3]) > .003
                        or not np.allclose(fresh['X'], geometry['X'], atol=1e-10, rtol=0)
                        or math.degrees(rotation_angle(fresh['Y'][:3,:3].T@geometry['Y'][:3,:3])) > .5):
                    raise RuntimeError('board scene/hand-eye changed during proof')
                fresh_proof=verify_fresh_scene(details,geometry['Y'],fresh['Y'],fresh['model'],projection_corner_box_batch,
                    self.geometry_batches[details['cell_enclosure_archive']])
                with self.lock:
                    self.guard()
                    now = base.rospy.Time.now()
                    age = now.to_nsec()-fresh['source_stamp_ns']
                    if fresh['source_stamp_ns'] < self.frozen_epoch or age < 0:
                        raise RuntimeError('precommand source predates epoch or is future')
                    if age > 350_000_000:
                        continue
                    if np.max(abs(base.positions(self.accepted[-1])[:6]-measured[:6])) > math.radians(.15):
                        raise RuntimeError('measured start changed during path proof')
                    if self.publisher.get_num_connections() < 1:
                        raise RuntimeError('driver command connection missing')
                    self.contract.begin(label, target, base.positions(self.accepted[-1]),
                        base.positions(self.reference[-1]), self.epoch, now.to_nsec())
                    # Contract uses exactly the already-proved start, not a slightly
                    # different current-centered response interval at publication.
                    self.contract.active['lower'] = np.asarray(details['lower_rad'])
                    self.contract.active['upper'] = np.asarray(details['upper_rad'])
                    self.contract.assert_feedback(base.positions(self.accepted[-1]), self.epoch)
                    self.expected_words = None
                    msg = base.JointState()
                    msg.header.stamp, msg.header.frame_id = now, 'gui_direct'
                    msg.name, msg.position = base.NAMES, target.tolist()
                    msg.effort = [0.]*7; msg.effort[self.contract.index]=1.
                    # Recheck immediately after message/contract construction. A
                    # stale source here aborts this reserved command, never retries.
                    self.guard()
                    publish_time=base.rospy.Time.now()
                    validate_source_binding(publish_time.to_nsec(),self.frozen_epoch,
                        self.accepted[-1].header.stamp.to_nsec(),self.reference[-1].header.stamp.to_nsec(),fresh['source_stamp_ns'])
                    msg.header.stamp=publish_time
                    self.contract.active['command_stamp_ns']=publish_time.to_nsec()
                    self.publish_count += 1
                    self.publisher.publish(msg)
                    sent_ns = publish_time.to_nsec()
                    self.event('command', label=label, edited_joints=[self.args.joint],
                        command_stamp_ns=sent_ns, starting_measured_rad=measured.tolist(),
                        target_rad=target.tolist(), expected_wire_words=words(target),
                        precommand_source_age_ms=(sent_ns-fresh['source_stamp_ns'])/1e6,
                        fresh_scene_continuous_proof=fresh_proof, **details)
                break
            else:
                raise RuntimeError('fresh source expired during all bounded precommand acquisitions')
            deadline = min(self.deadline, time.monotonic()+12.)
            while time.monotonic() < deadline:
                self.guard()
                with self.lock:
                    current = base.positions(self.accepted[-1])
                    active = self.contract.active
                    j=self.contract.index
                    responding = active['directions'][j]*(current[j]-active['start'][j]) >= .003
                    target_seen = self.contract.target_seen
                try:
                    stationary = self.stationary(sent_ns)
                except ValueError:
                    stationary = None
                if stationary is not None and target_seen and responding:
                    with self.lock:
                        self.contract.complete(current, self.epoch, True)
                    metrics = actual_motion_metrics(self.fk, measured, current)
                    self.event('settled', label=label, stationary=stationary, **metrics)
                    self.stable_hold_recording(3.)
                    observed = self.snapshot(label+'_post_hold')
                    self.event('post_hold_observation', label=label,
                        source_stamp_ns=observed['source_stamp_ns'], paired_rgb_count=len(self.saved_rgb),
                        **actual_motion_metrics(self.fk, self.contract.origin, observed['q']))
                    return observed
                time.sleep(.02)
            raise RuntimeError('wrist lacks bounded stationary response before step deadline')

        def run(self):
            reference, stationary = self.await_initial()
            geometry = self.snapshot('baseline')
            with self.lock:
                self.guard()
                measured = base.positions(self.accepted[-1])
                self.contract = WristContract(measured, reference, self.frozen_epoch, self.args.joint, self.args.direction)
                self.expected_words = words(reference)
            plan = dict(schema=1, execute=self.args.execute, edited_joints=[self.args.joint],
                initial_measured_rad=measured.tolist(), starting_sdk_rad=reference.tolist(),
                frozen_epoch_ns=self.frozen_epoch, step_words=self.contract.step_words,
                commands=[dict(label=label, target_rad=target.tolist(), words=words(target))
                    for label,target in zip(LABELS,self.contract.targets)],
                source='fresh current process only', bootstrap_pending=False,
                maximum_commands=2, overall_deadline_sec=85, stationary=stationary,
                recovery='none; no sync, enable, trim, stop or disable commands',
                archive_policy='local_only', upload_enabled=False, delete_enabled=False)
            if not self.args.execute:
                plan['prepare_geometry'] = {label:self.path(measured,target,geometry,reference)
                    for label,target in (('outbound',self.contract.targets[0]),)}
                base.write_json(self.out/'plan.json',plan)
                self.event('prepared_no_motion', plan=str(self.out/'plan.json'))
                return
            base.write_json(self.out/'plan.json',plan)
            for label,target in zip(LABELS,self.contract.targets):
                self.execute_step(label,target)
            self.event('completed_two_commands_same_sdk_return', commands_published=self.publish_count,
                original_words=words(reference), final_words=words(base.positions(self.reference[-1])),
                paired_rgb_count=len(self.saved_rgb))
    return Diagnostic


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execute', action='store_true')
    parser.add_argument('--joint',type=int,required=True,choices=(4,6))
    parser.add_argument('--step-deg',type=float,default=2.,choices=(2.,))
    parser.add_argument('--direction',type=int,default=1,choices=(-1,1))
    args = parser.parse_args(argv)
    args.bootstrap_pending = False
    args.bootstrap_sync_bound_deg = 4.8
    return args


def main():
    args = parse_args()
    if not os.path.ismount(str(MOUNT)) or not ARCHIVE.is_dir():
        raise RuntimeError('SanStand archive unavailable; no system-disk fallback')
    directory = ARCHIVE/('bounded_wrist_response_diagnostic_'+datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')+'_'+str(os.getpid()))
    directory.mkdir(); (directory/'ros').mkdir()
    os.environ['ROS_LOG_DIR'] = str(directory/'ros')
    source_names = ('bounded_wrist_response_diagnostic.py','bounded_joint_response_diagnostic.py',
                    'bootstrap_mixed_geometry.py','coupled_joint23_geometry.py')
    manifest = {}
    for name in source_names:
        data = (HERE/name).read_bytes(); (directory/name).write_bytes(data)
        manifest[name] = hashlib.sha256(data).hexdigest()
    (directory/'source_sha256.json').write_text(json.dumps(manifest,indent=2)+'\n')
    spec = importlib.util.spec_from_file_location('bounded_wrist_base',HERE/'bounded_joint_response_diagnostic.py')
    base = importlib.util.module_from_spec(spec); spec.loader.exec_module(base)
    base.rospy.init_node('bounded_wrist_response_diagnostic', anonymous=True, disable_signals=True)
    terminal = dict(status='failed', execute=args.execute, maximum_commands=2 if args.execute else 0)
    experiment = None
    def alarm(_signum,_frame):
        raise RuntimeError('85-second overall deadline exhausted; no further command')
    signal.signal(signal.SIGALRM,alarm); signal.setitimer(signal.ITIMER_REAL,85.)
    try:
        experiment = make_diagnostic_class(base)(args,directory)
        experiment.run()
        terminal['status'] = 'completed' if args.execute else 'prepared_no_motion'
    except BaseException as error:
        terminal['reason'] = str(error)
        if experiment is not None:
            experiment.event('no_further_commands', reason=str(error),
                recovery_attempted=False, stop_or_disable_sent=False)
        raise
    finally:
        signal.setitimer(signal.ITIMER_REAL,0)
        if experiment is not None:
            terminal['commands_published'] = experiment.publish_count
            terminal['paired_rgb_count'] = len(experiment.saved_rgb)
            experiment.close()
        base.write_json(directory/'terminal.json',terminal)
        print('SESSION '+str(directory),flush=True)


if __name__ == '__main__':
    main()
