#!/usr/bin/env python3
"""Fresh J2/J3 counterrotation experiment; default is preparation with no publisher.

--execute permits positive, original SDK, negative, original SDK logical legs.
Each leg is two ordered one-hot messages: J2, then J3 after actual SDK evidence
and independent stationary feedback. --positive-only predeclares two legs and
at most FOUR physical publications; default four legs permit at most EIGHT.
No bootstrap, sync, enable, trim, stop, disable, old-plan input or recovery path.
Recorded actual rotation is authoritative; opposite SDK words do not prove pure
translation. Geometry bounds are conditional models, not hardware guarantees.
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
from coupled_joint23_geometry import coupled_targets, words, rotation_angle

EDITED = (1, 2)
HELD = (0, 3, 4, 5)
LABELS = ('positive', 'return_after_positive', 'negative', 'return_after_negative')
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


def verify_fresh_scene(proof, previous, fresh, model, projection, enclosures=None):
    """Re-prove latest scene from cached continuous cell enclosures, not samples.

    A board-point change in world coordinates has the same norm in EVERY camera
    pose. Expanding each matching corner camera box by that corner's change
    bounds the new board without cross-corner depth mixing. Plane rotation is debited by a whole-cell CAD-to-origin
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
    if enclosures is None:
        enclosures=(proof['camera_link_cell_corner_lower_m'],proof['camera_link_cell_corner_upper_m'])
    lower=np.asarray(enclosures[0])-shifts[None,:,None]
    upper=np.asarray(enclosures[1])+shifts[None,:,None]
    depth_lower=float(np.min(lower[:,:,0]))
    if depth_lower<.12:
        raise RuntimeError('fresh-scene continuous board depth below 120 mm')
    rectangle=projection(model,lower,upper)
    ci=model.evidence['color_intrinsics']
    edge=float(min(rectangle[0,0],ci['width']-1-rectangle[0,1],rectangle[1,0],ci['height']-1-rectangle[1,1]))
    if clearance<.04 or edge<20.:
        raise RuntimeError('fresh-scene continuous proof violates 40 mm plane / 20 px board margin')
    return dict(minimum_plane_clearance_lower_bound_m=float(clearance),
        minimum_full_board_edge_lower_bound_px=edge,minimum_board_depth_lower_bound_m=depth_lower,
        observed_board_point_displacement_bound_m=shift,
        observed_board_translation_m=float(np.linalg.norm(delta_t)),observed_board_rotation_deg=math.degrees(angle))


def segment_rectangle(measured, target, reference, origin):
    """Independent edited-axis rectangle intersected with INITIAL absolute bounds.

    Held-axis intervals include current/SDK bridge and the observed-response
    allowance, without expanding the initial +/-0.4 degree held-axis bound.
    The entire rectangle is later modeled, not just its diagonal.
    """
    measured, target, reference, origin = map(vector, (measured, target, reference, origin))
    lower = np.maximum(origin[:6]-ABSOLUTE_RAD, measured[:6]-HELD_RAD)
    upper = np.minimum(origin[:6]+ABSOLUTE_RAD, measured[:6]+HELD_RAD)
    for index in HELD:
        lower[index] = max(lower[index], origin[index]-HELD_RAD)
        upper[index] = min(upper[index], origin[index]+HELD_RAD)
    directions = np.zeros(6)
    for index in EDITED:
        difference = target[index]-measured[index]
        if abs(difference) < .003:
            raise RuntimeError('edited axis has insufficient intended measured displacement')
        sign = directions[index] = 1. if difference > 0 else -1.
        lower[index] = max(origin[index]-ABSOLUTE_RAD,
            measured[index]-(REVERSE_RAD if sign > 0 else ABSOLUTE_RAD))
        upper[index] = min(origin[index]+ABSOLUTE_RAD,
            measured[index]+(ABSOLUTE_RAD if sign > 0 else REVERSE_RAD))
    if np.any(lower > upper):
        raise RuntimeError('empty directional/absolute response intersection')
    for label, q in (('current', measured), ('target', target)):
        if np.any(q[:6] < lower) or np.any(q[:6] > upper):
            raise RuntimeError(label+' escapes directional/initial absolute rectangle; never clamp target')
    if (np.any(abs(reference[:6]-origin[:6]) > ABSOLUTE_RAD)
            or np.any(abs(reference[list(HELD)]-origin[list(HELD)]) > HELD_RAD)):
        raise RuntimeError('held SDK bridge escapes initial absolute/held-axis bounds')
    return lower, upper, directions


def rectangle_grid(lower, upper):
    """Cell boundaries: edited spacing <=0.6 deg, held intervals bisected.

    MoveIt checks the full Cartesian product of six-dimensional cell centers.
    Each whole cell is separately enclosed by a continuous L1 geometry proof.
    These collision samples are explicitly not a continuous collision proof.
    """
    lower, upper = np.asarray(lower), np.asarray(upper)
    axes = []
    for index in range(6):
        if upper[index] < lower[index]:
            raise RuntimeError('reversed rectangle')
        count = (max(1,int(math.ceil((upper[index]-lower[index])/GRID_STEP_RAD))) if index in EDITED
                 else (2 if upper[index]>lower[index] else 1))
        axes.append(np.linspace(lower[index], upper[index], count+1))
    return axes


def rectangle_cells(axes):
    for indices in itertools.product(*(range(len(axis)-1) for axis in axes)):
        lower = np.array([axis[index] for axis,index in zip(axes,indices)])
        upper = np.array([axis[index+1] for axis,index in zip(axes,indices)])
        yield .5*(lower+upper), float(.5*np.sum(upper-lower))


class CounterrotationContract:
    """Publication sequence and per-feedback latch, independently testable of ROS."""
    def __init__(self, measured, reference, epoch, step_deg, positive_only=False):
        self.origin, self.reference = vector(measured), vector(reference)
        if type(epoch) is not int or epoch <= 0:
            raise RuntimeError('positive driver epoch required')
        self.epoch = epoch
        self.gripper_bounds = (min(self.origin[6],self.reference[6]),max(self.origin[6],self.reference[6]))
        pair, self.step_words = coupled_targets(self.reference, step_deg)
        if type(positive_only) is not bool:
            raise RuntimeError('positive_only must be an explicit boolean')
        self.positive_only=positive_only
        self.labels=LABELS[:2] if positive_only else LABELS
        complete=(pair[0],self.reference.copy(),pair[1],self.reference.copy())
        self.targets=complete[:2] if positive_only else complete
        self.count = 0
        self.wire_count = 0
        self.maximum_commands = 2*len(self.targets)
        self.failed = None
        self.active = None
        self.expected_words = words(self.reference)
        self.target_seen = False
        self.latest_sdk_stamp = 0
        self.assert_feedback(self.origin, epoch)
        for target in self.targets:
            if np.any(abs(target[:6]-self.origin[:6]) > ABSOLUTE_RAD):
                raise RuntimeError('requested target escapes initial absolute bounds; never clamp')

    def fail(self, message):
        if self.failed is None:
            self.failed = message
        raise RuntimeError(self.failed)

    def live(self, epoch):
        if self.failed is not None:
            raise RuntimeError(self.failed)
        if epoch != self.epoch:
            self.fail('driver epoch changed; no further command')

    def assert_feedback(self, current, epoch):
        self.live(epoch)
        try:
            q = vector(current)
        except RuntimeError as error:
            self.fail(str(error))
        if not self.gripper_bounds[0] <= q[6] <= self.gripper_bounds[1]:
            self.fail('measured gripper escaped frozen geometry opening interval')
        if np.any(abs(q[:6]-self.origin[:6]) > ABSOLUTE_RAD):
            self.fail('accepted feedback escaped initial absolute +/-4.8 degree bounds')
        if np.any(abs(q[list(HELD)]-self.origin[list(HELD)]) > HELD_RAD):
            self.fail('unedited axis moved beyond initial +/-0.4 degree bound')
        if self.active is not None:
            lo, hi = self.active['lower'], self.active['upper']
            if np.any(q[:6] < lo) or np.any(q[:6] > hi):
                self.fail('accepted feedback escaped directional response rectangle')
        return q

    def begin(self, label, target, measured, current_reference, epoch, stamp_ns):
        self.live(epoch)
        if self.count >= len(self.targets) or label != self.labels[self.count]:
            self.fail('declared %d-leg sequence exhausted or out of order'%len(self.targets))
        if self.active is not None and not self.active.get('settled'):
            self.fail('previous command lacks independently stationary completion')
        target, reference = vector(target), vector(current_reference)
        expected_reference = self.reference if self.count == 0 else self.targets[self.count-1]
        if words(reference) != words(expected_reference) or reference[6] != self.reference[6]:
            self.fail('current SDK reference differs from required sequence predecessor')
        if not np.array_equal(target, self.targets[self.count]):
            self.fail('target differs from exact original/opposite-word sequence')
        if any(target[j] != self.reference[j] for j in HELD) or target[6] != self.reference[6]:
            self.fail('command would change an unedited axis or gripper')
        current = self.assert_feedback(measured, epoch)
        lower, upper, directions = segment_rectangle(current, target, reference, self.origin)
        self.active = dict(label=label, start=current, target=target,
            prior_words=words(reference), target_words=words(target), lower=lower,
            upper=upper, directions=directions, command_stamp_ns=int(stamp_ns), settled=False,
            wire_sent=0, wire_acknowledged=False, wire_stationary=False,
            wire_target_words=words(reference), wire_prior_words=words(reference),
            wire_command_stamp_ns=0)
        self.count += 1
        self.target_seen = False
        self.expected_words = words(reference)
        return self.active

    def intersect_proved_rectangle(self, lower, upper):
        """Keep BOTH publication-start directional bounds and proved geometry bounds."""
        if self.active is None:
            self.fail('no command reservation for geometric binding')
        lo=np.maximum(self.active['lower'],np.asarray(lower,dtype=float))
        hi=np.minimum(self.active['upper'],np.asarray(upper,dtype=float))
        if lo.shape!=(6,) or hi.shape!=(6,) or not np.isfinite([lo,hi]).all() or np.any(lo>hi):
            self.fail('proved and current-start response rectangles do not intersect')
        self.active['lower'],self.active['upper']=lo,hi

    def reserve_wire(self, axis, current_reference, measured, epoch, stamp_ns,
                     accepted_ns, sdk_ns, board_ns):
        """Reserve exactly one GUI intent; J3 follows confirmed, stationary J2.

        The driver ignores non-edited input positions and retains its admitted
        manual goals. Every other position here is nevertheless copied from the
        freshly confirmed SDK vector and checked, never reconstructed from FK.
        """
        self.live(epoch)
        try:
            validate_source_binding(stamp_ns, self.epoch, accepted_ns, sdk_ns, board_ns)
            reference = vector(current_reference)
            current = self.assert_feedback(measured, epoch)
        except RuntimeError as error:
            self.fail(str(error))
        a = self.active
        if a is None or a['settled'] or a['wire_sent'] >= 2:
            self.fail('logical leg has no remaining one-hot publication')
        stage = a['wire_sent']
        if axis != EDITED[stage] or self.wire_count >= self.maximum_commands:
            self.fail('one-hot J2 then J3 sequence exhausted or out of order')
        expected = a['prior_words'] if stage == 0 else a['wire_target_words']
        if words(reference) != expected or reference[6] != self.reference[6]:
            self.fail('one-hot predecessor differs from actually written SDK words')
        if stage:
            if not a['wire_acknowledged'] or not a['wire_stationary']:
                self.fail('second axis requires SDK confirmation and independent stationary first axis')
            if min(accepted_ns, sdk_ns, board_ns) < a['wire_command_stamp_ns']:
                self.fail('second-axis source precedes first-axis publication')
        # Reusing the original whole-leg proof may only shrink physical bounds.
        # The new axis also keeps its own current-start 0.5 degree reverse limit.
        direction = a['directions'][axis]
        if direction*(a['target'][axis]-current[axis]) < .003:
            self.fail('next edited axis lacks intended directional actual displacement')
        lower = current[axis]-(REVERSE_RAD if direction > 0 else ABSOLUTE_RAD)
        upper = current[axis]+(ABSOLUTE_RAD if direction > 0 else REVERSE_RAD)
        a['lower'][axis] = max(a['lower'][axis], lower)
        a['upper'][axis] = min(a['upper'][axis], upper)
        if not a['lower'][axis] <= a['target'][axis] <= a['upper'][axis]:
            self.fail('one-hot target escapes preserved physical rectangle')
        target = reference.copy()
        target[axis] = a['target'][axis]
        target_words = words(target)
        if any(target_words[j] != expected[j] for j in range(6) if j != axis):
            self.fail('one-hot payload alters a held SDK word')
        a.update(wire_axis=axis, wire_start=current.copy(), wire_sent=stage+1,
            wire_command_stamp_ns=int(stamp_ns), wire_prior_words=expected.copy(),
            wire_target_words=target_words, wire_acknowledged=False, wire_stationary=False)
        self.wire_count += 1
        self.target_seen = False
        self.expected_words = None
        effort = [0.]*7; effort[axis] = 1.
        return target, effort

    def observe_sdk(self, current, stamp_ns, epoch):
        self.live(epoch)
        try:
            q = vector(current)
        except RuntimeError as error:
            self.fail(str(error))
        if type(stamp_ns) is not int or stamp_ns < self.epoch or stamp_ns < self.latest_sdk_stamp:
            self.fail('SDK reference source stamp regressed or precedes frozen epoch')
        if q[6] != self.reference[6]:
            self.fail('gripper SDK reference changed')
        w = words(q)
        if self.active is None:
            if w != words(self.reference):
                self.fail('unexpected SDK reference before first command')
        else:
            a = self.active
            if a['wire_sent'] == 0:
                if w != a['prior_words']:
                    self.fail('SDK changed before first one-hot publication')
            elif a['wire_acknowledged']:
                if w != a['wire_target_words']:
                    self.fail('SDK target changed after reaching one-hot commanded words')
            else:
                j = a['wire_axis']
                if (any(w[k] != a['wire_prior_words'][k] for k in range(6) if k != j)
                        or not min(a['wire_prior_words'][j], a['wire_target_words'][j]) <= w[j]
                        <= max(a['wire_prior_words'][j], a['wire_target_words'][j])):
                    self.fail('SDK reference outside active one-hot wire segment')
                if w == a['wire_target_words']:
                    if stamp_ns <= a['wire_command_stamp_ns']:
                        self.fail('target SDK evidence precedes publication')
                    a['wire_acknowledged'] = True
                    self.expected_words = w
                    self.target_seen = a['wire_sent'] == 2 and w == a['target_words']
        self.latest_sdk_stamp = max(self.latest_sdk_stamp, stamp_ns)

    def confirm_wire_stationary(self, current, epoch, independent_stationary):
        q = self.assert_feedback(current, epoch)
        a = self.active
        if a is None or not a['wire_acknowledged'] or independent_stationary is not True:
            self.fail('one-hot completion lacks written SDK and independent stationary feedback')
        j = a['wire_axis']
        if a['directions'][j]*(q[j]-a['wire_start'][j]) < .003:
            self.fail('edited axis lacks directional actual response')
        a['wire_stationary'] = True

    def complete(self, current, epoch, independent_stationary):
        q = self.assert_feedback(current, epoch)
        a = self.active
        if (a is None or a['wire_sent'] != 2 or not a['wire_stationary']
                or not self.target_seen or independent_stationary is not True):
            self.fail('completion lacks two one-hot SDK confirmations and independent stationary feedback')
        delta = q[:6]-a['start'][:6]
        if any(a['directions'][j]*delta[j] < .003 for j in EDITED):
            self.fail('both edited axes must show directional actual response')
        a['settled'] = True


def actual_motion_metrics(fk, before, after):
    before, after = vector(before), vector(after)
    first, last = fk(before[:6]), fk(after[:6])
    residual = math.degrees(rotation_angle(first[:3, :3].T @ last[:3, :3]))
    return dict(actual_delta_deg=np.degrees(after[:6]-before[:6]).tolist(),
        actual_joint23_sum_delta_deg=math.degrees(float((after-before)[1:3].sum())),
        actual_fk_rotation_residual_deg=residual,
        actual_fk_rotation_within_one_encoder_word=bool(residual <= math.degrees(QUANTUM)+1e-10),
        simultaneous_counterrotation=False,
        pure_translation_proven=False,
        interpretation='Opposite SDK words are not proof of actual rotation cancellation; no trim is applied.')


def make_diagnostic_class(base):
    class Diagnostic(base.Diagnostic):
        def __init__(self, args, directory):
            self.contract = None
            self.saved_rgb = set()
            self.publish_count = 0
            self.scene_enclosures = {}
            super().__init__(args, directory)
            self.deadline = time.monotonic()+110.

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
                    self.fault = 'counterrotation callback: '+str(error)
                    self.pair_changed.notify_all()
            # State cache is never locked during external bag writes. All pose_ok
            # exact RGB tuples are retained, including diagnostic-only older rows;
            # execution snapshots still enforce the unchanged 350 ms source age.
            for stamp, rgb, quality in pairs:
                now = base.rospy.Time.now()
                try:
                    self.write_bag('/bounded_joint23/paired_rgb', rgb, now)
                    self.write_bag('/bounded_joint23/pair_provenance', base.String(data=json.dumps(dict(
                        source_stamp_ns=stamp, recorded_stamp_ns=now.to_nsec(),
                        camera_model_fingerprint=quality.get('camera_model_fingerprint'),
                        board_frame=quality.get('board_frame'), exact_rgb_board_quality=True))), now)
                except Exception as error:
                    with self.pair_changed:
                        self.fault = 'counterrotation RGB archive: '+str(error)
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
            raise RuntimeError('counterrotation has no bootstrap/sync/enable path')

        def path_cell_partition(self, axes, geometry, openings):
            return rectangle_cells(axes)

        def path_cell_geometry(self, midpoint, radius, geometry, opening):
            from bootstrap_mixed_geometry import certify_local_cell_geometry
            return certify_local_cell_geometry(self.fk,self.gripper,
                np.r_[midpoint,opening],geometry,self.lo,self.hi,math.degrees(radius))

        def path(self, measured, target, geometry, reference=None):
            from bootstrap_mixed_geometry import projection_corner_box_batch
            path_started=time.monotonic()
            reference = base.positions(self.reference[-1]) if reference is None else vector(reference)
            lower, upper, directions = segment_rectangle(measured, target, reference, self.contract.origin)
            if np.any(lower < self.lo) or np.any(upper > self.hi):
                raise RuntimeError('response rectangle exceeds current joint limits')
            axes = rectangle_grid(lower, upper)
            proofs, visited = [], set()
            openings = sorted(set(self.contract.gripper_bounds))
            local_by_opening = {}
            for opening in openings:
                local=np.concatenate([base._box_corners(c,np.eye(3),size) for _,c,size in
                    base._stage_boxes(np.eye(4),self.gripper,opening,'y','z')])
                radii=np.array([max(self.fk.corner_motion_radii(p)) for p in local])
                local_by_opening[opening]=(local,radii)
            maximum_cad_distance=0.
            geometry_seconds=0.;moveit_seconds=0.
            def state_valid(q, opening):
                nonlocal moveit_seconds
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
                before_call=time.monotonic()
                valid=self.validity(request).valid
                moveit_seconds+=time.monotonic()-before_call
                if not valid:
                    raise RuntimeError('current MoveIt scene rejects full response cell grid or SDK bridge')
            def prove_cell(midpoint, radius):
                nonlocal maximum_cad_distance,geometry_seconds
                self.guard()
                for opening in openings:
                    before_geometry=time.monotonic()
                    proof = self.path_cell_geometry(midpoint,radius,geometry,opening)
                    proofs.append(proof)
                    local,radii=local_by_opening[opening];tool=self.fk(midpoint)
                    world=local@tool[:3,:3].T+tool[:3,3]
                    maximum_cad_distance=max(maximum_cad_distance,float(np.max(
                        np.linalg.norm(world-geometry['Y'][:3,3],axis=1)+radius*radii)))
                    geometry_seconds+=time.monotonic()-before_geometry
                    state_valid(midpoint, opening)
            # Held intervals are bisected: their centers are +/-0.2 degrees and
            # half-widths 0.2 degrees for a full +/-0.4 interval. No double count.
            response_cells=0
            for midpoint, radius in self.path_cell_partition(axes,geometry,openings):
                prove_cell(midpoint,radius)
                response_cells+=1
            # Nominal SDK bridges are separately proved. Existing command bias
            # can put a held SDK endpoint outside the physical reverse-response
            # interval; that NEVER expands the accepted-feedback permission.
            # Their endpoints remain inside the ORIGINAL absolute/held bounds.
            # SDK axes can update independently too. Prove their complete
            # command-space box, not merely current->SDK->target diagonals.
            endpoints=np.vstack((measured[:6],reference[:6],target[:6]))
            bridge_lower,bridge_upper=endpoints.min(axis=0),endpoints.max(axis=0)
            bridge_axes=rectangle_grid(bridge_lower,bridge_upper)
            bridge_cells=0
            for midpoint,radius in self.path_cell_partition(bridge_axes,geometry,openings):
                prove_cell(midpoint,max(radius,1e-12))
                bridge_cells+=1
            for first, last in ((measured[:6], target[:6]),
                    (measured[:6], reference[:6]), (reference[:6], target[:6])):
                for fraction in np.linspace(0., 1., 17):
                    for opening in openings:
                        state_valid(first*(1-fraction)+last*fraction, opening)
            corner_lower=np.asarray([p['camera_coordinate_taylor_bound']['corner_lower_xyz_m'] for p in proofs])
            corner_upper=np.asarray([p['camera_coordinate_taylor_bound']['corner_upper_xyz_m'] for p in proofs])
            key=str(geometry['source_stamp_ns'])+'_'+hashlib.sha256(np.r_[lower,upper,target].tobytes()).hexdigest()[:16]
            enclosure_path=self.out/('scene_enclosures_'+key+'.npz')
            np.savez(enclosure_path,lower=corner_lower,upper=corner_upper)
            corner_lower.setflags(write=False);corner_upper.setflags(write=False)
            self.scene_enclosures[key]=(corner_lower,corner_upper)
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
                scene_enclosures_key=key,scene_enclosures_file=str(enclosure_path),
                scene_enclosures_sha256=hashlib.sha256(enclosure_path.read_bytes()).hexdigest(),
                scene_enclosures_shape=list(corner_lower.shape),
                minimum_plane_clearance_lower_bound_m=min(p['minimum_plane_clearance_lower_bound_m'] for p in proofs),
                minimum_full_board_edge_lower_bound_px=min(p['minimum_full_board_edge_lower_bound_px'] for p in proofs),
                minimum_board_depth_lower_bound_m=min(p['minimum_board_depth_lower_bound_m'] for p in proofs),
                hardware_tracking_or_stopping_certified=False, whole_arm_continuous_collision_certified=False)
            before_projection=time.monotonic()
            result['whole_rectangle_scene_proof']=verify_fresh_scene(result,geometry['Y'],geometry['Y'],geometry['model'],projection_corner_box_batch,self.scene_enclosures[key])
            result['proof_timing_seconds']=dict(total=time.monotonic()-path_started,geometry_cells=geometry_seconds,moveit=moveit_seconds,final_scene_projection=time.monotonic()-before_projection)
            return result

        def publish_leg_axis(self, label, axis, target, measured, geometry, details, leg_deadline):
            """One publication only; no retry after reserving or publishing intent."""
            from bootstrap_mixed_geometry import projection_corner_box_batch
            for attempt in range(3):
                fresh = self.snapshot(label+'_j%d_precommand_%d'%(axis+1,attempt))
                if not np.allclose(fresh['X'], geometry['X'], atol=1e-10, rtol=0):
                    raise RuntimeError('hand-eye changed during proof')
                fresh_proof=verify_fresh_scene(details,geometry['Y'],fresh['Y'],fresh['model'],
                    projection_corner_box_batch,self.scene_enclosures[details['scene_enclosures_key']])
                with self.lock:
                    self.guard()
                    if time.monotonic() >= leg_deadline:
                        raise RuntimeError('logical leg deadline exhausted during fresh source acquisition')
                    now = base.rospy.Time.now()
                    source_ns = fresh['source_stamp_ns']
                    age = now.to_nsec()-source_ns
                    if source_ns < self.frozen_epoch or age < 0:
                        raise RuntimeError('precommand source predates epoch or is future')
                    active=self.contract.active
                    previous_ns=active['wire_command_stamp_ns'] if axis == EDITED[1] else 0
                    if age > 350_000_000 or source_ns < previous_ns:
                        continue
                    current=base.positions(self.accepted[-1])
                    reference=base.positions(self.reference[-1])
                    if axis == EDITED[0]:
                        if np.max(abs(current[:6]-measured[:6])) > math.radians(.15):
                            raise RuntimeError('measured start changed during path proof')
                        self.contract.begin(label,target,current,reference,self.epoch,now.to_nsec())
                        self.contract.intersect_proved_rectangle(details['lower_rad'],details['upper_rad'])
                    else:
                        # Independently re-read the trailing stationary window;
                        # a previously stationary J2 is not a perpetual permit.
                        self.stationary(previous_ns)
                    self.contract.assert_feedback(current,self.epoch)
                    if self.publisher.get_num_connections() < 1:
                        raise RuntimeError('driver command connection missing')
                    payload,effort=self.contract.reserve_wire(axis,reference,current,self.epoch,now.to_nsec(),
                        self.accepted[-1].header.stamp.to_nsec(),self.reference[-1].header.stamp.to_nsec(),source_ns)
                    self.expected_words = None
                    msg=base.JointState()
                    msg.header.stamp,msg.header.frame_id=now,'gui_direct'
                    msg.name,msg.position,msg.effort=base.NAMES,payload.tolist(),effort
                    self.guard()
                    publish_time=base.rospy.Time.now()
                    if time.monotonic() >= leg_deadline:
                        raise RuntimeError('logical leg deadline exhausted immediately before publication')
                    validate_source_binding(publish_time.to_nsec(),self.frozen_epoch,
                        self.accepted[-1].header.stamp.to_nsec(),self.reference[-1].header.stamp.to_nsec(),source_ns)
                    msg.header.stamp=publish_time
                    self.contract.active['wire_command_stamp_ns']=publish_time.to_nsec()
                    if axis == EDITED[0]:
                        self.contract.active['command_stamp_ns']=publish_time.to_nsec()
                    self.publish_count += 1
                    self.publisher.publish(msg)
                    sent_ns=publish_time.to_nsec()
                    self.event('command',label=label,edited_joint=axis+1,
                        logical_leg_number=self.contract.count,one_hot_part=self.contract.active['wire_sent'],
                        command_number=self.publish_count,command_stamp_ns=sent_ns,
                        starting_measured_rad=current.tolist(),target_rad=payload.tolist(),
                        logical_target_rad=target.tolist(),expected_wire_words=words(payload),
                        preceding_sdk_words=words(reference),source_stamp_ns=source_ns,
                        accepted_source_stamp_ns=self.accepted[-1].header.stamp.to_nsec(),
                        preceding_sdk_source_stamp_ns=self.reference[-1].header.stamp.to_nsec(),
                        precommand_source_age_ms=(sent_ns-source_ns)/1e6,
                        fresh_scene_continuous_proof=fresh_proof,**details)
                    return sent_ns
            raise RuntimeError('fresh source expired during all bounded precommand acquisitions')

        def await_onehot_stationary(self, label, axis, sent_ns, leg_deadline):
            while time.monotonic() < min(self.deadline,leg_deadline):
                self.guard()
                with self.lock:
                    current=base.positions(self.accepted[-1]);active=self.contract.active
                    sdk_seen=active['wire_acknowledged']
                    responding=active['directions'][axis]*(current[axis]-active['wire_start'][axis]) >= .003
                    accepted_ns=self.accepted[-1].header.stamp.to_nsec()
                    sdk_ns=self.reference[-1].header.stamp.to_nsec()
                try:
                    stationary=self.stationary(sent_ns)
                except ValueError:
                    stationary=None
                if stationary is not None and sdk_seen and responding and accepted_ns >= sent_ns and sdk_ns > sent_ns:
                    with self.lock:
                        self.guard()
                        current=base.positions(self.accepted[-1])
                        # The latest source must still be the exact one-hot endpoint.
                        self.contract.observe_sdk(base.positions(self.reference[-1]),
                            self.reference[-1].header.stamp.to_nsec(),self.epoch)
                        stationary=self.stationary(sent_ns)
                        self.contract.confirm_wire_stationary(current,self.epoch,True)
                        self.event('one_hot_sdk_confirmed_and_stationary',label=label,edited_joint=axis+1,
                            command_stamp_ns=sent_ns,sdk_source_stamp_ns=self.reference[-1].header.stamp.to_nsec(),
                            accepted_source_stamp_ns=self.accepted[-1].header.stamp.to_nsec(),
                            actual_sdk_words=words(base.positions(self.reference[-1])),
                            measured_rad=current.tolist(),stationary=stationary)
                    return current,stationary
                time.sleep(.02)
            raise RuntimeError('one-hot J%d lacks written SDK and stationary actual response before leg deadline'%(axis+1))

        def execute_step(self, label, target):
            if not self.args.execute or self.publisher is None:
                raise RuntimeError('preparation process cannot publish')
            self.guard()
            self.stationary(max(self.started_ns,self.epoch))
            geometry=self.snapshot(label)
            with self.lock:
                measured=base.positions(self.accepted[-1]);reference=base.positions(self.reference[-1])
            details=self.path(measured,target,geometry,reference)
            self.driver_connection()
            # One leg shares a 12 s response budget; intermediate observation
            # cannot extend it or allow a repeated/automatic recovery message.
            leg_deadline=min(self.deadline,time.monotonic()+12.)
            for axis in EDITED:
                if time.monotonic() >= leg_deadline:
                    raise RuntimeError('logical leg deadline exhausted before next one-hot publication')
                sent_ns=self.publish_leg_axis(label,axis,target,measured,geometry,details,leg_deadline)
                current,stationary=self.await_onehot_stationary(label,axis,sent_ns,leg_deadline)
            with self.lock:
                self.contract.complete(current,self.epoch,True)
            self.event('settled',label=label,stationary=stationary,
                endpoint_sequence='J2 then J3, each SDK-confirmed and independently stationary',
                **actual_motion_metrics(self.fk,measured,current))
            self.stable_hold_recording(3.)
            observed=self.snapshot(label+'_post_hold')
            self.event('post_hold_observation',label=label,source_stamp_ns=observed['source_stamp_ns'],
                paired_rgb_count=len(self.saved_rgb),**actual_motion_metrics(self.fk,self.contract.origin,observed['q']))
            return observed

        def run(self):
            reference, stationary = self.await_initial()
            geometry = self.snapshot('baseline')
            with self.lock:
                self.guard()
                measured = base.positions(self.accepted[-1])
                self.contract = CounterrotationContract(measured, reference, self.frozen_epoch, self.args.step_deg,
                    positive_only=getattr(self.args,'positive_only',False))
                self.expected_words = words(reference)
            plan = dict(schema=2, execute=self.args.execute, edited_joints=[2,3],
                initial_measured_rad=measured.tolist(), starting_sdk_rad=reference.tolist(),
                frozen_epoch_ns=self.frozen_epoch, step_words=self.contract.step_words,
                logical_legs=[dict(label=label, target_rad=target.tolist(), words=words(target),
                    one_hot_joint_order=[2,3])
                    for label,target in zip(self.contract.labels,self.contract.targets)],
                source='fresh current process only', bootstrap_pending=False,
                maximum_commands=self.contract.maximum_commands,
                experiment_mode='positive_only_out_and_back' if self.contract.positive_only else 'positive_and_negative_out_and_back',
                overall_deadline_sec=110, stationary=stationary,
                recovery='none; no sync, enable, trim, stop or disable commands',
                archive_policy='local_only', upload_enabled=False, delete_enabled=False)
            if not self.args.execute:
                plan['prepare_geometry'] = {}
                directions=[('positive',self.contract.targets[0])]
                if not self.contract.positive_only:
                    directions.append(('negative',self.contract.targets[2]))
                for label,target in directions:
                    self.stationary(max(self.started_ns,self.epoch))
                    current_geometry=self.snapshot('prepare_'+label)
                    with self.lock:
                        current_measured=base.positions(self.accepted[-1]);current_reference=base.positions(self.reference[-1])
                    details=self.path(current_measured,target,current_geometry,current_reference)
                    plan['prepare_geometry'][label]=details
                    self.event('prepare_direction_proved',label=label,**details)
                base.write_json(self.out/'plan.json',plan)
                self.event('prepared_no_motion', plan=str(self.out/'plan.json'))
                return
            base.write_json(self.out/'plan.json',plan)
            for label,target in zip(self.contract.labels,self.contract.targets):
                self.execute_step(label,target)
            self.event('completed_declared_commands_same_sdk_return', commands_published=self.publish_count,
                experiment_mode=plan['experiment_mode'],maximum_commands=self.contract.maximum_commands,
                original_words=words(reference), final_words=words(base.positions(self.reference[-1])),
                paired_rgb_count=len(self.saved_rgb), pure_translation_proven=False)
    return Diagnostic


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execute', action='store_true')
    parser.add_argument('--positive-only',action='store_true',
        help='Predeclare positive -> original SDK: two legs, four one-hot messages maximum; no fallback.')
    parser.add_argument('--step-deg', type=float, default=2., choices=(2.,3.,4.))
    args = parser.parse_args(argv)
    args.bootstrap_pending = False
    args.bootstrap_sync_bound_deg = 4.8
    args.joint = 2
    return args


def main():
    args = parse_args()
    if not os.path.ismount(str(MOUNT)) or not ARCHIVE.is_dir():
        raise RuntimeError('SanStand archive unavailable; no system-disk fallback')
    directory = ARCHIVE/('bounded_joint23_counterrotation_'+datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')+'_'+str(os.getpid()))
    directory.mkdir(); (directory/'ros').mkdir()
    os.environ['ROS_LOG_DIR'] = str(directory/'ros')
    source_names = ('bounded_joint23_counterrotation.py','bounded_joint_response_diagnostic.py',
                    'bootstrap_mixed_geometry.py','coupled_joint23_geometry.py')
    manifest = {}
    for name in source_names:
        data = (HERE/name).read_bytes(); (directory/name).write_bytes(data)
        manifest[name] = hashlib.sha256(data).hexdigest()
    (directory/'source_sha256.json').write_text(json.dumps(manifest,indent=2)+'\n')
    spec = importlib.util.spec_from_file_location('bounded_joint23_base',HERE/'bounded_joint_response_diagnostic.py')
    base = importlib.util.module_from_spec(spec); spec.loader.exec_module(base)
    base.rospy.init_node('bounded_joint23_counterrotation', anonymous=True, disable_signals=True)
    terminal = dict(status='failed',execute=args.execute,positive_only=args.positive_only,
        maximum_commands=(4 if args.positive_only else 8) if args.execute else 0)
    experiment = None
    def alarm(_signum,_frame):
        raise RuntimeError('110-second overall deadline exhausted; no further command')
    signal.signal(signal.SIGALRM,alarm); signal.setitimer(signal.ITIMER_REAL,110.)
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
