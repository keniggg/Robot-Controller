#!/usr/bin/env python3
"""Bounded diagnostic observation, not a grasp/track or calibration permit.

Default: fresh exact RGB/Z16/board association, one three-axis solve and first
step geometry proof, then exit without a publisher. Explicit execution can send
at most one one-hot command to each of J1/J2/J3. No return or automatic retry.
Historical evidence supplies only a board-local candidate neighbourhood; every
joint target and geometry proof is newly computed. A candidate is never promoted
to a production track here. The existing tracker must acquire it independently.
"""
import argparse
from collections import OrderedDict
from datetime import datetime, timezone
import hashlib
import importlib.util
import itertools
import io
import json
import math
import os
from pathlib import Path
import signal
import sys
import time

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import bounded_wrist_response_diagnostic as wrist
from coupled_joint23_geometry import words
from bounded_async_bag_writer import BoundedAsyncBagWriter

MOUNT, ARCHIVE = wrist.MOUNT, wrist.ARCHIVE
QUANTUM = wrist.QUANTUM
EDITED = (0, 1, 2)
LIMIT = math.radians(3.)
ABSOLUTE = wrist.ABSOLUTE_RAD
HELD, REVERSE = wrist.HELD_RAD, wrist.REVERSE_RAD
DEPTH = '/supervisor/camera/depth/image_raw'


def load_module(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def quantized_deltas(delta):
    delta = np.asarray(delta, dtype=float)
    if delta.shape != (3,) or not np.isfinite(delta).all() or np.any(abs(delta) > LIMIT+1e-10):
        raise RuntimeError('only three finite deltas within +/-3 degrees are admissible')
    # Truncate toward zero: quantization must never expand the advertised bound.
    increments = np.trunc(delta/QUANTUM).astype(int)
    increments[abs(increments) < 2] = 0
    return increments


def unique_component(pixels, points_board, candidate, prior_center, prior_extent):
    """Raw 2-pixel-grid connectivity; no dilation that could merge two objects."""
    import cv2
    pixels, points_board = np.asarray(pixels), np.asarray(points_board)
    candidate = np.asarray(candidate, dtype=bool)
    mask = np.zeros((240, 320), dtype=np.uint8)
    mask[pixels[candidate, 1]//2, pixels[candidate, 0]//2] = 1
    count, labels, stats, _ = cv2.connectedComponentsWithStats(mask, 8)
    significant = [i for i in range(1, count) if stats[i, 4] >= 15]
    if len(significant) != 1:
        raise RuntimeError('motor neighbourhood does not contain exactly one significant depth component')
    selected = candidate & (labels[pixels[:, 1]//2, pixels[:, 0]//2] == significant[0])
    if selected.sum() < .85*candidate.sum():
        raise RuntimeError('motor component has excessive fragmented or competing surface')
    center = np.median(points_board[selected], axis=0)
    extent = np.ptp(np.percentile(points_board[selected], [10, 90], axis=0), axis=0)
    if np.linalg.norm(center[:2]-np.asarray(prior_center)[:2]) > .008:
        raise RuntimeError('candidate left historical board-local motor neighbourhood')
    ratio = extent[:2]/np.asarray(prior_extent)[:2]
    if np.any(ratio < .5) or np.any(ratio > 2.):
        raise RuntimeError('candidate surface extent inconsistent with previous motor observation')
    pixel=np.median(pixels[selected],axis=0)
    return dict(observed_pixel_median=pixel.tolist(), observed_pixel_radius_px=float(np.linalg.norm(pixel-[320.,240.])),
        observed_within_center_60px=bool(np.linalg.norm(pixel-[320.,240.])<60.),
        center_board_m=center.tolist(), surface_extent_board_m=extent.tolist(),
        selected_points=int(selected.sum()), significant_components=1,
        production_target_identity_verified=False, association='board_local_diagnostic_only')


def solve_three_axes(fk, geometry, point_board):
    """One bounded minimum-norm visual solve; no tool-orientation objective."""
    from scipy.optimize import least_squares
    q0 = np.asarray(geometry['q'])[:6]
    X, Y, model = geometry['X'], geometry['Y'], geometry['model']
    world = Y[:3, :3]@np.asarray(point_board)+Y[:3, 3]
    def project(delta):
        q = q0+np.r_[delta, np.zeros(3)]
        camera = np.linalg.inv(fk(q)@X)
        p = camera[:3, :3]@world+camera[:3, 3]
        if p[0] <= .12:
            raise RuntimeError('motor predicted depth below 120 mm')
        return np.asarray(model.sdk.rs2_project_point_to_pixel(model.intrinsics, [-p[1], -p[2], p[0]]))
    initial = project(np.zeros(3))
    if np.linalg.norm(initial-[320., 240.]) <= 50.:
        return np.zeros(3, dtype=int), dict(initial_pixel=initial.tolist(), predicted_pixel=initial.tolist(),
            predicted_radius_px=float(np.linalg.norm(initial-[320., 240.])), already_centered=True)
    def residual(d):
        return np.r_[(project(d)-[295., 210.])/10., d/math.radians(5.)*.12]
    def jac(d):
        h = 1e-4  # SDK float32 projection: smaller automatic finite differences vanish.
        return np.column_stack([(residual(d+np.eye(3)[i]*h)-residual(d-np.eye(3)[i]*h))/(2*h) for i in range(3)])
    fit = least_squares(residual, np.zeros(3), jac=jac, bounds=(-LIMIT, LIMIT), max_nfev=160,
                        ftol=1e-9, xtol=1e-9, gtol=1e-9)
    if not fit.success:
        raise RuntimeError('bounded fresh centering solve did not converge')
    increments = quantized_deltas(fit.x)
    final = project(increments*QUANTUM)
    before, after = np.linalg.norm(initial-[320.,240.]), np.linalg.norm(final-[320.,240.])
    if after >= before-2. and np.any(increments):
        raise RuntimeError('quantized three-axis proposal does not improve centering by two pixels')
    return increments, dict(initial_pixel=initial.tolist(), continuous_delta_deg=np.degrees(fit.x).tolist(),
        quantized_delta_deg=np.degrees(increments*QUANTUM).tolist(), predicted_pixel=final.tolist(),
        predicted_radius_px=float(after), predicted_within_center_60px=bool(after < 60.),
        jacobian_step_rad=1e-4, optimizer_cost=float(fit.cost), solve_count=1,
        fallback='none; a completed finite observation may require separate new evidence')


class CenteringContract:
    """One frozen proposal, unique J1/J2/J3 publications, exact SDK ACKs."""
    def __init__(self, measured, reference, epoch, increments):
        self.origin, self.reference = wrist.vector(measured), wrist.vector(reference)
        if type(epoch) is not int or epoch <= 0:
            raise RuntimeError('positive driver epoch required')
        increments = np.asarray(increments)
        if increments.shape != (3,) or not np.equal(increments, increments.astype(int)).all():
            raise RuntimeError('three integral wire deltas required')
        if np.any(abs(increments)*QUANTUM > LIMIT) or np.any((abs(increments) == 1)):
            raise RuntimeError('wire deltas exceed 3 degrees or cannot demonstrate actual response')
        self.epoch = epoch
        self.indices = [j for j in EDITED if increments[j] != 0]
        self.labels = ['center_J%d' % (j+1) for j in self.indices]
        self.targets = []
        target = self.reference.copy()
        for j in self.indices:
            target = target.copy(); target[j] += int(increments[j])*QUANTUM
            if np.any(abs(target[:6]-self.origin[:6]) > ABSOLUTE):
                raise RuntimeError('nominal SDK target escapes initial +/-4.8 degrees')
            self.targets.append(target)
        self.gripper_bounds = (min(self.origin[6], self.reference[6]), max(self.origin[6], self.reference[6]))
        self.count, self.active, self.failed = 0, None, None
        self.expected_words, self.target_seen, self.latest_sdk_stamp = words(reference), False, 0
        self.index = None
        self.assert_feedback(self.origin, epoch)

    def fail(self, message):
        self.failed = self.failed or message
        raise RuntimeError(self.failed)

    def live(self, epoch):
        if self.failed: raise RuntimeError(self.failed)
        if epoch != self.epoch: self.fail('driver epoch changed')

    def assert_feedback(self, current, epoch):
        self.live(epoch)
        q = wrist.vector(current)
        allowed = np.array([ABSOLUTE]*3+[HELD]*3) if self.count else np.full(6,HELD)
        if np.any(abs(q[:6]-self.origin[:6]) > allowed):
            self.fail('actual feedback escaped initial absolute/held-axis bounds')
        if not self.gripper_bounds[0] <= q[6] <= self.gripper_bounds[1]:
            self.fail('actual gripper escaped frozen interval')
        if self.active is not None:
            a = self.active
            held=[j for j in range(6) if j!=self.index]
            if np.any(abs(q[held]-a['start'][held]) > HELD):
                self.fail('unedited axis exceeds independent per-step held bound')
            if np.any(q[:6] < a['lower']) or np.any(q[:6] > a['upper']):
                self.fail('actual feedback escaped proved per-step independent box')
            if a['directions'][self.index]*(q[self.index]-a['start'][self.index]) < -REVERSE:
                self.fail('active axis reversed more than 0.5 degrees')
        return q

    def rectangle(self, measured, target, reference, index):
        measured, target, reference = map(wrist.vector, (measured, target, reference))
        global_width = np.array([ABSOLUTE]*3+[HELD]*3)
        lo = np.maximum(self.origin[:6]-global_width, measured[:6]-HELD)
        hi = np.minimum(self.origin[:6]+global_width, measured[:6]+HELD)
        if np.any(abs(reference[:6]-self.origin[:6]) > ABSOLUTE):
            raise RuntimeError('initial SDK bias exceeds explicit nominal bridge bounds')
        difference = target[index]-measured[index]
        if abs(difference) < .003:
            raise RuntimeError('nominal edited target lacks measurable directional response')
        direction = math.copysign(1., difference)
        if direction*(target[index]-reference[index]) <= 0:
            raise RuntimeError('preserved SDK bias reverses intended actual displacement')
        lo[index] = max(self.origin[index]-ABSOLUTE, measured[index]-(REVERSE if direction>0 else ABSOLUTE))
        hi[index] = min(self.origin[index]+ABSOLUTE, measured[index]+(ABSOLUTE if direction>0 else REVERSE))
        dirs = np.zeros(6); dirs[index] = direction
        return lo, hi, dirs

    def begin(self, label, target, measured, current_reference, epoch, stamp_ns):
        self.live(epoch)
        if self.count >= len(self.indices) or label != self.labels[self.count]:
            self.fail('finite centering sequence exhausted or reordered')
        if self.active is not None and not self.active.get('settled'):
            self.fail('preceding one-hot command lacks SDK ACK and independent stationary completion')
        predecessor = self.reference if self.count == 0 else self.targets[self.count-1]
        if words(current_reference) != words(predecessor) or current_reference[6] != self.reference[6]:
            self.fail('SDK sequence predecessor changed')
        if not np.array_equal(target, self.targets[self.count]):
            self.fail('command differs from single frozen fresh proposal')
        current = self.assert_feedback(measured, epoch)
        self.index = self.indices[self.count]
        lo, hi, dirs = self.rectangle(current, target, current_reference, self.index)
        self.active = dict(start=current, lower=lo, upper=hi, directions=dirs,
            prior_words=words(predecessor), target_words=words(target), command_stamp_ns=int(stamp_ns), settled=False)
        self.count += 1
        self.target_seen, self.expected_words = False, None

    def observe_sdk(self, current, stamp_ns, epoch):
        self.live(epoch)
        q = wrist.vector(current); stamp_ns = int(stamp_ns)
        if stamp_ns < self.epoch or stamp_ns < self.latest_sdk_stamp:
            self.fail('SDK source predates epoch or regressed')
        if q[6] != self.reference[6]: self.fail('SDK gripper changed')
        w = words(q)
        if self.active is None:
            if w != words(self.reference): self.fail('SDK changed before first publication')
        else:
            a = self.active
            if self.target_seen and w != a['target_words']:
                self.fail('SDK changed after acknowledged one-hot target')
            if any(w[j] != a['prior_words'][j] for j in range(6) if j != self.index):
                self.fail('uncommanded SDK axis changed')
            if not min(a['prior_words'][self.index],a['target_words'][self.index]) <= w[self.index] <= max(a['prior_words'][self.index],a['target_words'][self.index]):
                self.fail('SDK active word left bounded segment')
            if w == a['target_words']:
                if stamp_ns <= a['command_stamp_ns']: self.fail('SDK endpoint is not newer than publication')
                self.target_seen, self.expected_words = True, w
        self.latest_sdk_stamp = stamp_ns

    def complete(self, current, epoch, independent_stationary):
        q = self.assert_feedback(current, epoch)
        if self.active is None or not self.target_seen or independent_stationary is not True:
            self.fail('completion lacks new written SDK ACK and independent 0.8-second stationary evidence')
        if self.active['directions'][self.index]*(q[self.index]-self.active['start'][self.index]) < .003:
            self.fail('actual axis response insufficient')
        self.active['settled'] = True


def joint_cells(lower, upper):
    # Zero-width axes remain exact, not an invented isotropic ball.
    axes = [np.linspace(a,b,max(1,int(math.ceil((b-a)/math.radians(.4)-1e-12)))+1)
            for a,b in zip(lower,upper)]
    for indices in itertools.product(*(range(len(axis)-1) for axis in axes)):
        lo = np.array([axis[i] for axis,i in zip(axes,indices)])
        hi = np.array([axis[i+1] for axis,i in zip(axes,indices)])
        yield .5*(lo+hi), .5*(hi-lo)


def table_clearance_from_cad_enclosure(corners_base, motion_radius_m, table_plane_base, motor_top_m):
    """Evaluate a NEW measured plane over the SAME full continuous CAD cells.

    Each corner remains in its proved radius ball. Keeping n and d together
    avoids a base-origin dependent |delta d| + |delta n|*|point| debit. This
    does not shrink the response box or waive the 40 mm physical clearance.
    """
    corners=np.asarray(corners_base,dtype=float)
    radius=np.asarray(motion_radius_m,dtype=float)
    plane=np.asarray(table_plane_base,dtype=float)
    if (corners.ndim!=2 or corners.shape[1]!=3 or radius.shape!=(len(corners),)
            or not len(corners) or plane.shape!=(4,)
            or not np.isfinite(corners).all() or not np.isfinite(radius).all()
            or not np.isfinite(plane).all() or np.any(radius<0)
            or not math.isfinite(motor_top_m) or not 0<=motor_top_m<=.04
            or float(np.linalg.norm(plane[:3]))<1e-8 or plane[2]<=0):
        raise RuntimeError('invalid continuous CAD or independently measured table enclosure')
    plane=plane/float(np.linalg.norm(plane[:3]))
    distances=corners@plane[:3]+plane[3]-radius-float(motor_top_m)
    index=int(np.argmin(distances));bound=float(distances[index])
    return dict(minimum_adjacent_table_plus_visible_motor_clearance_m=bound,
        required_clearance_m=.04,passes_40mm=bool(bound>=.04),
        evaluated_cad_corners=len(corners),limiting_corner_base_m=corners[index].tolist(),
        limiting_corner_motion_radius_m=float(radius[index]),
        table_plane_base=plane.tolist(),visible_motor_top_m=float(motor_top_m),
        proof='fresh plane evaluated directly over every stored continuous CAD corner enclosure')


def make_diagnostic_class(base):
    parent = wrist.make_diagnostic_class(base)
    class Diagnostic(parent):
        def __init__(self, args, directory):
            self.depth = OrderedDict()
            self.receipts = OrderedDict()
            self.production_rgb, self.production_depth = OrderedDict(), OrderedDict()
            self.production_pending, self.production_saved = set(), set()
            self.motor_snapshots = 0
            self.motor_reference = None
            self.table_path = None
            self.writer_summary = None
            self.receive_active = 0
            def archive_failure(reason):
                with self.pair_changed:
                    self.fault='centering archive: '+str(reason)
                    self.pair_changed.notify_all()
            def write_raw(metadata,payload,pytype):
                ns=metadata['stamp_ns'];stamp=base.rospy.Time(ns//1_000_000_000,ns%1_000_000_000)
                self.bag.write(metadata['topic'],(metadata['datatype'],payload,metadata['md5sum'],pytype),stamp,raw=True)
            self.archive_writer=BoundedAsyncBagWriter(write_raw,archive_failure)
            self.audit = load_module('motor_centering_audit', HERE/'audit_motor_adjacent_table.py')
            self.seed_report = json.loads(args.motor_evidence.read_text())
            if self.seed_report.get('production_target_identity_verified') is not False:
                raise RuntimeError('expected explicit diagnostic motor evidence')
            self.seed_digest = hashlib.sha256(args.motor_evidence.read_bytes()).hexdigest()
            rows = self.seed_report['frames']
            self.anchor = np.asarray(self.seed_report['manual_motor_seed']['anchor_board'])
            self.prior_center = np.median([r['motor_board_surface_median_m'] for r in rows], axis=0)
            self.prior_extent = np.median([np.diff(r['motor_board_surface_p10_p90_m'],axis=0)[0] for r in rows],axis=0)
            self.seed_camera_fingerprint = {r['camera_model_fingerprint'] for r in rows}
            if len(self.seed_camera_fingerprint) != 1 or np.any(self.prior_extent[:2] <= .002):
                raise RuntimeError('ambiguous or degenerate historical motor evidence')
            super().__init__(args, directory)
            from alicia_flexible_grasp_supervisor.msg import ObjectPose
            for prefix in ('/perception/','/perception/unknown/'):
                for suffix,kind in (('object_mask',base.Image),('detector_status',base.String),('object',ObjectPose)):
                    topic=prefix+suffix
                    self.subs.append(base.rospy.Subscriber(topic,kind,lambda msg,topic=topic:self.receive(topic,msg),queue_size=4,buff_size=2**24))
            self.subs.append(base.rospy.Subscriber('/grasp_mode/selection',base.String,
                lambda msg:self.receive('/grasp_mode/selection',msg),queue_size=2))
            self.deadline = time.monotonic()+180.

        def write_bag(self,topic,msg,stamp):
            # Immutable serialized bytes account for the actual bounded payload.
            # Source/receipt ROS stamps supplied by the existing collector are
            # preserved, not replaced with the later disk-writer time.
            buffer=io.BytesIO()
            try:msg.serialize(buffer)
            except BaseException as error:
                self.archive_writer.latch_failure('archive message serialization failed: '+str(error))
                raise
            self.archive_writer.submit(topic,buffer.getvalue(),stamp.to_nsec(),
                msg._type,msg._md5sum,msg.__class__)

        def guard(self):
            self.archive_writer.require_healthy()
            super().guard()
            self.archive_writer.require_healthy()

        def close(self):
            with self.pair_changed:
                self.closed=True;self.pair_changed.notify_all()
            for sub in self.subs:sub.unregister()
            with self.pair_changed:
                if not self.pair_changed.wait_for(lambda:self.receive_active==0,timeout=5.):
                    self.archive_writer.latch_failure('callbacks did not finish before archive close')
            try:
                self.writer_summary=self.archive_writer.close(timeout=30.)
                self.bag.close()
                # A failed individual write is retained in the bounded budget.
                # Archive its exact serialized payload separately and report it;
                # do not pretend the principal bag is complete.
                for index,(metadata,payload,_pytype,_cost) in enumerate(self.archive_writer.failed_payloads):
                    path=self.out/('archive_failed_message_%03d.bin'%index)
                    path.write_bytes(payload)
                    base.write_json(path.with_suffix('.json'),dict(metadata,
                        payload_sha256=hashlib.sha256(payload).hexdigest()))
            finally:
                self.writer_summary=self.archive_writer.summary()
                base.write_json(self.out/'archive_writer_summary.json',self.writer_summary)
                self.events.close()

        def receive(self,topic,msg):
            with self.pair_changed:
                if self.closed:return
                self.receive_active+=1
            try:return self._receive(topic,msg)
            finally:
                with self.pair_changed:
                    self.receive_active-=1;self.pair_changed.notify_all()

        def _receive(self, topic, msg):
            production_pairs=[]
            with self.lock:
                if not self.closed:
                    received_ns=base.rospy.Time.now().to_nsec()
                    stamps=[]
                    if topic in (DEPTH,base.RGB): stamps=[msg.header.stamp.to_nsec()]
                    elif topic=='/charuco/quality':
                        try: stamps=[json.loads(msg.data)['source_stamp_ns']]
                        except (ValueError,KeyError): pass
                    elif topic=='/tf': stamps=[t.header.stamp.to_nsec() for t in msg.transforms if t.header.frame_id=='camera_link']
                    for stamp in stamps: self.cache(self.receipts,(topic,stamp),received_ns,400)
                    if topic == DEPTH:
                        self.cache(self.depth, msg.header.stamp.to_nsec(), msg, 12)
                        self.cache(self.production_depth,msg.header.stamp.to_nsec(),msg,32)
                        self.pair_generation += 1; self.pair_changed.notify_all()
                    elif topic == base.RGB:
                        self.cache(self.production_rgb,msg.header.stamp.to_nsec(),msg,32)
                    elif topic in ('/perception/object_mask','/perception/unknown/object_mask'):
                        self.production_pending.add(msg.header.stamp.to_nsec())
                    for source in list(self.production_pending):
                        if source in self.production_saved:
                            self.production_pending.discard(source)
                        elif source in self.production_rgb and source in self.production_depth:
                            production_pairs.append((source,self.production_rgb[source],self.production_depth[source]))
                            self.production_saved.add(source);self.production_pending.discard(source)
                        elif received_ns-source>2_000_000_000:
                            self.production_pending.discard(source)
            super().receive(topic, msg)
            for source,rgb,depth in production_pairs:
                stamp=base.rospy.Time.now()
                self.write_bag('/bounded_motor/production_exact_rgb',rgb,stamp)
                self.write_bag('/bounded_motor/production_exact_depth',depth,stamp)
                self.write_bag('/bounded_motor/production_pair_provenance',base.String(data=json.dumps(dict(
                    rgb_source_stamp_ns=source,depth_source_stamp_ns=source,mask_source_stamp_ns=source,
                    recorded_stamp_ns=stamp.to_nsec(),identity_verified_by_client=False))),stamp)

        def snapshot(self, label):
            # Base enforces exact RGB/quality/board, SDK model and source age.
            deadline = min(self.deadline, time.monotonic()+4.)
            while True:
                g = super().snapshot(label)
                ns = g['source_stamp_ns']
                with self.lock:
                    depth = self.depth.get(ns); rgb = self.rgb.get(ns)
                if depth is not None and rgb is not None: break
                if time.monotonic() >= deadline: raise RuntimeError('no exact fresh Z16/RGB/board tuple')
                with self.pair_changed: self.pair_changed.wait(timeout=.02)
            if g['model'].fingerprint not in self.seed_camera_fingerprint:
                raise RuntimeError('motor evidence camera fingerprint changed')
            if self.frozen_epoch != self.seed_report['driver_epoch_ns']:
                raise RuntimeError('diagnostic board-layout evidence predates current driver epoch')
            from cv_bridge import CvBridge
            bridge = CvBridge(); image = bridge.imgmsg_to_cv2(rgb,'bgr8'); z16 = bridge.imgmsg_to_cv2(depth,'passthrough')
            row = json.loads((self.out/(label+'_geometry.json')).read_text())
            with self.lock:
                receipt={topic:self.receipts.get((topic,ns)) for topic in (DEPTH,base.RGB,'/charuco/quality','/tf')}
            if any(stamp is None or not 0<=stamp-ns<=350_000_000 for stamp in receipt.values()):
                raise RuntimeError('exact RGB/Z16/board/quality receipt evidence missing or delayed')
            row.update(offline_stationary_phase=label,
                quality_received_age_ms=(receipt['/charuco/quality']-ns)/1e6,tf_received_age_ms=(receipt['/tf']-ns)/1e6)
            stem = 'motor_%03d' % self.motor_snapshots; self.motor_snapshots += 1
            report = self.audit.frame(row,image,z16,self.anchor,self.out,stem)
            arrays = np.load(self.out/(stem+'.npz'))
            association = unique_component(arrays['pixels'],arrays['points_board'],arrays['motor_candidate'],
                self.prior_center,self.prior_extent)
            if self.motor_reference is not None and np.linalg.norm(np.asarray(association['center_board_m'])-self.motor_reference) > .008:
                raise RuntimeError('fresh motor board-local surface changed beyond bounded view allowance')
            report.update(association=association, motor_evidence_sha256=self.seed_digest,
                board_reference_id=self.seed_report['board_reference_id'], driver_epoch_ns=self.frozen_epoch,
                received_stamp_ns=receipt,
                source_age_ms=(base.rospy.Time.now().to_nsec()-ns)/1e6,
                diagnostic_only=True, production_target_identity_verified=False)
            base.write_json(self.out/(stem+'.json'),report)
            if self.motor_reference is None: self.motor_reference=np.asarray(association['center_board_m'])
            wrist.validate_source_binding(base.rospy.Time.now().to_nsec(),self.frozen_epoch,
                self.accepted[-1].header.stamp.to_nsec(),self.reference[-1].header.stamp.to_nsec(),ns)
            g['motor'] = association
            plane=np.asarray(report['table_plane_base'])
            if plane[2]<=0: raise RuntimeError('adjacent table normal is not upward in base frame')
            camera_plane=np.asarray(report['table_plane_camera'])
            motor_top=float(np.max(arrays['points_camera'][arrays['motor_candidate']]@camera_plane[:3]+camera_plane[3]))
            g['table_plane']=plane;g['motor_top_height_m']=motor_top
            if '_precommand_' in label and self.table_path is not None:
                proof=table_clearance_from_cad_enclosure(self.table_path[0],self.table_path[1],plane,motor_top)
                proof.update(source_stamp_ns=ns,cell_enclosure_archive=self.table_path[2])
                base.write_json(self.out/(label+'_fresh_table_proof.json'),proof)
                if not proof['passes_40mm']:
                    raise RuntimeError('fresh independent adjacent table/motor is below 40 mm continuous noncontact clearance')
            return g

        def path(self, measured, target, geometry, reference=None):
            from bootstrap_mixed_geometry import certify_joint_box_geometry, projection_corner_box_batch
            reference = base.positions(self.reference[-1]) if reference is None else wrist.vector(reference)
            lower,upper,directions = self.contract.rectangle(measured,target,reference,self.args.joint-1)
            if np.any(lower < self.lo) or np.any(upper > self.hi):
                raise RuntimeError('actual response box exceeds joint limits')
            endpoints=np.vstack((measured[:6],reference[:6],target[:6]))
            bridge_lower,bridge_upper=endpoints.min(0),endpoints.max(0)
            proofs=[]; visited=set(); maximum_distance=0.; table_clearance=float('inf'); counts=[]
            cad_world_corners=[];cad_motion_radii=[]
            openings=sorted(set(self.contract.gripper_bounds))
            def valid_state(q,opening):
                self.guard();key=tuple(q)+(opening,)
                if key not in visited:
                    request=base.GetStateValidityRequest(group_name='alicia');request.robot_state=base.RobotState()
                    request.robot_state.joint_state=base.JointState(name=base.NAMES,position=list(q)+[opening])
                    if not self.validity(request).valid: raise RuntimeError('MoveIt rejects response box or SDK bridge sample')
                    visited.add(key)
            for lo,hi in ((lower,upper),(bridge_lower,bridge_upper)):
                count=0
                for center,half in joint_cells(lo,hi):
                    self.guard(); count+=1
                    for opening in openings:
                        proof=certify_joint_box_geometry(self.fk,self.gripper,np.r_[center,opening],geometry,self.lo,self.hi,half)
                        proofs.append(proof)
                        T=self.fk(center)
                        local=np.concatenate([base._box_corners(c,np.eye(3),size) for _,c,size in base._stage_boxes(np.eye(4),self.gripper,opening,'y','z')])
                        radius=np.array([np.dot(self.fk.corner_motion_radii(p),half) for p in local])
                        world=local@T[:3,:3].T+T[:3,3]
                        maximum_distance=max(maximum_distance,float(np.max(np.linalg.norm(world-geometry['Y'][:3,3],axis=1)+radius)))
                        cad_world_corners.append(world);cad_motion_radii.append(radius)
                        table_proof=table_clearance_from_cad_enclosure(world,radius,geometry['table_plane'],geometry['motor_top_height_m'])
                        table_clearance=min(table_clearance,table_proof['minimum_adjacent_table_plus_visible_motor_clearance_m'])
                        if table_clearance < .04:
                            raise RuntimeError('continuous CAD clearance above independent table and visible motor is below 40 mm')
                        valid_state(center,opening)
                counts.append(count)
            for first,last in ((measured[:6],target[:6]),(measured[:6],reference[:6]),(reference[:6],target[:6])):
                for fraction in np.linspace(0.,1.,17):
                    for opening in openings: valid_state(first*(1-fraction)+last*fraction,opening)
            result=dict(lower_rad=lower.tolist(),upper_rad=upper.tolist(),directions=directions.tolist(),
                physical_response_cells=counts[0],nominal_sdk_bridge_cells=counts[1],moveit_samples=len(visited),
                continuous_cell_proofs=len(proofs),sdk_bridge_does_not_expand_feedback_rectangle=True,
                nominal_sdk_bridge_lower_rad=bridge_lower.tolist(),nominal_sdk_bridge_upper_rad=bridge_upper.tolist(),
                maximum_cad_distance_to_board_origin_m=maximum_distance,
                minimum_plane_clearance_lower_bound_m=min(p['minimum_plane_clearance_lower_bound_m'] for p in proofs),
                minimum_full_board_edge_lower_bound_px=min(p['minimum_full_board_edge_lower_bound_px'] for p in proofs),
                minimum_board_depth_lower_bound_m=min(p['minimum_board_depth_lower_bound_m'] for p in proofs),
                physical_clearance_requirement_m=.04,diagnostic_full_board_margin_only=True,
                minimum_adjacent_table_plus_visible_motor_clearance_m=table_clearance,
                hardware_tracking_or_stopping_certified=False,whole_arm_continuous_collision_certified=False)
            batch=(np.asarray([p['camera_coordinate_taylor_bound']['corner_lower_xyz_m'] for p in proofs]),
                   np.asarray([p['camera_coordinate_taylor_bound']['corner_upper_xyz_m'] for p in proofs]))
            self.path_sequence+=1; archive=self.out/('continuous_cells_%03d.npz'%self.path_sequence)
            cad_corners=np.concatenate(cad_world_corners);cad_radii=np.concatenate(cad_motion_radii)
            np.savez_compressed(archive,camera_lower=batch[0],camera_upper=batch[1],
                cad_world_corners=cad_corners,cad_motion_radii_m=cad_radii)
            result.update(cell_enclosure_archive=str(archive),cell_enclosure_sha256=hashlib.sha256(archive.read_bytes()).hexdigest())
            self.geometry_batches[str(archive)]=batch
            result['whole_rectangle_scene_proof']=wrist.verify_fresh_scene(result,geometry['Y'],geometry['Y'],geometry['model'],projection_corner_box_batch,batch)
            self.table_path=(cad_corners,cad_radii,str(archive))
            return result

        def run(self):
            reference,stationary=self.await_initial()
            geometry=self.snapshot('baseline')
            with self.lock:
                self.guard(); measured=base.positions(self.accepted[-1])
                if np.max(abs(measured[:6]-geometry['q'][:6])) > math.radians(.15):
                    raise RuntimeError('fresh RGB-D pose differs from settled actual start')
                geometry['q']=measured.copy()
            increments,proposal=solve_three_axes(self.fk,geometry,geometry['motor']['center_board_m'])
            with self.lock:
                self.guard()
                if np.max(abs(base.positions(self.accepted[-1])[:6]-measured[:6])) > math.radians(.15):
                    raise RuntimeError('measured pose changed during one-time solver')
                self.contract=CenteringContract(measured,reference,self.frozen_epoch,increments)
            plan=dict(schema=1,execute=self.args.execute,maximum_commands=len(self.contract.indices),
                initial_measured_rad=measured.tolist(),starting_sdk_rad=reference.tolist(),frozen_epoch_ns=self.frozen_epoch,
                source_stamp_ns=geometry['source_stamp_ns'],motor_evidence_sha256=self.seed_digest,
                diagnostic_motor_association=geometry['motor'],proposal=proposal,stationary=stationary,
                commands=[dict(label=label,joint=j+1,target_rad=t.tolist(),words=words(t)) for label,j,t in zip(self.contract.labels,self.contract.indices,self.contract.targets)],
                production_target_identity_verified=False,clears_grasp_fault=False,bootstrap_pending=False,
                source='fresh current process; historical report supplies board-local neighbourhood only',
                recovery='none',archive_policy='local_only',upload_enabled=False,delete_enabled=False)
            base.write_json(self.out/'plan.json',plan)
            if not self.args.execute:
                if self.contract.indices:
                    self.args.joint=self.contract.indices[0]+1
                    plan['first_step_geometry']=self.path(measured,self.contract.targets[0],geometry,reference)
                    base.write_json(self.out/'plan.json',plan)
                self.event('prepared_no_motion',plan=str(self.out/'plan.json'))
                return
            observed=geometry
            for j,label,target in zip(self.contract.indices,self.contract.labels,self.contract.targets):
                self.args.joint=j+1
                observed=self.execute_step(label,target)
            self.event('finite_observation_complete',commands_published=self.publish_count,
                actual_latest_motor_observation=observed['motor'],
                production_target_identity_verified=False,automatic_retry=False)
    return Diagnostic


def parse_args(argv=None):
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--execute',action='store_true')
    p.add_argument('--motor-evidence',required=True,type=Path)
    args=p.parse_args(argv)
    args.bootstrap_pending=False;args.bootstrap_sync_bound_deg=4.8;args.joint=1
    return args


def main():
    args=parse_args()
    # Fail before ROS initialization/publisher construction on mixed user/system NumPy.
    from scipy.optimize import least_squares as _least_squares_import_check
    import pyrealsense2 as _sdk_import_check
    if not os.path.ismount(MOUNT) or not ARCHIVE.is_dir():
        raise RuntimeError('SanStand unavailable; no system disk fallback')
    if MOUNT not in args.motor_evidence.resolve().parents:
        raise RuntimeError('motor diagnostic evidence must be the archived SanStand report')
    out=ARCHIVE/('bounded_motor_centering_'+datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')+'_'+str(os.getpid()))
    out.mkdir();(out/'ros').mkdir();os.environ['ROS_LOG_DIR']=str(out/'ros')
    manifest={}
    for name in ('bounded_motor_centering.py','bounded_wrist_response_diagnostic.py','bounded_joint_response_diagnostic.py',
                 'bootstrap_mixed_geometry.py','coupled_joint23_geometry.py','audit_motor_adjacent_table.py','analyze_depth_supported_board_motion.py','bounded_async_bag_writer.py'):
        data=(HERE/name).read_bytes();(out/name).write_bytes(data);manifest[name]=hashlib.sha256(data).hexdigest()
    (out/'motor_evidence.json').write_bytes(args.motor_evidence.read_bytes())
    (out/'source_sha256.json').write_text(json.dumps(manifest,indent=2)+'\n')
    base=load_module('motor_centering_base',HERE/'bounded_joint_response_diagnostic.py')
    base.rospy.init_node('bounded_motor_centering',anonymous=True,disable_signals=True)
    terminal=dict(status='failed',execute=args.execute,commands_published=0)
    diagnostic=None
    def alarm(_signum,_frame):raise RuntimeError('180-second finite observation deadline exhausted')
    signal.signal(signal.SIGALRM,alarm);signal.setitimer(signal.ITIMER_REAL,180.)
    try:
        diagnostic=make_diagnostic_class(base)(args,out);diagnostic.run()
        terminal['status']='completed' if args.execute else 'prepared_no_motion'
    except BaseException as error:
        terminal['reason']=str(error)
        if diagnostic is not None:diagnostic.event('no_further_commands',reason=str(error),recovery_attempted=False,stop_or_disable_sent=False)
        raise
    finally:
        signal.setitimer(signal.ITIMER_REAL,0)
        if diagnostic is not None:
            terminal['commands_published']=diagnostic.publish_count
            try:diagnostic.close()
            except BaseException as close_error:
                terminal['status']='failed';terminal['reason']='archive close failed: '+str(close_error)
            terminal['archive_writer']=diagnostic.writer_summary or diagnostic.archive_writer.summary()
            if not terminal['archive_writer']['archive_complete']:
                terminal['status']='failed';terminal['reason']=terminal['archive_writer']['failure'] or 'archive incomplete'
        base.write_json(out/'terminal.json',terminal);print('SESSION '+str(out),flush=True)
    if terminal['status']=='failed':raise RuntimeError(terminal.get('reason','archive failed'))


if __name__=='__main__':main()
