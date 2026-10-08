#!/usr/bin/env python3
"""Source-bound RGB ChArUco observation diagnostic, never grasp authorization.

Pure builder/audits have no ROS I/O. The optional executor factory delegates
read-only capture/model reads to board_source_capture; it creates no publisher.
The board printed face is a declared local table upper bound, not a measured
RGB-D support plane. Original depth support evidence must remain independent.
"""
from collections.abc import Mapping
from copy import deepcopy
import hashlib
import importlib.util
import json
import math
from pathlib import Path
from types import MappingProxyType

import numpy as np
import gen4_orthogonal_probe_plan as orthogonal
from bootstrap_mixed_geometry import (
    board_camera_box_taylor_bound, certify_joint_box_geometry,
    projection_corner_boxes,
)

base = orthogonal.base
high = orthogonal.high
bound_publish_target = high.bound_publish_target
BOARD_CORNERS = np.array([[x, y, 0.] for x in (0., .18) for y in (0., .135)])
MIN_CLEARANCE_M = .04
MIN_EDGE_PX = 20.
MIN_BOARD_DEPTH_M = .12
MAX_BOARD_DEPTH_M = .50
MAX_BOARD_TILT_DEG = 10.
MAX_BOX_CELLS = 1024


def _plain(value):
    if isinstance(value, Mapping):
        return {str(k): _plain(v) for k, v in value.items()}
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, (list, tuple)):
        return [_plain(v) for v in value]
    if isinstance(value, np.generic):
        return value.item()
    return value


def _digest(value):
    return hashlib.sha256(json.dumps(_plain(value), sort_keys=True,
                                    allow_nan=False, separators=(',', ':')).encode()).hexdigest()


def _model_digest(evidence):
    # Exactly the RuntimeColorModel fingerprint serialization, not _digest.
    return hashlib.sha256(json.dumps(_plain(evidence), sort_keys=True,
                                    allow_nan=False).encode()).hexdigest()


def _freeze(value):
    if isinstance(value, Mapping):
        return MappingProxyType({k: _freeze(v) for k, v in value.items()})
    if isinstance(value, (list, tuple)):
        return tuple(_freeze(v) for v in value)
    if isinstance(value, np.ndarray):
        a = np.ascontiguousarray(value)
        return np.frombuffer(a.tobytes(), dtype=a.dtype).reshape(a.shape)
    return value


def _rigid(value, label):
    a = np.array(value, dtype=float, copy=True)
    if (a.shape != (4, 4) or not np.isfinite(a).all()
            or not np.allclose(a[3], [0, 0, 0, 1], atol=1e-9, rtol=0)
            or not np.allclose(a[:3, :3].T @ a[:3, :3], np.eye(3), atol=1e-8, rtol=0)
            or abs(np.linalg.det(a[:3, :3])-1.) > 1e-8):
        raise RuntimeError(label+' must be a finite proper rigid transform')
    return a


def _joint(value, label):
    a = np.array(value, dtype=float, copy=True)
    if (a.shape != (7,) or not np.isfinite(a).all()
            or np.any(abs(a[:6]) > math.pi) or not 0. <= a[6] <= .05):
        raise RuntimeError(label+' must contain seven finite bounded joints')
    return a


def _stamp(value, label):
    if type(value) is not int or value <= 0:
        raise RuntimeError(label+' must be a positive integer nanosecond stamp')
    return value


def _finite(value, label):
    if type(value) not in (int, float) or not math.isfinite(value):
        raise RuntimeError(label+' must be finite numeric evidence')
    return float(value)


def _require_model(model, evidence, fingerprint):
    if not isinstance(evidence, Mapping) or not isinstance(fingerprint, str):
        raise RuntimeError('full RGB camera model evidence/fingerprint required')
    e = _plain(evidence)
    if fingerprint != _model_digest(e):
        raise RuntimeError('RGB model fingerprint/content mismatch')
    if (model.fingerprint != fingerprint or _plain(model.evidence) != e
            or model.backend_sdk_version != e.get('sdk_version')
            or e.get('sdk_version') != '2.56.5'
            or e.get('source') != 'realsense_runtime'
            or e.get('camera_frame') != 'camera_link'
            or e.get('image_topic') != '/supervisor/camera/color/image_raw'
            or e.get('color_projection_correction') is not False
            or not isinstance(e.get('serial_number'), str)
            or e['serial_number'] in ('', 'unavailable')):
        raise RuntimeError('bound SDK/device/full RGB model changed or unsupported')
    c, i = e['color_intrinsics'], model.intrinsics
    if (c.get('model') not in ('none', 'brown_conrady', 'inverse_brown_conrady')
            or any(type(c.get(k)) is not int or c[k] <= 0 for k in ('width', 'height'))
            or len(c.get('coeffs', [])) != 5
            or not np.isfinite([c[k] for k in ('fx', 'fy', 'cx', 'cy')]+list(c['coeffs'])).all()
            or min(c['fx'], c['fy']) <= 0
            or i.width != c['width'] or i.height != c['height']
            or i.model != getattr(model.sdk.distortion, c['model'])
            or not np.array_equal([i.fx, i.fy, i.ppx, i.ppy],
                                  np.asarray([c['fx'], c['fy'], c['cx'], c['cy']], dtype=np.float32))
            or not np.array_equal(i.coeffs, np.asarray(c['coeffs'], dtype=np.float32))):
        raise RuntimeError('projection intrinsics differ from frozen RGB evidence')


def build_board_snapshot(evidence, xml, model):
    """Validate one exact source bundle and return an independent frozen copy.

    Provided provenance still needs verification against the original bag/raw
    files. No Boolean flag substitutes for the timing/identity numeric checks.
    """
    if not isinstance(evidence, Mapping) or not isinstance(xml, str):
        raise RuntimeError('source bundle and XML required')
    s = deepcopy(_plain(evidence))
    ns = _stamp(s['source_stamp_ns'], 'source')
    epoch = _stamp(s['epoch_ns'], 'epoch')
    if not epoch < ns:
        raise RuntimeError('RGB source must be after frozen epoch')
    bind = s['source_binding']
    for name in ('rgb_stamp_ns', 'depth_stamp_ns', 'board_stamp_ns', 'quality_stamp_ns'):
        if type(bind.get(name)) is not int or bind[name] != ns:
            raise RuntimeError('RGB/depth/board/quality must share exact source stamp')
    received = _stamp(bind['snapshot_received_ros_ns'], 'snapshot received')
    if (s.get('accepted_source_stamp_ns', bind['accepted_source_stamp_ns']) != bind['accepted_source_stamp_ns']
            or s.get('snapshot_received_ros_ns', bind['snapshot_received_ros_ns']) != bind['snapshot_received_ros_ns']):
        raise RuntimeError('top-level/source binding time disagreement')
    quality_received = _stamp(bind['quality_received_ros_ns'], 'quality received')
    if any(not 0 <= now-ns <= 350_000_000 for now in (received, quality_received)):
        raise RuntimeError('source/quality reception must be within 350ms and not future')
    before = _stamp(bind['accepted_before_ns'], 'accepted before')
    after = _stamp(bind['accepted_after_ns'], 'accepted after')
    closest = _stamp(bind['accepted_source_stamp_ns'], 'accepted source')
    if (not epoch <= before <= ns <= after <= received or after-before > 200_000_000
            or closest not in (before, after) or abs(closest-ns) > 100_000_000
            or abs(closest-ns) != min(ns-before, after-ns)):
        raise RuntimeError('accepted feedback must bracket source within 200ms with nearest <=100ms')
    for key in ('sdk_stamp_ns', 'reference_stamp_ns'):
        stamp = _stamp(bind[key], key)
        if not epoch <= stamp <= received or received-stamp > 300_000_000:
            raise RuntimeError('SDK/reference source stale/future or before epoch')
    duration = _finite(bind['accepted_window_duration_sec'], 'stationary duration')
    span = np.array(bind['accepted_window_span_deg'], dtype=float)
    if (duration < .7 or span.shape != (6,) or not np.isfinite(span).all()
            or np.any(span < 0.) or np.max(span) > .15+1e-12):
        raise RuntimeError('independent accepted stationary window requires >=0.7s and <=0.15deg')
    if 'board_received_ros_ns' in bind and not 0 <= _stamp(bind['board_received_ros_ns'], 'board received')-ns <= 350_000_000:
        raise RuntimeError('exact board TF reception stale or future')
    q = _joint(s['q_rad'], 'accepted q')
    sdk = _joint(s['sdk_q_rad'], 'SDK q')
    reference = _joint(s['reference_q_rad'], 'reference q')
    if (s.get('sdk_frame_id') != 'sdk_transmitted' or s.get('reference_frame_id') != 'sdk_manual'
            or base.words(sdk) != base.words(reference) or sdk[6] != reference[6]):
        raise RuntimeError('SDK/reference identity, words or gripper mismatch')
    if (s.get('task_active') is not False or s.get('protection_latched') is not False
            or s.get('feedback_ready') is not True or s.get('gui_direct_mode') is not True
            or not isinstance(s.get('actuation_status'), str)
            or not s['actuation_status'].startswith('CONFIRMED:')):
        raise RuntimeError('fresh manual/CONFIRMED/task/protection authority unavailable')
    quality = s['board_quality']
    if (quality.get('source_stamp_ns') != ns or quality.get('pose_ok') is not True
            or type(quality.get('charuco_count')) is not int or quality['charuco_count'] < 35
            or quality.get('camera_frame') != 'camera_link'
            or not isinstance(quality.get('board_frame'), str) or not quality['board_frame']
            or quality.get('board_size') != [12, 9] or quality.get('dictionary_id') != 'DICT_5X5_100'
            or abs(_finite(quality.get('square_length_m'), 'square length')-.015) > 1e-10
            or abs(_finite(quality.get('marker_length_m'), 'marker length')-.01125) > 1e-10
            or not 0 <= _finite(quality.get('reprojection_rms_px'), 'PnP RMS') <= 1.5
            or _finite(quality.get('edge_clearance_px'), 'detected corner margin') < MIN_EDGE_PX
            or quality.get('intrinsics_source') != 'realsense_runtime'
            or quality.get('camera_model_change_latched') is not False or quality.get('camera_model_error')):
        raise RuntimeError('same-source board quality violates unchanged corner/RMS/model gates')
    fingerprint = s['camera_model_fingerprint']
    rgb_evidence = s['camera_model_evidence']
    if (quality.get('camera_model_fingerprint') != fingerprint
            or quality.get('camera_model') != rgb_evidence):
        raise RuntimeError('source quality and RGB model evidence disagree')
    _require_model(model, rgb_evidence, fingerprint)
    ci = rgb_evidence['color_intrinsics']
    if (quality.get('image_width') != ci['width'] or quality.get('image_height') != ci['height']
            or s.get('width') != ci['width'] or s.get('height') != ci['height']
            or not np.array_equal(s['K'], [ci[k] for k in ('fx', 'fy', 'cx', 'cy')])
            or not 0 < _finite(s['depth_scale'], 'depth scale') < .01):
        raise RuntimeError('source image geometry and runtime RGB model disagree')
    sha = hashlib.sha256(xml.encode()).hexdigest()
    if s.get('urdf_sha256') != sha:
        raise RuntimeError('source URDF hash mismatch')
    fk = base.SerialUrdfFk(xml, base.NAMES[:6])
    runtime_lo, runtime_hi = np.array(s['joint_min'], float), np.array(s['joint_max'], float)
    if (runtime_lo.shape != (6,) or runtime_hi.shape != (6,)
            or not np.isfinite([runtime_lo, runtime_hi]).all() or np.any(runtime_lo > runtime_hi)):
        raise RuntimeError('source joints/limits differ from unchanged URDF domain')
    # Runtime software limits may be broader than the mechanical URDF. Their
    # intersection tightens the source domain; neither limit is ever expanded.
    lo, hi = np.maximum(runtime_lo, fk.limits[:, 0]), np.minimum(runtime_hi, fk.limits[:, 1])
    if (np.any(lo > hi) or np.any(q[:6] < lo) or np.any(q[:6] > hi)
            or np.any(sdk[:6] < lo) or np.any(sdk[:6] > hi)):
        raise RuntimeError('source joints/limits differ from unchanged URDF domain')
    s.setdefault('recorded_runtime_joint_min', runtime_lo.tolist())
    s.setdefault('recorded_runtime_joint_max', runtime_hi.tolist())
    s.update(joint_min=lo.tolist(), joint_max=hi.tolist(),
             effective_joint_limits_source='intersection_of_recorded_runtime_and_unchanged_URDF')
    x = _rigid(s['T_tool_camera'], 'tool-camera')
    camera = _rigid(s['T_base_camera'], 'base-camera')
    b = _rigid(s['T_camera_board'], 'camera_link-board')
    y = _rigid(s['T_base_board'], 'base-board')
    if (np.max(abs(fk(q[:6])@x-camera)) > 1e-5
            or np.max(abs(camera@b-y)) > 1e-5):
        raise RuntimeError('exact source camera/board TF and independent accepted FK disagree')
    normal = y[:3, 2].copy()
    sign = 1 if normal[2] > 0 else -1
    normal *= sign
    if normal[2] < math.cos(math.radians(MAX_BOARD_TILT_DEG)):
        raise RuntimeError('board pose inconsistent with fixed flat local table contract')
    scene = s['scene_contract']
    if (scene.get('kind') != 'fixed_rigid_board_laid_flat_on_same_local_tabletop'
            or scene.get('board_printed_plane_is_local_table_upper_bound') is not True
            or scene.get('old_board_thickness_reused') is not False
            or not isinstance(scene.get('evidence'), str) or not scene['evidence'].strip()
            or scene.get('scope') != 'small bounded board-only diagnostic, not grasp authority'):
        raise RuntimeError('independent fixed board/local table upper-bound scene contract required')
    openings = sorted(set(_finite(v, 'gripper opening') for v in s['openings']))
    if (not openings or openings != sorted(set([float(q[6]), float(sdk[6])]))
            or any(not 0 <= v <= .05 for v in openings)):
        raise RuntimeError('actual/SDK gripper opening enclosure mismatch')
    depth_min = _finite(s.get('board_working_depth_min_m', MIN_BOARD_DEPTH_M), 'board depth minimum')
    depth_max = _finite(s.get('board_working_depth_max_m', MAX_BOARD_DEPTH_M), 'board depth maximum')
    if not MIN_BOARD_DEPTH_M <= depth_min < depth_max <= MAX_BOARD_DEPTH_M:
        raise RuntimeError('board working range may only tighten original 120..500mm bounds')
    corners = (np.linalg.inv(camera)@y@np.c_[BOARD_CORNERS, np.ones(4)].T).T[:, :3]
    rectangle = projection_corner_boxes(model, corners, corners)
    edge = min(rectangle[0, 0], ci['width']-1-rectangle[0, 1],
               rectangle[1, 0], ci['height']-1-rectangle[1, 1])
    if edge < MIN_EDGE_PX or corners[:, 0].min() < depth_min or corners[:, 0].max() > depth_max:
        raise RuntimeError('full-board distorted FOV/working-range source check failed')
    # A RGB-derived board plane has deliberately different names from table/depth
    # planes and cannot be passed as independent depth to audit_pair.
    s.update(kind='source_bound_RGB_PnP_board_observation_only',
        board_frame_convention='tracker_ROS_camera_link_board',
        T_base_board=y.tolist(), board_up_sign=sign,
        board_printed_upper_plane_base=np.r_[normal, -normal@y[:3, 3]].tolist(),
        board_plane_source='same_source_RGB_charuco_printed_face',
        board_working_depth_min_m=depth_min, board_working_depth_max_m=depth_max,
        initial_full_board_pixel_rectangle_uv=rectangle.tolist(),
        initial_full_board_edge_lower_bound_px=float(edge),
        grasp_authorized=False, original_grasp_depth_gates_unchanged=True,
        no_independent_depth_plane_claim=True, openings=openings)
    return _freeze(s)


class BoardGeometryAudit:
    """Fixed RGB board, modeled CAD clearance and full distorted projection.

    This proves model geometry conditional on the supplied board/table contract;
    it does not certify hardware tracking, unseen scene, or whole-arm collision.
    """
    def __init__(self, snapshot, xml, validity, model):
        self.snapshot = build_board_snapshot(snapshot, xml, model)
        self.fk = base.SerialUrdfFk(xml, base.NAMES[:6])
        self.X = np.array(self.snapshot['T_tool_camera'])
        self.Y = np.array(self.snapshot['T_base_board'])
        self.plane = np.array(self.snapshot['board_printed_upper_plane_base'])
        self.lo, self.hi = np.array(self.snapshot['joint_min']), np.array(self.snapshot['joint_max'])
        self.openings = self.snapshot['openings']
        self.model, self.validity, self.cache = model, validity, {}
        self.gripper = base.GripperGeometry(.05, .002, np.array([.0434, .0286, .06]),
                                           np.array([.1175, .155, .0774]), .003)
        self._geometry_digest = _digest([self.X, self.Y, self.plane, self.lo, self.hi, self.openings])

    def _integrity(self):
        if self._geometry_digest != _digest([self.X, self.Y, self.plane, self.lo, self.hi, self.openings]):
            raise RuntimeError('frozen board/handeye/limits/openings geometry changed')
        _require_model(self.model, self.snapshot['camera_model_evidence'],
                       self.snapshot['camera_model_fingerprint'])

    def _cell(self, center, half):
        self._integrity()
        proofs = []
        for opening in self.openings:
            proof = certify_joint_box_geometry(self.fk, self.gripper, np.r_[center, opening],
                dict(X=self.X, Y=self.Y, model=self.model), self.lo, self.hi, half)
            t = proof['camera_coordinate_taylor_bound']
            minimum = float(np.min(t['corner_lower_xyz_m'], axis=0)[0])
            maximum = float(np.max(t['corner_upper_xyz_m'], axis=0)[0])
            if (minimum < self.snapshot['board_working_depth_min_m']
                    or maximum > self.snapshot['board_working_depth_max_m']):
                raise RuntimeError('full-board continuous working depth range failed')
            proof.update(maximum_board_depth_upper_bound_m=maximum)
            proofs.append(proof)
        return proofs

    def geometry(self, q, check_collision=True):
        self._integrity()
        q = np.array(q, dtype=float)[:6]
        if q.shape != (6,) or not np.isfinite(q).all():
            raise RuntimeError('finite six-axis actual/SDK geometry required')
        proofs = self._cell(q, np.zeros(6))
        valid = None
        if check_collision and self.validity is not None:
            valid = True
            for opening in self.openings:
                key = tuple(q)+(opening,)
                if key not in self.cache:
                    self.cache[key] = bool(self.validity(q.copy(), opening))
                valid = valid and self.cache[key]
        edge = min(p['minimum_full_board_edge_lower_bound_px'] for p in proofs)
        board = dict(edge_px=edge,
            forward_min_m=min(p['minimum_board_depth_lower_bound_m'] for p in proofs),
            forward_max_m=max(p['maximum_board_depth_upper_bound_m'] for p in proofs),
            region='complete_180x135mm_printed_board', distorted_model_fingerprint=self.model.fingerprint)
        return dict(q_rad=q.tolist(), clearance_m=min(p['minimum_plane_clearance_lower_bound_m'] for p in proofs),
            target=board, texture=dict(board), joint_limits_pass=True, moveit_collision_valid=valid,
            geometry_pass=valid is not False, T_base_camera=(self.fk(q)@self.X).tolist(),
            plane_source='frozen_source_RGB_printed_board_upper_bound', grasp_authorized=False)

    @staticmethod
    def summarize(rows):
        result = base.GeometryAudit.summarize(rows)
        result.update(plane_source='frozen_source_RGB_printed_board_upper_bound',
                      grasp_authorized=False, independent_depth_plane=False)
        return result

    def guard_box(self, lower, upper, check_collision=True):
        self._integrity()
        lower, upper = np.array(lower, float), np.array(upper, float)
        if (lower.shape != (6,) or upper.shape != (6,) or not np.isfinite([lower, upper]).all()
                or np.any(lower > upper) or np.any(lower < self.lo) or np.any(upper > self.hi)):
            raise RuntimeError('continuous response box exceeds unchanged joint limits')
        todo, cells, summaries = [(lower, upper)], 0, []
        while todo:
            a, b = todo.pop()
            half, center = .5*(b-a), .5*(a+b)
            if half.sum() > math.radians(2.):
                if cells+len(todo) >= MAX_BOX_CELLS:
                    raise RuntimeError('bounded board continuous response proof budget exhausted')
                axis = int(np.argmax(half)); middle = center[axis]
                left, right = b.copy(), a.copy(); left[axis] = middle; right[axis] = middle
                todo.extend([(a, left), (right, b)])
                continue
            cells += 1
            if cells > MAX_BOX_CELLS:
                raise RuntimeError('bounded board continuous response proof budget exhausted')
            proofs = self._cell(center, half)
            row = self.geometry(center, check_collision)
            row['clearance_m'] = min(p['minimum_plane_clearance_lower_bound_m'] for p in proofs)
            for name in ('target', 'texture'):
                row[name]['edge_px'] = min(p['minimum_full_board_edge_lower_bound_px'] for p in proofs)
                row[name]['forward_min_m'] = min(p['minimum_board_depth_lower_bound_m'] for p in proofs)
                row[name]['forward_max_m'] = max(p['maximum_board_depth_upper_bound_m'] for p in proofs)
            summaries.append(row)
        result = self.summarize(summaries)
        result.update(continuous_model_box_certified=True, continuous_cells=cells,
            lower_rad=lower.tolist(), upper_rad=upper.tolist(),
            whole_arm_continuous_collision_certified=False, hardware_tracking_or_stopping_certified=False)
        return result

    def guard_segment(self, q0, q1, check_collision=True, step_deg=.15):
        a, b = np.array(q0, float)[:6], np.array(q1, float)[:6]
        proof = self.guard_box(np.minimum(a, b), np.maximum(a, b), check_collision)
        if not math.isfinite(step_deg) or not 0 < step_deg <= .15:
            raise RuntimeError('MoveIt path sample spacing cannot exceed original0.15deg')
        count = max(2, int(math.ceil(np.max(abs(np.degrees(b-a)))/step_deg))+1)
        rows = [self.geometry(a*(1-u)+b*u, check_collision) for u in np.linspace(0., 1., count)]
        result = self.summarize(rows)
        result.update(continuous_box_proof=proof)
        result['minimum_clearance_m'] = min(result['minimum_clearance_m'], proof['minimum_clearance_m'])
        for field in ('minimum_target_margin_px', 'minimum_texture_margin_px', 'minimum_target_forward_m'):
            result[field] = min(result[field], proof[field])
        result['maximum_target_forward_m'] = max(result['maximum_target_forward_m'], proof['maximum_target_forward_m'])
        result['all_geometry_pass'] &= proof['all_geometry_pass']
        result['all_moveit_valid'] &= proof['all_moveit_valid']
        return result


class BoardOrthogonalProbeAudit(orthogonal.OrthogonalProbeAudit):
    """Original sixteen SDK steps, with frozen board-only diagnostic geometry."""
    def __init__(self, snapshot, xml, validity, model, current_model_read=None):
        frozen = build_board_snapshot(snapshot, xml, model)
        self.current_model_read = current_model_read
        super().__init__(frozen, xml, validity,
            audit_factory=lambda s, x, v: BoardGeometryAudit(s, x, v, model))
        self.source_q = _freeze(self.source_q)
        self.reference = _freeze(self.reference)
        self.absolute_lo = _freeze(self.absolute_lo)
        self.absolute_hi = _freeze(self.absolute_hi)
        self._frozen_source_digest = _digest(self.snapshot)
        self._frozen_sequence_digest = _digest(self.sequence)
        self._frozen_control_digest = _digest([self.source_q, self.reference, self.absolute_lo, self.absolute_hi])
        self.contract.update(kind='source_bound_RGB_board_J2_J5_signed_repeat_diagnostic',
            board_plane_source='RGB_printed_face_local_table_upper_bound',
            camera_model_fingerprint=frozen['camera_model_fingerprint'],
            frozen_T_base_board=_plain(frozen['T_base_board']),
            scene_contract=_plain(frozen['scene_contract']),
            grasp_authorized=False, original_grasp_depth_gates_unchanged=True,
            independent_depth_plane_claim=False, automatic_return_after_failure=False)

    def _integrity(self):
        if (_digest(self.snapshot) != self._frozen_source_digest
                or _digest(self.sequence) != self._frozen_sequence_digest
                or _digest([self.source_q, self.reference, self.absolute_lo, self.absolute_hi]) != self._frozen_control_digest):
            raise RuntimeError('frozen source/SDK sequence changed')
        self.audit._integrity()

    def check_model(self):
        self._integrity()
        if self.current_model_read is None:
            raise RuntimeError('fresh current model/epoch reader required before submission')
        current = self.current_model_read()
        if (not isinstance(current, Mapping) or current.get('urdf_sha256') != self.snapshot['urdf_sha256']
                or current.get('epoch_ns') != self.snapshot['epoch_ns']
                or current.get('depth_scale') != self.snapshot['depth_scale']
                or current.get('camera_model_fingerprint') != self.snapshot['camera_model_fingerprint']
                or _plain(current.get('camera_model_evidence')) != _plain(self.snapshot['camera_model_evidence'])
                or not np.allclose(_rigid(current['T_tool_camera'], 'current handeye'),
                                   self.snapshot['T_tool_camera'], rtol=0, atol=1e-9)):
            raise RuntimeError('current URDF/handeye/full RGB model/driver epoch changed')
        _require_model(self.audit.model, current['camera_model_evidence'], current['camera_model_fingerprint'])

    def _evaluate(self, actual, target, axis, step_start, check_model):
        self._integrity()
        result = super()._evaluate(actual, target, axis, step_start, check_model)
        # Preserve original controller bounds. Certify the permitted physical
        # held-axis drift box separately from the nominal actual->SDK bridge.
        a = self.check_actual(actual)
        b = high.vector(target)[:6]
        start = high.vector(step_start)[:6]
        low, upper = np.radians(result['fixed_step_response_bounds_deg'])
        lower = np.maximum(start-math.radians(high.DRIFT_DEG), self.absolute_lo)
        higher = np.minimum(start+math.radians(high.DRIFT_DEG), self.absolute_hi)
        unedited = [i for i in range(6) if i not in high.EDITABLE]
        lower[unedited] = np.maximum(lower[unedited], self.source_q[unedited]-math.radians(high.DRIFT_DEG))
        higher[unedited] = np.minimum(higher[unedited], self.source_q[unedited]+math.radians(high.DRIFT_DEG))
        lower[axis], higher[axis] = low, upper
        nominal_lo, nominal_hi = np.minimum(a, b), np.maximum(a, b)
        nominal_lo[axis], nominal_hi[axis] = low, upper
        physical = self.audit.guard_box(lower, higher)
        nominal = self.audit.guard_box(nominal_lo, nominal_hi)
        if not all(p['all_geometry_pass'] and p['all_moveit_valid'] for p in (physical, nominal)):
            raise RuntimeError('board full response box/CAD/current MoveIt failed')
        result.update(continuous_physical_response=physical, independent_nominal_sdk_bridge=nominal,
                      board_only_diagnostic=True, grasp_authorized=False)
        # A service/geometry audit can take time. Recheck the model after all
        # replies so an in-audit model change never reaches the publisher.
        if check_model:
            self.check_model()
        return result

    def begin_step(self, index, actual, current_sdk):
        self._integrity()
        self.check_model()
        return super().begin_step(index, actual, current_sdk)

    def check_during_step(self, actual):
        self._integrity()
        return super().check_during_step(actual)

    def complete_step(self, actual, current_sdk):
        self._integrity()
        self.check_model()
        return super().complete_step(actual, current_sdk)


def make_runtime_model(evidence, sdk=None):
    """Construct only the tracker SDK projection/ray math, never a device."""
    path = base.ROOT/'src/real-arm/alicia_d_calibration/scripts/charuco_tracker.py'
    spec = importlib.util.spec_from_file_location('gen4_board_runtime_model_math', str(path))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.RuntimeColorModel(_plain(evidence), sdk=sdk)


def prepare_executor_audit(directory, cycles=7, j2_probe_sign=1):
    """Read-only optional adapter; capture helper owns ROS evidence acquisition."""
    if cycles != 7 or j2_probe_sign != 1:
        raise RuntimeError('board factory retains original fixed orthogonal protocol')
    import board_source_capture as source
    import rospy
    from sensor_msgs.msg import JointState
    from moveit_msgs.msg import RobotState
    from moveit_msgs.srv import GetStateValidity, GetStateValidityRequest
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=True)
    snapshot, xml = source.capture(directory)
    model = make_runtime_model(snapshot['camera_model_evidence'])
    rospy.wait_for_service('/check_state_validity', timeout=3)
    proxy = rospy.ServiceProxy('/check_state_validity', GetStateValidity)
    def valid(q, opening):
        request = GetStateValidityRequest(group_name='alicia')
        request.robot_state = RobotState()
        request.robot_state.joint_state = JointState(name=base.NAMES, position=list(q)+[opening])
        return proxy(request).valid
    obj = BoardOrthogonalProbeAudit(snapshot, xml, valid, model, source.current_model_evidence)
    proof = obj.audit_nominal_sequence()
    base.write(directory/'factory_audit_proof.json', proof)
    base.write(directory/'contract.json', obj.contract)
    (directory/'factory_source.py').write_bytes(Path(__file__).read_bytes())
    base.write(directory/'dependencies.json', dict(
        orthogonal_plan_sha256=hashlib.sha256(Path(orthogonal.__file__).read_bytes()).hexdigest(),
        high_plan_sha256=hashlib.sha256(Path(high.__file__).read_bytes()).hexdigest(),
        base_plan_sha256=hashlib.sha256(Path(base.__file__).read_bytes()).hexdigest(),
        board_source_sha256=hashlib.sha256(Path(source.__file__).read_bytes()).hexdigest()))
    return obj
