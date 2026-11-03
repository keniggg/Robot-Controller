"""Bind a current single-source silhouette diagnosis to its planning snapshot.

A fused SnapshotResult is only request context. Its median depth and voted mask
are never passed as same-source measurements. A missing exact original sample,
missing source model, or non-reproduced production mask fails closed. This
adapter returns diagnostic summaries, never a mask, geometry, or motion permit.
"""
from collections.abc import Mapping
from copy import deepcopy
from dataclasses import dataclass, field
import hashlib

import numpy as np

from .measured_mask_seeds import confidence_labels
from .rgb_instance import refine_candidate
from .selection import parse_selection
from .silhouette_ambiguity import evaluate_silhouette_ambiguity
from .tabletop import Config, _project, segment
from alicia_flexible_grasp.vision.rgbd_rectification import (
    rectification_rgb_valid_mask, require_rectified_source_after_activation)


@dataclass(frozen=True)
class SilhouetteSnapshotConfig:
    depth_scale: float
    perception: Config = field(default_factory=Config)
    perception_status: object = None
    camera_geometry: object = None


def _reject(code, **details):
    return dict(ok=False, diagnostic_only=True, execution_authority=False,
                observation_authority=False, code=code, **details)


def _ns(message):
    return int(message.header.stamp.to_nsec())


def _array_evidence(value):
    array = np.asarray(value)
    return dict(sha256=hashlib.sha256(array.tobytes()).hexdigest(),
                shape=list(array.shape), dtype=str(array.dtype))


def _configuration(config, intrinsics):
    if isinstance(config, SilhouetteSnapshotConfig):
        return config
    if isinstance(config, Mapping):
        raw = config.get('perception', {})
        perception = raw if isinstance(raw, Config) else Config(**raw)
        scale = config.get('depth_scale', getattr(intrinsics, 'depth_scale', None))
        return SilhouetteSnapshotConfig(float(scale), perception, config.get('perception_status'),
                                        config.get('camera_geometry'))
    if isinstance(config, Config):
        return SilhouetteSnapshotConfig(float(intrinsics.depth_scale), config)
    raise ValueError('silhouette snapshot config required')


def build_silhouette_snapshot_evidence(
        sample, snapshot, geometry, transform_base_optical, intrinsics,
        selection, ticket, config, depth_uncertainty_m):
    """Diagnose only an exact frozen last RgbdSample of this request.

    Capture it with SynchronizedRgbdBuffer.wait_for_exact_sample while fresh;
    never fetch a later frame to replace it. ``perception_status``, when given,
    must be the existing unknown-perception ready status for this same source.
    Otherwise one independent fit on that raw frame may be attempted. Both
    paths must reproduce the original published mask exactly. Fused geometry
    supplies target association context only, never the diagnostic height plane.
    Freshness and actual TF lookup remain caller-owned admission conditions.
    """
    try:
        selected = parse_selection(selection)
        if selected['mode'] != 'unknown':
            return _reject('SILHOUETTE_UNKNOWN_SELECTION_REQUIRED')
        ns = getattr(sample, 'stamp_ns', None)
        stamps = tuple(getattr(snapshot, 'sample_stamp_ns', ()))
        payload = getattr(ticket, 'payload', ())
        if (type(ns) is not int or ns <= 0 or not bool(getattr(snapshot, 'ok', False))
                or ns != snapshot.stamp_ns or not stamps or stamps[-1] != ns
                or len(set(stamps)) != len(stamps) or any(s <= selected['stamp_ns'] for s in stamps)
                or list(stamps) != sorted(stamps)
                or not isinstance(payload, (tuple, list)) or len(payload) != 2
                or payload[0] is not snapshot
                or type(ticket.request_id) is not int or ticket.request_id < 1
                or type(ticket.generation) is not int or ticket.generation < 0
                or float(ticket.snapshot_stamp_sec) != float(snapshot.stamp_sec)
                or sample.target_identity is None or sample.target_identity != snapshot.target_identity
                or sample.target_epoch != snapshot.target_epoch
                or sample.target_epoch != ticket.target_epoch
                or sample.target_identity.epoch != sample.target_epoch
                or sample.frame_id != snapshot.frame_id or sample.frame_id != 'camera_link'
                or _ns(sample.object_msg) != ns or not bool(sample.object_msg.detected)
                or not np.array_equal(sample.color_bgr, snapshot.color_bgr)):
            return _reject('SILHOUETTE_SOURCE_REQUEST_BINDING_MISMATCH')
        if not bool(getattr(geometry, 'ok', False)):
            return _reject('SILHOUETTE_REQUEST_GEOMETRY_UNAVAILABLE')
        cfg = _configuration(config, intrinsics)
        if cfg.camera_geometry is not None:
            require_rectified_source_after_activation(cfg.camera_geometry, ns)
        if (not isinstance(cfg.perception, Config) or not np.isfinite(cfg.depth_scale)
                or cfg.depth_scale <= 0 or not np.isfinite(depth_uncertainty_m)
                or depth_uncertainty_m < 0):
            return _reject('SILHOUETTE_CONFIG_INVALID')
        K = np.asarray([getattr(intrinsics, k) for k in ('fx', 'fy', 'cx', 'cy')], dtype=float)
        T_optical = np.asarray(transform_base_optical, dtype=float)
        if (not np.all(np.isfinite(K)) or np.any(K[:2] <= 0) or T_optical.shape != (4, 4)
                or not np.all(np.isfinite(T_optical))
                or not np.allclose(T_optical[3], [0, 0, 0, 1], atol=1e-9, rtol=0)
                or not np.allclose(T_optical[:3, :3].T@T_optical[:3, :3], np.eye(3), atol=1e-8, rtol=0)
                or not np.isclose(np.linalg.det(T_optical[:3, :3]), 1., atol=1e-8)):
            return _reject('SILHOUETTE_PROJECTION_INVALID')
        # Tabletop uses ROS camera_link [z,-x,-y]; the request supplies the
        # exact source's base<-optical transform, as in normal geometry setup.
        C = np.eye(4); C[:3, :3] = [[0, 0, 1], [-1, 0, 0], [0, -1, 0]]
        T = T_optical@C.T
        color = np.asarray(sample.color_bgr); raw = np.asarray(sample.depth_raw)
        published = np.asarray(sample.object_mask)
        if (raw.ndim != 2 or raw.dtype != np.uint16 or published.shape != raw.shape
                or published.dtype != np.uint8 or color.shape != raw.shape+(3,)
                or color.dtype != np.uint8 or not np.any(published)):
            return _reject('SILHOUETTE_ORIGINAL_SAMPLE_INVALID')
        rgb_valid = (None if cfg.camera_geometry is None else
                     rectification_rgb_valid_mask(cfg.camera_geometry, raw.shape))
        if cfg.camera_geometry is not None:
            camera = cfg.camera_geometry
            if (any(float(camera[key]) != float(getattr(intrinsics, key)) for key in ('fx', 'fy', 'cx', 'cy'))
                    or float(camera['depth_scale']) != cfg.depth_scale
                    or raw.shape != (camera['height'], camera['width'])
                    or camera['frame_id'] != sample.frame_id):
                return _reject('SILHOUETTE_SOURCE_CAMERA_GEOMETRY_MISMATCH')
        if rgb_valid is not None and np.any((published > 0) & ~rgb_valid):
            return _reject('SILHOUETTE_PUBLISHED_MASK_HAS_UNOBSERVED_RGB')
        depth = raw.astype(np.float32)*cfg.depth_scale
        position = sample.object_msg.pose_base.pose.position
        current_center = np.array([position.x, position.y, position.z], dtype=float)
        if not np.all(np.isfinite(current_center)):
            return _reject('SILHOUETTE_TARGET_CENTER_INVALID')
        status = cfg.perception_status
        if status is not None:
            if (not isinstance(status, Mapping) or status.get('state') != 'ready'
                    or status.get('mode') != 'unknown' or status.get('target_lost') is not False
                    or status.get('generation') != selected['generation']
                    or status.get('selection_stamp_ns') != selected['stamp_ns']
                    or status.get('exact_rgbd') is not True
                    or any(status.get(k) != ns for k in ('source_stamp_ns', 'color_stamp_ns', 'depth_stamp_ns', 'tf_lookup_stamp_ns'))
                    or any(status.get(k) != sample.frame_id for k in ('color_frame_id', 'depth_frame_id'))
                    or status.get('projection_frame_convention') != 'ros_camera_link'):
                return _reject('SILHOUETTE_PERCEPTION_STATUS_BINDING_MISMATCH')
            anchor = np.asarray(status['anchor_position_base'], dtype=float)
            plane = np.asarray(status['geometry_metrics']['support_plane_base'], dtype=float)
            if (anchor.shape != (3,) or plane.shape != (4,) or not np.all(np.isfinite(anchor))
                    or not np.all(np.isfinite(plane)) or abs(np.linalg.norm(plane[:3])-1.) > 1e-6):
                return _reject('SILHOUETTE_SOURCE_MODEL_INVALID')
            reconstructed = segment(depth, K, T, cfg.perception,
                target_position_base=anchor, plane_base=plane, fit_current_support=False)
            plane_provenance = dict(kind='same_source_perception_verified_model',
                verification_stamp_ns=ns, anchor_source_stamp_ns=status.get('anchor_source_stamp_ns'),
                producer_support_source=status['geometry_metrics'].get('support_source'))
        else:
            reconstructed = segment(depth, K, T, cfg.perception, target_position_base=current_center)
            plane_provenance = dict(kind='independent_single_raw_frame_fit', source_stamp_ns=ns)
        matching = [candidate for candidate in reconstructed.candidates
            if np.linalg.norm(candidate.position_base-current_center) <= cfg.perception.association_distance_m
            and np.count_nonzero((candidate.mask > 0) & (published > 0)) >= cfg.perception.min_object_points]
        if len(matching) != 1:
            return _reject('SILHOUETTE_DEPTH_SEED_NOT_UNIQUE', matching_candidates=len(matching))
        candidate = matching[0]
        reproduced = refine_candidate(color, depth, K, T, candidate, cfg.perception,
            **({} if rgb_valid is None else {'rgb_valid': rgb_valid}))
        if not np.array_equal(reproduced.mask, published):
            return _reject('SILHOUETTE_BASELINE_MASK_NOT_REPRODUCED',
                differing_pixels=int(np.count_nonzero((reproduced.mask > 0) ^ (published > 0))),
                plane_provenance=plane_provenance, needs_exact_perception_source_model=True)
        points = _project(depth, K)
        plane = T.T@np.asarray(candidate.metrics['support_plane_base'])
        heights = np.einsum('ijk,k->ij', points, plane[:3])+plane[3]
        seed = candidate.mask > 0; ys, xs = np.nonzero(seed)
        margin = max(8, int(round(16*min(raw.shape[1]/640., raw.shape[0]/480.))))
        bx0, bx1 = max(0, int(xs.min())-margin), min(raw.shape[1], int(xs.max())+margin+1)
        by0, by1 = max(0, int(ys.min())-margin), min(raw.shape[0], int(ys.max())+margin+1)
        first = np.s_[by0:by1, bx0:bx1]
        first_core, first_support, labels_evidence = confidence_labels(
            seed[first], points[first], depth[first], heights[first], plane[:3], cfg.perception,
            bgr=color[first])
        x0, y0, x1, y1 = reproduced.metrics['rgb_search_box_xyxy']; crop = np.s_[y0:y1, x0:x1]
        core = np.zeros((y1-y0, x1-x0), bool)
        core[by0-y0:by1-y0, bx0-x0:bx1-x0] = first_core
        if (x0, y0, x1, y1) == (bx0, by0, bx1, by1):
            background = first_support.copy()
        else:
            valid = np.isfinite(depth[crop]) & (depth[crop] >= cfg.perception.depth_min_m) & (depth[crop] <= cfg.perception.depth_max_m)
            background = (valid & (np.abs(heights[crop]) <= cfg.perception.support_threshold_m)
                          & labels_evidence['support_background_height_separated'])
        background[:3] = background[-3:] = True; background[:, :3] = background[:, -3:] = True
        diagnostic = evaluate_silhouette_ambiguity(color[crop], depth[crop], seed[crop], core,
            background, heights[crop], source_stamps_ns={k: ns for k in (
                'rgb', 'depth', 'seed', 'hard_foreground', 'hard_background', 'height')},
            upper_seed_height_m=labels_evidence['upper_seed_height_m'],
            support_height_separated=labels_evidence['support_background_height_separated'],
            depth_min_m=cfg.perception.depth_min_m, depth_max_m=cfg.perception.depth_max_m,
            support_band_m=cfg.perception.support_threshold_m)
        expected_hash = hashlib.sha256((published[crop] > 0).astype(np.uint8).tobytes()).hexdigest()
        if diagnostic['original']['sha256'] != expected_hash:
            return _reject('SILHOUETTE_DIAGNOSTIC_BASELINE_NOT_REPRODUCED')
        moments = diagnostic['disagreement']['valid_depth_ray_moments']
        anchor_base = None
        if moments['count']:
            z = moments['mean_z_m']
            point_optical = np.array([
                (moments['mean_u_z_m_px']+(x0-K[2])*z)/K[0],
                (moments['mean_v_z_m_px']+(y0-K[3])*z)/K[1], z])
            anchor_base = (T_optical[:3, :3]@point_optical+T_optical[:3, 3]).tolist()
        # Preserve the request's original fused collision geometry separately.
        # Only the disputed anchor comes from the current raw depth rays. The
        # original support plane here never supplies diagnostic seed heights.
        view_geometry = dict(
            target_center_base_m=np.asarray(geometry.center_base, dtype=float).tolist(),
            target_height_m=float(geometry.size_xyz_m[2]),
            support_normal_base=np.asarray(geometry.support_normal_base, dtype=float).tolist(),
            support_offset_m=float(geometry.support_offset_m),
            ambiguity_anchor_base_m=anchor_base,
            initial_camera_origin_base_m=T[:3, 3].tolist(),
            initial_camera_rotation_base=T[:3, :3].tolist(),
            depth_uncertainty_m=float(depth_uncertainty_m), source_frame_id=sample.frame_id,
            target_geometry_source='original_fused_request_collision_geometry',
            target_geometry_source_stamps_ns=list(stamps),
            ambiguity_anchor_source='mean_of_current_original_valid_depth_disagreement_rays',
            ambiguity_anchor_source_stamp_ns=ns, camera_rotation_convention='ros_camera_link')
        return dict(ok=True, diagnostic_only=True, execution_authority=False,
            observation_authority=False, code='SILHOUETTE_SOURCE_DIAGNOSTIC_READY',
            request_id=ticket.request_id, generation=ticket.generation,
            target_epoch=sample.target_epoch, target_track_id=sample.target_identity.track_id,
            selection=selected, source_stamp_ns=ns, snapshot_stamp_ns=snapshot.stamp_ns,
            snapshot_source_stamps_ns=list(stamps),
            source_kind='original_single_frame_of_fused_snapshot', frame_id=sample.frame_id,
            crop_xyxy=[x0, y0, x1, y1], depth_uncertainty_m=float(depth_uncertainty_m),
            plane_provenance=plane_provenance,
            perception_source_status=deepcopy(status),
            geometry_used_only_as_request_context=True, baseline_mask_reproduced=True,
            view_geometry=view_geometry,
            raw_source_evidence={name: _array_evidence(value) for name, value in (
                ('rgb', color), ('depth', raw), ('published_mask', published))},
            projection_evidence=dict(intrinsics=K.tolist(), depth_scale=cfg.depth_scale,
                                     transform_sha256=_array_evidence(T_optical)['sha256'],
                                     camera_geometry=deepcopy(cfg.camera_geometry)),
            measurement=diagnostic)
    except (AttributeError, KeyError, TypeError, ValueError, OverflowError) as exc:
        return _reject('SILHOUETTE_DIAGNOSTIC_INPUT_REJECTED', reason=str(exc))
