"""Choose between measured registration explanations, never a grasp plan.

The original ICP and measured-only fallback remain the default. A qualified
same-source raw box can disambiguate ICP from its original identity seed or a
measured support-plane correction, but cannot erase history or replace final
fused model/physical qualification.
"""
import numpy as np

from .fused_scene_geometry import _array_digest
from .multiview_surface import (DEFAULT_FUSION_VOXEL_SIZE_M, append_registered_view,
    evaluate_registration_transform)
from .object_geometry import GeometryEstimate
from .regular_box_inference import (RegularBoxInference, evaluate_box_measured_consistency,
    regular_box_geometry_sha256, validate_box_model_evidence)


def _registration_audit(result):
    return dict(ok=bool(result.ok), code=str(result.code),
        transform_base=np.asarray(result.transform_base).tolist(),
        inlier_count=int(result.inlier_count), overlap_fraction=float(result.overlap_fraction),
        rmse_m=float(result.rmse_m),
        maximum_point_displacement_m=float(result.maximum_point_displacement_m),
        support_normal_angle_deg=float(result.support_normal_angle_deg),
        support_plane_separation_m=float(result.support_plane_separation_m))


def _union_consistency(surface, geometry, model, transform):
    # Use only accepted PRE-append history, not a future voxel-filtered surface.
    inverse = np.linalg.inv(transform)
    history = surface.points_base @ inverse[:3, :3].T + inverse[:3, 3]
    combined = np.concatenate((geometry.object_points_base, history), axis=0)
    _, first = np.unique(combined, axis=0, return_index=True)
    union = combined[np.sort(first)]
    checked = evaluate_box_measured_consistency(model, union)
    return dict(ok=checked.ok, code=checked.code,
        maximum_observed_model_excess_m=checked.maximum_observed_model_excess_m,
        conflict=checked.conflict, raw_point_count=len(geometry.object_points_base),
        before_history_point_count=len(history), exact_union_point_count=len(union),
        exact_duplicates_removed=len(combined)-len(union),
        exact_union_sha256=_array_digest(union), all_before_history_points_checked=True)


def _bind_raw_model(model, geometry, moving):
    if not isinstance(model, RegularBoxInference):
        raise ValueError('qualified same-source raw model unavailable')
    if not isinstance(geometry, GeometryEstimate) or geometry.ok is not True:
        raise ValueError('valid raw geometry unavailable')
    evidence = validate_box_model_evidence(model.to_evidence_dict())
    source = evidence['source']
    if (source['target_identity'] != dict(epoch=moving.identity.epoch,
            track_id=moving.identity.track_id)
            or source['stamp_ns'] != moving.stamp_ns
            or source['geometry_sha256'] != regular_box_geometry_sha256(geometry)
            or not np.array_equal(np.asarray(evidence['registration_transform']), np.eye(4))
            or not np.array_equal(moving.points_base, geometry.object_points_base)
            or not np.array_equal(moving.support_normal_base, geometry.support_normal_base)
            or float(moving.support_offset_m) != float(geometry.support_offset_m)
            or not np.array_equal(np.asarray(evidence['source_support_normal_base']),
                                  geometry.support_normal_base)
            or float(evidence['source_support_offset_m']) != float(geometry.support_offset_m)):
        raise ValueError('raw model target/source/geometry/coordinate binding differs')
    return evidence


def _minimal_support_plane_transform(reference, moving):
    """Align measured planes without inventing in-plane translation or yaw.

    The shortest rotation maps the moving normal onto the reference normal.
    Translation is parallel to the reference normal and reconciles the two
    plane offsets. Normalize normal and offset together. This construction
    uses no box, point correspondences, opening or candidate/path scores.
    It remains only a hypothesis subject to all original registration gates.
    """
    reference_normal = np.asarray(reference.support_normal_base, dtype=float)
    moving_normal = np.asarray(moving.support_normal_base, dtype=float)
    if (reference_normal.shape != (3,) or moving_normal.shape != (3,)
            or not np.all(np.isfinite(reference_normal))
            or not np.all(np.isfinite(moving_normal))
            or not np.isfinite(reference.support_offset_m)
            or not np.isfinite(moving.support_offset_m)):
        raise ValueError('finite measured support planes required')
    reference_scale = float(np.linalg.norm(reference_normal))
    moving_scale = float(np.linalg.norm(moving_normal))
    if (not np.isfinite(reference_scale) or not np.isfinite(moving_scale)
            or reference_scale <= 0. or moving_scale <= 0.):
        raise ValueError('nonzero measured support normals required')
    nr = reference_normal / reference_scale
    nm = moving_normal / moving_scale
    dot = float(np.clip(np.dot(nm, nr), -1., 1.))
    if dot <= 0.:
        raise ValueError('support normals do not define the local minimal correction')
    cross = np.cross(nm, nr)
    skew = np.array([[0., -cross[2], cross[1]],
                     [cross[2], 0., -cross[0]],
                     [-cross[1], cross[0], 0.]])
    transform = np.eye(4)
    transform[:3, :3] = np.eye(3) + skew + (skew @ skew) / (1. + dot)
    transform[:3, 3] = nr * (moving.support_offset_m / moving_scale
                             - reference.support_offset_m / reference_scale)
    return transform


def _append_support_plane_explanation(surface, reference, moving, raw_geometry,
        raw_box_model, config, voxel_size_m, original, original_surface, audit):
    """Try a third measured explanation only after ICP and identity conflict."""
    transform = _minimal_support_plane_transform(reference, moving)
    registered = evaluate_registration_transform(reference, moving, transform, config)
    hypothesis = dict(registration=_registration_audit(registered), consistency=None,
        transform_source='measured_support_normals_and_offsets_only',
        rotation_policy='minimal_normal_alignment',
        translation_policy='reference_normal_only')
    audit['hypotheses']['support_plane_minimal'] = hypothesis
    if not registered.ok:
        audit['selection_reason'] = 'SUPPORT_PLANE_REGISTRATION_REJECTED_KEEP_MEASURED'
        return original, original_surface, audit
    consistent = _union_consistency(surface, raw_geometry, raw_box_model, transform)
    hypothesis['consistency'] = consistent
    if not consistent['ok']:
        audit['selection_reason'] = 'NO_BOX_CONSISTENT_HYPOTHESIS_KEEP_MEASURED'
        return original, original_surface, audit
    selected, fused = append_registered_view(surface, reference, moving,
        config, voxel_size_m, transform_base=transform)
    if not selected.ok:
        audit['selection_reason'] = 'SUPPORT_PLANE_APPEND_REJECTED_KEEP_MEASURED'
        return original, original_surface, audit
    audit.update(selected_hypothesis='support_plane_minimal', selected_box_explanation=True,
        selected_transform_base=np.asarray(selected.transform_base).tolist(),
        selection_reason='SUPPORT_PLANE_ORIGINAL_GATES_AND_COMPLETE_UNION_CONSISTENT')
    return selected, fused, audit


def append_box_consistent_registered_view(surface, reference, moving,
        raw_geometry, raw_box_model, config=None, voxel_size_m=DEFAULT_FUSION_VOXEL_SIZE_M):
    """Return ``(registration, fused_surface, diagnostic_audit)``.

    Call only after independently qualifying the raw model against the exact
    snapshot pixels/calibration. This pure boundary additionally binds its
    target, source, raw geometry digest and unregistered coordinates. No model
    or registration audit here grants execution or final fused-model authority.

    Baseline append is kept verbatim on original registration failure, missing
    model/source binding, or when no explanation fits the complete box.
    Its returned future surface is never an input to hypothesis selection.
    """
    original, original_surface = append_registered_view(
        surface, reference, moving, config, voxel_size_m)
    try:
        geometry_digest = regular_box_geometry_sha256(raw_geometry)
    except (ValueError, TypeError, AttributeError):
        geometry_digest = None
    audit = dict(schema_version=1, kind='box_surface_registration_choice',
        diagnostic_only=True, execution_authorized=False,
        raw_model_role='registration_choice_only', final_fused_model_authorized=False,
        selection_uses_opening_or_path_score=False,
        selected_hypothesis='original_icp', selected_box_explanation=False,
        selected_transform_base=np.asarray(original.transform_base).tolist(),
        raw_pixel_requalification_performed=False,
        selection_reason='ORIGINAL_REGISTRATION_FAILED' if not original.ok else 'RAW_MODEL_UNAVAILABLE',
        source=dict(target_epoch=moving.identity.epoch, target_track_id=moving.identity.track_id,
            moving_stamp_ns=moving.stamp_ns, reference_stamp_ns=reference.stamp_ns,
            before_view_stamps_ns=list(surface.view_stamps_ns),
            before_point_count=len(surface.points_base),
            before_points_sha256=_array_digest(surface.points_base),
            raw_geometry_sha256=geometry_digest),
        hypotheses=dict(original_icp=dict(registration=_registration_audit(original), consistency=None)))
    if not original.ok:
        return original, original_surface, audit
    try:
        evidence = _bind_raw_model(raw_box_model, raw_geometry, moving)
    except (ValueError, TypeError, AttributeError) as exc:
        audit['selection_reason'] = 'RAW_MODEL_BINDING_UNAVAILABLE'
        audit['binding_diagnostic'] = str(exc)[:400]
        return original, original_surface, audit
    audit['raw_model_sha256'] = evidence['model_sha256']
    audit['source']['raw_geometry_sha256'] = evidence['source']['geometry_sha256']
    consistent = _union_consistency(surface, raw_geometry, raw_box_model,
                                    np.asarray(original.transform_base))
    audit['hypotheses']['original_icp']['consistency'] = consistent
    if consistent['ok']:
        audit.update(selection_reason='ORIGINAL_ICP_BOX_CONSISTENT', selected_box_explanation=True)
        return original, original_surface, audit
    identity = evaluate_registration_transform(reference, moving, np.eye(4), config)
    audit['hypotheses']['identity'] = dict(registration=_registration_audit(identity), consistency=None)
    if not identity.ok:
        audit['selection_reason'] = 'IDENTITY_REGISTRATION_REJECTED_KEEP_MEASURED'
        return original, original_surface, audit
    identity_consistent = _union_consistency(surface, raw_geometry, raw_box_model, np.eye(4))
    audit['hypotheses']['identity']['consistency'] = identity_consistent
    if not identity_consistent['ok']:
        try:
            return _append_support_plane_explanation(surface, reference, moving,
                raw_geometry, raw_box_model, config, voxel_size_m,
                original, original_surface, audit)
        except (ValueError, np.linalg.LinAlgError, FloatingPointError) as exc:
            # This optional geometric explanation cannot replace an accepted
            # measured fallback with a numerical/contract failure. Control
            # cancellation and deadline exceptions are deliberately not caught.
            audit['selection_reason'] = 'SUPPORT_PLANE_EVALUATION_FAILED_KEEP_MEASURED'
            audit['support_plane_diagnostic'] = str(exc)[:400]
            return original, original_surface, audit
    selected, fused = append_registered_view(surface, reference, moving,
        config, voxel_size_m, transform_base=np.eye(4))
    # Fixed append independently repeats every original gate. Never replace a
    # baseline success with a failure from this optional disambiguation path.
    if not selected.ok:
        audit['selection_reason'] = 'IDENTITY_APPEND_REJECTED_KEEP_MEASURED'
        return original, original_surface, audit
    audit.update(selected_hypothesis='identity', selected_box_explanation=True,
                 selected_transform_base=np.asarray(selected.transform_base).tolist(),
                 selection_reason='IDENTITY_ORIGINAL_GATES_AND_COMPLETE_UNION_CONSISTENT')
    return selected, fused, audit
