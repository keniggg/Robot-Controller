"""Build a pure endpoint validator from one immutable, signed contact plan."""
from copy import deepcopy
from dataclasses import asdict
import numpy as np
from tf.transformations import quaternion_matrix

from .contact_evidence import decode_contact_evidence
from .gripper_geometry import (
    ANALYTICAL_GRIPPER_MODEL_NAME, GripperGeometry, gripper_contract_mismatch_reason,
)
from .measured_contact import (
    evaluate_measured_contact, evaluate_measured_aperture, evaluate_measured_stage_cad,
)
from .rich_plan_integrity import plan_id_matches_content, pose_values, validate_rich_geometry


def contact_stage_next_path_contract(gcfg, pose_index):
    """Name only the complete existing fresh-path compensation route.

    Missing or untyped policy values retain the historical aperture gate.
    This selection grants no path certificate or execution authority.
    """
    if (type(pose_index) is not int or pose_index not in (0, 1)
            or not isinstance(gcfg, dict)
            or any(gcfg.get(key) is not True for key in (
                'unknown_approach_held_axes_feedforward_enabled',
                'unknown_contact_cartesian_compensation_enabled',
                'contact_endpoint_precision_enabled', 'measured_endpoint_check_enabled',
                'pregrasp_cartesian_compensation_enabled',
                'unknown_pregrasp_feedforward_enabled',
                'unknown_pregrasp_held_axes_feedforward_enabled'))):
        return None
    return ('fresh_held_axes_approach', 'fresh_compensated_contact_linear')[pose_index]


def frozen_contact_geometry(plan, gripper_config, opening_width_m, *,
                            require_measured_bilateral_contact=False):
    """No new perception, coordinate offsets, motion or physical grip claims.

    Callers establish fresh actual FK and ongoing task/epoch authority. This
    closure copies only the evidence from that exact signed plan, so expected
    target occlusion cannot silently substitute another cloud or contact band.
    """
    if not plan_id_matches_content(plan):
        raise ValueError('MEASURED_CONTACT_PLAN_CHANGED')
    evidence = decode_contact_evidence(plan, required=True)
    if type(require_measured_bilateral_contact) is not bool:
        raise ValueError('CONTACT_EVIDENCE_POLICY_INVALID')
    if require_measured_bilateral_contact and evidence['schema_version'] == 3:
        raise ValueError('MEASURED_BILATERAL_CONTACT_REQUIRED')
    cfg = deepcopy(gripper_config)
    if cfg.get('tool_jaw_axis', 'y') != 'y' or cfg.get('tool_finger_length_axis', 'z') != 'z':
        raise ValueError('MEASURED_CONTACT_TOOL_AXES_CHANGED')
    gripper = GripperGeometry(
        float(cfg.get('max_inner_gap_m', .05)),
        float(cfg.get('width_safety_margin_per_side_m', .002)),
        np.asarray(cfg.get('finger_box_xyz_m', [.0434, .0286, .06])),
        np.asarray(cfg.get('palm_box_xyz_m', [.1175, .155, .0774])),
        float(cfg.get('support_clearance_m', .003)))
    mismatch = gripper_contract_mismatch_reason(
        gripper, .05, .05, ANALYTICAL_GRIPPER_MODEL_NAME, .05,
        tool_jaw_axis='y', tool_finger_length_axis='z')
    if mismatch:
        raise ValueError('MEASURED_CONTACT_GRIPPER_MISMATCH: ' + mismatch)
    values = pose_values(plan.poses[2])
    planned = quaternion_matrix(values[3:])
    planned[:3, 3] = values[:3]
    normal = plan.object_geometry.support_normal_base
    arguments = dict(
        gripper=gripper, planned_T_base_tool0=planned,
        contact_center_base=evidence['contact_center_base'],
        contact_height_axis_base=evidence['contact_height_axis_base'],
        contact_height_bounds_m=evidence['contact_height_bounds_m'],
        minimum_contact_overlap_m=evidence['minimum_contact_overlap_m'],
        support_normal_base=[normal.x, normal.y, normal.z],
        support_offset_m=float(plan.object_geometry.support_offset_m),
        opening_width_m=float(opening_width_m))
    nominal = evaluate_measured_contact(actual_T_base_tool0=planned, **arguments)
    if not nominal.ok:
        raise ValueError('MEASURED_CONTACT_PLAN_INVALID: ' + nominal.code)
    return arguments


def plan_contact_evaluator(plan, gripper_config, opening_width_m, *,
                           translation_evidence_limit_m=None,
                           require_measured_bilateral_contact=False):
    """Freeze geometry once; optional height stress shares the closure gate.

    Bare contact ok/code keep their existing meanings. The height result is
    independent: nominal overlap alone cannot certify the required margin.
    """
    arguments = frozen_contact_geometry(plan, gripper_config, opening_width_m,
        require_measured_bilateral_contact=require_measured_bilateral_contact)
    evidence = decode_contact_evidence(plan, required=True)
    geometry_source = evidence['measurement_source']
    plan_id = str(plan.plan_id)
    if translation_evidence_limit_m is not None:
        from .contact_height_margin import evaluate_contact_height_margin
        if (type(translation_evidence_limit_m) not in (float, int)
                or not np.isfinite(translation_evidence_limit_m)
                or translation_evidence_limit_m <= 0.):
            raise ValueError('CONTACT_HEIGHT_MARGIN_CONFIG_INVALID')
    def evaluate(actual):
        report = asdict(evaluate_measured_contact(actual_T_base_tool0=actual, **arguments))
        report.update(plan_id=plan_id, scope='frozen_geometry_model_contact_only',
                      physical_grip_verified=False, contact_geometry_source=geometry_source,
                      contact_sides_are_inferred=(evidence['schema_version'] == 3))
        if translation_evidence_limit_m is not None:
            height = evaluate_contact_height_margin(
                translation_evidence_limit_m=translation_evidence_limit_m,
                actual_T_base_tool0=actual, **arguments)
            height['plan_id'] = plan_id
            report['height_registration_margin'] = height
        return report
    return evaluate


def plan_aperture_evaluator(plan, gripper_config, opening_width_m, stage_reference_pose_index,
                           *, require_measured_bilateral_contact=False):
    """Freeze union/model from this plan; input remains a new measured stage FK."""
    arguments = frozen_contact_geometry(plan, gripper_config, opening_width_m,
        require_measured_bilateral_contact=require_measured_bilateral_contact)
    evidence = decode_contact_evidence(plan, required=True)
    pose, size, _ = validate_rich_geometry(plan.object_geometry)
    if type(stage_reference_pose_index) is not int or stage_reference_pose_index not in (0, 1, 2):
        raise ValueError('MEASURED_APERTURE_STAGE_INVALID')
    stage_values = pose_values(plan.poses[stage_reference_pose_index])
    stage = quaternion_matrix(stage_values[3:]); stage[:3, 3] = stage_values[:3]
    model = None
    if evidence['schema_version'] == 3:
        from alicia_flexible_grasp.vision.regular_box_inference import RegularBoxInference
        model = RegularBoxInference.from_evidence_dict(evidence['box_model_evidence'])
    kwargs = dict(gripper=arguments['gripper'], planned_stage_T_base_tool0=stage,
        planned_contact_T_base_tool0=arguments['planned_T_base_tool0'],
        contact_center_base=evidence['contact_center_base'],
        obb_center_base=pose[:3], R_base_obb=quaternion_matrix(pose[3:])[:3, :3],
        obb_size_xyz_m=size, support_normal_base=arguments['support_normal_base'],
        support_offset_m=arguments['support_offset_m'], opening_width_m=float(opening_width_m),
        opening_fit_clearance_each_side_m=evidence.get('opening_fit_clearance_each_side_m',
            arguments['gripper'].jaw_clearance_each_side_m),
        stage_reference_pose_index=stage_reference_pose_index, inferred_model=model)
    plan_id = str(plan.plan_id)
    def evaluate(actual):
        report = evaluate_measured_aperture(actual_T_base_tool0=actual, **kwargs)
        report.update(plan_id=plan_id, contact_geometry_source=evidence['measurement_source'],
            frozen_union_includes_inferred_uncertainty=(model is not None))
        return report
    return evaluate


def plan_current_stage_cad_evaluator(plan, gripper_config, opening_width_m,
        stage_reference_pose_index, *, require_measured_bilateral_contact=False):
    """Freeze current CAD for a separately checked next-stage motion route.

    The old constant Cartesian residual remains an explicit diagnostic. It
    does not describe a held-axis realignment or compensated Cartesian path.
    This factory does not establish the existence or safety of either path.
    """
    if (type(stage_reference_pose_index) is not int
            or stage_reference_pose_index not in (0, 1)):
        raise ValueError('MEASURED_CURRENT_STAGE_INVALID')
    arguments = frozen_contact_geometry(plan, gripper_config, opening_width_m,
        require_measured_bilateral_contact=require_measured_bilateral_contact)
    evidence = decode_contact_evidence(plan, required=True)
    pose, size, _ = validate_rich_geometry(plan.object_geometry)
    diagnostic = plan_aperture_evaluator(plan, gripper_config, opening_width_m,
        stage_reference_pose_index,
        require_measured_bilateral_contact=require_measured_bilateral_contact)
    kwargs = dict(gripper=arguments['gripper'], obb_center_base=pose[:3],
        R_base_obb=quaternion_matrix(pose[3:])[:3, :3], obb_size_xyz_m=size,
        support_normal_base=arguments['support_normal_base'],
        support_offset_m=arguments['support_offset_m'], opening_width_m=float(opening_width_m),
        opening_fit_clearance_each_side_m=evidence.get('opening_fit_clearance_each_side_m',
            arguments['gripper'].jaw_clearance_each_side_m),
        stage_reference_pose_index=stage_reference_pose_index)
    plan_id = str(plan.plan_id)
    geometry_source = evidence['measurement_source']
    inferred = evidence['schema_version'] == 3
    def evaluate(actual):
        report = evaluate_measured_stage_cad(actual_T_base_tool0=actual, **kwargs)
        projection = diagnostic(actual)
        report.update(plan_id=plan_id, contact_geometry_source=geometry_source,
            frozen_union_includes_inferred_uncertainty=inferred,
            rigid_residual_projection_diagnostic=dict(projection, diagnostic_only=True,
                future_path_checked=False, execution_authorized=False,
                assumption='current_stage_rigid_residual_unchanged_at_frozen_contact'))
        return report
    return evaluate
