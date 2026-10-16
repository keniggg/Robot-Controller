"""Build a pure endpoint validator from one immutable, signed contact plan."""
from copy import deepcopy
from dataclasses import asdict
import numpy as np
from tf.transformations import quaternion_matrix

from .contact_evidence import decode_contact_evidence
from .gripper_geometry import (
    ANALYTICAL_GRIPPER_MODEL_NAME, GripperGeometry, gripper_contract_mismatch_reason,
)
from .measured_contact import evaluate_measured_contact
from .rich_plan_integrity import plan_id_matches_content, pose_values


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
