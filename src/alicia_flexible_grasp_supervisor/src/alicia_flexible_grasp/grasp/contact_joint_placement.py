"""One bounded joint placement hint, never a path or execution certificate.

The caller supplies geometry freshly frozen from the same signed contact plan.
Both failed endpoint transforms are translated rigidly only to choose a hint.
Fresh following, IK, registered contact evidence and both complete curves must
be evaluated for that new family. The measured target and CAD never move.
"""
from copy import deepcopy
import math
import time

import numpy as np

from .contact_height_margin import (
    _contact_translation_intervals, evaluate_contact_height_margin,
)
from .gripper_geometry import (
    ANALYTICAL_FINGER_PAIR_CENTER_TOOL_XYZ_M, GRIPPER_CONTRACT_TOLERANCE_M, GripperGeometry,
    _box_corners, _stage_boxes,
)
from alicia_flexible_grasp.robot.finger_cad_separation import (
    _PADDING_M, _full_axis_features, validated_model_matches,
)
from alicia_flexible_grasp.robot.observation_path_guard import (
    FrozenObservationScene, SerialUrdfFk, gripper_target_overlap,
)
from alicia_flexible_grasp.robot.pregrasp_compensation import pose_matrix
from alicia_flexible_grasp.robot.stationary_following import ARM_NAMES


JOINT_PLACEMENT_POLICY = 'bounded_joint_frozen_cad_contact_placement_v3'
MAXIMUM_EVALUATIONS = 256
MAXIMUM_ITERATIONS = 48
MAXIMUM_SECONDS = 2.0
MAXIMUM_SOLVES = 3
SOLVER_RESOLUTION_MM = .0001


class _OriginalRequestUnavailable(Exception):
    """The caller returned a value other than the required exact True."""


class _CallerDeadlineError(Exception):
    """Carry an application exception past local input/optimizer handlers."""
    def __init__(self, error):
        self.error = error
        self.original_traceback = error.__traceback__


def joint_placement_policy_parameters():
    """Include this value in request-local selected-recipe parameter binding."""
    return dict(policy=JOINT_PLACEMENT_POLICY,
                maximum_evaluations=MAXIMUM_EVALUATIONS,
                maximum_iterations=MAXIMUM_ITERATIONS,
                maximum_seconds=MAXIMUM_SECONDS,
                maximum_solves=MAXIMUM_SOLVES,
                solver='SLSQP_shared_budget_separating_halfspace_branches',
                seed_policy='balanced_origins_then_nearest_CAD_halfspace_intersections',
                objective_tolerance_mm=SOLVER_RESOLUTION_MM,
                finite_difference_step_mm=SOLVER_RESOLUTION_MM,
                displacement_round_decimals=12,
                ray_hint=dict(policy=RAY_HINT_POLICY, maximum_alpha=1.,
                    maximum_intervals=7,
                    extra_proposal_margin_m=GRIPPER_CONTRACT_TOLERANCE_M,
                    extra_proposal_margin_source='GRIPPER_CONTRACT_TOLERANCE_M',
                    axis_policy='original_15_axes_full_CAD_union',
                    zero_translation_policy='no_hint_and_no_replacement_fallback'))


RAY_HINT_POLICY = 'first_sufficient_separation_on_balanced_ray'

def _ray_merge(intervals):
    merged=[]
    for lo,hi in sorted(intervals):
        if lo>hi:continue
        if merged and lo<=merged[-1][1]:merged[-1]=(merged[-1][0],max(hi,merged[-1][1]))
        else:merged.append((lo,hi))
    return merged

def _ray_intersect(first,second):
    return _ray_merge([(max(a,c),min(b,d)) for a,b in first for c,d in second if max(a,c)<=min(b,d)])

def _ray_halfspace(b,s,margin,maximum):
    if s==0.:return [(0.,maximum)] if b>=margin else []
    boundary=(margin-b)/s
    lo,hi=(max(0.,boundary),maximum) if s>0. else (0.,min(maximum,boundary))
    return [(lo,hi)] if lo<=hi else []


def _ray_rounded_displacement(alpha, seed, bound, lo, hi, maximum):
    displacement = np.round(alpha*seed, decimals=12)
    distance = float(np.linalg.norm(displacement))
    if (not np.all(np.isfinite(displacement)) or not math.isfinite(distance)
            or not 1e-9 < distance <= bound):
        return None, None
    actual_alpha = float(displacement @ seed / float(seed @ seed))
    if not (0. < actual_alpha <= maximum and lo <= actual_alpha <= hi):
        return None, None
    return displacement, actual_alpha


def _minimum_change_ray_hint(*,fk,transforms,seed,bound,scene,gripper,opening,geometry,limit,
                     solver_resolution_mm,check,maximum_evaluations):
    """Finite union-of-halfspaces proposals using every pinned CAD vertex.

    Caller has already validated the same-source failure and frozen geometry.
    A positive proposal still needs fresh evidence, IK and complete paths.
    """
    audit=dict(policy=RAY_HINT_POLICY,attempted=True,proposed=False,
        candidate_evaluations=0,extra_proposal_margin_m=GRIPPER_CONTRACT_TOLERANCE_M,
        extra_proposal_margin_source='GRIPPER_CONTRACT_TOLERANCE_M',
        extra_margin_is_not_physical_error_bound=True,full_hull_axes_generated=False)
    check()
    distance=float(np.linalg.norm(seed))
    if not 1e-9<distance<=bound:
        audit['reason']='original balanced ray unavailable';return None,audit
    model=fk.finger_cad_model
    if not validated_model_matches(model,fk.model_sha256):
        raise ValueError('validated finger CAD required for ray hint')
    maximum=min(1.,bound/distance)
    permitted=[(0.,maximum)]; constraints=[];supports=[]
    target=np.asarray(scene.center); target_r=np.asarray(scene.rotation); target_size=np.asarray(scene.size)
    for transform in transforms:
        check()
        rotation=transform[:3,:3]
        axes=np.concatenate((rotation.T,target_r.T,
            np.cross(rotation.T[:,None,:],target_r.T[None,:,:]).reshape(-1,3)))
        norms=np.linalg.norm(axes,axis=1)
        axes=axes[norms>1e-12]/norms[norms>1e-12,None]
        tool_axes=axes@rotation
        for body,center,size in _stage_boxes(transform,gripper,opening,'y','z'):
            check()
            support=float(np.min(_box_corners(center,rotation,size)@scene.normal+scene.offset)-gripper.support_clearance_m)
            support_slope=float(scene.normal@seed);supports.append((support,support_slope))
            permitted=_ray_intersect(permitted,_ray_halfspace(support,support_slope,1e-9,maximum))
            if body!='palm':
                lower=np.full(len(axes),np.inf);upper=np.full(len(axes),-np.inf)
                for part in model._body_components(body):
                    check()
                    projected=part.vertices_at_zero@tool_axes.T
                    shift=opening*(tool_axes@part.velocity_per_opening)
                    lower=np.minimum(lower,projected.min(axis=0)+shift)
                    upper=np.maximum(upper,projected.max(axis=0)+shift)
                padding=_PADDING_M*np.abs(tool_axes).sum(axis=1)
                lower,upper=lower-padding,upper+padding
                middle=axes@(target-transform[:3,3]);radius=np.abs(axes@target_r)@(target_size/2.)
                # Upper-bound the original roundoff reserve for every point
                # on this bounded ray; never subtract a smaller reserve.
                scale=max(np.max(np.abs(target)),np.max(np.abs(transform[:3,3]))+bound,
                    np.max(target_size),np.max(np.abs(lower)),np.max(np.abs(upper)))
                roundoff=64.*np.finfo(float).eps*(1.+float(scale))
                constants=np.r_[middle-radius-upper,lower-middle-radius]-roundoff
                vectors=np.vstack((-axes,axes))
            else:
                middle=axes@(center-target)
                radius=np.abs(axes@rotation)@(size/2.)+np.abs(axes@target_r)@(target_size/2.)
                constants=np.r_[middle-radius,-middle-radius]
                vectors=np.vstack((axes,-axes))
            slopes=vectors@seed
            constraints.append((constants,vectors))
            body_allowed=_ray_merge([interval for b,s in zip(constants,slopes)
                for interval in _ray_halfspace(float(b),float(s),GRIPPER_CONTRACT_TOLERANCE_M,maximum)])
            permitted=_ray_intersect(permitted,body_allowed)
    audit['feasible_ray_intervals']=permitted
    if not permitted:
        audit['reason']='no sufficient ray halfspace intersection';return None,audit
    if permitted[0][0]==0.:
        audit.update(zero_translation_sufficient=True,
            reason='zero translation already satisfies projections; retain original failure without a replacement')
        return None,audit
    # A union of at most two tails per body excludes at most six intervals.
    # Retain every interval; this is not a monotonic bisection assumption.
    if len(permitted)>7:raise ValueError('finite ray interval bound exceeded')
    step=solver_resolution_mm/1000./distance
    for lo,hi in permitted:
        check()
        if audit['candidate_evaluations']>=maximum_evaluations:
            raise TimeoutError('joint placement evaluation limit exhausted')
        alpha=min(hi,lo+step)
        displacement,actual_alpha = _ray_rounded_displacement(
            alpha,seed,bound,lo,hi,maximum)
        if displacement is None:
            continue
        target_margins=[float(np.max(b+a@displacement)) for b,a in constraints]
        support_margins=[float(b+scene.normal@displacement) for b,_ in supports]
        if min(target_margins)<GRIPPER_CONTRACT_TOLERANCE_M:continue
        audit['candidate_evaluations']+=1
        proposed=transforms[0].copy();proposed[:3,3]+=displacement
        height=evaluate_contact_height_margin(translation_evidence_limit_m=limit,
            actual_T_base_tool0=proposed,**dict(geometry,planned_T_base_tool0=proposed))
        check()
        height_slack=float(height.get('maximum_symmetric_normal_translation_m',0.))-limit
        if height.get('ok') is not True or min(support_margins+[height_slack])<=1e-9:continue
        accepted=True
        for transform in transforms:
            translated=transform.copy();translated[:3,3]+=displacement
            for body,center,size in _stage_boxes(translated,gripper,opening,'y','z'):
                check()
                rejected,_=gripper_target_overlap(fk,gripper,opening,body,translated,center,size,scene)
                clearance=float(np.min(_box_corners(center,translated[:3,:3],size)@scene.normal+scene.offset))
                if rejected or clearance<gripper.support_clearance_m:accepted=False;break
            if not accepted:break
        check()
        if not accepted:continue
        audit.update(proposed=True,alpha=actual_alpha,translation_base_m=displacement.tolist(),
            translation_distance_m=float(np.linalg.norm(displacement)),contact_height_margin=height,
            minimum_target_projection_margin_m=min(target_margins),
            minimum_rigid_proposal_margin_m=min(target_margins+support_margins+[height_slack]),
            reason='minimum finite ray hint; original independent endpoint kernels passed')
        return displacement,audit
    audit['reason']='ray candidates failed original endpoint or full height evidence gates'
    return None,audit


def _separating_branch_seed(affine, displacement, bound, supports, normal,
                            used_seeds, check):
    """Nearest alternative SAT cell hint, with no collision authority.

    A body's separation constraint is a *union* of halfspaces. A local
    max-min solve can balance two opposing jaw faces below zero and stop,
    while another CAD face has a clear solution. Project onto each other
    separating halfspace together with a currently blocking body's active
    halfspace. These small vectorized projections select one new cell; they
    never replace the full objective or the independent endpoint kernel.
    """
    check()
    values = [b+a @ displacement for b, a in affine]
    active = [int(np.argmax(value)) for value in values]
    blocked = [i for i, value in enumerate(values) if value[active[i]] <= 1e-9]
    candidates = []
    for changed in blocked:
        check()
        _, axes = affine[changed]
        wanted = -values[changed]
        others = [i for i in blocked if i != changed] or [None]
        for retained in others:
            check()
            if retained is None:
                # The projection is only needed for genuinely blocked axes.
                weights = np.maximum(wanted, 0.)
                steps = weights[:, None]*axes
                valid = wanted > 0.
            else:
                other = affine[retained][1][active[retained]]
                deficit = -values[retained][active[retained]]
                cosine = axes @ other
                denominator = 1.-cosine**2
                valid = denominator > 1e-12
                first = np.divide(wanted-cosine*deficit, denominator,
                    out=np.zeros(len(axes)), where=valid)
                second = np.divide(deficit-cosine*wanted, denominator,
                    out=np.zeros(len(axes)), where=valid)
                valid &= (first >= 0.) & (second >= 0.)
                steps = first[:, None]*axes+second[:, None]*other
                # Include the other two exact projection cases: only one
                # halfspace boundary is active and the other is satisfied.
                changed_only = np.maximum(wanted, 0.)[:, None]*axes
                retained_only = np.broadcast_to(max(deficit, 0.)*other, axes.shape)
                valid = np.r_[valid,
                    changed_only @ other >= deficit,
                    np.einsum('ij,ij->i', retained_only, axes) >= wanted]
                steps = np.concatenate((steps, changed_only, retained_only))
            points = displacement+steps
            valid &= np.all(np.isfinite(points), axis=1)
            valid &= np.linalg.norm(points, axis=1) <= bound
            valid &= np.min(supports)+points @ normal >= 0.
            for index in np.flatnonzero(valid):
                point = np.round(points[index], decimals=12)
                # Equivalent mesh triangles can generate slightly different
                # floating-point planes. A seed within the solver stencil of
                # an attempted one cannot provide meaningful branch coverage.
                repeated = any(np.linalg.norm(point-np.asarray(used))
                               <= SOLVER_RESOLUTION_MM/1000. for used in used_seeds)
                if repeated or not 1e-9 < np.linalg.norm(point) <= bound:
                    continue
                candidates.append((float(np.linalg.norm(steps[index])), changed,
                    -1 if retained is None else retained, int(index % len(axes)), point))
    check()
    if not candidates:
        return None, None
    _, changed, retained, index, point = min(candidates, key=lambda item: item[:4])
    return point, dict(policy='nearest_separating_halfspace_intersection',
        changed_body_index=changed, retained_body_index=retained,
        changed_axis_index=index, changed_axis_base=affine[changed][1][index].tolist(),
        previous_active_axis_base=affine[changed][1][active[changed]].tolist(),
        translation_base_m=point.tolist())


def _finite(value, name):
    if (isinstance(value, (bool, np.bool_))
            or not isinstance(value, (int, float, np.number))
            or not math.isfinite(float(value))):
        raise ValueError('finite numeric '+name+' required')
    return float(value)


def _array(value, shape, name):
    result = np.asarray(value)
    if (result.shape != shape or result.dtype.kind not in 'fiu'
            or any(isinstance(v, (bool, np.bool_))
                   for v in np.asarray(value, dtype=object).flat)
            or not np.all(np.isfinite(result))):
        raise ValueError('finite numeric '+name+' required')
    return np.array(result, dtype=float, copy=True)


def _rotation(value, name):
    result = _array(value, (3, 3), name)
    if (not np.allclose(result.T @ result, np.eye(3), rtol=0., atol=1e-8)
            or abs(float(np.linalg.det(result))-1.) > 1e-8):
        raise ValueError('proper '+name+' required')
    return result


def _unit(value, name):
    result = _array(value, (3,), name)
    if abs(float(np.linalg.norm(result))-1.) > 1e-6:
        raise ValueError('unit '+name+' required')
    return result


def _failure_seed(failure, plan_id, source_stamp_ns, goal_pose, normal, model_xml):
    """Validate the terminal contract independently of whether a seed exists."""
    if (not isinstance(failure, dict) or failure.get('success') is not False
            or failure.get('plan_id') != plan_id
            or failure.get('scope') != 'read_only_candidate_constant_bias_prediction'
            or failure.get('execution_authorized') is not False
            or failure.get('failed_stage') != 'grasp'
            or failure.get('failure_code') != 'CONTACT_SEQUENCE_PRECHECK_FAILED'
            or not str(failure.get('reason', '')).startswith(
                'predicted_command continuous geometry: target OBB collision at ')):
        raise ValueError('bound compensated endpoint failure unavailable')
    endpoint = failure.get('contact_terminal_failure', {})
    if (not isinstance(endpoint, dict) or type(endpoint.get('schema_version')) is not int
            or endpoint['schema_version'] != 1 or endpoint.get('plan_id') != plan_id
            or type(source_stamp_ns) is not int or source_stamp_ns <= 0
            or endpoint.get('source_stamp_ns') != source_stamp_ns
            or endpoint.get('curve') != 'predicted_command'
            or endpoint.get('stage') != 'grasp'
            or endpoint.get('endpoint_screen_only') is not True
            or endpoint.get('execution_authorized') is not False):
        raise ValueError('same-source terminal diagnostic unavailable')
    fk = SerialUrdfFk(model_xml, ARM_NAMES)
    sample = failure.get('source_sample', {})
    epoch = failure.get('epoch_ns')
    if (endpoint.get('model_sha256') != fk.model_sha256
            or failure.get('model_sha256') != fk.model_sha256
            or sample.get('model_sha256') != fk.model_sha256
            or tuple(sample.get('joint_names', ())) != ARM_NAMES
            or type(epoch) is not int or epoch <= 0
            or type(sample.get('epoch_ns')) is not int or sample.get('epoch_ns') != epoch
            or any(sample.get(key, 0) < epoch for key in ('sdk_stamp_ns', 'accepted_stamp_ns'))):
        raise ValueError('model or measured-bias epoch changed')
    bias = (_array(sample.get('sdk_positions_rad'), (6,), 'SDK joints')
            - _array(sample.get('accepted_positions_rad'), (6,), 'accepted joints'))
    if np.max(np.abs(bias)) > .035:
        raise ValueError('following outside original contact contract')
    q = _array(endpoint.get('command_endpoint_rad'), (6,), 'command endpoint')
    command, nominal = fk(q), fk(q-bias)
    values = _array(goal_pose, (7,), 'goal pose')
    if abs(float(np.linalg.norm(values[3:]))-1.) > 1e-6:
        raise ValueError('goal quaternion is not unit length')
    goal = pose_matrix(values)
    angle = math.acos(float(np.clip((np.trace(goal[:3, :3].T @ nominal[:3, :3])-1.)/2., -1., 1.)))
    if np.linalg.norm(goal[:3, 3]-nominal[:3, 3]) > .0005 or angle > .005:
        raise ValueError('nominal endpoint does not describe this fixed goal')
    residual = command[:3, 3]-nominal[:3, 3]
    seed = -.5*(residual-normal*float(normal @ residual))
    return fk, command, seed


def contact_joint_placement(*, failure, plan_id, source_stamp_ns, goal_pose,
        support_normal_base, model_xml, maximum_translation_m, scene,
        contact_geometry, translation_evidence_limit_m, deadline_check):
    """Suggest at most one replacement for the existing lateral retry slot.

    ``scene`` must come from ``FrozenObservationScene.from_contact_plan`` and
    ``contact_geometry`` from ``frozen_contact_geometry`` for that same plan.
    ``deadline_check`` checks the original request deadline/owner; it returns
    exactly True while available. No local clock may extend that deadline.
    Missing optimizers/models or an unsuccessful bounded solve return no hint.
    """
    started = time.monotonic()
    audit = dict(policy=JOINT_PLACEMENT_POLICY, proposed=False,
        fallback_allowed=False, execution_authorized=False, motion_authorized=False,
        path_verified=False, future_following_verified=False,
        target_relocated=False, joint_ik_recomputed=False,
        command_transform_is_rigid_proposal_only=True, evaluation_count=0,
        parameters=joint_placement_policy_parameters())

    def check():
        try:
            available = deadline_check()
        except Exception as error:
            # Application cancellation/contract errors can inherit the same
            # builtins used by numpy/scipy. They must never become a failed
            # optional proposal or unlock fallback to another family.
            raise _CallerDeadlineError(error) from error
        if available is not True:
            raise _OriginalRequestUnavailable('original request deadline or owner unavailable')
        if time.monotonic()-started > MAXIMUM_SECONDS:
            raise TimeoutError('joint placement computation budget exhausted')

    try:
        if not callable(deadline_check):
            raise ValueError('original request deadline check required')
        check()
        if (not isinstance(scene, FrozenObservationScene)
                or scene.plan_id != plan_id or scene.source_ns != source_stamp_ns
                or type(scene.source_ns) is not int or scene.source_ns <= 0
                or not scene.digest or not scene.geometry_digest or not scene.track_id):
            raise ValueError('same signed contact source/scene binding required')
        normal = _unit(support_normal_base, 'support normal')
        scene_normal = _unit(scene.normal, 'scene normal')
        target_center = _array(scene.center, (3,), 'target center')
        target_rotation = _rotation(scene.rotation, 'target rotation')
        target_size = _array(scene.size, (3,), 'target size')
        if np.any(target_size <= 0.):
            raise ValueError('positive target dimensions required')
        _finite(scene.offset, 'support offset')
        bound = _finite(maximum_translation_m, 'remaining displacement')
        limit = _finite(translation_evidence_limit_m, 'translation evidence')
        if bound <= 0. or limit <= 0.:
            raise ValueError('positive original displacement and evidence required')
        if not isinstance(contact_geometry, dict):
            raise ValueError('frozen contact geometry required')
        geometry = deepcopy(contact_geometry)
        gripper = geometry.get('gripper')
        if not isinstance(gripper, GripperGeometry):
            raise ValueError('frozen validated gripper required')
        goal = pose_matrix(goal_pose)
        if not np.array_equal(goal, np.asarray(geometry.get('planned_T_base_tool0'))):
            # A quaternion round trip need not reproduce identical matrix bits.
            if not np.allclose(goal, geometry.get('planned_T_base_tool0'), rtol=0., atol=1e-12):
                raise ValueError('proposal goal differs from frozen contact geometry')
        geometry_normal = _unit(geometry.get('support_normal_base'), 'contact normal')
        geometry_offset = _finite(geometry.get('support_offset_m'), 'contact support offset')
        if (not np.allclose(normal, geometry_normal, rtol=0., atol=1e-12)
                or not np.allclose(normal/np.linalg.norm(normal), scene_normal,
                                   rtol=0., atol=1e-12)
                or abs(geometry_offset/np.linalg.norm(normal)-scene.offset) > 1e-12):
            raise ValueError('contact and collision support planes differ')
        sample = failure.get('source_sample', {}) if isinstance(failure, dict) else {}
        if (sample.get('measurement_kind') != 'stationary_sdk_following_not_executed_endpoint'
                or sample.get('certifies_command_response_or_calibration') is not False):
            raise ValueError('original measured following source required')
        now = _finite(sample.get('stamp_sec'), 'following sample clock')
        for key in ('sdk_stamp_ns', 'accepted_stamp_ns'):
            stamp = sample.get(key)
            if (type(stamp) is not int or stamp <= 0
                    or not 0. <= now-stamp*1e-9 <= .5):
                raise ValueError('following source sample was stale or invalid')
        opening = _finite(sample.get('accepted_gripper_opening_m'), 'measured opening')
        if (not 0. < opening <= gripper.max_inner_gap_m
                or _finite(failure.get('opening_m'), 'failed opening') != opening
                or sample.get('opening_stamp_ns') != sample.get('accepted_stamp_ns')):
            raise ValueError('same measured opening source required')
        fk, command, seed = _failure_seed(failure, plan_id, source_stamp_ns,
                                        goal_pose, normal, model_xml)
        baseline_height = evaluate_contact_height_margin(
            translation_evidence_limit_m=limit, actual_T_base_tool0=goal, **geometry)
        if baseline_height.get('ok') is not True:
            raise ValueError('original full contact evidence interval is invalid')
        audit.update(fallback_allowed=True, input_valid=True)
        seed_distance = float(np.linalg.norm(seed))
        audit['seed_available'] = bool(1e-9 < seed_distance <= bound)
        if not audit['seed_available']:
            # A zero/out-of-range origin midpoint does not rule out another
            # CAD separation cell. The unchanged nominal origin is a bounded
            # starting point, never a zero-displacement accepted proposal.
            seed = np.zeros(3)
        check()
        if fk.finger_cad_model is None:
            raise ValueError('validated full finger CAD unavailable')
        audit['optimizer_attempts'] = []
        ray_evaluations = 0
        ray_displacement, ray_report = _minimum_change_ray_hint(
            fk=fk, transforms=(goal, command), seed=seed, bound=bound, scene=scene,
            gripper=gripper, opening=opening, geometry=geometry, limit=limit,
            solver_resolution_mm=SOLVER_RESOLUTION_MM, check=check,
            maximum_evaluations=MAXIMUM_EVALUATIONS)
        ray_evaluations = ray_report['candidate_evaluations']
        audit.update(ray_hint=ray_report, evaluation_count=ray_evaluations)
        if ray_report.get('zero_translation_sufficient') is True:
            check()
            audit.update(fallback_allowed=False,reason=ray_report['reason'],
                         elapsed_sec=time.monotonic()-started)
            return None, audit
        if ray_displacement is not None:
            check()
            proposed = goal.copy()
            proposed[:3, 3] += ray_displacement
            audit.update(proposed=True, plan_id=str(plan_id), source_stamp_ns=source_stamp_ns,
                model_sha256=fk.model_sha256, epoch_ns=failure['epoch_ns'],
                scene_digest=scene.digest, geometry_digest=scene.geometry_digest,
                normal_translation_m=float(scene_normal @ ray_displacement),
                translation_base_m=ray_displacement.tolist(),
                translation_distance_m=float(np.linalg.norm(ray_displacement)),
                maximum_translation_m=bound, opening_m=opening,
                minimum_rigid_proposal_margin_m=ray_report['minimum_rigid_proposal_margin_m'],
                contact_height_margin=ray_report['contact_height_margin'],
                seed_policy=RAY_HINT_POLICY,
                reason='finite ray hint only; fresh registered evidence, IK and complete paths required',
                elapsed_sec=time.monotonic()-started)
            return proposed, audit
        # Use the exact frozen nominal goal. The bound helper has separately
        # checked that FK(command-bias) represents this same endpoint.
        transforms = (goal, command)
        affine, supports = [], []
        features = {body: _full_axis_features(fk.finger_cad_model, body, opening)
                    for body in ('left_finger', 'right_finger')}
        for transform in transforms:
            rotation = transform[:3, :3]
            for body, center, size in _stage_boxes(transform, gripper, opening, 'y', 'z'):
                supports.append(float(np.min(_box_corners(center, rotation, size)
                    @ scene_normal+scene.offset)-gripper.support_clearance_m))
                if body != 'palm':
                    vertices, faces, edges = features[body]
                    target_local = rotation.T @ target_rotation
                    axes = np.concatenate((faces, target_local.T,
                        np.cross(edges[:, None, :], target_local.T[None, :, :]).reshape(-1, 3)))
                    lengths = np.linalg.norm(axes, axis=1)
                    axes = axes[lengths > 1e-12]/lengths[lengths > 1e-12, None]
                    projection = vertices @ axes.T
                    padding = _PADDING_M*np.abs(axes).sum(axis=1)
                    lo, hi = projection.min(axis=0)-padding, projection.max(axis=0)+padding
                    middle = axes @ (rotation.T @ (target_center-transform[:3, 3]))
                    radius = np.abs(axes @ target_local) @ (target_size/2.)
                    world_axes = axes @ rotation.T
                    affine.append((np.r_[middle-radius-hi, lo-middle-radius],
                                   np.vstack((-world_axes, world_axes))))
                else:
                    axes = np.concatenate((rotation.T, target_rotation.T,
                        np.cross(rotation.T[:, None, :], target_rotation.T[None, :, :]).reshape(-1, 3)))
                    lengths = np.linalg.norm(axes, axis=1)
                    axes = axes[lengths > 1e-12]/lengths[lengths > 1e-12, None]
                    middle = axes @ (center-target_center)
                    radius = np.abs(axes @ rotation) @ (size/2.)+np.abs(axes @ target_rotation) @ (target_size/2.)
                    affine.append((np.r_[middle-radius, -middle-radius], np.vstack((axes, -axes))))
        check()
        from scipy.optimize import minimize
        cache = {}
        # The exact production contact-interval kernel needs only this 2-D
        # point to change under translation. Do not rebuild three diagnostic
        # measured-contact reports for every optimizer finite difference.
        projection = goal[:3, :3].T[[0, 2], :]
        contact_point = (projection @ (np.asarray(geometry['contact_center_base'])
            - goal[:3, 3])-ANALYTICAL_FINGER_PAIR_CENTER_TOOL_XYZ_M[[0, 2]])
        height_direction = projection @ np.asarray(geometry['contact_height_axis_base'])
        normal_direction = projection @ geometry_normal
        band_lo, band_hi = geometry['contact_height_bounds_m']
        minimum_overlap = geometry['minimum_contact_overlap_m']
        original_support = baseline_height['nominal']['minimum_support_clearance_m']

        def evaluate(displacement_mm):
            check()
            key = tuple(float(v) for v in displacement_mm)
            if key not in cache:
                if ray_evaluations+len(cache) >= MAXIMUM_EVALUATIONS:
                    raise TimeoutError('joint placement evaluation limit exhausted')
                displacement = np.asarray(key)/1000.
                intervals = _contact_translation_intervals(
                    contact_point-projection @ displacement, height_direction,
                    normal_direction, band_lo, band_hi, minimum_overlap)
                support_lower = (gripper.support_clearance_m-original_support
                                 -float(geometry_normal @ displacement))
                slack = max([min(-max(lo, support_lower)-limit, hi-limit)
                             for lo, hi in intervals] or [-bound-limit])
                margins = [float(np.max(b+a @ displacement)) for b, a in affine]
                margins.extend(np.asarray(supports)+float(scene_normal @ displacement))
                margins.append(slack)
                cache[key] = np.asarray(margins)
                audit['evaluation_count'] = ray_evaluations+len(cache)
            return cache[key]

        bound_mm = bound*1000.
        iterations_left = MAXIMUM_ITERATIONS
        used_seeds = set()
        seed_report = dict(policy=('balance_failed_contact_tool_origins_in_support_plane'
            if audit['seed_available'] else 'original_nominal_origin'))
        audit['optimizer_attempts'] = []
        for attempt in range(MAXIMUM_SOLVES):
            check()
            used_seeds.add(tuple(np.round(seed, decimals=12)))
            initial = np.r_[seed*1000., float(evaluate(seed*1000.).min()*1000.)]
            optimized = minimize(lambda x: -x[3], initial,
                jac=lambda x: np.asarray([0., 0., 0., -1.]),
                method='SLSQP', bounds=[(-bound_mm, bound_mm)]*3+[(None, None)],
                constraints=[dict(type='ineq', fun=lambda x: evaluate(x[:3])*1000.-x[3]),
                             dict(type='ineq', fun=lambda x: bound_mm**2-float(x[:3] @ x[:3]))],
                # At max/min kinks, asking for convergence far below the
                # derivative stencil causes needless oscillation. This is
                # numerical solve accuracy only: all physical inequalities
                # and the independent original kernels remain unchanged.
                options=dict(maxiter=iterations_left, ftol=SOLVER_RESOLUTION_MM,
                             eps=SOLVER_RESOLUTION_MM))
            check()
            iterations_left -= max(1, int(optimized.nit))
            audit.update(optimizer_success=bool(optimized.success),
                         optimizer_message=str(optimized.message))
            displacement = np.round(np.asarray(optimized.x[:3])/1000., decimals=12)
            margins = evaluate(displacement*1000.)
            audit['optimizer_attempts'].append(dict(seed=seed_report,
                optimizer_success=bool(optimized.success), iterations=int(optimized.nit),
                minimum_margin_m=float(margins.min()), evaluation_count=ray_evaluations+len(cache)))
            if not optimized.success:
                raise ValueError('bounded joint placement optimizer did not converge')
            if float(margins.min()) > 1e-9:
                break
            if iterations_left <= 0 or attempt+1 >= MAXIMUM_SOLVES:
                break
            seed, seed_report = _separating_branch_seed(affine, displacement, bound,
                supports, scene_normal, used_seeds, check)
            if seed is None:
                break
        distance = float(np.linalg.norm(displacement))
        margins = evaluate(displacement*1000.)
        proposed = goal.copy()
        proposed[:3, 3] += displacement
        height = evaluate_contact_height_margin(
            translation_evidence_limit_m=limit, actual_T_base_tool0=proposed,
            **dict(geometry, planned_T_base_tool0=proposed))
        if (not 1e-9 < distance <= bound or float(margins.min()) <= 1e-9
                or height.get('ok') is not True):
            raise ValueError('joint hint does not satisfy original endpoint/evidence gates')
        # Independent unchanged production endpoint kernel checks all original
        # CAD vertices; optimizer projections never become a collision waiver.
        for transform in transforms:
            translated = transform.copy()
            translated[:3, 3] += displacement
            for body, center, size in _stage_boxes(translated, gripper, opening, 'y', 'z'):
                check()
                collision, _ = gripper_target_overlap(fk, gripper, opening,
                    body, translated, center, size, scene)
                clearance = float(np.min(_box_corners(center, translated[:3, :3], size)
                                         @ scene_normal+scene.offset))
                if collision or clearance < gripper.support_clearance_m:
                    raise ValueError('original endpoint kernel rejected joint hint')
        proposed = goal.copy()
        proposed[:3, 3] += displacement
        audit.update(proposed=True, plan_id=str(plan_id), source_stamp_ns=source_stamp_ns,
            model_sha256=fk.model_sha256, epoch_ns=failure['epoch_ns'],
            scene_digest=scene.digest, geometry_digest=scene.geometry_digest,
            normal_translation_m=float(scene_normal @ displacement),
            translation_base_m=displacement.tolist(), translation_distance_m=distance,
            maximum_translation_m=bound, opening_m=opening,
            minimum_rigid_proposal_margin_m=float(margins.min()),
            contact_height_margin=height,
            seed_policy=joint_placement_policy_parameters()['seed_policy'],
            reason='joint hint only; fresh registered evidence, IK and complete paths required')
        audit['elapsed_sec'] = time.monotonic()-started
        return proposed, audit
    except _CallerDeadlineError as wrapper:
        raise wrapper.error.with_traceback(wrapper.original_traceback)
    except _OriginalRequestUnavailable as exc:
        audit.update(fallback_allowed=False, reason=str(exc),
                     elapsed_sec=time.monotonic()-started)
        return None, audit
    except (ImportError, AttributeError, KeyError, TypeError, ValueError,
            RuntimeError, OverflowError, np.linalg.LinAlgError) as exc:
        # These errors originate inside input validation or the optimizer.
        # External callback exceptions were handled separately above.
        audit.update(reason=str(exc), elapsed_sec=time.monotonic()-started)
        return None, audit
    except TimeoutError as exc:
        audit.update(reason=str(exc), elapsed_sec=time.monotonic()-started)
        return None, audit
