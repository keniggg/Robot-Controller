"""Immutable, phase-bound candidate limits; no ROS calls or motion authority."""
from dataclasses import dataclass
import math
import numpy as np
from .rich_plan_integrity import (plan_id_matches_content, stamp_nanoseconds,
                                  validate_finite_pose, refinement_registration_policy)


class RefinementContinuityError(ValueError):
    def __init__(self, code, reason):
        super().__init__(reason)
        self.code = code


@dataclass(frozen=True)
class RefinementBinding:
    phase_id: int
    stage: str
    started_ns: int
    deadline_ns: int
    plan_id: str
    target_track_id: str
    frame_id: str
    candidate_source: str
    source_lineage: tuple
    position: tuple
    quaternion: tuple
    support_normal: tuple
    translation_limit_m: float
    yaw_limit_rad: float
    roll_pitch_limit_rad: float


def _unit(value):
    value = np.asarray(value, dtype=float)
    norm = float(np.linalg.norm(value))
    if not np.all(np.isfinite(value)) or norm <= 1e-12:
        raise ValueError('invalid direction')
    return value / norm


def _rotation(quaternion):
    x, y, z, w = _unit(quaternion)
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def binding_from_phase(phase, config):
    stage = str(getattr(phase, 'planning_stage', '') or 'contact_replan')
    if not phase.active:
        return None
    if stage == 'contact_replan':
        return None
    if stage not in ('final_refinement', 'post_refine_confirmation'):
        raise RefinementContinuityError('FINAL_REFINE_PHASE_INVALID', 'unknown planning stage')
    try:
        plan = phase.reference_contact_plan
        start, deadline = stamp_nanoseconds(phase.header.stamp), stamp_nanoseconds(phase.deadline)
        source = stamp_nanoseconds(plan.header.stamp)
        observation = stamp_nanoseconds(phase.reference_source_stamp)
        diagnostic = str(plan.diagnostic)
        if (phase.phase_id <= 0 or not 0 < source <= observation <= start < deadline
                or not phase.reference_center_valid or not plan.valid
                or len(plan.poses) != 4 or not plan_id_matches_content(plan)
                or not (diagnostic == 'CONTACT_EXECUTION_PLAN'
                        or (stage == 'post_refine_confirmation' and diagnostic.startswith('FINAL_VISUAL_REFINE:')))
                or not plan.target_track_id
                or plan.target_track_id != phase.reference_target_track_id
                or plan.target_track_id != plan.object_geometry.target_track_id
                or plan.header.frame_id != phase.header.frame_id):
            raise ValueError('bound contact plan does not match this phase and target')
        for index, pose in enumerate(plan.poses): validate_finite_pose(pose, 'reference pose %d' % index)
        pose = plan.poses[2]
        normal = plan.object_geometry.support_normal_base
        normal = tuple(float(v) for v in _unit([normal.x, normal.y, normal.z]))
        nested = config.get('final_visual_refine_registration', {})
        if nested is None:
            raise ValueError('final_visual_refine_registration must be a mapping')
        policy = dict(refinement_registration_policy(nested))
        aliases = {'final_visual_refine_max_translation_m': 'maximum_translation_m',
                   'final_visual_refine_max_yaw_deg': 'maximum_yaw_deg'}
        legacy = {aliases[key]: config[key] for key in aliases if key in config}
        if legacy:
            policy = dict(refinement_registration_policy(dict(policy, **legacy)))
        limits = [policy['maximum_translation_m'], policy['maximum_yaw_deg'],
                  config.get('final_visual_refine_max_roll_pitch_change_deg', 4.)]
        if any(isinstance(v, bool) or not math.isfinite(float(v)) or float(v) < 0 for v in limits):
            raise ValueError('invalid configured final refinement limits')
        return RefinementBinding(int(phase.phase_id), stage, start, deadline,
            str(plan.plan_id), str(plan.target_track_id), str(plan.header.frame_id),
            str(plan.candidate_source), tuple(plan.candidate_source_lineage),
            tuple(float(getattr(pose.position, a)) for a in 'xyz'),
            tuple(float(getattr(pose.orientation, a)) for a in 'xyzw'), normal,
            float(limits[0]), math.radians(float(limits[1])), math.radians(float(limits[2])))
    except (AttributeError, TypeError, ValueError, OverflowError) as exc:
        raise RefinementContinuityError('FINAL_REFINE_PHASE_INVALID', str(exc)) from exc


def validate_candidate_continuity(binding, plan, phase_id):
    """Reject incompatible candidates without altering their poses or evidence."""
    if binding is None:
        return None
    def reject(code, reason): raise RefinementContinuityError(code, reason)
    if (int(phase_id) != binding.phase_id
            or not binding.started_ns <= stamp_nanoseconds(plan.header.stamp) < binding.deadline_ns):
        reject('FINAL_REFINE_PHASE_INVALID', 'candidate does not belong to bound refinement phase')
    if (plan.target_track_id != binding.target_track_id
            or plan.object_geometry.target_track_id != binding.target_track_id
            or plan.header.frame_id != binding.frame_id):
        reject('FINAL_REFINE_TARGET_CHANGED', 'candidate target/frame changed')
    if (plan.candidate_source != binding.candidate_source
            or tuple(plan.candidate_source_lineage) != binding.source_lineage):
        # The task permits a complete source rebind only after its one
        # qualified clear-view move. Do not replace that task-owned decision
        # with an upstream pose filter for an unrelated family.
        return dict(reference_plan_id=binding.plan_id, phase_id=binding.phase_id,
                    task_source_rebind_required=True)
    if len(plan.poses) != 4: reject('FINAL_REFINE_PLAN_MALFORMED', 'candidate needs four poses')
    try:
        pose = plan.poses[2]; validate_finite_pose(pose, 'candidate grasp')
        position = np.array([getattr(pose.position, a) for a in 'xyz'])
        normal = np.array(binding.support_normal)
        before = _rotation(binding.quaternion)
        after = _rotation([getattr(pose.orientation, a) for a in 'xyzw'])
        jaw_before = _unit(before[:,1] - np.dot(before[:,1], normal)*normal)
        jaw_after = _unit(after[:,1] - np.dot(after[:,1], normal)*normal)
        yaw = math.atan2(float(np.dot(normal, np.cross(jaw_before,jaw_after))),
                         float(np.clip(np.dot(jaw_before,jaw_after),-1.,1.)))
        axis = after[:,2]
        # Rodrigues rotation by -yaw around the original support normal.
        untilted = axis*math.cos(yaw)-np.cross(normal,axis)*math.sin(yaw)+normal*np.dot(normal,axis)*(1-math.cos(yaw))
        tilt = math.acos(float(np.clip(np.dot(_unit(before[:,2]),_unit(untilted)),-1.,1.)))
        translation = float(np.linalg.norm(position-np.array(binding.position)))
    except (ValueError,TypeError,AttributeError) as exc:
        reject('FINAL_REFINE_INVALID',str(exc))
    for value,limit,code in [(translation,binding.translation_limit_m,'FINAL_REFINE_TRANSLATION_LIMIT'),
                            (abs(yaw),binding.yaw_limit_rad,'FINAL_REFINE_YAW_LIMIT'),
                            (tilt,binding.roll_pitch_limit_rad,'FINAL_REFINE_ROLL_PITCH_LIMIT')]:
        if value > limit+1e-12: reject(code,'candidate change %.9f exceeds unchanged limit %.9f' % (value,limit))
    return dict(reference_plan_id=binding.plan_id,phase_id=binding.phase_id,
                translation_m=translation,yaw_rad=yaw,roll_pitch_rad=tilt)
