"""Finite ownership bookkeeping for a future RGB-only acquisition adapter.

No ROS, motion, collision checker or perception acceptance lives here. Evidence
digests reference records independently verified by the caller. This module
checks their binding and event order, not their truth or physical sufficiency.
In particular a submission reservation is a request for gateway admission, not
permission to execute. The future gateway must independently check the actual
scene, path, controller reference and owner immediately before execution.

One owner permits one episode per selection. Replacing a preview/request or
receiving a different candidate cannot renew the episode's budgets. Both ROS
and monotonic deadlines are retained, and clock rollback revokes the episode.
Snapshots are diagnostic; they cannot restore or mint an execution capability.
"""
from dataclasses import asdict, dataclass, field
import hashlib
import json
import re

from .active_observation_policy import observation_budget


MAX_SOURCE_AGE_NS = 800_000_000
MAX_SESSION_DURATION_NS = 600_000_000_000
_TERMINAL = {'revoked', 'expired', 'exhausted', 'handed_off'}
_VISUAL_DEFICITS = {'insufficient_valid_depth', 'no_center_instance',
                   'insufficient_geometry_quality', 'initial_acquisition_incomplete'}


def _integer(value, name, minimum=1):
    if type(value) is not int or value < minimum:
        raise ValueError('RGB_ACQUISITION_INVALID_' + name.upper())
    return value


def _token(value, name):
    if (type(value) is not str or not value or len(value) > 256
            or any(c.isspace() for c in value)):
        raise ValueError('RGB_ACQUISITION_INVALID_' + name.upper())
    return value


def _hash(value, name):
    if type(value) is not str or re.fullmatch('[0-9a-f]{64}', value) is None:
        raise ValueError('RGB_ACQUISITION_INVALID_' + name.upper())
    return value


def _digest(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(',', ':'),
                                    allow_nan=False).encode()).hexdigest()


def _stamps(values):
    if type(values) is not tuple or not 3 <= len(values) <= 32:
        raise ValueError('RGB_ACQUISITION_REQUIRES_BOUNDED_DISTINCT_SOURCES')
    for stamp in values:
        _integer(stamp, 'source_stamp_ns')
    if any(a >= b for a, b in zip(values, values[1:])):
        raise ValueError('RGB_ACQUISITION_SOURCE_STAMPS_NOT_INCREASING')
    return values


@dataclass(frozen=True)
class AcquisitionContext:
    """Immutable identity/configuration, excluding pose and dynamic telemetry."""
    source_id: str
    selection_generation: int
    selection_stamp_ns: int
    camera_geometry_sha256: str
    handeye_sha256: str
    robot_description_sha256: str
    control_epoch_ns: int
    perception_policy_sha256: str
    mode: str = 'unknown'
    strategy: str = 'observe_return'

    def __post_init__(self):
        _token(self.source_id, 'source_id')
        for name in ('selection_generation', 'selection_stamp_ns', 'control_epoch_ns'):
            _integer(getattr(self, name), name)
        for name in ('camera_geometry_sha256', 'handeye_sha256', 'robot_description_sha256',
                     'perception_policy_sha256'):
            _hash(getattr(self, name), name)
        if self.mode != 'unknown' or self.strategy != 'observe_return':
            raise ValueError('RGB_ACQUISITION_SELECTION_UNSUPPORTED')

    @property
    def selection_key(self):
        return self.selection_generation, self.selection_stamp_ns


@dataclass(frozen=True)
class InterestSource:
    """Caller-verified stable RGB interest, never a real 3-D target identity.

    The evidence record must retain RGB/mask pixels, exact source TF and joint
    provenance, dimensions and candidate selection. A digest alone does not
    verify any of these inputs. Supplemental interests additionally require an
    independently verified RGB association record to the original interest.
    """
    context: AcquisitionContext
    candidate_id: str
    source_stamps_ns: tuple
    evidence_sha256: str
    rgb_sha256: str
    region_mask_sha256: str
    base_camera_sha256: str
    feedback_sha256: str
    image_shape: tuple
    bbox_xywh: tuple
    association_sha256: str = ''
    observation_only: bool = field(default=True, init=False)
    measured_target_identity: bool = field(default=False, init=False)

    def __post_init__(self):
        if type(self.context) is not AcquisitionContext:
            raise ValueError('RGB_ACQUISITION_CONTEXT_REQUIRED')
        _token(self.candidate_id, 'candidate_id')
        _stamps(self.source_stamps_ns)
        if (type(self.image_shape) is not tuple or len(self.image_shape) != 2
                or any(type(v) is not int or v <= 0 for v in self.image_shape)
                or type(self.bbox_xywh) is not tuple or len(self.bbox_xywh) != 4
                or any(type(v) is not int for v in self.bbox_xywh)):
            raise ValueError('RGB_ACQUISITION_PIXEL_GEOMETRY_INVALID')
        x, y, width, height = self.bbox_xywh
        if (min(x, y) < 0 or min(width, height) <= 0
                or x+width > self.image_shape[1] or y+height > self.image_shape[0]):
            raise ValueError('RGB_ACQUISITION_REGION_OUTSIDE_IMAGE')
        if self.source_stamps_ns[0] <= max(self.context.selection_stamp_ns,
                                          self.context.control_epoch_ns):
            raise ValueError('RGB_ACQUISITION_SOURCE_PREDATES_CONTEXT')
        for name in ('evidence_sha256', 'rgb_sha256', 'region_mask_sha256',
                     'base_camera_sha256', 'feedback_sha256'):
            _hash(getattr(self, name), name)
        if self.association_sha256:
            _hash(self.association_sha256, 'association_sha256')

    @property
    def source_stamp_ns(self):
        return self.source_stamps_ns[-1]


@dataclass(frozen=True)
class ObservationPreview:
    preview_id: str
    interest: InterestSource
    endpoint_pose_sha256: str
    preview_record_sha256: str
    motion_authorized: bool = field(default=False, init=False)

    def __post_init__(self):
        _token(self.preview_id, 'preview_id')
        if type(self.interest) is not InterestSource:
            raise ValueError('RGB_ACQUISITION_INTEREST_REQUIRED')
        _hash(self.endpoint_pose_sha256, 'endpoint_pose_sha256')
        _hash(self.preview_record_sha256, 'preview_record_sha256')

    @property
    def binding_sha256(self):
        return _digest(asdict(self))


@dataclass(frozen=True)
class SubmissionEvidence:
    """References the external checker/gateway must actually inspect.

    No ``collision_passed`` flag is accepted here. These hashes cannot be used
    by an adapter as a substitute for real scene/path validation or as proof
    that the RGB interest volume encloses all occupied or unknown space.
    """
    preview_binding_sha256: str
    scene_record_sha256: str
    trajectory_sha256: str
    path_validation_record_sha256: str
    controller_reference_sha256: str

    def __post_init__(self):
        for name in self.__dataclass_fields__:
            _hash(getattr(self, name), name)


@dataclass(frozen=True)
class SubmissionReservation:
    session_id: str
    submission_id: str
    observation_index: int
    preview: ObservationPreview
    evidence: SubmissionEvidence
    reserved_stamp_ns: int
    deadline_ns: int
    monotonic_deadline_ns: int
    requires_independent_gateway_admission: bool = field(default=True, init=False)
    certifies_scene_or_path: bool = field(default=False, init=False)
    motion_authorized: bool = field(default=False, init=False)
    contact_authorized: bool = field(default=False, init=False)


@dataclass(frozen=True)
class MeasuredTargetEvidence:
    """Only a reference to fresh standard perception + RGB association results.

    The caller must verify original geometry gates and the association record;
    this class does not turn a hash into measured points or an accepted target.
    Handoff still requires the normal grasp planner and all contact checks.
    """
    context: AcquisitionContext
    candidate_id: str
    target_track_id: str
    target_epoch: int
    source_stamps_ns: tuple
    perception_acceptance_sha256: str
    measured_geometry_sha256: str
    measured_points_sha256: str
    support_evidence_sha256: str
    rgb_association_sha256: str

    def __post_init__(self):
        if type(self.context) is not AcquisitionContext:
            raise ValueError('RGB_ACQUISITION_CONTEXT_REQUIRED')
        _token(self.candidate_id, 'candidate_id')
        _token(self.target_track_id, 'target_track_id')
        if self.target_track_id == self.candidate_id:
            raise ValueError('RGB_ACQUISITION_INTEREST_ID_IS_NOT_TARGET_ID')
        _integer(self.target_epoch, 'target_epoch', 0)
        _stamps(self.source_stamps_ns)
        for name in ('perception_acceptance_sha256', 'measured_geometry_sha256',
                     'measured_points_sha256', 'support_evidence_sha256', 'rgb_association_sha256'):
            _hash(getattr(self, name), name)


@dataclass(frozen=True)
class AcquisitionSnapshot:
    session_id: str
    context: AcquisitionContext
    candidate_id: str
    phase: str
    created_stamp_ns: int
    deadline_ns: int
    monotonic_deadline_ns: int
    max_observations: int
    consumed_submissions: int
    settled_source_stamp_ns: int
    terminal_reason: str
    preview_id: str
    submission_id: str
    motion_authorized: bool = field(default=False, init=False)
    contact_authorized: bool = field(default=False, init=False)
    restores_execution_authority: bool = field(default=False, init=False)


@dataclass(frozen=True)
class AcquisitionHandoff:
    session_id: str
    completed_stamp_ns: int
    submission_id: str
    gateway_receipt_sha256: str
    measured_settle_record_sha256: str
    settled_source_stamp_ns: int
    evidence: MeasuredTargetEvidence
    requires_standard_grasp_validation: bool = field(default=True, init=False)
    carries_rgb_synthetic_geometry: bool = field(default=False, init=False)
    motion_authorized: bool = field(default=False, init=False)
    contact_authorized: bool = field(default=False, init=False)


class RgbAcquisitionOwner:
    """Single-caller state machine; caller serializes methods and owns revocation.

    All methods are pure bookkeeping. Revoking an in-flight reservation does
    not cancel or stop physical hardware; an adapter must notify its existing
    gateway owner and monitor the actual result. No disable command is implied.
    Do not reconstruct this owner for retries or restart it to renew a budget.
    Process restart needs a separately audited persistent-owner recovery path;
    diagnostic snapshots deliberately provide no restore interface.
    """
    def __init__(self):
        self._context = None
        self._phase = 'empty'
        self._last_ros = self._last_monotonic = 0

    def start(self, *, session_id, interest, duration_ns, max_observations,
              now_ns, monotonic_ns, control_available):
        _token(session_id, 'session_id')
        if type(interest) is not InterestSource:
            raise ValueError('RGB_ACQUISITION_INTEREST_REQUIRED')
        _integer(duration_ns, 'duration_ns')
        if duration_ns > MAX_SESSION_DURATION_NS:
            raise ValueError('RGB_ACQUISITION_SESSION_DURATION_TOO_LONG')
        limit = observation_budget(max_observations)
        _integer(now_ns, 'now_ns'); _integer(monotonic_ns, 'monotonic_ns')
        if control_available is not True:
            raise ValueError('RGB_ACQUISITION_CONTROL_UNAVAILABLE')
        self._fresh(interest.source_stamp_ns, now_ns)
        if self._context is not None:
            old, new = self._context.selection_key, interest.context.selection_key
            if new == old:
                raise ValueError('RGB_ACQUISITION_EPISODE_ALREADY_EXISTS_FOR_SELECTION')
            if new[0] <= old[0] or new[1] <= old[1]:
                raise ValueError('RGB_ACQUISITION_SELECTION_ROLLBACK')
            if self._phase not in _TERMINAL:
                raise ValueError('RGB_ACQUISITION_REVOKE_PREVIOUS_OWNER_FIRST')
            if now_ns < self._last_ros or monotonic_ns < self._last_monotonic:
                raise ValueError('RGB_ACQUISITION_CLOCK_ROLLBACK')
            if session_id == self._session_id:
                raise ValueError('RGB_ACQUISITION_NEW_SESSION_ID_REQUIRED')
        self._session_id = session_id
        self._context, self._interest = interest.context, interest
        self._phase, self._reason = 'interest', ''
        self._created = self._last_ros = now_ns
        self._last_monotonic = monotonic_ns
        self._deadline, self._monotonic_deadline = now_ns+duration_ns, monotonic_ns+duration_ns
        self._limit, self._used = limit, 0
        self._preview = self._reservation = None
        self._completed = self._settled = 0
        self._gateway_receipt = self._settle_receipt = ''
        self._used_preview_ids = set()
        return self.snapshot()

    @staticmethod
    def _fresh(stamp, now):
        if not 0 <= now-stamp <= MAX_SOURCE_AGE_NS:
            raise ValueError('RGB_ACQUISITION_SOURCE_STALE_OR_FUTURE')

    def _live(self, *, current_context, now_ns, monotonic_ns, control_available):
        _integer(now_ns, 'now_ns'); _integer(monotonic_ns, 'monotonic_ns')
        if self._context is None or self._phase in _TERMINAL:
            raise ValueError('RGB_ACQUISITION_OWNER_NOT_ACTIVE')
        reason = None
        if current_context != self._context or type(current_context) is not AcquisitionContext:
            reason = 'context_changed'
        elif control_available is not True:
            reason = 'control_owner_changed'
        elif now_ns < self._last_ros or monotonic_ns < self._last_monotonic:
            reason = 'clock_rollback'
        elif now_ns >= self._deadline or monotonic_ns >= self._monotonic_deadline:
            self._phase, self._reason = 'expired', 'shared_session_deadline'
            raise ValueError('RGB_ACQUISITION_SHARED_DEADLINE_EXPIRED')
        if reason:
            self.revoke(reason)
            raise ValueError('RGB_ACQUISITION_OWNER_REVOKED_' + reason.upper())
        self._last_ros, self._last_monotonic = now_ns, monotonic_ns

    def _require_phase(self, *phases):
        if self._phase not in phases:
            raise ValueError('RGB_ACQUISITION_WRONG_PHASE_' + self._phase.upper())

    def _source(self, interest, now_ns, *, after_settle=False):
        if (type(interest) is not InterestSource or interest.context != self._context
                or interest.candidate_id != self._interest.candidate_id):
            raise ValueError('RGB_ACQUISITION_INTEREST_BINDING_CHANGED')
        self._fresh(interest.source_stamp_ns, now_ns)
        if after_settle and (interest.source_stamps_ns[0] <= self._settled
                             or not interest.association_sha256):
            raise ValueError('RGB_ACQUISITION_FRESH_ASSOCIATED_VIEW_REQUIRED')

    def refresh_interest(self, interest, **live):
        """A newer current-view request may replace a preview, never its budget."""
        self._live(**live); self._require_phase('interest', 'preview')
        self._source(interest, live['now_ns'], after_settle=self._settled > 0)
        if interest.source_stamp_ns <= self._interest.source_stamp_ns:
            raise ValueError('RGB_ACQUISITION_NEW_SOURCE_REQUIRED')
        self._interest, self._preview, self._phase = interest, None, 'interest'
        return self.snapshot()

    def preview(self, preview, **live):
        self._live(**live); self._require_phase('interest', 'preview')
        if type(preview) is not ObservationPreview or preview.interest != self._interest:
            raise ValueError('RGB_ACQUISITION_PREVIEW_SOURCE_MISMATCH')
        self._source(preview.interest, live['now_ns'], after_settle=self._settled > 0)
        if preview.preview_id in self._used_preview_ids:
            raise ValueError('RGB_ACQUISITION_PREVIEW_ALREADY_CONSUMED')
        self._preview, self._phase = preview, 'preview'
        return self.snapshot()

    def reserve_submission(self, evidence, **live):
        """Consume an attempt BEFORE an external call; no refund/retry replay."""
        self._live(**live); self._require_phase('preview')
        self._source(self._preview.interest, live['now_ns'], after_settle=self._settled > 0)
        if (type(evidence) is not SubmissionEvidence
                or evidence.preview_binding_sha256 != self._preview.binding_sha256):
            raise ValueError('RGB_ACQUISITION_PATH_PREVIEW_BINDING_MISMATCH')
        if self._used >= self._limit:
            self._phase, self._reason = 'exhausted', 'submission_budget_consumed'
            raise ValueError('RGB_ACQUISITION_SUBMISSION_BUDGET_EXHAUSTED')
        self._used += 1
        submission_id = _digest(dict(session_id=self._session_id, index=self._used,
            preview=self._preview.binding_sha256, evidence=asdict(evidence),
            deadline_ns=self._deadline, monotonic_deadline_ns=self._monotonic_deadline))
        self._reservation = SubmissionReservation(self._session_id, submission_id,
            self._used, self._preview, evidence, live['now_ns'], self._deadline,
            self._monotonic_deadline)
        self._used_preview_ids.add(self._preview.preview_id)
        self._phase, self._completed, self._settled = 'submission_reserved', 0, 0
        self._gateway_receipt = self._settle_receipt = ''
        return self._reservation

    def _receipt(self, submission_id, receipt_sha256):
        _token(submission_id, 'submission_id')
        _hash(receipt_sha256, 'external_gateway_receipt_sha256')
        if self._reservation is None or submission_id != self._reservation.submission_id:
            raise ValueError('RGB_ACQUISITION_SUBMISSION_RECEIPT_MISMATCH')

    def record_submission_rejected(self, *, submission_id, rejection_receipt_sha256, **live):
        """Only for verified rejection BEFORE motion; unknown/partial -> revoke.

        The external receipt must certify no trajectory was submitted. This
        class does not inspect it. The consumed submission is never refunded.
        """
        self._live(**live); self._require_phase('submission_reserved')
        self._receipt(submission_id, rejection_receipt_sha256)
        self._preview = self._reservation = None
        if self._used >= self._limit:
            self._phase, self._reason = 'exhausted', 'submission_budget_consumed'
        else:
            self._phase = 'interest'
        return self.snapshot()

    def record_execution_completed(self, *, submission_id, trajectory_sha256,
                                   completed_stamp_ns, gateway_receipt_sha256, **live):
        self._live(**live); self._require_phase('submission_reserved')
        self._receipt(submission_id, gateway_receipt_sha256)
        _hash(trajectory_sha256, 'trajectory_sha256')
        _integer(completed_stamp_ns, 'completed_stamp_ns')
        if (trajectory_sha256 != self._reservation.evidence.trajectory_sha256
                or not self._reservation.reserved_stamp_ns < completed_stamp_ns <= live['now_ns']):
            raise ValueError('RGB_ACQUISITION_EXECUTION_RECEIPT_INVALID')
        self._completed, self._phase = completed_stamp_ns, 'awaiting_settle'
        self._gateway_receipt = gateway_receipt_sha256
        return self.snapshot()

    def record_settled(self, *, submission_id, settled_source_stamp_ns,
                       measured_settle_record_sha256, **live):
        """Records the caller's measured feedback verdict, never a fixed sleep."""
        self._live(**live); self._require_phase('awaiting_settle')
        self._receipt(submission_id, measured_settle_record_sha256)
        _integer(settled_source_stamp_ns, 'settled_source_stamp_ns')
        if not self._completed < settled_source_stamp_ns <= live['now_ns']:
            raise ValueError('RGB_ACQUISITION_SETTLE_SOURCE_INVALID')
        self._fresh(settled_source_stamp_ns, live['now_ns'])
        self._settled, self._phase = settled_source_stamp_ns, 'awaiting_measured_target'
        self._settle_receipt = measured_settle_record_sha256
        return self.snapshot()

    def request_supplement(self, interest, *, deficit_code, deficit_source_stamp_ns,
                           deficit_record_sha256, **live):
        """A new view requires current visual deficit and RGB association.

        Missing TF, collisions, IK failures and stale packets are not visual
        deficits. The caller must verify the referenced perception record.
        """
        self._live(**live); self._require_phase('awaiting_measured_target')
        self._source(interest, live['now_ns'], after_settle=True)
        _hash(deficit_record_sha256, 'deficit_record_sha256')
        _integer(deficit_source_stamp_ns, 'deficit_source_stamp_ns')
        if (deficit_code not in _VISUAL_DEFICITS
                or deficit_source_stamp_ns != interest.source_stamp_ns):
            raise ValueError('RGB_ACQUISITION_CURRENT_VISUAL_DEFICIT_REQUIRED')
        if self._used >= self._limit:
            self._phase, self._reason = 'exhausted', 'observation_budget_consumed'
            raise ValueError('RGB_ACQUISITION_OBSERVATION_BUDGET_EXHAUSTED')
        self._interest, self._preview, self._reservation = interest, None, None
        self._phase = 'interest'
        return self.snapshot()

    def handoff_measured_target(self, evidence, **live):
        """Consume one fresh measured handoff; still no contact authorization."""
        self._live(**live); self._require_phase('awaiting_measured_target')
        if (type(evidence) is not MeasuredTargetEvidence
                or evidence.context != self._context
                or evidence.candidate_id != self._interest.candidate_id
                or evidence.source_stamps_ns[0] <= self._settled):
            raise ValueError('RGB_ACQUISITION_FRESH_MEASURED_HANDOFF_REQUIRED')
        self._fresh(evidence.source_stamps_ns[-1], live['now_ns'])
        result = AcquisitionHandoff(self._session_id, live['now_ns'],
            self._reservation.submission_id, self._gateway_receipt, self._settle_receipt,
            self._settled, evidence)
        self._phase = 'handed_off'
        self._preview = self._reservation = None
        return result

    def revoke(self, reason):
        """Invalidate bookkeeping only; caller handles any physical execution."""
        _token(reason, 'revocation_reason')
        if self._context is None:
            raise ValueError('RGB_ACQUISITION_OWNER_NOT_STARTED')
        if self._phase not in _TERMINAL:
            self._phase, self._reason = 'revoked', reason
        return self.snapshot()

    def snapshot(self):
        if self._context is None:
            raise ValueError('RGB_ACQUISITION_OWNER_NOT_STARTED')
        return AcquisitionSnapshot(self._session_id, self._context,
            self._interest.candidate_id, self._phase, self._created, self._deadline,
            self._monotonic_deadline, self._limit, self._used, self._settled,
            self._reason, self._preview.preview_id if self._preview else '',
            self._reservation.submission_id if self._reservation else '')
