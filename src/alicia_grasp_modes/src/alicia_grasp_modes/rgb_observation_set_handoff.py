"""Freeze and associate a bounded set of independent RGB observation interests.

A set is not a target, and its envelope is only for view projection/ranking.
Every original mask owns its original ray volume and appearance reference;
there is no union mask, merged measured geometry, or image-centre reselection.
The single-interest association retains the raw 4 degree normal check and
7 mm hard offset limit, with warnings above 4 mm. All members must associate
uniquely before a set is returned.
Measured target selection and its original three-frame confirmation remain
separate caller responsibilities. This module has no I/O or motion authority.
"""
from dataclasses import asdict, dataclass, field
import hashlib
import json

import numpy as np

from alicia_grasp_modes.rgb_observation_handoff import (
    RgbHandoffFrame, RgbInterestReference, _fingerprint,
    associate_rgb_interest, freeze_rgb_interest)
from alicia_grasp_modes.rgb_observation_region import RgbObservationRegions
from alicia_grasp_modes.rgb_observation_volume import (
    build_rgb_observation_volume, project_rgb_observation_volume)


def _member_payload(member):
    return dict(volume=asdict(member.volume), source_id=member.source_id,
        generation=member.generation, epoch_ns=member.epoch_ns,
        selection_stamp_ns=member.selection_stamp_ns,
        source_stamp_ns=member.source_stamp_ns,
        mask_fingerprint=member.mask_fingerprint,
        source_image_fingerprint=member.source_image_fingerprint,
        source_depth_fingerprint=member.source_depth_fingerprint,
        histogram=_fingerprint(member.histogram), contour=_fingerprint(member.contour),
        measured_support_footprint=_fingerprint(member.measured_support_footprint_base))


def _set_fingerprint(members, envelope):
    payload = dict(semantics='independent_rgb_interest_set_v1',
        members=[_member_payload(member) for member in members],
        projection_envelope=asdict(envelope))
    return hashlib.sha256(json.dumps(payload, sort_keys=True,
        separators=(',', ':'), allow_nan=False).encode()).hexdigest()


@dataclass(frozen=True)
class RgbInterestSetReference:
    """One immutable source, 1..4 independent references, no selected target.

    ``volume`` encloses the separate members for FOV/ranking only. The actual
    membership and cross-view matching always use ``members`` and their own
    volumes. In particular, the envelope must never become one object mask.
    """
    members: tuple
    volume: object = field(init=False)
    set_fingerprint: str = field(init=False)
    observation_only: bool = field(default=True, init=False)
    metric_geometry_authority: bool = field(default=False, init=False)
    motion_authorized: bool = field(default=False, init=False)
    synthetic_measured_points_added: int = field(default=0, init=False)

    def __post_init__(self):
        members = tuple(self.members)
        if not 1 <= len(members) <= 4 or any(type(m) is not RgbInterestReference for m in members):
            raise ValueError('rgb_interest_set_members_invalid')
        first = members[0]
        source_fields = ('source_id', 'generation', 'epoch_ns', 'selection_stamp_ns',
            'source_stamp_ns', 'source_image_fingerprint', 'source_depth_fingerprint')
        volume_fields = ('image_shape', 'camera_matrix', 'T_base_camera',
            'support_plane_base', 'support_provenance', 'foreground_max_m',
            'working_depth_range_m', 'source_stamp_ns')
        for member in members:
            if (any(getattr(member, name) != getattr(first, name) for name in source_fields)
                    or any(getattr(member.volume, name) != getattr(first.volume, name)
                           for name in volume_fields)
                    or not np.array_equal(member.measured_support_footprint_base,
                                          first.measured_support_footprint_base)):
                raise ValueError('rgb_interest_set_source_mismatch')
        if len({member.mask_fingerprint for member in members}) != len(members):
            raise ValueError('rgb_interest_set_duplicate_member')
        boxes = np.asarray([member.volume.bbox_xywh for member in members])
        lower = boxes[:, :2].min(axis=0)
        upper = (boxes[:, :2]+boxes[:, 2:]).max(axis=0)
        envelope = build_rgb_observation_volume(
            bbox_xywh=tuple(lower)+tuple(upper-lower),
            **{name: getattr(first.volume, name) for name in volume_fields})
        object.__setattr__(self, 'members', members)
        object.__setattr__(self, 'volume', envelope)
        object.__setattr__(self, 'set_fingerprint', _set_fingerprint(members, envelope))

    @property
    def source_id(self): return self.members[0].source_id

    @property
    def generation(self): return self.members[0].generation

    @property
    def epoch_ns(self): return self.members[0].epoch_ns

    @property
    def selection_stamp_ns(self): return self.members[0].selection_stamp_ns

    @property
    def source_stamp_ns(self): return self.members[0].source_stamp_ns

    @property
    def source_image_fingerprint(self): return self.members[0].source_image_fingerprint

    @property
    def source_depth_fingerprint(self): return self.members[0].source_depth_fingerprint


def freeze_rgb_interest_set(frame, regions, volumes, *, support_ring_outer_radius_px):
    """Freeze each exact-source mask with its own volume; never merge masks."""
    if type(frame) is not RgbHandoffFrame:
        raise ValueError('rgb_interest_set_frame_type_invalid')
    regions, volumes = tuple(regions), tuple(volumes)
    if not 1 <= len(regions) <= 4 or len(regions) != len(volumes):
        raise ValueError('rgb_interest_set_members_invalid')
    return RgbInterestSetReference(tuple(freeze_rgb_interest(frame, region, volume,
        support_ring_outer_radius_px=support_ring_outer_radius_px)
        for region, volume in zip(regions, volumes)))


@dataclass(frozen=True)
class RgbInterestSetAssociation:
    reason: str
    frame: RgbHandoffFrame = field(repr=False, compare=False)
    reference: RgbInterestSetReference = field(repr=False, compare=False)
    observation_index: int
    member_associations: tuple = field(default=(), repr=False, compare=False)
    projected_volume_pixels: tuple = ()
    observation_only: bool = field(default=True, init=False)
    metric_geometry_authority: bool = field(default=False, init=False)
    motion_authorized: bool = field(default=False, init=False)
    synthetic_measured_points_added: int = field(default=0, init=False)

    @property
    def associated(self):
        return self.reason == 'rgb_interest_set_associated'

    @property
    def regions(self):
        return tuple(a.region for a in self.member_associations) if self.associated else ()

    @property
    def support_audit(self):
        if not self.member_associations:
            return ()
        # Existing top-level fields describe the first member. Each member now
        # has its own fixed domain, so a later failure must remain observable.
        audit = dict(self.member_associations[0].support_audit)
        audit.update(support_audit_scope='first_member_compatibility_fields',
            support_member_audits=tuple(dict(member_index=index, reason=member.reason,
                audit=dict(member.support_audit))
                for index, member in enumerate(self.member_associations)),
            support_failing_member_indices=tuple(index
                for index, member in enumerate(self.member_associations)
                if member.reason.startswith('support_inconsistent:')))
        return tuple(audit.items())


def associate_rgb_interest_set(reference, frame, proposals, measured_candidates=(),
                               observation_index=1):
    """Require every original member's unique RGB match and one-to-one mapping.

    No best partial subset is returned. A missing/ambiguous member prevents set
    association; a successful measured match for another member cannot override
    that rejection. Empty proposals still audit every original support/volume.
    """
    if type(reference) is not RgbInterestSetReference or type(frame) is not RgbHandoffFrame:
        raise ValueError('rgb_interest_set_association_type_invalid')
    def report(reason, members=(), projected=()):
        return RgbInterestSetAssociation(reason, frame, reference, observation_index,
            members, projected)
    if reference.set_fingerprint != _set_fingerprint(reference.members, reference.volume):
        return report('rgb_interest_set_reference_mutated')
    if not isinstance(proposals, RgbObservationRegions):
        proposals = tuple(proposals)
    measured_candidates = tuple(measured_candidates)
    associations = tuple(associate_rgb_interest(member, frame, proposals,
        measured_candidates, observation_index=observation_index)
        for member in reference.members)
    # A match failure is not itself a support/projection failure. Audit/projection
    # are intentionally still available when asking for a new observation view.
    for association in associations:
        if (not association.projected_volume_pixels
                or association.reason.startswith('support_inconsistent:')):
            return report(association.reason, associations)
    try:
        projected = project_rgb_observation_volume(reference.volume,
            T_base_camera=frame.T_base_camera, camera_matrix=frame.camera_matrix,
            image_shape=frame.bgr.shape[:2],
            working_depth_range_m=reference.volume.working_depth_range_m)
    except ValueError as exc:
        return report('projection_rejected: '+str(exc), associations)
    for association in associations:
        if not association.associated:
            return report(association.reason, associations, projected)
    rgb_indices = [a.rgb_region_index for a in associations]
    masks = [_fingerprint(np.asarray(a.region.mask != 0, dtype=np.uint8)) for a in associations]
    if len(set(rgb_indices)) != len(associations) or len(set(masks)) != len(associations):
        return report('rgb_interest_set_rgb_not_one_to_one', associations, projected)
    measured = [a for a in associations if a.measured_candidate_index is not None]
    if (len({a.measured_candidate_index for a in measured}) != len(measured)
            or len({a.candidate_fingerprint for a in measured}) != len(measured)):
        return report('rgb_interest_set_measured_not_one_to_one', associations, projected)
    return report('rgb_interest_set_associated', associations, projected)
