"""Stable sets of RGB places to observe, never object or movement authority.

Comparable regions are retained together. Neither a score tie nor missing
depth is converted into evidence for a particular physical object.
"""
from dataclasses import asdict, dataclass, field, replace
import hashlib
from typing import Tuple

import numpy as np

from .rgb_observation_region import (
    CenterSearchPolicy, RgbObservationRegion, RgbObservationRegions, _intersection)
from .rgb_observation_source import digest


MAX_MEMBERS = 4


def _metadata(region):
    if type(region) is not RgbObservationRegion:
        raise ValueError('rgb_interest_set_region_type_invalid')
    mask = np.asarray(region.mask)
    if mask.ndim != 2 or mask.dtype != np.bool_ or not np.any(mask):
        raise ValueError('rgb_interest_set_mask_invalid')
    y, x = np.nonzero(mask)
    bbox = (int(x.min()), int(y.min()), int(x.max()-x.min()+1), int(y.max()-y.min()+1))
    if (tuple(region.bbox_xywh) != bbox or region.area_px != len(x)
            or not np.allclose(region.centroid_xy, (x.mean(),y.mean()), atol=1e-6,rtol=0.)
            or not np.isfinite(region.score)):
        raise ValueError('rgb_interest_set_region_pixel_binding_invalid')
    return dict(bbox_xywh=list(bbox), centroid_xy=list(map(float,region.centroid_xy)),
                area_px=int(region.area_px), image_shape=list(mask.shape),
                mask_sha256=hashlib.sha256(np.ascontiguousarray(mask).tobytes()).hexdigest())


def _metadata_key(value):
    return tuple(value['bbox_xywh']), value['mask_sha256']


def observation_set_metadata(regions):
    """Canonical independent member records; never a union-object mask."""
    regions = tuple(regions)
    if not 1 <= len(regions) <= MAX_MEMBERS:
        raise ValueError('rgb_interest_set_member_budget_exceeded')
    records = tuple(sorted((_metadata(region) for region in regions),key=_metadata_key))
    observation_set_metadata_digest(records)
    return records


def observation_set_metadata_digest(metadata):
    """Verify the small public schema before hashing it (no pixel authority)."""
    if not isinstance(metadata,(tuple,list)) or not 1 <= len(metadata) <= MAX_MEMBERS:
        raise ValueError('rgb_interest_set_metadata_invalid')
    keys = {'bbox_xywh','centroid_xy','area_px','image_shape','mask_sha256'}
    records=[]
    for row in metadata:
        if type(row) is not dict or set(row) != keys:
            raise ValueError('rgb_interest_set_metadata_schema_invalid')
        box,shape,center=row['bbox_xywh'],row['image_shape'],row['centroid_xy']
        if (not isinstance(box,(list,tuple)) or len(box)!=4
                or any(type(x) is not int for x in box)
                or not isinstance(shape,(list,tuple)) or len(shape)!=2
                or any(type(x) is not int or x<=0 for x in shape)
                or min(box[:2])<0 or min(box[2:])<=0
                or box[0]+box[2]>shape[1] or box[1]+box[3]>shape[0]
                or not isinstance(center,(list,tuple)) or len(center)!=2
                or any(isinstance(x,(bool,np.bool_)) or not isinstance(x,(int,float))
                       or not np.isfinite(x) for x in center)
                or not box[0] <= center[0] <= box[0]+box[2]-1
                or not box[1] <= center[1] <= box[1]+box[3]-1
                or type(row['area_px']) is not int or not 0<row['area_px']<=box[2]*box[3]
                or type(row['mask_sha256']) is not str or len(row['mask_sha256'])!=64
                or any(c not in '0123456789abcdef' for c in row['mask_sha256'])):
            raise ValueError('rgb_interest_set_metadata_invalid')
        records.append(dict(row,bbox_xywh=list(box),image_shape=list(shape),
                            centroid_xy=list(map(float,center))))
    if (any(row['image_shape']!=records[0]['image_shape'] for row in records)
            or len({row['mask_sha256'] for row in records})!=len(records)
            or records != sorted(records,key=_metadata_key)):
        raise ValueError('rgb_interest_set_metadata_order_or_identity_invalid')
    return digest(records)


def observation_set_digest(regions):
    return observation_set_metadata_digest(observation_set_metadata(regions))


def _canonical(regions):
    values=[(_metadata(region),region) for region in regions]
    values.sort(key=lambda pair:_metadata_key(pair[0]))
    return tuple(region for _,region in values)


def select_rgb_interest_set(proposals, policy):
    """Unique interest stays single; ties retain every original-margin member."""
    if type(proposals) is not RgbObservationRegions or type(policy) is not CenterSearchPolicy:
        raise ValueError('rgb_interest_set_proposal_or_policy_invalid')
    if 'budget' in proposals.reason:
        raise ValueError('rgb_interest_set_source_budget_exhausted')
    candidates=tuple(proposals.candidates)
    if not candidates:
        raise ValueError('rgb_interest_set_empty')
    if any(type(region) is not RgbObservationRegion for region in candidates):
        raise ValueError('rgb_interest_set_region_type_invalid')
    if any(not np.isfinite(region.score) or region.mask.shape!=proposals.image_shape
           for region in candidates):
        raise ValueError('rgb_interest_set_source_shape_or_score_invalid')
    ordered=sorted(candidates,key=lambda region:region.score)
    members=tuple(region for region in ordered
                  if region.score-ordered[0].score <= policy.ambiguity_score_margin)
    if (proposals.total_candidate_count>len(candidates)
            and len(members)==len(candidates)):
        raise ValueError('rgb_interest_set_competitors_truncated')
    if not 1 <= len(members) <= MAX_MEMBERS:
        raise ValueError('rgb_interest_set_member_budget_exceeded')
    if proposals.selected_index is not None:
        index=proposals.selected_index
        if (type(index) is not int or not 0<=index<len(candidates)
                or len(members)!=1 or candidates[index] is not members[0]):
            raise ValueError('rgb_interest_set_selection_inconsistent')
    elif proposals.reason!='ambiguous_rgb_regions' or len(members)<2:
        raise ValueError('rgb_interest_set_selection_unavailable')
    result=_canonical(members)
    observation_set_metadata(result)
    return result


@dataclass(frozen=True)
class RgbObservationSetTrack:
    confirmed: bool
    reason: str
    consistent_frames: int
    source_id: str
    generation: int
    source_stamps_ns: Tuple[int,...]
    regions: Tuple[RgbObservationRegion,...] = field(repr=False,compare=False)
    observation_only: bool = field(default=True,init=False)
    metric_geometry_authority: bool = field(default=False,init=False)
    object_identity_confirmed: bool = field(default=False,init=False)
    motion_authorized: bool = field(default=False,init=False)


def _unique_assignment(anchors,candidates,min_iou,max_shift):
    edges=[]
    for anchor in anchors:
        choices=[]
        for index,region in enumerate(candidates):
            intersection=_intersection(anchor,region)
            iou=intersection/max(1,anchor.area_px+region.area_px-intersection)
            shift=np.linalg.norm(np.asarray(anchor.centroid_xy)-region.centroid_xy)
            if iou>=min_iou and shift<=max_shift:
                choices.append(index)
        edges.append(choices)
    solutions=[]
    def visit(used,chosen):
        if len(solutions)>=2: return
        if len(chosen)==len(edges):
            solutions.append(tuple(chosen));return
        for index in edges[len(chosen)]:
            if index not in used: visit(used|{index},chosen+[index])
    visit(set(),[])
    return solutions[0] if len(solutions)==1 else None


class RgbObservationSetTracker:
    """Three same-source frames of a fixed, unordered set of visible regions.

    Previously included members remain while uniquely matched anywhere in the
    complete returned proposals, even if their score moves outside the margin.
    New competitors reset confirmation. Every match uses the fixed first mask,
    preventing pairwise drift and score-order changes from swapping identity.
    """
    def __init__(self,*,required_frames=3,min_mask_iou=.70,max_joint_delta_rad=.004,
                 max_gap_seconds=1.,max_centroid_shift_fraction=.015):
        values=(min_mask_iou,max_joint_delta_rad,max_gap_seconds,max_centroid_shift_fraction)
        if (type(required_frames) is not int or required_frames<3
                or not np.all(np.isfinite(values)) or not .70<=min_mask_iou<=1.
                or not 0<max_joint_delta_rad<=.004 or min(values[2:])<=0):
            raise ValueError('rgb_interest_set_tracker_policy_invalid')
        self.required_frames=required_frames;self.min_mask_iou=min_mask_iou
        self.max_joint_delta_rad=max_joint_delta_rad;self.max_gap_ns=int(max_gap_seconds*1e9)
        self.max_centroid_shift_fraction=max_centroid_shift_fraction
        self.reset()

    def reset(self):
        self._key=None;self._anchors=();self._joints=None;self._shape=None
        self._stamps=[];self._last_stamp=None

    def _start(self,regions,joints,shape):
        anchors=[]
        for region in regions:
            mask=np.array(region.mask,copy=True);mask.setflags(write=False)
            anchors.append(replace(region,mask=mask))
        self._anchors=tuple(anchors);self._joints=joints.copy();self._shape=shape;self._stamps=[]

    def observe(self,proposals,*,policy,source_id,generation,stamp_ns,joints_rad):
        if (not isinstance(source_id,str) or not source_id.strip()
                or type(generation) is not int or generation<0
                or type(stamp_ns) is not int or stamp_ns<=0
                or type(policy) is not CenterSearchPolicy):
            self.reset();raise ValueError('rgb_interest_set_source_invalid')
        joints=np.asarray(joints_rad,dtype=float)
        if joints.ndim!=1 or not joints.size or not np.all(np.isfinite(joints)):
            self.reset();raise ValueError('rgb_interest_set_measured_joints_invalid')
        key=(source_id,generation,digest(asdict(policy)))
        if key!=self._key:
            self.reset();self._key=key
        try:
            selected=select_rgb_interest_set(proposals,policy)
        except ValueError as exc:
            self._anchors=();self._stamps=[];self._last_stamp=stamp_ns
            return self._report(False,str(exc),key,())
        if self._last_stamp is not None and stamp_ns<=self._last_stamp:
            self._anchors=();self._stamps=[]
            return self._report(False,'nonincreasing_source_stamp',key,())
        gap=self._last_stamp is not None and stamp_ns-self._last_stamp>self.max_gap_ns
        self._last_stamp=stamp_ns
        consistent=(bool(self._anchors) and not gap and proposals.image_shape==self._shape
                    and joints.shape==self._joints.shape
                    and np.max(np.abs(joints-self._joints))<=self.max_joint_delta_rad)
        assignment=(_unique_assignment(self._anchors,proposals.candidates,self.min_mask_iou,
                    self.max_centroid_shift_fraction*min(proposals.image_shape)) if consistent else None)
        if assignment is None:
            regions=selected;self._start(regions,joints,proposals.image_shape)
        else:
            regions=tuple(proposals.candidates[index] for index in assignment)
            extras=tuple(region for region in selected if all(region is not old for old in regions))
            if len(regions)+len(extras)>MAX_MEMBERS:
                self._anchors=();self._stamps=[]
                return self._report(False,'rgb_interest_set_member_budget_exceeded',key,())
            if extras:
                old_anchors=self._anchors
                old_joints=self._joints.copy()
                self._start(extras,joints,proposals.image_shape)
                self._anchors=old_anchors+self._anchors;self._joints=old_joints
                regions+=extras
        self._stamps.append(stamp_ns);self._stamps=self._stamps[-self.required_frames:]
        confirmed=len(self._stamps)>=self.required_frames
        return self._report(confirmed,'stable_rgb_observation_set' if confirmed else
                            'collecting_consistent_rgb_set_frames',key,_canonical(regions))

    def _report(self,confirmed,reason,key,regions):
        return RgbObservationSetTrack(confirmed,reason,len(self._stamps),key[0],key[1],
                                      tuple(self._stamps),tuple(regions))
