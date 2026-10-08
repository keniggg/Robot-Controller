#!/usr/bin/env python3
"""Pure local-recovery evidence ledger; no ROS, filesystem or motion interface.

This module validates supplied structure and consistency, not authenticity of a
bag or production target binding. Hardware authorization is ALWAYS false. The
manual-ROI motor diagnostic adapter can never manufacture production identity.
"""
from dataclasses import dataclass, replace
import hashlib
import json
import math
import re
import numpy as np

HEX=re.compile(r'^[0-9a-f]{64}$')
Q=2*math.pi/4096


def canonical(value):return json.dumps(value,sort_keys=True,separators=(',',':'),allow_nan=False)
def digest(value):return hashlib.sha256(canonical(value).encode()).hexdigest()
def exact_int(v,name,minimum=1):
    if type(v) is not int or v<minimum:raise ValueError(name+' must be an exact integer')
    return v

def sha(value,name):
    if not isinstance(value,str) or not HEX.fullmatch(value):raise ValueError(name+' requires SHA256')
    return value

def text_id(value,name):
    if not isinstance(value,str) or not value.strip() or value!=value.strip():raise ValueError('missing '+name)
    return value

def vector(value,n,name):
    a=np.asarray(value,dtype=float)
    if a.shape!=(n,) or not np.isfinite(a).all():raise ValueError(name+' shape/finite values invalid')
    return a

def plane(value):
    a=vector(value,4,'table plane')
    if abs(np.linalg.norm(a[:3])-1)>1e-6:raise ValueError('table plane must be normalized')
    return a


@dataclass(frozen=True)
class FaultIdentity:
    original_plan_id:str
    original_plan_digest:str
    target_track_id:str
    target_epoch:int
    generation:int
    selection_stamp_ns:int
    fault_source_stamp_ns:int
    fault_code:str
    fault_digest:str

    def __post_init__(self):
        for key in ('original_plan_id','target_track_id','fault_code'):text_id(getattr(self,key),key)
        for key in ('original_plan_digest','fault_digest'):sha(getattr(self,key),key)
        for key in ('target_epoch','generation','selection_stamp_ns','fault_source_stamp_ns'):exact_int(getattr(self,key),key)
        if self.fault_source_stamp_ns<=self.selection_stamp_ns:raise ValueError('fault must follow selection')


@dataclass(frozen=True)
class ActiveTargetIdentity:
    target_track_id:str
    target_epoch:int
    generation:int
    selection_stamp_ns:int
    bridge_artifact_sha256:str

    def __post_init__(self):
        text_id(self.target_track_id,'active target track')
        for key in ('target_epoch','generation','selection_stamp_ns'):exact_int(getattr(self,key),key)
        sha(self.bridge_artifact_sha256,'historical failure to current target bridge')


@dataclass(frozen=True)
class LocalEvidence:
    payload_json:str
    level:str

    @property
    def data(self):return json.loads(self.payload_json)
    @property
    def evidence_digest(self):return digest({'level':self.level,'payload':self.data})

    @classmethod
    def from_motor_diagnostic(cls,report,frame,report_sha256):
        """Preserve a real manual-ROI report without creating a track or stamp."""
        sha(report_sha256,'source report')
        if (report.get('diagnostic_only') is not True or report.get('motion_authorized') is not False
                or report.get('production_target_identity_verified') is not False):
            raise ValueError('expected explicitly unbound diagnostic report')
        exact_int(frame['source_stamp_ns'],'source_stamp_ns')
        plane(frame['table_plane_board'])
        for key in ('rgb','z16'):sha(frame['source_sha256'][key],key)
        # Retain supplied report context; never invent identities or epochs.
        gaps=['production_target_binding','motor_support_footprint','bounded_local_response_domain']
        context={k:report[k] for k in ('board_reference_id','driver_epoch_ns','model') if k in report}
        if not report.get('board_reference_id'):gaps.append('board_reference_identity')
        if type(report.get('driver_epoch_ns')) is not int:gaps.append('driver_epoch')
        stamps=frame.get('exact_source_stamps_ns')
        if not isinstance(stamps,dict) or set(stamps)!={'rgb','z16','charuco_quality','camera_board_tf'}:
            gaps.append('complete_same_source_stamps')
        elif any(type(v) is not int or v!=frame['source_stamp_ns'] for v in stamps.values()):
            raise ValueError('diagnostic exact-source provenance disagrees')
        window=frame.get('independent_stationary_window',{})
        ts=window.get('source_stamps_ns',[]);qq=window.get('positions_rad',[])
        if not ts or len(ts)!=len(qq):gaps.append('independent_stationary_samples')
        else:
            ts=np.array([exact_int(v,'stationary source') for v in ts],dtype=np.int64)
            qs=np.array([vector(v[:6],6,'stationary q') for v in qq])
            source=frame['source_stamp_ns']
            if (len(ts)<6 or np.any(np.diff(ts)<=0) or np.max(np.diff(ts))>250_000_000
                    or ts[0]>source-400_000_000 or ts[-1]<source+400_000_000
                    or np.max(np.ptp(qs,axis=0))>math.radians(.15)):
                raise ValueError('diagnostic independent centered feedback provenance disagrees')
        data=dict(frame,source_report_sha256=report_sha256,source_session=report['session'],
                  supplied_report_context=context,provenance_gaps=gaps)
        return cls(canonical(data),'manual_roi_diagnostic')

    @classmethod
    def from_bound_observation(cls,data):
        """Future adapter input; it must supply exact production binding proof.

        Passing these structural checks is not proof that a source artifact is
        authentic. Runtime integration must verify referenced raw artifacts.
        """
        p=json.loads(canonical(data));source=exact_int(p['source_stamp_ns'],'source')
        if p.get('kind')!='same_source_motor_table_board':raise ValueError('wrong evidence kind')
        if p.get('plane_label')!='motor_adjacent_table':raise ValueError('board depth is not motor table')
        for key in ('board_reference_id','camera_model_fingerprint','target_track_id','ray_model'):
            text_id(p[key],key)
        if p['ray_model']!='realsense_sdk_rays':raise ValueError('SDK rays required; legacy differences remain diagnostic')
        for key in ('handeye_sha256','urdf_sha256','source_bag_sha256','rgb_sha256','z16_sha256'):
            sha(p[key],key)
        for key in ('driver_epoch_ns','target_epoch','generation','selection_stamp_ns'):
            exact_int(p[key],key)
        if not p['driver_epoch_ns']<source or not p['selection_stamp_ns']<source:raise ValueError('source predates identity')
        for key in ('rgb_stamp_ns','depth_stamp_ns','board_stamp_ns','quality_stamp_ns'):
            if type(p.get(key)) is not int or p[key]!=source:raise ValueError('not exact RGB/depth/board/quality source')
        for key in ('rgb_received_ns','depth_received_ns','board_received_ns','quality_received_ns'):
            received=exact_int(p[key],key)
            if not 0<=received-source<=350_000_000:raise ValueError('source is stale or future')
        accepted=exact_int(p['accepted_stamp_ns'],'accepted stamp')
        if abs(accepted-source)>100_000_000:raise ValueError('encoder association over 100 ms')
        q=vector(p['accepted_rad'],6,'accepted q')
        samples=p['stationary_samples'];stamps=np.array([exact_int(v['stamp_ns'],'stationary stamp') for v in samples],dtype=np.int64)
        qq=np.array([vector(v['q'],6,'stationary q') for v in samples])
        if (len(stamps)<6 or np.any(np.diff(stamps)<=0) or np.max(np.diff(stamps))>250_000_000
                or np.max(np.ptp(qq,axis=0))>math.radians(.15)):
            raise ValueError('independent stationary window missing')
        mode=p.get('stationary_window_mode')
        if mode=='centered_offline':
            if stamps[0]>source-400_000_000 or stamps[-1]<source+400_000_000:
                raise ValueError('independent centered offline window missing')
        elif mode=='causal_pre_source':
            if (stamps[-1]-stamps[0]<800_000_000 or abs(int(stamps[-1])-source)>100_000_000
                    or stamps[-1]>min(p[k] for k in ('rgb_received_ns','depth_received_ns','board_received_ns','quality_received_ns'))):
                raise ValueError('causal stationary window must span 0.8 s and end near source')
        else:raise ValueError('explicit stationary evidence mode required')
        match=[v for v in samples if v['stamp_ns']==accepted]
        if len(match)!=1 or not np.allclose(match[0]['q'],q,rtol=0,atol=1e-12):raise ValueError('accepted q not in source stationary evidence')
        binding=p['production_target_binding']
        if (binding.get('kind')!='exact_source_mask_track_association'
                or binding.get('target_track_id')!=p['target_track_id']
                or binding.get('target_epoch')!=p['target_epoch']
                or binding.get('source_stamp_ns')!=source
                or binding.get('rgb_sha256')!=p['rgb_sha256']
                or binding.get('z16_sha256')!=p['z16_sha256']):
            raise ValueError('manual ROI cannot substitute for production target binding')
        for key in ('association_artifact_sha256','production_mask_sha256','motor_mask_sha256'):sha(binding[key],key)
        if type(binding.get('mask_overlap_fraction')) not in (int,float) or not .85<=binding['mask_overlap_fraction']<=1:
            raise ValueError('production and motor mask association insufficient')
        plane(p['table_plane_board']);vector(p['anchor_board'],3,'board anchor')
        footprint=np.asarray(p['support_footprint_board'],dtype=float)
        if (footprint.ndim!=2 or footprint.shape[1]!=3 or len(footprint)<3 or not np.isfinite(footprint).all()
                or np.linalg.matrix_rank(footprint-footprint[0],tol=1e-8)<2):raise ValueError('measured motor support footprint missing')
        if np.max(abs(footprint@plane(p['table_plane_board'])[:3]+plane(p['table_plane_board'])[3]))>1e-5:
            raise ValueError('support footprint must lie on this measured table')
        sha(p['board_pose_quality_artifact_sha256'],'detected-corner board quality')
        # Full-board visibility is deliberately not required. Runtime proves
        # detected-corner quality/coverage under its unchanged observer policy.
        if p.get('board_pixels_excluded_from_table') is not True:raise ValueError('board/table surfaces not separated')
        if p.get('target_surface_correspondence_verified') is not True:raise ValueError('partial motor surface correspondence is unverified')
        sha(p['target_surface_correspondence_artifact_sha256'],'motor correspondence')
        return cls(canonical(p),'source_bound_local_observation')


def compare_local_support(a,b):
    first,second=a.data,b.data
    p0,p1=plane(first['table_plane_board']),plane(second['table_plane_board'])
    if p0[:3]@p1[:3]<=0:raise ValueError('normal sign changed')
    anchor=.5*(vector(first['anchor_board'],3,'anchor')+vector(second['anchor_board'],3,'anchor'))
    anchors=[anchor]
    for p in (first,second):anchors.extend(p.get('support_footprint_board',[]))
    points=np.asarray(anchors);delta=p1-p0
    separation=float(np.max(abs(points@delta[:3]+delta[3])))
    angle=math.degrees(math.acos(float(np.clip(p0[:3]@p1[:3],-1,1))))
    bound=(a.level==b.level=='source_bound_local_observation')
    return dict(board_local_plane_max_distance_m=separation,normal_angle_deg=angle,
        support_consistent=bool(separation<=.004 and angle<=4.),
        complete_target_evidence=bound,motion_authorized=False,
        note='No ICP or translation can erase a raw plane contradiction; diagnostic anchors alone do not validate a motor footprint.')


@dataclass(frozen=True)
class RecoveryContext:
    recovery_id:str
    fault:FaultIdentity
    board_reference_id:str
    driver_epoch_ns:int
    camera_model_fingerprint:str
    handeye_sha256:str
    urdf_sha256:str
    deadline_ns:int
    active_generation:int
    active_selection_stamp_ns:int
    active_target:object=None
    evidence:tuple=()

    def __post_init__(self):
        for key in ('recovery_id','board_reference_id','camera_model_fingerprint'):text_id(getattr(self,key),key)
        for key in ('handeye_sha256','urdf_sha256'):sha(getattr(self,key),key)
        exact_int(self.driver_epoch_ns,'driver epoch');exact_int(self.deadline_ns,'deadline')
        exact_int(self.active_generation,'active generation');exact_int(self.active_selection_stamp_ns,'active selection stamp')
        if not isinstance(self.fault,FaultIdentity):raise ValueError('historical failure record required')
        if self.active_target is not None and not isinstance(self.active_target,ActiveTargetIdentity):raise ValueError('typed active target required')

    def bind_active_target(self,identity,bridge):
        """Explicit history -> current target bridge, never an old-plan permit.

        New driver epochs may acquire a new track; an actual production
        acquisition artifact is required, never a manual seed or copied ID.
        """
        if not isinstance(identity,ActiveTargetIdentity):raise ValueError('typed active target required')
        if self.active_target is not None:raise ValueError('active target already immutable')
        if identity.generation!=self.active_generation or identity.selection_stamp_ns!=self.active_selection_stamp_ns:
            raise ValueError('active selection mismatch')
        if (bridge.get('kind')!='historical_failure_to_current_unique_motor'
                or bridge.get('old_fault_record_digest')!=self.fault.fault_digest
                or bridge.get('current_driver_epoch_ns')!=self.driver_epoch_ns
                or bridge.get('target_track_id')!=identity.target_track_id
                or bridge.get('target_epoch')!=identity.target_epoch
                or bridge.get('generation')!=self.active_generation
                or bridge.get('selection_stamp_ns')!=self.active_selection_stamp_ns
                or bridge.get('production_unique_acquisition_verified') is not True
                or bridge.get('manual_roi_only') is not False):
            raise ValueError('current unique production motor acquisition missing')
        sha(bridge['production_acquisition_artifact_sha256'],'production acquisition')
        stamps=bridge['production_acquisition_source_ns']
        if (len(stamps)<3 or any(type(v) is not int or v<=max(self.driver_epoch_ns,self.active_selection_stamp_ns) for v in stamps)
                or any(b<=a for a,b in zip(stamps,stamps[1:]))):
            raise ValueError('three new monotonic acquisition frames required')
        # The hash binds all claimed historical/current fields for later raw
        # artifact verification. It does not assert cross-epoch track equality.
        if digest(bridge)!=identity.bridge_artifact_sha256:raise ValueError('bridge artifact digest mismatch')
        return replace(self,active_target=identity)

    def append(self,evidence):
        if not isinstance(evidence,LocalEvidence):raise ValueError('typed evidence required')
        p=evidence.data
        if self.evidence and p['source_stamp_ns']<=self.evidence[-1].data['source_stamp_ns']:
            raise ValueError('source must increase; no replacing old observations')
        if evidence.level=='source_bound_local_observation':
            if self.active_target is None:raise ValueError('new current production target must be bound explicitly')
            expected=dict(board_reference_id=self.board_reference_id,driver_epoch_ns=self.driver_epoch_ns,
                camera_model_fingerprint=self.camera_model_fingerprint,handeye_sha256=self.handeye_sha256,
                urdf_sha256=self.urdf_sha256,target_track_id=self.active_target.target_track_id,target_epoch=self.active_target.target_epoch,
                generation=self.active_generation,selection_stamp_ns=self.active_selection_stamp_ns)
            if any(p.get(k)!=v for k,v in expected.items()):raise ValueError('local scene or target/model/epoch changed')
            prior=[e for e in self.evidence if e.level=='source_bound_local_observation']
            if prior and not compare_local_support(prior[0],evidence)['support_consistent']:
                raise ValueError('board-local support contradiction; original fault retained')
        return replace(self,evidence=self.evidence+(evidence,))

    def review_proposal(self,proposal,now_ns):
        """Structural eligibility only, never an execution permit.

        Caller must independently verify proof artifacts and the continuous
        path; copied booleans or endpoints cannot establish path clearance.
        """
        reasons=[];p=proposal;exact_int(now_ns,'current time')
        if now_ns>=self.deadline_ns:reasons.append('RECOVERY_DEADLINE_EXPIRED')
        if p.get('recovery_id')!=self.recovery_id or p.get('original_fault_digest')!=self.fault.fault_digest:
            reasons.append('ORIGINAL_FAULT_BINDING_MISSING')
        if p.get('original_plan_digest')!=self.fault.original_plan_digest:reasons.append('ORIGINAL_PLAN_BINDING_MISSING')
        if self.active_target is None:reasons.append('CURRENT_PRODUCTION_TARGET_NOT_ACQUIRED')
        elif p.get('target_bridge_artifact_sha256')!=self.active_target.bridge_artifact_sha256:
            reasons.append('HISTORICAL_FAILURE_CURRENT_TARGET_BRIDGE_MISSING')
        bound=[e for e in self.evidence if e.level=='source_bound_local_observation']
        if not bound:reasons.append('PRODUCTION_TARGET_LOCAL_EVIDENCE_MISSING')
        elif (p.get('latest_evidence_digest')!=bound[-1].evidence_digest
                or not 0<=now_ns-bound[-1].data['source_stamp_ns']<=350_000_000):reasons.append('LATEST_LOCAL_EVIDENCE_STALE_OR_CHANGED')
        # A new plan source later than the fault never removes the obligation.
        if p.get('scope') not in ('observation','contact'):reasons.append('UNKNOWN_PROPOSAL_SCOPE')
        for key in ('fresh_path_sha256','continuous_proof_sha256','controller_start_proof_sha256'):
            if not isinstance(p.get(key),str) or not HEX.fullmatch(p[key]):reasons.append('MISSING_'+key.upper())
        if p.get('scope')=='observation':
            clearance=p.get('certified_minimum_scene_clearance_m')
            if type(clearance) not in (int,float) or not math.isfinite(clearance) or clearance<.040:
                reasons.append('OBSERVATION_CLEARANCE_BELOW_40MM')
        if p.get('scope')=='contact':
            if p.get('new_contact_plan_digest')==self.fault.original_plan_digest or not HEX.fullmatch(str(p.get('new_contact_plan_digest',''))):
                reasons.append('NEW_CONTACT_PLAN_REQUIRED')
            sources={e.data['source_stamp_ns']:e for e in bound}
            training=p.get('response_training_source_ns',[]);heldout=p.get('response_holdout_source_ns',[])
            if (len(set(training))<2 or not heldout or set(training)&set(heldout)
                    or any(v not in sources for v in training+heldout)):
                reasons.append('INDEPENDENT_LOCAL_RESPONSE_HOLDOUT_MISSING')
            elif np.max(np.ptp([sources[k].data['accepted_rad'] for k in training],axis=0))<Q:
                reasons.append('TRAINING_POSES_NOT_DISTINCT')
            for key in ('local_response_domain_artifact_sha256','full_contact_gate_artifact_sha256'):
                if not HEX.fullmatch(str(p.get(key,''))):reasons.append('MISSING_'+key.upper())
            try:
                lower=vector(p['q_domain_lower_rad'],6,'q lower');upper=vector(p['q_domain_upper_rad'],6,'q upper')
                pathlo=vector(p['continuous_path_lower_rad'],6,'path lower');pathhi=vector(p['continuous_path_upper_rad'],6,'path upper')
                current=vector(bound[-1].data['accepted_rad'],6,'current q') if bound else None
                if (np.any(lower>upper) or np.any(pathlo>pathhi) or np.any(pathlo<lower) or np.any(pathhi>upper)
                        or current is None or np.any(current<pathlo) or np.any(current>pathhi)):
                    reasons.append('CONTINUOUS_CONTACT_PATH_OUTSIDE_LOCAL_DOMAIN')
            except (KeyError,ValueError):reasons.append('LOCAL_JOINT_DOMAIN_MISSING')
            if p.get('existing_calibration_policy_preserved') is not True:reasons.append('EXISTING_CALIBRATION_POLICY_NOT_PRESERVED')
        return dict(recovery_id=self.recovery_id,structurally_ready=not reasons,reasons=reasons,
            active_target_track_id=None if self.active_target is None else self.active_target.target_track_id,
            active_generation=self.active_generation,active_driver_epoch_ns=self.driver_epoch_ns,
            original_fault_digest=self.fault.fault_digest,original_fault_retained=True,motion_authorized=False,
            external_proof_verification_required=True)
