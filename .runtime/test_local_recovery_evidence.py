#!/usr/bin/env python3
import copy,importlib.util,unittest
from pathlib import Path
R=Path(__file__).resolve().parents[1]
s=importlib.util.spec_from_file_location('local_recovery',R/'.runtime/local_recovery_evidence.py');L=importlib.util.module_from_spec(s);s.loader.exec_module(L)
S='a'*64
class EvidenceContracts(unittest.TestCase):
    def fault(self):return L.FaultIdentity('old-plan',S,'old-track',5,9,100_000_000,1_000_000_000,'SUPPORT_GEOMETRY_CONTRADICTION','b'*64)
    def unbound(self):return L.RecoveryContext('recovery',self.fault(),'fixed-board',2_000_000_000,'camera',S,S,9_000_000_000,1,2_100_000_000)
    def context(self):
        c=self.unbound();b=dict(kind='historical_failure_to_current_unique_motor',old_fault_record_digest=c.fault.fault_digest,current_driver_epoch_ns=c.driver_epoch_ns,target_track_id='new-track',target_epoch=7,generation=1,selection_stamp_ns=c.active_selection_stamp_ns,production_unique_acquisition_verified=True,manual_roi_only=False,production_acquisition_artifact_sha256=S,production_acquisition_source_ns=[2_200_000_000,2_300_000_000,2_400_000_000])
        i=L.ActiveTargetIdentity('new-track',7,1,c.active_selection_stamp_ns,L.digest(b));return c.bind_active_target(i,b)
    def payload(self,source=3_000_000_000,q=0.,height=0.):
        p=dict(kind='same_source_motor_table_board',plane_label='motor_adjacent_table',source_stamp_ns=source,board_reference_id='fixed-board',camera_model_fingerprint='camera',target_track_id='new-track',target_epoch=7,generation=1,selection_stamp_ns=2_100_000_000,ray_model='realsense_sdk_rays',driver_epoch_ns=2_000_000_000,accepted_stamp_ns=source,accepted_rad=[q]*6,table_plane_board=[0,0,1,-height],anchor_board=[0,0,height],support_footprint_board=[[0,0,height],[.01,0,height],[0,.01,height]],board_pixels_excluded_from_table=True,target_surface_correspondence_verified=True,stationary_window_mode='causal_pre_source')
        for k in ('handeye_sha256','urdf_sha256','source_bag_sha256','rgb_sha256','z16_sha256','board_pose_quality_artifact_sha256','target_surface_correspondence_artifact_sha256'):p[k]=S
        for k in ('rgb','depth','board','quality'):p[k+'_stamp_ns']=source;p[k+'_received_ns']=source+50_000_000
        p['stationary_samples']=[dict(stamp_ns=source+i*100_000_000,q=[q]*6) for i in range(-8,1)]
        p['production_target_binding']=dict(kind='exact_source_mask_track_association',target_track_id='new-track',target_epoch=7,source_stamp_ns=source,rgb_sha256=S,z16_sha256=S,association_artifact_sha256=S,production_mask_sha256=S,motor_mask_sha256=S,mask_overlap_fraction=.96)
        return p
    def evidence(self,**kwargs):return L.LocalEvidence.from_bound_observation(self.payload(**kwargs))
    def proposal(self,c,scope='observation'):
        return dict(recovery_id=c.recovery_id,original_fault_digest=c.fault.fault_digest,original_plan_digest=c.fault.original_plan_digest,target_bridge_artifact_sha256=c.active_target.bridge_artifact_sha256,latest_evidence_digest=c.evidence[-1].evidence_digest,scope=scope,fresh_path_sha256=S,continuous_proof_sha256=S,controller_start_proof_sha256=S,certified_minimum_scene_clearance_m=.040)
    def test_new_epoch_new_track_explicit_bridge_allowed(self):
        c=self.context().append(self.evidence());self.assertNotEqual(c.active_target.target_track_id,c.fault.target_track_id)
        self.assertEqual(c.fault,self.fault())
    def test_manual_roi_never_manufactures_current_target(self):
        report=dict(diagnostic_only=True,motion_authorized=False,production_target_identity_verified=False,session='closed-bag')
        frame=dict(source_stamp_ns=3_000_000_000,table_plane_board=[0,0,1,0],anchor_board=[0,0,0],source_sha256=dict(rgb=S,z16=S))
        e=L.LocalEvidence.from_motor_diagnostic(report,frame,S);c=self.unbound().append(e)
        r=c.review_proposal(dict(scope='observation'),3_100_000_000)
        self.assertIn('CURRENT_PRODUCTION_TARGET_NOT_ACQUIRED',r['reasons']);self.assertFalse(r['motion_authorized'])
        self.assertNotIn('target_track_id',e.data)
    def test_exact_source_required(self):
        p=self.payload();p['depth_stamp_ns']+=1
        with self.assertRaises(ValueError):L.LocalEvidence.from_bound_observation(p)
    def test_stale_or_future_source_rejected(self):
        for dt in (-1,350_000_001):
            p=self.payload();p['rgb_received_ns']=p['source_stamp_ns']+dt
            with self.assertRaises(ValueError):L.LocalEvidence.from_bound_observation(p)
    def test_production_binding_cannot_be_bare_track_string(self):
        p=self.payload();p['production_target_binding']={'target_track_id':'new-track'}
        with self.assertRaises(ValueError):L.LocalEvidence.from_bound_observation(p)
    def test_measured_centered_and_causal_windows_distinguished(self):
        p=self.payload();p['stationary_window_mode']='centered_offline'
        with self.assertRaises(ValueError):L.LocalEvidence.from_bound_observation(p)
        p['stationary_samples']=[dict(stamp_ns=p['source_stamp_ns']+i*100_000_000,q=[0]*6) for i in range(-4,5)]
        self.assertEqual(L.LocalEvidence.from_bound_observation(p).level,'source_bound_local_observation')
    def test_stationary_flag_cannot_replace_samples(self):
        p=self.payload();p['stationary_samples']=[]
        with self.assertRaises((ValueError,IndexError)):L.LocalEvidence.from_bound_observation(p)
    def test_model_epoch_board_target_changes_rejected(self):
        for key,value in [('camera_model_fingerprint','other'),('driver_epoch_ns',2_000_000_001),('board_reference_id','moved')]:
            p=self.payload();p[key]=value
            with self.assertRaises(ValueError):self.context().append(L.LocalEvidence.from_bound_observation(p))
    def test_eleven_mm_base_change_does_not_rewrite_local_table(self):
        a=self.payload();b=self.payload(source=4_000_000_000,q=.02);b['diagnostic_base_height_drift_m']=.011
        r=L.compare_local_support(L.LocalEvidence.from_bound_observation(a),L.LocalEvidence.from_bound_observation(b))
        self.assertTrue(r['support_consistent']);self.assertFalse(r['motion_authorized'])
    def test_real_local_table_motion_rejected_without_icp(self):
        c=self.context().append(self.evidence())
        with self.assertRaises(ValueError):c.append(self.evidence(source=4_000_000_000,height=.011))
    def test_observation_40mm_bound_not_endpoint_or_camera_range(self):
        c=self.context().append(self.evidence());p=self.proposal(c);p['certified_minimum_scene_clearance_m']=.03999
        r=c.review_proposal(p,3_100_000_000);self.assertIn('OBSERVATION_CLEARANCE_BELOW_40MM',r['reasons'])
    def test_newer_plan_never_erases_historical_fault(self):
        c=self.context().append(self.evidence());p=self.proposal(c);p['plan_stamp_ns']=8_000_000_000;p.pop('original_fault_digest')
        r=c.review_proposal(p,3_100_000_000);self.assertIn('ORIGINAL_FAULT_BINDING_MISSING',r['reasons']);self.assertTrue(r['original_fault_retained'])
    def contact(self):
        c=self.context()
        for ns,q in [(3_000_000_000,0),(4_000_000_000,.02),(5_000_000_000,.01)]:c=c.append(self.evidence(source=ns,q=q))
        p=self.proposal(c,'contact');p.update(new_contact_plan_digest='c'*64,response_training_source_ns=[3_000_000_000,4_000_000_000],response_holdout_source_ns=[5_000_000_000],local_response_domain_artifact_sha256=S,full_contact_gate_artifact_sha256=S,q_domain_lower_rad=[0]*6,q_domain_upper_rad=[.02]*6,continuous_path_lower_rad=[.001]*6,continuous_path_upper_rad=[.019]*6,existing_calibration_policy_preserved=True)
        return c,p
    def test_no_hardware_authorization_even_structurally_complete(self):
        c,p=self.contact();r=c.review_proposal(p,5_100_000_000)
        self.assertTrue(r['structurally_ready']);self.assertFalse(r['motion_authorized']);self.assertTrue(r['external_proof_verification_required'])
    def test_contact_path_cannot_leave_local_domain(self):
        c,p=self.contact();p['continuous_path_upper_rad'][4]=.021
        self.assertIn('CONTINUOUS_CONTACT_PATH_OUTSIDE_LOCAL_DOMAIN',c.review_proposal(p,5_100_000_000)['reasons'])
    def test_training_is_not_independent_holdout(self):
        c,p=self.contact();p['response_holdout_source_ns']=[3_000_000_000]
        self.assertIn('INDEPENDENT_LOCAL_RESPONSE_HOLDOUT_MISSING',c.review_proposal(p,5_100_000_000)['reasons'])
    def test_old_plan_never_reused_as_new_contact(self):
        c,p=self.contact();p['new_contact_plan_digest']=c.fault.original_plan_digest
        self.assertIn('NEW_CONTACT_PLAN_REQUIRED',c.review_proposal(p,5_100_000_000)['reasons'])
    def test_immutability_fault_and_evidence_not_replaced(self):
        c=self.context();new=c.append(self.evidence());self.assertEqual(len(c.evidence),0);self.assertEqual(len(new.evidence),1)
        changed=new.evidence[0].data;changed['source_stamp_ns']=0;self.assertNotEqual(new.evidence[0].data['source_stamp_ns'],0)
if __name__=='__main__':unittest.main()
