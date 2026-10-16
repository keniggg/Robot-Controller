"""Offline retry-identity guards. No ROS node, selection write or hardware."""
from copy import deepcopy
import importlib.util
from pathlib import Path
import sys

import pytest

WORK = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(WORK/'src/alicia_grasp_modes/src'))
spec = importlib.util.spec_from_file_location('rgb_retry_guard',
    WORK/'tools/diagnostics/prepare_rgb_acquisition_retry.py')
guard = importlib.util.module_from_spec(spec)
spec.loader.exec_module(guard)


def fixture():
    selection = dict(mode='unknown', strategy='observe_return', generation=5, stamp_ns=100)
    ledger = dict(selection=selection, phase='planning_rejected', attempts=3,
        created_ns=1_000_000_000, deadline_ns=121_000_000_000,
        created_monotonic=10., deadline_monotonic=130.,
        settled_ns='10000000000', completed_ns='9800000000',
        reference_id='old-intent-not-authority')
    context = guard.retry_source_context(selection, 200, dict(fx=400.,fy=400.), {}, '<robot/>')
    # User confirmation is 70 seconds old. Only the new image/feedback has .8s TTL.
    receipt = dict(version=1, kind='explicit_user_target_alignment', target_aligned=True,
        confirmation_id='user-message-actual-id',
        confirmation_source=dict(kind='user_message',message_id='actual-id',text='已调整完成并对准目标，可以继续'),
        confirmed_ns=130_000_000_000, selection=selection, epoch_ns=200,
        bound_feedback=dict(stamp_ns=130_010_000_000,positions=[0.]*6,frame_id='sdk_measured'),
        source_context_sha256=guard.digest(context))
    frame = dict(source_context=context, source_id=guard.digest(context),
        source_stamp_ns=199_800_000_000, q_source=[0.]*6,
        accepted_before=[199_750_000_000,[0.]*6], accepted_after=[199_850_000_000,[0.]*6],
        source_rgb_sha256='a'*64, source_depth_sha256='b'*64)
    kwargs = dict(now_ns=200_000_000_000, monotonic_now=209., epoch_ns=200,
        current_feedback=dict(stamp_ns=199_950_000_000,positions=[0.]*6,frame_id='sdk_measured'),
        fresh_frame=frame, confirmation_receipt=receipt)
    return ledger, selection, kwargs


def test_old_confirmation_current_pose_new_rgbd_can_prepare_but_not_authorize_motion():
    ledger, selection, kwargs = fixture()
    original = deepcopy((ledger,selection,kwargs))
    result = guard.validate_retry_preparation(ledger,selection,**kwargs)
    assert result['new_generation_authorized']
    assert result['prior_attempts'] == 3
    assert not result['old_reference_reactivated']
    assert not result['motion_authorized'] and not result['original_budget_modified']
    assert result['confirmation_receipt_sha256'] == guard.digest(kwargs['confirmation_receipt'])
    assert (ledger,selection,kwargs) == original


@pytest.mark.parametrize('phase', ['awaiting_measured_target','planning_rejected','revoked'])
def test_final_failure_cannot_erase_earlier_motion_or_allow_silent_center_reselection(phase):
    ledger,selection,kwargs=fixture()
    ledger['phase']=phase
    ledger.pop('settled_ns');ledger.pop('completed_ns')
    ledger['motion_submitted']=False  # Last action's false flag is not episode history.
    kwargs['confirmation_receipt']=None
    with pytest.raises(ValueError,match='new_explicit_user_alignment_required'):
        guard.validate_retry_preparation(ledger,selection,**kwargs)


def test_unknown_execution_history_is_not_proof_of_no_motion():
    ledger,selection,kwargs=fixture()
    for k in ('attempts','settled_ns','completed_ns'):ledger.pop(k)
    kwargs['confirmation_receipt']=None
    with pytest.raises(ValueError,match='new_explicit_user_alignment_required'):
        guard.validate_retry_preparation(ledger,selection,**kwargs)


def test_zero_reservations_without_submission_preserves_original_expired_reset():
    ledger,selection,kwargs=fixture()
    ledger.update(attempts=0,phase='idle')
    ledger.pop('settled_ns');ledger.pop('completed_ns')
    kwargs['confirmation_receipt']=None
    result=guard.validate_retry_preparation(ledger,selection,**kwargs)
    assert result['policy']=='expired_unsubmitted_episode_only'
    assert result['confirmation_id'] is None and not result['previous_motion_possible']


@pytest.mark.parametrize('key',['submission_source_stamp_ns','submitted_ns','execution_started_ns',
                               'completed_ns','settled_ns'])
def test_submission_markers_require_confirmation_even_if_attempt_counter_zero(key):
    ledger,selection,kwargs=fixture()
    ledger.pop('settled_ns');ledger.pop('completed_ns');ledger['attempts']=0
    ledger[key]='10000000000';kwargs['confirmation_receipt']=None
    with pytest.raises(ValueError,match='new_explicit_user_alignment_required'):
        guard.validate_retry_preparation(ledger,selection,**kwargs)


@pytest.mark.parametrize('key',guard.MOTION_STAMP_FIELDS)
def test_confirmation_must_follow_every_submission_and_execution_timestamp(key):
    ledger,selection,kwargs=fixture()
    ledger[key]=150_000_000_000  # Actual motion evidence AFTER the 130s confirmation.
    with pytest.raises(ValueError,match='alignment_confirmation_predates_current_view'):
        guard.validate_retry_preparation(ledger,selection,**kwargs)


@pytest.mark.parametrize('change',['reserved','executing','ros_deadline','monotonic_deadline',
    'invalid_deadline','wrong_selection','missing_ledger','nan_monotonic'])
def test_old_episode_ownership_and_both_original_deadlines_remain_required(change):
    ledger,selection,kwargs=fixture()
    if change in ('reserved','executing'):ledger['phase']=change
    elif change=='ros_deadline':ledger['deadline_ns']=kwargs['now_ns']+1
    elif change=='monotonic_deadline':ledger['deadline_monotonic']=kwargs['monotonic_now']+1
    elif change=='invalid_deadline':ledger['deadline_monotonic']=True
    elif change=='wrong_selection':ledger['selection']=dict(selection,generation=4)
    elif change=='missing_ledger':ledger={}
    elif change=='nan_monotonic':kwargs['monotonic_now']=float('nan')
    with pytest.raises(ValueError):guard.validate_retry_preparation(ledger,selection,**kwargs)


@pytest.mark.parametrize('change',['reason_only','wrong_origin','missing_text','not_aligned','bad_version',
    'old_selection','old_epoch','old_confirmation','future_confirmation','unmeasured_confirmation',
    'feedback_before_confirmation','context_mismatch','camera_changed','handeye_changed','model_changed',
    'new_epoch','old_frame','stale_frame','stale_feedback','frame_before_confirmation','missing_bracket',
    'wide_bracket','changed_source_q','pose_moved','frame_pose_moved','wrong_feedback_type','missing_raw_sha'])
def test_confirmation_cannot_be_reused_across_invalid_context_or_source(change):
    ledger,selection,kwargs=fixture();r=kwargs['confirmation_receipt'];f=kwargs['fresh_frame']
    if change=='reason_only':kwargs['confirmation_receipt']={'reason':'new tested code'}
    elif change=='wrong_origin':r['confirmation_source']['kind']='automatic_retry'
    elif change=='missing_text':r['confirmation_source'].pop('text')
    elif change=='not_aligned':r['target_aligned']='true'
    elif change=='bad_version':r['version']=True
    elif change=='old_selection':r['selection']=dict(selection,generation=4)
    elif change=='old_epoch':r['epoch_ns']=199
    elif change=='old_confirmation':r['confirmed_ns']=9_000_000_000
    elif change=='future_confirmation':r['confirmed_ns']=kwargs['now_ns']+1
    elif change=='unmeasured_confirmation':r['bound_feedback']['frame_id']='desired'
    elif change=='feedback_before_confirmation':r['bound_feedback']['stamp_ns']=r['confirmed_ns']-1
    elif change=='context_mismatch':r['source_context_sha256']='f'*64
    elif change in ('camera_changed','handeye_changed','model_changed'):
        key=change.split('_')[0]; key='model_sha256' if key=='model' else key
        f['source_context'][key]='f'*64 if key=='model_sha256' else {'changed':1}
        f['source_id']=guard.digest(f['source_context'])
    elif change=='new_epoch':kwargs['epoch_ns']=201
    elif change=='old_frame':f['source_stamp_ns']=100
    elif change=='stale_frame':kwargs['now_ns']+=900_000_000
    elif change=='stale_feedback':kwargs['current_feedback']['stamp_ns']-=2_000_000_000
    elif change=='frame_before_confirmation':f['source_stamp_ns']=r['confirmed_ns']-1
    elif change=='missing_bracket':f.pop('accepted_after')
    elif change=='wide_bracket':f['accepted_before'][0]-=500_000_000
    elif change=='changed_source_q':f['q_source'][0]=.001
    elif change=='pose_moved':kwargs['current_feedback']['positions'][0]=.0041
    elif change=='frame_pose_moved':f['accepted_before'][1][0]=.01
    elif change=='wrong_feedback_type':kwargs['current_feedback']['frame_id']='controller_desired'
    elif change=='missing_raw_sha':f.pop('source_rgb_sha256')
    with pytest.raises(ValueError):guard.validate_retry_preparation(ledger,selection,**kwargs)


def test_confirmation_consumed_by_selection_change_cannot_authorize_another_generation():
    ledger,selection,kwargs=fixture()
    new_selection=dict(selection,generation=6,stamp_ns=200_100_000_000)
    ledger['selection']=new_selection
    with pytest.raises(ValueError,match='alignment_receipt_selection_changed'):
        guard.validate_retry_preparation(ledger,new_selection,**kwargs)


def test_cli_reason_never_substitutes_for_receipt_and_apply_is_explicit():
    args=guard.build_parser().parse_args(['--reason','fixed timeout'])
    assert args.alignment_receipt is None and args.apply is False
    ledger,selection,kwargs=fixture();kwargs['confirmation_receipt']=args.alignment_receipt
    with pytest.raises(ValueError,match='new_explicit_user_alignment_required'):
        guard.validate_retry_preparation(ledger,selection,**kwargs)


def test_telemetry_does_not_invalidate_geometry_binding_but_intrinsics_do():
    selection=fixture()[1];camera={'fx':400.,'temporal_filter_active':False}
    before=guard.retry_source_context(selection,200,camera,{},'<robot/>')
    camera['temporal_filter_active']=True
    assert guard.retry_source_context(selection,200,camera,{},'<robot/>')==before
    camera['fx']=401.
    assert guard.retry_source_context(selection,200,camera,{},'<robot/>')!=before


def test_latest_unbracketed_camera_frame_does_not_hide_previous_exact_pair():
    options=[(1_100_000_000,{'actual':'older'}),(1_133_000_000,{'actual':'latest'})]
    records=[(1_050_000_000,[0.]*6),(1_120_000_000,[0.]*6)]
    stamp,pair,bracket=guard.latest_confirmed_pair(options,records,
        epoch_ns=100,selection_ns=200,now_ns=1_150_000_000)
    assert stamp==1_100_000_000 and pair is options[0][1]
    assert bracket[1][0]==1_050_000_000 and bracket[2][0]==1_120_000_000


@pytest.mark.parametrize('now,records',[(2_000_000_001,[(1_050_000_000,[0.]*6),(1_120_000_000,[0.]*6)]),
    (1_150_000_000,[(1_000_000_000,[0.]*6)]),
    (1_450_000_000,[(1_000_000_000,[0.]*6),(1_400_000_000,[0.]*6)])])
def test_pair_selection_never_extrapolates_retimestamps_or_ignores_ttl(now,records):
    with pytest.raises(ValueError,match='fresh_measured_bracketed_rgbd_unavailable'):
        guard.latest_confirmed_pair([(1_100_000_000,{'actual':'frame'})],records,
            epoch_ns=100,selection_ns=200,now_ns=now)


@pytest.mark.parametrize('rgb,depth,rgb_frame,depth_frame',[
    ((480,640,3),(480,640),'camera_link','camera_link'),
    ((240,320,3),(240,320),'camera_link','camera_link'),
    ((480,640,3),(240,320),'camera_link','camera_link'),
    ((480,640,3),(480,640),'optical','optical'),
    ((480,640,3),(480,640),'camera_link','other'),
    ((480,640),(480,640),'camera_link','camera_link')])
def test_camera_context_matches_actual_source_dimensions_and_frames(rgb,depth,rgb_frame,depth_frame):
    camera=dict(width=640,height=480,frame_id='camera_link')
    valid=(rgb==(480,640,3) and depth==(480,640) and rgb_frame==depth_frame=='camera_link')
    if valid:
        guard.validate_image_geometry(rgb,depth,rgb_frame,depth_frame,camera)
    else:
        with pytest.raises(ValueError,match='rgbd_pair_geometry_mismatch'):
            guard.validate_image_geometry(rgb,depth,rgb_frame,depth_frame,camera)


def test_real_out_and_back_pose_excursion_remains_invalidated():
    receipt=fixture()[2]['confirmation_receipt'];monitor=guard.AlignmentPoseMonitor(receipt)
    ns=receipt['bound_feedback']['stamp_ns']
    monitor.observe(ns+10,[0.]*6)
    monitor.observe(ns+20,[.0041,0.,0.,0.,0.,0.])
    monitor.observe(ns+30,[0.]*6)
    with pytest.raises(ValueError,match='measured_pose_excursion_since_alignment'):
        monitor.require_stationary()


def test_pose_monitor_does_not_reinterpret_preconfirmation_feedback_or_within_bound_jitter():
    receipt=fixture()[2]['confirmation_receipt'];monitor=guard.AlignmentPoseMonitor(receipt)
    ns=receipt['bound_feedback']['stamp_ns']
    monitor.observe(ns-1,[.2]*6)
    monitor.observe(ns+1,[.004]*6)
    monitor.observe(ns+2,[-.004]*6)
    monitor.require_stationary()
    assert not monitor.invalidated


def test_pose_anchor_does_not_alias_mutable_external_receipt():
    receipt=fixture()[2]['confirmation_receipt'];monitor=guard.AlignmentPoseMonitor(receipt)
    receipt['bound_feedback']['positions'][0]=.1
    monitor.observe(receipt['bound_feedback']['stamp_ns']+1,[.1,0.,0.,0.,0.,0.])
    assert monitor.invalidated


def test_pair_cache_retains_frame_until_valid_delayed_sdk_bracket_and_stays_bounded():
    pairs={};base=1_000_000_000
    for i in range(8):
        ns=base+i*33_333_333
        guard.cache_rgbd_part(pairs,ns,'rgb',i)
        guard.cache_rgbd_part(pairs,ns,'depth',i)
    # Latest SDK samples bracket the first frame but not the seven newer images.
    records=[(base-50_000_000,[0.]*6),(base+10_000_000,[0.]*6)]
    stamp,pair,_=guard.latest_confirmed_pair(list(pairs.items()),records,
        epoch_ns=100,selection_ns=200,now_ns=base+300_000_000)
    assert stamp==base and pair=={'rgb':0,'depth':0}
    for i in range(8,80):guard.cache_rgbd_part(pairs,base+i*33_333_333,'rgb',i)
    assert len(pairs)==32 and min(pairs)==base+48*33_333_333


def carry_fixture(monkeypatch):
    import numpy as np
    ledger,previous,old=fixture()
    camera=guard.camera_geometry_snapshot(dict(fx=400.,fy=400.,cx=32.,cy=32.,
        width=64,height=64,frame_id='camera_link',depth_scale=.001,align_depth_to_color=True))
    context=old['fresh_frame']['source_context'];context['camera']=camera
    old['fresh_frame']['source_id']=guard.digest(context)
    old['confirmation_receipt']['source_context_sha256']=guard.digest(context)
    selection=dict(previous,generation=6,stamp_ns=200_100_000_000)
    result=dict(previous_selection=previous,previous_ledger=deepcopy(ledger),new_selection=selection,
        preview_only=False,motion_commands_sent=False,torque_commands_sent=False,
        original_budget_modified=False,response={'success':True,'message':'SWITCHING'},
        alignment_receipt=deepcopy(old['confirmation_receipt']),evidence=deepcopy(old),
        validation=guard.validate_retry_preparation(ledger,previous,**old))
    image=np.full((64,64,3),180,np.uint8);image[20:40,20:40]=20
    depth=np.full((64,64),200,np.uint16)
    mask=np.zeros((64,64),np.uint8);mask[20:40,20:40]=1
    region=guard.RgbObservationRegion(mask,(20,20,20,20),(29.5,29.5),400,1.,.8,100.,.05,.1,('gray',),1.)
    proposals=guard.RgbObservationRegions((region,),0,'observation_region_found',(64,64),1)
    monkeypatch.setattr(guard,'propose_rgb_observation_regions',lambda *args,**kwargs:proposals)
    row=dict(source_context=deepcopy(context),source_id=guard.digest(context),
        source_context_unchanged=True,fresh_for_existing_frame_limit=True,
        source_stamp_ns=180_000_000_000,q_source=[0.]*6,
        accepted_before=[179_950_000_000,[0.]*6],accepted_after=[180_050_000_000,[0.]*6],
        source_rgb_sha256=guard._array_sha(image),source_depth_sha256=guard._array_sha(depth),
        config=dict(guard.Config().__dict__))
    reference=guard.prepare_aligned_rgb_reference(result,row,image,depth)
    fresh=deepcopy(old['fresh_frame']);fresh['source_context']['selection']=selection
    fresh.update(source_id=guard.digest(fresh['source_context']),source_stamp_ns=201_000_000_000,
        accepted_before=[200_950_000_000,[0.]*6],accepted_after=[201_050_000_000,[0.]*6],
        source_rgb_sha256=guard._array_sha(image),source_depth_sha256=guard._array_sha(depth))
    current=dict(now_ns=201_200_000_000,monotonic_now=210.,epoch_ns=200,
        current_feedback=dict(stamp_ns=201_100_000_000,positions=[0.]*6,frame_id='sdk_measured'),
        fresh_frame=fresh,reference=reference,current_bgr=image,current_depth_raw=depth)
    return result,selection,ledger,current,proposals


@pytest.mark.parametrize('stage', ['original_reference', 'fresh_carry'])
def test_retry_rgb_identity_cannot_use_unobserved_padding(monkeypatch, stage):
    import numpy as np
    from alicia_grasp_modes.rgb_observation_region import propose_rgb_observation_regions
    transition, selection, ledger, kwargs, _ = carry_fixture(monkeypatch)
    valid = np.ones((64, 64), bool); valid[20:40, 20:40] = False
    seen = []
    def domain(camera, shape):
        seen.append((camera, shape))
        return valid
    monkeypatch.setattr(guard, 'rectification_rgb_valid_mask', domain)
    monkeypatch.setattr(guard, 'propose_rgb_observation_regions', propose_rgb_observation_regions)
    before = kwargs['current_depth_raw'].copy()
    if stage == 'original_reference':
        with pytest.raises(ValueError, match='original_rgb_intent_not_unique'):
            guard.prepare_aligned_rgb_reference(transition, kwargs['reference']['row'],
                kwargs['current_bgr'], kwargs['current_depth_raw'])
        expected_camera = kwargs['reference']['row']['source_context']['camera']
    else:
        with pytest.raises(ValueError):
            guard.validate_carry_forward_preparation(transition, selection, ledger, **kwargs)
        expected_camera = kwargs['fresh_frame']['source_context']['camera']
    assert seen == [(expected_camera, (64, 64))]
    np.testing.assert_array_equal(kwargs['current_depth_raw'], before)


def test_retry_checks_each_frame_against_its_own_bound_geometry_activation(monkeypatch):
    transition, selection, ledger, kwargs, _ = carry_fixture(monkeypatch)
    calls = []
    def checked(camera, stamp):
        calls.append((camera, stamp))
    monkeypatch.setattr(guard, 'require_rectified_source_after_activation', checked)
    guard.prepare_aligned_rgb_reference(transition, kwargs['reference']['row'],
        kwargs['current_bgr'], kwargs['current_depth_raw'])
    guard.validate_carry_forward_preparation(transition, selection, ledger, **kwargs)
    reference_call = (kwargs['reference']['row']['source_context']['camera'],
                      kwargs['reference']['row']['source_stamp_ns'])
    fresh_call = (kwargs['fresh_frame']['source_context']['camera'], kwargs['fresh_frame']['source_stamp_ns'])
    original_frame = transition['evidence']['fresh_frame']
    original_transition_call = (original_frame['source_context']['camera'], original_frame['source_stamp_ns'])
    assert reference_call in calls and fresh_call in calls
    # The immutable original transition is independently revalidated too; it
    # retains its own frame timestamp rather than borrowing the current one.
    assert all(call in (reference_call, fresh_call, original_transition_call) for call in calls)


def test_real_confirmation_can_continue_only_through_verified_unreserved_generation(monkeypatch):
    result,selection,ledger,kwargs,_=carry_fixture(monkeypatch)
    before=deepcopy(result)
    report=guard.validate_carry_forward_preparation(result,selection,ledger,**kwargs)
    assert report['confirmation_id']==result['alignment_receipt']['confirmation_id']
    assert report['policy']=='confirmed_unreserved_generation_same_rgb_intent'
    assert report['new_generation_authorized'] and report['prior_gateway_generation_unreserved']
    assert not report['motion_authorized'] and not report['old_reference_reactivated']
    assert not report['original_budget_modified'] and not report['historical_continuous_stationarity_claimed']
    assert report['appearance_association']['mask_iou']==1.
    assert result==before  # Original gen5 receipt and source IDs stay untouched.


@pytest.mark.parametrize('change',['preview','failed_response','failure','missing_receipt',
    'receipt_text','validation','old_evidence','wrong_generation','generation_skipped',
    'new_stamp','ledger_deleted','reservation','submitted','unchanged_phase_new_attempt'])
def test_carry_chain_rejects_missing_or_changed_confirmation_transition_and_any_reservation(monkeypatch,change):
    result,selection,ledger,kwargs,_=carry_fixture(monkeypatch)
    if change=='preview':result['preview_only']=True
    elif change=='failed_response':result['response']['success']=False
    elif change=='failure':result['failure']='failed'
    elif change=='missing_receipt':result.pop('alignment_receipt')
    elif change=='receipt_text':result['alignment_receipt']['confirmation_source']['text']='invented'
    elif change=='validation':result['validation']['new_generation_authorized']=False
    elif change=='old_evidence':result['evidence']['fresh_frame']['source_id']='f'*64
    elif change=='wrong_generation':selection=dict(selection,generation=7)
    elif change=='generation_skipped':selection=dict(selection,generation=7);result['new_selection']=selection
    elif change=='new_stamp':result['new_selection']['stamp_ns']=result['evidence']['now_ns']
    elif change=='ledger_deleted':ledger={}
    elif change=='reservation':ledger.update(selection=selection,attempts=1,phase='reserved')
    elif change=='submitted':ledger['submitted_ns']=201_100_000_000
    elif change=='unchanged_phase_new_attempt':ledger['attempts']+=1
    with pytest.raises((ValueError,TypeError)):
        guard.validate_carry_forward_preparation(result,selection,ledger,**kwargs)


@pytest.mark.parametrize('change',['camera','handeye','model','epoch','selection','sourceid',
    'stale','stale_feedback','pose','bracket_pose','false_q','rgb_hash','depth_hash','source_before_new_selection'])
def test_carry_never_reuses_old_geometry_feedback_or_freshness(monkeypatch,change):
    result,selection,ledger,kwargs,_=carry_fixture(monkeypatch);f=kwargs['fresh_frame']
    if change in ('camera','handeye','model'):
        key='model_sha256' if change=='model' else change
        f['source_context'][key]='f'*64 if change=='model' else {'changed':True}
        f['source_id']=guard.digest(f['source_context'])
    elif change=='epoch':kwargs['epoch_ns']=201
    elif change=='selection':f['source_context']['selection']=result['previous_selection']
    elif change=='sourceid':f['source_id']='f'*64
    elif change=='stale':kwargs['now_ns']+=900_000_000
    elif change=='stale_feedback':kwargs['current_feedback']['stamp_ns']-=2_000_000_000
    elif change=='pose':kwargs['current_feedback']['positions'][0]=.0041
    elif change=='bracket_pose':f['accepted_before'][1][0]=.01
    elif change=='false_q':f['q_source'][0]=.001
    elif change=='rgb_hash':f['source_rgb_sha256']='e'*64
    elif change=='depth_hash':f['source_depth_sha256']='d'*64
    elif change=='source_before_new_selection':f['source_stamp_ns']=200_000_000_000
    with pytest.raises(ValueError):guard.validate_carry_forward_preparation(result,selection,ledger,**kwargs)


@pytest.mark.parametrize('change',['old_confirmation','after_transition','unbracketed','q_moved',
    'changed_context','changed_hash','changed_raw','not_fresh_at_capture','ambiguous_original'])
def test_reference_must_be_actual_original_aligned_source(monkeypatch,change):
    result,selection,ledger,kwargs,proposals=carry_fixture(monkeypatch)
    row=deepcopy(kwargs['reference']['row']);image=kwargs['current_bgr'].copy();depth=kwargs['current_depth_raw']
    if change=='old_confirmation':row['source_stamp_ns']=129_000_000_000
    elif change=='after_transition':row['source_stamp_ns']=200_200_000_000
    elif change=='unbracketed':row['accepted_after'][0]=179_950_000_000
    elif change=='q_moved':row['accepted_before'][1][0]=.02
    elif change=='changed_context':row['source_context']['handeye']={'different':True};row['source_id']=guard.digest(row['source_context'])
    elif change=='changed_hash':row['source_id']='f'*64
    elif change=='changed_raw':image[0,0,0]+=1
    elif change=='not_fresh_at_capture':row['fresh_for_existing_frame_limit']=False
    elif change=='ambiguous_original':
        from dataclasses import replace
        monkeypatch.setattr(guard,'propose_rgb_observation_regions',lambda *a,**kw:replace(proposals,selected_index=None,reason='ambiguous_rgb_regions'))
    with pytest.raises(ValueError):guard.prepare_aligned_rgb_reference(result,row,image,depth)


def test_matching_old_object_elsewhere_cannot_authorize_new_center_selection(monkeypatch):
    from dataclasses import replace
    import numpy as np
    _,_,_,kwargs,proposals=carry_fixture(monkeypatch)
    mask=np.zeros((64,64),np.uint8);mask[40:55,40:55]=1
    stranger=replace(proposals.selected,mask=mask,bbox_xywh=(40,40,15,15))
    changed=replace(proposals,candidates=(stranger,proposals.selected),selected_index=0,total_candidate_count=2)
    with pytest.raises(ValueError,match='production_center_selection_differs'):
        guard.associate_aligned_rgb(kwargs['reference'],kwargs['current_bgr'],changed)


def test_ambiguous_nested_or_duplicate_match_cannot_choose_first(monkeypatch):
    from dataclasses import replace
    _,_,_,kwargs,proposals=carry_fixture(monkeypatch)
    changed=replace(proposals,candidates=(proposals.selected,proposals.selected),total_candidate_count=2)
    with pytest.raises(ValueError,match='same_station_rgb_intent_ambiguous'):
        guard.associate_aligned_rgb(kwargs['reference'],kwargs['current_bgr'],changed)


def test_same_location_replacement_with_different_appearance_is_rejected(monkeypatch):
    _,_,_,kwargs,proposals=carry_fixture(monkeypatch)
    image=kwargs['current_bgr'].copy();image[20:40,20:40]=[240,20,20]
    with pytest.raises(ValueError,match='no_unique_same_station_rgb_intent'):
        guard.associate_aligned_rgb(kwargs['reference'],image,proposals)


def test_carry_cli_requires_explicit_original_result_and_exact_source_not_reason():
    args=guard.build_parser().parse_args(['--reason','fixed','--carry-forward-result','result.json',
        '--aligned-observation-record','record.json','--aligned-source-stamp-ns','123'])
    assert args.aligned_source_stamp_ns==123 and not args.apply and args.alignment_receipt is None


@pytest.mark.parametrize('change',['none','camera','handeye','model','epoch','selection',
    'ledger','old_feedback','stale_source','future_feedback','moved_feedback','attention_changed'])
def test_after_appearance_final_recheck_covers_full_context_feedback_and_source_age(monkeypatch,change):
    result,selection,ledger,kwargs,_=carry_fixture(monkeypatch)
    evidence=dict(kwargs,confirmation_receipt=result['alignment_receipt'])
    context=deepcopy(kwargs['fresh_frame']['source_context']);current=deepcopy(kwargs['current_feedback'])
    now=kwargs['now_ns'];final_ledger=deepcopy(ledger)
    if change in ('camera','handeye','model'):
        key='model_sha256' if change=='model' else change
        context[key]='f'*64 if change=='model' else {'new':1}
    elif change=='epoch':context['epoch_ns']+=1
    elif change=='selection':context['selection']=dict(selection,generation=7)
    elif change=='ledger':final_ledger['attempts']+=1
    elif change=='old_feedback':current['stamp_ns']-=1
    elif change=='stale_source':now+=900_000_000
    elif change=='future_feedback':current['stamp_ns']=now+1
    elif change=='moved_feedback':current['positions'][0]=.0041
    params=dict(current_context=context,current_ledger=final_ledger,expected_ledger=ledger,
        current_feedback=current,now_ns=now,expected_radius_fraction=.125,
        current_radius_fraction=.2 if change=='attention_changed' else .125)
    if change=='none':guard.validate_final_carry_snapshot(evidence,**params)
    else:
        with pytest.raises(ValueError):guard.validate_final_carry_snapshot(evidence,**params)


def test_archived_reference_loader_checks_record_and_original_raw_file(monkeypatch,tmp_path):
    import numpy as np
    import json,hashlib
    result,_,_,kwargs,_=carry_fixture(monkeypatch);row=deepcopy(kwargs['reference']['row'])
    source=tmp_path/'original.npz'
    np.savez(source,color_bgr=kwargs['current_bgr'],depth_raw=kwargs['current_depth_raw'],
        source_stamp_ns=np.int64(row['source_stamp_ns']),q_source=np.zeros(6))
    row['source_file']=str(source);row['source_file_sha256']=hashlib.sha256(source.read_bytes()).hexdigest()
    record=tmp_path/'record.json';record.write_text(json.dumps({'rows':[row]}))
    loaded=guard.load_aligned_rgb_reference(result,record,row['source_stamp_ns'])
    assert loaded['rgb_sha256']==row['source_rgb_sha256']
    source.write_bytes(source.read_bytes()+b'changed')
    with pytest.raises(ValueError,match='aligned_source_file_hash_changed'):
        guard.load_aligned_rgb_reference(result,record,row['source_stamp_ns'])


def chained_carry_fixture(monkeypatch):
    original, selection, ledger, kwargs, _ = carry_fixture(monkeypatch)
    evidence = {k:deepcopy(v) for k,v in kwargs.items()
                if k not in ('reference','current_bgr','current_depth_raw')}
    evidence['confirmation_receipt'] = deepcopy(original['alignment_receipt'])
    validation = guard.validate_carry_forward_preparation(original, selection, ledger, **kwargs)
    new = dict(selection, generation=selection['generation']+1, stamp_ns=201_300_000_000)
    intervening = dict(previous_selection=selection, previous_ledger=deepcopy(ledger),
        new_selection=new, preview_only=False, response={'success':True},
        motion_commands_sent=False, torque_commands_sent=False, original_budget_modified=False,
        alignment_receipt=deepcopy(original['alignment_receipt']), evidence=evidence, validation=validation)
    current = deepcopy(kwargs)
    current['now_ns'] += 2_000_000_000
    current['current_feedback']['stamp_ns'] += 2_000_000_000
    frame=current['fresh_frame'];frame['source_context']['selection']=new
    frame['source_id']=guard.digest(frame['source_context']);frame['source_stamp_ns']+=2_000_000_000
    for key in ('accepted_before','accepted_after'):frame[key][0]+=2_000_000_000
    return original, [intervening], new, ledger, current


def test_bounded_software_reloads_always_match_original_rgb_and_preserve_budget(monkeypatch):
    original, chain, selection, ledger, kwargs=chained_carry_fixture(monkeypatch)
    before=deepcopy((original,chain,ledger,kwargs['reference']['row']))
    report=guard.validate_carry_forward_preparation(original,selection,ledger,
        unreserved_transitions=chain,**kwargs)
    assert report['unreserved_transition_sha256']==[guard.digest(chain[0])]
    assert report['appearance_association']['reference_source_stamp_ns']==180_000_000_000
    assert report['confirmation_id']==original['alignment_receipt']['confirmation_id']
    assert not report['motion_authorized'] and not report['old_reference_reactivated']
    assert not report['original_budget_modified']
    assert (original,chain,ledger,kwargs['reference']['row'])==before
    # Older deployed first-carry receipts do not contain the optional empty list.
    chain[0]['validation'].pop('unreserved_transition_sha256')
    guard.validate_carry_forward_preparation(original,selection,ledger,
        unreserved_transitions=chain,**kwargs)


@pytest.mark.parametrize('change',['omitted','repeated','budget','preview','failed','motion',
    'torque','budget_reset','reservation','receipt','skip','reorder_stamp','policy',
    'ancestor_digest','chain_digest','old_context','source_id','frame_stamp','q','feedback',
    'image_hash','changed_current_rgb','changed_current_q','stale_current'])
def test_reload_chain_cannot_reset_attempts_reanchor_or_hide_changed_sources(monkeypatch,change):
    import numpy as np
    original, chain, selection, ledger, kwargs=chained_carry_fixture(monkeypatch)
    r=chain[0];v=r['validation'];e=r['evidence'];f=e['fresh_frame']
    if change=='omitted':chain=[]
    elif change=='repeated':chain.append(deepcopy(r))
    elif change=='budget':chain=chain*(guard.MAX_UNRESERVED_TRANSITIONS+1)
    elif change=='preview':r['preview_only']=True
    elif change=='failed':r['response']['success']=False
    elif change=='motion':r['motion_commands_sent']=True
    elif change=='torque':r['torque_commands_sent']=True
    elif change=='budget_reset':r['original_budget_modified']=True
    elif change=='reservation':ledger['attempts']+=1
    elif change=='receipt':r['alignment_receipt']['confirmed_ns']+=1
    elif change=='skip':r['new_selection']['generation']+=1
    elif change=='reorder_stamp':r['new_selection']['stamp_ns']=e['now_ns']
    elif change=='policy':v['policy']='invented'
    elif change=='ancestor_digest':v['transition_result_sha256']='a'*64
    elif change=='chain_digest':v['unreserved_transition_sha256']=['a'*64]
    elif change=='old_context':f['source_context']['handeye']={'changed':True}
    elif change=='source_id':f['source_id']='a'*64
    elif change=='frame_stamp':f['source_stamp_ns']-=3_000_000_000
    elif change=='q':f['q_source'][0]=.005
    elif change=='feedback':e['current_feedback']['positions'][0]=.005
    elif change=='image_hash':v['source_rgb_sha256']='a'*64
    elif change=='changed_current_rgb':kwargs['current_bgr'][:]=0
    elif change=='changed_current_q':kwargs['current_feedback']['positions'][0]=.005
    elif change=='stale_current':kwargs['now_ns']+=2_000_000_000
    with pytest.raises((ValueError,TypeError)):
        guard.validate_carry_forward_preparation(original,selection,ledger,
            unreserved_transitions=chain,**kwargs)


def maintenance_fixture(monkeypatch, *, chained=False):
    if chained:
        original, chain, selection, ledger, kwargs = chained_carry_fixture(monkeypatch)
    else:
        original, selection, ledger, kwargs, _ = carry_fixture(monkeypatch)
        chain = []
    source_ns = kwargs['fresh_frame']['source_stamp_ns']
    old_context = deepcopy(kwargs['fresh_frame']['source_context'])
    new_context = deepcopy(old_context)
    new_context['epoch_ns'] = source_ns - 500_000_000
    old_nodes = {'/alicia_d_driver_node': {'uri': 'http://robot:10001/', 'pid': 10},
                 '/alicia_hardware_interface': {'uri': 'http://robot:10002/', 'pid': 11}}
    new_nodes = deepcopy(old_nodes)
    new_nodes['/alicia_d_driver_node'] = {'uri': 'http://robot:10003/', 'pid': 12}
    def feedback(ns):
        return dict(stamp_ns=ns, frame_id='sdk_measured', positions=[0.] * 6)
    receipt = dict(version=1, kind='authorized_driver_only_reload', status='completed',
        authorization_reference='external actual maintenance authorization event',
        driver_node='/alicia_d_driver_node', started_ns=source_ns-700_000_000,
        completed_ns=source_ns-350_000_000,
        binding=dict(original_transition_sha256=guard.digest(original),
            confirmation_receipt_sha256=guard.digest(original['alignment_receipt']),
            unreserved_transition_sha256=[guard.digest(item) for item in chain],
            aligned_source_stamp_ns=kwargs['reference']['row']['source_stamp_ns'],
            aligned_rgb_sha256=kwargs['reference']['rgb_sha256']),
        before=dict(nodes=old_nodes, epoch_ns=old_context['epoch_ns'], selection=deepcopy(selection),
            ledger=deepcopy(ledger), feedback=feedback(source_ns-750_000_000),
            source_context=old_context, driver_binary_sha256='a'*64),
        after=dict(nodes=new_nodes, epoch_ns=new_context['epoch_ns'], selection=deepcopy(selection),
            ledger=deepcopy(ledger), feedback=feedback(source_ns-400_000_000),
            source_context=new_context, driver_binary_sha256='b'*64),
        policy_after=dict(software_temperature_monitoring_enabled=False, temperature_poll_rate_hz=0.),
        actuation=dict(joint_motion_commands_sent=False, torque_disable_commands_sent=False,
            torque_enable_policy='positive_enable_only', positive_enable_observed=True,
            evidence_files=[dict(path='/external/actual_reload.log', sha256='c'*64)]))
    kwargs['epoch_ns'] = new_context['epoch_ns']
    kwargs['fresh_frame']['source_context'] = deepcopy(new_context)
    kwargs['fresh_frame']['source_id'] = guard.digest(new_context)
    kwargs['maintenance_receipt'] = receipt
    kwargs['unreserved_transitions'] = chain
    return original, selection, ledger, kwargs


@pytest.mark.parametrize('chained', [False, True])
def test_explicit_maintenance_epoch_edge_keeps_real_confirmation_and_actual_positive_enable(monkeypatch, chained):
    original, selection, ledger, kwargs = maintenance_fixture(monkeypatch, chained=chained)
    before = deepcopy((original, ledger, kwargs['maintenance_receipt'], kwargs['reference']['row']))
    report = guard.validate_carry_forward_preparation(original, selection, ledger, **kwargs)
    assert report['policy'] == 'confirmed_unreserved_generation_same_rgb_intent_after_driver_reload'
    edge = report['maintenance_epoch_transition']
    assert edge['before_epoch_ns'] != edge['after_epoch_ns'] == kwargs['epoch_ns']
    assert edge['before_source_id'] != edge['after_source_id'] == kwargs['fresh_frame']['source_id']
    assert edge['positive_enable_observed'] is True
    assert edge['torque_enable_policy'] == 'positive_enable_only'
    assert edge['recursive_epoch_carry_authorized'] is False and edge['maximum_epoch_edges'] == 1
    assert report['confirmation_id'] == original['alignment_receipt']['confirmation_id']
    assert report['appearance_association']['reference_source_stamp_ns'] == kwargs['reference']['row']['source_stamp_ns']
    assert report['new_generation_authorized']
    for key in ('motion_authorized', 'old_reference_reactivated', 'original_budget_modified',
                'historical_continuous_stationarity_claimed'):
        assert report[key] is False
    assert (original, ledger, kwargs['maintenance_receipt'], kwargs['reference']['row']) == before


def test_missing_maintenance_never_relaxes_existing_epoch_or_confirmation_guards(monkeypatch):
    original, selection, ledger, kwargs = maintenance_fixture(monkeypatch)
    kwargs.pop('maintenance_receipt')
    with pytest.raises(ValueError, match='carry_forward_geometry_or_epoch_changed'):
        guard.validate_carry_forward_preparation(original, selection, ledger, **kwargs)
    old_ledger, old_selection, old_kwargs = fixture()
    old_kwargs['epoch_ns'] += 1
    with pytest.raises(ValueError, match='alignment_receipt_epoch_changed'):
        guard.validate_retry_preparation(old_ledger, old_selection, **old_kwargs)


@pytest.mark.parametrize('change', ['unfinished', 'authorization', 'kind', 'driver_scope',
    'future_completion', 'bad_lifetime', 'old_epoch', 'same_epoch', 'epoch_before_reload',
    'before_feedback_stale', 'after_feedback_stale', 'before_feedback_future',
    'after_feedback_old_epoch', 'before_pose', 'after_pose', 'ledger', 'reservation',
    'selection', 'camera', 'handeye', 'model', 'node_missing', 'non_driver_replaced',
    'driver_not_replaced', 'binary_not_replaced', 'binary_invalid', 'temperature_enabled',
    'temperature_rate', 'joint_motion', 'torque_disable', 'enable_policy', 'enable_claim',
    'evidence_missing', 'evidence_sha', 'original_binding', 'confirmation_binding',
    'chain_binding', 'original_image_binding', 'current_epoch', 'current_context',
    'current_stale', 'current_feedback_stale', 'current_pose', 'cross_epoch_bracket',
    'wide_bracket', 'precompletion_rgbd'])
def test_maintenance_epoch_exception_requires_exact_scope_and_new_measured_source(monkeypatch, change):
    original, selection, ledger, kwargs = maintenance_fixture(monkeypatch)
    receipt = kwargs['maintenance_receipt']
    before, after, frame = receipt['before'], receipt['after'], kwargs['fresh_frame']
    if change == 'unfinished': receipt['status'] = 'running'
    elif change == 'authorization': receipt['authorization_reference'] = ''
    elif change == 'kind': receipt['kind'] = 'explicit_user_target_alignment'
    elif change == 'driver_scope': receipt['driver_node'] = '/other'
    elif change == 'future_completion': receipt['completed_ns'] = kwargs['now_ns']+1
    elif change == 'bad_lifetime': receipt['completed_ns'] = receipt['started_ns']
    elif change == 'old_epoch': before['epoch_ns'] += 1
    elif change == 'same_epoch': after['epoch_ns'] = before['epoch_ns']
    elif change == 'epoch_before_reload': after['epoch_ns'] = receipt['started_ns']-1
    elif change == 'before_feedback_stale': before['feedback']['stamp_ns'] -= 1_000_000_000
    elif change == 'after_feedback_stale': receipt['completed_ns'] += 900_000_000; kwargs['now_ns'] += 900_000_000
    elif change == 'before_feedback_future': before['feedback']['stamp_ns'] = receipt['started_ns']+1
    elif change == 'after_feedback_old_epoch': after['feedback']['stamp_ns'] = after['epoch_ns']-1
    elif change == 'before_pose': before['feedback']['positions'][0] = .005
    elif change == 'after_pose': after['feedback']['positions'][0] = .005
    elif change == 'ledger': after['ledger']['attempts'] += 1
    elif change == 'reservation': ledger['attempts'] += 1
    elif change == 'selection': after['selection']['generation'] += 1
    elif change == 'camera': after['source_context']['camera']['fx'] += 1
    elif change == 'handeye': after['source_context']['handeye']['changed'] = True
    elif change == 'model': after['source_context']['model_sha256'] = 'd'*64
    elif change == 'node_missing': after['nodes'].pop('/alicia_hardware_interface')
    elif change == 'non_driver_replaced': after['nodes']['/alicia_hardware_interface']['pid'] += 1
    elif change == 'driver_not_replaced': after['nodes']['/alicia_d_driver_node']['pid'] = before['nodes']['/alicia_d_driver_node']['pid']
    elif change == 'binary_not_replaced': after['driver_binary_sha256'] = before['driver_binary_sha256']
    elif change == 'binary_invalid': after['driver_binary_sha256'] = 'invalid'
    elif change == 'temperature_enabled': receipt['policy_after']['software_temperature_monitoring_enabled'] = True
    elif change == 'temperature_rate': receipt['policy_after']['temperature_poll_rate_hz'] = 1.
    elif change == 'joint_motion': receipt['actuation']['joint_motion_commands_sent'] = True
    elif change == 'torque_disable': receipt['actuation']['torque_disable_commands_sent'] = True
    elif change == 'enable_policy': receipt['actuation']['torque_enable_policy'] = 'arbitrary'
    elif change == 'enable_claim': receipt['actuation']['positive_enable_observed'] = 'false'
    elif change == 'evidence_missing': receipt['actuation']['evidence_files'] = []
    elif change == 'evidence_sha': receipt['actuation']['evidence_files'][0]['sha256'] = 'bad'
    elif change == 'original_binding': receipt['binding']['original_transition_sha256'] = 'd'*64
    elif change == 'confirmation_binding': receipt['binding']['confirmation_receipt_sha256'] = 'd'*64
    elif change == 'chain_binding': receipt['binding']['unreserved_transition_sha256'] = ['d'*64]
    elif change == 'original_image_binding': receipt['binding']['aligned_rgb_sha256'] = 'd'*64
    elif change == 'current_epoch': kwargs['epoch_ns'] += 1
    elif change == 'current_context': frame['source_context']['camera']['fx'] += 1
    elif change == 'current_stale': kwargs['now_ns'] += 900_000_000
    elif change == 'current_feedback_stale': kwargs['current_feedback']['stamp_ns'] -= 1_000_000_000
    elif change == 'current_pose': kwargs['current_feedback']['positions'][0] = .005
    elif change == 'cross_epoch_bracket': frame['accepted_before'][0] = after['epoch_ns']-1
    elif change == 'wide_bracket': frame['accepted_before'][0] = frame['accepted_after'][0]-200_000_001
    else: frame['source_stamp_ns'] = receipt['completed_ns']
    with pytest.raises((ValueError, TypeError)):
        guard.validate_carry_forward_preparation(original, selection, ledger, **kwargs)


def test_maintenance_still_matches_current_pixels_directly_to_original_image(monkeypatch):
    original, selection, ledger, kwargs = maintenance_fixture(monkeypatch)
    kwargs['current_bgr'][:] = 255
    kwargs['fresh_frame']['source_rgb_sha256'] = guard._array_sha(kwargs['current_bgr'])
    with pytest.raises(ValueError, match='no_unique_same_station_rgb_intent'):
        guard.validate_carry_forward_preparation(original, selection, ledger, **kwargs)


def test_maintenance_result_cannot_be_reused_as_a_recursive_epoch_or_selection_ancestor(monkeypatch):
    original, selection, ledger, kwargs = maintenance_fixture(monkeypatch)
    validation = guard.validate_carry_forward_preparation(original, selection, ledger, **kwargs)
    new = dict(selection, generation=selection['generation']+1, stamp_ns=kwargs['now_ns']+1)
    evidence = {key: deepcopy(value) for key, value in kwargs.items()
                if key not in ('reference', 'current_bgr', 'current_depth_raw', 'unreserved_transitions')}
    evidence['confirmation_receipt'] = deepcopy(original['alignment_receipt'])
    result = dict(previous_selection=selection, previous_ledger=deepcopy(ledger), new_selection=new,
        preview_only=False, response={'success': True}, motion_commands_sent=False,
        torque_commands_sent=False, original_budget_modified=False,
        alignment_receipt=deepcopy(original['alignment_receipt']), evidence=evidence,
        validation=validation, maintenance_receipt=deepcopy(kwargs['maintenance_receipt']))
    with pytest.raises((ValueError, TypeError)):
        guard.validate_unreserved_transitions(original, [result], new, ledger)
    with pytest.raises((ValueError, TypeError)):
        guard.validate_alignment_transition(result, new, ledger)
    with pytest.raises((ValueError, TypeError)):
        guard.validate_carry_forward_preparation(original, new, ledger, **kwargs)


def test_maintenance_file_loader_verifies_evidence_bytes_and_cli_scope(tmp_path, monkeypatch):
    import hashlib
    import json
    original, selection, ledger, kwargs = maintenance_fixture(monkeypatch)
    receipt = kwargs['maintenance_receipt']
    evidence = tmp_path/'driver_reload.log'
    evidence.write_text('actual external maintenance evidence\n')
    receipt['actuation']['evidence_files'] = [dict(path=str(evidence),
        sha256=hashlib.sha256(evidence.read_bytes()).hexdigest())]
    path = tmp_path/'maintenance.json'
    path.write_text(json.dumps(receipt))
    assert guard.load_maintenance_receipt(path) == receipt
    evidence.write_text('changed\n')
    with pytest.raises(ValueError, match='maintenance_evidence_file_changed'):
        guard.load_maintenance_receipt(path)
    args = guard.build_parser().parse_args(['--reason', 'authorized reload', '--maintenance-receipt', str(path)])
    assert args.maintenance_receipt == path and args.apply is False
    with pytest.raises(ValueError, match='carry_forward_requires_original_result'):
        guard.main(['--reason', 'authorized reload', '--maintenance-receipt', str(path)])


def test_original_flat_deployment_binding_is_verified_without_rewriting_receipt(monkeypatch):
    original, selection, ledger, kwargs = maintenance_fixture(monkeypatch)
    receipt = kwargs['maintenance_receipt']
    binding = receipt.pop('binding')
    receipt.update(binding)
    snapshot = deepcopy(receipt)
    report = guard.validate_carry_forward_preparation(original, selection, ledger, **kwargs)
    assert report['maintenance_receipt_sha256'] == guard.digest(snapshot)
    assert receipt == snapshot
    receipt['binding'] = dict(binding, original_transition_sha256='f'*64)
    with pytest.raises(ValueError, match='maintenance_alignment_binding_changed'):
        guard.validate_carry_forward_preparation(original, selection, ledger, **kwargs)


@pytest.mark.parametrize('change', ['epoch', 'ledger', 'pose', 'stale'])
def test_maintenance_does_not_bypass_last_check_before_selection_call(monkeypatch, change):
    original, selection, ledger, kwargs = maintenance_fixture(monkeypatch)
    guard.validate_carry_forward_preparation(original, selection, ledger, **kwargs)
    evidence = {key: deepcopy(value) for key, value in kwargs.items()
        if key not in ('reference', 'current_bgr', 'current_depth_raw', 'unreserved_transitions')}
    evidence['confirmation_receipt'] = original['alignment_receipt']
    final = dict(current_context=deepcopy(evidence['fresh_frame']['source_context']),
        current_ledger=deepcopy(ledger), expected_ledger=ledger,
        current_feedback=deepcopy(evidence['current_feedback']), now_ns=kwargs['now_ns'],
        current_radius_fraction=kwargs['reference']['search_policy'].radius_fraction,
        expected_radius_fraction=kwargs['reference']['search_policy'].radius_fraction)
    guard.validate_final_carry_snapshot(evidence, **final)
    if change == 'epoch': final['current_context']['epoch_ns'] += 1
    elif change == 'ledger': final['current_ledger']['attempts'] += 1
    elif change == 'pose': final['current_feedback']['positions'][0] = .005
    else: final['now_ns'] += 900_000_000
    with pytest.raises(ValueError):
        guard.validate_final_carry_snapshot(evidence, **final)


@pytest.mark.parametrize('change',['evidence_epoch','validation_epoch','validation_q',
    'validation_confirmation','validation_feedback','history_claim','reference_stamp',
    'reference_rgb','reference_mask'])
def test_intermediate_redundant_fields_cannot_contradict_original_source(monkeypatch,change):
    original,chain,selection,ledger,kwargs=chained_carry_fixture(monkeypatch)
    e=chain[0]['evidence'];v=chain[0]['validation'];a=v['appearance_association']
    if change=='evidence_epoch':e['epoch_ns']+=1
    elif change=='validation_epoch':v['epoch_ns']+=1
    elif change=='validation_q':v['q_source'][0]=1.
    elif change=='validation_confirmation':v['confirmation_id']='different-user-message'
    elif change=='validation_feedback':v['current_feedback_stamp_ns']=1
    elif change=='history_claim':v['historical_continuous_stationarity_claimed']=True
    elif change=='reference_stamp':a['reference_source_stamp_ns']+=1
    elif change=='reference_rgb':a['reference_rgb_sha256']='a'*64
    elif change=='reference_mask':a['reference_mask_sha256']='a'*64
    with pytest.raises(ValueError):
        guard.validate_carry_forward_preparation(original,selection,ledger,
            unreserved_transitions=chain,**kwargs)


@pytest.mark.parametrize('change',['none','root_bytes','source_bytes','source_stamp','root_path',
    'prior_file_link','missing_file','duplicate'])
def test_intervening_files_bind_original_bytes_and_order(tmp_path,change):
    import hashlib,json
    root=tmp_path/'root.json';rgb=tmp_path/'rgb.json';hop=tmp_path/'hop.json'
    root.write_text('{}');rgb.write_text('{}')
    record=dict(carry_forward_result_file=str(root),
        carry_forward_result_file_sha256=hashlib.sha256(root.read_bytes()).hexdigest(),
        aligned_observation_record=str(rgb),aligned_record_file_sha256=hashlib.sha256(rgb.read_bytes()).hexdigest(),
        aligned_source_stamp_ns=10)
    paths=[hop]
    if change=='root_bytes':root.write_text('{ }')
    elif change=='source_bytes':rgb.write_text('{ }')
    elif change=='source_stamp':record['aligned_source_stamp_ns']=11
    elif change=='root_path':record['carry_forward_result_file']=str(rgb)
    elif change=='prior_file_link':record['unreserved_transition_files']=[dict(path='unverified',sha256='a'*64)]
    elif change=='missing_file':paths=[tmp_path/'missing']
    elif change=='duplicate':paths=[hop,hop]
    hop.write_text(json.dumps(record))
    if change=='none':assert guard.load_unreserved_transitions(paths,root,rgb,10)==[record]
    else:
        with pytest.raises((ValueError,OSError)):
            guard.load_unreserved_transitions(paths,root,rgb,10)
