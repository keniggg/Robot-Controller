"""Live-interest proposal provenance; none of these tests sends ROS commands."""
from copy import deepcopy
from dataclasses import asdict, replace
import hashlib
import json
from pathlib import Path
import sys
from types import SimpleNamespace as NS

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[2]
for part in ('tools/diagnostics','src/alicia_grasp_modes/src','src/alicia_flexible_grasp_supervisor/src'):
    sys.path.insert(0,str(ROOT/part))
import run_rgb_acquisition as runner
from alicia_flexible_grasp.robot.observation_path_guard import SerialUrdfFk
from alicia_flexible_grasp.robot.observation_view_limits import DEFAULT_OBSERVATION_VIEW_PARAMETERS
from alicia_grasp_modes.rgb_observation_set import observation_set_metadata_digest


def fixture():
    description = (ROOT/'src/real-arm/alicia_d_descriptions/urdf/alicia_duo_with_gripper.urdf').read_text()
    fk = SerialUrdfFk(description,runner.NAMES)
    q = np.zeros(6)
    T = np.array([[0.,-1.,0.,0.],[0.,0.,1.,0.],[-1.,0.,0.,.35],[0.,0.,0.,1.]])
    mount = np.linalg.inv(fk(q)) @ T
    stamp = 1_000_000_000
    selection = dict(mode='unknown',strategy='observe_return',generation=1,stamp_ns=100)
    context = dict(selection=selection, epoch_ns=200, tool_camera=mount.tolist(),
        model_sha256=hashlib.sha256(description.encode()).hexdigest(), perception={},
        observation_view_parameters=DEFAULT_OBSERVATION_VIEW_PARAMETERS)
    source_id=runner.digest(context)
    support = runner.SupportPlaneProvenance(stamp,(0.,0.,1.,0.),'test',800,.9,.001,4)
    volume = runner.RGBObservationVolume((310,230,20,20),(480,640),
        ((400.,0.,320.),(0.,400.,240.),(0.,0.,1.)),T,(0.,0.,1.,0.),support,.12,stamp,(.07,.5))
    interest = dict(observation_interest_ready=True,source_stamp_ns=stamp,
        source_context=context, source_id=source_id, generation=1,selection_stamp_ns=100,epoch_ns=200,
        rgb_track=dict(confirmed=True,source_id=source_id,generation=1,
                       source_stamps_ns=[stamp-200_000_000,stamp-100_000_000,stamp]),
        interest_volume=asdict(volume), T_base_camera=T.tolist(),q_source=q.tolist())
    metadata = [dict(bbox_xywh=[310,230,20,20],centroid_xy=[319.5,239.5],area_px=400,
                     image_shape=[480,640],mask_sha256='a'*64)]
    interest.update(rgb_interest_members=metadata,observation_set_sha256=observation_set_metadata_digest(metadata),
        region_mask_sha256='a'*64,interest_volumes=[asdict(volume)])
    kwargs=dict(now_ns=stamp+100_000_000,selection=selection,description=description,
        joint_min=fk.limits[:,0].tolist(),joint_max=fk.limits[:,1].tolist())
    return interest,kwargs


def test_current_bound_interest_yields_bounded_predictions_only():
    interest,kwargs=fixture()
    ranking,candidates=runner.rank_live_interest(interest,**kwargs)
    assert len(candidates) <= 24 and ranking['ranked']
    assert not ranking['motion_authorized'] and not ranking['execution_authority']
    for row in ranking['ranked']:
        assert row['candidate_id'] in candidates
    assert ranking['member_visibility_checked'] and ranking['member_count'] == 1


@pytest.mark.parametrize('change', ['stale','not_ready','selection','context','model',
    'epoch','generation','track_source','track_duplicate','track_last','volume','vertices','q'])
def test_rejects_changed_or_stale_live_evidence(change):
    interest,kwargs=fixture()
    if change=='stale': kwargs['now_ns']+=1_000_000_000
    elif change=='not_ready': interest['observation_interest_ready']=False
    elif change=='selection': kwargs['selection']=dict(kwargs['selection'],generation=2)
    elif change=='context': interest['source_context']['perception']['foreground_max_m']=.2
    elif change=='model': kwargs['description']+=' '
    elif change=='epoch': interest['epoch_ns']+=1
    elif change=='generation': interest['generation']+=1
    elif change=='track_source': interest['rgb_track']['source_id']='wrong'
    elif change=='track_duplicate': interest['rgb_track']['source_stamps_ns'][1]=interest['source_stamp_ns']
    elif change=='track_last': interest['rgb_track']['source_stamps_ns'][-1]-=1
    elif change=='volume': interest['interest_volume']['source_stamp_ns']+=1
    elif change=='vertices':
        interest['interest_volume']['vertices_base_m']=np.zeros((8,3)).tolist()
    elif change=='q': interest['q_source'][1]=.1
    with pytest.raises(ValueError):
        runner.rank_live_interest(interest,**kwargs)


def test_status_diagnostics_explains_no_ready_without_retaining_image_or_context():
    diagnostics = runner.StatusDiagnostics()
    status = dict(state='unavailable', detail='no_center_instance', color_stamp_ns=10,
        observation_interest=dict(observation_interest_ready=False,
            reason='ambiguous_rgb_regions', rgb_candidate_count=2,
            rgb_track=dict(confirmed=False, consistent_frames=0,
                reason='ambiguous_rgb_regions', source_stamps_ns=list(range(1000))),
            source_context={'large_model': 'x'*100000}, mask=np.ones((480,640)),
            rgb_candidates=[dict(mask=np.ones((480,640)))]))
    diagnostics.observe(status)
    status['observation_interest']['reason'] = 'rgb_observation_rate_limited'
    diagnostics.observe(status)
    snapshot = diagnostics.snapshot()
    assert snapshot['status_count'] == 2 and snapshot['ready_status_count'] == 0
    assert snapshot['nonready_reasons'] == dict(ambiguous_rgb_regions=1,
        rgb_observation_rate_limited=1)
    assert snapshot['detector_details'] == {'no_center_instance': 2}
    assert snapshot['last_status']['observation_interest']['rgb_track']['consistent_frames'] == 0
    encoded = json.dumps(snapshot, allow_nan=False)
    assert len(encoded) < 2000
    assert all(key not in encoded for key in ('large_model','rgb_candidates','source_stamps_ns','mask'))
    assert status['observation_interest']['mask'].shape == (480,640)


def test_status_diagnostics_counts_ready_separately_and_freezes_last_scalars():
    diagnostics = runner.StatusDiagnostics()
    status = dict(state='unavailable', detail='initial_instance_rgb_seed_core_too_small',
        observation_interest=dict(observation_interest_ready=True,
            reason='observation_region_found', source_stamp_ns=123,
            rgb_track=dict(confirmed=True, consistent_frames=3)),
        rgb_acquisition_handoff=dict(state='observing', reference_id='ref',
            support_audit={'unexpected_points': [1,2,3]}))
    diagnostics.observe(status)
    snapshot = diagnostics.snapshot()
    status['rgb_acquisition_handoff']['state'] = 'changed'
    status['observation_interest']['rgb_track']['consistent_frames'] = 1
    assert snapshot['ready_status_count'] == 1 and snapshot['nonready_reasons'] == {}
    assert snapshot['last_status']['rgb_acquisition_handoff']['state'] == 'observing'
    assert snapshot['last_status']['observation_interest']['rgb_track']['consistent_frames'] == 3
    snapshot['detector_details'].clear()
    assert diagnostics.snapshot()['detector_details']


def test_status_diagnostics_reason_memory_is_bounded_even_for_varying_failures():
    diagnostics = runner.StatusDiagnostics()
    for index in range(1000):
        diagnostics.observe(dict(detail=str(index)+'x'*10000,
            observation_interest=dict(reason=str(index)+'y'*10000,
                observation_interest_ready=False, source_id='z'*10000)))
    snapshot = diagnostics.snapshot()
    assert len(snapshot['nonready_reasons']) <= diagnostics.MAX_REASON_KEYS+1
    assert len(snapshot['detector_details']) <= diagnostics.MAX_REASON_KEYS+1
    assert sum(snapshot['nonready_reasons'].values()) == 1000
    assert snapshot['nonready_reasons']['<other_reasons>'] == 968
    assert max(map(len,snapshot['nonready_reasons'])) <= diagnostics.MAX_TEXT_LENGTH
    assert len(json.dumps(snapshot,allow_nan=False)) < 20000


def test_status_diagnostics_ignores_malformed_nested_fields_and_nonfinite_numbers():
    diagnostics = runner.StatusDiagnostics()
    diagnostics.observe([])
    diagnostics.observe(dict(state={'large': [1]*1000}, observation_interest=None))
    diagnostics.observe(dict(observation_interest=dict(reason=[],
        observation_interest_ready='true', processing_seconds=float('nan'),
        rgb_track=['unexpected']), rgb_acquisition_handoff=['unexpected']))
    snapshot = diagnostics.snapshot()
    assert snapshot['malformed_status_count'] == 1
    assert snapshot['status_count'] == 2 and snapshot['ready_status_count'] == 0
    assert snapshot['nonready_reasons'] == {'<missing>': 2}
    assert 'processing_seconds' not in snapshot['last_status']['observation_interest']
    json.dumps(snapshot, allow_nan=False)


def two_member_interest():
    interest,kwargs=fixture()
    raw=interest['interest_volume']
    params={f.name:raw[f.name] for f in runner.fields(runner.RGBObservationVolume) if f.init}
    params['support_provenance']=runner.SupportPlaneProvenance(**params['support_provenance'])
    first=runner.RGBObservationVolume(**params)
    second=replace(first,bbox_xywh=(330,235,10,10))
    envelope=replace(first,bbox_xywh=(310,230,30,20))
    interest['rgb_interest_members'].append(dict(bbox_xywh=[330,235,10,10],
        centroid_xy=[334.5,239.5],area_px=100,image_shape=[480,640],mask_sha256='b'*64))
    interest.update(interest_volume=asdict(envelope),interest_volumes=[asdict(first),asdict(second)],
        observation_set_sha256=observation_set_metadata_digest(interest['rgb_interest_members']))
    return interest,kwargs


def test_all_members_pass_visibility_and_keep_their_independent_hashes():
    interest,kwargs=two_member_interest()
    ranking,_=runner.rank_live_interest(interest,**kwargs)
    assert ranking['ranked'] and ranking['member_count']==2
    assert ranking['rgb_interest_members']==interest['rgb_interest_members']
    assert len(ranking['member_visibility_rankings'])==2
    for row in ranking['ranked']:
        assert all(row['candidate_id'] in {r['candidate_id'] for r in result['ranked']}
                   for result in ranking['member_visibility_rankings'])


@pytest.mark.parametrize('change',['legacy','hash','omit','swap','box','stamp','tf','support','envelope','vertices'])
def test_member_set_cannot_be_replaced_by_envelope_or_changed_metadata(change):
    interest,kwargs=two_member_interest()
    if change=='legacy': interest.pop('rgb_interest_members')
    elif change=='hash': interest['rgb_interest_members'][1]['mask_sha256']='c'*64
    elif change=='omit': interest['interest_volumes'].pop()
    elif change=='swap': interest['interest_volumes'].reverse()
    elif change=='box': interest['interest_volumes'][1]['bbox_xywh']=(330,235,11,10)
    elif change=='stamp': interest['interest_volumes'][1]['source_stamp_ns']+=1
    elif change=='tf':
        T=np.array(interest['interest_volumes'][1]['T_base_camera']);T[0,3]+=.001
        interest['interest_volumes'][1]['T_base_camera']=T.tolist()
    elif change=='support': interest['interest_volumes'][1]['support_provenance']['source_record_id']='different'
    elif change=='envelope': interest['interest_volume']=deepcopy(interest['interest_volumes'][0])
    elif change=='vertices': interest['interest_volumes'][1]['vertices_base_m']=np.zeros((8,3)).tolist()
    with pytest.raises(ValueError): runner.rank_live_interest(interest,**kwargs)


def test_member_visibility_rejection_cannot_be_overridden_by_envelope_ranking(monkeypatch):
    interest,kwargs=two_member_interest(); calls=[]
    def ranking(volume,candidates,**options):
        calls.append(volume.bbox_xywh)
        accepted=['first','second'] if len(calls)<3 else ['second']
        return dict(ranked=[dict(candidate_id=k) for k in accepted],rejected=[])
    monkeypatch.setattr(runner,'rank_rgb_observation_candidates',ranking)
    ranked,_=runner.rank_live_interest(interest,**kwargs)
    assert len(calls)==3 and [r['candidate_id'] for r in ranked['ranked']]==['second']


def shifted_member_interest(shift_x):
    interest,kwargs=two_member_interest()
    for member in interest['rgb_interest_members']:
        member['bbox_xywh'][0]+=shift_x
        member['centroid_xy'][0]+=shift_x
    for raw in [interest['interest_volume']]+interest['interest_volumes']:
        values={field.name:raw[field.name] for field in runner.fields(runner.RGBObservationVolume) if field.init}
        values['support_provenance']=runner.SupportPlaneProvenance(**values['support_provenance'])
        x,y,w,h=values['bbox_xywh'];values['bbox_xywh']=(x+shift_x,y,w,h)
        raw.update(asdict(runner.RGBObservationVolume(**values)))
    interest['observation_set_sha256']=observation_set_metadata_digest(interest['rgb_interest_members'])
    return interest,kwargs


def test_reference_members_outside_old_center_window_can_still_rank_in_full_image():
    interest,kwargs=shifted_member_interest(110)
    ranking,_=runner.rank_live_interest(interest,**kwargs)
    assert ranking['ranked'] and ranking['member_count']==2
    assert ranking['roi_margin_px']==36.
    assert not ranking['acquisition_window_checked']
    assert all(not result['acquisition_window_checked'] for result in ranking['member_visibility_rankings'])
    assert all(member['centroid_xy'][0]>420 for member in interest['rgb_interest_members'])
    for row in ranking['ranked']:
        assert row['all_interest_vertices_visible']


@pytest.mark.parametrize('shift_x',[-295,295])
def test_full_image_margin_still_rejects_reference_members_at_image_edges(shift_x):
    interest,kwargs=shifted_member_interest(shift_x)
    with pytest.raises(ValueError,match='OUTSIDE_ROI'):
        runner.rank_live_interest(interest,**kwargs)


def continuation_retry_fixture():
    selection=dict(mode='unknown',strategy='observe_return',generation=1,stamp_ns=100)
    ledger=dict(phase='awaiting_measured_target',request_id='owner',reference_id='ref',source_id='source',
        selection=selection,observation_index=1,attempts=1,max_observations=3,duration_sec=120.,
        request_ids=['owner'],
        created_ns=str(10**9),deadline_ns=str(121*10**9),created_monotonic=10.,deadline_monotonic=130.)
    request=dict(request_id='owner',reference_id='ref',expected_source_id='source',selection=selection,
        observation_index=2,max_observations=3,duration_sec=120.)
    return ledger,request


def test_only_unchanged_unreserved_fresh_capture_timeout_is_retryable():
    ledger,request=continuation_retry_fixture()
    assert runner.unreserved_capture_retry(runner.TRANSIENT_CAPTURE_MESSAGE,ledger,deepcopy(ledger),request,
        now_ns=2*10**9,monotonic_now=11.)
    assert runner.charged_reservations(ledger,deepcopy(ledger))==0
    assert runner.charged_reservations(ledger,dict(ledger,attempts=2))==1


@pytest.mark.parametrize('change',['reserved','executing','planning_rejected','revoked','charged',
    'owner','reference','source','index','deadline_changed','expired_ros','expired_monotonic',
    'rollback','request_owner','request_ref','request_index','request_source','request_selection',
    'request_budget','message_hard_rejection','message_postplan','budget_exhausted'])
def test_capture_retry_cannot_revive_owner_or_budget_after_real_reservation(change):
    before,request=continuation_retry_fixture();after=deepcopy(before)
    now,mono,message=2*10**9,11.,runner.TRANSIENT_CAPTURE_MESSAGE
    if change in ('reserved','executing','planning_rejected','revoked'): after['phase']=change
    elif change=='charged': after['attempts']+=1
    elif change=='owner': after['request_id']='other'
    elif change=='reference': after['reference_id']='other'
    elif change=='source': after['source_id']='other'
    elif change=='index': after['observation_index']+=1
    elif change=='deadline_changed': after['deadline_ns']=str(122*10**9)
    elif change=='expired_ros': now=121*10**9
    elif change=='expired_monotonic': mono=130.
    elif change=='rollback': mono=9.
    elif change=='request_owner': request['request_id']='new'
    elif change=='request_ref': request['reference_id']='new'
    elif change=='request_index': request['observation_index']=1
    elif change=='request_source': request['expected_source_id']='new'
    elif change=='request_selection': request['selection']=dict(request['selection'],generation=2)
    elif change=='request_budget': request['max_observations']=4
    elif change=='message_hard_rejection': message=message.replace('FRESH_READY_RGB_D_SOURCE_REQUIRED','SOURCE_TF_CHANGED')
    elif change=='message_postplan': message=message.replace('INITIAL_CAPTURE_UNAVAILABLE','NO_FRESH_POST_PLAN_SOURCE')
    elif change=='budget_exhausted': before['attempts']=after['attempts']=3
    assert not runner.unreserved_capture_retry(message,before,after,request,now_ns=now,monotonic_now=mono)


def test_unreserved_new_capture_after_rejected_plan_preserves_budget_without_retired_reference():
    before,request=continuation_retry_fixture();before['phase']='planning_rejected'
    request.pop('reference_id');request.pop('observation_index');request['request_id']='pending-new-freeze'
    assert runner.unreserved_capture_retry(runner.TRANSIENT_CAPTURE_MESSAGE,before,deepcopy(before),request,
        now_ns=2*10**9,monotonic_now=11.)
    for changed in (dict(request,reference_id='ref'),dict(request,observation_index=2),
                    dict(request,request_id='owner')):
        assert not runner.unreserved_capture_retry(runner.TRANSIENT_CAPTURE_MESSAGE,before,deepcopy(before),
            changed,now_ns=2*10**9,monotonic_now=11.)


@pytest.mark.parametrize('always_timeout',[False,True])
@pytest.mark.parametrize('first_rejected',[False,True])
@pytest.mark.parametrize('prefix_only',[False,True])
def test_runner_waits_new_ready_after_uncharged_failure_and_preserves_episode(monkeypatch,tmp_path,always_timeout,first_rejected,prefix_only):
    import rospy
    from std_msgs.msg import String
    interest,kwargs=fixture(); selection=kwargs['selection']; clock=[100.]; callback=[]; calls=[]
    params={'/grasp_mode/selection':selection,'/robot_description':kwargs['description'],
            '/robot/joint_min_rad':kwargs['joint_min'],'/robot/joint_max_rad':kwargs['joint_max']}
    def status(*,reference=None,measured=False):
        value=deepcopy(interest);value['source_stamp_ns']=int((clock[0]-.001)*1e9)
        if reference:
            value.update(reference_id=reference,association_sha256='b'*64)
            value['q_source'][0]=.01
        payload=dict(state='ready' if measured else 'unavailable',observation_interest=value,
            rgb_acquisition_handoff=dict(state='measured_ready' if measured else 'observing',
                reference_id=reference,source_stamp_ns=value['source_stamp_ns']))
        callback[0](String(data=json.dumps(payload)))
    def subscribe(*args,**kwargs):
        callback.append(args[2]);status();return NS(unregister=lambda:None)
    def sleep(seconds):
        clock[0]+=max(seconds,.1)
        if calls: status(reference=None if first_rejected else 'ref')
    def rpc(_request):
        request=deepcopy(params['/rgb_acquisition/request']);calls.append(request)
        if len(calls)==1:
            params['/rgb_acquisition/ledger']=dict(phase='planning_rejected' if first_rejected else 'awaiting_measured_target',
                request_id=request['request_id'],request_ids=[request['request_id']],
                reference_id='retired' if first_rejected else 'ref',source_id=request['expected_source_id'],
                selection=selection,observation_index=1,attempts=1,max_observations=3,duration_sec=120.,
                receipt=('actuation_prefix_confirmed_requires_fresh_replan' if prefix_only else
                    'noncontact_observation_completed_requires_new_measured_target_and_rgb_association'),
                requested_target_reached=not prefix_only,
                created_ns=str(int(clock[0]*1e9)),deadline_ns=str(int((clock[0]+120)*1e9)),
                created_monotonic=clock[0],deadline_monotonic=clock[0]+120,settled_ns=str(int(clock[0]*1e9)))
            clock[0]+=.1;status(reference=None if first_rejected else 'ref')
            return NS(success=not first_rejected,message='pre-send proof rejected' if first_rejected else 'settled')
        if always_timeout or len(calls)==2:
            clock[0]+=3.
            # A newer ready published during the failed RPC is still older
            # than its return; the runner must wait for a subsequent source.
            status(reference=None if first_rejected else 'ref')
            return NS(success=False,message=runner.TRANSIENT_CAPTURE_MESSAGE)
        ledger=params['/rgb_acquisition/ledger'];ledger.update(attempts=2,observation_index=1 if first_rejected else 2,
            phase='awaiting_measured_target',request_id=request['request_id'],reference_id='ref',
            receipt='noncontact_observation_completed_requires_new_measured_target_and_rgb_association',
            requested_target_reached=True,
            settled_ns=str(int(clock[0]*1e9)))
        clock[0]+=.1;status(reference='ref',measured=True)
        return NS(success=True,message='settled')
    monkeypatch.setattr(runner,'MOUNT',NS(is_mount=lambda:True));monkeypatch.setattr(runner,'ROOT',tmp_path)
    monkeypatch.setattr(sys,'argv',['runner','--wait-sec','20'])
    monkeypatch.setenv('ROS_LOG_DIR',str(tmp_path))
    monkeypatch.setattr(rospy,'init_node',lambda *a,**k:None)
    monkeypatch.setattr(rospy,'Subscriber',subscribe)
    monkeypatch.setattr(rospy,'get_param',lambda key,default=None:deepcopy(params.get(key,default)))
    monkeypatch.setattr(rospy,'set_param',lambda key,value:params.__setitem__(key,deepcopy(value)))
    monkeypatch.setattr(rospy,'wait_for_service',lambda *a,**k:None)
    monkeypatch.setattr(rospy,'is_shutdown',lambda:False)
    monkeypatch.setattr(rospy,'ServiceProxy',lambda *a,**k:rpc)
    monkeypatch.setattr(rospy.Time,'now',lambda:rospy.Time.from_sec(clock[0]))
    monkeypatch.setattr(runner.time,'monotonic',lambda:clock[0]);monkeypatch.setattr(runner.time,'sleep',sleep)
    def mocked_rank(value,**arguments):
        # The fixture replaces geometric ranking, retaining its successful
        # provenance-bound history update before a camera candidate is issued.
        if value.get('reference_id'):
            ledger=arguments['reference_ledger']
            assert ledger['reference_id']==value['reference_id']
            assert ledger['phase']=='awaiting_measured_target'
            assert ledger['request_id']==arguments['view_history'].receipt['request_id']
        arguments['view_history'].observe_verified(value)
        return (dict(ranked=[dict(candidate_id='same' if calls else 'first')]),
            {name:NS(T_base_tool=np.eye(4)) for name in ('same','first')})
    monkeypatch.setattr(runner,'rank_live_interest',mocked_rank)
    runner.main()
    report=json.loads(next(tmp_path.glob('*/result.json')).read_text())
    assert len({request['request_id'] for request in calls})==(2 if first_rejected else 1)
    if first_rejected:
        assert all('reference_id' not in request and 'observation_index' not in request for request in calls[1:])
        assert len({request['request_id'] for request in calls[1:]})==1
    else:
        assert all(request.get('reference_id')=='ref' and request['observation_index']==2 for request in calls[1:])
    assert report['original_episode_deadline']['deadline_ns']==str(220*10**9)
    assert report['rpc_count']==len(calls)
    if always_timeout:
        assert len(calls)==4 and report['transient_capture_failures']==3
        assert report['charged_attempts']==1 and report['failure']=='transient_capture_failure_budget_exhausted'
        assert not report['measured_target_available']
    else:
        assert len(calls)==3 and report['transient_capture_failures']==1
        assert report['charged_attempts']==2 and report['measured_target_available']
    assert report['requests'][1]['charged_attempts']==0
    assert report['requests'][1]['retry_requires_source_after_ns']
    assert report['actuation_prefix_completed'] == (prefix_only and not first_rejected)
    if prefix_only and not first_rejected:
        assert report['requests'][0]['actuation_prefix_only'] is True
        assert report['requests'][0]['requested_target_reached'] is False
        assert report['requested_observation_target_reached'] == (not always_timeout)
    observations=report['observed_view_history']['observations']
    assert len(observations)==(1 if first_rejected else 2)
    assert observations[0]['origin']=='initial_verified_rgb_frame'
    assert all(row['origin']!='planned_endpoint' for row in observations)


def observed_history_fixture():
    interest,kwargs=fixture()
    history=runner.ObservedViewHistory()
    runner.rank_live_interest(interest,view_history=history,**kwargs)
    ledger=dict(phase='awaiting_measured_target',source_id=interest['source_id'],
        selection=kwargs['selection'],reference_id='original-ref',observation_index=1,
        settled_ns=str(interest['source_stamp_ns']+100_000_000),request_id='owner')
    return history,interest,ledger


def test_success_receipt_without_fresh_source_and_planned_pose_cannot_add_history():
    history,interest,ledger=observed_history_fixture()
    original=history.snapshot()
    ledger['target_matrix']=(np.eye(4)*100).tolist()
    history.bind_success(ledger)
    assert len(history.records)==1 and history.records==original['observations']
    current=deepcopy(interest);current.update(reference_id='original-ref',association_sha256='b'*64)
    with pytest.raises(ValueError,match='fresh_post_settle'):
        history.observe_verified(current)
    current['source_stamp_ns']=int(ledger['settled_ns'])+1
    current['T_base_camera'][0][3]+=.02
    history.observe_verified(current)
    assert len(history.records)==2
    assert history.records[-1]['T_base_camera']==current['T_base_camera']
    assert history.records[-1]['T_base_camera']!=ledger['target_matrix']
    current['T_base_camera'][0][3]+=1.
    assert history.records[-1]['T_base_camera']!=current['T_base_camera']
    current['source_stamp_ns']+=1
    history.observe_verified(current)
    assert len(history.records)==2  # One actual observation per completed move.


@pytest.mark.parametrize('phase',['planning_rejected','reserved','executing','revoked'])
def test_failed_or_unfinished_rpc_never_adds_or_binds_view_history(phase):
    history,interest,ledger=observed_history_fixture();before=history.snapshot()
    ledger['phase']=phase
    with pytest.raises(ValueError,match='success_receipt'):
        history.bind_success(ledger)
    assert history.snapshot()==before


@pytest.mark.parametrize('change',['source','generation','reference'])
def test_changed_source_generation_or_reference_cannot_keep_old_observed_views(change):
    history,interest,ledger=observed_history_fixture();history.bind_success(ledger)
    current=deepcopy(interest);current.update(reference_id='original-ref',association_sha256='b'*64,
        source_stamp_ns=int(ledger['settled_ns'])+1)
    history.observe_verified(current);assert len(history.records)==2
    if change=='source': current['source_id']='different-source'
    elif change=='generation': current['source_context']['selection']['generation']+=1
    else: current['reference_id']='different-ref'
    with pytest.raises(ValueError,match='unbound_reference'):
        history.observe_verified(current)
    assert history.records==[] and history.reference_id is None and history.receipt is None


def test_new_initial_selection_starts_empty_history_and_four_views_is_original_limit():
    history,interest,ledger=observed_history_fixture()
    for index in range(1,4):
        ledger.update(observation_index=index,settled_ns=str(interest['source_stamp_ns']+index*100_000_000))
        history.bind_success(ledger)
        current=deepcopy(interest);current.update(reference_id='original-ref',association_sha256='b'*64,
            source_stamp_ns=int(ledger['settled_ns'])+1)
        current['T_base_camera'][0][3]+=.01*index
        history.observe_verified(current)
    assert len(history.records)==4
    ledger['observation_index']=4
    with pytest.raises(ValueError,match='success_receipt'):history.bind_success(ledger)
    new=deepcopy(interest);new['source_context']['selection']['generation']+=1
    history.observe_verified(new)
    assert len(history.records)==1 and history.reference_id is None
    assert history.snapshot()['measured_information_gain'] is False


def test_rank_live_interest_records_only_after_existing_source_verification():
    interest,kwargs=fixture();history=runner.ObservedViewHistory()
    interest['q_source'][1]+=.1
    with pytest.raises(ValueError,match='source_fk_tf_mismatch'):
        runner.rank_live_interest(interest,view_history=history,**kwargs)
    assert history.records==[]
    interest['q_source'][1]-=.1
    result,_=runner.rank_live_interest(interest,view_history=history,**kwargs)
    assert len(history.records)==1 and result['history_aware_utility']
    assert all(row['history_aware_utility'] for row in result['member_visibility_rankings'])


def frozen_continuation_fixture():
    interest,kwargs=two_member_interest()
    envelope=deepcopy(interest['interest_volume'])
    members=deepcopy(interest['interest_volumes'])
    for value in [envelope]+members:
        value['source_stamp_ns']-=300_000_000
        value['support_provenance']['source_stamp_ns']=value['source_stamp_ns']
    ledger=dict(phase='awaiting_measured_target',source_id=interest['source_id'],
        selection=deepcopy(kwargs['selection']),reference_id='original-ref',request_id='original-owner',
        observation_index=1,settled_ns=str(interest['source_stamp_ns']-100_000_000),
        deadline_ns=str(interest['source_stamp_ns']+120_000_000_000),
        reference_volume_json=json.dumps(envelope),reference_volumes_json=json.dumps(members),
        reference_observation_set_sha256=interest['observation_set_sha256'])
    interest.update(reference_id=ledger['reference_id'],association_sha256='f'*64,
        rgb_search_basis='fixed_reference_volume_projection',
        association_reference_volume=envelope,association_reference_volumes=members,
        association_reference_set_sha256=ledger['reference_observation_set_sha256'])
    kwargs['reference_ledger']=ledger
    return interest,kwargs


def preflight_receipt(ledger, *, q, epoch, capture_stamp):
    ledger.update(phase='preflight_rejected', actuation_prefix_attempted=True,
        actuation_prefix_submission_started=False, contact_authorized=False)
    fields=('request_id','reference_id','source_id','selection','created_ns','deadline_ns',
        'created_monotonic','deadline_monotonic','duration_sec','max_observations','attempts','observation_index')
    proof={key:deepcopy(ledger[key]) for key in fields}
    proof.update(kind='rgb_prefix_preflight_no_submission_v1', submission_started=False,
        original_reference_preserved=True, epoch_ns=epoch, q_source=list(q),
        failed_reservation_request_id=ledger.get('reservation_request_id',ledger['request_id']),
        last_capture_source_stamp_ns=capture_stamp)
    ledger['actuation_prefix_preflight']=proof
    return ledger


def frozen_preflight_fixture():
    interest,kwargs=frozen_continuation_fixture();ledger=kwargs['reference_ledger']
    ledger.update(reference_id='a'*64,request_ids=[ledger['request_id']],attempts=1,
        created_ns='500000000',deadline_ns='120500000000',
        created_monotonic=.5,deadline_monotonic=120.5,duration_sec=120.,max_observations=3)
    ledger.pop('settled_ns')
    interest['reference_id']=ledger['reference_id']
    preflight_receipt(ledger,q=interest['q_source'],epoch=interest['epoch_ns'],capture_stamp=900_000_000)
    return interest,kwargs


def test_preflight_ranks_original_members_and_never_counts_unexecuted_view():
    interest,kwargs=frozen_preflight_fixture();ledger=kwargs['reference_ledger']
    initial=deepcopy(interest);initial.pop('reference_id');initial['source_stamp_ns']=700_000_000
    history=runner.ObservedViewHistory();history.observe_verified(initial)
    original=history.snapshot()['observations']
    history.bind_preflight(ledger)
    ranking,_=runner.rank_live_interest(interest,view_history=history,**kwargs)
    assert ranking['ranked'] and ranking['frozen_reference_visibility_checked']
    assert ranking['member_count']==2 and ranking['reference_id']==ledger['reference_id']
    assert history.snapshot()['observations']==original
    assert history.receipt is None and len(history.records)==1


@pytest.mark.parametrize('change',['missing_proof','submitted','owner','ref','no_association',
    'no_ref','q','epoch','old_frame','member'])
def test_preflight_ranking_cannot_replace_reference_or_measured_source(change):
    interest,kwargs=frozen_preflight_fixture();ledger=kwargs['reference_ledger']
    if change=='missing_proof':ledger.pop('actuation_prefix_preflight')
    elif change=='submitted':ledger['actuation_prefix_submission_started']=True
    elif change=='owner':ledger['request_id']='new-owner'
    elif change=='ref':interest['reference_id']='b'*64
    elif change=='no_association':interest.pop('association_sha256')
    elif change=='no_ref':interest.pop('reference_id')
    elif change=='q':interest['q_source'][0]=.005
    elif change=='epoch':interest['epoch_ns']+=1
    elif change=='old_frame':interest['source_stamp_ns']=900_000_000
    elif change=='member':interest['association_reference_volumes'].pop()
    with pytest.raises(ValueError):runner.rank_live_interest(interest,**kwargs)


def test_preflight_unreserved_timeout_preserves_pending_reservation_identity():
    interest,kwargs=frozen_preflight_fixture();ledger=kwargs['reference_ledger']
    request=dict(request_id=ledger['request_id'],reservation_request_id='new-reservation',
        reference_id=ledger['reference_id'],observation_index=1,
        expected_source_id=ledger['source_id'],selection=ledger['selection'],duration_sec=120.,max_observations=3)
    assert runner.unreserved_capture_retry(runner.TRANSIENT_CAPTURE_MESSAGE,ledger,deepcopy(ledger),request,
        now_ns=1_100_000_000,monotonic_now=1.1)
    for changed in (dict(request,request_id='new-owner'),dict(request,observation_index=2),
            dict(request,reservation_request_id=ledger['request_id']),dict(request,reference_id='b'*64)):
        assert not runner.unreserved_capture_retry(runner.TRANSIENT_CAPTURE_MESSAGE,ledger,deepcopy(ledger),changed,
            now_ns=1_100_000_000,monotonic_now=1.1)


@pytest.mark.parametrize('scenario',['prefix','capture_timeout','all_preflights','legacy',
    'bad_receipt','unknown_submission','changed_reference','no_new_source'])
def test_main_preflight_retry_keeps_owner_reference_budget_and_truthful_completion(monkeypatch,tmp_path,scenario):
    import rospy
    from std_msgs.msg import String
    interest,kwargs=fixture();selection=kwargs['selection'];clock=[100.];callback=[];calls=[]
    reference='a'*64
    params={'/grasp_mode/selection':selection,'/robot_description':kwargs['description'],
        '/robot/joint_min_rad':kwargs['joint_min'],'/robot/joint_max_rad':kwargs['joint_max']}
    def status(measured=False):
        value=deepcopy(interest);value['source_stamp_ns']=int((clock[0]-.001)*1e9)
        if calls:
            value.update(reference_id='b'*64 if scenario=='changed_reference' else reference,
                         association_sha256='c'*64)
            if scenario=='no_new_source':value['source_stamp_ns']=int(99.999*1e9)
        callback[0](String(data=json.dumps(dict(state='ready' if measured else 'unavailable',
            observation_interest=value,rgb_acquisition_handoff=dict(
                state='measured_ready' if measured else 'observing', reference_id=reference,
                source_stamp_ns=value['source_stamp_ns'])))))
    def subscribe(*args,**kwargs):
        callback.append(args[2]);status();return NS(unregister=lambda:None)
    def sleep(seconds):
        clock[0]+=max(seconds,.1);status()
    def rpc(_request):
        request=deepcopy(params['/rgb_acquisition/request']);calls.append(request)
        if len(calls)==1:
            ledger=dict(request_id=request['request_id'],request_ids=[request['request_id']],
                reference_id=reference,source_id=request['expected_source_id'],selection=selection,
                observation_index=1,attempts=1,max_observations=3,duration_sec=120.,
                created_ns=str(int(clock[0]*1e9)),deadline_ns=str(int((clock[0]+120)*1e9)),
                created_monotonic=clock[0],deadline_monotonic=clock[0]+120)
            preflight_receipt(ledger,q=interest['q_source'],epoch=interest['epoch_ns'],capture_stamp=int(99.999*1e9))
            if scenario=='legacy':ledger['phase']='planning_rejected';ledger.pop('actuation_prefix_preflight')
            elif scenario=='bad_receipt':ledger.pop('actuation_prefix_preflight')
            elif scenario=='unknown_submission':ledger['actuation_prefix_submission_started']=True
            params['/rgb_acquisition/ledger']=ledger
            clock[0]+=.1;status()
            return NS(success=False,message='preflight path rejected')
        ledger=params['/rgb_acquisition/ledger']
        if scenario=='capture_timeout' and len(calls)==2:
            clock[0]+=3.;status()
            return NS(success=False,message=runner.TRANSIENT_CAPTURE_MESSAGE)
        ledger['attempts']+=1
        ledger['request_ids'].append(request['reservation_request_id'])
        ledger['reservation_request_id']=request['reservation_request_id']
        if scenario=='all_preflights':
            preflight_receipt(ledger,q=interest['q_source'],epoch=interest['epoch_ns'],
                capture_stamp=int((clock[0]-.001)*1e9))
            clock[0]+=.1;status()
            return NS(success=False,message='next preflight path rejected')
        ledger.update(phase='awaiting_measured_target',settled_ns=str(int(clock[0]*1e9)),
            actuation_prefix_submission_started=True,requested_target_reached=False,
            receipt='actuation_prefix_confirmed_requires_fresh_replan')
        ledger.pop('actuation_prefix_preflight')
        clock[0]+=.1;status(measured=True)
        return NS(success=True,message='prefix settled; original requested target not reached')
    monkeypatch.setattr(runner,'MOUNT',NS(is_mount=lambda:True));monkeypatch.setattr(runner,'ROOT',tmp_path)
    monkeypatch.setattr(sys,'argv',['runner','--wait-sec','20']);monkeypatch.setenv('ROS_LOG_DIR',str(tmp_path))
    monkeypatch.setattr(rospy,'init_node',lambda *a,**k:None);monkeypatch.setattr(rospy,'Subscriber',subscribe)
    monkeypatch.setattr(rospy,'get_param',lambda key,default=None:deepcopy(params.get(key,default)))
    monkeypatch.setattr(rospy,'set_param',lambda key,value:params.__setitem__(key,deepcopy(value)))
    monkeypatch.setattr(rospy,'wait_for_service',lambda *a,**k:None);monkeypatch.setattr(rospy,'is_shutdown',lambda:False)
    monkeypatch.setattr(rospy,'ServiceProxy',lambda *a,**k:rpc)
    monkeypatch.setattr(rospy.Time,'now',lambda:rospy.Time.from_sec(clock[0]))
    monkeypatch.setattr(runner.time,'monotonic',lambda:clock[0]);monkeypatch.setattr(runner.time,'sleep',sleep)
    def rank(value,**arguments):
        if calls:
            assert arguments['reference_ledger']['phase']=='preflight_rejected'
            assert arguments['view_history'].receipt is None
            assert value['reference_id']==reference
        arguments['view_history'].observe_verified(value)
        return (dict(ranked=[dict(candidate_id=name) for name in ('first','second','third')]),
                {name:NS(T_base_tool=np.eye(4)) for name in ('first','second','third')})
    monkeypatch.setattr(runner,'rank_live_interest',rank)
    result=runner.main();report=json.loads(next(tmp_path.glob('*/result.json')).read_text())
    assert len({r['request_id'] for r in calls})==1
    assert all(r['reference_id']==reference and r['observation_index']==1 for r in calls[1:])
    assert report['original_episode_deadline']['deadline_ns']==str(220*10**9)
    assert report['rpc_count']==len(calls)
    assert report['observed_view_history']['observations'][0]['origin']=='initial_verified_rgb_frame'
    assert len(report['observed_view_history']['observations'])==1
    assert not report['requested_observation_target_reached']
    if scenario in ('prefix','capture_timeout'):
        assert result==0 and report['actuation_prefix_completed'] and report['measured_target_available']
        assert report['charged_attempts']==2
        assert calls[1]['reservation_request_id']!=calls[0]['request_id']
        if scenario=='capture_timeout':
            assert len(calls)==3 and calls[1]['reservation_request_id']==calls[2]['reservation_request_id']
            assert report['requests'][1]['charged_attempts']==0
    else:
        assert result==2 and not report['observation_completed'] and not report['actuation_prefix_completed']
        assert len(calls)==(3 if scenario=='all_preflights' else 1)
        assert report['charged_attempts']==len(calls)


def test_continuation_ranks_all_frozen_wire_vertices_and_preserves_them_exactly(monkeypatch):
    interest,kwargs=frozen_continuation_fixture()
    # Even a serialization/reconstruction ulp must not replace the frozen value.
    envelope=interest['association_reference_volume']
    vertices=np.asarray(envelope['vertices_base_m']).copy()
    vertices[0,0]=np.nextafter(vertices[0,0],np.inf)
    envelope['vertices_base_m']=vertices.tolist()
    kwargs['reference_ledger']['reference_volume_json']=json.dumps(envelope)
    expected=deepcopy(interest)
    calls=[];original=runner.rank_rgb_observation_candidates
    def rank(volume,candidates,**arguments):
        calls.append(arguments['frozen_reference_vertices'])
        return original(volume,candidates,**arguments)
    monkeypatch.setattr(runner,'rank_rgb_observation_candidates',rank)
    ranking,_=runner.rank_live_interest(interest,**kwargs)
    assert ranking['ranked'] and ranking['frozen_reference_visibility_checked']
    assert ranking['utility_geometry']=='original_frozen_envelope'
    assert ranking['reference_id']=='original-ref'
    assert len(calls)==3 and all(len(item)==3 for item in calls)
    for item in calls:
        assert np.array_equal(item[0],vertices)
        for value,raw in zip(item[1:],interest['association_reference_volumes']):
            assert np.array_equal(value,raw['vertices_base_m'])
    assert interest==expected


@pytest.mark.parametrize('change',['missing_ledger','missing_reference','source','generation','ref',
    'owner','phase','index','settled','deadline','association','basis','setsha',
    'envelope','member','omit_member','wire_vertex','bad_json','missing_payload'])
def test_frozen_reference_cannot_silently_become_current_frame_geometry(change):
    interest,kwargs=frozen_continuation_fixture();ledger=kwargs['reference_ledger']
    if change=='missing_ledger':kwargs.pop('reference_ledger')
    elif change=='missing_reference':interest.pop('reference_id')
    elif change=='source':ledger['source_id']='different'
    elif change=='generation':ledger['selection']['generation']+=1
    elif change=='ref':ledger['reference_id']='different'
    elif change=='owner':ledger['request_id']=''
    elif change=='phase':ledger['phase']='planning_rejected'
    elif change=='index':ledger['observation_index']=True
    elif change=='settled':ledger['settled_ns']=str(interest['source_stamp_ns'])
    elif change=='deadline':ledger['deadline_ns']=str(kwargs['now_ns'])
    elif change=='association':interest['association_sha256']=''
    elif change=='basis':interest['rgb_search_basis']='current_centre'
    elif change=='setsha':interest['association_reference_set_sha256']='b'*64
    elif change=='envelope':interest['association_reference_volume']=interest['interest_volume']
    elif change=='member':interest['association_reference_volumes']=interest['interest_volumes']
    elif change=='omit_member':interest['association_reference_volumes'].pop()
    elif change=='wire_vertex':
        value=np.array(interest['association_reference_volumes'][0]['vertices_base_m'])
        value[0,0]=np.nextafter(value[0,0],np.inf)
        interest['association_reference_volumes'][0]['vertices_base_m']=value.tolist()
    elif change=='bad_json':ledger['reference_volume_json']='{'
    elif change=='missing_payload':ledger.pop('reference_volumes_json')
    history=runner.ObservedViewHistory()
    with pytest.raises(ValueError):runner.rank_live_interest(interest,view_history=history,**kwargs)
    assert history.records==[]


def test_planning_rejected_reference_is_not_carried_into_new_initial_ranking():
    interest,kwargs=fixture()
    kwargs['reference_ledger']=dict(phase='planning_rejected',source_id=interest['source_id'])
    ranking,_=runner.rank_live_interest(interest,**kwargs)
    assert ranking['ranked'] and not ranking['frozen_reference_visibility_checked']


def test_bound_frozen_vertices_are_copied_not_mutable_caller_aliases():
    interest,kwargs=frozen_continuation_fixture()
    result=runner.bound_frozen_reference_vertices(interest,kwargs['reference_ledger'],now_ns=kwargs['now_ns'])
    value=result[0][0][0]
    interest['association_reference_volume']['vertices_base_m']=np.zeros((8,3)).tolist()
    assert result[0][0][0]==value and type(result[0]) is tuple


@pytest.mark.parametrize('change',[None,'owner','observation_index','settled_ns'])
def test_frozen_reference_and_visited_history_require_same_success_receipt(change):
    interest,kwargs=frozen_continuation_fixture()
    initial=deepcopy(interest)
    for key in list(initial):
        if key=='reference_id' or key.startswith('association_'):
            initial.pop(key)
    initial['source_stamp_ns']-=300_000_000
    initial['rgb_track']['source_stamps_ns']=[x-300_000_000 for x in initial['rgb_track']['source_stamps_ns']]
    initial['interest_volume']=deepcopy(interest['association_reference_volume'])
    initial['interest_volumes']=deepcopy(interest['association_reference_volumes'])
    history=runner.ObservedViewHistory()
    initial_args=dict(kwargs,now_ns=initial['source_stamp_ns']+100_000_000,reference_ledger=None)
    runner.rank_live_interest(initial,view_history=history,**initial_args)
    history.bind_success(kwargs['reference_ledger'])
    ledger=kwargs['reference_ledger']
    if change=='owner':ledger['request_id']='other-nonempty-owner'
    elif change=='observation_index':ledger['observation_index']=2
    elif change=='settled_ns':ledger['settled_ns']=str(int(ledger['settled_ns'])+1)
    if change is None:
        ranking,_=runner.rank_live_interest(interest,view_history=history,**kwargs)
        assert len(history.records)==2 and ranking['observed_view_count']==2
        assert ranking['utility_geometry']=='original_frozen_envelope'
    else:
        with pytest.raises(ValueError,match='frozen_reference_history_owner_changed'):
            runner.rank_live_interest(interest,view_history=history,**kwargs)
        assert len(history.records)==1
