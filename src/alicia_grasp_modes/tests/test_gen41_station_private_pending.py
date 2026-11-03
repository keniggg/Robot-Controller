"""Real station callbacks and current admission; synthetic pixels, no motion."""
from copy import deepcopy
from types import SimpleNamespace as NS
import itertools
import json
import pytest
from test_observation_measurement_remote import context
from test_station_compute_wait_remote import station, row_at, clock, feedback
from test_station_compute_work_handoff import submitted, rgbd
from test_gen11_contact_remote_receipt import contact_plan
from test_unknown_pending_identity import deliver

ACTUAL_PARENT=1792224864862662315
ACTUAL_GAP=1792224865330518007
ACTUAL_PENDING_RECEIVED=1792224865624195575
ACTUAL_READY_SOURCE=1792224865779169321
ACTUAL_READY_RECEIVED=1792224866145929098
ACTUAL_BEFORE=1792224865305582708
ACTUAL_AFTER=1792224865585654750

def gap_row(c,parent):
    source=parent['source_ns']+(ACTUAL_GAP-ACTUAL_PARENT)
    before=parent['source_ns']+(ACTUAL_BEFORE-ACTUAL_PARENT)
    after=parent['source_ns']+(ACTUAL_AFTER-ACTUAL_PARENT)
    now=parent['source_ns']+(ACTUAL_PENDING_RECEIVED-ACTUAL_PARENT)
    feedback(c,before);feedback(c,after);clock(c,now)
    row=row_at(c,source)
    row['object'].detected=False
    row['mask'].data=bytes(len(row['mask'].data))
    evidence=dict(accepted_before=[before,[0.]*6],accepted_after=[after,[0.]*6],
        bracket_qualified=False,maximum_span_ns=200_000_000,span_ns=ACTUAL_AFTER-ACTUAL_BEFORE)
    gap=dict(discarded_source_stamp_ns=source,discarded_source_stamps_ns=[source],
        feedback_gap_evidence=deepcopy(evidence),discarded_source_evidence=[dict(source_stamp_ns=source,feedback_gap_evidence=deepcopy(evidence))],
        source_qualified=False,identity_verified=False,metric_geometry_authority=False,motion_authorized=False,reference_refreshed=False)
    row['status'].update(state='processing_deadline_pending',detail='station_accepted_feedback_bracket_pending_revalidation',
        target_lost=False,first_loss=None,discarded_source_stamp_ns=source,
        station_compute_wait=deepcopy(parent),station_compute_recovery_chain=[deepcopy(parent)],station_feedback_gap=gap)
    return row

def fresh_after_gap(c,parent,gap):
    source=parent['source_ns']+(ACTUAL_READY_SOURCE-ACTUAL_PARENT)
    now=parent['source_ns']+(ACTUAL_READY_RECEIVED-ACTUAL_PARENT)
    feedback(c,source-10_000_000);feedback(c,source+10_000_000);clock(c,now)
    row=row_at(c,source,recovery=parent)
    row['status']['station_feedback_gap_recovery']=deepcopy(gap['status']['station_feedback_gap'])
    rgbd(c,row)
    return row


@pytest.fixture
def work(station):
    c=station;n=c.node
    _t,row,ticket=submitted(c)
    return c,ticket,contact_plan(n,ticket),deepcopy(row['status']['station_compute_wait']),n._capture_geometry_generation()

def publish_blocked(n,plan,generation):
    accepted,code=n._publish_plan_pair_if_current(plan,generation)
    assert accepted is False and code=='TARGET_PROCESSING_PENDING'
    assert not bool(getattr(n.latest_object_geometry,'valid',False))
    assert not bool(getattr(n.latest_preview_rich_plan,'valid',False))

@pytest.mark.parametrize('order',list(itertools.permutations(('status','object','mask'))))
def test_typed_gap_recovery_and_clock_retains_private_only(work,order):
    c,ticket,plan,parent,generation=work;n=c.node
    gap=gap_row(c,parent)
    for kind in order:
        deliver(n,gap,(kind,));deliver(n,gap,(kind,))
        publish_blocked(n,plan,generation)
    assert n._station_compute_work_certificate(ticket)
    assert n._capture_geometry_generation()==generation
    recovered=fresh_after_gap(c,parent,gap);deliver(n,recovered)
    assert not n._station_compute_pending and n._station_compute_work_certificate(ticket)
    assert n._capture_geometry_generation()==generation
    receipt=n._station_contact_computation_receipt(plan,ticket)
    assert receipt['motion_authorized'] is False and receipt['metric_geometry_authority'] is False
    assert receipt['plan_source_ns']==ticket.payload[0].stamp_ns
    assert receipt['current']['contract']['source_ns']==recovered['status']['source_stamp_ns']
    current=deepcopy(n._station_compute_reference['contract'])
    clock(c,current['geometry_deadline_ns']+86252000)
    idle=deepcopy(recovered);idle['status'].update(state='processing_deadline_pending',
        detail='processing_deadline_pending',target_lost=False,first_loss=None,
        discarded_source_stamp_ns=None)
    idle['status'].pop('station_feedback_gap_recovery',None)
    deliver(n,idle,('status',))
    publish_blocked(n,plan,generation)
    assert n._station_compute_work_certificate(ticket) and n._capture_geometry_generation()==generation
    assert n._station_compute_reference['contract']['phase_deadline_ns']==parent['phase_deadline_ns']

@pytest.mark.parametrize('ready_order',list(itertools.permutations(('status','object','mask'))))
@pytest.mark.parametrize('gap_order',list(itertools.permutations(('status','object','mask'))))
def test_zero_fragment_complete_recovery_ready_does_not_wait_for_old_empty_topics(work,ready_order,gap_order):
    c,ticket,plan,parent,generation=work;n=c.node
    gap=gap_row(c,parent);recovered=fresh_after_gap(c,parent,gap)
    for kind in ready_order:
        deliver(n,recovered,(kind,));deliver(n,recovered,(kind,))
    assert not n._station_compute_pending
    assert n._station_compute_work_certificate(ticket) and n._capture_geometry_generation()==generation
    assert n._station_compute_reference['contract']['source_ns']==recovered['status']['source_stamp_ns']
    for kind in gap_order:
        deliver(n,gap,(kind,));deliver(n,gap,(kind,))
    assert not n._station_compute_pending and n._station_compute_work_certificate(ticket)
    assert n._capture_geometry_generation()==generation
    assert n._station_compute_reference['contract']['source_ns']==recovered['status']['source_stamp_ns']

def test_partial_gap_then_newer_ready_does_not_clear_private_work(work):
    c,ticket,plan,parent,generation=work;n=c.node
    gap=gap_row(c,parent)
    deliver(n,gap,('object','mask'))
    recovered=fresh_after_gap(c,parent,gap);deliver(n,recovered)
    assert not n._station_compute_pending
    assert n._station_compute_work_certificate(ticket)
    deliver(n,gap,('status',))
    assert not n._station_compute_pending and n._station_compute_work_certificate(ticket)

@pytest.mark.parametrize('order',list(itertools.permutations(('status','object','mask'))))
def test_post_recovery_current_ready_duplicates_do_not_requarantine(work,order):
    c,ticket,plan,parent,generation=work;n=c.node
    gap=gap_row(c,parent);deliver(n,gap)
    recovered=fresh_after_gap(c,parent,gap);deliver(n,recovered)
    assert not n._station_compute_pending and n._station_compute_work_certificate(ticket)
    invalidations=len(n.geometry_pub.messages)
    for kind in order:
        deliver(n,recovered,(kind,));deliver(n,recovered,(kind,))
        assert not n._station_compute_pending
        assert n._station_compute_work_certificate(ticket)
        assert n._capture_geometry_generation()==generation
    assert len(n.geometry_pub.messages)==invalidations

@pytest.mark.parametrize('first',['object','mask'])
def test_first_unqualified_fragment_is_truthful_nonmetric_partial_witness(work,first):
    c,ticket,plan,parent,generation=work;n=c.node
    gap=gap_row(c,parent);deliver(n,gap,(first,))
    publish_blocked(n,plan,generation)
    payload=json.loads(n.geometry_pub.messages[-1].failure_reason.partition(':')[2])
    assert payload['clock_only'] is False and 'station_pending_notice' not in payload
    marker=payload['station_pending_input']
    assert marker['input_kind']==first and marker['source_ns']==gap[first].header.stamp.to_nsec()
    assert marker['parent_contract']==parent
    assert all(marker[k] is False for k in ('source_qualified','identity_verified',
        'metric_geometry_authority','motion_authorized','reference_refreshed'))
    assert n._station_compute_work_certificate(ticket)

@pytest.mark.parametrize('fault',[
    'missing_receipt','changed_parent','support_reference_changed','owner_selection',
    'control_epoch','manual_takeover','measured_pose_changed','true_target_loss',
    'original_input_mask_changed','discarded_source_became_true','identity_deadline'])
def test_real_contradictions_destroy_original_private_work(work,fault):
    c,ticket,plan,parent,generation=work;n=c.node
    gap=gap_row(c,parent);valid_gap=deepcopy(gap)
    if fault=='missing_receipt':gap['status'].pop('station_feedback_gap')
    elif fault=='changed_parent':gap['status']['station_compute_wait']['binding']['config_sha256']='e'*64
    elif fault=='support_reference_changed':gap['status']['reference_support_plane_base'][3]+=.01
    deliver(n,gap)
    if fault=='owner_selection':
        from alicia_grasp_modes.selection import SELECTION_PARAM
        c.params[SELECTION_PARAM]={};n._station_compute_poll()
    elif fault=='control_epoch':n._measurement_remote_epoch_ns+=1;n._station_compute_poll()
    elif fault=='manual_takeover':c.params['/gui/joint_direct_mode']=True;n._station_compute_poll()
    elif fault=='measured_pose_changed':
        feedback(c,n._ros_source_clock_ns(),q=[.02,0,0,0,0,0]);n._station_compute_poll()
    elif fault=='true_target_loss':
        bad=deepcopy(gap);bad['status'].update(target_lost=True,first_loss={'reason':'explicit real loss'})
        deliver(n,bad,('status',))
    elif fault=='original_input_mask_changed':
        input_ns=ticket.payload[0].sample_stamp_ns[0]
        bad=deepcopy(n._station_compute_masks[input_ns][2]);pixels=n.bridge.imgmsg_to_cv2(bad,desired_encoding='mono8').copy()
        pixels.flat[0]^=255
        changed=n.bridge.cv2_to_imgmsg(pixels,encoding='mono8');changed.header=deepcopy(bad.header)
        n.mask_cb(changed)
    elif fault=='discarded_source_became_true':
        bad=deepcopy(gap);bad['object'].detected=True;deliver(n,bad,('object',))
    elif fault=='identity_deadline':clock(c,parent['identity_deadline_ns']+1000000);n._station_compute_poll()
    assert not n._station_compute_work_certificate(ticket)
    assert not bool(getattr(n.latest_object_geometry,'valid',False))
    assert not bool(getattr(n.latest_preview_rich_plan,'valid',False))
    if fault in ('missing_receipt','changed_parent','support_reference_changed','true_target_loss'):
        recovered=fresh_after_gap(c,parent,valid_gap);deliver(n,recovered)
        assert not n._station_compute_work_certificate(ticket)



def _idle_feedback_echo_then_fresh_after_parent_expiry(c, gap, parent):
    """Component source envelopes; the separate chain test calls the producer."""
    n = c.node
    # Only the failure fragments arrive first. The original gap status is
    # deliberately delayed until after recovery and the old parent's expiry.
    deliver(n, gap, ('object', 'mask'))
    when = parent['geometry_deadline_ns'] + 86_251_926
    clock(c, when)
    echo = row_at(c, parent['source_ns'])
    echo['status'].update(state='processing_deadline_pending',
        detail='processing_deadline_pending', target_lost=False, first_loss=None,
        station_compute_wait=deepcopy(parent),
        station_compute_recovery_chain=[deepcopy(parent)],
        discarded_source_stamp_ns=gap['status']['source_stamp_ns'],
        station_feedback_gap=deepcopy(gap['status']['station_feedback_gap']))
    deliver(n, echo, ('status',))
    recovered = fresh_after_gap(c, parent, gap); deliver(n, recovered)
    assert not n._station_compute_pending
    current = deepcopy(n._station_compute_reference['contract'])
    ns = parent['identity_deadline_ns'] + 100_000_000
    assert ns + 50_000_000 < current['identity_deadline_ns']
    # Every real synthetic SDK callback follows its own advancing clock.
    clock(c, ns - 10_000_000); clock(c, ns + 10_000_000)
    clock(c, ns + 50_000_000)
    newest = row_at(c, ns, recovery=current); rgbd(c, newest); deliver(n, newest)
    assert not n._station_compute_pending
    assert n._station_compute_reference['contract']['source_ns'] == ns
    assert n._ros_source_clock_ns() > parent['identity_deadline_ns']
    return echo, newest


def test_idle_feedback_echo_and_late_original_cause_cannot_overwrite_fresh_owner(work):
    c, ticket, plan, parent, generation = work; n = c.node
    gap = gap_row(c, parent)
    echo, newest = _idle_feedback_echo_then_fresh_after_parent_expiry(c, gap, parent)
    frozen = deepcopy(n._station_compute_reference)
    public_count = len(n.geometry_pub.messages)
    assert n._station_compute_work_certificate(ticket)
    for old in (echo, gap, echo, gap):
        deliver(n, old, ('status',))
        assert not n._station_compute_pending and not n._station_compute_retired
        assert n._station_compute_reference == frozen
        assert n._station_compute_work_certificate(ticket)
        assert n._capture_geometry_generation() == generation
    assert len(n.geometry_pub.messages) == public_count
    assert n._station_compute_reference['contract']['source_ns'] == newest['status']['source_stamp_ns']
    assert ticket.payload[0].stamp_ns == plan.header.stamp.to_nsec()


def test_late_feedback_cause_never_hides_mixed_recovery_or_changed_endpoints(work):
    from alicia_grasp_modes.station_compute_reacquisition import validate_station_pending_notice
    from alicia_grasp_modes.unknown_pending_identity import reference_binding
    c, ticket, plan, parent, generation = work; n = c.node
    gap = gap_row(c, parent)
    # A pending rejected bracket cannot simultaneously claim support or READY
    # recovery, even when its immutable original bracket was already known.
    for field in ('station_support_gap_recovery', 'station_feedback_gap_recovery'):
        mixed = deepcopy(gap['status']); mixed[field] = deepcopy(mixed['station_feedback_gap'])
        with pytest.raises(ValueError):
            validate_station_pending_notice(mixed, contract=parent,
                now_ns=n._ros_source_clock_ns(),
                status_reference_binding=reference_binding(mixed, n._mode_selection))
    _idle_feedback_echo_then_fresh_after_parent_expiry(c, gap, parent)
    bad = deepcopy(gap)
    bad['status']['station_feedback_gap']['feedback_gap_evidence']['span_ns'] += 1
    bad['status']['station_feedback_gap']['discarded_source_evidence'][0]['feedback_gap_evidence']['span_ns'] += 1
    deliver(n, bad, ('status',))
    assert not n._station_compute_work_certificate(ticket)
    assert n._station_compute_retired and not n._station_compute_work
    assert not bool(getattr(n.latest_object_geometry, 'valid', False))
    assert not bool(getattr(n.latest_preview_rich_plan, 'valid', False))
