"""A current-frame clock expiry cannot erase an independently intact station."""
from copy import deepcopy

import pytest
from std_msgs.msg import Header

from alicia_grasp_modes.station_compute_wait_producer import StationComputeDeadline
from test_station_compute_wait_producer import armed
from test_locked_rgb_hold import custom_frame, latest_status, module


def enrolled_next(monkeypatch):
    node, clock, joint, params, marker, _ = armed(monkeypatch)
    clock[0] += .2
    with monkeypatch.context() as queue_only:
        queue_only.setattr(node, 'process', lambda _:None)
        source = custom_frame(node, clock, joint, target_depth=.35)
    node._station_compute_begin_frame(source)
    assert node._station_compute_frame['source'] is source
    return node, clock, joint, params, marker, source


def boundary(monkeypatch, node, clock, joint, params, source, location, offset, change=None):
    fired = []
    source_ns = source[0].header.stamp.to_nsec()
    original = getattr(node, {'scope':'_station_compute_scope',
        'context':'_rgb_interest_source_context', 'sdk':'_rgb_bracketed_feedback'}[location])
    def inject(*args, **kwargs):
        value = original(*args, **kwargs)
        if not fired:
            fired.append(True)
            now = source_ns+800_000_000+offset
            clock[0] = now*1e-9
            monkeypatch.setattr(module.rospy.Time, 'now', staticmethod(
                lambda:module.rospy.Time(now//10**9,now%10**9)))
            joint(clock[0])
            if change == 'manual': params['/gui/joint_direct_mode'] = True
            elif change == 'epoch': node._rgb_interest_epoch_cb(Header(
                stamp=module.rospy.Time(now//10**9,now%10**9), frame_id='sdk_reference_epoch'))
            elif change == 'pose_return':
                joint(clock[0], [.005,0.,0.,0.,0.,0.]); joint(clock[0]+.000001)
            elif change == 'model': params['/robot_description'] = '<robot name="changed"/>'
        return value
    monkeypatch.setattr(node, {'scope':'_station_compute_scope',
        'context':'_rgb_interest_source_context', 'sdk':'_rgb_bracketed_feedback'}[location], inject)
    return fired


@pytest.mark.parametrize('location', ['scope','context','sdk'])
@pytest.mark.parametrize('offset', [-1,0,1], ids=['before','exact','after'])
def test_registered_source_deadline_has_same_typed_result_at_every_audit_boundary(monkeypatch, location, offset):
    n, clock, joint, params, marker, source = enrolled_next(monkeypatch)
    hold, last, registration = n._locked_rgb_hold_reference, n.tracker.last, n._station_compute_registration
    contract, deadline = deepcopy(registration['contract']), registration['monotonic_deadline']
    fired = boundary(monkeypatch,n,clock,joint,params,source,location,offset)
    if offset < 0:
        assert n._observation_arrival_retry_reference(source, params['/camera']) is hold
    else:
        with pytest.raises(StationComputeDeadline) as caught:
            n._observation_arrival_retry_reference(source, params['/camera'])
        assert n._station_compute_timeout(source,caught.value)
        assert latest_status(n)['state'] == 'processing_deadline_pending'
        assert not n.pubs['object'].publish.call_args[0][0].detected
    assert fired
    assert n._station_compute_registration is registration
    assert registration['contract'] == contract and registration['monotonic_deadline'] == deadline
    assert n._locked_rgb_hold_reference is hold and n.tracker.last is last
    assert hold['observation_arrival_retry'] is marker and not marker.get('invalidated_reason')
    assert n.last_good_source_stamp_ns == contract['source_ns']
    assert n.first_loss is None and not n.tracker.lost


@pytest.mark.parametrize('location', ['scope','context','sdk'])
@pytest.mark.parametrize('change', ['manual','epoch','pose_return','model'])
def test_source_expiry_with_real_station_change_never_enters_identity_pending(monkeypatch, location, change):
    n, clock, joint, params, marker, source = enrolled_next(monkeypatch)
    old = n.last_good_source_stamp_ns
    fired = boundary(monkeypatch,n,clock,joint,params,source,location,1,change)
    try:
        result = n._observation_arrival_retry_reference(source, params['/camera'])
    except StationComputeDeadline as error:
        # A concurrent full-context fault can occur after an earlier scope
        # check; the final original timeout router still must reject it.
        assert not n._station_compute_timeout(source,error)
    else:
        assert result is None
    assert fired and n.last_good_source_stamp_ns == old
    assert latest_status(n)['state'] != 'processing_deadline_pending'
