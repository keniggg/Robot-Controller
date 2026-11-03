"""A non-authoritative pending preview can precede the task's first frame join."""
from copy import deepcopy
from itertools import permutations
from types import SimpleNamespace as NS

import pytest

from alicia_grasp_modes import station_compute_wait_task as wait
from test_station_compute_wait_task import station, frame, deliver
from test_observe_return_task import at_ns
from test_task_modes import configured


def pending(c):
    # Same production message shape recorded in supplemental_view_r5; no plan,
    # target or poses are asserted by this computational status notification.
    return NS(valid=False, candidate_source='near_field_pending', plan_id='',
        diagnostic='TARGET_PROCESSING_PENDING: current geometry expired; identity only',
        header=NS(stamp=at_ns(c.now[0])), target_track_id='', poses=[])


@pytest.mark.parametrize('order', list(permutations(('object','mask','status'))))
def test_pending_before_first_local_join_waits_without_creating_authority(station, order):
    c=station; n=c.node
    assert n._station_task_record is None and n._station_task_receiving()
    original_deadline=n._near_field_phase_deadline_sec
    original_plan=deepcopy(n._bound_execution_plan)
    n.latest_grasp6d_preview_plan=pending(c)
    for _ in range(3):
        result, candidate=n._copy_near_field_preview_candidate(c.plan, c.now[0]-1,
            {'near_field_strategy':'single_snapshot_direct'})
        assert result.code==wait.PENDING and not result.ok and not result.terminal_for_request
        assert candidate is None and n._station_task_record is None
        assert n._near_field_phase_deadline_sec==original_deadline
        assert n._bound_execution_plan==original_plan
    row=frame(c);deliver(c,row,order)
    assert n._station_task_record['contract']==row['status']['station_compute_wait']
    saved=deepcopy(n._station_task_record)
    result,candidate=n._copy_near_field_preview_candidate(c.plan,0,{})
    assert result.code==wait.PENDING and candidate is None
    assert n._station_task_record==saved and not saved['pending']


def test_pending_cannot_hide_a_retired_station(station):
    c=station;c.node._station_task_retired=True
    c.node.latest_grasp6d_preview_plan=pending(c)
    result,candidate=c.node._copy_near_field_preview_candidate(c.plan,0,{})
    assert result.code=='TARGET_LOST' and not result.ok and candidate is None


@pytest.mark.parametrize('mutation', ['valid','source','code','disarmed'])
def test_other_invalid_previews_are_not_admitted_as_initial_pending(station, mutation):
    c=station;n=c.node;p=pending(c)
    if mutation=='valid':p.valid=True
    elif mutation=='source':p.candidate_source='other'
    elif mutation=='code':p.diagnostic='INVALID: different failure'
    else:n._station_task_disarmed=n._station_task_owner_key()
    n.latest_grasp6d_preview_plan=p
    result,candidate=n._copy_near_field_preview_candidate(c.plan,0,
        {'near_field_strategy':'single_snapshot_direct'})
    assert result.code!=wait.PENDING and not result.ok and candidate is None
    assert n._station_task_record is None
