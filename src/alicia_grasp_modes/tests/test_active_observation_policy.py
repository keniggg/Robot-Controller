"""Measured coverage, finite budgets and non-repeating view policy; no ROS."""
from copy import deepcopy
from types import SimpleNamespace as NS
import numpy as np
import pytest

from alicia_grasp_modes.active_observation_policy import (
    registered_information_gain, distinct_view, observation_budget)
from alicia_grasp_modes.observe_return_session import parse_session


def surfaces(extra=()):
    identity = NS(epoch=4, track_id='same-target')
    old = np.array([[0., y, .02] for y in (.001, .011, .021)])
    points = np.vstack((old, np.array(extra).reshape(-1, 3)))
    return (NS(identity=identity, points_base=old, view_stamps_ns=(10,), view_indices=np.zeros(3, int)),
        NS(identity=identity, points_base=points, view_stamps_ns=(10, 20),
           view_indices=np.r_[np.zeros(3, int), np.ones(len(points)-3, int)]))


def test_new_registered_coverage_requires_distinct_occupied_cells_beyond_noise():
    before, after = surfaces([[.01, y, .02] for y in (.001, .011, .021)])
    report = registered_information_gain(before, after, NS(ok=True), depth_uncertainty_m=.001)
    assert report['ok'] and report['new_occupied_voxel_count'] == 3
    assert report['source_stamp_ns'] == 20 and not report['contact_or_motion_authorized']


@pytest.mark.parametrize('kind', ['timestamp_only', 'same_voxels', 'below_noise', 'only_two', 'old_provenance'])
def test_new_stamp_or_repeated_measurements_never_count_as_new_information(kind):
    extra = [[.01, y, .02] for y in (.001, .011, .021)]
    if kind == 'timestamp_only': extra = []
    if kind == 'same_voxels': extra = [[.0001, y, .02] for y in (.001, .011, .021)]
    if kind == 'below_noise': extra = [[.003, y, .02] for y in (.001, .011, .021)]
    if kind == 'only_two': extra = extra[:2]
    before, after = surfaces(extra)
    if kind == 'old_provenance': after.view_indices[:] = 0
    result = registered_information_gain(before, after, NS(ok=True), depth_uncertainty_m=.004)
    assert not result['ok']


@pytest.mark.parametrize('fault', ['registration', 'identity', 'source_chain', 'stale_source', 'nan', 'missing_noise'])
def test_invalid_registered_evidence_is_not_information_gain(fault):
    before, after = surfaces([[.01, y, .02] for y in (.001, .011, .021)])
    registration=NS(ok=True); noise=.001
    if fault == 'registration': registration.ok=False
    if fault == 'identity': after.identity=NS(epoch=5,track_id='same-target')
    if fault == 'source_chain': after.view_stamps_ns=(9,20)
    if fault == 'stale_source': after.view_stamps_ns=(10,10)
    if fault == 'nan': after.points_base[-1,0]=float('nan')
    if fault == 'missing_noise': noise=None
    with pytest.raises(ValueError): registered_information_gain(before,after,registration,depth_uncertainty_m=noise)


def test_repeated_rays_radial_changes_and_small_uncertain_turns_are_not_new_views():
    seen=[[.2,0.,0.]]
    assert not distinct_view([.3,0.,0.], [0.,0.,0.], seen, .003)
    assert not distinct_view([.2,.001,0.], [0.,0.,0.], seen, .003)
    assert distinct_view([.2,.08,0.], [0.,0.,0.], seen, .003)


@pytest.mark.parametrize('bad', [0,4,True,2.,None])
def test_budget_cannot_be_unbounded_or_coerced(bad):
    with pytest.raises(ValueError): observation_budget(bad)


def initial(version=2):
    value=dict(version=version,session_id='s',selection=dict(mode='unknown',strategy='observe_return',
        generation=1,stamp_ns='1'),phase='observing',stamp_ns='100',deadline_ns='600000000100',
        target_epoch=4,target_track_id='same',origin_plan_id='origin',origin_snapshot_stamp_ns='10',
        return_verified_stamp_ns=0,contact_deadline_ns=0)
    if version==2:value.update(max_observations=3,view_index=1,view_plan_id='origin',
        view_source_stamp_ns='10',view_request_id='initial',previous_gain_sha256='0'*64)
    return value


def step(previous,index):
    return dict(previous,view_index=index,view_plan_id='view%d'%index,
        view_source_stamp_ns=str(index*100),stamp_ns=str(index*100+1),
        view_request_id='request%d'%index,previous_gain_sha256=str(index)*64)


def test_v2_three_observations_share_original_return_and_deadline_with_wire_ns_strings():
    one=initial();two=step(one,2);three=step(two,3)
    parse_session(one,one['selection'],101)
    parse_session(two,one['selection'],202,one)
    last=parse_session(three,one['selection'],302,two)
    assert last['deadline_ns']==int(one['deadline_ns']) and last['origin_plan_id']=='origin'
    with pytest.raises(ValueError):parse_session(step(three,4),one['selection'],402,three)
    returning=dict(three,phase='returning',stamp_ns=400)
    parse_session(returning,one['selection'],401,three)
    returned=dict(returning,phase='returned',stamp_ns=600,return_verified_stamp_ns=500,contact_deadline_ns=700)
    parse_session(returned,one['selection'],601,returning)


@pytest.mark.parametrize('change', [dict(view_index=3),dict(view_plan_id='origin'),
    dict(view_request_id='initial'),dict(view_source_stamp_ns=10),dict(previous_gain_sha256='0'*64),
    dict(deadline_ns='600000000101'),dict(target_epoch=5),dict(origin_plan_id='other'),
    dict(max_observations=2),dict(version=1)])
def test_v2_cannot_replay_skip_rebind_renew_or_downgrade(change):
    one=initial();two=step(one,2);two.update(change)
    with pytest.raises(ValueError):parse_session(two,one['selection'],250,one)


def test_v1_remains_single_view_and_cannot_smuggle_v2_fields():
    one=initial(1)
    parse_session(one,one['selection'],101)
    with pytest.raises(ValueError):parse_session(dict(one,stamp_ns=200),one['selection'],201,one)
    with pytest.raises(ValueError):parse_session(dict(one,max_observations=3),one['selection'],101)
    parse_session(dict(one,phase='returning',stamp_ns=200),one['selection'],201,one)
