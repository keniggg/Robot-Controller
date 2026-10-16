"""Independent adversarial checks of advisory search evidence and late results.

The station owner admission/receipt integration is covered separately. These
tests do not manufacture a station certificate or claim endpoint feasibility.
"""
from copy import deepcopy
from types import SimpleNamespace as NS

import numpy as np
import pytest

from test_remote_modes import configured, base, module
from test_observation_fk_first_strict import loop_context
from alicia_grasp_modes.observation_search_progress import SearchProgress
from alicia_grasp_modes.silhouette_observation_order import transform_fingerprint


def endpoint_evidence(configured, kind):
    node, _ = configured
    matrix = np.eye(4)
    matrix[:3, 3] = [.2, .03, .15]
    pose = base.make_pose_stamped('base_link', matrix[:3, 3], [0., 0., 0., 1.],
                                 stamp=base.rospy.Time(10, 0))
    source = dict(binding=dict(source_stamp_ns=10_000_000_000, request_id=4))
    context = dict(prepared=NS(snapshot=NS(stamp_ns=10_000_000_000)),
                   source=source, owner=dict(model_sha256='current-model'))
    audit = dict(information_utility=dict(utility=.5))
    variant = dict(sequence=NS(pregrasp=pose), observation_view=audit,
                   search_family='joint_space_fk' if kind == 'fk' else 'camera_orientation_lattice')
    if kind == 'fk':
        audit['proposal'] = dict(execution_authority=False, binding=deepcopy(source['binding']),
                                 model_sha256='current-model', T_base_tool0=matrix.tolist())
    else:
        audit['check_scheduling_metrics'] = dict(endpoint_ik_hint=dict(
            status='endpoint_found', pose_sha256=transform_fingerprint(matrix),
            source_stamp_ns=10_000_000_000))
    return node, variant, context


@pytest.mark.parametrize('kind', ['fk', 'ik'])
def test_current_positive_evidence_is_advisory_and_does_not_change_pose(configured, kind):
    node, row, context = endpoint_evidence(configured, kind)
    frozen = deepcopy(row)
    assert node._station_observation_positive_endpoint(row, context)
    assert row == frozen
    assert not node.rich_plan_pub.messages[-1].valid


@pytest.mark.parametrize('fault', ['source', 'pose', 'status', 'zero_information'])
def test_ik_hint_for_another_source_or_pose_cannot_get_positive_rank(configured, fault):
    node, row, context = endpoint_evidence(configured, 'ik')
    hint = row['observation_view']['check_scheduling_metrics']['endpoint_ik_hint']
    if fault == 'source':
        hint['source_stamp_ns'] -= 1
    elif fault == 'pose':
        row['sequence'].pregrasp.pose.position.x += .001
    elif fault == 'status':
        hint['status'] = 'timeout'
    else:
        row['observation_view']['information_utility']['utility'] = 0.
    assert not node._station_observation_positive_endpoint(row, context)


@pytest.mark.parametrize('fault', ['binding', 'model', 'pose', 'authority', 'label_only', 'zero_information'])
def test_fk_label_does_not_substitute_for_current_bound_fk_evidence(configured, fault):
    node, row, context = endpoint_evidence(configured, 'fk')
    proposal = row['observation_view']['proposal']
    if fault == 'binding':
        proposal['binding']['request_id'] += 1
    elif fault == 'model':
        proposal['model_sha256'] = 'old-model'
    elif fault == 'pose':
        row['sequence'].pregrasp.pose.position.y += .001
    elif fault == 'authority':
        proposal['execution_authority'] = True
    elif fault == 'label_only':
        row['observation_view']['proposal'] = {}
    else:
        row['observation_view']['information_utility']['utility'] = 0.
    assert not node._station_observation_positive_endpoint(row, context)


def test_late_successful_strict_rpc_does_not_commit_into_changed_station(
        configured, monkeypatch, tmp_path):
    node, prepared, runtime, rows, _, _, events, _, clock = loop_context(
        configured, monkeypatch, tmp_path)
    progress = SearchProgress()
    old = progress.bind({'station': 'old'}, remaining_sec=90., monotonic_now=clock[0])
    context = dict(prepared=prepared, progress=progress, token=old,
                   clock=lambda: clock[0], dispatched=[])
    runtime['station_observation_search'] = context
    runtime['_station_search_dispatch'] = dict(family='old-family', variant=rows[1])
    node._active_unknown_observation_check_runtime = runtime
    previous_strict = base.RemoteGrasp6DNode._strict_moveit_evaluation

    def response_after_owner_changed(self, target, observation=False):
        response = previous_strict(self, target, observation=observation)
        assert response[0].reachable
        progress.bind({'station': 'new'}, remaining_sec=80., monotonic_now=clock[0])
        return response

    monkeypatch.setattr(base.RemoteGrasp6DNode, '_strict_moveit_evaluation', response_after_owner_changed)
    # Only the post-RPC owner lookup is supplied here. The generic fixture's
    # real prepared-source guard and production strict wrapper still execute.
    monkeypatch.setattr(node, '_station_observation_search_context',
                        lambda _prepared: dict(token=progress.token))
    with pytest.raises(base.StreamResultCancelled, match='GENERATION_STALE'):
        node._strict_moveit_evaluation(rows[1]['sequence'].pregrasp, observation=True)
    assert events == [('strict', 1)]
    assert progress.families == {} and progress.maximum_check_sec == 0.
    assert context['dispatched'] == []
    assert not node.rich_plan_pub.messages[-1].valid


def test_backward_wall_budget_cannot_restore_spent_monotonic_reserve():
    progress = SearchProgress()
    progress.bind({'station': 1}, remaining_sec=5., monotonic_now=20.)
    progress.bind({'station': 1}, remaining_sec=100., monotonic_now=24.)
    result = progress.reserve(remaining_sec=100., monotonic_now=24.,
                              planning_sec=1., publication_sec=.85)
    assert result['remaining_sec'] == 1.
    assert result['can_start'] is False
    assert progress.deadline_monotonic == 25.
