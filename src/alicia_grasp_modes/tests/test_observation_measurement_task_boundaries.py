"""A measurement handoff cannot erase previously observed target/support loss."""
from copy import deepcopy
import json
from types import SimpleNamespace as NS

import pytest

from test_task_modes import configured, module, base
from test_task_near_field_loss import status as lost_status
from test_support_observation import status as support_status
from test_support_source_qualification import install, messages


@pytest.mark.parametrize('state', ['executing', 'requalifying', 'qualified'])
def test_measurement_metadata_never_revives_same_selection_target_loss(
        configured, monkeypatch, state):
    node, params = configured
    params[module.SELECTION_PARAM] = dict(params[module.SELECTION_PARAM],
        strategy='observe_return', generation=3, stamp_ns='2200000000')
    assert node.reset_mode_cb(None).success
    node._mode_bound_selection = deepcopy(node._mode_selection)
    monkeypatch.setattr(module.rospy.Time, 'now',
        staticmethod(lambda: module.rospy.Time(5)))
    selection = node._mode_bound_selection
    original = lost_status(selection)
    node._unknown_support_status_cb(NS(data=json.dumps(original)))
    first = deepcopy(node._unknown_target_loss)

    # These later status/receipt fields do not constitute a new selection.
    followup = dict(original, target_lost=False, first_loss=None,
        state='ready' if state == 'qualified' else 'unavailable',
        observation_measurement=dict(state=state, metric_geometry_authority=False,
            motion_authorized=False, source_stamps_ns=[4100000000, 4300000000, 4500000000]))
    node._unknown_support_status_cb(NS(data=json.dumps(followup)))
    plan = NS(model_choice='unknown_tabletop', diagnostic='FAR_FIELD_OBSERVATION_PLAN')
    assert node._unknown_target_loss == first
    assert node._near_field_wait_failure(plan, {}).code == 'TARGET_LOST'


@pytest.mark.parametrize('moving', [False, True])
def test_measurement_metadata_does_not_replace_original_source_motion_evidence(
        configured, monkeypatch, moving):
    node, _ = configured
    node._mode_bound_selection = deepcopy(node._mode_selection)
    payload = support_status(node._mode_selection)
    payload.update(target_lost=False, first_loss=None,
        observation_measurement=dict(state='executing', metric_geometry_authority=False,
            motion_authorized=False))
    install(node, *messages(payload['source_stamp_ns'], moving=moving))
    node._unknown_support_status_cb(NS(data=json.dumps(payload)))
    plan = NS(model_choice='unknown_tabletop', diagnostic='CONTACT_EXECUTION_PLAN',
        header=NS(stamp=module.rospy.Time(3)))
    monkeypatch.setattr(base.GraspTaskNode, '_validate_bound_plan_locked',
        lambda *a: base.PlanValidationResult(True))
    calls = []
    validation, result = node._invoke_plan_bound_action(
        plan, {}, 'test contact boundary', lambda: calls.append('invoked'))
    if moving:
        assert validation.ok and calls == ['invoked']
        assert node._unknown_support_fault is None
    else:
        assert not validation.ok and validation.code == 'SUPPORT_GEOMETRY_CONTRADICTION'
        assert result is None and calls == []
        assert node._unknown_support_fault['limit'] == .004
