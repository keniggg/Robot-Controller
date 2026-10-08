from collections import deque
from copy import deepcopy
import json
import threading
from types import SimpleNamespace as NS

import pytest
from alicia_flexible_grasp.robot.stationary_following import ARM_NAMES, SDK_QUANTUM_RAD
from alicia_grasp_modes.support_observation import qualify_support_source
from test_task_modes import configured, module, base
from test_support_observation import status


def messages(source=10_000_000_000, moving=False):
    offsets = [-350_000_000, -250_000_000, -150_000_000, -50_000_000, 50_000_000]
    def message(stamp, frame, names, positions):
        return NS(header=NS(stamp=NS(to_nsec=lambda: stamp), frame_id=frame),
                  name=list(names), position=list(positions))
    sdk, accepted = [], []
    for i, offset in enumerate(offsets):
        q = [0.] * 6
        if moving:
            q[4] = i * SDK_QUANTUM_RAD * .6
        sdk.append(message(source + offset, 'sdk_transmitted', ARM_NAMES, q))
        accepted.append(message(source + offset, 'sdk_measured', ARM_NAMES + ('right_finger',), q + [.04]))
    return sdk, accepted


def qualify(sdk, accepted, source=10_000_000_000, epoch=1_000_000_000, expected=1_000_000_000):
    return qualify_support_source(sdk, accepted, source_stamp_ns=source,
                                 epoch_ns=epoch, expected_epoch_ns=expected)


def install(node, sdk, accepted, epoch=2_000_000_000):
    node._following_evidence_lock = threading.RLock()
    node._following_sdk_history = deque(sdk, maxlen=128)
    node._following_accepted_history = deque(accepted, maxlen=128)
    node._following_epoch_ns = epoch


def contact_plan(stamp=3):
    return NS(model_choice='unknown_tabletop', diagnostic='CONTACT_EXECUTION_PLAN',
              header=NS(stamp=module.rospy.Time(stamp)))


def test_full_original_stationary_window_preserves_existing_quantum_bound():
    sdk, accepted = messages()
    accepted[2].position[4] = SDK_QUANTUM_RAD
    result = qualify(sdk, accepted)
    assert result['state'] == 'stationary'
    assert result['window_start_ns'] == 9_700_000_000
    assert result['accepted_source_bracket_ns'] == [9_950_000_000, 10_050_000_000]
    assert result['motion_authorized'] is False


def test_subquantum_adjacent_motion_is_not_mislabeled_stationary():
    sdk, accepted = messages(moving=True)
    assert accepted[-1].position[4] - accepted[-2].position[4] < SDK_QUANTUM_RAD
    result = qualify(sdk, accepted)
    assert result['state'] == 'moving'
    assert result['moving_joint_names'] == ['Joint5']


def test_sdk_quantized_plateau_uses_only_the_accepted_source_time_resolution():
    _, accepted = messages(moving=True)
    sdk = []
    for offset_ms in range(-350, 71, 20):
        stamp = 10_000_000_000 + offset_ms * 1_000_000
        q = [0.] * 6
        q[4] = ((offset_ms + 350) // 100) * SDK_QUANTUM_RAD * .6
        sdk.append(NS(header=NS(stamp=NS(to_nsec=lambda ns=stamp: ns),
                                frame_id='sdk_transmitted'), name=list(ARM_NAMES), position=q))
    result = qualify(sdk, accepted)
    assert result['sdk_source_bracket_ns'] == [9_990_000_000, 10_010_000_000]
    assert result['sdk_motion_evidence_window_ns'] == [9_950_000_000, 10_050_000_000]
    assert result['state'] == 'moving'


@pytest.mark.parametrize('fault', ['sdk_only', 'accepted_only', 'other_axis', 'stopped_at_source',
    'no_before', 'no_after', 'accepted_gap', 'sdk_gap', 'duplicate', 'reverse', 'frame',
    'names', 'nan', 'opening', 'epoch_changed', 'epoch_missing', 'cross_epoch',
    'earlier_motion_then_source_jitter', 'source_direction_opposed', 'sdk_change_after_bracket'])
def test_missing_or_noncontemporaneous_motion_never_exempts_a_support_fault(fault):
    sdk, accepted = messages(moving=True)
    kwargs = {}
    if fault == 'sdk_only':
        for m in accepted: m.position[4] = 0.
    elif fault == 'accepted_only':
        for m in sdk: m.position[4] = 0.
    elif fault == 'other_axis':
        for m in sdk: m.position[3], m.position[4] = m.position[4], 0.
    elif fault == 'stopped_at_source': accepted[-1].position[4] = accepted[-2].position[4]
    elif fault == 'earlier_motion_then_source_jitter': sdk[-1].position[4] = sdk[-2].position[4]
    elif fault == 'source_direction_opposed':
        sdk[-1].position[4] = sdk[-2].position[4] - SDK_QUANTUM_RAD
    elif fault == 'sdk_change_after_bracket':
        sdk[-1].header.stamp = NS(to_nsec=lambda: 10_060_000_000)
    elif fault == 'no_before': sdk.pop(0)
    elif fault == 'no_after': accepted.pop()
    elif fault == 'accepted_gap': accepted[:] = [accepted[0], accepted[1], accepted[-1]]
    elif fault == 'sdk_gap': sdk[:] = [sdk[0], sdk[1], sdk[-1]]
    elif fault == 'duplicate': sdk.insert(2, sdk[1])
    elif fault == 'reverse': accepted[1], accepted[2] = accepted[2], accepted[1]
    elif fault == 'frame': accepted[2].header.frame_id = 'requested'
    elif fault == 'names': sdk[2].name[1] = sdk[2].name[0]
    elif fault == 'nan': accepted[2].position[4] = float('nan')
    elif fault == 'opening': accepted[2].position[-1] = .051
    elif fault == 'epoch_changed': kwargs['expected'] = 2_000_000_000
    elif fault == 'epoch_missing': kwargs['epoch'] = None
    elif fault == 'cross_epoch': kwargs.update(epoch=9_800_000_000, expected=9_800_000_000)
    assert qualify(sdk, accepted, **kwargs)['state'] == 'unqualified'


def test_delayed_moving_source_does_not_use_later_stationary_history():
    sdk, accepted = messages(moving=True)
    later_sdk, later_accepted = messages(source=12_000_000_000)
    result = qualify(sdk + later_sdk, accepted + later_accepted)
    assert result['state'] == 'moving'
    assert result['sdk_window_ns'][-1] == 10_050_000_000


def test_late_bracket_is_cached_before_long_action_evicts_history(configured, monkeypatch):
    node, _ = configured
    node._mode_bound_selection = deepcopy(node._mode_selection)
    payload = status(node._mode_selection)
    source = payload['source_stamp_ns']
    sdk, accepted = messages(source, moving=True)
    install(node, sdk[:-1], accepted[:-1])
    monkeypatch.setattr(base.GraspTaskNode, '_validate_bound_plan_locked', lambda *a: base.PlanValidationResult(True))
    node._unknown_support_status_cb(NS(data=json.dumps(payload)))
    assert node._validate_bound_plan_locked(contact_plan(), {}).code == 'SUPPORT_OBSERVATION_UNQUALIFIED'
    node.following_sdk_cb(sdk[-1])
    node.accepted_joint_cb(accepted[-1])
    assert not node._unknown_support_pending and node._unknown_support_fault is None
    assert node._unknown_support_diagnostics[-1]['source_qualification']['state'] == 'moving'
    later = messages(source + 15_000_000_000)
    install(node, *later)
    node._unknown_support_status_cb(NS(data=json.dumps(payload)))
    assert node._validate_bound_plan_locked(contact_plan(), {}).ok
    assert node._unknown_support_fault is None


def test_missing_original_history_remains_unqualified_after_occlusion(configured, monkeypatch):
    node, _ = configured
    node._mode_bound_selection = deepcopy(node._mode_selection)
    payload = status(node._mode_selection)
    install(node, [], [])
    monkeypatch.setattr(base.GraspTaskNode, '_validate_bound_plan_locked', lambda *a: base.PlanValidationResult(True))
    node._unknown_support_status_cb(NS(data=json.dumps(payload)))
    payload['detail'] = 'perception_failure: locked_support_plane_no_longer_visible'
    node._unknown_support_status_cb(NS(data=json.dumps(payload)))
    assert node._unknown_support_fault is None
    assert node._validate_bound_plan_locked(contact_plan(), {}).code == 'SUPPORT_OBSERVATION_UNQUALIFIED'


def test_ordinary_occlusion_does_not_require_new_support_observation(configured, monkeypatch):
    node, _ = configured
    node._mode_bound_selection = deepcopy(node._mode_selection)
    payload = status(node._mode_selection)
    payload['detail'] = 'perception_failure: locked_support_plane_no_longer_visible'
    monkeypatch.setattr(base.GraspTaskNode, '_validate_bound_plan_locked', lambda *a: base.PlanValidationResult(True))
    node._unknown_support_status_cb(NS(data=json.dumps(payload)))
    assert node._validate_bound_plan_locked(contact_plan(), {}).ok
    assert not node._unknown_support_pending


def test_static_hard_fault_survives_later_moving_fault_and_occlusion(configured, monkeypatch):
    node, _ = configured
    node._mode_bound_selection = deepcopy(node._mode_selection)
    payload = status(node._mode_selection)
    install(node, *messages(payload['source_stamp_ns']))
    monkeypatch.setattr(base.GraspTaskNode, '_validate_bound_plan_locked', lambda *a: base.PlanValidationResult(True))
    node._unknown_support_status_cb(NS(data=json.dumps(payload)))
    first = deepcopy(node._unknown_support_fault)
    for key in ('source_stamp_ns', 'color_stamp_ns', 'depth_stamp_ns', 'tf_lookup_stamp_ns'):
        payload[key] += 1_000_000_000
    install(node, *messages(payload['source_stamp_ns'], moving=True))
    node._unknown_support_status_cb(NS(data=json.dumps(payload)))
    payload['detail'] = 'perception_failure: locked_support_plane_no_longer_visible'
    node._unknown_support_status_cb(NS(data=json.dumps(payload)))
    assert node._unknown_support_fault == first
    assert node._validate_bound_plan_locked(contact_plan(), {}).code == 'SUPPORT_GEOMETRY_CONTRADICTION'
