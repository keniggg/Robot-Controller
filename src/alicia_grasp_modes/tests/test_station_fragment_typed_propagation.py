"""Typed control outcomes survive real fragment geometry and RGB boundaries.

Boundary sentinels are injected only after the original native/refinement or
appearance calculation runs. Process tests stop at the outer error router;
they prove propagation, not that a synthetic sentinel authorizes waiting.
"""
import inspect

import pytest

from alicia_grasp_modes import arrival_fragment_group as group
from alicia_grasp_modes.station_compute_wait_producer import (
    ObservationArrivalStationChanged, StationComputeDeadline,
    StationComputeRetired, StationFeedbackBracketGap)
from test_arrival_matching_fragments import scene
from test_arrival_same_instance_confirmation import completed_matching_arrival, matching_frame
from test_locked_rgb_hold import latest_status
from test_metric_ambiguity_hold import frame as ambiguity_frame


BOUNDARIES = ['refine_1', 'refine_2', 'core_1', 'core_2',
              'appearance_1', 'appearance_2', 'appearance_3']
KINDS = ['deadline', 'retired', 'feedback_gap', 'station_changed']


def sentinel(kind):
    if kind == 'deadline': return StationComputeDeadline('matching_fragment_typed_deadline')
    if kind == 'retired': return StationComputeRetired({'sentinel': True})
    if kind == 'feedback_gap': return StationFeedbackBracketGap({'sentinel': True}, {'sentinel': True})
    if kind == 'station_changed': return ObservationArrivalStationChanged('matching_fragment_station_changed')
    raise AssertionError(kind)


def interrupt_boundary(monkeypatch, boundary, error):
    kind, index = boundary.rsplit('_', 1)
    index = int(index)
    state = {'calls': 0, 'injected': False}
    if kind == 'appearance':
        owner, name = group, 'match_observation_appearance'
    else:
        owner, name = group._FrameRefinement, ('refine' if kind == 'refine' else 'confidence_labels')
    original = getattr(owner, name)
    def interrupted(*args, **kwargs):
        # confidence_labels also runs inside the refiner; target the separate
        # mandatory native-member core proof in the matching helper only.
        selected = kind != 'core' or inspect.currentframe().f_back.f_code.co_name == '_recover_matching_fragments'
        result = original(*args, **kwargs)
        if selected:
            state['calls'] += 1
            if state['calls'] == index:
                state['injected'] = True
                raise error
        return result
    monkeypatch.setattr(owner, name, interrupted)
    return state


@pytest.mark.parametrize('kind', KINDS)
@pytest.mark.parametrize('boundary', BOUNDARIES)
def test_each_native_and_consensus_boundary_preserves_typed_exception_identity(monkeypatch, kind, boundary):
    inputs = scene()
    error = sentinel(kind)
    state = interrupt_boundary(monkeypatch, boundary, error)
    with pytest.raises(type(error)) as raised:
        group.recover_arrival_matching_fragments(**inputs)
    assert state['injected'] and raised.value is error
    assert raised.value.__cause__ is None
    assert len(inputs['result'].candidates) == 2


@pytest.mark.parametrize('boundary', BOUNDARIES)
def test_plain_original_gate_error_retains_optional_proposal_wrapper(monkeypatch, boundary):
    inputs = scene()
    original = ValueError('independent_original_rgb_or_geometry_rejected')
    state = interrupt_boundary(monkeypatch, boundary, original)
    with pytest.raises(ValueError, match='^matching_fragment_original_gate_rejected: independent_original_rgb_or_geometry_rejected$') as raised:
        group.recover_arrival_matching_fragments(**inputs)
    assert state['injected'] and type(raised.value) is ValueError
    assert raised.value.__cause__ is original


@pytest.mark.parametrize('kind', KINDS)
@pytest.mark.parametrize('boundary', BOUNDARIES)
def test_real_process_optional_proposal_cannot_consume_typed_outcome(monkeypatch, kind, boundary):
    node, clock, _, joint, _, _ = completed_matching_arrival(monkeypatch)
    previous_last, previous_reference = node.tracker.last, node._locked_rgb_hold_reference
    previous_source = node.last_good_source_stamp_ns
    previous_publications = node.pubs['object'].publish.call_count
    error = sentinel(kind)
    state = interrupt_boundary(monkeypatch, boundary, error)
    routed = []
    original = node._station_requalification_wait
    def router(source, caught=None):
        if caught is error:
            routed.append(caught)
            return True  # Inert routing sentinel, no pending/motion authority.
        return original(source, caught)
    monkeypatch.setattr(node, '_station_requalification_wait', router)
    matching_frame(node, clock, joint)
    assert state['injected'] and routed == [error], latest_status(node)
    assert node.tracker.last is previous_last
    assert node._locked_rgb_hold_reference is previous_reference
    assert node.last_good_source_stamp_ns == previous_source
    assert node.pubs['object'].publish.call_count == previous_publications
    assert node.first_loss is None and node._metric_ambiguity_pending is None


def test_actual_separate_rgb_objects_keep_original_ambiguity_pending(monkeypatch):
    node, clock, _, joint, _, _ = completed_matching_arrival(monkeypatch)
    previous_last, previous_source = node.tracker.last, node.last_good_source_stamp_ns
    clock[0] += .1
    ambiguity_frame(node, clock, joint)
    status = latest_status(node)
    assert node._metric_ambiguity_pending is not None and not node.tracker.lost
    assert node.tracker.last is previous_last and node.last_good_source_stamp_ns == previous_source
    assert not node.pubs['object'].publish.call_args[0][0].detected
    assert status['frame_timing']['matching_fragment_proposal']['accepted'] is False
    assert status['frame_timing']['matching_fragment_proposal']['reason'].startswith('matching_fragment')


def test_actual_separate_rgb_and_support_pixels_keep_plain_ownership_rejection():
    inputs = scene()
    inputs['rgb'][210:250, 313:317] = (60, 90, 190)
    inputs['depth'][210:250, 313:317] = .3
    with pytest.raises(ValueError, match='^matching_fragment_competitor_not_owned$') as raised:
        group.recover_arrival_matching_fragments(**inputs)
    assert type(raised.value) is ValueError
