"""Optional geometric proposals must preserve the caller's typed control event."""
from types import SimpleNamespace

import pytest

from alicia_grasp_modes import arrival_fragment_group as group
from alicia_grasp_modes.station_compute_wait_producer import (
    ObservationArrivalStationChanged, StationComputeDeadline,
    StationComputeRetired, StationFeedbackBracketGap,
)


def run_with_failure(monkeypatch, error):
    members = [object(), object()]
    result = SimpleNamespace(candidates=members)
    monkeypatch.setattr(group, '_matching_candidates',
                        lambda *args: [(0., member) for member in members])
    def fail(*args, **kwargs):
        raise error
    monkeypatch.setattr(group, '_recover_matching_fragments', fail)
    return group.recover_arrival_matching_fragments(
        result, None, None, None, None, None, None, None, object(), None, None, .5)


@pytest.mark.parametrize('kind', ['deadline', 'retired', 'feedback_gap', 'station_changed'])
def test_optional_proposal_preserves_original_control_exception(monkeypatch, kind):
    owner = {'lease_id': 'already_registered'}
    errors = {
        'deadline': StationComputeDeadline('station_compute_current_measurement_expired'),
        'retired': StationComputeRetired(owner),
        'feedback_gap': StationFeedbackBracketGap(owner, {'source_ns': 123}),
        'station_changed': ObservationArrivalStationChanged('control_reference_epoch_changed'),
    }
    original = errors[kind]
    with pytest.raises(type(original)) as caught:
        run_with_failure(monkeypatch, original)
    assert caught.value is original
    assert caught.value.__cause__ is None
    if hasattr(original, 'registration'):
        assert original.registration is owner


def test_plain_geometry_failure_retains_original_rejection_context(monkeypatch):
    original = ValueError('rgb_instance_not_supported_by_current_target_depth')
    with pytest.raises(ValueError, match=(
            '^matching_fragment_original_gate_rejected: '
            'rgb_instance_not_supported_by_current_target_depth$')) as caught:
        run_with_failure(monkeypatch, original)
    assert type(caught.value) is ValueError
    assert caught.value.__cause__ is original


def test_existing_matching_fragment_rejection_is_not_rewritten(monkeypatch):
    original = ValueError('matching_fragment_independent_geometry_required')
    with pytest.raises(ValueError) as caught:
        run_with_failure(monkeypatch, original)
    assert caught.value is original
