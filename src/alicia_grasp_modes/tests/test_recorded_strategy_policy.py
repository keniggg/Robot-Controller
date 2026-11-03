import pytest
from alicia_grasp_modes.runner_contract import aligned_unknown_strategy


def selection(strategy='observe_return', mode='unknown'):
    return dict(mode=mode, strategy=strategy, generation=4, stamp_ns=123456789)


def test_initial_contact_and_auxiliary_observation_share_one_recorded_strategy():
    assert aligned_unknown_strategy(selection()) == 'observe_return'
    assert aligned_unknown_strategy(selection(), 'observe_return') == 'observe_return'


@pytest.mark.parametrize('legacy', ['direct', 'two_stage'])
def test_legacy_environment_override_cannot_select_another_workflow(legacy):
    with pytest.raises(ValueError, match='cannot override'):
        aligned_unknown_strategy(selection(), legacy)
    with pytest.raises(ValueError, match='requires observe_return'):
        aligned_unknown_strategy(selection(legacy))


def test_carton_selection_is_not_changed_to_unknown():
    assert aligned_unknown_strategy(selection('two_stage', 'carton')) == 'two_stage'
    with pytest.raises(ValueError, match='cannot override'):
        aligned_unknown_strategy(selection('two_stage', 'carton'), 'observe_return')
