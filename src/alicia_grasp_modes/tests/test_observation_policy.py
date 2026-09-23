from copy import deepcopy

import pytest

from alicia_grasp_modes.observation_policy import observation_config


@pytest.mark.parametrize('mode,strategy', [('carton','two_stage'), ('carton','direct'),
                                         ('unknown','direct')])
def test_existing_modes_keep_original_configuration(mode, strategy):
    config = {'observation_camera_target_max_distance_m': .22, 'keep': object()}
    assert observation_config(config, dict(mode=mode, strategy=strategy)) is config


def test_unknown_view_search_preserves_all_physical_configuration():
    config = {'observation_camera_target_nominal_distance_m': .2,
              'observation_camera_target_min_distance_m': .18,
              'observation_camera_target_max_distance_m': .22,
              'observation_camera_target_range_check_enabled': True,
              'contact_endpoint_precision_enabled': True,
              'near_field_replan_enabled': True}
    before = deepcopy(config)
    result = observation_config(config, dict(mode='unknown', strategy='two_stage'))
    assert result == dict(before, observation_camera_target_nominal_distance_m=.26,
                         observation_camera_target_min_distance_m=.22,
                         observation_camera_target_max_distance_m=.30)
    assert config == before
