import pytest

from alicia_flexible_grasp.robot.observation_view_limits import effective_observation_view_limits


def test_shared_defaults_and_tighter_values():
    assert effective_observation_view_limits({})==dict(
        roi_margin_px=36.,minimum_depth_m=.070,maximum_depth_m=.500)
    assert effective_observation_view_limits(dict(camera_visibility_margin_px=48,
        camera_visibility_min_depth_m=.09,camera_visibility_max_depth_m=.40))==dict(
        roi_margin_px=48.,minimum_depth_m=.09,maximum_depth_m=.40)


@pytest.mark.parametrize('key,value',[
    ('camera_visibility_margin_px',35.999),('camera_visibility_min_depth_m',.06999),
    ('camera_visibility_max_depth_m',.50001)])
def test_looser_than_original_limits_are_rejected(key,value):
    with pytest.raises(ValueError,match='cannot relax'):
        effective_observation_view_limits({key:value})


@pytest.mark.parametrize('value',[True,False,None,'36',[],float('nan'),float('inf'),float('-inf')])
@pytest.mark.parametrize('key',['camera_visibility_margin_px','camera_visibility_min_depth_m','camera_visibility_max_depth_m'])
def test_invalid_values_never_coerce_to_permitted_limits(key,value):
    with pytest.raises(ValueError):effective_observation_view_limits({key:value})


@pytest.mark.parametrize('cfg',[None,[],dict(camera_visibility_min_depth_m=.5),
    dict(camera_visibility_min_depth_m=.4,camera_visibility_max_depth_m=.3),
    dict(camera_visibility_max_depth_m=.07)])
def test_bad_mapping_or_empty_depth_range_rejected(cfg):
    with pytest.raises(ValueError):effective_observation_view_limits(cfg)
