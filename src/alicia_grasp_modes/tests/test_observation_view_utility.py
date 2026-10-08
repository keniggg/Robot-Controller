import math
import pytest
from alicia_grasp_modes.observation_view_utility import (
    predicted_information_utility as info, information_motion_rank as rank)


def test_gen4_barely_above_noise_loses_to_informative_view_despite_shorter_move():
    weak = info(.00301137, .003, .38176, 396.)
    useful = info(.012, .003, .29, 396.)
    assert weak['utility'] < .002
    assert rank(useful['utility'], 15.) < rank(weak['utility'], 12.23)
    assert weak['prediction_only'] and not weak['execution_authority']


def test_same_gain_prefers_better_resolved_depth_and_intrinsics():
    assert info(.008, .003, .2, 400.)['utility'] > info(.008, .003, .4, 400.)['utility']
    assert info(.008, .003, .2, 800.)['utility'] > info(.008, .003, .2, 400.)['utility']
    assert info(.008, .003, .2, 400.)['gain_px'] == pytest.approx(16.)


def test_saturation_does_not_buy_large_wrist_flip():
    adequate = info(.03, .003, .25, 400.)['utility']
    more = info(.06, .003, .25, 400.)['utility']
    assert 0. < adequate < more < 1.
    assert rank(adequate, 5., .2, .4) < rank(more, 16., 2., 4.)
    assert rank(adequate, 5., .2, .4) < rank(adequate, 6., .2, .4)


def test_no_excess_information_is_not_fabricated():
    assert info(.003, .003, .3, 400.)['utility'] == 0.
    assert math.isinf(rank(0., 0.)[0])


@pytest.mark.parametrize('field', range(4))
@pytest.mark.parametrize('bad', [float('nan'), float('inf'), -1., True, '1'])
def test_invalid_information_inputs_fail(field, bad):
    args = [.01, .003, .25, 400.]; args[field] = bad
    with pytest.raises(ValueError): info(*args)


@pytest.mark.parametrize('args', [(1.01, 1.), (.5, -1.), (.5, float('nan')), (True, 1.)])
def test_invalid_motion_score_fails(args):
    with pytest.raises(ValueError): rank(*args)
