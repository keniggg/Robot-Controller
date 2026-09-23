"""Plane refits must not change the original target association contract."""
import pathlib
import sys

import numpy as np
import pytest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1] / 'src'))
from alicia_grasp_modes.tabletop import Candidate, Segmentation, TargetTracker


def candidate(x=0.0, extent=(0.040, 0.018, 0.006)):
    position = np.array([x, 0.0, 0.288])
    return Candidate(np.ones((8, 8), np.uint8), np.array([160., 120.]),
                     (150, 112, 20, 16), position.copy(), position,
                     np.array(extent), .288, .9)


def result(*candidates, plane=None):
    return Segmentation(list(candidates),
                        np.array([0., 0., 1., -.282] if plane is None else plane), {})


def locked_tracker():
    tracker = TargetTracker()
    initial = candidate()
    chosen, _ = tracker.choose(result(initial), (240, 320))
    assert chosen is initial
    return tracker, initial


def test_current_plane_never_replaces_initial_plane_anchor_or_size_reference():
    tracker, initial = locked_tracker()
    immutable_plane = tracker.plane_base.copy()
    current = candidate(.005, (.050, .022, .006))
    chosen, _ = tracker.choose(result(current, plane=[0., .02, .9998, -.283]), (240, 320))
    assert chosen is current
    assert tracker.anchor is initial
    np.testing.assert_array_equal(tracker.plane_base, immutable_plane)
    np.testing.assert_array_equal(tracker.anchor.extent_m, [.040, .018, .006])


def test_repeated_small_motion_cannot_accumulate_past_original_anchor_bound():
    tracker, _ = locked_tracker()
    for x in (.01, .02):
        assert tracker.choose(result(candidate(x)), (240, 320))[0] is not None
    selected, _ = tracker.choose(result(candidate(.03)), (240, 320))
    assert selected is None
    assert tracker.lost


def test_opposite_sides_of_anchor_still_respect_single_step_bound():
    tracker, _ = locked_tracker()
    assert tracker.choose(result(candidate(-.02)), (240, 320))[0] is not None
    selected, _ = tracker.choose(result(candidate(.02)), (240, 320))
    assert selected is None
    assert tracker.lost


@pytest.mark.parametrize('extent', [(0.040, 0.0361, .006), (0.0199, .018, .006)])
def test_plane_change_does_not_relax_extent_ratios(extent):
    tracker, _ = locked_tracker()
    selected, _ = tracker.choose(result(candidate(.005, extent)), (240, 320))
    assert selected is None
    assert tracker.lost
    # A later better segmentation cannot silently reacquire the lost instance.
    assert tracker.choose(result(candidate()), (240, 320))[0] is None


def test_two_eligible_instances_remain_ambiguous_even_when_one_is_closer():
    tracker, _ = locked_tracker()
    selected, reason = tracker.choose(result(candidate(.001), candidate(.020)), (240, 320))
    assert selected is None
    assert reason == 'ambiguous_locked_instances'
    assert tracker.lost
