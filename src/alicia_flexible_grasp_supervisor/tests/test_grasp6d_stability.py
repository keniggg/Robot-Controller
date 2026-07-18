import math
import pathlib
import sys

import numpy as np
import pytest


ROOT = pathlib.Path(__file__).resolve().parents[1]
if str(ROOT / 'src') not in sys.path:
    sys.path.insert(0, str(ROOT / 'src'))

from alicia_flexible_grasp.grasp.grasp6d_stability import (
    CandidateObservation,
    CandidateTracker,
    TrackingConfig,
    parallel_jaw_orientation_distance_rad,
    quaternion_angle_rad,
    weighted_median,
)


def identity_quaternion():
    return (0.0, 0.0, 0.0, 1.0)


def tool_rz180():
    return (0.0, 0.0, 1.0, 0.0)


def tool_rx180():
    return (1.0, 0.0, 0.0, 0.0)


def multiply_xyzw(first, second):
    x1, y1, z1, w1 = first
    x2, y2, z2, w2 = second
    return (
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
    )


def observation(
    request_id,
    *,
    target_epoch=3,
    target_label='carton',
    model_choice='carton_segmentation',
    center_x=0.100,
    tool_x=None,
    quaternion=None,
    approach=(0.0, 0.0, -1.0),
    width_m=0.040,
    model_width_m=None,
    model_score=0.8,
    geometry_margin_m=0.012,
    pre_moveit_score=0.25,
    payload=None,
):
    if tool_x is None:
        tool_x = center_x
    if quaternion is None:
        quaternion = identity_quaternion()
    if model_width_m is None:
        model_width_m = width_m - 0.002
    if payload is None:
        payload = {'request_id': request_id}
    return CandidateObservation(
        request_id=request_id,
        snapshot_stamp_sec=100.0 + request_id,
        target_epoch=target_epoch,
        target_label=target_label,
        model_choice=model_choice,
        center_base_xyz=(center_x, 0.0, 0.4),
        tool0_position_xyz=(tool_x, 0.0, 0.4),
        quaternion_xyzw=quaternion,
        approach_base_xyz=approach,
        required_open_width_m=width_m,
        model_width_m=model_width_m,
        model_score=model_score,
        geometry_margin_m=geometry_margin_m,
        pre_moveit_score=pre_moveit_score,
        payload=payload,
    )


def two_hit_config(**overrides):
    values = {
        'window_size': 5,
        'min_hits': 2,
        'position_threshold_m': 0.025,
        'orientation_threshold_deg': 25.0,
        'approach_threshold_deg': 20.0,
        'width_threshold_m': 0.008,
    }
    values.update(overrides)
    return TrackingConfig(**values)


def test_candidate_track_requires_three_distinct_hits_in_last_five_requests():
    tracker = CandidateTracker(TrackingConfig(window_size=5, min_hits=3))

    assert tracker.update(1, [observation(1, center_x=0.100)]) == []
    assert tracker.update(2, [observation(2, center_x=0.104)]) == []
    stable = tracker.update(3, [observation(3, center_x=0.099)])

    assert len(stable) == 1
    assert stable[0].hit_count == 3
    assert stable[0].window_count == 3
    assert stable[0].request_id == 3


def test_same_request_cannot_count_two_candidates_as_two_hits():
    tracker = CandidateTracker(TrackingConfig(window_size=5, min_hits=3))

    tracker.update(
        1,
        [observation(1, center_x=0.100), observation(1, center_x=0.101)],
    )
    tracker.update(2, [observation(2, center_x=0.100)])

    assert tracker.stable_candidates() == []


def test_parallel_jaw_local_rz180_is_same_physical_track():
    tracker = CandidateTracker(
        two_hit_config(orientation_threshold_deg=5.0)
    )
    tracker.update(1, [observation(1, quaternion=identity_quaternion())])

    stable = tracker.update(2, [observation(2, quaternion=tool_rz180())])

    assert len(stable) == 1


def test_parallel_jaw_symmetry_is_composed_in_the_local_tool_frame():
    tool_rx90 = (
        math.sin(math.radians(45.0)),
        0.0,
        0.0,
        math.cos(math.radians(45.0)),
    )
    local_half_turn = multiply_xyzw(tool_rx90, tool_rz180())
    global_half_turn = multiply_xyzw(tool_rz180(), tool_rx90)

    local_angle, _aligned = parallel_jaw_orientation_distance_rad(
        tool_rx90, local_half_turn
    )
    global_angle, _aligned = parallel_jaw_orientation_distance_rad(
        tool_rx90, global_half_turn
    )

    assert local_angle == pytest.approx(0.0)
    assert global_angle > math.radians(90.0)


def test_parallel_jaw_fused_representation_stays_continuous_across_window_shift():
    tracker = CandidateTracker(TrackingConfig(window_size=5, min_hits=3))
    fused_by_request = {}

    for request_id in range(1, 7):
        quaternion = (
            identity_quaternion()
            if request_id % 2 == 1
            else tool_rz180()
        )
        stable = tracker.update(
            request_id,
            [observation(request_id, quaternion=quaternion)],
        )
        if stable:
            fused_by_request[request_id] = stable[0].quaternion_xyzw

    assert quaternion_angle_rad(
        fused_by_request[5], fused_by_request[6]
    ) < math.radians(2.0)


def test_arbitrary_180_degree_rotation_is_not_parallel_jaw_equivalent():
    angle, _aligned = parallel_jaw_orientation_distance_rad(
        identity_quaternion(), tool_rx180()
    )
    assert angle == pytest.approx(math.pi)

    tracker = CandidateTracker(
        two_hit_config(orientation_threshold_deg=5.0)
    )
    tracker.update(1, [observation(1, quaternion=identity_quaternion())])
    assert tracker.update(
        2, [observation(2, quaternion=tool_rx180())]
    ) == []


@pytest.mark.parametrize(
    'changed',
    [
        {'target_epoch': 4},
        {'target_label': 'other-carton'},
        {'model_choice': 'generic_object'},
        {'center_x': 0.126},
        {'quaternion': (0.0, 0.0, math.sin(math.radians(13.0)),
                        math.cos(math.radians(13.0)))},
        {'approach': (math.sin(math.radians(21.0)), 0.0,
                      -math.cos(math.radians(21.0)))},
        {'width_m': 0.049},
    ],
)
def test_target_identity_pose_approach_and_width_are_hard_match_gates(changed):
    tracker = CandidateTracker(two_hit_config())
    tracker.update(1, [observation(1)])

    assert tracker.update(2, [observation(2, **changed)]) == []


@pytest.mark.parametrize(
    'changed_identity',
    [
        {'target_epoch': 4},
        {'target_label': 'replacement-carton'},
        {'model_choice': 'replacement-model'},
    ],
)
def test_active_target_identity_switch_removes_old_stable_tracks(changed_identity):
    tracker = CandidateTracker(TrackingConfig(window_size=5, min_hits=3))
    for request_id in (1, 2, 3):
        tracker.update(request_id, [observation(request_id)])
    assert len(tracker.stable_candidates()) == 1

    assert tracker.update(4, [observation(4, **changed_identity)]) == []
    assert tracker.update(5, [observation(5, **changed_identity)]) == []
    stable = tracker.update(6, [observation(6, **changed_identity)])

    assert len(stable) == 1
    assert stable[0].request_id == 6
    assert stable[0].target_epoch == changed_identity.get('target_epoch', 3)
    assert stable[0].target_label == changed_identity.get(
        'target_label', 'carton'
    )
    assert stable[0].model_choice == changed_identity.get(
        'model_choice', 'carton_segmentation'
    )


def test_mixed_target_identity_batch_is_rejected_without_consuming_request():
    tracker = CandidateTracker(two_hit_config())
    tracker.update(1, [observation(1)])

    with pytest.raises(ValueError, match='single target identity'):
        tracker.update(
            2,
            [observation(2), observation(2, target_epoch=4)],
        )

    stable = tracker.update(2, [observation(2)])
    assert len(stable) == 1
    assert stable[0].hit_request_ids == (1, 2)


def test_explicit_target_identity_switch_clears_old_stable_on_empty_batch():
    tracker = CandidateTracker(TrackingConfig(window_size=5, min_hits=3))
    old_identity = (3, 'carton', 'carton_segmentation')
    new_identity = (4, 'carton', 'carton_segmentation')
    for request_id in (1, 2, 3):
        tracker.update(
            request_id,
            [observation(request_id)],
            target_identity=old_identity,
        )
    assert len(tracker.stable_candidates()) == 1

    assert tracker.update(4, [], target_identity=new_identity) == []
    assert tracker.stable_candidates() == []
    assert tracker.update(
        5,
        [observation(5, target_epoch=4)],
        target_identity=new_identity,
    ) == []


def test_explicit_identity_must_match_nonempty_batch_before_state_changes():
    tracker = CandidateTracker(two_hit_config())
    old_identity = (3, 'carton', 'carton_segmentation')
    new_identity = (4, 'carton', 'carton_segmentation')
    tracker.update(1, [observation(1)], target_identity=old_identity)

    with pytest.raises(ValueError, match='target_identity'):
        tracker.update(
            2,
            [observation(2)],
            target_identity=new_identity,
        )

    stable = tracker.update(
        2,
        [observation(2)],
        target_identity=old_identity,
    )
    assert len(stable) == 1
    assert stable[0].hit_request_ids == (1, 2)


@pytest.mark.parametrize(
    'invalid_identity',
    [
        [3, 'carton', 'carton_segmentation'],
        (3, 'carton'),
        (True, 'carton', 'carton_segmentation'),
        (-1, 'carton', 'carton_segmentation'),
        (3, '', 'carton_segmentation'),
        (3, 'carton', ''),
    ],
)
def test_explicit_target_identity_validation_is_fail_closed(invalid_identity):
    tracker = CandidateTracker()

    with pytest.raises(ValueError, match='target_identity'):
        tracker.update(1, [], target_identity=invalid_identity)

    assert tracker.update(
        1,
        [],
        target_identity=(3, 'carton', 'carton_segmentation'),
    ) == []


def test_track_ids_remain_unique_and_monotonic_across_identity_resets():
    tracker = CandidateTracker(TrackingConfig(window_size=5, min_hits=1))
    first_identity = (3, 'carton', 'carton_segmentation')
    second_identity = (4, 'carton', 'carton_segmentation')
    third_identity = (5, 'replacement', 'replacement_model')

    first = tracker.update(
        1,
        [observation(1)],
        target_identity=first_identity,
    )[0]
    assert tracker.update(2, [], target_identity=second_identity) == []
    second = tracker.update(
        3,
        [observation(3, target_epoch=4)],
        target_identity=second_identity,
    )[0]
    third = tracker.update(
        4,
        [
            observation(
                4,
                target_epoch=5,
                target_label='replacement',
                model_choice='replacement_model',
            )
        ],
        target_identity=third_identity,
    )[0]

    assert [first.track_id, second.track_id, third.track_id] == [1, 2, 3]


def test_assignment_is_deterministic_and_each_observation_hits_at_most_one_track():
    tracker = CandidateTracker(two_hit_config(position_threshold_m=0.040))
    tracker.update(
        1,
        [observation(1, center_x=0.09375), observation(1, center_x=0.15625)],
    )

    stable = tracker.update(2, [observation(2, center_x=0.125)])

    assert len(stable) == 1
    assert stable[0].track_id == 1
    assert stable[0].hit_count == 2


def test_tracker_rejects_duplicate_or_non_increasing_request_ids_atomically():
    tracker = CandidateTracker(two_hit_config())
    tracker.update(2, [observation(2)])

    with pytest.raises(ValueError, match='strictly increasing'):
        tracker.update(2, [observation(2)])
    with pytest.raises(ValueError, match='strictly increasing'):
        tracker.update(1, [observation(1)])

    assert tracker.stable_candidates() == []


def test_tracker_rejects_observation_from_a_different_request():
    tracker = CandidateTracker(two_hit_config())

    with pytest.raises(ValueError, match='request_id'):
        tracker.update(2, [observation(1)])

    assert tracker.stable_candidates() == []


def test_tracks_age_out_of_the_five_request_window():
    tracker = CandidateTracker(TrackingConfig(window_size=5, min_hits=3))
    for request_id in (1, 2, 3):
        tracker.update(request_id, [observation(request_id)])

    assert len(tracker.stable_candidates()) == 1
    tracker.update(4, [])
    tracker.update(5, [])
    assert len(tracker.stable_candidates()) == 1

    tracker.update(6, [])
    assert tracker.stable_candidates() == []
    tracker.update(7, [])
    tracker.update(8, [])
    assert tracker.track_count == 0


def test_observation_defensively_copies_normalizes_and_freezes_numpy_inputs():
    center = np.array([0.1, 0.0, 0.4], dtype=np.float64)
    position = np.array([0.11, 0.0, 0.4], dtype=np.float64)
    quaternion = np.array([0.0, 0.0, 0.0, -2.0], dtype=np.float64)
    approach = np.array([0.0, 0.0, -4.0], dtype=np.float64)
    item = CandidateObservation(
        request_id=1,
        snapshot_stamp_sec=101.0,
        target_epoch=3,
        target_label='carton',
        model_choice='carton_segmentation',
        center_base_xyz=center,
        tool0_position_xyz=position,
        quaternion_xyzw=quaternion,
        approach_base_xyz=approach,
        required_open_width_m=0.04,
        model_width_m=0.038,
        model_score=0.8,
        geometry_margin_m=0.012,
        pre_moveit_score=0.25,
        payload=None,
    )
    center[0] = 9.0
    position[0] = 9.0
    quaternion[3] = 9.0
    approach[2] = 9.0

    assert item.center_base_xyz.tolist() == [0.1, 0.0, 0.4]
    assert item.tool0_position_xyz.tolist() == [0.11, 0.0, 0.4]
    assert item.quaternion_xyzw.tolist() == [0.0, 0.0, 0.0, 1.0]
    assert item.approach_base_xyz.tolist() == [0.0, 0.0, -1.0]
    for value in (
        item.center_base_xyz,
        item.tool0_position_xyz,
        item.quaternion_xyzw,
        item.approach_base_xyz,
    ):
        assert value.flags.writeable is False
        with pytest.raises(ValueError):
            value[0] = 0.0


@pytest.mark.parametrize(
    ('kwargs', 'attribute'),
    [
        (
            {'quaternion': (1e308, -1e308, 1e308, -1e308)},
            'quaternion_xyzw',
        ),
        (
            {'approach': (1e308, -1e308, 1e308)},
            'approach_base_xyz',
        ),
    ],
)
def test_observation_normalizes_extreme_finite_orientation_vectors(kwargs, attribute):
    item = observation(1, **kwargs)
    vector = getattr(item, attribute)

    assert np.all(np.isfinite(vector))
    assert float(np.linalg.norm(vector)) == pytest.approx(1.0)


@pytest.mark.parametrize(
    ('field', 'value'),
    [
        ('center_base_xyz', (0.0, 1.0)),
        ('tool0_position_xyz', (0.0, 1.0, math.nan)),
        ('quaternion_xyzw', (0.0, 0.0, 0.0)),
        ('quaternion_xyzw', (0.0, 0.0, 0.0, 0.0)),
        ('approach_base_xyz', (0.0, 0.0, math.inf)),
        ('approach_base_xyz', (0.0, 0.0, 0.0)),
        ('snapshot_stamp_sec', math.nan),
        ('required_open_width_m', math.inf),
        ('model_score', math.nan),
        ('geometry_margin_m', math.inf),
        ('pre_moveit_score', math.nan),
    ],
)
def test_observation_rejects_bad_dimensions_non_finite_and_zero_norm(field, value):
    values = {
        'request_id': 1,
        'snapshot_stamp_sec': 101.0,
        'target_epoch': 3,
        'target_label': 'carton',
        'model_choice': 'carton_segmentation',
        'center_base_xyz': (0.1, 0.0, 0.4),
        'tool0_position_xyz': (0.1, 0.0, 0.4),
        'quaternion_xyzw': identity_quaternion(),
        'approach_base_xyz': (0.0, 0.0, -1.0),
        'required_open_width_m': 0.04,
        'model_width_m': 0.038,
        'model_score': 0.8,
        'geometry_margin_m': 0.012,
        'pre_moveit_score': 0.25,
        'payload': None,
    }
    values[field] = value

    with pytest.raises(ValueError, match=field):
        CandidateObservation(**values)


def test_quaternion_helpers_validate_normalize_and_align_local_symmetry():
    assert quaternion_angle_rad(
        (0.0, 0.0, 0.0, 2.0), (0.0, 0.0, 0.0, -3.0)
    ) == pytest.approx(0.0)
    angle, aligned = parallel_jaw_orientation_distance_rad(
        identity_quaternion(), tool_rz180()
    )
    assert angle == pytest.approx(0.0)
    assert quaternion_angle_rad(aligned, identity_quaternion()) == pytest.approx(0.0)

    with pytest.raises(ValueError, match='quaternion'):
        quaternion_angle_rad(identity_quaternion(), (math.nan, 0.0, 0.0, 1.0))


def test_weighted_median_validates_inputs_and_uses_weights():
    assert weighted_median([0.0, 10.0, 20.0], [1.0, 8.0, 1.0]) == 10.0
    with pytest.raises(ValueError, match='same non-zero length'):
        weighted_median([1.0], [1.0, 2.0])
    with pytest.raises(ValueError, match='positive'):
        weighted_median([1.0, 2.0], [1.0, 0.0])
    with pytest.raises(ValueError, match='finite'):
        weighted_median([1.0, math.nan], [1.0, 1.0])


def test_fusion_uses_robust_position_width_and_parallel_jaw_orientation():
    tracker = CandidateTracker(TrackingConfig(window_size=5, min_hits=3))
    tracker.update(
        1,
        [
            observation(
                1,
                center_x=0.100,
                tool_x=0.100,
                width_m=0.040,
                model_score=0.80,
            )
        ],
    )
    tracker.update(
        2,
        [
            observation(
                2,
                center_x=0.101,
                tool_x=0.101,
                width_m=0.041,
                quaternion=(
                    0.0,
                    0.0,
                    math.sin(math.radians(0.5)),
                    math.cos(math.radians(0.5)),
                ),
                model_score=0.81,
            )
        ],
    )
    stable = tracker.update(
        3,
        [
            observation(
                3,
                center_x=0.102,
                tool_x=0.140,
                width_m=0.048,
                quaternion=tool_rz180(),
                model_score=0.99,
            )
        ],
    )

    fused = stable[0]
    assert fused.center_base_xyz[0] < 0.110
    assert fused.tool0_position_xyz[0] < 0.110
    assert fused.required_open_width_m < 0.045
    assert fused.model_width_m < 0.043
    assert quaternion_angle_rad(
        fused.quaternion_xyzw, identity_quaternion()
    ) < math.radians(2.0)
    assert fused.tool0_position_xyz.flags.writeable is False
    assert fused.quaternion_xyzw.flags.writeable is False


def test_orientation_medoid_downweights_one_angular_outlier():
    tracker = CandidateTracker(TrackingConfig(window_size=5, min_hits=3))
    rotations_deg = (0.0, 1.0, 20.0)
    for request_id, angle_deg in enumerate(rotations_deg, start=1):
        tracker.update(
            request_id,
            [
                observation(
                    request_id,
                    quaternion=(
                        math.sin(math.radians(angle_deg / 2.0)),
                        0.0,
                        0.0,
                        math.cos(math.radians(angle_deg / 2.0)),
                    ),
                )
            ],
        )

    fused = tracker.stable_candidates()[0]
    assert quaternion_angle_rad(
        fused.quaternion_xyzw, identity_quaternion()
    ) < math.radians(4.0)


def test_fusion_retains_robust_scores_conservative_margin_and_newest_payload():
    tracker = CandidateTracker(TrackingConfig(window_size=5, min_hits=3))
    payloads = [object(), object(), object()]
    observations = (
        observation(
            1,
            model_score=0.70,
            geometry_margin_m=0.010,
            pre_moveit_score=0.30,
            payload=payloads[0],
        ),
        observation(
            2,
            model_score=0.80,
            geometry_margin_m=0.012,
            pre_moveit_score=0.20,
            payload=payloads[1],
        ),
        observation(
            3,
            model_score=0.99,
            geometry_margin_m=0.100,
            pre_moveit_score=2.00,
            payload=payloads[2],
        ),
    )
    for item in observations:
        tracker.update(item.request_id, [item])

    fused = tracker.stable_candidates()[0]
    assert fused.model_score == pytest.approx(0.80)
    assert fused.pre_moveit_score == pytest.approx(0.30)
    assert fused.geometry_margin_m == pytest.approx(0.011)
    assert fused.request_id == 3
    assert fused.snapshot_stamp_sec == pytest.approx(103.0)
    assert fused.payload is payloads[2]
    assert fused.moveit_cost is None


def test_fusion_scales_extreme_but_finite_model_scores_before_weighting():
    tracker = CandidateTracker(TrackingConfig(window_size=5, min_hits=3))

    for request_id in (1, 2, 3):
        stable = tracker.update(
            request_id,
            [observation(request_id, model_score=1e200)],
        )

    assert len(stable) == 1
    assert stable[0].model_score == pytest.approx(1e200)
    assert np.all(np.isfinite(stable[0].quaternion_xyzw))


def test_failed_fusion_rolls_back_update_and_allows_same_request_retry(monkeypatch):
    tracker = CandidateTracker(
        TrackingConfig(window_size=5, min_hits=1)
    )
    before = tracker.update(1, [observation(1, center_x=0.100)])[0]
    previous_fused_orientation = tracker._tracks[1]._last_fused_quaternion
    original_fuse = tracker._fuse
    injected_orientation = np.asarray(tool_rz180(), dtype=np.float64)
    injected_orientation.setflags(write=False)

    def fail_fusion(track):
        track._last_fused_quaternion = injected_orientation
        raise RuntimeError('synthetic fusion failure')

    monkeypatch.setattr(tracker, '_fuse', fail_fusion)
    with pytest.raises(RuntimeError, match='synthetic fusion failure'):
        tracker.update(
            2,
            [
                observation(2, target_epoch=4, center_x=0.100),
                observation(2, target_epoch=4, center_x=0.200),
            ],
        )
    monkeypatch.setattr(tracker, '_fuse', original_fuse)

    assert tracker.track_count == 1
    assert tracker._tracks[1]._last_fused_quaternion is previous_fused_orientation
    after = tracker.stable_candidates()[0]
    assert after.hit_request_ids == before.hit_request_ids == (1,)
    assert after.request_id == before.request_id == 1
    assert after.target_epoch == before.target_epoch == 3
    assert after.window_count == before.window_count == 1

    retried = tracker.update(
        2,
        [
            observation(2, center_x=0.100),
            observation(2, center_x=0.200),
        ],
    )
    assert [item.track_id for item in retried] == [1, 2]
    assert retried[0].hit_request_ids == (1, 2)
    assert retried[1].hit_request_ids == (2,)
    assert tracker.track_count == 2


def test_stable_candidates_public_query_does_not_mutate_continuity_state():
    tracker = CandidateTracker(TrackingConfig(window_size=5, min_hits=1))
    tracker.update(
        1,
        [
            observation(1, center_x=0.100),
            observation(1, center_x=0.200, quaternion=tool_rz180()),
        ],
    )
    before = {
        track_id: track._last_fused_quaternion
        for track_id, track in tracker._tracks.items()
    }

    stable = tracker.stable_candidates()

    assert [item.track_id for item in stable] == [1, 2]
    assert all(
        tracker._tracks[track_id]._last_fused_quaternion is quaternion
        for track_id, quaternion in before.items()
    )


def test_second_track_query_failure_cannot_partially_commit_first_track(monkeypatch):
    tracker = CandidateTracker(TrackingConfig(window_size=5, min_hits=1))
    tracker.update(
        1,
        [
            observation(1, center_x=0.100),
            observation(1, center_x=0.200, quaternion=tool_rz180()),
        ],
    )
    before = {
        track_id: track._last_fused_quaternion
        for track_id, track in tracker._tracks.items()
    }
    original_fuse = tracker._fuse
    calls = []

    def fail_second_fusion(track):
        fused = original_fuse(track)
        calls.append(track.track_id)
        if len(calls) == 2:
            raise RuntimeError('synthetic second-track fusion failure')
        return fused

    monkeypatch.setattr(tracker, '_fuse', fail_second_fusion)
    with pytest.raises(RuntimeError, match='second-track fusion failure'):
        tracker.stable_candidates()
    monkeypatch.setattr(tracker, '_fuse', original_fuse)

    assert calls == [1, 2]
    assert all(
        tracker._tracks[track_id]._last_fused_quaternion is quaternion
        for track_id, quaternion in before.items()
    )
    assert [item.track_id for item in tracker.stable_candidates()] == [1, 2]


def test_single_low_side_outlier_stays_below_weighted_median_cutoff():
    tracker = CandidateTracker(TrackingConfig(window_size=5, min_hits=3))
    items = (
        observation(1, center_x=0.100, tool_x=0.000, model_score=4.0),
        observation(2, center_x=0.100, tool_x=0.100, model_score=1.0),
        observation(3, center_x=0.100, tool_x=0.100, model_score=1.0),
    )

    weights = tracker._fusion_weights(items)
    assert np.all(np.isfinite(weights))
    assert np.all(weights > 0.0)
    assert math.fsum(float(weight) for weight in weights) == pytest.approx(1.0)
    assert float(np.max(weights)) < 0.5

    for item in items:
        stable = tracker.update(item.request_id, [item])

    assert stable[0].tool0_position_xyz[0] == pytest.approx(0.100)
