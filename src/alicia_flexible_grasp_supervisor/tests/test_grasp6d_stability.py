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
