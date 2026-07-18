import pathlib
import sys
import threading
import time
import types

import numpy as np

ROOT = pathlib.Path(__file__).resolve().parents[1]
if str(ROOT / 'src') not in sys.path:
    sys.path.insert(0, str(ROOT / 'src'))

from alicia_flexible_grasp.vision.rgbd_snapshot import (
    DepthQuality,
    RgbdSample,
    SnapshotResult,
    SynchronizedRgbdBuffer,
    fuse_stable_samples,
    mask_iou,
)


FUSION_KWARGS = {
    'require_mask': True,
    'min_mask_iou': 0.85,
    'max_centroid_shift_px': 5.0,
    'max_joint_delta_rad': 0.01,
    'erosion_px': 2,
    'depth_scale': 0.0001,
    'depth_min_m': 0.03,
    'depth_max_m': 2.0,
    'mad_scale': 3.5,
    'mad_absolute_floor_m': 0.002,
    'internal_hole_max_area_px': 25,
}


def sample(depth_value, mask, stamp, joints=None, bbox=(5, 4, 20, 12)):
    depth = (
        np.full((20, 30), depth_value, dtype=np.uint16)
        if np.isscalar(depth_value)
        else np.asarray(depth_value, dtype=np.uint16).copy()
    )
    return RgbdSample(
        color_bgr=np.zeros((20, 30, 3), dtype=np.uint8),
        depth_raw=depth,
        object_mask=None if mask is None else mask.copy(),
        bbox=bbox,
        object_msg=types.SimpleNamespace(detected=True),
        stamp_sec=float(stamp),
        frame_id='camera_link',
        joint_positions=np.asarray(joints or [0.0] * 6, dtype=float),
    )


def stable_mask():
    mask = np.zeros((20, 30), dtype=np.uint8)
    mask[5:15, 8:22] = 255
    return mask


def fuse(samples, **overrides):
    kwargs = dict(FUSION_KWARGS)
    kwargs.update(overrides)
    return fuse_stable_samples(samples, **kwargs)


def synchronized_buffer_at(source_now_ns, monotonic_clock=None):
    return SynchronizedRgbdBuffer(
        source_clock_ns=lambda: int(source_now_ns),
        monotonic_clock=monotonic_clock,
    )


def add_complete_sample(buffer, stamp):
    color = np.zeros((3, 4, 3), dtype=np.uint8)
    depth = np.full((3, 4), 2200, dtype=np.uint16)
    mask = np.ones((3, 4), dtype=np.uint8) * 255
    detected = types.SimpleNamespace(
        detected=True,
        bbox_x=0,
        bbox_y=0,
        bbox_width=4,
        bbox_height=3,
    )
    buffer.update_joints([0.0] * 6)
    buffer.update_color(color, stamp, 'camera_link')
    buffer.update_depth(depth, stamp, 'camera_link')
    buffer.update_mask(mask, stamp, 'camera_link')
    buffer.update_object(detected, stamp)


def add_identity_bound_sample(buffer, stamp, target_identity):
    color = np.zeros((3, 4, 3), dtype=np.uint8)
    depth = np.full((3, 4), 2200, dtype=np.uint16)
    mask = np.ones((3, 4), dtype=np.uint8) * 255
    detected = types.SimpleNamespace(
        detected=True,
        label=target_identity[1],
        bbox_x=0,
        bbox_y=0,
        bbox_width=4,
        bbox_height=3,
    )
    buffer.update_joints([0.0] * 6)
    buffer.update_color(color, stamp, 'camera_link')
    buffer.update_depth(depth, stamp, 'camera_link')
    buffer.update_mask(mask, stamp, 'camera_link')
    buffer.update_object(
        detected,
        stamp,
        target_epoch=target_identity[0],
        target_identity=target_identity,
    )


def test_zero_timeout_polls_accumulate_low_rate_exact_samples_persistently():
    source_now_ns = [10_000_000_000]
    monotonic_now = [10.0]
    buffer = SynchronizedRgbdBuffer(
        source_clock_ns=lambda: source_now_ns[0],
        monotonic_clock=lambda: monotonic_now[0],
    )
    identity = (3, 'carton', 'carton_segment')

    for index, stamp_ns in enumerate(
        (10_000_000_000, 10_670_000_000, 11_340_000_000)
    ):
        source_now_ns[0] = stamp_ns
        monotonic_now[0] = 10.0 + 0.67 * index
        add_identity_bound_sample(buffer, stamp_ns * 1e-9, identity)
        samples = buffer.wait_for_samples(
            3,
            0.0,
            require_mask=True,
            max_age_sec=0.35,
            collection_span_sec=3.0,
            max_inference_latency_sec=1.2,
            target_identity=identity,
        )

    assert [item.stamp_ns for item in samples] == [
        10_000_000_000,
        10_670_000_000,
        11_340_000_000,
    ]
    assert all(item.target_identity == identity for item in samples)


def test_persistent_window_never_mixes_target_identity_epochs():
    source_now_ns = [20_000_000_000]
    monotonic_now = [20.0]
    buffer = SynchronizedRgbdBuffer(
        source_clock_ns=lambda: source_now_ns[0],
        monotonic_clock=lambda: monotonic_now[0],
    )
    old_identity = (7, 'carton', 'carton_segment')
    new_identity = (8, 'carton', 'carton_segment')

    for index, identity in enumerate(
        (old_identity, old_identity, new_identity, new_identity)
    ):
        stamp_ns = 20_000_000_000 + index * 400_000_000
        source_now_ns[0] = stamp_ns
        monotonic_now[0] = 20.0 + index * 0.4
        add_identity_bound_sample(buffer, stamp_ns * 1e-9, identity)
        requested_identity = old_identity if index < 2 else new_identity
        samples = buffer.wait_for_samples(
            3,
            0.0,
            require_mask=True,
            max_age_sec=0.35,
            collection_span_sec=3.0,
            max_inference_latency_sec=1.2,
            target_identity=requested_identity,
        )
        assert samples == []

    stamp_ns = 21_600_000_000
    source_now_ns[0] = stamp_ns
    monotonic_now[0] = 21.6
    add_identity_bound_sample(buffer, stamp_ns * 1e-9, new_identity)
    samples = buffer.wait_for_samples(
        3,
        0.0,
        require_mask=True,
        max_age_sec=0.35,
        collection_span_sec=3.0,
        max_inference_latency_sec=1.2,
        target_identity=new_identity,
    )

    assert [item.target_identity for item in samples] == [new_identity] * 3
    assert list(buffer._collection_windows) == [(True, new_identity)]
    assert [item.stamp_ns for item in samples] == [
        20_800_000_000,
        21_200_000_000,
        21_600_000_000,
    ]


def test_mask_iou_uses_binary_overlap():
    first = np.zeros((4, 5), dtype=np.uint8)
    second = np.zeros((4, 5), dtype=np.uint8)
    first[1:3, 1:4] = 1
    second[1:3, 2:5] = 255

    assert mask_iou(first, second) == 0.5


def test_three_stable_masks_fuse_by_pixel_median():
    mask = stable_mask()
    result = fuse(
        [sample(2190, mask, 1.00), sample(2200, mask, 1.03), sample(2210, mask, 1.06)]
    )

    assert result.ok
    assert result.quality.fused_frames == 3
    assert int(np.median(result.depth_raw[result.object_mask > 0])) == 2200
    assert result.source_mode == 'instance_mask'
    assert result.stamp_sec == 1.06


def test_joint_motion_rejects_window_instead_of_smearing_depth():
    mask = np.ones((20, 30), dtype=np.uint8) * 255
    moving = sample(2200, mask, 1.03, joints=[0.02, 0, 0, 0, 0, 0])
    result = fuse([sample(2190, mask, 1.00), moving, sample(2210, mask, 1.06)])

    assert not result.ok
    assert result.failure_code == 'DEPTH_UNSTABLE'
    assert 'joint' in result.failure_reason


def test_mask_iou_below_threshold_rejects_window():
    first = stable_mask()
    shifted = np.zeros_like(first)
    shifted[5:15, 18:29] = 255

    result = fuse([sample(2200, first, 1.00), sample(2200, shifted, 1.03), sample(2200, shifted, 1.06)])

    assert not result.ok
    assert result.failure_code == 'DEPTH_UNSTABLE'
    assert 'IoU' in result.failure_reason


def test_centroid_shift_above_threshold_rejects_even_with_permissive_iou():
    first = stable_mask()
    shifted = np.zeros_like(first)
    shifted[5:15, 14:28] = 255

    result = fuse(
        [sample(2200, first, 1.00), sample(2200, shifted, 1.03), sample(2200, shifted, 1.06)],
        min_mask_iou=0.20,
    )

    assert not result.ok
    assert result.failure_code == 'DEPTH_UNSTABLE'
    assert 'centroid' in result.failure_reason


def test_centroid_drift_across_full_window_rejects_even_when_adjacent_steps_are_small():
    masks = []
    for x0 in (4, 8, 12):
        mask = np.zeros((20, 30), dtype=np.uint8)
        mask[5:15, x0:x0 + 14] = 255
        masks.append(mask)

    result = fuse(
        [
            sample(2200, masks[0], 1.00),
            sample(2200, masks[1], 1.03),
            sample(2200, masks[2], 1.06),
        ],
        min_mask_iou=0.20,
        max_centroid_shift_px=5.0,
    )

    assert not result.ok
    assert result.failure_code == 'DEPTH_UNSTABLE'
    assert 'centroid' in result.failure_reason


def test_wrong_mask_shape_returns_size_mismatch():
    wrong = np.ones((10, 10), dtype=np.uint8) * 255
    result = fuse([sample(2200, wrong, 1.00)] * 3)

    assert not result.ok
    assert result.failure_code == 'MASK_SIZE_MISMATCH'


def test_detect_mode_fuses_bbox_without_masks():
    samples = [
        sample(2190, None, 1.00, bbox=(5, 4, 20, 12)),
        sample(2200, None, 1.03, bbox=(5, 4, 20, 12)),
        sample(2210, None, 1.06, bbox=(5, 4, 20, 12)),
    ]

    result = fuse(samples, require_mask=False)

    assert result.ok
    assert result.source_mode == 'bbox_depth'
    assert result.object_mask is not None
    assert np.count_nonzero(result.object_mask) > 0
    assert np.count_nonzero(result.target_depth_raw) > 0


def test_detect_mode_rejects_unstable_bbox_overlap():
    samples = [
        sample(2200, None, 1.00, bbox=(1, 1, 6, 6)),
        sample(2200, None, 1.03, bbox=(20, 10, 6, 6)),
        sample(2200, None, 1.06, bbox=(20, 10, 6, 6)),
    ]

    result = fuse(samples, require_mask=False)

    assert not result.ok
    assert result.failure_code == 'DEPTH_UNSTABLE'
    assert 'bbox' in result.failure_reason


def test_mask_erosion_removes_edge_depth_from_target_only():
    mask = stable_mask()
    result = fuse([sample(2200, mask, 1.00), sample(2200, mask, 1.03), sample(2200, mask, 1.06)])

    assert result.ok
    assert result.depth_raw[5, 8] == 2200
    assert result.target_depth_raw[5, 8] == 0
    assert result.target_depth_raw[7, 10] == 2200
    assert result.object_mask[5, 8] == 0
    assert result.object_mask[7, 10] == 255


def test_mad_filter_removes_fly_point_from_target_depth():
    mask = stable_mask()
    frames = [np.full((20, 30), 2200, dtype=np.uint16) for _ in range(3)]
    for frame in frames:
        frame[9, 12] = 4000

    result = fuse(
        [sample(frames[0], mask, 1.00), sample(frames[1], mask, 1.03), sample(frames[2], mask, 1.06)]
    )

    assert result.ok
    assert result.depth_raw[9, 12] == 4000
    assert result.target_depth_raw[9, 12] == 0


def test_fully_enclosed_nine_pixel_depth_hole_is_filled():
    mask = np.zeros((20, 30), dtype=np.uint8)
    mask[2:18, 4:26] = 255
    frames = [np.full((20, 30), 2200, dtype=np.uint16) for _ in range(3)]
    for frame in frames:
        frame[8:11, 12:15] = 0

    result = fuse(
        [sample(frames[0], mask, 1.00), sample(frames[1], mask, 1.03), sample(frames[2], mask, 1.06)]
    )

    assert result.ok
    np.testing.assert_array_equal(result.target_depth_raw[8:11, 12:15], np.full((3, 3), 2200))


def test_hole_touching_eroded_mask_edge_remains_zero():
    mask = np.zeros((20, 30), dtype=np.uint8)
    mask[2:18, 4:26] = 255
    frames = [np.full((20, 30), 2200, dtype=np.uint16) for _ in range(3)]
    for frame in frames:
        frame[4:7, 10:13] = 0

    result = fuse(
        [sample(frames[0], mask, 1.00), sample(frames[1], mask, 1.03), sample(frames[2], mask, 1.06)]
    )

    assert result.ok
    np.testing.assert_array_equal(result.target_depth_raw[4:7, 10:13], np.zeros((3, 3)))


def test_hole_touching_image_and_mask_boundary_remains_zero():
    mask = np.ones((20, 30), dtype=np.uint8) * 255
    frames = [np.full((20, 30), 2200, dtype=np.uint16) for _ in range(3)]
    for frame in frames:
        frame[0:2, 10:12] = 0

    result = fuse(
        [sample(frames[0], mask, 1.00), sample(frames[1], mask, 1.03), sample(frames[2], mask, 1.06)],
        erosion_px=0,
    )

    assert result.ok
    np.testing.assert_array_equal(result.target_depth_raw[0:2, 10:12], np.zeros((2, 2)))


def test_missing_joint_snapshot_rejects_window():
    mask = stable_mask()
    samples = [sample(2200, mask, 1.00), sample(2200, mask, 1.03), sample(2200, mask, 1.06)]
    samples[1].joint_positions = np.zeros(0, dtype=float)

    result = fuse(samples)

    assert not result.ok
    assert result.failure_code == 'DEPTH_UNSTABLE'
    assert 'joint' in result.failure_reason


def test_result_arrays_are_defensive_non_writeable_copies():
    mask = stable_mask()
    samples = [sample(2200, mask, 1.00), sample(2200, mask, 1.03), sample(2200, mask, 1.06)]
    result = fuse(samples)

    samples[-1].color_bgr[:] = 77
    samples[-1].depth_raw[:] = 99
    samples[-1].object_mask[:] = 0

    assert not result.color_bgr.flags.writeable
    assert not result.depth_raw.flags.writeable
    assert not result.target_depth_raw.flags.writeable
    assert not result.object_mask.flags.writeable
    assert result.color_bgr[0, 0, 0] == 0
    assert result.depth_raw[0, 0] == 2200
    with np.testing.assert_raises(ValueError):
        result.depth_raw[0, 0] = 0


def test_direct_snapshot_result_construction_freezes_defensive_array_copies():
    color = np.zeros((2, 3, 3), dtype=np.uint8)
    depth = np.ones((2, 3), dtype=np.uint16)
    target = depth.copy()
    mask = np.ones((2, 3), dtype=np.uint8) * 255

    result = SnapshotResult(
        ok=True,
        failure_code='',
        failure_reason='',
        color_bgr=color,
        depth_raw=depth,
        target_depth_raw=target,
        object_mask=mask,
        bbox=(0, 0, 3, 2),
        object_msg=None,
        stamp_sec=1.0,
        frame_id='camera_link',
        quality=DepthQuality(3, 6, 6, 1.0, 0.1, 0.0),
        source_mode='instance_mask',
    )
    color[:] = 7
    depth[:] = 8
    target[:] = 9
    mask[:] = 0

    assert result.color_bgr[0, 0, 0] == 0
    assert result.depth_raw[0, 0] == 1
    assert result.target_depth_raw[0, 0] == 1
    assert result.object_mask[0, 0] == 255
    assert all(
        not array.flags.writeable
        for array in (
            result.color_bgr,
            result.depth_raw,
            result.target_depth_raw,
            result.object_mask,
        )
    )


def test_wait_for_samples_requires_newest_stamp_after_previous_window():
    buffer = SynchronizedRgbdBuffer(
        source_clock_ns=lambda: 10_000_000_000,
        monotonic_clock=lambda: 10.0,
    )
    for stamp in (9.1, 9.2, 9.3):
        add_complete_sample(buffer, stamp)

    first = buffer.wait_for_samples(
        3, 0.01, True, 1.0, newest_after_ns=0,
    )
    repeated = buffer.wait_for_samples(
        3,
        0.01,
        True,
        1.0,
        newest_after_ns=first[-1].stamp_ns,
    )
    add_complete_sample(buffer, 9.4)
    advanced = buffer.wait_for_samples(
        3,
        0.01,
        True,
        1.0,
        newest_after_ns=first[-1].stamp_ns,
    )

    assert repeated == []
    assert [sample.stamp_sec for sample in advanced] == [9.2, 9.3, 9.4]


def test_synchronized_buffer_requires_exact_timestamp_components_and_returns_copies():
    buffer = synchronized_buffer_at(1_100_000_000)
    color = np.zeros((3, 4, 3), dtype=np.uint8)
    depth = np.full((3, 4), 2200, dtype=np.uint16)
    mask = np.ones((3, 4), dtype=np.uint8) * 255
    detected = types.SimpleNamespace(
        detected=True,
        bbox_x=0,
        bbox_y=0,
        bbox_width=4,
        bbox_height=3,
    )
    buffer.update_joints([0.0] * 6)
    for stamp in (1.00, 1.03, 1.06):
        buffer.update_color(color, stamp, 'camera_link')
        buffer.update_depth(depth, stamp + 0.001, 'camera_link')
        buffer.update_mask(mask, stamp, 'camera_link')
        buffer.update_object(detected, stamp)

    assert buffer.wait_for_samples(3, 0.01, require_mask=True, max_age_sec=10.0) == []

    for stamp in (1.00, 1.03, 1.06):
        buffer.update_depth(depth, stamp, 'camera_link')
    samples = buffer.wait_for_samples(3, 0.01, require_mask=True, max_age_sec=10.0)

    assert [item.stamp_sec for item in samples] == [1.00, 1.03, 1.06]
    color[:] = 91
    depth[:] = 92
    mask[:] = 0
    assert samples[0].color_bgr[0, 0, 0] == 0
    assert samples[0].depth_raw[0, 0] == 2200
    assert samples[0].object_mask[0, 0] == 255


def test_detect_buffer_ignores_mask_but_requires_detected_object():
    buffer = synchronized_buffer_at(2_100_000_000)
    color = np.zeros((3, 4, 3), dtype=np.uint8)
    depth = np.full((3, 4), 2200, dtype=np.uint16)
    buffer.update_joints([0.0] * 6)
    for stamp in (2.00, 2.03, 2.06):
        buffer.update_color(color, stamp, 'camera_link')
        buffer.update_depth(depth, stamp, 'camera_link')
        buffer.update_object(
            types.SimpleNamespace(
                detected=stamp != 2.03,
                bbox_x=0,
                bbox_y=0,
                bbox_width=4,
                bbox_height=3,
            ),
            stamp,
        )

    assert buffer.wait_for_samples(3, 0.01, require_mask=False, max_age_sec=10.0) == []

    buffer.update_object(
        types.SimpleNamespace(
            detected=True,
            bbox_x=0,
            bbox_y=0,
            bbox_width=4,
            bbox_height=3,
        ),
        2.03,
    )
    samples = buffer.wait_for_samples(3, 0.01, require_mask=False, max_age_sec=10.0)

    assert len(samples) == 3
    assert all(item.object_mask is None for item in samples)


def test_buffer_rejects_components_from_different_camera_frames():
    buffer = synchronized_buffer_at(2_100_000_000)
    color = np.zeros((3, 4, 3), dtype=np.uint8)
    depth = np.full((3, 4), 2200, dtype=np.uint16)
    detected = types.SimpleNamespace(
        detected=True,
        bbox_x=0,
        bbox_y=0,
        bbox_width=4,
        bbox_height=3,
    )
    buffer.update_joints([0.0] * 6)
    for stamp in (2.00, 2.03, 2.06):
        buffer.update_color(color, stamp, 'camera_link')
        buffer.update_depth(depth, stamp, 'different_camera')
        buffer.update_object(detected, stamp)

    assert buffer.wait_for_samples(3, 0.01, require_mask=False, max_age_sec=10.0) == []


def test_wait_for_samples_blocks_until_three_complete_entries_arrive():
    buffer = synchronized_buffer_at(3_100_000_000)
    color = np.zeros((3, 4, 3), dtype=np.uint8)
    depth = np.full((3, 4), 2200, dtype=np.uint16)
    mask = np.ones((3, 4), dtype=np.uint8) * 255
    detected = types.SimpleNamespace(
        detected=True,
        bbox_x=0,
        bbox_y=0,
        bbox_width=4,
        bbox_height=3,
    )
    buffer.update_joints([0.0] * 6)

    def publish():
        time.sleep(0.02)
        for stamp in (3.00, 3.03, 3.06):
            buffer.update_color(color, stamp, 'camera_link')
            buffer.update_depth(depth, stamp, 'camera_link')
            buffer.update_mask(mask, stamp, 'camera_link')
            buffer.update_object(detected, stamp)

    thread = threading.Thread(target=publish)
    thread.start()
    samples = buffer.wait_for_samples(3, 0.5, require_mask=True, max_age_sec=10.0)
    thread.join()

    assert len(samples) == 3


def test_request_collector_accumulates_three_fresh_one_hz_samples_past_shared_retention():
    source_now_ns = [1_000_000_000]
    monotonic_now = [10.0]
    buffer = SynchronizedRgbdBuffer(
        source_clock_ns=lambda: source_now_ns[0],
        monotonic_clock=lambda: monotonic_now[0],
    )
    color = np.zeros((3, 4, 3), dtype=np.uint8)
    depth = np.full((3, 4), 2200, dtype=np.uint16)
    mask = np.ones((3, 4), dtype=np.uint8) * 255
    detected = types.SimpleNamespace(
        detected=True,
        bbox_x=0,
        bbox_y=0,
        bbox_width=4,
        bbox_height=3,
    )
    buffer.update_joints([0.0] * 6)
    admitted = [threading.Event() for _index in range(3)]
    sample_from_entry = buffer._sample_from_entry

    def record_admission(entry, require_mask):
        result = sample_from_entry(entry, require_mask)
        admitted[(int(entry['stamp_ns']) - 1_000_000_000) // 1_000_000_000].set()
        return result

    buffer._sample_from_entry = record_admission

    def publish():
        for index in range(3):
            stamp_ns = 1_000_000_000 + index * 1_000_000_000
            source_now_ns[0] = stamp_ns
            monotonic_now[0] = 10.0 + index
            stamp = stamp_ns * 1e-9
            buffer.update_color(color, stamp, 'camera_link')
            buffer.update_depth(depth, stamp, 'camera_link')
            source_now_ns[0] = stamp_ns + 800_000_000
            monotonic_now[0] = 10.8 + index
            buffer.update_mask(mask, stamp, 'camera_link')
            buffer.update_object(detected, stamp)
            assert admitted[index].wait(1.0)

    thread = threading.Thread(target=publish)
    thread.start()
    samples = buffer.wait_for_samples(
        3,
        4.0,
        require_mask=True,
        max_age_sec=0.35,
        collection_span_sec=3.0,
        max_inference_latency_sec=1.2,
    )
    thread.join()

    assert [item.stamp_ns for item in samples] == [
        1_000_000_000,
        2_000_000_000,
        3_000_000_000,
    ]
    assert 1_000_000_000 not in buffer._entries


def test_request_collector_enforces_bounded_source_stamp_span():
    source_now_ns = [1_000_000_000]
    monotonic_now = [10.0]
    buffer = SynchronizedRgbdBuffer(
        source_clock_ns=lambda: source_now_ns[0],
        monotonic_clock=lambda: monotonic_now[0],
    )
    color = np.zeros((3, 4, 3), dtype=np.uint8)
    depth = np.full((3, 4), 2200, dtype=np.uint16)
    detected = types.SimpleNamespace(
        detected=True,
        bbox_x=0,
        bbox_y=0,
        bbox_width=4,
        bbox_height=3,
    )
    buffer.update_joints([0.0] * 6)
    offsets_ns = (0, 1_700_000_000, 3_400_000_000)
    admitted = [threading.Event() for _offset in offsets_ns]
    sample_from_entry = buffer._sample_from_entry

    def record_admission(entry, require_mask):
        result = sample_from_entry(entry, require_mask)
        stamp_offset_ns = int(entry['stamp_ns']) - 1_000_000_000
        admitted[offsets_ns.index(stamp_offset_ns)].set()
        return result

    buffer._sample_from_entry = record_admission

    def publish():
        for index, offset_ns in enumerate(offsets_ns):
            stamp_ns = 1_000_000_000 + offset_ns
            source_now_ns[0] = stamp_ns
            monotonic_now[0] = 10.0 + index * 1.7
            stamp = stamp_ns * 1e-9
            buffer.update_color(color, stamp, 'camera_link')
            buffer.update_depth(depth, stamp, 'camera_link')
            buffer.update_object(detected, stamp)
            assert admitted[index].wait(1.0)
        monotonic_now[0] = 14.1
        buffer.update_joints([0.0] * 6)

    thread = threading.Thread(target=publish)
    thread.start()
    samples = buffer.wait_for_samples(
        3,
        4.0,
        require_mask=False,
        max_age_sec=0.35,
        collection_span_sec=3.0,
    )
    thread.join()

    assert samples == []


def test_request_collector_invalidates_same_stamp_mask_updated_to_empty():
    source_now_ns = [1_000_000_000]
    monotonic_now = [10.0]
    buffer = SynchronizedRgbdBuffer(
        source_clock_ns=lambda: source_now_ns[0],
        monotonic_clock=lambda: monotonic_now[0],
    )
    color = np.zeros((20, 30, 3), dtype=np.uint8)
    depth = np.full((20, 30), 2200, dtype=np.uint16)
    mask = stable_mask()
    detected = types.SimpleNamespace(
        detected=True,
        bbox_x=5,
        bbox_y=4,
        bbox_width=20,
        bbox_height=12,
    )
    buffer.update_joints([0.0] * 6)
    first_admitted = threading.Event()
    second_admitted = threading.Event()
    sample_from_entry = buffer._sample_from_entry

    def record_admission(entry, require_mask):
        result = sample_from_entry(entry, require_mask)
        if int(entry['stamp_ns']) == 1_000_000_000:
            first_admitted.set()
        if int(entry['stamp_ns']) == 2_000_000_000:
            second_admitted.set()
        return result

    buffer._sample_from_entry = record_admission

    def publish():
        buffer.update_color(color, 1.0, 'camera_link')
        buffer.update_depth(depth, 1.0, 'camera_link')
        buffer.update_mask(mask, 1.0, 'camera_link')
        buffer.update_object(detected, 1.0)
        assert first_admitted.wait(1.0)
        monotonic_now[0] = 10.1
        buffer.update_mask(np.zeros_like(mask), 1.0, 'camera_link')
        source_now_ns[0] = 2_000_000_000
        monotonic_now[0] = 11.0
        buffer.update_color(color, 2.0, 'camera_link')
        buffer.update_depth(depth, 2.0, 'camera_link')
        buffer.update_mask(mask, 2.0, 'camera_link')
        buffer.update_object(detected, 2.0)
        assert second_admitted.wait(1.0)
        source_now_ns[0] = 3_000_000_000
        monotonic_now[0] = 12.0
        buffer.update_color(color, 3.0, 'camera_link')
        buffer.update_depth(depth, 3.0, 'camera_link')
        buffer.update_mask(mask, 3.0, 'camera_link')
        buffer.update_object(detected, 3.0)

    thread = threading.Thread(target=publish)
    thread.start()
    samples = buffer.wait_for_samples(
        2,
        3.0,
        require_mask=True,
        max_age_sec=0.35,
        collection_span_sec=3.0,
    )
    thread.join()

    assert [item.stamp_ns for item in samples] == [2_000_000_000, 3_000_000_000]
    assert all(np.any(item.object_mask) for item in samples)


def test_buffer_accepts_bounded_delayed_inference_immediately_after_completion():
    source_now_ns = [1_000_000_000]
    monotonic_now = [10.0]
    buffer = SynchronizedRgbdBuffer(
        source_clock_ns=lambda: source_now_ns[0],
        monotonic_clock=lambda: monotonic_now[0],
    )
    color = np.zeros((3, 4, 3), dtype=np.uint8)
    depth = np.full((3, 4), 2200, dtype=np.uint16)
    mask = np.ones((3, 4), dtype=np.uint8) * 255
    detected = types.SimpleNamespace(
        detected=True,
        bbox_x=0,
        bbox_y=0,
        bbox_width=4,
        bbox_height=3,
    )
    buffer.update_joints([0.0] * 6)
    buffer.update_color(color, 1.0, 'camera_link')
    buffer.update_depth(depth, 1.0, 'camera_link')
    source_now_ns[0] = 1_890_000_000
    monotonic_now[0] = 10.89
    buffer.update_mask(mask, 1.0, 'camera_link')
    buffer.update_object(detected, 1.0)

    samples = buffer.wait_for_samples(
        1,
        0.01,
        require_mask=True,
        max_age_sec=0.35,
        max_inference_latency_sec=1.2,
    )

    assert [item.stamp_ns for item in samples] == [1_000_000_000]


def test_complete_empty_mask_is_never_admitted_as_a_sample():
    source_now_ns = [1_100_000_000]
    monotonic_now = [10.1]
    buffer = SynchronizedRgbdBuffer(
        source_clock_ns=lambda: source_now_ns[0],
        monotonic_clock=lambda: monotonic_now[0],
    )
    color = np.zeros((3, 4, 3), dtype=np.uint8)
    depth = np.full((3, 4), 2200, dtype=np.uint16)
    detected = types.SimpleNamespace(
        detected=True,
        bbox_x=0,
        bbox_y=0,
        bbox_width=4,
        bbox_height=3,
    )
    buffer.update_joints([0.0] * 6)
    buffer.update_color(color, 1.0, 'camera_link')
    buffer.update_depth(depth, 1.0, 'camera_link')
    buffer.update_mask(np.zeros((3, 4), dtype=np.uint8), 1.0, 'camera_link')
    buffer.update_object(detected, 1.0)

    assert buffer.wait_for_samples(
        1,
        0.0,
        require_mask=True,
        max_age_sec=0.35,
        max_inference_latency_sec=1.2,
    ) == []
    assert buffer.mask_timeout_failure(1, 0.35, 1.2)[0] == 'MASK_EMPTY'


def test_pre_request_duplicate_does_not_refresh_completion_age():
    source_now_ns = [1_000_000_000]
    monotonic_now = [10.0]
    buffer = SynchronizedRgbdBuffer(
        source_clock_ns=lambda: source_now_ns[0],
        monotonic_clock=lambda: monotonic_now[0],
    )
    color = np.zeros((3, 4, 3), dtype=np.uint8)
    depth = np.full((3, 4), 2200, dtype=np.uint16)
    mask = np.ones((3, 4), dtype=np.uint8) * 255
    detected = types.SimpleNamespace(
        detected=True,
        bbox_x=0,
        bbox_y=0,
        bbox_width=4,
        bbox_height=3,
    )
    buffer.update_joints([0.0] * 6)
    buffer.update_color(color, 1.0, 'camera_link')
    buffer.update_depth(depth, 1.0, 'camera_link')
    buffer.update_mask(mask, 1.0, 'camera_link')
    buffer.update_object(detected, 1.0)
    source_now_ns[0] = 1_300_000_000
    monotonic_now[0] = 10.3
    buffer.update_mask(mask, 1.0, 'camera_link')
    monotonic_now[0] = 10.351

    assert buffer.wait_for_samples(
        1,
        0.0,
        require_mask=True,
        max_age_sec=0.35,
        max_inference_latency_sec=1.2,
    ) == []


def test_buffer_rejects_inference_that_completes_over_latency_limit():
    source_now_ns = [1_000_000_000]
    monotonic_now = [10.0]
    buffer = SynchronizedRgbdBuffer(
        source_clock_ns=lambda: source_now_ns[0],
        monotonic_clock=lambda: monotonic_now[0],
    )
    color = np.zeros((3, 4, 3), dtype=np.uint8)
    depth = np.full((3, 4), 2200, dtype=np.uint16)
    mask = np.ones((3, 4), dtype=np.uint8) * 255
    detected = types.SimpleNamespace(
        detected=True,
        bbox_x=0,
        bbox_y=0,
        bbox_width=4,
        bbox_height=3,
    )
    buffer.update_joints([0.0] * 6)
    buffer.update_color(color, 1.0, 'camera_link')
    buffer.update_depth(depth, 1.0, 'camera_link')
    source_now_ns[0] = 2_210_000_000
    monotonic_now[0] = 11.21
    buffer.update_mask(mask, 1.0, 'camera_link')
    buffer.update_object(detected, 1.0)

    assert buffer.wait_for_samples(
        1,
        0.0,
        require_mask=True,
        max_age_sec=0.35,
        max_inference_latency_sec=1.2,
    ) == []


def test_buffer_rejects_completed_sample_after_post_completion_age_limit():
    source_now_ns = [1_000_000_000]
    monotonic_now = [10.0]
    buffer = SynchronizedRgbdBuffer(
        source_clock_ns=lambda: source_now_ns[0],
        monotonic_clock=lambda: monotonic_now[0],
    )
    color = np.zeros((3, 4, 3), dtype=np.uint8)
    depth = np.full((3, 4), 2200, dtype=np.uint16)
    mask = np.ones((3, 4), dtype=np.uint8) * 255
    detected = types.SimpleNamespace(
        detected=True,
        bbox_x=0,
        bbox_y=0,
        bbox_width=4,
        bbox_height=3,
    )
    buffer.update_joints([0.0] * 6)
    buffer.update_color(color, 1.0, 'camera_link')
    buffer.update_depth(depth, 1.0, 'camera_link')
    source_now_ns[0] = 1_800_000_000
    monotonic_now[0] = 10.8
    buffer.update_mask(mask, 1.0, 'camera_link')
    buffer.update_object(detected, 1.0)
    monotonic_now[0] = 11.151

    assert buffer.wait_for_samples(
        1,
        0.0,
        require_mask=True,
        max_age_sec=0.35,
        max_inference_latency_sec=1.2,
    ) == []


def test_buffer_keeps_adjacent_large_epoch_nanosecond_stamps_distinct():
    class Stamp:
        def __init__(self, secs, nsecs):
            self.secs = secs
            self.nsecs = nsecs

        def to_sec(self):
            return float(self.secs) + float(self.nsecs) * 1e-9

    base_ns = 1_700_000_000 * 1_000_000_000
    buffer = synchronized_buffer_at(base_ns + 10)
    color = np.zeros((3, 4, 3), dtype=np.uint8)
    depth = np.full((3, 4), 2200, dtype=np.uint16)
    detected = types.SimpleNamespace(
        detected=True,
        bbox_x=0,
        bbox_y=0,
        bbox_width=4,
        bbox_height=3,
    )
    buffer.update_joints([0.0] * 6)
    stamps = [Stamp(1_700_000_000, 1), Stamp(1_700_000_000, 2)]
    for stamp in stamps:
        buffer.update_color(color, stamp, 'camera_link')
        buffer.update_depth(depth, stamp, 'camera_link')
        buffer.update_object(detected, stamp)

    samples = buffer.wait_for_samples(2, 0.01, require_mask=False, max_age_sec=10.0)

    assert len(samples) == 2


def test_exact_nanosecond_discard_preserves_next_large_epoch_entry():
    class Stamp:
        def __init__(self, secs, nsecs):
            self.secs = secs
            self.nsecs = nsecs

        def to_sec(self):
            return float(self.secs) + float(self.nsecs) * 1e-9

    base_ns = 1_700_000_000 * 1_000_000_000
    buffer = synchronized_buffer_at(base_ns + 10)
    color = np.zeros((3, 4, 3), dtype=np.uint8)
    depth = np.full((3, 4), 2200, dtype=np.uint16)
    detected = types.SimpleNamespace(
        detected=True,
        bbox_x=0,
        bbox_y=0,
        bbox_width=4,
        bbox_height=3,
    )
    buffer.update_joints([0.0] * 6)
    rejected_stamps = [Stamp(1_700_000_000, nsecs) for nsecs in (1, 2, 3)]
    for stamp in rejected_stamps:
        buffer.update_color(color, stamp, 'camera_link')
        buffer.update_depth(depth, stamp, 'camera_link')
        buffer.update_object(detected, stamp)

    first_window = buffer.wait_for_samples(3, 0.01, require_mask=False, max_age_sec=10.0)

    assert [item.stamp_ns for item in first_window] == [
        base_ns + 1,
        base_ns + 2,
        base_ns + 3,
    ]
    fresh_stamp = Stamp(1_700_000_000, 4)
    buffer.update_color(color, fresh_stamp, 'camera_link')
    buffer.update_depth(depth, fresh_stamp, 'camera_link')
    buffer.update_object(detected, fresh_stamp)
    buffer.discard_through_ns(first_window[-1].stamp_ns)
    remaining = buffer.wait_for_samples(1, 0.01, require_mask=False, max_age_sec=10.0)
    assert [item.stamp_ns for item in remaining] == [base_ns + 4]


def test_buffer_rejects_replayed_source_stamp_received_now():
    now_ns = 1_700_000_000_500_000_000
    buffer = synchronized_buffer_at(now_ns)
    color = np.zeros((3, 4, 3), dtype=np.uint8)
    depth = np.full((3, 4), 2200, dtype=np.uint16)
    detected = types.SimpleNamespace(
        detected=True,
        bbox_x=0,
        bbox_y=0,
        bbox_width=4,
        bbox_height=3,
    )
    buffer.update_joints([0.0] * 6)
    stale_stamp = (now_ns - 1_210_000_000) * 1e-9
    buffer.update_color(color, stale_stamp, 'camera_link')
    buffer.update_depth(depth, stale_stamp, 'camera_link')
    buffer.update_object(detected, stale_stamp)

    assert buffer.wait_for_samples(
        1,
        0.01,
        require_mask=False,
        max_age_sec=0.35,
        max_inference_latency_sec=1.2,
    ) == []
    assert buffer.wait_for_samples(
        1,
        0.01,
        require_mask=False,
        max_age_sec=0.35,
        collection_span_sec=3.0,
        max_inference_latency_sec=1.2,
    ) == []


def test_mask_timeout_is_missing_when_buffer_has_never_observed_a_mask():
    buffer = synchronized_buffer_at(1_700_000_000_500_000_000)

    code, reason = buffer.mask_timeout_failure(3, max_age_sec=0.35)

    assert code == 'MASK_MISSING'
    assert 'no instance mask' in reason


def test_mask_timeout_is_stale_after_mask_was_observed_but_cannot_form_fresh_window():
    now_ns = 1_700_000_000_500_000_000
    buffer = synchronized_buffer_at(now_ns)
    mask = np.ones((3, 4), dtype=np.uint8) * 255
    stale_stamp = (now_ns - 500_000_000) * 1e-9
    buffer.update_mask(mask, stale_stamp, 'camera_link')

    assert buffer.wait_for_samples(3, 0.01, require_mask=True, max_age_sec=0.35) == []
    code, reason = buffer.mask_timeout_failure(3, max_age_sec=0.35)

    assert code == 'MASK_STALE'
    assert 'three fresh exact timestamp-matched' in reason


def test_latest_empty_mask_timeout_is_empty_including_same_stamp_replacement():
    now_ns = 1_700_000_000_500_000_000
    buffer = synchronized_buffer_at(now_ns)
    stamp = (now_ns - 100_000_000) * 1e-9
    buffer.update_mask(np.ones((3, 4), dtype=np.uint8) * 255, stamp, 'camera_link')
    buffer.update_mask(np.zeros((3, 4), dtype=np.uint8), stamp, 'camera_link')

    code, reason = buffer.mask_timeout_failure(3, max_age_sec=0.35)

    assert code == 'MASK_EMPTY'
    assert 'latest instance mask is empty' in reason

    buffer.update_mask(np.ones((3, 4), dtype=np.uint8) * 255, stamp, 'camera_link')
    assert buffer.mask_timeout_failure(3, max_age_sec=0.35)[0] == 'MASK_STALE'


def test_empty_mask_timeout_marker_survives_discard():
    now_ns = 1_700_000_000_500_000_000
    buffer = synchronized_buffer_at(now_ns)
    stamp = (now_ns - 100_000_000) * 1e-9
    buffer.update_mask(np.zeros((3, 4), dtype=np.uint8), stamp, 'camera_link')

    buffer.discard_through(stamp)

    assert buffer.mask_timeout_failure(3, max_age_sec=0.35)[0] == 'MASK_EMPTY'


def test_nonempty_mask_timeout_marker_survives_monotonic_prune():
    now_ns = 1_700_000_000_500_000_000
    monotonic_now = [10.0]
    buffer = synchronized_buffer_at(now_ns, monotonic_clock=lambda: monotonic_now[0])
    stamp = (now_ns - 100_000_000) * 1e-9
    buffer.update_mask(np.ones((3, 4), dtype=np.uint8) * 255, stamp, 'camera_link')
    monotonic_now[0] = 12.1

    assert buffer.wait_for_samples(1, 0.0, require_mask=True, max_age_sec=0.35) == []
    assert buffer.mask_timeout_failure(3, max_age_sec=0.35)[0] == 'MASK_STALE'


def test_unlimited_completion_age_timeout_reason_does_not_claim_zero_second_limit():
    buffer = synchronized_buffer_at(1_700_000_000_500_000_000)
    buffer.update_mask(np.ones((3, 4), dtype=np.uint8) * 255, 1.0, 'camera_link')

    code, reason = buffer.mask_timeout_failure(3, max_age_sec=0.0)

    assert code == 'MASK_STALE'
    assert '0.000 s post-completion age limit' not in reason
    assert 'post-completion age is unlimited' in reason


def test_buffer_accepts_fresh_source_stamp_from_injected_clock():
    now_ns = 1_700_000_000_500_000_000
    buffer = synchronized_buffer_at(now_ns)
    color = np.zeros((3, 4, 3), dtype=np.uint8)
    depth = np.full((3, 4), 2200, dtype=np.uint16)
    detected = types.SimpleNamespace(
        detected=True,
        bbox_x=0,
        bbox_y=0,
        bbox_width=4,
        bbox_height=3,
    )
    buffer.update_joints([0.0] * 6)
    fresh_stamp = (now_ns - 100_000_000) * 1e-9
    buffer.update_color(color, fresh_stamp, 'camera_link')
    buffer.update_depth(depth, fresh_stamp, 'camera_link')
    buffer.update_object(detected, fresh_stamp)

    samples = buffer.wait_for_samples(1, 0.01, require_mask=False, max_age_sec=0.35)

    assert len(samples) == 1


def test_buffer_rejects_future_source_stamp():
    now_ns = 1_700_000_000_500_000_000
    buffer = synchronized_buffer_at(now_ns)
    color = np.zeros((3, 4, 3), dtype=np.uint8)
    depth = np.full((3, 4), 2200, dtype=np.uint16)
    detected = types.SimpleNamespace(
        detected=True,
        bbox_x=0,
        bbox_y=0,
        bbox_width=4,
        bbox_height=3,
    )
    buffer.update_joints([0.0] * 6)
    future_stamp = (now_ns + 1_000_000) * 1e-9
    buffer.update_color(color, future_stamp, 'camera_link')
    buffer.update_depth(depth, future_stamp, 'camera_link')
    buffer.update_object(detected, future_stamp)

    assert buffer.wait_for_samples(1, 0.01, require_mask=False, max_age_sec=10.0) == []


def test_buffer_retention_uses_monotonic_receive_time():
    now_ns = 1_700_000_000_500_000_000
    monotonic_now = [10.0]
    buffer = synchronized_buffer_at(now_ns, monotonic_clock=lambda: monotonic_now[0])
    color = np.zeros((3, 4, 3), dtype=np.uint8)
    depth = np.full((3, 4), 2200, dtype=np.uint16)
    detected = types.SimpleNamespace(
        detected=True,
        bbox_x=0,
        bbox_y=0,
        bbox_width=4,
        bbox_height=3,
    )
    buffer.update_joints([0.0] * 6)
    fresh_stamp = (now_ns - 100_000_000) * 1e-9
    buffer.update_color(color, fresh_stamp, 'camera_link')
    buffer.update_depth(depth, fresh_stamp, 'camera_link')
    buffer.update_object(detected, fresh_stamp)
    assert len(buffer.wait_for_samples(1, 0.01, require_mask=False, max_age_sec=10.0)) == 1

    monotonic_now[0] = 12.1

    assert buffer.wait_for_samples(1, 0.0, require_mask=False, max_age_sec=10.0) == []
