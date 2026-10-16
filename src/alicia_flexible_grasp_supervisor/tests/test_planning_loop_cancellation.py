"""Old observation work must release the worker when near-field starts."""
import threading
import types

import numpy as np
import pytest

from test_remote_grasp6d_streaming import remote_node, streaming_node
from test_observation_distance_search import recorded_scene, variants, remote


def test_stable_recheck_stops_before_next_variant_after_phase_change():
    node = streaming_node(start_worker=False)
    node.streaming_enabled = True
    identity = node._current_stream_target_identity()
    prepared = types.SimpleNamespace(
        snapshot=types.SimpleNamespace(target_identity=identity),
        geometry=types.SimpleNamespace(center_base=(0.0, 0.0, 0.0)),
        pose_estimator=types.SimpleNamespace(transform_sha256='frozen'),
        near_field=False, stamp=remote_node.rospy.Time(1),
        ticket=types.SimpleNamespace(
            generation=node._stream_generation,
            target_epoch=node.target_instance_epoch),
    )
    node._frozen_observation_reference_pose = lambda prepared: None
    payload = remote_node.NormalizedPlanningCandidate(
        candidate_source='tabletop_geometry', source_index=0, variant_index=0,
        source_lineage=('tabletop_geometry',), contact_center_base=(0, 0, 0),
        T_base_tool0=np.eye(4), insertion_axis_base=(0, 0, -1),
        jaw_axis_base=(0, 1, 0), required_open_width_m=0.04,
        model_width_m=None, model_score=None, source_local_score=0,
        common_physical_cost=0, geometry_gate=remote_node.CandidateGateResult(
            ok=True, failure_code='', failure_reason='',
            required_open_width_m=0.04, center_distance_m=0,
            support_clearance_m=0.01, jaw_alignment=1,
            motion_cost=0, geometry_cost=0, failed_gate='', passed_gate_count=6,
        ), grasp_sequence=None,
        payload=None, audit={},
    )
    stable = types.SimpleNamespace(payload=payload, track_id=1)
    attempted = []
    def recheck(stable, variant, stamp):
        attempted.append(variant)
        node._stream_generation += 1
        raise ValueError('candidate invalid during a phase transition')
    node._stable_variant_pose = recheck
    try:
        with pytest.raises(remote_node.StreamResultCancelled):
            node._recheck_and_score_stable(prepared, (stable,))
        assert attempted == [0]
    finally:
        node.shutdown_streaming_worker()


def test_observation_search_stops_before_next_view_after_phase_change():
    node, geometry, prepared, reference = recorded_scene()
    node._stream_condition = threading.Condition(threading.RLock())
    node._stream_shutdown = threading.Event()
    node.streaming_enabled = True
    node._stream_generation = 1
    node.target_instance_epoch = 1
    prepared.ticket = types.SimpleNamespace(generation=1, target_epoch=1)
    make = node._make_observation_sequence
    attempted = []
    def make_and_switch(*args, **kwargs):
        attempted.append(kwargs['observation_camera_distance_m'])
        result = make(*args, **kwargs)
        node._stream_generation += 1
        return result
    node._make_observation_sequence = make_and_switch
    with pytest.raises(remote.StreamResultCancelled):
        variants(node, geometry, prepared, reference)
    assert attempted == [0.20]
