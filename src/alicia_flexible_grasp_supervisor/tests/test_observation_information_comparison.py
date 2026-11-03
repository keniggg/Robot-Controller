"""A useful checked view must remain comparable after the first costly proof."""
from types import SimpleNamespace as NS
import pytest
from test_observation_distance_search import remote
from test_remote_grasp6d_streaming import streaming_node, MutableClock, snapshot
from alicia_flexible_grasp.grasp.grasp6d_pipeline import MoveItResult, bounded_moveit_select


def context():
    node = streaming_node(clock=MutableClock(50.), start_worker=False)
    node.execution_plan_validity_sec = 120.
    node.mujoco_selection_snapshot_reserve_sec = 30.
    elapsed = MutableClock(0.)
    node._observation_tilt_clock = elapsed
    node._observation_comparison_clock = elapsed
    node._information_driven_observation_comparison = lambda: True
    node._order_observation_checks = lambda variants: variants
    node.start_streaming(); node.submit_stream_snapshot(snapshot(49.8))
    variants = []
    for index in range(5):
        pose = remote.PoseStamped(); pose.pose.position.x = index
        variants.append(dict(sequence=NS(pregrasp=pose), camera_tilt_offset_deg=10.,
            camera_roll_offset_deg=0., preference_index=index))
    runtime = dict(prepared=NS(ticket=node._stream_worker_ticket),
        grasp_pose=variants[0]['sequence'].pregrasp,
        observation_sequence=variants[0]['sequence'], observation_variants=variants, soft_evidence={})
    node._stable_variant_runtime = {(3, 1): runtime}
    checked = []
    def strict(pose, observation=False):
        index = int(pose.pose.position.x); checked.append(index); elapsed.value += 4.
        return MoveItResult(True, .1, .1,
            'observation_screened_duration_sec=%s' % (12.-index)), {}, ''
    node._strict_moveit_evaluation = strict
    return node, elapsed, runtime, variants, checked


def test_existing_eight_second_comparison_is_not_cancelled_by_old_three_second_budget():
    node, _, runtime, variants, checked = context()
    try:
        result = node._check_moveit_stable_candidate(NS(track_id=3, variant_index=1))
        assert result.reachable and checked == [0, 1, 2]
        assert runtime['observation_sequence'] is variants[2]['sequence']
        audit = runtime['observation_orientation_search']
        assert audit['information_motion_comparison']
        assert audit['comparison_budget_reached'] and not audit['search_complete']
        assert len([r for r in audit['variant_results'] if not r['checked']]) == 2
    finally: node.shutdown_streaming_worker()


def test_view_families_share_deadline_instead_of_resetting_it():
    node, elapsed, runtime, _, checked = context()
    try:
        checker, continuing = node._far_field_observation_search()
        checker(NS(track_id=3, variant_index=1))
        assert node._observation_information_comparison['deadline'] == 12.
        assert elapsed.value == 12. and not continuing()
        checker(NS(track_id=3, variant_index=1))
        assert checked == [0, 1, 2]
        assert node._observation_information_comparison['deadline'] == 12.
    finally: node.shutdown_streaming_worker()


def test_source_reserve_still_forbids_a_new_tilt_during_comparison():
    node, elapsed, _, _, _ = context()
    try:
        node.execution_plan_validity_sec = 30.  # source + lifetime - reserve < now
        budget, reason = node._claim_observation_tilt_check(node._stream_worker_ticket,
            comparison_deadline=elapsed()+8.)
        assert budget is None and reason == 'OBSERVATION_TILT_SNAPSHOT_RESERVE_REACHED'
    finally: node.shutdown_streaming_worker()
