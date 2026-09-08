"""Select only candidates whose final registered contact family passes."""

from dataclasses import asdict

from test_grasp6d_pipeline import scored_candidate, reachable_moveit_result
from test_remote_grasp6d_streaming import remote_node
from alicia_flexible_grasp.grasp.grasp6d_pipeline import bounded_moveit_select


def selection_node():
    node = remote_node.RemoteGrasp6DNode.__new__(remote_node.RemoteGrasp6DNode)
    node._stable_variant_runtime = {(index, 0): {} for index in (1, 2, 3)}
    node._check_moveit_stable_candidate = lambda candidate: reachable_moveit_result()
    return node


def test_final_contact_rejection_tries_next_ranked_candidate_without_publishing():
    node = selection_node()
    checked = []
    final_result = reachable_moveit_result(0.8, 0.4)

    def build(candidate):
        checked.append(candidate.track_id)
        assert candidate.moveit_result.reachable
        assert candidate.final_score is not None
        if candidate.track_id == 1:
            raise remote_node.CandidateContractError(
                'BILATERAL_SURFACE_EVIDENCE_MISSING',
                'current fused surface lacks measured bilateral support',
            )
        node._stable_variant_runtime[(candidate.track_id, 0)][
            'final_strict_moveit_result'] = asdict(final_result)

    node._build_selected_preview_bundle = build
    node._publish_selected_preview = lambda candidate: (_ for _ in ()).throw(
        AssertionError('checking candidates must not publish'))
    result = bounded_moveit_select(
        [scored_candidate(index, index) for index in (1, 2, 3)],
        node._check_direct_registered_candidate,
        top_n=3, first_reachable_by_rank=True,
        ranking_key=lambda candidate: candidate.track_id,
    )
    assert checked == [1, 2]
    assert result.selected.track_id == 2
    assert result.selected.moveit_result == final_result
    assert result.checked[0].moveit_result.failure_code == 'MOVEIT_CHECK_ERROR'
    assert result.checked[0].moveit_result.reason.startswith(
        'BILATERAL_SURFACE_EVIDENCE_MISSING:')
    assert node._stable_variant_runtime[(1, 0)]['final_contact_rejection'][
        'code'] == 'BILATERAL_SURFACE_EVIDENCE_MISSING'


def test_original_unreachable_does_not_build_final_preview():
    node = selection_node()
    failed = remote_node.MoveItResult(
        reachable=False, joint_path_cost=0.0, joint_max_delta_rad=0.0,
        reason='unreachable', failure_code='MOVEIT_UNREACHABLE')
    node._check_moveit_stable_candidate = lambda candidate: failed
    node._build_selected_preview_bundle = lambda candidate: (_ for _ in ()).throw(
        AssertionError('unreachable candidates must not build a preview'))
    assert node._check_direct_registered_candidate(scored_candidate(1, 1)) == failed


def test_final_registered_motion_still_obeys_joint_delta_limit():
    node = selection_node()
    def build(candidate):
        node._stable_variant_runtime[(candidate.track_id, 0)][
            'final_strict_moveit_result'] = asdict(reachable_moveit_result(2.0, 1.5))
    node._build_selected_preview_bundle = build
    result = bounded_moveit_select(
        [scored_candidate(1, 1)], node._check_direct_registered_candidate,
        top_n=1, first_reachable_by_rank=True, max_joint_delta_rad=1.0,
        ranking_key=lambda candidate: candidate.track_id,
    )
    assert result.selected is None
    assert result.checked[0].moveit_result.failure_code == 'MOVEIT_JOINT_DELTA_LIMIT'
