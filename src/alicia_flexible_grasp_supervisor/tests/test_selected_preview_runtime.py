"""Request/candidate cache binding after target loss; uses fake ROS publishers."""
import dataclasses
from types import SimpleNamespace as NS

import pytest

from test_remote_grasp6d_streaming import (
    MutableClock, RecordingPublisher, remote_node, snapshot, streaming_node,
)


@pytest.fixture
def bound_runtime():
    node = streaming_node(clock=MutableClock(20.), start_worker=False)
    node.start_streaming()
    node.submit_stream_snapshot(snapshot(19.8))
    ticket = node._stream_worker_ticket
    selected = NS(track_id=3, variant_index=1, evaluation_request_id=ticket.request_id,
                  stable_candidate=NS(target_epoch=ticket.target_epoch))
    runtime = {'prepared': NS(ticket=ticket)}
    node._stable_variant_runtime = {(3, 1): runtime}
    node.preview_plan_pub = RecordingPublisher()
    node.preview_rich_plan_pub = RecordingPublisher()
    yield node, ticket, selected, runtime
    node.shutdown_streaming_worker()


@pytest.mark.parametrize('invalidation,code', [('target', 'TARGET_EPOCH_STALE'), ('stop', 'GENERATION_STALE')])
def test_selected_preview_cleared_by_invalidation_is_cancellation(bound_runtime, invalidation, code):
    node, ticket, selected, _ = bound_runtime
    if invalidation == 'target':
        node._advance_target_instance_epoch('TARGET_LOST')
    else:
        node.stop_streaming()
    assert node._stable_variant_runtime == {}
    with pytest.raises(remote_node.StreamResultCancelled) as exc:
        node._publish_selected_preview(selected, ticket=ticket)
    assert exc.value.code == code
    assert node._stable_variant_runtime == {}
    assert node.preview_plan_pub.messages == node.preview_rich_plan_pub.messages == []


def test_old_ticket_cannot_read_new_generation_runtime_under_same_key(bound_runtime):
    node, ticket, selected, _ = bound_runtime
    node.stop_streaming()
    node.start_streaming()
    new_runtime = {'prepared': NS(ticket=dataclasses.replace(ticket, generation=node._stream_generation,
                                                            target_epoch=node.target_instance_epoch))}
    node._stable_variant_runtime[(3, 1)] = new_runtime
    with pytest.raises(remote_node.StreamResultCancelled) as exc:
        node._selected_preview_runtime(selected, ticket=ticket)
    assert exc.value.code == 'GENERATION_STALE'
    assert node._stable_variant_runtime[(3, 1)] is new_runtime


def test_current_ticket_missing_runtime_remains_real_failure(bound_runtime):
    node, ticket, selected, _ = bound_runtime
    node._stable_variant_runtime.clear()
    with pytest.raises(RuntimeError, match='runtime is unavailable') as exc:
        node._publish_selected_preview(selected, ticket=ticket)
    assert not isinstance(exc.value, remote_node.StreamResultCancelled)
    assert node.preview_plan_pub.messages == node.preview_rich_plan_pub.messages == []


@pytest.mark.parametrize('explicit', [True, False])
def test_current_cached_rebuild_remains_bound_without_bypass(bound_runtime, explicit):
    node, ticket, selected, runtime = bound_runtime
    assert node._selected_preview_runtime(selected, ticket=ticket if explicit else None) is runtime


def test_no_explicit_ticket_still_rejects_stale_cached_runtime(bound_runtime):
    node, _, selected, runtime = bound_runtime
    node._advance_target_instance_epoch('TARGET_LOST')
    node._stable_variant_runtime[(3, 1)] = runtime  # Deliberate stale fixture only.
    with pytest.raises(remote_node.StreamResultCancelled) as exc:
        node._build_selected_preview_bundle(selected)
    assert exc.value.code == 'TARGET_EPOCH_STALE'


@pytest.mark.parametrize('runtime', [{}, {'prepared': NS()}])
def test_no_ticket_or_prepared_is_not_an_authority_bypass(bound_runtime, runtime):
    node, _, selected, _ = bound_runtime
    node._stable_variant_runtime[(3, 1)] = runtime
    with pytest.raises(RuntimeError, match='runtime ticket is unavailable'):
        node._selected_preview_runtime(selected)


def test_other_current_request_cache_cannot_satisfy_explicit_ticket(bound_runtime):
    node, ticket, selected, runtime = bound_runtime
    runtime['prepared'].ticket = dataclasses.replace(ticket, request_id=ticket.request_id + 1)
    with pytest.raises(RuntimeError, match='runtime ticket mismatch'):
        node._selected_preview_runtime(selected, ticket=ticket)


@pytest.mark.parametrize('wrong', ['request', 'target'])
def test_selected_identity_must_match_actual_cached_ticket(bound_runtime, wrong):
    node, ticket, selected, _ = bound_runtime
    if wrong == 'request':
        selected.evaluation_request_id += 1
    else:
        selected.stable_candidate.target_epoch += 1
    with pytest.raises(RuntimeError, match='candidate does not match'):
        node._selected_preview_runtime(selected, ticket=ticket)


def test_check_and_cache_lookup_share_invalidation_lock(bound_runtime):
    node, ticket, selected, runtime = bound_runtime
    checked = []
    original_check = node._require_stream_ticket_current_locked
    def check(value):
        assert node._stream_condition._is_owned()
        checked.append(value)
        return original_check(value)
    class Cache(dict):
        def get(self, key, default=None):
            assert node._stream_condition._is_owned()
            assert checked == [ticket]  # Explicit cancellation check precedes lookup.
            return super().get(key, default)
    node._require_stream_ticket_current_locked = check
    node._stable_variant_runtime = Cache({(3, 1): runtime})
    assert node._selected_preview_runtime(selected, ticket=ticket) is runtime
    assert checked == [ticket, ticket]


def test_invalidation_after_capture_is_still_blocked_by_final_publish_guard(bound_runtime):
    node, ticket, selected, runtime = bound_runtime
    def bundle(candidate, ticket=None):
        assert node._selected_preview_runtime(candidate, ticket=ticket) is runtime
        node._advance_target_instance_epoch('TARGET_LOST')
        return dict(rich_plan=NS(), legacy_plan=NS(), ticket=ticket,
                    commit_callback=lambda: pytest.fail('must not commit stale preview'),
                    proposal_metadata={}, proposal={})
    node._build_selected_preview_bundle = bundle
    with pytest.raises(remote_node.StreamResultCancelled) as exc:
        node._publish_selected_preview(selected, ticket=ticket)
    assert exc.value.code == 'TARGET_EPOCH_STALE'
    assert node.preview_plan_pub.messages == node.preview_rich_plan_pub.messages == []
