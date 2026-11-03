"""Offline original producer plus its next real station poll; no production edit."""
from pathlib import Path
import hashlib
import json
from test_gen31_observation_search_progress import (
    context, station,
    test_actual_station_certificate_enters_context_and_bound_failure_publishes_while_pending)


def test_next_poll_after_actual_pending_budget_terminal(station, monkeypatch):
    c = station
    n = c.node
    test_actual_station_certificate_enters_context_and_bound_failure_publishes_while_pending(c, monkeypatch)
    def state():
        p = n.preview_rich_plan_pub.messages[-1]
        return dict(paused=n._observe_return_paused,
            pending=n._station_compute_pending, retired=n._station_compute_retired,
            target_epoch=n.target_instance_epoch,
            last_preview_source=p.candidate_source, last_preview_diagnostic=p.diagnostic,
            preview_count=len(n.preview_rich_plan_pub.messages),
            last_geometry_code=getattr(n.latest_object_geometry, 'failure_code', None),
            last_geometry_reason=getattr(n.latest_object_geometry, 'failure_reason', None))
    before = state()
    n._station_compute_poll()
    after = state()
    O = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_gen29_restart_20261016T000728Z/verification/gen31_budget_postpublication_diagnostic_r1')
    W = Path(__file__).resolve().parents[1]
    sources = ['src/alicia_grasp_modes/src/alicia_grasp_modes/silhouette_observation_planning.py',
               'src/alicia_grasp_modes/src/alicia_grasp_modes/station_compute_wait_remote.py',
               'src/alicia_grasp_modes/src/alicia_grasp_modes/unknown_pending_identity.py']
    result = dict(mode='offline_actual_producer_then_actual_next_station_poll',
        before=before, after=after, original_time_unchanged=True,
        source_sha256={p:hashlib.sha256((W/p).read_bytes()).hexdigest() for p in sources},
        no_ros_or_motion=True, production_modified=False)
    (O/'result.json').write_text(json.dumps(result, indent=2, sort_keys=True)+'\n')
    assert after['target_epoch'] == before['target_epoch'], 'failure-only publication became target loss on its next station tick'
    assert not after['retired']
