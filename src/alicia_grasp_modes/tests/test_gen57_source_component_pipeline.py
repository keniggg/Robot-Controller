"""Whole current measurements use the real registered-station commit path.

The transport fixture completes the original three-frame observation arrival.
Only the segmentation boundary is split into adjacent, independently measured
native proposals. Source topology, complete RGB/depth refinement, fixed station
appearance, staged tracking, SDK checks and publication are real production
methods. This is a controlled integration fixture, not a replay of a real bag.
"""
from copy import deepcopy
from types import SimpleNamespace as NS

import numpy as np
import pytest
from std_msgs.msg import Header
from alicia_flexible_grasp_supervisor.msg import GraspState
from alicia_grasp_modes.source_sampling_topology import SourceSamplingTopologyCache
from alicia_grasp_modes.tabletop import Segmentation, _matching_candidates

import test_locked_rgb_hold as hold_fixture
from test_gen30_source_connected_owner import _candidate
from test_gen30_source_sampling_topology import identity_camera
from test_locked_rgb_hold import custom_frame, latest_status, module
from test_station_compute_wait_producer import armed


def pipeline(monkeypatch, *, source_map=True):
    original_node = hold_fixture.rgb_interest_node

    def calibrated_node(patch):
        node, clock, feed, joint, params = original_node(patch)
        if source_map:
            camera = params['/camera']
            camera.update(identity_camera((240, 320), (400., 400., 160., 120.)))
            cache = node._source_sampling_topology = SourceSamplingTopologyCache()
            cache.offer(camera)
            worker = cache._thread
            if worker is not None:
                worker.join(2.)
                assert not worker.is_alive()
            cache.require(camera, (240, 320), (400., 400., 160., 120.), 100)
        return node, clock, feed, joint, params

    monkeypatch.setattr(hold_fixture, 'rgb_interest_node', calibrated_node)
    node, clock, joint, params, marker, ready = armed(monkeypatch)
    before = NS(last=node.tracker.last, anchor=node.tracker.anchor,
        source_ns=node.last_good_source_stamp_ns,
        registration=node._station_compute_registration,
        contract=deepcopy(node._station_compute_registration['contract']),
        template=marker['station_appearance'],
        session_ns=marker['session_deadline_ns'],
        session_mono=marker['session_deadline_monotonic'],
        publication_counts={name: pub.publish.call_count for name, pub in node.pubs.items()})
    evidence = NS(originals=[], bundles=[], recovery_inputs=[], helper_calls=0)
    original_segment = module.segment_with_support_fallback
    original_recover = module.recover_source_connected_measurement
    original_station_recover = node._station_compute_measure_recovery

    def split_current(*args, **kwargs):
        result = original_segment(*args, **kwargs)
        depth, intrinsics, transform = args[:3]
        # Retain every nonmember. The central complete measured proposal is
        # partitioned without filling a depth gap or introducing any pixel.
        central = min(result.candidates,
            key=lambda candidate: np.linalg.norm(candidate.center_uv-[160., 120.]))
        yy, xx = np.indices(depth.shape)
        native = central.mask > 0
        rows, columns = np.nonzero(native)
        diagonal = ((yy-rows.min())/max(1, np.ptp(rows))
                    >= (xx-columns.min())/max(1, np.ptp(columns)))
        pieces = []
        for selected in (native & diagonal, native & ~diagonal):
            measured = _candidate(selected.astype(np.uint8)*255, depth,
                intrinsics, transform, result.plane_base)
            measured.metrics = dict(deepcopy(central.metrics),
                valid_points=int(selected.sum()),
                median_height_m=measured.metrics['median_height_m'])
            pieces.append(measured)
        assert len(_matching_candidates(pieces, node.tracker.anchor,
            node.tracker.last, node.config)) == 2
        values = []
        for candidate in result.candidates:
            values.extend(pieces if candidate is central else [candidate])
        original = Segmentation(values, result.plane_base.copy(), deepcopy(result.metrics))
        assert not np.any((pieces[0].mask > 0) & (pieces[1].mask > 0))
        np.testing.assert_array_equal(
            (pieces[0].mask > 0) | (pieces[1].mask > 0), native)
        evidence.originals.append(original)
        return original

    def recover(*args, **kwargs):
        evidence.helper_calls += 1
        result = original_recover(*args, **kwargs)
        if result is not None:
            assert node.tracker.last is before.last
            evidence.bundles.append(result)
        return result

    def station_recover(source, result, candidate, camera):
        evidence.recovery_inputs.append((result, candidate))
        return original_station_recover(source, result, candidate, camera)

    monkeypatch.setattr(module, 'segment_with_support_fallback', split_current)
    monkeypatch.setattr(module, 'recover_source_connected_measurement', recover)
    monkeypatch.setattr(node, '_station_compute_measure_recovery', station_recover)

    def run(**kwargs):
        clock[0] += .2
        return custom_frame(node, clock, joint, target_depth=.35, **kwargs)

    return NS(node=node, clock=clock, joint=joint, params=params, marker=marker,
        ready=ready, before=before, evidence=evidence, run=run)


def assert_no_new_geometry(env):
    node = env.node
    assert node.last_good_source_stamp_ns == env.before.source_ns
    assert node.tracker.last is env.before.last
    assert not node.pubs['object'].publish.call_args[0][0].detected
    assert not node.pubs['object_detected'].publish.call_args[0][0].data
    assert not node.pubs['raw_object_detected'].publish.call_args[0][0].data
    assert not np.any(node.bridge.imgmsg_to_cv2(
        node.pubs['object_mask'].publish.call_args[0][0], 'mono8'))
    assert latest_status(node)['state'] != 'ready'


def test_whole_current_measurement_reaches_original_fixed_station_commit(monkeypatch):
    env = pipeline(monkeypatch)
    source = env.run()
    node, evidence = env.node, env.evidence
    assert latest_status(node)['state'] == 'ready', latest_status(node)
    assert len(evidence.bundles) == 1
    bundle = evidence.bundles[0]
    assert bundle.original is evidence.originals[0]
    assert len(bundle.original.candidates) == 2
    assert bundle.normalized_result.candidates == [bundle.raw]
    assert node.tracker.anchor is env.before.anchor and node.tracker.last is bundle.raw
    assert evidence.recovery_inputs
    assert all(result is bundle.normalized_result and candidate is bundle.refined
               for result, candidate in evidence.recovery_inputs)
    assert node.last_good_source_stamp_ns == source[0].header.stamp.to_nsec()
    np.testing.assert_array_equal(node.bridge.imgmsg_to_cv2(
        node.pubs['object_mask'].publish.call_args[0][0], 'mono8'), bundle.refined.mask)
    assert node._locked_rgb_hold_reference['observation_arrival_retry'] is env.marker
    assert env.marker['station_appearance'] is env.before.template
    assert env.marker['session_deadline_ns'] == env.before.session_ns
    assert env.marker['session_deadline_monotonic'] == env.before.session_mono
    audit = latest_status(node)['frame_timing']['source_component_measurement']
    assert audit['derivation']['original_candidate_count'] == 2
    assert sorted(audit['derivation']['original_metric_match_indices']) == [0, 1]
    assert sorted(audit['derivation']['member_indices']) == [0, 1]
    assert audit['appearance_identity']['appearance_match']
    assert not audit['metric_geometry_authority'] and not audit['motion_authorized']


def mutate(env, kind, monkeypatch):
    node, clock, joint, params = env.node, env.clock, env.joint, env.params
    if kind == 'selection':
        params['/grasp_mode/selection']['generation'] += 1
    elif kind == 'epoch':
        node._rgb_interest_epoch_cb(Header(stamp=module.rospy.Time(2),
            frame_id='sdk_reference_epoch'))
    elif kind == 'sdk_out_and_back':
        joint(clock[0], [.005, 0., 0., 0., 0., 0.])
        joint(clock[0]+.000001)
    elif kind == 'manual':
        params['/gui/joint_direct_mode'] = True
    elif kind == 'cancel':
        node._station_compute_state_cb(GraspState(state='CANCELLED', active=False))
    elif kind == 'phase':
        phase = deepcopy(node._station_compute_phase)
        phase.phase_id += 1
        node._station_compute_phase_cb(phase)
    elif kind == 'camera':
        params['/camera']['fx'] += 1.
    elif kind == 'source_header':
        node._station_compute_frame['source'][1].header.stamp = module.rospy.Time(1)
    elif kind == 'source_expired':
        clock[0] = node._station_compute_frame['source_ns']*1e-9+.801
        joint(clock[0])
    elif kind == 'monotonic':
        monkeypatch.setattr(module.time, 'monotonic',
            lambda: env.before.registration['monotonic_deadline']+.001)
    elif kind == 'session':
        env.marker['session_deadline_ns'] = module.rospy.Time.now().to_nsec()
    else:
        raise AssertionError(kind)


@pytest.mark.parametrize('kind', ['selection', 'epoch', 'sdk_out_and_back',
    'manual', 'cancel', 'phase', 'camera', 'source_header', 'source_expired',
    'monotonic', 'session'])
def test_complete_measurement_does_not_survive_final_owner_or_time_change(monkeypatch, kind):
    env = pipeline(monkeypatch)
    original = env.node._prepare_locked_rgb_hold_reference
    injected = []

    def before_commit(*args, **kwargs):
        result = original(*args, **kwargs)
        assert len(env.evidence.bundles) == 1
        assert env.node.tracker.last is env.before.last
        mutate(env, kind, monkeypatch)
        injected.append(True)
        return result

    monkeypatch.setattr(env.node, '_prepare_locked_rgb_hold_reference', before_commit)
    source = env.run()
    assert injected
    if kind == 'phase':
        # The existing typed retirement path silently discards old-owner work.
        # Its historical READY is not a newly published measurement.
        assert env.node.last_good_source_stamp_ns == env.before.source_ns
        assert env.node.tracker.last is env.before.last
        assert {name: pub.publish.call_count for name, pub in env.node.pubs.items()} == (
            env.before.publication_counts)
        witness = env.node._processing_timing['station_compute_retired']
        assert witness['source_ns'] == source[0].header.stamp.to_nsec()
        assert witness['source_ns'] > env.before.source_ns
        assert not witness['metric_geometry_authority'] and not witness['motion_authorized']
        assert not witness['reference_refreshed']
        assert env.node._station_compute_registration is None
        return
    assert_no_new_geometry(env)
    if kind == 'source_expired':
        assert latest_status(env.node)['state'] == 'processing_deadline_pending'
        assert env.node._station_compute_registration is env.before.registration
        assert env.before.registration['contract'] == env.before.contract


@pytest.mark.parametrize('kind', ['epoch', 'cancel', 'source_expired'])
def test_current_checks_also_run_after_complete_helper_before_template(monkeypatch, kind):
    env = pipeline(monkeypatch)
    original = module.recover_source_connected_measurement
    injected = []

    def after_geometry(*args, **kwargs):
        result = original(*args, **kwargs)
        assert result is not None
        mutate(env, kind, monkeypatch)
        injected.append(True)
        return result

    monkeypatch.setattr(module, 'recover_source_connected_measurement', after_geometry)
    env.run()
    assert injected
    assert_no_new_geometry(env)


@pytest.mark.parametrize('kind', ['raw', 'refined', 'original', 'normalized', 'audit'])
def test_complete_measurement_binding_is_rechecked_at_final_commit(monkeypatch, kind):
    env = pipeline(monkeypatch)
    original = env.node._prepare_locked_rgb_hold_reference

    def corrupt_after_template(*args, **kwargs):
        result = original(*args, **kwargs)
        bundle = env.evidence.bundles[0]
        if kind in ('raw', 'refined'):
            getattr(bundle, kind).position_base[0] += .001
        elif kind == 'original':
            bundle.original.candidates.reverse()
        elif kind == 'normalized':
            bundle.normalized_result.candidates.append(bundle.original.candidates[0])
        else:
            bundle.audit['original_metric_match_indices'] = [0]
        return result

    monkeypatch.setattr(env.node, '_prepare_locked_rgb_hold_reference', corrupt_after_template)
    env.run()
    assert len(env.evidence.bundles) == 1
    assert_no_new_geometry(env)


def test_remaining_measured_competitor_is_not_discarded_for_normalization(monkeypatch):
    env = pipeline(monkeypatch)
    depth = np.full((240, 320), .4, np.float32)
    depth[106:134, 148:173] = .35
    # Distinct current pixels, separated by one genuine measured table column:
    # 26 px * .35 m / 400 px = 22.75 mm, within the unchanged 25 mm gate.
    depth[106:134, 174:199] = .35
    assert np.all(depth[106:134, 173] == np.float32(.4))
    env.run(depth_image=depth)
    assert len(env.evidence.originals[0].candidates) == 3
    assert len(_matching_candidates(env.evidence.originals[0].candidates,
        env.before.anchor, env.before.last, env.node.config)) == 3
    assert env.evidence.helper_calls == 1 and not env.evidence.bundles
    assert_no_new_geometry(env)


def test_complete_helper_strict_failure_is_not_converted_to_an_optional_proposal(monkeypatch):
    env = pipeline(monkeypatch)
    failure = ValueError('current_fragment_native_core_not_owned')
    called = []

    def reject(*args, **kwargs):
        called.append(True)
        raise failure

    monkeypatch.setattr(module, 'recover_source_connected_measurement', reject)
    env.run()
    assert called
    assert_no_new_geometry(env)
    assert latest_status(env.node)['detail'] == 'perception_failure: '+str(failure)
    assert env.node.first_loss is not None


def test_missing_source_map_keeps_original_multi_match_path(monkeypatch):
    env = pipeline(monkeypatch, source_map=False)
    env.run()
    assert len(env.evidence.originals) == 1
    assert env.evidence.helper_calls == 0 and not env.evidence.bundles
    # This fixture's old grouping may qualify or reject. The assertion is the
    # compatibility boundary: no new component permission without a source map.
    assert 'source_component_measurement' not in latest_status(env.node)['frame_timing']
