"""Top-only segmentation must retain the independently rendered RGB cuboid.

The existing source/ACK/three-frame/registration/common acceptance/task chain
and SDK settle gates run unchanged. Only scene pixels and explicit existing
device/transport providers are fixtures. A simulated close is not grip proof.
"""
from types import SimpleNamespace as NS
from copy import deepcopy
import hashlib
import json

import numpy as np
import pytest

import test_gen35_minimal_chain_replay as chain
from test_gen38_cuboid_chain import cuboid_scene
from test_gen39_registration_chain import _install_real_reached_source_adapter


@pytest.fixture(autouse=True)
def no_unsimulated_boundaries(monkeypatch):
    import rospy
    import socket

    def forbidden(*args, **kwargs):
        raise chain.UnsimulatedExternalCall('GEN43 unsimulated boundary '+repr(args))

    for name in ('connect','connect_ex','sendto','bind','listen'):
        monkeypatch.setattr(socket.socket, name, forbidden)
    monkeypatch.setattr(socket, 'create_connection', forbidden)
    for name in ('init_node','Publisher','Subscriber','Service','ServiceProxy',
                 'wait_for_service','set_param','delete_param'):
        monkeypatch.setattr(rospy, name, forbidden)


def top_only_depth_scene(*, width=.030, length=.065):
    """Independent slab source, initially containing real side measurements.

    The original GEN35 renderer paints RGB from scene.mask. Keep that full
    independently ray-cast silhouette; zero only the actually visible side
    depth only after the original observation. The producer owns its mask.
    This is an analytic sensor-failure fixture, not a new observation search.
    """
    scene = cuboid_scene(width=width, length=length, tilt=35.)
    # GEN38 intentionally removes a 3 mm top band to test another precision
    # case. This regression needs a fully measured top and missing side
    # segmentation. Restore only that band's original independent ray-hit
    # optical depth; never synthesize a model, mask or qualification result.
    original_optical_depth = (scene.top-scene.T[:3, 3]) @ scene.T[:3, 2]
    scene.depth[scene.unknown] = np.rint(
        original_optical_depth[scene.unknown]/.0001).astype(np.uint16)
    assert np.all(scene.depth[scene.visible_top] > 0)
    assert np.count_nonzero(scene.visible_side) > 100
    assert np.all(scene.mask[scene.visible_side] == 255)
    scene.measured_side_face = scene.visible_side & (np.abs(scene.local[:, :, 1]+width/2.) < .0002)
    scene.missing_side_face = scene.visible_side & ~scene.measured_side_face
    assert np.count_nonzero(scene.measured_side_face) > 100
    assert np.count_nonzero(scene.missing_side_face) > 100
    return scene



def _independent_face_rgb_producer(monkeypatch, params_override=None):
    """Original camera transport fixture with distinct physical face colors.

    A dark top and light visible side keep the full outer RGB silhouette but
    let the actual current depth/RGB segmenter omit an unmeasured side. No
    published mask, target identity, qualified model or policy is replaced.
    """
    scene = top_only_depth_scene()
    producer, clock, _, joint, params = chain.rgb_interest_node(monkeypatch)
    params['/camera'].update({key: getattr(scene.K, key)
        for key in ('fx', 'fy', 'cx', 'cy', 'width', 'height')})
    params.update(deepcopy(params_override or {}))
    optical_to_link = np.array([[0., 0., 1.], [-1., 0., 0.], [0., -1., 0.]])
    T = scene.T.copy(); T[:3, :3] = scene.T[:3, :3] @ optical_to_link.T
    monkeypatch.setattr(chain.producer_module, 'transform_matrix',
        lambda args: T.copy() if args[0] == 'base_link' else np.eye(4))
    # White tabletop plus a colored quadrilateral mat keeps every outer box
    # edge visible while the white side also has a real current background
    # color explanation. The segmenter must resolve ownership from real
    # current depth; no foreground/background mask labels are injected.
    rgb = np.full(scene.depth.shape+(3,), (240, 240, 240), np.uint8)
    # Fixed physical tabletop mat vertices in base coordinates. They were
    # chosen independently of the producer from pinhole/support ray geometry.
    # The mat stays fixed through every frame and side-depth dropout.
    yy, xx = np.indices(scene.depth.shape)
    optical_rays = np.stack(((xx-scene.K.cx)/scene.K.fx,
        (yy-scene.K.cy)/scene.K.fy, np.ones_like(xx)), axis=-1)
    base_rays = optical_rays @ scene.T[:3, :3].T
    times = -scene.T[2, 3]/base_rays[:, :, 2]
    support_points = scene.T[:3, 3]+base_rays*times[:, :, None]
    mat_vertices = np.array([[-.06240301742927101, .03439986694828816],
        [.03837193391547428, .02924584302425708],
        [.03837193391547428, -.027059237938331315],
        [-.06240301742927101, -.03182791427925726]])
    mat = np.ones(scene.depth.shape, bool)
    for first, last in zip(mat_vertices, np.roll(mat_vertices, -1, axis=0)):
        edge = last-first; delta = support_points[:, :, :2]-first
        mat &= edge[0]*delta[:, :, 1]-edge[1]*delta[:, :, 0] <= 1e-12
    rgb[mat] = (60, 90, 190)
    rgb[scene.visible_top] = (20, 20, 20)
    rgb[scene.visible_side] = (240, 240, 240)
    rgb[scene.measured_side_face] = (20, 20, 20)
    mount = np.eye(4); mount[:3, :3] = optical_to_link.T; mount[2, 3] = -.08
    sources = []
    def feed():
        stamp = clock[0]-.1
        pair = (producer.bridge.cv2_to_imgmsg(rgb, encoding='bgr8'),
                producer.bridge.cv2_to_imgmsg(scene.depth, encoding='16UC1'))
        for message in pair:
            message.header.stamp = chain.producer_module.rospy.Time.from_sec(stamp)
            message.header.frame_id = producer.camera_frame
        joint(stamp-.05); joint(stamp+.05)
        producer.latest = pair; producer.process(None); sources.append(pair)
        return pair
    for _ in range(3):
        feed(); clock[0] += .2
    ready = chain.publication(producer)
    assert ready['object'].detected, ready['status']
    assert producer.tracker.anchor is not None and not producer.tracker.lost
    return NS(scene=scene, producer=producer, clock=clock, joint=joint, params=params,
        feed=feed, sources=sources, rgb=rgb, T_link=T, mount=mount, ready=ready)


def _masked_source_geometry(context, row):
    """Use the published mask for target depth, preserving full support depth.

    GEN35's analytic helper gives the full desktop as target_depth_raw. The
    real snapshot fuser masks that input. Adapt only the source provider here;
    the original geometry estimator, full-depth support and gates stay active.
    """
    from alicia_flexible_grasp.vision.object_geometry import estimate_object_geometry
    mask = context.producer.bridge.imgmsg_to_cv2(row['mask'], 'mono8')
    ys, xs = np.nonzero(mask)
    bbox = (int(xs.min()), int(ys.min()), int(xs.max()-xs.min()+1), int(ys.max()-ys.min()+1))
    target_depth = np.where(mask > 0, context.scene.depth, 0)
    assert not np.any(target_depth[mask == 0])
    geometry = estimate_object_geometry(context.scene.depth, target_depth, mask, bbox,
        context.scene.K, .0001, context.scene.T, 'instance_mask', .35, .003, .001,
        60, 60, .004, .20, .20, None)
    assert geometry.ok, (geometry.failure_code, geometry.failure_reason)
    return geometry, mask, bbox


def _install(monkeypatch, *, manual=False):
    monkeypatch.setattr(chain, '_geometry', _masked_source_geometry)
    monkeypatch.setattr(chain, 'ray_scene', top_only_depth_scene)
    _install_real_reached_source_adapter(monkeypatch)
    original_producer = _independent_face_rgb_producer
    def staged_producer(*args, **kwargs):
        context = original_producer(*args, **kwargs)
        request = context.producer._observation_measurement_request_cb
        context.gen43_side_dropout_applied = False
        def measured_request(message):
            state = json.loads(message.data)['state']
            if state == 'requalifying' and not context.gen43_side_dropout_applied:
                # Simulate missing raw depth on the independently ray-cast
                # visible side. Keep all RGB pixels and measured top depth.
                missing = context.scene.missing_side_face
                assert np.count_nonzero(missing) > 100
                context.scene.depth = context.scene.depth.copy()
                context.scene.depth[missing] = 0
                context.gen43_missing_side_depth = missing
                context.gen43_side_dropout_applied = True
            return request(message)
        monkeypatch.setattr(context.producer, '_observation_measurement_request_cb', measured_request)
        return context
    monkeypatch.setattr(chain, 'same_scene_producer', staged_producer)
    original_accept = chain.accepted_same_scene
    captured = []

    def accept(context, patch, tmp_path):
        result = original_accept(context, patch, tmp_path)
        # Validate and retain the healthy source-bound evidence before a real
        # fault retires mutable remote caches. This never reinstates a cache.
        context.gen43_healthy_source_and_model = _source_and_model(context)
        context.gen43_healthy_source_validated_before_fault = True
        context.gen43_manual_rebind_calls = []
        context.gen43_manual_causality = None
        captured.append(context)
        if manual:
            invoke = context.task._invoke_plan_bound_action
            def manual_input_at_rebind(plan, cfg, stage_label, action):
                if stage_label == 'near-field contact rebind':
                    # Every GEN35 positive stream-admission assertion has now
                    # completed. Change only the live input immediately before
                    # the original atomic validation/freeze boundary.
                    assert context.task._station_contact_proposal == context.contact
                    assert not context.task._frozen_contact_pub.messages
                    assert {'duplicate_preview_audit', 'duplicate_start_rejected'} <= set(context.injected_events)
                    assert not context.gen43_manual_rebind_calls
                    task = context.task
                    record = task._station_task_record
                    candidate = task._station_contact_rebind_candidate
                    assert record is not None and candidate == context.contact
                    # Check the actual station gate on this healthy owner
                    # before changing its sole live input. These are read-only
                    # validation calls; the atomic action is never submitted.
                    assert task._station_task_plan_failure(candidate) is None
                    task._station_task_context(record['contract'], admission=True)
                    def inputs():
                        source = context.ticket.payload[0]
                        raw = source.original_sample
                        return dict(plan_id=context.contact.plan_id,
                            bound_plan_id=task._bound_execution_plan_id,
                            rebind_plan_id=task._station_contact_rebind_candidate.plan_id,
                            source_stamp_ns=source.stamp_ns,
                            source_target_epoch=source.target_identity.epoch,
                            source_track_id=source.target_identity.track_id,
                            color_sha256=hashlib.sha256(np.ascontiguousarray(raw.color_bgr).tobytes()).hexdigest(),
                            depth_sha256=hashlib.sha256(np.ascontiguousarray(raw.depth_raw).tobytes()).hexdigest(),
                            mask_sha256=hashlib.sha256(np.ascontiguousarray(raw.object_mask).tobytes()).hexdigest(),
                            selection=deepcopy(task._mode_bound_selection),
                            epoch_ns=task._following_epoch_ns,
                            station_record_owner=id(task._station_task_record),
                            station_contract=deepcopy(task._station_task_record['contract']))
                    before = inputs(); parameters_before = deepcopy(context.params)
                    assert context.params.get('/gui/joint_direct_mode', False) is False
                    context.params['/gui/joint_direct_mode'] = True
                    after = inputs()
                    parameter_changes = {key for key in set(parameters_before) | set(context.params)
                        if parameters_before.get(key) != context.params.get(key)}
                    assert parameter_changes == {'/gui/joint_direct_mode'} and before == after
                    causal = dict(healthy_station_gate_accepted=True,
                        healthy_context_accepted=True, inputs_before=before,
                        inputs_after_manual_before_validation=after,
                        changed_parameter_names=sorted(parameter_changes),
                        manual_before=False, manual_after=True,
                        context_exceptions=[], retirement=[])
                    context.gen43_manual_causality = causal
                    original_context, original_retire = task._station_task_context, task._station_task_retire
                    def observed_context(*args, **kwargs):
                        try:
                            return original_context(*args, **kwargs)
                        except ValueError as exc:
                            causal['context_exceptions'].append(dict(type=type(exc).__name__,
                                reason=str(exc), manual=context.params['/gui/joint_direct_mode'],
                                record_still_original=task._station_task_record is record))
                            raise
                    def observed_retire(reason):
                        row = dict(reason=reason, record_still_original=task._station_task_record is record)
                        causal['retirement'].append(row)
                        value = original_retire(reason)
                        row.update(retired=task._station_task_retired,
                            record_cleared=task._station_task_record is None,
                            target_loss=deepcopy(task._unknown_target_loss))
                        return value
                    context.injected_events.append('manual_control_taken_at_contact_rebind')
                    with patch.context() as trace:
                        trace.setattr(task, '_station_task_context', observed_context)
                        trace.setattr(task, '_station_task_retire', observed_retire)
                        actual = invoke(plan, cfg, stage_label, action)
                    context.gen43_manual_rebind_calls.append(dict(
                        stage_label=stage_label, code=actual[0].code,
                        ok=actual[0].ok, frozen=actual[1] is not None))
                    return actual
                return invoke(plan, cfg, stage_label, action)
            patch.setattr(context.task, '_invoke_plan_bound_action', manual_input_at_rebind)
        return result

    monkeypatch.setattr(chain, 'accepted_same_scene', accept)
    return captured


def _source_and_model(context):
    from alicia_flexible_grasp.grasp.contact_evidence import decode_contact_evidence
    from alicia_flexible_grasp.vision.regular_box_inference import RegularBoxInference

    source = context.ticket.payload[0]
    retained = source.original_sample
    evidence = decode_contact_evidence(context.contact, required=True)
    assert evidence['measurement_source'] == 'regular_box_model_inference'
    model = RegularBoxInference.from_evidence_dict(evidence['box_model_evidence'])
    interval = model.qualification['cuboid_rgb_interval']
    # These are the meaningful regression conditions: producer segmentation
    # omits a visible side while its retained RGB still contains that side.
    missing_side = context.scene.visible_side & (retained.object_mask == 0)
    assert context.gen43_side_dropout_applied
    assert np.count_nonzero(missing_side) > 100
    assert np.all(retained.color_bgr[context.scene.missing_side_face] == (240,240,240))
    assert np.all(retained.color_bgr[context.scene.measured_side_face] == (20,20,20))
    assert np.array_equal(retained.color_bgr, context.rgb)
    assert np.all(retained.depth_raw[context.gen43_missing_side_depth] == 0)
    assert interval['silhouette_mask_coverage'] < .88
    assert interval['top_mask_coverage'] >= .88
    assert max(interval['full_silhouette_rgb_distances_px']) <= np.sqrt(8.)
    assert max(interval['corner_rgb_distances_px']) <= np.sqrt(8.)
    assert interval['source_color_sha256'] == hashlib.sha256(
        np.ascontiguousarray(retained.color_bgr).tobytes()).hexdigest()
    assert model.source['stamp_ns'] == source.stamp_ns == context.current_row['object'].header.stamp.to_nsec()
    assert model.source['target_identity']['track_id'] == context.identity.track_id
    assert context.remote._regular_box_snapshot['result'].audit['measured_side_surfaces'] is False
    assert evidence['required_open_width_m'] <= .05
    assert context.contact.required_open_width_m <= .05
    return interval, evidence


@pytest.mark.parametrize('order', [
    ('status','ack','object','mask'),
    ('mask','object','status','ack'),
    ('ack','mask','status','object'),
])
def test_partial_side_mask_keeps_full_rgb_and_traverses_original_common_handoff_once(
        monkeypatch, tmp_path, order):
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path/'runtime_logs'))
    captured = _install(monkeypatch)
    context = chain.unified_replay(monkeypatch, tmp_path, order, perturb=True)
    assert len(captured) == 1 and captured[0] is context
    interval, evidence = _source_and_model(context)
    from alicia_flexible_grasp.grasp.rich_plan_integrity import compute_plan_id
    assert context.contact.valid and compute_plan_id(context.contact) == context.contact.plan_id
    assert [row['state'] for row in context.requests] == ['executing','requalifying','qualified']
    assert {'qualified_then_late_pending_and_duplicate_ack','late_ancestor_clock_preview',
        'duplicate_preview_audit','duplicate_start_rejected'} <= set(context.injected_events)
    observation = [row for row in context.device_calls
        if row[0] == 'motion' and row[1] == context.origin.plan_id]
    contact = [row for row in context.device_calls
        if row[0] == 'motion' and row[1] == context.contact.plan_id]
    close = [row for row in context.device_calls if row[0] == 'gripper' and row[2] < .05]
    assert len(observation) == 1 and len(contact) == 3 and len(close) == 1
    assert len(context.task._frozen_contact_pub.messages) == 1
    assert context.execution_result is False and context.states[-1].state == 'HOLDING'
    assert 'GRASP_HOLD_UNVERIFIED' in context.states[-1].message
    assert not any(row[0] == 'lift' for row in context.device_calls)
    (tmp_path/'gen43_top_mask_handoff.json').write_text(json.dumps(dict(
        source_stamp_ns=context.ticket.payload[0].stamp_ns,
        source_rgb_preserved=True, actual_producer_segmentation_used=True,
        whole_mask_coverage=interval['silhouette_mask_coverage'],
        top_mask_coverage=interval['top_mask_coverage'],
        required_open_width_m=evidence['required_open_width_m'],
        injected_events=context.injected_events, physical_grasp_verified=False,
        original_source_geometry_registration_qualification_and_task_gates=True,
        original_worker_main_claimed=False, device_and_transport_providers='explicit simulators'),
        indent=2)+'\n')


@pytest.mark.parametrize('change', ['manual_control','target_geometry_conflict'])
def test_qualified_top_mask_never_bypasses_real_control_or_target_revocation(
        monkeypatch, tmp_path, change):
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path/'runtime_logs'))
    captured = _install(monkeypatch, manual=change == 'manual_control')
    context = chain.unified_replay(monkeypatch, tmp_path,
        ('mask','object','status','ack'), perturb=True,
        fault='same_source_conflict' if change == 'target_geometry_conflict' else None)
    assert len(captured) == 1 and captured[0] is context
    assert context.gen43_healthy_source_validated_before_fault
    interval, _evidence = context.gen43_healthy_source_and_model
    # The contact proof itself remains immutable; retired mutable cache state
    # is intentionally unavailable and must not be reconstructed or restored.
    from alicia_flexible_grasp.grasp.contact_evidence import decode_contact_evidence
    assert decode_contact_evidence(context.contact, required=True) == _evidence
    assert context.execution_result is False and context.states[-1].state == 'FAILED'
    assert sum(row[0] == 'motion' for row in context.device_calls) == 1
    assert not any(row[0] == 'gripper' and row[2] < .05 for row in context.device_calls)
    assert not context.task._frozen_contact_pub.messages
    if change == 'manual_control':
        assert context.gen43_manual_rebind_calls == [dict(
            stage_label='near-field contact rebind', code='TARGET_LOST',
            ok=False, frozen=False)]
        assert 'manual_control_taken_at_contact_rebind' in context.injected_events
        causal = context.gen43_manual_causality
        assert causal['healthy_station_gate_accepted'] and causal['healthy_context_accepted']
        assert causal['changed_parameter_names'] == ['/gui/joint_direct_mode']
        assert causal['manual_before'] is False and causal['manual_after'] is True
        assert causal['inputs_before'] == causal['inputs_after_manual_before_validation']
        assert causal['context_exceptions'] == [dict(type='ValueError',
            reason='station_task_owner_changed', manual=True, record_still_original=True)]
        assert causal['retirement'] == [dict(reason='station_task_owner_changed',
            record_still_original=True, retired=True, record_cleared=True,
            target_loss=dict(selection=context.task._mode_bound_selection,
                reason='TARGET_LOST: station_task_owner_changed'))]
        assert context.task._station_task_retired and context.task._station_task_record is None
        assert context.task._unknown_target_loss == dict(
            selection=context.task._mode_bound_selection,
            reason='TARGET_LOST: station_task_owner_changed')
        # Existing station-owner retirement precedes the base MANUAL check.
        # This is control revocation, not evidence of actual target movement.
        assert any(row.message == 'TARGET_LOST: station wait was permanently retired before near-field contact rebind'
            for row in context.states), context.states
    else:
        assert context.remote._regular_box_snapshot is None
        assert not context.gen43_manual_rebind_calls
        assert context.task._execution_authority_revoked
        assert 'same_source_geometry_conflict' in context.injected_events
    (tmp_path/'gen43_top_mask_true_revocation.json').write_text(json.dumps(dict(
        change=change, whole_mask_coverage=interval['silhouette_mask_coverage'],
        top_mask_coverage=interval['top_mask_coverage'],
        states=[dict(state=row.state,message=row.message) for row in context.states],
        actual_common_plan_was_qualified=True, healthy_source_validated_before_fault=True,
        manual_rebind_calls=context.gen43_manual_rebind_calls,
        manual_causality=context.gen43_manual_causality, contact_motions=0, closes=0,
        physical_grasp_verified=False), indent=2)+'\n')
