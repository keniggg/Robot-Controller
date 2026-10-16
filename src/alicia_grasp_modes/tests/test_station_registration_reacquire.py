"""Real recorded registration clouds through real preparation/worker/reacquisition.

The station/measurement transport has synthetic timestamps and images. Only the
geometry estimator and external inference transport are substituted. Exact GEN18
point clouds and the original registration config are replayed without changes.
Passing registration is not a claim of contact/path/grasp success.
"""
from copy import deepcopy
from dataclasses import replace
import json
from pathlib import Path
import threading
import time
from types import SimpleNamespace as NS

import numpy as np
import pytest

from test_station_compute_wait_remote import station, clock, feedback
from test_observation_measurement_remote import context
from test_station_compute_work_handoff import submitted, next_row
from test_remote_modes import base
from alicia_flexible_grasp.vision.multiview_surface import (RegistrationConfig,
    SurfaceView, fused_surface_from_view, append_registered_view)
from alicia_flexible_grasp.vision.object_geometry import GeometryEstimate

FIXTURES = Path(__file__).parent / 'fixtures'
RECORDED = json.loads((FIXTURES / 'gen18_registration_reacquire.json').read_text())


def setup_recorded_geometry(c, monkeypatch, ticket):
    n = c.node
    n.candidate_frame_convention = 'ros_camera_link'
    n.gripper_geometry = base.GripperGeometry(.05, .002, (.0434,.0286,.06), (.1175,.155,.0774), .002)
    data = np.load(str(FIXTURES / 'gen18_registration_reacquire_clouds.npz'))
    ns = ticket.payload[0].stamp_ns
    reference = SurfaceView(n._current_stream_target_identity(), ns-2_000_000_000,
        data['reference_points'], data['reference_support_normal'],
        float(data['reference_support_offset']), 30)
    surface = fused_surface_from_view(reference)
    # Synthetic successful reached-view provenance, using measured reference
    # samples unchanged. This does not reconstruct GEN18's whole fused surface.
    result, surface = append_registered_view(surface, reference,
        replace(reference, stamp_ns=ns-1_000_000_000))
    assert result.ok
    n._near_field_fused_surface = surface
    n._near_field_reference_view = reference
    n._near_field_surface_phase_id = n._near_field_phase_id
    n._near_field_surface_generation = n._stream_generation
    n.multiview_registration_config = RegistrationConfig(**RECORDED['offline_replay']['config'])
    state = dict(kind='failed', candidates=[], inference=[])
    box = RECORDED['requests']['42']['geometry']

    def estimate(**kwargs):
        kind = state['kind']
        axes=np.array(box['R_base_obb']); center=np.array(box['obb_center_base_m'])
        size=np.array(box['obb_size_xyz_m'])
        if kind == 'new_disjoint':
            # The recorded fixture stores exact points/plane, not its OBB.
            # Form a conservative measured box for estimator transport only.
            z=data[kind+'_support_normal']; x=axes[:,0]-z*np.dot(axes[:,0],z)
            x=x/np.linalg.norm(x); axes=np.column_stack((x,np.cross(z,x),z))
            local=data[kind+'_points']@axes; low=local.min(axis=0); high=local.max(axis=0)
            center=axes@((low+high)/2); size=high-low
        return GeometryEstimate(True, '', '', center, axes, size,
            data[kind+'_support_normal'], float(data[kind+'_support_offset']),
            .9, data[kind+'_points'], 'instance_mask')

    monkeypatch.setattr(base, 'estimate_object_geometry', estimate)
    monkeypatch.setattr(n, '_snapshot_base_optical_transform', lambda *_: np.eye(4))
    monkeypatch.setattr(n, '_camera_intrinsics', lambda: NS(depth_scale=.001))
    monkeypatch.setattr(n, '_require_graspnet_input_prerequisites', lambda *_: None)
    monkeypatch.setattr(n, '_refresh_runtime_params', lambda: None)
    monkeypatch.setattr(n, '_capture_silhouette_source', lambda *_: None)
    monkeypatch.setattr(n, '_add_silhouette_source_diagnostic', lambda p, *_: p)
    monkeypatch.setattr(n, '_add_silhouette_envelope_diagnostic', lambda p: p)
    monkeypatch.setattr(n, '_gripper_contract_mismatch_reason', lambda: '')
    monkeypatch.setattr(n, '_maybe_prepare_observation_from_contact_geometry', lambda *_: False)
    monkeypatch.setattr(n, '_build_frozen_graspnet_input', lambda *_a, **_k: (NS(), {}))

    def candidates(*a, **k):
        state['candidates'].append(k['snapshot'].stamp_ns)
        return (), {}
    def inference(*a, **k):
        state['inference'].append(a[0].request_id)
        return (), {}, {}, 0., 0., 0.
    monkeypatch.setattr(n, '_generate_tabletop_candidates', candidates)
    monkeypatch.setattr(n, '_predict_remote', inference)
    return state, surface


def wait_metrics(n, count):
    deadline = time.perf_counter()+5.
    while time.perf_counter()<deadline:
        with n._stream_condition:
            if len(n.pipeline_metrics_pub.messages)>=count and not n._stream_worker_busy:
                break
        time.sleep(.002)
    with n._stream_condition:
        assert not n._stream_worker_busy
        assert len(n.pipeline_metrics_pub.messages) >= count
    return json.loads(n.pipeline_metrics_pub.messages[count-1].data)


@pytest.mark.parametrize('failed_batches', [1, 2])
def test_recorded_registration_real_worker_releases_retained_work_and_rejoins_three_new_frames(
        station, monkeypatch, failed_batches):
    c=station; n=c.node
    start,row,ticket=submitted(c)
    assert n._station_compute_work_certificate(ticket)  # Never withheld.
    state,surface=setup_recorded_geometry(c,monkeypatch,ticket)
    accepted=[]; failures=[]
    monkeypatch.setattr(n, '_accept_prediction', accepted.append)
    monkeypatch.setattr(n, '_observe_stream_prediction_failure', lambda *a: failures.append(a))
    generation=n._stream_generation; identity=n._current_stream_target_identity()
    phase_deadline=n._near_field_phase_deadline_sec
    session_deadline=n._observe_return_session['deadline_ns']
    n._stream_worker=threading.Thread(target=n._stream_worker_main,daemon=True)
    n._stream_worker.start()
    try:
        for batch in range(failed_batches):
            metrics=wait_metrics(n,batch+1)
            assert metrics['status']=='TARGET_PROCESSING_PENDING', (metrics, failures)
            assert not accepted and not failures
            assert state['candidates']==[] and state['inference']==[]
            assert n._direct_near_field_submission_generation is None
            assert n._near_field_fused_surface is surface
            assert not n.latest_object_geometry.valid and not n.latest_preview_rich_plan.valid
            assert n._observe_return_registration_ticket is None
            diagnostic=n._observe_return_registration_pending
            expected=RECORDED['offline_replay']['failed_window']['registration']
            assert diagnostic['rotation_deg']==pytest.approx(expected['rotation_deg'],abs=1e-9)
            assert diagnostic['rmse_m']==pytest.approx(expected['rmse_m'],abs=1e-12)
            assert diagnostic['code']=='YAW_BOUND_EXCEEDED'
            assert diagnostic['source_stamp_ns']==n.last_submitted_stamp_ns
            assert n._stream_generation==generation and n._current_stream_target_identity()==identity
            assert n._near_field_phase_deadline_sec==phase_deadline
            assert n._observe_return_session['deadline_ns']==session_deadline
            watermark=n.last_submitted_stamp_ns
            assert not n.submit_stream_snapshot(ticket.payload[0],require_idle_worker=True)
            state['kind']='new_disjoint' if batch+1==failed_batches else 'failed'
            for j in range(3):
                row=next_row(c,start+(3*(batch+1)+j)*400_000_000,row['status']['station_compute_wait'])
                assert n._poll_stream_snapshot() is (j==2)
                if j<2:
                    assert n.last_submitted_stamp_ns==watermark
        metrics=wait_metrics(n,failed_batches+1)
        assert metrics['status']=='ACCEPTED',(metrics, failures)
        assert len(accepted)==1 and not failures
        prepared=accepted[0]
        assert all(ns>watermark for ns in prepared.snapshot.sample_stamp_ns)
        assert prepared.ticket.request_id!=ticket.request_id
        assert prepared.snapshot.stamp_ns in n._near_field_fused_surface.view_stamps_ns
        assert ticket.payload[0].stamp_ns not in n._near_field_fused_surface.view_stamps_ns
        expected=RECORDED['offline_replay']['new_disjoint_window']['registration']
        result=n._latest_registration_evidence.result
        assert result.ok and result.code=='REGISTERED'
        assert n._registration_rotation_deg(result.transform_base)==pytest.approx(expected['rotation_deg'],abs=1e-9)
        assert result.rmse_m==pytest.approx(expected['rmse_m'],abs=1e-12)
        assert state['candidates']==[prepared.snapshot.stamp_ns]
        assert n._near_field_phase_deadline_sec==phase_deadline
        assert n._observe_return_session['deadline_ns']==session_deadline
        assert not n.latest_preview_rich_plan.valid  # No execution/old plan publication.
    finally:
        n.shutdown_streaming_worker()


def test_recorded_window_provenance_is_fully_disjoint_and_original_gate_unchanged():
    replay=RECORDED['offline_replay']
    assert min(replay['new_disjoint_window']['source_stamps_ns'])>max(replay['failed_window']['source_stamps_ns'])
    assert replay['config']['maximum_yaw_deg']==10.
    assert replay['failed_window']['recorded_points_array_equal'] is True
