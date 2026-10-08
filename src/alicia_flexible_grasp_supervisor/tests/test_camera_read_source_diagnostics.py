"""Exact source association with fake endpoints and unchanged publication."""
import copy
import json
import threading
import xmlrpc.client

import pytest

from test_camera_async_publication import camera, BASE_STAMP_NS, wait_for


def diagnostic(number=10):
    raw = dict(present=True, frame_number=number, device_timestamp_ms=1791284753317.1453,
        timestamp_domain='timestamp_domain.global_time', stream_fps=30.,
        metadata_raw=dict(frame_timestamp=3516079678, backend_timestamp=1791284753318),
        unavailable={}, profile_signature={'not_repeated_in_recent_history':True})
    return dict(schema=2, result='complete', failed_stage=None, total_sec=.03,
        raw_frames={'color':copy.deepcopy(raw), 'depth':copy.deepcopy(raw)},
        host_wait={'enter':{'wall_time_sec':1791284753.3,'monotonic_sec':9.97},
                   'exit':{'wall_time_sec':1791284753.32,'monotonic_sec':9.99}},
        pair_timing=dict(accepted=True, reason='within_nominal_pair_tolerance',skew_ms=.3),
        stages_sec={'wait_for_frames':.02,'regional_stationary_depth':.005},
        sdk_color_alignment_applied=True, stationary_pair_filter_host_monotonic=9.995,
        regional_stationary_evidence=dict(filtered_pixel_count=99,
            global_history_reset_reason=None,
            regions=[dict(name='center',filter_applied=True,qualified=True,
                visual=dict(continuous_frames=15,reason='stationary',motion_p90_px=.13))]))


def enable(node):
    node.cam.runtime_profile['stationary_temporal']={'active':False,'regional_enabled':True}
    node.cam.last_read_diagnostics=diagnostic()


def entries(node):
    return decoded(node)['entries']


def decoded(node):
    return json.loads(node._recent_successful_read_parameters()['payload_json'])


def test_actual_read_completion_stamp_is_reused_even_when_publication_not_due(camera):
    node,ros,_=camera;enable(node);original=node.cam.read;count=[0]
    def read():
        count[0]+=1;node.cam.last_read_diagnostics=diagnostic(count[0])
        if count[0]==2:ros.shutdown=True
        return original()
    node.cam.read=read;due=iter([False,True]);node._publication_is_due=lambda:next(due)
    node.spin();wait_for(lambda:node._publication_worker.summary()['published']==1)
    rows=entries(node)
    assert ros.time_calls==2
    assert [r['source_stamp_ns'] for r in rows]==[BASE_STAMP_NS,BASE_STAMP_NS+101]
    assert [r['evidence']['raw_frames']['color']['frame_number'] for r in rows]==[1,2]
    assert all(r['publication_outcome_unknown'] is True for r in rows)
    assert all(r['exposure_clock_calibrated'] is False for r in rows)
    assert node.pub_color.messages[0].header.stamp.to_nsec()==BASE_STAMP_NS+101
    assert node.pub_depth.messages[0].header.stamp.to_nsec()==BASE_STAMP_NS+101
    assert int(node.pub_depth.messages[0].image[0,0])==100


def test_history_is_32_entries_and_xmlrpc_preserves_exact_source_and_sdk_values(camera):
    node,ros,_=camera;enable(node);before=copy.deepcopy(node.cam.last_read_diagnostics)
    for i in range(40):node._remember_successful_read(BASE_STAMP_NS+i,10.+i/30.)
    node._publish_temporal_status(None)
    status=ros.params['/camera_read_diagnostics']
    envelope=xmlrpc.client.loads(xmlrpc.client.dumps((status,)))[0][0]
    assert envelope['schema']==1 and envelope['encoding']=='json'
    history=json.loads(envelope['payload_json'])
    rows=history['entries']
    assert history['capacity']==32 and len(rows)==32
    assert rows[0]['source_stamp_ns']==BASE_STAMP_NS+8
    assert rows[-1]['source_stamp_ns']==BASE_STAMP_NS+39
    assert rows[-1]['evidence']['raw_frames']['depth']['metadata_raw']['frame_timestamp']==3516079678
    assert rows[-1]['evidence']['failed_stage'] is None
    assert rows[-1]['evidence']['regional_stationary_evidence']['global_history_reset_reason'] is None
    assert rows[-1]['evidence']['raw_frames']['color']['metadata_raw']==before['raw_frames']['color']['metadata_raw']
    assert history['copy_errors']==0 and history['motion_authorized'] is False
    assert node.cam.last_read_diagnostics==before
    assert 'recent_successful_reads' not in node.cam.runtime_profile['stationary_temporal']
    assert 'recent_successful_reads' not in ros.params['/camera/runtime_profile/stationary_temporal']
    assert 'camera_read_diagnostics' not in ros.params['/camera']
    assert ros.time_calls==0  # A diagnostic never manufactures a source stamp.
    assert node.pub_color.messages==node.pub_depth.messages==[]


def test_later_raw_mutation_and_parameter_mutation_do_not_change_frozen_evidence(camera):
    node,_,_=camera;enable(node);node._remember_successful_read(BASE_STAMP_NS,10.)
    node.cam.last_read_diagnostics['raw_frames']['color']['frame_number']=999
    node.cam.last_read_diagnostics['regional_stationary_evidence']['regions'][0]['filter_applied']=False
    first=entries(node);assert first[0]['evidence']['raw_frames']['color']['frame_number']==10
    assert first[0]['evidence']['regional_stationary_evidence']['regions'][0]['filter_applied'] is True
    first[0]['evidence']['stages_sec']['regional_stationary_depth']=700.
    assert entries(node)[0]['evidence']['stages_sec']['regional_stationary_depth']==.005


@pytest.mark.parametrize('change',['geometry','stop'])
def test_geometry_replacement_or_camera_stop_clears_old_source_history(camera,change):
    node,_,_=camera;enable(node);node._remember_successful_read(BASE_STAMP_NS,10.)
    previous=decoded(node)['geometry_generation']
    if change=='geometry':
        node.cam.runtime_profile['geometry_intrinsics']['fx']=77.
        node._publish_runtime_camera_params()
    else:node._stop_camera()
    result=decoded(node)
    assert result['entries']==[] and result['geometry_generation']>previous


def test_source_copy_in_flight_cannot_cross_geometry_reset(camera,monkeypatch):
    node,_,module=camera;enable(node);entered=threading.Event();release=threading.Event()
    original=module._compact_read_evidence
    def blocked(raw):
        entered.set();assert release.wait(2.);return original(raw)
    monkeypatch.setattr(module,'_compact_read_evidence',blocked)
    worker=threading.Thread(target=lambda:node._remember_successful_read(BASE_STAMP_NS,10.))
    try:
        worker.start();assert entered.wait(1.)
        node._clear_recent_successful_reads();release.set();worker.join(1.)
        assert not worker.is_alive() and entries(node)==[]
        assert decoded(node)['copy_errors']==1
    finally:release.set();worker.join(1.)


def test_completed_old_read_is_not_bound_to_replacement_camera_diagnostics(camera):
    node,_,_=camera;enable(node);old=node.cam;generation=node._recent_read_generation
    node._start_camera(False);enable(node)
    node.cam.last_read_diagnostics['raw_frames']['color']['frame_number']=999
    node._remember_successful_read(BASE_STAMP_NS,10.,camera=old,geometry_generation=generation)
    assert entries(node)==[] and decoded(node)['copy_errors']==1


@pytest.mark.parametrize('blocked_parameter',['/camera/runtime_profile/stationary_temporal','/camera_read_diagnostics'])
def test_blocked_xmlrpc_does_not_hold_read_history_lock(camera,blocked_parameter):
    node,ros,_=camera;enable(node);entered=threading.Event();release=threading.Event()
    original=ros.set_param
    def blocked(name,value):
        if name==blocked_parameter:
            entered.set();assert release.wait(2.)
        original(name,value)
    ros.set_param=blocked
    timer=threading.Thread(target=lambda:node._publish_temporal_status(None))
    def remember():
        for i in range(40):node._remember_successful_read(BASE_STAMP_NS+i,10.)
    reader=threading.Thread(target=remember)
    try:
        timer.start();assert entered.wait(1.);reader.start();reader.join(.5)
        assert not reader.is_alive() and len(entries(node))==32
    finally:release.set();timer.join(1.);reader.join(1.)


def test_metadata_copy_failure_does_not_recover_camera_or_change_publication(camera):
    node,ros,_=camera;enable(node);original=node.cam.read
    class BadMetadata:
        def __deepcopy__(self,memo):raise ValueError('injected diagnostic copy failure')
    node.cam.last_read_diagnostics['stages_sec']['injected']=BadMetadata()
    def read():ros.shutdown=True;return original()
    node.cam.read=read;node._publication_is_due=lambda:True
    node.spin();wait_for(lambda:node._publication_worker.summary()['published']==1)
    assert node._camera_started and node._read_failures==0
    assert entries(node)==[] and decoded(node)['copy_errors']==1
    assert node.pub_color.messages[0].header.stamp.to_nsec()==BASE_STAMP_NS
    assert node.pub_depth.messages[0].header.stamp.to_nsec()==BASE_STAMP_NS


def test_rejected_pair_creates_neither_stamp_nor_successful_read_record(camera):
    node,ros,_=camera;enable(node)
    def read():
        node.cam.last_read_diagnostics={'result':'rejected_pair'}
        ros.shutdown=True;return None,None
    node.cam.read=read;node.spin()
    assert entries(node)==[] and ros.time_calls==0
    assert node.pub_color.messages==node.pub_depth.messages==[]


def test_missing_diagnostic_is_explicit_and_extra_pixel_payload_is_never_copied(camera):
    node,_,module=camera;enable(node)
    class PixelsForbidden:
        def __deepcopy__(self,memo):raise AssertionError('pixels must not be copied')
    raw=diagnostic();raw['pixels']=PixelsForbidden()
    result=module._compact_read_evidence(raw)
    assert result['available'] and 'pixels' not in result
    assert 'profile_signature' not in result['raw_frames']['color']
    node.cam.last_read_diagnostics=None;node._remember_successful_read(BASE_STAMP_NS,10.)
    assert entries(node)[0]['evidence']==dict(available=False,reason='complete_read_diagnostic_unavailable')
