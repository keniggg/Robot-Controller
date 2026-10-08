"""Source-only permission to park private history, never image admission."""
import copy
import json
import xmlrpc.client

import pytest

from alicia_flexible_grasp.vision.pair_history_continuity import (
    interleaved_pair_reason, successor_pair_reason)
from alicia_flexible_grasp.vision.realsense_manager import _parameter_pair_history
from test_stationary_pair_history_independent import Rig


def pair(index):
    def stream(name, offset):
        return dict(present=True, frame_number=offset+index,
            device_timestamp_ms=1000.+index*1000./30., stream_fps=30.,
            timestamp_domain='global_time', metadata_raw={'sensor_timestamp':index},
            profile_signature=dict(stream_type=name, stream_index=0,
                format='bgr8' if name=='color' else 'z16', unique_id=offset,
                width=640, height=480, fx=400., fy=400., ppx=320., ppy=240.,
                coeffs=[0.]*5, model='none', fps=30.))
    return {name:stream(name,offset) for name,offset in (('color',100),('depth',1000))}


@pytest.mark.parametrize('repeated',['color','depth'])
def test_exact_interleaving_parks_but_is_never_a_complete_successor(repeated):
    old,new=pair(0),pair(1)
    bad=copy.deepcopy(new);bad[repeated]=copy.deepcopy(old[repeated])
    frozen=copy.deepcopy((old,bad,new))
    assert interleaved_pair_reason(old,bad,0.,.02) is None
    assert successor_pair_reason(old,bad,0.,.02) is not None
    assert successor_pair_reason(old,new,0.,1./30.) is None
    assert interleaved_pair_reason(old,new,0.,1./30.) is not None
    assert (old,bad,new)==frozen


@pytest.mark.parametrize('field,value',[
    ('frame_number',True),('frame_number',1.5),('device_timestamp_ms',float('nan')),
    ('stream_fps',float('inf')),('timestamp_domain','system_time'),
    ('profile_signature',None),('present',1)])
def test_missing_or_invalid_source_is_not_permission(field,value):
    old,new=pair(0),pair(1);new['color'][field]=value
    assert successor_pair_reason(old,new,0.,1./30.) is not None


def test_repeat_requires_original_metadata_not_just_same_counter_and_stamp():
    old,bad=pair(0),pair(1)
    bad['color']=copy.deepcopy(old['color'])
    bad['color']['metadata_raw']['sensor_timestamp']=55
    assert interleaved_pair_reason(old,bad,0.,.02)=='not_one_identical_previous_stream'


@pytest.mark.parametrize('host',[-1.,0.,.100001,float('inf'),float('nan'),True])
def test_source_continuity_cannot_renew_or_replace_host_clock(host):
    assert successor_pair_reason(pair(0),pair(1),0.,host) is not None


def test_full_diagnostic_keeps_raw_anchor_rejected_successor_and_original_gates():
    r=Rig().warm();r.quarantine()
    first=copy.deepcopy(r.manager.last_read_diagnostics['stationary_pair_history'])
    assert first['state']=='quarantined'
    assert first['last_complete_raw_frames']==r.last_complete['raw_frames']
    assert first['rejected_raw_frames']==r.manager.last_read_diagnostics['raw_frames']
    assert first['next_accepted_raw_frames'] is None
    r.feed(12)
    decision=r.manager.last_read_diagnostics['stationary_pair_history']
    assert decision['state']=='resumed_for_existing_gates'
    assert decision['rejected_raw_frames']==first['rejected_raw_frames']
    assert decision['next_accepted_raw_frames']==r.manager.last_read_diagnostics['raw_frames']
    assert decision['existing_gate_evaluation']['encoder_active'] is True
    assert decision['existing_gate_evaluation']['filtered_pixel_count']>0
    assert decision['authorizes_pair_or_motion'] is False
    assert _parameter_pair_history(decision)==r.manager.runtime_profile['stationary_temporal']['pair_history']
    json.dumps(decision,allow_nan=False)


def test_repeated_rejection_diagnostic_does_not_lose_exact_reason_or_first_clock():
    r=Rig().warm();r.quarantine()
    first=copy.deepcopy(r.manager.last_read_diagnostics['stationary_pair_history'])
    r.feed(11,12,host=10.47)
    decision=r.manager.last_read_diagnostics['stationary_pair_history']
    assert decision['state']=='discarded'
    assert decision['reason']=='color_host_discontinuity'
    assert decision['first_rejection_host_monotonic']==first['first_rejection_host_monotonic']
    assert decision['last_complete_filter_host_monotonic']==first['last_complete_filter_host_monotonic']
    assert decision['rejected_packet_count']==2
    assert _parameter_pair_history(decision)==r.manager.runtime_profile['stationary_temporal']['pair_history']
    assert r.manager._stationary_pair_quarantine is None


def test_nonadjacent_first_rejection_preserves_actual_failed_packet_for_diagnosis():
    r=Rig().warm();r.feed(11,13,host=10.383)
    decision=r.manager.last_read_diagnostics['stationary_pair_history']
    assert decision['state']=='discarded'
    assert decision['reason']=='depth_not_immediate_successor'
    assert decision['rejected_raw_frames']==r.manager.last_read_diagnostics['raw_frames']
    assert decision['last_complete_raw_frames']==r.last_complete['raw_frames']


def test_parameter_encoding_is_lossless_for_large_sdk_integers_and_marks_nulls():
    raw=dict(timestamp=1791284753318, frame_timestamp=3516079678,
        int32_low=-(2**31), int32_high=2**31-1, below=-(2**31)-1,
        active=True, fractions=[.5,None], source=None)
    before=copy.deepcopy(raw)
    with pytest.raises(OverflowError):
        xmlrpc.client.dumps((raw,))
    parameter=_parameter_pair_history(raw)
    encoded=xmlrpc.client.dumps((parameter,))
    decoded=xmlrpc.client.loads(encoded)[0][0]
    for name in ('timestamp','frame_timestamp','below'):
        assert isinstance(decoded[name],str) and int(decoded[name])==raw[name]
    assert decoded['int32_low']==raw['int32_low']
    assert decoded['int32_high']==raw['int32_high']
    assert decoded['active'] is True
    assert decoded['fractions']==[.5,'']
    assert 'source' not in decoded
    assert decoded['parameter_encoding']['omitted_null_fields']==[['source']]
    assert decoded['parameter_encoding']['empty_string_list_nulls']==[['fractions','1']]
    assert raw==before


@pytest.mark.parametrize('tail',['resume','late_reject','post_filter_error'])
def test_real_manager_status_marshals_without_altering_typed_raw_history(tail):
    r=Rig();raw_reader=r.manager._raw_frame_diagnostics
    def sdk_diagnostics(frames):
        raw=raw_reader(frames)
        for row in raw.values():
            row['metadata_raw'].update(backend_timestamp=1791284753318,
                frame_timestamp=3516079678,time_of_arrival=1791284753325)
        return raw
    r.manager._raw_frame_diagnostics=sdk_diagnostics
    r.warm();r.quarantine()
    first=copy.deepcopy(r.manager.last_read_diagnostics['stationary_pair_history'])
    xmlrpc.client.dumps((r.manager.runtime_profile['stationary_temporal'],))
    if tail=='resume':r.feed(12)
    elif tail=='late_reject':r.feed(11,12,host=10.47)
    else:
        r.failure='rectify'
        with pytest.raises(RuntimeError,match='rectify'):r.feed(12)
    last=r.manager.last_read_diagnostics['stationary_pair_history']
    assert last['last_complete_raw_frames']==first['last_complete_raw_frames']
    value=last['last_complete_raw_frames']['color']['metadata_raw']['backend_timestamp']
    assert type(value) is int and value==1791284753318
    status=r.manager.runtime_profile['stationary_temporal']
    xmlrpc.client.dumps((status,))
    assert status['pair_history']['last_complete_raw_frames']['color']['metadata_raw']['backend_timestamp']==str(value)
