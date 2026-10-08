from pathlib import Path
import importlib.util
import json
import struct
import sys

import numpy as np
import pytest

path=Path(__file__).with_name('analyze_current_support_rgbd_20261007.py')
spec=importlib.util.spec_from_file_location('current_support_analysis',path)
a=importlib.util.module_from_spec(spec); spec.loader.exec_module(a)


def image_bytes(array,encoding,endian=0,padding=0):
    h,w=array.shape[:2]; step=array.strides[0]+padding
    def string(value):
        raw=value.encode();return struct.pack('<I',len(raw))+raw
    header=struct.pack('<III',7,10,123)+string('camera_link')+struct.pack('<II',h,w)+string(encoding)
    data=b''.join(row.tobytes()+bytes(padding) for row in array)
    return header+struct.pack('<BII',endian,step,len(data))+data


def test_original_depth_serialization_little_endian_with_padding():
    original=np.arange(12,dtype='<u2').reshape(3,4)
    header,pixels=a.decode_image(image_bytes(original,'16UC1',padding=2))
    assert header['source_stamp_ns']==10_000_000_123
    assert header['frame_id']=='camera_link'
    assert np.array_equal(pixels,original)


def test_original_depth_serialization_big_endian():
    original=np.arange(12,dtype='>u2').reshape(3,4)
    _,pixels=a.decode_image(image_bytes(original,'16UC1',endian=1))
    assert np.array_equal(pixels,original)


def test_original_bgr_bytes_exact():
    original=np.arange(36,dtype='u1').reshape(3,4,3)
    _,pixels=a.decode_image(image_bytes(original,'bgr8'))
    assert np.array_equal(pixels,original)


def test_extra_serialization_bytes_rejected():
    original=np.arange(12,dtype='<u2').reshape(3,4)
    with pytest.raises(ValueError): a.decode_image(image_bytes(original,'16UC1')+b'extra')


def test_plane_uses_original_quality_and_same_segment_path():
    cfg=a.table.Config(); depth=np.full((240,320),.4,np.float32)
    row=a.diagnose_depth(depth,[300,300,160,120],np.eye(4),cfg)
    assert row['support_available'] is True
    assert row['selected_outer_base_px']==170
    assert row['current_frame_segment']['passed'] is True
    assert row['current_frame_segment']['selection_performed'] is False
    assert [r['consulted_by_original_sequence'] for r in row['rings']]==[True,False,False]
    assert row['target_identity'] is None and row['motion_authorized'] is False
    assert np.allclose(row['plane_camera'],[-1,0,0,.4],atol=1e-6)


def test_missing_depth_is_not_replaced_by_reference_or_smaller_ring():
    row=a.diagnose_depth(np.zeros((240,320),np.float32),[300,300,160,120],None,a.table.Config())
    assert row['support_available'] is False
    assert row['support_reason']=='insufficient_support_depth'
    assert row['current_frame_segment']['unavailable']=='exact same-stamp TF unavailable'
    assert [r['consulted_by_original_sequence'] for r in row['rings']]==[True,False,False]


def test_missing_exact_tf_allows_only_camera_plane_diagnostic():
    row=a.diagnose_depth(np.full((240,320),.4,np.float32),[300,300,160,120],None,a.table.Config())
    assert row['support_available']
    assert 'plane_base' not in row
    assert row['current_frame_segment']['unavailable']=='exact same-stamp TF unavailable'


def test_unavailable_regional_evidence_not_invented():
    owner,regions,audit=a.diagnostic_owner({}, {}, (240,320))
    assert owner is None and regions is None and not audit['available']


def test_camera_invariants_ignore_counters_but_preserve_actual_geometry():
    camera=dict(fx=100,runtime_profile=dict(rectification={'map':'same'},geometry_activation_stamp_ns=1,
        stationary_temporal={'counter':1}))
    before=a.config_invariant('/camera',camera)
    camera['runtime_profile']['stationary_temporal']['counter']=2
    assert a.config_invariant('/camera',camera)==before
    camera['runtime_profile']['geometry_activation_stamp_ns']=2
    assert a.config_invariant('/camera',camera)!=before


def test_decompression_respects_original_size_and_trailing_data():
    import zlib
    raw=b'test original'
    compressed=zlib.compress(raw)
    assert a.bounded_original_message(compressed,len(raw))==raw
    with pytest.raises(ValueError): a.bounded_original_message(compressed,len(raw)-1)
    with pytest.raises(ValueError): a.bounded_original_message(compressed+b'tail',len(raw))
    with pytest.raises(ValueError): a.bounded_original_message(compressed,9*1024*1024)


def test_short_time_planes_do_not_create_target_reference_or_compare_cross_epoch():
    def row(ns,offset,epoch):
        return dict(source_stamp_ns=ns,analysis=dict(support_available=True,
            plane_camera=[-1,0,0,offset],plane_base=[-1,0,0,offset]),
            original_source_pose_evidence=dict(feedback=dict(epoch_stamp_ns=epoch)))
    result=a.short_time_changes([row(100,.4,1),row(200,.401,2)])
    assert abs(result[0]['camera']['plane_equation_offset_delta_mm']-1)<1e-9
    assert result[0]['target_reference_created'] is False
    assert 'base' not in result[0]
    assert result[0]['base_comparison_unavailable']=='different_or_missing_control_reference_epoch'


def test_uniform_selection_uses_original_ns_time_not_quality_or_float_rounding():
    start=1791426945272122621
    rows=[dict(source_stamp_ns=start+n,quality=100-n) for n in (1,2,3,4,5,90,91,92,100)]
    selected=a.uniform_source_pairs(rows,3)
    assert [row['source_stamp_ns']-start for row in selected]==[1,90,100]
    assert all(any(row is original for original in rows) for row in selected)
    assert a.uniform_source_pairs(rows,6)[0] is rows[0]
    assert len(a.uniform_source_pairs(rows,6))<=6


@pytest.mark.parametrize('rows,maximum',[
    ([dict(source_stamp_ns=1),dict(source_stamp_ns=1)],6),
    ([dict(source_stamp_ns=True)],6),([dict(source_stamp_ns=0)],6),
    ([dict(source_stamp_ns=1)],7),([dict(source_stamp_ns=1)],True)])
def test_invalid_or_duplicate_source_sampling_rejected(rows,maximum):
    with pytest.raises(ValueError): a.uniform_source_pairs(rows,maximum)


def evidence_fixture():
    ns=1791426945272122621
    entry=dict(source_stamp_ns=ns,frame_id='camera_link',geometry_generation=2,
        geometry_activation_stamp_ns='123',evidence=dict(result='complete',raw_frames={}))
    payload=dict(entries=[entry],geometry_generation=2,copy_errors=0)
    record=dict(parameter='/camera_read_diagnostics',original_rpc_code=1,
        original_parameter_value=dict(encoding='json',payload_json=json.dumps(payload)))
    saved=dict(source_stamp_ns=ns,frame_id='camera_link',exact_read_history_unique=True,
        exact_read_history_references=[dict(entry='metadata/0.json',source_sha256=a.sha(
            (json.dumps(entry,ensure_ascii=False,sort_keys=True,indent=2)+'\n').encode()))])
    camera=dict(runtime_profile=dict(geometry_activation_stamp_ns=123))
    return saved,record,payload,camera


def test_exact_read_preserves_raw_values_and_does_not_claim_whole_history():
    saved,record,payload,camera=evidence_fixture()
    payload['copy_errors']=2
    record['original_parameter_value']['payload_json']=json.dumps(payload)
    result,audit=a.exact_read_evidence(saved,lambda _:json.dumps(record),camera)
    assert result==payload['entries'][0]
    assert audit['copy_errors']==2 and audit['no_unsampled_history_completeness_claim']


def test_missing_exact_read_does_not_use_available_neighbour():
    saved,record,payload,camera=evidence_fixture()
    saved['exact_read_history_unique']=False
    def forbidden(_): raise AssertionError('must not inspect a neighbour')
    result,audit=a.exact_read_evidence(saved,forbidden,camera)
    assert result is None and not audit['available']


@pytest.mark.parametrize('change',['stamp','sha','activation','frame_id','generation'])
def test_conflicting_exact_source_or_owner_rejected(change):
    saved,record,payload,camera=evidence_fixture()
    entry=payload['entries'][0]
    if change=='stamp': entry['source_stamp_ns']+=1
    elif change=='sha': saved['exact_read_history_references'][0]['source_sha256']='0'*64
    elif change=='activation': camera['runtime_profile']['geometry_activation_stamp_ns']=124
    elif change=='frame_id': saved['frame_id']='other'
    elif change=='generation': payload['geometry_generation']=3
    record['original_parameter_value']['payload_json']=json.dumps(payload)
    with pytest.raises(ValueError): a.exact_read_evidence(saved,lambda _:json.dumps(record),camera)


def test_camera_plane_only_cli_does_not_import_sdk_or_consult_target(tmp_path,monkeypatch):
    """A sealed recorder-shaped archive without TF/target may diagnose its plane."""
    import zipfile
    import zlib
    capture=tmp_path/'capture';capture.mkdir()
    output=tmp_path/'analysis'
    raw_depth=image_bytes(np.full((240,320),4000,dtype='<u2'),'16UC1')
    raw_color=image_bytes(np.zeros((240,320,3),dtype='u1'),'bgr8')
    ns=10_000_000_123
    camera=dict(fx=300.,fy=300.,cx=160.,cy=120.,width=320,height=240,
        depth_scale=.0001,frame_id='camera_link',fps=15,runtime_profile=dict(geometry_activation_stamp_ns='123'))
    records=[dict(parameter='/camera',original_rpc_code=1,original_parameter_value=camera,started={},finished={}),
        dict(parameter='/unknown_perception',original_rpc_code=-1,
            original_rpc_message='Parameter [/unknown_perception] is not set',started={},finished={})]
    data={f'metadata/{i}.json':a.encoded(record) for i,record in enumerate(records)}
    images=[]
    for stream,raw in [('color',raw_color),('depth',raw_depth)]:
        name=f'pairs/00_{stream}.rosmsg.zlib';data[name]=zlib.compress(raw)
        images.append(dict(stream=stream,entry=name,serialized_bytes=len(raw),serialized_sha256=a.sha(raw),
            source_stamp_ns=ns,source_seq=7))
    data['pairs/00.json']=a.encoded(dict(source_stamp_ns=ns,images=images,at_source={}))
    archive=capture/'diagnostic.zip'
    with zipfile.ZipFile(archive,'w') as z:
        for name,raw in data.items(): z.writestr(name,raw)
    summary=dict(sealed=True,ready=True,error=None,rpc_thread_alive=False,archive_sha256=a.sha(archive.read_bytes()),
        archive_entries=[dict(name=name,bytes=len(raw),sha256=a.sha(raw)) for name,raw in data.items()],
        parameter_records=[dict(entry=f'metadata/{i}.json') for i in range(2)],
        pairs=[dict(entry='pairs/00.json',source_stamp_ns=ns,frame_id='camera_link',
            exact_read_history_unique=False,exact_read_history_references=[])])
    summary_raw=a.encoded(summary)
    (capture/'summary.json').write_bytes(summary_raw)
    (capture/'SHA256SUMS').write_text(a.sha(summary_raw)+'  summary.json\n'+summary['archive_sha256']+'  diagnostic.zip\n')
    monkeypatch.setattr(a,'EXTERNAL',tmp_path)
    monkeypatch.setattr(Path,'is_mount',lambda _:True)
    monkeypatch.setattr(a,'diagnostic_owner',lambda *a,**k:pytest.fail('SDK mapping forbidden'))
    monkeypatch.setattr(sys,'argv',['analysis','--input',str(capture),'--output',str(output),
        '--max-pairs','6','--camera-plane-only'])
    sdk_before='pyrealsense2' in sys.modules
    a.main()
    report=json.loads((output/'report.json').read_text())
    assert report['quality_pass_count']==1
    assert report['camera_parameters']['fps']==15
    assert report['exact_read_history_count']==0 and not report['sdk_quality_association_complete']
    assert report['target_identity'] is None and not report['motion_authorized']
    assert 'plane_base' not in report['rows'][0]['analysis']
    assert ('pyrealsense2' in sys.modules)==sdk_before
