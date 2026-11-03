"""Offline same-frame analysis contracts; no ROS, physical SDK or motion."""
import copy
import importlib.util
import json
from pathlib import Path
import sys
from types import SimpleNamespace

import numpy as np
import pytest

HERE = Path(__file__).resolve().parent
spec = importlib.util.spec_from_file_location('new_shadow_analysis', HERE/'analyze_matched_shadow_capture_20261005.py')
m = importlib.util.module_from_spec(spec)
spec.loader.exec_module(m)


def source_fixture():
    anchor=2000000000;q=[0.]*7
    feedback=dict(anchor_ros_stamp_ns=anchor,qualified=True,faults=[],
        accepted_before=dict(source_stamp_ns=anchor-10000000,q=q),
        accepted_after=dict(source_stamp_ns=anchor+10000000,q=q),span_ns=20000000,
        maximum_span_ns=200000000,q_at_anchor=q)
    tf=dict(parent='base_link',child='camera_link',source_stamp_ns=anchor,
            translation=[0.,0.,0.],quaternion_xyzw=[0.,0.,0.,1.])
    info=dict(complete=True,anchor_ros_stamp_ns=anchor,production_read_diagnostics=dict(result='complete',pair_timing=dict(accepted=True)),
        source_evidence=dict(evidence_complete=True,feedback=feedback,exact_tf=tf,
            tf_stamp_audit=dict(requested_stamp_ns=anchor,returned_stamp_ns=anchor,delta_ns=0,exact_match=True)))
    summary=dict(frozen_guard=dict(epoch=anchor-1000000000,accepted=dict(q=q)),guard_faults=[])
    return info,summary


def test_source_qualified_with_actual_strict_bracket():
    info,summary=source_fixture()
    assert m.source_classification(info,summary)['source_qualified']


@pytest.mark.parametrize('delta',[-1,1,72])
def test_false_evidence_claim_never_promotes_tf_quantization(delta):
    info,summary=source_fixture()
    audit=info['source_evidence']['tf_stamp_audit']
    audit['returned_stamp_ns']+=delta;audit['delta_ns']=delta;audit['exact_match']=False
    result=m.source_classification(info,summary)
    assert result['original_pair_complete'] and not result['source_qualified']
    assert 'strict_tf_not_qualified' in result['rejection_reasons']
    assert result['tf_stamp_audit']['delta_ns']==delta


@pytest.mark.parametrize('fault',['span','unbracketed','joint_change','epoch','nonfinite','pair','guard'])
def test_source_contract_rejects_actual_feedback_or_pair_fault(fault):
    info,summary=source_fixture();feedback=info['source_evidence']['feedback']
    if fault=='span':feedback['accepted_after']['source_stamp_ns']+=200000001;feedback['span_ns']+=200000001
    elif fault=='unbracketed':feedback['accepted_before']['source_stamp_ns']=2100000000
    elif fault=='joint_change':feedback['accepted_after']['q']=[1.]*7
    elif fault=='epoch':summary['frozen_guard']['epoch']=2000000000
    elif fault=='nonfinite':feedback['q_at_anchor']=[float('nan')]*7
    elif fault=='pair':info['production_read_diagnostics']['pair_timing']['accepted']=False
    else:summary['guard_faults']=['moved']
    assert not m.source_classification(info,summary)['source_qualified']


def shadow_fixture(*,active=True,first=False):
    now=2000000000
    window={kind:[[stamp,[0.]*size] for stamp in range(1500000000,2000000001,100000000)]
            for kind,size in [('sdk',6),('accepted',7)]}
    raw={kind:dict(present=True,frame_number=60,device_timestamp_ms=2000.,stream_fps=30,timestamp_domain='hardware_clock') for kind in ('color','depth')}
    record=dict(active=active,first_history_frame=first,history_id=1,original_output_replaced=False,geometry_authority=False,
        alpha=.4,delta_depth_units=100,persistence=0,color_source=raw['color'],depth_source=raw['depth'],
        output_kind='filtered' if active else 'current_frame_passthrough',
        visual=dict(qualified=active,reason='stationary' if active else 'warming_up'),
        encoder=dict(qualified=True,evaluation_ros_stamp_ns=now,source_window=window,source_window_sha256=m.digest(window)))
    info=dict(matched_stationary_shadow=record,production_read_diagnostics=dict(raw_frames=raw))
    pre=np.full((8,8),2000,np.uint16);pre[0,0]=0
    shadow=pre.copy()
    if active and not first:shadow[1,1]=1990
    return info,dict(pre_stationary_depth=pre,matched_shadow_depth=shadow)


@pytest.mark.parametrize('active,first,kind',[(False,False,'inactive_passthrough'),(True,True,'first_history_frame'),(True,False,'history_filtered')])
def test_shadow_passthrough_first_and_history_are_distinct(active,first,kind):
    info,arrays=shadow_fixture(active=active,first=first)
    result=m.shadow_classification(info,arrays)
    assert result['kind']==kind and not result['contradictions']
    assert result['history_filtered']==(active and not first)
    assert result['full_hash_chain_replay_possible'] is False


@pytest.mark.parametrize('fault',['hole','first_changed','source','encoder_hash','encoder_claim','parameters','authority','pair'])
def test_shadow_value_or_provenance_fault_never_qualifies(fault):
    info,arrays=shadow_fixture();record=info['matched_stationary_shadow']
    if fault=='hole':arrays['matched_shadow_depth'][0,0]=2000
    elif fault=='first_changed':record['first_history_frame']=True
    elif fault=='source':record['depth_source']=dict(record['depth_source'],frame_number=59)
    elif fault=='encoder_hash':record['encoder']['source_window_sha256']='0'*64
    elif fault=='encoder_claim':record['encoder']['qualified']=False
    elif fault=='parameters':record['persistence']=1
    elif fault=='authority':record['geometry_authority']=True
    else:record['pair_complete']=False
    result=m.shadow_classification(info,arrays)
    assert not result['eligible'] and not result['history_filtered'] and result['contradictions']


def saved_sample(tmp_path):
    arrays=dict(raw_rgb=np.full((4,5,3),10,np.uint8),pre_stationary_depth=np.full((4,5),2000,np.uint16))
    np.savez(tmp_path/'sample_01.npz',**arrays)
    info=dict(sample_index=1,arrays={k:dict(dtype=str(a.dtype),shape=list(a.shape),bytes=a.nbytes,sha256=m.array_sha(a)) for k,a in arrays.items()})
    (tmp_path/'sample_01.json').write_text(json.dumps(info))
    summary=dict(maximum_artifact_bytes=159000000,files=[dict(name=p.name,bytes=p.stat().st_size,sha256=m.sha(p)) for p in sorted(tmp_path.glob('sample_*'))])
    (tmp_path/'summary.json').write_text(json.dumps(summary))
    return summary,info,arrays


def test_file_and_array_manifest_success(tmp_path):
    summary,info,arrays=saved_sample(tmp_path)
    checked,names,audit=m.verify_artifacts(tmp_path)
    assert names==['sample_01'] and audit['manifest_sample_files_verified']
    read,actual=m.read_sample(tmp_path,'sample_01')
    assert read==info and all(np.array_equal(arrays[k],actual[k]) for k in arrays)


@pytest.mark.parametrize('fault',['json_byte','npz_byte','duplicate','missing','escape','orphan'])
def test_file_manifest_rejects_corruption_and_orphan(tmp_path,fault):
    summary,info,arrays=saved_sample(tmp_path)
    if fault.endswith('_byte'):
        suffix='json' if fault=='json_byte' else 'npz';p=tmp_path/('sample_01.'+suffix);p.write_bytes(p.read_bytes()+b' ')
    elif fault=='duplicate':summary['files'].append(summary['files'][0])
    elif fault=='missing':summary['files']=summary['files'][:1]
    elif fault=='escape':summary['files'][0]['name']='../sample_01.json'
    else:(tmp_path/'sample_02.json').write_text('{}')
    (tmp_path/'summary.json').write_text(json.dumps(summary))
    with pytest.raises(ValueError):m.verify_artifacts(tmp_path)


@pytest.mark.parametrize('field,value',[('dtype','float32'),('shape',[5,4]),('bytes',41),('sha256','0'*64)])
def test_array_manifest_rejects_altered_metadata(tmp_path,field,value):
    summary,info,arrays=saved_sample(tmp_path)
    info['arrays']['pre_stationary_depth'][field]=value
    (tmp_path/'sample_01.json').write_text(json.dumps(info))
    with pytest.raises(ValueError):m.read_sample(tmp_path,'sample_01')


def test_npz_unrecorded_array_rejected(tmp_path):
    summary,info,arrays=saved_sample(tmp_path)
    np.savez(tmp_path/'sample_01.npz',**dict(arrays,extra=np.ones((4,5),np.uint16)))
    with pytest.raises(ValueError,match='keyset'):m.read_sample(tmp_path,'sample_01')


def test_fit_original_rejection_preserved_and_previous_trace_restored():
    rng=np.random.RandomState(44)
    points=rng.normal(size=(500,3))*.02
    tracer=lambda *args:None
    old=sys.gettrace()
    try:
        sys.settrace(tracer)
        plane,audit=m.fit_audit(points,m.tabletop.Config())
        assert sys.gettrace() is tracer
    finally:sys.settrace(old)
    assert not audit['fit_passed'] and audit['geometry_authority'] is False


def test_fixed_ring_common_empty_has_no_residual_and_no_synthetic_values():
    depth=np.zeros((96,128),np.uint16)
    arrays={name:depth.copy() for name in m.STAGES}
    arrays['pre_stationary_depth'][:]=2000
    result=m.compare_stages(arrays,.0001,[100.,100.,64.,48.],m.tabletop.Config())
    assert result['common_valid_pixels']==0
    for item in result['stages'].values():
        assert not item['fixed_ring_common_coverage_fit']['fit_passed']
    assert result['deltas']['pre__matched_shadow_depth']['common_signed_z_delta_mm'] is None


def test_plane_and_holes_use_same_ring_without_relaxing_2mm_gate():
    depth=np.full((96,128),2000,np.uint16)
    arrays={name:depth.copy() for name in m.STAGES}
    arrays['matched_shadow_depth'][20:35,50:65]=0
    originals={k:v.copy() for k,v in arrays.items()}
    result=m.compare_stages(arrays,.0001,[100.,100.,64.,48.],m.tabletop.Config())
    assert result['common_valid_pixels']<result['fixed_ring']['pixels']
    assert all(item['original_initial_support']['passed'] for item in result['stages'].values())
    assert result['deltas']['pre__matched_shadow_depth']['ring_created_nonzero']==0
    assert all(np.array_equal(arrays[k],originals[k]) for k in arrays)


def test_rectification_mapping_uses_expanded_raster_not_aligned_indices():
    class Mapper:
        source_height=2;source_width=3;height=3;width=5
        source_intrinsics={'width':3,'height':2};output_intrinsics={'width':5,'height':3}
        contract={'test':'injective_translation'}
        target_indices=np.array([6,7,8,11,12,13])
        def apply(self,rgb,depth):
            out=np.zeros((3,5),np.uint16);out.ravel()[self.target_indices]=depth.ravel()
            color=np.zeros((3,5,3),np.uint8);color.reshape(-1,3)[self.target_indices]=rgb.reshape(-1,3)
            return color,out
    mapper=Mapper();pre=np.arange(2000,2006,dtype=np.uint16).reshape(2,3)
    arrays={name:pre.copy() for name in m.STAGES[:-1]};arrays['raw_rgb']=np.ones((2,3,3),np.uint8)
    arrays['final_rgb'],arrays['final_depth']=mapper.apply(arrays['raw_rgb'],pre)
    profile=dict(rectification=mapper.contract,geometry_intrinsics=mapper.output_intrinsics,color_intrinsics=mapper.source_intrinsics,
                 depth_scale_m_per_unit=.0001,depth_filters=dict(range_m=[.03,2.]))
    info=dict(runtime_profile=profile,published_camera_parameters=dict(runtime_profile=profile),depth_scale_m=.0001)
    mapped=m.mapped_stages(info,arrays,mapper)
    assert np.array_equal(mapped['pre_stationary_depth'],arrays['final_depth'])
    assert mapped['pre_stationary_depth'][1,1]==pre[0,0] and mapped['pre_stationary_depth'][0,0]==0
    arrays['final_depth'][0,0]=2000
    with pytest.raises(ValueError,match='not reproducible'):m.mapped_stages(info,arrays,mapper)


def test_clip_uses_recorded_integer_unit_bounds():
    z=np.array([[299,300,20000,20001]],np.uint16)
    assert m.clip_raw(z,.0001,[.03,2.]).tolist()==[[0,300,20000,0]]
    assert z.tolist()==[[299,300,20000,20001]]


def continuity_fixture():
    final=dict(label='B',reads=100,eligible_reads=60,reasons=dict(stationary=60,warming_up=40),resets={},chain_sha256='a'*64)
    summary=dict(matched_stationary_shadow=dict(phases=[final]))
    rows=[]
    for index,read in enumerate([40,70],1):
        snapshot=dict(label='B',reads=read,eligible_reads=read-20,reasons=dict(stationary=read-20,warming_up=20),resets={},chain_sha256='b'*64)
        rows.append(dict(index=index,phase='B',shadow=dict(kind='history_filtered',recorded=dict(
            phase=snapshot,history_id=2,depth_source=dict(frame_number=read+1000)))))
    return summary,rows


def test_saved_frame_gaps_are_not_invented_drop_events():
    summary,rows=continuity_fixture()
    audit=m.continuity_audit(summary,rows)
    delta=audit['selected_sample_changes'][1]['since_previous_saved_sample']
    assert delta['sampled_depth_frame_number_difference']==30
    assert delta['reset_increments']=={} and not audit['full_chain_replayed']
    assert 'not acquisition drop counts' in audit['limitation']


@pytest.mark.parametrize('fault',['sum','final_exceeded','backwards','history','hash'])
def test_continuity_counter_contradictions_rejected(fault):
    summary,rows=continuity_fixture();snapshot=rows[1]['shadow']['recorded']['phase']
    if fault=='sum':snapshot['reasons']['stationary']+=1
    elif fault=='final_exceeded':snapshot['reads']=200;snapshot['reasons']['stationary']=180
    elif fault=='backwards':snapshot.update(reads=30,eligible_reads=10,reasons=dict(stationary=10,warming_up=20))
    elif fault=='history':rows[1]['shadow']['recorded']['history_id']=1
    else:snapshot['chain_sha256']='invalid'
    with pytest.raises(ValueError):m.continuity_audit(summary,rows)


def test_no_previous_target_reference_or_control_imports():
    source=(HERE/'analyze_matched_shadow_capture_20261005.py').read_text()
    assert 'analyze_camera_preset_ab_20261004' not in source
    assert 'import rospy' not in source and 'rs.context(' not in source and 'REFERENCE' not in source


@pytest.mark.parametrize('runner_failed',[False,True])
def test_full_new_capture_keeps_tf_rejection_but_compares_camera_local_stages(tmp_path,monkeypatch,runner_failed):
    # Identity distortion requires no SDK functions, context or physical device.
    sdk=SimpleNamespace(__version__='2.56.5')
    monkeypatch.setitem(sys.modules,'pyrealsense2',sdk)
    mapper=m.PairedPinholeRectification(dict(width=128,height=96,fx=100.,fy=100.,cx=64.,cy=48.,model='none',coeffs=[0.]*5),sdk=sdk)
    folder=tmp_path/'stages';folder.mkdir()
    info,summary=source_fixture();shadow,unused=shadow_fixture()
    info.update(shadow)
    info['production_read_diagnostics'].update(result='complete',pair_timing=dict(accepted=True))
    info['source_evidence']['evidence_complete']=False
    info['source_evidence']['exact_tf']=None
    info['source_evidence']['tf_error']='TF timestamp differs from exact requested anchor'
    info['source_evidence']['tf_stamp_audit'].update(returned_stamp_ns=2000000001,delta_ns=1,exact_match=False)
    phase=dict(label='A1',reads=50,eligible_reads=20,reasons=dict(stationary=20,warming_up=30),resets={},chain_sha256='a'*64)
    info['matched_stationary_shadow']['phase']=copy.deepcopy(phase)
    pre=np.full((96,128),2000,np.uint16)
    arrays={name:pre.copy() for name in m.STAGES}
    arrays.update(raw_depth=pre.copy(),aligned_depth=pre.copy(),post_spatial_depth=pre.copy(),raw_rgb=np.zeros((96,128,3),np.uint8),final_rgb=np.zeros((96,128,3),np.uint8))
    arrays['matched_shadow_depth'][:]=2001
    profile=dict(rectification=mapper.contract,geometry_intrinsics=mapper.output_intrinsics,color_intrinsics=mapper.source_intrinsics,
        depth_scale_m_per_unit=.0001,depth_filters=dict(range_m=[.03,2.]))
    info.update(sample_index=1,phase='A1',runtime_profile=profile,published_camera_parameters=dict(runtime_profile=profile),depth_scale_m=.0001,
        arrays={k:dict(dtype=str(a.dtype),shape=list(a.shape),bytes=a.nbytes,sha256=m.array_sha(a)) for k,a in arrays.items()})
    np.savez(folder/'sample_01.npz',**arrays)
    (folder/'sample_01.json').write_text(json.dumps(info))
    summary.update(maximum_artifact_bytes=159000000,files=[dict(name=p.name,bytes=p.stat().st_size,sha256=m.sha(p)) for p in sorted(folder.glob('sample_*'))],
        experiment_valid=False,capture_complete=True,counts=dict(written=1),errors=['strict TF rejection'],restoration=dict(succeeded=True),
        matched_stationary_shadow=dict(phases=[phase]))
    (folder/'summary.json').write_text(json.dumps(summary))
    outer=(dict(experiment_complete=False,visual_nodes_restored=False,error='ConnectionRefusedError')
           if runner_failed else dict(sensor_summary=summary))
    (tmp_path/'result.json').write_text(json.dumps(outer))
    (tmp_path/'camera_preset_ab_observer.py').write_text('# archived fixture, never executed\n')
    (folder/'loaded_modules.json').write_text(json.dumps(dict(wrapper_sha256=m.sha(tmp_path/'camera_preset_ab_observer.py'))))
    report=m.analyze(tmp_path)
    row=report['rows'][0]
    assert report['target_identity_or_grasp_geometry_evaluated'] is False
    assert report['runner_receipt']['captured_summary_crosschecked'] is not runner_failed
    if runner_failed:assert report['runner_receipt']['visual_nodes_restored'] is False
    assert row['source']['original_pair_complete'] and not row['source']['source_qualified']
    assert row['shadow']['history_filtered'] and row['original_final_rgb_depth_reconstructed_exactly']
    assert report['all_compared_samples_common_ring']['pixels']>120
    scopes=report['phase_summary']['A1']['comparisons']
    assert scopes['all_complete_camera_local_diagnostics']['rows']==1
    assert scopes['strict_source_qualified_diagnostics']['rows']==0
    assert scopes['shadow_history_eligible_diagnostics']['rows']==1
    json.dumps(report,allow_nan=False)
