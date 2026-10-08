"""Pure exact-state visual replay; no ROS, camera or recorded-image writes."""
import copy,importlib.util,json
from pathlib import Path
import cv2,numpy as np,pytest
P=Path(__file__).with_name('replay_raw_visual_snapshots_20261006.py')
s=importlib.util.spec_from_file_location('raw_visual_replay_test',P);m=importlib.util.module_from_spec(s);s.loader.exec_module(m)
@pytest.fixture(autouse=True)
def threads():
 old=cv2.getNumThreads();cv2.setNumThreads(1);yield;cv2.setNumThreads(old)
def capture(region,kind):
 rgb=np.random.RandomState(141).randint(0,256,(480,640,3),np.uint8);rgb=cv2.GaussianBlur(rgb,(5,5),1.)
 bounds={'center':[128,48,512,432],'nested_center':[224,144,416,336]}[region]
 color,_,_=m.gray_for(rgb,bounds);window=m.V()
 source=lambda i:dict(frame_number=i+1,device_timestamp_ms=1000.+i*1000./30.,timestamp_domain='hardware_clock',stream_fps=30.)
 for i in range(11 if kind=='stationary' else 1):window.observe(color,source(i),10.+i/30.)
 i=11 if kind=='stationary' else 1
 current=rgb.copy() if kind=='stationary' else np.roll(rgb,1,axis=1)
 pre=dict(previous=window._previous,anchor_device_timestamp_ms=window._anchor_stamp,
   anchor_host_monotonic=window._anchor_host,continuous_frames=window._count,
   anchor_bound_verified=True,anchor_source=source(0),anchor_filter_host_monotonic=10.,
   previous_source_tuple=window._previous,continuous_frame_count=window._count,
   anchor_raw_color_shape=list(rgb.shape),anchor_gray_shape=list(window._anchor.shape),
   seed_points_count=len(window._points))
 arrays=dict(raw_color=current,anchor_raw_color=rgb,anchor_gray=window._anchor.copy(),seed_points=window._points.copy())
 crop,_,_=m.gray_for(current,bounds)
 result=window.observe(crop,source(i),10.+i/30.)
 meta=dict(schema=1,diagnostic_only=True,motion_authorized=False,pixels_changed=False,
   coordinate_domain=m.DOMAIN,publication_outcome_unknown=True,enclosing_read_outcome_unknown=True,
   pre_observe=pre,bounds_xyxy=bounds,source=source(i),filter_host_monotonic=10.+i/30.,
   raw_color_shape=list(current.shape),observed_crop_shape=list(crop.shape),
   scale_full_resolution_xy=m.gray_for(current,bounds)[2].tolist(),
   source_implementation={m.VISUAL_MODULE:dict(path=str(m.VISUAL_PATH.resolve()),sha256=m.sha(m.VISUAL_PATH.read_bytes()))},
   original_return=result,region=region,category=result['reason'])
 return arrays,json.loads(json.dumps(meta))
@pytest.mark.parametrize('region',['center','nested_center'])
@pytest.mark.parametrize('kind',['stationary','visual_motion'])
def test_actual_original_prestate_round_trip_exact_report_and_gray(region,kind):
 arrays,meta=capture(region,kind);saved=copy.deepcopy(arrays)
 result=m.exact_replay(arrays,meta)
 assert result['exact_return_equal'] and result['anchor_raw_reproduces_gray']
 assert result['original_return']['reason']==kind
 assert all(np.array_equal(arrays[k],saved[k]) for k in arrays)
 assert len(result['points'])==len(arrays['seed_points'])
@pytest.mark.parametrize('fault',['wrong_anchor_raw','wrong_anchor_source','wrong_anchor_host'])
def test_derived_anchor_or_current_source_cannot_replace_real_preanchor(fault):
 arrays,meta=capture('center','visual_motion')
 if fault=='wrong_anchor_raw':arrays['anchor_raw_color']=arrays['raw_color'].copy()
 elif fault=='wrong_anchor_source':meta['pre_observe']['anchor_source']['device_timestamp_ms']+=1
 else:meta['pre_observe']['anchor_filter_host_monotonic']+=1
 with pytest.raises(ValueError):m.exact_replay(arrays,meta)


@pytest.mark.parametrize('fault',['scope','wrong_implementation','bounds','current_source','current_host',
    'previous_tuple','source_domain','scale','point_dtype','count','anchor_bound','category'])
def test_pseudo_source_or_different_geometry_cannot_claim_exact_replay(fault):
 arrays,meta=capture('center','visual_motion')
 if fault=='scope':meta['coordinate_domain']='published_rectified_rgb'
 elif fault=='wrong_implementation':meta['source_implementation'][m.VISUAL_MODULE]['sha256']='wrong'
 elif fault=='bounds':meta['bounds_xyxy'][0]+=1
 elif fault=='current_source':meta['source']['frame_number']+=4
 elif fault=='current_host':meta['filter_host_monotonic']+=1
 elif fault=='previous_tuple':meta['pre_observe']['previous_source_tuple'][0]+=1
 elif fault=='source_domain':meta['source']['timestamp_domain']='system_time'
 elif fault=='scale':meta['scale_full_resolution_xy'][0]=2
 elif fault=='point_dtype':arrays['seed_points']=arrays['seed_points'].astype(np.float64)
 elif fault=='count':meta['pre_observe']['continuous_frame_count']=999
 elif fault=='anchor_bound':meta['pre_observe']['anchor_bound_verified']=False
 elif fault=='category':meta['category']='warming_up'
 with pytest.raises(ValueError):m.exact_replay(arrays,meta)


def test_report_mismatch_is_explicit_and_not_replaced_by_replayed_value():
 arrays,meta=capture('center','visual_motion')
 meta['original_return']['motion_p90_px']+=.1
 row=m.exact_replay(arrays,meta,include_gaussian=False)
 assert row['exact_return_equal'] is False
 assert row['original_return']==meta['original_return']
 assert row['replayed_return']['motion_p90_px']!=row['original_return']['motion_p90_px']


def test_an_older_anchor_frame_cannot_keep_same_count_and_claim_exact_report():
 arrays,meta=capture('center','stationary')
 meta['pre_observe']['anchor_source']['frame_number']=0
 with pytest.raises(ValueError,match='anchor_frame_count_lineage_invalid'):
  m.exact_replay(arrays,meta,include_gaussian=False)


@pytest.mark.parametrize('region',['center','nested_center'])
def test_known_translation_has_full_pixel_positions_consistent_direction_and_original_gate(region):
 arrays,meta=capture(region,'visual_motion')
 row=m.exact_replay(arrays,meta,include_gaussian=False)
 metrics=row['recomputed_lk_metrics']
 assert metrics['motion_p90_px']==row['original_return']['motion_p90_px']
 assert metrics['motion_median_px']==row['original_return']['motion_median_px']
 assert metrics['tracked_points']==row['original_return']['tracked_points']
 directions=row['above_threshold_motion_direction']
 assert directions['count']>=24
 assert directions['unit_vector_resultant']>.99
 assert directions['mean_direction_xy'][0]>.99
 assert directions['fraction_within_30deg_of_mean']>.99
 valid=[p for p in row['points'] if p['valid']]
 assert np.median([p['motion_full_input_xy'][0] for p in valid])==pytest.approx(1.,abs=.06)
 for point in valid:
  assert point['seed_raw_uv'][0]>=meta['bounds_xyxy'][0]
  assert point['seed_raw_uv'][1]>=meta['bounds_xyxy'][1]
  assert point['seed_quadrant'] in range(4)
  assert point['fb_full_input_px']<=m.V.MAX_FORWARD_BACKWARD_PX
  np.testing.assert_allclose(np.array(point['current_raw_uv'])-point['seed_raw_uv'],point['motion_full_input_xy'],atol=.00005)


@pytest.mark.parametrize('region',['center','nested_center'])
def test_gaussian_diagnostic_retains_fixed_anchor_seeds_and_reports_known_subpixel_controls(region):
 arrays,meta=capture(region,'stationary');saved={k:v.copy() for k,v in arrays.items()}
 row=m.exact_replay(arrays,meta)
 diag=row['gaussian_diagnostic']
 assert diag['original_seed_points_preserved'] and diag['anchor_reseeded'] is False
 assert diag['original_motion_p90_threshold_px']==.25
 assert [x['sigma'] for x in diag['actual_pair']]==[None,.6,.8,1.]
 assert all(x['p90_motion_gate_passed'] for x in diag['actual_pair'])
 controls=diag['synthetic_full_image_translations']
 assert len(controls)==9
 assert {x['magnitude_full_px'] for x in controls}=={.3,.5,.8}
 assert all(len(case['comparisons'])==4 for case in controls)
 # This known textured reference must retain detection of all tested shifts.
 # Real-reference failures remain reported; production parameters never change.
 assert diag['all_synthetic_motion_detected_by_original_p90']
 for case in controls:
  for result in case['comparisons']:
   assert result['tracks_eligible'] and result['motion_p90_px']>.25
 assert all(np.array_equal(saved[k],v) for k,v in arrays.items())


def test_direction_statistics_do_not_call_opposed_motion_coherent():
 result=m.direction_statistics(np.array([[1.,0.],[-1.,0.],[0.,1.],[0.,-1.]]))
 assert result['unit_vector_resultant']==0
 assert result['mean_direction_xy'] is None
 assert m.direction_statistics(np.empty((0,2)))['count']==0


def test_sealed_reader_rejects_unsealed_or_path_escape_and_checks_byte_binding(tmp_path):
 p=tmp_path/'snapshot_00.npz';p.write_bytes(b'fake')
 summary=dict(schema=1,sealed=True,errors=[],diagnostic_only=True,pixels_changed=False,motion_authorized=False,
  coordinate_domain=m.DOMAIN,original_process_restored=True,original_observe_restored=True,original_reset_restored=True,
  queued_unwritten=0,inflight_copies=0,queue_bytes=0,files=[dict(name=p.name,bytes=4)])
 target=tmp_path/'summary.json';target.write_text(json.dumps(summary))
 assert m.sealed_entries(tmp_path)[1]==summary['files']
 for change in ({'sealed':False},{'queued_unwritten':1},{'original_observe_restored':False},
                {'files':[]},{'files':[dict(name='../elsewhere.npz',bytes=4)]},{'files':[dict(name=p.name,bytes=5)]}):
  broken=dict(summary,**change);target.write_text(json.dumps(broken))
  with pytest.raises(ValueError):m.sealed_entries(tmp_path)
 target.write_text(json.dumps(summary));alias=tmp_path/'alias.npz';alias.symlink_to(p)
 summary['files']=[dict(name='alias.npz',bytes=4)];target.write_text(json.dumps(summary))
 with pytest.raises(ValueError,match='snapshot_path_or_duplicate_invalid'):m.sealed_entries(tmp_path)
 target.unlink();original=tmp_path/'original.json';original.write_text(json.dumps(summary));target.symlink_to(original)
 with pytest.raises(ValueError,match='summary_symlink_not_allowed'):m.sealed_entries(tmp_path)
