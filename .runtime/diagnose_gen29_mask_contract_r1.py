"""Pure offline RGB mask path diagnosis, no producer/ROS/actuation authority."""
from pathlib import Path
from types import SimpleNamespace
from dataclasses import fields, asdict
import hashlib
import json
import sys

W=Path(__file__).resolve().parents[1]
for p in ('src/alicia_grasp_modes/src','src/alicia_flexible_grasp_supervisor/src'):
    sys.path.insert(0,str(W/p))
import cv2
import numpy as np
from alicia_grasp_modes import tabletop as table
from alicia_grasp_modes import rgb_instance as rgbmod
from alicia_flexible_grasp.vision.rgbd_rectification import rectification_rgb_valid_mask

E=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
N=E/'ros_log/full_latest_gen22_restart_20261015T003700Z'
I=N/'verification/gen29_attempt1_shadow_frames'
O=I/'mask_contract_review_r1'
R=E/'grasp_records/20261015T111226.886093Z_0a4de2be16394344b0fa84a6ad2f8318'
STAMPS=[1792062835164456367,1792062835497817993,1792062850569185018,1792062851088963747]

def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def ah(a):return hashlib.sha256(np.ascontiguousarray(a).tobytes()).hexdigest()
def normhash(a):return ah(np.asarray(a>0,dtype=np.uint8))
def iou(a,b):
    a,b=a>0,b>0
    return float(np.count_nonzero(a&b)/np.count_nonzero(a|b))
def serial(x):
    if isinstance(x,np.ndarray):return x.tolist()
    if isinstance(x,np.generic):return x.item()
    raise TypeError(type(x).__name__)
def write(path,data):
    assert not path.exists(),path
    path.write_text(json.dumps(data,ensure_ascii=False,indent=2,default=serial,allow_nan=False)+'\n')

assert Path('/media/zhuyupei/SanStand').is_mount() and not O.exists()
source_paths=[
 'src/alicia_grasp_modes/scripts/unknown_tabletop_perception.py',
 'src/alicia_grasp_modes/src/alicia_grasp_modes/tabletop.py',
 'src/alicia_grasp_modes/src/alicia_grasp_modes/rgb_instance.py',
 'src/alicia_grasp_modes/src/alicia_grasp_modes/measured_mask_seeds.py',
 'src/alicia_grasp_modes/src/alicia_grasp_modes/observation_arrival_confirmation.py',
 'src/alicia_grasp_modes/src/alicia_grasp_modes/arrival_fragment_group.py',
 'src/alicia_grasp_modes/src/alicia_grasp_modes/station_compute_wait_producer.py',
 'src/alicia_grasp_modes/src/alicia_grasp_modes/observation_appearance.py',
 'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/vision/rgbd_rectification.py']
sources={p:sha(W/p) for p in source_paths}
release=json.loads((N/'verification/gen29_release_manifest.json').read_text())
assert all(release['source_sha256'][p]==h for p,h in sources.items())
export=json.loads((I/'result.json').read_text())
assert export['record_closed_verified'] and export['source_bag_stat_unchanged']
inputs={str(I/'result.json'):sha(I/'result.json'),str(R/'perception_execution_configuration.json'):sha(R/'perception_execution_configuration.json')}
cfgvalues=json.loads((R/'perception_execution_configuration.json').read_text())['parameters']['/unknown_perception']
cfg=table.Config(**{f.name:cfgvalues[f.name] for f in fields(table.Config) if f.name in cfgvalues})
loaded={}
for stamp in STAMPS:
    folder=I/('source_%d'%stamp)
    metadata=json.loads((folder/'metadata.json').read_text())
    status=json.loads((folder/'detector_status.json').read_text())['payload']
    arrays=dict(np.load(folder/'raw_arrays.npz',allow_pickle=False))
    for name in ('raw_arrays.npz','detector_status.json','metadata.json'):
        inputs[str(folder/name)]=sha(folder/name)
    assert all(ah(arrays[k])==v for k,v in metadata['array_sha256'].items())
    loaded[stamp]=(arrays,status,metadata)
qstatus=loaded[STAMPS[0]][1]
anchor=SimpleNamespace(position_base=np.asarray(qstatus['anchor_position_base']),
    extent_m=np.asarray(qstatus['anchor_extent_m']),
    support_footprint_base=np.asarray(qstatus['reference_support_footprint_base']),
    metrics=dict(median_height_m=qstatus['reference_object_height_m']))
plane=np.asarray(qstatus['reference_support_plane_base'])
spatial=rgbmod.freeze_observation_spatial_reference(anchor,plane)
O.mkdir()
rows=[]
reconstructed_raw={}
reconstructed_masks={}
for stamp in STAMPS:
    arrays,status,metadata=loaded[stamp]
    rgb=arrays['color_bgr'];depth=arrays['depth_raw'].astype(np.float32)*float(arrays['depth_scale_m'])
    K=arrays['intrinsics'];T=arrays['T_base_camera_link']
    valid=rectification_rgb_valid_mask(metadata['camera_parameters'],depth.shape)
    if stamp==STAMPS[1]:
        last=SimpleNamespace(position_base=np.asarray(qstatus['position_base']))
        prior_basis='previous qualified grouped raw center equals recorded final center'
    elif stamp==STAMPS[-1]:
        last=reconstructed_raw[STAMPS[-2]]
        prior_basis='reconstructed actual unique last-good raw candidate'
    else:
        last=SimpleNamespace(position_base=np.asarray(status['position_base']))
        prior_basis='recorded final center proxy; not unrecorded prior raw tracker state'
    result=table.segment_with_support_fallback(depth,K,T,cfg,anchor,last,plane,prefer_current_support=True)
    matches=table._matching_candidates(result.candidates,anchor,last,cfg)
    # Also check whether the last-position proxy affects the branch at all.
    anchorlast=table._matching_candidates(result.candidates,anchor,anchor,cfg)
    assert [normhash(v.mask) for _,v in anchorlast]==[normhash(v.mask) for _,v in matches]
    row=dict(source_stamp_ns=stamp,prior_tracker_basis=prior_basis,
        candidate_count=len(result.candidates),matching_count=len(matches),
        matching_independent_of_recorded_last_vs_anchor_proxy=True,
        support_source=result.metrics.get('support_source'),support_plane=result.plane_base,
        segmentation_metrics=result.metrics,native_candidates=[])
    frame=rgbmod._FrameRefinement(depth,K,T,spatial,cfg)
    refined=[]
    for _,candidate in matches:
        current=(frame.refine(rgb,candidate,rgb_valid=valid) if len(matches)>1 else
            rgbmod.refine_candidate(rgb,depth,K,T,candidate,cfg,rgb_valid=valid,
                observation_arrival_retry=True,observation_spatial_reference=spatial))
        refined.append(current)
        row['native_candidates'].append(dict(native_pixels=int(np.count_nonzero(candidate.mask)),
            native_normalized_mask_sha256=normhash(candidate.mask),native_bbox=candidate.bbox,
            position_base=candidate.position_base,anchor_distance_m=float(np.linalg.norm(candidate.position_base-anchor.position_base)),
            prior_step_m=float(np.linalg.norm(candidate.position_base-last.position_base)),
            refined_pixels=int(np.count_nonzero(current.mask)),refined_normalized_mask_sha256=normhash(current.mask),
            refined_metrics=current.metrics))
        assert cv2.imwrite(str(O/('%d_native_%d.png' % (stamp,len(refined)))), candidate.mask)
        assert cv2.imwrite(str(O/('%d_refined_%d.png' % (stamp,len(refined)))), current.mask)
    assert len(refined) in (1,2),len(refined)
    mask=(np.logical_and.reduce([c.mask>0 for c in refined]).astype(np.uint8)*255 if len(refined)>1 else refined[0].mask)
    reconstructed_masks[stamp]=mask
    if len(matches)==1:reconstructed_raw[stamp]=matches[0][1]
    expected=(status['frame_timing']['observation_appearance_identity'][0]['current_mask_sha256']
        if stamp==STAMPS[-1] else normhash(arrays['object_mask']))
    row.update(reconstructed_normalized_mask_sha256=normhash(mask),expected_normalized_mask_sha256=expected,
        exact_recorded_mask_hash_matches=normhash(mask)==expected,reconstructed_mask_pixels=int(np.count_nonzero(mask)),
        fixed_station_mask_iou=iou(loaded[STAMPS[0]][0]['object_mask'],mask),
        published_mask_pixels=int(np.count_nonzero(arrays['object_mask'])),
        mask_combination='recorded all-owned intersection (appearance acceptance not replayed)' if len(refined)>1 else 'original refine_candidate')
    assert cv2.imwrite(str(O/('%d_reconstructed.png'%stamp)),mask)
    rows.append(row)
    print(json.dumps({k:row[k] for k in ['source_stamp_ns','support_source','matching_count','reconstructed_mask_pixels','exact_recorded_mask_hash_matches','fixed_station_mask_iou']}),flush=True)

assert all(row['exact_recorded_mask_hash_matches'] for row in rows),'mask reproduction differed; do not claim exact producer reconstruction'
final=rows[-1]
runtime=loaded[STAMPS[-1]][1]['frame_timing']['observation_appearance_identity'][0]
assert abs(final['fixed_station_mask_iou']-runtime['same_station_mask_iou'])<1e-15
same_station=loaded[STAMPS[0]][0]['object_mask']>0
for row in rows:
    m=reconstructed_masks[row['source_stamp_ns']]>0
    missing=same_station&~m;outside=m&~same_station
    depth=loaded[row['source_stamp_ns']][0]['depth_raw']
    row['template_difference']=dict(missing_template_pixels=int(missing.sum()),current_outside_template_pixels=int(outside.sum()),
        missing_template_pixels_with_nonzero_current_depth=int(np.count_nonzero(missing&(depth>0))),
        missing_template_fraction_of_template=float(missing.sum()/same_station.sum()))
    if row['source_stamp_ns']!=STAMPS[0]:
        overlay=loaded[row['source_stamp_ns']][0]['color_bgr'].copy()
        overlay[missing]=(overlay[missing].astype(float)*.45+np.array([0,0,255])*.55).astype(np.uint8)
        overlay[outside]=(overlay[outside].astype(float)*.45+np.array([0,255,0])*.55).astype(np.uint8)
        assert cv2.imwrite(str(O/('%d_template_difference.png'%row['source_stamp_ns'])),overlay)
assert sources=={p:sha(W/p) for p in sources}
assert inputs=={p:sha(p) for p in inputs}
report=dict(schema=1,source_sha256=sources,source_unchanged=True,release_source_binding_valid=True,
    input_sha256=inputs,input_unchanged=True,script_sha256=sha(__file__),
    no_ros_calls=True,no_hardware_access=True,production_modified=False,
    execution_authority=False,physical_grasp_verified=False,
    effective_config=asdict(cfg),recorded_unknown_perception=cfgvalues,
    qualification_template_source_ns=STAMPS[0],rows=rows,
    first_loss_pairwise_last_good_iou=iou(reconstructed_masks[STAMPS[-2]],reconstructed_masks[STAMPS[-1]]),
    recorded_first_loss_appearance=runtime,
    findings=[
      'Both published masks are RGB ownership masks, not depth-validity masks.',
      'Qualified template used locked-plane two-native-fragment independently refined all-owned intersection.',
      'Ordinary tracking used independently current-refit support and a unique native fragment; same RGB refiner and spatial reference, but no trusted side pixels.',
      'Every reconstructed mask matches the recorded published or runtime internal normalized mask SHA256 exactly.',
      'Normal READY may retain a recovery_proof_error until old-reference expiry; expiry makes fixed-template proof mandatory. No threshold changed.'],
    limitations=[
      'Not a full asynchronous producer replay: no source timing, feedback, ownership or ROS callbacks were executed.',
      'Qualified and last-good segmentation use recorded final center as prior-position proxy; tested matching result identical using anchor center and recovered exact mask hash.',
      'Qualified two-fragment intersection reconstructs recorded mask arithmetic; independent appearance acceptance is taken from sealed status, not replayed.',
      'Raw missing depth is present, but optical cause (visible shadow, reflectance, stereo texture or exposure) is not established by these images.',
      'No geometry or execution authority is granted by reconstruction.'])
write(O/'result.json',report)
print(json.dumps(dict(result=str(O/'result.json'),sha256=sha(O/'result.json'),
    all_masks_exact=True,last_good_iou=report['first_loss_pairwise_last_good_iou'])),flush=True)
