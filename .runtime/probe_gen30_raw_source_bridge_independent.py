"""Independent raw-only source bridge case on one recorded two-face frame.

The original fixed adjacent object fixture is retained. Exactly one support-gap
source cell is changed to the same measured height; no proposal or RGB output is
prescribed. This tests whether the raw graph, beyond prefiltered candidates,
provides the independent articulation veto. Offline only; no hardware calls.
"""
from pathlib import Path
from types import SimpleNamespace as NS
from dataclasses import replace
import hashlib,json,os,sys,time
W=Path(__file__).resolve().parents[1];M=Path('/media/zhuyupei/SanStand');N=M/'实验记录/Robot-Controller/ros_log/full_latest_gen22_restart_20261015T003700Z/verification';I=N/'gen29_attempt1_shadow_frames';O=N/'gen30_raw_source_bridge_independent_r1'
assert M.is_mount() and not O.exists() and os.environ['ROS_MASTER_URI']=='http://127.0.0.1:1'
sys.path[:0]=[str(W/'.runtime/realsense-2.56.5'),str(W/'src/alicia_grasp_modes/src'),str(W/'src/alicia_flexible_grasp_supervisor/src')]
import cv2,numpy as np
from alicia_grasp_modes import tabletop as table,rgb_instance as rgb,source_connected_owner as owner
from alicia_grasp_modes.source_sampling_topology import SourceSamplingTopology
from alicia_grasp_modes.rgb_observation_handoff import _appearance
from alicia_grasp_modes.observation_appearance import FrozenObservationAppearance,match_observation_appearance
from alicia_grasp_modes.station_compute_reacquisition import validate_contract
from alicia_flexible_grasp.vision.rgbd_rectification import rectification_rgb_valid_mask

def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def serial(x):
 if isinstance(x,np.ndarray):return x.tolist()
 if isinstance(x,np.generic):return x.item()
 raise TypeError(type(x).__name__)
inputs={}
def read(p):inputs[str(p)]=sha(p);return json.loads(p.read_text())
meta=read(I/'mask_contract_review_r2/result.json');cfg=table.Config(**meta['effective_config']);ns=meta['rows'][1]['source_stamp_ns'];p=I/f'source_{ns}'
status=read(p/'detector_status.json')['payload'];camera=read(p/'metadata.json')['camera_parameters'];inputs[str(p/'raw_arrays.npz')]=sha(p/'raw_arrays.npz');a=dict(np.load(p/'raw_arrays.npz',allow_pickle=False))
qns=meta['qualification_template_source_ns'];qdir=I/f'source_{qns}';qmeta=read(qdir/'metadata.json');inputs[str(qdir/'raw_arrays.npz')]=sha(qdir/'raw_arrays.npz');q=dict(np.load(qdir/'raw_arrays.npz',allow_pickle=False))
qvalid=rectification_rgb_valid_mask(qmeta['camera_parameters'],q['depth_raw'].shape);hist,contour=_appearance(q['color_bgr'],q['object_mask']);reference=FrozenObservationAppearance(q['color_bgr'],q['object_mask'],qvalid,qns,hist,contour)
topology=SourceSamplingTopology(camera);cv2.setNumThreads(1)
depth=a['depth_raw'].astype(np.float32)*float(a['depth_scale_m']);K=a['intrinsics'];T=a['T_base_camera_link'];color=a['color_bgr'].copy()
anchor=NS(position_base=np.asarray(status['anchor_position_base']),extent_m=np.asarray(status['anchor_extent_m']),support_footprint_base=np.asarray(status['reference_support_footprint_base']),metrics={'median_height_m':status['reference_object_height_m']})
last=NS(position_base=np.asarray(meta['rows'][1]['prior_position_used']));locked=np.asarray(status['reference_support_plane_base']);spatial=rgb.freeze_observation_spatial_reference(anchor,locked);valid=rectification_rgb_valid_mask(camera,depth.shape)
# Fixture geometry chosen once: a six-source-pixel-wide 24-row strip outside
# the recorded full outline, with a one-source-pixel measured support gap.
# The existing full mask is used ONLY to locate the synthetic perturbation;
# neither current proposal nor primary output receives any prescribed mask.
mp=N/'gen30_measured_component_prototype_r1'/f'{ns}_whole_refined.png';inputs[str(mp)]=sha(mp);full=cv2.imread(str(mp),0)>0;source_full=topology.source_array(full)
yy,xx=np.nonzero(source_full);center=int(np.median(yy));neighbor_source=np.zeros(topology.source_shape,bool);gap_source=np.zeros_like(neighbor_source)
for y in range(center-12,center+12):
 xs=np.flatnonzero(source_full[y]);edge=int(xs.min())
 neighbor_source[y,edge-7:edge-1]=True;gap_source[y,edge-1]=True
neighbor=topology.output_mask(neighbor_source);gap=topology.output_mask(gap_source)
assert not np.any(neighbor&full) and not np.any(gap&full)
# Keep only current cells that the exact injective map can represent.
rays=table._project(np.ones(depth.shape,np.float32),K).astype(float);pc=T.T@locked;den=np.einsum('ijk,k->ij',rays,pc[:3])
height=float(status['reference_object_height_m']);depth[neighbor]=((height-pc[3])/den)[neighbor];depth[gap]=(-pc[3]/den)[gap]
# One fixed middle-row source cell changes a real support gap into a
# measured articulation. It is not added to any candidate or seed.
bridge_source=np.zeros_like(gap_source); bridge_source[center]=gap_source[center]
assert bridge_source.sum()==1
bridge=topology.output_mask(bridge_source)
depth[bridge]=((height-pc[3])/den)[bridge]
# Same bright side colour, independently located object. No old pixels fill a
# proposed foreground mask: these modifications are explicit synthetic inputs.
colour=np.median(color[full & (np.indices(depth.shape)[0]>int(np.median(np.nonzero(full)[0])))],axis=0).astype(np.uint8)
color[neighbor]=colour;color[gap]=(20,35,190);color[bridge]=colour
source=table.segment_with_support_fallback(depth,K,T,cfg,anchor,last,locked,prefer_current_support=True)
matches=table._matching_candidates(source.candidates,anchor,last,cfg)
rows=[dict(index=i,pixels=int(np.count_nonzero(c.mask)),neighbor_pixels=int(np.count_nonzero((c.mask>0)&neighbor)),matching=bool(table._matching_candidates([c],anchor,last,cfg))) for i,c in enumerate(source.candidates)]
report=dict(original_source_stamp_ns=ns,synthetic_input=True,synthetic_bridge_source_pixels=int(bridge_source.sum()),synthetic_neighbor_source_pixels=int(neighbor_source.sum()),synthetic_neighbor_output_pixels=int(neighbor.sum()),neighbor_reference_fraction=float(neighbor.sum()/np.count_nonzero(reference.mask)),native_candidates=rows,matching_count=len(matches),known_neighbor_colour_bgr=colour,production_or_tests_changed=False,production_deployed=False,no_ros_network_calls=True,no_motion=True)
O.mkdir();cv2.imwrite(str(O/'rgb.png'),color);cv2.imwrite(str(O/'known_neighbor.png'),neighbor.astype(np.uint8)*255);cv2.imwrite(str(O/'support_gap.png'),gap.astype(np.uint8)*255)
try:
 assert len(matches)==1
 primary=rgb.refine_candidate(color,depth,K,T,matches[0][1],cfg,rgb_valid=valid,observation_arrival_retry=True,observation_spatial_reference=spatial)
 contract=status['station_compute_wait'];binding=contract['binding'];request=status['observation_measurement']['request']
 assert binding['selection']==request['selection'] and status['generation']==binding['selection']['generation'] and status['selection_stamp_ns']==binding['selection']['stamp_ns']
 assert request['target_epoch']==binding['target_epoch'] and request['target_track_id']==binding['target_track_id']
 checks=[0]
 def check():
  checks[0]+=1;validate_contract(contract,binding=binding,now_ns=ns+100000000)
 context=dict(reference=reference,reference_role='qualified_station_identity_template',source_ns=ns,same_station_mask_iou_minimum=.85,check_current=check)
 appearance=match_observation_appearance(color,primary.mask,hist,contour,rgb_valid=valid,appearance_context=context)
 report['original_primary_appearance']=appearance;report['actual_new_branch_reachable']=appearance['code']=='SAME_STATION_COMPLETE_OUTLINE_MISMATCH';report['original_primary_neighbor_pixels']=int(np.count_nonzero((primary.mask>0)&neighbor));cv2.imwrite(str(O/'primary_refined.png'),primary.mask)
 if report['actual_new_branch_reachable']:
  # Capture the actual original refiner output for evidence even if the new
  # ownership guard rejects it; do not alter or prescribe that output.
  original_refine=owner._FrameRefinement.refine
  outputs=[]
  def record(frame,*args,**kwargs):
   result=original_refine(frame,*args,**kwargs);outputs.append(result);return result
  owner._FrameRefinement.refine=record
  try:
   restored=owner.recover_source_connected_fragment_owner(source,color,depth,K,T,cfg,anchor,last,spatial,primary,.5,source_topology=topology,camera=camera,source_stamp_ns=ns,rgb_valid=valid,check_current=check)
   report['recovery_outcome']='unchanged' if restored is None else 'restored'
   if restored is not None:report['recovery_audit']=restored.audit
  except ValueError as exc:report.update(recovery_outcome='rejected',reason=str(exc))
  finally:owner._FrameRefinement.refine=original_refine
  if outputs:
   final=outputs[-1];cv2.imwrite(str(O/'complete_original_rgb_output.png'),final.mask)
   report['complete_original_rgb_neighbor_pixels']=int(np.count_nonzero((final.mask>0)&neighbor))
   report['complete_output_fixed_template']=match_observation_appearance(color,final.mask,hist,contour,rgb_valid=valid,appearance_context=context)
  report['source_checks']=checks[0]
except (ValueError,AssertionError) as exc:report.update(fixture_error=type(exc).__name__+': '+str(exc),actual_new_branch_reachable=False)
np.savez_compressed(O/'synthetic_arrays.npz',color_bgr=color,depth_m=depth,known_neighbor=neighbor,support_gap=gap,raw_bridge=bridge,intrinsics=K,T_base_camera_link=T)
report.update(input_sha256=inputs,script_sha256=sha(__file__),source_sha256={str(m.__file__):sha(m.__file__) for m in (owner,table,rgb,sys.modules['alicia_grasp_modes.source_sampling_topology'])},limitations=['One explicitly synthetic neighbor plus one raw source-cell bridge on a recorded frame, not a physical grasp experiment.','Station/selection/source binding checked from preserved immutable records at a fixed offline time; no fresh clearance or runtime authority.','No primary mask or refiner output was prescribed; original segmentation and RGB ran on the modified inputs.'])
assert all(sha(p)==h for p,h in inputs.items())
(O/'result.json').write_text(json.dumps(report,indent=2,default=serial,allow_nan=False)+'\n');print(json.dumps(report,default=serial));print(O/'result.json',sha(O/'result.json'))
