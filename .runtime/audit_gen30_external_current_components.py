"""Finite raw-source component audit of three outputs and one reachable fixture."""
from pathlib import Path
from types import SimpleNamespace as NS
import os,sys,json,hashlib
W=Path(__file__).resolve().parents[1];M=Path('/media/zhuyupei/SanStand');N=M/'实验记录/Robot-Controller/ros_log/full_latest_gen22_restart_20261015T003700Z/verification';I=N/'gen29_attempt1_shadow_frames';O=N/'gen30_external_current_components_r1'
assert M.is_mount() and not O.exists() and os.environ['ROS_MASTER_URI']=='http://127.0.0.1:1'
sys.path[:0]=[str(W/'.runtime/realsense-2.56.5'),str(W/'src/alicia_grasp_modes/src'),str(W/'src/alicia_flexible_grasp_supervisor/src')]
import cv2,numpy as np
from alicia_grasp_modes import tabletop as table,rgb_instance as rgb
from alicia_grasp_modes.source_sampling_topology import SourceSamplingTopology
NATIVE=N/'gen30_source_connected_owner_r2';meta=json.loads((I/'mask_contract_review_r2/result.json').read_text());cfg=table.Config(**meta['effective_config']);rows=[];topo=None
for index in range(4):
 prior=meta['rows'][1:][min(index,2)] if index<3 else meta['rows'][1];ns=prior['source_stamp_ns'];p=I/f'source_{ns}'
 camera=json.loads((p/'metadata.json').read_text())['camera_parameters'];a=dict(np.load(p/'raw_arrays.npz',allow_pickle=False));depth=a['depth_raw'].astype(np.float32)*float(a['depth_scale_m']);color=a['color_bgr'];K=a['intrinsics'];T=a['T_base_camera_link'];plane=np.asarray(prior['support_plane'])
 status=json.loads((p/'detector_status.json').read_text())['payload'];anchor=NS(position_base=np.asarray(status['anchor_position_base']),extent_m=np.asarray(status['anchor_extent_m']),support_footprint_base=np.asarray(status['reference_support_footprint_base']),metrics={'median_height_m':status['reference_object_height_m']});last=NS(position_base=np.asarray(prior['prior_position_used']));locked=np.asarray(status['reference_support_plane_base'])
 if topo is None:topo=SourceSamplingTopology(camera)
 if index<3:mask=cv2.imread(str(NATIVE/f'{ns}_owner.png'),0)>0
 else:
  aa=dict(np.load(N/'gen30_reachable_source_neighbor_r1/synthetic_arrays.npz',allow_pickle=False));depth=aa['depth_m'];color=aa['color_bgr'];mask=cv2.imread(str(N/'gen30_reachable_source_neighbor_r1/complete_original_rgb_output.png'),0)>0
 result=table.segment_with_support_fallback(depth,K,T,cfg,anchor,last,locked,prefer_current_support=True);plane=result.plane_base
 frame=rgb._FrameRefinement(depth,K,T,rgb.freeze_observation_spatial_reference(anchor,locked),cfg);pc=T.T@plane;heights=np.einsum('ijk,k->ij',frame.points,pc[:3])+pc[3]
 valid=np.isfinite(depth)&(depth>=cfg.depth_min_m)&(depth<=cfg.depth_max_m)&(heights>=cfg.foreground_min_m)&(heights<=cfg.foreground_max_m)
 components=table._depth_components(topo.source_array(valid).astype(np.uint8),topo.source_array(depth),cfg.min_object_points,cfg.depth_edge_m,connectivity=4)
 primary=table._matching_candidates(result.candidates,anchor,last,cfg)[0][1];primary_source=topo.source_array(primary.mask>0)
 details=[]
 for i,c in enumerate(components):
  output=topo.output_mask(c);overlap=mask&output
  if not overlap.any():continue
  yy,xx=np.nonzero(output);margin=16;sl=np.s_[max(0,yy.min()-margin):min(depth.shape[0],yy.max()+margin+1),max(0,xx.min()-margin):min(depth.shape[1],xx.max()+margin+1)]
  core,_,evidence=frame.confidence_labels(output[sl],frame.points[sl],depth[sl],heights[sl],pc[:3],cfg,bgr=color[sl]);fullcore=np.zeros(depth.shape,bool);fullcore[sl]=core
  details.append(dict(index=i,pixels=int(c.sum()),contains_primary=bool(np.any(c&primary_source)),rgb_overlap=int(overlap.sum()),trusted_core_pixels=int(core.sum()),trusted_core_overlap=int(np.count_nonzero(fullcore&mask)),neighbor_overlap=None if index<3 else int(np.count_nonzero(output&aa['known_neighbor']))))
 rows.append(dict(case=str(ns) if index<3 else 'reachable_neighbor',component_count=len(components),overlapping_components=details))
O.mkdir();(O/'result.json').write_text(json.dumps(dict(rows=rows,production_changed=False,no_ros=True),indent=2)+'\n');print(json.dumps(rows,indent=2));print(O/'result.json',hashlib.sha256((O/'result.json').read_bytes()).hexdigest())
