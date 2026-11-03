"""Bounded raw-current side labeling diagnosis; no modified production output."""
from pathlib import Path
from types import SimpleNamespace as NS
import json,hashlib,sys
import numpy as np
W=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(W/'src/alicia_grasp_modes/src'))
from alicia_grasp_modes import tabletop as table
from alicia_grasp_modes.measured_mask_seeds import local_surface_normals
E=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
N=E/'ros_log/full_latest_gen22_restart_20261015T003700Z'
I=N/'verification/gen29_attempt1_shadow_frames'
O=N/'verification/gen30_current_side_label_diagnosis'
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
assert Path('/media/zhuyupei/SanStand').is_mount() and not O.exists()
meta=json.loads((I/'mask_contract_review_r2/result.json').read_text())
q=np.load(I/'source_1792062835164456367/raw_arrays.npz')['object_mask']>0
source_hashes={str(Path(m.__file__)):sha(m.__file__) for m in (table,)}
rows=[];inputs={str(I/'mask_contract_review_r2/result.json'):sha(I/'mask_contract_review_r2/result.json')}
for row in meta['rows'][1:]:
 ns=row['source_stamp_ns'];p=I/('source_%d'%ns);a=np.load(p/'raw_arrays.npz')
 s=json.loads((p/'detector_status.json').read_text())['payload'];cfg=table.Config(**meta['effective_config'])
 inputs[str(p/'raw_arrays.npz')]=sha(p/'raw_arrays.npz');inputs[str(p/'detector_status.json')]=sha(p/'detector_status.json')
 current=np.asarray(__import__('cv2').imread(str(I/('mask_contract_review_r2/%d_reconstructed.png'%ns)),0))>0
 region=q&~current;depth=a['depth_raw'].astype(np.float32)*float(a['depth_scale_m']);T=a['T_base_camera_link'];K=a['intrinsics']
 points=table._project(depth,K);valid=(depth>=cfg.depth_min_m)&(depth<=cfg.depth_max_m)
 normals,reliable=local_surface_normals(points,depth,valid)
 item=dict(source_stamp_ns=ns,missing_template_pixels=int(region.sum()),planes={})
 anchor=NS(position_base=np.asarray(s['anchor_position_base']),extent_m=np.asarray(s['anchor_extent_m']),support_footprint_base=np.asarray(s['reference_support_footprint_base']))
 for name,plane in [('locked',np.asarray(s['reference_support_plane_base'])),('current',np.asarray(row['support_plane']))]:
  camera=T.T@plane;heights=np.einsum('ijk,k->ij',points,camera[:3])+camera[3]
  side=reliable&(np.abs(np.einsum('ijk,k->ij',normals,camera[:3]/np.linalg.norm(camera[:3])))<=np.sqrt(.5))
  def count(r):return int(np.count_nonzero(region&valid&r))
  result=table.segment(depth,K,T,cfg,fit_current_support=False,target_position_base=anchor.position_base,plane_base=plane,target_footprint_base=anchor.support_footprint_base)
  item['planes'][name]=dict(plane=plane.tolist(),valid_pixels=int(np.count_nonzero(region&valid)),
   height_quantiles=np.quantile(heights[region&valid],[0,.05,.5,.95,1]).tolist(),
   hard_support=count(np.abs(heights)<=cfg.support_threshold_m),
   below_minus_support=count(heights<-cfg.support_threshold_m),
   positive_grey=count((heights>cfg.support_threshold_m)&(heights<cfg.foreground_min_m)),
   foreground=count((heights>=cfg.foreground_min_m)&(heights<=cfg.foreground_max_m)),
   reliable_side_foreground=count(side&(heights>=cfg.foreground_min_m)&(heights<=cfg.foreground_max_m)),
   candidates=[dict(pixels=int(np.count_nonzero(c.mask)),bbox=c.bbox,position_base=c.position_base.tolist(),
     missing_region_overlap=int(np.count_nonzero((c.mask>0)&region)),
     distance_to_anchor=float(np.linalg.norm(c.position_base-anchor.position_base)),
     step_to_replayed_last=float(np.linalg.norm(c.position_base-np.asarray(row['prior_position_used'])))) for c in result.candidates])
 rows.append(item)
O.mkdir();out=dict(rows=rows,input_sha256=inputs,source_sha256=source_hashes,
 script_sha256=sha(__file__),no_ros_calls=True,production_modified=False,execution_authority=False,
 template_mask_used_only_as_diagnostic_region=True,
 limitations=['Plane comparison is diagnostic only; neither plane is selected for execution by this script.',
 'No previous image supplies current seed labels. Current raw pixel heights are evaluated inside a diagnostic difference region.'])
(O/'result.json').write_text(json.dumps(out,indent=2)+'\n')
for row in rows:print(json.dumps(row),flush=True)
print('RESULT',O/'result.json',sha(O/'result.json'))
