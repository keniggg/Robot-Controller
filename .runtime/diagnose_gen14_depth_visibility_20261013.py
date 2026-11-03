"""Original measured 3D reprojection/visibility diagnostics, no image or mask alteration."""
import sys,json
from pathlib import Path
import cv2,numpy as np
sys.path.insert(0,str(Path(__file__).parent))
from diagnose_gen14_fixed_proposals_20261013 import F,PRE,plain
from alicia_grasp_modes.tabletop import _project

def main():
 out=Path(sys.argv[1]);out.mkdir(parents=True,exist_ok=False)
 f=json.load(open(F/'fixture.json')); cam=f['camera'];k=np.array([cam[n]for n in ('fx','fy','cx','cy')]);pre=np.load(F/f'{PRE}_source.npz');plane=np.array(json.load(open(F/'original_geometry_reconstruction_v2.json'))['seed']['reference_plane_base'])
 def reproj(a,am,b,bm):
  xyz=_project(a['depth'],k);base=xyz@a['T_base_camera'][:3,:3].T+a['T_base_camera'][:3,3];h=base@plane[:3]+plane[3]
  target=(base-b['T_base_camera'][:3,3])@b['T_base_camera'][:3,:3];x=target[:,:,0];u=k[2]-k[0]*target[:,:,1]/x;v=k[3]-k[1]*target[:,:,2]/x
  initial=(am>0)&np.isfinite(a['depth'])&(a['depth']>.03)&(a['depth']<2);good=initial&np.isfinite(target).all(axis=2)&(x>.01)&(u>=0)&(v>=0)&(u<bm.shape[1]-.5)&(v<bm.shape[0]-.5)
  yy,xx=np.nonzero(good);iu=np.rint(u[good]).astype(int);iv=np.rint(v[good]).astype(int);targetdepth=b['depth'][iv,iu];inside=bm[iv,iu]>0;valid=np.isfinite(targetdepth)&(targetdepth>.03)&(targetdepth<2);delta=x[good]-targetdepth;heights=h[good]
  output={}
  for label,take in [('all',np.ones(len(xx),bool)),('height_le_15mm',heights<=.015),('height_gt_15mm',heights>.015)]:
   measure=take&valid&inside
   output[label]=dict(projected_count=int(take.sum()),inside_target_mask=int((take&inside).sum()),inside_target_with_depth=int(measure.sum()),depth_delta_quantiles_m=np.quantile(delta[measure],[0,.05,.25,.5,.75,.95,1])if measure.any()else[],behind_target_more_3mm=int((measure&(delta>.003)).sum()),behind_target_more_7mm=int((measure&(delta>.007)).sum()),aligned_abs_le_3mm=int((measure&(abs(delta)<=.003)).sum()),aligned_abs_le_7mm=int((measure&(abs(delta)<=.007)).sum()))
  output['source_pixels']=int((am>0).sum());output['source_valid_depth']=int(initial.sum());output['in_target_canvas']=int(good.sum())
  return output
 rows=[]
 for fr in f['frames']:
  ns=fr['source_stamp_ns']
  if ns<1791860212000000000:continue
  cur=np.load(fr['file']);cm=cv2.imread(str(F/f'{ns}_rejected_proposal_0_mask.png'),0);rows.append(dict(source_ns=ns,pre_to_current=reproj(pre,pre['recorded_mask'],cur,cm),current_to_pre=reproj(cur,cm,pre,pre['recorded_mask'])))
 (out/'result.json').write_text(json.dumps(plain(dict(scope='Original 3D/depth reprojection only; no occlusion authorization inferred from missing or discrepant depth.',rows=rows)),indent=2)+'\n');print(json.dumps(plain(rows[0]),indent=2))
if __name__=='__main__':main()
