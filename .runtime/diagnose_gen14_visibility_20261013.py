"""Fixed four cyclic outline proposals and immutable source-depth diagnostics. No authority."""
from pathlib import Path
import json,time,sys,hashlib
import numpy as np,cv2
from dataclasses import asdict
sys.path.insert(0,str(Path(__file__).parent))
from diagnose_gen14_fixed_proposals_20261013 import F,PRE,plain,quad
from alicia_grasp_modes.arrival_dense_identity import freeze_reference,propose_homography,evaluate_appearance
from alicia_flexible_grasp.vision.rgbd_rectification import rectification_rgb_valid_mask
from alicia_grasp_modes.tabletop import _project

def main():
 out=Path(sys.argv[1]);out.mkdir(parents=True,exist_ok=False);cv2.setNumThreads(1)
 f=json.loads((F/'fixture.json').read_text());cam=f['camera']; k=[cam[n]for n in ('fx','fy','cx','cy')]
 original=json.loads((F/'original_geometry_reconstruction_v2.json').read_text());plane=np.asarray(original['seed']['reference_plane_base'])
 raw=np.load(F/f'{PRE}_source.npz');valid=rectification_rgb_valid_mask(cam,raw['depth'].shape)
 ref=freeze_reference(raw['rgb'],raw['recorded_mask'],valid,PRE);q0=quad(ref.mask); rows=[]
 def depth_stats(frame,mask,label):
  mask=mask>0;xyz=_project(frame['depth'],k);base=xyz@frame['T_base_camera'][:3,:3].T+frame['T_base_camera'][:3,3]
  height=base@plane[:3]+plane[3];finite=np.isfinite(frame['depth'])&(frame['depth']>.03)&(frame['depth']<2)
  area=int(mask.sum());h=height[mask&finite]
  counts={name:int((mask&finite&test).sum())for name,test in [('support_abs_le_3mm',abs(height)<=.003),('support_abs_le_7mm',abs(height)<=.007),('height_gt_7mm',height>.007),('height_gt_15mm',height>.015)]}
  rgb=frame['rgb'].copy();rgb[~mask]//=4
  overlay=rgb.copy();overlay[mask&finite&(abs(height)<=.003)]=[0,0,255];overlay[mask&~finite]=[255,0,255]
  x,y,w,hbox=cv2.boundingRect(mask.astype('uint8'));cv2.imwrite(str(out/f'{label}_depth_classes.png'),cv2.resize(overlay[max(0,y-6):y+hbox+6,max(0,x-6):x+w+6],None,fx=4,fy=4,interpolation=cv2.INTER_NEAREST))
  return dict(mask_pixels=area,valid_depth_pixels=int((mask&finite).sum()),missing_depth_pixels=int((mask&~finite).sum()),height_quantiles_m=np.quantile(h,[0,.01,.05,.1,.25,.5,.75,.9,.95,.99,1]),counts=counts,red_is_measured_support_le_3mm=True,magenta_is_missing_depth=True)
 predepth=depth_stats(raw,ref.mask,'premotion')
 for fr in f['frames']:
  ns=fr['source_stamp_ns']
  if ns<1791860212000000000:continue
  frame=np.load(fr['file']);m=cv2.imread(str(F/f'{ns}_rejected_proposal_0_mask.png'),0);q1=quad(m);v=rectification_rgb_valid_mask(cam,m.shape)
  cyclic=[]
  for shift in range(4):
   H=cv2.getPerspectiveTransform(q0,np.roll(q1,shift,axis=0));p=propose_homography(ref,ns,H)
   start=time.perf_counter();e=evaluate_appearance(ref,frame['rgb'],m,v,ns,p,deadline_monotonic=time.monotonic()+1)
   cyclic.append(dict(shift=shift,H=H,evidence=asdict(e),wall_sec=time.perf_counter()-start))
  rows.append(dict(source_ns=ns,depth_stats=depth_stats(frame,m,str(ns)),four_predefined_cyclic_proposals=cyclic))
  print(ns,[(c['shift'],c['evidence']['code'],c['evidence']['metrics'].get('holdout_ncc'))for c in cyclic],flush=True)
 (out/'result.json').write_text(json.dumps(plain(dict(scope='Diagnostic four fixed cyclic mappings, no score selection; original gates unchanged. Depth classifications do not alter masks.',premotion=predepth,reference_plane_base=plane,reference_digest=ref.digest,rows=rows)),indent=2)+'\n')
if __name__=='__main__':main()
