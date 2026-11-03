"""Single source-frozen top plane proposal; diagnostic only."""
import sys,inspect,json,time
from pathlib import Path
import numpy as np,cv2
from dataclasses import asdict
sys.path.insert(0,str(Path(__file__).parent))
from diagnose_gen14_fixed_proposals_20261013 import F,PRE,plain,plane_homography
import alicia_grasp_modes.arrival_dense_identity as dense
from alicia_flexible_grasp.vision.rgbd_rectification import rectification_rgb_valid_mask

def main():
 out=Path(sys.argv[1]);out.mkdir(parents=True,exist_ok=False);cv2.setNumThreads(1)
 f=json.load(open(F/'fixture.json'));cam=f['camera'];k=[cam[x]for x in ('fx','fy','cx','cy')];seed=json.load(open(F/'original_geometry_reconstruction_v2.json'))['seed'];raw=np.load(F/f'{PRE}_source.npz');ref=dense.freeze_reference(raw['rgb'],raw['recorded_mask'],rectification_rgb_valid_mask(cam,raw['depth'].shape),PRE)
 plane=np.array(seed['reference_plane_base']);height=seed['premotion_refined_metrics']['upper_seed_height_m'];normal=raw['T_base_camera'][:3,:3].T@plane[:3];offset=plane[3]-height+plane[:3]@raw['T_base_camera'][:3,3]
 src=inspect.getsource(dense.evaluate_appearance).replace('def evaluate_appearance(', 'def diagnostic_downstream(').replace("return finish(False, 'WHOLE_MASK_INCONSISTENT')", "metrics['diagnostic_original_whole_mask_failed'] = True")
 space=dict(dense.__dict__);exec(src,space);rows=[]
 for fr in f['frames']:
  ns=fr['source_stamp_ns']
  if ns<1791860212000000000:continue
  cur=np.load(fr['file']);mask=cv2.imread(str(F/f'{ns}_rejected_proposal_0_mask.png'),0);valid=rectification_rgb_valid_mask(cam,mask.shape);H=plane_homography(k,raw['T_base_camera'],cur['T_base_camera'],normal,offset);prop=dense.propose_homography(ref,ns,H);actual=asdict(dense.evaluate_appearance(ref,cur['rgb'],mask,valid,ns,prop,deadline_monotonic=time.monotonic()+2));conditional=None
  if actual['code']=='WHOLE_MASK_INCONSISTENT':
   conditional=asdict(space['diagnostic_downstream'](ref,cur['rgb'],mask,valid,ns,prop,deadline_monotonic=time.monotonic()+2));conditional.update(downstream_code=conditional['code'],code='DIAGNOSTIC_ONLY_ORIGINAL_WHOLE_MASK_REJECTED',appearance_match=False,diagnostic_only=True)
  rows.append(dict(source_ns=ns,H=H,evidence=actual,conditional_dense_probe=conditional));m=(conditional or actual)['metrics'];print(ns,actual['code'],(conditional or{}).get('downstream_code'),{x:m.get(x)for x in ('forward_mask_iou','reference_retained_fraction','holdout_ncc','good_test_cells','required_good_test_cells')},flush=True)
 (out/'result.json').write_text(json.dumps(plain(dict(scope='Original support normal plus original measured upper seed height; frozen before current. Conditional downstream remains false and cannot authorize.',frozen_support_plane=plane,frozen_upper_seed_height=height,reference_digest=ref.digest,rows=rows)),indent=2)+'\n')
if __name__=='__main__':main()
