from pathlib import Path
import json,hashlib
import numpy as np
O=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_ros_20261009T003407Z/gen7_registration_analysis')
texture=json.loads((O/'texture_registration_deduplicated.json').read_text());branch=json.loads((O/'branch_trace.json').read_text());finite=json.loads((O/'finite_yaw_seed_trace.json').read_text());third=next(r for r in finite['rows'] if r['initial_yaw_deg']==0)
rows=[]
for row in texture['rows']:
 for fit in row.get('thresholds',[]):
  if fit['inlier_limit_m']!=.003:continue
  matches=row['raw_pixel_pairs'];a=np.asarray([p['reference_point_base'] for p in matches]);b=np.asarray([p['moving_point_base'] for p in matches]);indices=fit['inlier_indices'];results={}
  for name,t in dict(original_pca=branch['pca']['transform'],identity_seed=branch['identity']['transform'],translation_only_seed=third['transform'],texture_self_fit=fit['transform_base']).items():
   T=np.asarray(t);delta=b@T[:3,:3].T+T[:3,3]-a;res=np.linalg.norm(delta,axis=1);z=res[indices];results[name]=dict(frozen_inlier_rmse_mm=float(np.sqrt(np.mean(z*z))*1000),frozen_inlier_max_mm=float(z.max()*1000),frozen_inlier_median_mm=float(np.median(z)*1000),all_residuals_mm=(res*1000).tolist())
  rows.append(dict(detector=row['detector'],ratio=row['ratio'],frozen_inlier_count=len(indices),frozen_indices=indices,texture_yaw_deg=fit['yaw_deg'],results=results))
report=dict(diagnostic_only=True,execution_authorized=False,matching_reselected=False,fit_recomputed=False,method='Evaluate all transforms on identical frozen mutually matched valid-depth feature correspondences and the pre-existing 3mm texture consensus. No new feature or consensus filtering.',input_sha256={n:hashlib.sha256((O/n).read_bytes()).hexdigest() for n in ['texture_registration_deduplicated.json','branch_trace.json','finite_yaw_seed_trace.json']},third_seed=third,rows=rows)
(O/'texture_frozen_transform_comparison.json').write_text(json.dumps(report,indent=2)+'\n')
for row in rows:print(row['detector'],row['ratio'],row['frozen_inlier_count'],{k:round(v['frozen_inlier_rmse_mm'],4) for k,v in row['results'].items()})
