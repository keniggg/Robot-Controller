from pathlib import Path
import json,hashlib
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
ROOT=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_ros_latest_start_20261002/gen4_root_cause/high_posture_20261003T081809Z')
OUT=ROOT/'support_plane_review'
s=json.loads((OUT/'fixed_local_support_report.json').read_text())
events=[json.loads(x) for x in (ROOT/'events.jsonl').read_text().splitlines()]
frames=[x for x in events if x.get('event')=='stationary_snapshot'];order={r['label']:i for i,r in enumerate(frames)}
results=[]
for r in s['results']:
 if r.get('projection_model')!='production_pinhole':continue
 other=next(x for x in s['results'] if x.get('label')==r['label'] and x.get('projection_model')=='sdk_inverse_brown')
 q=np.degrees(r['frame_binding']['bracketing_actual_q_rad'][0][:6]);p=r['fixed_plane_prediction'];folds=r['fixed_plane_prediction_folds']
 results.append(dict(label=r['label'],snapshot_index=order[r['label']],publication_header_stamp_ns=r['destination_ns'],q2_deg=q[1],q5_deg=q[4],local_points=r['counts']['valid_rgbd_correspondences'],spatial_holdout_points=r['heldout'],tf_holdout_rmse_mm=r['tf_heldout_residual_m']['rmse']*1000,visual_holdout_rmse_mm=r['heldout_residual_m']['rmse']*1000,rotation_disagreement_deg=r['tf_rotation_disagreement_deg'],anchor_shift_mm=p['anchor_shift_mm'],fixed_footprint_max_mm=p['max_abs_footprint_shift_mm'],fold_anchor_range_mm=[min(k['anchor_shift_mm'] for k in folds),max(k['anchor_shift_mm'] for k in folds)],pnp_forward_reverse_anchor_mm=[r[k]['anchor_shift_mm'] for k in ('fixed_plane_prediction_pnp_forward','fixed_plane_prediction_pnp_reverse')],inverse_brown_anchor_mm=other['fixed_plane_prediction']['anchor_shift_mm'],matched_local_plane_delta_mm=r['matched_local_planes']['destination_vs_source']['anchor_shift_mm']))
results.sort(key=lambda r:r['snapshot_index'])
summary=dict(scope='Completed fixed local physical-support comparison, no deployment or motion',source_plane_stamp_ns=s['source_plane_stamp_ns'],baseline_header_stamp_ns=s['baseline']['publication_header_stamp_ns'],clock_limitation=s['clock_limitation'],urdf_sha256=s['model']['urdf_sha256'],source_plane_base=s['source_plane_base'],fixed_footprint_base=s['fixed_footprint_base'],baseline_anchor=s['anchor_base'],key_findings=['The first tested substantial error already appears after domain_07_j2, before the seventh J5 movement.','Error is history-dependent: after returning J2 to about51deg, the discrepancy is still much larger than outbound about51deg.','The complete physical pose/SDK return restores small support discrepancy, about0.23mm at the original anchor.','Matched local plane fitting agrees with independent visual motion applied to the original plane; not a consequence of blindly fitting a different global tabletop patch.','Recorded header-bound TF equals current-model FK at both bracketing actual poses to numerical precision, but exposure timing remains uncalibrated.','None of these results uniquely identifies a robot joint, mechanical component, calibration parameter or queue timing cause.'],results=results)
(OUT/'summary.json').write_text(json.dumps(summary,indent=2,allow_nan=False)+'\n')
fig,axs=plt.subplots(2,1,figsize=(11,7),sharex=True)
x=[0]+[r['snapshot_index'] for r in results];y=[0]+[r['anchor_shift_mm'] for r in results]
axs[0].plot(x,y,'o-',label='Frozen plane transported by independent RGB-D motion')
axs[0].plot([r['snapshot_index'] for r in results],[r['matched_local_plane_delta_mm'] for r in results],'x--',label='Same matched local support: fitted plane difference')
axs[0].axhline(0,color='gray',lw=.7);axs[0].set_ylabel('At fixed source anchor (mm)');axs[0].grid(alpha=.2);axs[0].legend(fontsize=8)
for r in results:
 if r['label'] in ('domain_06_j5','domain_07_j2','domain_07_j5','high_j2_positive3_return','return_late'):
  axs[0].annotate(r['label'],(r['snapshot_index'],r['anchor_shift_mm']),textcoords='offset points',xytext=(0,9),ha='center',fontsize=7)
axs[1].plot([r['snapshot_index'] for r in results],[r['tf_holdout_rmse_mm'] for r in results],'o-',label='Recorded TF')
axs[1].plot([r['snapshot_index'] for r in results],[r['visual_holdout_rmse_mm'] for r in results],'o-',label='Independent visual fit')
axs[1].set_xlabel('Completed stationary snapshot index');axs[1].set_ylabel('Spatial holdout RGB-D RMSE (mm)');axs[1].grid(alpha=.2);axs[1].legend()
fig.suptitle('Same local tabletop support across the high-posture sequence\nROS publication-header TF; exposure latency not independently calibrated',fontsize=11)
fig.tight_layout();fig.savefig(str(OUT/'fixed_local_support_comparison.png'),dpi=150);plt.close(fig)
(OUT/'sha256.json').write_text(json.dumps({str(p.name):hashlib.sha256(p.read_bytes()).hexdigest() for p in [OUT/'fixed_local_support_report.json',OUT/'summary.json',OUT/'analysis_source.py',OUT/'fixed_local_support_comparison.png']},indent=2)+'\n')
for r in results:print(r['label'],'q2/q5',round(r['q2_deg'],3),round(r['q5_deg'],3),'anchor',round(r['anchor_shift_mm'],3),'local',round(r['matched_local_plane_delta_mm'],3))
