from pathlib import Path
import hashlib,json
import cv2,numpy as np
from alicia_grasp_modes.rgb_observation_handoff import _appearance
BASE=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_restart_20261010T002347Z/gen21_analysis')
p=BASE/'replay_inputs';m=json.loads((p/'manifest.json').read_text());ab=json.loads((BASE/'crop_ab_replay_r1.json').read_text());refs=[];frames={}
for row in m['files']:
 a=dict(np.load(row['path']));frames[row['source_stamp_ns']]=a
 if row['recorded_refined_mask_available']:
  mask=a['recorded_refined_mask'];h,c=_appearance(a['color'],mask);refs.append((row,h,c))
rows=[]
def color_summary(rgb,mask):
 lab=cv2.cvtColor(rgb,cv2.COLOR_BGR2LAB)[mask!=0];hsv=cv2.cvtColor(rgb,cv2.COLOR_BGR2HSV)[mask!=0];
 return dict(mean_lab=lab.mean(axis=0).tolist(),median_lab=np.median(lab,axis=0).tolist(),p10_p90_l=np.percentile(lab[:,0],[10,90]).tolist(),mean_bgr=rgb[mask!=0].mean(axis=0).tolist(),high_red_fraction=float(np.mean(((hsv[:,0]<10)|(hsv[:,0]>170))&(hsv[:,1]>80))))
refsum={str(r['source_stamp_ns']):color_summary(frames[r['source_stamp_ns']]['color'],frames[r['source_stamp_ns']]['recorded_refined_mask']) for r,_,_ in refs}
for r in ab['results']:
 ns=r['source_ns'];rgb=frames[ns]['color'];mask=cv2.imread(str(BASE/('crop_ab_'+r['variant']+'_'+str(ns)+'_mask.png')),cv2.IMREAD_GRAYSCALE)
 if mask is None:continue
 h,c=_appearance(rgb,mask)
 distances=[dict(reference_source_ns=ref['source_stamp_ns'],colour_distance=float(np.sqrt(max(0.,1.-np.sum(np.sqrt(h*hr))))),contour_distance=float(cv2.matchShapes(cr,c,cv2.CONTOURS_MATCH_I1,0))) for ref,hr,cr in refs]
 rows.append(dict(source_ns=ns,variant=r['variant'],reference_distances=distances,mask_colour_summary=color_summary(rgb,mask)))
report=dict(ab_report=dict(path=str(BASE/'crop_ab_replay_r1.json'),sha256=hashlib.sha256((BASE/'crop_ab_replay_r1.json').read_bytes()).hexdigest()),record=m['record'],recorded_reference_summaries=refsum,results=rows,diagnostic_only=True,policy_thresholds_unchanged=True,limitations=['Alternate recorded references test sensitivity only; they cannot replace the executing request frozen reference.','Mean LAB and HSV red fraction are diagnostics, not identity or foreground proof.','Masks come from real production refinement or explicitly labeled crop-only counterfactual; no synthetic RGB/depth was used.'])
out=BASE/'appearance_reference_sensitivity.json';out.write_text(json.dumps(report,indent=2)+'\n')
print('REPORT',str(out),hashlib.sha256(out.read_bytes()).hexdigest())
for variant in ('production','union48_counterfactual'):
 print(variant)
 for ref,_,_ in refs:
  vals=[next(x for x in r['reference_distances'] if x['reference_source_ns']==ref['source_stamp_ns'])['colour_distance'] for r in rows if r['variant']==variant]
  print(ref['role'],sum(x<=.4 for x in vals),len(vals),min(vals),max(vals))
print('reference_lab',refsum)
print('arrival_lab',rows[-1]['mask_colour_summary'])
