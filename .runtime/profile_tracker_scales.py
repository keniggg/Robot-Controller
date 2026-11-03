import importlib.util,json,time
from pathlib import Path
import rospy,cv2,numpy as np
root=Path('/home/zhuyupei/alicia_wa_full/.worktrees/protocol-v3-upgrade')
out=Path(json.loads((root/'.runtime/current_grasp_repair_resume.json').read_text())['directory'])
s=importlib.util.spec_from_file_location('tracker',root/'src/real-arm/alicia_d_calibration/scripts/charuco_tracker.py');m=importlib.util.module_from_spec(s);s.loader.exec_module(m)
rospy.init_node('readonly_board_scale_comparison',anonymous=True,disable_signals=True)
evidence=m.runtime_color_model_evidence(rospy.get_param('/camera'),'camera_link','/supervisor/camera/color/image_raw');model=m.RuntimeColorModel(evidence)
image=cv2.imread(str(out/'tracker_restart_source_rgb.png'))
rows=[]
for scale in [1.,1.5]:
 tr=m.CharucoTracker.__new__(m.CharucoTracker);tr.dictionary=cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_100);tr.board=cv2.aruco.CharucoBoard((12,9),.015,.01125,tr.dictionary);tr.board.setLegacyPattern(True);tr.detection_scale=scale;tr.detection_roi_padding_px=45.;tr.detection_roi=None
 for i in range(3):
  st=time.perf_counter();c,ids=tr.detect_charuco_board(image);dt=(time.perf_counter()-st)*1000
  p,ip=tr._board_points(c,ids);rv,tv,rms=model.solve(p,ip)
  rows.append(dict(scale=scale,iteration=i,detect_ms=dt,corners=len(ids),edge_clearance_px=tr.last_edge_clearance_px,reprojection_rms_px=rms,rvec=rv.reshape(-1).tolist(),tvec=tv.reshape(-1).tolist(),roi=tr.detection_roi))
a=rows[2];b=rows[5];Ra=cv2.Rodrigues(np.array(a['rvec']))[0];Rb=cv2.Rodrigues(np.array(b['rvec']))[0]
report=dict(model_fingerprint=model.fingerprint,rows=rows,translation_difference_mm=(np.array(a['tvec'])-np.array(b['tvec'])).tolist(),rotation_difference_deg=float(np.degrees(np.arccos(np.clip((np.trace(Ra@Rb.T)-1)/2,-1,1)))))
report['translation_difference_mm']=(1000*(np.array(a['tvec'])-np.array(b['tvec']))).tolist();report['translation_difference_norm_mm']=float(np.linalg.norm(report['translation_difference_mm']))
(out/'tracker_restart_scale_comparison.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))
