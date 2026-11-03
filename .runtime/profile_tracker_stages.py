import importlib.util,json,time
from pathlib import Path
import cv2
root=Path('/home/zhuyupei/alicia_wa_full/.worktrees/protocol-v3-upgrade')
out=Path(json.loads((root/'.runtime/current_grasp_repair_resume.json').read_text())['directory'])
s=importlib.util.spec_from_file_location('tracker',root/'src/real-arm/alicia_d_calibration/scripts/charuco_tracker.py');m=importlib.util.module_from_spec(s);s.loader.exec_module(m)
image=cv2.imread(str(out/'tracker_restart_source_rgb.png'));gray=cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
d=cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_100);b=cv2.aruco.CharucoBoard((12,9),.015,.01125,d);b.setLegacyPattern(True)
p=cv2.aruco.DetectorParameters();p.cornerRefinementMethod=cv2.aruco.CORNER_REFINE_NONE
rows=[]
for _ in range(4):
 row={};t=time.perf_counter();scaled,scale=m.prepare_detection_image(gray[137:438,132:498],1.5);row['resize_ms']=(time.perf_counter()-t)*1000;t=time.perf_counter();detector=cv2.aruco.ArucoDetector(d,p);cd=cv2.aruco.CharucoDetector(b,detectorParams=p);row['construct_ms']=(time.perf_counter()-t)*1000;t=time.perf_counter();corners,ids,_=detector.detectMarkers(scaled);row['markers_ms']=(time.perf_counter()-t)*1000;t=time.perf_counter();cc,ci,_,_=cd.detectBoard(scaled,None,None,corners,ids);row['charuco_ms']=(time.perf_counter()-t)*1000;row['markers']=len(ids);row['corners']=len(ci);rows.append(row)
(out/'tracker_restart_stages.json').write_text(json.dumps(rows,indent=2));print(json.dumps(rows))
