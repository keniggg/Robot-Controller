#!/usr/bin/env python3
"""Read-only original full-outline RGB identity metrics on archived exact masks."""
import hashlib
import json
from pathlib import Path
import cv2
import numpy as np
from alicia_grasp_modes.rgb_observation_handoff import _appearance, HandoffPolicy

OUT = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_ros_20261009T003407Z/gen18_handoff_analysis/canonical_rgb_replay')
source = OUT/'result.json'
d = json.loads(source.read_text())
frames = {}
for row in d['rows']:
    ns = row['source_stamp_ns']
    if 'error' in row:
        continue
    frames[ns] = (cv2.imread(str(OUT/(str(ns)+'_rgb.png'))), cv2.imread(str(OUT/(str(ns)+'_mask.png')), 0))
policy = HandoffPolicy()
def compare(a, b):
    ra, ma = frames[a]; rb, mb = frames[b]
    ha, ca = _appearance(ra, ma); hb, cb = _appearance(rb, mb)
    intersection = int(np.count_nonzero((ma>0)&(mb>0)))
    union = int(np.count_nonzero((ma>0)|(mb>0)))
    iou = intersection/union
    colour = float(np.sqrt(max(0.,1.-np.sum(np.sqrt(ha*hb)))))
    contour = float(cv2.matchShapes(ca,cb,cv2.CONTOURS_MATCH_I1,0.))
    return dict(reference_source_ns=a,current_source_ns=b,reference_pixels=int(np.count_nonzero(ma)),current_pixels=int(np.count_nonzero(mb)),intersection_pixels=intersection,union_pixels=union,mask_iou=iou,colour_distance=colour,contour_distance=contour,original_identity_metrics_pass=bool(iou>=.85 and colour<=policy.max_colour_distance and contour<=policy.max_contour_distance))
normal = sorted(r['source_stamp_ns'] for r in d['rows'] if r.get('online_detail')=='tracked')[-6:]
result = dict(source_result=str(source),source_result_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),original_gate=dict(iou_min=.85,colour_max=policy.max_colour_distance,contour_max=policy.max_contour_distance),last_six_normal_sources=normal,consecutive_normal_pairs=[compare(a,b) for a,b in zip(normal,normal[1:])],acquired_to_lastgood=compare(d['source_stamps']['anchor'],d['source_stamps']['lastgood']),acquired_to_each_normal=[compare(d['source_stamps']['anchor'],n) for n in normal],lastgood_to_firstloss=compare(d['source_stamps']['lastgood'],d['source_stamps']['firstloss']),limitations=['Original RGB identity metrics on the already bitwise-verified exact published masks; no new segmentation or pixel edits.', 'Not a new geometry, freshness or lifecycle qualification; no synthetic Candidate geometry.', 'Normal frames were accepted online without this inter-frame identity comparison; these pairwise decisions are diagnostic.'],ros_calls=False)
p=OUT/'pairwise_identity.json'; p.write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
print(json.dumps(dict(path=str(p),sha256=hashlib.sha256(p.read_bytes()).hexdigest(),**result),ensure_ascii=False,indent=2))
