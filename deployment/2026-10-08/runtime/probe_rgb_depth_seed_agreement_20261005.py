#!/usr/bin/env python3
"""Diagnostic CURRENT RGB / measured top seed agreement, no production changes."""
import json
from dataclasses import fields
from pathlib import Path
import shutil
import cv2
import numpy as np
import check_saved_rgb_hold_overlap_20261005 as base
from alicia_grasp_modes import tabletop
from alicia_grasp_modes.measured_mask_seeds import confidence_labels

CAP=base.EXT/'ros_log/rgb_observation_bootstrap_20261005T034713Z'
GEO=CAP/'offline_geometry_20261005T0348Z'
OUT=CAP/'offline_rgb_depth_seed_agreement_20261005T0441Z'
if not base.MOUNT.is_mount():
    raise RuntimeError('SanStand unavailable; no fallback')
capture=json.loads((CAP/'result.json').read_text())
geometry=json.loads((GEO/'report.json').read_text())
if geometry['original_result_sha256']!=base.sha(CAP/'result.json'):
    raise ValueError('capture hash mismatch')
rows=[]
cv2.setNumThreads(1)
for index,(entry,geomrow) in enumerate(zip(capture['rows'],geometry['rows']),1):
    geom=geomrow.get('replay',{})
    if 'refined' not in geom:
        continue
    if base.sha(entry['source_file'])!=entry['source_file_sha256']:
        raise ValueError('source file hash mismatch')
    with np.load(entry['source_file'],allow_pickle=False) as f:
        rgb=f['color_bgr'];depth=f['depth_raw'].astype(np.float32)*float(f['depth_scale'])
    artifact=geom['mask_artifacts']['depth_seed'];path=GEO/artifact['path']
    if base.sha(path)!=artifact['sha256']:
        raise ValueError('measured seed hash mismatch')
    seed=cv2.imread(str(path),cv2.IMREAD_GRAYSCALE)>0
    camera=entry['source_context']['camera']
    cfg=tabletop.Config(**{f.name:entry['config'][f.name] for f in fields(tabletop.Config)})
    points=tabletop._project(depth,[camera[k] for k in ('fx','fy','cx','cy')])
    plane=np.array(geom['selected']['metrics']['support_plane_base'])
    heights=np.einsum('ijk,k->ij',points,plane[:3])+plane[3]
    ys,xs=np.nonzero(seed);margin=max(8,int(round(16*min(depth.shape[1]/640.,depth.shape[0]/480.))))
    x0,x1=max(0,int(xs.min())-margin),min(depth.shape[1],int(xs.max())+margin+1)
    y0,y1=max(0,int(ys.min())-margin),min(depth.shape[0],int(ys.max())+margin+1)
    sl=np.s_[y0:y1,x0:x1]
    core,support,evidence=confidence_labels(seed[sl],points[sl],depth[sl],heights[sl],plane[:3],cfg,bgr=rgb[sl])
    if evidence['upper_surface_band_used']:
        raise ValueError('this captured diagnostic requires separate explicit upper-band handling')
    if core.sum()!=geom['refined']['metrics']['trusted_core_pixels']:
        raise ValueError('original hard core differs')
    top=core&(heights[sl]>=evidence['upper_seed_height_m'])
    policy=base.CenterSearchPolicy(tuple(np.asarray(geom['selected']['center_uv'])/[rgb.shape[1],rgb.shape[0]]),radius_fraction=cfg.center_radius_fraction)
    valid=base.rectification_rgb_valid_mask(camera,depth.shape)
    regions=base.propose_rgb_observation_regions(rgb,policy=policy,**({} if valid is None else {'rgb_valid':valid}))
    comparisons=[]
    measured=seed[sl]&(heights[sl]>=cfg.foreground_min_m)&(heights[sl]<=cfg.foreground_max_m)
    for ci,region in enumerate(regions.candidates):
        outline=region.mask[sl]
        interior=(cv2.erode(region.mask.astype(np.uint8),np.ones((3,3),np.uint8))>0)[sl]
        fullinterior=int(np.count_nonzero(cv2.erode(region.mask.astype(np.uint8),np.ones((3,3),np.uint8))))
        topcoverage=float(np.count_nonzero(top&outline)/max(1,np.count_nonzero(top)))
        measuredfraction=float(np.count_nonzero(interior&measured)/max(1,fullinterior))
        proposed=top|(core&interior)
        comparisons.append(dict(candidate_index=ci,bbox=list(region.bbox_xywh),
            measured_top_inside_current_rgb_fraction=topcoverage,
            current_rgb_interior_measured_foreground_fraction=measuredfraction,
            current_rgb_interior_pixels=fullinterior,
            hypothetical_hard_core_pixels=int(proposed.sum()),
            hypothetical_side_hard_to_probable_pixels=int(np.count_nonzero(core&~proposed)),
            dual_original_retention_scale_pass=bool(topcoverage>=.85 and measuredfraction>=.85 and proposed.sum()>=25)))
    winners=[c['candidate_index'] for c in comparisons if c['dual_original_retention_scale_pass']]
    rows.append(dict(index=index,source_stamp_ns=entry['source_stamp_ns'],original_hard_core_pixels=int(core.sum()),
        original_top_pixels=int(top.sum()),original_side_pixels=evidence['trusted_side_pixels'],
        proposal_reason=regions.reason,candidates=comparisons,unique_joint_evidence=len(winners)==1,
        candidate_indices_meeting_both_scales=winners))
OUT.mkdir(exist_ok=False)
report=dict(capture=str(CAP),capture_sha256=base.sha(CAP/'result.json'),
    geometry_sha256=base.sha(GEO/'report.json'),script_sha256=base.sha(__file__),
    robot_commands_sent=False,production_changes=False,
    limitations=['Conditional algorithm feasibility only, not a new production contract. The two .85 overlap uses are explicit proposed evidence tests, not proof existing identity gates passed.',
        'No current support can be inferred in missing-support frames; this inspects only accepted saved frames to improve future reference quality.',
        'No external physical truth establishes whether disagreement pixels are genuine faces, reflected colors or stereo ramps.'],rows=rows,
    summary=dict(frames=len(rows),unique_joint_evidence=sum(r['unique_joint_evidence'] for r in rows)))
shutil.copy2(__file__,OUT/Path(__file__).name)
(OUT/'report.json').write_text(json.dumps(report,ensure_ascii=False,indent=2,allow_nan=False)+'\n')
print(json.dumps(dict(report=str(OUT/'report.json'),sha256=base.sha(OUT/'report.json'),summary=report['summary'],rows=rows)))
