#!/usr/bin/env python3
"""Inspect original same-frame metric hard seeds in RGB outline disagreements."""
import argparse
from dataclasses import fields
import json
from pathlib import Path
import shutil

import cv2
import numpy as np

import check_saved_rgb_hold_overlap_20261005 as base
from alicia_grasp_modes import tabletop
from alicia_grasp_modes.measured_mask_seeds import confidence_labels


def colours(rgb, mask):
    values=rgb[mask]
    if not len(values):
        return None
    chroma=values.astype(float)/np.maximum(values.astype(float).sum(-1),1.)[:,None]
    return dict(pixels=len(values),median_bgr=np.median(values,axis=0).tolist(),
        median_normalized_bgr=np.median(chroma,axis=0).tolist())


def analyze(capture, geometry, outline, output):
    if not base.MOUNT.is_mount():
        raise RuntimeError('SanStand unavailable; no fallback')
    output.resolve().relative_to(capture.resolve())
    capture.resolve().relative_to(base.EXT)
    record=json.loads((capture/'result.json').read_text())
    metric=json.loads((geometry/'report.json').read_text())
    comparison=json.loads((outline/'report.json').read_text())
    rows=[]
    for cmp in comparison['rows']:
        if cmp['comparison']!='same_frame':
            continue
        index=cmp['reference_index']-1
        entry=record['rows'][index]
        current=max(cmp['current_rgb_seeded_outline']['candidates'],key=lambda c:c['iou'])
        artifacts=[c['mask_artifact'] for c in cmp['current_candidate_refinements'] if 'mask_artifact' in c]
        rgbmask_path=outline/artifacts[current['index']]['path']
        if base.sha(rgbmask_path)!=artifacts[current['index']]['sha256']:
            raise ValueError('RGB outline artifact hash mismatch')
        rgbmask=cv2.imread(str(rgbmask_path),cv2.IMREAD_GRAYSCALE)>0
        geom=metric['rows'][index]['replay']
        masks={}
        for kind in ('depth_seed','refined'):
            artifact=geom['mask_artifacts'][kind]
            file=geometry/artifact['path']
            if base.sha(file)!=artifact['sha256']:
                raise ValueError('original mask artifact hash mismatch')
            masks[kind]=cv2.imread(str(file),cv2.IMREAD_GRAYSCALE)>0
        if base.sha(entry['source_file'])!=entry['source_file_sha256']:
            raise ValueError('source file hash mismatch')
        with np.load(entry['source_file'],allow_pickle=False) as data:
            rgb=data['color_bgr'];depth=data['depth_raw'].astype(np.float32)*float(data['depth_scale'])
        camera=entry['source_context']['camera']
        config=tabletop.Config(**{f.name:entry['config'][f.name] for f in fields(tabletop.Config)})
        points=tabletop._project(depth,[camera[k] for k in ('fx','fy','cx','cy')])
        plane=np.array(geom['selected']['metrics']['support_plane_base'])
        heights=np.einsum('ijk,k->ij',points,plane[:3])+plane[3]
        ys,xs=np.nonzero(masks['depth_seed'])
        margin=max(8,int(round(16*min(depth.shape[1]/640.,depth.shape[0]/480.))))
        x0,x1=max(0,int(xs.min())-margin),min(depth.shape[1],int(xs.max())+margin+1)
        y0,y1=max(0,int(ys.min())-margin),min(depth.shape[0],int(ys.max())+margin+1)
        sl=np.s_[y0:y1,x0:x1]
        core,support,evidence=confidence_labels(masks['depth_seed'][sl],points[sl],depth[sl],heights[sl],plane[:3],config,bgr=rgb[sl])
        if int(np.count_nonzero(core))!=geom['refined']['metrics']['trusted_core_pixels']:
            raise ValueError('original hard foreground count was not reproduced')
        metric_only=(masks['refined']&~rgbmask)[sl]
        rgb_only=(rgbmask&~masks['refined'])[sl]
        rows.append(dict(index=index+1,source_stamp_ns=entry['source_stamp_ns'],
            metric_only=colours(rgb[sl],metric_only),rgb_only=colours(rgb[sl],rgb_only),
            current_rgb_outline=colours(rgb,rgbmask),original_current_support=colours(rgb[sl],support),
            metric_only_hard_foreground_pixels=int(np.count_nonzero(metric_only&core)),
            original_hard_foreground_pixels=int(np.count_nonzero(core)),
            original_seed_evidence=evidence,
            metric_only_heights_mm=None if not np.any(metric_only) else np.percentile(heights[sl][metric_only]*1000,[0,50,100]).tolist()))
    output.mkdir(exist_ok=False)
    report=dict(capture=str(capture),capture_sha256=base.sha(capture/'result.json'),
        metric_report_sha256=base.sha(geometry/'report.json'),outline_report_sha256=base.sha(outline/'report.json'),
        script_sha256=base.sha(__file__),robot_commands_sent=False,metric_geometry_authority=False,
        limitations=['Camera-link diagnostic per-frame original seed labels, not actual live cache or exact failed frame.',
            'Color agreement with local measured support suggests competing appearance evidence; it does not alone prove physical ownership, reflectance, shadow or calibrated true height.'],rows=rows)
    shutil.copy2(__file__,output/Path(__file__).name)
    (output/'report.json').write_text(json.dumps(report,indent=2,ensure_ascii=False,allow_nan=False)+'\n')
    print(json.dumps(dict(report=str(output/'report.json'),sha256=base.sha(output/'report.json'),rows=rows)))


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    for name in ('capture','geometry','outline','output'):
        p.add_argument('--'+name,type=Path,required=True)
    args=p.parse_args()
    analyze(args.capture,args.geometry,args.outline,args.output)
