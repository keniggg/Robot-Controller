"""Independent diagnostic only: no new ownership rule or changed production gate."""
import hashlib,json
from pathlib import Path
import cv2,numpy as np
from alicia_grasp_modes.tabletop import _project

BASE=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_restart_20261010T002347Z/gen24_analysis')
src=BASE/'fragment_ownership'; out=BASE/'fragment_side_independent';out.mkdir(exist_ok=True)
manifest=json.loads((BASE/'replay_inputs/manifest.json').read_text())
row=next(x for x in manifest['files'] if x['role']=='first_failure')
f=dict(np.load(row['path'])); m=dict(np.load(src/'masks.npz'))
rgb=m['rgb']; depth=m['depth']; heights=m['heights']; domain=m['domain'].astype(bool)
cores=m['cores'].astype(bool); instances=m['instances'].astype(bool);native=m['native'].astype(bool)
lost=cores[1]&~instances[0]; retained=cores[1]&instances[0]
lab=cv2.cvtColor(rgb,cv2.COLOR_BGR2LAB).astype(float)
chroma=rgb.astype(float)/np.maximum(rgb.astype(float).sum(-1),1)[...,None]
anchor=json.loads((BASE/'replay_inputs/anchor_status.json').read_text())['message']
plane=np.array(anchor['reference_support_plane_base']); normal=plane[:3]/np.linalg.norm(plane[:3])
fp=np.array(anchor['reference_support_footprint_base'])
axis=np.eye(3)[np.argmin(abs(normal))];u=np.cross(normal,axis);u/=np.linalg.norm(u)
basis=np.stack((u,np.cross(normal,u)),axis=1)
hull=cv2.convexHull(np.asarray(fp@basis,np.float32)).reshape(-1,2)
points=_project(depth,f['K']);base=points@f['T_base_camera'][:3,:3].T+f['T_base_camera'][:3,3]
xy=base@basis
y,x=np.indices(depth.shape);local=(x>=220)&(x<427)&(y>=300)&(y<464)
support=local&np.isfinite(depth)&(depth>0)&(abs(heights)<=.002)&~instances[1]
top=cores[0]
def quant(v):return np.percentile(v,[0,10,50,90,100],axis=0).tolist() if len(v) else []
def summary(mask):
    valid=mask&np.isfinite(depth)&(depth>0)
    components,_,stats,_=cv2.connectedComponentsWithStats(mask.astype(np.uint8),8)
    sd=np.array([cv2.pointPolygonTest(hull,tuple(map(float,p)),True) for p in xy[valid]])
    return dict(pixels=int(mask.sum()),valid_depth=int(valid.sum()),
        height_to_frozen_plane_mm_quantiles=quant(heights[valid]*1e3),
        signed_distance_inside_frozen_footprint_mm_quantiles=quant(sd*1e3),
        lab_quantiles=quant(lab[mask]),normalized_bgr_chroma_quantiles=quant(chroma[mask]),
        lit_pixels=int((mask&(rgb.astype(float).sum(-1)>60)).sum()),
        strongly_chromatic_pixels=int((mask&(np.ptp(chroma,axis=-1)>.5)).sum()),
        connected_component_sizes=sorted(stats[1:,cv2.CC_STAT_AREA].tolist(),reverse=True),
        outside_frozen_domain=int((mask&~domain).sum()))
def histogram_distance(a,b):
    ha=cv2.calcHist([rgb],[0,1,2],a.astype('uint8'),[8,8,8],[0,256]*3)
    hb=cv2.calcHist([rgb],[0,1,2],b.astype('uint8'),[8,8,8],[0,256]*3)
    ha/=ha.sum();hb/=hb.sum()
    return float(cv2.compareHist(ha,hb,cv2.HISTCMP_BHATTACHARYYA))
result=dict(input_manifest_sha256=hashlib.sha256((BASE/'replay_inputs/manifest.json').read_bytes()).hexdigest(),
    masks_sha256=hashlib.sha256((src/'masks.npz').read_bytes()).hexdigest(),
    actual_source_stamp_ns=int(f['source_stamp_ns']),
    regions={k:summary(v) for k,v in [('unowned_side',lost),('retained_same_core',retained),('other_top_core',top),('current_local_support',support)]},
    diagnostic_bgr_histogram_distance=dict(unowned_to_support=histogram_distance(lost,support),unowned_to_top=histogram_distance(lost,top)),
    facts=['All unowned points are real same-frame measured depth. Height is computed against the original frozen support plane, whose independently audited current discrepancy is about 1.3 mm in this frame.',
           'Their native raw component already includes the upper face. Component membership, local smoothness, global appearance match, and footprint proximity are evidence, not independent proof of a unique physical object.',
           'The initial measured footprint is not a complete object-volume model; points just outside it cannot be rejected as a different object or accepted as new surface solely by that fact.'],
    required_negative_cases=[
        'Two touching same-colour objects with smooth depth and similar whole-image appearance: do not merge merely because depth is connected or a union matches the old box extent.',
        'Foreground-looking stereo ramp attached to support or shadow: valid positive depth and local planarity alone must not promote it to trusted identity.',
        'Nonmatching third native candidate adjacent to the accepted face: account for it independently; do not discard because it falls outside the frozen domain.',
        'Only one seed expansion owns the other seed: larger-mask selection or hard seeding all competitor cores cannot substitute for independent evidence.',
        'A narrow bridge in invalid depth or unsupported RGB pixels: no dilation, interpolation, convex hull, historical fill or component joining can create measured association.',
        'Overlap/ownership 84.9 percent, a boundary source/epoch change, or stale support: preserve current rejection and freshness rules.'],
    limitations=['No new classifier, threshold, synthetic positive geometry, or merge is introduced.',
                 'No result here proves physical object identity in general; provenance must remain exact same-frame and bounded by existing geometry and original appearance checks.'],
    hardware_access=False)
p=out/'side_evidence_and_negative_cases.json';p.write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps(dict(path=str(p),sha256=hashlib.sha256(p.read_bytes()).hexdigest(),regions=result['regions'],colour=result['diagnostic_bgr_histogram_distance'])))
