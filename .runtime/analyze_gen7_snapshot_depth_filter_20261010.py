"""Sealed GEN7 pixel-level snapshot filter attribution, no node/action calls."""
from pathlib import Path
import hashlib,json
import cv2
import numpy as np

_reference_script=Path(__file__).with_name('reconstruct_gen7_contact_surface_20261010.py')
_prefix=_reference_script.read_text().split('for group in groups:\n')[0]
exec(compile(_prefix,str(_reference_script),'exec'))
OUT=NEW/'gen7_handoff_analysis/contact_analysis/depth_filter_pixels'
OUT.mkdir(exist_ok=True,parents=True)

def spatial_statistics(depth,mask,T,normal,offset):
    v,u=np.where(mask & np.isfinite(depth) & (depth>0))
    z=depth[v,u].astype(float)*scale
    p=np.column_stack(((u-K.cx)*z/K.fx,(v-K.cy)*z/K.fy,z))
    p=p@T[:3,:3].T+T[:3,3]
    h=p@normal+offset
    bins=np.array([-100,0,4,8,12,16,18,20,22,24,28,100],float)/1000
    return dict(pixel_count=len(z),camera_depth_quantiles_m=np.quantile(z,[0,.01,.05,.5,.95,.99,1]).tolist() if len(z) else [],
        support_height_quantiles_m=np.quantile(h,[0,.01,.05,.5,.95,.99,1]).tolist() if len(z) else [],
        height_bin_edges_m=bins.tolist(),height_counts=np.histogram(h,bins)[0].tolist(),
        span_xyz_base_m=np.ptp(p,axis=0).tolist() if len(p) else [],
        at_4_to_16mm=int(np.count_nonzero((h>0.004)&(h<0.016))),
        at_4_to_18mm=int(np.count_nonzero((h>0.004)&(h<0.018))))

rows=[];frame_rows=[];artifacts=[]
for index,group in enumerate(groups):
    samples=[]
    for sn in group:
        f=frames[sn];o=f[topics[3]][0];box=tuple(getattr(o,k) for k in ('bbox_x','bbox_y','bbox_width','bbox_height'))
        samples.append(RgbdSample(bridge.imgmsg_to_cv2(f[topics[0]][0],'bgr8'),bridge.imgmsg_to_cv2(f[topics[1]][0],'passthrough'),bridge.imgmsg_to_cv2(f[topics[2]][0],'mono8'),box,o,sn/1e9,'camera_link',min(joints,key=lambda v:abs(v[0]-sn))[1],sn,identity.epoch,identity))
    snap=fuse_stable_samples(samples,**fusion);assert snap.ok,snap.failure_reason
    q=buf.lookup_transform_core('base_link','camera_link',tm(group[-1])).transform
    T=quaternion_matrix([q.rotation.x,q.rotation.y,q.rotation.z,q.rotation.w]);T[:3,3]=[q.translation.x,q.translation.y,q.translation.z];T=T@opt
    est=estimate_object_geometry(snap.depth_raw,snap.target_depth_raw,snap.object_mask,snap.bbox,K,scale,T,snap.source_mode,**geom);assert est.ok,est.failure_reason
    normal,offset=est.support_normal_base,est.support_offset_m
    clipped=[];masks=[]
    for s in samples:
        d=s.depth_raw.copy();good=np.isfinite(d)&(d.astype(float)*scale>=fusion['depth_min_m'])&(d.astype(float)*scale<=fusion['depth_max_m']);d[~good]=0;clipped.append(d);masks.append(s.object_mask>0)
        frame_rows.append(dict(source_ns=s.stamp_ns if hasattr(s,'stamp_ns') else s.source_stamp_ns,
            original_mask=spatial_statistics(d,s.object_mask>0,T,normal,offset)))
    depth=np.rint(np.median(np.stack(clipped),axis=0)).astype(clipped[0].dtype)
    assert np.array_equal(depth,snap.depth_raw)
    mask_vote=(np.count_nonzero(np.stack(masks),axis=0)>=int(np.ceil(len(masks)/2))).astype(np.uint8)*255
    erosion=int(fusion['erosion_px']);eroded=cv2.erode(mask_vote,np.ones((2*erosion+1,2*erosion+1),np.uint8)) if erosion else mask_vote
    assert np.array_equal(eroded,snap.object_mask)
    valid=(eroded>0)&(depth>0);values=depth[valid].astype(float);median=float(np.median(values));mad=float(np.median(np.abs(values-median)))
    threshold=max(fusion['mad_scale']*mad*scale,fusion['mad_absolute_floor_m'])
    keep=valid&(np.abs(depth.astype(float)-median)*scale<=threshold);removed=valid&~keep
    assert np.array_equal(snap.target_depth_raw[valid],np.where(keep,depth,0)[valid])
    no_global_mad=np.where(valid,depth,0).astype(depth.dtype)
    diagnostic=estimate_object_geometry(snap.depth_raw,no_global_mad,snap.object_mask,snap.bbox,K,scale,T,snap.source_mode,**geom)
    row=dict(snapshot_index=index,source_stamps_ns=group,median_depth_m=median*scale,mad_m=mad*scale,threshold_m=threshold,
        mask_before_erosion=spatial_statistics(depth,mask_vote>0,T,normal,offset),
        erosion_removed=spatial_statistics(depth,(mask_vote>0)&(eroded==0),T,normal,offset),
        before_global_mad=spatial_statistics(depth,valid,T,normal,offset),
        global_mad_removed=spatial_statistics(depth,removed,T,normal,offset),
        after_global_mad=spatial_statistics(depth,keep,T,normal,offset),
        final_hole_fill_additions=int(np.count_nonzero((snap.target_depth_raw>0)&~keep)),
        original_estimate=dict(ok=est.ok,point_count=len(est.object_points_base),height_span_m=float(np.ptp(est.object_points_base@normal)),size_xyz_m=est.size_xyz_m.tolist()),
        before_global_mad_diagnostic_estimate=dict(ok=diagnostic.ok,failure_reason=diagnostic.failure_reason,
            point_count=len(diagnostic.object_points_base),height_span_m=float(np.ptp(diagnostic.object_points_base@normal)) if len(diagnostic.object_points_base) else None,
            size_xyz_m=diagnostic.size_xyz_m.tolist()))
    rows.append(row)
    ap=OUT/('snapshot_%d_pixels.npz'%index)
    np.savez_compressed(ap,source_stamps_ns=np.array(group,np.int64),color_bgr=samples[-1].color_bgr,depth_raw=depth,mask_before_erosion=mask_vote,mask_eroded=eroded,mad_removed=removed,mad_kept=keep,final_target_depth_raw=snap.target_depth_raw,T_base_optical=T,support_normal_base=normal,support_offset_m=offset,intrinsics=np.array([K.fx,K.fy,K.cx,K.cy]),depth_scale=scale)
    artifacts.append(dict(path=str(ap),sha256=sha(ap)))

report=dict(schema=1,record=str(REC),audit_sha256=sha(P),source_reconstruction_script_sha256=sha(_reference_script),script_sha256=sha(__file__),
    producer='original recorded /perception/object_mask',fusion_settings=fusion,geometry_settings=geom,
    snapshots=rows,original_frames=frame_rows,artifacts=artifacts,
    interpretation='Pixel masks and original filters are reproduced exactly. The no-global-MAD estimate is an isolated diagnostic input, never an authorized grasp surface.',
    limitations=['The recorded mask is already the output of producer segmentation/RGB refinement; it does not reveal surfaces excluded before publication.',
        'Support-normal height is computed with the original fit per snapshot. Height strata alone do not prove that pixels are correctly associated physical side faces.',
        'Earlier masks/depth in a stable snapshot use the final exact camera transform for diagnostics, matching stationary fusion; no physical movement is inferred from source count.',
        'No contact threshold, running parameter, source node or executable plan was changed.'])
rp=OUT/'pixel_filter_attribution.json'
with rp.open('x') as f:json.dump(plain(report),f,ensure_ascii=False,indent=2);f.write('\n')
print(json.dumps(dict(path=str(rp),sha256=sha(rp),snapshots=[{k:r[k] for k in ('source_stamps_ns','mad_m','threshold_m','before_global_mad','global_mad_removed','original_estimate','before_global_mad_diagnostic_estimate')} for r in rows]),indent=2))
