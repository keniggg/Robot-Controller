"""Per-source camera-depth evidence at published mask boundary, sealed data."""
from pathlib import Path
import json,hashlib
import numpy as np
import cv2
_reference_script=Path(__file__).with_name('reconstruct_gen7_contact_surface_20261010.py')
exec(compile(_reference_script.read_text().split('for group in groups:\n')[0],str(_reference_script),'exec'))
OUT=NEW/'gen7_handoff_analysis/contact_analysis/depth_filter_pixels'
rows=[]
for group_index,group in enumerate(groups):
    proof=np.load(OUT/('snapshot_%d_pixels.npz'%group_index))
    n=proof['support_normal_base'];offset=float(proof['support_offset_m'])
    for sn in group:
        f=frames[sn];depth=bridge.imgmsg_to_cv2(f[topics[1]][0],'passthrough');mask=bridge.imgmsg_to_cv2(f[topics[2]][0],'mono8')>0
        rgb=bridge.imgmsg_to_cv2(f[topics[0]][0],'bgr8')
        q=buf.lookup_transform_core('base_link','camera_link',tm(sn)).transform
        T=quaternion_matrix([q.rotation.x,q.rotation.y,q.rotation.z,q.rotation.w]);T[:3,3]=[q.translation.x,q.translation.y,q.translation.z];T=T@opt
        v,u=np.indices(mask.shape);z=depth.astype(float)*scale
        points=np.stack(((u-K.cx)*z/K.fx,(v-K.cy)*z/K.fy,z),axis=-1)@T[:3,:3].T+T[:3,3]
        h=points@n+offset;valid=np.isfinite(z)&(z>0)&(z>=fusion['depth_min_m'])&(z<=fusion['depth_max_m'])
        my,mx=np.where(mask);x0=max(0,int(mx.min())-30);x1=min(mask.shape[1],int(mx.max())+31);y0=max(0,int(my.min())-30);y1=min(mask.shape[0],int(my.max())+31)
        roi=np.zeros_like(mask);roi[y0:y1,x0:x1]=True
        def stats(area):
            a=area&valid
            return dict(total_pixels=int(area.sum()),valid_pixels=int(a.sum()),zero_depth_pixels=int(np.count_nonzero(area&(depth==0))),
                height_under_4mm=int(np.count_nonzero(a&(h<.004))),height_4_to_16mm=int(np.count_nonzero(a&(h>=.004)&(h<.016))),
                height_16_to_18mm=int(np.count_nonzero(a&(h>=.016)&(h<.018))),height_18_to_24mm=int(np.count_nonzero(a&(h>=.018)&(h<.024))),
                height_over_24mm=int(np.count_nonzero(a&(h>=.024))))
        rings={}
        for radius in (2,4,8,16):
            ring=(cv2.dilate(mask.astype(np.uint8),np.ones((radius*2+1,radius*2+1),np.uint8))>0)&~mask
            rings[str(radius)]=stats(ring)
        rows.append(dict(source_ns=sn,roi_xyxy=[x0,y0,x1,y1],mask=stats(mask),context_roi=stats(roi),outside_mask_in_roi=stats(roi&~mask),outside_mask_dilation_rings_px=rings,tf_source_exact=True,
            serialized_depth_sha256=hashlib.sha256(depth.tobytes()).hexdigest(),mask_array_sha256=hashlib.sha256(mask.tobytes()).hexdigest()))
report=dict(schema=1,script_sha256=sha(__file__),source_script_sha256=sha(_reference_script),record=str(REC),source_topics=topics,rows=rows,
 interpretation='Recorded aligned camera depth before snapshot temporal median, erosion or global MAD is evaluated separately at every actual source transform. Boundary rings contain unknown scene pixels, not certified target geometry.',
 limitations=['Input is the recorded /supervisor/camera/depth/image_raw stream after its camera processing chain, not guaranteed native unfiltered sensor depth.',
 'Support plane remains the original snapshot support estimate; RGB visibility of a narrow side face does not supply metric depth.',
 'No mask, camera, threshold, motion or source code changed.'])
out=OUT/'original_depth_boundary.json'
with out.open('x') as f:json.dump(report,f,ensure_ascii=False,indent=2);f.write('\n')
print(json.dumps(dict(path=str(out),sha256=sha(out),rows=rows),indent=2))
