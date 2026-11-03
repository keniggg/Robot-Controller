from pathlib import Path
import hashlib,json
import cv2,numpy as np
# Load definitions and fixed feature setup only; do not rerun previous suite.
source=Path('.runtime/gen22_orb_identity_diagnostic_20261010.py').read_text();exec(compile(source[:source.index('results=[];plots=[]')],'<fixed_orb_setup>','exec'))
old=json.loads((B/'orb_texture_correspondence_diagnostic.json').read_text());reference_mask=(last['recorded_refined_mask']>0).astype(np.uint8)
def integrity(H,mask):
 H=np.asarray(H,float);current=(mask>0).astype(np.uint8);height,width=current.shape
 forward=cv2.warpPerspective(reference_mask,H,(width,height),flags=cv2.INTER_NEAREST)>0
 inv=np.linalg.inv(H);back=cv2.warpPerspective(current,inv,(reference_mask.shape[1],reference_mask.shape[0]),flags=cv2.INTER_NEAREST)>0
 ref=reference_mask>0;cur=current>0
 x,y,w,h=cv2.boundingRect(reference_mask);corners=np.array([[x,y],[x+w,y],[x+w,y+h],[x,y+h]],float);hc=np.c_[corners,np.ones(4)]@H.T;quad=hc[:,:2]/hc[:,2,None]
 signed=lambda p:.5*float(np.sum(p[:,0]*np.roll(p[:,1],-1)-p[:,1]*np.roll(p[:,0],-1)))
 return dict(forward_mask_iou=float(np.count_nonzero(forward&cur)/max(1,np.count_nonzero(forward|cur))),backward_mask_iou=float(np.count_nonzero(back&ref)/max(1,np.count_nonzero(back|ref))),current_mask_retained_fraction=float(np.count_nonzero(forward&cur)/max(1,cur.sum())),reference_mask_retained_fraction=float(np.count_nonzero(back&ref)/max(1,ref.sum())),forward_mask_pixels=int(forward.sum()),current_mask_pixels=int(cur.sum()),projected_reference_box_xy=quad.tolist(),projected_box_area_ratio=signed(quad)/(w*h),same_orientation=signed(quad)>0,projected_quad_convex=bool(cv2.isContourConvex(quad.astype(np.float32))),projective_denominator_same_sign=bool(np.all(hc[:,2]>0) or np.all(hc[:,2]<0)),homography_determinant=float(np.linalg.det(H)),denominator_values=hc[:,2].tolist())
results=[];controls=[];plots=[]
for item in old['results']:
 if item['variant']!='recorded_object':continue
 ns=item['source_ns'];f=dict(np.load(idx[ns]['path']));image=f['color'];mask=cv2.imread(str(B/('crop_ab_production_'+str(ns)+'_mask.png')),0)
 results.append(dict(source_ns=ns,**integrity(item['homography'],mask)))
 x,y,w,h=cv2.boundingRect(mask);flat=image.copy();flat[mask>0]=np.median(image[mask>0],axis=0).astype(np.uint8)
 # A single real 32px patch, selected geometrically at the object center before
 # inspecting matches, is retained; the rest of the object's interior is flat.
 cx,cy=x+w//2,y+h//2;local=flat.copy();local[cy-16:cy+16,cx-16:cx+16]=image[cy-16:cy+16,cx-16:cx+16]
 # Repeated local true texture retains a palette but destroys whole-object layout.
 repeated=flat.copy();patch=image[cy-16:cy+16,cx-16:cx+16];tiles=np.tile(patch,(int(np.ceil(h/32)),int(np.ceil(w/32)),1))[:h,:w];localmask=mask[y:y+h,x:x+w]>0;crop=repeated[y:y+h,x:x+w];crop[localmask]=tiles[localmask]
 # Deliberately different synthetic print, with colors sampled from this frame.
 other=flat.copy();ink=np.percentile(image[mask>0],15,axis=0).astype(np.uint8);median=np.median(image[mask>0],axis=0).astype(np.uint8)
 paper=np.empty((h,w,3),np.uint8);paper[:]=median
 for j,text in enumerate(['OTHER','ABCD','2026']):cv2.putText(paper,text,(3,25+j*32),cv2.FONT_HERSHEY_SIMPLEX,.65,tuple(map(int,ink)),2,cv2.LINE_AA)
 crop=other[y:y+h,x:x+w];crop[localmask]=paper[localmask]
 # Actual current target image with a fixed +60 OpenCV hue shift; texture stays.
 hsv=cv2.cvtColor(image,cv2.COLOR_BGR2HSV);shifted=hsv.copy();shifted[:,:,0]=(shifted[:,:,0].astype(int)+60)%180;hue=image.copy();new=cv2.cvtColor(shifted,cv2.COLOR_HSV2BGR);hue[mask>0]=new[mask>0]
 for variant,img in [('synthetic_single_real_patch_32px',local),('synthetic_repeated_real_patch_32px',repeated),('synthetic_same_palette_other_text',other),('synthetic_real_target_hue_plus60',hue)]:
  row,viz=compare(img,mask,ns,variant)
  if row.get('homography') is not None:row['mask_integrity']=integrity(row['homography'],mask)
  controls.append(row)
  if ns==old['results'][0]['source_ns']:
   path=B/(variant+'_first_control.png');cv2.imwrite(str(path),img);plots.append(dict(variant=variant,path=str(path),sha256=hashlib.sha256(path.read_bytes()).hexdigest()))
report=dict(record=m['record'],reference_source_ns=lr['source_stamp_ns'],fixed_ORB_report=dict(path=str(B/'orb_texture_correspondence_diagnostic.json'),sha256=hashlib.sha256((B/'orb_texture_correspondence_diagnostic.json').read_bytes()).hexdigest()),actual_fixed_homography_mask_integrity=results,synthetic_negative_controls=controls,control_images=plots,parameters_unchanged=True,production_changed=False,limitations=['All extra controls are explicitly synthetic and do not demonstrate rejection of real look-alike objects.','Pixel mask warps use estimated planar H, not proof of full 3D object identity.','A hue-preserving-independent geometric match is expected to remain possible for hue-shifted texture; report any such case instead of calling texture alone sufficient.','Real current inputs remain adjacent recorded sources, not online assessed frames.'])
out=B/'orb_mask_integrity_and_extra_negatives.json';out.write_text(json.dumps(report,indent=2)+'\n')
print('REPORT',str(out),hashlib.sha256(out.read_bytes()).hexdigest())
for k in ['forward_mask_iou','backward_mask_iou','current_mask_retained_fraction','reference_mask_retained_fraction','projected_box_area_ratio']:print(k,min(r[k] for r in results),max(r[k] for r in results))
print('actual_all_nofold',all(r['same_orientation'] and r['projected_quad_convex'] and r['projective_denominator_same_sign'] for r in results))
for variant in sorted(set(r['variant'] for r in controls)):
 rr=[r for r in controls if r['variant']==variant];print(variant,'matches',[min(r['mutual_ratio_matches'] for r in rr),max(r['mutual_ratio_matches'] for r in rr)],'inliers',[min(r['ransac_inliers'] for r in rr),max(r['ransac_inliers'] for r in rr)],'withH',sum(r['homography']is not None for r in rr),'iou',[round(r['mask_integrity']['forward_mask_iou'],3) for r in rr if 'mask_integrity'in r])
