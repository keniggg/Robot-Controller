from pathlib import Path
import hashlib,json
import cv2,numpy as np
s=Path('.runtime/gen22_orb_identity_diagnostic_20261010.py').read_text();exec(compile(s[:s.index('results=[];plots=[]')],'<fixed_orb_setup>','exec'))
mode='requested_descriptor_support';support_stats=[]
def detect(image,mask):
 binary=(mask>0).astype(np.uint8);dt=cv2.distanceTransform(binary,cv2.DIST_L2,cv2.DIST_MASK_PRECISE)
 gray=cv2.cvtColor(image,cv2.COLOR_BGR2GRAY);keypoints=orb.detect(gray,binary*255);kept=[]
 for k in keypoints:
  x,y=k.pt;xx=int(np.floor(x));yy=int(np.floor(y));patch=dt[max(0,yy):min(dt.shape[0],yy+2),max(0,xx):min(dt.shape[1],xx+2)]
  radius=k.size*np.sqrt(.5)+1.
  if mode=='descriptor_plus_blur_margin':radius+=3.*(k.size/31.)
  if patch.size and float(np.min(patch))>=radius:kept.append(k)
 support_stats.append(dict(mode=mode,raw_keypoints=len(keypoints),retained=len(kept)))
 return orb.compute(gray,kept)
def controls(image,mask):
 x,y,w,h=cv2.boundingRect(mask);lm=mask[y:y+h,x:x+w]>0;med=np.median(image[mask>0],axis=0).astype(np.uint8);flat=image.copy();flat[mask>0]=med
 cloth=np.zeros(mask.shape,np.uint8);cloth[100:250,100:250]=255
 fullflat=np.empty_like(image);fullflat[:]=med
 cx,cy=x+w//2,y+h//2;patch=image[cy-16:cy+16,cx-16:cx+16];single=flat.copy();single[cy-16:cy+16,cx-16:cx+16]=patch
 repeated=flat.copy();tiled=np.tile(patch,(int(np.ceil(h/32)),int(np.ceil(w/32)),1))[:h,:w];repeated[y:y+h,x:x+w][lm]=tiled[lm]
 other=flat.copy();paper=np.empty((h,w,3),np.uint8);paper[:]=med;ink=np.percentile(image[mask>0],15,axis=0).astype(np.uint8)
 for j,text in enumerate(['OTHER','ABCD','2026']):cv2.putText(paper,text,(3,25+j*32),cv2.FONT_HERSHEY_SIMPLEX,.65,tuple(map(int,ink)),2,cv2.LINE_AA)
 other[y:y+h,x:x+w][lm]=paper[lm]
 hsv=cv2.cvtColor(image,cv2.COLOR_BGR2HSV);hsv[:,:,0]=(hsv[:,:,0].astype(int)+60)%180;hue=image.copy();hue[mask>0]=cv2.cvtColor(hsv,cv2.COLOR_HSV2BGR)[mask>0]
 crop=image[y:y+h,x:x+w];sh=(int(np.ceil(h/16))*16,int(np.ceil(w/16))*16);padded=cv2.copyMakeBorder(crop,0,sh[0]-h,0,sh[1]-w,cv2.BORDER_REFLECT_101);tiles=[padded[yy:yy+16,xx:xx+16].copy() for yy in range(0,sh[0],16) for xx in range(0,sh[1],16)];perm=np.random.RandomState(1729).permutation(len(tiles));shuffled=padded.copy();j=0
 for yy in range(0,sh[0],16):
  for xx in range(0,sh[1],16):shuffled[yy:yy+16,xx:xx+16]=tiles[perm[j]];j+=1
 shuffle=image.copy();shuffle[y:y+h,x:x+w]=shuffled[:h,:w]
 return [('recorded_object',image,mask),('recorded_wrong_tablecloth_roi',image,cloth),('synthetic_texture_tile_shuffle',shuffle,mask),('synthetic_same_colour_textureless',fullflat,mask),('synthetic_single_real_patch_32px',single,mask),('synthetic_repeated_real_patch_32px',repeated,mask),('synthetic_same_palette_other_text',other,mask),('synthetic_real_target_hue_plus60',hue,mask)]
results=[]
for mode in ('requested_descriptor_support','descriptor_plus_blur_margin'):
 kr,dr=detect(last['color'],last['recorded_refined_mask'])
 for item in rep['results']:
  ns=item['source_ns'];f=dict(np.load(idx[ns]['path']));mask=cv2.imread(str(B/('crop_ab_production_'+str(ns)+'_mask.png')),0)
  for variant,image,region in controls(f['color'],mask):
   row,viz=compare(image,region,ns,variant);row['support_policy']=mode;results.append(row)
report=dict(record=m['record'],reference_source_ns=lr['source_stamp_ns'],original_ORB_report_sha256=hashlib.sha256((B/'orb_texture_correspondence_diagnostic.json').read_bytes()).hexdigest(),definition=dict(requested_descriptor_support='minimum of floor/ceil distance-transform samples >= keypoint.size*sqrt(.5)+1 pixel',descriptor_plus_blur_margin='requested support radius plus 3*(keypoint.size/31) pixels as additional conservative local Gaussian preprocessing margin'),matching_parameters='Same ORB1000, pyramid1.2x8, patch31, FAST20, mutual KNN ratio.75, RANSAC3px/2000/.995/seed1729; no retuning.',results=results,support_filter_counts=support_stats,limitations=['No mask-exterior zeroing or artificial boundary features are used.','Distance test covers descriptor footprint and optional local blur margin; it is not a formal bound on every internal OpenCV pyramid dependency.','Negative controls remain synthetic except the real tablecloth ROI.','No production changes or identity admission decision.'])
out=B/'orb_complete_descriptor_support_diagnostic.json';out.write_text(json.dumps(report,indent=2)+'\n');print('REPORT',str(out),hashlib.sha256(out.read_bytes()).hexdigest())
for policy in ('requested_descriptor_support','descriptor_plus_blur_margin'):
 print(policy)
 for variant in sorted(set(r['variant'] for r in results)):
  rr=[r for r in results if r['support_policy']==policy and r['variant']==variant];good=[r for r in rr if r.get('reference_coverage')];print(variant,'refkp',rr[0]['reference_keypoints'],'matches',[min(r['mutual_ratio_matches'] for r in rr),max(r['mutual_ratio_matches'] for r in rr)],'inliers',[min(r['ransac_inliers'] for r in rr),max(r['ransac_inliers'] for r in rr)],'n12',sum(r['ransac_inliers']>=12 for r in rr),'quadrants',[min(r['reference_coverage']['quadrants'] for r in good),max(r['reference_coverage']['quadrants'] for r in good)] if good else [])
