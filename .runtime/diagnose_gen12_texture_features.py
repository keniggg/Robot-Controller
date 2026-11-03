#!/usr/bin/env python3
"""Offline ORB mutual correspondences only; no segmentation or identity authority."""
from pathlib import Path
import hashlib,json,time
import cv2
import numpy as np

ROOT=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_ros_20261009T003407Z/gen12_arrival_analysis')
OUT=ROOT/'feature_diagnostic'
REF=ROOT/'reference_images/reference_1791520119101696014_rgb.png'
MASK=ROOT/'reference_images/reference_1791520119101696014_mask.png'

def coverage(points,mask):
    h,w=mask.shape;pts=np.asarray(points,dtype=np.float32).reshape(-1,2)
    pixels=np.rint(pts).astype(int);good=(pixels[:,0]>=0)&(pixels[:,0]<w)&(pixels[:,1]>=0)&(pixels[:,1]<h)
    inside=np.zeros(len(pts),bool);inside[good]=mask[pixels[good,1],pixels[good,0]]>0
    ys,xs=np.nonzero(mask);cx,cy=np.mean(xs),np.mean(ys)
    quad=[int(np.sum((pts[:,0]<cx)&(pts[:,1]<cy))),int(np.sum((pts[:,0]>=cx)&(pts[:,1]<cy))),
          int(np.sum((pts[:,0]<cx)&(pts[:,1]>=cy))),int(np.sum((pts[:,0]>=cx)&(pts[:,1]>=cy)))]
    hullmask=np.zeros(mask.shape,np.uint8)
    if len(pts)>=3:cv2.fillConvexPoly(hullmask,np.rint(cv2.convexHull(pts)).astype(np.int32),255)
    return dict(points=len(pts),points_inside_mask=int(inside.sum()),mask_pixels=int(np.count_nonzero(mask)),
                convex_hull_intersection_mask_fraction=float(np.count_nonzero((hullmask>0)&(mask>0))/len(xs)),
                quadrants=quad,occupied_quadrants=sum(v>0 for v in quad))

def main():
    cv2.setNumThreads(1);cv2.setRNGSeed(7);OUT.mkdir(parents=True,exist_ok=True)
    ref=cv2.imread(str(REF));mask=cv2.imread(str(MASK),0)
    detector=cv2.ORB_create(nfeatures=1500)
    kr,dr=detector.detectAndCompute(cv2.cvtColor(ref,cv2.COLOR_BGR2GRAY),mask)
    bf=cv2.BFMatcher(cv2.NORM_HAMMING)
    def ratio(a,b):
        if a is None or b is None or len(b)<2:return {}
        return {pair[0].queryIdx:pair[0] for pair in bf.knnMatch(a,b,k=2)
                if len(pair)==2 and pair[0].distance<.75*pair[1].distance}
    report=dict(execution_authority=False,contact_or_motion_authorized=False,ros_calls=False,
        purpose='Independent texture feasibility diagnostic; original masks and sealed RGB only.',
        reference_rgb=str(REF),reference_mask=str(MASK),reference_source_stamp_ns=1791520119101696014,
        opencv_version=cv2.__version__,algorithm='ORB',sift_available=hasattr(cv2,'SIFT_create'),
        parameters=dict(nfeatures=1500,other_ORB_parameters='OpenCV defaults',mutual_ratio=.75,
                        homography_ransac_threshold_px=3.,ransac_max_iterations=2000,ransac_confidence=.995,seed=7),
        reference_keypoints=len(kr),rows=[],limitations=[
            'Reference detector centres are restricted to the original measured mask; descriptor neighborhoods may extend beyond mask edges.',
            'Arrival ROI is the saved 48-pixel counterfactual mask bounding box plus 16 pixels for this diagnostic only; the separate mask case restricts detector centres to the saved counterfactual mask.',
            'Homography inliers can reflect repeated print patterns. Residual and spatial coverage are diagnostic evidence, not calibrated identity probability or execution authority.',
            'No production thresholds, masks, geometry, or detector behavior are modified. A counterfactual mask is not the online published mask.',
            'SIFT is unavailable in the production OpenCV build; no package installation or alternative runtime is used.'])
    sources=[1791520145310290098,1791520145541970252,1791520146945273876,1791520147210473537]
    for source in sources:
        root=ROOT/'crop_counterfactual_48';rgb_path=root/f'{source}_rgb.png';mask_path=root/f'{source}_refined_mask.png'
        rgb=cv2.imread(str(rgb_path));current=cv2.imread(str(mask_path),0)
        actual=cv2.imread(str(ROOT/'identity_replay'/f'{source}_refined_mask.png'),0)
        assert rgb is not None and current is not None and actual is not None
        y,x=np.nonzero(current);x0,x1=max(0,x.min()-16),min(rgb.shape[1],x.max()+17)
        y0,y1=max(0,y.min()-16),min(rgb.shape[0],y.max()+17)
        roi=np.zeros(current.shape,np.uint8);roi[y0:y1,x0:x1]=255
        for name,detmask in (('bounded_rgb_roi',roi),('counterfactual_48_mask',current)):
            started=time.perf_counter();kc,dc=detector.detectAndCompute(cv2.cvtColor(rgb,cv2.COLOR_BGR2GRAY),detmask)
            forward,reverse=ratio(dr,dc),ratio(dc,dr)
            matches=sorted([m for i,m in forward.items() if m.trainIdx in reverse and reverse[m.trainIdx].trainIdx==i],key=lambda m:m.distance)
            row=dict(source_stamp_ns=source,search=name,reference_keypoints=len(kr),current_keypoints=len(kc),
                ratio_forward=len(forward),ratio_reverse=len(reverse),mutual_matches=len(matches),roi_xyxy=[int(x0),int(y0),int(x1),int(y1)],
                rgb_sha256=hashlib.sha256(rgb_path.read_bytes()).hexdigest(),counterfactual_mask_sha256=hashlib.sha256(mask_path.read_bytes()).hexdigest())
            if len(matches)>=4:
                first=np.float32([kr[m.queryIdx].pt for m in matches]);second=np.float32([kc[m.trainIdx].pt for m in matches])
                cv2.setRNGSeed(7)
                H,inliers=cv2.findHomography(first,second,cv2.RANSAC,3.,maxIters=2000,confidence=.995)
                if H is not None and np.isfinite(H).all():
                    inside=inliers.ravel().astype(bool);p0,p1=first[inside],second[inside]
                    pred=cv2.perspectiveTransform(p0[:,None,:],H)[:,0,:]
                    inv=cv2.perspectiveTransform(p1[:,None,:],np.linalg.inv(H))[:,0,:]
                    error=np.linalg.norm(pred-p1,axis=1);backerror=np.linalg.norm(inv-p0,axis=1)
                    row.update(homography=H.tolist(),inliers=int(inside.sum()),inlier_fraction=float(inside.mean()),
                        reprojection_error_px=dict(median=float(np.median(error)),max=float(np.max(error)),rmse=float(np.sqrt(np.mean(error**2)))),
                        inverse_reprojection_error_px=dict(median=float(np.median(backerror)),max=float(np.max(backerror))),
                        reference_coverage=coverage(p0,mask),current_counterfactual_coverage=coverage(p1,current),
                        current_original_refined_coverage=coverage(p1,actual),
                        fixed_correspondences=[dict(reference_px=first[i].tolist(),current_px=second[i].tolist(),hamming_distance=float(m.distance),ransac_inlier=bool(inside[i])) for i,m in enumerate(matches)])
                    drawn=cv2.drawMatches(ref,kr,rgb,kc,matches,None,matchesMask=inside.astype(int).tolist(),flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)
                    image_name=f'{source}_{name}_inliers.png';assert cv2.imwrite(str(OUT/image_name),drawn);row['inlier_image']=image_name
                else:row['reason']='homography_not_estimated'
            else:row['reason']='fewer_than_four_mutual_ratio_matches'
            row['wall_sec']=time.perf_counter()-started;report['rows'].append(row)
            print(json.dumps({k:row[k] for k in ('source_stamp_ns','search','mutual_matches','inliers','reprojection_error_px','reference_coverage','current_counterfactual_coverage','reason') if k in row}),flush=True)
    path=OUT/'result.json'
    with path.open('x') as f:json.dump(report,f,indent=2,allow_nan=False)
    print(str(path),flush=True)

if __name__=='__main__':main()
