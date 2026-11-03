"""Independent static-table RGB-D registration, with source-disjoint checks."""
from pathlib import Path
import hashlib
import json
import math
import sys
import cv2
import numpy as np

ROOT = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_restart_20261010T002347Z/gen13_handoff_analysis')
OUT = ROOT/'rgbd_station_transform'
INPUT = OUT
ROOTSIFT = '--rootsift' in sys.argv
RATIO = .80 if ROOTSIFT else .72
if ROOTSIFT:
    OUT = INPUT/'rootsift_comparison'
    OUT.mkdir(exist_ok=True)
PLANES = ROOT/'support_plane_replay'
assert Path('/media/zhuyupei/SanStand').is_mount()
assert not (OUT/'report.json').exists()
cv2.setNumThreads(1)
NS = [1791608931326233386,1791608977329783916,1791609019931774139,1791609020930994033,1791609021932393312]
LABELS = ['initial','first_observation','second_observation','second_holdout_1','second_holdout_2']
plane_report = json.loads((PLANES/'report.json').read_text())
planes = {row['source_stamp_ns']: row for row in plane_report['frames']}
polygons = [
    [[157,0],[525,0],[520,150],[518,261],[650,350],[650,488],[107,488]],
    [[0,0],[498,0],[497,248],[626,487],[0,487]],
    [[0,80],[489,0],[492,100],[474,261],[426,337],[330,487],[0,487]],
]
exclusions = [
    [[274,171],[411,174],[410,283],[272,283]],
    [[129,301],[296,299],[295,446],[130,446]],
    [[297,171],[391,171],[393,295],[296,296]],
]
threshold = .003
sift = cv2.SIFT_create(nfeatures=3500, contrastThreshold=.025, edgeThreshold=10)
frames = []
for i, ns in enumerate(NS):
    image = cv2.imread(str(INPUT/('rgb_%d.png'%ns)))
    with np.load(str(PLANES/('source_%d.npz'%ns))) as data:
        depth = data['depth_raw'].astype(float)*float(data['depth_scale'])
        K = data['intrinsics'].copy()
        T = data['T_base_camera'].copy()
    v,u = np.indices(depth.shape)
    x = np.stack((depth,-(u-K[2])*depth/K[0],-(v-K[3])*depth/K[1]),axis=-1)
    plane = np.array(planes[ns]['plane_camera'])
    residual = abs(x@plane[:3]+plane[3])
    mask = np.zeros(depth.shape,np.uint8)
    cv2.fillPoly(mask,[np.array(polygons[min(i,2)],np.int32)],255)
    cv2.fillPoly(mask,[np.array(exclusions[min(i,2)],np.int32)],0)
    mask[:4]=0;mask[-4:]=0;mask[:,:4]=0;mask[:,-4:]=0
    valid = (depth>.03)&(depth<2)&(residual<=.003)
    # No depth synthesis: each feature uses its own exact nearest pixel. The
    # neighborhood only rejects holes/edges, never replaces a depth sample.
    count = cv2.boxFilter(valid.astype(np.float32),-1,(3,3),normalize=False)
    smooth = (cv2.dilate(depth.astype(np.float32),np.ones((3,3),np.uint8))-
              cv2.erode(depth.astype(np.float32),np.ones((3,3),np.uint8))) <= .006
    mask[~(valid&(count>=7)&smooth)] = 0
    keypoints, descriptors = sift.detectAndCompute(cv2.cvtColor(image,cv2.COLOR_BGR2GRAY),mask)
    if ROOTSIFT and descriptors is not None:
        descriptors = np.sqrt(descriptors/(np.sum(descriptors,axis=1,keepdims=True)+1e-12)).astype(np.float32)
    cv2.imwrite(str(OUT/('mask_%d.png'%ns)),mask)
    frames.append(dict(ns=ns,label=LABELS[i],image=image,depth=depth,K=K,T=T,
        keypoints=keypoints,descriptors=descriptors,mask=mask))

def fit(a,b):
    ca,cb=a.mean(0),b.mean(0)
    u,sv,vt=np.linalg.svd((a-ca).T@(b-cb))
    if sv[1]<1e-10:raise ValueError('collinear samples')
    R=vt.T@u.T
    if np.linalg.det(R)<0:vt[-1]*=-1;R=vt.T@u.T
    T=np.eye(4);T[:3,:3]=R;T[:3,3]=cb-R@ca
    return T

def residuals(T,a,b):return np.linalg.norm(a@T[:3,:3].T+T[:3,3]-b,axis=1)
def metrics(a):
    return None if len(a)==0 else dict(count=len(a),median_mm=float(np.median(a)*1000),
        p90_mm=float(np.quantile(a,.9)*1000),rms_mm=float(np.sqrt(np.mean(a*a))*1000))
def spatial(a):
    if not len(a):return None
    centered=a-a.mean(0)
    return dict(pixel_bbox=[a.min(0).tolist(),a.max(0).tolist()],
        occupied_80px_cells=len(set(map(tuple,(a//80).astype(int)))),
        pixel_covariance_eigenvalues=np.linalg.eigvalsh(centered.T@centered/max(1,len(a)-1)).tolist())

def matches(f,g):
    bf=cv2.BFMatcher(cv2.NORM_L2)
    fw=bf.knnMatch(f['descriptors'],g['descriptors'],k=2)
    rev=bf.knnMatch(g['descriptors'],f['descriptors'],k=2)
    backwards={a.queryIdx:a.trainIdx for a,b in rev if a.distance < RATIO*b.distance}
    pairs=[a for a,b in fw if a.distance < RATIO*b.distance and backwards.get(a.trainIdx)==a.queryIdx]
    # SIFT may attach multiple orientations to one physical keypoint. Keep
    # one match per 3-pixel location in both images to avoid fake support.
    seen_f,seen_g=set(),set();result=[]
    for m in sorted(pairs,key=lambda m:m.distance):
        a=np.array(f['keypoints'][m.queryIdx].pt);b=np.array(g['keypoints'][m.trainIdx].pt)
        ka=tuple(np.rint(a/3).astype(int));kb=tuple(np.rint(b/3).astype(int))
        if ka in seen_f or kb in seen_g:continue
        seen_f.add(ka);seen_g.add(kb);result.append(m)
    return result

reports=[]
for ia,ib in [(0,1),(0,2),(1,2),(0,3),(0,4),(1,3),(1,4),(2,3),(2,4)]:
    f,g=frames[ia],frames[ib]
    ms=matches(f,g)
    pix_a=np.array([f['keypoints'][m.queryIdx].pt for m in ms]);pix_b=np.array([g['keypoints'][m.trainIdx].pt for m in ms])
    def points(frame,pixels):
        uv=np.rint(pixels).astype(int);z=frame['depth'][uv[:,1],uv[:,0]];fx,fy,cx,cy=frame['K']
        return np.column_stack((z,-(pixels[:,0]-cx)*z/fx,-(pixels[:,1]-cy)*z/fy))
    row=dict(source=f['label'],target=g['label'],source_stamp_ns=f['ns'],target_stamp_ns=g['ns'],
        mutual_ratio_matches=len(ms),ratio_test=RATIO,threshold_m=threshold,
        fit_used_expected_tf=False,scale_fitted=False)
    if len(ms)<20:
        row.update(qualified=False,reason='insufficient mutual descriptor matches');reports.append(row);continue
    a,b=points(f,pix_a),points(g,pix_b)
    # Fixed index split precedes all geometric fitting. Holdout matches do not
    # select a model, threshold, inlier set or fitted pose.
    rng=np.random.default_rng(71013+ia*100+ib)
    shuffled=rng.permutation(len(ms));hold=shuffled[::3];train=np.setdiff1d(np.arange(len(ms)),hold)
    best=None;best_score=(-1,float('-inf'))
    for _ in range(4000):
        index=rng.choice(train,3,replace=False)
        try:T=fit(a[index],b[index])
        except (ValueError,np.linalg.LinAlgError):continue
        e=residuals(T,a[train],b[train]);inlier=e<=threshold
        score=(int(inlier.sum()),-float(np.median(e[inlier])) if inlier.any() else -1e9)
        if score>best_score:best_score=score;best=T
    if best is None:
        row.update(qualified=False,reason='no noncollinear RANSAC model');reports.append(row);continue
    for _ in range(3):
        inliers=train[residuals(best,a[train],b[train])<=threshold]
        if len(inliers)<3:break
        best=fit(a[inliers],b[inliers])
    err=residuals(best,a,b)
    training_inliers=train[err[train]<=threshold];holdout_inliers=hold[err[hold]<=threshold]
    expected=np.linalg.inv(g['T'])@f['T'];expected_err=residuals(expected,a,b)
    delta=best@np.linalg.inv(expected)
    rotation_deg=float(np.degrees(np.arccos(np.clip((np.trace(delta[:3,:3])-1)/2,-1,1))))
    accepted=np.r_[training_inliers,holdout_inliers]
    sp_a,sp_b=spatial(pix_a[accepted]),spatial(pix_b[accepted])
    qualified=(len(training_inliers)>=20 and len(holdout_inliers)>=10
        and len(holdout_inliers)/max(1,len(hold))>=.5
        and sp_a['occupied_80px_cells']>=4 and sp_b['occupied_80px_cells']>=4
        and min(sp_a['pixel_covariance_eigenvalues'])>=150
        and min(sp_b['pixel_covariance_eigenvalues'])>=150)
    row.update(qualified=qualified,reason='bounded static-texture registration plus reserved-match check' if qualified else 'insufficient independently reserved or spatially spread support',
        train_match_count=len(train),training_inlier_count=len(training_inliers),holdout_match_count=len(hold),
        holdout_inlier_count=len(holdout_inliers),training_fit_residual=metrics(err[training_inliers]),
        reserved_inlier_residual=metrics(err[holdout_inliers]),reserved_all_residual=metrics(err[hold]),
        expected_tf_on_same_training_inliers=metrics(expected_err[training_inliers]),
        expected_tf_on_same_reserved_inliers=metrics(expected_err[holdout_inliers]),
        source_pixel_coverage=sp_a,target_pixel_coverage=sp_b,
        fitted_target_from_source=best.tolist(),expected_target_from_source=expected.tolist(),
        fitted_vs_expected_delta=delta.tolist(),fitted_vs_expected_origin_translation_mm=float(np.linalg.norm(delta[:3,3])*1000),
        fitted_vs_expected_rotation_deg=rotation_deg,
        source_centered_3d_singular_values_m=np.linalg.svd(a[training_inliers]-a[training_inliers].mean(0),compute_uv=False).tolist(),
        correspondence_file='%s_to_%s_correspondences.json'%(f['label'],g['label']))
    correspondence=dict(source_pixels=pix_a.tolist(),target_pixels=pix_b.tolist(),
        source_camera_points=a.tolist(),target_camera_points=b.tolist(),
        train_indices=train.tolist(),reserved_indices=hold.tolist(),
        training_inlier_indices=training_inliers.tolist(),reserved_inlier_indices=holdout_inliers.tolist(),
        fitted_residual_m=err.tolist(),expected_tf_residual_m=expected_err.tolist())
    (OUT/row['correspondence_file']).write_text(json.dumps(correspondence,indent=2)+'\n')
    drawn=cv2.drawMatches(f['image'],f['keypoints'],g['image'],g['keypoints'],
        [ms[k] for k in accepted],None,flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)
    cv2.imwrite(str(OUT/('%s_to_%s_inlier_matches.png'%(f['label'],g['label']))),drawn)
    reports.append(row)
    print(json.dumps({k:row[k] for k in ['source','target','qualified','mutual_ratio_matches','training_inlier_count','holdout_inlier_count','reserved_inlier_residual','expected_tf_on_same_reserved_inliers','fitted_vs_expected_origin_translation_mm','fitted_vs_expected_rotation_deg']}),flush=True)

report=dict(record='20261010T050849.339487Z_fa4a10a8358a4fa2b80214f1f626b5c7',
    frames=[dict(label=f['label'],source_stamp_ns=f['ns'],feature_count=len(f['keypoints']),
        valid_static_roi_pixels=int((f['mask']>0).sum()),npz_path=str(PLANES/('source_%d.npz'%f['ns'])),
        rgb_path=str(INPUT/('rgb_%d.png'%f['ns']))) for f in frames],
    roi_polygons=polygons,excluded_target_polygons=exclusions,
    comparisons=reports,method=dict(detector='RootSIFT' if ROOTSIFT else 'SIFT',descriptor_ratio=RATIO,mutual=True,
        ransac_trials=4000,rigid_fit='unit scale Kabsch',inlier_threshold_m=threshold,
        fixed_reserved_fraction='one third before any fitting',depth_interpolation=False,
        missing_depth_filled=False,expected_tf_used_for_fitting=False,
        plane_usage='station-local observed support plane only masks non-table points; coordinates remain original published depth',
        source_units='camera_link convention X forward, Y left, Z up'),
    qualifications=['Twenty training inliers, ten reserved inliers, at least 50% reserved match consensus, four 80-pixel cells in each image, and minimum 2D covariance eigenvalue 150 pixel^2.',
        'Repeated text risks period-offset matches; mutual ratio, rigid 3D consensus, manual ROI exclusion, spatial coverage and saved match overlays support review but cannot make texture nonrepeating.'],
    limitations=['Depth is the exact recorded published depth, which already includes upstream camera processing; this is not native unfiltered depth.',
        'The observed tablecloth is assumed static between these short stationary captures. No independent external fixture proves the cloth did not move.',
        'Reserved feature matches are excluded from rigid fitting; second-station later frames provide repeatability, not a new robot pose or external metrology.',
        'Even a well-supported discrepancy establishes inconsistency among RGB-D observations and recorded transforms, not whether depth scale/bias, hand-eye, robot kinematics or compliance is the unique cause.',
        'Near-planar support geometry and shared camera systematics prevent certification of a new calibration. No correction is estimated for deployment.'],
    production_edited=False,calibration_changed=False,live_ros_access=False,physical_actions=False)
path=OUT/'report.json';path.write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
print(json.dumps({'report':str(path),'sha256':hashlib.sha256(path.read_bytes()).hexdigest()}))
