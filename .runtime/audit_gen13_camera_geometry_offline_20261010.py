"""Independent array/SDK math audit of sealed GEN13 camera geometry. No device I/O."""
from pathlib import Path
import hashlib,json,math,sys
import numpy as np
import rosbag

WORK=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(WORK/'.runtime/realsense-2.56.5'))
import pyrealsense2 as rs

ROOT=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
NEW=ROOT/'ros_log/full_latest_restart_20261010T002347Z'
INPUT=NEW/'gen13_handoff_analysis/support_plane_replay'
OUT=NEW/'gen13_handoff_analysis/offline_camera_chain'
assert Path('/media/zhuyupei/SanStand').is_mount()
record=json.loads((INPUT/'report.json').read_text())
frames=record['frames'][:3]
metadata=[]
with rosbag.Bag(str(Path(record['record'])/'keyframes.bag')) as bag:
    topics=list(bag.get_type_and_topic_info().topics)
    for topic,msg,received in bag.read_messages(topics=['/grasp_record/camera_runtime_metadata']):
        camera=json.loads(msg.data)['parameters']
        profile=camera['runtime_profile']
        metadata.append(dict(received_ns=received.to_nsec(),top={k:camera.get(k) for k in ('width','height','fx','fy','cx','cy','model','coeffs','depth_scale','frame_id','intrinsics_source','align_depth_to_color','stream_width','stream_height')},
            profile={k:profile.get(k) for k in ('color_intrinsics','native_depth_intrinsics','depth_to_color_extrinsics','alignment','geometry_intrinsics','geometry_intrinsics_source','depth_scale_m_per_unit','depth_scale_source','rectification','geometry_activation_stamp_ns')}))

camera=metadata[0]['top'];profile=metadata[0]['profile'];source=profile['color_intrinsics'];output=profile['geometry_intrinsics'];contract=profile['rectification']
intrinsic=rs.intrinsics()
intrinsic.width,intrinsic.height=source['width'],source['height']
intrinsic.fx,intrinsic.fy=source['fx'],source['fy'];intrinsic.ppx,intrinsic.ppy=source['cx'],source['cy']
intrinsic.model=getattr(rs.distortion,source['model']);intrinsic.coeffs=source['coeffs']
uu,vv=np.meshgrid(np.arange(source['width']),np.arange(source['height']))
pixels=np.column_stack((uu.ravel(),vv.ravel()))
source_rays=np.asarray([rs.rs2_deproject_pixel_to_point(intrinsic,uv.tolist(),1.) for uv in pixels])
ideal=np.column_stack((source_rays[:,0]*output['fx']+output['cx'],source_rays[:,1]*output['fy']+output['cy']))
dest=np.rint(ideal).astype(np.int64);target_indices=dest[:,1]*output['width']+dest[:,0]
source_indices=np.arange(len(pixels),dtype=np.int64)
h=hashlib.sha256();h.update(source_indices.tobytes());h.update(target_indices.tobytes());forward_sha=h.hexdigest()
assert forward_sha==contract['depth_forward_map_sha256']
assert len(np.unique(target_indices))==len(pixels)==contract['unique_destination_pixel_count']
ou,ov=np.meshgrid(np.arange(output['width']),np.arange(output['height']))
K=np.asarray([[output['fx'],0,output['cx']],[0,output['fy'],output['cy']],[0,0,1.]])
hom=np.column_stack((ou.ravel(),ov.ravel(),np.ones(ou.size)))
output_rays=(np.linalg.inv(K)@hom.T).T
pull=np.asarray([rs.rs2_project_point_to_pixel(intrinsic,ray.tolist()) for ray in output_rays])
mapx=pull[:,0].reshape(ou.shape).astype(np.float32);mapy=pull[:,1].reshape(ou.shape).astype(np.float32)
h=hashlib.sha256();h.update(mapx.tobytes());h.update(mapy.tobytes());inverse_sha=h.hexdigest()
assert inverse_sha==contract['rgb_inverse_map_sha256']
chosen=np.unique(np.r_[np.linspace(0,len(pixels)-1,4096,dtype=int),0,639,640*479,640*480-1])
roundtrip=np.asarray([rs.rs2_project_point_to_pixel(intrinsic,source_rays[i].tolist()) for i in chosen])-pixels[chosen]
outputback=(K@output_rays.T).T
outputback=outputback[:,:2]/outputback[:,2,None]
unmapped=np.ones(ou.size,bool);unmapped[target_indices]=False
axis=np.asarray([[0.,0.,1.],[-1.,0.,0.],[0.,-1.,0.]])
assert np.linalg.det(axis)==1 and np.array_equal(axis@np.asarray([0.,0.,1.]),[1.,0.,0.])
inverse_source=np.full(ou.size,-1,np.int64);inverse_source[target_indices]=source_indices

def independent_robust_plane(points):
    """LMedS triplet seed + full-ring Tukey IRLS, unrelated to production RANSAC."""
    points=np.asarray(points,dtype=np.float64)
    sample=points[np.linspace(0,len(points)-1,min(6000,len(points)),dtype=int)]
    rng=np.random.RandomState(138471)
    best=None;score=np.inf
    for unused in range(192):
        triple=sample[rng.choice(len(sample),3,replace=False)]
        n=np.cross(triple[1]-triple[0],triple[2]-triple[0]);length=np.linalg.norm(n)
        if length<1e-10:continue
        n/=length;offset=-float(n@triple[0]);median=float(np.median(abs(sample@n+offset)))
        if median<score:score=median;best=np.r_[n,offset]
    assert best is not None
    for unused in range(15):
        residual=points@best[:3]+best[3]
        sigma=max(.00015,float(np.median(abs(residual-np.median(residual))))*1.4826)
        normalized=residual/(4.685*sigma)
        weights=np.where(abs(normalized)<1,(1-normalized**2)**2,0.)
        center=(points*weights[:,None]).sum(axis=0)/weights.sum()
        centered=points-center
        eigenvalues,eigenvectors=np.linalg.eigh((centered*weights[:,None]).T@centered)
        n=eigenvectors[:,0]
        if n@best[:3]<0:n=-n
        new=np.r_[n,-float(n@center)]
        if np.linalg.norm(new-best)<1e-12:best=new;break
        best=new
    if best[2]<0:best=-best
    residual=points@best[:3]+best[3];inliers=abs(residual)<=.002
    return best,dict(point_count=len(points),inlier_fraction_at_original_2mm=float(inliers.mean()),inlier_rms_mm=float(np.sqrt(np.mean(residual[inliers]**2)))*1000,
        all_abs_residual_mm=np.percentile(abs(residual)*1000,[50,90,95,99]).tolist())

Rdc=np.asarray(profile['depth_to_color_extrinsics']['rotation_column_major']).reshape(3,3,order='F')
tdc=np.asarray(profile['depth_to_color_extrinsics']['translation_m'])
replays=[];planes={}
for frame in frames:
    ns=frame['source_stamp_ns']
    with np.load(frame['source_file'],allow_pickle=False) as z:
        raw=z['depth_raw'];scale=float(z['depth_scale']);T=z['T_base_camera'];savedK=z['intrinsics']
    assert raw.shape==(output['height'],output['width']) and raw.dtype==np.uint16
    assert np.array_equal(savedK,[output[k] for k in ('fx','fy','cx','cy')])
    assert scale==camera['depth_scale']==profile['depth_scale_m_per_unit']
    assert np.count_nonzero(raw.ravel()[unmapped])==0
    srcdepth=raw.ravel()[target_indices];roundtripdepth=np.zeros(raw.size,np.uint16);roundtripdepth[target_indices]=srcdepth
    assert np.array_equal(roundtripdepth.reshape(raw.shape),raw)
    meter=raw.ravel().astype(np.float64)*scale
    center=np.asarray(frame['ring_center_px']);radius=np.hypot(ou.ravel()-center[0],ov.ravel()-center[1]);size_scale=min(raw.shape[1]/640.,raw.shape[0]/480.)
    mask=(radius>=70*size_scale)&(radius<=frame['ring_outer_radius_px'])&(meter>=.03)&(meter<=2.)
    indices=np.flatnonzero(mask);src=inverse_source[indices];assert (src>=0).all()
    ray_options={'published_rectified_pinhole':output_rays[indices],'recovered_exact_source_sdk_rays':source_rays[src]}
    results={}
    published_points=None
    for name,rays in ray_options.items():
        optical=rays*meter[indices,None]
        camera_points=optical@axis.T
        base=camera_points@T[:3,:3].T+T[:3,3]
        plane,evidence=independent_robust_plane(base)
        results[name]=dict(plane_base=plane.tolist(),**evidence)
        planes.setdefault(name,[]).append(plane)
        if published_points is None:published_points=base
        else:results[name]['point_delta_from_published_mm']=np.percentile(np.linalg.norm(base-published_points,axis=1)*1000,[50,90,95,99,100]).tolist()
    # If an align implementation retains original native-depth Z, the exact
    # color Z is solved analytically; this is a conditional sensitivity only.
    rays=source_rays[src];corrected=(meter[indices]+float(Rdc[:,2]@tdc))/(rays@Rdc[:,2])
    cb=(rays*corrected[:,None])@axis.T@T[:3,:3].T+T[:3,3]
    cp,ce=independent_robust_plane(cb)
    planes.setdefault('conditional_native_Z_to_color_Z',[]).append(cp)
    results['conditional_native_Z_to_color_Z']=dict(plane_base=cp.tolist(),**ce,
        z_correction_mm=np.percentile((corrected-meter[indices])*1000,[0,50,100]).tolist(),
        execution_authority=False,condition='Only applicable if aligned image stores native-depth-camera axial Z; native unaligned frames are not recorded.')
    production=np.asarray(frame['plane_base'])
    for result in results.values():
        p=np.asarray(result['plane_base']);delta=p-production
        result['plane_vs_production_anchor_mm']=float(np.r_[record['reference']['anchor_base'],1.]@delta)*1000
        result['plane_vs_production_normal_deg']=float(np.rad2deg(np.arccos(np.clip(p[:3]@production[:3],-1,1))))
    replays.append(dict(source_stamp_ns=ns,encoding='16UC1',shape=list(raw.shape),scale_m_per_unit=scale,
        depth_units_roundtrip_exact=True,unmapped_nonzero_count=0,valid_source_depth_count=int((srcdepth>0).sum()),results=results))

comparisons={};anchor=np.asarray(record['reference']['anchor_base']);footprint=np.asarray(record['reference']['footprint_base'])
for name,series in planes.items():
    # Use each independent route's own initial fitted plane; project original
    # target footprint onto it before comparing the later measured planes.
    ref=series[0];proj=footprint-(footprint@ref[:3]+ref[3])[:,None]*ref[:3]
    comparisons[name]=[]
    for i,p in enumerate(series):
        delta=p-ref;center=proj.mean(axis=0)
        comparisons[name].append(dict(source_stamp_ns=frames[i]['source_stamp_ns'],anchor_signed_delta_mm=float(np.r_[anchor,1.]@delta)*1000,
            footprint_center_signed_delta_mm=float(np.r_[center,1.]@delta)*1000,
            maximum_local_delta_mm=max(abs(float(np.r_[anchor,1.]@delta)),float(np.max(abs(proj@delta[:3]+delta[3]))))*1000,
            normal_delta_deg=float(np.rad2deg(np.arccos(np.clip(p[:3]@ref[:3],-1,1))))))

stable={k:len({json.dumps(m['profile'].get(k),sort_keys=True) for m in metadata}) for k in profile}
out=dict(record=record['record'],source_report_sha256=hashlib.sha256((INPUT/'report.json').read_bytes()).hexdigest(),
    camera_info_topics_in_sealed_bag=[t for t in topics if 'camera_info' in t],
    camera_info_limitation='No CameraInfo is present in this sealed bag; current camera_node publishes paired image headers and runtime geometry parameters, not a CameraInfo publisher.',
    metadata_sample_count=len(metadata),unique_geometry_values_by_field=stable,camera=metadata[0],
    sdk=dict(version=rs.__version__,module=rs.__file__,module_sha256=hashlib.sha256(Path(rs.__file__).read_bytes()).hexdigest()),
    independent_map_checks=dict(depth_forward_map_sha256=forward_sha,recorded_depth_forward_map_sha256=contract['depth_forward_map_sha256'],
        rgb_inverse_map_sha256=inverse_sha,recorded_rgb_inverse_map_sha256=contract['rgb_inverse_map_sha256'],
        unique_source_count=len(pixels),unique_target_count=len(np.unique(target_indices)),
        pixel_center_quantization_max_abs_xy_px=abs(ideal-dest).max(axis=0).tolist(),
        sdk_source_deproject_project_max_abs_xy_px=abs(roundtrip).max(axis=0).tolist(),
        pinhole_output_roundtrip_max_abs_xy_px=abs(outputback-hom[:,:2]).max(axis=0).tolist(),
        optical_to_ros_camera_link=axis.tolist(),native_stream_shape=[480,640],published_shape=[489,653]),
    independent_fit_method='Float64 inverse-K optical rays, proper optical-to-camera_link rotation, exact saved base TF; independent LMedS seed and full-ring Tukey IRLS. Alternative route recovers every original aligned source pixel through the injective map and directly applies SDK distorted rays.',
    frames=replays,independent_cross_station_plane_comparisons=comparisons,
    restrictions=dict(no_ros_master=True,no_hardware_interfaces=True,no_live_edits=True,no_threshold_edits=True),
    limitations=['Recorded depth has SDK alignment and filtering already applied. The original unaligned depth/color pair is unavailable, so the alignment stage cannot be byte-replayed.',
        'Plane differences caused by independent robust fitting are not new runtime geometry.',
        'Conditional native-Z correction is a sensitivity calculation, not a proved application bug or deployable correction.'])
OUT.mkdir(parents=True,exist_ok=True);p=OUT/'camera_geometry_audit.json'
with p.open('x') as f:json.dump(out,f,indent=2,ensure_ascii=False);f.write('\n')
print(json.dumps(dict(path=str(p),sha256=hashlib.sha256(p.read_bytes()).hexdigest(),stability=stable,comparisons=comparisons)))
