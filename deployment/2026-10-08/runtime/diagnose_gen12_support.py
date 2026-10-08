"""Bounded, closed-bag support-fit diagnosis. No ROS node/service/device I/O."""
import argparse, hashlib, json
from dataclasses import asdict
from pathlib import Path
import numpy as np
import cv2, rosbag, rospy, tf2_ros
from cv_bridge import CvBridge
from tf.transformations import quaternion_matrix
from alicia_grasp_modes.tabletop import (Config, _project_pixels, project_base_to_pixel,
    _fit_plane, _support_quadrants, _normalized_plane, _validate_support_plane_update)

RECORD = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/grasp_records/20261006T074059.722455Z_4f669c3e99b047e69ee49b6a8966a8ab')
WANTED = [1791272468047529458,1791272468247684955,1791272468447761774,
          1791272471452625989,1791272471685503005,1791272471884931564,
          1791272472085348844,1791272472283080101]

def fitting(points,cfg):
    if len(points)<cfg.support_min_points: return {'error':'insufficient_support_depth'},None
    sampled=points[np.linspace(0,len(points)-1,min(len(points),cfg.support_max_samples),dtype=int)]
    rng=np.random.RandomState(17);best=np.zeros(len(sampled),bool);plane=None
    for _ in range(128):
        ix=rng.randint(len(sampled),size=3)
        if len(set(ix.tolist()))<3:continue
        a,b,c=sampled[ix];n=np.cross(b-a,c-a);length=np.linalg.norm(n)
        if length<1e-9:continue
        n/=length;inside=abs(np.einsum('ij,j->i',sampled-a,n))<=cfg.support_threshold_m
        if np.count_nonzero(inside)>np.count_nonzero(best):best=inside;plane=np.r_[n,-n@a]
    ratios=[float(best.mean())];planes=[plane.tolist()]
    for _ in range(2):
        center=np.mean(sampled[best],axis=0)
        _,_,vh=np.linalg.svd(sampled[best]-center,full_matrices=False)
        normal=vh[-1];offset=-float(normal@center)
        best=abs(np.einsum('ij,j->i',sampled,normal)+offset)<=cfg.support_threshold_m
        ratios.append(float(best.mean()));planes.append(np.r_[normal,offset].tolist())
    if offset<0:normal,offset=-normal,-offset
    plane=np.r_[normal,offset]
    distance=points@normal+offset
    summary={'sample_count':len(sampled),'ratios_ransac_svd1_svd2':ratios,
        'planes_ransac_svd1_svd2':planes,'full_inlier_ratio':float(np.mean(abs(distance)<=cfg.support_threshold_m)),
        'sample_residual_m':float(np.sqrt(np.mean((sampled[best]@normal+offset)**2))),
        'full_abs_residual_percentile_mm':np.percentile(abs(distance)*1000,[25,50,70,80,90,95,99]).tolist(),
        'original_fit_pass':bool(ratios[-1]>=cfg.support_min_ratio)}
    try:
        original=_fit_plane(points,cfg)
        summary['original_fit_check']={'ratio':original[1],'residual_m':original[2], 'plane_equal':bool(np.array_equal(original[0],plane))}
    except ValueError as exc:summary['original_fit_check']={'error':str(exc)}
    return summary,plane

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--out',required=True);args=parser.parse_args()
    manifest=json.loads((RECORD/'manifest.json').read_text())
    assert manifest['state']=='complete' and manifest['recorder_closed'] is True
    out=Path(args.out);assert Path('/media/zhuyupei/SanStand').is_mount();out.mkdir(exist_ok=False,parents=True)
    cfg=Config();bridge=CvBridge();cv2.setNumThreads(1)
    buffer=tf2_ros.Buffer(cache_time=rospy.Duration(20),debug=False)
    statuses={};depths={};colors={};camera=None
    with rosbag.Bag(str(RECORD/'keyframes.bag'),'r') as bag:
        for topic,msg,t in bag.read_messages(topics=['/tf_static','/grasp_record/camera_runtime_metadata']):
            if topic=='/tf_static':
                for tr in msg.transforms:buffer.set_transform_static(tr,'sealed_record')
            elif camera is None:camera=json.loads(msg.data)['parameters']
            if camera is not None and buffer.can_transform_core('tool0','camera_link',rospy.Time(0))[0]:break
        topics=['/tf','/perception/unknown/detector_status','/supervisor/camera/color/image_raw','/supervisor/camera/depth/image_raw']
        for topic,msg,t in bag.read_messages(topics=topics,start_time=rospy.Time.from_sec(1791272467.5),end_time=rospy.Time.from_sec(1791272473)):
            if topic=='/tf':
                for tr in msg.transforms:buffer.set_transform(tr,'sealed_record')
            elif topic.endswith('detector_status'):
                s=json.loads(msg.data);ns=s.get('source_stamp_ns')
                if ns in WANTED:statuses[ns]=s
            elif msg.header.stamp.to_nsec() in WANTED:
                (depths if '/depth/' in topic else colors)[msg.header.stamp.to_nsec()]=msg
    assert camera is not None
    K=[camera[k] for k in ['fx','fy','cx','cy']]
    (out/'camera_metadata_parameters.json').write_text(json.dumps(camera,indent=2)+'\n')
    report={'record':str(RECORD),'schema':1,'source_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        'config':asdict(cfg),'config_provenance':'current source Config defaults; original unknown_perception parameter snapshot not in bag; confirm by ready-frame exact reproduction',
        'selected_source_count':len(WANTED),'rows':[]}
    for ns in WANTED:
        if ns not in depths or ns not in statuses:
            report['rows'].append({'source_ns':ns,'missing_exact_depth_or_status':True});continue
        s=statuses[ns];msg=depths[ns];d=bridge.imgmsg_to_cv2(msg,'passthrough').astype(np.float32)*camera['depth_scale']
        tr=buffer.lookup_transform_core('base_link','camera_link',msg.header.stamp)
        q=tr.transform.rotation;t=tr.transform.translation;F=quaternion_matrix([q.x,q.y,q.z,q.w]);F[:3,3]=[t.x,t.y,t.z]
        anchor=np.array(s['anchor_position_base']);ref=np.array(s['reference_support_plane_base']);footprint=np.array(s['reference_support_footprint_base'])
        center=project_base_to_pixel(anchor,F,K);v,u=np.indices(d.shape);scale=min(d.shape[1]/640.,d.shape[0]/480.)
        radius=np.hypot(u-center[0],v-center[1]);valid=np.isfinite(d)&(d>=cfg.depth_min_m)&(d<=cfg.depth_max_m)
        row={'source_ns':ns,'detail':s['detail'],'source_qualification':s.get('source_qualification'),'frame_timing':s.get('frame_timing'),
             'recorded_geometry_metrics':s.get('geometry_metrics'),'center_uv':center.tolist(),'T_base_camera':F.tolist(),'reference_plane_base':ref.tolist(),'rings':[]}
        residual_map=np.full(d.shape,np.nan)
        for outer in (170,140,110):
            region=(radius>=70*scale)&(radius<=outer*scale);ring=region&valid;pu,pv=u[ring],v[ring]
            points=_project_pixels(d[ring],pu,pv,K);fit,plane=fitting(points,cfg)
            entry={'outer_base_px':outer,'outer_actual_px':outer*scale,'valid_points':len(points),'valid_fraction':float(ring.sum()/region.sum()),**fit}
            if plane is not None:
                dist=points@plane[:3]+plane[3];supporting=abs(dist)<=cfg.support_threshold_m
                entry['quadrants']=_support_quadrants(supporting,pu,pv,center)
                current=_normalized_plane(np.linalg.solve(F.T,plane))
                try:
                    current,agreement=_validate_support_plane_update(ref,current,anchor,cfg,footprint)
                    entry.update(consistency_pass=True,agreement=agreement)
                except ValueError as exc:entry.update(consistency_pass=False,consistency_error=str(exc))
                pc=F.T@ref;rd=points@pc[:3]+pc[3]
                entry['frozen_reference_inlier_ratio']=float(np.mean(abs(rd)<=cfg.support_threshold_m))
                entry['frozen_reference_signed_percentiles_mm']=np.percentile(rd*1000,[1,5,10,25,50,75,90,95,99]).tolist()
                entry['signed_tail_fractions']={str(x):float(np.mean(rd>x/1000)) for x in (2,4,8,12)}
                entry['quadrant_inlier_ratios']=[]
                for left,top in ((True,True),(True,False),(False,True),(False,False)):
                    subset=((pu<center[0]) if left else (pu>=center[0]))&((pv<center[1]) if top else (pv>=center[1]))
                    entry['quadrant_inlier_ratios'].append({'left':left,'top':top,'n':int(subset.sum()),'ratio':float(supporting[subset].mean())})
                if outer==170:
                    residual_map[ring]=rd*1000
                    bad=np.zeros(d.shape,np.uint8);bad[ring]=(abs(rd)>.002).astype(np.uint8)
                    n,labels,stats,centers=cv2.connectedComponentsWithStats(bad,8)
                    entry['largest_outlier_components_xywh_area']=[stats[i].tolist() for i in np.argsort(stats[1:,4])[-6:][::-1]+1]
            row['rings'].append(entry)
        if ns in colors:
            rgb=bridge.imgmsg_to_cv2(colors[ns],'bgr8');np.savez_compressed(out/('%d.npz'%ns),depth_m=d,bgr=rgb,reference_residual_mm=residual_map,T_base_camera=F)
            # Numerical analysis plot; no changed pixels are fed to perception.
            import matplotlib;matplotlib.use('Agg');import matplotlib.pyplot as plt
            fig,ax=plt.subplots(1,2,figsize=(12,4));ax[0].imshow(rgb[:,:,::-1]);ax[0].set_title(str(ns))
            for outer in (70,110,140,170):ax[0].add_patch(plt.Circle(center,outer*scale,fill=False,color='yellow',linewidth=.6))
            im=ax[1].imshow(residual_map,cmap='coolwarm',vmin=-8,vmax=8);ax[1].set_title('Ring residual to frozen plane (mm)');fig.colorbar(im,ax=ax[1]);fig.tight_layout();fig.savefig(out/('%d.png'%ns));plt.close(fig)
        report['rows'].append(row)
        print(json.dumps({'source_ns':ns,'detail':s['detail'],'rings':[{k:r.get(k) for k in ['outer_base_px','valid_fraction','ratios_ransac_svd1_svd2','quadrants','consistency_pass','agreement','frozen_reference_inlier_ratio']} for r in row['rings']]}),flush=True)
    (out/'report.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')

if __name__=='__main__':main()
