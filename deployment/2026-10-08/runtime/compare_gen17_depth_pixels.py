"""Offline measured same-pixel depth differences; no plane refit or ROS node."""
import dataclasses, hashlib, json, zlib,zipfile
from pathlib import Path
import numpy as np
from sensor_msgs.msg import Image
from tf.transformations import quaternion_matrix
from alicia_grasp_modes import tabletop

BASE=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/gen17_support_measurement_20261006')
CAP=BASE/'raw_support_v2'

def quant(a):
 a=np.asarray(a,dtype=float)
 if a.size==0:return {'count':0}
 return dict(count=int(a.size),percentile_mm={str(p):float(np.percentile(a,p)*1000)for p in (1,5,25,50,75,95,99)},
  mean_mm=float(np.mean(a)*1000),standard_deviation_mm=float(np.std(a)*1000),
  fraction_absolute_above_mm={str(t):float(np.mean(np.abs(a)>t/1000))for t in (.5,1,2,5)})
def matrix(row):
 t=row['exact_tf'];m=quaternion_matrix(t['quaternion_xyzw']);m[:3,3]=t['translation'];return m

def main():
 validation=json.loads((BASE/'source_transport_validation_v2.json').read_text())
 assert not validation['tf_lookup_failures']
 proofs={r['source_stamp_ns']:r for r in validation['source_pairs']}
 good=validation['first_loss']['last_good_source_stamp_ns']
 with zipfile.ZipFile(CAP/'diagnostic.zip')as z:
  metadata=json.loads(z.read('metadata/0.json'));camera=metadata['parameters']['/camera'];c=metadata['parameters']['/unknown_perception']
  config=tabletop.Config(**{f.name:c[f.name]for f in dataclasses.fields(tabletop.Config)if f.name in c})
  K=[camera[k]for k in ('fx','fy','cx','cy')];scale=camera['depth_scale']
  statuses=[json.loads(json.loads(z.read(n))['original_status_data'])for n in z.namelist()if n.startswith('status/')]
  by={r['source_stamp_ns']:r for r in statuses if r.get('generation')==17 and r.get('source_stamp_ns')}
  sources=[r['source_stamp_ns']for r in statuses if r.get('generation')==17 and r.get('detail')=='support_measurement_pending_revalidation']
  def depth(ns):
   raw=zlib.decompress(z.read('pairs/%d_depth.rosmsg.zlib'%ns));msg=Image().deserialize(raw)
   assert msg.header.stamp.to_nsec()==ns
   assert msg.encoding=='16UC1'
   dtype=np.dtype('>u2'if msg.is_bigendian else '<u2')
   rawarray=np.frombuffer(msg.data,dtype=dtype).reshape(msg.height,msg.step//2)[:,:msg.width]
   return rawarray.astype(np.float32)*scale
  D0=depth(good);V0=np.isfinite(D0)&(D0>=config.depth_min_m)&(D0<=config.depth_max_m)
  T0=matrix(proofs[good]);anchor=np.array(by[good]['anchor_position_base']);plane=np.array(by[good]['reference_support_plane_base'])
  center=tabletop.project_base_to_pixel(anchor,T0,K);v,u=np.indices(D0.shape);pxscale=min(D0.shape[1]/640,D0.shape[0]/480)
  radius=np.hypot(u-center[0],v-center[1])/pxscale
  regions=dict(full=np.ones(D0.shape,bool),target_center_radius70=radius<70)
  for size in (110,140,170):regions['support_ring70_%d'%size]=(radius>=70)&(radius<=size)
  points0=tabletop._project(D0,K);res0=np.einsum('ijk,k->ij',points0,T0[:3,:3].T@plane[:3])+plane[:3]@T0[:3,3]+plane[3]
  output=[]
  for stamp in sources:
   D=depth(stamp);V=np.isfinite(D)&(D>=config.depth_min_m)&(D<=config.depth_max_m);T=matrix(proofs[stamp])
   rotation=T[:3,:3]@T0[:3,:3].T;angle=float(np.degrees(np.arccos(np.clip((np.trace(rotation)-1)/2,-1,1))))
   points=tabletop._project(D,K);res=np.einsum('ijk,k->ij',points,T[:3,:3].T@plane[:3])+plane[:3]@T[:3,3]+plane[3]
   regions_out={}
   for name,R in regions.items():
    common=R&V&V0;delta=D[common]-D0[common];count=np.count_nonzero(R)
    regions_out[name]=dict(total_pixels=int(count),reference_valid_fraction=float(np.mean(V0[R])),current_valid_fraction=float(np.mean(V[R])),
       shared_valid_fraction=float(np.count_nonzero(common)/count),reference_valid_now_invalid_fraction=float(np.count_nonzero(R&V0&~V)/count),
       reference_invalid_now_valid_fraction=float(np.count_nonzero(R&~V0&V)/count),depth_delta=quant(delta),
       shared_reference_plane_residual_before=quant(res0[common]),shared_reference_plane_residual_after=quant(res[common]),
       fixed_original_plane_inlier_fraction_before=float(np.mean(np.abs(res0[common])<=config.support_threshold_m)),
       fixed_original_plane_inlier_fraction_after=float(np.mean(np.abs(res[common])<=config.support_threshold_m)))
   tiles=[]
   for yy in range(0,D.shape[0],64):
    for xx in range(0,D.shape[1],64):
     region=np.s_[yy:min(yy+64,D.shape[0]),xx:min(xx+64,D.shape[1])];sel=V[region]&V0[region]
     if not np.any(sel):continue
     delta=(D[region]-D0[region])[sel]
     tiles.append(dict(top_left=[xx,yy],common_pixels=int(len(delta)),median_depth_delta_mm=float(np.median(delta)*1000),p95_absolute_delta_mm=float(np.percentile(np.abs(delta),95)*1000)))
   output.append(dict(source_stamp_ns=stamp,reference_source_stamp_ns=good,original_status_detail=by[stamp]['detail'],
     original_tf_translation_change_m=float(np.linalg.norm(T[:3,3]-T0[:3,3])),original_tf_rotation_change_deg=angle,
     projected_anchor_center_pixels=center.tolist(),regions=regions_out,tiles_64px=tiles))
 result=dict(schema=1,reference_source_stamp_ns=good,reference_kind='latest preceding ready measured frame; not replacement for pending data',
   original_reference_plane_base=plane.tolist(),depth_range_m=[config.depth_min_m,config.depth_max_m],depth_scale=scale,
   raw_shape=list(D0.shape),comparison_results=output,
   source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),tabletop_source_sha256=hashlib.sha256(Path(tabletop.__file__).read_bytes()).hexdigest(),
   limitations=['Pixel correspondences use the original fixed projection; original TF differences are reported, no point registration or data correction is applied.',
    'Fixed reference plane inlier fractions are descriptive, not a replacement for the production current-plane fit or quality gates.',
    'A small median with broad signed differences supports spatially varying measurements over a uniform shift; it does not identify the sensor/filter hardware cause.',
    'Camera regional filter runtime snapshots are asynchronous; no per-exposure regional attribution is made.'])
 out=BASE/'depth_pixel_comparison.json';assert not out.exists();out.write_text(json.dumps(result,ensure_ascii=False,indent=2,allow_nan=False)+'\n')
 print(json.dumps([dict(source=r['source_stamp_ns'],tf_translation_m=r['original_tf_translation_change_m'],tf_rotation_deg=r['original_tf_rotation_change_deg'],rings={k:{'valid_before':v['reference_valid_fraction'],'valid_after':v['current_valid_fraction'],'difference':v['depth_delta']}for k,v in r['regions'].items()if k.startswith('support')})for r in output],ensure_ascii=False))
if __name__=='__main__':main()
