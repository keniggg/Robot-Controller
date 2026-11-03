#!/usr/bin/env python3
"""Closed-bag production mask/table/board comparison; no hardware or identity edits."""
import argparse,json,sys,hashlib
from pathlib import Path
import numpy as np
import cv2,rosbag
from cv_bridge import CvBridge
HERE=Path(__file__).resolve().parent;sys.path.insert(0,str(HERE))
import audit_motor_adjacent_table as A
D=A.D

def normalized(p):
 p=np.array(p,dtype=float);return p/np.linalg.norm(p[:3])
def plane_to_local(T,plane):
 p=normalized(plane);return normalized(np.r_[T[:3,:3].T@p[:3],p[3]+p[:3]@T[:3,3]])
def compare_plane(p,q,points):
 p,q=normalized(p),normalized(q)
 if p[:3]@q[:3]<0:q=-q
 d=points@(p[:3]-q[:3])+p[3]-q[3]
 return dict(normal_deg=float(np.degrees(np.arccos(np.clip(p[:3]@q[:3],-1,1)))),max_footprint_difference_mm=float(np.max(abs(d))*1000),median_signed_difference_mm=float(np.median(d)*1000))

def main():
 ap=argparse.ArgumentParser();ap.add_argument('session',type=Path);ap.add_argument('--output',type=Path,required=True);args=ap.parse_args();session,out=args.session,args.output
 if D.MOUNT not in out.resolve().parents:raise RuntimeError('external archive required')
 out.mkdir(parents=True,exist_ok=True)
 baseline=json.loads((session/'baseline_geometry.json').read_text());geos,ga=D.collect_paired_geometries(session,baseline,maximum_per_window=1000)
 bysource={g['source_stamp_ns']:g for g in geos.values() if g.get('offline_stationary_phase')=='center_J3'}
 statuses={};masks={};rgb={};depth={};objects={};bridge=CvBridge()
 topics=['/perception/unknown/detector_status','/perception/unknown/object_mask','/perception/unknown/object','/bounded_motor/production_exact_rgb','/bounded_motor/production_exact_depth']
 with rosbag.Bag(str(session/'evidence.bag')) as b:
  for topic,msg,t in b.read_messages(topics=topics):
   if topic.endswith('detector_status'):
    s=json.loads(msg.data);ns=s.get('source_stamp_ns');
    if ns in bysource and s.get('state')=='ready':statuses[ns]=(s,t.to_nsec())
   else:
    ns=msg.header.stamp.to_nsec()
    if ns not in bysource:continue
    if topic.endswith('object_mask'):masks[ns]=bridge.imgmsg_to_cv2(msg,'mono8')
    elif topic.endswith('exact_rgb'):rgb[ns]=bridge.imgmsg_to_cv2(msg,'bgr8')
    elif topic.endswith('exact_depth'):depth[ns]=bridge.imgmsg_to_cv2(msg,'passthrough')
    else:objects[ns]=dict(detected=msg.detected,label=msg.label,u=msg.u,v=msg.v,confidence=msg.confidence)
 available=sorted(set(statuses)&set(masks)&set(rgb)&set(depth)&set(objects));chosen=[available[i] for i in np.unique(np.linspace(0,len(available)-1,min(3,len(available))).round().astype(int))] if available else []
 seed=json.loads((session/'motor_evidence.json').read_text());anchor=np.array(seed['manual_motor_seed']['anchor_board']);rows=[]
 for ns in chosen:
  g=bysource[ns];s,status_receipt=statuses[ns];name=str(ns);row=A.frame(g,rgb[ns],depth[ns],anchor,out,name)
  arrays=np.load(out/(name+'.npz'));uv=arrays['pixels'];pc=arrays['points_camera'];pb=arrays['points_board'];cand=arrays['motor_candidate']
  onmask=masks[ns][uv[:,1],uv[:,0]]>0;valid=np.isfinite(pc).all(1)&(pc[:,0]>.1)&(pc[:,0]<.7);onmask&=valid
  intersection=int(np.count_nonzero(onmask&cand));prod_sampled=int(onmask.sum());candidate=int(cand.sum())
  Y=np.array(g['T_base_board']);prod=plane_to_local(Y,s['geometry_metrics']['support_plane_base']);table=normalized(row['table_plane_board'])
  ys,xs=np.nonzero(masks[ns]>0);maskuv=np.c_[xs,ys];z=depth[ns][ys,xs]*g['camera_parameters']['depth_scale'];good=(z>.1)&(z<.7)
  model=D.CM.RuntimeColorModel(g['quality']['camera_model']);allpc=(D.rays(model,maskuv[good])*z[good,None])@D.O[:3,:3].T
  B=np.array(g['T_camera_board']);allpb=A.transform(np.linalg.inv(B),allpc)
  footprint=allpb-(allpb@table[:3]+table[3])[:,None]*table[:3]
  # Independently fit measured board depth, using PnP only for interior ROI selection.
  yy,xx=np.mgrid[0:480:3,0:640:3];buv=np.c_[xx.ravel(),yy.ravel()];bz=depth[ns][buv[:,1],buv[:,0]]*g['camera_parameters']['depth_scale']
  bpc=(D.rays(model,buv)*bz[:,None])@D.O[:3,:3].T;bpb=A.transform(np.linalg.inv(B),bpc)
  inside=(bz>.1)&(bz<.7)&(bpb[:,0]>.012)&(bpb[:,0]<.168)&(bpb[:,1]>.012)&(bpb[:,1]<.123)
  normal,offset,bmeta=D.fit_plane(bpb[inside],threshold=.002)
  board=normalized(np.r_[normal,offset])
  if board[:3]@table[:3]<0:board=-board
  ci=model.evidence['color_intrinsics'];opt=np.c_[(maskuv[good,0]-ci['cx'])/ci['fx']*z[good],(maskuv[good,1]-ci['cy'])/ci['fy']*z[good],z[good]];pinhole=opt@D.O[:3,:3].T
  delta=np.linalg.norm(allpc-pinhole,axis=1)*1000
  result=dict(source_stamp_ns=ns,status_receipt_age_ms=(status_receipt-ns)/1e6,
   raw_source_sha256=row['source_sha256'],production_mask_sha256=hashlib.sha256(masks[ns].tobytes()).hexdigest(),
   production_status=s,production_object=objects[ns],geometry=g,
   production_sampled_valid_mask_pixels=prod_sampled,diagnostic_candidate_pixels=candidate,
   production_mask_fraction_inside_unique_motor_candidate=intersection/max(1,prod_sampled),
   diagnostic_candidate_fraction_inside_production_mask=intersection/max(1,candidate),
   measured_sparse_mask_iou=intersection/max(1,np.count_nonzero(onmask|cand)),
   support_production_board=prod.tolist(),support_adjacent_table_board=table.tolist(),support_independent_board_top_board=board.tolist(),
   board_depth_fit=bmeta,table_depth_fit=row['table_fit'],
   production_vs_adjacent_table=compare_plane(prod,table,footprint),production_vs_board_top=compare_plane(prod,board,footprint),
   board_top_vs_adjacent_table=compare_plane(board,table,footprint),
   sdk_vs_production_pinhole_mask_point_distance_mm=dict(median=float(np.median(delta)),p95=float(np.percentile(delta,95)),maximum=float(delta.max())),
   motor_height_above_adjacent_table_mm=(np.percentile(allpb@table[:3]+table[3],[10,50,90])*1000).tolist(),
   offline_only=True,motion_authorized=False,remote_track_identity_bound=False)
  (out/(name+'_audit.json')).write_text(json.dumps(result,indent=2)+'\n')
  overlay=rgb[ns].copy();contours,_=cv2.findContours((masks[ns]>0).astype(np.uint8),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE);cv2.drawContours(overlay,contours,-1,(255,255,0),1);cv2.imwrite(str(out/(name+'_production_mask.png')),overlay)
  rows.append({k:result[k] for k in ('source_stamp_ns','production_mask_fraction_inside_unique_motor_candidate','diagnostic_candidate_fraction_inside_production_mask','measured_sparse_mask_iou','production_vs_adjacent_table','production_vs_board_top','board_top_vs_adjacent_table','sdk_vs_production_pinhole_mask_point_distance_mm','motor_height_above_adjacent_table_mm')})
 report=dict(session=str(session),available_exact_stationary_production_sources=available,chosen_sources=chosen,rows=rows,stationary_audit=ga,production_identity_not_manufactured=True,diagnostic_only=True,motion_authorized=False)
 (out/'report.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2),flush=True)
if __name__=='__main__':main()
