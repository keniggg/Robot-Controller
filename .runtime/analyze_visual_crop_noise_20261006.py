#!/usr/bin/env python3
"""Offline spatial LK diagnostics; no frame continuity/qualification is fabricated."""
import argparse,json,hashlib,sys,zipfile,zlib
from pathlib import Path
import cv2,numpy as np
from sensor_msgs.msg import Image
W=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(W/'src/alicia_flexible_grasp_supervisor/src'))
from alicia_flexible_grasp.vision.visual_stationary import VisualStationaryWindow as V
EXT=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
def sha(b):return hashlib.sha256(b).hexdigest()
def calc(a,b,region,seed_kind='old',child_resize=False):
 h,w=a.shape[:2];parent=(int(.2*w),int(.1*h),int(.8*w),int(.9*h));x0,y0,x1,y1=parent
 if region=='child':
  dx,dy=(x1-x0)//2,(y1-y0)//2;x0+=(x1-x0-dx)//2;y0+=(y1-y0-dy)//2;x1,y1=x0+dx,y0+dy
 colors=[np.ascontiguousarray(im[y0:y1,x0:x1]) for im in (a,b)];hh,ww=colors[0].shape[:2]
 ratio=min(1.,320./max(hh,ww)) if not child_resize else min(1.,320./max(parent[2]-parent[0],parent[3]-parent[1]))
 grays=[cv2.cvtColor(cv2.resize(im,(round(ww*ratio),round(hh*ratio)),interpolation=cv2.INTER_AREA) if ratio<1 else im,cv2.COLOR_BGR2GRAY) for im in colors]
 scale=np.float32([ww/grays[0].shape[1],hh/grays[0].shape[0]])
 window=V();window._seed(grays[0],None,None);pts=window._points
 if seed_kind!='old':
  pts=[];gh,gw=grays[0].shape
  for row in range(2):
   for col in range(2):
    mask=np.zeros((gh,gw),np.uint8);mask[row*gh//2:(row+1)*gh//2,col*gw//2:(col+1)*gw//2]=255
    if seed_kind=='full_mask_margin':mask[:31]=0;mask[-31:]=0;mask[:,:31]=0;mask[:,-31:]=0
    q=cv2.goodFeaturesToTrack(grays[0],16,.02,8,mask=mask,blockSize=5)
    if q is not None:pts.append(q)
  pts=np.concatenate(pts) if pts else None
 if pts is None:return dict(error='no_seed')
 kw=dict(winSize=(15,15),maxLevel=2,criteria=(cv2.TERM_CRITERIA_EPS|cv2.TERM_CRITERIA_COUNT,15,.01))
 f,s,e=cv2.calcOpticalFlowPyrLK(grays[0],grays[1],pts,None,**kw)
 back,r,_=cv2.calcOpticalFlowPyrLK(grays[1],grays[0],f,None,**kw)
 fb=np.linalg.norm((back-pts).reshape(-1,2)*scale,axis=1);xy=f.reshape(-1,2)
 valid=(s.ravel()==1)&(r.ravel()==1)&(fb<=.5)&(xy[:,0]>=0)&(xy[:,0]<grays[0].shape[1])&(xy[:,1]>=0)&(xy[:,1]<grays[0].shape[0])
 delta=(f-pts).reshape(-1,2)*scale;norm=np.linalg.norm(delta,axis=1);coverage=window._coverage(f[valid],grays[0].shape)
 p=pts.reshape(-1,2);gh,gw=grays[0].shape
 internal=np.minimum(abs(p[:,0]-gw/2),abs(p[:,1]-gh/2));border=np.minimum.reduce((p[:,0],p[:,1],gw-1-p[:,0],gh-1-p[:,1]))
 def group(mask):
  v=mask&valid
  return dict(n=int(v.sum()),p90=float(np.percentile(norm[v],90)) if v.any() else None,above_025=int((v&(norm>.25)).sum()))
 return dict(seed_count=len(pts),valid_count=int(valid.sum()),coverage=coverage,
  evidence_complete=bool(valid.sum()>=max(24,np.ceil(.8*len(pts))) and coverage>=3),
  p90=float(np.percentile(norm[valid],90)),median=float(np.median(norm[valid])),median_vector=np.median(delta[valid],axis=0).tolist(),
  fb_p90=float(np.percentile(fb[valid],90)),above_025=int((valid&(norm>.25)).sum()),
  internal_border_4px=group(internal<4),outer_border_31px=group(border<31),interior=group(border>=31),
  crop_shape=[hh,ww],gray_shape=list(grays[0].shape),scale=scale.tolist())

def main():
 p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);a=p.parse_args();assert EXT in a.output.resolve().parents;a.output.mkdir(exist_ok=False)
 cv2.setNumThreads(1);cases=[];sources=[]
 raw=EXT/'ros_log/camera_stage_capture_20261004T124933Z/stages/sample_06.npz';meta=json.loads(raw.with_suffix('.json').read_text())
 with np.load(raw,allow_pickle=False) as z:rgb=np.ascontiguousarray(z['raw_rgb'])
 assert sha(rgb.tobytes())==meta['arrays']['raw_rgb']['sha256'];sources.append(dict(path=str(raw),file_sha256=sha(raw.read_bytes()),coordinate_domain='old original SDK RGB; not latest capture'))
 rng=np.random.RandomState(271828)
 for sigma in (0.,1.,2.,4.,8.):
  for shift in (0.,.1,.3,.5,.8):
   changed=cv2.warpAffine(rgb,np.float32([[1,0,shift],[0,1,0]]),(rgb.shape[1],rgb.shape[0]),flags=cv2.INTER_LINEAR,borderMode=cv2.BORDER_REFLECT)
   if sigma:changed=np.clip(changed.astype(float)+rng.normal(0,sigma,changed.shape),0,255).astype(np.uint8)
   cases.append(dict(kind='synthetic_shift_and_independent_pixel_noise',shift_x_full_input_px=shift,noise_sigma_8bit=sigma,frames=(rgb,changed)))
 capture=EXT/'ros_log/gen17_support_measurement_20261006/raw_support_v2'
 summary=json.loads((capture/'summary.json').read_text());assert summary['archive_closed'] and summary['complete_without_writer_error']
 stamps=summary['saved_pair_source_stamps_ns'][:8];images=[]
 with zipfile.ZipFile(capture/'diagnostic.zip') as z:
  for ns in stamps:
   b=zlib.decompress(z.read('pairs/%d_color.rosmsg.zlib'%ns));info=json.loads(z.read('pairs/%d.json'%ns));assert sha(b)==info['streams']['color']['serialized_sha256']
   msg=Image().deserialize(b);assert msg.header.stamp.to_nsec()==ns
   images.append(np.frombuffer(msg.data,np.uint8).reshape(msg.height,msg.step)[:,:msg.width*3].reshape(msg.height,msg.width,3).copy())
   sources.append(dict(archive=str(capture/'diagnostic.zip'),source_stamp_ns=ns,serialized_sha256=sha(b),coordinate_domain='gen17 rectified ROS publication, not original SDK RGB'))
 for index in range(1,len(images)):
  cases.append(dict(kind='old_sparse_rectified_adjacent_pair',reference_stamp_ns=stamps[index-1],current_stamp_ns=stamps[index],frames=(images[index-1],images[index])))
 report=dict(schema=1,production_modified=False,hardware_or_ROS_commands=False,sources=sources,
  source_sha256={str(x.relative_to(W)):sha(x.read_bytes()) for x in (Path(__file__).resolve(),W/'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/vision/visual_stationary.py')},
  scope='Spatial estimates only; never calls observe or invents SDK source cadence; variants are diagnostics only',rows=[])
 for case in cases:
  x,y=case.pop('frames');case['variants']={}
  for key,region,seed,resize in [('parent_original','parent','old',False),('child_original','child','old',False),('child_full_mask_seed','child','full_mask',False),('child_full_mask_margin31','child','full_mask_margin',False),('child_parent_scale','child','old',True)]:
   case['variants'][key]=calc(x,y,region,seed,resize)
  report['rows'].append(case)
 (a.output/'report.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
 print(json.dumps(dict(output=str(a.output),cases=len(cases),report_sha256=sha((a.output/'report.json').read_bytes()))))
if __name__=='__main__':main()
