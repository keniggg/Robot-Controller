from pathlib import Path
import hashlib,json
import cv2,numpy as np
setup=Path('.runtime/gen22_orb_inside_mask_diagnostic_20261010.py').read_text();exec(compile(setup[:setup.index('\nresults=[]\nfor mode')],'<fixed_control_definitions>','exec'))
original=json.loads((B/'orb_texture_correspondence_diagnostic.json').read_text());Hs={r['source_ns']:np.asarray(r['homography']) for r in original['results'] if r['variant']=='recorded_object'}
reference=last['color'];rm=(last['recorded_refined_mask']>0).astype(np.uint8);rvalid=last.get('rgb_valid',np.ones(rm.shape,bool));rgray=cv2.cvtColor(reference,cv2.COLOR_BGR2GRAY).astype(float);rx,ry,rw,rh=cv2.boundingRect(rm)
def corr(x,y):
 sx=float(np.std(x));sy=float(np.std(y))
 return None if len(x)<3 or min(sx,sy)<1e-9 else float(np.corrcoef(x,y)[0,1])
def dense(H,image,mask,valid):
 # Use nearest existing pixels only: no image warp, padding, interpolation,
 # depth completion or synthesized reference pixels enter comparisons.
 yy,xx=np.nonzero((cv2.erode(rm,np.ones((7,7),np.uint8))>0)&rvalid);xy=np.c_[xx,yy,np.ones(len(xx))]@H.T;good=np.isfinite(xy).all(1)&(abs(xy[:,2])>1e-9);uv=xy[:,:2]/xy[:,2,None];ij=np.rint(uv).astype(int);ix,iy=ij.T;good&=(ix>=0)&(ix<mask.shape[1])&(iy>=0)&(iy<mask.shape[0]);inside=np.zeros(len(good),bool);sel=np.flatnonzero(good);cm=cv2.erode((mask>0).astype(np.uint8),np.ones((7,7),np.uint8))>0;inside[sel]=cm[iy[sel],ix[sel]]&valid[iy[sel],ix[sel]];keep=good&inside
 xx,yy,ix,iy=xx[keep],yy[keep],ix[keep],iy[keep];curgray=cv2.cvtColor(image,cv2.COLOR_BGR2GRAY).astype(float);x=rgray[yy,xx];y=curgray[iy,ix];cells=np.c_[((xx-rx)//8),((yy-ry)//8)];train=(cells.sum(1)%2)==0;test=~train
 result=dict(real_pixel_pairs=len(x),training_pairs=int(train.sum()),test_pairs=int(test.sum()),reference_mask_sample_fraction=float(len(x)/max(1,rm.sum())),test_rule='8x8 source-image cells; even parity fits, odd parity validates',valid=False)
 if train.sum()<100 or test.sum()<100:return result
 A=np.c_[x[train],np.ones(train.sum())];target=y[train];coef=np.linalg.lstsq(A,target,rcond=None)[0]
 # Fixed Huber IRLS, delta10 gray units, three reweights; no positive clipping.
 for iteration in range(3):
  residual=target-A@coef;weights=np.minimum(1.,10./np.maximum(abs(residual),1e-12));sw=np.sqrt(weights);coef=np.linalg.lstsq(A*sw[:,None],target*sw,rcond=None)[0]
 pred=coef[0]*x[test]+coef[1];err=y[test]-pred;std=float(np.std(y[test]));rows=[]
 for key in sorted(set(map(tuple,cells[test]))):
  sel=test&np.all(cells==key,axis=1);count=int(sel.sum())
  if count<25:continue
  vx=x[sel];vy=y[sel];sx=float(np.std(vx));sy=float(np.std(vy));ncc=corr(vx,vy);tex=min(sx,sy)>=10.
  rows.append(dict(cell_xy=list(map(int,key)),pairs=count,reference_std=sx,current_std=sy,texture_qualified=tex,ncc=ncc,rmse=float(np.sqrt(np.mean((vy-(coef[0]*vx+coef[1]))**2))),quadrant=[int(np.mean(xx[sel])>=rx+rw/2),int(np.mean(yy[sel])>=ry+rh/2)]))
 eligible=[r for r in rows if r['texture_qualified']];nccs=[r['ncc'] for r in eligible];positive=[r for r in eligible if r['ncc'] is not None and r['ncc']>=.7]
 result.update(valid=True,affine_slope=float(coef[0]),affine_intercept=float(coef[1]),positive_slope=bool(coef[0]>0),test_ncc=corr(x[test],y[test]),test_rmse=float(np.sqrt(np.mean(err**2))),test_normalized_rmse=float(np.sqrt(np.mean(err**2))/std) if std>0 else None,test_abs_error_p90=float(np.percentile(abs(err),90)),reference_test_std=float(np.std(x[test])),current_test_std=std,test_cells=len(rows),contrast_qualified_test_cells=len(eligible),contrast_qualified_quadrants=len(set(tuple(r['quadrant']) for r in eligible)),qualified_cell_ncc_median=float(np.median(nccs)) if nccs else None,qualified_cell_ncc_p10=float(np.percentile(nccs,10)) if nccs else None,test_cells_ncc_ge_point7=len(positive),ncc_ge_point7_quadrants=len(set(tuple(r['quadrant']) for r in positive)),test_cell_metrics=rows)
 return result
results=[]
for item in rep['results']:
 ns=item['source_ns'];f=dict(np.load(idx[ns]['path']));mask=cv2.imread(str(B/('crop_ab_production_'+str(ns)+'_mask.png')),0);H=Hs[ns]
 for variant,image,region in controls(f['color'],mask):
  proposal=H.copy();basis='actual original ORB H, used only as proposal'
  if variant!='recorded_object':basis='deliberately supplied same target H for content-negative diagnosis, not an ORB accepted plan'
  if variant=='recorded_wrong_tablecloth_roi':
   x,y,w,h=cv2.boundingRect(mask);shift=np.eye(3);shift[0,2]=175-(x+w/2);shift[1,2]=175-(y+h/2);proposal=shift@H;basis='actual H translated into a real wrong cloth ROI, deliberately favorable negative proposal'
  row=dense(proposal,image,region,f['rgb_valid']);results.append(dict(source_ns=ns,variant=variant,proposal_basis=basis,**row))
report=dict(record=m['record'],reference_source_ns=lr['source_stamp_ns'],original_ORB_report_sha256=hashlib.sha256((B/'orb_texture_correspondence_diagnostic.json').read_bytes()).hexdigest(),method='Independent dense nearest-original-pixel measurements inside BOTH eroded observed masks; fixed checkerboard train/test cells; fitted positive-direction brightness test without modifying source intensities.',parameters=dict(mask_erosion_px=3,cell_size_px=8,minimum_cell_pairs=25,minimum_cell_std_gray=10,affine_fit='unconstrained slope reported; reject negative conceptually, never forced positive',huber_delta=10,huber_iterations=3,ncc_point7_count='summary diagnostic only, not production gate'),results=results,production_changed=False,limitations=['No threshold tuning or production authorization from these outputs.','Original ORB H may depend on exterior descriptor samples, therefore this diagnostic evaluates only whether independent in-mask measurements support or contradict that proposal.','Synthetic negative content is deliberately checked even without a valid ORB proposal.','Nearest-pixel H correspondences are estimated planar associations, not calibrated physical point identity.','Holdout checkerboard cells are disjoint fit/test samples but have spatial correlation. Genuine alternate physical look-alike objects were unavailable.'])
out=B/'dense_masked_photometric_identity_diagnostic.json';out.write_text(json.dumps(report,indent=2)+'\n');print('REPORT',str(out),hashlib.sha256(out.read_bytes()).hexdigest())
for variant in sorted(set(r['variant'] for r in results)):
 rr=[r for r in results if r['variant']==variant and r['valid']];print(variant,'valid',len(rr),'NCC',[round(min(r['test_ncc'] or -1 for r in rr),3),round(max(r['test_ncc'] or -1 for r in rr),3)] if rr else [],'cellncc',[round(min(r['qualified_cell_ncc_median'] or -1 for r in rr),3),round(max(r['qualified_cell_ncc_median'] or -1 for r in rr),3)] if rr else [],'n>=.7',[min(r['test_cells_ncc_ge_point7'] for r in rr),max(r['test_cells_ncc_ge_point7'] for r in rr)] if rr else [],'q',[min(r['ncc_ge_point7_quadrants'] for r in rr),max(r['ncc_ge_point7_quadrants'] for r in rr)] if rr else [],'slope',[round(min(r['affine_slope'] for r in rr),3),round(max(r['affine_slope'] for r in rr),3)] if rr else [])
