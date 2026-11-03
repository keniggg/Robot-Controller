"""Bounded single offline chain probe; preserve real qualifier audit without altering gates."""
import os,sys,json,time,hashlib,traceback
from pathlib import Path
W=Path(__file__).resolve().parents[1];C=W/'.runtime/gen43_mask_candidate'
N=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_gen41_restart_20261018T011222Z')
O=Path(sys.argv[1]).resolve();assert O.parent==N/'verification' and not O.exists();O.mkdir()
os.environ['ROS_LOG_DIR']=str(O/'ros');os.environ['PYTHONDONTWRITEBYTECODE']='1'
sys.path.append(str(W/'devel/lib/python3/dist-packages'))
for p in reversed([C/'src/alicia_flexible_grasp_supervisor/src',C/'src/alicia_grasp_modes/src',C/'src/alicia_flexible_grasp_supervisor/tests',C/'src/alicia_grasp_modes/tests']):sys.path.insert(0,str(p))
import numpy as np
import pytest
import test_gen43_top_mask_handoff as m
import test_remote_grasp6d_streaming as f
from alicia_flexible_grasp.vision import cuboid_rgb_interval as c
from collections.abc import Mapping
def plain(v):
 if isinstance(v,Mapping):return {str(k):plain(x) for k,x in v.items()}
 if isinstance(v,(tuple,list)):return [plain(x) for x in v]
 if isinstance(v,np.ndarray):return v.tolist()
 if isinstance(v,np.generic):return v.item()
 return v
rows=[]
from _pytest.monkeypatch import MonkeyPatch
patch=MonkeyPatch()
m.no_unsimulated_boundaries.__wrapped__(patch)
import test_remote_modes as rm
original=rm.base.qualify_regular_box_snapshot
group_calls=[]
original_groups=c._groups
def groups(*args,**kwargs):
 model,ep=args[:2]; summary=[]
 for row in model['edges']:
  if row['kind']=='vertical':continue
  a,b=np.asarray(row['start_pixel']),np.asarray(row['end_pixel']);v=b-a
  along=(ep-a)@v/float(v@v); across=np.abs(np.cross(v,ep-a))/np.linalg.norm(v)
  summary.append(dict(kind=row['kind'],axis=row['axis'],response_pixels=int(np.count_nonzero((along>=.08)&(along<=.92)&(across<=2.))),edge_length_px=float(np.linalg.norm(v))))
 call=dict(edges=summary,axes=plain(args[4]),height=float(args[7]),model=plain(model))
 group_calls.append(call)
 try:return original_groups(*args,**kwargs)
 except ValueError as exc:call['error']=str(exc);raise
patch.setattr(c,'_groups',groups)
import importlib.util
from alicia_flexible_grasp.vision import regular_box_inference as box
baseline_path=W/'.runtime/gen42_source_before_gen43/src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/vision/cuboid_rgb_interval.py'
assert hashlib.sha256(baseline_path.read_bytes()).hexdigest()=='4e127a38faac34b2b981d2f757f22399934b38655e44376f268580a3243dffdf'
spec=importlib.util.spec_from_file_location('alicia_flexible_grasp.vision._gen42_fixture_qualification_baseline',baseline_path)
old=importlib.util.module_from_spec(spec);spec.loader.exec_module(old)
inference_pairs=[];original_infer=box.infer_cuboid_rgb_interval
def paired_infer(*args,**kwargs):
 current=original_infer(*args,**kwargs)
 previous=old.infer_cuboid_rgb_interval(*args,**kwargs)
 inference_pairs.append(dict(candidate=plain(current),baseline=plain(previous)))
 return current
patch.setattr(box,'infer_cuboid_rgb_interval',paired_infer)

def watched(*args,**kwargs):
    result=original(*args,**kwargs)
    row=dict(ok=result.ok,code=result.code,audit=plain(result.audit),stamp_ns=args[-1],call_index=len(rows))
    rows.append(row)
    (O/'qualifications.json').write_text(json.dumps(rows,ensure_ascii=False,indent=2,allow_nan=False)+'\n')
    if not result.ok:
        snapshot=args[0]
        np.save(O/('call%d_color.npy'%row['call_index']),snapshot.original_sample.color_bgr)
        np.save(O/('call%d_mask.npy'%row['call_index']),snapshot.original_sample.object_mask)
        np.save(O/('call%d_depth.npy'%row['call_index']),snapshot.original_sample.depth_raw)
    return result
patch.setattr(rm.base,'qualify_regular_box_snapshot',watched)
(O/'case').mkdir()
r=dict(probe='single original chain qualification audit',tests_run=False,ROS_initialized=False,motion_commands_sent=False,temperature_queried=False)
try:
 m._install(patch)
 if '--prepare-only' in sys.argv:
  context=m.chain.arrived_same_scene(patch,perturb=True)
  candidates=m.chain.same_source_contact(context)
  model=context.remote._current_inferred_box()
  r['prepare_only']=dict(candidate_count=len(candidates),model_qualification=plain(model.qualification))
  raise SystemExit('prepare-only completed')
 if '--arrival-only' in sys.argv:
  context=m.chain.arrived_same_scene(patch,perturb=True)
  mask=context.producer.bridge.imgmsg_to_cv2(context.arrival_row['mask'],'mono8')
  r['arrival_only']=dict(dropout_applied=context.gen43_side_dropout_applied,missing_side_pixels=int(np.count_nonzero(context.scene.visible_side & (mask==0))),side_pixels=int(np.count_nonzero(context.scene.visible_side)),status=plain(context.arrival_row['status']))
  raise SystemExit('arrival-only completed')
 if '--producer-only' in sys.argv:
  context=m.chain.same_scene_producer(patch)
  context.scene.depth=context.scene.depth.copy();context.scene.depth[context.scene.visible_side]=0
  out=[]
  for i in range(3):
   context.clock[0]+=.2;context.feed();row=m.chain.publication(context.producer);mask=context.producer.bridge.imgmsg_to_cv2(row['mask'],'mono8')
   out.append(dict(detected=bool(row['object'].detected),missing_side_pixels=int(np.count_nonzero(context.scene.visible_side & (mask==0))),side_pixels=int(np.count_nonzero(context.scene.visible_side)),status=plain(row['status'])))
  r['producer_only']=out
  raise SystemExit('producer-only completed')
 result=m.chain.unified_replay(patch,O/'case',('status','ack','object','mask'),perturb=True)
 m._source_and_model(result)
 r['source_and_model_assertions_passed']=True
 r.update(chain_returned=True,dropout_applied=result.gen43_side_dropout_applied,side_depth_missing_pixels=int(np.count_nonzero(result.scene.visible_side & (result.scene.depth==0))),side_final_mask_missing_pixels=int(np.count_nonzero(result.scene.visible_side & (result.ticket.payload[0].original_sample.object_mask==0))),states=[dict(state=x.state,message=x.message) for x in result.states])
except BaseException as exc:
 r.update(error=repr(exc),traceback=traceback.format_exc())
finally:
 patch.undo()
 r.update(qualifications=rows,groups=group_calls,inference_pairs=inference_pairs,source_sha256={str(Path(x)):hashlib.sha256(Path(x).read_bytes()).hexdigest() for x in [m.__file__,c.__file__,__file__]})
 (O/'result.json').write_text(json.dumps(r,ensure_ascii=False,indent=2,allow_nan=False)+'\n')
 print(json.dumps({'output':str(O),'qualifications':[{'ok':x['ok'],'code':x['code'],'reason':x['audit'].get('reason'),'top':x['audit'].get('cuboid_rgb_interval',{}).get('top_mask_coverage'),'whole':x['audit'].get('cuboid_rgb_interval',{}).get('silhouette_mask_coverage')} for x in rows],'error':r.get('error')},ensure_ascii=False))
