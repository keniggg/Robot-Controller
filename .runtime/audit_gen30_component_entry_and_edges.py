"""One negative entry check and three recorded graph-edge audits; no new matrix."""
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace as NS
from collections import Counter
import ast,hashlib,json,os,sys,time
W=Path(__file__).resolve().parents[1];M=Path('/media/zhuyupei/SanStand')
N=M/'实验记录/Robot-Controller/ros_log/full_latest_gen22_restart_20261015T003700Z/verification';I=N/'gen29_attempt1_shadow_frames'
O=N/'gen30_component_entry_and_edges_r1'
assert M.is_mount() and not O.exists() and os.environ['ROS_MASTER_URI']=='http://127.0.0.1:1'
sys.path[:0]=[str(W/'src/alicia_grasp_modes/src'),str(W/'src/alicia_grasp_modes/tests')]
import cv2,numpy as np
from alicia_grasp_modes import tabletop as table,rgb_instance as rgb,current_fragment_owner as existing
from alicia_grasp_modes.rgb_observation_handoff import _appearance
from alicia_grasp_modes.observation_appearance import FrozenObservationAppearance,match_observation_appearance
from test_arrival_fragment_recovery_guards import fragment_scene,disconnected_scene,measured_candidate,K as TEST_K,PLANE as TEST_PLANE
from test_arrival_matching_fragments import scene as two_match_scene
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def ah(a):return hashlib.sha256(np.ascontiguousarray(a).tobytes()).hexdigest()
def serial(x):
 if isinstance(x,np.ndarray):return x.tolist()
 if isinstance(x,np.generic):return x.item()
 raise TypeError(type(x).__name__)
inputs={}
def read(p):inputs[str(p)]=sha(p);return json.loads(Path(p).read_text())
source=W/'.runtime/prototype_gen30_measured_component.py';inputs[str(source)]=sha(source)
tree=ast.parse(source.read_text());names={'reference_for','synthetic_cases'}
exec(compile(ast.Module(body=[n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name in names],type_ignores=[]),str(source),'exec'),globals())
name,d,neighbor,reference,_=next(r for r in synthetic_cases() if r[0]=='wide_diagonal_contact')
result=table.segment_with_support_fallback(d['depth'],d['intrinsics'],d['transform'],d['config'],d['anchor'],d['last'],TEST_PLANE,prefer_current_support=True)
primary=table._matching_candidates(result.candidates,d['anchor'],d['last'],d['config'])[0][1]
frame=rgb._FrameRefinement(d['depth'],d['intrinsics'],d['transform'],d['spatial_reference'],d['config'])
refined=frame.refine(d['rgb'],primary,rgb_valid=np.ones(d['depth'].shape,bool))
appearance=match_observation_appearance(d['rgb'],refined.mask,reference.histogram,reference.contour,
 rgb_valid=np.ones(d['depth'].shape,bool),appearance_context=dict(reference=reference,reference_role='qualified_station_identity_template',
 source_ns=3001,same_station_mask_iou_minimum=.85,check_current=lambda:None))
negative=dict(case=name,original_primary_refinement_mask_sha256=ah((refined.mask>0).astype(np.uint8)),
 original_primary_neighbor_pixels=int(np.count_nonzero((refined.mask>0)&neighbor)),fixed_template_appearance=appearance,
 new_branch_reachable=appearance['code']=='SAME_STATION_COMPLETE_OUTLINE_MISMATCH',
 interpretation='This is a baseline hazard, not a demonstrated reachable new-branch counterexample, unless the original primary fails fixed outline identity.')
meta=read(I/'mask_contract_review_r2/result.json');cfg=table.Config(**meta['effective_config']);old=read(N/'gen30_current_fragment_owner_r2/result.json')
rows=[]
for prior,oldrow in zip(meta['rows'][1:],old['rows']):
 ns=prior['source_stamp_ns'];p=I/f'source_{ns}'/'raw_arrays.npz';inputs[str(p)]=sha(p);a=dict(np.load(p,allow_pickle=False))
 depth=a['depth_raw'].astype(np.float32)*float(a['depth_scale_m']);K=a['intrinsics'];T=a['T_base_camera_link'];plane=np.asarray(prior['support_plane'])
 path=N/'gen30_measured_component_prototype_r1'/f'{ns}_whole_raw.png';inputs[str(path)]=sha(path);raw=cv2.imread(str(path),0)>0
 components=table._depth_components(raw.astype(np.uint8),depth,cfg.min_object_points,cfg.depth_edge_m,connectivity=4)
 expected=sorted(m['native_pixels'] for m in oldrow['audit']['native_members']);assert sorted(int(m.sum()) for m in components)==expected
 labels=np.zeros(depth.shape,np.int32)
 for i,m in enumerate(components):labels[m]=i+1
 points=table._project(depth,K);pc=T.T@plane;heights=np.einsum('ijk,k->ij',points,pc[:3])+pc[3]
 valid=np.isfinite(depth)&(depth>=cfg.depth_min_m)&(depth<=cfg.depth_max_m)
 edges=[];summary=Counter();member_witness_graph={i:set() for i in range(len(components))}
 def category(y,x,za,zb):
  if not valid[y,x]:return 'unknown'
  if abs(heights[y,x])<=cfg.support_threshold_m:return 'measured_support'
  if cfg.foreground_min_m<=heights[y,x]<=cfg.foreground_max_m:
   return 'measured_foreground_continuous' if max(abs(float(depth[y,x])-za),abs(float(depth[y,x])-zb))<=cfg.depth_edge_m else 'foreground_depth_discontinuous'
  return 'measured_grey_or_other'
 yy,xx=np.nonzero(raw)
 for y,x in zip(yy.tolist(),xx.tolist()):
  for dx in (-1,1):
   y2,x2=y+1,x+dx
   if not(0<=x2<raw.shape[1] and y2<raw.shape[0] and raw[y2,x2]):continue
   first,second=labels[y,x],labels[y2,x2]
   if first==second or abs(float(depth[y,x])-float(depth[y2,x2]))>cfg.depth_edge_m:continue
   cats=[category(y,x2,float(depth[y,x]),float(depth[y2,x2])),category(y2,x,float(depth[y,x]),float(depth[y2,x2]))]
   key=' + '.join(sorted(cats));summary[key]+=1
   witness='measured_foreground_continuous' in cats
   if witness:member_witness_graph[first-1].add(second-1);member_witness_graph[second-1].add(first-1)
   edges.append(dict(first_yx=[y,x],second_yx=[y2,x2],native_ids=[int(first),int(second)],orthogonal_categories=cats,
    orthogonal_depth_m=[float(depth[y,x2]) if valid[y,x2] else None,float(depth[y2,x]) if valid[y2,x] else None],
    orthogonal_height_m=[float(heights[y,x2]) if valid[y,x2] else None,float(heights[y2,x]) if valid[y2,x] else None],
    foreground_continuous_witness=witness))
 unseen=set(member_witness_graph);sets=[]
 while unseen:
  stack=[unseen.pop()];part=set(stack)
  while stack:
   for nxt in member_witness_graph[stack.pop()]:
    if nxt not in part:part.add(nxt);unseen.discard(nxt);stack.append(nxt)
  sets.append(sorted(part))
 rows.append(dict(source_stamp_ns=ns,original_four_neighbor_native_pixels=expected,cross_native_diagonal_edges=len(edges),
  category_counts=dict(summary),edges_with_measured_foreground_witness=sum(e['foreground_continuous_witness'] for e in edges),
  graph_components_if_unknown_never_bridges=sets,edges=edges))
O.mkdir();cv2.imwrite(str(O/'negative_original_primary_refined.png'),refined.mask)
report=dict(negative_entry=negative,actual_diagonal_edges=rows,input_sha256=inputs,script_sha256=sha(__file__),
 source_sha256={str(m.__file__):sha(m.__file__) for m in (table,rgb,existing)},no_production_change=True,no_motion=True,no_ros_network_calls=True,
 scope='One prior negative original entry refinement; actual three sources only current depth-neighbor classification, no new RGB or matrix replay.',
 classification='Original unsafe direct-prototype report preserved; new-branch reachability must be stated separately.')
assert all(sha(p)==h for p,h in inputs.items())
(O/'result.json').write_text(json.dumps(report,indent=2,default=serial)+'\n');print(json.dumps(negative,default=serial))
for row in rows:print({k:v for k,v in row.items() if k!='edges'})
print(O/'result.json',sha(O/'result.json'))
